#pragma once
#include <string>
#include <vector>
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Safety_Performance_Metric/ParametersSP.h"
#include "sources/Optimization/OptimizeSP_TL_Incre.h"

namespace SP_OPT_PA {

struct JobRecord {
    int taskId;
    int jobId;
    LLint releaseTime;
    LLint startTime;
    LLint finishTime;
    double executionTime;
    bool isOverrun;
};

class BaseSimulationOrchestrator {
public:
    BaseSimulationOrchestrator(const std::string& input_folder,
                               const std::string& output_folder,
                               LLint interval_duration_ms = 10000);
    virtual ~BaseSimulationOrchestrator() = default;

    virtual void RunSimulation() = 0;

    // Helper for testing
    const std::vector<JobRecord>& GetJobHistory() const { return job_history_; }
    const std::vector<double>& GetIntervalSPMetrics() const { return interval_sp_metrics_; }

    // Sum of per-interval scheduler (DeterminePrioritiesAndBudgets) wall-time
    // ONLY — the optimizer decision, where the RTA cache + Transaction live.
    // Deliberately EXCLUDES RTDA rollout, SP-metric computation, I/O, export:
    // those are simulation, not scheduling. FTP accumulates per interval; CFS
    // never increments (stays 0.0, matching the Python CFS hardcode). Written
    // to scheduler_execution_time.txt; the total RunSimulation() wall-time is
    // printed to stdout separately as a reference number.
    double GetSchedulerExecutionTime() const { return scheduler_exec_time_s_; }

    void PrintHyperperiodSchedule(LLint start_time, LLint end_time) const;

protected:
    std::string input_folder_;
    std::string output_folder_;
    LLint interval_duration_ms_;

    std::vector<DAG_Model> dag_tasks_vecs_;
    std::vector<TaskSetInfoDerived> tasks_info_vecs_;
    std::vector<SP_Parameters> sp_parameters_vecs_;

    std::vector<JobRecord> job_history_;
    std::vector<double> interval_sp_metrics_;

    // Accumulator for GetSchedulerExecutionTime(); incremented inside
    // DeterminePrioritiesAndBudgets. See that accessor's comment.
    double scheduler_exec_time_s_ = 0.0;

    void LoadIntervalConfigs();
    std::vector<float> LoadJobExecutionTraces(int task_id, int path_idx, int inst_idx);
    void ExportResults(const std::string& scheduler_name);
};

class FixedTaskPrioritySchedulingOrchestrator : public BaseSimulationOrchestrator {
public:
    FixedTaskPrioritySchedulingOrchestrator(const std::string& input_folder,
                                            const std::string& output_folder,
                                            const std::string& scheduler_mode,
                                            LLint interval_duration_ms = 10000);

    void RunSimulation() override;

    // Refactored helper functions for unit testing and cleaner SimulateInterval
    ResourceOptResult DeterminePrioritiesAndBudgets(DAG_Model& dag_tasks, const SP_Parameters& sp_parameters);
    void ApplyTaskConfigurations(DAG_Model& dag_tasks, const ResourceOptResult& res);
    void RecordFinishedJobs(LLint time_now, RunQueue& run_queue, const ResourceOptResult& res, const DAG_Model& dag_tasks);
    void ReleaseJobs(LLint time_now, LLint end_time, const DAG_Model& dag_tasks, const ResourceOptResult& res,
                     RunQueue& run_queue, std::unordered_map<int, std::vector<float>>& traces,
                     std::unordered_map<int, size_t>& trace_indices, int processor_id = -1);

private:
    std::string scheduler_mode_;
    OptimizePA_Incre_with_TimeLimits incr_optimizer_;

    void SimulateInterval(int interval_idx, LLint start_time, LLint end_time);
};

// INEFFICIENCY NOTE: CFSSimulationOrchestrator implements a task-level CFS
// policy using per-millisecond stepping. Because each tick scans the entire
// job_queue_ linearly (e.g. to find the running job in UpdateCFSVirtualTimes,
// or to locate the oldest job for a given task in ScheduleCFS), complexity is
// broadly O(N) per tick (N = queue length). In normal workloads with queue
// sizes of 10–20 jobs this adds roughly 5–7x overhead versus fixed-priority
// scheduling. The set<...> run_queue_set is maintained at TASK granularity so
// that the scheduler picks the task with the smallest virtual runtime (fairness)
// but always dispatches the OLDEST pending JOB for that task (FIFO within task).
// This task-level design means all jobs belonging to the same task are treated
// as one fairness entity, replicating a "cgroup"-like CFS rather than a pure
// per-job CFS. Future work can replace linear scans with direct indexes per
// task (e.g. head-of-queue pointers) and a direct running-task tracker so
// that all per-tick operations become O(log T) or O(1) where T = #tasks.
class CFSSimulationOrchestrator : public BaseSimulationOrchestrator {
public:
    CFSSimulationOrchestrator(const std::string& input_folder,
                              const std::string& output_folder,
                              LLint interval_duration_ms = 10000);

    void RunSimulation() override;

    // Refactored helper functions for CFS unit testing

    // INEFFICIENCY: Scans the entire job_queue_ linearly each tick to find the
    // currently running job and update its accumulated execution time. This is
    // O(N) per tick. A direct "running job pointer" or a per-tick delta update
    // could reduce this to O(log T) or O(1).
    void UpdateCFSVirtualTimes(int running_task_id,
                               std::unordered_map<int, double>& accumulated_et,
                               std::set<std::pair<double, int>>& run_queue_set);

    // INEFFICIENCY: Iterates over the full schedule map every tick to detect
    // jobs whose finish time equals time_now. A callback triggered when a job
    // actually finishes would make this O(1) per event instead of O(|schedule|).
    void RecordFinishedJobsCFS(LLint time_now, RunQueue& run_queue, const DAG_Model& dag_tasks);

    // INEFFICIENCY: Rebuilds the per-task active-job counts by scanning the
    // entire job_queue_ every tick (O(N)). Maintaining an inline counter in
    // ReleaseJobsCFS and RemoveFinishedJob would make this O(1).
    void UpdateActiveCounts(RunQueue& run_queue, const DAG_Model& dag_tasks,
                            std::unordered_map<int, int>& active_jobs_count,
                            std::set<std::pair<double, int>>& run_queue_set,
                            const std::unordered_map<int, double>& accumulated_et);

    // INEFFICIENCY: Although release is O(1), the caller then rescans the
    // queue to update active counts. Coalescing count updates into release
    // would avoid the second linear pass.
    void ReleaseJobsCFS(LLint time_now, LLint end_time, const DAG_Model& dag_tasks, RunQueue& run_queue,
                        std::unordered_map<int, std::vector<float>>& traces,
                        std::unordered_map<int, size_t>& trace_indices, int processor_id = -1);

    // INEFFICIENCY: After picking the task with minimum virtual runtime from
    // the ordered set (O(log T)), this function performs a linear scan over
    // job_queue_ to find the oldest job for that task. Maintaining a
    // task_id -> head-index map would make the dispatch O(1) (plus O(log T)
    // for set operations).
    void ScheduleCFS(LLint time_now, RunQueue& run_queue,
                     std::set<std::pair<double, int>>& run_queue_set,
                     int& running_task_id);

private:
    // INEFFICIENCY: The simulation loop runs per-millisecond (for-loop over
    // every single ms) and invokes the above linear-scan helpers each tick.
    // For long durations (e.g. 10 000 ms intervals) this means millions of
    // unnecessary O(N) operations. A discrete-event simulator (next event
    // time = min(next release, next finish, next preemption)) would provide
    // orders-of-magnitude speedup.
    void SimulateInterval(int interval_idx, LLint start_time, LLint end_time);
};

} // namespace SP_OPT_PA

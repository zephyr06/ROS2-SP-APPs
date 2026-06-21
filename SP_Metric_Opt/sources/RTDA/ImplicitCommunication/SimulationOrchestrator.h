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
                     std::unordered_map<int, size_t>& trace_indices);

private:
    std::string scheduler_mode_;
    OptimizePA_Incre_with_TimeLimits incr_optimizer_;

    void SimulateInterval(int interval_idx, LLint start_time, LLint end_time);
};

class CFSSimulationOrchestrator : public BaseSimulationOrchestrator {
public:
    CFSSimulationOrchestrator(const std::string& input_folder,
                              const std::string& output_folder,
                              LLint interval_duration_ms = 10000);

    void RunSimulation() override;

    // Refactored helper functions for CFS unit testing
    void UpdateCFSVirtualTimes(LLint time_now, LLint start_time, RunQueue& run_queue,
                               std::unordered_map<int, double>& accumulated_et,
                               std::set<std::pair<double, int>>& run_queue_set);
    void RecordFinishedJobsCFS(LLint time_now, RunQueue& run_queue, const DAG_Model& dag_tasks);
    void UpdateActiveCounts(RunQueue& run_queue, const DAG_Model& dag_tasks,
                            std::unordered_map<int, int>& active_jobs_count,
                            std::set<std::pair<double, int>>& run_queue_set,
                            const std::unordered_map<int, double>& accumulated_et);
    void ReleaseJobsCFS(LLint time_now, LLint end_time, const DAG_Model& dag_tasks, RunQueue& run_queue,
                        std::unordered_map<int, std::vector<float>>& traces,
                        std::unordered_map<int, size_t>& trace_indices);
    void ScheduleCFS(LLint time_now, RunQueue& run_queue, std::set<std::pair<double, int>>& run_queue_set);

private:
    void SimulateInterval(int interval_idx, LLint start_time, LLint end_time);
};

} // namespace SP_OPT_PA

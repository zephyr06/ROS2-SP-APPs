#include "sources/RTDA/ImplicitCommunication/ScheduleSimulation.h"
#include <limits>
#include <unordered_map>
#include <set>

namespace SP_OPT_PA {

void AddTasksToRunQueue(RunQueue &run_queue, const DAG_Model &dag_tasks,
                        int processor_id, LLint time_now) {
    for (int task_id = 0;
         task_id < static_cast<int>(dag_tasks.GetTaskSet().size()); task_id++) {
        Task task_curr = dag_tasks.GetTask(task_id);
        if (task_curr.processorId == processor_id &&
            time_now % task_curr.period == 0) {
            JobCEC job_curr(task_id, time_now / task_curr.period);
            run_queue.insert(job_curr);
        }
    }
}

Schedule SimulatedFTP_SingleCore(const DAG_Model &dag_tasks,
                                 const TaskSetInfoDerived &tasks_info,
                                 int processor_id) {
    const TaskSet &tasks = dag_tasks.GetTaskSet();
    RunQueue run_queue(tasks);
    for (LLint time_now = 0; time_now <= tasks_info.hyper_period; time_now++) {
        // first remove jobs that have been finished at this time
        run_queue.RemoveFinishedJob(time_now);

        // check whether to add new instances
        if (time_now < tasks_info.hyper_period)
            AddTasksToRunQueue(run_queue, dag_tasks, processor_id, time_now);

        // Run jobs with highest priority
        run_queue.RunJobHigestPriority(time_now);
    }
    return run_queue.GetSchedule();
}

std::vector<int> GetProcessorIds(const DAG_Model &dag_tasks) {
    std::vector<int> processor_ids;
    std::unordered_set<int> id_record;
    processor_ids.reserve(dag_tasks.GetTaskSet().size());
    for (uint task_id = 0; task_id < dag_tasks.GetTaskSet().size(); task_id++) {
        int processor_id = dag_tasks.GetTask(task_id).processorId;
        if (id_record.find(processor_id) == id_record.end()) {
            id_record.insert(processor_id);
            processor_ids.push_back(processor_id);
        }
    }
    return processor_ids;
}

Schedule SimulateFixedPrioritySched(const DAG_Model &dag_tasks,
                                    const TaskSetInfoDerived &tasks_info) {
    Schedule schedule_all;
    schedule_all.reserve(tasks_info.length);
    std::vector<int> processor_ids = GetProcessorIds(dag_tasks);
    for (int processor_id : processor_ids) {
        Schedule schedule_curr =
            SimulatedFTP_SingleCore(dag_tasks, tasks_info, processor_id);
        schedule_all.insert(schedule_curr.begin(), schedule_curr.end());
    }

    return schedule_all;
}

struct CFS_TaskCompare {
    bool operator()(const std::pair<double, int> &a, const std::pair<double, int> &b) const {
        if (a.first != b.first) {
            return a.first < b.first;
        }
        return a.second > b.second; // larger ID first for tie-breakers
    }
};

Schedule SimulatedCFS_SingleCore(const DAG_Model &dag_tasks,
                                 const TaskSetInfoDerived &tasks_info,
                                 int processor_id) {
    const TaskSet &tasks = dag_tasks.GetTaskSet();
    RunQueue run_queue(tasks);
    std::unordered_map<int, double> accumulated_et;
    std::unordered_map<int, int> active_jobs_count;
    std::set<std::pair<double, int>, CFS_TaskCompare> run_queue_set;

    for (const auto &task : tasks) {
        accumulated_et[task.id] = 0.0;
        active_jobs_count[task.id] = 0;
    }

    for (LLint time_now = 0; time_now <= tasks_info.hyper_period; time_now++) {
        // 1. Update accumulated execution time for the job that ran in [time_now - 1, time_now]
        if (time_now > 0) {
            for (auto &job_info : run_queue.job_queue_) {
                if (job_info.running) {
                    int running_taskId = job_info.job.taskId;
                    double old_et = accumulated_et[running_taskId];
                    if (run_queue_set.count({old_et, running_taskId})) {
                        run_queue_set.erase({old_et, running_taskId});
                        accumulated_et[running_taskId] += 1.0;
                        run_queue_set.insert({accumulated_et[running_taskId], running_taskId});
                    } else {
                        accumulated_et[running_taskId] += 1.0;
                    }
                    break;
                }
            }
        }

        // 2. Remove finished jobs at this time
        run_queue.RemoveFinishedJob(time_now);

        // Update run_queue_set based on finished jobs
        std::unordered_map<int, int> new_counts;
        for (const auto &job_info : run_queue.job_queue_) {
            new_counts[job_info.job.taskId]++;
        }
        for (const auto &task : tasks) {
            int old_c = active_jobs_count[task.id];
            int new_c = new_counts[task.id];
            if (old_c > 0 && new_c == 0) {
                run_queue_set.erase({accumulated_et[task.id], task.id});
            }
            active_jobs_count[task.id] = new_c;
        }

        // 3. Add new instances
        if (time_now < tasks_info.hyper_period) {
            AddTasksToRunQueue(run_queue, dag_tasks, processor_id, time_now);
        }

        // Update run_queue_set based on newly arrived jobs
        std::unordered_map<int, int> added_counts;
        for (const auto &job_info : run_queue.job_queue_) {
            added_counts[job_info.job.taskId]++;
        }
        for (const auto &task : tasks) {
            int old_c = active_jobs_count[task.id];
            int new_c = added_counts[task.id];
            if (old_c == 0 && new_c > 0) {
                run_queue_set.insert({accumulated_et[task.id], task.id});
            }
            active_jobs_count[task.id] = new_c;
        }

        // 4. Select the job with the minimum accumulated ET
        if (!run_queue_set.empty()) {
            int min_taskId = run_queue_set.begin()->second;
            int min_job_index = -1;
            for (size_t i = 0; i < run_queue.size(); i++) {
                if (run_queue.job_queue_[i].job.taskId == min_taskId) {
                    min_job_index = i;
                    break;
                }
            }

            // 5. Preempt if a different job needs to run
            if (min_job_index != -1) {
                bool already_running = run_queue.job_queue_[min_job_index].running;
                if (!already_running) {
                    run_queue.PreemptJob(time_now);
                    JobScheduleInfo selected_job = run_queue.job_queue_[min_job_index];
                    run_queue.job_queue_.erase(run_queue.job_queue_.begin() + min_job_index);
                    run_queue.job_queue_.insert(run_queue.job_queue_.begin(), selected_job);
                    if (!run_queue.RunJob(0, time_now)) {
                        CoutError("Error in RunQueue SimulatedCFS_SingleCore RunJob!");
                    }
                }
            }
        }
    }
    return run_queue.GetSchedule();
}

Schedule SimulateCFSSched(const DAG_Model &dag_tasks,
                          const TaskSetInfoDerived &tasks_info) {
    Schedule schedule_all;
    schedule_all.reserve(tasks_info.length);
    std::vector<int> processor_ids = GetProcessorIds(dag_tasks);
    for (int processor_id : processor_ids) {
        Schedule schedule_curr =
            SimulatedCFS_SingleCore(dag_tasks, tasks_info, processor_id);
        schedule_all.insert(schedule_curr.begin(), schedule_curr.end());
    }

    return schedule_all;
}

}  // namespace SP_OPT_PA

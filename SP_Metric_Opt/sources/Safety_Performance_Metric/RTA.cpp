
#include <unordered_map>

#include "sources/TaskModel/RegularTasks.h"
#include "sources/Utils/Parameters.h"

namespace SP_OPT_PA {

FiniteDist GetRTA_OneTask(const Task& task_curr, const TaskSet& hp_tasks) {
    FiniteDist rta_cur = task_curr.execution_time_dist;
    bool if_new_preempt = false;
    for (const Task& task_hp : hp_tasks) {
        rta_cur.CompressDistributionWithOnlySize(GlobalVariables::Granularity *
                                                 2);
        rta_cur.Convolve(task_hp.execution_time_dist);
        if_new_preempt =
            if_new_preempt || (rta_cur.max_time / task_hp.period > 1);
    }
    int n_hp = hp_tasks.size();
    std::vector<int> hp_jobs_considered(n_hp, 1);
    while (if_new_preempt && rta_cur.min_time <= task_curr.deadline) {
        if_new_preempt = false;
        for (int i = 0; i < n_hp; i++) {
            const Task& task_hp = hp_tasks[i];
            if (ceil(rta_cur.max_time / task_hp.period) >
                hp_jobs_considered[i]) {
                rta_cur.AddOnePreemption(
                    task_hp.execution_time_dist,
                    hp_jobs_considered[i] * task_hp.period);
                rta_cur.CompressDeadlineMissProbability(task_curr.deadline);
                hp_jobs_considered[i]++;
                if_new_preempt = true;
            }
        }
    }
    rta_cur.CompressDeadlineMissProbability(task_curr.deadline);
    rta_cur.CompressDistributionWithOnlySize(GlobalVariables::Granularity * 2);
    rta_cur.UpdateMinMaxValues();
    return rta_cur;
}

std::vector<FiniteDist> ProbabilisticRTA_TaskSet_SingleCore(
    const TaskSet& tasks_input) {
    std::unordered_map<int, int> task_id_to_index;
    for (int i = 0; i < tasks_input.size(); i++)
        task_id_to_index[tasks_input[i].id] = i;

    TaskSet tasks = tasks_input;
    std::sort(tasks.begin(), tasks.end(), [](const Task& t1, const Task& t2) {
        return t1.priority < t2.priority;
    });
    int n = tasks.size();
    std::vector<FiniteDist> rtas(n);
    TaskSet hp_tasks;
    hp_tasks.reserve(n - 1);
#if defined(RYAN_HE_CHANGE_DEBUG)
    if (GlobalVariables::debugMode & DBG_PRT_MSK_SP_Metric) {
        std::cout << "####ProbabilisticRTA_TaskSet_SingleCore: " << n
                  << " tasks: " << std::endl;
    }
#endif
    for (int i = 0; i < n; i++) {
        FiniteDist rta_curr = GetRTA_OneTask(tasks[i], hp_tasks);
#if defined(RYAN_HE_CHANGE_DEBUG)
        if (GlobalVariables::debugMode & DBG_PRT_MSK_SP_Metric) {
            std::cout << "####ProbabilisticRTA_TaskSet_SingleCore: tasks[" << i
                      << "]'s RTA: " << std::endl;
            rta_curr.print();
        }
#endif
        // rtas.push_back(rta_curr);
        rtas[task_id_to_index[tasks[i].id]] = rta_curr;
        hp_tasks.push_back(tasks[i]);
    }
    return rtas;
}
// TODO: remove ProcessorTaskSet struct and methods
std::unordered_map<int, TaskSet> ExtractTaskSetPerProcessor(
    const TaskSet& tasks) {
    std::unordered_map<int, TaskSet> processor_task_set;
    for (const Task& task : tasks) {
        if (processor_task_set.find(task.processorId) ==
            processor_task_set.end()) {
            processor_task_set[task.processorId] = TaskSet();
        }
        processor_task_set[task.processorId].push_back(task);
    }
    return processor_task_set;
}

std::vector<FiniteDist> ProbabilisticRTA_TaskSet(const TaskSet& tasks) {
    std::unordered_map<int, int> task_id2index;
    for (uint i = 0; i < tasks.size(); i++) {
        // printf("RYAN %s i=%d, tasks[i].id=%d\n",__func__,i,tasks[i].id);
        task_id2index[tasks[i].id] = i;
    }

    std::unordered_map<int, TaskSet> processor_task_set =
        ExtractTaskSetPerProcessor(tasks);

    std::vector<FiniteDist> rtas(tasks.size());
    // analyze RTA for each task set individually
    for (auto itr = processor_task_set.begin(); itr != processor_task_set.end();
         itr++) {
        const TaskSet& tasks = itr->second;
        std::vector<FiniteDist> rtas_curr =
            ProbabilisticRTA_TaskSet_SingleCore(tasks);
        for (uint i = 0; i < tasks.size(); i++) {
            rtas[task_id2index[tasks[i].id]] = rtas_curr[i];
        }
    }
    return rtas;
}

// assumes not preempted? no! finite_dist is the distribution is from task set
// so premption is considered
double GetDDL_MissProbability(const FiniteDist& finite_dist, double ddl) {
    auto itr = std::upper_bound(finite_dist.distribution.begin(),
                                finite_dist.distribution.end(), ddl,
                                [](double ddl, const Value_Proba& element) {
                                    return ddl < element.value;
                                });
    if (itr == finite_dist.distribution.end())
        return 0;
    else {
        double ddl_miss = 0;
        for (auto ite = itr; ite != finite_dist.distribution.end(); ite++) {
            ddl_miss += ite->probability;
        }
        return ddl_miss;
    }
}
}  // namespace SP_OPT_PA
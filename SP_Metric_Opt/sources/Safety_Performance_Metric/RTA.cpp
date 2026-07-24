
#include <unordered_map>

#include "sources/Optimization/OptimizeSP_Base.h"  // P1.14: BFSharedBudgetCancelled
#include "sources/Safety_Performance_Metric/RTA.h"
#include "sources/TaskModel/RegularTasks.h"
namespace SP_OPT_PA {

void ResolvePreemptionsAndCompress(FiniteDist& rta_cur, const Task& task_curr, const TaskSet& hp_tasks, bool if_new_preempt) {
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
    rta_cur.CompressDistributionWithOnlySize(GlobalVariables::Granularity * 1);
    rta_cur.UpdateMinMaxValues();
}

FiniteDist GetRTA_OneTask(const Task& task_curr, const TaskSet& hp_tasks) {
    FiniteDist rta_cur = task_curr.execution_time_dist;
    bool if_new_preempt = false;
    for (const Task& task_hp : hp_tasks) {
        rta_cur.CompressDistributionWithOnlySize(GlobalVariables::Granularity *
                                                 1);
        rta_cur.Convolve(task_hp.execution_time_dist);
        if_new_preempt =
            if_new_preempt || (rta_cur.max_time / task_hp.period > 1);
    }
    ResolvePreemptionsAndCompress(rta_cur, task_curr, hp_tasks, if_new_preempt);
    return rta_cur;
}

FiniteDist GetRTA_OneTask(const Task& task_curr, const TaskSet& hp_tasks, const FiniteDist& hp_tasks_et_conv) {
    FiniteDist rta_cur = task_curr.execution_time_dist;
    rta_cur.CompressDistributionWithOnlySize(GlobalVariables::Granularity * 1);
    rta_cur.Convolve(hp_tasks_et_conv);

    bool if_new_preempt = false;
    for (const Task& task_hp : hp_tasks) {
        if_new_preempt =
            if_new_preempt || (rta_cur.max_time / task_hp.period > 1);
    }
    ResolvePreemptionsAndCompress(rta_cur, task_curr, hp_tasks, if_new_preempt);
    return rta_cur;
}

std::vector<FiniteDist> ProbabilisticRTA_TaskSet_SingleCore(
    const TaskSet& tasks_input) {
    std::vector<FiniteDist> hp_tasks_et_conv_ignored;
    return ProbabilisticRTA_TaskSet_SingleCore(tasks_input, hp_tasks_et_conv_ignored);
}

std::vector<FiniteDist> ProbabilisticRTA_TaskSet_SingleCore(
    const TaskSet& tasks_input, std::vector<FiniteDist>& hp_tasks_et_conv_vec) {
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

    // Checkpoint store: hp_tasks_et_conv_vec[i] is the HP-tasks'-ET convolution
    // of tasks[0..i) — i.e. the rolling hp_tasks_et_conv snapshotted at the top
    // of iteration i, before tasks[i] is folded in. hp_tasks_et_conv_vec[0] is
    // the empty-HP-set identity. This is exactly the value the 3-arg
    // GetRTA_OneTask consumes, so a patched suffix reuses it verbatim.
    hp_tasks_et_conv_vec.assign(n, FiniteDist({Value_Proba(0, 1.0)}));
    FiniteDist hp_tasks_et_conv({Value_Proba(0, 1.0)});
    for (int i = 0; i < n; i++) {
        // P1.14 — cooperative cancel: this per-task RTA loop is the hottest
        // part of ObtainSP_DAG (the HP-ET convolution grows the distribution
        // support combinatorially, so a single task's GetRTA_OneTask +
        // Convolve can take seconds on wide ET distributions). Poll the BF
        // shared budget between tasks so a runaway single
        // EvaluateSPWithPriorityVec call can be interrupted in place rather
        // than stranding the BF search past TIME_LIMIT. No-op outside a BF
        // search (BFSharedBudgetCancelled() is false). On cancel we return
        // the (partial, garbage) rtas built so far; the caller
        // (ObtainSP_TaskSet -> ObtainSP_DAG -> EvaluateSPWithPriorityVec)
        // discards the partial result via its post-call
        // BFSharedBudgetCancelled() check, so the incomplete rtas never
        // influences the BF incumbent.
        if (BFSharedBudgetCancelled()) return rtas;
        hp_tasks_et_conv_vec[i] = hp_tasks_et_conv;
        FiniteDist rta_curr =
            GetRTA_OneTask(tasks[i], hp_tasks, hp_tasks_et_conv);

        rtas[task_id_to_index[tasks[i].id]] = rta_curr;
        hp_tasks.push_back(tasks[i]);

        hp_tasks_et_conv.CompressDistributionWithOnlySize(
            GlobalVariables::Granularity * 1);
        hp_tasks_et_conv.Convolve(tasks[i].execution_time_dist);
    }
    return rtas;
}
// TODO: remove ProcessorTaskSet struct and methods
std::vector<TaskSet> ExtractTaskSetPerProcessor(const TaskSet& tasks) {
    int max_p = -1;
    for (const Task& task : tasks) {
        if (task.processorId > max_p) max_p = task.processorId;
    }
    std::vector<TaskSet> processor_task_set(max_p + 1);
    for (const Task& task : tasks) {
        processor_task_set[task.processorId].push_back(task);
    }
    return processor_task_set;
}

std::vector<FiniteDist> ProbabilisticRTA_TaskSet(const TaskSet& tasks) {
    std::unordered_map<int, int> task_id2index;
    for (uint i = 0; i < tasks.size(); i++) task_id2index[tasks[i].id] = i;

    std::vector<TaskSet> processor_task_set = ExtractTaskSetPerProcessor(tasks);

    std::vector<FiniteDist> rtas(tasks.size());
    // analyze RTA for each task set individually. Per-core results are written
    // into `rtas` scattered by task id, so core iteration order is irrelevant.
    for (const TaskSet& tasks_curr : processor_task_set) {
        if (tasks_curr.empty()) continue;
        std::vector<FiniteDist> rtas_curr =
            ProbabilisticRTA_TaskSet_SingleCore(tasks_curr);
        for (uint i = 0; i < tasks_curr.size(); i++) {
            rtas[task_id2index[tasks_curr[i].id]] = rtas_curr[i];
        }
    }
    return rtas;
}

// ComputeRTA_FullAndCache + the cache reuse-query/patcher API live in
// RTA_Cache.cpp (declared in RTA_Cache.h). They were relocated out of RTA.cpp
// because the locked signature takes PriorityVec (OptimizeSP_Base.h), which
// would form a header cycle through RTA.h.

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
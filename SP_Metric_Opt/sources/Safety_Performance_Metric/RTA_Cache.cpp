#include "sources/Safety_Performance_Metric/RTA_Cache.h"

#include <algorithm>
#include <unordered_set>

#include "sources/Safety_Performance_Metric/SP_Metric.h"  // ApplyTimeLimitsToTasksExecutionTime

namespace SP_OPT_PA {

// --- PerCoreRTACache build path (role 1 helper) ---
const std::vector<FiniteDist>& PerCoreRTACache::Populate(
    int processor_id, const TaskSet& core_tasks,
    const std::vector<double>& time_limits) {
    processor_id_ = processor_id;

    // Canonicalize HP-first by priority so rta_[i] / hp_tasks_et_conv_vec_[i]
    // / sorted_task_ids_[i] / tl_vec_[i] all align at sorted position i.
    // ProbabilisticRTA_TaskSet_SingleCore scatters its returned rtas by INPUT
    // position but fills hp_tasks_et_conv_vec_ by SORTED position, so passing an
    // already-sorted taskset makes the two align.
    TaskSet core_sorted = core_tasks;
    std::sort(core_sorted.begin(), core_sorted.end(),
              [](const Task& a, const Task& b) {
                  return a.priority < b.priority;
              });

    sorted_task_ids_.clear();
    sorted_task_ids_.reserve(core_sorted.size());
    tl_vec_.clear();
    tl_vec_.reserve(core_sorted.size());
    for (const Task& t : core_sorted) {
        sorted_task_ids_.push_back(t.id);
        // time_limits is indexed by task id (== position in dag.tasks); -1 if
        // the caller passed a shorter vec or this task has no TL.
        tl_vec_.push_back(t.id < static_cast<int>(time_limits.size())
                              ? time_limits[t.id]
                              : -1.0);
    }

    // Drives the 2-arg SingleCore (emits hp_tasks_et_conv_vec_); rta_[i] is the
    // RTA of the task at sorted position i. core_sorted is already sorted, so
    // SingleCore's internal re-sort is a no-op and its input-position scatter
    // lands in the same order as sorted_task_ids_.
    rta_ = ProbabilisticRTA_TaskSet_SingleCore(core_sorted,
                                               hp_tasks_et_conv_vec_);
    return rta_;
}

// ============================================================================
// (1) BUILD — full compute + populate the cache.
// ============================================================================

std::vector<FiniteDist> ComputeRTA_FullAndCache(
    const DAG_Model& dag_tasks, const PriorityVec& priority_assignment,
    const std::vector<double>& time_limits,
    std::unordered_map<int, PerCoreRTACache>& cache) {
    // Apply TLs (by-id) then priorities (sorts HP-first), matching the live
    // path order: ObtainSP_DAG 3-arg → ApplyTimeLimits on the by-id TaskSet →
    // ObtainSP_DAG 2-arg → ... → SingleCore sorts by priority.
    TaskSet tasks_with_tl =
        ApplyTimeLimitsToTasksExecutionTime(dag_tasks.tasks, time_limits);
    TaskSet tasks_prioritized =
        UpdateTaskSetPriorities(tasks_with_tl, priority_assignment);

    std::unordered_map<int, int> task_id2index;
    for (size_t i = 0; i < tasks_prioritized.size(); i++) {
        task_id2index[tasks_prioritized[i].id] = static_cast<int>(i);
    }

    std::unordered_map<int, TaskSet> processor_task_set =
        ExtractTaskSetPerProcessor(tasks_prioritized);

    std::vector<FiniteDist> rtas(tasks_prioritized.size());
    cache.clear();

    for (const auto& kv : processor_task_set) {
        int processor_id = kv.first;
        const TaskSet& tasks_core = kv.second;

        PerCoreRTACache& entry = cache[processor_id];
        entry.Populate(processor_id, tasks_core, time_limits);

        // Scatter this core's rtas back into the flat vector by task id. The
        // cache's rta_ is aligned to sorted position; the flat vector is
        // aligned to dag.tasks position (== task id), so the lookup-by-id is
        // the same as the legacy ProbabilisticRTA_TaskSet scatter.
        for (size_t i = 0; i < tasks_core.size(); i++) {
            int tid = tasks_core[i].id;
            rtas[task_id2index.at(tid)] = entry.Rta()[i];
        }
    }
    return rtas;
}

// ============================================================================
// (2) REUSE QUERY — per-task classification (v0: same-processor check).
// ============================================================================

std::vector<RTAReuseClass> ClassifyReuse(
    const std::unordered_map<int, PerCoreRTACache>& cache,
    const DAG_Model& dag_tasks,
    const std::vector<int>& changed_task_ids) {
    // Empty cache → nothing reusable for anyone.
    if (cache.empty()) {
        return std::vector<RTAReuseClass>(
            dag_tasks.tasks.size(), RTAReuseClass::Recompute);
    }

    // Collect the processorIds that own at least one changed task. A task
    // sharing any of these is conservatively `Recompute` (its core's RTA may
    // have shifted, even if this task sits above the changed position — v0 does
    // not exploit the within-core prefix reuse point yet).
    std::unordered_set<int> changed_processors;
    for (int changed_task_id : changed_task_ids) {
        if (changed_task_id >= 0 &&
            changed_task_id < static_cast<int>(dag_tasks.tasks.size())) {
            changed_processors.insert(
                dag_tasks.tasks[changed_task_id].processorId);
        }
    }

    std::vector<RTAReuseClass> result(dag_tasks.tasks.size(),
                                      RTAReuseClass::RtaReuse);
    for (size_t i = 0; i < dag_tasks.tasks.size(); i++) {
        const Task& task = dag_tasks.tasks[i];
        bool core_in_cache = cache.find(task.processorId) != cache.end();
        bool core_changed = changed_processors.count(task.processorId) > 0;
        if (!core_in_cache || core_changed) {
            result[i] = RTAReuseClass::Recompute;
        }
        // else: this task's core has no changed task and is cached → RtaReuse.
        // (RecomputeWithHpPrefix is declared but not produced by v0.)
    }
    return result;
}

// Patchers (PatchRTA_OneTaskTL, PatchRTA_PriorityMove) are declared in
// RTA_Cache.h and defined in steps 4/5 (Loop B / Loop A dispatch).

}  // namespace SP_OPT_PA

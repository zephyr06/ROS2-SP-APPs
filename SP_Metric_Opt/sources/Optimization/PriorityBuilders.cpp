#include <numeric>
#include <vector>

#include "sources/Optimization/PriorityBuilders.h"

namespace SP_OPT_PA {

// Pick the TL for one task per `policy`: a grid endpoint or -1.0.
static double PickTimeLimit(const Task& task, TimeLimitPolicy policy) {
    if (policy == TimeLimitPolicy::kNone)
        return -1.0;
    const auto& pairs = task.timePerformancePairs;
    if (pairs.empty())
        return -1.0;
    return policy == TimeLimitPolicy::kSmallestGrid ? pairs[0].time_limit
                                                    : pairs.back().time_limit;
}

// P0.10 — shared priority-plan builder. Sorts by (group lock, sort key, avg ET
// ascending), then fills priority_vec + id2time_limit. Mirrors P0.9's
// DeadlineMonotonicPriorityVec comparator shape + the BF DM modes' TL loops.
ResourceOptResult BuildPriorityPlan(const DAG_Model& dag_tasks,
                                    const PriorityBuilderConfig& config) {
    std::vector<int> sorted(dag_tasks.tasks.size());
    std::iota(sorted.begin(), sorted.end(), 0);
    std::sort(sorted.begin(), sorted.end(), [&](int a, int b) {
        const Task& ta = dag_tasks.tasks[a];
        const Task& tb = dag_tasks.tasks[b];
        if (config.group_lock == GroupLock::kImportantFirst &&
            ta.is_important != tb.is_important)
            return ta.is_important;
        double key_a = config.sort_key == SortKey::kDeadline ? ta.deadline
                                                             : ta.period;
        double key_b = config.sort_key == SortKey::kDeadline ? tb.deadline
                                                             : tb.period;
        if (key_a != key_b)
            return key_a < key_b;
        return ta.execution_time_dist.GetAvgValue() <
               tb.execution_time_dist.GetAvgValue();
    });

    ResourceOptResult res;
    for (int idx : sorted) {
        res.priority_vec.push_back(idx);
        res.id2time_limit[idx] = PickTimeLimit(dag_tasks.tasks[idx],
                                               config.tl_policy);
    }
    return res;
}

}  // namespace SP_OPT_PA

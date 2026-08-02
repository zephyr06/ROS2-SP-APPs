#include <stdexcept>
#include <vector>

#include "sources/Optimization/OptimizeFallback.h"
#include "sources/Optimization/PriorityBuilders.h"
#include "sources/Utils/testMy.h"

namespace SP_OPT_PA {

// P0.10 — RM-Fast group-locked {pa, tl}: period key (RM), important-first group
// lock, smallest-grid TL. Delegates to the shared BuildPriorityPlan.
ResourceOptResult RateMonotonicFastGroupLocked(const DAG_Model& dag_tasks) {
    return BuildPriorityPlan(
        dag_tasks, {SortKey::kPeriod, GroupLock::kImportantFirst,
                    TimeLimitPolicy::kSmallestGrid});
}

// Reconstruct the index-ordered tl vector a ResourceOptResult carries: tl[i]
// pairs with dag_tasks.tasks[i] (id == index under the gate's contract). -1.0
// for any task with no stored limit. Mirrors ReconstructTimeLimitVecFromResOpt.
static std::vector<double> TimeLimitVecFromResult(
    const DAG_Model& dag_tasks, const ResourceOptResult& res) {
    std::vector<double> tl(dag_tasks.tasks.size(), -1.0);
    for (size_t i = 0; i < dag_tasks.tasks.size(); i++) {
        int id = dag_tasks.tasks[i].id;
        auto it = res.id2time_limit.find(id);
        if (it != res.id2time_limit.end())
            tl[i] = it->second;
    }
    return tl;
}

// P0.10 §2 — gate BF's result; on FAIL swap to RM-Fast; on double-fail throw.
// Self-contained gate overload (derives RTAs fresh): BF has no live RTA cache,
// mirroring AdoptFallbackIfUnschedulable's post-walk backstop.
ResourceOptResult AdoptRmFastFallbackIfUnschedulable(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters,
    const ResourceOptResult& bf_result) {
    std::vector<double> tl_bf = TimeLimitVecFromResult(dag_tasks, bf_result);
    if (ImportantTasksMeetThresholds(dag_tasks, sp_parameters,
                                     bf_result.priority_vec, tl_bf))
        return bf_result;

    ResourceOptResult rm_fast = RateMonotonicFastGroupLocked(dag_tasks);
    std::vector<double> tl_fast = TimeLimitVecFromResult(dag_tasks, rm_fast);
    if (!ImportantTasksMeetThresholds(dag_tasks, sp_parameters,
                                      rm_fast.priority_vec, tl_fast)) {
        CoutWarning(
            "AdoptRmFastFallbackIfUnschedulable: BF failed the important-task "
            "gate AND the RM-Fast fallback also fails it (no safe priority "
            "assignment exists for this task set). Regenerate the task set.");
        throw std::runtime_error(
            "AdoptRmFastFallbackIfUnschedulable: RM-Fast fallback fails the "
            "important-task gate (no safe solution exists)");
    }
    return rm_fast;
}

}  // namespace SP_OPT_PA

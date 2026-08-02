#include "sources/Optimization/OptimizeFallback.h"
#include "sources/Optimization/PriorityBuilders.h"

namespace SP_OPT_PA {

// P0.10 — RM-Fast group-locked {pa, tl}: period key (RM), important-first group
// lock, smallest-grid TL. Delegates to the shared BuildPriorityPlan.
ResourceOptResult RateMonotonicFastGroupLocked(const DAG_Model& dag_tasks) {
    return BuildPriorityPlan(
        dag_tasks, {SortKey::kPeriod, GroupLock::kImportantFirst,
                    TimeLimitPolicy::kSmallestGrid});
}

}  // namespace SP_OPT_PA

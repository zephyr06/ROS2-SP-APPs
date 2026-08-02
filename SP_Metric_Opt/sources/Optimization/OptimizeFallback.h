#pragma once

#include "sources/Optimization/OptimizeSP_Base.h"

namespace SP_OPT_PA {

// P0.10 — BF's fallback plan: RM-Fast group-locked {priority_vec, id2time_limit}.
// Same important-first group lock as P0.9's DeadlineMonotonicPriorityVec, but
// period-keyed (RM, not DM) and with the "fast" TL (smallest grid option, else
// -1.0) like DM_FAST. A free fn over DAG_Model — BF has no optimizer instance.
// Important tasks occupy the top slots (the lock), RM-ordered within the group;
// non-important fill the lower slots, RM-ordered within theirs; ties by avg ET
// ascending. RM (not DM) because this is a cheap safety floor, not an optim.
ResourceOptResult RateMonotonicFastGroupLocked(const DAG_Model& dag_tasks);

// P0.10 §2 — gate a BF (EnumeratePA_with_TimeLimits) result on important-task
// schedulability; on FAIL swap in the RM-Fast group-locked plan, on PASS keep
// BF. If the RM-Fast plan ITSELF fails the gate → throw std::runtime_error
// (no safe solution exists for this task set — mirror of AdoptFallbackIf-
// Unschedulable's loud double-fail; never silently ship an infeasible result).
// Self-contained gate overload (derives RTAs fresh): BF has no live RTA cache.
ResourceOptResult AdoptRmFastFallbackIfUnschedulable(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters,
    const ResourceOptResult& bf_result);

}  // namespace SP_OPT_PA

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

}  // namespace SP_OPT_PA

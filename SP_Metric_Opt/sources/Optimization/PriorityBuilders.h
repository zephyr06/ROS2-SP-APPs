#pragma once

#include "sources/Optimization/OptimizeSP_Base.h"

namespace SP_OPT_PA {

// P0.10 — the shared shape behind BF's priority modes (DM/DM_FAST/DM_SLOW,
// inline in SimulationOrchestrator) and the RM-Fast fallback
// (RateMonotonicFastGroupLocked). Sort task indices, then fill a
// ResourceOptResult's priority_vec + id2time_limit.

// Sort key for the within-group ordering. kDeadline = DM, kPeriod = RM.
enum class SortKey { kDeadline, kPeriod };

// kImportantFirst locks every important task above every non-important task
// (the P0.9 group lock). kNone is plain key-sorted (the BF DM modes).
enum class GroupLock { kNone, kImportantFirst };

// id2time_limit fill policy. kNone = -1.0 (no TL). kSmallestGrid =
// timePerformancePairs[0] (the "fast" / DM_FAST grid). kLargestGrid =
// timePerformancePairs.back() (the DM_SLOW grid). No perf grid → -1.0.
enum class TimeLimitPolicy { kNone, kSmallestGrid, kLargestGrid };

struct PriorityBuilderConfig {
    SortKey sort_key;
    GroupLock group_lock;
    TimeLimitPolicy tl_policy;
};

// Build a ResourceOptResult from `dag_tasks` per `config`. Sorts task indices
// (group lock first, then the sort key, then avg-ET ascending tiebreak), fills
// priority_vec in rank order, and sets id2time_limit per the TL policy.
// Deterministic; no optimizer state. Used by BF's modes + the RM-Fast fallback.
ResourceOptResult BuildPriorityPlan(const DAG_Model& dag_tasks,
                                    const PriorityBuilderConfig& config);

}  // namespace SP_OPT_PA

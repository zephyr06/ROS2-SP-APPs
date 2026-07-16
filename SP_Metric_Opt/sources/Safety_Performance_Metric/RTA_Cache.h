#pragma once
// Per-core RTA cache (P1.9 — Incremental RTA Patching). See
// agents/active_tasks/P1_9_incremental_rta_patching/{goal,tasks}.md for the
// full design; this header is the API surface.
//
// Why a dedicated header (not RTA.h): the locked cache signature takes
// `PriorityVec`, declared in OptimizeSP_Base.h, which includes SP_Metric.h,
// which includes RTA.h — so putting these decls in RTA.h would be a header
// cycle. RTA_Cache.h is a downstream leaf nothing-upstream includes; it pulls
// in RTA.h (FiniteDist/TaskSet/GetRTA_OneTask/SingleCore/ExtractTaskSetPer
// Processor) + DAG_Model.h (DAG_Model) + OptimizeSP_Base.h (PriorityVec),
// all cycle-free.
//
// API revision 2026-07-15: the role-2 reuse query is a per-task enum vector
// (`ClassifyReuse` → `std::vector<RTAReuseClass>`, indexed by task id), NOT the
// per-core `CacheReuseInfo` struct from the staged 3c. `PerCoreRTACache` is a
// CLASS (private data + accessors), not a struct. `CacheConsistentWith` is
// dropped (derivable: "all tasks RtaReuse"). See tasks.md §"API revision".

#include <unordered_map>
#include <vector>

#include "sources/Optimization/OptimizeSP_Base.h"  // PriorityVec
#include "sources/Safety_Performance_Metric/RTA.h"
#include "sources/TaskModel/DAG_Model.h"

namespace SP_OPT_PA {

// Memoized RTA *output* (no PA-search state) — a pure function of (per-core
// taskset, per-core priority order, per-core ET dists). Within an interval the
// last two reduce to (sorted_task_ids, tl_vec), two of the fields below.
//
// A CLASS (not a struct) so the alignment invariant — sorted_task_ids / rta /
// tl_vec / hp_tasks_et_conv_vec are all length-n parallel arrays indexed by
// sorted-priority position — is centralized in one type instead of trusted to
// every free function. Read accessors + the Populate build path + per-core
// query helpers land now; patcher mutators land with steps 4/5.
class PerCoreRTACache {
   public:
    PerCoreRTACache() = default;
    explicit PerCoreRTACache(int processor_id) : processor_id_(processor_id) {}

    // --- read accessors (const) ---
    int ProcessorId() const { return processor_id_; }
    const std::vector<int>& SortedTaskIds() const { return sorted_task_ids_; }
    const std::vector<double>& TlVec() const { return tl_vec_; }
    const std::vector<FiniteDist>& Rta() const { return rta_; }
    const std::vector<FiniteDist>& HpTasksEtConvVec() const {
        return hp_tasks_et_conv_vec_;
    }

    // --- per-core query helpers (used by the classifier + patchers) ---
    int Size() const { return static_cast<int>(sorted_task_ids_.size()); }

    // Sorted-priority position of `task_id` on this core, or -1 if it is not
    // on this core.
    int PositionOfTask(int task_id) const {
        for (int i = 0; i < Size(); i++) {
            if (sorted_task_ids_[i] == task_id) return i;
        }
        return -1;
    }
    bool ContainsTask(int task_id) const {
        return PositionOfTask(task_id) >= 0;
    }

    // --- build path (role 1 helper) ---
    // Populate this core's cache from `core_tasks` (sorted HP-first by
    // priority — Populate re-sorts defensively so the caller may pass it in any
    // order). Sets processor_id_, sorted_task_ids_, tl_vec_ (looked up by task
    // id from `time_limits`, indexed by task id; -1 if missing), and drives the
    // 2-arg ProbabilisticRTA_TaskSet_SingleCore to fill rta_ +
    // hp_tasks_et_conv_vec_. Because the sort canonicalizes the order, rta_[i]
    // / hp_tasks_et_conv_vec_[i] / sorted_task_ids_[i] / tl_vec_[i] all align
    // at sorted position i. Returns a const ref to the populated rta_.
    const std::vector<FiniteDist>& Populate(
        int processor_id,
        const TaskSet& core_tasks,
        const std::vector<double>& time_limits);

   private:
    int processor_id_ = -1;
    // sorted_task_ids_[i] = task id at priority index i on this core (sorted by
    // pa_vec priority, descending HP-first). The SUFFICIENT proxy for pa_vec:
    // RTA cares about per-core ORDER, not priority VALUES — a cross-core
    // priority shift preserving each core's internal order changes no core's
    // RTA. pa_vec is therefore NOT stored (redundant for validity).
    std::vector<int> sorted_task_ids_;
    // tl_vec_[i] = time limit of sorted_task_ids_[i] at cache-build time; -1 =
    // no TL (the immutable base Gaussian dist). ET-DIST VALIDITY PROXY: within
    // an interval ET dists mutate ONLY via TL application
    // (ApplyTimeLimitsToTasksExecutionTime → GetUnitExecutionTimeDist(tl), a
    // deterministic point mass at tl), so
    //   ET-dist[i] unchanged  <=>  stored tl_vec_[i] == current tl_vec_[i]
    // (tl==-1 = the immutable base dist, which never moves mid-walk). The one
    // exception, the one-time WCET ablation (ApplyWCETAblationIfRequired), is a
    // setup boundary covered by the interval reset.
    std::vector<double> tl_vec_;
    // rta_[i] = RTA distribution of sorted_task_ids_[i]; length ==
    // sorted_task_ids_.size().
    std::vector<FiniteDist> rta_;
    // hp_tasks_et_conv_vec_[i] = HP-ET convolution of sorted_task_ids_[0..i):
    //   hp_tasks_et_conv_vec_[0] == FiniteDist({Value_Proba(0, 1.0)}), and
    //   hp_tasks_et_conv_vec_[i] is exactly what the 3-arg GetRTA_OneTask
    //   consumes. This is the reuse primitive the patchers replay a suffix from.
    std::vector<FiniteDist> hp_tasks_et_conv_vec_;
};

// ============================================================================
// (1) BUILD — full compute + populate the cache (the baseline/descent path).
// ============================================================================

// Full compute: applies pa_vec (UpdateTaskSetPriorities) + tl_vec
// (ApplyTimeLimitsToTasksExecutionTime) to `dag_tasks.tasks`, drives each core's
// PerCoreRTACache::Populate, and returns the flat rta vector (same shape &
// values as ProbabilisticRTA_TaskSet — bit-identical by construction; the
// differential test pins it). Overwrites `cache` (it is its own commit — call
// only for the new champion / new interval). `time_limits[i] == -1` means no TL
// for task i (keep base dist).
std::vector<FiniteDist> ComputeRTA_FullAndCache(
    const DAG_Model& dag_tasks,
    const PriorityVec& priority_assignment,
    const std::vector<double>& time_limits,
    std::unordered_map<int, PerCoreRTACache>& cache);

// ============================================================================
// (2) REUSE QUERY — read-only: "for each task, can its RTA be read from
//     cache?" No RTA. Returns one verdict per task, indexed by task id.
// ============================================================================

// Per-task reuse classification of `cache` against a candidate described by the
// set of tasks whose ET dist or priority changed (`changed_task_ids`). Read-only
// — never mutates the cache, never runs RTA. The return is indexed by task id
// (length == dag_tasks.tasks.size(); the codebase invariant dag.tasks[i].id==i
// makes id == position). Richer than a bool because reuse is not binary.
//
// v0 (2026-07-15) is deliberately SIMPLE: a task is `Recompute` if any changed
// task shares its processorId (conservative — assume the worst within a changed
// core), else `RtaReuse` (changed tasks are on a different core → this task's
// RTA is untouched). Empty cache → all `Recompute`. A task whose core is absent
// from the cache is `Recompute` (partition change / uncached core). This does
// NOT yet exploit the within-core HP-prefix reuse point — the unchanged prefix
// above a changed position is marked `Recompute` too, not `RtaReuse`/
// `RecomputeWithHpPrefix`. That prefix refinement lands paired with the patchers
// (steps 4/5); `RecomputeWithHpPrefix` is declared but NOT produced by v0.
enum class RTAReuseClass {
    RtaReuse,               // cached rta valid verbatim — return it, zero work
    RecomputeWithHpPrefix,  // own rta stale, but hp_tasks_et_conv_vec[p] reusable
    Recompute               // must recompute from scratch (prefix invalidated)
};
std::vector<RTAReuseClass> ClassifyReuse(
    const std::unordered_map<int, PerCoreRTACache>& cache,
    const DAG_Model& dag_tasks,
    const std::vector<int>& changed_task_ids);

// ============================================================================
// (3) UPDATE — cache-update patchers (Loop A + Loop B). MUTATE cache in place.
//     Defined in steps 4/5; declared here so the API surface is complete.
// ============================================================================

// TL patch (Loop B, no PA change): one task's TL changed to `new_tl` at
// priority position `priority_position` on `core`. Reuses
// hp_tasks_et_conv_vec[priority_position], recomputes rta[p..n) via the 3-arg
// GetRTA_OneTask, leaves other cores' cached rta untouched. Writes the suffix
// rta + the rolled-forward hp_tasks_et_conv_vec[p+1..n] + updated tl_vec back
// into cache[core]. Returns the flat rta vector. TDD: bit-identical to a full
// recompute (ComputeRTA_FullAndCache) on the same (pa_vec, tl_vec).
std::vector<FiniteDist> PatchRTA_OneTaskTL(
    int changed_task_id, int core, int priority_position, double new_tl,
    const DAG_Model& dag_tasks,
    std::unordered_map<int, PerCoreRTACache>& cache);

// Priority-move patch (Loop A, PA change): one task moved old_pos→new_pos on
// `core`. Reuses hp_tasks_et_conv_vec[min(old_pos,new_pos)], replays the suffix
// in the new sorted order. The O(N²)→O(k) payoff. Writes the reordered suffix
// back into cache[core]. Returns the flat rta vector. TDD: bit-identical to a
// full recompute.
std::vector<FiniteDist> PatchRTA_PriorityMove(
    int moved_task_id, int core, int old_pos, int new_pos,
    const DAG_Model& dag_tasks,
    std::unordered_map<int, PerCoreRTACache>& cache);

}  // namespace SP_OPT_PA

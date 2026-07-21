#pragma once
// Single-champion RTA cache (P1.9 rev 3). One cache object is bound to ONE
// champion solution. The P1.10 single-change invariant (|diff|<=1 per SP-eval
// vs champion on the serialized path) means Evaluate only ever answers:
// nothing changed (reuse all), or one task's ET and/or priority position
// changed (patch the suffix from that task).
//
// Lives in a dedicated leaf header (not RTA.h) to avoid a header cycle: the
// cache signature takes PriorityVec (OptimizeSP_Base.h → SP_Metric.h → RTA.h),
// so RTA.h cannot depend back on these decls.

#include <unordered_map>
#include <vector>

#include "sources/Optimization/OptimizeSP_Base.h"  // PriorityVec
#include "sources/Safety_Performance_Metric/RTA.h"
#include "sources/TaskModel/DAG_Model.h"

namespace SP_OPT_PA {

// Per-task reuse extent, DERIVED from `TaskSetDifference` (a locator carries no
// verdict — that's a per-task analyst's job). `Evaluate` dispatches on the same
// derivation. Kept as an enum so the per-task vector has a name.
enum class RTAReusePerTask {
    NoReuse,        // Full recompute. No champion; or |diff|>1 (off-path); or,
                    // on the serialized path, every task on the SAME core as the
                    // single change (its HP set shifts).
    FullReuse,      // Return champion rta_[task] verbatim. |diff|==0; or, under
                    // |diff|==1, every task on a DIFFERENT core than the change
                    // (untouched core → identical HP set → identical RTA).
    ReuseHpTasksEt, // RESERVED for the future same-core-suffix refinement (reuse
                    // the champion HP-ET prefix, recompute only the suffix). NOT
                    // emitted by v1, which recomputes the whole changed core.
};

// The single whole-taskset diff `ComputeTaskSetDifference` returns — a set of
// LOCATORS, NOT a decision. `changed_task_id == -1` means |diff|==0 (→
// FullReuse); otherwise the one changed task + its core + its old/new per-core
// priority positions. The cache's ACTION is Evaluate's dispatch derived from
// these locators, not a field on the struct.
struct TaskSetDifference {
    int changed_task_id;  // the one moved/ET-changed task; -1 iff |diff|==0
    int core;     // processorId of the changed task; -1 iff |diff|==0
    int old_pos;  // changed task's position in the CHAMPION's per-core order
    int new_pos;  // changed task's position in the CANDIDATE's per-core order
};

// Memoized RTA output (no PA-search state) bundled to ONE champion. Stores the
// champion (dag, pa, tl) triple + its flat RTA + per-core HP-prefix
// checkpoints. `Evaluate` reads champion state to patch a candidate that
// differs by at most one task; `AdoptChampion` promotes a candidate to
// champion. Evaluate never mutates committed champion state, so a rejected
// candidate's RTA never touches it (commit is AdoptChampion only).
class RTACache {
   public:
    RTACache() = default;

    // Compute the full N-task RTA for (dag, pa, tl) and store it + the triple +
    // the per-core HP-prefix checkpoints. Overwrites all prior state. Expensive
    // (full RTA) — call once per interval / new champion, NOT per candidate.
    // `tl[i] == -1` means no TL for task i (keep its base dist). Returns the
    // flat rta vector indexed by task id (bit-identical to
    // ProbabilisticRTA_TaskSet).
    const std::vector<FiniteDist>& Initialize(const DAG_Model& dag_tasks,
                                              const PriorityVec& pa,
                                              const std::vector<double>& tl);

    // Cheap commit: promote the candidate (dag, pa, tl, rtas) the caller just
    // evaluated to champion, with NO full RTA. `rtas` MUST be the vector a prior
    // Evaluate/Initialize returned for this same triple; the cache stores it as
    // `rta_` then rebuilds `hp_prefix_per_core_` by re-rolling the per-core
    // ET-convolution from (dag, pa, tl) — O(N) convolves, no RTA work.
    void AdoptChampion(const DAG_Model& dag_tasks, const PriorityVec& pa,
                       const std::vector<double>& tl,
                       const std::vector<FiniteDist>& rtas);

    // Compute the candidate's full flat RTA for (dag, pa, tl), exploiting the
    // single-change invariant vs the stored champion. Does NOT mutate champion
    // state — writes the candidate RTA to `candidate_rta_` and returns it;
    // commit via AdoptChampion. Verdict-driven: the per-task reuse verdict is
    // obtained from ClassifyReusePerTask (the single source of that decision),
    // then Evaluate seeds every task with the champion RTA (FullReuse tasks keep
    // it) and recomputes the NoReuse tasks via GetRTA_OneTask in candidate
    // priority order. A future same-core-suffix refinement only needs the
    // verdict to gain a ReuseHpTasksEt value + a branch in the recompute loop.
    //   • no champion → Initialize (full compute).
    // Returned ref is valid until the next Evaluate/Initialize/AdoptChampion.
    const std::vector<FiniteDist>& Evaluate(const DAG_Model& dag_tasks,
                                            const PriorityVec& pa,
                                            const std::vector<double>& tl);

    // --- read accessors (const) ------------------------------------------
    bool HasChampion() const { return !rta_.empty(); }
    const std::vector<FiniteDist>& Rta() const { return rta_; }

    // The difference between the candidate (dag, pa, tl) and the stored
    // champion, WITHOUT computing any RTA. The pure query half of Evaluate.
    // Returns a LOCATOR set (no verdict field): `changed_task_id == -1` iff the
    // candidate is identical to the champion (|diff|==0); otherwise the one
    // changed task + its core + its old/new per-core priority positions.
    //   • no champion → {changed_task_id:-1, ...}
    //   • |diff|==0   → {changed_task_id:-1, ...}
    //   • |diff|==1   → locators filled in (old_pos==new_pos for an ET-only move)
    //   • |diff|>1    → THROWS (violates the P1.10 single-change invariant; the
    //     cache only serves |diff|<=1). Use IsSingleTaskChange to ask first.
    // Pure; may be called on a cache with no champion (returns {-1,...}).
    TaskSetDifference ComputeTaskSetDifference(
        const DAG_Model& dag_tasks, const PriorityVec& pa,
        const std::vector<double>& tl) const;

    // The single shared single-change analyzer. Returns true + fills `out` iff
    // the candidate differs from the champion by AT MOST one task (ET and/or
    // per-core priority position); false otherwise. No-champion → false. NEVER
    // throws — the throwing boundary is ComputeTaskSetDifference. Algorithm:
    //   1. ET diff: FindTaskWithDifferentEt. >1 ET-changed task → false.
    //   2. Per-core priority order: a core with differing sizes → false (task
    //      migrated cores). At most ONE core may differ.
    //   3. Remove-one-compare-rest:
    //      - 1 ET-diff task X: remove X from both per-core orders; if the rest
    //        still differs, a SEPARATE task also moved → false (X's own priority
    //        move is absorbed: combined ET+move = single change).
    //      - 0 ET-diff (pure priority move): at the first mismatch i, the moved
    //        task is champ[i] or cand[i] — try removing each; if either makes
    //        the rest match, that's the single move; if neither → false.
    //      - 0 ET diff + every core identical → |diff|==0, out.changed_task_id
    //        stays -1, returns true.
    //   4. Cross-check: the priority-move core (if any) and the ET-diff task's
    //      core must be the same single core, else >1 change → false.
    bool IsSingleTaskChange(const DAG_Model& dag_tasks,
                            const PriorityVec& pa,
                            const std::vector<double>& tl,
                            TaskSetDifference& out) const;

    // Per-task reuse view: result[i] = reuse extent for task i. DERIVED from
    // the locator set ComputeTaskSetDifference returns:
    //   • no champion             → every task NoReuse
    //   • changed_task_id == -1   → every task FullReuse (|diff|==0)
    //   • changed_task_id >= 0    → DIFFERENT core than diff.core is FullReuse;
    //                               SAME core is NoReuse (its HP set shifted).
    // Pure; may be called on a cache with no champion (returns all-NoReuse).
    std::vector<RTAReusePerTask> ClassifyReusePerTask(
        const DAG_Model& dag_tasks, const PriorityVec& pa,
        const std::vector<double>& tl) const;

   private:
    // The champion is carried ONLY in its baked forms (champ_prioritized_ /
    // champ_tasks_baked_ / champ_per_core_) + rta_ + hp_prefix_per_core_. The
    // raw (dag, pa, tl) triple is consumed at bake time and not stored: pa and
    // tl are read once by the bake (UpdateTaskSetPriorities /
    // ApplyTimeLimitsToTasksExecutionTime) and never again, and Evaluate /
    // IsSingleTaskChange read the baked forms, not a stored pa/tl. A future
    // caller that needs the champion's pa/tl back should add a const accessor
    // rather than carry dead state here.

    // Champion tasks TL-baked + pa-sorted (the exact `tasks_prioritized`
    // Initialize/AdoptChampion built). Invariant across one champion lifetime,
    // so Evaluate's reindex reads this instead of re-baking the champion DAG
    // every call. Empty iff no champion.
    TaskSet champ_prioritized_;
    // Champion tasks TL-baked in CANONICAL (task-id) order (the `tasks_baked`
    // built before pa-sorting). Invariant across one champion lifetime, so
    // IsSingleTaskChange reads this instead of re-baking every call
    // (FindTaskWithDifferentEt walks .tasks[i] by index → needs canonical, not
    // pa-sorted, order). Empty iff no champion.
    TaskSet champ_tasks_baked_;
    // Champion per-core priority order: per processorId, the task ids on that
    // core sorted ascending by priority value (the exact artifact
    // PerCoreOrderOfPrioritized(champ_prioritized_) produces). Invariant across
    // one champion lifetime, so IsSingleTaskChange reads this instead of
    // rebuilding the champion partition every call; only the CANDIDATE side is
    // rebuilt per call. Empty iff no champion.
    std::unordered_map<int, std::vector<int>> champ_per_core_;
    // The champion RTA, flat by task id (rta_[i] = RTA of task i).
    std::vector<FiniteDist> rta_;
    // Per-core HP-prefix checkpoints (the reuse primitive):
    // hp_prefix_per_core_[processorId][i] = HP-ET convolution of the tasks
    // sorted above position i on that core = exactly what 3-arg GetRTA_OneTask
    // consumes. Built by Initialize, consumed by Evaluate's patch branch,
    // rolled forward by AdoptChampion.
    std::unordered_map<int, std::vector<FiniteDist>> hp_prefix_per_core_;

    // Candidate RTA buffer (Evaluate's output; champion rta_ untouched until
    // AdoptChampion). AdoptChampion's `rtas` arg is a vector returned here.
    std::vector<FiniteDist> candidate_rta_;

    // Per-core priority ORDER from (dag, pa): for each processorId, the task
    // ids on that core sorted ascending by priority value (lower = higher
    // priority). Shared by ComputeTaskSetDifference (candidate side) and
    // Evaluate's patch branch (candidate order for the suffix recompute). Pure.
    std::unordered_map<int, std::vector<int>> PerCoreOrderFromPa(
        const DAG_Model& dag_tasks, const PriorityVec& pa) const;
    // Per-core order from an ALREADY-prioritized TaskSet (no re-sort). Shared
    // body of PerCoreOrderFromPa + the champion-order cache build
    // (champ_prioritized_ is already pa-sorted, so re-sorting it would be
    // redundant work). Pure.
    std::unordered_map<int, std::vector<int>> PerCoreOrderOfPrioritized(
        const TaskSet& prioritized) const;

    // Rebuild hp_prefix_per_core_ from `tasks_prioritized` by re-rolling the
    // per-core ET-convolution. Shared by Initialize + AdoptChampion.
    void RebuildPrefixes(const TaskSet& tasks_prioritized);
};

}  // namespace SP_OPT_PA

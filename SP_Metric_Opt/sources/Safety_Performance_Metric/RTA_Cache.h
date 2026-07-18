#pragma once
// Single-champion RTA cache (P1.9 — Incremental RTA Patching, rev 3). See
// agents/active_tasks/P1_9_incremental_rta_patching/{goal,tasks}.md.
//
// One cache object bundles to ONE champion solution. The P1.10 single-change
// invariant (`|diff|<=1` per SP-eval vs champion on the serialized path) means
// Evaluate only ever answers: nothing changed? (reuse all) or one task's ET
// and/or priority position changed? (patch the suffix from that task).
//
// Lives in a dedicated leaf header (not RTA.h) to avoid a header cycle:
// the cache signature takes `PriorityVec` (OptimizeSP_Base.h → SP_Metric.h →
// RTA.h), so RTA.h cannot depend back on these decls.

#include <unordered_map>
#include <vector>

#include "sources/Optimization/OptimizeSP_Base.h"  // PriorityVec
#include "sources/Safety_Performance_Metric/RTA.h"
#include "sources/TaskModel/DAG_Model.h"

namespace SP_OPT_PA {

// Per-task reuse extent, DERIVED from `TaskSetDifference` inside
// `ClassifyReusePerTask` (a locator has no business carrying a per-task verdict
// — that's a per-task analyst's job, not the diff's). `Evaluate` dispatches on
// the same derivation. Kept as an enum so the per-task vector has a name.
enum class RTAReusePerTask {
    // Full recompute of this task's RTA. Fires for every task when the cache
    // has no champion OR (off-path) the candidate differs by more than one
    // task. On the serialized path it fires for every task on the SAME core as
    // the single change (its HP set shifts).
    NoReuse,
    // Return the champion's stored rta_[task] verbatim. Fires under |diff|==0
    // for every task, and under |diff|==1 for every task on a DIFFERENT core
    // than the change (untouched core → identical HP set → identical RTA).
    FullReuse,
    // Reserved for the future same-core-suffix refinement (reuse the champion's
    // HP-ET prefix at min(old_pos,new_pos), recompute only the suffix). NOT
    // emitted by ClassifyReusePerTask v1, which recomputes the whole changed
    // core (every same-core task → NoReuse). Kept so the enum names the v2 path.
    ReuseHpTasksEt,
};

// The single whole-taskset diff `ComputeTaskSetDifference` returns — a set of
// LOCATORS, NOT a decision. `changed_task_id == -1` means |diff|==0 (no change
// → FullReuse); otherwise the one changed task + its core + its old/new
// per-core priority positions. This is a *difference* (a property of the pair);
// the cache's ACTION is Evaluate's dispatch derived from these locators, not a
// field on the struct.
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
// champion. Pure memoization: Evaluate never mutates committed champion state,
// so a rejected candidate's RTA never touches it (commit is AdoptChampion
// only).
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
    // evaluated to champion, with NO full RTA. `rtas` MUST be the vector a
    // prior Evaluate/Initialize returned for this same triple; the cache stores
    // it as `rta_` then rebuilds `hp_prefix_per_core_` by re-rolling the per-core
    // ET-convolution from (dag, pa, tl) — O(N) convolves, no RTA work. On the
    // serialized path the single champion writer is CommitIncumbent
    // (OptimizeSP_TL_Incre.cpp:699), so the AdoptChampion call goes inside it.
    void AdoptChampion(const DAG_Model& dag_tasks, const PriorityVec& pa,
                       const std::vector<double>& tl,
                       const std::vector<FiniteDist>& rtas);

    // Compute the candidate's full flat RTA for (dag, pa, tl), exploiting the
    // single-change invariant vs the stored champion. Does NOT mutate champion
    // state — writes the candidate RTA to `candidate_rta_` and returns it;
    // commit via AdoptChampion. Verdict-driven dispatch: Evaluate calls
    // ClassifyReusePerTask to get the per-task reuse verdict, seeds every task
    // with the champion RTA (FullReuse tasks keep it), then recomputes the
    // NoReuse tasks via GetRTA_OneTask in candidate priority order so each
    // recompute sees the correct candidate-ET HP set. This keeps Evaluate a
    // mechanical per-task dispatcher: a future same-core-suffix refinement
    // only needs ClassifyReusePerTask to emit ReuseHpTasksEt + a branch here,
    // not a rewrite of the dispatch loop.
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
    //
    // The single-change check (the algorithm IsSingleTaskChange used to carry
    // separately) is now INLINED here via TryComputeSingleChange — see its
    // doc for the find-ET-diff → remove-one → compare-rest algorithm.
    //   • no champion             → {changed_task_id:-1, ...} (callers read as
    //     "no verdict / all-NoReuse"; distinct from |diff|==0 only by context)
    //   • |diff|==0               → {changed_task_id:-1, ...}
    //   • |diff|==1               → locators filled in (core = changed task's
    //     core; old_pos==new_pos for an ET-only move)
    //   • |diff|>1                → THROWS (violates the P1.10 single-change
    //     invariant; the cache only serves the |diff|<=1 cases). Use
    //     IsSingleTaskChange to ask first if you cannot prove the invariant.
    // Pure; may be called on a cache with no champion (returns {-1,...}, no throw).
    TaskSetDifference ComputeTaskSetDifference(
        const DAG_Model& dag_tasks, const PriorityVec& pa,
        const std::vector<double>& tl) const;

    // Boolean predicate: does the candidate differ from the stored champion by
    // AT MOST one task (ET and/or per-core priority position)? true for |diff|
    // in {0,1} (the two cases Evaluate serves), false for no champion or >1.
    // The P1.10 single-change invariant as a query instead of an assertion —
    // AssertSingleChangeInvariant (OptimizeSP_TL_Incre.cpp:270, debugMode-only)
    // throws on the >1 case; IsSingleTaskChange lets a caller ASK (e.g. a
    // budget-aware caller that wants to skip a multi-change candidate before
    // any RTA work). Thin non-throwing wrapper over TryComputeSingleChange;
    // ComputeTaskSetDifference throws when this returns false and a champion
    // exists.
    bool IsSingleTaskChange(const DAG_Model& dag_tasks, const PriorityVec& pa,
                            const std::vector<double>& tl) const;

    // Per-task reuse view: result[i] = reuse extent for task i. DERIVED from
    // the locator set ComputeTaskSetDifference returns (no klass on the diff):
    //   • no champion             → every task NoReuse
    //   • changed_task_id == -1   → every task FullReuse (|diff|==0)
    //   • changed_task_id >= 0    → every task on a DIFFERENT core than
    //     `diff.core` is FullReuse; every task on the SAME core is NoReuse
    //     (its HP set shifted under the single change).
    // (v1: cross-core reuse; same-core suffix reuse is a later refinement —
    // ReuseHpTasksEt is reserved for it and not emitted here.)
    // Pure; may be called on a cache with no champion (returns all-NoReuse).
    std::vector<RTAReusePerTask> ClassifyReusePerTask(
        const DAG_Model& dag_tasks, const PriorityVec& pa,
        const std::vector<double>& tl) const;

   private:
    // The champion triple:
    DAG_Model dag_champion_;
    PriorityVec pa_champion_;
    std::vector<double> tl_champion_;
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
    // priority; matches ProbabilisticRTA_TaskSet_SingleCore's sort at
    // RTA.cpp:72-74). Shared by ComputeTaskSetDifference (candidate side) and
    // Evaluate's patch branch (candidate order for the suffix recompute). Pure.
    std::unordered_map<int, std::vector<int>> PerCoreOrderFromPa(
        const DAG_Model& dag_tasks, const PriorityVec& pa) const;

    // Rebuild hp_prefix_per_core_ from `tasks_prioritized` (TL-baked + pa-sorted)
    // by re-rolling the per-core ET-convolution. Shared by Initialize +
    // AdoptChampion (both rebuild prefixes after setting the champion triple).
    void RebuildPrefixes(const TaskSet& tasks_prioritized);

    // The single shared single-change analyzer (the merged IsSingleTaskChange
    // algorithm). Returns true + fills `out` with the change locators iff the
    // candidate differs from the champion by AT MOST one task (ET and/or
    // per-core priority position); returns false otherwise (and leaves `out`
    // untouched). No-champion → returns false. NEVER throws — the throwing
    // boundary is ComputeTaskSetDifference. Algorithm:
    //   1. ET diff: FindTaskWithDifferentEt(champion_baked, candidate_baked).
    //      If >1 task's ET moved → false (would-be |diff|>1).
    //   2. Per-core priority order: for each core, the candidate's order vs the
    //      champion's. A core with differing sizes → false (task migrated
    //      cores, a multi-change). At most ONE core may differ.
    //   3. The "remove one task from both vectors, compare the rest" test:
    //      - 1 ET-diff task X (the known change): remove X from both the
    //        champion and candidate per-core order; if the rest still differs,
    //        a SEPARATE task also moved → false. Absorbs X's own priority move
    //        (combined ET+move → single change).
    //      - 0 ET-diff task (pure priority move): scan left-to-right; at the
    //        first mismatch i, the moved task is champ[i] OR cand[i] — try
    //        removing each; if either makes the rest match, that's the single
    //        move (locate moved_task_id, old_pos, new_pos); if neither → false.
    //      - 0 ET diff + every core identical → |diff|==0, out.changed_task_id
    //        stays -1, returns true.
    //   4. Cross-check: the priority-move core (if any) and the ET-diff task's
    //      core must be the same single core, else >1 change → false.
    // out.changed_task_id == -1 after a true return ⟺ |diff|==0.
    bool TryComputeSingleChange(const DAG_Model& dag_tasks,
                                const PriorityVec& pa,
                                const std::vector<double>& tl,
                                TaskSetDifference& out) const;
};

}  // namespace SP_OPT_PA

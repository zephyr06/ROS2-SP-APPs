#pragma once
// Single-champion RTA cache. Bound to ONE champion solution. The serialized
// optimizer's single-change invariant (|diff|<=1 vs champion) means Evaluate
// only answers: nothing changed (reuse all), or one task's ET and/or priority
// position changed (patch the suffix from that task).
//
// Leaf header (not RTA.h) to avoid a cycle: the cache takes PriorityVec
// (OptimizeSP_Base.h -> SP_Metric.h -> RTA.h), so RTA.h can't depend back here.

#include <vector>

#include "sources/Optimization/OptimizeSP_Base.h"  // PriorityVec
#include "sources/Safety_Performance_Metric/RTA.h"
#include "sources/TaskModel/DAG_Model.h"

namespace SP_OPT_PA {

// Per-task reuse extent derived from `TaskSetDifference`. Evaluate dispatches on
// the same derivation. Enum so the per-task vector has a name.
enum class RTAReusePerTask {
    NoReuse,    // Full recompute. No champion; |diff|>1; or every task on the
                // SAME core as the single change (its HP set shifts).
    FullReuse,  // champion rta_[task] verbatim. |diff|==0; or, under |diff|==1,
                // every task on a DIFFERENT core (untouched HP set).
    ReuseHpTasksEt,  // RESERVED: reuse champion HP-ET prefix, recompute suffix.
                     // NOT emitted by v1.
};

// Whole-taskset diff: LOCATORS, not a verdict. changed_task_id==-1 means |diff|==0.
// Evaluate's dispatch is derived from these, not a field on the struct.
struct TaskSetDifference {
    int changed_task_id;  // the moved/ET-changed task; -1 iff |diff|==0
    int core;             // processorId of the changed task; -1 iff |diff|==0
    int old_pos;  // changed task's position in the CHAMPION's per-core order
    int new_pos;  // changed task's position in the CANDIDATE's per-core order
    bool has_et_diff;  // the changed task's ET differs from the champion (Rule A);
                       // false iff pure priority move (Rule B). false iff |diff|==0.
                       // Populated by IsSingleTaskChange (which already computes the
                       // ET diff); a locator-class fact, not a verdict.
};

// Full-champion state: the members AdoptChampion/Initialize overwrite. All
// members are copyable, so the whole-cache reject-path backup (a full RTACache
// copy) deep-copies the champion correctly. candidate_rta_ is deliberately NOT
// here: it is a scratch buffer fully overwritten before read on every Evaluate,
// so it never needs copying.
struct ChampionState {
    std::vector<FiniteDist> rta;
    TaskSet champ_prioritized;
    TaskSet champ_tasks_baked;
    // Flat per-core vectors indexed by processorId (dense 0-based). Stay
    // copyable so the whole-cache backup deep-copies correctly.
    std::vector<std::vector<int>> champ_per_core;
    std::vector<std::vector<FiniteDist>> hp_prefix_per_core;
};

// Memoized RTA output bundled to ONE champion: the (dag, pa, tl) triple + flat
// RTA + per-core HP-prefix checkpoints. Evaluate patches a candidate that
// differs by <=1 task; AdoptChampion promotes a candidate. Evaluate never
// mutates committed champion state, so a rejected candidate's RTA never touches it.
class RTACache {
   public:
    RTACache() = default;

    // Full N-task RTA for (dag, pa, tl) + store the triple + flat rta + per-core
    // HP-prefix checkpoints. Overwrites all prior state. Expensive — call once
    // per interval/champion, NOT per candidate. tl[i]==-1 means no TL for task i.
    // Returns the flat rta indexed by task id (bit-identical to
    // ProbabilisticRTA_TaskSet).
    const std::vector<FiniteDist>& Initialize(const DAG_Model& dag_tasks,
                                              const PriorityVec& pa,
                                              const std::vector<double>& tl);

    // Cheap commit: promote the candidate (dag, pa, tl, rtas) to champion with
    // NO full RTA. rtas MUST be the vector a prior Evaluate/Initialize returned
    // for this triple; stored as champion_.rta, then hp_prefix_per_core is
    // rebuilt by re-rolling the per-core ET-convolution (O(N) convolves, no RTA).
    void AdoptChampion(const DAG_Model& dag_tasks, const PriorityVec& pa,
                       const std::vector<double>& tl,
                       const std::vector<FiniteDist>& rtas);

    // Candidate's full flat RTA for (dag, pa, tl), exploiting the single-change
    // invariant vs the stored champion. Does NOT mutate champion state — writes
    // candidate_rta_ and returns it; commit via AdoptChampion. Verdict-driven:
    // ClassifyReusePerTask is the single source of the reuse decision; Evaluate
    // seeds every task with the champion RTA (FullReuse keeps it) and recomputes
    // NoReuse tasks via GetRTA_OneTask in candidate priority order.
    //   - no champion -> Initialize (full compute).
    // Ref valid until the next Evaluate/Initialize/AdoptChampion.
    const std::vector<FiniteDist>& Evaluate(const DAG_Model& dag_tasks,
                                            const PriorityVec& pa,
                                            const std::vector<double>& tl);

    // --- read accessors (const) ------------------------------------------
    bool HasChampion() const { return !champion_.rta.empty(); }
    const std::vector<FiniteDist>& Rta() const { return champion_.rta; }

    // Difference between candidate (dag, pa, tl) and champion, WITHOUT computing
    // RTA. The pure query half of Evaluate. Returns LOCATORS (no verdict):
    // changed_task_id==-1 iff |diff|==0; otherwise the changed task + core +
    // old/new per-core priority positions.
    //   - no champion -> {changed_task_id:-1, ...}
    //   - |diff|==0   -> {changed_task_id:-1, ...}
    //   - |diff|==1   -> locators filled (old_pos==new_pos for an ET-only move;
    //     has_et_diff flags Rule A vs Rule B).
    //   - |diff|>1    -> THROWS (violates the single-change invariant; the cache
    //     only serves |diff|<=1). Guard with IsSingleTaskChange first.
    // Pure; may be called with no champion (returns {-1,...}).
    TaskSetDifference ComputeTaskSetDifference(
        const DAG_Model& dag_tasks, const PriorityVec& pa,
        const std::vector<double>& tl) const;

    // Single shared single-change analyzer. Returns true + fills `out` iff the
    // candidate differs from the champion by AT MOST one task (ET and/or
    // per-core priority position); false otherwise. No-champion -> false. Never
    // throws — the throwing boundary is ComputeTaskSetDifference. Algorithm is
    // inline in the .cpp (ET diff via FindTaskWithDifferentEt, then a
    // remove-one-compare-rest pass on the per-core priority orders).
    bool IsSingleTaskChange(const DAG_Model& dag_tasks, const PriorityVec& pa,
                            const std::vector<double>& tl,
                            TaskSetDifference& out) const;

    // Per-task reuse view: result[i] = reuse extent for task i, derived from
    // ComputeTaskSetDifference:
    //   - no champion             -> every task NoReuse
    //   - changed_task_id == -1   -> every task FullReuse (|diff|==0)
    //   - changed_task_id >= 0    -> DIFFERENT core than diff.core is FullReuse;
    //   on the SAME core as the change, narrowed by the master rules:
    //     Rule A (diff.has_et_diff): pos < p_min -> FullReuse, pos >= p_min -> NoReuse.
    //       (ET changed: every task at/above the changed task's min position has an
    //       altered HP-ET convolution; tasks above reuse verbatim.)
    //     Rule B (pure priority move): pos < p_min OR pos > p_max -> FullReuse,
    //       p_min <= pos <= p_max -> NoReuse.
    //       (Priority move: only the shift window's HP sets change.)
    // Pure; may be called with no champion (returns all-NoReuse).
    std::vector<RTAReusePerTask> ClassifyReusePerTask(
        const DAG_Model& dag_tasks, const PriorityVec& pa,
        const std::vector<double>& tl) const;

   private:
    // Champion lives ONLY in `champion_`. The raw (dag, pa, tl) triple is
    // consumed at bake time and not stored: pa/tl are read once by the bake and
    // never again, and Evaluate/IsSingleTaskChange read the baked forms. A caller
    // needing the champion's pa/tl back should add a const accessor rather than
    // carry dead state here. On a rejected sub-incremental walk step the caller
    // restores the whole cache by copy-assigning a pre-walk RTACache backup;
    // struct-copy deep-copies all members, so the backup is drift-proof by
    // construction.
    ChampionState champion_;

    // Candidate RTA buffer (Evaluate's output; champion_.rta untouched until
    // AdoptChampion). AdoptChampion's rtas arg is a vector returned here.
    std::vector<FiniteDist> candidate_rta_;

    // Per-core priority ORDER from (dag, pa): for each processorId, task ids on
    // that core sorted ascending by priority value (lower = higher priority).
    // Shared by ComputeTaskSetDifference (candidate side) and Evaluate's patch
    // branch. Pure. Flat vector indexed by processorId (dense 0-based).
    std::vector<std::vector<int>> PerCoreOrderFromPa(
        const DAG_Model& dag_tasks, const PriorityVec& pa) const;
    // Per-core order from an ALREADY-prioritized TaskSet (no re-sort). Shared
    // body of PerCoreOrderFromPa + the champion-order cache build
    // (champ_prioritized is already pa-sorted). Pure.
    std::vector<std::vector<int>> PerCoreOrderOfPrioritized(
        const TaskSet& prioritized) const;

    // Write the 4 champion baked-form members of champion_ from (dag, pa, tl):
    // champ_tasks_baked (canonical TL-bake, read by FindTaskWithDifferentEt by
    // index), champ_prioritized (pa-sorted, read by Evaluate's reindex +
    // RebuildPrefixes), champ_per_core (read by IsSingleTaskChange), and
    // hp_prefix_per_core. Shared by Initialize (which then computes champion_.rta
    // from champ_prioritized) and AdoptChampion (which takes rtas as a param) —
    // the two differ only on WHERE champion_.rta comes from. Pure extract-method.
    void BakeChampionForms(const DAG_Model& dag_tasks, const PriorityVec& pa,
                           const std::vector<double>& tl);

    // Rebuild champion_.hp_prefix_per_core from `tasks_prioritized` by re-rolling
    // the per-core ET-convolution. Shared by Initialize + AdoptChampion (via
    // BakeChampionForms).
    void RebuildPrefixes(const TaskSet& tasks_prioritized);
};

}  // namespace SP_OPT_PA

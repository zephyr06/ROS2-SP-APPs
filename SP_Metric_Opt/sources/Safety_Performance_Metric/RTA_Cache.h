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

#include <memory>
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
    NoReuse,    // Full recompute. No champion; or |diff|>1 (off-path); or,
                // on the serialized path, every task on the SAME core as the
                // single change (its HP set shifts).
    FullReuse,  // Return champion rta_[task] verbatim. |diff|==0; or, under
                // |diff|==1, every task on a DIFFERENT core than the change
                // (untouched core → identical HP set → identical RTA).
    ReuseHpTasksEt,  // RESERVED for the future same-core-suffix refinement
                     // (reuse the champion HP-ET prefix, recompute only the
                     // suffix). NOT emitted by v1, which recomputes the whole
                     // changed core.
};

// The single whole-taskset diff `ComputeTaskSetDifference` returns — a set of
// LOCATORS, NOT a decision. `changed_task_id == -1` means |diff|==0 (→
// FullReuse); otherwise the one changed task + its core + its old/new per-core
// priority positions. The cache's ACTION is Evaluate's dispatch derived from
// these locators, not a field on the struct.
struct TaskSetDifference {
    int changed_task_id;  // the one moved/ET-changed task; -1 iff |diff|==0
    int core;             // processorId of the changed task; -1 iff |diff|==0
    int old_pos;  // changed task's position in the CHAMPION's per-core order
    int new_pos;  // changed task's position in the CANDIDATE's per-core order
};

// The full-champion state: the 5 members AdoptChampion/Initialize overwrite.
// This is BOTH the cache's live storage type (RTACache::champion_) AND the
// Memento the Transaction snapshots — so capture/restore are a single struct
// copy/move, and a future 6th champion member cannot drift between live state
// and the snapshot (the Memento IS the state). `candidate_rta_` is DELIBERATELY
// NOT a member of this struct: it is a scratch buffer fully overwritten before
// read on every Evaluate (reindex fills FullReuse slots, recompute overwrites
// NoReuse slots), so it never needs rollback — excluding it removes one of the
// three heavy vector<FiniteDist> copies from the snapshot. See
// RTACache::Transaction.
struct ChampionState {
    std::vector<FiniteDist> rta;
    TaskSet champ_prioritized;
    TaskSet champ_tasks_baked;
    std::unordered_map<int, std::vector<int>> champ_per_core;
    std::unordered_map<int, std::vector<FiniteDist>> hp_prefix_per_core;
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
    // evaluated to champion, with NO full RTA. `rtas` MUST be the vector a
    // prior Evaluate/Initialize returned for this same triple; the cache stores
    // it as `champion_.rta` then rebuilds `champion_.hp_prefix_per_core` by
    // re-rolling the per-core ET-convolution from (dag, pa, tl) — O(N)
    // convolves, no RTA work.
    void AdoptChampion(const DAG_Model& dag_tasks, const PriorityVec& pa,
                       const std::vector<double>& tl,
                       const std::vector<FiniteDist>& rtas);

    // Compute the candidate's full flat RTA for (dag, pa, tl), exploiting the
    // single-change invariant vs the stored champion. Does NOT mutate champion
    // state — writes the candidate RTA to `candidate_rta_` and returns it;
    // commit via AdoptChampion. Verdict-driven: the per-task reuse verdict is
    // obtained from ClassifyReusePerTask (the single source of that decision),
    // then Evaluate seeds every task with the champion RTA (FullReuse tasks
    // keep it) and recomputes the NoReuse tasks via GetRTA_OneTask in candidate
    // priority order. A future same-core-suffix refinement only needs the
    // verdict to gain a ReuseHpTasksEt value + a branch in the recompute loop.
    //   • no champion → Initialize (full compute).
    // Returned ref is valid until the next Evaluate/Initialize/AdoptChampion.
    const std::vector<FiniteDist>& Evaluate(const DAG_Model& dag_tasks,
                                            const PriorityVec& pa,
                                            const std::vector<double>& tl);

    // --- read accessors (const) ------------------------------------------
    bool HasChampion() const { return !champion_.rta.empty(); }
    const std::vector<FiniteDist>& Rta() const { return champion_.rta; }

    // --- transactions (lazy copy-on-write) ------------------------------
    // A scoped RAII guard for the speculative serialized walk
    // (EvaluateTimeLimitConfig_SubIncremental). In-walk AdoptChampion calls
    // advance the champion SPECULATIVELY; if UpdateRecords rejects the trial
    // config they must roll back so the cache champion stays in sync with
    // res_opt_. Opening a Transaction lets the caller attempt those adopts and
    // either keep them (Commit) or undo them (~Transaction on scope exit).
    //
    // LAZY copy-on-write: the pre-tx champion snapshot is captured on the FIRST
    // champion mutation in scope (the common reject-without-adopt path pays
    // ZERO copy — snapshot stays null and ~Transaction is a no-op). The capture
    // fires from RTACache's own mutators via SnapshotPreMutationStateIfOpen()
    // — see that method for why the hook lives on the cache side.
    //
    // The Transaction class itself is a THIN RAII wrapper: it holds no state
    // except `committed_` and calls RTACache's public Begin/Commit/Rollback
    // API. No friend, no back-pointer on the wrapper. No nesting (Begin asserts
    // none is open). Non-copyable, non-movable.
    void BeginTransaction();
    void CommitTransaction();
    void RollbackTransaction();
    bool InTransaction() const { return in_transaction_; }

    class Transaction {
       public:
        explicit Transaction(RTACache& cache) : cache_(cache) {
            cache_.BeginTransaction();
        }
        // The dtor is the SINGLE resolution point: it dispatches to
        // CommitTransaction (keep mutations) or RollbackTransaction (undo),
        // either of which closes the tx on the cache. This guarantees the
        // cache-side state is cleaned up exactly once even if Commit() ran
        // and an exception then fires before scope exit.
        ~Transaction() {
            if (committed_)
                cache_.CommitTransaction();
            else
                cache_.RollbackTransaction();
        }
        Transaction(const Transaction&) = delete;
        Transaction& operator=(const Transaction&) = delete;
        // Mark the transaction as accepted: ~Transaction will call
        // CommitTransaction (keep) instead of RollbackTransaction (undo).
        // noexcept + idempotent (a second call is a no-op); the real cleanup
        // is deferred to the dtor so it is exception-safe.
        void Commit() noexcept { committed_ = true; }

       private:
        RTACache& cache_;
        bool committed_ = false;
    };

    // The difference between the candidate (dag, pa, tl) and the stored
    // champion, WITHOUT computing any RTA. The pure query half of Evaluate.
    // Returns a LOCATOR set (no verdict field): `changed_task_id == -1` iff the
    // candidate is identical to the champion (|diff|==0); otherwise the one
    // changed task + its core + its old/new per-core priority positions.
    //   • no champion → {changed_task_id:-1, ...}
    //   • |diff|==0   → {changed_task_id:-1, ...}
    //   • |diff|==1   → locators filled in (old_pos==new_pos for an ET-only
    //   move) • |diff|>1    → THROWS (violates the P1.10 single-change
    //   invariant; the
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
    //        still differs, a SEPARATE task also moved → false (X's own
    //        priority move is absorbed: combined ET+move = single change).
    //      - 0 ET-diff (pure priority move): at the first mismatch i, the moved
    //        task is champ[i] or cand[i] — try removing each; if either makes
    //        the rest match, that's the single move; if neither → false.
    //      - 0 ET diff + every core identical → |diff|==0, out.changed_task_id
    //        stays -1, returns true.
    //   4. Cross-check: the priority-move core (if any) and the ET-diff task's
    //      core must be the same single core, else >1 change → false.
    bool IsSingleTaskChange(const DAG_Model& dag_tasks, const PriorityVec& pa,
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
    // The champion is carried ONLY in `champion_` (its 5 baked-form / RTA /
    // prefix members — see ChampionState). The raw (dag, pa, tl) triple is
    // consumed at bake time and not stored: pa and tl are read once by the bake
    // (UpdateTaskSetPriorities / ApplyTimeLimitsToTasksExecutionTime) and never
    // again, and Evaluate / IsSingleTaskChange read the baked forms, not a
    // stored pa/tl. A future caller that needs the champion's pa/tl back should
    // add a const accessor rather than carry dead state here.
    //
    // `champion_` is the ChampionState struct itself (NOT 5 loose members): it
    // is both the live state and the Transaction's Memento type, so capture is
    // `ChampionState(champion_)` and restore is `champion_ = move(snapshot)`.
    // This makes drift impossible — a future 6th champion member added to
    // ChampionState is automatically snapshot+restored, whereas 5 loose members
    // + a hand-written Memento could silently forget one.
    ChampionState champion_;

    // Candidate RTA buffer (Evaluate's output; champion_.rta untouched until
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
    // (champion_.champ_prioritized is already pa-sorted, so re-sorting it would
    // be redundant work). Pure.
    std::unordered_map<int, std::vector<int>> PerCoreOrderOfPrioritized(
        const TaskSet& prioritized) const;

    // Write the 4 champion baked-form members of `champion_` from (dag, pa,
    // tl): the canonical-order TL-bake (champion_.champ_tasks_baked, what
    // FindTaskWithDifferentEt reads by index), the pa-sorted form
    // (champion_.champ_prioritized, what Evaluate's reindex + RebuildPrefixes
    // read), the per-core order (champion_.champ_per_core, what
    // IsSingleTaskChange reads), and the HP-prefix checkpoints
    // (champion_.hp_prefix_per_core). Shared by Initialize (which then computes
    // champion_.rta from champion_.champ_prioritized) and AdoptChampion (which
    // takes rtas as a param). champion_.rta / candidate_rta_ stay the caller's
    // job — the two differ on WHERE champion_.rta comes from (Initialize
    // computes it; AdoptChampion receives it), which is exactly what each
    // caller owns. Pure extract-method; the only reorder vs the original inline
    // sequence is RebuildPrefixes preceding the champion_.rta compute in
    // Initialize, which is safe (both read only champion_.champ_prioritized;
    // neither reads the other's output).
    void BakeChampionForms(const DAG_Model& dag_tasks, const PriorityVec& pa,
                           const std::vector<double>& tl);

    // Rebuild champion_.hp_prefix_per_core from `tasks_prioritized` by
    // re-rolling the per-core ET-convolution. Shared by Initialize +
    // AdoptChampion (via BakeChampionForms).
    void RebuildPrefixes(const TaskSet& tasks_prioritized);

    // --- transaction internals (lazy copy-on-write) ---------------------
    // Why the snapshot trigger lives on the cache, not on Transaction: lazy
    // capture means "snapshot the champion the instant BEFORE the first
    // mutation in scope" — and the mutation happens deep inside the cache's
    // own mutators (AdoptChampion/Initialize, invoked from
    // OptimizeIncre_SingleTask far below the tx scope). Only the mutator knows
    // that moment. So each full-champion overwrite calls
    // SnapshotPreMutationStateIfOpen() at its TOP, before writing any member.
    // This is the ONE coupling point lazy COW requires; it is honest-named
    // (a verb describing exactly what it does) and cheap (one branch). The
    // alternative — eager capture at tx-open — removes the hook but pays a
    // full 5-member copy on every walk step; rejected under the
    // "champion updates rare, give-ups common" premise.
    //
    // in_transaction_ : true between BeginTransaction and Commit/Rollback.
    // snapshot_       : the pre-FIRST-mutation champion, or null. Null ⟺ no
    //                   mutation has fired in scope yet (the zero-copy path).
    //                   The bool is redundant with the null check; one source
    //                   of truth.
    bool in_transaction_ = false;
    std::unique_ptr<ChampionState> snapshot_;

    // Called at the top of every full-champion overwrite (AdoptChampion +
    // Initialize), BEFORE any member write. If a tx is open and the snapshot
    // is still null, captures the CURRENT (pre-mutation) champion — so the tx
    // can restore it on rejection. First-capture-only: a no-op on the 2nd+
    // mutation in the same tx (the snapshot already holds the pre-tx state).
    // No-op when no tx is open (the common case outside the serialized walk).
    // No-op on the no-champion → first-Initialize path: capturing an empty
    // champion is pointless (a restore would restore empty state), and skipping
    // it keeps the zero-copy invariant pure.
    void SnapshotPreMutationStateIfOpen();
    // Copy the 5 champion members into a Memento (excludes candidate_rta_).
    ChampionState CaptureChampionState() const;
    // Move-assign the 5 champion members back out of a Memento.
    void RestoreChampionState(ChampionState&& state);
};

}  // namespace SP_OPT_PA

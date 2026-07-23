# P1.21 Design Document — RTA Cache API Redesign & Decoupled Transaction Management

## 1. Executive Summary & Goals

The initial implementation of `RTACache::Transaction` achieved the performance objective (Lazy Copy-on-Write for speculative search walks), but introduced code readability issues and tight architectural coupling:
1. **Tight Coupling**: `RTACache` held a raw backpointer (`active_transaction_`) to the active `Transaction`, required `friend class Transaction`, and invoked hidden observer callbacks (`NotifyChampionAdoption`) deep inside `AdoptChampion` and `Initialize`.
2. **Obscure Caller Logic**: In `CommitIncumbent` (`OptimizeSP_TL_Incre.cpp`), callers were forced to execute `rta_cache_.Evaluate(...)` immediately before `rta_cache_.AdoptChampion(...)` to fetch candidate RTAs, appearing to a reader as an unnecessary/duplicate RTA calculation.
3. **Complex Internal Flows**: Multi-step algorithms in `Evaluate` and `IsSingleTaskChange` lacked clear step-by-step documentation.

### Primary Objectives of this Redesign
- **Complete Decoupling**: Make `RTACache` 100% self-contained. Remove backpointers, friend classes, and hidden cross-object calls.
- **Explicit Transaction API**: Expose clean public methods on `RTACache`: `BeginTransaction()`, `CommitTransaction()`, `RollbackTransaction()`. Keep `RTACache::Transaction` as a pure, lightweight RAII wrapper around these methods.
- **Explicit Promotion API**: Add `AdoptEvaluatedCandidate(dag, pa, tl)` to `RTACache` to make candidate adoption clear and eliminate mystery code in callers.
- **Enhanced Readability**: Document and structure internal multi-step algorithms cleanly with step-by-step headers and self-explanatory variable names.

---

## 2. Architecture & Class Design

### 2.1 Decoupled Transaction Architecture

Instead of `Transaction` registering itself into `RTACache` via raw pointers, `RTACache` directly manages its transaction state:

```
+-------------------------------------------------------------+
|                          RTACache                           |
|                                                             |
|  - in_transaction_ : bool                                   |
|  - has_snapshot_   : bool                                   |
|  - snapshot_       : std::unique_ptr<ChampionState>         |
|                                                             |
|  + BeginTransaction()                                       |
|  + CommitTransaction()                                      |
|  + RollbackTransaction()                                    |
|  + AdoptEvaluatedCandidate(dag, pa, tl)                     |
|  + AdoptChampion(dag, pa, tl, rtas)                         |
|  + Evaluate(dag, pa, tl)                                    |
+-------------------------------------------------------------+
                               ^
                               | (Calls public API)
+------------------------------+------------------------------+
|                     RTACache::Transaction                   |
|  - cache_ : RTACache&                                       |
|  - committed_ : bool                                        |
|                                                             |
|  + Transaction(RTACache& cache) -> cache.BeginTransaction() |
|  + ~Transaction()               -> if (!c) Rollback()       |
|  + Commit()                     -> cache.CommitTransaction()|
+-------------------------------------------------------------+
```

### 2.2 Detailed `RTACache` Interface

```cpp
namespace SP_OPT_PA {

struct ChampionState {
    std::vector<FiniteDist> rta;
    TaskSet champ_prioritized;
    TaskSet champ_tasks_baked;
    std::unordered_map<int, std::vector<int>> champ_per_core;
    std::unordered_map<int, std::vector<FiniteDist>> hp_prefix_per_core;
};

class RTACache {
   public:
    RTACache() = default;

    // --- Core Lifecycle & Evaluation --------------------------------------
    const std::vector<FiniteDist>& Initialize(const DAG_Model& dag_tasks,
                                              const PriorityVec& pa,
                                              const std::vector<double>& tl);

    const std::vector<FiniteDist>& Evaluate(const DAG_Model& dag_tasks,
                                            const PriorityVec& pa,
                                            const std::vector<double>& tl);

    void AdoptChampion(const DAG_Model& dag_tasks, const PriorityVec& pa,
                       const std::vector<double>& tl,
                       const std::vector<FiniteDist>& rtas);

    // Promote the most recently evaluated candidate (stored in candidate_rta_)
    // to champion state without redundant parameter passing or re-evaluations.
    void AdoptEvaluatedCandidate(const DAG_Model& dag_tasks,
                                 const PriorityVec& pa,
                                 const std::vector<double>& tl);

    // --- State Queries ----------------------------------------------------
    bool HasChampion() const { return !rta_.empty(); }
    const std::vector<FiniteDist>& Rta() const { return rta_; }

    // --- Transaction API (Explicit & Self-Contained) ----------------------
    void BeginTransaction();
    void CommitTransaction();
    void RollbackTransaction();
    bool InTransaction() const { return in_transaction_; }

    // Lightweight RAII Guard
    class Transaction {
       public:
        explicit Transaction(RTACache& cache);
        ~Transaction();
        Transaction(const Transaction&) = delete;
        Transaction& operator=(const Transaction&) = delete;

        void Commit() noexcept;

       private:
        RTACache& cache_;
        bool committed_ = false;
    };

    // --- Diff Queries -----------------------------------------------------
    TaskSetDifference ComputeTaskSetDifference(
        const DAG_Model& dag_tasks, const PriorityVec& pa,
        const std::vector<double>& tl) const;

    bool IsSingleTaskChange(const DAG_Model& dag_tasks,
                            const PriorityVec& pa,
                            const std::vector<double>& tl,
                            TaskSetDifference& out) const;

    std::vector<RTAReusePerTask> ClassifyReusePerTask(
        const DAG_Model& dag_tasks, const PriorityVec& pa,
        const std::vector<double>& tl) const;

   private:
    // Champion State Members
    TaskSet champ_prioritized_;
    TaskSet champ_tasks_baked_;
    std::unordered_map<int, std::vector<int>> champ_per_core_;
    std::vector<FiniteDist> rta_;
    std::unordered_map<int, std::vector<FiniteDist>> hp_prefix_per_core_;

    // Evaluation Scratch Buffer
    std::vector<FiniteDist> candidate_rta_;

    // Transaction State
    bool in_transaction_ = false;
    bool has_snapshot_ = false;
    std::unique_ptr<ChampionState> snapshot_;

    // Internal Helpers
    void SaveSnapshotIfInTransaction();
    ChampionState CaptureChampionState() const;
    void RestoreChampionState(ChampionState&& state);
    void BakeChampionForms(const DAG_Model& dag_tasks, const PriorityVec& pa,
                           const std::vector<double>& tl);
    void RebuildPrefixes(const TaskSet& tasks_prioritized);
    std::unordered_map<int, std::vector<int>> PerCoreOrderFromPa(
        const DAG_Model& dag_tasks, const PriorityVec& pa) const;
    std::unordered_map<int, std::vector<int>> PerCoreOrderOfPrioritized(
        const TaskSet& prioritized) const;
};

}  // namespace SP_OPT_PA
```

---

## 3. Detailed Logic & Control Flow

### 3.1 Lazy Copy-on-Write Transaction Flow

1. **Opening a Transaction**: `RTACache::Transaction tx(cache)` calls `cache.BeginTransaction()`.
   - `in_transaction_` is set to `true`.
   - `has_snapshot_` is set to `false`.
   - No memory allocation or deep copy takes place (**$O(1)$ zero-copy entry**).
2. **Speculative Candidate Evaluations**: Calls to `cache.Evaluate(...)` write strictly to `candidate_rta_`. Champion state remains untouched.
3. **Champion Adoption (First Mutate)**: When `AdoptChampion` or `AdoptEvaluatedCandidate` is called:
   - `SaveSnapshotIfInTransaction()` checks `if (in_transaction_ && !has_snapshot_)`.
   - On the first adoption, `CaptureChampionState()` deep-copies the 5 champion members into `snapshot_` and sets `has_snapshot_ = true`.
   - Subsequent adoptions within the same transaction do nothing in `SaveSnapshotIfInTransaction()` because `has_snapshot_` is already `true`.
4. **Transaction Resolution**:
   - **Commit**: `tx.Commit()` calls `cache.CommitTransaction()`, resetting `in_transaction_` and clearing `snapshot_`.
   - **Rollback / Reject**: `~Transaction()` calls `cache.RollbackTransaction()`. If `has_snapshot_` is `true`, `RestoreChampionState` moves `snapshot_` back into the champion members.

### 3.2 Simplified Candidate Adoption in `CommitIncumbent`

Before redesign (confusing duplicate calculation):
```cpp
if (rta_cache_active_) {
    const std::vector<FiniteDist>& rtas = rta_cache_.Evaluate(dag_tasks_, pa, tl);
    rta_cache_.AdoptChampion(dag_tasks_, pa, tl, rtas);
}
```

After redesign (clean & explicit):
```cpp
if (rta_cache_active_) {
    // Synchronize cache champion with the newly committed incumbent solution.
    rta_cache_.AdoptEvaluatedCandidate(dag_tasks_, pa, tl);
}
```

`AdoptEvaluatedCandidate` simply invokes `AdoptChampion(dag_tasks, pa, tl, candidate_rta_)`. If `candidate_rta_` already matches the triple (from a prior `Evaluate`), zero extra work is performed.

---

## 4. Code Readability & Maintenance Enhancements

- **Self-Documenting Implementation**: `RTA_Cache.cpp` functions (`Evaluate`, `IsSingleTaskChange`, `AdoptChampion`) use step-by-step section comments (`// Step 1: ...`, `// Step 2: ...`) to guide maintainers.
- **No Inverted Dependencies**: `Transaction` is a pure consumer of `RTACache`'s public interface. `RTACache` has no knowledge of `Transaction` class internals.
- **Clear Contracts**: Exception safety and non-nesting assertions are clearly stated in comments and enforced via assertions.

---

## 5. Verification Plan

1. **Compilation & Build**:
   ```bash
   make -j$(nproc) testRTA
   ```
2. **Unit Tests**:
   Execute `./tests/testRTA` to verify that all 56 tests (including all `Transaction_*` pins) pass without failure.
3. **Integration Verification**:
   Execute `testIncreOpt_w_TL` and `testOptimizeIncrePA` to ensure bit-identical safety performance (SP) metrics across all optimization runs.

# P1.21 — Abstract RTA cache transactions using RAII

## The Goal
Abstract the `cache_backup` and rollback mechanism in `EvaluateTimeLimitConfig_SubIncremental` to avoid redundant full-cache copy overhead and keep the optimizer-facing API clean.

Currently, `EvaluateTimeLimitConfig_SubIncremental` copies the entire `RTACache` on every sub-incremental walk step. This triggers deep copies of heavy vectors and maps of `FiniteDist` objects, which is extremely wasteful since most steps are eventually rejected by `UpdateRecords` without modifying the cache.

The goals of this task are:
1. Define a scoped `RTACache::Transaction` RAII helper that manages transaction lifetimes.
2. Implement **Lazy Copy-on-Write (COW)**: Only perform the deep copy of the champion state when `AdoptChampion` is called for the first time inside an active transaction.
3. Automatically trigger `RollbackTransaction()` in the transaction's destructor if it is not explicitly committed (e.g. on early returns, walk rejection, or exceptions).
4. Integrate the new `RTACache::Transaction` into `EvaluateTimeLimitConfig_SubIncremental`, simplifying the optimizer code.

---

## Proposed Changes

### C++ Optimizers & Cache

#### [MODIFY] [RTA_Cache.h](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/sources/Safety_Performance_Metric/RTA_Cache.h)
- Declare `struct ChampionState` — the Memento holding the 5 champion members that
  `AdoptChampion` mutates: `rta_`, `champ_prioritized_`, `champ_tasks_baked_`,
  `champ_per_core_`, `hp_prefix_per_core_`. `candidate_rta_` is DELIBERATELY
  excluded (scratch buffer; see Grounded design below).
- Declare `class RTACache::Transaction` as a nested RAII transaction manager
  (constructor / destructor / `Commit()`; non-copyable; no nesting).
- Add `Transaction* active_transaction_` (null iff no open tx) + `friend`.
- Declare `CaptureChampionState()` / `RestoreChampionState()` + the
  `NotifyChampionAdoption()` hook called at the top of `AdoptChampion`.

#### [MODIFY] [RTA_Cache.cpp](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/sources/Safety_Performance_Metric/RTA_Cache.cpp)
- Implement `CaptureChampionState()` (copy the 5 members) / `RestoreChampionState()`
  (move the 5 members back).
- Implement `RTACache::Transaction` constructor (registers `active_transaction_`),
  destructor (restores unless `Commit()` ran, then deregisters), `Commit()`.
- Add `NotifyChampionAdoption()` at the top of `AdoptChampion`: if a tx is open and
  hasn't captured yet, snapshot the pre-adopt champion (first-capture-only).

#### [MODIFY] [OptimizeSP_TL_Incre.cpp](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/sources/Optimization/OptimizeSP_TL_Incre.cpp)
- In `EvaluateTimeLimitConfig_SubIncremental`, replace `RTACache cache_backup =
  rta_cache_;` + `if (!updated) rta_cache_ = cache_backup;` with
  `RTACache::Transaction tx(rta_cache_);` at entry + `if (updated) tx.Commit();`
  after `UpdateRecords`.

---

## Grounded design (2026-07-21)

Read of the call sites (`OptimizeSP_TL_Incre.cpp:179-300`, `:775-796`;
`OptimizeSP_Incre.cpp:310-359`) established:

1. **The only champion mutator on the serialized path is `AdoptChampion`.**
   `Evaluate` writes only the scratch buffer `candidate_rta_` (`:208`, `:436-448`)
   — never `rta_` / `champ_*` / `hp_prefix_per_core_`. So the snapshot target is
   exactly those 5 members.
2. **Two `AdoptChampion` sites, different transaction semantics:**
   - `OptimizeIncre_SingleTask`'s adopt (`OptimizeSP_Incre.cpp:352`) — SPECULATIVE,
     inside the walk; must roll back on rejection. Fires the lazy capture.
   - `CommitIncumbent`'s adopt (`OptimizeSP_TL_Incre.cpp:794`) — runs INSIDE
     `UpdateRecords` on the ACCEPT branch, AFTER `tx.Commit()` would have... NO:
     `UpdateRecords` returns BEFORE the tx commits. So `CommitIncumbent`'s adopt
     fires while the tx is still open → it also triggers the (already-done) lazy
     capture (first-capture-only = no-op the 2nd time). Then `tx.Commit()` keeps it.
     Correct: the accept-path adopt is preserved, not rolled back.
3. **`candidate_rta_` is safe to exclude from the snapshot.** It is fully
   overwritten before any read on every `Evaluate` (`resize` no-op at invariant N,
   reindex writes every FullReuse slot, recompute overwrites every NoReuse slot;
   under the single-change invariant candidate task set == champion task set so
   every slot is reached). After a reject-restore it is stale but harmless — the
   next `Evaluate` overwrites it fully before read. Excluding it cuts ~1 of the 3
   heavy `vector<FiniteDist>` copies (`rta_`, `hp_prefix_per_core_`, `candidate_rta_`).

### Open design fork (awaiting user decision)

**DECIDED 2026-07-21: Lazy COW.** User premise: champion updates rare, give-ups
common → reject-without-adopt (zero-copy row) is the dominant case. Lazy ≤ eager
in every outcome, strictly better on the dominant case; the capture (O(N)
FiniteDist copy) moves off the common path onto the rare adopt path. Capture hook
guards every full-champion overwrite → top of `AdoptChampion` AND `Initialize`;
`Evaluate` untouched (scratch buffer only). See dev_log 2026-07-21 for the
per-outcome copy table.

---

## Verification Plan

### Automated Tests
- Build and run C++ tests:
  ```bash
  make -j$(nproc)
  ctest --output-on-failure
  ```
- Run Python tests:
  ```bash
  pytest tests/python/
  ```

### Manual Verification
- Verify that performance is improved during the sub-incremental serialized walk without any correctness regressions (bit-identical SP results).

# P1.25 — Remove the RTA Cache Transaction Layer (keep the RTA cache)

## The Goal
Delete the `RTACache::Transaction` RAII abstraction (lazy copy-on-write, P1.21)
and all its machinery from the RTA cache, keeping the cache itself
(`RTACache` + `Initialize`/`AdoptChampion`/`Evaluate`/etc.) intact. The cache
champion must stay correctly in sync with `res_opt_` across rejected sub-
incremental walk steps WITHOUT the transaction — i.e. a clean, simpler revert
mechanism (or proof that none is needed). Behavior must stay bit-identical SP
to current HEAD (differential TDD, same gate as P1.12/P1.17).

## Why (the evidence trail)
- **P1.22 Phase 1a frequency data:** of 1632 opened transactions, only 8%
  commit; **~40% capture a snapshot** (pay the deep `ChampionState` copy) and
  **~80% of those captures are thrown away by rollback** (530 of 660). The lazy
  design buys a real win on the 60% zero-copy path, but on the 40% that mutate
  it pays capture on 100% and an extra restore on 80% — most of that work
  serves no kept result.
- **P1.23 timer-artifact finding:** the original "+5.02%@N=10 transaction
  slowdown" that P1.22 was filed to chase appears to be a timer-scoping
  artifact (old timer bracketed all of `RunSimulation`, washing the optimizer
  delta in RTDA+I/O noise). Under the corrected timer (`cc9aa0ce`) the cache
  arm was ~36% *faster*, not slower. So the transaction's *premise of a
  regression to fix* is in doubt.
- **The cost question is moot once the layer is gone:** the long P1.22
  capture-vs-commit / lazy-copy debate was about whether the transaction pays
  for itself. Removing the layer sidesteps the debate entirely.

## The correctness spine (NOT optional)
The transaction is **not** purely perf machinery. It is the ONLY mechanism
keeping the cache champion in sync with `res_opt_` when a sub-incremental walk
step is REJECTED by `UpdateRecords`:

- `EvaluateTimeLimitConfig_SubIncremental` (`OptimizeSP_TL_Incre.cpp:202`)
  opens a `Transaction`. The in-walk `AdoptChampion` calls inside
  `OptimizeIncre_SingleTask` advance the cache champion SPECULATIVELY.
- If `UpdateRecords` rejects the trial (`:308`), `~Transaction` calls
  `RollbackTransaction` → restores the pre-walk champion. Without this, the
  champion drifts.
- The ACCEPT path is ALREADY covered WITHOUT the transaction:
  `CommitIncumbent` (`:807-810`) re-adopts the committed triple via
  `Evaluate`+`AdoptChampion` on every commit. So accepts keep the champion in
  sync regardless of the transaction.
- The REJECT path is what the transaction uniquely owns. If we delete the
  transaction and add nothing, a rejected walk leaves the champion advanced to
  a trial PA that `res_opt_` never committed → the next `Evaluate` sees
  `|diff|>1` → `ComputeTaskSetDifference` THROWS (the exact P1.15 / P1.16
  crash class). So deletion MUST land a replacement revert on reject, OR prove
  the champion is re-seeded before the next eval regardless.

## Scope (what gets deleted / what stays)
**Delete** (`RTA_Cache.h` + `RTA_Cache.cpp`):
- `class RTACache::Transaction` (the RAII wrapper) + `BeginTransaction` /
  `CommitTransaction` / `RollbackTransaction` / `InTransaction`.
- `SnapshotPreMutationStateIfOpen`, `CaptureChampionState`,
    `RestoreChampionState`.
- members `in_transaction_`, `snapshot_` (`std::unique_ptr<ChampionState>`).
- The `SnapshotPreMutationStateIfOpen()` call at the top of `AdoptChampion` +
  `Initialize`.

**Keep:** `RTACache` class, `ChampionState` struct (still the live storage
type for `champion_`), `Initialize`/`AdoptChampion`/`Evaluate`/
`ComputeTaskSetDifference`/`IsSingleTaskChange`/`ClassifyReusePerTask`, the
read accessors.

**Modify** (`OptimizeSP_TL_Incre.cpp`): remove the `RTACache::Transaction
tx(rta_cache_);` at `:202` and `tx.Commit()` at `:314`; replace the implicit
reject-restore with the chosen revert mechanism (see Decisions).

## Gate
- 17/17 ctest DEBUG green; `testRTA` 56/56 green.
- Bit-identical SP to HEAD on a probe run (same contract as P1.12).
- Specifically: a rejected sub-incremental walk MUST NOT leave the champion
  desynced (a TDD test that rejects a walk then issues a follow-up `Evaluate`
  and asserts no throw + correct RTA).

## Decisions
- **D1 — reject-path revert = (b) eager save/restore, scoped to the reject
  branch.** DECIDED 2026-07-23 per user pointer to the pre-transaction commit
  `ecbed896` ("refactor RTA cache", the immediate parent of `4d7d14b6` "add rta
  cache transaction"). `ecbed896` already had the RTA cache but NO transaction,
  and its reject path was exactly the (b) shape:
  ```cpp
  RTACache cache_backup = rta_cache_;            // eager full-copy at entry
  ...
  if (!updated) { rta_cache_ = cache_backup; }   // restore on reject
  ```
  So (b) is proven-correct by git history (it ran in production at `ecbed896`).
  This overrides an earlier (un-implemented) recommendation of (a).

  Trade-off acknowledged: (b) re-introduces the per-step deep copy of the whole
  champion that P1.21 was filed to remove — but (i) P1.22/P1.23 showed the
  transaction's perf premise was a timer artifact and the layer wasn't earning
  its complexity, and (ii) (b) is the known-good mechanism whereas (a) re-seed-
  at-entry is a fresh mechanism needing a fresh correctness check (no `Evaluate`
  may throw between a reject and the next entry re-seed). Simplicity + proven
  correctness win here; if a later profiler flags the deep copy, (a) is the
  deferred upgrade path.

  The three options, for the record:
  - (a) Re-seed the champion from `res_opt_` at the TOP of every
    `EvaluateTimeLimitConfig_SubIncremental` (`CommitIncumbent`-style
    `Evaluate`+`AdoptChampion`). Cheapest (one |diff|==0 FullReuse per entry;
    makes in-walk speculative adopts harmless). DEFERRED as a future upgrade.
  - (b) Eager save/restore scoped to the reject branch (`RTACache backup =
    rta_cache_;` on reject only). Reverts to the P1.16/`ecbed896` shape.
    **CHOSEN.**
  - (c) Stop advancing the champion speculatively in-walk (accept-path
    `CommitIncumbent` re-adopt is the only champion writer). May cost in-walk
    reuse; needs measurement. Not chosen.
- **D2 — P1.22 cleanup:** the TEMPORARY uncommitted `TransactionCounters`
  instrumentation in `RTA_Cache.cpp` + the throwaway `p1_22_counter_probe.json`
  MUST be reverted/deleted as part of (or before) this task.
- **D3 — P1.22/P1.23 task status:** with the transaction removed, P1.22
  (investigate the slowdown) is moot; P1.23 (A/B cache speedup) stands on its
  own. Recommend closing P1.22 as superseded-by-P1.25.

## Sibling tasks
- P1.21 (the transaction being removed) → will move to `finished_tasks/` once
  P1.25 lands (the transaction is reverted, but the lesson stands).
- P1.22 (investigate slowdown) → superseded by P1.25; close.
- P1.23 (cache A/B speedup) → unaffected; the cache itself stays.
- P1.17 (redundant ops) → its item 4 (`cache_backup` smell) was already closed
  by P1.21; P1.25 re-opens the question of what (if anything) replaces the
  reject-restore.

# P1.25 — Tasks (working checklist)

> Goal: Remove the `RTACache::Transaction` layer; keep the RTA cache. Champion
> must stay in sync with `res_opt_` on rejected walk steps WITHOUT the
> transaction. Gate = bit-identical SP to HEAD.

> **CLOSED 2026-07-23 — COMMITTED `0448db9c`.** Transaction layer deleted from
> `RTA_Cache.h/.cpp`; D1=(b) eager backup restored in
> `EvaluateTimeLimitConfig_SubIncremental` (`1217d227` shape). Phase 3 verify
> GREEN: 17/17 ctest + bit-identical SP to HEAD (stdout diff = 0 lines). P1.22
> closed as superseded; P1.21 reverted. Moved to `finished_tasks/`. See
> [[p125-remove-rta-cache-transaction]], [[p122-investigate-rta-cache-transaction-slowdown]],
> [[p121-rta-cache-transaction-raii]].

## Phase 0 — Decisions & cleanup (before any code)

- [x] **0a. User decides D1** = **(b) eager save/restore scoped to the reject
  branch** (the `ecbed896` pre-transaction shape: `RTACache backup = rta_cache_;`
  at entry, `rta_cache_ = backup;` only on reject). DECIDED 2026-07-23 per user
  pointer to commit `ecbed896` (the pre-transaction parent of `4d7d14b6`).
- [x] **0b. Strip P1.22 temporary instrumentation** — folded into Phase 2b
  (removed with the transaction deletion: the top-of-file `TransactionCounters`
  struct + `atexit` dump + the stray counter increments in `Initialize`/
  `AdoptChampion` + `#include <cstdio>`). Done 2026-07-23.
- [x] **0c. Delete throwaway probe config**
  `simulation_experiments/configs/p1_22_counter_probe.json`. Done 2026-07-23
  (it was untracked — plain `rm`).

## Phase 1 — TDD red (pin the reject-path contract FIRST)

- [x] **1a. Add a red test (reject-path revert is load-bearing):** drive a real
  sub-incremental walk to a REJECT (trial whose final best does NOT beat
  `res_opt_.sp_opt`), then assert the cache champion is bit-identical to the
  committed/incumbent triple's RTA — NOT the rejected trial PA's RTA. Gate is
  bit-identical RTA, NOT just "no throw": the desync after a reject-with-adopt
  is ≤1 task (per `OptimizeIncre_SingleTask`'s one-task-per-variation contract,
  `OptimizeSP_Incre.cpp:325-327`), so a naive delete does NOT reliably throw
  `|diff|>1`; it silently patches the next `Evaluate` against the WRONG champion
  → wrong candidate RTA. So the red assertion must compare the stored champion
  RTA (`cache.Rta()`) to the incumbent's oracle RTA. This MUST fail against a
  naive delete-the-transaction change (champion stuck at rejected trial PA),
  proving the (b) revert is load-bearing. Level = walk (`OptimizeIncre_w_TL`
  end-to-end on a real taskset that hits a reject step), via a test subclass
  exposing `protected rta_cache_`; mirrors `SerializedIncremental_SingleChangeInvariant`.
  DONE 2026-07-23: `SubIncrementalReject_RevertKeepsChampionOnCommittedTriple`
  in `tests/testIncreOpt_w_TL.cpp` (v19→v21). PASSES on HEAD (green baseline).
- [x] **1b. Add a red test (accept-path regression guard):** an ACCEPT walk
  leaves the champion tracking the committed triple (the `CommitIncumbent`
  re-adopt path, `OptimizeSP_TL_Incre.cpp:807-810`, must stay the accept-path
  writer). Asserts champion RTA == committed triple's oracle RTA after an
  accept. Should pass even pre-deletion (accept path is tx-independent) — a
  guard that the deletion doesn't break the accept re-adopt.
  DONE 2026-07-23: `SubIncrementalAccept_ChampionTracksCommittedTriple`
  (ReOptimizePeriodic bootstrap + `OptimizeIncre_w_TL` on v19→v19 to arm the
  cache + baseline-adopt via CommitIncumbent with no reject). PASSES on HEAD.

**TDD arc status (REVISED 2026-07-23):** 1a/1b written + PASS on HEAD = green
baseline. The naive delete (Phase 2a/2b + 2c-WITHOUT-backup) did NOT make 1a
FAIL — 1a stayed GREEN on the naive-delete, because the v19→v21 walk's single
reject had no in-walk adopt (no drift to observe). So the planned red-then-green
arc could NOT be demonstrated. The (b) backup's necessity rests on git-history
proof (`1217d227`) + the reject-after-adopt mechanism, not a live red. Awaiting
user decision on whether to strengthen 1a into a real red first (option II) or
proceed on the git-history proof (option I). See dev_log 2026-07-23.

## Phase 2 — Delete the transaction layer

- [x] **2a.** In `RTA_Cache.h`: removed `class Transaction`, the public
  `BeginTransaction`/`CommitTransaction`/`RollbackTransaction`/`InTransaction`,
  the private `SnapshotPreMutationStateIfOpen`/`CaptureChampionState`/
  `RestoreChampionState`, members `in_transaction_` + `snapshot_`, and the now-
  unused `#include <memory>`. Updated the stale `champion_`/`ChampionState`
  comments to the D1=(b) framing. DONE 2026-07-23. (The prior session had
  removed the public API but left the private internals + the still-calling
  `OptimizeSP_TL_Incre.cpp` — a non-compiling intermediate; this resume
  finished it.)
- [x] **2b.** In `RTA_Cache.cpp`: removed the implementations of all the above;
  removed the `SnapshotPreMutationStateIfOpen()` call at the top of
  `AdoptChampion` and `Initialize`; stripped the P1.22 `TransactionCounters`
  instrumentation block + stray counter increments + `#include <cstdio>` +
  `#include <cassert>` (no longer used). DONE 2026-07-23.
- [x] **2c-RED.** Naive-delete checkpoint in
  `EvaluateTimeLimitConfig_SubIncremental` (`OptimizeSP_TL_Incre.cpp`): dropped
  `RTACache::Transaction tx(rta_cache_);` + `tx.Commit()` with NO backup yet
  (reject branch is a no-op `(void)updated;`). Clean build of SP_OPT lib +
  `testIncreOpt_w_TL` target PASSED. **1a did NOT go RED — it PASSED (green) on
  the naive-delete.** See dev_log 2026-07-23 (Phase 2c-RED did NOT go red): the
  v19→v21 walk's single reject had no in-walk adopt, so no drift survived for
  1a's final-state assertion to observe. 1a is too weak to pin the contract
  (exactly the silent-bug hazard flagged earlier). The (b) backup is STILL
  load-bearing — proven by `1217d227` ("add cache_backup... to reduce
  rta_cache_ becoming outdated") + the reject-after-adopt mechanism, NOT by a
  live red. AWAITING USER decision (options I/II/III in dev_log) before 2c-GREEN.
- [x] **2c-GREEN.** Restored the D1=(b) shape (from `1217d227`, the P1.16
  pre-transaction backup — NOT `ecbed896`, which only touched RTA_Cache):
  `RTACache cache_backup = rta_cache_;` at entry + `if (!updated) {
  rta_cache_ = cache_backup; }` on the reject branch. User chose option (I)
  (land on git-history proof + mechanism; 1a/1b = no-regression guards, not a
  red-then-green pin). DONE 2026-07-23. Also rewrote the entry comment from the
  RED-marker to the final (b) framing, and corrected the test comments (1a/1b
  headers + the shared block) to the honest "guard not red pin" framing + the
  `1217d227` reference (was imprecise `ecbed896`) + stale `:807-810`→`:791-793`.
- [x] **2d.** Updated the stale P1.21/P1.22 comments in
  `OptimizeSP_TL_Incre.cpp` (`:186-202`, `:312-316`) to the D1=(b) framing
  (RED-marker now; will become the final (b) comment in 2c-GREEN). DONE
  2026-07-23.
- [x] **2e.** (added) Removed the three P1.21 `Transaction_*` unit tests from
  `tests/testRTA.cpp` (lines 570-727) — they tested the deleted RAII guard
  directly and would not compile. Replaced with a pointer comment to the walk-
  level 1a/1b pins that now cover the same contract. DONE 2026-07-23.

## Phase 3 — Verify

- [x] **3a.** `cmake --build build_test --target check.SP_OPT --clean-first -j5`
  (clean-first because `RTA_Cache.h` layout changed — stale `.o` causes phantom
  crashes). 17/17 ctest green (incl. `testIncreOpt_w_TL` + `testRTA`).
  DONE 2026-07-23.
- [x] **3b.** Bit-identical SP probe vs HEAD. Stashed the 5 P1.25-touched files,
  clean-built HEAD `testOptimizeIncrePA` + `testIncreOpt_w_TL`, captured stdout
  (26 + 49 tests pass), restored, rebuilt (b)-backup, captured again. Diff of
  stdout with test-timing fields normalized out = **0 lines** (the only raw
  diffs were `(N ms)` timing). Covers the explicit
  `Differential_BitIdenticalOnSingleEtChange` +
  `Differential_BitIdenticalToOracle_OnSingleEtChange` tests. DONE 2026-07-23.
- [x] **3c.** 1a `SubIncrementalReject_RevertKeepsChampionOnCommittedTriple` +
  1b `SubIncrementalAccept_ChampionTracksCommittedTriple` both PASS green
  (run directly via `--gtest_filter`). DONE 2026-07-23.

## Phase 4 — Bookkeeping

- [x] **4a.** `git add` (agent stages; user commits). Only P1.25 changes in
  the commit. DONE 2026-07-23 — committed by user in `0448db9c`.
- [x] **4b.** P1.21 already in `finished_tasks/` (transaction reverted; lesson
  stands). DONE.
- [x] **4c.** Close P1.22 as superseded-by-P1.25; update
  `agents/overall_tasks.md` + memory `p122-...`. DONE 2026-07-23 — P1.22
  moved to `finished_tasks/`, memory `p122-` updated.
- [x] **4d.** Update `agents/overall_tasks.md` P1 table with the P1.25 row.
  DONE 2026-07-23.
- [x] **4e.** (added) Move P1.25 to `finished_tasks/`. DONE 2026-07-23.

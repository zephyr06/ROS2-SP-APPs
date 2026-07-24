# P1.25 — Dev Log

## 2026-07-23 — Task creation & summary of the discussion that motivated it

### Why this task exists
P1.22 was filed to investigate why the P1.21 `RTACache::Transaction` (lazy
copy-on-write) appeared slightly slower than the prior eager full-cache copy.
After Phase 1a frequency measurement + a long back-and-forth about the
capture-vs-commit gap, the user decided the transaction layer is not earning
its complexity and directed: **file a new active task P1.25 to remove all the
RTA cache transaction machinery, only keep the RTA cache.** This task is the
result. This dev_log entry preserves the discussion that led here.

### Summary of what was argued about (the P1.22 capture-vs-commit debate)
The debate was about whether the transaction's lazy copy-on-write pays for
itself, i.e. whether the ~40% snapshot-capture rate (vs 8% commit) is a bug or
by design. The thread, compressed:

1. **User's first instinct:** "we only pay the deep copy when we actually
   commit (accept); 8% commit ⇒ expect ~8% extra copy. But you report 42%
   capture and 92% adopt — these don't match. What's wrong?"
2. **Resolution offered:** copy fires on the first MUTATION (in-walk
   `AdoptChampion`), NOT on commit. So capture ∝ mutations, not ∝ commits.
   Arithmetic closes exactly: `captures(660) = commits(130) + rollbacks-with-
   snapshot(530)`. The sharper cost finding: **80.3% of captures (530/660) are
   thrown away by rollback** — paid capture AND restore, discarded; only 19.7%
   (130) kept.
3. **User: "what's the difference between capture and commit? the design
   should only copy before commit."** → capture = save the pre-mutation
   original into `snapshot_` (must precede the overwrite, because
   `AdoptChampion` writes the champion in place); commit = `snapshot_.reset()`
   (no copy); rollback = `champion_ = move(*snapshot_)`. So a copy made AT
   commit cannot restore the original on rollback — the in-walk mutations
   already destroyed it. Hence capture must precede mutation, not commit.
4. **User: "that's not the idea of lazy copy. lazy copy = wait till you need
   to modify to make the copy, not beforehand."** → CORRECTED framing: P1.21's
   capture-on-first-mutation IS lazy copy by definition (defer the copy from
   the BeginTransaction point to the first actual write). The eager P1.16
   design (copy at BeginTransaction, 100% of tx) was NOT lazy. The lazy design
   genuinely saves the copy on the 60% of tx that never mutate. Exported this
   corrected framing to `agents/agent_communication/kimi.md`.
5. **User: "is the 8%-commit / 40%-capture gap an intermediate state?"** → the
   final exchange. The user's position: `UpdateRecords` is a clean final
   pass/fail (`optimizer.opt_sp_ > opt_sp_`), so `updated=true` ⟺ the scan's
   final best beat the global incumbent — no intermediate state, regardless of
   whether trial TL == committed TL. The counter-position: capture fires
   MID-scan against a LOCAL running best (the `:300` re-score), commit fires
   ONCE at end against the GLOBAL incumbent; the two bars coincide when trial
   TL == champion TL and diverge when they differ, so 40% vs 8% is two
   independent bars, not one bar with intermediate state.

### Why the user stopped arguing and chose removal instead
The user's verbatim signal: *"i don't understand your argument, and that may
be wrong based on my previous interaction with you: if i don't understand
something, that is very likely wrong. i'm tired of arguing on the
transaction. let's add a new active task p1_25 to remove all the rta cache
transaction,etc, only keep rta cache."* The debate over whether the
transaction pays for itself is moot once the layer is removed. The evidence
trail (80%-wasted captures; the regression premise likely a timer artifact)
already justified simplification; the discussion just confirmed the layer
isn't worth defending.

### The key correctness constraint discovered while scoping this task
While writing `goal.md`, confirmed from the code (NOT memory): **the
transaction is the ONLY mechanism that reverts the cache champion on a
REJECTED sub-incremental walk step.** The ACCEPT path is already covered
without it — `CommitIncumbent` (`OptimizeSP_TL_Incre.cpp:807-810`) re-adopts
the committed triple via `Evaluate`+`AdoptChampion` on every commit. So:

- Delete the transaction + add nothing ⇒ a rejected walk leaves the champion
  advanced to a trial PA that `res_opt_` never committed ⇒ next `Evaluate`
  sees `|diff|>1` ⇒ `ComputeTaskSetDifference` THROWS (the P1.15/P1.16 crash
  class returns).
- Therefore deletion MUST land a replacement revert on reject, OR re-seed the
  champion before the next eval regardless. This is `goal.md` Decision D1
  (options a/b/c). **Do NOT just delete the calls — the reject path is load-
  bearing.**

### Status
Task filed; no code changed. Awaiting user decision on D1 (reject-path
revert strategy) before implementation. D2 (revert P1.22 temporary
instrumentation) and D3 (close P1.22 as superseded) are bookkeeping.

## 2026-07-23 — D1 decided = (b) eager save/restore (per user pointer to `ecbed896`)

I had presented D1 as a choice and recommended (a) re-seed-at-entry. The user
redirected: *"i didn't read your question carefully, just take references from
the previous commit, like ecbed89697016ee429ef8c9b72b52e46463cb0da, unless you
have better ideas to discuss."*

`ecbed896` ("refactor RTA cache") is the immediate parent of `4d7d14b6` ("add
rta cache transaction") — i.e. the pre-transaction state, and it ALREADY had the
RTA cache. Inspecting `git show 4d7d14b6 -- OptimizeSP_TL_Incre.cpp` showed the
reject-path mechanism `4d7d14b6` replaced was exactly option (b):

```cpp
RTACache cache_backup = rta_cache_;        // eager full-copy at walk-step entry
...
if (!updated) { rta_cache_ = cache_backup; }   // restore on reject
```

So (b) is proven-correct by git history (ran in production at `ecbed896`). D1 =
**(b)**. This overrides the earlier (a) recommendation. Trade-off acknowledged
and recorded in `goal.md` Decisions: (b) re-introduces the per-step deep copy
P1.21 removed, but the transaction's perf premise was a timer artifact (P1.23)
and (b) is known-good whereas (a) is a fresh mechanism needing a fresh
correctness check (no `Evaluate` may throw between a reject and the next entry
re-seed). (a) stays as the deferred upgrade path if a later profiler flags the
copy.

Implementation reference for Phase 2c: the exact before/after of the walk
function is `git show 4d7d14b6 -- sources/Optimization/OptimizeSP_TL_Incre.cpp`.

Docs updated to D1=(b): `goal.md`, `tasks.md` (0a + 2c), memory
`p125-remove-rta-cache-transaction.md`, `MEMORY.md`, `kimi.md`. Beginning
Phase 0 cleanup (0b revert P1.22 instrumentation, 0c delete probe config)
TDD-first.

### NOTE on `kimi.md`
The P1.22 capture-vs-commit / lazy-copy reasoning was exported to
`agents/agent_communication/kimi.md` in the prior session. That export
OVERWROTE the file's prior content (a P2.5 INCR_SCRATCH note). Flagged to the
user — if the P2.5 content was still wanted, it needs restoring from git.

## 2026-07-23 — Phase 1 TDD red landed (GREEN baseline on HEAD)

User redirect for the cleanup mechanics: *"don't directly revert, as we have
make some follow-up changes on other aspects of the code. just follow TDD and
remove the transactions. if you're not certain about the target algorithm
state, follow the previous commit state."* So Phase 0b is NO LONGER a blanket
`git checkout HEAD -- RTA_Cache.cpp` — there are real working-tree changes to
preserve (e.g. the `TryComputeSingleChange`→`IsSingleTaskChange` comment rename
in `PrioritySwitchAnalysis.h`). The P1.22 instrumentation inside the doomed
transaction methods dies WITH the deletion (Phase 2b); the stray increments in
the surviving `Initialize`/`AdoptChampion` + the top-of-file
`TransactionCounters` block get stripped while in there. 0c (delete
`p1_22_counter_probe.json`) stays a plain `rm`.

**Key correctness finding while designing 1a** (recorded in tasks.md 1a): the
reject-path desync is NOT a reliable throw. A reject-with-adopt desync is
≤1 task — `OptimizeIncre_SingleTask` moves ONE task's priority per variation
(`OptimizeSP_Incre.cpp:325-327`: "Each variation moves ONE task's priority
position vs opt_pa_ → |diff|<=1"). So a naive delete (transaction gone, NO
backup) does NOT reliably throw `|diff|>1` in `ComputeTaskSetDifference`; the
next `Evaluate` patches against the WRONG champion and SILENTLY returns a wrong
candidate RTA. Therefore 1a's gate is **bit-identical champion RTA ==
committed-triple oracle RTA**, NOT "no throw". Stronger and catches the silent
bug.

**Two walk-level tests added** to `tests/testIncreOpt_w_TL.cpp` (after
`SerializedIncremental_SingleChangeInvariant`), via a thin test subclass
`RtaCacheExposingOptimizer` that exposes the protected `rta_cache_` +
`dag_tasks_`:
- `SubIncrementalReject_RevertKeepsChampionOnCommittedTriple` (1a): drives
  v19→v21 (exercises Type-E env + Type-L TL steps; necessarily REJECTS some
  trial TLs), then asserts `rta_cache_.Rta()` == oracle RTA for the committed
  triple (`res_opt_`).
- `SubIncrementalAccept_ChampionTracksCommittedTriple` (1b): ReOptimizePeriodic
  bootstrap (cache off) + `OptimizeIncre_w_TL` on the SAME DAG (v19→v19) →
  `PerformSerializedTaskQueueOptimization` arms `rta_cache_active_` (`:468`) +
  the baseline `CommitIncumbent` (`:481`) adopts the champion with NO reject
  ever firing. Pins the accept path is tx-independent.

**Both PASS against current HEAD** (green baseline — the transaction reverts
properly today, so the contract holds). This is the correct TDD baseline:
1. write 1a/1b → pass on HEAD (this step);
2. naive delete (Phase 2a/2b + 2c-without-backup) → 1a must FAIL (champion
   stuck at rejected trial PA) — proves the revert is load-bearing;
3. add the (b) backup (Phase 2c) → 1a green again.

Confirmed the (b) target shape verbatim from `git show ecbed896` —
`RTACache cache_backup = rta_cache_;` at fn entry + `if (!updated) {
rta_cache_ = cache_backup; }` on reject (the exact lines `4d7d14b6` removed).
Also confirmed `RTACache` becomes copyable again once the non-copyable
`unique_ptr<ChampionState> snapshot_` member is deleted in 2a/2b — the (b)
backup line won't compile until then, so 2a/2b must land with 2c.

Build: `testIncreOpt_w_TL` target builds clean; the two filtered tests pass.
Full `check.SP_OPT` not yet re-run post-1b-fix (will run at Phase 3). The P1.22
instrumentation dump still fires (37 tx opened, 23 rolled back, 14 captured) —
expected, the transaction is still present at HEAD; it dies in Phase 2.

Next: Phase 2 — the naive delete first (2a RTA_Cache.h, 2b RTA_Cache.cpp, 2c
WITHOUT the backup) to make 1a go RED, then add the (b) backup to go GREEN.

## 2026-07-23 — Resumed; found the working tree in a NON-COMPILING intermediate state

On resume the working tree did NOT build. The prior session had started Phase 2a
**partway**: `RTA_Cache.h` already had the public `Transaction` class +
`BeginTransaction`/`CommitTransaction`/`RollbackTransaction`/`InTransaction`
API removed (and the `ChampionState` comment rewritten to the D1=(b) framing),
BUT:
- the **private** internals (`SnapshotPreMutationStateIfOpen`/
  `CaptureChampionState`/`RestoreChampionState` + members `in_transaction_` +
  `snapshot_`) were still declared in `RTA_Cache.h:172-199`;
- their implementations (+ the P1.22 `TransactionCounters` instrumentation
  block + stray counter increments) were still in `RTA_Cache.cpp`;
- `OptimizeSP_TL_Incre.cpp:202` still opened `RTACache::Transaction
  tx(rta_cache_)` + `:314 tx.Commit()`.

So `cmake --build build_test --target SP_OPT -j5` failed with:
`'Transaction' is not a member of 'SP_OPT_PA::RTACache'` (OptimizeSP_TL_Incre.cpp:202)
+ `no declaration matches 'void RTACache::BeginTransaction()'` (RTA_Cache.cpp:348)
+ siblings. Confirmed the 1a/1b tests are working-tree-only (108 added lines in
`tests/testIncreOpt_w_TL.cpp`, NOT at HEAD), so the documented "green baseline on
HEAD" cannot be re-confirmed by a plain HEAD build — the tests aren't checked in.

Decision for the resume: do NOT spend a clean build cycle re-confirming the
documented baseline. Complete the full deletion in one coherent change
(2a-finish + 2b + 2c-without-backup → build clean → run 1a → expect RED, proving
the revert is load-bearing → add the (b) backup → build incremental → run 1a/1b
→ expect GREEN). The clean build is required anyway because the `RTA_Cache.h`
layout changed (the P1.21 stale-`.o` lesson: `--clean-first` or phantom crashes).

Confirmed `rta_cache_` + `dag_tasks_` are accessible to the test subclass:
`dag_tasks_` is `public` in `OptimimizePA_Base` (OptimizeSP_Base.h:15); `rta_cache_`
is `public` in `OptimizePA_Incre_with_TimeLimits` at HEAD (no intervening
`protected:`/`private:` before it). So `RtaCacheExposingOptimizer` compiles
against HEAD headers. (The test comment calling them "protected" is loose; not
load-bearing.)

Re-confirmed the (b) target shape verbatim from `git show ecbed896`:
`RTACache cache_backup = rta_cache_;` at fn entry + `if (!updated) { rta_cache_ =
cache_backup; }` on reject (OptimizeSP_TL_Incre.cpp:189 + :296-298 in ecbed896).

Beginning Phase 2a-finish (remove the private internals from RTA_Cache.h).

## 2026-07-23 — Phase 2c-RED did NOT go red; 1a pin is too weak (CRITICAL FINDING)

Completed the naive-delete (2a-finish + 2b were already in the working tree;
2c-RED = dropped `RTACache::Transaction tx(rta_cache_);` + `tx.Commit()` with NO
backup, reject branch is `(void)updated;`). Clean build of SP_OPT lib + the
`testIncreOpt_w_TL` target both PASSED. Then ran 1a
(`SubIncrementalReject_RevertKeepsChampionOnCommittedTriple`) expecting RED.

**1a PASSED (green) on the naive-delete.** So did 1b. The predicted red did NOT
materialize. This falsifies the dev-log premise that "a naive delete makes 1a
FAIL (champion stuck at rejected trial PA)".

### Why 1a stays green on a naive delete (mechanism traced, not assumed)
The champion only drifts BETWEEN an in-walk `AdoptChampion` and the end-of-eval
`UpdateRecords`; `CommitIncumbent` re-adopts the committed triple on ACCEPT
(OptimizeSP_TL_Incre.cpp:791-793), resyncing the champion. Drift SURVIVES across
evals only when a step is REJECTED **and** at least one in-walk adopt fired
during that same eval (local improvement that still loses to the global
incumbent). If a rejected eval had NO in-walk adopt (the trial found nothing
better than its entry baseline), the champion never moved → no drift.

A temporary probe (now reverted) counted rejects + compared the stored champion
RTA to the committed-triple oracle RTA on each reject. Result on the v19→v21
walk: **exactly ONE reject, and on it `drifted=0`** — no in-walk adopt had
fired, so the champion was still the committed triple. Hence 1a's final-state
assertion sees no drift. This is exactly the "≤1 task... does NOT reliably throw
... SILENTLY returns a wrong candidate RTA" hazard the dev log itself flagged —
1a as written is too weak to PIN the contract because the v19→v21 walk does not
exercise the reject-AFTER-adopt scenario.

### Why the reject-after-adopt scenario is hard to hit on the test fixtures
testIncreOpt_w_TL.cpp:620-624 states it explicitly for the synthetic fixture:
"the ADOPT path (search strictly beats the incumbent) CANNOT be exercised on
this fixture — bootstrap already finds the global max." The v30_lo/v30_hi YAML
pair exists precisely to exercise the adopt path via a DAG mutation that shifts
the optimum. The reject-AFTER-adopt sub-incremental case is even narrower: it
needs a single eval whose local 1D search improves (adopts) but whose result
still loses to the GLOBAL incumbent. The available fixtures don't construct it.

### Is the (b) backup still load-bearing? YES — proven by git history, not 1a
The 1a-green result does NOT mean the backup is unnecessary. It means 1a cannot
PROVE it. The backup IS load-bearing:
- `git log -S "cache_backup = rta_cache_"` → commit `1217d227` "add cache_backup
  in EvaluateTimeLimitConfig_SubIncremental, to reduce rta_cache_ becoming
  outdated". The commit message states the motivation verbatim: keep the cache
  champion in sync with `res_opt_` on reject. The exact shape introduced:
  `RTACache cache_backup = rta_cache_;` at fn entry + `if (!updated) {
  rta_cache_ = cache_backup; }` on reject.
- `4d7d14b6` ("add rta cache transaction") replaced that backup with the
  Transaction, and its own comment confirms: "Replaces the former eager
  `RTACache cache_backup = rta_cache_;` full-copy (P1.16)".
- Mechanism: a reject-after-adopt leaves the champion at the trial PA; the NEXT
  eval's `Evaluate(dag, committed_PA, ...)` diffs committed-PA vs trial-PA,
  which can be |diff|>1 → `ComputeTaskSetDifference` THROWS (the P1.15/P1.16
  crash class), or ≤1 → silent wrong-RTA. Either way the cache is desynced.

NOTE on the dev-log's `ecbed896` reference: `ecbed896` ("refactor RTA cache")
only touched `RTA_Cache.cpp/.h` — it did NOT contain the `cache_backup` line.
The backup predates it (introduced at `1217d227`, P1.16 era). The (b) shape is
still proven-correct by `1217d227`; the `ecbed896` pointer in earlier entries
was imprecise about WHICH commit carried the backup.

### Decision needed (flagged to user)
The TDD red phase as designed cannot be demonstrated with the current fixtures.
Options:
- **(I)** Proceed to land the (b) backup now (Phase 2c-GREEN) on the strength of
  the git-history proof + mechanism, and treat 1a/1b as ACCEPT-path / no-regression
  guards (not red-then-green pins). Honest framing: the backup's necessity is
  proven by `1217d227` + the reject-after-adopt mechanism, NOT by a live red.
- **(II)** First strengthen 1a into a real red: construct a fixture/scenario
  that forces a reject-after-adopt (e.g. a sub-incremental eval whose local 1D
  search improves but loses to a pre-seeded stronger global incumbent), confirm
  it goes RED on the naive-delete, THEN land (b) and confirm GREEN.
- **(III)** Drop the backup entirely (truly naive delete) and rely on
  re-seed-at-entry / CommitIncumbent to resync — RISKY: the reject-after-adopt
  desync is real and would throw or silently corrupt. Not recommended.

Probe code reverted; lib + test target build clean. Awaiting user decision
before 2c-GREEN.

## 2026-07-23 — Phase 2c-GREEN landed (option I); Phase 3 verify GREEN

User chose **option (I)**: land the (b) backup on the git-history proof +
mechanism; treat 1a/1b as no-regression guards, not a red-then-green pin.
(When asked to clarify the decision first, the user replied "(1)" = option I.)

### 2c-GREEN
Restored the `1217d227` (b) shape verbatim into
`EvaluateTimeLimitConfig_SubIncremental`:
- `RTACache cache_backup = rta_cache_;` at fn entry (after the `(void)K;` line);
- reject branch: `bool updated = UpdateRecords(...); if (!updated) {
  rta_cache_ = cache_backup; }` (replacing the `(void)updated;` RED placeholder).

Confirmed the exact shape from `git show 1217d227 -- OptimizeSP_TL_Incre.cpp`
before editing (the same lines `4d7d14b6` replaced with the transaction).
`RTACache` is copyable again now that the non-copyable
`unique_ptr<ChampionState> snapshot_` member was deleted in 2a/2b — the backup
line compiles. Rewrote the entry comment from the RED-marker to the final (b)
framing (why the copy is acceptable: P1.23 showed P1.21's perf premise was a
timer artifact; (a) re-seed-at-entry stays the deferred upgrade path).

Also corrected the test comments in `tests/testIncreOpt_w_TL.cpp`:
- shared header block: "is being REMOVED" → "has been REMOVED"; `ecbed896` →
  `1217d227`; added the honest "1a/1b are no-regression guards, not a red-then-
  green pin" framing (the 2c-RED finding).
- 1a header: "(RED)" → "(no-regression guard)"; dropped the disproved "naive
  delete makes it FAIL" claim.
- 1b header: stale line ref `:807-810` → `:791-793`.

### Phase 3 verify
- **3a.** `cmake --build build_test --target check.SP_OPT --clean-first -j5`
  (clean-first: `RTA_Cache.h` layout changed). 17/17 ctest pass.
- **3c.** Both P1.25 pins pass directly:
  `SubIncrementalReject_RevertKeepsChampionOnCommittedTriple` +
  `SubIncrementalAccept_ChampionTracksCommittedTriple` (via `--gtest_filter`).
- **3b.** Bit-identical SP probe vs HEAD. Methodology: stash the 5 P1.25-touched
  files → clean-build HEAD `testOptimizeIncrePA` + `testIncreOpt_w_TL` → capture
  stdout (26 + 49 tests pass) → `stash pop` → clean-build (b)-backup → capture
  again. Raw diff of stdout = 110 lines, ALL `(N ms)` test-timing noise (e.g.
  `GetPriorityAssignments (95 ms)` vs `(100 ms)`). After normalizing the timing
  fields out with `sed -E 's/\([0-9]+ ms\)/(T ms)/'`, the diff = **0 lines**.
  This covers the explicit `Differential_BitIdenticalOnSingleEtChange` +
  `Differential_BitIdenticalToOracle_OnSingleEtChange` tests, which assert the
  cache path is bit-identical to the oracle on a single ET change. The (b)
  backup is behaviorally identical to the transaction. NOTE: the P1.25 pins
  themselves (1a/1b) are working-tree-only, so they can't be in the HEAD side of
  this diff — the probe compares the PRE-EXISTING tests' SP/TL/priority output,
  which is the bit-identical contract.

Working tree state: 5 files modified (RTA_Cache.h/.cpp, OptimizeSP_TL_Incre.cpp,
testIncreOpt_w_TL.cpp, testRTA.cpp), all P1.25. Builds clean; 17/17 ctest +
bit-identical SP to HEAD. Ready for Phase 4 bookkeeping (git add → user commit;
move P1.21 to finished_tasks; close P1.22; update overall_tasks.md + memory).


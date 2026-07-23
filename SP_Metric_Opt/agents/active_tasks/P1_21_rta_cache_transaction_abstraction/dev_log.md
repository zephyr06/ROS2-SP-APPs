# P1.21 — Dev Log

## 2026-07-20 — Initial Proposal and Design

### The Problem
During the serialized queue walk, speculative cache champion updates must be reverted if the trial configuration is rejected by `UpdateRecords`. Currently, this is handled by copying the entire `RTACache` instance on every entry of `EvaluateTimeLimitConfig_SubIncremental` and restoring it if rejected. 

This upfront copy is expensive because it copies all champion distributions and maps, even though the vast majority of steps are rejected and do not modify the cache champion (since `AdoptChampion` is only called if a new local optimal SP is found). Furthermore, doing this copy/restore manually in the optimizer code exposes low-level cache implementation details, making the API messy and error-prone (e.g., if early returns are added).

### The Proposed RAII Transaction Solution
We propose wrapping transaction management inside a nested helper class `RTACache::Transaction`. 
1. **RAII-based Scope Control**: Declaring `RTACache::Transaction tx(rta_cache_);` at the start of `EvaluateTimeLimitConfig_SubIncremental` enters a transaction. If the transaction is not explicitly committed via `tx.Commit()`, the destructor will automatically roll back `rta_cache_` to its pre-transaction state on scope exit. This handles early returns and exceptions gracefully.
2. **Lazy Copy-on-Write (COW)**: Instead of copy-constructing the backup state immediately, we register the transaction pointer with `RTACache`. When `AdoptChampion` is called, it checks if a transaction is active and, if so, triggers a lazy backup. If `AdoptChampion` is never called, no backup copy is created, saving substantial overhead.

### Expected Performance and Design Benefits
- **Zero-Copy Rejections**: In common paths where no local SP improvements are found, copy overhead is reduced to zero.
- **Clean API**: The optimizer code is simplified, shielding it from internal backup variables and states.

## 2026-07-21 — Grounded read of the call sites + refined API

Before implementing, read every cache-touching site on the serialized path to pin
exactly what the "transaction" must protect and where it can fire.

### What mutates the champion on the serialized path

`EvaluateTimeLimitConfig_SubIncremental` (`OptimizeSP_TL_Incre.cpp:179-300`) is the
transaction scope. Inside one call, the cache is touched at:

- `:285` `rta_cache_.Evaluate(dag_tasks_cur, opt_pa_, time_limits)` — the baseline
  re-score. Writes `candidate_rta_` ONLY (the scratch buffer). **Champion state
  untouched.**
- `:291-293` `challenger.OptimizeIncre_SingleTask(..., std::ref(rta_cache_))` —
  internally calls `Evaluate` (scratch only) and, on every strict 1D improvement,
  `AdoptChampion` (`OptimizeSP_Incre.cpp:351-355`). **This is the SPECULATIVE
  champion mutation that must roll back on rejection.**
- `:295` `UpdateRecords(challenger, time_limits)` → on the accept branch calls
  `CommitIncumbent` (`:775-796`), which itself does `Evaluate` + `AdoptChampion`
  on the committed triple (`:791-795`). **This is the COMMIT; must NOT roll back.**

So the only champion mutator is `AdoptChampion`, and it fires at two sites with
opposite transaction roles. The 5 champion members it mutates: `rta_`,
`champ_prioritized_`, `champ_tasks_baked_`, `champ_per_core_`, `hp_prefix_per_core_`.

### `candidate_rta_` does NOT need rollback

It's a scratch buffer. On every `Evaluate` it is fully overwritten before any read:
`resize(rta_.size())` is a no-op under the invariant N (candidate task set ==
champion task set), the reindex loop writes every FullReuse slot, and the recompute
loop overwrites every NoReuse slot. After a reject-restore it is stale but the very
next `Evaluate` overwrites it fully before read. Excluding it from the snapshot
removes one of the three heavy `vector<FiniteDist>` copies (the other two are
`rta_` flat + `hp_prefix_per_core_` per-core prefixes).

### Two-shape API both exclude `candidate_rta_`; fork is lazy-vs-eager

**Lazy COW (filed design):**
- `RTACache::Transaction tx(rta_cache_);` registers `active_transaction_`.
- `AdoptChampion` calls `NotifyChampionAdoption()` at its top: if a tx is open and
  `snapshot_` is still null, `snapshot_ = make_unique(CaptureChampionState())`.
- `~Transaction`: if `!committed_ && snapshot_`, `RestoreChampionState(move(*snapshot_))`;
  deregister either way.
- `tx.Commit()`: sets `committed_ = true`.
- Zero copy when no `AdoptChampion` fires (reject-without-adopt, common near a
  local optimum). Cost: a back-pointer member + a hook in `AdoptChampion`'s hot
  path + a first-capture flag.

**Eager 5-member snapshot (alternative):**
- `RTACache::Transaction tx(rta_cache_);` captures immediately:
  `snapshot_ = make_unique(CaptureChampionState())`.
- `~Transaction`: if `!committed_`, restore; deregister.
- No `NotifyChampionAdoption` hook, no back-pointer on the hot path.
- Pays one 5-member copy every walk step. Still cheaper than today (today copies 6
  members including `candidate_rta_`).

Both are bit-identical (same 5 members, same restore-on-reject). Lazy is strictly
≤ eager in copy count and wins on the reject-without-adopt path; the cost is one
extra branch in `AdoptChampion`'s hot path + the back-pointer.

### Accept-path adopt ordering (verified)

`UpdateRecords` (`:105-139`) returns `bool`. `CommitIncumbent`'s adopt (`:794`)
fires INSIDE `UpdateRecords` on the accept branch — i.e. BEFORE `:295` returns and
BEFORE `tx.Commit()` at the call site. So when the accept-path adopt runs, the tx is
still open and `NotifyChampionAdoption` sees the already-captured snapshot (first-
capture-only → no-op). Then `tx.Commit()` keeps it. The accept-path adopt is
preserved, not rolled back. Correct for both designs.

### Status

Design grounded + recorded. **Decision 2026-07-21: Lazy COW** (user premise:
champion updates rare, give-ups common). Per-outcome copy count:

| Walk-step outcome | AdoptChampion fires inside? | Lazy | Eager |
|---|---|---|---|
| Accept (outer) | yes | 1 | 1 |
| Reject, no inner 1D improvement | no | **0** | 1 |
| Reject, inner 1D improvement, < outer | yes | 1 | 1 |

Lazy ≤ eager every row, strictly better on reject-without-adopt (row 2, common in
a TL descent). Capture (O(N) FiniteDist deep copy) moves off the common path; the
`NotifyChampionAdoption` branch sits on the rare adopt path. Capture hook guards
every full-champion overwrite → top of `AdoptChampion` AND `Initialize` (before any
member write); `Evaluate` untouched (scratch buffer only). Proceeding TDD-first.

## 2026-07-21 — Implementation landed + integrated (Phase 1, 2a, 2b, 2c)

### Phase 1 + 2a — cache-side API (RTA_Cache.{h,cpp})

All declared + implemented in the working tree (NOT yet committed):

- `struct ChampionState` (`RTA_Cache.h:55-61`): the Memento over the 5 champion
  members `rta_` / `champ_prioritized_` / `champ_tasks_baked_` / `champ_per_core_`
  / `hp_prefix_per_core_`. `candidate_rta_` DELIBERATELY excluded (scratch buffer;
  fully overwritten before read on every Evaluate → never needs rollback; cuts one
  of the three heavy `vector<FiniteDist>` copies from the snapshot).
- `class RTACache::Transaction` (`RTA_Cache.h:120-137`): RAII, non-copyable,
  non-movable, no nesting (ctor asserts `active_transaction_ == nullptr`).
  `Commit()` is `noexcept` + idempotent. `~Transaction` deregisters, then restores
  iff `!committed_ && snapshot_ != nullptr`.
- `active_transaction_` back-pointer (`RTA_Cache.h:268`) + `friend class
  Transaction` (`:264`).
- `NotifyChampionAdoption()` (`RTA_Cache.cpp:267-274`): first-capture-only.
  Called at the TOP of `Initialize` (`:196`) and `AdoptChampion` (`:215`), BEFORE
  any member write.
- `CaptureChampionState()` (`:278-286`) / `RestoreChampionState(ChampionState&&)`
  (`:290-296`): copy-out / move-back of the 5 members.

### Phase 2c — cache-level pins (testRTA.cpp)

3 pins appended at `testRTA.cpp:570+`, one per dev_log-table outcome. All compare
`cache.Rta()` (the STORED champion) directly — NOT `Evaluate(...)` — because
Evaluate returns a correct CANDIDATE RTA for any |diff|<=1 vs whatever champion
happens to be stored, so an Evaluate-based check would pass even with a wrong
(un-rolled-back) champion (a vacuous pin). The Transaction contract is about the
stored CHAMPION state, which is what the NEXT walk step's |diff| is measured
against.

- `Transaction_Commit_KeepsInTxAdopt` — ACCEPT: in-tx adopt survives `Commit()`.
- `Transaction_NoCommit_RollsBackInTxAdopt` — REJECT-WITH-ADOPT: in-tx adopt
  rolled back; secondary `Evaluate` identity check catches a restore that put back
  the right `rta_` but left `champ_*` bakes stale.
- `Transaction_NoAdopt_NoCommit_ChampionUntouched` — REJECT-WITHOUT-ADOPT: zero-
  copy path; null snapshot → no-op restore; champion byte-identical to pre-tx.

### Phase 2b — integration (OptimizeSP_TL_Incre.cpp)

`EvaluateTimeLimitConfig_SubIncremental` (`:179-317`) switched from the P1.16
eager full-cache copy to the lazy Transaction:

- REMOVED `RTACache cache_backup = rta_cache_;` (was `:189`) + `if (!updated)
  rta_cache_ = cache_backup;` (was `:296-298`).
- ADDED `RTACache::Transaction tx(rta_cache_);` at entry (`:201`) + `if (updated)
  tx.Commit();` after `UpdateRecords` (`:308-313`).

Accept-path ordering RE-VERIFIED at integration time:
`UpdateRecords` (`:105-141`) returns `bool`; on the accept branch it calls
`CommitIncumbent` (`:131`), which (gated by `rta_cache_active_`, armed `true` at
`:448` for the walk body) calls `Evaluate` + `AdoptChampion` (`:792-794`) — all
BEFORE `UpdateRecords` returns, i.e. BEFORE `tx.Commit()` at the call site. So the
accept-path adopt fires while the tx is still open → `NotifyChampionAdoption` sees
an already-captured snapshot (first-capture-only → no-op) → `tx.Commit()` retains
it. The accept-path adopt is preserved, not rolled back. Correct.

### Gate

17/17 ctest GREEN from a fresh `-DCMAKE_BUILD_TYPE=DEBUG` build (21.11s) AFTER the
2b integration. The load-bearing regression checks:
- `testRTA` — 56/56 incl. the 3 new Transaction pins.
- `testIncreOpt_w_TL` — the TL walk that exercises
  `EvaluateTimeLimitConfig_SubIncremental` end-to-end (accept + reject paths).
- `testOptimizeIncrePA` — the cache path (`OptimizeIncre_SingleTask`'s speculative
  adopts).
- `testINCRTimeout` — P1.14 cancel contract (unchanged; the tx is inert under
  cancel: a cancelled eval returns INT_MIN, loses the strict-> adopt test, so no
  adopt fires → ~tx is the zero-copy no-op).

Bit-identical SP confirmed (no oracle value changed; only the backup mechanism
changed). Phase 3b (perf measurement) deferred — it is a perf, not correctness,
gate; the per-outcome copy table already proves lazy ≤ eager in every row.

### Status

Phase 1 + 2a + 2b + 2c + 3a + 3c DONE in the working tree (NOT committed). 3b
(perf measurement) deferred. Ready for user review + `git commit`.

## 2026-07-21 — Decoupled / readable redesign (Phase 4)

### Trigger
User: the lazy impl was correct but unreadable (`NotifyChampionAdoption` cited
as the lying-name example); "i need a re-design without sacrificing current
code's efficiency" (PINS lazy COW). Then: "i asked gemini to generate a new
design ... read the design, consider whether what else you can do to improve
it." Gemini's `design.md` proposed moving tx state into the cache with an
explicit Begin/Commit/Rollback API + `AdoptEvaluatedCandidate`. Critique
identified 1 correctness-ish gap + several simplicity wins; user then said
"you can start implementing it."

### What landed (Phase 4, working tree, NOT committed)

**RTA_Cache.h:**
- `active_transaction_` back-pointer + `friend class Transaction` REMOVED.
- Cache now owns tx state directly: `bool in_transaction_` +
  `std::unique_ptr<ChampionState> snapshot_`. ONE source of truth — Gemini's
  design proposed a redundant `has_snapshot_` bool alongside `snapshot_`; dropped
  it (null-check on the pointer suffices, two invariants not needed).
- Public tx API: `BeginTransaction()` / `CommitTransaction()` /
  `RollbackTransaction()` / `InTransaction()`.
- `class Transaction` is now a THIN RAII wrapper (no friend, no back-pointer,
  no `snapshot_` member): ctor calls `BeginTransaction`; dtor is the SINGLE
  cleanup point — dispatches to `CommitTransaction` (if `committed_`) else
  `RollbackTransaction`. This fixes a subtlety in the first cut where `Commit()`
  only set the flag and never called `CommitTransaction`, leaving
  `in_transaction_=true` post-commit → the next `BeginTransaction` would
  assert-fail. Dtor-as-single-resolution-point is also exception-safe.
- `NotifyChampionAdoption()` renamed → `SnapshotPreMutationStateIfOpen()`
  (honest verb; the old name lied — "notify" implied passive observation, but
  it captures state). Header doc explains WHY the hook lives on the cache side:
  lazy capture means "snapshot the instant before the first in-scope mutation,"
  and only the mutator knows that instant.

**RTA_Cache.cpp:**
- `SnapshotPreMutationStateIfOpen()` guards on `in_transaction_ && snapshot_ ==
  nullptr` (was `active_transaction_ != nullptr && active_transaction_->snapshot_
  != nullptr`).
- `BeginTransaction()`: `assert(!in_transaction_)` (no nesting, now explicit —
  Gemini §4 prose claimed it, §2.2 didn't show it); flips the flag; `snapshot_.reset()`.
- `CommitTransaction()`: clears flag + drops snapshot (keep mutations).
- `RollbackTransaction()`: clears flag + restores from `snapshot_` if non-null
  (else no-op — the zero-copy reject-without-adopt path).
- `AdoptEvaluatedCandidate(dag, pa, tl)`: NEW public method.
  `Evaluate(dag, pa, tl)` then `AdoptChampion(dag, pa, tl, candidate_rta_)`.
  On the committed triple the Evaluate short-circuits to |diff|==0 FullReuse →
  returns the cached `candidate_rta_` with NO RTA recompute. So the "zero extra
  work" property is real AND safe (freshness guaranteed, not assumed — the
  alternative of silently trusting `candidate_rta_` was rejected as fragile).

**OptimizeSP_TL_Incre.cpp:**
- `CommitIncumbent`'s `Evaluate`+`AdoptChampion` pair (incl. the user's own
  `// TODO: ... duplicate RTA calculation?` at :803) → ONE call:
  `rta_cache_.AdoptEvaluatedCandidate(dag_tasks_, pa, tl)`. The TODO is now
  explicitly resolved (the method doc + call-site comment state the refetch is a
  zero-cost reuse, not a duplicate RTA).
- Tx comments in `EvaluateTimeLimitConfig_SubIncremental` refreshed
  (`NotifyChampionAdoption` → `SnapshotPreMutationStateIfOpen`; "~tx restores" →
  "~tx calls RollbackTransaction"; "tx.Commit() ... NOT restore" → "marks
  accepted; ~tx calls CommitTransaction").

### Finding #1 (Initialize hook) — resolved, NOT a live regression
Gemini's §3.1 listed only `AdoptChampion`/`AdoptEvaluatedCandidate` as snapshot
triggers, omitting `Initialize`. Traced: `Initialize` is a full-champion
mutator (BakeChampionForms + rta_ + candidate_rta_), so omitting it would let
an in-tx `Initialize` on a different triple mutate without snapshot →
reject-rollback fails → next walk step sees a wrong champion. BUT today
`Initialize` does NOT fire inside a tx scope: the tx body
(`EvaluateTimeLimitConfig_SubIncremental:201`) calls `OptimizeIncre_SingleTask`
(→ `Evaluate`+`AdoptChampion` only), NOT `OptimizeIncre` (where the `Initialize`
calls at `OptimizeSP_Incre.cpp:396/407` live); and the cache is pre-seeded by
`CommitIncumbent` at `:474` before the walk, so `Evaluate`'s `!HasChampion()` →
`Initialize` fallback (`RTA_Cache.cpp:466`) doesn't trigger either. So this is
DEFENSIVE coverage of a latent in-tx path, not a live correctness regression.
Kept the hook on `Initialize` regardless: one cheap branch, and it makes the
invariant "every full-champion overwrite is guarded" self-evidently true (so a
future caller that does `Initialize` in-scope is safe by construction).

### Gate
17/17 ctest DEBUG green (7.97s); 3 Transaction pins GREEN under the new decoupled
machinery; `testIncreOpt_w_TL` + `testOptimizeIncrePA` GREEN → bit-identical SP.
The `Transaction_NoAdopt_NoCommit_ChampionUntouched` pin (null-snapshot zero-
copy path) still holds: `Evaluate`-only in-scope leaves `snapshot_` null →
`RollbackTransaction` no-ops.

### What the design got right (acknowledged)
Honest hook name; `AdoptEvaluatedCandidate` resolves the user's own TODO; full
decoupling (no friend, no back-pointer, Transaction is a pure RAII wrapper over
public API); keeps lazy efficiency (zero-copy on reject-without-adopt). The
redesign is Gemini's design + the 4 critique fixes (Initialize hook,
has_snapshot_ dropped, AdoptEvaluatedCandidate freshness-guaranteed-not-assumed,
BeginTransaction assert).

### Status
Phase 4 DONE in the working tree (NOT committed). Ready for user review +
`git commit`. 3b (perf measurement) still deferred.

## 2026-07-21 — Phase 5: ChampionState AS the storage type (encapsulation)

### Trigger
User: "if you add ChampionState, you should use it in RTACache to encapsulate
some members." Until now `ChampionState` was a Memento struct that DUPLICATED
the field set of 5 loose private members (`rta_` / `champ_prioritized_` /
`champ_tasks_baked_` / `champ_per_core_` / `hp_prefix_per_core_`); capture/
restore were 5 hand-written field copies each. If a 6th champion member were
added, the Memento would silently forget it → incomplete rollback → latent
correctness bug. Refactor + perf-neutral (NOT correctness), but it removes a
whole class of drift bug by construction.

### What landed (Phase 5, working tree, NOT committed)
- `ChampionState` is now BOTH the live storage type AND the Memento type:
  `RTACache::champion_` is a single `ChampionState` member (replaces the 5
  loose members). `candidate_rta_` stays a separate scratch buffer (NOT in the
  struct — same exclusion as before).
- `CaptureChampionState()` collapses to `return champion_;` (struct copy) and
  `RestoreChampionState(ChampionState&&)` to `champion_ = std::move(state);`.
  No hand-written field list to drift.
- ~16 access sites in `RTA_Cache.cpp` retargeted: `champ_prioritized_` →
  `champion_.champ_prioritized`, `rta_` → `champion_.rta`, etc. Field names
  INSIDE `ChampionState` unchanged (preserves codebase symbols; mechanical
  retargeting).
- `HasChampion()` / `Rta()` accessors now read `champion_.rta`. The test pins
  read `cache.Rta()` (the accessor), NOT the members, so they're untouched.
- Header docs updated: `ChampionState` doc now states it is the storage type +
  Memento (drift-proof by construction); `BakeChampionForms` / `RebuildPrefixes`
  / `PerCoreOrderOfPrioritized` / `AdoptChampion` doc comments retargeted to
  the qualified `champion_.<field>` names.

### Gate
17/17 ctest DEBUG green (8.16s); 3/3 Transaction pins green; `testRTA` green
(incl. `Evaluate_*CrossCoreScramble_SP_BitIdenticalToOracle`,
`Evaluate_NoReuseBitIdenticalToOracle_*`,
`Evaluate_PriorityMoveSequence_BitIdenticalToOracle`,
`AdoptChampion_RebuiltPrefixesValidForNextPatch`, the multi-champion staleness
guards) → bit-identical SP. `testIncreOpt_w_TL` + `testOptimizeIncrePA` green.

### Status
Phase 5 DONE in the working tree (NOT committed). Ready for user review +
`git commit`. Stacked on Phase 4 (same commit set).

## 2026-07-21 — Phase 6: drop `AdoptEvaluatedCandidate` (the wrapper) + stale-build ABI postmortem

### Trigger
User: "remove the funtcion AdoptEvaluatedCandidate". Phase 4c added
`AdoptEvaluatedCandidate(dag,pa,tl)` as a named wrapper around the
`Evaluate`+`AdoptChampion` pair in `CommitIncumbent`, to make the "adopt what I
just scored" intent read as one line. User decided the wrapper is not worth the
extra API surface — inline the pair and drop the method.

### What landed (Phase 6, working tree, NOT committed)
- Removed `RTACache::AdoptEvaluatedCandidate` — decl + 13-line doc block in
  `RTA_Cache.h` (between `AdoptChampion` and `Evaluate`), impl in `RTA_Cache.cpp`.
- `CommitIncumbent` (`OptimizeSP_TL_Incre.cpp:808`) now inlines the pair:
  `const auto& rtas = rta_cache_.Evaluate(dag_tasks_, pa, tl);`
  `rta_cache_.AdoptChampion(dag_tasks_, pa, tl, rtas);` — gated by
  `rta_cache_active_`, with the "zero-cost reuse, not duplicate RTA" comment
  retained. Short local `rtas` per the incremental-optimizer short-names rule.
- Semantically identical to the removed wrapper (it did exactly this internally),
  so no behavior change. Item 4c is SUPERSEDED by this removal — the named
  wrapper is gone; the intent now lives in the call-site comment.

### Stale-build ABI postmortem (NOT a code defect)
While verifying Phase 6, ctest showed 5 failures (`testBF_w_TL`,
`testINCRTimeout`, `testIncreOpt_w_TL`, `testOptimizeIncrePA`,
`testScheduleSimulate`) aborting with `double free or corruption (out)` in
`RTACache::BakeChampionForms` (`RTA_Cache.cpp:244`,
`champion_.champ_tasks_baked = ApplyTimeLimitsToTasksExecutionTime(...)`) →
`vector<Task>::_M_move_assign` destroying a vector with `__first=<stack addr>`,
`__last=0x1` (garbage). `testRTA` itself passed 59/59; the crash was in the
optimizer-path tests via `OptimizeIncre` → `Initialize` → `BakeChampionForms`.

Root cause: **stale-object ABI mismatch**, NOT a P1.21 code bug. Phase 5
changed `RTA_Cache.h`'s class layout (5 loose members → one `ChampionState
champion_`), but `build_test/` held `.o` files compiled against the PRE-Phase-5
header. An incremental build hadn't recompiled the dependents, so two TUs
disagreed on `RTACache`'s member offsets → `champion_.champ_tasks_baked` read
as a garbage stack-pointer vector → double-free on assign. Confirmed:
- Clean worktree at HEAD `ecbed896` (pre-P1.21) passes 56/56 `testRTA` + 26/26
  `testOptimizeIncrePA` (no crash).
- `cmake --build build_test --target testOptimizeIncrePA --clean-first` (full
  recompile against the current header) → `testOptimizeIncrePA` 26/26 PASS, no
  crash. Then `cmake --build build_test --target check.SP_OPT -j5` (builds ALL
  test binaries + runs ctest) → **17/17 PASS (23.55s)**.

Lesson: when `RTA_Cache.h` layout changes, an incremental `cmake --build` may
leave stale `.o` files in `build_test/` (the dependency tracking did not catch
every consumer). After any header-layout change, build with `--clean-first` OR
run `cmake --build build_test --target check.SP_OPT` (which rebuilds all test
binaries) — do NOT trust a partial incremental build's ctest result. The
"17/17 green (8.16s)" Phase 5 claim was TRUE when freshly built; the crash
appeared only later against a half-rebuilt tree.

### Gate
17/17 ctest DEBUG green (23.55s) from a clean `check.SP_OPT` build AFTER the
`AdoptEvaluatedCandidate` removal. 3/3 Transaction pins still green;
`testRTA` 59/59; `testIncreOpt_w_TL` + `testOptimizeIncrePA` green →
bit-identical SP.

### Status
Phase 6 DONE in the working tree (NOT committed). `git add`'d
(`RTA_Cache.h`, `RTA_Cache.cpp`, `OptimizeSP_TL_Incre.cpp`). Ready for user
review + `git commit`.

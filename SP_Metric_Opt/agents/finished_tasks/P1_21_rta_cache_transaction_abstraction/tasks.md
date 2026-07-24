# P1.21 — Tasks (working checklist)

> **STATUS: CLOSED 2026-07-23 → moved to `finished_tasks/`.** All code committed
> at HEAD (`4d7d14b6` "add rta cache transaction" — the full Phases 1–6 redesign,
> incl. Phase 5's `ChampionState champion_` storage-type refactor + Phase 6's
> `AdoptEvaluatedCandidate` removal, all landed together in the committed
> `RTA_Cache.h`). 17/17 ctest green (23.55s from a clean `check.SP_OPT` build);
> bit-identical SP gate held through every phase. Last open item — 3b (perf
> measurement) — is an explicitly-deferred NON-gate; its subject (the
> Transaction's measured cost) is delegated to open task P1.22.

> Refactor + perf. Bit-identical SP output is the only acceptance gate.
> Scoped RAII transaction abstraction to replace upfront full-cache copy.
> Per the agent rule: `git add` only; the user commits.

## Phase 1 — API Design and Framework Setup

- [x] **1a. Declare structures in `RTA_Cache.h`**
  - Define `struct ChampionState` containing all mutating champion variables.
  - Define `class RTACache::Transaction` with constructor, destructor, and `Commit()`.
  - Add `active_transaction_` pointer and friend class declaration.
- [x] **1b. Implement capture and restore helpers in `RTA_Cache.cpp`**
  - Implement `CaptureChampionState()` returning a `ChampionState` (by value; moved into the tx's `unique_ptr`).
  - Implement `RestoreChampionState(ChampionState&&)` with moves.
- [x] **1c. Implement nested class `Transaction` in `RTA_Cache.cpp`**
  - Setup registration/deregistration of the transaction.
  - Implement automatic rollback on destruction if not committed.

## Phase 2 — Lazy COW & Integration

- [x] **2a. Update `AdoptChampion` to trigger lazy copy**
  - `NotifyChampionAdoption()` at the top of BOTH `AdoptChampion` AND `Initialize`.
  - First-capture-only: no-op on 2nd+ adopt in the same tx; no-op when no tx open.
- [x] **2b. Integrate into `OptimizeSP_TL_Incre.cpp`**
  - Removed `RTACache cache_backup = rta_cache_;` + `rta_cache_ = cache_backup;` in `EvaluateTimeLimitConfig_SubIncremental`.
  - `RTACache::Transaction tx(rta_cache_);` at entry.
  - `tx.Commit()` on successful `UpdateRecords`.
- [x] **2c. Pin correctness with tests**
  - 3 cache-level pins in `testRTA.cpp` (Commit-keeps / NoCommit-rolls-back / NoAdopt-noop), all GREEN.

## Phase 3 — Verify + Measure

- [x] **3a. Full gate green**
  - 17/17 ctest green from fresh `-DCMAKE_BUILD_TYPE=DEBUG` build (21.11s) AFTER 2b integration.
  - testRTA 56/56 (incl. 3 new Transaction pins); testIncreOpt_w_TL + testOptimizeIncrePA (the sub-incremental + cache path exercisers) GREEN → bit-identical SP confirmed.
- [x] **3b. Document performance impact** — DEFERRED-TO-P1.22 (2026-07-23).
  - Compare execution times of walk steps with and without lazy copy-on-write
    transaction pattern. NOT a correctness gate; closed as a P1.21 item because
    the measured-cost question is the entire subject of open task P1.22
    (investigate the Transaction's slowdown: high-frequency mid-walk
    `AdoptChampion`, `make_unique<ChampionState>` heap churn, redundant
    `RebuildPrefixes`, pointer indirection). P1.21 = refactor+gate; the
    copy-count table in dev_log already proves lazy ≤ eager per outcome.
- [x] **3c. Update overall tasks and logs**
  - dev_log + tasks updated; memory file + MEMORY.md updated.

## Phase 4 — Decoupled / readable redesign (Gemini design + critique fixes)

> Constraint: keep lazy COW efficiency (zero-copy on reject-without-adopt).
> Goal: clear, easy-to-read code; cut the RTACache↔Transaction coupling.

- [x] **4a. Move tx state INTO the cache; explicit public API (`RTA_Cache.h/.cpp`)**
  - Replaced `active_transaction_` back-pointer + `friend class Transaction` with cache-owned
    `in_transaction_` bool + `snapshot_` (one source of truth — dropped the redundant
    `has_snapshot_` bool Gemini's design proposed; null-check on `snapshot_` suffices).
  - Public API: `BeginTransaction()` (asserts `!in_transaction_`, zero-copy entry),
    `CommitTransaction()`, `RollbackTransaction()`, `InTransaction()`.
  - `Transaction` is now a thin RAII wrapper (no friend, no back-pointer, no member
    snapshot): ctor→`BeginTransaction`, dtor dispatches to `Commit`/`RollbackTransaction`
    based on `committed_` (dtor is the SINGLE cleanup point → exception-safe).
- [x] **4b. Honest snapshot hook name**
  - `NotifyChampionAdoption()` → `SnapshotPreMutationStateIfOpen()` (the old name lied —
    "notify" implied passive observation, but it captures state). Kept on BOTH `AdoptChampion`
    AND `Initialize` (every full-champion overwrite) — this is the ONE coupling point lazy
    COW requires; documented why it must live on the cache side (only the mutator knows the
    pre-mutation instant).
  - Finding #1 resolution: `Initialize` kept in the hook list (Gemini's §3.1 omitted it).
    Today `Initialize` does NOT fire inside a tx scope (the walk pre-seeds via
    `CommitIncumbent` and `OptimizeIncre_SingleTask` only calls `Evaluate`+`AdoptChampion`),
    so this is defensive coverage of a latent in-tx path (`Evaluate`'s `!HasChampion()` →
    `Initialize` fallback), NOT a live regression. The hook makes "every full-champion
    overwrite is guarded" self-evidently true.
- [x] **4c. `AdoptEvaluatedCandidate` resolves the user's TODO at `OptimizeSP_TL_Incre.cpp:803`**
  - Replaced the `Evaluate`+`AdoptChampion` pair in `CommitIncumbent` with one named call.
  - Finding #2 resolution: did NOT silently trust `candidate_rta_` matches the committed
    triple; the method internally calls `Evaluate(dag, pa, tl)` first, which short-circuits
    to |diff|==0 FullReuse (returns the cached `candidate_rta_`, zero RTA recompute) on the
    committed triple — so the "zero extra work" property is real AND safe (the freshness is
    guaranteed, not assumed). This makes the user's "duplicate RTA?" TODO explicitly false:
    the refetch is a zero-cost reuse, documented at the method + call site.
  - **SUPERSEDED by Phase 6 (2026-07-21):** the wrapper was removed at the user's request;
    `CommitIncumbent` now inlines `Evaluate`+`AdoptChampion`. The "zero-cost reuse, not
    duplicate RTA" intent lives in the call-site comment. See Phase 6.
- [x] **4d. Non-nesting assert in `BeginTransaction()`** (§4 prose claimed it; §2.2 didn't show
  it) — now explicit.
- [x] **4e. Document the lazy invariant** in the header: snapshot captures pre-FIRST-mutation
  state == tx-open state (nothing mutates between tx-open and first mutation), so a first-
  capture-only snapshot is correct.
- [x] **4f. Verify gate + test-pin mapping**
  - 17/17 ctest DEBUG green; 3 Transaction pins GREEN under the new decoupled machinery;
    testIncreOpt_w_TL + testOptimizeIncrePA GREEN → bit-identical SP. The `NoAdopt` pin
    (null-snapshot zero-copy path) still holds: `Evaluate`-only in-scope leaves `snapshot_`
    null → `RollbackTransaction` no-ops.
- [x] **4g. Update records** — tasks/dev_log/memory/MEMORY.md.

## Phase 5 — ChampionState AS the storage type (encapsulation)

> Goal: `ChampionState` is both the live storage type AND the Memento type —
> capture/restore collapse to struct copy/move, and a future 6th champion
> member cannot drift between live state and the snapshot (the Memento IS the
> state). Perf-neutral refactor (NOT correctness); gate = bit-identical SP.

- [x] **5a. Replace the 5 loose champion members with one `ChampionState champion_`**
  - `RTACache` private section: removed `rta_` / `champ_prioritized_` /
    `champ_tasks_baked_` / `champ_per_core_` / `hp_prefix_per_core_`; added a
    single `ChampionState champion_`. `candidate_rta_` stays separate (scratch).
  - `ChampionState` doc updated: it is now BOTH the storage type AND the Memento
    (drift-proof by construction — `candidate_rta_` excluded from the struct, so
    a snapshot can never accidentally capture the scratch buffer either).
- [x] **5b. Collapse `CaptureChampionState`/`RestoreChampionState`**
  - `Capture` → `return champion_;` (struct copy); `Restore` →
    `champion_ = std::move(state);`. No hand-written 5-field list to drift.
- [x] **5c. Retarget ~16 access sites in `RTA_Cache.cpp`**
  - `champ_prioritized_` → `champion_.champ_prioritized`, `rta_` →
    `champion_.rta`, etc. Field names INSIDE `ChampionState` unchanged (preserves
    codebase symbols; mechanical retargeting). `HasChampion`/`Rta()` accessors
    retargeted; test pins read `cache.Rta()` so untouched.
- [x] **5d. Refresh header doc comments** — `BakeChampionForms` /
  `RebuildPrefixes` / `PerCoreOrderOfPrioritized` / `AdoptChampion` now reference
  the qualified `champion_.<field>` names.
- [x] **5e. Verify gate**
  - 17/17 ctest DEBUG green (8.16s); 3/3 Transaction pins green; `testRTA` green
    (incl. `Evaluate_*CrossCoreScramble_SP_BitIdenticalToOracle`,
    `Evaluate_NoReuseBitIdenticalToOracle_*`,
    `Evaluate_PriorityMoveSequence_BitIdenticalToOracle`,
    `AdoptChampion_RebuiltPrefixesValidForNextPatch`, the multi-champion
    staleness guards) → bit-identical SP. `testIncreOpt_w_TL` +
    `testOptimizeIncrePA` green.
- [x] **5f. Update records** — dev_log/tasks/memory/MEMORY.md.

## Phase 6 — Drop `AdoptEvaluatedCandidate` (the wrapper)

> User: "remove the funtcion AdoptEvaluatedCandidate". Phase 4c's named wrapper
> around `Evaluate`+`AdoptChampion` is not worth the API surface — inline the
> pair, drop the method. Semantically identical (the wrapper did exactly this).

- [x] **6a. Remove `AdoptEvaluatedCandidate` decl + impl**
  - Deleted the decl + 13-line doc block in `RTA_Cache.h` (between `AdoptChampion`
    and `Evaluate`); deleted the impl in `RTA_Cache.cpp`.
- [x] **6b. Inline the pair at `CommitIncumbent` (`OptimizeSP_TL_Incre.cpp:808`)**
  - `const auto& rtas = rta_cache_.Evaluate(dag_tasks_, pa, tl);`
    `rta_cache_.AdoptChampion(dag_tasks_, pa, tl, rtas);` — gated by
    `rta_cache_active_`; "zero-cost reuse, not duplicate RTA" comment retained.
  - Short local `rtas` per the incremental-optimizer short-names rule.
- [x] **6c. Verify gate**
  - 17/17 ctest DEBUG green (23.55s) from a clean `check.SP_OPT` build; 3/3
    Transaction pins green; `testRTA` 59/59; `testIncreOpt_w_TL` +
    `testOptimizeIncrePA` green → bit-identical SP.
- [x] **6d. Stale-build ABI postmortem** (NOT a code defect — recorded so it
  isn't re-investigated as a bug). Phase 5's `RTA_Cache.h` layout change (5 loose
  members → `ChampionState champion_`) + a half-rebuilt `build_test/` (stale `.o`
  files against the pre-Phase-5 header) → member-offset mismatch →
  `champion_.champ_tasks_baked` read as a garbage stack-pointer vector → double-
  free in `BakeChampionForms:244`. Clean rebuild (`--clean-first` / `check.SP_OPT`
  target) → gone. **After any `RTA_Cache.h` layout change, build with
  `--clean-first` or the `check.SP_OPT` target; do NOT trust a partial
  incremental build's ctest result.**
- [x] **6e. Update records** — dev_log/tasks/memory/MEMORY.md.

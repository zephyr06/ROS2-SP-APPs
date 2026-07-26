# P2.11 — Tasks (working checklist)

> Behavior-CHANGING merge: re-opt = incremental path + (RM-Fast interval-0 init)
> + (one upfront re-opt step seeding the champion) + (patience+1). NOT a refactor;
> needs its own A/B. Prerequisite: P2.10 landed.

## Phase 0 — Prerequisites + design confirm

- [x] **0a. P2.10 landed** (shared `OptimizeOneTaskTimeLimit` + renames
  + Phase 5a bit-identity gate flag OFF = clean baseline).
- [x] **0b. Confirm reading (a) vs (b)** with the user. (a) = full merge incl.
  Type-E in the reopt queue (chosen in goal.md); (b) = Type-L only (= P2.9
  flag-on, zero new code). **RESOLVED 2026-07-25** — user approved the updated
  call path (which includes Type-E on the reopt queue). Reading (a) confirmed.
- [x] **0c. Confirm RM-Fast interval-0 semantics** — strict smallest-TL
  (`SmallestTimeLimitVec`) vs Gaussian-mean (`InitializeTimeLimitsFromETConfig`).
  **RESOLVED 2026-07-25** — `ResetIncumbentBaseline(true)` at interval 0 already
  seeds `SmallestTimeLimitVec()` + `RateMonotonicPriorityVec()` = RM-Fast (strict
  smallest-TL). The Gaussian-mean is only the seed vector into
  `OptimizeIntervalFromScratch`; the reset overrides it.
- [x] **0d. Verify the Type-E plumbing preserves `|diff|<=1`** — read
  `RunReoptTLDescent` + `BuildSerializedTaskQueue` + `EvaluateTimeLimitConfig_SingleTaskPatch`
  in `OptimizeSP_TL_Incre.cpp`; confirm a Type-E entry (env-changed task at its
  committed TL) is a single-task diff vs the champion, so the cache contract
  holds. If NOT, fall back to reading (b) or special-case Type-E in reopt.
  **VERIFIED 2026-07-25** — Type-E = `|diff|==0` (env move absorbed into
  `dag_tasks_` on both sides → cancels), the lighter FullReuse path. Contract
  holds PROVIDED `dag_tasks_prev_pre_tl` is captured before the absorb (Phase 3a).

## Phase 0.5 — Rename misnamed eval (REVERTED — not pursued)

> **SUPERSEDED 2026-07-25.** The `_PAReopt` rename was reverted along with the
> P2.10 13-fn rename (dev_log 2026-07-25 "State correction first"). The code
> KEEPS the OLD name `EvaluateTimeLimitConfig_ScratchOrIncre`. The design docs
> (goal.md/tasks.md below) reference non-existent renamed symbols
> (`RunIntervalDescent`, `_PAReopt`, `_SingleTaskPatch`, `WalkOneTaskTimeLimit_
> FullBeam`) — these do NOT exist; all implementation uses the real names. This
> phase is NOT pursued; the checkboxes below are the historical record of the
> reverted attempt, not pending work.

- [x] **0.5a. Rename `EvaluateTimeLimitConfig_PAReopt` → `_PAReopt`** across
  `OptimizeSP_TL_Incre.{h,cpp}`, `Parameters.h`, `tests/testIncreOpt_w_TL.cpp`,
  `tests/testScheduleSimulate.cpp`. The name `GlobalBeam` is wrong — the fn does
  both `OptimizeFromScratch` (from_scratch=true) AND `OptimizeIncre` warm-start
  (from_scratch=false); it re-optimizes the PA. Parallels `_SingleTaskPatch`.
  *(REVERTED — code keeps `EvaluateTimeLimitConfig_ScratchOrIncre`)*
- [x] **0.5b. Build + test green** — `cmake --build build_test --target
  check.SP_OPT -j5 --clean-first` → 17/17 ctest green. Used `--clean-first`
  (stale-`.o` lesson from P1.21 after a header change). *(REVERTED)*
- [~] **0.5c. User review + commit** the rename as a standalone modular step
  BEFORE the behavior-changing merge (per "work by module, commit by module").
  *(VOID — the rename was reverted; nothing to commit. The merge landed without
  it — Phase 2 unconditionally routes reopt through the cache-routed
  `EvaluateTimeLimitConfig_SubIncremental`, and `EvaluateTimeLimitConfig_Scratch
  OrIncre` stays as the one upfront baseline-reopt eval only.)*

## Phase 1 — Unified descent body

> Real symbol map (design docs use non-existent renamed symbols — do NOT trust
> them): incremental descent body = `PerformSerializedTaskQueueOptimization`;
> reopt descent body = `PerformCoordinateDescentForTaskConfigOpt`; incremental
> entry = `OptimizeIncre_w_TL`; reopt entry = `ReOptimizePeriodic`; legacy
> full-beam walk wrapper = `OptimizeSingleTaskTimeLimit`; PA-reopt eval =
> `EvaluateTimeLimitConfig_ScratchOrIncre`.

- [x] **1a. Extract shared per-entry dispatch → `WalkSerializedTaskQueue`**
  (behavior-neutral dedup). The Type-E/Type-L per-entry loop was duplicated
  inline in `PerformSerializedTaskQueueOptimization` AND as a local lambda in
  the `PerformCoordinateDescentForTaskConfigOpt` flag-on arm; both now call one
  helper. **DONE 2026-07-25** — 17/17 ctest green (incl. `testIncreOpt_w_TL`
  which pins both the incremental serialized walk + the Phase 3b Type-E reopt
  reach test). Bit-identity gate: flag OFF (default) → incremental path only;
  the extracted body is byte-identical to the inline loop it replaced.
- [ ] **1b. Unify the two descent bodies into one `RunIntervalDescent(K, tl,
  mode, dag_prev_pre_tl)`** parameterized over `mode ∈ {Incremental, Reopt}`.
  Body selects by `mode`: baseline seed (dedicated re-score vs
  `EvaluateTimeLimitConfig_ScratchOrIncre` upfront re-opt), patience (0 vs 1),
  cache-active (true both, but reopt arms via the upfront `AdoptChampion` block).
  The walk is now shared via 1a's helper, so the remaining delta is the setup
  preamble + the legacy-arm fallback.
- [ ] **1c. Route `OptimizeIncre_w_TL` → `RunIntervalDescent(Incremental)`.**
- [ ] **1d. Route `ReOptimizePeriodic` → `RunIntervalDescent(Reopt)`** (interval-0
  RM-Fast bootstrap stays at the entry, before the descent).
- [ ] **1e. Delete `PerformSerializedTaskQueueOptimization` +
  `PerformCoordinateDescentForTaskConfigOpt`** once both callers route through
  `RunIntervalDescent`.

## Phase 2 — Delete the legacy reopt arm + P2.9 flag

> Only AFTER the Phase 5 A/B accepts the merge. **A/B accepted 2026-07-25 (5c);
> gate confirmed 2026-07-26 (N=10 = the accepted gate, no N=16 re-run).**

- [x] **2a. Delete `OptimizeSingleTaskTimeLimit`** (reopt-only legacy full-beam
  walk wrapper). `EvaluateTimeLimitConfig_ScratchOrIncre` STAYS — it is still
  the one upfront baseline re-opt step in reopt mode. **DONE 2026-07-26** — the
  7-arg wrapper was already deleted in the working tree (P2.11 Phase 1 prep);
  this step finished the migration by repointing the 7 `TrialAndErrorTLWalkSynthetic`
  walk-core tests to call `OptimizeSingleTaskTimeLimit_Impl` directly with an
  eval lambda built by a new `StubTLWalkOptimizer::MakeScratchOrIncreEval(K,
  from_scratch)` helper (eliminates 7× lambda repetition; behavior-preserving —
  the stub's `EvaluateTimeLimitConfig_ScratchOrIncre` override ignores
  `from_scratch`, so `evaluated_tls` recording + all assertions unchanged). The
  `_Impl` seam is the documented unit-test surface (header: "The walk core is
  unit-tested directly with an injected TL→SP stub").
- [x] **2b. Delete the `ReoptimizationUseSubIncrementalWalk` flag** + its
  `Parameters.h`/`parameters.yaml` entries + the 2 P2.9 `CounterDispatcherSynthetic`
  tests' flag-dispatch assertions (keep the `subincremental_calls` counter — it
  now asserts reopt ALWAYS hits the sub-incremental arm). **DONE 2026-07-26** —
  deleted the flag from `Parameters.{h,cpp}` + `parameters.yaml`; deleted
  `ReoptWalk_Legacy_Off_RoutesTrialsThroughScratchOrIncre` (its premise flag=0⇒
  legacy⇒`subincremental_calls==0` is gone); rewrote
  `ReoptWalk_LeverA_On_RoutesWalkTrialsThroughSubIncremental` →
  `ReoptWalk_RoutesWalkTrialsThroughSubIncremental` (drop flag set, keep
  `subincremental_calls>0` now unconditional) + `ReoptWalk_LeverA_On_ReachesEnvChangedTaskViaSerializedQueue`
  → `ReoptWalk_ReachesEnvChangedTaskViaSerializedQueue` (drop flag set, keep
  Type-E reach) + `ReoptFlagOn_AtInterval0_DoesNotThrowWhenReoptMovesMultipleTLs`
  → `Reopt_AtInterval0_DoesNotThrowWhenReoptMovesMultipleTLs` (drop flag set +
  remove the TEMP DIAGNOSTIC `[DIAG]` `std::cerr` block flagged in 5.5a);
  stripped `saved_subincremental_walk_` save/restore from both
  `CounterDispatcherSynthetic` + `ReoptFlagOnMultiTLFlexibleSynthetic` fixtures.
  `grep -rn ReoptimizationUseSubIncrementalWalk sources/ tests/` → empty.
- [x] **2c. Update comments** in `Parameters.h` + `OptimizeSP_TL_Incre.h` (the
  flag comment repointed in P2.10 Phase 3b now points at the unified body).
  **DONE 2026-07-26** — cleaned stray "flag-on arm" / "lever-A" phrasing in
  `OptimizeSP_TL_Incre.{h,cpp}` (3 sites: `WalkSerializedTaskQueue` header
  comment, the `.cpp:355` per-entry-dispatch comment, the `.cpp:754` pre-absorb
  capture comment) + the test-file `subincremental_calls` field comment + the
  `ReoptFlagOnMultiTLFlexibleSynthetic` fixture class comment. The config
  `p211_reopt_ab_config.json` `_comment` now leads with "A/B PASSED 2026-07-25;
  flag DELETED in Phase 2; procedure NO LONGER RE-RUNNABLE" (retained as the
  historical gate-procedure record).

## Phase 3 — Type-E in the reopt queue (reading (a) only)

- [x] **3a. Plumb `dag_prev_pre_tl` into `OptimizeIntervalFromScratch`** so
  `BuildSerializedTaskQueue` can diff champion DAG vs new DAG and emit Type-E
  entries on the reopt path. **DONE 2026-07-25** — `ReOptimizePeriodic` now
  captures `dag_tasks_prev_pre_tl` before absorbing `dag_tasks_update` and
  passes it (required arg, no default per the optional-arg rule) to
  `PerformCoordinateDescentForTaskConfigOpt`. Real symbol names retained (the
  P2.10/P2.11 rename table was reverted; code keeps
  `ReOptimizePeriodic`/`PerformCoordinateDescentForTaskConfigOpt`/
  `EvaluateTimeLimitConfig_ScratchOrIncre`/`_SubIncremental`).
- [x] **3b. TDD** — a `CounterDispatcherSynthetic` test asserting a reopt
  dispatch with an env-changed task produces Type-E entries (queue contains an
  `EnvChanged` kind for the moved task). **DONE 2026-07-25** —
  `ReoptWalk_LeverA_On_ReachesEnvChangedTaskViaSerializedQueue`: a flag-ON
  reopt on a DAG with an env-changed T_noise (no perf pair → `{-1}`-only,
  skipped by the legacy `sorted_indices` arm) must drive a
  `EvaluateTimeLimitConfig_SubIncremental` call with `task_idx==1`. Went RED
  first (`subincremental_task_idx: { 0, 0 }` — T_noise never reached), then
  GREEN after the flag-on arm walked `BuildSerializedTaskQueue(dag_tasks_prev_pre_tl)`
  with the same Type-E/Type-L dispatch as `PerformSerializedTaskQueueOptimization`.
  The `RecordingDispatcherOpt` seam was extended to record
  `subincremental_task_idx`. 17/17 ctest green. **Flag-gated (default OFF) →
  prod path bit-identical; only the flag-on path changed.**

## Phase 4 — Build + verify

- [x] **4a. Build** `cmake --build build_test --target check.SP_OPT -j5 --clean-first`.
  **DONE 2026-07-26** — clean build OK (used `--clean-first` per the stale-`.o`
  lesson from P1.21, since Phase 2 deleted the `OptimizeSingleTaskTimeLimit`
  wrapper + changed the `.h`).
- [x] **4b. 17/17 ctest green** + `testIncreOpt_w_TL` green (updated P2.9 tests +
  new Type-E test). **DONE 2026-07-26** — 17/17 ctest passed in 17.38s, incl.
  `testIncreOpt_w_TL` (1.66s, covers the rewritten `ReoptWalk_Routes*` /
  `ReoptWalk_ReachesEnvChangedTaskViaSerializedQueue` /
  `Reopt_AtInterval0_DoesNotThrowWhenReoptMovesMultipleTLs` + the 7 repointed
  `TrialAndErrorTLWalkSynthetic` walk-core tests via `MakeScratchOrIncreEval`).

## Phase 5 — A/B experiment (the gate; NOT bit-identity)

- [x] **5a. Run `INCR_Reopt_1` + `INCR_Reopt_10`**: current-default-reopt
  (P2.10 baseline, flag OFF) vs new-merged-reopt (flag ON). Compare SP +
  per-activation ET. **PASSED 2026-07-25** — the A/B ran
  (`p211_reopt_ab_config.json`, N=10, 5 tasksets, `BIN_DIR=release`) and the user
  accepted the merge ("A/B test already runs, consider it as passed"). History:
  the first flag=1 run CRASHED at `taskset_0/INCR_Reopt_1` with SIGABRT (uncaught
  `runtime_error` from `RTACache::ComputeTaskSetDifference`, `|diff|>1`) — the
  reopt flag-on arm adopted the champion TL from the from-scratch reopt but
  walked the un-re-synced Gaussian `starting_time_limits`, so the first walk
  step diffed champion-TL vs Gaussian in >1 task → throw. Fixed by re-syncing
  `starting_time_limits = ReconstructTimeLimitVecFromResOpt()` before the walk
  (mirrors the incremental path) — committed `a6922ff5` (5.5b) with the 5.5a
  regression guard. After the fix + release-build re-run, the merge was accepted.

## Phase 5.6 — Fix the flag=1 slowdown + TIME_LIMIT escape (prerequisite to 5a)

> The flag=1 arm (after the 5.5 crash fix) does NOT crash, but is **much
> SLOWER** than flag=0 for both `INCR_Reopt_1`/`_10`, AND escapes the per-
> interval `TIME_LIMIT=1s` budget ~10× (a taskset ran 5 min when max ~30s).
> This is the active blocker. Re-frames "slower than expected" as
> "cooperative budget not effective."

- [x] **5.6a. Confirm the budget-polling gap** — verify
  `BFSharedBudgetCancelled()` is ABSENT from the 3 flag=1 hot loops
  (`WalkSerializedTaskQueue`, `OptimizeSingleTaskTimeLimit_Impl`,
  `OptimizeIncre_SingleTask`) and identify where the legacy path
  (`OptimizeFromScratch` → `EvaluateSPWithPriorityVec`) DOES poll. Read
  `OptimizeSP_Base.cpp` for `EvaluateSPWithPriorityVec` + the BF beam cancel.
  **CONFIRMED 2026-07-25** — full call graph in dev_log; the gap it identified
  is fixed in 5.6b (`a6922ff5`). Polls EXIST in
  `RTA.cpp:102`, `SP_Metric.cpp:63/107/122`, `OptimizeSP_Base.cpp:192/206`,
  `OptimizeSP_Incre.cpp:71`, `OptimizeSP_TL_Incre.cpp:218`. Polls ABSENT in the
  3 flag=1 hot loops + the cache-path scorers (`RTACache::Evaluate` recompute,
  `ObtainSP_DAG_From_Dists`). DECISIVE defect: the `:218` entry poll guards only
  the baseline; `:228 OptimizeIncre_SingleTask` runs UNCONDITIONALLY with its
  O(N) PA loop unpollled → budget escape ~10× → ~5 min for `INCR_Reopt_1`.
- [x] **5.6b. Fix** — add `BFSharedBudgetCancelled()` polling at 4 sites
  (committed `a6922ff5`): (1) `EvaluateTimeLimitConfig_SubIncremental:228` —
  SKIP `OptimizeIncre_SingleTask` when the `:218` entry poll fired (highest
  leverage); (2) `OptimizeIncre_SingleTask` PA loop (`OptimizeSP_Incre.cpp:330`)
  — `if (BFSharedBudgetCancelled()) break;` per variation; (3)+(4)
  (defense-in-depth) `WalkSerializedTaskQueue:349` +
  `OptimizeSingleTaskTimeLimit_Impl:461` — break on cancel. All 4 inert outside
  a `BFDLSharedBudget` scope + inert for the within-budget incremental path →
  prod bit-identical. **Note:** the commit bundled 5.5b + 5.6b + the 5.5a
  regression test (the defects are interleaved in the same file region and
  could not be cleanly split without interactive staging). 17/17 ctest green.
- [x] **5.6c. Validate** — single fast reproducer: one taskset / `INCR_Reopt_1`
  / flag=1 that previously ran 5 min → completion in ~30s = budget respected =
  fixed. No new A/B, no TDD arc (per "speed up dev, skip TDD temporarily").
  **RESOLVED 2026-07-25 via P2.12 (`RunSpeedTest`)** — the standalone release-
  mode benchmark committed `6b5065c3` measures `INCR_Reopt_1` at **0.042 s/act**
  and `INCR_Reopt_10` at **0.013 s/act** (both ≪ 0.1 s threshold), PASS. The
  "5 min" reproducer and the budget escape it implied were the DEBUG-build
  artifact, not a real flag=1 cost: the slow A/B run had used `BIN_DIR=build_test`
  (~8.5× slower — see P2.11 5.5d NOTE + the `validate_bin_dir` enforcement
  committed `fa113d23`). Re-measured in release, flag=1 respects the budget.

## Phase 5.7 — SEPARATE ISSUE: flag=0 (prod) per-interval opt also slow at N=16
> **CLOSED 2026-07-25 — root cause = DEBUG-build artifact (the same one P2.12 +
> the release-only enforcement closed).** The reported "~5× per-interval slowdown"
> was measured against the `build_test` (DEBUG) binary, which is ~8.5× slower and
> turns absolute scheduler ET into a measurement artifact, not a real signal.
> P2.12's release-mode `RunSpeedTest` benchmark (`6b5065c3`) shows the prod
> flag=0 path at **0.013–0.042 s/act** (well under 0.1 s) — no real regression to
> bisect. The dedicated `validate_bin_dir` guard (`fa113d23`) now hard-rejects
> `build_test` on the e2e path so this artifact cannot recur. The 5.7a–d bisect
> is therefore moot and NOT executed.

- [x] **5.7a–d. Bisect / profile / fix the flag=0 per-interval slowdown.**
  **VOID — premise was the DEBUG artifact.** See the phase header above. No
  profiling suspects (P1.25 cache copy, P1.18 Rule B, P2.11 per-call overhead)
  were exercised — the "slow" number was the build, not the algorithm.

## Phase 5.5 — Fix the flag-on walk crash (DONE; superseded by 5.6 for the run)

- [x] **5.5a. TDD regression test** — drive a real multi-task TL divergence
  between the from-scratch reopt and the Gaussian seed (the fixture gap that let
  the crash ship). Assert a flag-ON reopt at interval 0 does NOT throw and
  completes the walk. **DONE 2026-07-25 (committed `a6922ff5`) as a
  no-regression guard** — `ReoptFlagOnMultiTLFlexibleSynthetic` fixture +
  `ReoptFlagOn_AtInterval0_DoesNotThrowWhenReoptMovesMultipleTLs`: builds 2
  TL-flexible tasks (`{100,200,300,400}` + Gaussian avg ~250 → seed=200) + a
  `{-1}`-only T_noise; `EXPECT_NO_THROW(opt.ReOptimizePeriodic(dag_tasks, 2))`,
  `EXPECT_GE(divergent, 2)` (pins the fixture reproduces the divergence),
  `EXPECT_GT(subincremental_calls, 0)` (walk reached). NOT a strict RED-first
  TDD arc (cf. P1.25 precedent) — a guard that pins the crash signature. The
  original "skip TDD temporarily" call (2026-07-25 (2) entry) was superseded:
  the real `p211_reopt_ab_config.json` run is too coarse to keep as a
  regression pin. **CLEANUP NEEDED:** the test still carries a TEMP DIAGNOSTIC
  `std::cerr` `[DIAG]` block from the divergence investigation — remove before
  the next commit touching this test.
- [x] **5.5b. One-line fix** — `PerformCoordinateDescentForTaskConfigOpt`:
  `starting_time_limits = ReconstructTimeLimitVecFromResOpt();` before
  `WalkSerializedTaskQueue`. **DONE 2026-07-25, committed `a6922ff5`** — applied
  at `sources/Optimization/OptimizeSP_TL_Incre.cpp` (the
  `if (use_subincremental_walk)` block before the `WalkSerializedTaskQueue`
  call). Re-syncs the walk vector to the adopted champion TL so the first walk
  step diffs champion-TL vs champion-TL (|diff|<=1) instead of champion-TL vs
  Gaussian-seed-TL (>1 → throw). Mirrors the incremental path
  (`PerformSerializedTaskQueueOptimization` establishes champion-TL ==
  walk-start TL via `CommitIncumbent`).
- [x] **5.5c. Rebuild + 17/17 ctest green** (`cmake --build build_test --target
  check.SP_OPT -j5 --clean-first` — header unchanged but safe). **DONE
  2026-07-25** — 17/17 ctest green (incl. `testIncreOpt_w_TL`); shipped in
  `a6922ff5`.
- [~] **5.5d. Re-run the flag=1 A/B arm** (`p211_reopt_ab_config.json`,
  `RERUN_MODE=clear_results`, `SKIP_EVAL=1`, `BIN_DIR=release`). **DONE but
  SUPERSEDED by 5.6** — the crash is fixed (no SIGABRT), but the run is far
  slower than flag=0 AND blows the TIME_LIMIT budget. See Phase 5.6.
  *(NOTE: the original 5.5d run used `BIN_DIR=build_test` — the DEBUG build,
  ~8.5× slower, which inflated the scheduler ET and was the source of the
  "optimization got much slower" confusion. All future A/B runs MUST use
  `BIN_DIR=release`. Corrected 2026-07-25; see p211 config `_comment_on_bin_dir`.)*
- [-] **5b. Compare against pure incremental (`INCR_Reopt_∞`/no reopt)** to test
  the user's hypothesis (merged reopt ≈ current frequent reopt, sometimes worse
  than incremental). **DEFERRED 2026-07-25** — not part of the gate (5a is the
  gate: merged-reopt vs current-default-reopt). The pure-incremental comparison
  is an interesting follow-up, not a blocker for landing the merge.
- [x] **5c. Verdict** — **ACCEPTED 2026-07-25.** The user declared the A/B passed
  ("A/B test already runs, consider it as passed"). Proceeding to Phase 2 (delete
  the legacy reopt arm + the `ReoptimizationUseSubIncrementalWalk` flag), which
  makes the merged reopt the unconditional path. Phase 1b-1e (unify the two
  descent bodies into `RunIntervalDescent(mode)`) follows as structural cleanup.

## Phase 6 — Closeout

- [ ] **6a. Update `agents/overall_tasks.md`** with P2.11.
- [ ] **6b. Add memory** `p211-merge-reopt-into-incremental.md` + one-line pointer
  in `MEMORY.md`.
- [ ] **6c. Update P2.9 + P2.10 records** — P2.9's flag is deleted by this task;
  P2.10's shared arm is the foundation. Cross-link.

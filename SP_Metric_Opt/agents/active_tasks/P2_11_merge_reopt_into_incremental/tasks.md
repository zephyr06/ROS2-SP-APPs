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

## Phase 0.5 — Rename misnamed eval (behavior-neutral, standalone commit)

- [x] **0.5a. Rename `EvaluateTimeLimitConfig_PAReopt` → `_PAReopt`** across
  `OptimizeSP_TL_Incre.{h,cpp}`, `Parameters.h`, `tests/testIncreOpt_w_TL.cpp`,
  `tests/testScheduleSimulate.cpp`. The name `GlobalBeam` is wrong — the fn does
  both `OptimizeFromScratch` (from_scratch=true) AND `OptimizeIncre` warm-start
  (from_scratch=false); it re-optimizes the PA. Parallels `_SingleTaskPatch`.
- [x] **0.5b. Build + test green** — `cmake --build build_test --target
  check.SP_OPT -j5 --clean-first` → 17/17 ctest green. Used `--clean-first`
  (stale-`.o` lesson from P1.21 after a header change).
- [ ] **0.5c. User review + commit** the rename as a standalone modular step
  BEFORE the behavior-changing merge (per "work by module, commit by module").

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

> Only AFTER the Phase 5 A/B accepts the merge.

- [ ] **2a. Delete `OptimizeSingleTaskTimeLimit`** (reopt-only legacy full-beam
  walk wrapper). `EvaluateTimeLimitConfig_ScratchOrIncre` STAYS — it is still
  the one upfront baseline re-opt step in reopt mode.
- [ ] **2b. Delete the `ReoptimizationUseSubIncrementalWalk` flag** + its
  `Parameters.h`/`parameters.yaml` entries + the 2 P2.9 `CounterDispatcherSynthetic`
  tests' flag-dispatch assertions (keep the `subincremental_calls` counter — it
  now asserts reopt ALWAYS hits the sub-incremental arm).
- [ ] **2c. Update comments** in `Parameters.h` + `OptimizeSP_TL_Incre.h` (the
  flag comment repointed in P2.10 Phase 3b now points at the unified body).

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

- [ ] **4a. Build** `cmake --build build_test --target check.SP_OPT -j5 --clean-first`.
- [ ] **4b. 17/17 ctest green** + `testIncreOpt_w_TL` green (updated P2.9 tests +
  new Type-E test).

## Phase 5 — A/B experiment (the gate; NOT bit-identity)

- [~] **5a. Run `INCR_Reopt_1` + `INCR_Reopt_10` at N=16**: current-default-reopt
  (P2.10 baseline, flag OFF) vs new-merged-reopt. Compare SP + per-activation ET.
  **BLOCKED 2026-07-25** — flag=0 baseline ran fine; flag=1 (merged) CRASHED at
  `taskset_0/INCR_Reopt_1` with SIGABRT (uncaught `runtime_error` from
  `RTACache::ComputeTaskSetDifference`, `|diff|>1`). Root cause: the reopt
  flag-on arm adopts the champion TL from the from-scratch reopt
  (`PerformCoordinateDescentForTaskConfigOpt:556`) but walks the un-re-synced
  Gaussian `starting_time_limits` (`:566`), so the first walk step diffs
  champion-TL vs Gaussian in >1 task → throw. Fix = re-sync
  `starting_time_limits = ReconstructTimeLimitVecFromResOpt()` before the walk
  (mirrors the incremental path). See dev_log 2026-07-25 Phase 5 crash entry.
  Needs a TDD regression test (RED) + the one-line fix (GREEN) before re-running.

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
- [ ] **5.6c. Validate** — single fast reproducer: one taskset / `INCR_Reopt_1`
  / flag=1 that previously ran 5 min → completion in ~30s = budget respected =
  fixed. No new A/B, no TDD arc (per "speed up dev, skip TDD temporarily").

## Phase 5.7 — SEPARATE ISSUE: flag=0 (prod) per-interval opt also slow at N=16

> Reported by the user 2026-07-25 AFTER the flag=1 diagnosis. DISTINCT from 5.6:
> affects the DEFAULT (flag=0) path = prod. ~5× per-interval regression
> (under-0.1s → ~0.5s) with NO config/flag change. NOT a TIME_LIMIT escape
> (0.5s < 1s budget) — a raw per-eval cost regression. Recorded separately so it
> is not conflated with the flag=1 fix. See dev_log 2026-07-25 SEPARATE ISSUE.

- [ ] **5.7a. Identify a known-fast ref** — find a commit the user recalls as
  "under 0.1s" at N=16 flag=0 (likely pre-P2.9/P2.10/P2.11, before `5dfd146e`).
  Ask the user / check the dev_log for the last timed-fast run.
- [ ] **5.7b. Bisect** — `git bisect` from the known-fast ref to HEAD, timing
  one N=16 flag=0 interval per step. Pin the regressing commit.
- [ ] **5.7c. Profile (if bisect inconclusive)** — scoped timer around
  `PerformSerializedTaskQueueOptimization` + `EvaluateTimeLimitConfig_SubIncremental`
  (eval count × per-eval time) to split MORE-evals vs SLOWER-per-eval. Prime
  suspects: P1.25 eager `RTACache` full-copy per eval (`:185`/`:234`); per-eval
  double DAG rebuild (`:191`+`:203`); P1.18 Rule B `ClassifyReusePerTask`
  over-conservative → more NoReuse recompute; P2.11 Phase 1a/3a added per-call
  overhead in the shared walk.
- [ ] **5.7d. Fix + validate** — address the root cause; confirm N=16 flag=0
  per-interval opt returns to ~0.1s. DEFERRED until after the 5.6 flag=1 fix
  lands (don't conflate the two).

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
- [ ] **5b. Compare against pure incremental (`INCR_Reopt_∞`/no reopt)** to test
  the user's hypothesis (merged reopt ≈ current frequent reopt, sometimes worse
  than incremental).
- [ ] **5c. Verdict** — accept the merge if SP is acceptable vs current reopt AND
  the structural simplification lands; else fall back to reading (b) or revert.

## Phase 6 — Closeout

- [ ] **6a. Update `agents/overall_tasks.md`** with P2.11.
- [ ] **6b. Add memory** `p211-merge-reopt-into-incremental.md` + one-line pointer
  in `MEMORY.md`.
- [ ] **6c. Update P2.9 + P2.10 records** — P2.9's flag is deleted by this task;
  P2.10's shared arm is the foundation. Cross-link.

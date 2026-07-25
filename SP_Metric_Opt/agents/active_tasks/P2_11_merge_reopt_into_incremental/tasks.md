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

- [ ] **5a. Run `INCR_Reopt_1` + `INCR_Reopt_10` at N=16**: current-default-reopt
  (P2.10 baseline, flag OFF) vs new-merged-reopt. Compare SP + per-activation ET.
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

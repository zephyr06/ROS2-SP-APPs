# P1.4 — Tasks (working checklist)

> See `goal.md` for the finding, the reasoning, and the measured tradeoff.
> One review-and-commit cycle per `agent_coding_rules.md`. TDD: tests first.
> **Choice (b) adopted** (over the initially-implemented (a)) at the user's
> direction: "always use reopt-start-from-adopted-tl, remove the flag, clean
> related choice code and unused code."

## Step 1 — Record findings + reasoning (DONE)
- [x] Confirm `InitializeTimeLimitsFromETConfig` is YAML-derived
      (`et_dist_` from `RegularTasks.cpp:61-106`; `GetAvgValue()` resolves to a
      Gaussian-mean-tied value; generator writes `mu` independent of the option
      grid — `taskset_generator.py:244-245` / `orchestrator.py:309`).
- [x] Confirm `ReconstructTimeLimitsFromResOpt` is algorithm-derived
      (`res_opt_`, the optimizer's own `CommitIncumbent` write).
- [x] Record the P1.3 A/B tradeoff: incumbent seed 0.3–0.8 SP pts below the
      YAML seed at high reopt frequency, →0 at P60.
- [x] Write `goal.md`.

## Step 2 — TDD: tests first (`tests/testIncreOpt_w_TL.cpp:1199+`) (DONE)
- [x] `ReOptimizePeriodic_StartsFromAdoptedTLByDefault` → renamed to
      `ReOptimizePeriodic_StartsFromAdoptedTL`; dropped the
      `ASSERT_TRUE(ReoptStartFromAdoptedTL)` (the flag is removed); keeps the
      incumbent-start assertion on interval 1+.
- [x] **DELETED** `ReOptimizePeriodic_OptOutStartsFromGaussianMeanTL` — the
      `false`/opt-out path no longer exists (the knob is gone).
- [x] `ReOptimizePeriodic_Interval0FallsBackToGaussianMean` — stripped the
      flag mention in the body comment; logic unchanged (interval-0 fallback
      is `IfInitialized()`-gated, flag-independent).
- [x] Net `testIncreOpt_w_TL`: 49 → 48 tests.

## Step 3 — Remove the knob (choice (b)) (DONE)
- [x] `sources/Utils/Parameters.cpp` — removed the `ReoptStartFromAdoptedTL`
      definition + its comment block.
- [x] `sources/Utils/Parameters.h` — removed the `extern bool` + its comment
      block.
- [x] `sources/parameters.yaml` — removed the `ReoptStartFromAdoptedTL` note
      block.

## Step 4 — Unconditional seed + `_ADOPTED` cleanup (DONE)
- [x] `sources/Optimization/OptimizeSP_TL_Incre.cpp:478` — the flag ternary
      became `IfInitialized() ? ReconstructTimeLimitsFromResOpt() :
      InitializeTimeLimitsFromETConfig()`; comment states the policy is
      permanent + unconditional.
- [x] `sources/Optimization/OptimizeSP_TL_Incre.h:132` — `starting_time_limits`
      origin comment updated (uniform incumbent seed for both paths; Gaussian-
      mean TL is the interval-0/INCR_SCRATCH fallback only).
- [x] `tests/RunOrchestrator.cpp` `MaybeOverrideReoptPeriod` — dropped the
      `_ADOPTED` suffix handling; a trailing suffix after the digits is now a
      HARD ERROR (loud stderr, no silent fall-through — the P1.3 trap). Header
      comment + `--help` Usage line updated.
- [x] `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp`
      `IsINCRPeriodVariant` — simplified to `INCR_P<n>` with no trailing
      suffix; history comment added.
- [x] `simulation_experiments/configs/p25_period_ab_config.json` — removed the
      4 `_ADOPTED` arms from both `test_mode` and `prod_mode`
      `main_scheduler_list` (10→6 arms each); rewrote `_comment` + both mode
      `_comment`s.

## Step 5 — Build + test + index (DONE)
- [x] `cmake --build build --target check.SP_OPT -j5` (DEBUG) — 16/16 green;
      `testIncreOpt_w_TL` 48/48 green.
- [x] `agents/overall_tasks.md` — P1.4 row updated to reflect choice (b)
      (knob + arms removed, not just defaulted).
- [x] Top-level `agents/dev_log.md` — P1.4 (b) milestone appended.
- [x] Memory `reopt-tl-init-adopted-arms.md` — P1.4 (b) UPDATE appended.
- [x] `git add` the P1.4 (b) unit; unrelated working-tree changes left
      unstaged. Handed to user for review (no commit).

## Standing constraints
- No `git commit` (user's task; `git add` only).
- No running the A/B suite myself (user runs `run_end_to_end.sh`).
- The incumbent seed is now the ONLY reopt seed (choice (b) — the flag and the
  `_ADOPTED` arms are gone); the `IfInitialized()` gate still handles interval
  0 / INCR_SCRATCH correctly (the irreducible fallback).

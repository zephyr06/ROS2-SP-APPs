# P3.6 — Tasks (working checklist)

> See `goal.md` for the finding + design decisions. TDD: tests first. One
> review-and-commit cycle per `agent_coding_rules.md`.

## Step 0 — Design decisions (SETTLED 2026-07-26)

- [x] D1 — Mode name = `INCR_NO_REOPT`.
- [x] D2 — Dedicated `scheduler_mode_` branch.
- [x] D3 — `OptimizePureIncremental` entry point + `BootstrapIncumbentFromRMFast`
      seed-only helper.
- [x] D4 — Config = `paper_simulation_config.json` (both modes).
- [x] D5 — Paper-grade baseline, NOT an E3 gate.

## Step 1 — TDD: tests first (DONE)

- [x] `tests/testIncreOpt_w_TL.cpp`: `OptimizePureIncremental_Interval0IsSeedOnly`
      — interval 0 produces the RM-fast incumbent with **zero** descent evals
      (`eval_count_` stays 0; `opt_sp_`/`opt_pa_`/TLs match the RM-fast primitives).
- [x] `OptimizePureIncremental_AdvancesCounterOncePerCall` — counter advances
      once per call (uniform with `Optimize_w_TL_ScratchOrIncre`).
- [x] `OptimizePureIncremental_NeverReoptsEvenAtPeriodOne` — with
      `ReoptimizationPeriod=1` (would force reopt every interval under the reopt
      dispatcher), interval 0 has STRICTLY FEWER evals than the reopt arm's
      interval-0 (skips the from-scratch beam); interval 1+ takes the incremental
      path (evals grow, incumbent carried).
- [x] Tests FAIL on current code first (red = compile error, no such method),
      then PASS after Step 2 (green).

## Step 2 — Wire the arm (DONE)

- [x] `sources/Optimization/OptimizeSP_TL_Incre.h` — declared
      `OptimizePureIncremental` + `BootstrapIncumbentFromRMFast`.
- [x] `sources/Optimization/OptimizeSP_TL_Incre.cpp` — implemented:
      `BootstrapIncumbentFromRMFast` = set `dag_tasks_` +
      `ApplyWCETAblationIfRequired` +
      `time_limit_option_for_each_task_=RecordTimeLimitOptions(dag_tasks_)` +
      `ResetIncumbentBaseline(true)` (NO descent). `OptimizePureIncremental` =
      `count==0` → bootstrap, else `OptimizeIncre_w_TL`; advance
      `reoptimization_interval_count_`; return `opt_pa_`. Wrapped in
      `BFDLSharedBudget` (same per-interval budget rationale as
      `Optimize_w_TL_ScratchOrIncre`).
- [x] `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp` — added
      `INCR_NO_REOPT` to the pre-loop construction condition + a dispatch branch
      calling `incr_optimizer_.OptimizePureIncremental(...)`.
- [~] `tests/RunOrchestrator.cpp` — N/A: that source file no longer exists
      (legacy binary only; mode strings are config-driven, validated in the
      orchestrator dispatch). tasks.md item was stale.

## Step 3 — Config + docs (DONE)

- [x] `simulation_experiments/configs/paper_simulation_config.json` — added
      `INCR_NO_REOPT` to `ablation_scheduler_list` (test_mode + prod_mode); updated
      the prod_mode `_comment`. (Placed in ablation, NOT main, so the main figure's
      5 production arms are unchanged and the contrast lands in the ablation
      figure where "does reopt earn its cost?" belongs.)
- [x] In-source comments at the new method + branch (paper-grade wording).
- [x] Memory file + `MEMORY.md` pointer (DONE — `p36-incr-no-reopt-baseline.md`
      + MEMORY.md "Deferred (P2/P3)" pointer).

## Step 4 — Build + test + index (DONE)

- [x] `cmake --build build_test --target check.SP_OPT -j5` → 17/17 ctest green
      (re-verified 2026-07-26 resumption: 16.20s); SP bit-identical for existing
      arms (additive change; existing `Optimize_w_TL_ScratchOrIncre` untouched).
- [~] `agents/overall_tasks.md` P3.6 row — already present at `:30` but LABELED
      `P1.6` (label mismatch, not a missing row). NOT FIXED: that file is
      concurrently edited by another agent (P2.9–P2.12 row expansion is their
      unstaged work); editing concurrently risks a conflict. Flagged for the
      user / parallel agent to reconcile the `P1.6`→`P3.6` label.
- [x] Top-level `agents/dev_log.md` — appended the P3.6 milestone (2026-07-26).
- [x] `git add` the P3.6 unit; hand to user for review (no commit).

## Standing constraints

- No `git commit` (`git add` only). No running the A/B myself. Inherits P1.4's
  seed unchanged. Paper-grade A/B baseline, NOT an E3 gate. Dispatch shape = a
  dedicated `scheduler_mode_` branch; optimizer lifetime = persistent (like
  `INCR`), NOT fresh-each-interval.

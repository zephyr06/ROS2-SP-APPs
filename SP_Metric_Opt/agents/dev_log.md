# Development Log

> **Archived Historical Log**: Complete detailed chronological development records up to 2026-07-25 are archived in [`agents/finished_tasks/dev_log_2026-07-25.md`](finished_tasks/dev_log_2026-07-25.md).

---

## Historical Milestones & Summary (2026-06-20 – 2026-07-24)

- **Pipeline Foundation & Optimization (2026-06-20 – 2026-07-01)**
  - Initial pipeline completion, `opt_sp_` initialization fix, linear coordinate descent for task configuration optimization.
  - Sourced library `scripts/lib/common.sh`, unified simulation runners into `run_simulation.sh`, per-run figure namespacing, line plots, PNG+PDF exports.
  - **P12 SP Normalization**: Normalized SP by theoretical maximum ceiling ($\Sigma \text{sp\_weight} = 5.0$).
  - **P10**: Unified End-to-End Orchestrator.

- **Multi-Core & Period Optimization (2026-07-02 – 2026-07-04)**
  - **P13 / P14 / P19**: Per-core CPU utilization fix, task-count ceiling removal, unified period pool & random CPU utilization range.
  - **P24 / P25**: Periodic reoptimization & counter-driven dispatcher with compare-and-keep.
  - **P25 Fix A & Fix B**: Baseline advancement fix (`OptimizeIncre`) + incumbent state preservation (`SeedStateFromIncumbent`) TDD-verified green.

- **Incumbent Redesign & Algorithm Hygiene (2026-07-05 – 2026-07-13)**
  - **P0.5 Incumbent Redesign (DONE)**: Owned once in `res_opt_` (removed `prev_optimizer_`), `CommitIncumbent` / `BuildChallengerFromIncumbent` helpers.
  - **P1.1 Efficiency Optimizations**: Single-point convolve fast path (`ConvolveSinglePoint`), ~3× speedup per convolve. Closed 2026-07-24 as runtime adequate.
  - **P2.3 / P0.1**: `FiniteDist::approx_equal` cleanup and test wiring.

- **Serialized Optimization & RTA Cache Evolution (2026-07-17 – 2026-07-24)**
  - **P1.10 Serialized Optimization**: Phase 1 `OptimizeIncre_SingleTask` extraction LANDED & COMMITTED. `FindEnvTaskWithDifferentEt` structural filter landed.
  - **P1.11 / P1.12 / P1.13 RTA Cache Integration**: `RTACache` single-champion cache introduced & wired into sub-incremental evaluation.
  - **P1.14 / P1.15 / P1.16 / P1.25 Cache Resilience & Budget**:
    - P1.14: `BFDLSharedBudget` whole-call time limit enforcement for BF/INCR.
    - P1.15: Python harness loud-failure detection & crash reporting.
    - P1.16 & P1.25: Eager backup/restore on rejected sub-incremental walks, removing transient transaction overhead.
  - **P1.18 ClassifyReusePerTask**: Fine-grained per-task reuse (Rule A & Rule B) DONE & CLOSED (2026-07-24).
  - **P1.19 `--rerun_mode`**: `clear_all` & `clear_results` options implemented and verified (2026-07-24).
  - **P1.20 Processor Map Vectorization**: Processor-to-task-set maps converted from hash table to O(1) vectors DONE (2026-07-23).

---

## Active & Recent Development Logs

### 2026-07-24 — P1.18 ClassifyReusePerTask Fine-Grained Reuse (DONE)
- **Summary**: Upgraded `RTACache::ClassifyReusePerTask` to perform fine-grained per-task reuse instead of whole-core fallback.
- **Rules Implemented**:
  - **Rule A** (`has_et_diff == true`, Task ET Changed): `pos < p_min → FullReuse`, `pos >= p_min → NoReuse`.
  - **Rule B** (`has_et_diff == false`, Pure Priority Move): `pos < p_min → FullReuse`, `[p_min, p_max] → NoReuse`, `pos > p_max → FullReuse`.
- **Verification**: Monotonic safety bound preserved. 17/17 `ctest` + 63/63 `testRTA` green. Committed (`09d1fca9`). Full record in `agents/finished_tasks/P1_18_classify_reuse_per_task_more_types/`.

### 2026-07-24 — P2.8 Scripts and Configs Refactor (In Progress)
- **Summary**: Streamlined config hierarchy and entry-point scripts.
- **Key Changes**:
  - Consolidated JSON configs into `paper_simulation_config.json` (deleted `gate_eval_config.json` and redundant test configs).
  - Renamed scripts: `run_end_to_end.sh` → `run_simulation_and_plot_figures.sh`, `run_evaluation_suite.sh` → `run_simulation_plot_eval_ns.sh`.
  - D6: Removed double-build block from eval script (delegates to pipeline).
- **Verification**: 43/43 Python tests green. `DRY_RUN=1` verified.

### 2026-07-25 — P2.13 Important-Task Miss Rate vs SP Metric Study (FILED)
- **Summary**: Investigated discrepancy where `Important_Miss_Rate` swings (0.48 → 0.567) while `Mean_SP_Metric` stays flat (~0.578) on N=10 `INCR_Reopt_1`/`_10`.
- **Findings**:
  1. Analytic RTA vs empirical job-history miss count measure different signals.
  2. `SP_Func` saturation near miss probability 1.0 (large miss rate gap moves SP by ~0.008).
  3. Weight dilution (1 important task @ 0.133 of sum 1.0; 9 low-miss tasks dominate).
  4. 100% miss in both arms on the swing task yields zero SP differential.
- **Action**: Perform feasibility check (DDL vs WCET) on existing run output.

### 2026-07-25 — P2.14 & P2.15 Schema Harmonization & Weight Randomization
- **P2.14**: Removed `SP_THRESHOLDS_SET` from generator schema; standardized on continuous `SP_THRESHOLD_RANGE: [0.001, 0.9]`.
- **P2.15**: Replaced hardcoded 2:1 weight split (`sp_weight_base` 2.0 / 1.0) with continuous `SP_WEIGHT_RANGE: [0.1, 1.0]` sampling while preserving `SP_WEIGHTS_SUM` normalization. 372/372 Python tests green.

### 2026-07-26 — P3.6 INCR_NO_REOPT Baseline (IMPL DONE — awaiting user review)
- **Summary**: New scheduler arm `INCR_NO_REOPT` — pure incremental with RM-fast bootstrap at interval 0 (no from-scratch descent), `OptimizeIncre_w_TL` every interval after, NEVER `ReOptimizePeriodic`. Contrasts `INCR_Reopt_10` (production arm, reopts every 10th interval): does periodic reopt earn its cost? Paper-grade baseline, NOT an E3 gate.
- **Behavior**: interval 0 = seed incumbent from RM-fast (`RateMonotonicPriorityVec` + `SmallestTimeLimitVec` via `ResetIncumbentBaseline(true)`, NO descent); intervals 1+ = `OptimizeIncre_w_TL` (warm-started from carried incumbent); persistent `incr_optimizer_` (like `INCR`), never fresh-each-interval.
- **Code**: `OptimizePureIncremental(dag, beam)` + `BootstrapIncumbentFromRMFast(dag)` in `OptimizeSP_TL_Incre.{h,cpp}`; `INCR_NO_REOPT` construction condition + dispatch branch in `SimulationOrchestrator.cpp`; `INCR_NO_REOPT` in `ablation_scheduler_list` (test_mode + prod_mode) of `paper_simulation_config.json`.
- **TDD**: 3 tests in `testIncreOpt_w_TL.cpp` under `CompareAndKeepSynthetic` (`Interval0IsSeedOnly`, `AdvancesCounterOncePerCall`, `NeverReoptsEvenAtPeriodOne`). "No descent" pin = `eval_count_==0` after interval 0.
- **Verify**: `cmake --build build_test --target check.SP_OPT -j5` → 17/17 ctest green (16.20s). SP bit-identical for existing arms (additive change; `Optimize_w_TL_ScratchOrIncre` untouched).
- **Status**: NOT committed (`git add`-only, user's standing rule). Awaits user review + `git commit`, rebuild `release/`, re-run the A/B. Full record in `agents/active_tasks/P3_6_incr_only_baseline/`.

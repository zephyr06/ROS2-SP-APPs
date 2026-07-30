# Development Log

> **Archived Historical Log**: Complete detailed chronological development records up to 2026-07-25 are archived in [`agents/finished_tasks/dev_log_2026-07-25.md`](finished_tasks/dev_log_2026-07-25.md).

---

## 2026-07-29

- **P0.8 config-tuning round — make the gate pass within budget on the REAL
  paper config (working tree, `git add`-only — NOT committed, awaits review).**
  The committed gate (`3d2360ed`) is a correct loud-raise certifier, but the
  real config made it reject too often. Root cause was config + the perf-WCET
  rule, not the gate. Three fixes: (1) perf WCET → `execution_time_mu` (= et_mean)
  — faithful (sim runs `min(et_mean, TL)`, TL is a downward cap) + tightest
  sound; dropped `tl_grid_upper`/`cfgs` params + made the TL grid verdict-irrelevant;
  (2) env cap 0.45→0.27 + variance [0.5,0.6]→[0.3,0.4] → env WCET/period ≤0.486;
  (3) (3a) no-inflation: cpu_util [0.5,1.5]→[0.5,1.0] + DROP the proportional
  redistribution block (raises non-env `u_i` above drawn; inflates perf
  `execution_time_mu`); strictly safe. Plus `DEADLINE_MODE=implicit` (RM≡DM).
  Verify: `pytest Gen_Taskset/tests/` = 47 passed; faithful gate
  `measure_gate_rejection_rate --samples 2 --ns 4 8 16` → N=4/8 attempt 1,
  N=16 ≤3 attempts, 0 rejections/0 raises. Step 2 prod-wiring + Step 3 deferred.

- **P0.8 Step 2b refactor — de-duplicate path/config scaffolding + fix
  `dir_path=None` regression (user review).** The shell/body/gate split had
  triplicated `OPT_SP_PROJECT_PATH`, config-path resolution, cfgs-load, and
  dir-path resolution. The duplication also caused a regression: the original
  resolved `dir_path=None` → `TaskData/<cfg>_gen_1` in the body; after the
  split only the gate did, so the shell forwarded `None` to
  `_run_pipeline_with_cfgs` (which raises) — would have broken the canonical
  CLI's no-`--dir_path` invocation (no test caught it; all ~22 callers pass
  `dir_path=` explicitly). Fix = extract `_resolve_config_path` /
  `_load_and_validate_cfgs` / `_resolve_dir_path` helpers + hoist
  `OPT_SP_PROJECT_PATH` to a module constant; the shell resolves `dir_path`
  before the body (regression fixed), the gate shares the same helpers (no
  divergence). `pytest Gen_Taskset/tests/` = 47 passed; end-to-end
  `dir_path=None` run confirmed.

## 2026-07-28

- **P0.8 Step 2b — important-task gate wrapper LANDED + staged (NOT committed, awaits
  user review).** `run_full_generation_pipeline_with_important_task_gate` (`orchestrator.py`):
  a generation-time gate that certifies every emitted taskset is schedulable for the
  important tasks under DM-with-top-priority-lock at the seed point. Seed-advancing
  retry loop (budget 20, D4), loud `RuntimeError` on exhaustion (NEVER silent — prevents
  re-creating the P1.8 substrate). **Design:** split `run_full_generation_pipeline` into
  a thin shell + `_run_pipeline_with_cfgs(cfgs, ...)` (REQUIRED cfgs, no default args —
  user preference) because the shell reloads cfgs from the config file each call, which
  would discard an advanced seed (no-op-retry bug). **DRY refactor + key-normalization
  fix:** extracted `_load_emitted_tasks_by_gid` (normalizes emitted `important` → RTA's
  `is_important`; without it the gate would be hollow) + `_wcets_from_loaded_tasks`.
  **Tests:** 4 TDD red→green wrapper tests (mock pipeline, real RTA); `pytest Gen_Taskset/tests/`
  = 47 passed (+4, no regressions). Step 2 (prod wiring) + Step 3 (rejection-rate) deferred.

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

### 2026-07-28 — P0.9 System-wide DM + Important-First Priority Lock (LANDED — FULLY COMMITTED)
- **Summary**: Switched the priority-assignment **seed** system-wide from plain Rate Monotonic (period sort) to **Deadline Monotonic (deadline sort) + important-first group lock**, so the C++ scheduler's seed agrees with P0.8's Python RTA certification. Generator sets `deadline = period·U(0.5,1.0)` → constrained deadlines (`D<T`), where DM ≥ RM (optimal fixed-priority). Group lock = important tasks (top-50% by `sp_weight`, `bool Task::is_important`) occupy TOP slots, non-important fill the bottom → the important-group RTA is self-contained. Lockstep prerequisite for P0.6 + P0.8 (both certify/seed against this exact order). D1–D7 ALL RESOLVED (D4 rename RM→DM, D7 FULL system-wide incl. orchestrator arms, D6 accept behavior change — NOT bit-identical; verify "schedulability improves/holds + SP within tolerance"; prod A/B = user-go, NOT run unilaterally).
- **Step 1 — C++ seed PA (LANDED):** `RateMonotonicPriorityVec`→`DeadlineMonotonicPriorityVec` (group-locked DM sort: partition by `is_important`, sort each group deadline asc / ties avg ET asc, concat [important]++[non-important]); `SeedIncumbentFromRMFast`→`SeedIncumbentFromDMFast`, `BootstrapIncumbentFromRMFast`→`BootstrapIncumbentFromDMFast`. TDD red→green: 2 new discriminating tests (`RanksImportantGroupFirstThenDmOrdersEach`, `BreaksDeadlineTiesByExecutionTimeAscending`) + 3 call sites + test rename in `testIncreOpt_w_TL.cpp`. `check.SP_OPT` = 17/17.
- **Step 2 — Python RTA (LANDED):** `_important_priority_order` (`important_task_rta.py`) sort key `period`→`deadline`; docstrings RM→DM; discriminating test `test_dm_ordering_is_deadline_based_not_period_based`. `pytest Gen_Taskset/tests/` = 43 green.
- **Step 2 — Orchestrator baseline arms C++ (LANDED, D7):** `SimulationOrchestrator.cpp` arms `"RM"`/`"RM_FAST"`/`"RM_SLOW"`→`"DM"`/`"DM_FAST"`/`"DM_SLOW"` + each sort `period`→`deadline` (PLAIN deadline sort — NO group lock; group lock is seed-PA only); `RunOrchestrator.cpp` help/comments; `testScheduleSimulate.cpp` (mode strings + `/RM/`→`/DM/` export paths + test names). Fixture `deadline==period` → DM/RM agree → assertions unchanged. **Latent bug exposed+fixed:** `ExactResponseTimeValidation` parsed 7 cols (`+overrun`) but the P2.14 writer drops `is_overrun`→6 cols; test only passed via STALE June-21 `RM/` artifacts (never cleared dir, never set level 3). Fixed: `remove_all`+`EXPORT_DETAIL_LEVEL=3`+drop `overrun` from both parse sites. `check.SP_OPT` = 17/17.
- **Step 2 — Configs + Python cascade (LANDED, D7 full system-wide):** renamed `RM_FAST`→`DM_FAST`, `RM_SLOW`→`DM_SLOW`, bare `"RM"`→`"DM"` (the bare `"RM"` arm was already retired in configs/Python per `evaluation_suite.py:122` — "replaced by RM_FAST+RM_SLOW") across **18 source files**: 3 active JSON configs (`paper_simulation_config` test+prod + 2 prose, `compare_against_bf`, `incr_et_profiling`); 6 `simulation_experiments/` scripts; 6 debug scripts (DISTINCT copies in both `simulation_experiments/debug_analysis/` AND `tests/debug_analysis/`); `visualize_SP_distribution.py`; 5 `tests/python/` fixtures (synthetic `"RM"` labels + the config-pin assertion coupled to the renamed config + `ALL_SCHEDULERS` coupled to renamed `compare_optimizers.py`). **2 user-decided LEAVES:** `draw_trajectory_error.py:16` (`"RM"` = SLAM-trajectory data-file key `CameraTrajectory_rm.txt`, a real Jan-2025 artifact, NOT a scheduler dispatch) + `p211_reopt_ab_config.json` (retained historical record, prose-only, "do not delete"). STALE result outputs LEFT for prod A/B (`tests/{radius_comparison,comparison_runs,e2e_eval_runs,k_variation}/` + `simulation_experiments/optimizer_comparison/runs/`).
- **Step 4 — Records (LANDED this session):** relabeled P0.6 + P0.8 plan docs (`goal.md`+`tasks.md`) RM→DM system-wide via sed bulk — `AssignRMRespectingGroupOrder`→`AssignDMRespectingGroupOrder`, `RateMonotonicPriorityVec`→`DeadlineMonotonicPriorityVec`, seed point → "DM-grouped + min-TL + WCET", "RM-ordered"→"DM-ordered", "RM-with-top-lock"→"DM-with-top-lock". Residual RM check = clean. Historical `dev_log.md` entries in both folders left as point-in-time records (NOT rewritten — "don't falsify history"); each prepended with a dated 2026-07-28 P0.9-supersedence pointer.
- **Issue-2 review (2026-07-28):** another agent flagged a `TestGateQ3` mock-key mismatch (`Q3_BASELINES` has `DM_FAST`/`DM_SLOW` but the mock uses bare `(8,"DM")`). Verified via `git show HEAD:` this is **NOT a P0.9 regression** — HEAD already had the identical gap (`Q3_BASELINES=["RM_FAST","RM_SLOW",...]` + mock bare `(8,"RM")`); `evaluate_q3`/`_q3_at_n` iterate ONLY `Q3_BASELINES`, so the bare key was dead weight (never consumed) both before and after. The rename faithfully carried it forward (bare RM→bare DM). DM_FAST/DM_SLOW are "missing baselines" (noted, not fatal per the gate's design). Filed as optional P2 test-quality cleanup (populate the mock with real `DM_FAST`/`DM_SLOW` keys) — OUT of P0.9 scope. Issue-3 (`compare_against_bf.json time_limit_seconds=10` vs expected `1`) confirmed pre-existing (P2.14-known; my edit touched only `main_scheduler_list`).
- **Verify (Step 2 final):** `pytest Gen_Taskset/tests/` = 43 green; `pytest tests/python/` = 351/353 (2 fails PRE-EXISTING config — `compare_against_bf.json time_limit_seconds`, untouched by P0.9, = P2.14-known); `cmake --build build_test --target check.SP_OPT -j5` = 17/17 green.
- **Commit state:** Steps 1+2 COMMITTED by the user in 3 commits — `0b9dae4a` (Step 1 C++ seed PA: `OptimizeSP_TL_Incre.{h,cpp}` + `testIncreOpt_w_TL.cpp`), `6a35080b` (Step 2 C++ orchestrator: `SimulationOrchestrator.cpp` + `RunOrchestrator.cpp` + `testScheduleSimulate.cpp`), `352f13d5` (Step 2 configs/Python: 18 source files + the P0.9 task-folder `dev_log.md`/`tasks.md` snapshot). Step 4 records (P0.6/P0.8 plan-doc relabels + dev_log pointers + this milestone) = COMMITTED by the user as `7dd1a7ac` ("update task records"; 9 files, 1297/11; exactly the Step-4 record paths, no unrelated files).
- **Status**: Steps 1+2+4 ALL committed (`0b9dae4a`/`6a35080b`/`352f13d5`/`7dd1a7ac`). **D6 behavior-change flag**: seed PA shifts whenever `D≠T` (always) or the group lock reorders; schedulability should improve, global SP may move — the verification gate is "schedulability improves/holds + SP within tolerance", NOT bit-identical. Prod A/B re-run = user-go (NOT run unilaterally). Full record in `agents/active_tasks/P0_9_dm_and_important_first_priority/`.

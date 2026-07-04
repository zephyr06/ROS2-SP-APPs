# SP-Metric Optimization: Task List

## Completed (Pre-2026-07-01)

- [x] Fix `opt_sp_` initialization bug (0 → -1).
- [x] Refactor configuration optimization from exponential combination to linear iteration with task sorting.
- [x] Add Complete Fairness Scheduler (CFS) with tests.
- [x] Add Python simulated training data generator (`Gen_Taskset/`).
  - [x] Add python simulated environment comparing schedulers.
  - [x] Add pipeline to generate task sets from JSON config end-to-end.
- [x] Ablation baselines
  - [x] `INCR_NO_TL`
  - [x] `INCR_WCET`
- [x] GP-based ET distribution prediction.
- [x] C++ end-to-end task set execution scheduler (`RunOrchestrator`).
- [x] Additional baselines
  - [x] CFS via `SimulateCFSSched`
  - [x] Ablation switches (`disable_time_limit_opt`, `use_wcet_execution_time`)
  - [x] `RM_FAST` (shortest TL) and `RM_SLOW` (longest TL)
- [x] Rename interval YAML files to `taskset_characteristics_interval_{k}.yaml` for unambiguous glob matching; update all Python/C++ readers and tests.

---

## Completed (2026-07-01 Session)

### Configuration & Plotting Infrastructure

- [x] `simulation_experiments/configs/experiment_config.json` -- Single JSON config with `test_mode` / `prod_mode` + global `plotting` and `analysis` sections.
- [x] `simulation_experiments/experiment_config_loader.py` -- `load_experiment_config(mode)` returns flattened dict used by all scripts.
- [x] `simulation_experiments/plotting_config.py` -- `setup_publication_style()`, `get_scheduler_color_map()`, `save_figure()` with colorblind-safe palette, 14/16/18 pt fonts, PNG+PDF output.
- [x] `simulation_experiments/aggregate_across_tasks.py` -- Scans `optimizer_comparison/`, loads `comparison_summary.csv`, produces:
  - Fig 1A: Mean SP vs. # tasks (Main group: INCR, BF, RM, CFS)
  - Fig 1B: Std SP vs. # tasks (Main, debug)
  - Fig 1C: Mean execution time vs. # tasks (Main)
  - Fig 1D: Mean miss rate vs. # tasks (Main, debug)
  - Fig 1E: Std miss rate vs. # tasks (Main, debug)
  - Fig 1F: SP distribution box plot (single fixed task count, tunable)
  - Fig Ab-A: Mean SP vs. # tasks (Ablation group)
  - Fig Ab-B: Mean execution time vs. # tasks (Ablation group)

### Design Alignment (see `plan_alignment.md`)

The following decisions were finalized between the original Claude plan and Gemini review:

| Item | Decision |
|---|---|
| Ablation group composition | **BF + INCR + INCR_NO_TL + INCR_WCET + INCR_SCRATCH** (BF included as ceiling reference) |
| Important-task threshold | **Tunable** via `--important_task_pct` (default 0.10) |
| Parallel workers | **Tunable** via `--num_workers` (default `min(4, os.cpu_count())`) |
| Resume / crash recovery | **Tunable** via `--resume` flag; skips existing `interval_sp_metrics.txt` |
| Statistical annotation | Paired t-test code kept but **default OFF** until normality confirmed |
| Font sizes | 14 pt base / 16 pt labels / 18 pt titles |
| Output formats | PNG (300 dpi) + PDF (vector) for all figures |

---

## In Progress: Remaining Implementation

### P5 -- Important-Task Miss Rate (Figure 3)

- [x] Add `compute_miss_rate_by_task()` to `simulation_experiments/utils.py`
- [x] Modify `analyze_single_instance()` in `simulation_experiments/run_sim_experiments.py` to:
  - Load `taskset_characteristics_interval_0.yaml` to get `sp_weight` per task
  - Determine important tasks: top X% by `sp_weight` (rounded up, min 1)
  - Compute `important_miss_rate` and return it
- [x] Update `compare_optimizers.py` to collect `important_miss_rate`, add `Important_Miss_Rate` column to `comparison_summary.csv`
- [x] Generate Fig 3 (grouped bar chart of important-task miss rate by scheduler)
- [x] Generate Fig 3b (debug: non-important-task miss rate)

### P6 -- Trigger-Interval Sweep (Figure 2)

- [x] Create `simulation_experiments/interval_sweep.py`
- [x] For each interval in `[1, 5, 10, 20, 30, 60]` (prod) or `[5, 10]` (test):
  - Run `compare_optimizers.py` with that `scheduler_trigger_interval`
  - Collect results into `optimizer_comparison/interval_sweep/`
- [x] Plot Fig 2: X=interval, Y=mean SP, line for INCR, horizontal dashed ref lines for BF/RM

### P7 -- End-to-End Shell Scripts

- [x] `scripts/run_simulation.sh` -- core runner with env-var overrides
- [x] `scripts/run_sim_4_tasks.sh`
- [x] `scripts/run_sim_6_tasks.sh`
- [x] `scripts/run_sim_8_tasks.sh`
- [x] `scripts/run_all_experiments.sh` -- master script: 4/6/8 + aggregation
- [x] `scripts/run_paper_figures.sh` -- interval sweep + aggregate + all figures
- [x] `scripts/run_interval_sweep.sh`

Requirements:
- Verify `release/tests/RunOrchestrator` exists before launching
- Echo clear header with parameters
- Trap Ctrl-C to kill child processes cleanly
- Print START_TIME and estimated completion
- Accept env-var overrides (e.g., `NUM_TASKSETS=5 ./run_sim_6_tasks.sh`)

### P8 -- Tests

- [x] `tests/python/test_aggregate.py` -- mock CSV aggregation, verify mean/std and figure file creation
- [x] `tests/python/test_plotting.py` -- synthetic data, assert output PNG and PDF exist and are non-empty
- [x] `tests/python/test_run_sim_experiments.py` -- update with `compute_miss_rate_by_task` test

### P9 -- Integration Verification

- [x] All 6 figures (1A-1F, 2, 3) generation code present and verified with mock data
- [x] `comparison_summary.csv` includes `Important_Miss_Rate` column
- [x] Scripts have `+x` permission and pass `bash -n` syntax check
- [x] All new Python code has docstrings and no syntax errors
- [x] `tasks.md` updated to mark P5-P9 as `[x]` once verified

---

## P24/P25 — Periodic reoptimization with compare-and-keep (C++ INCR optimizer)

Branch `clean_simulation`. Design + commit breakdown in `agents/P24_task.md`.
Full narrative in `agents/dev_log.md`.

### Status (2026-07-04, HEAD `6cd0f24f`, Commit 6 staged)

- [x] **Commits 1–5 committed.** `ctest` 16/16 green; `testIncreOpt_w_TL` 23/23.
- [x] **Commit 6 — counter-driven dispatcher + radius rename (staged, NOT committed — `git add` only).**
      `testIncreOpt_w_TL` 26/26; full `ctest` 16/16.
- [ ] **Commit 7 — runtime A/B + dev_log before/after (NOT done).**

### Commit 1 — `radius` param plumbed end-to-end
- [x] `OptimizeIncre_w_TL(dag, K, radius)` overload; `RecordCloseTimeLimitOptions(dag, radius)` parameterized.
- [x] Old `OptimizeIncre_w_TL(dag, K)` forwards with `IncrementalTimeLimitSearchRadius` (corrected in Commit 6 — was mistakenly forwarding to `ReoptimizationTimeLimitSearchRadius` after the Commit 5 refactor).
- [x] Call sites in `tests/{testBF_w_TL,AnalyzePriorityAssignmentIncrementalExample}.cpp` pass radius explicitly.

### Commit 2 — Replace `timelimit2optimizer_` cache with `prev_optimizer_`
- [x] Single persistent `OptimizePA_Incre prev_optimizer_` (the incumbent), no per-TL cache.
- [x] Incremental path warm-starts from `prev_optimizer_` via `OptimizeIncre`; falls back to `OptimizeFromScratch(K)` when no incumbent.
- [x] `UpdateRecords` updates `prev_optimizer_` whenever a new optimum is saved.

### Commit 3 — `EvaluateTimeLimitConfig_ScratchOrIncre(..., bool from_scratch)`
- [x] Explicit `bool from_scratch` dispatch (was implicit via `prev_optimizer_.IfInitialized()`).
- [x] `from_scratch=true` → fresh `OptimizePA_Incre` + `OptimizeFromScratch(K)` (escapes PA drift).
- [x] `from_scratch=false` + incumbent → warm-start `OptimizeIncre`.
- [x] `PerformCoordinateDescentForTaskConfigOpt(..., bool from_scratch)` forwards the flag.

### Commit 4 — Rename `OptimizeFromScratch_w_TL(int K)` → `ReOptimizePeriodic(int K)`
- [x] Mechanical rename, zero behavior change.

### Commit 5 — `ReOptimizePeriodic(dag, K, radius)` compare-and-keep + 4-tuple refactor
- [x] Optimizer status modeled as 4-tuple `{dag, sp, pa, tl}` in `prev_optimizer_`.
- [x] `SeedIncumbentBaseline()` — re-eval incumbent under new DAG, or RM+min-TL at interval 0.
- [x] `SeedStateFromIncumbent(dag_with_tl, pa, sp, tl)` — seeds full 4-tuple into state.
- [x] `ReconstructTimeLimitVec()`, `RateMonotonicPriorityVec()` (period-asc, ET tiebreak), `SmallestTimeLimitVec()`.
- [x] `UpdateRecords`' compare guard (strictly-greater SP, tie-break lower TL-sum) IS compare-and-keep once `opt_sp_` holds the seeded baseline — no `restore_incumbent` block.
- [x] Tests: 2 end-to-end (`AdoptsWhenWideSearchWins`, `KeepsIncumbentWhenWideSearchLoses`) + 7 direct helper unit tests. `testIncreOpt_w_TL` 15→23.

### Commit 6 — Wire counter-driven dispatch in `Optimize_w_TL_ScratchOrIncre` (STAGED)
- [x] Add `Optimize_w_TL_ScratchOrIncre(dag_tasks_update, K)` — the one method the orchestrator calls per interval.
- [x] Owns `reoptimization_interval_count_` (member, init 0) + radius decision:
      `count % ReoptimizationPeriod == 0 ? ReoptimizationTimeLimitSearchRadius : IncrementalTimeLimitSearchRadius`.
- [x] Wide-radius branch delegates to `ReOptimizePeriodic(dag, K, wide_radius)` (compare-and-keep); narrow to `OptimizeIncre_w_TL(dag, K, narrow_radius)`.
- [x] `++count` after the decision. **Counter never resets** (per user: modular arithmetic alone decides; overflow accepted as non-issue for the sim env).
- [x] Orchestrator `DeterminePrioritiesAndBudgets`: INCR/INCR_NO_TL/INCR_WCET branches switch from `OptimizeIncre_w_TL` → `Optimize_w_TL_ScratchOrIncre` (`SimulationOrchestrator.cpp:267,281,289`). `INCR_SCRATCH` untouched.
- [x] Knobs `ReoptimizationPeriod` (default 10) + `ReoptimizationTimeLimitSearchRadius` (default 6) were **unread** — now live.
- [x] **Interval-0 bootstrap fix (latent bug):** at interval 0 `prev_optimizer_` is uninitialized, so the old INCR path threw (`CoutError` at `OptimizeSP_TL_Incre.cpp:153`). At `count==0`, `0 % period == 0` routes to `ReOptimizePeriodic` → `SeedIncumbentBaseline` synthesizes the RM+min-TL incumbent. Dispatcher doubles as the bootstrap.
- [x] **Radius-forwarding fix:** 2-arg `OptimizeIncre_w_TL(dag,K)` was forwarding to `ReoptimizationTimeLimitSearchRadius` (6) instead of `IncrementalTimeLimitSearchRadius` (2) — corrected. Restores the two-radius design (incremental=narrow 2, reopt=wide 6).
- [x] **`ReoptimizationPeriod == 0` dropped** (positive-only, min 1). The "0 disables" path would crash the interval-0 bootstrap. `parameters.yaml:12` comment update deferred to Commit 7.
- [x] Tests: 3 new under `CounterDispatcherSynthetic` (`CounterAdvancesEveryCall_NeverResets`, `TriggersReoptAtCountZero_BootstrapsIncumbent`, `RoutesToIncrementalAtNonModularCount`); `OptimizeWithOptimizationSpace` assertion updated for radius-2 window. `testIncreOpt_w_TL` 23→26.
- [x] **Radius-knob rename (mechanical):** `TimeLimitSearchRadiusIncr` → `IncrementalTimeLimitSearchRadius` (2, narrow); `ReoptimizationTimeLimitsSearchRadius` → `ReoptimizationTimeLimitSearchRadius` (6, wide; singularized). Prefix style matches `ReoptimizationPeriod`. 9 files: `parameters.yaml`, `Parameters.{h,cpp}`, `OptimizeSP_TL_Incre.cpp`, `tests/{testIncreOpt_w_TL,testBF_w_TL}.cpp`, `tests/debug_analysis/run_radius_comparison.py` (YAML key kept in lockstep), `agents/{tasks,P24_task}.md`. Historical session log `agents/claude_sessions/session_clean_simulation_2026-06-21.md` deliberately NOT rewritten. `ctest` 16/16 green after explicit test-binary rebuild.

### Commit 7 — Verify + docs
- [ ] Full `ctest` green.
- [ ] A/B re-run: `ReoptimizationPeriod=0` (off) vs `=10` (on), same taskset, compare SP + per-activation runtime. **Note:** `ReoptimizationPeriod == 0` semantics were dropped in Commit 6 (positive-only, min 1); A/B is now "period=1 (reopt every interval, wide radius) vs period=10 (default)". A period-0 baseline requires re-introducing the disable path or a separate knob — decide before running.
- [ ] `dev_log.md` before/after numbers.
- [x] Rewrite memory `p24-reoptimization-design.md` — done this session (was stale: described removed `restore_incumbent`/`SumTimeLimits`; said Commit 5 uncommitted).

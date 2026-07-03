# Plan: Publication-Quality Figures, Analysis, and End-to-End Pipeline

**Date:** 2026-07-01  
**Branch:** clean_simulation  
**Goal:** Produce paper-ready figures and a fully automated experiment pipeline.

> **Status (updated 2026-07-01, EOD):** Implemented & superseded by the session
> summaries in `dev_log.md`. This plan is retained as the original spec. Two
> deviations from the file/script lists below: (1) the three
> `run_sim_{4,6,8}_tasks.sh` wrappers were **not** kept — they were collapsed
> into `run_simulation.sh`'s positional task-count arg; (2) figures are written
> under `optimizer_comparison/runs/<run_id>/figures/` (per-run namespacing), not
> the flat `optimizer_comparison/figures/` shown here. See `tasks.md` for the
> current state.

---

## 1. Figure Summary

| ID | Figure | Audience | Group |
|---|---|---|---|
| **1A** | Mean SP Metric vs. Number of Tasks (4/6/8) | Paper | Main: INCR, BF, RM, CFS |
| **1B** | Std SP Metric vs. Number of Tasks | Debug | Main: INCR, BF, RM, CFS |
| **1C** | Mean Scheduler Execution Time vs. Number of Tasks | Paper | Main: INCR, BF, RM, CFS |
| **1D** | Mean Miss Rate vs. Number of Tasks | Debug | Main: INCR, BF, RM, CFS |
| **1E** | Std Miss Rate vs. Number of Tasks | Debug | Main: INCR, BF, RM, CFS |
| **1F** | SP Metric Distribution Box Plot (fixed task count) | Paper | Main: INCR, BF, RM, CFS |
| **Ab-A** | Mean SP Metric vs. Number of Tasks | Paper | Ablation: INCR, INCR_NO_TL, INCR_WCET, INCR_SCRATCH |
| **Ab-B** | Mean Execution Time vs. Number of Tasks | Paper | Ablation: INCR, INCR_NO_TL, INCR_WCET, INCR_SCRATCH |
| **2** | INCR SP vs. Optimizer Invocation Interval [1,5,10,20,30,60]s | Paper | INCR + BF/RM references |
| **3** | Miss Rate of Important Tasks (top 10% by sp_weight) | Paper | All schedulers |
| **3b** | Miss Rate of Non-Important Tasks | Debug | All schedulers |

---

## 2. Implementation Phases

### Phase 1 — Plotting Style Configuration

**New file:** `simulation_experiments/plotting_config.py`
- `setup_publication_style()` — sets rcParams (font sizes 14/16/18, colorblind palette)
- Color maps for Main group and Ablation group
- Constants: `DPI = 300`, figure sizes (`FIGSIZE_SINGLE = (8, 6)`, `FIGSIZE_GROUPED = (12, 6)`)
- Helper: `save_figure(fig, path_stem)` — saves both `.png` and `.pdf`

### Phase 2 — Cross-Task-Count Aggregation (Figs 1A–1F, Ab-A, Ab-B)

**New file:** `simulation_experiments/aggregate_across_tasks.py`

**Input:** `optimizer_comparison/tasks{N}_dur*_*_seed*/comparison_summary.csv` for N in {4,6,8}.

**Workflow:**
1. Glob all experiment directories matching `tasks*_dur*_interval*_seed*`.
2. Parse `num_tasks` from directory name (e.g., `tasks6_dur300_interval10_seed1000` → 6).
3. Load each `comparison_summary.csv`.
4. Build DataFrame:
   ```
   num_tasks | scheduler | mean_sp | std_sp | mean_miss | std_miss | mean_sched_time | std_sched_time
   ```
5. Group into **Main** schedulers and **Ablation** schedulers.
6. For each group, call plotting functions for 1A, 1B, 1C, 1D, 1E.
7. Fig 1F reads raw `interval_sp_metrics.txt` from one representative experiment directory (default the 6-task one) and produces a standalone box plot with publication style.

**Output:**
- `optimizer_comparison/figures/` (or `paper_figures/`)
- `fig1a_mean_sp_vs_tasks_main.{png,pdf}`
- `fig1b_std_sp_vs_tasks_main.{png,pdf}`
- `fig1c_mean_exec_time_vs_tasks_main.{png,pdf}`
- `fig1d_mean_miss_vs_tasks_main.{png,pdf}`
- `fig1e_std_miss_vs_tasks_main.{png,pdf}`
- `fig1f_sp_distribution_boxplot.{png,pdf}`
- `fig_ablation_mean_sp_vs_tasks.{png,pdf}`
- `fig_ablation_mean_exec_time_vs_tasks.{png,pdf}`

### Phase 3 — Trigger-Interval Sweep (Fig 2)

**New files:**
- `simulation_experiments/interval_sweep.py`
- `scripts/run_interval_sweep.sh`

**Parameters:**
- `--num_tasks` (default 6 for dev, tunable)
- `--intervals` (default `"1 5 10 20 30 60"`)
- `--n_tasksets` (default 10)
- `--n_sec` (default 300)

**Workflow:**
1. For each interval in the list:
   a. Run `compare_optimizers.py` with `--scheduler_trigger_interval {interval}`.
   b. Save results to `optimizer_comparison/interval_sweep/tasks{N}_interval{interval}/`.
2. Load each result directory's `comparison_summary.csv`.
3. Build DataFrame:
   ```
   interval | scheduler | mean_sp | std_sp
   ```
4. Plot Fig 2:
   - X = interval (seconds)
   - Y = mean SP
   - Line + markers for INCR
   - Horizontal dashed lines for BF and RM (averaged across intervals, since they don't change)
   - Error bars = std
5. Save as `{output_dir}/fig2_sp_vs_interval.{png,pdf}`.

### Phase 4 — Important-Task Miss Rate (Fig 3)

**Modify:** `simulation_experiments/utils.py`

**New function:**
```python
def compute_miss_rate_by_task(response_dir, task_deadlines) -> dict[int, float]:
    """Return per-task miss rate from response_times_task_*.txt files."""
```

**Modify:** `simulation_experiments/run_sim_experiments.py`

In `analyze_single_instance()`:
1. After computing aggregate `miss_rate`, also compute `miss_rate_by_task`.
2. Load `taskset_characteristics.yaml` from `taskset_dir` to get `sp_weight` per task.
3. Determine `important_tasks`:
   - Sort tasks by `sp_weight` descending.
   - Take `max(1, ceil(0.10 * n_tasks))` tasks.
4. Compute `important_miss_rate = mean(miss_rate_by_task[t] for t in important_tasks)`.
5. Return `important_miss_rate` as an additional value.

**Modify:** `simulation_experiments/compare_optimizers.py`

1. Collect `important_miss_rate` from `analyze_single_instance()` into `results_by_scheduler[scheduler]["important_miss_rates"]`.
2. Add `Important_Miss_Rate` column to `comparison_summary.csv`.
3. Generate Fig 3 (grouped bar chart: scheduler vs. mean important-task miss rate).
4. Generate Fig 3b (debug: non-important-task miss rate).

### Phase 5 — End-to-End Shell Scripts

**New files:**
- `scripts/run_simulation.sh`
- `scripts/run_sim_4_tasks.sh`
- `scripts/run_sim_6_tasks.sh`
- `scripts/run_sim_8_tasks.sh`
- `scripts/run_all_experiments.sh`
- `scripts/run_paper_figures.sh`
- `scripts/run_interval_sweep.sh`

All scripts must:
- Accept environment-variable overrides (e.g., `NUM_TASKSETS=5 ./run_sim_6_tasks.sh`).
- Verify `release/tests/RunOrchestrator` exists.
- Trap `INT`/`TERM` to kill child Python processes.
- Echo start time + parameters.

### Phase 6 — Tests

**New files:**
- `tests/python/test_aggregate.py` — mock CSV aggregation
- `tests/python/test_plotting.py` — synthetic data figure generation

**Update:**
- `tests/python/test_run_sim_experiments.py` — add `compute_miss_rate_by_task` test coverage

---

## 3. Design Decisions

| Decision | Value | Rationale |
|---|---|---|
| **Important tasks** | Top 10% by `sp_weight` (rounded up, min 1) | Captures tasks that most influence SP metric |
| **Interval sweep** | `[1, 5, 10, 20, 30, 60]` seconds | Includes fast adaptation (1s) and slow (60s); 10s is current default |
| **Dev task count** | 6 tasks (tunable) | Balances runtime (INCR ~0.02s) with complexity; 4 tasks is even faster for CI |
| **Color palette** | `seaborn.color_palette("colorblind")` | Accessible, distinguishable in B&W print |
| **Font sizes** | 14 pt base, 16 pt labels, 18 pt titles | Readable at single-column paper width |
| **Output format** | PNG (300 dpi) + PDF (vector) | PNG for web, PDF for LaTeX inclusion |
| **Statistical tests** | Paired t-test, default OFF | Keep annotation code ready; enable after confirming normality |
| **Group separation** | Main vs. Ablation in separate figures | Prevents visual clutter; ablation is a secondary analysis |
| **Fig 1F task count** | 6 tasks by default, tunable via CLI | 6 tasks is the primary paper configuration |

---

## 4. File Checklist

### New Files
- [ ] `simulation_experiments/plotting_config.py`
- [ ] `simulation_experiments/aggregate_across_tasks.py`
- [ ] `simulation_experiments/interval_sweep.py`
- [ ] `scripts/run_simulation.sh`
- [ ] `scripts/run_sim_4_tasks.sh`
- [ ] `scripts/run_sim_6_tasks.sh`
- [ ] `scripts/run_sim_8_tasks.sh`
- [ ] `scripts/run_all_experiments.sh`
- [ ] `scripts/run_paper_figures.sh`
- [ ] `scripts/run_interval_sweep.sh`
- [ ] `tests/python/test_aggregate.py`
- [ ] `tests/python/test_plotting.py`

### Modified Files
- [ ] `simulation_experiments/utils.py` (add `compute_miss_rate_by_task`)
- [ ] `simulation_experiments/run_sim_experiments.py` (important-task miss rate)
- [ ] `simulation_experiments/compare_optimizers.py` (Fig 3, pub style, interval sweep integration)
- [ ] `simulation_experiments/reanalyze_results.py` (pub style)
- [ ] `tests/python/test_run_sim_experiments.py` (new test cases)

---

## 5. Integration Verification

- [ ] All figures render without label clipping.
- [ ] `comparison_summary.csv` contains `Important_Miss_Rate` column after running any experiment.
- [ ] `./scripts/run_sim_6_tasks.sh --dry-run` validates arguments.
- [ ] `pytest tests/python/test_aggregate.py` passes.
- [ ] `pytest tests/python/test_plotting.py` passes.
- [ ] `pytest tests/python/test_run_sim_experiments.py` passes.
- [ ] `bash -n` passes on all `*.sh` scripts.

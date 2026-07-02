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

- [x] `scripts/run_simulation.sh` -- core runner with env-var overrides + optional positional task-count arg (`./run_simulation.sh 4`)
- [x] `scripts/lib/common.sh` -- sourced shared helpers (header/footer, binary guard, signal trap, PROJECT_ROOT)
- [x] ~~`scripts/run_sim_{4,6,8}_tasks.sh`~~ -- removed; collapsed into `run_simulation.sh`'s positional arg
- [x] `scripts/run_all_experiments.sh` -- master script: 4/6/8 + aggregation
- [x] `scripts/run_paper_figures.sh` -- interval sweep + aggregate + all figures
- [x] `scripts/run_interval_sweep.sh`

Requirements:
- Verify `release/tests/RunOrchestrator` exists before launching
- Echo clear header with parameters
- Trap Ctrl-C to kill child processes cleanly
- Print START_TIME and estimated completion
- Accept env-var overrides (e.g., `NUM_TASKSETS=5 ./scripts/run_simulation.sh 6`)

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

### P10 -- Unified End-to-End Orchestrator (Single Entry Point)

- [x] Create `simulation_experiments/run_end_to_end_experiments.py`
  - Load `configs/experiment_config.json` via `load_experiment_config(mode)`.
  - Step 1 (simulate): Run `compare_optimizers.py` for task counts in `num_tasks_for_cross_task_comparison`, passing the **union** of `main_scheduler_list` + `ablation_scheduler_list` so one pass feeds both main and ablation figures.
  - Step 2 (sweep): Run `interval_sweep.py` to sweep trigger intervals (Fig 2).
  - Step 3 (aggregate): Run `aggregate_across_tasks.py` to compile Figs 1A-1F, ablation figures, and 3.
  - All experiment parameters come from config; CLI only selects `--mode`, `--steps`, `--bin_dir`, `--verbose`, `--dry_run`.
  - `--steps` runs stages in fixed order (simulate → sweep → aggregate); a failed stage aborts the pipeline.
- [x] Create `scripts/run_end_to_end.sh`
  - Accepts `MODE` env-var (default "test"); also `STEPS`, `BIN_DIR`, `VERBOSE`, `DRY_RUN`, `PYTHON`.
  - Validates `release/tests/RunOrchestrator` before launching (skipped in dry-run).
  - Traps Ctrl-C / TERM to kill child processes cleanly.
  - Prints header + start/end time, calls `run_end_to_end_experiments.py`.
- [x] Fix: `aggregate_across_tasks.py` now skips `*_sweep_*` dirs so interval-sweep result directories (which also match `tasks(\d+)_`) don't pollute / overwrite the cross-task comparison bars in Figs 1A-1E.
- [x] Tests: `tests/python/test_run_end_to_end.py` (16 tests) — command construction per stage, scheduler-union dedup, `--dry_run`, `--steps` subset, stage-failure abort.

---

## Pending (2026-07-01)

### P11 -- Execution-Time Figure: Log-Scale Y Axis

- [x] In `aggregate_across_tasks.py`, switch the mean-execution-time figure (Fig 1C / Fig Ab-B) Y axis to **log scale** (e.g. `ax.set_yscale('log')`).
  - **Rationale:** Per-task and per-scheduler execution times differ by orders of magnitude (optimizer warm-up / search cost vs. fast baselines like RM/CFS), so a linear axis compresses the small values into an unreadable band at the bottom.
  - Keep PNG+PDF export and the line-plot style (`ax.errorbar`, no bars).
  - Verify label formatting is sensible on log scale (e.g. ScalarFormatter / minor ticks) so axis labels don't read as raw powers of 10.
  - Done: `build_line_chart(log_y=True)` added; Fig 1C + Ab-B pass it. `ScalarFormatter` (scientific off) + minor gridlines applied so labels read as plain numbers.

### P13 -- Constant per-core CPU utilization + remove task-count ceiling

**Two separate commits (docs-first, see `dev_log.md`):**

#### Commit A -- Fix: hold per-core CPU utilization constant at 0.9 (config-only)

`MEAN_CPU_UTIL` is **per-core** utilization (generator: `cpu_util = MEAN_CPU_UTIL
× N_CORES`, UUniFast distributes total across N tasks). It must stay **constant
across all N** — it represents the aggregated load state, independent of task
count. Existing `paper_{4,6,8}.json` violated this: they rose 0.9 → 1.2 → 1.6
(holding per-task ≈ 0.4 constant instead). Flatten to **0.9** per-core for all
(total = 0.9 × 2 = 1.8; system 90 % loaded / under-subscribed).

- [x] `taskset_cfg_paper_6.json`: `MEAN_CPU_UTIL` 1.2 → 0.9
- [x] `taskset_cfg_paper_8.json`: `MEAN_CPU_UTIL` 1.6 → 0.9
- [x] `taskset_cfg_paper_4.json` already 0.9 -- verify only, no change
- [x] `git diff` shows only the two `MEAN_CPU_UTIL` lines
- [x] `pytest tests/python -q` still passing (now **211** — grew with the
      P13/P16/P17-related test additions; `TestResolveTasksetConfigPath` among
      them)
- [x] Regenerate N=6 taskset, confirm `Σ execution_time/period ≈ 1.8` in
      `taskset_characteristics_interval_0.yaml` — verified: resolved config
      `taskset_cfg_paper_6.json` (`MEAN_CPU_UTIL=0.9, N_CORES=2`) realizes
      `cpu_util = 1.8` exactly via `generate_taskset_parameters`.
- [x] Stage the two config files only (`git add`); user commits.
- [x] **Acceptance:** old 4/6/8 run data (SP values in `dev_log.md`,
      `optimizer_comparison/` tasksets) now stale; 0.9 is tunable later as a
      separate edit.

#### Commit B -- Feat: remove the task-count ceiling (support arbitrary N)

The 4/6/8 ceiling is enforced in two places: (1) CLI `choices=[4,6,8]`, (2)
named-file lookup `taskset_cfg_paper_{N}.json` (only 4/6/8 exist) → prod
`experiment_config.json` declaring `[4,6,8,10]` crashed at the CLI for N=10
(the `gemini.md` finding). Remove both; synthesize configs dynamically so
10/12/14/16/18 work without pre-created files.

- [x] **B1** `generation_config_parser.py`: add `resolve_taskset_config_path(
      num_tasks, config_dir=None)` -- returns existing `paper_{N}.json` if
      present, else synthesizes a thin temp override file (INCLUDE base +
      `N_BIG=2`, `N_SMALL=N-2`, `MEAN_CPU_UTIL=0.9`, `N_CORES=2`, `RANDOM_SEED`
      =42). Reuses the existing INCLUDE-resolution path -- no generator changes.
- [x] **B2** `compare_optimizers.py:342-345` + `run_sim_experiments.py:311-314`:
      replace the hard-coded `f".../taskset_cfg_paper_{N}.json"` lookup with
      `resolve_taskset_config_path(args.num_tasks)`.
- [x] **B2 (latent bug fix)** `run_sim_experiments.py:373-374`: change raw
      `json.load` → `load_generation_config(config_file_abs)` so INCLUDE is
      resolved (consistent with `compare_optimizers.py:411`); required once
      configs are synthesized (they rely on INCLUDE).
- [x] **B3** `compare_optimizers.py:257` + `run_sim_experiments.py:260`: drop
      `choices=[4,6,8]`; add floor guard `if args.num_tasks < 2: parser.error`.
      Update `--help` text. `interval_sweep.py` unchanged (already `default=
      None`, gated downstream).
- [x] **B4** `experiment_config.json` prod_mode:
      `num_tasks_for_cross_task_comparison` `[4,6,8,10]` → `[4,6,8,10,12,14,16,
      18]`. Test mode stays `[4,6]`.
- [x] **B5** Tests: `test_generation_config_parser.py` add
      `TestResolveTasksetConfigPath` (existing 4/6/8 unchanged; N=10/14/18
      synthesizes valid config; N<2 rejected); `test_compare_optimizers.py` /
      `test_run_sim_experiments.py` add `--num_tasks 10` acceptance test.
- [x] `pytest tests/python -q` all passing (**211**).
- [x] Smoke: `--num_tasks 10` (and 18 if feasible) generates + simulates
      without CLI/config errors.
- [x] Stage; user commits.

**Out of scope:** tuning the 0.9 value; expanding the small-period pool
(duplicate periods OK at large N); regenerating/committing run data.

---

### P14 -- Random per-core CPU utilization range (per-taskset sampling)

**Goal:** During task-set generation, instead of a fixed `MEAN_CPU_UTIL`
constant, **for each task set** first randomly sample the per-core average CPU
utilization from a configurable range `[low, high]` (default `[0.5, 1.5]`),
then run the existing UUniFast distribution against the sampled total
(`cpu_util = sampled_per_core × N_CORES`).

**Rationale:** A single fixed utilization (P13's 0.9) only exercises one load
point. Sampling per-taskset across `[0.5, 1.5]` sweeps under-subscribed →
over-subscribed systems within one experiment run, giving a distribution of SP
behavior over load rather than a point estimate. Keeps P13's per-core
semantics (`MEAN_CPU_UTIL` is per-core, `× N_CORES` for total) — only the
*fixed value* becomes a *sampled range*.

**Design (to implement later — NOT started yet):**

- [ ] **P14.1** Generation config: add a new task-set generation config
      parameter controlling the sampling range. Tentative shape:
      - `CPU_UTIL_RANDOM_RANGE: [0.5, 1.5]` (per-core, inclusive) in
        `taskset_cfg_paper_base.json` (or the relevant base template).
      - When present, the generator samples `per_core_util ~
        Uniform(low, high)` **per task set** and uses it in place of a fixed
        `MEAN_CPU_UTIL`.
      - Decide interaction with existing `MEAN_CPU_UTIL`: either (a) the range
        **supersedes** the scalar when both are present, or (b) the scalar
        becomes the fallback when the range is absent. (Recommend (b) for
        backward compat — old configs without the range keep working.)
- [ ] **P14.2** Generator (`Gen_Taskset/lib/taskset_generator.py:241-242`):
      replace `cpu_util = cfgs['MEAN_CPU_UTIL'] * n_cores` with a branch —
      if the range config is present, sample per-core util from it first
      (using the taskset's RNG, seeded per-taskset so results are
      reproducible), then `cpu_util = sampled * n_cores`. Otherwise fall back
      to the existing fixed-`MEAN_CPU_UTIL` path.
- [ ] **P14.3** Reproducibility: the sampled value must be seeded by the
      per-taskset RNG (alongside `RANDOM_SEED`) so a given seed reproduces the
      same sampled util — record the realized per-core util + total
      `cpu_util` in the generated `taskset_characteristics_interval_0.yaml`
      (or equivalent output) so each task set's load is inspectable.
- [ ] **P14.4** Tests: `tests/python/test_taskset_generator.py` (or
      `test_generation_config_parser.py`) — assert that with the range config
      present, sampled per-core util ∈ [low, high] and total = sampled ×
      N_CORES; assert deterministic under a fixed seed; assert fallback to
      fixed `MEAN_CPU_UTIL` when the range is absent (P13 configs still
      produce 1.8 total).
- [ ] **P14.5** Relationship to P13: P13's "hold constant at 0.9" stays the
      default (no range config → 0.9). P14 is **opt-in** via the new range
      parameter. No removal of P13's configs. Update `dev_log.md` to record
      that P14 supersedes the "0.9 is itself tunable later" out-of-scope note
      in P13 — the tunable knob is now the range, not a scalar.

**Out of scope (for now):** implementation; deciding whether the range should
also vary with N (current design: same `[low, high]` for all N, per-core);
cross-N comparison implications (sampling changes SP distributions — may need
to re-examine P12 normalization under per-taskset load variance).

---

### P15 -- Randomize N_BIG / N_SMALL period-task split (per-taskset sampling)

**Goal:** During task-set generation, instead of a fixed
`N_BIG_PERIOD_TASKS` / `N_SMALL_PERIOD_TASKS` split from the config, **for
each task set** randomly decide how many of the N tasks are big-period vs
small-period (subject to floors), then run the existing period-picking and
UUniFast allocation against the resulting split.

**Rationale:** Today the split is pinned by the config (on-disk paper files
and the synthesized configs both hardcode `N_BIG_PERIOD_TASKS=2`,
`N_SMALL_PERIOD_TASKS=N-2`). Every task set of a given N therefore has the
same big/small composition. Randomizing the split per-taskset exercises a
wider region of the period-composition space within one experiment run,
giving a distribution of SP behavior over composition rather than a point
estimate. Sibling to P14 (which randomizes load); both are opt-in knobs that
leave the fixed defaults intact.

**Current behavior (verified):**
- `Gen_Taskset/lib/taskset_generator.py:246-248`:
  `g_n_big_periods = cfgs.get("N_BIG_PERIOD_TASKS", 2)`;
  `g_n_small_periods = cfgs.get("N_SMALL_PERIOD_TASKS", 8)`;
  `n_tasks = g_n_big_periods + g_n_small_periods`.
- Periods picked at lines 263-268: `g_n_big_periods` big + `g_n_small_periods`
  small via `pick_period(...)`.
- `N_TASKS` is derived in `standardize_config()` as `N_BIG + N_SMALL`; with
  the random split, the generator must treat `N_TASKS` as the input and
  sample the split from it (not the reverse).

**Design (to implement later — NOT started yet):**

- [ ] **P15.1** Generation config: add a task-set generation config parameter
      controlling the sampling. Tentative shape:
      - `N_BIG_PERIOD_TASKS_RANDOM_RANGE: [2, 4]` (inclusive) in
        `taskset_cfg_paper_base.json` (or relevant base template), OR a
        fraction-style knob like `N_BIG_PERIOD_TASKS_FRAC: [0.2, 0.5]` of N.
        Pick one shape (count-range is simpler and matches the existing
        integer `N_BIG_PERIOD_TASKS` semantics).
      - When present, the generator samples
        `n_big ~ UniformInt(low, high)` per task set (clamped so
        `low >= 2` — need at least two big-period tasks — and
        `high <= N-1` so at least one small-period task remains), then
        `n_small = N - n_big`.
      - Decide interaction with existing `N_BIG_PERIOD_TASKS` /
        `N_SMALL_PERIOD_TASKS`: recommend the range **supersedes** the fixed
        counts when present; fixed counts remain the fallback when the range
        is absent (backward compat — old configs keep working, P13 configs
        still produce the 2 / N-2 split).
- [ ] **P15.2** Generator (`Gen_Taskset/lib/taskset_generator.py:246-248`):
      replace the two `cfgs.get(...)` reads with a branch — if the range
      config is present, sample `n_big` from it (using the taskset's RNG,
      seeded per-taskset for reproducibility), set `n_small = N - n_big`;
      otherwise fall back to the existing fixed-count path. The downstream
      period-picking loop (263-268) and `uunifast_distribution(n_tasks, ...)`
      (274) then use the sampled split unchanged — `n_tasks` stays `N`.
- [ ] **P15.3** Reproducibility: the sampled `n_big` must be drawn from the
      per-taskset RNG (alongside `RANDOM_SEED`) so a given seed reproduces
      the same split. Record the realized `n_big` / `n_small` (and the
      sampled value) in the generated `taskset_characteristics_interval_0.yaml`
      (or equivalent output) so each task set's composition is inspectable —
      mirror P14.3's record-realized-value approach.
- [ ] **P15.4** Config-validation interaction: `standardize_config()` in
      `generation_config_parser.py` currently computes
      `N_TASKS = N_BIG_PERIOD_TASKS + N_SMALL_PERIOD_TASKS`. With the random
      split, `N_TASKS` must come from the caller (the CLI's `--num_tasks`,
      or the synthesized config's `N_BIG+N_SMALL`), and the split is sampled
      *at generation time*, not at config-load time. Ensure the resolver
      (`resolve_taskset_config_path`) and synthesized configs expose a stable
      `N_TASKS` that the generator samples *within*.
- [ ] **P15.5** Tests: `tests/python/test_taskset_generator.py` (or
      `test_generation_config_parser.py`) — assert that with the range config
      present, sampled `n_big ∈ [low, high]` and `n_big + n_small == N`;
      assert at least one big and one small period task per task set;
      assert deterministic under a fixed seed; assert fallback to the fixed
      2 / N-2 split when the range is absent (P13 configs unaffected).
- [ ] **P15.6** Relationship to P13/P14: P13's fixed `2 / N-2` split stays
      the default (no range config → 2 / N-2). P15 is **opt-in** via the new
      range parameter. Composes with P14 (load range) independently — a
      task set may sample both load and composition. Update `dev_log.md` to
      record P15 as a sibling randomization knob to P14.

**Out of scope (for now):** implementation; deciding whether the big/small
period *values* (not just counts) should also be randomized (current design:
counts randomized, period pool unchanged — duplicate periods OK at large N,
per P13); cross-N comparison implications under per-taskset composition
variance (may interact with P12 normalization).

---

### P12 -- SP-Metric Cross-Task Comparison: Investigate Trend + Normalization

- [x] **Investigate** whether mean SP increases as the number of tasks per task set increases.
  - **Hypothesis:** With more tasks, the SP upper bound (best achievable SP-weighted schedule) grows because there are more weighted deadlines to satisfy / more slack to redistribute, so mean SP should trend upward.
  - **Finding (documented in `dev_log.md`):** Hypothesis **refuted**. `SP_Func ∈ [0,1]`; the simulation exports the node-only term `Σ SP_Func × sp_weight × perf_coefficient`; `perf_coefficient=1.0` (no `timePerformancePairs`); and the generator normalizes `Σ sp_weight = SP_WEIGHTS_SUM = 5.0` for every task set regardless of N (`taskset_generator.py:475-480`). So the theoretical SP ceiling is **flat at ~5.0** across N — it does not grow. Empirically, the *fraction of ceiling achieved* **decreases** with N (N=4 → ~0.76, N=6 → ~0.59): more tasks = more contention = harder to meet all deadlines. This is the opposite of the hypothesis.
- [x] Add a **config option to normalize output SP metrics to the `[0, 1]` range**, applied per task-set size, so cross-N comparison is fair.
  - Done: `analysis.normalize_sp` (bool, default false) + `analysis.sp_normalization_method` in `experiment_config.json`.
  - **Definition of 1.0 (per user requirement):** normalized SP = `raw_SP / ideal_SP`, where `ideal_SP = Σ sp_weight × perf_coefficient` is the best possible with **infinite computation** — every task meets its deadline perfectly (`SP_Func = 1`) and the optimizer uses the best config. Because `SP_Func ∈ [0,1]` with non-negative weights, `raw_SP ≤ ideal_SP` always, so the ratio is guaranteed ∈ [0, 1] with 1.0 = "all deadlines met perfectly".
  - Default method **`upper_bound`** (ideal-SP divisor) — the only method that satisfies the requirement. `ideal_SP` is flat at ~5.0 across N (generator normalizes `Σ sp_weight = 5.0`), so the denominator is a *constant ceiling*, not a per-scheduler mean — no figure can exceed 1.0.
  - **Removed `reference_scheduler`** + `sp_normalization_reference`: dividing by BF (a realized value, not a ceiling) was the root cause of normalized values > 1.0 — INCR ties BF at N=4 (both 3.7972), and the Fig 1F boxplot divided each per-interval point by BF's *mean*, pushing whiskers to 1.30/1.11. BF's value is irrelevant to normalization. `minmax_per_task_count` kept as a non-default debug option (relative ranking, not fraction-of-ideal).
  - `compute_sp_upper_bound(experiment_dir)` reads `taskset_characteristics_interval_0.yaml` → `Σ sp_weight × perf_coefficient` (+ chain term if present); verified = 5.0 on the on-disk N=4/N=6 test data.
  - `normalize_records_sp(records, cfg, ...)` returns a **new** list (raw records untouched); degenerate/zero-ceiling cases fall back to raw.
  - Applied **consistently to every SP-metric figure** (same per-N `ideal_SP` divisor, computed from each figure's own data — no shared state): Fig 1A, Fig 1B (debug, reuses 1A's scaled records), Fig 1F (boxplot — divides each raw SP point by the ceiling, **not** BF's mean, fixing the >1.0 whiskers), Fig Ab-A, and Fig 2 (interval sweep — `_draw_figure_2` + `_normalize_sweep_data` helpers). Raw figures always kept alongside the normalized variant. Normalization is a reporting transform — raw CSV values unchanged.
  - Tests: `TestComputeSPUpperBound` (4) + `TestNormalizeRecordsSP` (8 — `upper_bound` default + minmax; `reference_scheduler` cases removed) + `TestBuildLineChart.test_log_y_*` (2) in `test_aggregate.py`; `test_interval_sweep.py` (6) covering Fig 2 normalization + `_normalize_sweep_data`. Full suite **193 passing**.

---

### P16 -- Remove MIN_PERIOD_WITH_PERFORMANCE_RECORDS Constraint

- [x] Remove the `MIN_PERIOD_WITH_PERFORMANCE_RECORDS` limitation from taskset generation so all tasks, regardless of their periods, are eligible to be chosen as time-limit (performance-record) tasks.
  - **Verified:** removed the period-floor gate (`if period < min_period_perf: continue`) in `taskset_generator.py` perf-record candidate selection; the legacy `MIN_PERIOD_WITH_PERFORMANCE_RECORDS` key is still accepted by `generation_config_parser.py` (default now `0`) as a documented no-op for external-config backward-compat. Removed the dead key from all shipped configs/templates that set it (`taskset_cfg_paper_base.json` — propagates to `paper_4/6/8.json` via INCLUDE; `test_standard_{4,6,8}.json`) and from tests that set it (`test_integration.py`, `test_taskset_generator.py` ×2). `test_integration.py` now asserts that short-period tasks are eligible perf-record tasks (expect exactly 2 soft tasks at `PERF_RECORD_TASK_PROBABILITY=1.0` with 2 non-env tasks, down from the old `1 <= len <= 2`). Full suite **225 passing**.

---

### P17 -- Relax/Remove N_BIG and N_SMALL Task Count Constraints

- [x] Relax or remove the constraints forcing a minimum number of big-period or small-period tasks (e.g. allowing `N_BIG = 0` or `N_SMALL = 0`) to support single-rate or arbitrary period-ratio taskset generations.

      **Verified (P17):** the generator (`taskset_generator.py:246-248`) never
      enforced a minimum — the period-pick loops are no-ops at count 0, and two
      pre-existing `test_coordinate_system.py` cases already ran `N_SMALL=0`
      end-to-end. The real constraints were upstream: `resolve_taskset_config_path`
      rejected `num_tasks < 2` and **hardcoded `N_BIG_PERIOD_TASKS=2`** in every
      synthesized config (so `N=1` was unreachable and `N_SMALL=n-2` would go
      negative), mirrored by two CLI guards (`compare_optimizers.py:345`,
      `run_sim_experiments.py:352`) with the rationale "needs >=1 big + >=1 small
      period task." Relaxations applied:
      - `resolve_taskset_config_path`: floor dropped `>= 2` → `>= 1`; synthesized
        split keeps `2 / (N-2)` for `N >= 2` (on-disk paper_4/6/8 and the
        cross-task sweep `[4,6,8,10,12,14,16,18]` unaffected) but emits
        `N_BIG=0, N_SMALL=1` for `N == 1` (minimal single-rate taskset the old
        hardcode could not represent). The N==1 synthesized config also drops to
        `N_CORES=1` so `cpu_util = 0.9` stays feasible for the lone task
        (util < 1.0); with `N_CORES=2` the single task would carry `cpu_util=
        1.8`, which `uunifast_distribution` can only realize as a single 1.8
        utilization — overloaded/unschedulable. N=1 is never on the cross-task
        sweep; this just keeps the corner case schedulable.
      - `standardize_config`: added explicit `N_BIG >= 0`, `N_SMALL >= 0`,
        `N_TASKS >= 1` validation (rejects a degenerate all-zero config with a
        clear message instead of a divide-by-zero in `uunifast_distribution`).
      - Both CLI guards relaxed `>= 2` → `>= 1`; help text updated.
      - Generator count-reads gained a P17 comment documenting 0-count support.
      Tests: rewrote the three `n < 2`-rejection tests to assert `N=1` accepted
      (routed to resolver) and `N=0` rejected; added `TestP17ZeroCountTasksets`
      (4 cases: all-small, all-big, single-task N=1, both-zero-rejected). Full
      suite **232 passing** (was 225).

---

### P18 -- INCR optimizer run-time speed (per-interval scheduling time > 0.1 s)

**Goal:** Investigate and resolve the INCR optimizer's slow per-invocation
run-time. Observed per-interval scheduling time exceeds **0.1 s** — too slow
for an optimizer meant to re-run every trigger interval (default 10 s) in the
loop, and visibly inflates INCR's bar on the execution-time figures relative
to the fast baselines (RM/CFS). Resolve later (not blocking current figure
work).

**Measurement path (verified):**
- The C++ orchestrator writes `scheduler_execution_time.txt` per scheduler
  output dir; Python reads it in `run_sim_experiments.py:114-143`
  (`total_exec_time` → `avg_sched_time = total_exec_time / num_intervals`).
- `compare_optimizers.py` collects `avg_sched_time` into
  `results_by_scheduler[s]["sched_times"]` (line ~546) and writes it to
  `comparison_summary.csv`.
- Surfaced in **Fig 1C** (mean exec time vs. # tasks) and **Fig Ab-B**
  (ablation group), both now on log-Y (`P11`). The log axis is what made the
  >0.1 s INCR latency readable in the first place.

**Likely hot spot (to profile, not assume):**
- `OptimizePA_Incre_with_TimeLimits::OptimizeIncre_w_TL(...)` is invoked
  **once per interval** via
  `FixedTaskPrioritySchedulingOrchestrator::DeterminePrioritiesAndBudgets`
  (`sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp:266-270`).
  The incremental search over
  `GlobalVariables::Layer_Node_During_Incremental_Optimization` is the prime
  suspect — candidate for redundant recomputation, unbounded layer/node
  expansion, or per-interval work that could be memoized / incrementalized
  across intervals instead of redone from the visited-state each tick.

**Out of scope (for now):** implementation; deciding whether the fix is
algorithmic (incrementalize/memoize the search) or implementation-level
(data-structure / pruning). First step when picked up: profile a single N=6
prod run to confirm where the >0.1 s is actually spent before changing
anything.


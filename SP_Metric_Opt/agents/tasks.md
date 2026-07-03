# SP-Metric Optimization: Task List

# RULE NUMBER 0:
- Read `agent_coding_rules.md` and follow it closely

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
  - All experiment parameters come from config; CLI only selects `--mode`, `--bin_dir`, `--verbose`, `--dry_run`. (P21: `--steps` was removed — the three stages are dependent and always run together in fixed order; a failed stage aborts the pipeline.)
- [x] Create `scripts/run_end_to_end.sh`
  - Accepts `MODE` env-var (default "test"); also `BIN_DIR`, `VERBOSE`, `DRY_RUN`, `PYTHON`. (P21: `STEPS` env-var removed for the same reason as `--steps`.)
  - Validates `release/tests/RunOrchestrator` before launching (skipped in dry-run).
  - Traps Ctrl-C / TERM to kill child processes cleanly.
  - Prints header + start/end time, calls `run_end_to_end_experiments.py`.
- [x] Fix: `aggregate_across_tasks.py` now skips `*_sweep_*` dirs so interval-sweep result directories (which also match `tasks(\d+)_`) don't pollute / overwrite the cross-task comparison bars in Figs 1A-1E.
- [x] P21: `aggregate_across_tasks.py` empty-data path is now actionable — prints the exact expected dir prefixes it looked for (none matched) and points at `./scripts/run_end_to_end.sh`, instead of the bare "Run simulations first." Aggregate stays read-only (does NOT auto-run simulate).
- [x] Tests: `tests/python/test_run_end_to_end.py` — command construction per stage, scheduler-union dedup, `--dry_run`, all-stages-always-run, `--steps`-rejected, stage-failure abort.

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

**Implemented (this change):**

- [x] **P14.1** Generation config: added `CPU_UTIL_RANDOM_RANGE: [low, high]`
      (per-core, inclusive) — validated in `standardize_config` (2-list of
      numbers, `0 <= low <= high`, coerced to floats; malformed →
      `ValueError`). **Opt-in only**: the key is NOT added to the base
      template, so absent the key the generator keeps P13's fixed-`MEAN_CPU_UTIL`
      path (0.9). Resolved the P14.1 open question in favor of **fallback**
      (option (b)) for backward compat — old configs without the range keep
      working; when the range is present it is used in place of the scalar.
- [x] **P14.2** Generator (`Gen_Taskset/lib/taskset_generator.py:~248`):
      replaced `cpu_util = cfgs['MEAN_CPU_UTIL'] * n_cores` with a branch —
      if the range config is present, `per_core_cpu_util =
      random.uniform(low, high)` (the first draw off the seeded RNG, so
      reproducible); else `per_core_cpu_util = cfgs['MEAN_CPU_UTIL']`. Then
      `cpu_util = per_core_cpu_util * n_cores` in both paths.
- [x] **P14.3** Reproducibility + inspection: the sampled value is the first
      draw off the seeded global RNG (`random.seed(RANDOM_SEED)` runs
      immediately before), so a given `RANDOM_SEED` reproduces the same
      sampled per-core util. The realized `per_core_cpu_util` is recorded in
      the returned dict and lands in `taskset_param.yaml` (top-level, alongside
      `cpu_util`) so each task set's load is inspectable. Existing YAML
      readers use `safe_load` + named-key access, so the new key is additive.
- [x] **P14.4** Tests: `test_taskset_generator.py::TestP14RandomCpuUtilRange`
      (5 — sampled util ∈ [low, high]; `cpu_util = per_core × N_CORES`;
      deterministic under fixed seed; varies across seeds; range-absent
      fallback to fixed 0.9) + `test_generation_config_parser.py` (+3 — valid
      range accepted/normalized; malformed rejected; absent by default). The
      test class snapshots/restores global RNG state so its seeded generations
      don't leak into later modules.
- [x] **P14.5** Relationship to P13: P13's "hold constant at 0.9" stays the
      default (no range config → 0.9). P14 is **opt-in** via the new range
      parameter. No removal of P13's configs. `dev_log.md` records that P14
      supersedes P13's "0.9 is itself tunable later" out-of-scope note — the
      tunable knob is now the range, not a scalar. Full suite **253 passing**
      (was 245; +8 P14 tests).

**Out of scope (for now):** deciding whether the range should also vary with N
(current design: same `[low, high]` for all N, per-core); cross-N comparison
implications (sampling changes SP distributions — may need to re-examine P12
normalization under per-taskset load variance); enabling the range on the
shipped paper configs (a one-line config edit when the load-sweep experiment
is wanted).

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

### P19 -- Unified period pool (PERIODS_MS + N_TASKS), drop big/small split

- [x] Collapse the paired big/small period schema into a single period pool +
  single task count, and remove the Hz/period-split keys from the config schema
  entirely.

      **Why:** the `BIG_PERIOD_HZ` / `SMALL_PERIOD_HZ` (+ `N_BIG_PERIOD_TASKS`
      / `N_SMALL_PERIOD_TASKS`) split was an artifact of an older Hz-based
      config. Periods are stored in ms everywhere they are used; the split only
      added a `pick_period` branching path, an ad-hoc <10 Hz / >=10 Hz split for
      the legacy single `HZ` list, and a fixed `N_BIG=2` hardcode that fought
      the P13/P17 task-count relaxation. One pool + one count is strictly
      simpler and removes the last big/small coupling.

      **Schema (canonical, the only accepted period/count inputs):**
      - `PERIODS_MS` (list[int]) — the period pool every task draws from, in ms.
      - `N_TASKS` (int) — total task count; every task draws one period from the
        pool. `pool exhaustion at large N allows duplicate periods (acceptable,
        per P13 note) — `pick_period` tries to avoid dups then falls back.

      **Two-phase landing:**
      - **Phase 1 (commit `a0cc8021`):** migrated source
        (`generation_config_parser.py`, `taskset_generator.py`), all shipped
        configs/templates (`paper_base`, `paper_{4,6,8}`, `test_standard_{4,6,8}`)
        and `debug_uunifast.py` to `PERIODS_MS` + `N_TASKS`. `standardize_config`
        kept a backward-compat alias so old-shape configs kept working: it built
        `PERIODS_MS` = big-pool periods + small-pool periods and `N_TASKS` =
        `N_BIG + N_SMALL`, then deleted the old keys. `pick_period` dropped its
        `prd_sel` arg (one pool, no branch). `resolve_taskset_config_path`
        synthesized configs set `N_TASKS` (N=1 still drops to `N_CORES=1`).
      - **Phase 2 (this change):** removed the backward-compat alias. Legacy
        period keys (`HZ`, `SMALL_PERIOD_HZ`, `BIG_PERIOD_HZ`,
        `SMALL_PERIODS_MS`, `BIG_PERIODS_MS`) and count keys
        (`N_BIG_PERIOD_TASKS`, `N_SMALL_PERIOD_TASKS`) now raise `ValueError`
        pointing at the canonical replacement instead of being silently aliased.
        All shipped configs already set the canonical keys, so nothing in-tree
        breaks; the alias was a transitional bridge and is gone. A bare config
        (no period/count info) defaults to the base template's pool
        (`[1000, 500, 200, 100, 50, 33, 20]`) and `N_TASKS=10`.

      **Tests:** rewrote `TestP17ZeroCountTasksets` →
      `TestP19UnifiedPoolTasksets` (5 cases: unified-pool sourcing,
      pool-exhaustion duplicates, N=1 single-rate, legacy-keys-rejected,
      N_TASKS<1-rejected). The legacy-aliasing tests across
      `test_generation_config_parser.py`,
      `Gen_Taskset/tests/test_generation_config.py`, and
      `Gen_Taskset/tests/test_specifications.py` were flipped to assert
      `ValueError` rejection. Remaining test files migrated to the canonical
      keys in-place. Full suite **238 passing** (`tests/python/` 224 +
      `Gen_Taskset/tests/` 14).

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

---

### P20 -- End-to-end: all stages share the same task sets

**Goal:** `run_end_to_end.sh` (the e2e orchestrator) must use the **same
generated task sets** across all stages so metrics are cross-referenceable,
instead of each stage generating its own. The regeneration prompt itself is
**not** the bug — it is a desired drift guard the user wants kept (a `[Y/n]`
prompt when an on-disk taskset's config has drifted is correct behavior, not
something to silence). Per user spec:

1. **Main step (and any non-sweep step):** generate each task set **once**;
   later stages reuse the main step's task sets, not newly generated ones.
2. **Sweep step:** regeneration is allowed for a *different* interval (the
   interval-sliced `taskset_characteristics_interval_*.yaml` files genuinely
   depend on the trigger interval), **but** first reuse the main step's
   already-generated task sets when the scheduler interval matches the main
   step's interval — do not generate a duplicate set.

**The actual bug (verified):** the sweep stage wrote to its own dirs
(`tasks{N}_sweep_interval{I}_seed{S}`) and ran the full generation pipeline per
interval — so for the interval that matches the main step's default, it
generated a *second, duplicate* set of task sets instead of reusing the main
step's. The core random draws are already identical by seed (see below), but
the on-disk files + scheduler outputs were duplicated into separate dirs,
breaking cross-reference and wasting work.

**Key facts (verified, constrain the design):**
- `generate_taskset_parameters()` (the core: periods, ET means/sigmas,
  sp_weights, priorities, paths) seeds **only off `RANDOM_SEED`** — it does
  **not** read `UPDATE_INTERVAL_S`. So the core random task set is already
  identical across the main step and every sweep interval today (same
  `base_seed + idx`, same N → same draws). "Same tasksets across steps" is
  already true at the random-draw level; the fix is about reusing the
  *generated files*, not the draws.
- The interval-sliced `taskset_characteristics_interval_*.yaml` files (which
  the C++ `RunOrchestrator` reads) **do** depend on the interval:
  `n_intervals = ceil(n_sec / update_interval_s)` and Et bucketing use it.
  Different sweep intervals therefore need different sliced files; the core
  cannot be shared verbatim across intervals at the *file* level — which is
  why a non-matching sweep interval still regenerates.
- `write_summary_and_plots()` (`utils.py:192`) opens
  `comparison_summary.csv` in `"w"` mode (overwrite). The main step writes the
  **union** scheduler list (main + ablation); the sweep plots the **main** list
  only. A matching-interval sweep point that *ran* `compare_optimizers` against
  the main step's output dir would clobber the union summary with a main-only
  summary, dropping the ablation rows the aggregate step reads. **So
  matching-interval reuse must skip the run and read the existing summary**,
  not re-run into the main dir.
- The regeneration prompt (`_should_generate()` in
  `run_sim_experiments.py:153`) fires on genuine config drift (e.g. the on-disk
  baselines are stale after the P19 refactor). That is **desired** — the fix
  must not silence it. (An earlier draft of P20 forced
  `--on_taskset_config_change=regenerate` to kill the prompt; that was wrong
  and was reverted. The orchestrator now leaves the policy at its `prompt`
  default.)

**Design:**
- [x] **P20.3** Sweep interval reuse: before running `compare_optimizers` for a
      sweep interval `I`, check whether the main step's dir for `(N, n_sec, I,
      seed)` exists with **fresh** task sets (every `taskset_{idx}`'s saved
      `generator_config.json` matches the resolved config + `RANDOM_SEED =
      base_seed + idx` + `UPDATE_INTERVAL_S = I`). If so, **skip** the run and
      read the main dir's `comparison_summary.csv` directly (its union summary
      already contains every scheduler the sweep plots) — no regeneration, no
      clobbering. If `I` differs from the main step's interval or the main dir
      is absent/stale, run `compare_optimizers` as before (regeneration
      allowed for a genuinely different interval).
- [x] **P20.4** Tests: `test_run_end_to_end.py` asserts the simulate command
      does **not** override the regeneration policy (prompt kept) and the
      sweep command carries `--reuse_matching_interval` (and does not override
      the policy). `test_interval_sweep.py::TestSweepReuseMatchingInterval`
      covers: fresh main dir reused (no subprocess), absent main dir runs,
      stale main dir not reused, policy forwarding, reuse-disabled always
      runs.

**Implemented (this change):**
- `run_end_to_end_experiments.build_sweep_command` appends
  `--reuse_matching_interval` (and **only** that — the regeneration policy is
  left at the sweep's `prompt` default, not silenced).
  `build_simulate_command` is unchanged: it does not touch the regeneration
  policy, so compare_optimizers' default `prompt` is kept.
- `interval_sweep.run_single_interval` gained `reuse_matching_interval` +
  `on_taskset_config_change` params (default `prompt`). When reuse is on,
  `_main_dir_is_fresh` reconstructs the main-step dir name
  (`build_experiment_dir_name`) and, for every `taskset_{idx}`, runs
  `_should_generate` (policy `"regenerate"` used purely as a staleness
  detector — it returns True iff absent/stale; `"keep"` would be wrong, it
  returns False even when stale) against the resolved config overlaid with
  `RANDOM_SEED = base_seed + idx` / `UPDATE_INTERVAL_S = interval_sec`. If all
  tasksets are fresh + a `comparison_summary.csv` exists, the sweep **skips**
  `compare_optimizers` and returns the main dir. A stale/absent or
  non-matching interval falls through to a normal `compare_optimizers` run.
- New CLI flags on `interval_sweep.py`: `--reuse_matching_interval` (off by
  default) + `--on_taskset_config_change` (default `prompt`).
- Tests: `test_run_end_to_end.py` +2 (simulate keeps prompt policy; sweep
  carries `--reuse_matching_interval` and keeps prompt policy);
  `test_interval_sweep.py` +5 (`TestSweepReuseMatchingInterval`: fresh-main-dir
  reuse skips the subprocess, absent main dir runs it, stale main dir is not
  reused, the policy is forwarded to the child, reuse-disabled always runs).
  Full suite **231 passing** (was 224).

**Out of scope:** refactoring `Gen_Taskset/lib/orchestrator.py` to split the
interval-independent core from interval-dependent slicing (the core is already
shared by seed today; the redundant core regen in the sweep is wasted work but
not incorrect, and the refactor is non-trivial). P20 targets the prompts +
correct reuse, not the generation-pipeline split.

---

### P24 -- Periodic Reoptimization (general drift-bound; prerequisite for P21)

**Status: IMPLEMENTED (working tree uncommitted, 2026-07-02) — P24.1–P24.6 done;
`build/tests/testIncreOpt_w_TL` 17/17 green, Release `RunOrchestrator` builds
clean, smoke + A/B verified no SP regression (SP improves on reopt intervals).
Prerequisite for P21 (warm-start) is now satisfied.**

**Goal:** the incremental optimizer (`OptimizeIncre_w_TL`, called once per
`SimulateInterval`) reuses a persistent `timelimit2optimizer_` cache that is
**never cleared** — every interval after the first only ever calls `OptimizeIncre`
on cached TL configs. Over a long sim (prod = 60 intervals) the priority
assignment can drift toward a local optimum as task ETs evolve, with no mechanism
to re-ground it. P24 adds a general, per-interval **periodic reoptimization**: on
every N-th interval, re-run the from-scratch coordinate-descent TL search with a
**wider TL window** (cache kept — a "warm re-explore"). This bounds drift for
**every** `INCR`-family mode, independent of warm-start (P21 reuses it).

**Agreed design (per user direction 2026-07-02):**

- **Two new params** in `sources/Utils/Parameters.{h,cpp}` + `sources/parameters.yaml`,
  declared alongside `TimeLimitSearchRadiusIncr` (general tunables, not per-mode
  runtime flips):
  - `extern int ReoptimizationPeriod;` — default **10**, `0` = **disabled**
    (reproduces today's pure cache+scratch baseline exactly — the off-switch the
    A/B uses). `N>0` = run a reoptimization interval every N-th interval.
  - `extern int ReoptimizationTimeLimitsSearchRadius;` — default **4**. TL window
    radius during reoptimization intervals. Normal intervals keep
    `TimeLimitSearchRadiusIncr` (=2), unchanged. (Renamed from
    `ReoptimizationSearchRadius` on 2026-07-02 for clarity — it tunes the
    *time-limit* option window, mirroring `TimeLimitSearchRadiusIncr`.)
- **Single orchestrator entry point — `Optimize_w_TL_ScratchOrIncre()`** (user
  direction 2026-07-02): add one new method on
  `OptimizePA_Incre_with_TimeLimits` that the orchestrator calls from the first
  interval to the last. The method owns the per-interval counter and the
  reoptimization-vs-incremental decision, so the orchestrator no longer needs to
  know about intervals — it just calls the one method per `SimulateInterval`.
  (See P24.3 for the body.) The INCR_NO_TL / INCR_WCET ablation-flag wrappers
  stay as thin save/flip/restore shells around this one call — unchanged from how
  they wrap `OptimizeIncre_w_TL` today.
- **Cadence = per-interval**, counted inside `Optimize_w_TL_ScratchOrIncre` (one
  call per `SimulateInterval` → `DeterminePrioritiesAndBudgets`). "Reset by
  interval count" ≡ "reset by call count" — these are the *same* option, because
  `DeterminePrioritiesAndBudgets` calls `OptimizeIncre_w_TL` exactly once per
  `SimulateInterval` (verified in `SimulationOrchestrator.cpp:267-269`, loop at
  253-257); the call count *is* the interval count, 1:1 and deterministic. The
  counter lives on `OptimizePA_Incre_with_TimeLimits` (not the orchestrator) so
  `tests/testIncreOpt_w_TL.cpp` can drive a sequence of
  `Optimize_w_TL_ScratchOrIncre` calls and assert the reoptimization path fires
  without a full sim. Counter resets at the start of each `RunSimulation`
  (optimizer construction / first interval).
- **Interval 0 = the first reoptimization interval** (option A, user-confirmed):
  interval 0 uses the wider `ReoptimizationTimeLimitsSearchRadius` window.
  Reoptimizations then fire at 0, N, 2N, …. This slightly changes today's
  interval-0 search (more TL configs evaluated) — acceptable and intended.
- **Warm re-explore, cache kept** (user-confirmed): on a reoptimization interval the
  `timelimit2optimizer_` cache is **not** cleared. Already-seen TL configs reuse
  `OptimizeIncre`; only TL combos not yet in the cache get a true
  `OptimizeFromScratch(K)`. So a reoptimization interval costs ~the narrow→wide
  delta, not a full cold re-ground. (NB: this is the *incremental* path with a
  wider radius — it is **not** `OptimizeFromScratch_w_TL`, which uses the full
  `RecordTimeLimitOptions` list. The 2026-07-02 "full from-scratch on reopt" reading
  was superseded by this bounded-wider-radius design, because a full-list search on
  interval 0 — cache empty — would `OptimizeFromScratch(K)` every unseen TL vector:
  the unpredictable cost the user explicitly rejected earlier.)
- **Mechanics (verified in code):** `OptimizeIncre_w_TL` uses
  `RecordCloseTimeLimitOptions`, which today hard-reads
  `GlobalVariables::TimeLimitSearchRadiusIncr` and returns a windowed slice (radius
  centered on each task's current ET, clamped to `timePerformancePairs.size()`).
  P24 **parameterizes** it as `RecordCloseTimeLimitOptions(dag_tasks, radius)` so
  the reoptimization path passes `ReoptimizationTimeLimitsSearchRadius` while the
  normal path keeps passing `TimeLimitSearchRadiusIncr`. The radius is bounded by
  the list length (the loop already clamps `j < timePerformancePairs.size()`), so
  it never silently degenerates to "search all options" — that only happens if a
  user sets the radius ≥ list size, which is their explicit knob.
- **Mode scope (architecturally dictated):** the reoptimization branch lives inside
  `Optimize_w_TL_ScratchOrIncre` (the single entry point every persistent-INCR mode
  routes through), so it applies to **INCR / INCR_NO_TL / INCR_WCET** automatically
  and stays decoupled from mode strings. `INCR_NO_TL` short-circuits via
  `disable_time_limit_opt` (radius moot there). `INCR_SCRATCH` / `BF` / `RM` have
  no persistent cache and are structurally excluded.
- **Why per-interval, not per-eval (user direction):** the number of
  `EvaluateTimeLimitConfig` calls per interval varies with N (task count), beam
  width K, `TimeLimitSearchRadiusIncr`, and per-task TL-option count — so a per-eval
  cadence fires at unpredictable wall-clock points and makes speedup/quality
  numbers unreproducible. Per-interval is deterministic regardless of taskset size:
  more tractable, and it prevents the strange issues a varying per-call count would
  introduce.
- **Not a no-op in test mode (user correction):** a 7-interval test sim
  (`70s/10s`) still does from-scratch on interval 0 (interval 0 is a reoptimization
  interval); with default period 10 it never reaches interval 10, so the *periodic
  repeat* path is unexercised but behavior is stable/comparable (it does *not* "do
  nothing" — it does the same from-scratch-on-interval-0 it always did, now with
  the wider window). Prod runs (`600s/10s` = 60 intervals) fire ~6 reoptimizations
  and actually bound ET drift.

**Key correctness constraint (verified in code):** on a reoptimization interval,
the incremental body (`OptimizeIncre_w_TL`, now called via the
`Optimize_w_TL_ScratchOrIncre` wrapper) must (a) pull in the current interval's DAG
(`dag_tasks_ = dag_tasks_update`) and apply WCET ablation **before** building the
wide-window TL options, and (b) re-initialize `time_limits` from the current ET
config (`InitializeTimeLimitsFromETConfig`) — exactly what the existing
`OptimizeIncre_w_TL` body already does. The only change is the radius passed to
`RecordCloseTimeLimitOptions`. `PerformCoordinateDescentForTaskConfigOpt` and
`EvaluateTimeLimitConfig` are reused unchanged.

**Implementation plan (module-by-module, TDD — DONE 2026-07-02, working tree uncommitted):**

- [x] **P24.1 (params)** `sources/Utils/Parameters.h` + `.cpp` +
      `sources/parameters.yaml`: add `extern int ReoptimizationPeriod;` (default
      **10**) and `extern int ReoptimizationTimeLimitsSearchRadius;` (default
      **4**), loaded from YAML keys of the same names, declared alongside
      `TimeLimitSearchRadiusIncr`. Both are general tunables.
- [x] **P24.2 (parameterize the radius)** `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}`:
      change `RecordCloseTimeLimitOptions(const DAG_Model&)` to
      `RecordCloseTimeLimitOptions(const DAG_Model&, int radius)`, replacing the
      hard-coded `GlobalVariables::TimeLimitSearchRadiusIncr` read with the `radius`
      param. Update the call sites: the chosen radius is now passed from
      `OptimizeIncre_w_TL` (see P24.3 — the radius depends on the counter, so
      `OptimizeIncre_w_TL` takes the radius as a param or the wrapper computes it and
      passes it down); the constructor's `RecordTimeLimitOptions` call is unaffected
      (it is the full-list variant). No behavior change yet for normal intervals.
      Implemented: `RecordCloseTimeLimitOptions(dag, radius)` +
      `OptimizeIncre_w_TL(dag, K, radius)`; all call sites in
      `tests/{testIncreOpt_w_TL,testBF_w_TL,AnalyzePriorityAssignmentIncrementalExample}.cpp`
      updated to pass `TimeLimitSearchRadiusIncr`.
- [x] **P24.3 (wrapper + counter + radius decision)** `OptimizePA_Incre_with_TimeLimits`:
      add the new single entry point the orchestrator calls every interval:
      `PriorityVec Optimize_w_TL_ScratchOrIncre(const DAG_Model& dag_tasks_update, int K)`.
      It owns (i) the per-interval counter and (ii) the reoptimization-vs-incremental
      radius decision, then delegates to the existing `OptimizeIncre_w_TL` body.
      Concretely:
      - Add member `int reoptimization_interval_count_ = 0;`. Initialize to 0 at
        construction; reset to 0 in `OptimizeFromScratch_w_TL` (that method signals
        a fresh from-scratch run by setting `opt_sp_ = -1.0`). **Do NOT zero it
        inside `OptimizeIncre_w_TL` / the wrapper's per-call path** — zeroing every
        call would freeze it at interval 0 so it never advances. (Increment once per
        call = once per interval; that *is* the per-interval count — see the cadence
        bullet above.)
      - In `Optimize_w_TL_ScratchOrIncre`, decide the radius from the *current*
        counter value, then increment:
        `int radius = (ReoptimizationPeriod > 0 && reoptimization_interval_count_ %
        ReoptimizationPeriod == 0) ? ReoptimizationTimeLimitsSearchRadius :
        TimeLimitSearchRadiusIncr;`
        `++reoptimization_interval_count_;`  // after the radius decision
        then call the `OptimizeIncre_w_TL` body with that `radius` (either pass
        `radius` as a new param to `OptimizeIncre_w_TL`, or have the wrapper set
        `time_limit_option_for_each_task_ = RecordCloseTimeLimitOptions(dag_tasks_,
        radius)` and call a radius-taking overload — pick whichever the implementer
        finds cleanest; the body below that point is unchanged).
      - Trace: count starts 0 → interval 0 reopt (0%10==0), count→1; intervals 1-9
        normal (narrow radius); interval 10 reopt (10%10==0); … = option A (reopts
        at 0, N, 2N, …). With `ReoptimizationPeriod == 0`, radius is always
        `TimeLimitSearchRadiusIncr` → today's behavior exactly.
- [x] **P24.4 (TDD tests)** `tests/testIncreOpt_w_TL.cpp`: failing-then-passing —
      (a) `ReoptimizationPeriod` defaults to 10,
      `ReoptimizationTimeLimitsSearchRadius` to 4; (b) `ReoptimizationPeriod == 0`
      → radius passed is always `TimeLimitSearchRadiusIncr` (assert via a getter or
      by checking the `time_limit_option_for_each_task_` window size on a task with
      enough TL options), reproducing today's behavior; (c) with period=N, a
      sequence of `Optimize_w_TL_ScratchOrIncre` calls uses the wide radius on
      intervals 0, N, 2N, … and the narrow radius on 1..N-1, N+1..2N-1, … (assert
      via `reoptimization_interval_count_` and window size); (d) the counter
      resets to 0 across a fresh optimizer / new `OptimizeFromScratch_w_TL`. Build
      + run `build/tests/testIncreOpt_w_TL`.
- [x] **P24.4b (orchestrator wiring)** `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp`:
      in `DeterminePrioritiesAndBudgets`, replace the `incr_optimizer_.OptimizeIncre_w_TL(...)`
      call in the `INCR` branch (and the same call inside the `INCR_NO_TL` /
      `INCR_WCET` save/flip/restore wrappers) with
      `incr_optimizer_.Optimize_w_TL_ScratchOrIncre(...)`. The orchestrator now
      makes one call per interval and carries no interval logic. (`INCR_SCRATCH`
      is untouched — it still constructs a fresh optimizer + `OptimizeFromScratch_w_TL`
      per interval, structurally outside P24.) Build.
- [x] **P24.5 (smoke)** Build the orchestrator; run a short `INCR` sim
      (`incr_et_8tasks_config`, 7 intervals) — confirms no crash, interval 0 uses
      the wide window, no repeat fires (period 10 > 7). Then a prod-scale run
      (`experiment_config`, 60 intervals) confirms ~6 reoptimization intervals fire
      without regression in SP or `Mean_Scheduler_Execution_Time_s`.
      **Verified 2026-07-02 (CORRECTED):** short smoke (dur70, 7 intervals, INCR)
      ran clean — no crash, 7 SP values emitted. A/B on the same 7-interval taskset
      (period 0 vs period 3, the latter firing reopts at 0/3/6 = the periodic
      **repeat** path, not just interval 0), run with the **correct** `duration_ms`
      arg (= `scheduler_trigger_interval*1000` = 10000, the per-interval horizon —
      NOT the total sim 70000; see `runorchestrator-duration-arg-semantics` memory):

      | config | reopt intervals | exec (s) | avg SP |
      |---|---|---|---|
      | P24 OFF (period 0) | none | 0.568 | 1.56568 |
      | P24 ON period 10 (default, reopt @0) | 0 | 0.613 | 1.60964 |
      | P24 ON period 3 (reopt @0,3,6) | 0,3,6 | 0.712 | 1.73874 |
      | P24 ON period 1 (reopt every interval) | all | 0.750 | 2.12851 |

      **No SP regression — SP improves** (the wider re-explore finds better
      assignments); real exec cost is **~+45 ms per reopt interval** (period 3 vs
      off: 0.712−0.568 = 0.144s over 3 reopts), NOT the +1.35s/reopt-interval logged
      in the original (bogus) A/B (which passed 70000 as `duration_ms`). The
      60-interval prod run is ~60×0.08s ≈ 5s, not 19+ min — P18's ">0.1s/interval"
      was likewise inflated by the same arg misuse and needs re-measurement.
      `build/tests/testIncreOpt_w_TL` = **17/17 green** (13 existing + 4 P24);
      `release/tests/RunOrchestrator` Release build clean.
- [x] **P24.6 (docs)** Update `tasks.md` checkboxes + `dev_log.md` with the smoke
      results. `git add` (user commits).

**Open questions:** none — all design choices resolved 2026-07-02 (naming =
reoptimization; single orchestrator entry point `Optimize_w_TL_ScratchOrIncre`;
interval 0 = first reopt, option A; cache kept = warm re-explore; reopt =
wider-radius *incremental* path, not full `OptimizeFromScratch_w_TL`; mode scope =
all persistent-INCR; cadence = per-interval = per-call (1:1); radius knob renamed
`ReoptimizationTimeLimitsSearchRadius`; defaults 10 / 4).

**Out of scope:** warm-start incumbent seeding (that is P21, which depends on
P24); within-interval per-eval escape (deferred — would need a separate
clearly-named within-interval counter, explicitly *not* this general
reoptimization).

### P24-Eval — Re-optimization interval sweep (per-activation optimizer runtime)

**Status: IN PROGRESS (2026-07-03).** Requested by user after discovering the
P24.5 A/B exec times were bogus (RunOrchestrator `duration_ms` arg misuse — see
`runorchestrator-duration-arg-semantics` memory). Re-measure correctly and sweep
the re-optimization interval to characterize the cost/quality trade-off.

**Goal:** measure the **average per-activation optimizer runtime** (NOT total
`Mean_Scheduler_Execution_Time_s`, which is dominated by the per-ms discrete-event
sim loop and therefore conflates optimizer cost with sim cost). Report how
per-activation optimizer runtime varies with the re-optimization period and with
task-set size.

**Method:**
- **Sweep:** `ReoptimizationPeriod ∈ {0, 1, 3, 5, 10, 20}` (0 = off = today's
  pure-cache baseline). 6 configs.
- **Task counts:** `N ∈ {6, 8, 10, 12}`.
- **Replicates:** 10 random task sets per `(period, N)` cell.
- **Scale:** 60 intervals each (`n_sec=600`, `scheduler_trigger_interval=10` →
  `duration_ms=10000` per interval, the CORRECT arg).
- **Scheduler:** INCR only (no BF — user explicitly: "only focus on INCR
  performance evaluation, don't consider BF for speed up").
- **Metric:** average per-activation optimizer runtime = (sum of optimizer
  wall-clock over all activation intervals) / (number of activations). An
  "activation" = one call to `Optimize_w_TL_ScratchOrIncre` (= one `SimulateInterval`).
  Isolate via chrono instrumentation around the optimizer call, written to
  `optimizer_runtime_per_interval.csv` per run; NOT the existing
  `scheduler_execution_time.txt` (which times the whole `RunSimulation`).

**Task-set source:** reuse the on-disk `tasks{6,8,10}_dur600_interval10_seed1000`
tasksets (10 each, 9 for N=10 — already 60 intervals). Generate 10 fresh
`tasks12_dur600_interval10_seed1000` tasksets matching the same generator config
for a fair cross-N comparison.

**Sub-tasks:**
- [x] **P24-Eval.1 (instrument)** Add `std::chrono` timing around
      `Optimize_w_TL_ScratchOrIncre` in `DeterminePrioritiesAndBudgets`; record
      `(interval_idx, radius, runtime_s)` per interval and write
      `optimizer_runtime_per_interval.csv` at `ExportResults`. Isolates optimizer
      cost from the per-ms sim loop. Build release.
      **DONE (working tree 2026-07-03):** timing wraps the `OptimizeIncre_w_TL`
      call inside `Optimize_w_TL_ScratchOrIncre` (the optimizer body only, not
      the per-ms sim loop). New `PerActivationRuntime{interval_idx,radius,
      runtime_s}` struct + `per_activation_runtimes_` member + const accessor on
      `OptimizePA_Incre_with_TimeLimits`; `SimulationOrchestrator::GetPerActivation
      Runtimes()` forwards it; `tests/RunOrchestrator.cpp` writes
      `optimizer_runtime_per_interval.csv` (one row/interval, to `output/mode/`)
      and reads an optional `REOPTIMIZATION_PERIOD` env-var override so the
      reopt-period sweep varies it per-run without editing the tracked YAML.
      Release `RunOrchestrator` builds clean + verified on a smoke run (CSV has
      61 lines = header + 60 intervals, radius 2 for period=0, 4 for period=1).
- [x] **P24-Eval.2 (gen tasksets)** Generate fresh `dur600` tasksets; verify 60
      interval files each.
      **DONE (2026-07-03):** regenerated 10 tasksets each for N=6,8,10,12 (not
      just N=12 — the on-disk `tasks{6,8,10}_dur600_*` were stale, generated
      pre-P13/P14/P19: N=6 all @ per-core load 1.2, N=8 half 1.6/half 0.9, N=10
      @ 0.9 — invalid for a cross-N comparison). New
      `simulation_experiments/gen_p24eval_tasksets.py` mirrors
      compare_optimizers's generation path (same config resolution, seed scheme
      `base_seed+idx`, `UPDATE_INTERVAL_S`, `run_full_generation_pipeline`) but
      generates ONLY tasksets — no scheduler sim — so the dirs stay clean for
      the sweep. All 4 N values now draw from the same current canonical config
      (paper_base: `CPU_UTIL_RANDOM_RANGE [0.5,1.5]`, unified `PERIODS_MS`),
      so the N axis is the only structural varying factor. Verified: 10
      tasksets/N, 60 interval files each.
- [~] **P24-Eval.3 (run sweep)** Run INCR for the 6 periods × 4 task counts × 10
      tasksets (240 runs, 60 intervals each). Collect CSV from each.
      **PARTIAL (2026-07-03):** `simulation_experiments/run_p24eval_sweep.py`
      runs each cell via `RunOrchestrator <ts_dir> <ts_dir>/INCR INCR 10000`
      (per-interval horizon) with `REOPTIMIZATION_PERIOD` env-var, copies the
      runtime CSV to `p24_eval/runtime_p{P}_N{N}_ts{idx}.csv`, writes a
      `sweep_manifest.csv`. The previous sweep run completed **210/240 cells**:
      N∈{6,8,10} × all 6 periods {0,1,3,5,10,20} (180) + N=12 × periods {0,1,3}
      (30). **30 cells missing: N=12 × periods {5,10,20}** (sweep stopped there).
      The script's resume-skip (`_cell_done`) caches the 210 done cells, so
      re-running it with the full grid runs ONLY the 30 missing (~3-5 min, `-j 6`).
      Nothing currently running. One script bug found+fixed earlier (f-string
      `({el:.1fs elapsed)` → `({el:.1f}s elapsed)`).
- [x] **P24-Eval.4 (report)** Aggregate: average per-activation optimizer runtime
      per `(period, N)`, with activations-per-run (= reopt count) for context.
      Table in dev_log.md. Do NOT report total exec.
      **DONE (2026-07-03, on 210/240 cells):** `simulation_experiments/
      aggregate_p24eval.py` reads the copied CSVs (+ manifest, with a mid-sweep
      filename-scan fallback), pools all activations across replicates, writes
      `p24eval_summary.csv` + `.md` (Markdown table: mean/std/median/max runtime,
      reopt/run, **reps**, mean SP). Fixed: the stale on-disk `sweep_manifest.csv`
      had only 180 rows (no N=12 at all), so the 30 existing N=12 cells (p0/p1/p3)
      were invisible — rebuilt the manifest from the on-disk CSVs (210 rows,
      `avg_sp` re-read from each taskset's SP summary; old manifest backed up to
      `sweep_manifest.stale.bak`). Added a `reps` column to the table so partial
      cells are explicit. Table pasted into `dev_log.md` (P24-Eval section). N=12
      p5/p10/p20 are absent from the table because they were not run — re-run
      P24-Eval.3 to fill them, then re-aggregate.

**Reporting:** average per-activation optimizer runtime (s), one row per
`(period, N)`, plus the number of reopt activations per 60-interval run
(60/period for period>0; 0 for period=0). No total-exec column.

---



**Status: DESIGN PHASE — no C++ edits landed; working tree clean on 2026-07-02.
Plan revised 2026-07-02: the periodic from-scratch **reoptimization** is a
*general* drift-bound (not a warm-start detail), so it has been split out as its
own independent task **P24** (lands first — we are working on it before any
warm-start code). P24 owns the knobs `ReoptimizationPeriod` (per-interval,
default **10**, `0`=off) and `ReoptimizationTimeLimitsSearchRadius` (default
**4**), and adds the single orchestrator entry point
`Optimize_w_TL_ScratchOrIncre` that owns the per-interval counter + radius
decision. The
warm-start-specific `WarmStartReseedPeriod` (default 5) from the earlier draft is
**dropped** — warm-start reuses P24's `ReoptimizationPeriod`. P21 (this task) is
now warm-start *only* and depends on P24. See "Agreed design" + open questions
below.**

**Goal (from `agents/improve_efficiency.md` §3.C):** `EvaluateTimeLimitConfig`
(`sources/Optimization/OptimizeSP_TL_Incre.cpp`) runs a full Audsley beam search
`OptimizeFromScratch(K)` (O(K·N²) RTA calls) on every *cache-miss* time-limit
config. The optimizer already has a good priority assignment from the previous
configuration; warm-start reuses it and calls `OptimizeIncre` (1-D variation search
on the changed task) instead of `OptimizeFromScratch` — O(N) RTA lookups.

**Agreed design (incumbent-seeding + shared periodic reoptimization, per user
direction 2026-07-02):**

The scheduler holds a good config before it performs TL optimization and uses it as
the starting point for the incremental optimization in the outer (TL-option) loop,
the same way `OptimizeIncre` does in the inner (priority) loop. For a TL-capable
task, the outer loop picks neighbor time-limit options and evaluates each; within
each option the optimizer has a complete `DAG_Model` and may run either
`OptimizeFromScratch` or `OptimizeIncre` (treating the task whose TL changed as one
of the tasks whose ET changed). Drift toward a local optimum is bounded by the
**shared periodic reoptimization** added in **P24** — warm-start no longer
introduces its own.

- **Seed = the incumbent (global-best) config**, not a single evolving optimizer and
  not a cache neighbor search. The incumbent is already tracked by `UpdateRecords` →
  `this->opt_pa_` / `opt_sp_` / `res_opt_.id2time_limit`. For each
  `EvaluateTimeLimitConfig` on the warm path:
  1. Reconstruct the incumbent DAG: `UpdateExtDistBasedOnTimeLimit(dag_tasks_,
     incumbent_time_limits)` where `incumbent_time_limits` = `time_limits` with the
     swept position reverted to its pre-sweep value (so the diff vs `dag_tasks_cur` is
     exactly the one swept task).
  2. Seed a per-eval `OptimizePA_Incre` with (incumbent DAG, incumbent `opt_pa_`,
     incumbent `opt_sp_`).
  3. `optimizer.OptimizeIncre(dag_tasks_cur)` → `FindTaskWithDifferentEt` diffs
     incumbent vs new, detects the swept task, searches its 1-D variations.
  4. `UpdateRecords` promotes the result if it beats the incumbent.
- **Periodic escape (SHARED, not warm-start-specific):** the re-grounding to a wider
  from-scratch TL search is driven by `GlobalVariables::ReoptimizationPeriod` +
  `ReoptimizationTimeLimitsSearchRadius` (**P24**), **not** a `WarmStartReseedPeriod`.
  Per the 2026-07-02 design decision the reoptimization cadence is **per-interval**
  (orchestrator level, not per-eval), and it is **active for baseline `INCR` too** —
  so both arms carry the same drift-bound and the A/B isolates "cache+scratch vs
  incumbent+`OptimizeIncre`" *between* reoptimizations. Implication for warm-start:
  see open question 3 below (the earlier per-eval in-sweep escape is *not* provided
  by a per-interval knob; v1 defers it).
- **`timelimit2optimizer_` cache + `OptimizeFromScratch`-on-miss stays unchanged for
  the baseline `INCR`** (flag off). Warm mode bypasses the cache (on the warm path
  `OptimizeIncre` is already cheap, so the map has nothing to save). This keeps the
  comparison clean: baseline = current cache+scratch path + shared periodic
  reoptimization; warm = incumbent-seed + `OptimizeIncre` + the same shared periodic
  reoptimization.
  > [!WARNING]
  > Deciding whether to trigger incremental or from-scratch priority optimization based on the presence of a cached time-limit combination (`timelimit2optimizer_.count(time_limits)`) is not a reliable method. As the number of possible time-limit combinations explodes, cache misses dominate, rendering incremental optimization less useful. P21 addresses this by introducing a true warm-start path that bypasses the cache and seeds from the incumbent.
- **Experiment vehicle:** a temporary scheduler mode `INCR_WARM` that flips
  `use_warm_start_incremental_opt` on around the `INCR`
  `Optimize_w_TL_ScratchOrIncre` call — mirrors `INCR_NO_TL`/`INCR_WCET` exactly.
  Baseline = `INCR` (flag off). Removed by the Decisive Action (folded into `INCR`
  if good; dropped if bad).
- **Testability (one per-eval counter, not two — user direction 2026-07-02):** a
  single per-eval counter on `OptimizePA_Incre_with_TimeLimits`, reset at each
  `RunSimulation` start, alongside the per-interval one:
  - `reoptimization_interval_count_` (P24, per-interval) — incremented inside
    `Optimize_w_TL_ScratchOrIncre` when an interval takes the reoptimization path.
    Assertable from a sequence of `Optimize_w_TL_ScratchOrIncre` calls without a
    full sim.
  - `warm_start_eval_count_` (P21.2, per-eval) — incremented inside
    `EvaluateTimeLimitConfig` each time the warm path runs (incumbent-seed +
    `OptimizeIncre`). Only meaningful when `use_warm_start_incremental_opt` is on.
    The from-scratch-on-warm-path evals are **not** separately counted — they only
    occur on reopt intervals, which `reoptimization_interval_count_` already
    identifies, so a dedicated `from_scratch_eval_count_` is redundant and dropped.
  Deterministic unit-test assertions (timing micro-tests are too flaky); real speedup
  measured by the P21.6 experiment via `Mean_Scheduler_Execution_Time_s`.
  `warm_start_eval_count_` is the signal that disambiguates a null P21.6 speedup
  ("warm-start didn't fire" vs "fired but didn't help").

**Key correctness constraint (verified in code):** `OptimizeIncre` finds changed
tasks via `FindTaskWithDifferentEt(dag_tasks_, dag_tasks_update)` — it compares the
optimizer's *stored* DAG against the update arg. So the seeded optimizer's
`dag_tasks_` must be the **incumbent's** DAG (the previous config), and the new
config is passed as `dag_tasks_update`; then exactly the one swept task is detected
and searched. `OptimizeIncre` errors if `opt_pa_` is empty, so the seed must supply a
non-empty `opt_pa_` (guaranteed: the first eval uses scratch, establishing the
incumbent; later evals seed from it).

**Verified coordinate-descent property:** `PerformCoordinateDescentForTaskConfigOpt`
sets `time_limits[idx] = best_option_val` before moving to the next task, so while
task `idx` is swept, `time_limits` carries the finalized best for all earlier tasks.
The candidate `time_limits` (with `idx=val`) therefore differs from the incumbent
(time_limits with `idx` reverted to its pre-sweep value) in **exactly position
`idx`** — a clean 1-task diff per `EvaluateTimeLimitConfig`. (At a task-sweep
boundary the incumbent is re-established as the just-finalized best, so the property
holds across the whole sweep.)

**Implementation plan (module-by-module, TDD — NOT started, pending alignment):**

> **Prerequisite:** P24 (periodic reoptimization) must land first — P21.2 and
> P21.3 reference `ReoptimizationPeriod` and the `reoptimization_interval_count_`
> counter that P24 introduces.

- [ ] **P21.1 (C++ core — warm-start flag only)** `sources/Utils/Parameters.h` +
      `.cpp`: add `extern bool use_warm_start_incremental_opt;` (default false).
      Mirrors the existing `disable_time_limit_opt` / `use_wcet_execution_time`
      ablation flags. **No warm-start-specific reoptimization knob is added** —
      warm-start reuses `ReoptimizationPeriod` from **P24** (the reoptimization is
      shared, see design above).
- [ ] **P21.2 (C++ core — warm-start)** `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}`:
      add `SeedFromIncumbent(OptimizePA_Incre& target, const std::vector<double>&
      candidate_tl, int swept_idx) const` (reconstructs incumbent DAG = candidate
      with `candidate_tl[swept_idx]` reverted to incumbent value, copies
      `opt_pa_`/`opt_sp_`; returns false if `opt_pa_` empty). Wire into
      `EvaluateTimeLimitConfig` else-branch: when flag on and this is **not** a
      reoptimization interval (per the shared `ReoptimizationPeriod` counter from
      P24) → `SeedFromIncumbent` + `OptimizeIncre(dag_tasks_cur)` +
      `warm_start_eval_count_++`; when flag on and this **is** a reoptimization
      interval → `OptimizeFromScratch(K)` (same path baseline takes on a
      reoptimization interval); if seed fails → fall back to `OptimizeFromScratch(K)`.
      (Only `warm_start_eval_count_` is incremented — the from-scratch-on-warm-path
      evals are identified by the reopt interval, not a separate counter. See
      testability note above.) Open question for Gemini: how does
      `EvaluateTimeLimitConfig` learn `swept_idx`? (See open questions below.)
      Reset `warm_start_eval_count_` in `OptimizeFromScratch_w_TL` and
      `Optimize_w_TL_ScratchOrIncre` (the per-interval entry point —
      `OptimizeIncre_w_TL` is now its delegated body, see P24.3).
- [ ] **P21.3 (TDD tests)** `tests/testIncreOpt_w_TL.cpp`: failing-then-passing —
      (a) flag defaults false; (b) with flag ON and `ReoptimizationPeriod`
      large (so no reoptimization mid-test), `warm_start_eval_count_ > 0` on
      `OptimizeFromScratch_w_TL` (v19 taskset); (c) with flag ON and a small period,
      the reoptimization interval takes the from-scratch path (assert via
      `reoptimization_interval_count_` from P24, since there is no separate
      from-scratch counter); (d) with flag OFF, `warm_start_eval_count_ == 0`;
      (e) SP quality: warm-start `opt_sp_` within tolerance of from-scratch
      `opt_sp_` on the same taskset. Build + run `build/tests/testIncreOpt_w_TL`.
- [ ] **P21.4 (orchestrator mode)** `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp`:
      add `INCR_WARM` branch in `DeterminePrioritiesAndBudgets` (save/flip/restore
      flag around `incr_optimizer_.Optimize_w_TL_ScratchOrIncre` + `CollectResults`
      — the entry point P24.4b wired in for `INCR`/`INCR_NO_TL`/`INCR_WCET`) and add
      `"INCR_WARM"` to the `RunSimulation()` mode list that constructs
      `incr_optimizer_`. `tests/RunOrchestrator.cpp`: add `INCR_WARM` to the usage
      string. Build.
- [ ] **P21.5 (Python harness)** `simulation_experiments/compare_optimizers.py`: add
      `"INCR_WARM"` to `ALL_SCHEDULERS`. Run `pytest tests/python -q`.
- [ ] **P21.6 (experiment + decisive action)** New config
      `simulation_experiments/configs/warm_start_comparison_config.json`
      (`main_scheduler_list: ["INCR", "INCR_WARM"]`, a few tasksets, profiling on,
      prod-scale duration so the per-interval reoptimization actually fires). Run via
      `./scripts/run_end_to_end.sh`; read `Mean_Scheduler_Execution_Time_s`
      (speedup ratio) + SP from `comparison_summary.csv`. **Decisive Action** (confirm
      with user before flipping default — "permanently enable" changes baseline
      scheduler behavior):
      * SP quality close/identical + real speedup → set flag default **true**, remove
        `INCR_WARM` mode + flag (bake warm-start into `EvaluateTimeLimitConfig`).
      * SP quality significantly worse → remove `INCR_WARM` mode, keep flag false,
        report + iterate on the variation search (`FindPriorityVec1D_Variations`).
- [ ] **P21.7 (docs)** Update `tasks.md` checkboxes + `dev_log.md` with results and
      the decisive-action outcome. `git add` (user commits).

**Open questions for Gemini alignment:**
1. How does `EvaluateTimeLimitConfig` learn `swept_idx`? Candidate answers: (a)
   thread it as a new param from `PerformCoordinateDescent`; (b) infer it by diffing
   `candidate_tl` against `res_opt_.id2time_limit` (no API change, but assumes the
   incumbent TL vector is recoverable). Recommend (b) to keep the call signature
   unchanged.
2. Incumbent TL vector source: derive from `res_opt_.id2time_limit` (ordered by task
   id) at seed time, vs. store a dedicated `incumbent_time_limits_` member. Recommend
   the member (explicit, avoids re-deriving / ordering assumptions).
3. ~~Reoptimization cadence: flat every-Nth eval counter vs. per-task first-option-is-scratch.~~
   **Resolved 2026-07-02:** per-interval cadence (user direction), counted inside
   `Optimize_w_TL_ScratchOrIncre` (1 call per `SimulateInterval`, so per-interval
   ≡ per-call), shared by baseline + warm-start via `ReoptimizationPeriod`
   (**P24**). The earlier per-eval in-sweep escape is **not**
   provided by a per-interval knob. v1 defers any within-interval escape: if SP
   quality degrades in the P21.6 experiment, revisit then (it would need a separate,
   clearly-named within-interval counter — explicitly *not* the general
   reoptimization).

**Out of scope:** the `improve_efficiency.md` §1/§2/§3.A/§3.B memory/algorithm
optimizations (pointer-ize `PriorityPartialPath`, `shared_ptr` cache, flat-vector
convolve, RTA cache, zero-weight skip) — separate efficiency work; P21 is
specifically the warm-start (§3.C).


---

### P22 -- End-to-end: always run all stages; actionable aggregate empty-data (DONE)

**Goal:** the e2e orchestrator had a `--steps` flag (and `STEPS` env-var) to run a
*subset* of stages. That was a footgun: the three stages are dependent (aggregate
reads what simulate wrote; sweep reuses simulate's tasksets), so selecting a subset
against stale or missing data produced confusing "No experiment records found"
failures instead of doing the right thing. Per user request ("the only case is run
end-to-end"), the stage-selection knob is removed entirely. Separately, the INCR-ET
measurement config drops its `prod_mode` block (a drift footgun: test/prod had
drifted to different task counts), and aggregate's empty-data path becomes
actionable instead of a bare error.

**Implemented (commit `46a62b49`, branch `clean_simulation`):**
- [x] **P22.1** `run_end_to_end_experiments.py`: remove `--steps` arg,
      `ALL_STAGES` constant, and the `stage_funcs` loop. `main()` runs
      simulate → sweep → aggregate unconditionally in fixed order; a failed stage
      still aborts. Binary pre-flight simplified (always required unless
      `--dry_run`).
- [x] **P22.2** `scripts/run_end_to_end.sh`: drop the `STEPS` env-var, its usage
      example, and the `--steps` forwarding block. Header prints the fixed stage
      order (`simulate -> sweep -> aggregate`).
- [x] **P22.3** `aggregate_across_tasks.py`: the empty-data path (was a bare "Run
      simulations first." + exit 1) now prints the exact expected dir prefixes it
      looked for (`tasks{N}_dur{D}_interval{I}_seed{S}`), notes none matched, and
      points at `./scripts/run_end_to_end.sh`. Stays **read-only** -- does NOT
      auto-run simulate, so a direct re-plot can't surprise the user with a long
      C++ sim.
- [x] **P22.4** `configs/incr_et_8tasks_config.json` (new): single-purpose
      INCR-only ET measurement config. `test_mode` block only (no `prod_mode`):
      `[10]` tasks, `2` tasksets, `70s` duration, `10s` interval, seed `1000`,
      `main_scheduler_list: ["INCR"]`, `enable_execution_time_profiling: true`.
      Run it with
      `CONFIG_JSON=simulation_experiments/configs/incr_et_8tasks_config.json ./scripts/run_end_to_end.sh`.
      Scheduler avg ET = `Mean_Scheduler_Execution_Time_s` ("Avg Sched Time") in
      `comparison_summary.csv`.
- [x] **P22.5** Tests: `test_run_end_to_end.py` rewritten -- `test_all_stages_always_run`,
      `test_steps_flag_rejected` (argparse exits 2), `test_failed_stage_aborts`,
      `test_dry_run_prints_and_does_not_execute`; dropped the `--steps`-subset
      tests. 17 e2e + 35 aggregate tests pass.

**Note:** the commit title still reads `P21: ...` (it was tagged P21 before the
warm-start collision was noticed). This entry records it as **P22** per the
renumber decision (warm-start keeps P21). Supersedes the `--steps` documentation in
P10/P20.

---

### P23 -- Unify sim + figures output under one run folder (DONE)

**Goal:** co-locate each run's simulation output (incl. the task-set simulation
config -- periods, utilizations, characteristics YAMLs, generator config) with its
derived figures under the run dir, instead of sims scattered at the top level and
figures nested under `runs/<run_id>/`. Also drop a copy of the driving config JSON
into the run dir so a run is self-describing.

**Current layout (the problem)** -- sims and figures live at *different* levels:

```
optimizer_comparison/                                 <- output_parent (base)
├── tasks10_dur70_interval10_seed1000/                <- main-step sim (compare_optimizers)
│   └── taskset_0/  (generator_config.json, taskset_param.yaml,
│                   taskset_characteristics*.yaml, path_Et_task_*.txt, INCR/, *.png)
├── tasks10_sweep_interval5_seed1000/                 <- sweep sim (interval_sweep)
└── runs/run_test_dur70_interval10_seed1000_tasks10/  <- build_run_id
    └── figures/                                      <- aggregate (+ sweep fig2) write here
```

**Target layout:**

```
optimizer_comparison/
└── runs/run_test_dur70_interval10_seed1000_tasks10/   <- build_run_id
    ├── config.json                                    <- copy of the active config file
    ├── figures/                                       <- derived figures (unchanged)
    └── sim/                                           <- ALL raw sim output
        ├── tasks10_dur70_interval10_seed1000/         <- main-step sim + taskset_*/
        └── tasks10_sweep_interval5_seed1000/          <- sweep sim + taskset_*/
```

**Key constraint:** the three stages must agree on where sims live.
compare_optimizers writes the sim dir; interval_sweep *reconstructs that same path*
to reuse the main step's tasksets (`_main_step_dir` → `output_parent/tasks{N}_dur...`);
aggregate *scans* that location for `comparison_summary.csv`. So moving sims under
`runs/<run_id>/sim/` means all three writers/readers, plus the orchestrator, must
move together. The run dir is `build_run_id(cfg)` -- one per run -- so "where sims
go" is a function of cfg, computed once and threaded through.

**Design (approved):**
- [x] **P23.1** New helper `build_run_root(output_parent, cfg)` in
      `experiment_config_loader.py` → `os.path.join(output_parent, "runs",
      build_run_id(cfg))`. Sims derive as `<run_root>/sim/`, figures as
      `<run_root>/figures/`. Reuses `build_run_id` (no id-scheme change).
- [x] **P23.2** `compare_optimizers.py`: `resolve_run_output_dir()` gains a
      `run_root=None` param. When set, sims go to `<run_root>/sim/<subfolder>`;
      when None (standalone CLI), falls back to today's `<output_dir>/<subfolder>`
      so the module stays usable by itself. New `--run_root` CLI arg (default
      None) -- only the e2e orchestrator opts in.
- [x] **P23.3** `interval_sweep.py`: `_main_step_dir()` + `_main_dir_is_fresh()`
      + `run_single_interval()` take `run_root`; the sweep dir is written to
      `<run_root>/sim/tasks{N}_sweep_interval{I}_seed{S}`; fig2 →
      `<run_root>/figures/`. `main()` computes `run_root` from cfg. **No new CLI
      arg** -- sweep derives `run_root` internally (it already loads cfg).
- [x] **P23.4** `aggregate_across_tasks.py`: `aggregate_data_from_directories()`
      scans `<run_root>/sim/` when cfg-scoped (legacy unscoped branch keeps
      scanning `output_parent`). `generate_distribution_boxplot()` (Fig 1F, reads
      the module-global `OPTIMIZER_COMPARISON_DIR`) gets `run_root` threaded from
      `main()`. `main()` computes `run_root`, sets `figures_dir = <run_root>/figures`,
      updates the empty-data message to point at `<run_root>/sim/`. Module globals
      stay as defaults for the unscoped/standalone fallback path.
- [x] **P23.5** Copy the driving config JSON into the run root in
      `aggregate.main()` (last stage, run root guaranteed to exist, read-only
      otherwise so a copy is benign): `shutil.copy2(cfg["_config_source_path"],
      <run_root>/config.json)`, skipping if already there.
- [x] **P23.6** Orchestrator `run_end_to_end_experiments.py`: compute `run_root`
      once in `main()`; pass `--run_root` to the simulate command only (sweep +
      aggregate derive it from cfg internally -- no new args). Final figures-dir
      print uses `<run_root>/figures`.
- [x] **P23.7** Tests: `test_compare_optimizers.py` (run_root path variant),
      `test_interval_sweep.py` (reuse test is critical -- fresh main dir must be
      written where `_main_step_dir` now looks, under `sim/`), `test_aggregate.py`
      (build dirs under `sim/`, thread `run_root` for Fig 1F),
      `test_run_end_to_end.py` (assert `--run_root` present in simulate cmd).

**Legacy / old data:** old top-level sim dirs (`tasks4/6/8/10_dur70_...`,
`tasks*_sweep_...`, `tasks1_dur300_...`, `tasks*_dur600_...`) are **left in place**
-- not migrated. Once aggregate's scan root moves to `<run_root>/sim/`, those become
invisible to new runs (desired clean per-run isolation); user deletes by hand.

**Verification:** unit tests green + `--dry_run` shows `--run_root`; real smoke run
with the INCR config confirms `runs/<run_id>/{config.json,figures/,sim/tasks...}`;
re-run reuses the same run dir + tasksets (P20 reuse still finds them under `sim/`).

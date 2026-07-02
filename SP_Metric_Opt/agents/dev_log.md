# Development Log

## 2026-07-01

### Investigation: Does mean SP grow with N? + SP normalization design (P12)

User asked to investigate whether mean SP increases as the number of tasks per
task set increases (their hypothesis: more tasks → higher SP upper bound → mean
SP trends up), and to add a config option to normalize SP to `[0,1]` per
task-set size for fair cross-N comparison.

**SP-metric definition (traced through C++):**
- `SP_Func(violate_prob, threshold)` (`SP_Metric.h:31-41`) interpolates from
  `PenaltyFunc(1,thr) → 0` to `RewardFunc(0,thr) → 1`, so `SP_Func ∈ [0, 1]`.
- `ObtainSP_TaskSet` (`SP_Metric.cpp:53-68`) = `Σ_i SP_Func(ddl_miss_i, thr_i)
  × weights_node_i × perf_coefficient_i`. Node term only — the chain/path term
  lives in `ObtainSP_DAG`, but the simulation exports
  `ObtainSP_TaskSet_And_TimeLimits` (node-only) at
  `SimulationOrchestrator.cpp:482,703`, so realized SP has no path term.
- `perf_coefficient = 1.0` when `timePerformancePairs` is empty
  (`RegularTasks.h:83-89`); confirmed generated tasksets have **no**
  `timePerformancePairs` and **no** `chains`.

**Ceiling is constant across N (does not grow):** the generator normalizes
`sp_weight` so `Σ sp_weight = SP_WEIGHTS_SUM = 5.0` for every task set,
regardless of N (`taskset_generator.py:475-480`). Verified on disk: N=4 →
1.25×4 = 5.0; N=6 → 0.714×5 + 1.428×1 = 5.0. Therefore the theoretical SP
upper bound per interval ≈ 5.0 × 1.0 = 5.0, independent of N. The realized
fraction of that ceiling **decreases** with N: on the dur70 test data, BF (the
best a scheduler can reach) achieves ~3.80 (≈0.76 of ceiling) at N=4 but only
~2.95 (≈0.59) at N=6 — more tasks = more contention = harder to meet all
deadlines.

**Conclusion:** the user's hypothesis (ceiling grows with N) is **not** how
this generator behaves — the weight sum is actively normalized to a constant,
so the ceiling is flat; realized SP *falls* with N. Because both the ceiling
and a strong reference (BF) are flat/known, two valid normalizations preserve
"fraction of best achievable" semantics: divide by BF's SP at each N
(`reference_scheduler`, the implemented **default** — direct optimality gap),
or divide by the flat ~5.0 ceiling (`upper_bound`). min-max per N is available
as a fallback. See the P12 implementation entry below.

### Implementation: SP normalization config + Fig 1A-normalized variant (P12)

Added a normalization layer in the **aggregation** stage (raw values still
stored in the CSV; normalization is a plotting/reporting transform, not a
re-simulation):

- `configs/experiment_config.json` → new `analysis` keys:
  - `normalize_sp` (bool, default `false`): master toggle.
  - `sp_normalization_method` (`"upper_bound"` | `"minmax_per_task_count"`,
    default `"upper_bound"`): scheme selector.
- `aggregate_across_tasks.py`:
  - `compute_sp_upper_bound(experiment_dir)` — reads
    `taskset_characteristics_interval_0.yaml` from a representative taskset
    in the experiment dir, returns `Σ sp_weight × perf_coefficient`
    (perf_coefficient defaults to 1.0 since generated tasksets carry no
    `timePerformancePairs`). Falls back to `None` if no YAML is found.
  - `normalize_records_sp(records, cfg, ...)` — per task count, divides
    `mean_sp`/`std_sp` by the per-N upper bound (`upper_bound` method) or
    applies min-max across schedulers at fixed N (`minmax_per_task_count`).
    Returns a **new** record list; raw records are untouched. Handles the
    degenerate all-equal / zero-ceiling case (returns the raw value).
  - When `normalize_sp` is on, `generate_main_group_figures` and
    `generate_ablation_group_figures` emit an **additional** normalized Fig 1A
    / ablation SP figure (`fig1a_mean_sp_normalized_vs_tasks_main`,
    `fig_ablation_mean_sp_normalized_vs_tasks`) alongside the raw figure, so
    both views are available for the paper.

### Implementation: ET figure log-scale Y axis (P11)

`build_line_chart` now accepts a `log_y` kwarg. The mean-execution-time
figures (Fig 1C main, Ab-B ablation) pass `log_y=True` → `ax.set_yscale("log")`.
SP/miss-rate figures remain linear.

The Y-axis tick labeling was rewritten after the original `ScalarFormatter`
approach produced squeezed-together, unreadable labels on real data (N=6
spans 0.026s…1.62s, ~2 orders of magnitude). Two defects were fixed:

1. **Minor ticks were labeled.** On a log axis matplotlib emits minor ticks at
   every integer 2×..9× within each decade; labeling all of them produces a
   wall of overlapping numbers. Fix: only the **major** (decade) ticks are
   labeled; minor ticks get gridlines but `set_minor_formatter` is never
   called.
2. **`ScalarFormatter` collapses small decades to "0.00".** It picks one fixed
   decimal place for the whole axis, so once the range includes large values
   (1.62) the sub-0.01 decades all render as `0.00` → a stack of identical
   "0.00" labels. Fix: replaced `ScalarFormatter` with a `FuncFormatter` using
   `f"{val:g}"`, which formats each decade tick on its own scale
   (`0.0001`, `0.001`, `0.01`, `0.1`, `1`), all distinct plain numbers.

Verified on the on-disk N=4/N=6 dur70 data: major labels now
`0.0001 / 0.001 / 0.01 / 0.1 / 1`, minor ticks unlabeled. Test
`test_log_y_sets_log_scale` extended to assert `set_major_formatter` is called
once and `set_minor_formatter` is never called. Full suite 193 passing.

### Implementation: reference_scheduler normalization + extend to all SP figures (P12, cont.)

Two follow-ups to the P12 normalization work above (which only added a
normalized **Fig 1A** / ablation variant and documented two methods with
`upper_bound` as default). Both are superseded by this session.

#### (1) New default method: `reference_scheduler`

`normalize_records_sp` gained a third method, now the default: divide each
scheduler's SP by the **reference scheduler's** SP at the same task count
(`analysis.sp_normalization_reference`, default `BF`). BF is brute-force
enumeration over all priority assignments + time limits — the best a
scheduler with unlimited computation could reach — so it plots at 1.0 and
every other scheduler shows its optimality gap (≤ 1.0). This is more
meaningful for the paper than `upper_bound` (which divides by the flat
~5.0 weight-sum ceiling) because it answers "how close to optimal is each
method" directly. `upper_bound` and `minmax_per_task_count` remain
selectable. Config: `normalize_sp: true`,
`sp_normalization_method: "reference_scheduler"`, `sp_normalization_reference: "BF"`.

#### (2) Normalize every SP-metric figure, consistently

User feedback: "extend normalization to all SP-metric figures, applied
consistently." Previously only Fig 1A and Ab-A got a normalized variant.
Now every figure whose y-axis is an SP value emits a normalized variant
using the **same** reference-scheduler transform, computed from each
figure's own data (no shared state, no signature churn):

- **Fig 1A** (`fig1a_mean_sp_normalized_vs_tasks_main`) — mean SP, main group.
- **Fig 1B** (`fig1b_std_sp_normalized_vs_tasks_main`, debug) — std SP; the
  `norm_records` computed once for 1A is reused (its `std_sp` is already
  scaled by the per-N reference divisor).
- **Fig 1F** (`fig1f_sp_distribution_boxplot_normalized`) — SP distribution
  boxplot at a fixed task count; divides every raw SP value by BF's mean
  SP at that count so BF centers at 1.0. Factored out a `_draw_sp_boxplot`
  helper so raw + normalized share drawing code. Skips cleanly if the
  reference scheduler is absent.
- **Fig Ab-A** (`fig_ablation_mean_sp_normalized_vs_tasks`) — ablation mean SP.
- **Fig 2** (`fig2_sp_vs_interval_normalized`) — INCR SP vs. trigger interval;
  `interval_sweep.generate_figure_2` now emits a normalized variant where each
  scheduler's mean/std SP at interval *i* is divided by BF's mean SP at *i*
  (BF reference line → 1.0). Factored out `_draw_figure_2` +
  `_normalize_sweep_data` helpers.

#### Cleanup

- Removed a no-op ternary in `compute_sp_upper_bound`:
  `perf_coeff = 1.0 if not has_perf_pairs else 1.0` (both branches 1.0) →
  plain `perf_coeff = 1.0` with a clarifying comment.
- Module docstrings in `aggregate_across_tasks.py` and `interval_sweep.py`
  list the new normalized outputs; the config `_comment_normalize_sp` reflects
  the new default + full figure coverage.

#### Verification

- `python3 -m pytest tests/python/` → **195 passed** (+8 new: main-group
  1A/1B normalized call-count + stems; 1F normalized emitted / skipped on
  missing reference; new `test_interval_sweep.py` with Fig 2 normalization +
  `_normalize_sweep_data` unit tests).
- `python3 -m simulation_experiments.aggregate_across_tasks --mode test`
  against existing test-mode sim data → all normalized figures generated
  (1A, 1B, 1F, Ab-A). Fig 2 normalized verified separately against the
  existing sweep CSVs. 15 PNGs (+ matching PDFs) total in the run figures dir.

### Correction: normalize SP by the ideal (theoretical-ceiling) SP, not BF's value (P12 rework)

User feedback: "after normalization, 1.0 SP implies the best possible results
if we have infinite computation resources (i.e., all tasks meet deadline
perfectly, all task config optimization uses the best possible option).
Therefore, during normalization, we should divide raw SP by this ideal SP
value. Normalization has nothing to do with BF's value."

This overturns the `reference_scheduler` default adopted earlier this session
(§"reference_scheduler normalization + extend to all SP figures" above). That
approach was wrong on two levels:

1. **BF is a realized value, not a ceiling.** BF = brute-force enumeration over
   priority assignments + time limits is the best *single scheduling run*, but
   its mean SP is a number another scheduler can tie or exceed — on the dur70
   test data INCR ties BF at N=4 (both 3.7972). SP is not bounded above by BF,
   so dividing by BF does not bound the ratio at 1.0.
2. **Boxplot divide-by-mean produced values > 1.0.** Fig 1F divided each
   per-interval SP point by BF's *mean*. A boxplot's upper whisker naturally
   exceeds the mean, so points landed at **1.30 (N=4) / 1.11 (N=6)** — the
   "1.2" the user spotted. This is a category error: a per-point value was
   normalized by a per-set statistic.

**Correct divisor — the ideal SP:** with infinite computation every task meets
its deadline perfectly (`SP_Func = 1` for all tasks) and the optimizer picks
the best config, so

    ideal_SP = Σ_i sp_weight_i × perf_coefficient_i

This is exactly `compute_sp_upper_bound(experiment_dir)` (reads
`taskset_characteristics_interval_0.yaml`). Because `SP_Func ∈ [0,1]` with
non-negative weights, **raw SP ≤ ideal_SP always**, so `raw / ideal ∈ [0, 1]`
with 1.0 = "all deadlines met perfectly". The generator normalizes
`Σ sp_weight = 5.0` per task set, so `ideal_SP = 5.0` (flat across N) — the
denominator is a *constant ceiling*, not a per-scheduler mean, so no figure can
exceed 1.0 and the boxplot upper whiskers stay in range. This also makes the
cross-N comparison honest: N=4 BF ≈ 3.80/5.0 = 0.76, N=6 BF ≈ 2.95/5.0 = 0.59
— the realized fraction of the ideal, falling with N as contention rises.

**Implementation (applied):**
- Made `upper_bound` (ideal-SP) the **default and primary** method. This is
  the only method that satisfies "1.0 = best possible with infinite
  computation".
- **Removed `reference_scheduler`** + the `sp_normalization_reference` config
  key — it conflates a realized baseline with the theoretical ideal and is the
  source of the >1.0 bug. Kept `minmax_per_task_count` as a non-default debug
  option, clearly labeled as relative-ranking (not fraction-of-ideal).
- All SP figures (1A, 1B, 1F, Ab-A, Fig 2) divide by the same per-N `ideal_SP`
  (constant ~5.0). The boxplot divides each point by the ceiling (not BF's
  mean), fixing the >1.0 whiskers. `interval_sweep.generate_figure_2` now
  takes an `ideal_sp` arg (computed from any sweep experiment dir via
  `compute_sp_upper_bound`), and `_normalize_sweep_data(data, ideal_sp)`
  divides by that constant.
- Updated config `_comment_normalize_sp`, module docstrings, and tests.

**Verification:**
- `python3 -m pytest tests/python/` → **193 passed** (was 195; net −2 from
  consolidating 4 `reference_scheduler` unit tests into 2 `upper_bound`-default
  tests; sweep tests gained `test_normalized_skipped_when_no_ceiling`).
- Regenerated all figures on the on-disk dur70 test data; recomputed the max
  normalized boxplot point: **0.99 (N=4) / 0.66 (N=6)** — both ≤ 1.0, where
  1.0 = theoretical ideal (all deadlines met). The previous 1.30/1.11 are
  gone. `ideal_SP` ceiling verified = 5.0 on both N=4 and N=6.



User feedback: "code refactor, remove repeated code. example: run_sim_4 6 8 tasks.sh"

The `scripts/` directory had grown three byte-identical clones
(`run_sim_{4,6,8}_tasks.sh`, differing only by `NUM_TASKS=N`) plus copy-pasted
boilerplate (SCRIPT_DIR/PROJECT_ROOT resolution, the `====` header/footer
banners, the binary-existence guard, and the INT/TERM `kill 0` trap) across all
five runner scripts.

#### (1) New sourced library: `scripts/lib/common.sh`

Centralizes the shared pieces so each runner sources it instead of duplicating:
- `set -euo pipefail` + `SCRIPT_DIR`/`PROJECT_ROOT` resolution (caller sets
  `SCRIPT_DIR`, the lib resolves `PROJECT_ROOT`).
- `print_header "Title" "Key: val" ...` / `print_footer ["extra line"]`.
- `require_binary "${SIM_BIN}"` — the RunOrchestrator guard.
- `register_signal_trap` — installs the Ctrl-C/TERM `kill 0` trap.

#### (2) Collapsed `run_sim_{4,6,8}_tasks.sh` into `run_simulation.sh`

Deleted the three clones. `run_simulation.sh` now accepts an optional
**positional task-count** that overrides `NUM_TASKS`, so:
```bash
./run_simulation.sh 4          # replaces ./run_sim_4_tasks.sh
NUM_TASKSETS=2 ./run_simulation.sh 6
```
The `^[468]$` regex validation is unchanged. `run_all_experiments.sh` now loops
`NUM_TASKS=N ./run_simulation.sh` for 4/6/8 instead of calling the deleted
wrappers.

#### (3) Remaining runners source `common.sh`

`run_all_experiments.sh`, `run_interval_sweep.sh`, `run_paper_figures.sh`, and
`run_end_to_end.sh` each reduced to: source lib → set local vars →
`print_header` → run python module → `print_footer`. The duplicated header
banner, binary guard, and trap blocks were removed in favor of the shared
helpers. `run_end_to_end.sh` keeps its dry-run-skips-binary-guard behavior.

#### Verification

- `bash -n` passes on all six files (`common.sh` + five runners).
- `./run_simulation.sh 4` resolves the positional arg and launches correctly.
- `DRY_RUN=1 ./scripts/run_end_to_end.sh` (test mode): header renders via
  `common.sh`, orchestrator builds the correct simulate→sweep→aggregate commands
  from config, dry-run respected.
- `./scripts/run_all_experiments.sh` chains into `run_simulation.sh` with
  `NUM_TASKS=4` after printing its header.
- `python3 -m pytest tests/python/` → **171 passed** (no regressions; no test or
  Python module referenced the deleted wrappers).
- All five runners keep `+x`.

---

### Session Summary: Per-run figure namespacing, bar→line, and always PNG+PDF export

Three related changes to the simulation-experiment figure pipeline, all verified
with a real `./scripts/run_end_to_end.sh` test-mode run (exit 0, ~26–36s) and the
full Python suite (**171 passed**).

#### (1) Group figures by run-time config (fixes cross-run clobbering + aggregation pollution)

User feedback: every run's figures were dumped into one flat folder
(`optimizer_comparison/figures/`), so test-mode and prod-mode (or runs with
different durations/seeds) silently clobbered each other's `fig1a…fig3b` files.
The dev-log aggregation bug (stale `tasks6_dur300_*` dirs ingested with
last-write-wins, yielding 23 records instead of 14) was the data-side symptom.

- **`simulation_experiments/experiment_config_loader.py`** — added
  `build_run_id(cfg)` → e.g. `run_test_dur70_interval10_seed1000_tasks4x6`.
  Keyed on mode + duration + trigger interval + base seed + task-count list;
  `num_tasksets` intentionally excluded (sampling depth, not run identity).
- **`simulation_experiments/aggregate_across_tasks.py`** —
  `aggregate_data_from_directories(cfg, output_parent)` now scopes ingestion to
  dirs matching `tasks{N}_dur{D}_interval{I}_seed{S}` for the current run,
  excluding both `*_sweep_*` dirs and stale dirs with a different dur/interval/
  seed. `main()` writes figures to `runs/<run_id>/figures/`. Added `--output_parent`
  CLI; `generate_*` take an optional `figures_dir`. Legacy no-`cfg` path preserved.
- **`simulation_experiments/interval_sweep.py`** — fig2 written under
  `runs/<run_id>/figures/`.
- **`simulation_experiments/run_end_to_end_experiments.py`** —
  `build_aggregate_command(cfg, output_parent)` appends `--output_parent`; final
  "Figures:" print shows `runs/<run_id>/figures`.
- New test `test_aggregation_excludes_stale_dirs` in `tests/python/test_aggregate.py`
  locks in the scoping (scoped → only dur70 record; legacy → both).

Real run confirmed: **14 records** (2 task counts × 7 schedulers), figures land
in `runs/run_test_dur70_interval10_seed1000_tasks4x6/figures/`, flat `figures/`
no longer written by the pipeline.

#### (2) No bar plots — line plots everywhere

User feedback: "no bar plots anywhere — line plots always."
- **`aggregate_across_tasks.py`** — `build_grouped_bar_chart` → `build_line_chart`
  (one line per scheduler over num_tasks, `ax.errorbar` with markers); fig3/3b
  converted to connected-point lines over scheduler index.
- **`compare_optimizers.py`** — `plot_optimizer_bar_comparison` →
  `plot_optimizer_sp_line`, `plot_optimizer_exec_time` →
  `plot_optimizer_exec_time_line`, `plot_per_taskset_radar` →
  `plot_per_taskset_line` (all bar→line).
- Box plots (Fig 1F, `comparison_plots` boxplot panel) kept — not bar plots.
- Figure filenames unchanged to limit churn.

#### (3) Always export both PNG and PDF

User feedback: "figures, export both pdf and png figures, always."
The paper-figure path (`save_figure`, default `["png","pdf"]`) already did both;
the gap was the **per-run debug plots**, which used raw `plt.savefig(..., dpi=300)`
with hardcoded `.png` names and never produced PDF.
- **`compare_optimizers.py`** — the three per-run plot fns now route through
  `save_figure()` (strips a trailing `.png` to form the stem, so call sites like
  `optimizer_sp_bar.png` are unchanged and now also emit `.pdf`).
- **`utils.py`** — `write_summary_and_plots` uses `save_figure(fig, "comparison_plots")`
  (PNG+PDF) instead of `plt.savefig`.
- Tests updated to mock `save_figure` (one call with the stem) instead of
  `plt.savefig` (one `.png` call): `test_compare_optimizers.py` (3 tests),
  `test_run_sim_experiments.py::test_write_summary_and_plots`.

Real run confirmed: per-run plots (`optimizer_sp_bar`, `optimizer_exec_time`,
`optimizer_per_taskset`, `comparison_plots`) each now have `.png` **and** `.pdf`;
11 paper figures = 11 PNG + 11 PDF.

---

### Session Summary: Unified End-to-End Orchestrator (P10)

User feedback: "when I say a script that runs end-to-end, I mean one script that does everything, rather than manually doing step 1 2 3 4 ... one script that has all those things based on simulation_experiments/configs/experiment_config.json."

Delivered a single entry point that runs the full pipeline from the JSON config, replacing the manual 3-step workflow.

#### New files

- **`simulation_experiments/run_end_to_end_experiments.py`** — orchestrator with three stages, each delegating to an existing module:
  - **simulate** → `compare_optimizers.py` once per task count in `num_tasks_for_cross_task_comparison`, passing the **union** of `main_scheduler_list` + `ablation_scheduler_list` (deduped, order-preserving) so a single simulation pass feeds both the main-group and ablation-group figures.
  - **sweep** → `interval_sweep.py` (Fig 2).
  - **aggregate** → `aggregate_across_tasks.py` (Figs 1A-1F, ablation, Fig 3).
  - Every experiment parameter is read from `experiment_config.json` via `load_experiment_config(mode)`. The only CLI flags are `--mode`, `--steps`, `--bin_dir`, `--verbose`, `--config_json`, `--dry_run` (no experiment knobs are hardcoded).
  - Stages run in fixed order; `--steps` selects a subset; a failed stage aborts the pipeline with non-zero exit. Pre-flight checks the binary exists when a simulating stage is selected.
  - Key helpers: `build_scheduler_union(cfg)`, `build_simulate_command(num_tasks, cfg, output_parent, verbose)`, `build_sweep_command(cfg, output_parent, verbose)`, `build_aggregate_command(cfg)`, `run_command(cmd, dry_run)`.

- **`scripts/run_end_to_end.sh`** — bash wrapper. `MODE` (default test), `STEPS`, `BIN_DIR` (default release), `VERBOSE`, `DRY_RUN`, `PYTHON` env vars; validates binary, traps Ctrl-C/TERM, prints header + timing, calls the Python orchestrator. Passes `bash -n`; executable (`+x`).

- **`tests/python/test_run_end_to_end.py`** — 16 tests: scheduler-union dedup/order, per-stage command construction (config values, `--resume`, `--num_workers`, important-task %), `run_command` dry-run/no-exec/failure semantics, and `main()` pipeline tests (all-stages / subset / simulate-only dry-run counts, failure-abort). All pass.

#### Correctness fix found while wiring the orchestrator

- **`simulation_experiments/aggregate_across_tasks.py`** — `aggregate_data_from_directories()` now skips directories whose name contains `sweep`. Previously, `interval_sweep.py` writes per-interval dirs named `tasks{N}_sweep_interval{I}_seed{S}` directly into `optimizer_comparison/`, and the aggregator matched every `tasks(\d+)_*` dir — so sweep runs were ingested as cross-task data points and **overwrote the real bars** for that task count in Figs 1A-1E. The end-to-end flow now produces correct figures.

#### Verification (re-confirmed this session)

- `python3 -m pytest tests/python/` → **170 passed** (full suite, no regressions); orchestrator subset = 16 passed.
- `bash -n scripts/run_end_to_end.sh` → OK.
- `DRY_RUN=1 ./scripts/run_end_to_end.sh` (test mode) → 2 simulate cmds + sweep + aggregate, all from config; `MODE=prod` → 3 task counts + 6-interval sweep.
- `STEPS="simulate aggregate" ...` correctly skips the sweep (no STAGE 2); aggregate-only (`STEPS="aggregate"`) skips binary validation since no simulating stage is selected.

#### Usage

```bash
# One command, everything, fast smoke-test parameters:
DRY_RUN=1 ./scripts/run_end_to_end.sh                       # preview commands
./scripts/run_end_to_end.sh                                 # test mode, all stages
MODE=prod ./scripts/run_end_to_end.sh                       # paper-grade
STEPS="simulate aggregate" ./scripts/run_end_to_end.sh      # skip sweep
STEPS="aggregate" MODE=prod ./scripts/run_end_to_end.sh     # figures only, no sim
```

#### Design note

`--bin_dir` lives in the orchestrator CLI (and `scripts/run_end_to_end.sh`'s `BIN_DIR`) rather than in `experiment_config.json`, because the binary location is an environment/machine concern, not an experiment parameter. Everything else — task counts, durations, trigger intervals, scheduler lists, seeds, worker count, resume, important-task %, export level — is config-driven via `experiment_config.json`.

#### Post-fix: test-mode duration too short for hyper-period

A real (non-dry-run) `./scripts/run_end_to_end.sh` test-mode run failed at taskset generation:

```
ValueError: Total simulated time (30s = 30000ms) must be at least 2× the
hyper-period (66000ms). Hyper-period of periods [500, 200, 33, 20] = 33000ms.
Increase n_sec to >= 66s.
```

Root cause: `test_mode.simulation_duration_seconds` was `30`, but the taskset
generator (`Gen_Taskset/lib/orchestrator.py`) requires `n_sec ≥ 2 × hyper-period`.
Period pools (from `taskset_cfg_paper_base.json`, included by all `paper_*` configs):
big `[1000, 500, 200]` ms, small `[100, 50, 33, 20]` ms. The coprime pair
`1000` & `33` gives a worst-case hyper-period of **33 s** across all task counts
(4/6/8), so the generator needs `n_sec ≥ 66 s`. The `--dry_run` path couldn't
catch this because generation never runs in dry-run.

Fix: `simulation_experiments/configs/experiment_config.json` — bumped
`test_mode.simulation_duration_seconds` from `30` to `70` (smallest safe
multiple of 10 above 66; prod mode's `300` was already fine). Added an
inline `_comment` explaining the 66 s floor so it isn't lowered again.

Re-ran `./scripts/run_end_to_end.sh` (test mode): **completed in 53.1s**,
all 11 figures (1A-1F, ablation ×2, Fig 2, Fig 3/3b) generated as PNG+PDF,
no errors. Verified the union scheduler list (7 schedulers: INCR, BF, RM,
CFS, INCR_NO_TL, INCR_WCET, INCR_SCRATCH) flows through both main and
ablation figures from a single simulate pass.

#### Known issue: aggregator picks up stale experiment dirs (filed, not yet fixed)

The real test run also exposed a latent aggregation bug:
`aggregate_data_from_directories()` ingests **every** `tasks{N}_*` directory,
with last-write-wins per `(num_tasks, scheduler)`. Result dirs are named only
by `(num_tasks, duration, interval, seed)` — so runs with different
durations coexist and silently corrupt each other's bars. In this run the
6-task bars were polluted by a stale `tasks6_dur300` dir (9 schedulers,
incl. RM_FAST/RM_SLOW, from 2026-06-28) and a leftover `tasks4_dur30`
partial dir from the failed run — yielding 23 records instead of 14.

The sweep-dir exclusion (added earlier this session) handles the
`*_sweep_*` collision but not the `*_dur{D}_*` collision. Proper fix (not
yet applied): have the aggregator select, per task count, only the directory
matching the configured `simulation_duration_seconds` + `base_random_seed`
(rather than every matching dir), or scope aggregation to a specific
run-name prefix. For now, stale dirs were removed by hand before
re-aggregating.

---

### Session Summary: Simulation Experiment Pipeline Completion

Completed all remaining tasks (P5–P9) for the simulation experiment pipeline. The system now supports full end-to-end generation, simulation, aggregation, and figure production for publication.

#### Configuration & Plotting Infrastructure

- **`simulation_experiments/configs/experiment_config.json`**
  - Central JSON config with `test_mode` / `prod_mode` sections.
  - `test_mode`: `[4, 6]` tasks, 2 tasksets, 70s duration (≥ 66s hyper-period floor; see P10 note), intervals `[5, 10]`.
  - `prod_mode`: `[4, 6, 8]` tasks, 10 tasksets, 300s duration, intervals `[1, 5, 10, 20, 30, 60]`.
  - Global `plotting` section: 14/16/18 pt fonts, 300 dpi, colorblind palette, PNG+PDF output.
  - Global `analysis` section: 10% important-task threshold, resume support.

- **`simulation_experiments/experiment_config_loader.py`**
  - `load_experiment_config(mode)` returns flattened dict merging mode-specific params with global settings.

- **`simulation_experiments/plotting_config.py`**
  - `setup_publication_style()` configures matplotlib rcParams.
  - `get_scheduler_color_map()` provides consistent colors per scheduler.
  - `save_figure()` writes PNG + PDF automatically.

#### P5 — Important-Task Miss Rate (Figure 3)

- `compute_miss_rate_by_task()` and `compute_important_task_miss_rate()` in `utils.py`.
- `analyze_single_instance()` returns `(important_miss_rate, non_important_miss_rate)`.
- `compare_optimizers.py` aggregates both metrics; `write_summary_and_plots()` writes them to `comparison_summary.csv`.
- `aggregate_across_tasks.py` generates Fig 3 (important-task miss rate) and Fig 3b (non-important, debug).

#### P6 — Trigger-Interval Sweep (Figure 2)

- **`simulation_experiments/interval_sweep.py`** — orchestrates multiple `compare_optimizers.py` runs, one per trigger interval from config.
- Generates Fig 2: X=interval (s), Y=mean SP, INCR with error bars, BF/RM as horizontal dashed reference lines.
- Supports `--mode test` (fast) and `--mode prod` (paper-grade).

#### P7 — Shell Scripts

Created 7 scripts in `scripts/` (note: the three `run_sim_{4,6,8}_tasks.sh`
wrappers below were later collapsed into `run_simulation.sh`'s positional
task-count arg — see the "code refactor" session later this day):

| Script | Purpose |
|---|---|
| `run_simulation.sh` | Core runner; accepts env-var overrides (`NUM_TASKS`, `NUM_TASKSETS`, `N_SEC`, `SCHEDULERS`, etc.). Validates binary, traps Ctrl-C, prints timing header. |
| `run_sim_4_tasks.sh` | Wrapper: `NUM_TASKS=4` |
| `run_sim_6_tasks.sh` | Wrapper: `NUM_TASKS=6` |
| `run_sim_8_tasks.sh` | Wrapper: `NUM_TASKS=8` |
| `run_all_experiments.sh` | Runs 4/6/8 sequentially, then calls `aggregate_across_tasks`. |
| `run_interval_sweep.sh` | Runs interval sweep standalone. |
| `run_paper_figures.sh` | Regenerates all figures from existing data (no simulation). |

All scripts pass `bash -n` and have executable permissions.

#### P8 — Tests

- **`tests/python/test_run_sim_experiments.py`** — 21 tests covering miss-rate computation (summary + trace fallbacks), `run_single_simulation` args, `analyze_single_instance` with exec time / effective intervals / important tasks, and `write_summary_and_plots` with the updated CSV header.
- **`tests/python/test_aggregate.py`** — 16+ tests for task-count parsing, summary loading, aggregation logic, grouped bar charts, main/ablation figure generation, Fig 3, distribution boxplot, and a full end-to-end integration test with mock data.
- **`tests/python/test_plotting.py`** — 12 tests for publication-style setup, color maps, figure saving in multiple formats, and synthetic figure generation (grouped bar, box plot, line chart with error bars, single bar chart).

---

## 2026-06-20

### Task 1: Fix opt_sp_ initialization bug
- Changed `opt_sp_` initialization from `0` to `-1` in `OptimizeFromScratch_w_TL` and `OptimizeIncre_w_TL` to properly support negative safety performance values (staged, awaiting commit).

### Task 2: Linear Coordinate Descent for Task Configuration Optimization
- Replaced the exponential recursive traversal logic in task execution time limit configuration with a linear coordinate descent algorithm.
- Implemented task sorting via `TaskSortingHeuristic` to process tasks in descending weight and ascending threshold priority order.
- Encapsulated initialization and coordinate descent logic in separate functions: `InitializeTimeLimitsFromETConfig` and `PerformCoordinateDescentForTaskConfigOpt`.
- Implemented a tie-breaker rule at both the search and record-update levels: if multiple configurations yield the same optimal SP metric, choose the one with the lower total execution time limit.
- Re-enabled original test cases (expecting 400ms limit under coordinate descent with tie-breaker).
- Designed and added the `OptimizeWithOptimizationSpace` test case showing three distinct limits for:
  - Default ET configuration (1000ms / 1500.93ms)
  - Incremental optimization (800ms)
  - Scratch optimization (600ms)
  And corresponding SP improvements (`SP_600 > SP_800 > SP_1000`).
- Verified all 15 test suites pass successfully.

---

## How to Test-Run the Pipeline (Test Mode)

### Prerequisites

1. **Compile the C++ release binary** (scripts validate this exists before running):
   ```bash
   # Release build — the scripts expect:
   #   release/tests/RunOrchestrator
   mkdir -p release && cd release && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) RunOrchestrator
   ```
   > Note: `build/` folder is reserved for debug builds. Production / paper runs always use `release/`.

2. **Python dependencies:**
   ```bash
   pip install matplotlib numpy pyyaml seaborn pytest
   ```

### Test-Run Levels

#### Level 1 — Unit Tests Only (No Simulation, ~5 seconds)
```bash
python3 -m pytest tests/python/test_run_sim_experiments.py -v
python3 -m pytest tests/python/test_aggregate.py -v
python3 -m pytest tests/python/test_plotting.py -v
```

#### Level 2 — Single Task Count Smoke Test (~1–3 minutes)
```bash
# Uses test defaults from experiment_config.json:
#   2 tasksets, 70s duration (≥66s hyper-period floor), 10s trigger interval
# Positional arg overrides NUM_TASKS (the old run_sim_{4,6,8}_tasks.sh wrappers
# were collapsed into run_simulation.sh):
NUM_TASKS=4 NUM_TASKSETS=2 N_SEC=70 ./scripts/run_simulation.sh 4
```
Or directly via Python:
```bash
python3 -m simulation_experiments.compare_optimizers \
    --num_tasks 4 --n_tasksets 2 --n_sec 70 \
    --schedulers INCR BF RM CFS
```

#### Level 3 — Full Test Mode End-to-End (one command)
```bash
# Runs simulate -> sweep -> aggregate, all from experiment_config.json:
./scripts/run_end_to_end.sh
```
Equivalently, the three stages manually:
```bash
NUM_TASKS=4 ./scripts/run_simulation.sh 4 && NUM_TASKS=6 ./scripts/run_simulation.sh 6
python3 -m simulation_experiments.aggregate_across_tasks --mode test
python3 -m simulation_experiments.interval_sweep --mode test
```

### Viewing Results
```bash
# Generated figures land under a run-id-scoped subdir (per-run namespacing,
# not the old flat figures/ dir):
ls simulation_experiments/optimizer_comparison/runs/run_test_dur70_interval10_seed1000_tasks4x6/figures/

# Summary CSV (includes Important_Miss_Rate column)
cat simulation_experiments/optimizer_comparison/tasks4_*/comparison_summary.csv
```

### Test vs. Production Mode Differences

| Parameter | Test Mode | Prod Mode |
|---|---|---|
| Task counts | `[4, 6]` | `[4, 6, 8]` |
| Tasksets | 2 | 10 |
| Duration | 30s | 300s |
| Sweep intervals | `[5, 10]` | `[1, 5, 10, 20, 30, 60]` |
| Export level | 1 | 1 |

---

## 2026-07-02

### Per-core CPU utilization fix + task-count ceiling removal (P13)

Two issues surfaced while reviewing `agents/gemini.md` (a staged-code review
note). Both are real; the user wants them as **two separate commits**, and the
4/6/8 task-count ceiling removed entirely (support 10/12/14/16/18 …).

#### Issue 1 -- `MEAN_CPU_UTIL` was not held constant per-core (Commit A)

`MEAN_CPU_UTIL` is **per-core** utilization by the generator's arithmetic:
`taskset_generator.py:241-242` does `cpu_util = cfgs['MEAN_CPU_UTIL'] *
n_cores`, then UUniFast distributes that total across the N tasks. So it
represents the **aggregated load state** and must stay constant regardless of
N or core count — exactly as the user stated: "mean cpu utilization is
per-core utilization, it remains the same no matter how many tasks or how many
cores."

The existing per-N configs violated this:

| N | old `MEAN_CPU_UTIL` | total (`×2`) | per-task (`/N`) |
|---|---|---|---|
| 4 | 0.9 | 1.8 | 0.450 |
| 6 | 1.2 | 2.4 | 0.300 |
| 8 | 1.6 | 3.2 | 0.225 |

They rose 0.9 → 1.2 → 1.6, i.e. they held **per-task ≈ 0.3–0.45** roughly
constant instead of per-core. That is the wrong semantics; the user was
correctly skeptical. Fix (Commit A, config-only): flatten all three to **0.9**
per-core (total 1.8, system 90 % loaded / under-subscribed; matches the
existing N=4 setting). `paper_4.json` already 0.9 (no change); `paper_6.json`
1.2 → 0.9; `paper_8.json` 1.6 → 0.9.

**Acceptance:** old 4/6/8 run data (SP values recorded below, the
`optimizer_comparison/` tasksets) becomes stale and would need regeneration —
user accepts this. The 0.9 value is itself tunable later as a separate edit;
duplicate periods at large N are OK (period pool `[10,20,30,50]` Hz unchanged).

#### Issue 2 -- 4/6/8 task-count ceiling crashes prod N=10 (Commit B)

`experiment_config.json` prod declares
`num_tasks_for_cross_task_comparison: [4, 6, 8, 10]`, but the ceiling is
enforced in two places (both must be removed):

1. **CLI** `choices=[4, 6, 8]` on `--num_tasks` in `compare_optimizers.py:257`
   and `run_sim_experiments.py:260` → N=10 fails argparse validation, aborting
   the pipeline (`subprocess.run(..., check=True)` → `CalledProcessError`).
2. **Named-file lookup** `f".../taskset_cfg_paper_{N}.json"` at
   `compare_optimizers.py:342-345` and `run_sim_experiments.py:311-314` — only
   `paper_{4,6,8}.json` exist, so even without the CLI gate, N=10 has no config
   file. (This is the root cause; the user objected to the hard-coded value at
   `run_sim_experiments.py:313` and asked to "support it" rather than
   pre-create files.)

**Fix (Commit B):**
- **B1** Add `resolve_taskset_config_path(num_tasks, config_dir=None)` to
  `generation_config_parser.py`: returns the existing `paper_{N}.json` if
  present, else synthesizes a thin temp override file (same shape as the
  on-disk ones: `INCLUDE` base + `N_BIG=2`, `N_SMALL=N-2`, `MEAN_CPU_UTIL=0.9`,
  `N_CORES=2`, `RANDOM_SEED=42`). Downstream `load_generation_config(path)`
  then resolves INCLUDE exactly as for on-disk files — zero new code paths in
  the generator. The 0.9 constant lives in one place here for synthesized
  configs; on-disk 4/6/8 carry their own (Commit A); the two are kept in sync.
- **B2** Replace the hard-coded lookup at both call sites with the resolver.
  **Latent bug fix:** `run_sim_experiments.py:373-374` used raw `json.load`
  (does **not** resolve INCLUDE), unlike `compare_optimizers.py:411` which uses
  `load_generation_config`. It worked today only because `paper_6.json`'s
  INCLUDE base params happen to be optional/defaulted. Switch it to
  `load_generation_config` — required for correctness once synthesized configs
  (which rely on INCLUDE) are in play.
- **B3** Drop `choices=[4,6,8]` at both CLIs; add a floor guard
  (`num_tasks >= 2`, matching fixed `N_BIG_PERIOD_TASKS=2`). `interval_sweep.py`
  already has `default=None` with no `choices` and forwards `--num_tasks`
  downstream — unchanged.
- **B4** Extend prod task list `[4,6,8,10]` → `[4,6,8,10,12,14,16,18]`. Test
  mode stays `[4, 6]`.
- **B5** Tests for the resolver (existing 4/6/8 unchanged; N=10/14/18
  synthesizes a valid INCLUDE-resolved config; N<2 rejected) + CLI acceptance
  of `--num_tasks 10`.

**Note on period pool:** `SMALL_PERIOD_HZ=[10,20,30,50]` (4 distinct) with
`N_SMALL_PERIOD_TASKS=N-2` means any N>6 forces duplicate small periods
(N=8 already has 2 dups; N=18 → 12 dups). `pick_period` tries to avoid dups
then falls back to allowing them. Duplicate periods are acceptable per the
user — pool unchanged.

See [[taskset-config-architecture]] (memory) for the full N-flow diagram.

### Unified period pool: drop the big/small split (P19)

Follow-on to P13/P16/P17. The paired `BIG_PERIOD_HZ` / `SMALL_PERIOD_HZ` (+ the
`N_BIG_PERIOD_TASKS` / `N_SMALL_PERIOD_TASKS` counts) schema was an Hz-era
artifact: periods are ms everywhere downstream, the split only added a
`pick_period` branch, an ad-hoc <10 Hz / >=10 Hz split for the legacy single
`HZ` list, and a fixed `N_BIG=2` hardcode that fought the P13/P17 count
relaxation. Collapsed to one pool + one count.

**Canonical schema (the only accepted period/count inputs):**
- `PERIODS_MS` (list[int]) — period pool every task draws from, in ms.
- `N_TASKS` (int) — total task count; each task draws one period from the pool.
  Pool exhaustion at large N allows duplicate periods (`pick_period` avoids dups
  then falls back) — acceptable per the P13 note.

**Landed in two phases:**

- **Phase 1 — `a0cc8021` ("hz to period ms, merge big and small periods into
  single period list").** Migrated source (`generation_config_parser.py`,
  `taskset_generator.py`), every shipped config/template (`paper_base`,
  `paper_{4,6,8}`, `test_standard_{4,6,8}`), and `debug_uunifast.py` to
  `PERIODS_MS` + `N_TASKS`. `standardize_config` kept a **backward-compat
  alias** so old-shape configs kept working: built `PERIODS_MS` = big-pool
  periods + small-pool periods and `N_TASKS = N_BIG + N_SMALL`, then deleted
  the old keys. `pick_period(cfgs, picked_periods)` dropped its `prd_sel` arg
  (one pool, no branch). `resolve_taskset_config_path` synthesized configs set
  `N_TASKS`; N=1 still drops to `N_CORES=1` for schedulability (lone task's
  util stays < 1.0; with N_CORES=2 it would carry 1.8 → unschedulable).

- **Phase 2 — this change (alias removal).** The backward-compat alias was a
  transitional bridge; now removed. Legacy period keys
  (`HZ`, `SMALL_PERIOD_HZ`, `BIG_PERIOD_HZ`, `SMALL_PERIODS_MS`,
  `BIG_PERIODS_MS`) and count keys (`N_BIG_PERIOD_TASKS`,
  `N_SMALL_PERIOD_TASKS`) raise `ValueError` pointing at the canonical
  replacement instead of being silently aliased. Nothing in-tree breaks — every
  shipped config already sets the canonical keys. A bare config (no period/
  count info) defaults to the base template's pool
  (`[1000, 500, 200, 100, 50, 33, 20]`) and `N_TASKS=10`.

**Tests:** `TestP17ZeroCountTasksets` rewritten as `TestP19UnifiedPoolTasksets`
(5 cases: unified-pool sourcing, pool-exhaustion duplicates, N=1 single-rate,
legacy-keys-rejected, N_TASKS<1-rejected). Legacy-aliasing tests in
`test_generation_config_parser.py`, `Gen_Taskset/tests/test_generation_config.py`,
and `Gen_Taskset/tests/test_specifications.py` flipped to assert `ValueError`.
Remaining test files migrated to canonical keys in-place. Full suite
**238 passing** (`tests/python/` 224 + `Gen_Taskset/tests/` 14).

### Random per-core CPU utilization range (P14)

Follow-on to P13. P13 held per-core `MEAN_CPU_UTIL` constant at 0.9 so every
task set exercises a single load point (90 % loaded / under-subscribed). P14
makes that load **sampleable per task set**: when a new optional
`CPU_UTIL_RANDOM_RANGE: [low, high]` key is present, the generator samples the
per-core utilization uniformly from `[low, high]` for each task set, then runs
the existing UUniFast distribution against the sampled total
(`cpu_util = sampled_per_core × N_CORES`). One experiment run therefore sweeps
under-subscribed → over-subscribed systems, giving a distribution of SP
behavior over load rather than a point estimate. P13's per-core semantics are
preserved — only the fixed *value* becomes a *sampled range*.

**Supersedes the P13 out-of-scope note.** P13's acceptance line said "the 0.9
value is itself tunable later as a separate edit." P14 is that edit: the
tunable knob is now the **range** (`CPU_UTIL_RANDOM_RANGE`), not a scalar. P13's
fixed 0.9 remains the default load when the range is absent — P14 is strictly
opt-in (see below).

**Design decisions (resolving the P14.1 open question):**
- *Range supersedes or falls back?* — **fallback** (option (b) in the task
  spec) for backward compat. When `CPU_UTIL_RANDOM_RANGE` is present it is
  used; when absent, the generator takes the existing fixed-`MEAN_CPU_UTIL`
  path unchanged. Old P13 configs without the range keep working verbatim.
- *Where does the key live?* — **opt-in, NOT in the base template.** P14.5
  requires "no range config → 0.9" as the default. Adding the key to
  `taskset_cfg_paper_base.json` would turn sampling ON for every paper config
  via INCLUDE, contradicting P14.5. So the key is recognized by the parser/
  generator only when a config sets it explicitly; no shipped config sets it.
  Enabling sampling for the paper experiments is a one-line config edit (add
  `"CPU_UTIL_RANDOM_RANGE": [0.5, 1.5]` to the desired `paper_*.json`).

**Implementation:**
- **P14.1** `generation_config_parser.standardize_config`: validates
  `CPU_UTIL_RANDOM_RANGE` shape when present — must be a `[low, high]` pair of
  numbers with `0 <= low <= high`; coerced to floats. Rejected with
  `ValueError` otherwise (wrong arity, non-numeric, `low > high`, `low < 0`,
  bool). The key is not synthesized when absent (opt-in).
- **P14.2** `taskset_generator.generate_taskset_parameters` (~line 248):
  replaced `cpu_util = cfgs['MEAN_CPU_UTIL'] * n_cores` with a branch — if the
  range is present, `per_core_cpu_util = random.uniform(low, high)`; else
  `per_core_cpu_util = cfgs['MEAN_CPU_UTIL']`. Then `cpu_util = per_core_cpu_util
  × n_cores` in both paths.
- **P14.3** Reproducibility + inspection: the sampled value is the **first
  draw** off the seeded global RNG (`random.seed(RANDOM_SEED)` runs immediately
  before, at ~line 242), so a given `RANDOM_SEED` reproduces the same sampled
  per-core util. The realized `per_core_cpu_util` is recorded in the returned
  dict and lands in `taskset_param.yaml` (top-level, alongside `cpu_util`) so
  each task set's load is inspectable. (Existing readers use
  `yaml.safe_load` + named-key access, so the new key is additive — no reader
  changes.) Enabling the range shifts all downstream random draws (it consumes
  one RNG value), so task sets generated with the range are not byte-identical
  to the fixed-0.9 ones under the same seed — inherent to the design and
  documented in the task spec.
- **P14.5** P13's 0.9 stays the default; P14 is opt-in. No P13 config removed.
  This entry records that P14 supersedes P13's "0.9 is itself tunable later"
  note (above).

**Tests** (`tests/python/`):
- `test_taskset_generator.py::TestP14RandomCpuUtilRange` (5): sampled util ∈
  `[low, high]`; `cpu_util = per_core × N_CORES` (realized per-task sum within
  a tolerance — the pre-existing `max(1.0, u_i×period)` floor on small-period
  tasks can push the realized sum slightly above the target); deterministic
  under a fixed seed; varies across 30 seeds; range-absent falls back to fixed
  0.9. The class snapshots/restores global `random` + `np.random` state in
  setUp/tearDown so its many seeded generations don't leak RNG state into later
  modules (surfaced a pre-existing latent flake in
  `test_trajectory.py::test_generate_path_only`, which reads global RNG
  unseeded via `generate_path_only`).
- `test_generation_config_parser.py` (+3): valid range accepted + normalized;
  malformed ranges rejected; range absent by default (opt-in).

Full suite **253 passing** (was 245 at HEAD; +8 P14 tests).


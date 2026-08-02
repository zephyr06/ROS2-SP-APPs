# P0.3 — Run Prod Pipeline + Generate Core Figures

**Priority:** P0 (THE publication deliverable)
**Status:** not started (records synced 2026-08-02; see `dev_log.md`)
**Depends on:** P0.1 — RESOLVED 2026-07-10 (`7fa2e9d2`; subsumed by P0.5)

## Goal

Run the full prod pipeline end-to-end and produce the core figure set under
`optimizer_comparison/runs/<run_id>/figures/`. `optimizer_comparison/` is absent
in a clean tree and is created on the first run.

## Entry point

```
MODE=prod ./scripts/run_simulation_plot_eval_ns.sh
```

Default config: `simulation_experiments/configs/paper_simulation_config.json`
(override with `CONFIG_JSON=<path>`). The old `run_simulation_and_plot_figures.sh`
was folded into this single entry point by P2.8 (experiments differ by
`CONFIG_JSON`, not by a new `.sh`).

## Pipeline stages (fixed order; a failed stage aborts the whole run)

build → patch `parameters.yaml` → simulate → sweep → aggregate (→ evaluate
unless `SKIP_EVAL=1`). Stages 3-5 are `run_end_to_end_experiments`'s fixed
simulate→sweep→aggregate; there is no `--steps` override. The sweep is SKIPPED
only when `num_tasks_for_single_task_figures == 0` (it is 10 in prod → sweep
runs, and a sweep failure would abort before aggregate).

## Core figure set (must-have for paper)

With `normalize_sp: true` (config default), each SP figure emits ONLY the
normalized variant — raw SP is suppressed as redundant (the normalized view is
the fair cross-N comparison). So `fig1a`/`fig1f`/`fig_ab_a` below refer to their
normalized stems, and the "≤ 1.0" checks in the verification checklist are
against those.

| Fig (logical) | Emitted stem | Source |
|---------------|--------------|--------|
| `fig1a` | `fig1a_mean_sp_normalized_vs_tasks_main` | `aggregate_across_tasks.py` |
| `fig1c` | `fig1c_mean_exec_time_vs_tasks_main` | `aggregate_across_tasks.py` |
| `fig1f` | `fig1f_sp_distribution_boxplot_normalized` | `aggregate_across_tasks.py` |
| `fig_ab_a` | `fig_ablation_mean_sp_normalized_vs_tasks` | `aggregate_across_tasks.py` |
| `fig_ab_b` | `fig_ablation_mean_exec_time_vs_tasks` | `aggregate_across_tasks.py` |
| `fig2` | `fig2_sp_vs_interval` (+ `_normalized`) | `interval_sweep.py` |
| `fig3` | `fig3_important_task_miss_rate` | `aggregate_across_tasks.py` |
| `fig_fallback_rejection_ratio` | **NOT YET IMPLEMENTED** | new generator (see below) |
| `fig_p25_et_vs_period` | **DEFERRED → P3** (not this cycle) | spec parked in a future P3 `optional_figures` task |

Debug figures (keep code, don't polish): `fig1b`, `fig1d`, `fig1e`, `fig3b`.

## The new fallback-rejection-ratio figure (KEEP & BUILD this cycle)

`fig_fallback_rejection_ratio` plots the **cost of the P0.7 fallback gate** vs
number of tasks. For each (taskset, scheduler) it computes:

```
ratio = during_walk_reject_count / improving_challenger_count   ∈ [0, 1]
```

— i.e. of all the SP-improving challengers the incremental walk found, what
fraction the during-walk important-task gate had to veto (trigger b-i). The
funnel invariant `evaluated ≥ improving ≥ during_walk_reject` bounds the ratio
to [0,1]; ratio is skipped (undefined) when `improving_challenger_count == 0`.

**Data source (the gap):** the two counters are written per-interval per-taskset
to `interval_fallback_log.txt` (`during_walk_reject_count`, trigger b-i, in
`IntervalFallbackOutcome`) and `interval_walk_stats.txt`
(`improving_challenger_count`, in `IntervalWalkStats`; written via
`FormatIntervalWalkStatsCsv`). On-disk path:
`<run>/<taskset_n>/<sched>/<sched>/interval_{fallback_log,walk_stats}.txt`.
`comparison_summary.csv` does **NOT** aggregate these counters, so the figure
needs a NEW aggregation reader that walks the per-interval log files, sums both
counters across intervals per (taskset, scheduler), then averages the ratio
across tasksets at each N → records of {num_tasks, scheduler, mean_ratio,
std_ratio}.

Generator to add in `aggregate_across_tasks.py`:
- NEW reader: walk `taskset_*/<sched>/<sched>/interval_{fallback_log,walk_stats}.txt`.
- NEW `generate_fig_fallback_rejection_ratio(records)` → x=num_tasks, y=ratio,
  one line per scheduler; reuse `build_line_chart` + `save_figure` (PNG+PDF).
- Wire into `main()` after the existing generators (sequential-call pattern).
- Unit test in `tests/python/test_aggregate.py` (mock data): assert PNG+PDF
  non-empty + ratio ∈ [0,1] + funnel invariant holds.

Depends on the staged-but-uncommitted `interval_walk_stats.txt` walk-quality
counters (#1/#3); that work lands first, then the reader consumes it.

## The P25 period figure — DEFERRED → P3 (not this cycle)

`fig_p25_et_vs_period` (ET vs reoptimization period, from the `INCR_Reopt_X`
ablation arms) is **deferred** out of this cycle per the 2026-08-02 figure
triage: it's a self-justification figure (the P25 bounded-ET fix already shipped
in `986a9cfe`+`de4e9636`) and ET is awkward to SP-normalize. The full spec is
parked for revival in a future P3 `optional_figures` task, alongside the closed
P2.6 sim-RT SP cross-check. Not discarded.

## Done when

- All must-have figures exist as non-empty `.png` + `.pdf` under
  `optimizer_comparison/runs/<run_id>/figures/` (including the new
  `fig_fallback_rejection_ratio`).
- No normalization >1.0 regressions (spot-check `fig1a` mean and `fig1f`
  whiskers — both ≤ 1.0).
- `fig_fallback_rejection_ratio` shows the gate-cost story (ratio ∈ [0,1]).
- Run-id + findings recorded in top-level `agents/dev_log.md`.

## Out of scope

- Debug figures 1B/1D/1E/3b (keep code, don't polish for paper).
- Interval-sweep stale-flags: the prior
  `--on_taskset_config_change`/`--run_root` unrecognized-argument crash is
  RESOLVED — `compare_optimizers.py` accepts both flags again (verified
  2026-08-02). The old `--steps simulate aggregate` workaround is itself gone.

# P0.3 — Run Prod Pipeline + Generate Core Figures

**Priority:** P0 (THE publication deliverable)
**Status:** not started
**Depends on:** P0.1 (clean committed baseline)

## Goal

Run the full prod pipeline end-to-end and produce the core figure set under
`optimizer_comparison/runs/<run_id>/figures/`. The previous prod run
(`run_prod_dur600...tasks4x6x8x10x12x14x16x18`) has an **empty** `figures/` —
the aggregate stage never completed there. This task completes it.

## Core figure set (must-have for paper)

Per `agents/plan_publication_figures.md` + the user's "keep plan set + add P25
period figure" answer:

| Fig | Description | Source |
|-----|-------------|--------|
| `fig1a` | Mean SP vs # tasks (Main: INCR, BF, RM, CFS) | `aggregate_across_tasks.py` |
| `fig1c` | Mean execution time vs # tasks (Main) | `aggregate_across_tasks.py` |
| `fig1f` | SP distribution boxplot (single fixed N) | `aggregate_across_tasks.py` |
| `fig_ab_a` | Ablation mean SP vs # tasks | `aggregate_across_tasks.py` |
| `fig_ab_b` | Ablation mean exec time vs # tasks | `aggregate_across_tasks.py` |
| `fig2` | SP vs trigger-interval sweep | `interval_sweep.py` (see P2.1) |
| `fig3` | Important-task miss rate (grouped bar) | `aggregate_across_tasks.py` |
| `fig_p25_et_vs_period` | **NEW** per-activation ET vs reoptimization period | new generator (data already exists) |

## The new P25 figure

Data already exists at
`runs/p25periodAB_run_test_dur600_interval10_seed1000_tasks6/` (from the
2026-07-04 A/B re-run). Need a new generator in `aggregate_across_tasks.py`
that:
- Reads the per-period per-activation ET from the P25 A/B run.
- Plots ET vs `ReoptimizationPeriod` (P1, P10, P30, P60). (P2.5 removed the
  `INCR_SCRATCH` amnesiac floor that this figure previously drew — the
  period-monotonicity read now stands on the INCR_Reopt_X arms alone.)
- Reuses `compute_sp_upper_bound` for any normalization.

## Run command

```
MODE=prod CONFIG_JSON=simulation_experiments/configs/experiment_config.json \
  ./scripts/run_end_to_end.sh
```

## Files

- `scripts/run_end_to_end.sh`
- `simulation_experiments/aggregate_across_tasks.py` (add P25 period generator)
- `simulation_experiments/configs/experiment_config.json`

## Done when

- All 8 core figures exist as non-empty `.png` + `.pdf` under
  `optimizer_comparison/runs/<run_id>/figures/`.
- No normalization >1.0 regressions (spot-check `fig1a` mean and `fig1f`
  whiskers — both should be ≤ 1.0 after the P12 fix).
- Findings + run-id recorded in top-level `dev_log.md`.

## Out of scope

- Debug figures 1B/1D/1E/3b (keep code, don't polish for paper).
- The interval-sweep stale-flags confirmation is P2.1 (can run during this task).

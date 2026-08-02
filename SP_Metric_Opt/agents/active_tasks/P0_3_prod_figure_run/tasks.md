# P0.3 — Tasks (working checklist)

> See `goal.md` for scope. Depends on P0.1 — RESOLVED (`7fa2e9d2`).

## INCR walk-quality counters #1 (evaluated) + #3 (improving) — DONE 2026-08-02, staged (user commits)
- [x] New `IntervalWalkStats` struct + `FormatIntervalWalkStatsCsv` (separate from `IntervalFallbackOutcome`/fallback CSV — fallback log stays pure)
- [x] Instrument `UpdateRecords`: `evaluated++`/`improving++` on the live interval's back entry
- [x] Push one `IntervalWalkStats` per dispatch in `Optimize_w_TL_ScratchOrIncre` + `OptimizePureIncremental`
- [x] Forward via `SimulationOrchestrator.h`; `RunOrchestrator.cpp` writes `interval_walk_stats.txt`
- [x] TDD: 3 CSV-shape tests + funnel-invariant `evaluated ≥ improving ≥ during_walk_reject_count`; 131/131 `testIncreOpt_w_TL`

## New P25 period figure generator
- [ ] Add `fig_p25_et_vs_period` generator to `aggregate_across_tasks.py`
- [ ] Read per-activation ET from the prod run's `INCR_Reopt_1`/`_10`/`_30`/`_60` ablation arms (NOT the removed `p25periodAB_run_test...` dir)
- [ ] Plot ET vs `ReoptimizationPeriod` (1, 10, 30, 60) (no `INCR_SCRATCH` floor — P2.5 removed it)
- [ ] Reuse `compute_sp_upper_bound` for normalization
- [ ] Unit test the generator with mock data (assert PNG+PDF non-empty)

## Run the prod pipeline
- [ ] Confirm `release/tests/RunOrchestrator` exists (built fresh by the `.sh` build stage; do NOT rely on a stale binary)
- [ ] `MODE=prod ./scripts/run_simulation_plot_eval_ns.sh`
  (default config `paper_simulation_config.json`; override via `CONFIG_JSON=`)
- [ ] Confirm all must-have figures land under `optimizer_comparison/runs/<run_id>/figures/`
  (`fig1a` normalized, `fig1c`, `fig1f` normalized, `fig_ab_a` normalized,
  `fig_ab_b`, `fig2`, `fig3`, `fig_p25_et_vs_period`)

## Verification
- [ ] `fig1a` (normalized) mean SP ≤ 1.0 (no normalization regression)
- [ ] `fig1f` (normalized) whiskers ≤ 1.0
- [ ] `fig2` produced (sweep stage runs in prod — `num_tasks_for_single_task_figures=10`; the prior stale-flags sweep crash is RESOLVED)
- [ ] `fig_p25_et_vs_period` shows the flat/bounded ET story
- [ ] Run-id + findings recorded in top-level `agents/dev_log.md`

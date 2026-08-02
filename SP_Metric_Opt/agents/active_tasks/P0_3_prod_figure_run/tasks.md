# P0.3 — Tasks (working checklist)

> See `goal.md` for scope. Depends on P0.1 — RESOLVED (`7fa2e9d2`).

## INCR walk-quality counters #1 (evaluated) + #3 (improving) — DONE 2026-08-02, staged (user commits)
- [x] New `IntervalWalkStats` struct + `FormatIntervalWalkStatsCsv` (separate from `IntervalFallbackOutcome`/fallback CSV — fallback log stays pure)
- [x] Instrument `UpdateRecords`: `evaluated++`/`improving++` on the live interval's back entry
- [x] Push one `IntervalWalkStats` per dispatch in `Optimize_w_TL_ScratchOrIncre` + `OptimizePureIncremental`
- [x] Forward via `SimulationOrchestrator.h`; `RunOrchestrator.cpp` writes `interval_walk_stats.txt`
- [x] TDD: 3 CSV-shape tests + funnel-invariant `evaluated ≥ improving ≥ during_walk_reject_count`; 131/131 `testIncreOpt_w_TL`

## P25 period figure generator — DEFERRED → P3 (2026-08-02 triage)
> Self-justification figure (P25 bounded-ET fix already shipped `986a9cfe`+
> `de4e9636`); ET awkward to SP-normalize. Spec parked for a future P3
> `optional_figures` task. Not discarded.
- [ ] Add `fig_p25_et_vs_period` generator to `aggregate_across_tasks.py`
- [ ] Read per-activation ET from the prod run's `INCR_Reopt_1`/`_10`/`_30`/`_60` ablation arms (NOT the removed `p25periodAB_run_test...` dir)
- [ ] Plot ET vs `ReoptimizationPeriod` (1, 10, 30, 60) (no `INCR_SCRATCH` floor — P2.5 removed it)
- [ ] Reuse `compute_sp_upper_bound` for normalization
- [ ] Unit test the generator with mock data (assert PNG+PDF non-empty)

## New fallback-rejection-ratio figure generator (KEEP & BUILD this cycle)
> `ratio = during_walk_reject_count / improving_challenger_count` ∈ [0,1] vs N.
> Justifies the P0.7 fallback gate's cost. `comparison_summary.csv` does NOT
> aggregate these counters → needs a NEW per-interval-log reader. Depends on the
> staged `interval_walk_stats.txt` counters landing first.
- [ ] NEW aggregation reader: walk `taskset_*/<sched>/<sched>/interval_{fallback_log,walk_stats}.txt`, sum `during_walk_reject_count` + `improving_challenger_count` across intervals per (taskset, scheduler), average the ratio across tasksets at each N
- [ ] Add `generate_fig_fallback_rejection_ratio(records)` to `aggregate_across_tasks.py` (x=num_tasks, y=ratio, one line per scheduler; reuse `build_line_chart` + `save_figure`; skip when `improving==0`)
- [ ] Wire into `main()` after the existing generators (sequential-call pattern)
- [ ] Unit test in `tests/python/test_aggregate.py` (mock data): assert PNG+PDF non-empty + ratio ∈ [0,1] + funnel invariant `evaluated ≥ improving ≥ during_walk_reject`

## Run the prod pipeline
- [ ] Confirm `release/tests/RunOrchestrator` exists (built fresh by the `.sh` build stage; do NOT rely on a stale binary)
- [ ] `MODE=prod ./scripts/run_simulation_plot_eval_ns.sh`
  (default config `paper_simulation_config.json`; override via `CONFIG_JSON=`)
- [ ] Confirm all must-have figures land under `optimizer_comparison/runs/<run_id>/figures/`
  (`fig1a` normalized, `fig1c`, `fig1f` normalized, `fig_ab_a` normalized,
  `fig_ab_b`, `fig2`, `fig3`, `fig_fallback_rejection_ratio`)

## Verification
- [ ] `fig1a` (normalized) mean SP ≤ 1.0 (no normalization regression)
- [ ] `fig1f` (normalized) whiskers ≤ 1.0
- [ ] `fig2` produced (sweep stage runs in prod — `num_tasks_for_single_task_figures=10`; the prior stale-flags sweep crash is RESOLVED; P2.1 closed as subsumed by this run)
- [ ] `fig_fallback_rejection_ratio` shows the gate-cost story (ratio ∈ [0,1])
- [ ] Run-id + findings recorded in top-level `agents/dev_log.md`

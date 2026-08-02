# P2.1 — Tasks (working checklist)

> See `goal.md` for scope. Can run during P0.3.

- [ ] Run the sweep stage via the pipeline: `./scripts/run_simulation_and_plot_figures.sh`
      (the standalone `run_interval_sweep.sh` wrapper was removed in P2.8; the
      sweep is now the pipeline's 2nd stage)
- [ ] Confirm no stale-flags crash (`compare_optimizers.py` accepts the flags)
- [ ] Assert `fig2_sp_vs_interval.{png,pdf}` land under the run's `figures/`
- [ ] Coordinate with P2.2: flip memory `interval-sweep-stale-flags-bug` to RESOLVED
- [ ] Record the run-id + result in `dev_log.md`

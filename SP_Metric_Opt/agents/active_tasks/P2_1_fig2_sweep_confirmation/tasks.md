# P2.1 — Tasks (working checklist)

> See `goal.md` for scope. Can run during P0.3.

- [ ] Run the sweep stage standalone: `./scripts/run_interval_sweep.sh`
      (or `--steps simulate sweep aggregate` if e2e supports step selection)
- [ ] Confirm no stale-flags crash (`compare_optimizers.py` accepts the flags)
- [ ] Assert `fig2_sp_vs_interval.{png,pdf}` land under the run's `figures/`
- [ ] Coordinate with P2.2: flip memory `interval-sweep-stale-flags-bug` to RESOLVED
- [ ] Record the run-id + result in `dev_log.md`

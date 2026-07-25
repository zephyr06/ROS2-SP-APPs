# P2.1 — Confirm Fig 2 (Interval Sweep) Runs in Prod

**Priority:** P2 (figure safety)
**Status:** not started
**Can run during:** P0.3

## Goal

Confirm the interval-sweep stage (`fig2_sp_vs_interval`) runs cleanly in a full
prod sweep. The stale-flags bug (memory `interval-sweep-stale-flags-bug`) is
**resolved in code** but was never confirmed on a full prod run.

## What's resolved

- `simulation_experiments/compare_optimizers.py` accepts `--on_taskset_config_change`
  and `--run_root` (lines 303, 333) — the flags `interval_sweep.py` emits.
- `tests/python/test_interval_sweep.py` covers the forwarding.

So the crash the memory described should no longer happen. This task confirms
that on a real prod sweep, not just unit tests.

## Approach

Run the sweep stage (now reachable only via the pipeline's sweep stage — the
standalone `run_interval_sweep.sh` wrapper was removed in P2.8):

```
./scripts/run_simulation_and_plot_figures.sh
```

Assert `fig2_sp_vs_interval.{png,pdf}` land under the run's `figures/`.

## Files

- `simulation_experiments/interval_sweep.py`
- `simulation_experiments/compare_optimizers.py` (the flag consumer)

## Done when

- `fig2_sp_vs_interval.{png,pdf}` produced from a clean prod sweep run.
- No stale-flags crash.
- Memory `interval-sweep-stale-flags-bug` flipped to RESOLVED (this overlaps
  with P2.2's doc hygiene — coordinate).

## Out of scope

- Re-architecting the sweep (it works; this is confirmation only).
- Other figures (P0.3 owns the full set).

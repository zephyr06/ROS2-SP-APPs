# P2.1 — Confirm Fig 2 (Interval Sweep) Runs in Prod

**Priority:** P2 (figure safety)
**Status:** CLOSED 2026-08-02 — superseded by P0.3 (not worked). See
"Disposition" below.
**Can run during:** P0.3

## Disposition — CLOSED 2026-08-02 (deferred to / subsumed by P0.3)

Closed during the pre-prod figure triage. P2.1 was never a new figure — only a
verification task for the already-built `fig2_sp_vs_interval` (from
`interval_sweep.py`; SP vs interval index within a run). Its sole remaining
done-when ("run a clean prod sweep, confirm fig2 lands, no stale-flags crash")
is **subsumed by the P0.3 prod run**: P0.3's pipeline runs the sweep stage and
lists `fig2` among its must-have figures, so a clean P0.3 run IS the P2.1
confirmation.

The stale-flags crash the memory described is **already fixed in code**:
`compare_optimizers.py` accepts `--on_taskset_config_change`/`--run_root` again
(`compare_optimizers.py:303`/`:333`), and the fixed-order pipeline
(simulate→sweep→aggregate, P22) now runs the sweep stage. So the crash P2.1
existed to catch no longer reproduces. The memory
`interval-sweep-stale-flags-bug` is flipped to RESOLVED as part of this closure
(the code fix is shipped; the P0.3 prod run will confirm end-to-end).

The `goal.md`/`tasks.md` "Approach" sections referenced the deleted
`run_simulation_and_plot_figures.sh` (removed by P2.8; the one entry point is
now `run_simulation_plot_eval_ns.sh`). That staleness is moot post-closure.

No code, agent doc (beyond this disposition), or memory was edited for the
*crash* — only this closure note + the memory flip. Revival, if ever needed,
would be a one-line check inside a P0.3 prod run, not a standalone task.

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

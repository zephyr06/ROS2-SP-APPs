# P0.3 — Tasks (working checklist)

> See `goal.md` for scope. Depends on P0.1 (clean baseline).

## New P25 period figure generator
- [ ] Add `fig_p25_et_vs_period` generator to `aggregate_across_tasks.py`
- [ ] Read per-period per-activation ET from the P25 A/B run
- [ ] Plot ET vs `ReoptimizationPeriod` (P1, P10, P30, P60) + `INCR_SCRATCH` floor
- [ ] Reuse `compute_sp_upper_bound` for normalization
- [ ] Unit test the generator with mock data (assert PNG+PDF non-empty)

## Run the prod pipeline
- [ ] Confirm `release/tests/RunOrchestrator` exists
- [ ] `MODE=prod CONFIG_JSON=simulation_experiments/configs/experiment_config.json ./scripts/run_end_to_end.sh`
- [ ] Confirm all 8 figures land under `optimizer_comparison/runs/<run_id>/figures/`

## Verification
- [ ] `fig1a` mean SP ≤ 1.0 (no normalization regression)
- [ ] `fig1f` whiskers ≤ 1.0
- [ ] `fig2` produced (coordinate with P2.1 for the sweep confirmation)
- [ ] `fig_p25_et_vs_period` shows the flat/bounded ET story
- [ ] Run-id + findings recorded in top-level `dev_log.md`

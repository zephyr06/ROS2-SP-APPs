# P0.4 — Tasks (working checklist)

> See `goal.md` for scope. **Filed only — do not start implementation** until
> the user picks this task up (per their instruction "just add this as an active
> task"). TDD where it applies (gate-checking logic).

## Decide-first (when the task is picked up)
- [ ] Resolve N-scope vs. E1: keep N=4/6/8 for quality + add an N=10/16
      overhead-only probe, or extend the suite's N range? (E1 wants ≥ N=10,
      ideally N=16; user scoped the suite to 4/6/8.)
- [ ] Confirm the exact baseline set for gate Q3 ("outperform all other
      baselines"): ablation group (INCR_NO_TL, INCR_WCET) + CFS + RM_FAST/RM_SLOW?
- [ ] Decide seed pinning granularity: one seed per N, or a small fixed
      multi-seed bundle (keep total ~20 min)?

## Taskset pinning
- [ ] Pin deterministic seeds for N = 4, 6, 8 (reuse existing taskset family,
      no new generator)
- [ ] Record the pinned tasksets in a fixed config
      (`simulation_experiments/configs/evaluation_suite_config.json`)

## Metric collection
- [ ] For each scheduler (BF, INCR, SCRATCH + baselines) on each N: collect
      mean normalized SP (÷ `ideal_SP`)
- [ ] For each scheduler on each N: collect per-interval ET / interval_period
      (overhead %)
- [ ] Reuse `aggregate_across_tasks.py` / `compare_optimizers.py` metric code —
      no parallel harness

## Gate-checking (the verdict)
- [ ] Implement the 5 north-star gates as PASS/FAIL checks:
      - Q1 small-N BF gap ≤ 30%
      - Q2 large-N INCR/SCRATCH ≥ BF
      - Q3 INCR/SCRATCH ≥ every baseline
      - E1 overhead ≤ 5% (ideal ≤ 1%) at N≥10 (per the decide-first resolution)
      - E2 INCR_ET ≤ SCRATCH_ET
- [ ] Emit a PASS/FAIL table + a machine-readable JSON (for trend tracking)

## Entry point + wiring
- [ ] `scripts/run_evaluation_suite.sh` — single command, bounded ~20 min,
      deterministic
- [ ] Suite runs green on the current clean baseline (known-good reference)

## Verdict + docs
- [ ] One-paragraph "how to run + how to read the verdict" note in this
      folder's `dev_log.md`
- [ ] Milestone appended to top-level `agents/dev_log.md`

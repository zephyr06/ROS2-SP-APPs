important tasks:
- SP metric optimization, main function is correct:
-- At small N, BF is better than INCR, but gap is low (30% is red-flag line)
-- At large N, INCR outperforms BF, due to BF time-out issues.
-- INCR outperforms all other baseline

- Execution time / overhead
-- At reasonable N, at least N=10, overhead from scheduler (per-interval ET / interval_period) is low. Ideally, extend to N=16;
--- 5% is the red-flag line. 1% is more ideal.
-- E3: INCR_Reopt_10 >= INCR_Reopt_30 >= INCR_Reopt_60 (mean SP non-increasing as the reoptimization period grows, over the ENFORCED realistic-period chain). INCR_Reopt_1 is a special max-reopt stress arm (its pair is NOT enforced); INCR_Reopt_5 is not simulated in prod. Investigation gate for the P1.1 residual, currently FAILs by design (suite stays red until P1.1 resolves).

- Simulation task set configuration
-- Try to follow reasonable configuations, but we have some flexibility to tune it.
important tasks:
- SP metric optimization, main function is correct:
-- At small N, BF is better than INCR and From-scratch, but gap is low (30% is red-flag line)
-- At large N, INCR and From-scratch outperform BF, due to BF time-out issues.
-- INCR and SCRATCH outperform all other baseline

- Execution time / overhead
-- At reasonable N, at least N=10, overhead from scheduler (per-interval ET / interval_period) is low. Ideally, extend to N=16;
--- 5% is the red-flag line. 1% is more ideal.
-- INCR cannot run slower than SCRATCH
-- E3: INCR_P1 >= INCR_P10 >= INCR_P30 >= INCR_P60 (per-activation ET non-increasing as the reoptimization period grows); investigation gate for the P1.1 residual, currently FAILs by design (suite stays red until P1.1 resolves).

- Simulation task set configuration
-- Try to follow reasonable configuations, but we have some flexibility to tune it.
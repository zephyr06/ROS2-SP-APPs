important tasks:
- SP metric optimization, main function is correct:
-- At small N, BF is better than INCR and From-scratch, but gap is low
-- At large N, INCR and From-scratch outperform BF, due to BF time-out issues.

- Execution time / overhead
-- At reasonable N, at least N=10, overhead from scheduler (per-interval ET / interval_period) is low. Ideally, extend to N=16;
--- 5% is the red-flag line. 1% is more ideal.

- Simulation task set configuration
-- Try to follow reasonable configuations, but we have some flexibility to tune it.
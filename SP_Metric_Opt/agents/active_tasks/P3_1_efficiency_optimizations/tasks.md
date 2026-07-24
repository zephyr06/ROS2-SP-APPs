# P3.1 — Tasks (working checklist)

> See `goal.md` for scope. **Deferred** — perf only, not correctness. Pick up
> only if P1.1 finds per-activation ET is a paper blocker.

- [ ] (only if unblocked) `PriorityPartialPath` → `const DAG_Model*` /
      `const SP_Parameters*` pointers in `sources/Optimization/OptimizeSP_Incre.h`
- [ ] (only if unblocked) Incremental HP-task convolution in
      `sources/Safety_Performance_Metric/RTA.cpp` — verify dynamic preemptions
      + deadlines hold under merged convolutions before landing
- [ ] (only if profiling shows it's a blocker) Reuse one challenger across
      `EvaluateTimeLimitConfig_ScratchOrIncre` calls instead of rebuilding from
      `res_opt_` each candidate (`sources/Optimization/OptimizeSP_TL_Incre.cpp`
      incremental branch `:160` / `BuildChallengerFromIncumbent` `:402`). Moved
      from P0.5 Phase-5 issue 5h on 2026-07-10. See `goal.md` for the trade-off
      (the rebuild-from-champion design was chosen over the persistent
      challenger in P0.5 5b — the diff baseline drift risk makes this risky).

## Done when
- [ ] Either implemented (with `make check.SP_OPT -j5` green + `ctest` 16/16) or
      explicitly let-go with rationale recorded here

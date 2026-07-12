# P1.1 — Tasks (working checklist)

> See `goal.md` for scope. **Deferred** — perf only, not correctness. Pick up
> only if P1.1 finds per-activation ET is a paper blocker.

- [ ] (only if unblocked) `PriorityPartialPath` → `const DAG_Model*` /
      `const SP_Parameters*` pointers in `sources/Optimization/OptimizeSP_Incre.h`
- [ ] (only if profiling shows it's a blocker) Reuse one challenger across
      `EvaluateTimeLimitConfig_ScratchOrIncre` calls instead of rebuilding from
      `res_opt_` each candidate (`sources/Optimization/OptimizeSP_TL_Incre.cpp`
      incremental branch `:160` / `BuildChallengerFromIncumbent` `:402`). Moved
      from P0.5 Phase-5 issue 5h on 2026-07-10. See `goal.md` for the trade-off
      (the rebuild-from-champion design was chosen over the persistent
      challenger in P0.5 5b — the diff baseline drift risk makes this risky).
- [ ] Implement Delta-Thresholding for Task ET Changes in `FindTaskWithDifferentEt` (avoid triggering `OptimizeIncre` if the ET distribution's change is below a threshold $\epsilon_{ET}$).
- [ ] Implement Processor-Level and Priority-Level RTA Memoization (isolate RTA evaluations per processor; reuse convolved HP prefix when priority variations are evaluated in 1D search).
- [ ] (Optional) Implement Low-Probability Tail Pruning during convolution (filter out state combinations with probability $< 10^{-12}$ to reduce Convolve/Sort/Coalesce overhead).
- [ ] (Optional) Implement Early Stopping in Coordinate Descent (terminate the task-level walk loop early if $P_{CD}$ consecutive tasks result in zero SP improvement).

## Done when
- [ ] Either implemented (with `make check.SP_OPT -j5` green + `ctest` 16/16) or
      explicitly let-go with rationale recorded here

# P3.1 — Tasks (working checklist)

> See `goal.md` for scope. **Deferred** — perf only, not correctness. Pick up
> only if P1.1 finds per-activation ET is a paper blocker.

- [ ] (only if unblocked) `PriorityPartialPath` → `const DAG_Model*` /
      `const SP_Parameters*` pointers in `sources/Optimization/OptimizeSP_Incre.h`
- [ ] (only if unblocked) Incremental HP-task convolution in
      `sources/Safety_Performance_Metric/RTA.cpp` — verify dynamic preemptions
      + deadlines hold under merged convolutions before landing

## Done when
- [ ] Either implemented (with `make check.SP_OPT -j5` green + `ctest` 16/16) or
      explicitly let-go with rationale recorded here

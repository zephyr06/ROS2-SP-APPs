# P2.20 — Dev Log

## 2026-08-01 — Filed from P2.19 (stub; not started)

**Origin:** P2.19's user-supplied 3-point design. P2.19 takes points (1) worst-
case DAG perf → min TL option + (3) rename (the crash fix). Point (2) —
iterate `ComputeSafeFallback` to convergence — is a behavior-change enhancement,
NOT the crash fix, so it's split into this separate task to keep P2.19 small and
isolate the behavior change.

**Current state:** `OptimizeIncre_w_TL` (`OptimizeSP_TL_Incre.cpp:839-858`) runs
single-pass (`PerformSerializedTaskQueueOptimization` once). The convergence loop
would re-run the serialized pass until a full pass cannot improve the best SP.

**Prerequisite:** P2.19 landed + green (crash gone, worst-case DAG uses min TL
for perf tasks).

**Not started.** Design points + TDD red to be filled in once P2.19 lands. See
`goal.md` for the design questions to resolve at start.

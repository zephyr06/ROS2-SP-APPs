# P1.1 — P25 Residual Investigation

User comment: This task seems to be staled

**Priority:** P1 (open research question; user: "gain more understanding before closing")
**Status:** not started
**Reference docs:** `agents/investigation/` (the 3 P25 docs)

## Context

P25 = "INCR per-activation ET grows with reoptimization period". The
pathological 3× growth is **eliminated** (Fix A `986a9cfe` + Fix B `de4e9636`,
TDD-verified, committed): P60/P1 collapsed 3.4×→1.47× (ts0), 2.1×→1.13× (ts2);
the numerical pass bar (P30/P60 vs P10 ≤ 1.1–1.3×) is met on both.

BUT the literal monotonic flip `INCR_P1 ≥ P10 ≥ P30 ≈ P60` was **NOT** met —
P1 is still the cheapest INCR arm. The residual gap is the per-variation
`ObtainSP_DAG` asymmetry that **Fix C** (deferred) was meant to address.

**Per user (2026-07-06):** do NOT re-frame the claim or implement Fix C yet —
**investigate first**. This task is investigation, not implementation.

**Update (2026-07-07):** the gate is RESOLVED and the yardstick corrected (YAML
Gaussian ≠ ground truth for TL-optimizable tasks — see `dev_log.md` YARDSTICK
CORRECTION section). User approved implementing the fix the correction
identifies: start the incremental descent from the carried adopted TL
(`ReconstructTimeLimitVecFromResOpt()`) instead of the Gaussian-mean TL
(`InitializeTimeLimitsFromETConfig()`), at `OptimizeIncre_w_TL:373`, with an
edge-case guard for tasks that lost their perf pair. This lifts the 2026-07-06
hold FOR THIS LEVER ONLY. Fix C and Fix D remain out of scope (wrong levers);
equal-radii A/B dropped.

## The gate: reconcile the 2-vs-8 discrepancy

Until this is resolved, Fix C and Fix D are both poorly scoped.

- **Ground truth** (YAML diff of the changed taskset): **2** tasks differ
  (task ids 4 and 9).
- **Instrumented runtime** (`[INCR-NDIFF-DBG]` with `debugMode:1`): reports
  **8** changed tasks.

These should agree. They don't. Why?

## Investigation steps

1. **Capture an instrumented changed-task-count log.** Rebuild with
   `debugMode:1`, run N=10 `taskset_0` INCR_P10, grep
   `[INCR-ET-DBG]` / `[INCR-NDIFF-DBG]`. Reconcile 2-vs-8.
2. **Equal-radii A/B** (folds in the sibling "NEW TASK" from the old
   `tasks.md`): runtime A/B of from-scratch vs incremental with
   `ReoptimizationTimeLimitSearchRadius == IncrementalTimeLimitSearchRadius`.
   Directly informs whether the residual is *structural* (REOPT genuinely
   cheaper per-variation) or an *artifact* of the radius asymmetry.
3. **Confirm Fix D is inert.** `FiniteDist::approx_equal`
   (`sources/Safety_Performance_Metric/Probability.cpp:345-364`) ignores its
   tolerance parameter (see `investigation/investigation_problems_encountered.md`
   Problem 3). This means the "GetAvgValue band" idea from Fix D does nothing
   today — record this as a confirmed finding.
4. **Only after (1)+(2)+(3):** decide among:
   - Re-frame the claim to "flat, bounded ET" (proven by the current data).
   - Implement Fix C (incremental per-variation scoring).
   - Implement the GetAvgValue band (the only live part of Fix D — but step 3
     says the band is inert, so this needs the tolerance fix first).

## Files

- `sources/Optimization/OptimizeSP_TL_Incre.cpp`
- `sources/Optimization/OptimizeSP_Incre.cpp:145` (`FindTaskWithDifferentEt`)
- `sources/Safety_Performance_Metric/Probability.cpp:345-364` (`approx_equal`)
- `simulation_experiments/repro_et_grows_with_period.py`
- `agents/investigation/investigation_problems_encountered.md`
- `agents/investigation/runtime_profiling_guide.md`
- `agents/investigation/debug_runtime0704_incr.md`

## Done when

- Instrumented log captured; 2-vs-8 reconciled (documented in this task's
  `dev_log.md` + a milestone in top-level `dev_log.md`).
- Equal-radii A/B run; result recorded.
- Fix D inert-ness confirmed in writing.
- An explicit decision recorded: re-frame / Fix C / GetAvgValue band (with
  rationale tied to the investigation findings).

## Out of scope

- Implementing Fix C before the investigation converges.
- Re-framing the paper claim before the investigation converges.

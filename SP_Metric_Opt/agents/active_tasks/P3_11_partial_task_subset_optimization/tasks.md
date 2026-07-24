# P1.11 — Task Steps

> TDD, one small commit-able milestone at a time. Each step: add a failing
> test, make it pass, refactor for review. `make check.SP_OPT -j5` in the
> DEBUG build folder (`cmake -DCMAKE_BUILD_TYPE=DEBUG ..`) must stay green at
> every step. Agent runs `git add` only; the user commits.
>
> **Prerequisite:** the user resolves open questions D1–D7 (`goal.md`) before
> Step 1. D1 (config shape), D2 (horizon formula), D4 (binary vs continuous),
> D7 (walk order) are load-bearing for the tests.

---

## Step 1 — Config + range check (no behavior change)

- Add `IncrementalTaskOptimizationPercentage` to `Parameters.h` (extern double)
  + `Parameters.cpp` (load from `parameters.yaml`, no fallback) + a static-init
  range check rejecting X≤0 or X>1 via `CoutError` (mirror `_exportDefaultsSet`'s
  shape, but reject instead of fall back).
- Ship `parameters.yaml` with `IncrementalTaskOptimizationPercentage: 1.0`.
- Test: `Parameters_LoadsIncrementalTaskOptimizationPercentage` (or a load-time
  range-check test if the pattern is testable) — X=1.0 loads; out-of-range
  values are rejected. (If a load-time test is awkward, cover via Step 7's
  integration run at X=1.0.)
- **Gate:** 16/16 ctest green; no behavior change (X=1.0 everywhere).

## Step 2 — `CoverageHorizon` free function

- Add `int CoverageHorizon(double optimization_percentage)` to
  `OptimizeSP_TL_Incre.h` / `.cpp` — pure: `ceil(1.0 / X)`.
- Tests (unit, free-fn):
  - `CoverageHorizon_HalfIsTwo` — X=0.5 → 2.
  - `CoverageHorizon_QuarterIsFour` — X=0.25 → 4.
  - `CoverageHorizon_ThreeQuartersIsTwo` — X=0.75 → 2 (ceil(1.33)).
  - `CoverageHorizon_FullIsOne` — X=1.0 → 1.
- **Gate:** tests red → green; 16/16 ctest.

## Step 3 — `SelectTaskSubset` free function (pure, no optimizer state)

- Add `SelectTaskSubset(queue, X, staleness)` free fn to
  `OptimizeSP_TL_Incre.h` / `.cpp` (pseudocode in `goal.md` §"The algorithm").
  X≥1.0 or empty queue → early-return `queue` untouched + `staleness` untouched.
- Tests (unit — the core of the TDD plan, `goal.md` §"TDD plan" 1–6):
  1. `SelectTaskSubset_FullPercentageReturnsAllTasks` — X=1.0 → full queue,
     order preserved, `staleness` unchanged. (Bit-identical anchor.)
  2. `SelectTaskSubset_HalfSelectsHalfByWeight` — X=0.5, N=10, distinct weights
     → K=5, top-5 selected; staleness selected→0, unselected→+1.
  3. `SelectTaskSubset_StaleTasksForceIncluded` — low-weight task with
     staleness ≥ horizon is force-included.
  4. `SelectTaskSubset_CoverageWithinHorizon` — drive 5 intervals at X=0.5 on a
     fixed queue, assert every task selected ≥1× within 3 intervals.
  5. `SelectTaskSubset_PreservesWeightDescOrder` — returned subset is in the
     input queue's order.
  6. `SelectTaskSubset_XAboveOneClampsToFull` — X=1.5 → full queue (defensive).
- **Gate:** all six red → green; 16/16 ctest.

## Step 4 — `intervals_since_last_optimized_` member + hook in the serialized loop

- Add `std::unordered_map<int, int> intervals_since_last_optimized_;` member to
  `OptimizePA_Incre_with_TimeLimits` (`OptimizeSP_TL_Incre.h`).
- Hook in `PerformSerializedTaskQueueOptimization` (`OptimizeSP_TL_Incre.cpp:381`),
  after `BuildSerializedTaskQueue` (`:411`), before the walk (`:419`):
  `queue = SelectTaskSubset(queue, GlobalVariables::IncrementalTaskOptimizationPercentage, intervals_since_last_optimized_);`
- Integration tests (`goal.md` §"TDD plan" 7–8):
  7. `PerformSerializedTaskQueueOptimization_X1p0_BitIdenticalToBaseline` — X=1.0
     vs pre-P1.11 baseline on a fixed taskset → identical `opt_pa_` + `opt_sp_`.
     (Regression anchor — the load-bearing correctness property.)
  8. `PerformSerializedTaskQueueOptimization_SubsetPreservesSingleChangeInvariant`
     — X=0.5, debugMode on, `AssertSingleChangeInvariant` never throws.
- **Gate:** tests red → green; 16/16 ctest. **X=1.0 bit-identical confirmed.**

## Step 5 — Staleness clear on reopt (the coverage backstop)

- In `ResetIncumbentBaseline(bool from_scratch)` (`OptimizeSP_TL_Incre.h:262`):
  `if (from_scratch) intervals_since_last_optimized_.clear();`
- Test:
  9. `ReOptimizePeriodic_ClearsStaleness` — populate the map with non-zero
     values, run a from-scratch reopt, assert the map is empty afterwards.
- **Gate:** test red → green; 16/16 ctest.

## Step 6 — A/B config arms (user runs the A/B)

- Add X-sweep arms to the p25 / eval config (D5 resolution shapes this):
  X ∈ {0.25, 0.5, 0.75, 1.0}, 1.0 = control.
- Wire the per-arm override of `IncrementalTaskOptimizationPercentage`
  (mirror the `INCR_Reopt_X` arm-override pattern).
- No new tests (config plumbing); the eval suite is the gate.
- **Gate:** eval suite runs clean at each X; user reads Q1/Q2/Q3/E1/E3 + the
  new per-interval SP-eval metric.

---

## Notes

- **P1.10 (serialized queue) is COMPLETE + committed** — this task builds
  directly on `PerformSerializedTaskQueueOptimization` + `BuildSerializedTaskQueue`.
- **P1.9 (RTA cache) is independent** of P1.11's landing order (D6) — both touch
  `PerformSerializedTaskQueueOptimization`'s vicinity but at different points
  (P1.11 before the walk, P1.9 inside the eval). Land one, then the other.
- The single-change invariant (`|diff|≤1` per SP-eval) is PRESERVED — a subset
  is just fewer single-task steps. P1.9's cache premise is unaffected.
- Each step is one review-and-commit cycle per `agent_coding_rules.md`.

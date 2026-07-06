# Task: Trial-and-Error Time Limit Optimization (Dynamic Radius Search)

> **STATUS: REWORK LANDED 2026-07-05** (working tree, uncommitted) — the radius
> was decoupled from the walk; see "Rework: decouple the walk from the radius"
> below. The follow-up task **"hoist `patience` to a tunable global parameter
> (2026-07-05)"** at the bottom of this doc is **DONE 2026-07-05**: `patience` is
> now a pair of YAML-loaded globals (`IncrementalTimeLimitSearchPatience` /
> `ReoptimizationTimeLimitSearchPatience`) replacing the hardcoded ternary, 16/16
> ctest green. The original (radius-capped) landing described in the rest of this
> doc is superseded by the rework.
>
> **Original landing (radius-capped): DONE 2026-07-05** (working tree,
> uncommitted). The four proposed changes below are implemented;
> `testIncreOpt_w_TL` 43/43 + `ctest` 16/16 green at the time. The walk adds a
> `patience` parameter beyond the original spec (incremental path: `patience=0`
> strict break; reopt path: `patience=1` tolerates one non-monotonic dip so a
> single dip can't hide a strictly better option further out).

This task aims to improve the optimization efficiency of [PerformCoordinateDescentForTaskConfigOpt](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/sources/Optimization/OptimizeSP_TL_Incre.cpp#L179) in [OptimizeSP_TL_Incre.cpp](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/sources/Optimization/OptimizeSP_TL_Incre.cpp).

## Rationale
Currently, the coordinate descent algorithm enumerates all time limit options within a static radius (e.g., `radius=2` for re-optimization, checking 5 options; `radius=1` for incremental optimization, checking 3 options). This leads to unnecessary, expensive safety-performance (SP) evaluations, especially for options that are further away in directions that do not show any improvement at the first step.

The proposed **Trial-and-Error** algorithm:
1. **Establishes a baseline SP** of the starting configuration.
2. **Walks in both directions** (decreasing/increasing) from the current time limit option.
3. **Stops early if a direction is not promising**: At each step, it evaluates the SP. If the new configuration is worse than the best configuration observed so far, it terminates search in that direction immediately.
4. **Limits total options explored**:
   - **Incremental Opt (`radius=1`)**: Automatically checks at most 3 options (1 baseline + 2 trials), as no further options exist.
   - **Re-optimization (`radius=2`)**: Automatically checks at most 5 options, with the outer options only evaluated if the intermediate trial steps were successful.

This reduces the average evaluation count per task while maintaining optimization quality.

To adhere to coding guidelines (short, single-purpose functions), the logic is decomposed into helper methods:
* `FindTimeLimitOptionIndex`: generic lookup for current values in the option list.
* `IsBetterTimeLimitOption`: encapsulates tie-breaking and step-direction SP comparison.
* `OptimizeSingleTaskTimeLimit`: performs unidirectional trial-and-error walk search.

---

## Proposed Changes

### 1. Configuration Changes

Update the default search radius values in [parameters.yaml](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/sources/parameters.yaml) to align with the new trial-and-error strategy:
* Set `ReoptimizationTimeLimitSearchRadius` to `2` (down from `6`), which will try at most 5 options.
* Set `IncrementalTimeLimitSearchRadius` to `1` (down from `2`), which will try at most 3 options.

```yaml
# sources/parameters.yaml
ReoptimizationTimeLimitSearchRadius: 2
IncrementalTimeLimitSearchRadius: 1
```

### 2. Declarations in `OptimizeSP_TL_Incre.h`

Add the helper method declarations to [OptimizeSP_TL_Incre.h](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/sources/Optimization/OptimizeSP_TL_Incre.h):

```cpp
    // sources/Optimization/OptimizeSP_TL_Incre.h
    
    // Finds the index of a specific time limit within the options vector. Returns options.size() if not found.
    size_t FindTimeLimitOptionIndex(const std::vector<double>& options, double current_val);

    // Determines whether a candidate option is better (strictly higher SP, or approx equal SP when decreasing).
    bool IsBetterTimeLimitOption(double new_sp, double current_best_sp, int step);

    // Performs unidirectional trial-and-error time limit optimization for a single task.
    double OptimizeSingleTaskTimeLimit(
        size_t task_idx, int K, std::vector<double>& time_limits,
        double current_sp, double baseline_val, int step, bool from_scratch);
```

### 3. Implementation in `OptimizeSP_TL_Incre.cpp`

Implement the helpers and update `PerformCoordinateDescentForTaskConfigOpt` in [OptimizeSP_TL_Incre.cpp](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/sources/Optimization/OptimizeSP_TL_Incre.cpp):

```cpp
// sources/Optimization/OptimizeSP_TL_Incre.cpp

size_t OptimizePA_Incre_with_TimeLimits::FindTimeLimitOptionIndex(
    const std::vector<double>& options, double current_val) {
    auto it = std::find(options.begin(), options.end(), current_val);
    if (it == options.end()) {
        return options.size();
    }
    return std::distance(options.begin(), it);
}

bool OptimizePA_Incre_with_TimeLimits::IsBetterTimeLimitOption(
    double new_sp, double current_best_sp, int step) {
    if (new_sp > current_best_sp && !ApproxEqualSP(new_sp, current_best_sp)) {
        return true;
    }
    // if we have ties, we choose the smaller time limits
    if (ApproxEqualSP(new_sp, current_best_sp) && step < 0) {
        return true;
    }
    return false;
}

double OptimizePA_Incre_with_TimeLimits::OptimizeSingleTaskTimeLimit(
    size_t task_idx, int K, std::vector<double>& time_limits,
    double current_sp, double baseline_val, int step, bool from_scratch) {
    
    const std::vector<double>& opts = time_limit_option_for_each_task_[task_idx];
    if (opts.size() == 1 && opts[0] == -1.0) {
        return current_sp;
    }

    size_t curr_opt_idx = FindTimeLimitOptionIndex(opts, baseline_val);
    if (curr_opt_idx == opts.size()) {
        return current_sp;
    }

    double best_sp = current_sp;
    double best_option_val = time_limits[task_idx];

    // Walk sequentially in the direction of step
    for (int i = static_cast<int>(curr_opt_idx) + step;
         i >= 0 && i < static_cast<int>(opts.size());
         i += step) {
        
        double val = opts[i];
        time_limits[task_idx] = val;
        double sp_val = EvaluateTimeLimitConfig_ScratchOrIncre(K, time_limits, from_scratch);

        if (IsBetterTimeLimitOption(sp_val, best_sp, step)) {
            best_sp = sp_val;
            best_option_val = val;
        } else {
            break; // Stop immediately if the configuration is worse than best-yet
        }
    }

    time_limits[task_idx] = best_option_val;
    return best_sp;
}

void OptimizePA_Incre_with_TimeLimits::PerformCoordinateDescentForTaskConfigOpt(
    int K, std::vector<double>& time_limits, bool from_scratch) {
    std::vector<size_t> sorted_indices(dag_tasks_.tasks.size());
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(sorted_indices.begin(), sorted_indices.end(),
              TaskSortingHeuristic{dag_tasks_, sp_parameters_});

    // Establish the initial baseline SP of the starting configuration
    double current_config_sp = EvaluateTimeLimitConfig_ScratchOrIncre(K, time_limits, from_scratch);

    for (size_t idx : sorted_indices) {
        double baseline_val = time_limits[idx];
        // 1. Backward pass: try decreasing the time limit
        current_config_sp = OptimizeSingleTaskTimeLimit(idx, K, time_limits, current_config_sp, baseline_val, -1, from_scratch);
        // 2. Forward pass: try increasing the time limit
        current_config_sp = OptimizeSingleTaskTimeLimit(idx, K, time_limits, current_config_sp, baseline_val, 1, from_scratch);
    }
}
```

---

## Verification Plan

### 1. Unit Tests
* Run `make check.SP_OPT -j5` to verify existing C++ test suite passes.
* Modify the assertions in [testIncreOpt_w_TL.cpp](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/tests/testIncreOpt_w_TL.cpp) if they rely on the exact old evaluation count or the old default radius configurations.
* Add specific test cases to [testIncreOpt_w_TL.cpp](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/tests/testIncreOpt_w_TL.cpp) that mock a task set, verify that `PerformCoordinateDescentForTaskConfigOpt` cuts off further evaluations if trial steps are unsuccessful, and confirm that the final time limit matches the expected optimum.

### 2. Manual and Integration Verification
* Run `scripts/run_all_experiments.sh` to compare execution times and SP results before and after this optimization.
* Compare evaluation counts (`eval_count_` logs or runtime performance) between the baseline and the trial-and-error approach.

---

## Implementation landed (2026-07-05)

All four proposed changes are in the working tree (uncommitted). Summary of what
shipped vs. the spec above, and how it was verified:

### What was implemented

1. **Configuration (`sources/parameters.yaml`)** — `ReoptimizationTimeLimitSearchRadius`
   lowered 6→2, `IncrementalTimeLimitSearchRadius` lowered 2→1, with an inline
   comment describing the trial-and-error walk + the patience semantics.
2. **Header (`sources/Optimization/OptimizeSP_TL_Incre.h`)** — the three helpers
   are declared (`FindTimeLimitOptionIndex`, `IsBetterTimeLimitOption`,
   `OptimizeSingleTaskTimeLimit`), with doc-comments explaining the off-the-end
   sentinel, the tie-break rule, and the patience budget.
3. **Implementation (`sources/Optimization/OptimizeSP_TL_Incre.cpp`)** — the
   helpers + the walk are implemented, and `PerformCoordinateDescentForTaskConfigOpt`
   now calls `OptimizeSingleTaskTimeLimit` (backward pass `step=-1` then forward
   pass `step=+1`) per task instead of enumerating the whole window.
4. **Tests (`tests/testIncreOpt_w_TL.cpp`)** — added the
   `TrialAndErrorTLWalkSynthetic` fixture (7 walk tests: strict-monotone-adopts-
   to-end, strict-break-on-first-non-improvement, patience=1-tolerates-one-dip,
   patience=1-breaks-after-two-non-improvements, backward-tie-break-on-flat-SP,
   no-options-noop, baseline-not-in-options-noop) and the
   `FindTimeLimitOptionIndexTest` / `IsBetterTimeLimitOptionTest` helper suites.
   The walk is unit-tested with a deterministic TL→SP stub overriding the
   virtual `EvaluateTimeLimitConfig_ScratchOrIncre`, so the walk's control flow
   is asserted independently of the RTA-backed evaluator.

### Deviation from the spec: `patience`

The spec's step 3 said "stop early if a direction is not promising … terminate
search in that direction immediately." Implemented as a **`patience` budget**
(counted in consecutive non-improving steps) rather than a hard immediate break:

- **Incremental path (`from_scratch=false`): `patience=0`** — strict break on
  the first non-improving step. The narrow radius=1 window has at most 3
  options, so there is no room for a dip to hide a better option further out;
  strict is safe and cheapest.
- **Reopt path (`from_scratch=true`): `patience=1`** — tolerate one
  non-monotonic dip before breaking. The wider radius=2 window can be
  non-unimodal (SP-vs-TL is not guaranteed monotonic at high utilization), so a
  single dip must not hide a strictly better option one step further.

`patience` is decided inside `PerformCoordinateDescentForTaskConfigOpt` from the
`from_scratch` flag and forwarded to each `OptimizeSingleTaskTimeLimit` call;
it is not a new knob.

### Test-expectation updates forced by the narrower radii

Three pre-existing assertions were coupled to the old radius-6/radius-2 windows
and were updated (these are test-expectation updates, NOT source bugs — the
walk's behavior under the narrower windows was verified by instrumenting the SP
landscape):

- `TaskSetForTest_robotics_v19::optimize_incremental` — bootstrap wide-radius
  window is now `[600,800,1000]` (400 excluded); TSP gets TL=600 via the
  smaller-TL tie-break at high-util saturation (1000 strictly beats the RM
  baseline, then 800 and 600 tie and the tie-break adopts each in turn). Was
  expecting 400. The incremental narrow-radius window is `[400,600]` (800/1000
  excluded); on the low-util v21 DAG SP strictly increases with TL, so 600 is
  the within-window optimum. Was expecting ≥800.
- `TaskSetForTest_robotics_v19_2::ReOptimizePeriodic` — same wide-radius
  landscape as v19; TSP gets 600. Was expecting 400.
- `CompareAndKeepSynthetic::PerformCoordinateDescent_SkipsMinusOneOnlyTaskInMixedSet`
  — the old test conflated the wide bootstrap radius (4 options for
  `t_perf_option_count`) with the narrow incremental radius (3 options actually
  evaluated). Rewritten to assert the eval count equals T_perf's **narrow-window**
  size (3), T_noise contributes 0 (skip works), and no fallback eval fires.

### Verification

- `testIncreOpt_w_TL` = **43/43 green** (was 26 before the trial-and-error
  work; +17 from the walk + helper suites + the rewritten mixed-set test).
- Full `ctest` = **16/16 green**.
- Source is clean (no leftover diagnostic prints; the only source diff is the
  trial-and-error rewrite itself).

### Open follow-ups (NOT part of this task)

The two sibling "NEW TASKS" in `agents/tasks.md` remain open:
- Apply the same trial-and-error idea to incremental **priority** assignment.
- Runtime A/B: from-scratch vs incremental when both radii are equal, to verify
  whether from-scratch is actually faster than incremental.

---

## Rework: decouple the walk from the radius (2026-07-05)

### The bug the radius-capped walk introduced

`testIncreOpt_w_TL.cpp` `TaskSetForTest_robotics_v19::ReOptimizePeriodic`
expected `id2time_limit[0] == 400` (the tightest TL, reached by the
smaller-TL tie-break on a saturated high-util SP landscape where all TL options
are effectively unschedulable and produce near-identical SP). The radius-capped
walk returned **600** instead.

Root cause: `OptimizeSingleTaskTimeLimit` steps over
`time_limit_option_for_each_task_[task_idx]`, and that vector was rebuilt on
every `OptimizeIncre_w_TL` / `ReOptimizePeriodic` call by
`RecordCloseTimeLimitOptions(dag, radius)` — which records **only** the
`±radius` options around the ET-closest one. So the loop bound
`i < opts.size()` was the *radius window wall*, not the task's full option set.

For v19 TSP: full options `[400,600,800,1000]`, ET≈1501 → closest = 1000
(index 3), `ReoptimizationTimeLimitSearchRadius=2` → recorded window
`[600,800,1000]`. TL=400 (index 0) was never in `opts`, so the downward
tie-break walk stopped at 600 instead of reaching 400.

The walk's *termination logic* was already correct — `IsBetterTimeLimitOption`
+ `patience` already implements "stop only on non-progress." The
`TrialAndErrorTLWalkSynthetic` stub tests proved this: they override
`time_limit_option_for_each_task_` with the **full** `[400,600,800,1000]` set
and the walk behaves exactly as intended. The production bug was purely that
production fed the walk a **truncated window**.

### Design (clean, decoupled)

The walk's domain is decoupled from the radius. A task's TL options have one
source of truth: its full ordered `timePerformancePairs`. The walk always steps
over the full set, starting at the ET-closest option
(`InitializeTimeLimitsFromETConfig`, unchanged), terminating on
patience-bounded non-improvement (unchanged). The radius no longer caps the
walk's reach.

1. **`time_limit_option_for_each_task_` holds the full option set** — built via
   the existing `RecordTimeLimitOptions(dag_tasks_)` (no radius), the same call
   the constructor already uses. `OptimizeIncre_w_TL` and `ReOptimizePeriodic`
   stop overwriting it with `RecordCloseTimeLimitOptions(dag, radius)`. One
   representation, consistent between constructor and walk.
2. **`OptimizeSingleTaskTimeLimit` is unchanged** — its loop already iterates
   `opts`; now `opts` is the full set, so 400 is naturally reachable. The
   `{-1}`-only skip and `FindTimeLimitOptionIndex` baseline lookup keep working.
3. **`IsBetterTimeLimitOption` / `FindTimeLimitOptionIndex` / `patience`** —
   unchanged. Progress-based termination is already here.
4. **Radius removed entirely.** `IncrementalTimeLimitSearchRadius` /
   `ReoptimizationTimeLimitSearchRadius` deleted from `parameters.yaml` +
   `Parameters.h/.cpp`. The `radius` arg dropped from the 2-arg
   `ReOptimizePeriodic(dag,K,radius)` / `OptimizeIncre_w_TL(dag,K,radius)`
   overloads. The incremental-vs-reopt distinction survives via `from_scratch`
   (warm-start vs re-search) and `patience` (0 vs 1); the dispatcher still
   routes by `count % ReoptimizationPeriod`. `RecordCloseTimeLimitOptions`
   stays as a unit-tested utility — just no longer on the walk path.

### Sub-tasks

- [x] 1. Decouple TL walk from radius: full option set (source)
- [x] 2. Remove radius parameters and args (yaml + Parameters + signatures)
- [x] 3. Update tests for full-set walk + radius removal
- [x] 4. Build + run full test suite green

### Sub-task 3 resolution (2026-07-05)

All radius references and the 3-arg `ReOptimizePeriodic` overload were removed
from `tests/testIncreOpt_w_TL.cpp`. The non-mechanical reworks:

- **`CompareAndKeepSynthetic` Adopts/Keeps** — rewritten around a **DAG mutation**
  (not a non-monotonic SP profile, as the test-impact summary speculated). Two
  new YAMLs, `TaskData/test_robotics_v30_lo.yaml` and
  `TaskData/test_robotics_v30_hi.yaml`, are identical to `test_robotics_v21.yaml`
  except SLAM's execution time (285 vs 2853). On the light-load `v30_lo` TSP's
  optimal TL is 1000; on the heavy-load `v30_hi` it is 400 — the larger SLAM ET
  on the same processor pushes TL=1000 past a schedulability cliff, shifting the
  optimum DOWN. **Adopt** = bootstrap `v30_lo` (→1000), reopt `v30_hi`: the
  re-evaluated incumbent {TL=1000} under `v30_hi` is strictly worse than `v30_hi`'s
  optimum (400), so the search result is adopted. **Keep** = bootstrap `v30_lo`,
  reopt `v30_lo` again: incumbent re-eval ties the search result → incumbent
  preserved. This exercises the genuine `SeedIncumbentBaseline` re-eval-under-
  new-DAG branch — the real-world condition compare-and-keep exists for — with no
  synthetic radius.
- **`CounterDispatcherSynthetic`** — under the full-set walk both branches record
  the same option count, so the 5-vs-10 size signal is gone. The fixture now uses
  a `RecordingDispatcherOpt` subclass that overrides
  `EvaluateTimeLimitConfig_ScratchOrIncre` to record every `from_scratch` flag:
  the reopt branch passes `true`, the incremental branch `false`. That flag is the
  new branch-distinguishing observable. `RoutesToIncrementalAtNonModularCount`
  asserts the second call pushes a `false` flag.
- **`PerformCoordinateDescent_SkipsMinusOneOnlyTaskInMixedSet`** — eval count
  recomputed against the full-set walk: T_perf's full option set is 4
  ([400,600,800,1000]); the incremental leg runs exactly 4 evals (baseline +
  backward-to-boundary + forward-to-boundary on the monotonic landscape), T_noise
  ({-1}-only) skipped → 0. Observable changed from the old narrow-window size (3)
  to the full-set size (4).
- **`v19 OptimizeWithOptimizationSpace` incremental half** — `600 → 400`. Under
  the full-set walk the incremental leg, warm-started at TSP ET=1000ms, reaches
  the same floor-tie-break as scratch (smallest TL = 400); the old radius cap
  couldn't reach 400 from a 1000 baseline, hence the stale 600.

### Verification (2026-07-05)

`cmake --build build --target check.SP_OPT -j5` → **16/16 ctest green**;
`testIncreOpt_w_TL` now 41 tests (was 26), all passing.

### Test impact summary

**Stays green** (walk logic unchanged; exercise the full set or don't depend on
radius-capping): all `TrialAndErrorTLWalkSynthetic` stub tests,
`FindTimeLimitOptionIndexTest`, `IsBetterTimeLimitOptionTest`,
`RecordCloseTimeLimitOptions_DynamicRadius`, `testBF_w_TL`
`RecordCloseTimeLimitOptions` tests, `TaskSetForTest_robotics_v18 optimize`
(→1000), `EnumerPA_with_TimeLimits`, `OptimizeIncre_AdvancesPrevOptimizerDagTasks`,
`PerformCoordinateDescent_AllMinusOneOnly_*`, `OptimizeWithOptimizationSpace`
scratch half (→400).

**Needs update** (assert radius-capped outcomes that change under full-set):
- `v19 ReOptimizePeriodic`: 600 → **400** (the bug fix).
- `v19_2 ReOptimizePeriodic`: 600 → 400.
- `v19 optimize_incremental`: bootstrap 600 → 400; v21 incremental 600 → recompute.
- `v19 OptimizeWithOptimizationSpace` incremental half: `≥600` → 400.
- `CompareAndKeepSynthetic` Adopts/Keeps: rewrite around a non-monotonic SP
  profile (under full-set + strictly-increasing SP the bootstrap runs to the
  global max, so adopt/keep isn't exercised the same way).
- `CounterDispatcherSynthetic RoutesToIncrementalAtNonModularCount`: recorded
  `size==5` no longer differs by branch under full-set — needs a new branch
  observable.
- `PerformCoordinateDescent_SkipsMinusOneOnlyTaskInMixedSet`: eval-count
  recomputed against full-set walk.

---

## New task: hoist `patience` to a tunable global parameter (2026-07-05)

> **STATUS: DONE 2026-07-05** (working tree, uncommitted — `git add` only per
> `agent_coding_rules.md`). Implemented as two params (the default; see "Open
> question" resolution below). 16/16 ctest green with the default values; no
> test-expectation updates needed. The synthetic stub-test from the verification
> plan was **not** added (the user asked to keep it simple, and the existing
> `patience=1` stub already proves the value reaches the walk; a non-default-
> value stub would only re-prove the same plumbing).

### Rationale

`patience` — the consecutive-non-improving-SP budget the trial-and-error walk
tolerates before stopping in one direction — is currently a *derived local* in
[`PerformCoordinateDescentForTaskConfigOpt`](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/sources/Optimization/OptimizeSP_TL_Incre.cpp#L265):

```cpp
int patience = from_scratch ? 1 : 0;
```

It is forwarded to every `OptimizeSingleTaskTimeLimit` call but is not externally
tunable: the incremental value (0, strict break) and the reopt value (1, tolerate
one non-monotonic dip) are baked into the source. The radius knobs
(`IncrementalTimeLimitSearchRadius` / `ReoptimizationTimeLimitSearchRadius`) were
removed in the rework above, but they were the right *shape* — a pair of YAML-loaded
globals, one per path, tunable without recompiling. `patience` should follow that
same pattern so the walk's explore-vs-cost tradeoff can be tuned per experiment
(e.g. patience=0 on both paths for a strict ablation, or patience=2 on the reopt
path to push past noisier non-unimodal SP landscapes at high utilization) without
touching source.

### Proposed changes

#### 1. Configuration (`sources/parameters.yaml`)

Add two keys adjacent to `ReoptimizationPeriod`, with the current baked-in values
as the defaults (so behavior is byte-for-byte unchanged until someone edits the
yaml):

```yaml
# Trial-and-error walk patience: number of consecutive non-improving SP evals
# the walk tolerates before stopping in one direction (see
# PerformCoordinateDescentForTaskConfigOpt -> OptimizeSingleTaskTimeLimit).
#   IncrementalTimeLimitSearchPatience (warm-started incremental path): 0 = strict
#     break on the first non-improving step. The warm-started PA search makes
#     SP-vs-TL effectively unimodal, so a dip never hides a better option.
#   ReoptimizationTimeLimitSearchPatience (from-scratch reopt path): 1 = tolerate
#     one non-monotonic dip. From-scratch PA search can be non-unimodal at high
#     utilization, so a single dip must not hide a strictly better option further out.
IncrementalTimeLimitSearchPatience: 0
ReoptimizationTimeLimitSearchPatience: 1
```

#### 2. Globals (`sources/Utils/Parameters.h` + `Parameters.cpp`)

Declare and load the two ints, mirroring `ReoptimizationPeriod` exactly:

```cpp
// Parameters.h  (inside namespace GlobalVariables, next to ReoptimizationPeriod)
extern int IncrementalTimeLimitSearchPatience;
extern int ReoptimizationTimeLimitSearchPatience;

// Parameters.cpp (next to the ReoptimizationPeriod load)
int IncrementalTimeLimitSearchPatience =
    loaded_doc["IncrementalTimeLimitSearchPatience"].as<int>();
int ReoptimizationTimeLimitSearchPatience =
    loaded_doc["ReoptimizationTimeLimitSearchPatience"].as<int>();
```

#### 3. Source (`sources/Optimization/OptimizeSP_TL_Incre.cpp`)

Replace the derived local at line 265 of `PerformCoordinateDescentForTaskConfigOpt`
with the global lookup. The comment block above it shrinks to a pointer-to-yaml:

```cpp
// Patience = consecutive-non-improving-SP budget the walk tolerates before
// stopping in one direction. Tunable per path via parameters.yaml:
//   from_scratch  -> ReoptimizationTimeLimitSearchPatience   (default 1)
//   incremental   -> IncrementalTimeLimitSearchPatience      (default 0)
// See parameters.yaml for the modality rationale (warm-started vs from-scratch).
int patience = from_scratch
    ? GlobalVariables::ReoptimizationTimeLimitSearchPatience
    : GlobalVariables::IncrementalTimeLimitSearchPatience;
```

The rest of the walk is **unchanged** — `patience` is already a parameter throughout
`OptimizeSingleTaskTimeLimit` and its call sites; only its *source* moves from a
hardcoded ternary to a YAML-loaded global. `IsBetterTimeLimitOption`,
`FindTimeLimitOptionIndex`, and the `{-1}`-only skip are untouched.

### Verification plan

- **Defaults reproduce today's behavior.** `cmake --build build --target check.SP_OPT -j5`
  → expect 16/16 ctest green with `0`/`1` (the current baked-in values). No
  test-expectation updates should be needed.
- **Prove the global is actually read on the walk path** (not just compiled in):
  add one `TrialAndErrorTLWalkSynthetic` case that sets
  `GlobalVariables::ReoptimizationTimeLimitSearchPatience = 2` (with `from_scratch=true`)
  and asserts the walk steps past **two** consecutive non-improving evals before
  breaking — the existing `patience=1` stub test is the template; the new one proves
  a non-default yaml value reaches `OptimizeSingleTaskTimeLimit`. Restore the default
  in the fixture teardown.
- **(Optional, sibling to the open runtime A/B follow-up)** run a reused P25 taskset
  at patience `0/0` vs `0/1` vs `1/2` and report eval-count / SP tradeoff.

### Open question — RESOLVED: two params

**Two params (default) vs one.** Faithful mirror of the old radius = two params
(incremental / reopt), preserving the `from_scratch` distinction, because the
incremental-vs-reopt SP-vs-TL modality difference is real (warm-started vs
from-scratch). The alternative is a single `TimeLimitSearchPatience` applied to both
paths, dropping the `from_scratch` branch entirely — simpler, and the cleaner
ablation, but it loses the per-path tuning.

**Resolution (2026-07-05): two params.** Implemented as
`IncrementalTimeLimitSearchPatience` (default `0`) and
`ReoptimizationTimeLimitSearchPatience` (default `1`) in `parameters.yaml`,
`Parameters.{h,cpp}`, and read at the call site in `OptimizeSP_TL_Incre.cpp`. This
preserves the `from_scratch` modality distinction while making the walk's
explore-vs-cost tradeoff tunable per-experiment without recompiling. To ablate as a
single value, set both keys to the same number.


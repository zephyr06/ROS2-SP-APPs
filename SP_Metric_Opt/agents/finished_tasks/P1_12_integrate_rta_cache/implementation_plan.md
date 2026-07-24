# P1.12 — Integrate RTA Cache into the Incremental Optimizer — Implementation Plan

Integrate the P1.11 `RTACache` (Phase 0 DONE, frozen) into the **serialized**
incremental eval path only. The cache exploits the P1.10 single-change
invariant (`|diff| ≤ 1` per SP-eval vs the committed champion `res_opt_`):
`Evaluate` patches only the one changed task's RTA instead of recomputing the
whole taskset.

## Key decisions (resolved 2026-07-18)

- **Gating = flag (F).** Add `bool rta_cache_active_ = false;`. Set `true` only
  inside `PerformSerializedTaskQueueOptimization` (the sole path where the
  invariant holds); cleared in `ResetIncumbentBaseline`. `CommitIncumbent` does
  cache work only when the flag is true. Rationale: `CommitIncumbent` is shared
  with the reopt path (`ReOptimizePeriodic` → memoryless `OptimizeFromScratch`,
  can commit a >1 change → `Evaluate` would **throw** — `ComputeTaskSetDifference`
  throws on >1, called unguarded by `ClassifyReusePerTask`). The flag both
  avoids the throw and avoids any reopt-path regression.
- **Staged landing:** 2a (write-side, behavior-preserving) → 2b (`:247`
  read-side swap, differential bit-identity gate) → 3/N (`:287` hot loop,
  base-class `RTACache&` threading, Hazard A). Each increment reviewed + tested
  before the next.
- **Hazard B helper (resolved by P1.13).** P1.13 already fixed `ObtainSP_DAG_From_Dists` in place to multiply `perf_coefficient` correctly (Hazard B). P1.13 also added `ObtainSP_Full_From_NodeRTAs` as a unified helper to assemble the full SP from the node RTAs (which wraps the corrected `ObtainSP_DAG_From_Dists` and handles chain latencies). So, a separate `ObtainSP_DAG_From_Dists_With_Perf_Coeff` helper is **no longer needed**. We can reuse `ObtainSP_Full_From_NodeRTAs` directly.
- **`RTACache` signatures take NO `sp_parameters`.** Correct calls:
  `Evaluate(dag, pa, tl)`, `AdoptChampion(dag, pa, tl, rtas)`. The cache bakes
  TLs internally (`ApplyTimeLimitsToTasksExecutionTime`); callers pass RAW
  `dag_tasks_` + the `tl` vector, NOT a TL-baked dag.
- **Cache reset = default-construct.** `rta_cache_ = RTACache();` in
  `ResetIncumbentBaseline`. NO `RTA_Cache.h` change, NO new `Clear()` method.
- **`OptimizeIncre` (full) threading REMOVED from scope.** It is a multi-task
  descent — its variations differ from `res_opt_` by >1 → `Evaluate` throws.
  It is also never called while the cache is active (serialized path bypasses
  `EvaluateTimeLimitConfig_ScratchOrIncre`). Stays on the oracle. Only
  `OptimizeIncre_SingleTask` (the |diff|≤1 primitive) gets the cache, in 3/N.

## User Review Required

> [!IMPORTANT]
> 2a and 2b are confined to the **derived** class `OptimizePA_Incre_with_TimeLimits`
> (`rta_cache_` reachable directly) — **no base-class signature changes**, **no
> test-caller changes**. Only 3/N threads `RTACache&` + `const std::vector<double>&`
> into the BASE `OptimizeIncre_SingleTask` (Hazard A); at that point
> `testOptimizeIncrePA.cpp` callers pass a local `RTACache` + a `time_limits`
> vector of `-1.0` ("no time limit").

---

## Increment 2a — Write-side (behavior-preserving; cache written, NOT read)

### [MODIFY] `sources/Optimization/OptimizeSP_TL_Incre.h`
- Add member `bool rta_cache_active_ = false;` next to `rta_cache_` (`:293`).
  Doc: true only during `PerformSerializedTaskQueueOptimization`; gates
  `CommitIncumbent`'s cache write so the reopt path (can commit >1) neither
  throws nor regresses.
- Update `rta_cache_` doc comment: after 2a it is "written at
  `CommitIncumbent` (gated by `rta_cache_active_`), reset at
  `ResetIncumbentBaseline`; NOT yet read (oracle `EvaluateSPWithPriorityVec`
  still live)."

### [MODIFY] `sources/Optimization/OptimizeSP_TL_Incre.cpp`
- `ResetIncumbentBaseline` (`:738`): at the top of BOTH `from_scratch`
  branches, `rta_cache_ = RTACache();` and `rta_cache_active_ = false;`.
  Rationale: new interval → new env → `res_opt_` re-seeded; a stale
  cache-champion would make the first `:247` eval diff >1 → throw.
- `PerformSerializedTaskQueueOptimization` (`:253`): set
  `rta_cache_active_ = true;` AFTER the `ResetIncumbentBaseline` call (so the
  reset clears it, then this re-arms it for the walk body).
- `CommitIncumbent` (`:699`): after storing `res_opt_`, if
  `rta_cache_active_`: `const auto& rtas = rta_cache_.Evaluate(dag_tasks_, pa, tl);`
  (same triple as the adopting eval → `FullReuse` → returns cached
  `candidate_rta_`, near-zero cost — NO extra RTA, NO cost regression) then
  `rta_cache_.AdoptChampion(dag_tasks_, pa, tl, rtas);`. This advances the
  cache-champion to the committed incumbent, keeping subsequent evals at
  |diff|≤1.

### Verification (2a)
- `cmake --build build --target check.SP_OPT -j5` (DEBUG) → expect 16/16 ctest
  green. No behavior change (cache written but not read).

---

## Increment 2b — Read-side `:247` swap (differential bit-identity gate)

### [MODIFY] `sources/Optimization/OptimizeSP_TL_Incre.cpp`
- `EvaluateTimeLimitConfig_SubIncremental` (`:178`), at `:247`: replace
  `EvaluateSPWithPriorityVec(dag_tasks_cur, sp_parameters_, challenger.opt_pa_)`
  with the cache eval + `ObtainSP_Full_From_NodeRTAs` assembly:
  ```cpp
  if (BFSharedBudgetCancelled()) {
      challenger.opt_sp_ = INT_MIN;
  } else {
      const std::vector<FiniteDist>& baseline_rtas = rta_cache_.Evaluate(
          dag_tasks_cur, challenger.opt_pa_, time_limits);
      if (BFSharedBudgetCancelled()) {
          challenger.opt_sp_ = INT_MIN;
      } else {
          challenger.opt_sp_ = ObtainSP_Full_From_NodeRTAs(
              dag_tasks_cur, sp_parameters_, challenger.opt_pa_,
              time_limits, baseline_rtas);
      }
  }
  ```
  `:249`/`:287` (`OptimizeIncre_SingleTask`) stay on the oracle this increment
  (base class, Hazard A — deferred to 3/N).

### Verification (2b)
- Verify `testIncreOpt_w_TL::OptimizeWithOptimizationSpace` stays green (covers the end-to-end serialized walk with the cache read-side active).
- `ctest --test-dir build -R SP_OPT --output-on-failure` → expect 17/17 green.

---

## Increment 3/N — `:287` hot loop + base-class threading (Hazard A)

### [MODIFY] `sources/Optimization/OptimizeSP_Incre.h`
- Forward declare `class RTACache;`.
- Add `RTACache& cache` + `const std::vector<double>& time_limits` params to
  `OptimizeIncre_SingleTask` (and `OptimizeIncre` only if it calls
  `OptimizeIncre_SingleTask` — confirm at edit time).

### [MODIFY] `sources/Optimization/OptimizeSP_Incre.cpp`
- `#include "sources/Safety_Performance_Metric/RTA_Cache.h"`.
- `OptimizeIncre_SingleTask`: replace the `:287` variation-loop
  `EvaluateSPWithPriorityVec` calls with `cache.Evaluate(...)` +
  `ObtainSP_Full_From_NodeRTAs(...)` (same assembly as 2b).
- Caller (`EvaluateTimeLimitConfig_SubIncremental:249`) threads `rta_cache_`
  + `time_limits`.

### [MODIFY] `tests/testOptimizeIncrePA.cpp`
- Update `OptimizeIncre_SingleTask` callers to pass a local `RTACache` + a
  `time_limits` vector of `-1.0`.

### Verification (3/N)
- Differential tests vs full recompute; 16/16 ctest green in DEBUG.

---

## Phase 2 (deferred) — dispatch in hot loops + scalability

- TL patch dispatch (Loop B `OptimizeSingleTaskTimeLimit`) via `Evaluate`.
- Priority-move patch dispatch (Loop A) via `Evaluate`.
- End-to-end profiling at N=6/10/16 with/without cache; document speedup.

## What is NOT changing
- `RTA_Cache.h` / `RTA_Cache.cpp` / `PrioritySwitchAnalysis.h` — frozen (Phase 0).
- `ObtainSP_DAG_From_Dists` (existing) — untouched (live caller).
- `OptimizeIncre` (full) / `EvaluateTimeLimitConfig_ScratchOrIncre` incremental
  branch — stay on the oracle (cache unsafe there: multi-task descent).
- CMake — sources are `GLOB_RECURSE`d; new header none, helper is in existing TU.

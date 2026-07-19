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
- **Hazard B helper IS needed (verified).** `ObtainSP` (`SP_Metric.cpp:11-15`)
  returns `SP_Func(...) * weight` — **no `perf_coefficient`**. The existing
  `ObtainSP_DAG_From_Dists` (`:129-149`) calls `ObtainSP`, so it also omits
  `perf_coefficient`. The oracle path (`EvaluateSPWithPriorityVec` →
  `ObtainSP_DAG` → `ObtainSP_TaskSet` `:53-67`) **does** multiply
  `perf_coefficient` (`:61,65`). A naive drop-in would NOT be bit-identical →
  add `ObtainSP_DAG_From_Dists_With_Perf_Coeff`.
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

### [MODIFY] `sources/Safety_Performance_Metric/SP_Metric.h`
- Declare `ObtainSP_DAG_From_Dists_With_Perf_Coeff(dag, sp_params, node_rts,
  path_lats)` — same as `ObtainSP_DAG_From_Dists` but multiplies each task's
  `perf_coefficient` (mirrors `ObtainSP_TaskSet:61,65`).

### [MODIFY] `sources/Safety_Performance_Metric/SP_Metric.cpp`
- Implement `ObtainSP_DAG_From_Dists_With_Perf_Coeff`: copy
  `ObtainSP_DAG_From_Dists` (`:129-149`) but replace the per-task
  `ObtainSP(...)` call with the `perf_coefficient`-scaled form
  (`SP_Func(...) * weight * perf_coefficient`, as in `ObtainSP_TaskSet`).
  Chain/path term unchanged (no `perf_coefficient` on paths). Do NOT modify the
  existing `ObtainSP_DAG_From_Dists` (it has a live caller, `ObtainSPFromRTAFiles:221`).

### [MODIFY] `sources/Optimization/OptimizeSP_TL_Incre.cpp`
- `EvaluateTimeLimitConfig_SubIncremental` (`:178`), at `:247`: replace
  `EvaluateSPWithPriorityVec(dag_tasks_cur, sp_parameters_, challenger.opt_pa_)`
  with the cache eval + perf-coeff assembly against the SORTED TL-baked DAG
  (mirroring the oracle, which rebuilds via `UpdateTaskSetPriorities`):
  ```cpp
  // dag_tasks_cur already TL-baked at :194 (UpdateExtDistBasedOnTimeLimit).
  TaskSet tasks_sorted = UpdateTaskSetPriorities(dag_tasks_cur.tasks,
                                                 challenger.opt_pa_);
  DAG_Model dag_eval = dag_tasks_cur;
  dag_eval.tasks = tasks_sorted;
  const std::vector<FiniteDist>& rtas =
      rta_cache_.Evaluate(dag_tasks_, challenger.opt_pa_, time_limits);  // RAW dag + tl
  std::vector<FiniteDist> path_lats =
      GetRTDA_Dist_AllChains<ObjReactionTime>(dag_eval);
  challenger.opt_sp_ = ObtainSP_DAG_From_Dists_With_Perf_Coeff(
      dag_eval, sp_parameters_, rtas, path_lats);
  ```
  `:249`/`:287` (`OptimizeIncre_SingleTask`) stay on the oracle this increment
  (base class, Hazard A — deferred to 3/N).

### Verification (2b)
- Add a differential test (assert cache-eval SP == oracle
  `EvaluateSPWithPriorityVec` SP, bit-identical, on serialized-path fixtures).
- `ctest --test-dir build -R SP_OPT --output-on-failure` → expect 16/16 green.
- `./build/tests/testRTA` → expect 46/46 green (cache tests untouched).

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
  `ObtainSP_DAG_From_Dists_With_Perf_Coeff(...)` (same assembly as 2b).
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

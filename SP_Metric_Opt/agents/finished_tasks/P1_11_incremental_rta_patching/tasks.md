# P1.11 — Tasks (working checklist)

> See `goal.md` for scope and design.
> One small sub-task at a time, review + commit after each.
>
> **Scope narrowed 2026-07-18:** P1.11 owns the cache design + build only. The
> integration (wiring the cache into the optimizer, dispatching it in the hot
> loops) moved to **P1.12** — see
> [`../P1_12_integrate_rta_cache/tasks.md`](../P1_12_integrate_rta_cache/tasks.md).
> The Phase 1 / Phase 2 sections below are retained as historical record of the
> original plan; they are tracked in P1.12 now.

---

## Phase 0 — Cache + API + Direct Unit Tests ✅ DONE (working tree, NOT committed) → P1.11 scope complete

> P1.11's owned scope (cache design + build) is complete here. The remaining
> integration work below (Phase 1 + Phase 2) moved to P1.12 on 2026-07-18.

- [x] Rev-3 single-champion `RTACache` class (`RTA_Cache.h/.cpp`): `Initialize` / `AdoptChampion` / `Evaluate` / `ComputeTaskSetDifference` / `IsSingleTaskChange` / `ClassifyReusePerTask`.
- [x] Priority-analysis utilities extracted to leaf header `PrioritySwitchAnalysis.h` (`RestEqualAfterRemoving` two-pointer walk, `FindCoreOfTask`, `AnalyzePrioritySwitchPerCore`, `AnalyzePrioritySwitch`).
- [x] Differential tests (bit-identical to `ProbabilisticRTA_TaskSet` oracle) + 24 direct unit tests for the priority-analysis helpers.
- [x] **16/16 ctest + 46/46 testRTA green.**

---

## Phase 1 — Wire Cache into Live Eval Path (Step 3b, baseline-only) ← NEXT

- [ ] **Infrastructure wiring**:
  - Add `RTACache rta_cache_` member to `OptimizePA_Incre_with_TimeLimits` (in `OptimizeSP_TL_Incre.h`).
  - Clear/reset the cache at the start of each interval's walk in `ResetIncumbentBaseline` (in `OptimizeSP_TL_Incre.cpp`).
  - Add a required `RTACache&` parameter to the incremental optimizer seam:
    - Thread it from `EvaluateTimeLimitConfig_SubIncremental` down to `OptimizeIncre_SingleTask` (in `OptimizeSP_Incre.cpp`).
    - Note: This ensures cache is always active and threaded by reference without slicing base/derived boundaries (resolves Hazard A).
  - Wire the commit point: Call `rta_cache_.AdoptChampion(...)` inside `CommitIncumbent(...)` (in `OptimizeSP_TL_Incre.cpp`).

- [ ] **Replace evaluations inside `OptimizeIncre_SingleTask`**:
  - In the baseline re-score and per-variation walk loops, replace `EvaluateSPWithPriorityVec` with calls to `rta_cache_.Evaluate(...)`.
  - Implement a named free helper function to assemble the full SP from the cache's node RTAs + path latencies, ensuring the `perf_coefficient` is correctly multiplied (resolves Hazard B).

- [ ] **Verification**:
  - Update `tests/testOptimizeIncrePA.cpp` to pass a local `RTACache` instance to `OptimizeIncre`.
  - Write differential tests asserting that cache-aware evaluations produce bit-identical SP to the old oracle.
  - Run all tests to verify (16/16 ctest green in DEBUG).

---

## Phase 2 — Dispatch Cache in Hot Loops (Patching)

- [ ] **TL patch dispatch (Loop B)**:
  - Optimize the TL walk (`OptimizeSingleTaskTimeLimit`) to utilize `rta_cache_.Evaluate(...)` under the single-task change patch path.
  - Verify via differential tests vs the full recompute.
- [ ] **Priority-move patch dispatch (Loop A)**:
  - Optimize the 1D priority walk to utilize `Evaluate(...)` patch path.
  - Verify via differential tests.
- [ ] **End-to-end scalability measurement**:
  - Profile the candidate evaluation time at N=6/10/16 with and without the cache.
  - Document the speedup.

# P1.11 — Tasks (working checklist)

> See `goal.md` for scope and design.
> One small sub-task at a time, review + commit after each.

---

## Phase 1 — Wire Cache into Live Eval Path (Step 3b, baseline-only)

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

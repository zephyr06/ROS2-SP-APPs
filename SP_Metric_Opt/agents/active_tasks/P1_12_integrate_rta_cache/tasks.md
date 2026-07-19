# P1.12 — Tasks (working checklist)

> See `goal.md` for scope and design.
> One small sub-task at a time, review + commit after each.
> Split out of P1.11's former Phase 1 + Phase 2 on 2026-07-18.

---

## State (working tree, 2026-07-19, post-2a-re-land; NOT committed)

**Increment 2a RE-LANDED** (working tree, NOT committed — agents only `git add`).
The 2a write-side scaffolding (`rta_cache_` member + `rta_cache_active_` gate +
gated `AdoptChampion` in `CommitIncumbent` + reset in `ResetIncumbentBaseline`,
on `OptimizePA_Incre_with_TimeLimits` in `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}`)
is back, re-landed by hand from `p1_12_increment_2a_backup.patch` (the patch's
P1.14-mirror BF hunks were SKIPPED — already committed at HEAD `ecf0c597`+
`d7de4ad1`). **Behavior-preserving:** the cache is WRITTEN at `CommitIncumbent`
but NOT yet READ — the oracle `EvaluateSPWithPriorityVec` still answers every SP
eval. **17/17 ctest green** (19.74s), all SP outputs bit-identical to HEAD.

Cumulative uncommitted diffs vs HEAD `71da8a45`: the 2b fix
(`sources/Safety_Performance_Metric/RTA_Cache.cpp` + `tests/testRTA.cpp`) +
this 2a re-land (`sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}`, +53/-0).

**Still no live TL-path READ.** The serialized eval still calls the ORACLE
`EvaluateSPWithPriorityVec` at `OptimizeSP_TL_Incre.cpp:247`. The 2a write-side
keeps the cache warm (champion tracks `res_opt_`); the 2b read-side swap will
consume it.

### The 2b BLOCKER — RESOLVED 2026-07-19

Root cause confirmed + isolated at the primitive level: the two
`GetRTA_OneTask` overloads (RTA.cpp:32 2-arg vs RTA.cpp:46 3-arg) are NOT
equivalent on wide-ET multi-HP input. The 2-arg form Compresses+Convolves the
running RTA PER HP task (`n_hp` compresses); the 3-arg form Compresses ONCE then
Convolves against a pre-built rolling HP-ET prefix (1 compress).
`CompressDistributionWithOnlySize` is LOSSY once support > `Granularity`, so the
differing compress count diverges. The 3-arg form IS the oracle's definition of
correct (`ProbabilisticRTA_TaskSet_SingleCore` calls it at RTA.cpp:105).
Pre-fix, `Evaluate`'s NoReuse path called the 2-arg form (RTA_Cache.cpp:458) →
diverged. Fix: the NoReuse walk now mirrors the oracle's loop exactly (rolling
`hp_tasks_et_conv` + 3-arg `GetRTA_OneTask`). Full detail in `dev_log.md`
(2026-07-19 2b-resolved entry).

---

## Phase 1 — Wire Cache into Live Eval Path (baseline-only)

- [x] **0. BLOCKER — fix `RTACache::Evaluate` NoReuse bit-identity FIRST (TDD).** DONE 2026-07-19.
  - [x] Add a NEW differential test in `tests/testRTA.cpp` with WIDE ET
    distributions. The load-bearing reproduction is the DIRECT primitive
    differential `GetRTA_OneTaskDifferential.TwoArgDivergesFromThreeArgOnWideEt`
    (calls both overloads on identical wide-ET 2-HP input, asserts they DIFFER —
    pins the mechanism; an `Evaluate`-level repro on the same fixture was tried
    but is NOT load-bearing because the fixture's task 2 never lands as a
    wide-ET NoReuse task with >=2 wide HP tasks in a divergent shape).
  - [x] Trace the exact compress/convolve divergence: 2-arg Compresses the running
    RTA `n_hp` times; 3-arg Compresses ONCE. `CompressDistributionWithOnlySize`
    (Probability.cpp:382 → :335, returns early when `size <= max_size`) is LOSSY
    past `Granularity` → the compress-count difference diverges. (NOT the
    `if_new_preempt` flag — that was the prior hypothesis, wrong; both forms share
    `ResolvePreemptionsAndCompress` identically.)
  - [x] Fix `Evaluate` to reproduce the oracle's compress-count + convolution
    rolling order EXACTLY (RTA_Cache.cpp:452-464: per core, rolling
    `hp_tasks_et_conv` via `RollPrefix`, 3-arg `GetRTA_OneTask`). 7 `Evaluate_*`
    tests + P1.13 differential + 3 wide-ET repro tests + the new guard all green;
    17/17 ctest.

- [ ] **1. Infrastructure wiring** (re-land the 2a scaffolding from
  `p1_12_increment_2a_backup.patch` AFTER the blocker is fixed — order matters:
  the read-side swap cannot land before `Evaluate` is bit-identical).
  - [x] ~~`RTACache rta_cache_` member on `OptimizePA_Incre_with_TimeLimits`~~
    (DONE then REVERTED by P1.14 2c; backup at `p1_12_increment_2a_backup.patch`).
  - [x] ~~`bool rta_cache_active_` gate + `CommitIncumbent` adopt + reset in
    `ResetIncumbentBaseline`~~ (DONE then REVERTED; same backup patch).
  - [x] **RE-LANDED 2026-07-19** — `rta_cache_` member + `rta_cache_active_` gate
    + gated `AdoptChampion` in `CommitIncumbent` + reset in
    `ResetIncumbentBaseline`, re-landed by hand from the backup patch (the
    patch's P1.14-mirror BF hunks SKIPPED — already committed at HEAD).
    Behavior-preserving: cache WRITTEN, NOT yet READ. 17/17 ctest green.
  - [ ] Add a required `RTACache&` parameter to the incremental optimizer seam:
    thread from `EvaluateTimeLimitConfig_SubIncremental` down to
    `OptimizeIncre_SingleTask` (in `OptimizeSP_Incre.cpp`); ensures the cache is
    always active and threaded by reference without slicing base/derived
    boundaries (resolves Hazard A). — DEFERRED to Phase 2 (Hazard A).
  - [x] ~~Wire the commit point: call `rta_cache_.AdoptChampion(...)` inside
    `CommitIncumbent(...)`~~ (DONE in this 2a re-land; gated by
    `rta_cache_active_`).

- [ ] **2. Replace evaluations inside the TL-walk re-score**:
  - Swap `EvaluateSPWithPriorityVec` for `rta_cache_.Evaluate(...)` at the
    `:247` baseline re-score in `EvaluateTimeLimitConfig_SubIncremental`.
  - Hazard B (perf_coefficient) is already FIXED IN PLACE at HEAD by P1.13
    (`ObtainSP_DAG_From_Dists` multiplies `perf_coefficient`); reuse that helper,
    do NOT add a parallel `*_With_Perf_Coeff`. (The `implementation_plan.md`
    note about needing a new helper is SUPERSEDED.)
  - `:249`/`:287` (`OptimizeIncre_SingleTask` full swap) stay on the oracle this
    phase — base-class threading is Hazard A, deferred to Phase 2.

- [ ] **3. Verification**:
  - Differential test asserting cache-eval SP == oracle SP bit-identical on the
    TL walk (the `testIncreOpt_w_TL::OptimizeWithOptimizationSpace` that surfaced
    the blocker must go green).
  - 16/16 ctest green in DEBUG.

---

## Phase 2 — Dispatch Cache in Hot Loops (Patching) + base-class threading

- [ ] **Base-class `RTACache&` threading (Hazard A, the `:249`/`:287` swap)**:
  thread the cache from `EvaluateTimeLimitConfig_SubIncremental` into the base
  `OptimizePA_Incre::OptimizeIncre_SingleTask` so the per-variation walk uses the
  cache. (P1.13 already wired `OptimizeIncre_SingleTask`'s cache branch on the
  priority path; this threads the TL path into the same branch.)
- [ ] **TL patch dispatch (Loop B)**:
  - Optimize the TL walk (`OptimizeSingleTaskTimeLimit`) to utilize
    `rta_cache_.Evaluate(...)` under the single-task change patch path.
  - Verify via differential tests vs the full recompute.
- [ ] **Priority-move patch dispatch (Loop A)**:
  - Optimize the 1D priority walk to utilize `Evaluate(...)` patch path.
  - Verify via differential tests.
- [ ] **End-to-end scalability measurement**:
  - Profile the candidate evaluation time at N=6/10/16 with and without the
    cache.
  - Document the speedup.

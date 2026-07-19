# P1.12 — Tasks (working checklist)

> See `goal.md` for scope and design.
> One small sub-task at a time, review + commit after each.
> Split out of P1.11's former Phase 1 + Phase 2 on 2026-07-18.

---

## State at HEAD (`d7de4ad1`, re-verified 2026-07-19)

**P1.12 has ZERO source diffs vs HEAD.** The 2a write-side work recorded below as
DONE was REVERTED by P1.14 Phase 2c "split clean" — all RTA-cache hunks were
removed from `OptimizeSP_TL_Incre.{h,cpp}`, backed up at
`p1_12_increment_2a_backup.patch` (9377 bytes, on disk), and only the P1.14-mirror
time-limit-budget hunks were committed. So **no TL-path cache integration is live
at HEAD** — the serialized eval still calls the ORACLE `EvaluateSPWithPriorityVec`
at `OptimizeSP_TL_Incre.cpp:247`.

What IS live at HEAD (not P1.12's work, but consumes the same cache): P1.13
(priority path) committed in `3e9b6518`, always-engaged inside `OptimizeIncre`.

### The 2b BLOCKER (unresolved, gates all read-side work)

`RTACache::Evaluate`'s NoReuse recompute path is NOT bit-identical to the oracle
`ProbabilisticRTA_TaskSet`. Surfaced by `testIncreOpt_w_TL::OptimizeWithOptimizationSpace`
(incremental SP 10.2046 > scratch 9.67111). The baseline re-score path (`|diff|==0`,
`FullReuse`, `Initialize`) IS bit-identical; only the per-variation NoReuse path
diverges (up to ~0.53 SP off). Full root-cause + the attempted+reverted fix are in
`dev_log.md` (2026-07-18 2b entry + 2026-07-19 re-anchoring). The blocker also
makes P1.13 a latent correctness hazard (P1.13's small-ET fixtures stay under
`Granularity` and mask the divergence).

---

## Phase 1 — Wire Cache into Live Eval Path (baseline-only)

- [ ] **0. BLOCKER — fix `RTACache::Evaluate` NoReuse bit-identity FIRST (TDD).**
  - [ ] Add a NEW differential test in `tests/testRTA.cpp` with WIDE ET
    distributions (convolved support >> `Granularity`) asserting
    `RTACache::Evaluate` on a single-task-TL-change candidate (NoReuse path) is
    bit-identical to `OracleRtas` / `ProbabilisticRTA_TaskSet_SingleCore`.
    Must FAIL on HEAD (reproduces the 2b blocker in isolation, cheaper than the
    full `testIncreOpt_w_TL` integration).
  - [ ] Trace the exact compress/convolve divergence between `Evaluate`'s NoReuse
    path (2-arg `GetRTA_OneTask(task, hp_tasks)`, Compress+Convolve PER HP task)
    and the oracle's 3-arg path (ONE Compress, Convolve against precomputed
    rolling HP-ET convolution). Likely suspect: `ResolvePreemptionsAndCompress`'s
    `if_new_preempt` flag differs between the two forms (2-arg derives it from the
    running RTA's `max_time`; 3-arg from the precomputed convolution).
  - [ ] Fix `Evaluate` to reproduce the oracle's compress-count + convolution
    rolling order EXACTLY. Must keep the 7 `Evaluate_*` tests + the P1.13
    `OptimizeIncre_SingleTask.Differential_BitIdenticalOnSingleEtChange` test green
    (the fix makes the cache MORE correct, never less).

- [ ] **1. Infrastructure wiring** (re-land the 2a scaffolding from
  `p1_12_increment_2a_backup.patch` AFTER the blocker is fixed — order matters:
  the read-side swap cannot land before `Evaluate` is bit-identical).
  - [x] ~~`RTACache rta_cache_` member on `OptimizePA_Incre_with_TimeLimits`~~
    (DONE then REVERTED by P1.14 2c; backup at `p1_12_increment_2a_backup.patch`).
  - [x] ~~`bool rta_cache_active_` gate + `CommitIncumbent` adopt + reset in
    `ResetIncumbentBaseline`~~ (DONE then REVERTED; same backup patch).
  - [ ] Clear/reset the cache at the start of each interval's walk in
    `ResetIncumbentBaseline` (in `OptimizeSP_TL_Incre.cpp`).
  - [ ] Add a required `RTACache&` parameter to the incremental optimizer seam:
    thread from `EvaluateTimeLimitConfig_SubIncremental` down to
    `OptimizeIncre_SingleTask` (in `OptimizeSP_Incre.cpp`); ensures the cache is
    always active and threaded by reference without slicing base/derived
    boundaries (resolves Hazard A).
  - [ ] Wire the commit point: call `rta_cache_.AdoptChampion(...)` inside
    `CommitIncumbent(...)` (in `OptimizeSP_TL_Incre.cpp`).

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

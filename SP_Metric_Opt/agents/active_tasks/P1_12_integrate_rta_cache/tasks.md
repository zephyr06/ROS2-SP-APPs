# P1.12 — Tasks (working checklist)

> See `goal.md` for scope and design.
> One small sub-task at a time, review + commit after each.
> Split out of P1.11's former Phase 1 + Phase 2 on 2026-07-18.

---

## State (2026-07-19, Phase 1 COMPLETE + COMMITTED; Phase 2 item 1 DONE in working tree)

HEAD = `c0e1bde0` ("add more tests"). The Phase 1 increments are ALL COMMITTED:
- `71da8a45` = 2b blocker fix (`RTA_Cache.cpp` NoReuse bit-identity + `testRTA.cpp`).
- `8e18c39b` = 2a write-side re-land (`OptimizeSP_TL_Incre.{h,cpp}` + task docs).
- `5a172973` = 2b READ-SIDE SWAP (`OptimizeSP_TL_Incre.{h,cpp}`, +65/-31).
- `c0e1bde0` = Phase 1 step 3 differential tests (`tests/testRTA.cpp` +83).

So 2a (write-side), the 2b `Evaluate` fix, AND the 2b read-side swap are LIVE at
HEAD. The `:247` baseline re-score in `EvaluateTimeLimitConfig_SubIncremental`
now routes through `rta_cache_.Evaluate` + `ObtainSP_Full_From_NodeRTAs`
(P1.13's helper, which mirrors the oracle body + handles perf_coefficient =
Hazard B). Wrapped in `BFSharedBudgetCancelled()` checks to preserve the P1.14
cancel contract (INT_MIN on cancel → discarded by the strict-> adopt guard).
Double-bake is idempotent + `TryComputeSingleChange` bakes both sides before
diffing → invariant holds (Type-L → |diff|==1 patch, Type-E → |diff|==0
FullReuse). **17/17 ctest green** re-verified from clean build (18.51s);
differential gate `OptimizeWithOptimizationSpace` passes (cache read-side
bit-identical to oracle on the full serialized walk).

**Phase 2 item 1 (`:285` flip) DONE 2026-07-19 in the working tree (NOT committed):**
`EvaluateTimeLimitConfig_SubIncremental`'s `:285` call now passes
`std::ref(rta_cache_)` into `OptimizeIncre_SingleTask` (3-arg → 4-arg), so the
TL-walk's per-variation priority search shares the serialized champion's warm
cache (was: throwaway local cache per call). Unblocked by the 1b reindex fix in
`RTACache::Evaluate` (FullReuse seeding reindexed by task-id, not positional
copy — fixes the PA-move scramble). `OptimizeWithOptimizationSpace` gate PASSES
with the flip live; 17/17 ctest green (23.13s). `:249`/`:289` (the OTHER
`OptimizeIncre_SingleTask` call sites in the TL path) — `:285` was THE hot-loop
call; the remaining Phase 2 work is Loop A/B dispatch refinement + measurement.

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

- [x] **2. Replace evaluations inside the TL-walk re-score** (COMMITTED `5a172973`):
  - [x] Swap `EvaluateSPWithPriorityVec` for `rta_cache_.Evaluate(...)` at the
    `:247` baseline re-score in `EvaluateTimeLimitConfig_SubIncremental`.
    Uses `ObtainSP_Full_From_NodeRTAs` (P1.13's helper) for the SP assembly.
  - [x] Hazard B (perf_coefficient) — already FIXED IN PLACE at HEAD by P1.13
    (`ObtainSP_DAG_From_Dists` multiplies `perf_coefficient`, called inside
    `ObtainSP_Full_From_NodeRTAs`). Reused; did NOT add a parallel
    `*_With_Perf_Coeff`. (The `implementation_plan.md` note about needing a new
    helper is SUPERSEDED.)
  - [x] P1.14 cancel contract preserved: `BFSharedBudgetCancelled()` checks at
    entry + post-Evaluate → INT_MIN on cancel (mirrors oracle's cancel contract).
  - [ ] `:249`/`:289` (`OptimizeIncre_SingleTask` full swap) stay on the oracle
    this phase — base-class threading is Hazard A, deferred to Phase 2.

- [x] **3. Verification**:
  - [x] `testIncreOpt_w_TL::OptimizeWithOptimizationSpace` (the differential
    gate that surfaced the 2b blocker) PASSES — cache read-side bit-identical to
    oracle on the full serialized walk (`res_incre.sp_opt <= res_scratch.sp_opt`,
    TL[0]==400).
  - [x] 17/17 ctest green in DEBUG, zero warnings.
  - [x] Focused direct differential test: `rta_cache_.Evaluate`+
    `ObtainSP_Full_From_NodeRTAs` == `EvaluateSPWithPriorityVec` bit-identical on
    a TL-walk fixture. DONE 2026-07-19: 2 new tests in `tests/testRTA.cpp`
    (`SP_Assembly_TypeLChange_BitIdenticalToOracle` pins the |diff|==1 Type-L
    patch branch; `SP_Assembly_TypeE_NoChange_BitIdenticalToOracle` pins the
    |diff|==0 Type-E FullReuse branch) + an `OracleSP` helper
    (`UpdateExtDistBasedOnTimeLimit`→`EvaluateSPWithPriorityVec`). Both
    `EXPECT_DOUBLE_EQ` (exact) against the oracle. 17/17 ctest green (18.59s).

---

**Phase 1 COMPLETE.** The cache is wired into the live serialized TL-walk
read-side (2a write-side + 2b read-side swap + 2b blocker fix all committed;
step-3 differential pins the bit-identity at the seam). Phase 2 (base-class
`RTACache&` threading into `OptimizeIncre_SingleTask:289` = Hazard A + Loop A/B
dispatch + measurement) is NOT started.

---

## Phase 2 — Dispatch Cache in Hot Loops (Patching) + base-class threading

- [x] **1. `:285` cache dispatch (the `OptimizeIncre_SingleTask` call in
  `EvaluateTimeLimitConfig_SubIncremental`)** — DONE 2026-07-19 (working tree,
  NOT committed). NOT the one-liner the old "base-class threading" framing
  implied. P1.13 already did the threading (the `RTACacheOpt` param + cache
  branch in `OptimizeIncre_SingleTask` exist + are used at the `:428`
  priority-path call). Item 1 = feed `std::ref(rta_cache_)` at `:285`
  (3-arg → 4-arg). **ROOT CAUSE FOUND + FIXED 2026-07-19**: the divergence was
  a **cache-INTERNAL bug in `RTACache::Evaluate` (`RTA_Cache.cpp:410-475`), NOT
  the P1.12 integration**. `Evaluate` seeded `candidate_rta_ = rta_` (a positional
  copy of champion RTAs indexed by CHAMPION priority-position) then overwrote
  NoReuse slots using CANDIDATE priority-position indexing; when candidate PA ≠
  champion PA (a 1D PA move — exactly what `:285` sends), the FullReuse slots
  held the WRONG task's RTA → scramble. The existing `OptimizeIncre_Cache`
  differentials missed this because they mutate `execution_time_dist` DIRECTLY
  (candidate PA == champion PA → no reindex → no scramble). `:247` re-score was
  bit-identical only because there PA == champion PA.
  - [x] **1a. Write the missing primitive-level differential test** —
    candidate PA = a 1D priority MOVE of the champion PA (NOT a direct
    `execution_time_dist` mutation), cache-engaged vs oracle, asserting
    `EXPECT_DOUBLE_EQ` per-task RTA + SP. Two tests in `tests/testRTA.cpp`:
    `Evaluate_PriorityMove_CrossCoreScramble_BitIdenticalToOracle` (per-task
    RTA, the load-bearing pin) + `Evaluate_PriorityMove_CrossCoreScramble_SP_*
    BitIdenticalToOracle` (SP-level). Goes RED on `Evaluate` pre-fix.
  - [x] **1b. Apply the fix** in `RTACache::Evaluate` (`RTA_Cache.cpp:425-459`):
    when seeding `candidate_rta_`, reindex `rta_` from champion priority-
    positions to candidate priority-positions (map by task-id:
    `candidate_rta_[cand_pos(t)] = rta_[champ_pos(t)]` for every FullReuse task).
    1a → GREEN. Fix is internal to `Evaluate` — no API change.
  - [x] **1c. Re-flip `:285`** to `std::ref(rta_cache_)`;
    `OptimizeWithOptimizationSpace` gate PASSES (cache/incre SP ≤ scratch SP,
    adopted TL==400). 17/17 ctest green.

- [ ] **Base-class `RTACache&` threading (Hazard A)** — DONE by P1.13 (the
  `RTACacheOpt` param on `OptimizeIncre_SingleTask`/`OptimizeIncre`). The
  stale "thread `RTACache&` into the base class" framing in `goal.md` is
  superseded; the remaining work is the `:285` *call-site* fix (item 1), not
  base-class threading.
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

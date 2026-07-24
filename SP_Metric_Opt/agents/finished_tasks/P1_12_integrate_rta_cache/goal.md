# P1.12 — Integrate RTA Cache into the Incremental Optimizer

> Split out of **P1.11** on 2026-07-18. P1.11's scope is *the cache design +
> build* (Phase 0, DONE, committed `843f9603`); P1.12 owns the **integration**
> of that cache into the live incremental+TL optimizer eval path (P1.11's former
> Phase 1 + Phase 2). The `RTACache` class + `PrioritySwitchAnalysis.h` utilities
> + their tests are the contract P1.12 consumes — see
> [`../P1_11_incremental_rta_patching/`](../P1_11_incremental_rta_patching/).

---

## Current state (2026-07-19; Phase 1 + Phase 2 item 1 + P1.16 cache_backup ALL COMMITTED)

HEAD = `77effccb`. Full increment chain COMMITTED:
- `71da8a45` = 2b blocker fix (`RTA_Cache.cpp` NoReuse bit-identity + `testRTA.cpp`).
- `8e18c39b` = 2a write-side re-land (`OptimizeSP_TL_Incre.{h,cpp}` + task docs).
- `5a172973` = 2b READ-SIDE SWAP (`OptimizeSP_TL_Incre.{h,cpp}`, +65/-31).
- `c0e1bde0` = Phase 1 step 3 differential tests (`tests/testRTA.cpp` +83).
- `bfbec7e5` = Phase 2 item 1 (`:285` flip + 1a tests + 1b `Evaluate` reindex fix).
- `1217d227` = P1.16 cache_backup on rejected walks (`UpdateRecords` returns
  `bool`; `rta_cache_` restored when a candidate is NOT adopted → closes the
  Reopt_X>1 SIGABRT desync; task moved to `finished_tasks/`).

The serialized TL-walk baseline re-score in
`EvaluateTimeLimitConfig_SubIncremental` now routes through `rta_cache_.Evaluate`
+ `ObtainSP_Full_From_NodeRTAs` (P1.13's helper; mirrors the oracle body, handles
perf_coefficient = Hazard B), wrapped in `BFSharedBudgetCancelled()` checks
(P1.14 cancel contract). **17/17 ctest green** (20.37s) from a fresh
`-DCMAKE_BUILD_TYPE=DEBUG` build; differential gate `OptimizeWithOptimizationSpace`
passes → cache read-side bit-identical to the oracle on the full serialized walk,
BOTH `:247` (baseline re-score) AND `:285` (per-variation `OptimizeIncre_SingleTask`).

**NEXT:** Phase 2 Loop A/B dispatch is CONFIRMED COMPLETE (grounded read
2026-07-19: the production incremental path `Optimize_w_TL_ScratchOrIncre` →
`OptimizeIncre_w_TL` → `PerformSerializedTaskQueueOptimization` →
`EvaluateTimeLimitConfig_SubIncremental` already routes both Loop B (`:247`
re-score + `:285` `OptimizeIncre_SingleTask`) and Loop A (`:437` inside
`OptimizeIncre`, cache forced engaged at `:369-371`) through the shared
`rta_cache_`; the remaining `EvaluateSPWithPriorityVec` sites are one-shot
baselines with a cold cache (`:460`/`:846`/`:856` in `ResetIncumbentBaseline` +
the `OptimizeFromScratch` final score `:162`) or dead defensive arms
(`:339`/`:396`)). The ONLY remaining P1.12 work is the **end-to-end
scalability measurement at N=6/10/16** (cache on vs off) — a release-binary
profiling run, NOT a code change. Gated on P1.15 Phase 3 or a standalone
micro-profile.

---

## Why this exists (the lever)

Every SP candidate the optimizer evaluates pays one full N-task RTA. Two hot
loops change only one task per step, and the P1.10 single-change invariant
(`|diff| <= 1` vs the running champion per SP-eval) makes that RTA reusable:

- **Loop B (TL walk)** — `OptimizeSingleTaskTimeLimit`: changes one task's TL →
  `|diff| == 1`.
- **Loop A (1D priority walk)** — `OptimizeIncre_SingleTask`: moves one task to
  a new priority position → `|diff| == 1`.
- **Type-E (env step)** — `|diff| == 0` (same DAG, only PA varies; RTA is
  PA-independent → reuse verbatim).

The rev-3 `RTACache` (built in P1.11) turns each of these into a cheap
single-task patch instead of a full recompute. P1.12 wires that cache into the
optimizer so the hot loops actually call it.

---

## Scope (what P1.12 owns)

1. **Phase 1 — wire the cache into the live eval path (baseline-only).**
   - Add `RTACache rta_cache_` member to `OptimizePA_Incre_with_TimeLimits`.
   - Reset the cache at the start of each interval's walk in
     `ResetIncumbentBaseline`.
   - Thread a required `RTACache&` from `EvaluateTimeLimitConfig_SubIncremental`
     down to `OptimizeIncre_SingleTask` (resolves **Hazard A** — by reference,
     no base/derived slicing).
   - Call `rta_cache_.AdoptChampion(...)` inside `CommitIncumbent(...)` (the
     single champion-writer, the natural cache-adopt point).
   - Replace `EvaluateSPWithPriorityVec` with `rta_cache_.Evaluate(...)` in
     `OptimizeIncre_SingleTask`'s baseline re-score + per-variation walk.
   - Implement a named free helper to assemble the full SP from the cache's node
     RTAs + path latencies, ensuring `perf_coefficient` is correctly multiplied
     (resolves **Hazard B**).

2. **Phase 2 — dispatch the cache in the hot loops (patching).**
   - TL patch dispatch (Loop B): `OptimizeSingleTaskTimeLimit` uses
     `rta_cache_.Evaluate(...)` under the single-task-change patch path.
   - Priority-move patch dispatch (Loop A): the 1D priority walk uses the
     `Evaluate(...)` patch path.
   - End-to-end scalability measurement at N=6/10/16, with vs without the cache.

## Out of scope / non-goals

- No change to the cache API itself (that is P1.11's surface; P1.12 only
  *calls* it). If the integration reveals an API gap, file it back to P1.11.
- No change to the generator, topology, or baseline priority assignment.
- Correctness is paramount: cache-aware evaluations must be **bit-identical** to
  the full recomputation (`EvaluateSPWithPriorityVec` oracle), verified by
  differential TDD.
- The v2 same-core-suffix refinement (`ReuseHpTasksEt`, reserved but not emitted
  by the v1 `Evaluate`) is NOT part of P1.12 — it is a future optimization on
  top of a landed, wired, measured v1.

---

## Hazards to resolve during integration

- **Hazard A (slicing / lifetime):** the cache must be threaded by reference
  (`RTACache&`) through the incremental optimizer seam — stored once on the
  derived owner (`OptimizePA_Incre_with_TimeLimits`), not copied or sliced
  across the base/derived `OptimizeIncre` boundary.
- **Hazard B (perf_coefficient):** the SP-assembly helper that consumes the
  cache's node RTAs + path latencies must multiply `perf_coefficient` correctly;
  the old `EvaluateSPWithPriorityVec` path did this implicitly and the
  replacement must preserve it exactly (differential tests guard this).

## Grounded integration points (verified in source, from P1.11)

- `CommitIncumbent(pa, sp, tl)` (`OptimizeSP_TL_Incre.cpp:699`) — single writer
  of `res_opt_` (the champion) → the cache-adopt point (`AdoptChampion`).
- `BuildChallengerFromIncumbent()` (`:716`) — rebuilds the throwaway challenger
  from `res_opt_` → the cache's diff base = the champion.
- `EvaluateTimeLimitConfig_SubIncremental(K, time_limits, task_idx, et_increased)`
  (`:178`) — the serialized eval (the single-change invariant's home); calls
  `BuildChallengerFromIncumbent` then `OptimizeIncre_SingleTask`.
- `ResetIncumbentBaseline` — the per-interval reset point for the cache.
- `ReconstructTimeLimitVecFromResOpt()` (`:657`) — champion TL as a positional
  vector (`dag.tasks[i].id == i` invariant holds).
- `EvaluateSPWithPriorityVec` — the oracle P1.12 replaces (differential
  baseline for the bit-identity tests).

# P1.17 — Tasks (working checklist)

> Refactor + perf. Bit-identical SP output is the only acceptance gate.
> TDD-first: pin each redundancy with a differential test (or an existing one)
> BEFORE touching it, prove the refactor is bit-identical, then commit per item.
> Per the agent rule: `git add` only; the user commits.

## Phase 0 — Inventory (no code changes)

- [ ] **0a. Enumerate the copy/recompute hotspots** in `RTA_Cache.cpp::Evaluate`
  (`:410-502`) + `EvaluateTimeLimitConfig_SubIncremental`
  (`OptimizeSP_TL_Incre.cpp:~270-300`). Candidates observed on first read:
  - `Evaluate` bakes BOTH `tasks_baked` (candidate, `:437-439`) AND
    `champ_baked`/`champ_prioritized` (champion, `:448-451`) per call — two
    `ApplyTimeLimitsToTasksExecutionTime` + two `UpdateTaskSetPriorities` per
    Evaluate. The champion bake is invariant across calls within one champion
    lifetime; it could be cached on `AdoptChampion`/`Initialize` and reused.
  - `PerCoreOrderFromPa` is called by both `ComputeTaskSetDifference` (candidate
    side) AND `ClassifyReusePerTask` (`:392`) on the SAME candidate `(dag, pa)`
    within one `Evaluate` → rebuilt at least twice per call.
  - `ExtractTaskSetPerProcessor(tasks_prioritized)` (`:476-477`) partitions the
    already-sorted `tasks_prioritized`; `PerCoreOrderFromPa` produced the same
    partition by a different route. Likely one partition suffices.
  - `RTACache cache_backup = rta_cache_;` (`OptimizeSP_TL_Incre.cpp:~187`) copies
    the WHOLE cache (champion triple + `rta_` + `hp_prefix_per_core_` +
    `candidate_rta_`) on EVERY `EvaluateTimeLimitConfig_SubIncremental` entry,
    even when `UpdateRecords` will adopt (the common case on the improving walk).
    Consider: only back up the parts `UpdateRecords` can mutate, or move the
    backup to the rejection branch.
  - `candidate_rta_.assign(rta_.size(), FiniteDist({Value_Proba(0, 1.0)}))`
    (`:444`) zero-initializes then overwrites every slot — the assign is
    redundant when every slot is written by the reindex or the recompute loop.
- [ ] **0b. Profile a single interval at N=6 and N=10** (release build) to rank
  the hotspots by actual cost before refactoring. Record the ranking in
  `dev_log.md`. (Defer if the profiling harness is not trivially available — the
  0a list is enough to start; profile to VALIDATE, not to discover.)
- [x] **0c. Confirm the differential gate that pins bit-identity.** Re-run
  `testIncreOpt_w_TL::OptimizeWithOptimizationSpace` + the `testRTA.cpp`
  `Evaluate_*` / `SP_Assembly_*` differentials GREEN on the unmodified tree as
  the baseline before any change. — DONE 2026-07-19: 17/17 ctest DEBUG PASS
  (17.71s) from `build/`.

## Phase 1 — Cache-internal redundancies (RTA_Cache.{h,cpp})

- [x] **1a. Cache the champion's baked+prioritized form on `AdoptChampion`/
  `Initialize`** so `Evaluate` does not re-bake the champion every call. Store
  `champ_prioritized_` (+ its `task_id2index`) as members, invalidated on the
  next `AdoptChampion`/`Initialize`. Differential GREEN.
  - DONE 2026-07-19 (Evaluate reindex block): `champ_prioritized_` member
    added; `Initialize`/`AdoptChampion` populate it; `Evaluate` reads it instead
    of re-baking. 17/17 green.
  - DONE 2026-07-20 (1a remainder — `TryComputeSingleChange`'s champion bake):
    added `champ_tasks_baked_` member (canonical-order TL-bake, the artifact
    `FindTaskWithDifferentEt` needs — NOT the pa-sorted `champ_prioritized_`,
    which would break index correspondence); `Initialize`/`AdoptChampion`
    populate it; `TryComputeSingleChange` reads it instead of re-baking. New pin
    `Evaluate_TLChange_AfterAdoptChampion_StaleBakeGuard` (multi-champion
    staleness). 17/17 ctest + 55/55 testRTA green.
- [~] **1b. Collapse the per-core partition to one compute per `Evaluate`.**
  `PerCoreOrderFromPa` (candidate side, used by `ComputeTaskSetDifference` +
  `ClassifyReusePerTask`) and `ExtractTaskSetPerProcessor` (used by the recompute
  loop) produce overlapping data; compute once, reuse. Differential GREEN.
  - DONE 2026-07-19 (Evaluate-side): `Evaluate` builds `per_core` once and
    derives the verdict inline (no `ClassifyReusePerTask` call). 17/17 green.
  - DONE 2026-07-20 (1b remainder — champion side): `TryComputeSingleChange`
    reads the cached `champ_per_core_` member (built in `Initialize`/
    `AdoptChampion` via the new `PerCoreOrderOfPrioritized` helper) instead of
    rebuilding `PerCoreOrderFromPa(dag_champion_, pa_champion_)` every call. New
    pin `Evaluate_PriorityMove_AfterAdoptChampion_StalePerCoreOrderGuard`
    (multi-champion PA-swap staleness). 17/17 ctest + 56/56 testRTA green. The
    CANDIDATE-side `PerCoreOrderFromPa` call remains (per-call `pa` genuinely
    changes; threading `Evaluate`'s already-built partition through the `const`
    `ComputeTaskSetDifference → TryComputeSingleChange` API is a deeper, separate
    increment).
- [~] **1c. Drop the redundant `candidate_rta_.assign(...)` zero-init** when
  every slot is provably written (prove via the reindex loop covering all
  FullReuse tasks + the recompute loop covering all NoReuse tasks). If a slot
  can be left unwritten in some path, KEEP the assign (correctness first).
  Differential GREEN.
  - DONE 2026-07-19: proved every slot is written — the reindex loop writes
    every champion task id into its candidate slot (FullReuse), the recompute
    loop overwrites the NoReuse slots, and under the single-change invariant
    candidate task set == champion task set so every candidate task id is a
    champion task id (reached by reindex). Replaced the `assign(N, identity-zero)`
    with `resize(N)` (sizes only, no per-slot FiniteDist construction; no-op when
    Initialize/AdoptChampion already sized the member). New TDD pin
    `Evaluate_IdentityCandidate_EverySlotFilled_NoZeroInit` (the |diff|==0 early-
    return path skips the recompute loop, so all slots must come from the reindex
    loop alone — the exact case a dropped-but-unwritten regression would trip).
    17/17 ctest DEBUG green (19.96s); 54/54 testRTA.
- [ ] **1d. Factor the shared "bake + prioritize + index" body** out of
  `Evaluate`, `ComputeTaskSetDifference`, `ClassifyReusePerTask` into a private
  helper returning a small struct (the baked tasks, the per-core order, the
  id→index map). Pure refactor; differential GREEN.
  - NOTE 2026-07-20: the candidate-side bake is duplicated across `Evaluate`
    (`:408-410`) and `IsSingleTaskChange` (`:261-262`) only in the single
    `ApplyTimeLimitsToTasksExecutionTime` call — they diverge after it (Evaluate
    sorts; IsSingleTaskChange uses `PerCoreOrderFromPa`, no sort), so a one-line
    wrapper is not a clean win. The deeper waste here is that `Evaluate`'s call
    to `ClassifyReusePerTask` (`:426`) → `ComputeTaskSetDifference` →
    `IsSingleTaskChange` RE-BAKES the same candidate `Evaluate` already baked
    (`:409` vs `:262`); fixing it = threading the baked form through the `const`
    query API, which is the real 1d increment.
- [x] **1e. Dedup the champion-bake block between `Initialize` and
  `AdoptChampion`.** The same 4-line sequence (`champ_tasks_baked_` →
  `champ_prioritized_` → `champ_per_core_` → `RebuildPrefixes`) was duplicated
  verbatim (Initialize `:199-205`, AdoptChampion `:221-225`). Extracted into
  `BakeChampionForms(dag, pa, tl)`; each caller keeps its own `rta_`/
  `candidate_rta_` logic (no optional arg). Pure extract-method; the only
  reorder is `RebuildPrefixes` preceding the `rta_` compute in `Initialize`
  (safe — independent reads of `champ_prioritized_`). Differential GREEN.
  - DONE 2026-07-20: 17/17 ctest DEBUG PASS (20.82s). See dev_log 2026-07-20.

## Phase 2 — Optimizer-path redundancies (OptimizeSP_TL_Incre / OptimizeSP_Incre)

- [ ] **2a. Narrow the `cache_backup` scope in
  `EvaluateTimeLimitConfig_SubIncremental`.** Either (i) back up only the
  members `UpdateRecords`/`AdoptChampion` can mutate, or (ii) defer the backup
  to the rejection branch (copy-on-reject). Measure: the copy is currently O(N)
  FiniteDists per call. Differential GREEN + crash repro
  (`taskset_3 INCR_Reopt_5`) still exits 0.
- [ ] **2b. Remove double-bakes across the TL-walk re-score + the
  `OptimizeIncre_SingleTask` call.** If `EvaluateTimeLimitConfig_SubIncremental`
  bakes the candidate DAG and then `OptimizeIncre_SingleTask` bakes it again on
  the same `(dag, pa, tl)`, pass the baked form through. Differential GREEN.
- [ ] **2c. Audit `OptimizeSP_Incre.cpp` for the same patterns** (per-variation
  rebuilds of HP sets, full-vector copies where a moved-from local suffices).
  File findings; fix only the ones that are clearly redundant, not speculative.

## Phase 3 — Verify + measure

- [ ] **3a. Full gate GREEN:** 17/17 ctest (DEBUG) + `tests/python/` 300/300 +
  the P1.15 crash suite; release build clean.
- [ ] **3b. Scalability measurement at N=6/10/16** (the P1.12 Phase 2 item 5
  gate, now meaningful because the cache is lean): record per-candidate Evaluate
  time before/after this task in `dev_log.md`. This is the same measurement
  P1.12 Phase 2 item 5 deferred; P1.17 unblocks it.
- [ ] **3c. Update `overall_tasks.md` + memory.** Mark P1.17 resolved; cross-
  link from P1.12 (the cache's measurement is now honest) and P1.18 (the
  refactor keeps the extension surface P1.18 needs clean).

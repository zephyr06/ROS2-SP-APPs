# P2.7 — Tasks (working checklist)

> Code quality & documentation hygiene. Zero code behavior changes.

## Phase 1 — Audit & Rule Definition

- [x] **1a. Define Comment Simplification Guiding Rules**
  - No task tags (`P1.x`), explain WHY not WHAT, concise (1-2 lines), remove stale narratives, clean header/impl split.
- [x] **1b. Audit Target Files for Excess Noise**
  - Identified task-tag counts: `RTA_Cache.{h,cpp}` (6 tags), `PrioritySwitchAnalysis.h` (3),
    `OptimizeSP_TL_Incre.{h,cpp}` (37 — heaviest), `OptimizeSP_Incre.{h,cpp}` (15). Concentrated in
    the incremental-optimizer + cache code. Multi-paragraph changelog narratives worst in
    `OptimizeSP_TL_Incre.cpp` (P1.10/P1.14/P1.25 changelog anecdotes).

## Phase 2 — Comment Refactoring

- [x] **2a. Clean `RTA_Cache.{h,cpp}`**
  - Stripped all 6 task tags (`P1.9`/`P1.10`/`P1.20`/`P1.25 D1=(b)`). Condensed the `champion_`
    private-member block (was 10 lines of `D1=(b)`/`EvaluateTimeLimitConfig_SubIncremental` changelog
    → 8 lines, drift-proof-by-construct invariant only) and the `ChampionState` docstring (dropped
    the P1.25 reject-path anecdote). Preserved all WHY-rationale: the safe-upper-bound gate, the
    reindex-by-task-id invariant, the 3-arg vs 2-arg `GetRTA_OneTask` bit-identity reasoning.
    **17/17 ctest green** (comment-only, zero behavior change). Tag count now 0/0.
- [x] **2b. Clean `OptimizeSP_TL_Incre.{h,cpp}`** (DONE 2026-07-31)
  - Stripped all 37 task tags + `§8c`/`§8` doc-section refs. Compressed the heaviest
    narrative blocks: `OptimizeIncreSingleTask` (4-paragraph cancel-contract +
    ghost-SP narrative → 2 focused blocks; ~35 lines cut), `SeedBaselineAndArmCache`
    re-sync rationale (dropped "5.5 crash fix" / "old reopt body" history, kept the
    >1-diff-throw WHY), `RunIntervalDescent` ("behavior-neutral bit-identical to the
    two bodies this replaces" stale changelog removed), `DeadlineMonotonicPriorityVec`
    (dropped "replaces former plain-RM" narrative), `ComputeSafeFallback` (removed a
    DUPLICATED loud-fail comment left by the §9b refactor — 2nd copy dropped, kept
    ONE at the call site). Preserved all WHY: cache-arming asymmetry, |diff|<=1
    invariant, sibling-isolation → byte-identical, worst-case-DAG stochastic
    dominance. `.h` 391→355, `.cpp` 1068→1013. Tag count now 0/0.
- [x] **2c. Clean `OptimizeSP_Incre.{h,cpp}`** (DONE 2026-07-31)
  - `OptimizeSP_Incre.h` already 0 tags. `OptimizeSP_Incre.cpp`: stripped the 2 tags
    (`P2.11 5.6b` cooperative-budget, `P0.5` throwaway-challenger) — kept the WHY
    (inert within budget → byte-identical; throwaway dies with the local, cross-
    interval invariant is res_opt_.id2time_limit). `PrioritySwitchAnalysis.h` was
    already at 0 tags from Phase 1b/2a; no change. Tag count now 0/0.

## Phase 3 — Verification

- [x] **3a. Build & Run Tests** (DONE 2026-07-31)
  - `cmake --build build_test --target check.SP_OPT -j5 --clean-first` → **17/17
    ctest green** (incl. the `testPublisher` flake this run). Comment-only; zero
    behavior change. `pytest` out of scope (C++ comment cleanup; no Python touched).

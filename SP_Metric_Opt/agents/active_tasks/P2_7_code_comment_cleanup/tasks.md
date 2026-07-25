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
- [ ] **2b. Clean `OptimizeSP_TL_Incre.{h,cpp}`**
  - Remove task tags and historical walk-step changelog narratives.
- [ ] **2c. Clean `OptimizeSP_Incre.{h,cpp}` & `PrioritySwitchAnalysis.h`**
  - Simplify inline comments around 1D search and priority switch helpers.

## Phase 3 — Verification

- [ ] **3a. Build & Run Tests**
  - Run `ctest` and `pytest` to confirm zero functional or compilation regressions.

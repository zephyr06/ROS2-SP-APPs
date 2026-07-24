# P2.7 — Tasks (working checklist)

> Code quality & documentation hygiene. Zero code behavior changes.

## Phase 1 — Audit & Rule Definition

- [x] **1a. Define Comment Simplification Guiding Rules**
  - No task tags (`P1.x`), explain WHY not WHAT, concise (1-2 lines), remove stale narratives, clean header/impl split.
- [ ] **1b. Audit Target Files for Excess Noise**
  - Identify lines containing `P1.` task references or redundant multi-paragraph comments.

## Phase 2 — Comment Refactoring

- [ ] **2a. Clean `RTA_Cache.{h,cpp}`**
  - Strip task tags, condense class/function docstrings to crisp 1-2 line summaries.
- [ ] **2b. Clean `OptimizeSP_TL_Incre.{h,cpp}`**
  - Remove task tags and historical walk-step changelog narratives.
- [ ] **2c. Clean `OptimizeSP_Incre.{h,cpp}` & `PrioritySwitchAnalysis.h`**
  - Simplify inline comments around 1D search and priority switch helpers.

## Phase 3 — Verification

- [ ] **3a. Build & Run Tests**
  - Run `ctest` and `pytest` to confirm zero functional or compilation regressions.

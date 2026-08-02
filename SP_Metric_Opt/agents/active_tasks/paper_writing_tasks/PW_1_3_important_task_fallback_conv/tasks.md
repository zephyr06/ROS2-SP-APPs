# PW.1.3 — Tasks (working checklist)

> See `goal.md` for scope. Third slice of the code read: important-task safe
> fallback + offline convergence loop. Mostly a reading task.

## Read the fallback / guarantee sources
- [ ] `sources/TaskModel/RegularTasks.h:111` — `is_important` (top-50% by `sp_weight`)
- [ ] `sources/TaskModel/WorstCaseDAG.cpp` — `BuildWorstCaseDagAcrossIntervals`
- [ ] `OptimizeFallback.{h,cpp}` — `RateMonotonicFastGroupLocked`, `AdoptRmFastFallbackIfUnschedulable` (throws on double-fail)
- [ ] `OptimizeSP_TL_Incre.{h,cpp}` — `ComputeSafeFallback`, `AdoptFallbackIfUnschedulable`, `enable_fallback_use_`, `OptimizeIncre_w_TL` (virtual), `OptimizeIncre_w_TL_UntilConvergence`, `BootstrapIncumbentFromDMFast`, `SeedBaselineAndArmCache`, `BackstopVerdict`
- [ ] `OptimizeSP_TL_BF.{h,cpp}` — `AdoptRmFastFallbackIfUnschedulable` gate inside `Optimize()` (BF also gated)

## Write `sketch_fallback.md` (items 1–2 from goal.md)
> For each item state BOTH the **algorithm** (what) AND the **motivation** (why it exists / what tension it resolves), tied to the concrete symbol/file.

- [ ] 1. Important-task safety guarantee: `is_important` = top-50% by `sp_weight`; `ComputeSafeFallback` → worst-case DAG → RM/DM-fast group-locked seed; during-walk gate in `UpdateRecords`; `AdoptFallbackIfUnschedulable`/`AdoptRmFastFallbackIfUnschedulable` (throws on double-fail); BOTH online INCR + offline BF gated. *Why: critical tasks must never miss; framework self-guarantees a schedulable fallback.*
- [ ] 2. `OptimizeIncre_w_TL_UntilConvergence` (offline, `ComputeSafeFallback` only): loops `OptimizeIncre_w_TL` until no strict `opt_sp_` improvement (`opt_sp_ <= sp_before || ApproxEqualSP(...)`, no cap). *Why: one pass may miss cascaded gains; loop until no strict gain — offline only to bound online cost.*
- [ ] Each item tied to the concrete symbol/file it came from
- [ ] Convergence loop clearly marked OFFLINE ONLY (not the online walk)

## Verification
- [ ] Each of 1–2 states both algorithm AND motivation
- [ ] Guarantee statement precise enough for PW.3 to ground the new "important-task safety guarantee" subsection sentence-by-sentence
- [ ] A reader could state the guarantee using only this sketch + existing `.tex`, without re-reading C++

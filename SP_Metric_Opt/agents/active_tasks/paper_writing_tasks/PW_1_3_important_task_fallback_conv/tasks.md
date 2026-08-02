# PW.1.3 — Tasks (working checklist)

> See `goal.md` for scope. Third slice of the code read: important-task safe
> fallback + offline convergence loop. Mostly a reading task.

## Read the fallback / guarantee sources
- [x] `sources/TaskModel/RegularTasks.h:111` — `is_important` (top-50% by `sp_weight`) — confirmed via `taskset_generator.py:566-581` + `generation_config_parser.py:373` (ratio=0.5)
- [x] `sources/TaskModel/WorstCaseDAG.cpp` — `BuildDAGForObtainSafeFallBAckAcrossIntervals` (non-perf=max WCET; perf=min TL option)
- [x] `OptimizeFallback.{h,cpp}` — `RateMonotonicFastGroupLocked`, `AdoptRmFastFallbackIfUnschedulable` (throws on double-fail)
- [x] `OptimizeSP_TL_Incre.{h,cpp}` — `ComputeSafeFallback`, `AdoptFallbackIfUnschedulable`, `enable_fallback_use_`, `OptimizeIncre_w_TL` (virtual), `OptimizeIncre_w_TL_UntilConvergence`, `BootstrapIncumbentFromDMFast`, `SeedBaselineAndArmCache`, `BackstopVerdict`
- [x] `OptimizeSP_TL_BF.{h,cpp}` — `AdoptRmFastFallbackIfUnschedulable` gate inside `Optimize()` (BF also gated, `:73`)
- [x] orchestrator call site — `SimulationOrchestrator.cpp:321-323` (offline pre-compute before sim loop)

## Write `sketch_fallback.md` (items 1–2 from goal.md)
> For each item state BOTH the **algorithm** (what) AND the **motivation** (why it exists / what tension it resolves), tied to the concrete symbol/file.

- [x] 1. Important-task safety guarantee: `is_important` = top-50% by `sp_weight`; `ComputeSafeFallback` → worst-case DAG → RM/DM-fast group-locked seed; during-walk gate in `UpdateRecords`; `AdoptFallbackIfUnschedulable`/`AdoptRmFastFallbackIfUnschedulable` (throws on double-fail); BOTH online INCR + offline BF gated. *Why: critical tasks must never miss; framework self-guarantees a schedulable fallback.*
- [x] 2. `OptimizeIncre_w_TL_UntilConvergence` (offline, `ComputeSafeFallback` only): loops `OptimizeIncre_w_TL` until no strict `opt_sp_` improvement (`opt_sp_ <= sp_before || ApproxEqualSP(...)`, no cap). *Why: one pass may miss cascaded gains; loop until no strict gain — offline only to bound online cost.*
- [x] Each item tied to the concrete symbol/file it came from
- [x] Convergence loop clearly marked OFFLINE ONLY (not the online walk)

## Verification
- [x] Each of 1–2 states both algorithm AND motivation
- [x] Guarantee statement precise enough for PW.3 to ground the new "important-task safety guarantee" subsection sentence-by-sentence
- [x] A reader could state the guarantee using only this sketch + existing `.tex`, without re-reading C++
- [x] Draft drift flagged: §13.3 guarantee is conditional in draft, SELF-guaranteed in code; §11 `\sen` note is an answered question

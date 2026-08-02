# PW.1.3 — Important-Task Safe Fallback & Offline Convergence Loop

**Priority:** P0 (grounding — third slice of the code read)
**Status:** not started
**Depends on:** —

## Goal

Produce `sketch_fallback.md` in this folder: the third slice of the code-side
truth. Covers the **important-task safety-performance guarantee** (the new claim
the paper must state and ground) and the **offline convergence loop**, each with
both the **algorithm** (what) and the **motivation** (why it exists), tied to
the concrete symbol/file.

A small, focused reading task — the safety-fallback machinery, separated from
the optimization core (PW.1.2) and the foundations (PW.1.1).

## Entry point

Read these sources (verified present) and write `sketch_fallback.md`:

- `sources/TaskModel/RegularTasks.h:111` — `bool is_important = false;`
  (top-50% by `sp_weight`, persisted to YAML as `important`).
- `sources/TaskModel/WorstCaseDAG.cpp` — `BuildWorstCaseDagAcrossIntervals`
  (worst-case DAG across intervals for the safe-fallback seed).
- `sources/Optimization/OptimizeFallback.{h,cpp}` —
  `RateMonotonicFastGroupLocked(dag_tasks)`,
  `AdoptRmFastFallbackIfUnschedulable(...)` (throws on double-fail).
- `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}` —
  `ComputeSafeFallback(worst_case_dag)`,
  `AdoptFallbackIfUnschedulable`, `enable_fallback_use_ = true` (≈line 469),
  virtual `OptimizeIncre_w_TL(beam_search_width)`,
  `OptimizeIncre_w_TL_UntilConvergence`,
  `BootstrapIncumbentFromDMFast`, `SeedBaselineAndArmCache`,
  `BackstopVerdict { kNone, kKeptWalk, kAdoptedFallback }`.
- `sources/Optimization/OptimizeSP_TL_BF.{h,cpp}` —
  `AdoptRmFastFallbackIfUnschedulable` gate inside `Optimize()` (BF also gated).

## What this slice must capture

For each: state the **algorithm** (what) **and the motivation** (why), each tied
to the concrete symbol/file.

1. **Important-task safety guarantee (the new claim).** `is_important` =
   top-50% by `sp_weight`. The framework guarantees a safe fallback for
   important tasks: `ComputeSafeFallback` builds a worst-case DAG across
   intervals (`WorstCaseDAG.cpp`) → seeds an RM/DM-fast group-locked plan
   (`RateMonotonicFastGroupLocked` / `DeadlineMonotonicPriorityVec`); a
   during-walk important-task gate inside `UpdateRecords` rejects any
   intermediate priority/TL move that would make an important task
   unschedulable; `AdoptFallbackIfUnschedulable` /
   `AdoptRmFastFallbackIfUnschedulable` adopt the fallback when the walk's
   result is unschedulable (throws on double-fail). **Both the online INCR walk
   and the offline BF are gated.**
   *Why:* critical tasks (e.g. MPC) must never miss a deadline; rather than hope
   the optimizer happens to keep them schedulable, the framework self-guarantees
   a schedulable fallback discovered by its own machinery.
2. **Convergence loop.** `OptimizeIncre_w_TL_UntilConvergence` loops
   `OptimizeIncre_w_TL` until a pass fails to strictly improve `opt_sp_`
   (`opt_sp_ <= sp_before || ApproxEqualSP(...)`, no cap). Used offline by
   `ComputeSafeFallback`, not the online walk.
   *Why:* a single incremental pass can leave improvement on the table when an
   ET jump cascades through neighbors; looping until no strict gain harvests it
   — but offline only, so the online per-interval latency budget is not blown.

## Done when

- `sketch_fallback.md` exists and captures items 1–2 above, each with both the
  **algorithm** and the **motivation**, tied to the concrete symbol/file.
- The guarantee statement is precise enough that PW.3 can ground every sentence
  of the new "important-task safety guarantee" subsection against this sketch.
- It is clear that the convergence loop is **offline only** (not the online
  walk).

## Out of scope

- Foundations (SP/pRTA/cache/environment) — PW.1.1.
- PA / TL optimization core — PW.1.2.
- The code-vs-draft revision plan — PW.1.4.
- Editing any `.tex` (PW.3/PW.4).

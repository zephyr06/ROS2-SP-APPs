# PW.1.2 — Priority Assignment & Task-Config (TL) Optimization

**Priority:** P0 (grounding — second slice of the code read)
**Status:** not started
**Depends on:** —

## Goal

Produce `sketch_optimization.md` in this folder: the second slice of the
code-side truth. Covers the **priority-assignment** methods and the
**time-limit (task-config) optimization**, each with both the **algorithm**
(what) and the **motivation** (why it exists), tied to the concrete symbol/file.

A small, focused reading task — the optimization core, separated from the
foundations (PW.1.1) and the safety-fallback machinery (PW.1.3).

## Entry point

Read these sources (verified present) and write `sketch_optimization.md`:

- `sources/Optimization/OptimizeSP_BF.{h,cpp}` — brute-force priority assignment.
- `sources/Optimization/OptimizeSP_Incre.{h,cpp}` — incremental PA;
  `FindTaskWithDifferentEt` (`DiffObj{i, et_increased}`),
  `FindEnvTaskWithDifferentEt` (filters TL-flexible tasks), `RemoveOneTask`,
  `GetProrityIndex`.
- `sources/Optimization/OptimizeSP_TL_BF.{h,cpp}` — `EnumeratePA_with_TimeLimits`
  (BF with TL).
- `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}` —
  `OptimizePA_Incre_with_TimeLimits`; virtual
  `OptimizeIncre_w_TL(beam_search_width)`, `RunIntervalDescent`,
  `ReconstructTimeLimitVecFromResOpt`, `IntervalDescentMode { Incremental, Reopt }`.
- `sources/Optimization/PriorityBuilders.{h,cpp}` — `SortKey {kDeadline, kPeriod}`,
  `GroupLock {kNone, kImportantFirst}`,
  `TimeLimitPolicy {kNone, kSmallestGrid, kLargestGrid}`,
  `PriorityBuilderConfig`, `BuildPriorityPlan(dag_tasks, config)`.
- Config: `DEADLINE_MODE=implicit` (config-level); DM (deadline-monotonic)
  priority building (commit `0b9dae4a`).

## What this slice must capture

For each: state the **algorithm** (what) **and the motivation** (why), each tied
to the concrete symbol/file.

1. **Priority assignment — three modes.**
   (a) Brute force = enumerate `n!` orderings.
   (b) Modified Audsley + beam search (width `beam_search_width`/`m`) — a
       **heuristic**, NOT "optimal"/exhaustive.
   (c) Incremental = find the task whose ET changed (`FindTaskWithDifferentEt`),
       adjust its priority by ≤1 level (four scenarios). Note the DM seed
       (`DeadlineMonotonicPriorityVec`) + important-first group lock
       (`BuildPriorityPlan`, `GroupLock::kImportantFirst`).
   *Why:* SP is non-monotone in priority ordering, so search is required; BF
   gives optimality at small N / offline, modified-Audsley+beam makes it
   tractable, and incremental keeps the *online* per-interval cost bounded by
   exploiting that only one task's ET typically changed.
2. **Time-limit (config) optimization.** Sequential coordinate descent over the
   TL grid, local search radius δ (3 candidates), prioritized task ordering
   (desc weight, asc deadline, asc ID), resource-aware tie-break. Confirm the
   O(M^N) → O(M·N) complexity claim and the challenger-from-champion →
   1-task-ET-diff property (the `\agent`-noted theorem question — decide proof
   vs. stated property). Confirm `eq: incremental_configuration`
   (`‖λ − λ^(k)‖ ≤ δ`) and δ → 3 candidates matches code/config.
   *Why:* TL trades safety against performance, and the full TL grid is M^N;
   coordinate descent sidesteps the blowup.
3. **The env-task reframing (the `\sen` note).** TL-optimizable tasks treated as
   a special kind of env-dependent task → trigger incremental optimization after
   assuming their ET changed (this is the `FindEnvTaskWithDifferentEt` path).
   *Why:* unifies TL-flex handling with ET-change handling under one incremental
   mechanism — the section-8 conceptual reframing.

## Done when

- `sketch_optimization.md` exists and captures items 1–3 above, each with both
  the **algorithm** and the **motivation**, tied to the concrete symbol/file.
- Modified-Audsley + beam search is labeled a **heuristic**, not "optimal".
- The env-task reframing and the O(M^N)→O(M·N) / 1-task-ET-diff property are
  stated; the `\agent` theorem question has a recommendation (proof or stated
  property).

## Out of scope

- Foundations (SP/pRTA/cache/environment) — PW.1.1.
- Important-task fallback / convergence loop — PW.1.3.
- The code-vs-draft revision plan — PW.1.4.
- Editing any `.tex` (PW.3/PW.4).

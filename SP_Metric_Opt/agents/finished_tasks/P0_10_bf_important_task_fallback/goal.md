# P0.10 — BF Important-Task Schedulability Check + RM-Fast Fallback

**Priority:** P0
**Status:** SCAFFOLDED (task folder created; no code yet)
**Depends on:** P0.8 (`ImportantTasksMeetThresholds` gate + `is_important`), P0.9
(the important-first group-lock pattern + DM grouping this mirrors with RM)
**Related:** P0.7 (the INCR-side fall-back; this is the BF-side analogue)

## Goal

Run the important-task schedulability check on **BF's result**. If BF's
{priority_vec, time_limit} is **unschedulable for the important tasks**, fall
back to a static, cheap-by-construction priority assignment:

> **RM-Fast of important tasks + RM-Fast of non-important tasks**

i.e. Rate-Monotonic (period-ordered) priority, **important-first group-locked**
(same lock shape as P0.9's `DeadlineMonotonicPriorityVec`, but period key instead
of deadline), "fast" TL = the smallest grid option (`timePerformancePairs[0]`,
matching `DM_FAST`'s `:393-403` precedent; `-1.0` for no-grid tasks).

The user's verbatim direction:
> "we'll also run important tasks' schedulability check for BF, and let it fall
> back to RM-Fast of important tasks + RM-Fast of non-important tasks if
> unschedulable for important tasks"

## Why (the motivation)

BF is a baseline (no fall-back machinery of its own — unlike INCR, which has
P0.7's trigger (a)+(b)). Today BF returns whatever `EnumeratePA_with_TimeLimits`
finds; nothing guarantees the important tasks meet their deadlines. This task
gives BF the same safety floor P0.7 gives INCR — but with a **different fallback
plan** (RM-Fast group-locked, not P0.6's offline-walk `safe_fallback_`), because
BF has no incremental walk to seed from. RM-Fast is chosen because it is
cheap-by-construction (a sort, no descent) and RM is the natural static priority
for periodic tasks.

## What "RM-Fast of important + RM-Fast of non-important" means (precise)

Mirror P0.9's `DeadlineMonotonicPriorityVec` (`OptimizeSP_TL_Incre.cpp:878-894`)
exactly, swapping the **period** key for the **deadline** key:

- Important tasks occupy the top `n_important` priority slots, **RM-ordered
  within the group** (shorter period = higher priority).
- Non-important tasks fill the lower slots, **RM-ordered within their group**.
- Every non-important task below every important task (the group lock —
  non-important tasks never interfere with the important group, making its RTA
  self-contained, exactly as P0.9's lock does for DM).
- Ties broken by avg ET ascending (same tiebreak as P0.9).
- TL = "fast": `timePerformancePairs[0].time_limit` (smallest grid option) if the
  task has a perf grid, else `-1.0` (no TL). Identical to `DM_FAST` (`:393-403`).

**Why RM not DM here** (contrast with P0.9): P0.9's seed uses DM because the
taskset has constrained deadlines (`deadline = period * U(0.5,1.0)`, so DM is the
correct static priority for the *schedulability-optimal* seed). This task's
fallback is NOT an optimization — it is a cheap safety floor BF falls to when its
own result is unsafe. RM (period-ordered) is the conventional static priority for
periodic tasks and is the user's explicit choice for the BF fallback. (If RM-Fast
turns out unschedulable too — see Open D2.)

## Where it hooks in (code grounding)

- **BF dispatch:** `SimulationOrchestrator.cpp:353-354` —
  `res = EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters);` inside
  `DeterminePrioritiesAndBudgets`. The gate + fallback wrap THIS call: compute
  BF's `res`, run the gate on it, on FAIL replace `res` with the RM-Fast plan.
- **The gate:** `ImportantTasksMeetThresholds(dag, sp_params, pa, tl)` — the
  self-contained overload (`SP_Metric.h:174`, derives RTAs fresh, one eval). BF
  has no live RTA cache → self-contained overload is the right call (mirrors
  `AdoptFallbackIfUnschedulable`'s post-walk backstop in P0.7).
- **The RM-Fast group-lock builder:** NEW. P0.9's `DeadlineMonotonicPriorityVec`
  is the template. Question: where does the new builder live? (See Open D1.)
- **The "fast" TL:** mirror `DM_FAST`'s `:393-403` inline loop (min-grid / -1.0).

## Scope (what this task IS / IS NOT)

**IS:**
- A gate on BF's result via `ImportantTasksMeetThresholds` (reuse, no new gate
  logic — D2 of P0.7: same detection, no drift).
- A new RM-Fast group-locked priority+TL builder (the fallback plan).
- Wiring at the BF dispatch site (`:353-354`): on gate-fail, swap `res` to the
  RM-Fast plan.
- TDD: the RM-Fast builder (important-first lock, RM within group, fast TL), and
  the BF gate-fail→fallback swap (BF unsafe → RM-Fast adopted; BF safe → kept).

**IS NOT:**
- A change to `EnumeratePA_with_TimeLimits` itself — BF's search is untouched;
  the gate runs on its OUTPUT.
- A new "RM_FAST" dispatchable **mode** (not adding a top-level `scheduler_mode_`
  branch) — UNLESS the user wants RM-Fast as a standalone baseline too (see Open
  D3). The fallback is an internal swap, not a user-selectable mode.
- A TL optimization — "fast" = smallest grid, no walk. (Contrast P0.6 which
  walks TL for best-SP; this is the cheap floor.)
- Touching INCR/P0.7 — that arm already has its own fall-back.

## Done when

- [ ] New RM-Fast group-locked {pa, tl} builder, TDD red→green (mirrors P0.9's
      builder's test shape).
- [ ] BF gate: `ImportantTasksMeetThresholds` on BF's `res`; on FAIL → swap to
      RM-Fast plan; on PASS → keep BF. TDD red→green.
- [ ] Wired at `SimulationOrchestrator.cpp:353-354`.
- [ ] `cmake --build build_test --target check.SP_OPT -j5` green (17/17 ctest).
- [ ] `dev_log.md` + `tasks.md` + memory updated; `git add` staged, user reviews.

## Resolved decisions (settled 2026-08-01)

- **D1 — Fall-back code gets its OWN translation unit + migrate existing.**
  Create a NEW dedicated `.h`/`.cpp` pair for fall-back-related code. The new
  RM-Fast builder lives there. ALSO Migrate the existing fall-back code currently
  in `OptimizeSP_TL_Incre.{h,cpp}` INTO this new file (consolidate). Scope of
  "fall-back related code" to migrate: `DetectETJump`, `SkipOptOnETJump`,
  `AdoptSafeFallbackAsIncumbent`, `ComputeSafeFallback`,
  `AdoptFallbackIfUnschedulable`, `safe_fallback_` + accessors
  (`HasSafeFallback`/`SetSafeFallbackForTest`/`GetSafeFallbackComputeTime`/
  `GetIntervalFallbackLog`/`interval_fallback_log_`), `IntervalFallbackOutcome` +
  `FormatIntervalFallbackLogCsv`. NOT migrated: `WorstImportantTaskMissInfo`
  (lives in `SP_Metric.{h,cpp}` — the gate's diagnostic companion, stays with the
  gate); `BuildWorstCaseDagAcrossIntervals`/`TaskStructureMatches` (already in
  `sources/TaskModel/WorstCaseDAG.cpp`). **Mechanics open:** most migrated
  symbols are MEMBERS of `OptimizePA_Incre_with_TimeLimits` accessing
  `dag_tasks_`/`sp_parameters_`/`res_opt_`/`opt_pa_`/`opt_sp_`/`rta_cache_`/
  `safe_fallback_` — can't be lifted verbatim. Resolve at migration time: either
  member defs in the new `.cpp` (decl stays in `OptimizeSP_TL_Incre.h`) OR a
  `FallbackManager` helper holding the state + a ref to the optimizer.
  Behavior-identical; TDD-green must hold.
- **D2 — RM-Fast also-unschedulable → THROW.** Mirror P0.7's
  `AdoptFallbackIfUnschedulable` rescue-also-fails throw. A double-fail = the
  P0.8 generation certificate is violated (the taskset was certified RM/DM-
  schedulable for important tasks at generation). Loud, not silent.
- **D3 — No standalone `RM_FAST` mode.** Internal fallback swap inside the BF
  branch only (user's "fall back to" phrasing). No new `scheduler_mode_` branch,
  no `tests/RunOrchestrator.cpp` usage change.

## Sequencing concern (flagged, not yet decided)

The D1 migration touches `OptimizeSP_TL_Incre.{h,cpp}` — the EXACT files staged
uncommitted for P0.7 Groups B/C/D. Migrating before those commits land would
restructure a file the user is mid-reviewing, tangling two concerns in one diff.
Recommend: **finish committing P0.7 Groups B/C/D FIRST**, then do the migration
as its own behavior-identical refactor commit, then the P0.10 RM-Fast builder +
BF gate as fresh commits. Awaiting user's call on ordering.

## Reference docs

- `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp:339-426` —
  mode dispatch; BF at `:353-354`; `DM_FAST` fast-TL precedent at `:384-404`.
- `sources/Optimization/OptimizeSP_TL_Incre.cpp:878-894` — P0.9
  `DeadlineMonotonicPriorityVec` (the group-lock template; swap deadline→period).
- `sources/Safety_Performance_Metric/SP_Metric.h:162-177` —
  `ImportantTasksMeetThresholds` (gate; self-contained overload at `:174`).
- `sources/Optimization/OptimizeSP_TL_BF.h:40-41` — `EnumeratePA_with_TimeLimits`
  (BF's free-fn entry; returns `ResourceOptResult`).
- Memory `p07-fallback-mechanism` — the INCR-side analogue (trigger (b-ii)
  backstop shape; throw-on-rescue-also-fails).
- Memory `p09-dm-and-important-first-priority` — the group-lock pattern.
- Memory `p08-important-task-schedulability` — the gate + generation certificate.

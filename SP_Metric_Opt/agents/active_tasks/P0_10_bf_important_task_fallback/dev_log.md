# P0.10 — Dev Log

## 2026-08-01 — Task scaffolded (no code yet)

Grounded the design against the codebase:

- **BF dispatch** = `SimulationOrchestrator.cpp:353-354`:
  `res = EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters)` — free fn,
  returns `ResourceOptResult` (`priority_vec` + `id2time_limit`). The gate +
  fallback wrap this call.
- **No `RM_FAST` mode exists today.** "Fast" variants present = `DM_FAST`/
  `DM_SLOW` (deadline-sorted + min/max grid TL, inline at `:384-425`). RM
  (period-sorted) priority is new — needs a builder. The "fast" TL pattern
  (`timePerformancePairs[0]` / `-1.0`) is directly reusable from `DM_FAST`.
- **Gate** = `ImportantTasksMeetThresholds` self-contained overload
  (`SP_Metric.h:174`) — derives RTAs fresh, one eval. BF has no live RTA cache
  → this is the right overload (mirrors P0.7's `AdoptFallbackIfUnschedulable`
  post-walk backstop).
- **Group-lock template** = P0.9's `DeadlineMonotonicPriorityVec`
  (`OptimizeSP_TL_Incre.cpp:878-894`): important-first lock, DM within group,
  avg-ET tiebreak. RM-Fast = same shape, period key instead of deadline.

Created `goal.md` + `tasks.md`. Three open decisions flagged for the user
(D1 builder location; D2 throw-on-rescue-also-fails; D3 standalone RM_FAST mode).
No code changes yet — waiting on D1–D3 before TDD.

## 2026-08-01 — Decisions settled

User settled all three:
- **D1 = new dedicated `.h`/`.cpp` for fall-back code + MIGRATE existing.** Not
  just a free fn — a new translation unit, AND move the existing fall-back code
  out of `OptimizeSP_TL_Incre.{h,cpp}` into it. Symbols to migrate:
  `DetectETJump`, `SkipOptOnETJump`, `AdoptSafeFallbackAsIncumbent`,
  `ComputeSafeFallback`, `AdoptFallbackIfUnschedulable`, `safe_fallback_` +
  accessors, `IntervalFallbackOutcome` + `FormatIntervalFallbackLogCsv`. Stay:
  `WorstImportantTaskMissInfo` (with the gate in `SP_Metric`);
  `BuildWorstCaseDagAcrossIntervals`/`TaskStructureMatches` (already in
  `WorstCaseDAG.cpp`). Migration mechanics open (most are members accessing
  private optimizer state — member-def-in-new-cpp vs `FallbackManager` helper;
  behavior-identical either way).
- **D2 = throw on double-fail** (mirror P0.7).
- **D3 = no standalone RM_FAST mode** (internal BF-branch swap only).

Flagged the sequencing concern: D1's migration touches `OptimizeSP_TL_Incre.{h,cpp}`
— the exact files staged uncommitted for P0.7 Groups B/C/D. Recommended committing
those FIRST so the migration is its own clean behavior-identical refactor commit,
not a tangle. Awaiting user's call on ordering.

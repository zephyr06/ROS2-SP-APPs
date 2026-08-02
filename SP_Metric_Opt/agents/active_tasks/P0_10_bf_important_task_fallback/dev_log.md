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

## 2026-08-01 — §1 RM-Fast builder LANDED (TDD red→green)

Sequencing concern resolved: P0.7 code is fully committed
(`c87ae0d4`…`a8148dc7`…`f371c543`); `OptimizeSP_TL_Incre.{h,cpp}` +
`testIncreOpt_w_TL.cpp` are clean at HEAD. The D1 migration no longer tangles
with an in-flight review. Started §1 (the builder) as the first small,
self-contained increment; the existing-code migration is its OWN later
behavior-identical refactor commit (mechanics still open).

**New TU** `sources/Optimization/OptimizeFallback.{h,cpp}` — the dedicated
fallback translation unit (D1). For §1 it holds ONLY the new RM-Fast builder;
the migrated existing fall-back code lands there in a later commit.

**Builder** `ResourceOptResult RateMonotonicFastGroupLocked(const DAG_Model&)`:
free fn (BF has no optimizer instance — the builder must be callable from the BF
branch with just `dag_tasks`). Mirrors P0.9's `DeadlineMonotonicPriorityVec`
group-lock shape (`OptimizeSP_TL_Incre.cpp:881-897`) + `DM_FAST`'s TL loop
(`SimulationOrchestrator.cpp:408-418`), period-keyed instead of deadline-keyed:
important-first lock → within group sort by `period` asc → avg-ET tiebreak;
TL = `timePerformancePairs[0].time_limit` (smallest grid) else `-1.0`.

**TDD:** 3 tests in `testIncreOpt_w_TL.cpp` (`RateMonotonicFastGroupLocked.*`),
constructed so plain period-sort would scramble the lock (an important task has
the longest period) — discriminates the group-lock from plain RM:
`RanksImportantGroupFirstThenRmOrdersEach` (lock + within-group RM),
`BreaksPeriodTiesByExecutionTimeAscending` (tiebreak),
`TimeLimitIsSmallestGridOrMinusOne` (fast TL). RED vs stub (empty
`priority_vec`, `id2time_limit`=0) → GREEN after impl.

**Green:** 3/3 new pass; `testIncreOpt_w_TL` 119/119 (was 116); 16/17 ctest —
sole failure `OrchestratorTest.CFS_RunOrchestrator_Binary` is PRE-EXISTING
(shells out to a RELEASE binary absent from this DEBUG tree; identical on clean
HEAD via `git stash`; unrelated to P0.10's files).

**Staged:** `OptimizeFallback.h`, `OptimizeFallback.cpp`, `testIncreOpt_w_TL.cpp`
(+ records). NOT committed — user reviews. NEXT = §2 BF gate + fallback swap.

## 2026-08-01 — §1b Extract shared BuildPriorityPlan + config (TDD red→green)

User asked whether `RateMonotonicFastGroupLocked` could reuse incremental-opt
code, and suggested moving related fall-back code into `OptimizeFallback` + a
new header for "common functions like RM-Fast's config". Investigated (see
§1b reuse table below). Finding: no directly-callable reuse — the closest
analogue `DeadlineMonotonicPriorityVec` (`OptimizeSP_TL_Incre.cpp:881`) is a
MEMBER of `OptimizePA_Incre_with_TimeLimits` (BF's dispatch has no instance →
uncallable) and returns `PriorityVec` (no TL). But the SHAPE is shared across
5 sites, so the clean reuse = a parameterized config + builder.

**Reuse table** (sort indices → fill {priority_vec, id2time_limit}):

| site | key | group lock | TL policy | returns |
|---|---|---|---|---|
| `DeadlineMonotonicPriorityVec` (`OptimizeSP_TL_Incre.cpp:881`) | deadline | important-first | none | `PriorityVec` (member) |
| `RateMonotonicFastGroupLocked` (§1) | period | important-first | smallest grid / -1 | `ResourceOptResult` (free) |
| `DM` inline (`SimulationOrchestrator.cpp:386`) | deadline | none | -1 | `ResourceOptResult` |
| `DM_FAST` inline (`:399`) | deadline | none | smallest grid / -1 | `ResourceOptResult` |
| `DM_SLOW` inline (`:420`) | deadline | none | largest grid / -1 | `ResourceOptResult` |

**Sequencing decision (user, option 1):** config helper → §2 BF gate → D1
migration (its own later commit). So §1b = ONLY the extraction + delegate
rewrite; the 3 inline DM modes are NOT unified here (later); the D1 fall-back-
code migration is NOT done here (later, member-state-coupled, behavior-
identical).

**New TU** `sources/Optimization/PriorityBuilders.{h,cpp}`:
`PriorityBuilderConfig` (`SortKey` deadline/period, `GroupLock`
none/important-first, `TimeLimitPolicy` none/smallest-grid/largest-grid) +
`BuildPriorityPlan(dag, config) -> ResourceOptResult`. Sort = group lock →
sort key → avg-ET tiebreak; TL = grid endpoint or -1.0 (PickTimeLimit helper).

**Rewrote** `RateMonotonicFastGroupLocked` (in `OptimizeFallback.cpp`) as a
3-line delegate: `{kPeriod, kImportantFirst, kSmallestGrid}`. Signature
unchanged → 3 existing tests are the regression net.

**TDD:** 2 new `BuildPriorityPlan` tests cover configs the RM-Fast tests
don't — `DeadlineKeyNoLockOrdersByDeadline` (DM shape: important flag IGNORED,
deadline-ties break by avg ET; kNone TL = -1.0) and
`LargestGridPolicyPicksBackPair` (DM_SLOW shape: `pairs.back()`). RED vs stub
→ GREEN.

**Green:** 5/5 builder tests; `testIncreOpt_w_TL` 121/121 (was 119, +2). ctest:
`testScheduleSimulate` + `CFS_RunOrchestrator_Binary` both fail — VERIFIED
pre-existing (clean-HEAD `git stash -u` + reconfigure: `testScheduleSimulate`
= 39 pass / 1 fail = the same CFS test; the ctest "Failed" label is that one
CFS test's non-zero exit bubbling up). Unrelated to P0.10.

**Staged:** `PriorityBuilders.h`, `PriorityBuilders.cpp`, `OptimizeFallback.cpp`,
`testIncreOpt_w_TL.cpp` (+ records). NOT committed. NEXT = §2 BF gate + swap.

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

## 2026-08-01 — §2 BF gate + fallback swap LANDED (TDD red→green)

User's §1b review committed as `2f6c7c4c`; tree clean. Started §2 (the BF gate +
swap), keeping the change modular + testable.

**Design — a free fn, not an inline branch.** The §2 logic (gate BF's res → on
FAIL swap to RM-Fast → on double-fail throw) is wired into
`FixedTaskPrioritySchedulingOrchestrator::DeterminePrioritiesAndBudgets`, whose
full construction (input folder, etc.) is a heavy harness. To keep the test
harness light (just `DAG_Model` + `SP_Parameters`, like §1/§1b) the logic is a
FREE fn `AdoptRmFastFallbackIfUnschedulable(dag, sp, bf_result)` in
`OptimizeFallback.{h,cpp}` — mirrors `RateMonotonicFastGroupLocked` being free
(BF has no optimizer instance). The BF branch becomes a 1-line delegate:
```cpp
} else if (scheduler_mode_ == "BF") {
    res = EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters);
    res = AdoptRmFastFallbackIfUnschedulable(dag_tasks, sp_parameters, res);
}
```

**Gate contract recap:** `ImportantTasksMeetThresholds(dag, sp, pa, tl)` (self-
contained overload, `SP_Metric.h:174`) — `pa` is task IDs (== indices under the
id-ordered taskset contract `UpdateTaskSetPriorities` enforces), `tl` is index-
ordered (pairs with `dag_tasks.tasks[i]`). So the fn reconstructs the index-
ordered `tl` from `bf_result.id2time_limit` (keyed by id) via a static helper
`TimeLimitVecFromResult` (mirrors `ReconstructTimeLimitVecFromResOpt`). On FAIL
it builds the RM-Fast plan, reconstructs its `tl`, re-gates; on second FAIL
`CoutWarning` + `throw std::runtime_error` (D2 — mirror of
`AdoptFallbackIfUnschedulable`'s certificate-violation throw; never silently
ship an infeasible result).

**TDD:** 3 tests `AdoptRmFastFallbackIfUnschedulable.*`:
- `KeepsBfResultWhenImportantTasksFeasible` — loose deadline → gate PASSES → BF
  `{pa,tl}` kept verbatim.
- `AdoptsRmFastWhenBfFailsGate` — the discriminative case: important T0 (ET 10,
  deadline 40) MEETS at the top slot (RTA 10) but MISSES below non-important T1
  (ET 40, RTA 50 > 40). BF order `[1,0]` fails → swapped to RM-Fast `[0,1]`.
  (First cut used an impossibly-tight deadline 1.0 — that double-fails under
  ANY PA since RTA >= ET 10 > 1; corrected to a deadline tight enough to flip
  with priority order but meettable at the top.)
- `ThrowsWhenRmFastAlsoFailsGate` — BOTH important tasks deadline 1.0 → no PA
  can meet → `EXPECT_THROW` `std::runtime_error`.

RED vs stub (returns `bf_result`): tests 2+3 fail. GREEN after impl.

**Green:** 3/3 new; `testIncreOpt_w_TL` 124/124 (was 121, +3); 16/17 ctest —
sole failure `testScheduleSimulate`/`CFS_RunOrchestrator_Binary` pre-existing
(shells out to an absent RELEASE binary; verified 39 pass / 1 fail, the same
CFS test). Unrelated to §2.

**Staged:** `OptimizeFallback.h`, `OptimizeFallback.cpp`,
`SimulationOrchestrator.cpp`, `testIncreOpt_w_TL.cpp` (+ records). NOT
committed — user reviews. NEXT = §3 records + handoff (then D1 migration as its
own later commit, per the option-1 sequencing).

## 2026-08-01 — §2 placement moved INSIDE EnumeratePA_with_TimeLimits (revised)

User redirect: the gate+fallback should live INSIDE the BF computation (ideally
`OptimizePA_with_TimeLimitsStatus`), not as a post-hoc orchestrator wrapper.
Rationale: every `EnumeratePA_with_TimeLimits` caller is then gated
automatically — the safety floor is a property of BF itself, not a call-site
convention the orchestrator has to remember.

**Change:** moved the gate call into `EnumeratePA_with_TimeLimits`
(`OptimizeSP_TL_BF.cpp`): after `optimizer.Optimize()`,
`return AdoptRmFastFallbackIfUnschedulable(dag_tasks, sp_parameters,
optimizer.res_opt);` instead of `return optimizer.res_opt;`. Reverted the
orchestrator's BF branch to its original 1-line form (`res =
EnumeratePA_with_TimeLimits(...)`) + dropped the now-unneeded
`OptimizeFallback.h` include there. The free fn
`AdoptRmFastFallbackIfUnschedulable` stays — it's the unit-testable seam (the
3 existing tests still call it directly) and the BF entry point now calls it.

**Blast-radius check (decisive):** moving the gate inside BF means EVERY caller
is gated, including legacy tests (`testOptimizePA`, `testBF_w_TL`,
`testBFRTimeout`). Safe because `is_important` defaults `false`
(`RegularTasks.h:111`) and the gate VACUOUSLY PASSES when no task is important
(`SP_Metric.cpp:238` `return true; // every important task (vacuously, if none)`).
So legacy BF tests are inert under the gate; only task sets that set
`is_important` (the P0.10 tests) can trigger swap/throw. Confirmed: all three
legacy suites stay green (10 + 5 + 2).

**`priority_vec` populated?** Verified `OptimizePA_BruteForce` calls
`res.UpdatePriorityVec(pa_vec)` (`OptimizeSP_BF.h:25`) → BF's `res_opt.priority_vec`
is valid for the gate's `pa`.

**New end-to-end test** `EnumeratePA_with_TimeLimits.SwapsToRmFastWhenImportant-
TaskMissesGate` — calls the real BF entry point with the §2 interference task
set (important T0 ET 10 / deadline 40, non-important T1 ET 40); asserts the
returned plan has T0 in the top slot (BF-safe order kept OR RM-Fast swap
applied — either way the important task is on top). Proves the gate fires
through the BF computation, not just the free-fn seam.

**Green:** 4/4 P0.10 gate tests (3 free-fn + 1 end-to-end); `testIncreOpt_w_TL`
125/125 (was 124, +1); legacy BF suites 10/5/2 all green; 16/17 ctest (sole
failure `testScheduleSimulate`/`CFS_RunOrchestrator_Binary` pre-existing —
shells out to an absent RELEASE binary).

**Staged (revised):** `OptimizeFallback.h`, `OptimizeFallback.cpp`,
`OptimizeSP_TL_BF.cpp`, `testIncreOpt_w_TL.cpp` (+ records). The orchestrator
file is NO LONGER touched by §2 (reverted to original). NOT committed — user
reviews.

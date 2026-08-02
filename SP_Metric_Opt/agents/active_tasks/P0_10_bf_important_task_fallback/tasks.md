# P0.10 — Tasks (working checklist)

> See `goal.md` for the design (BF gate on important-task schedulability; on
> fail → RM-Fast important + RM-Fast non-important, group-locked like P0.9 but
> period-keyed + fast min-grid TL). Depends on P0.8 (gate + `is_important`),
> P0.9 (group-lock template).

## 0. Design decisions (SETTLED 2026-08-01)
- [x] **D1 — Own translation unit + migrate existing fall-back code.** New
      dedicated `.h`/`.cpp` for fall-back code; new RM-Fast builder lives there.
      ALSO migrate existing fall-back code out of `OptimizeSP_TL_Incre.{h,cpp}`
      into it (`DetectETJump`, `SkipOptOnETJump`, `AdoptSafeFallbackAsIncumbent`,
      `ComputeSafeFallback`, `AdoptFallbackIfUnschedulable`, `safe_fallback_` +
      accessors, `IntervalFallbackOutcome` + `FormatIntervalFallbackLogCsv`).
      NOT migrated: `WorstImportantTaskMissInfo` (stays in `SP_Metric` with the
      gate); `BuildWorstCaseDagAcrossIntervals`/`TaskStructureMatches` (already
      in `sources/TaskModel/WorstCaseDAG.cpp`). Mechanics (member-vs-helper)
      resolved at migration time; behavior-identical, TDD-green.
- [x] **D2 — RM-Fast also-unschedulable → THROW.** Mirror P0.7's rescue-also-
      fails throw (P0.8 certificate violation; loud not silent).
- [x] **D3 — No standalone `RM_FAST` mode.** Internal BF-branch swap only.

## 0a. Sequencing (open — awaiting user)
- [x] Decide: commit P0.7 Groups B/C/D FIRST (so the D1 migration doesn't
      restructure files the user is mid-reviewing), THEN migration, THEN P0.10
      builder+gate. RECOMMENDED. — RESOLVED 2026-08-01: P0.7 code is fully
      committed (`c87ae0d4`…`a8148dc7`…`f371c543`); `OptimizeSP_TL_Incre.{h,cpp}`
      + `testIncreOpt_w_TL.cpp` are CLEAN at HEAD → the D1 migration no longer
      tangles with an in-flight P0.7 review. Proceeding with §1 builder first;
      D1 migration (existing fall-back code out of `OptimizeSP_TL_Incre`) is its
      OWN later behavior-identical refactor commit (mechanics still open).

## 1. RM-Fast group-locked {pa, tl} builder — LANDED (TDD red→green)
> Mirror P0.9's `DeadlineMonotonicPriorityVec` (`OptimizeSP_TL_Incre.cpp:878`)
> with period key; TL = `timePerformancePairs[0].time_limit` (min grid) or -1.0
> (no grid), matching `DM_FAST` (`SimulationOrchestrator.cpp:399-419`).
- [x] TDD red: 3 builder tests — important-first lock (every important above
      every non-important); RM within group (shorter period = higher priority);
      fast TL (min grid / -1.0); tie-break by avg ET ascending. All RED vs stub.
- [x] Implement builder: free fn `RateMonotonicFastGroupLocked(const DAG_Model&)`
      → `ResourceOptResult` in NEW dedicated TU `sources/Optimization/OptimizeFallback.{h,cpp}`
      (D1 — the fallback TU; the existing-code migration is a SEPARATE later
      commit). Important-first lock + period key + avg-ET tiebreak; TL = smallest
      grid option else -1.0.
- [x] TDD green: 3/3 new tests pass; `testIncreOpt_w_TL` 119/119 (was 116);
      16/17 ctest (sole failure `CFS_RunOrchestrator_Binary` pre-existing).

## 1b. Extract shared `BuildPriorityPlan` + config — LANDED (TDD red→green)
> The "sort indices → fill {priority_vec, id2time_limit}" shape is shared by
> `RateMonotonicFastGroupLocked` AND BF's inline `DM`/`DM_FAST`/`DM_SLOW` modes
> (`SimulationOrchestrator.cpp:386-441`) — extract ONE parameterized builder.
- [x] New TU `sources/Optimization/PriorityBuilders.{h,cpp}`: `PriorityBuilderConfig`
      (`SortKey` deadline/period, `GroupLock` none/important-first, `TimeLimitPolicy`
      none/smallest-grid/largest-grid) + `BuildPriorityPlan(dag, config)`.
- [x] Rewrite `RateMonotonicFastGroupLocked` as a thin delegate:
      `{kPeriod, kImportantFirst, kSmallestGrid}`. 3 existing tests stay green
      (regression net).
- [x] TDD red→green: 2 new `BuildPriorityPlan` tests cover configs the RM-Fast
      tests don't — deadline key + no-lock (DM shape) + largest-grid TL (DM_SLOW
      shape). `testIncreOpt_w_TL` 121/121 (was 119). DM-mode unification (the 3
      inline `SimulationOrchestrator` branches delegate too) = LATER, not here.

## 2. BF gate + fallback swap — LANDED (TDD red→green), placement REVISED
> Gate BF's `res` via `ImportantTasksMeetThresholds` (self-contained overload);
> on FAIL → swap `res` to RM-Fast plan; on PASS → keep BF. On RM-Fast also-fail
> → throw (per D2).
- [x] TDD red: 3 tests on the free fn `AdoptRmFastFallbackIfUnschedulable`
      (dag, sp, bf_result) — BF-safe → kept; BF-unsafe → RM-Fast adopted;
      double-fail → `std::runtime_error`. RED vs stub (returns bf_result).
- [x] Implement: free fn in `OptimizeFallback.{h,cpp}` (gate BF's res via the
      self-contained overload; on FAIL swap to `RateMonotonicFastGroupLocked`;
      re-gate the RM-Fast plan; on second FAIL `CoutWarning` + throw).
- [x] Placement REVISED per user: gate lives INSIDE `EnumeratePA_with_TimeLimits`
      (`OptimizeSP_TL_BF.cpp`, after `optimizer.Optimize()`) — every caller is
      gated, not just the orchestrator's branch. Orchestrator BF branch reverted
      to original 1-line form. Safe for legacy BF tests: `is_important` defaults
      false → gate vacuously passes when no task is important
      (`SP_Metric.cpp:238`). Legacy suites `testOptimizePA`/`testBF_w_TL`/
      `testBFRTimeout` all green.
- [x] End-to-end test `EnumeratePA_with_TimeLimits.SwapsToRmFastWhenImportant-
      TaskMissesGate` — proves the gate fires through the BF entry point.
- [x] TDD green: 4/4 gate tests (3 free-fn + 1 e2e); `testIncreOpt_w_TL`
      125/125 (was 124, +1); legacy BF 10/5/2 green; 16/17 ctest (sole failure
      `testScheduleSimulate`/`CFS_RunOrchestrator_Binary` pre-existing — shells
      out to an absent RELEASE binary, verified clean-HEAD 39 pass / 1 fail).

## 3. Records + handoff — DONE (records); awaiting user commit
- [x] `dev_log.md` (this folder + top-level) + memory updated.
- [x] `git add` staged; user reviews (no commit).

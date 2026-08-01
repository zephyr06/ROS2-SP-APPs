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
- [ ] Decide: commit P0.7 Groups B/C/D FIRST (so the D1 migration doesn't
      restructure files the user is mid-reviewing), THEN migration, THEN P0.10
      builder+gate. RECOMMENDED. — OR — proceed with migration now.
      Flagged; not decided.

## 1. RM-Fast group-locked {pa, tl} builder — NOT STARTED
> Mirror P0.9's `DeadlineMonotonicPriorityVec` (`OptimizeSP_TL_Incre.cpp:878`)
> with period key; TL = `timePerformancePairs[0].time_limit` (min grid) or -1.0
> (no grid), matching `DM_FAST` (`SimulationOrchestrator.cpp:393-403`).
- [ ] TDD red: builder tests — important-first lock (every important above every
      non-important); RM within group (shorter period = higher priority); fast TL
      (min grid / -1.0); tie-break by avg ET ascending.
- [ ] Implement builder (free fn, location per D1).
- [ ] TDD green.

## 2. BF gate + fallback swap — NOT STARTED
> Gate BF's `res` via `ImportantTasksMeetThresholds` (self-contained overload);
> on FAIL → swap `res` to RM-Fast plan; on PASS → keep BF. On RM-Fast also-fail
> → throw (per D2).
- [ ] TDD red: BF-unsafe → RM-Fast adopted (assert gate verdicts + that `res`
      became the RM-Fast plan); BF-safe → `res` kept (BF's {pa,tl} unchanged).
- [ ] Implement gate+swap at the BF dispatch site
      (`SimulationOrchestrator.cpp:353-354`).
- [ ] TDD green: 17/17 ctest.

## 3. Records + handoff — NOT STARTED
- [ ] `dev_log.md` (this folder + top-level) + memory updated.
- [ ] `git add` staged; user reviews (no commit).

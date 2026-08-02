# P2.19 — Tasks (working checklist)

## 0. Triage + reproduce
- [x] Reproduce deterministically (RunOrchestrator taskset_3 INCR_Reopt_10 →
      core dump + loud-fail message).
- [x] Rule out stale binary: crashing binary (14:05) INCLUDES P2.18 fix
      `f371c543` (13:56); predates P0.10 §2 `02f3c8fd` (19:17) but that's
      IRRELEVANT (crash is INCR-side, not BF).
- [x] Create task folder + goal.md.
- [x] Verify the crash is the `ComputeSafeFallback` loud-fail throw (not P2.18's
      `|diff|>1` RTA-cache throw, not P0.10 §2's BF-gate throw).

## 1. Diagnose (which hypothesis — H1..H4) — DONE (H1 confirmed)
- [x] Instrument the loud-fail: print `WorstCaseImportantTaskMissInfo` for the
      failing candidate (which task, miss_chance vs threshold, excess).
- [x] Re-gate the SEED {DM PA, tl_seed} on the worst-case DAG BEFORE the walk —
      does the seed already miss? (H1/H3: feasibility-by-construction claim.)
      → YES: seed ok=0, task 0 miss=1.0 > thr=0.5. H1 confirmed.
- [x] Root cause: `BuildWorstCaseDagAcrossIntervals` uses
      `execution_time_dist.max_time` for ALL tasks; for a PERF task this is the
      TL-grid bound (= `execution_time_max`, e.g. 45.0), NOT the faithful WCET
      `et_mean` (= `execution_time_mu`, e.g. 4.225). P0.8's authoritative gate
      uses `execution_time_mu` for perf tasks → the worst-case DAG over-inflates
      perf WCET ~10x → seed TL = 45.0 (max grid option) → misses deadline 50.
      Verified: `exec_time_gauss.mu` (= `et_mean`) is set at read time
      (`RegularTasks.cpp:101`) for every YAML-loaded task → reliable source.

## 2. Fix — user's 3-point design (SUPERSEDES fix A; fix A used et_mean, user (1)
       uses min-TL-option — more conservative + robust). Scope DECIDED:
       (1)+(3) land in P2.19; (2) deferred to P2.20 (separate task).
  - Mechanism recap (why min-TL is sound): the gate
    `ImportantTasksMeetThresholds` BAKES the chosen TL into the perf dist
    (`ApplyTimeLimitsToTasksExecutionTime`, `SP_Metric.cpp:76-86`), so the
    worst-case DAG's perf dist only feeds the SEED selector
    (`SeedTimeLimitsAtOrBelowEtMean`) + DM tie-break (`DeadlineMonotonicPriorityVec`),
    NOT the gate's RTA. Min TL → least-interference seed → most feasible; the walk
    still has the FULL TL grid (`RecordTimeLimitOptions`) to climb. Non-perf
    dominance unchanged; perf soundness holds (gate certifies the chosen TL).
- [x] **(1)** `WorstCaseDAG.cpp` dist-overwrite loop (lines 62-70): split by type.
      Perf (`!timePerformancePairs.empty()`) → point mass at
      `timePerformancePairs[0].time_limit` (min TL option; same convention as
      `SmallestTimeLimitVec`, read from `interval_dags[0]`). Non-perf → unchanged
      (max `execution_time_dist.max_time` across intervals).
- [x] **(3)** Rename `BuildWorstCaseDagAcrossIntervals` →
      `BuildDAGForObtainSafeFallBAckAcrossIntervals` (user-specified). Touches:
      `DAG_Model.h` (decl + comment), `WorstCaseDAG.cpp` (def + 3 throw msgs),
      `SimulationOrchestrator.cpp:319` (prod caller), `testIncreOpt_w_TL.cpp`
      (13 refs). Rename-only commit. Casing "FallBAck" flagged at review
      (suggested `BuildSafeFallbackDagAcrossIntervals`); defer to user.
- [x] **TDD red→green:** rewrote `PreservesPerfTaskTimeLimitGrid` (4133) → assert
      perf worst-case dist = point mass at `timePerformancePairs[0].time_limit`.
      Added `WorstCaseDagUsesMinTimeLimitForPerfTasks`: perf max_time(45) >>
      min-TL(2.5) across 2 intervals → `GetAvgValue()` == 2.5. Both RED before
      fix, GREEN after. Non-perf tests unaffected (kept green).
- [x] Remove P2.19 temp diagnostics in `ComputeSafeFallback`
      (`OptimizeSP_TL_Incre.cpp`: seed-gate print + candidate print). KEPT the
      loud-fail re-gate throw — P1.15 net. Also fixed the now-stale seed comment.
- [x] **(2) DEFERRED → P2.20** (iterate `ComputeSafeFallback` to convergence:
      loop `OptimizeIncre_w_TL` until a full pass can't improve best SP). Behavior-
      change enhancement, NOT the crash fix. Stub task created in Step 0.
- [x] Build (`cmake --build build_test --target check.SP_OPT -j5 --clean-first`)
      + regression: `testIncreOpt_w_TL` 125/125; legacy BF (`testBF_w_TL` +
      `testBFRTimeout` + `testOptimizePA`) green; ctest 16/17 (sole pre-existing CFS).
- [x] Repro: `RunOrchestrator <taskset_3> /tmp/out INCR_Reopt_10 10000 1`
      → exit 0 (was SIGABRT). DM_FAST clean. BF slow (pre-existing, unrelated —
      BF doesn't call the worst-case DAG builder).

## 3. Records + handoff
- [x] `dev_log.md` (this folder) + memory + MEMORY.md. P2.20 stub created in
      `agents/active_tasks/P2_20_incr_fallback_convergence_loop/`.
- [x] Stale comment cleanup (missed by the first pass): the caller comment in
      `SimulationOrchestrator.cpp:316`, the header doc-comment in
      `DAG_Model.h:144`, the test section header `testIncreOpt_w_TL.cpp:3986`,
      and the `StochasticallyDominatesEveryInterval` comment all still described
      the OLD "point mass at max(execution_time_max) / stochastically dominates"
      behavior. Rewrote each to reflect perf=min-TL-option; tagged the
      dominance test as the non-perf soundness leg. Comment-only; rebuild green
      (126/126 testIncreOpt_w_TL; 16/17 ctest sole pre-existing CFS).
- [x] `git add` staged (P2.19 files only — P0.10 uncommitted work left
      unstaged); user reviews (no commit).

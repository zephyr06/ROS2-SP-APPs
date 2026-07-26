# P2.10 — Dev Log

## 2026-07-25 — Task filed; design + call paths grounded

### Why this task
Prior design guidance for a sibling task referenced opaque "R1–R4"/"R1–R5" labels
(review-round labels in `P3_2_reopt_incumbent_degradation/`), which the user could not
interpret. The deeper issue: the incremental/reopt code has (a) a duplicated Type-L
walk body and (b) confusable function names (`OptimizeSingleTaskTimeLimit` vs
`…_Impl`, `EvaluateTimeLimitConfig_ScratchOrIncre` vs `…_SubIncremental`). This
refactor makes both paths share one walk and names everything by job.

### The duplication (verified, exact)
The Type-L walk body is character-for-character identical in two places, differing
only in `entry.task_id` (incremental) vs `idx` (reopt):
- `PerformSerializedTaskQueueOptimization` (`OptimizeSP_TL_Incre.cpp:384-400`,
  incremental Type-L body): build eval lambda → `EvaluateTimeLimitConfig_SubIncremental`
  → backward `OptimizeSingleTaskTimeLimit_Impl(step=-1)` → forward `(step=+1)` →
  `ReconstructTimeLimitVecFromResOpt()`.
- `PerformCoordinateDescentForTaskConfigOpt` (`:538-563`, reopt sub-incremental arm):
  the SAME pattern, copy-pasted, gated by `ReoptimizationUseSubIncrementalWalk`.

### The merge design
Extract `OptimizeOneTaskTimeLimit` — builds the sub-incremental eval lambda
(binding `EvaluateTimeLimitConfig_SingleTaskPatch`) and calls the walk core
`WalkOneTaskTimeLimit`. Both descents call it for backward + forward passes. The reopt
legacy arm `WalkOneTaskTimeLimit_FullBeam` (binding
`EvaluateTimeLimitConfig_PAReopt`) stays reopt-only (default arm when the P2.9 flag
is OFF).

### Function calling paths (the APIs)

**Incremental path** (count % period ≠ 0):
```
OptimizeInterval(dag, K)                              // was Optimize_w_TL_ScratchOrIncre
└─ OptimizeIntervalIncremental(dag, K)                // was OptimizeIncre_w_TL
   └─ RunIncrementalTLDescent(K, tl, dag_prev_pre_tl) // was PerformSerializedTaskQueueOptimization
      ├─ ResetIncumbentBaseline(false); rta_cache_active_ = true
      ├─ baseline: EvaluateSPWithPriorityVec + CommitIncumbent
      ├─ BuildSerializedTaskQueue(dag_prev_pre_tl)
      └─ for each entry:
         ├─ Type-E → EvaluateTimeLimitConfig_SingleTaskPatch(...)
         └─ Type-L → OptimizeOneTaskTimeLimit(step=-1)  // SHARED
                   → OptimizeOneTaskTimeLimit(step=+1)  // SHARED
                   → ReconstructTimeLimitVecFromResOpt()
```

**Re-optimization path** (count % period == 0):
```
OptimizeInterval(dag, K)                              // was Optimize_w_TL_ScratchOrIncre
└─ OptimizeIntervalFromScratch(dag, K)                // was ReOptimizePeriodic
   └─ RunReoptTLDescent(K, tl, from_scratch=true)     // was PerformCoordinateDescentForTaskConfigOpt
      ├─ patience = ReoptimizationTimeLimitSearchPatience (=1)
      ├─ use_subincremental_walk = ReoptimizationUseSubIncrementalWalk  // P2.9 flag, default OFF
      ├─ ResetIncumbentBaseline(true)
      ├─ baseline: EvaluateTimeLimitConfig_PAReopt(from_scratch=true)
      ├─ if use_subincremental_walk: re-arm cache + AdoptChampion
      └─ for each TL-flexible task:
         ├─ if use_subincremental_walk:
         │    OptimizeOneTaskTimeLimit(step=-1)  // SHARED
         │    OptimizeOneTaskTimeLimit(step=+1)  // SHARED
         └─ else (legacy, default):
              WalkOneTaskTimeLimit_FullBeam(step=-1)        // reopt-only arm
              WalkOneTaskTimeLimit_FullBeam(step=+1)
```

**Shared by both:** `WalkOneTaskTimeLimit` (core),
`OptimizeOneTaskTimeLimit` (arm),
`EvaluateTimeLimitConfig_SingleTaskPatch`, `UpdateRecords`, `CommitIncumbent`,
`BuildChallengerFromIncumbent`, `ReconstructTimeLimitVecFromResOpt`,
`ResetIncumbentBaseline`.

### Dead code + outlier
- `TraverseTimeLimitOptions` — declared `OptimizeSP_TL_Incre.h:75`, NO definition
  anywhere (only other hit is a debug-cout string label at `:134`). Delete the
  declaration; repoint the label to `"UpdateRecords:"`.
- `Find_Close_ExecutionTime` — lone snake_case outlier (`:42`); rename to
  `FindClosestExecutionTimeIndex`.

### Status
Awaiting implementation. Gate = bit-identical SP at N=16 with the P2.9 flag OFF.

## 2026-07-25 — Phases 1–4 DONE (rename + dedup + build green)

### What landed (working tree, NOT committed)
The refactor was already mostly staged in the working tree (header fully renamed;
`.cpp` had the new definitions + the extracted `OptimizeOneTaskTimeLimit`
shared helper + the dedup of both walk blocks). This session finished the leftover
call-site renames so the tree builds and links.

**Phase 2 (.cpp leftovers — the link-breaking gap):** the `ReOptimizePeriodic`
definition was still named `ReOptimizePeriodic` while the header declared
`OptimizeIntervalFromScratch` and `OptimizeInterval` called it at `:603` — i.e. the
tree did NOT link. Renamed the definition to `OptimizeIntervalFromScratch`; fixed its
2 internal stale call sites (`OptimizeWithTimeLimitOptDisabled`→
`OptimizeIntervalWithTLOptDisabled`, `PerformCoordinateDescentForTaskConfigOpt`→
`RunReoptTLDescent`) + 2 stale comments (`PerformSerializedTaskQueueOptimization`→
`RunIncrementalTLDescent`, `OptimizeIncre_w_TL`→`OptimizeIntervalIncremental`).

**Phase 3a (SimulationOrchestrator.cpp):** 3 `Optimize_w_TL_ScratchOrIncre` call
sites → `OptimizeInterval` + 3 comment refs.

**Phase 3b (Parameters.h + parameters.yaml):** comment-only — `PerformCoordinate-
DescentForTaskConfigOpt`→`RunReoptTLDescent`, `EvaluateTimeLimitConfig_SubIncremental`
→`…_SingleTaskPatch`, `EvaluateTimeLimitConfig_ScratchOrIncre`→`…_GlobalBeam`.

**Phase 3c (tests + applications):** whole-word sed rename across 7 test files
(testIncreOpt_w_TL ~118 refs incl. the `RecordingDispatcherOpt`/`StartTLStub` virtual
overrides + TEST_F name prefixes like `ReOptimizePeriodic_…`→`OptimizeIntervalFromScratch_…`;
testINCRTimeout ~8; testScheduleSimulate ~4; RunOrchestrator ~2;
AnalyzePriorityAssignmentIncrementalExample ~2; testOptimizeIncrePA ~1; testBF_w_TL ~1)
+ 1 call site in `applications/real_time_manager/.../scheduler_wrapper.h`
(`OptimizeIncre_w_TL`→`OptimizeIntervalIncremental`; that subdir is commented out of
the top CMakeLists so it is NOT in the SP_OPT build, but renamed for consistency).

`agents/finished_tasks/` historical docs were NOT touched (immutable records).

### Verify
- `cmake --build build_test --target check.SP_OPT -j5 --clean-first` → clean, **17/17
  ctest green** (20.3s).
- `testIncreOpt_w_TL` → **53/53 green**, incl. the 2 P2.9 `CounterDispatcherSynthetic`
  `ReoptWalk_LeverA_*` tests (only override sigs changed; assertions unchanged).
- `grep` for all 11 old names across `sources/`+`tests/`+`applications/` → **0 hits**.

### Note on 5b
The P2.9 `ReoptWalk_LeverA_On_RoutesWalkTrialsThroughSubIncremental` test already
asserts `subincremental_calls>0` on a reopt dispatch with the flag ON — it passed, so
the "shared helper is reached from the reopt arm" half of 5b is already proven by the
unit test. 5a (N=16 bit-identity with flag OFF) is the remaining empirical gate.

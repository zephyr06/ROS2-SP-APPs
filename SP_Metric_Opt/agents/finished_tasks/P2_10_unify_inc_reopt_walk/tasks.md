# P2.10 — Tasks (working checklist)

> Behavior-preserving refactor: merge the duplicated Type-L walk into one shared
> helper, rename confusable functions to self-describing names. Gate = bit-identical
> SP on the default path (`ReoptimizationUseSubIncrementalWalk=0`).

## Phase 1 — Header (`sources/Optimization/OptimizeSP_TL_Incre.h`)

- [x] **1a. Rename declarations** per the rename table (goal.md): the 4 public
  dispatchers, the 2 descent bodies, the 2 virtual evals, the 2 walk fns; repoint
  comments.
- [x] **1b. Add `OptimizeOneTaskTimeLimit` declaration** (the new shared
  helper).
- [x] **1c. Delete `TraverseTimeLimitOptions` declaration** (dead code, `:75`).

## Phase 2 — Implementation (`sources/Optimization/OptimizeSP_TL_Incre.cpp`)

- [x] **2a. Rename all definitions** to match the header.
- [x] **2b. Extract `OptimizeOneTaskTimeLimit`** (builds the
  `…_SingleTaskPatch` eval lambda, calls `WalkOneTaskTimeLimit`).
- [x] **2c. Collapse the two duplicated walk blocks** — the incremental Type-L body
  (was `PerformSerializedTaskQueueOptimization:384-400`) and the reopt sub-incremental
  arm (was `PerformCoordinateDescentForTaskConfigOpt:538-563`) — to twin calls to the
  new helper.
- [x] **2d. Rename `Find_Close_ExecutionTime` → `FindClosestExecutionTimeIndex`**
  (def `:42`, uses `:64`, `:414`).
- [x] **2e. Fix stale debug-cout label** `"TraverseTimeLimitOptions:"` (`:134`) →
  `"UpdateRecords:"`.

## Phase 3 — Call sites

- [x] **3a. `SimulationOrchestrator.cpp`** — 3 call sites (`:325,334,342`) +
  comments → `OptimizeInterval`.
- [x] **3b. `Parameters.h`** — comment-only: repoint the
  `ReoptimizationUseSubIncrementalWalk` flag comment to the new walk-fn names.
- [x] **3c. Tests** — rename call sites + overridden virtuals + TEST_F names:
  `testIncreOpt_w_TL.cpp` (~118 refs, incl. `RecordingDispatcherOpt` overrides
  `:1075,1083` + `CounterDispatcherSynthetic` tests), `testINCRTimeout.cpp` (~8),
  `testScheduleSimulate.cpp` (~4), `RunOrchestrator.cpp` (~2),
  `AnalyzePriorityAssignmentIncrementalExample.cpp` (~2),
  `testOptimizeIncrePA.cpp` (~1), `testBF_w_TL.cpp` (~1).

## Phase 4 — Build + verify

- [x] **4a. Build** `cmake --build build_test --target check.SP_OPT -j5 --clean-first`
  (lib = `libSP_OPTDebug.so`; `--clean-first` to avoid stale-`.o` after header edit).
- [x] **4b. 17/17 ctest green** + **53/53 `testIncreOpt_w_TL` green** (the 2 P2.9
  `CounterDispatcherSynthetic` tests keep their assertions; only override sigs change).

## Phase 5 — Bit-identity gate

- [ ] **5a. Run `INCR_Reopt_1` at N=16 with flag OFF** (default); `ndiff`/stdout-diff
  SP + `scheduler_execution_time.txt` vs pre-refactor baseline → expect ZERO diff.
- [ ] **5b. Flip `ReoptimizationUseSubIncrementalWalk=1`**; confirm
  `subincremental_calls>0` on a count==0 reopt dispatch (the
  `ReoptWalk_LeverA_On_*` test) — proves the shared helper is reached from the reopt
  arm as well as the incremental arm.

## Phase 6 — Closeout

- [ ] **6a. Update `agents/overall_tasks.md`** with P2.10.
- [ ] **6b. Add memory** `p210-unify-inc-reopt-walk.md` + one-line pointer in
  `MEMORY.md`.
- [ ] **6c. Fold P2.9 working-tree changes** (the refactor rewrites P2.9's exact area;
  P2.9's A/B then runs on the refactored code).

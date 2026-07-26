# P2.16 — `OptimizeSP_TL_Incre` Readability & Dead-Code Refactor

## The Goal

A dedicated readability + dead-code pass on `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}`
(+ `tests/testIncreOpt_w_TL.cpp` for test-seam renames), **after** P2.11's two merge rounds
(Phase 2 `76c45114` behavior change + Phase 1b-1e `bd9f5912` structural unification) left
behind artifacts that no longer match the merged code. Per `agent_coding_rules.md`
("reduce repeated code", "ruthlessly prune unused features", "leave stages for code
refactor", "code readability is important", "use short functions; extract sub-functions
for long functions") this is the cleanup stage P2.11 explicitly deferred.

**Readability is improved by removing confusing names, dead code, and verbose history
comments — NOT by adding comments.**

## Why it matters now

P2.11 collapsed two descent bodies into `RunIntervalDescent` + `SeedBaselineAndArmCache` +
`enum class IntervalDescentMode`, leaving:

1. **A dead declaration** — `TraverseTimeLimitOptions` (`OptimizeSP_TL_Incre.h:84`) has no
   definition and no callers; its only `.cpp` occurrence is a stale `debugMode` print
   string literal inside `UpdateRecords` (`OptimizeSP_TL_Incre.cpp:134`) that names the
   non-existent function. (P2.10's checklist line "delete `TraverseTimeLimitOptions`" was
   never executed — the P2.10 rename was reverted — so the dead decl survived.)
2. **A confusing `_Impl` suffix** — `OptimizeSingleTaskTimeLimit_Impl` got its suffix from a
   deleted 7-arg wrapper (`OptimizeSingleTaskTimeLimit`, removed in P2.11 Phase 2a). With
   the wrapper gone, `_Impl` implies a non-existent "public" sibling. `WalkOneTaskTimeLimit`
   describes the behavior.
3. **Dense inline history comments** that re-narrate merged P2.11 history ("RELOCATED
   VERBATIM from the old…", stale "the legacy `PerformCoordinateDescentForTaskConfigOpt`
   remains the from-scratch/reopt descent" when both bodies are now thin delegators).
4. **A duplicated TL-sum tie-break** inlined twice in `UpdateRecords` (sum the
   non-(-1) TLs for new vs old, keep the lower sum on an approx-equal SP tie).

## Scope

**Files touched (ONLY):** `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}` +
`tests/testIncreOpt_w_TL.cpp`. All clean at HEAD `d1d05827` (on top of `bd9f5912`).

**Parallel-agent boundary:** another agent is concurrently editing
`agents/agent_coding_rules.md`, `agents/overall_tasks.md`,
`simulation_experiments/configs/p211_reopt_ab_config.json`, `tests/RunSpeedTest.cpp` — this
task does NOT touch any of those. No git operations; the user reviews + commits each step.
Agent runs `git add` only.

## Steps (each = one review/commit unit; behavior-neutral, TDD-pinned)

1. **Delete dead `TraverseTimeLimitOptions` + stale debug print.** Remove the header decl
   (`OptimizeSP_TL_Incre.h:84-85`) + the `if (GlobalVariables::debugMode){…}` block in
   `UpdateRecords` (`OptimizeSP_TL_Incre.cpp:131-136`). Dead-code; trivially bit-identical.
2. **Rename `OptimizeSingleTaskTimeLimit_Impl` → `WalkOneTaskTimeLimit`.** Header decl +
   `.cpp` definition + 2 call sites in `OptimizeOneTaskTimeLimit` + any test-seam overrides
   in `testIncreOpt_w_TL.cpp`. Pure rename; bit-identical.
3. **Trim dense inline history comments** to one-line invariants. Comments only; bit-identical.
4. **Extract `UpdateRecords`' TL-sum tie-break** into a named helper
   `SumTimeLimitsExcludingSentinel`. Pure extraction; bit-identical.

## Guardrails (what this task is NOT)

- NOT a behavior change. Every step is SP bit-identical (pinned by the existing
  `RunIntervalDescent_Incremental_MatchesWrapperSP` guard + the `*BitIdentical*` probes).
- NOT renaming the load-bearing test-seam names (`ReOptimizePeriodic`,
  `PerformCoordinateDescentForTaskConfigOpt`, `PerformSerializedTaskQueueOptimization`,
  `EvaluateTimeLimitConfig_ScratchOrIncre`/`_SubIncremental`, `Optimize_w_TL_ScratchOrIncre`) —
  these are virtual overrides in `RecordingDispatcherOpt` / `CounterDispatcherSynthetic` /
  `StartTLStub` at `testIncreOpt_w_TL.cpp:1100,:1612`; renaming cascades into many test
  files. User previously rejected broad renames (P2.10 13-fn rename dropped as "too many
  files / out of scope").
- NOT the Phase 0.5 `_PAReopt` rename (VOID per P2.11 memory; not pursued).
- NOT touching `OptimizeWithTimeLimitOptDisabled`'s `from_scratch` arg (user: keep as-is).
- NOT touching the cache contract (`|diff|<=1`), the beam search, the RTA layer, or any
  algorithm.

## Verification

After **each** step: `cmake --build build_test --target check.SP_OPT -j5 --clean-first`
→ expect 17/17 ctest green (per `sp-opt-test-build-debug-config` memory; `--clean-first`
after any header change per the P1.21 stale-`.o` lesson). No release-mode speed test needed
(no hot-path logic change; `RunSpeedTest.cpp` is in the parallel agent's set anyway).

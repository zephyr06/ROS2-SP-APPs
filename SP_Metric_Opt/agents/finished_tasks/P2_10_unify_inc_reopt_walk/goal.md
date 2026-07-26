# P2.10 — Unify the Incremental/Reopt TL Walk + Rename for Self-Description

## The Goal

Refactor the SP-Metric optimizer's incremental-optimization and re-optimization code
paths so that:

1. **Merged duplicated code.** The Type-L (TL walk) body is currently copy-pasted
   verbatim in two places — `PerformSerializedTaskQueueOptimization` (incremental) and
   `PerformCoordinateDescentForTaskConfigOpt` (reopt sub-incremental arm). They differ
   only in `entry.task_id` vs `idx`. Extract ONE shared TL-walk helper used by both
   paths. For the shared execution path the two already share, use the SAME functions.

2. **Self-describing function names.** Rename every function so its name states its
   goal. **Absolutely avoid functions with similar names but different
   responsibilities** — the prior `OptimizeSingleTaskTimeLimit` (legacy wrapper) vs
   `OptimizeSingleTaskTimeLimit_Impl` (the actual walk core, `_Impl` suffix on the real
   one) and `EvaluateTimeLimitConfig_ScratchOrIncre` vs `…_SubIncremental` were
   confusable. No external legend (R1–R4 style) should be needed to read the code.

## Why it matters now

The prior design guidance for a sibling task referenced opaque "R1–R4"/"R1–R5" labels
(review-round labels in `P3_2_reopt_incumbent_degradation/`), which the user could not
interpret. The deeper issue is that the code itself has confusable names AND duplicated
walk bodies, so reading either path requires cross-referencing two near-identical
blocks. This refactor makes both paths share one walk and name everything by job, so
the call graph is readable at a glance.

## The merge design

Extract `OptimizeOneTaskTimeLimit` — one helper that builds the
sub-incremental eval lambda (binding `EvaluateTimeLimitConfig_SingleTaskPatch`) and
calls the walk core `WalkOneTaskTimeLimit`. Both descents then call it for the
backward (`step=-1`) and forward (`step=+1`) passes, instead of each inlining the
lambda + twin core calls. The reopt legacy full-beam arm (`WalkOneTaskTimeLimit_FullBeam`,
binding `EvaluateTimeLimitConfig_PAReopt`) stays reopt-only — it is the default arm
when `ReoptimizationUseSubIncrementalWalk` (P2.9 lever A) is OFF.

## The rename table

| Current | New | Responsibility |
|---|---|---|
| `Optimize_w_TL_ScratchOrIncre` | `OptimizeInterval` | Public dispatcher: reopt vs incremental. |
| `ReOptimizePeriodic` | `OptimizeIntervalFromScratch` | Reopt entry: from-scratch compare-and-keep. |
| `OptimizeIncre_w_TL` | `OptimizeIntervalIncremental` | Incremental entry: warm-started serialized queue. |
| `OptimizeWithTimeLimitOptDisabled` | `OptimizeIntervalWithTLOptDisabled` | TL-opt bypass: pin min TL, single eval. |
| `PerformCoordinateDescentForTaskConfigOpt` | `RunReoptTLDescent` | Reopt descent body (from-scratch, patience=1). |
| `PerformSerializedTaskQueueOptimization` | `RunIncrementalTLDescent` | Incremental descent body (E+L queue, patience=0). |
| `EvaluateTimeLimitConfig_ScratchOrIncre` | `EvaluateTimeLimitConfig_PAReopt` | TL-config eval that re-optimizes PA (beam / multi-task incre). |
| `EvaluateTimeLimitConfig_SubIncremental` | `EvaluateTimeLimitConfig_SingleTaskPatch` | TL-config eval that patches 1 task (re-score carried PA + 1D \|diff\|<=1, cache-routed). |
| `OptimizeSingleTaskTimeLimit` (wrapper) | `WalkOneTaskTimeLimit_FullBeam` | Walk arm binding `…_GlobalBeam` (reopt legacy arm). |
| `OptimizeSingleTaskTimeLimit_Impl` (core) | `WalkOneTaskTimeLimit` | Walk core: injected eval, patience-bounded outward walk. |
| *(new, extracted)* | `OptimizeOneTaskTimeLimit` | Walk arm binding `…_SingleTaskPatch`. **Shared by both paths.** |
| `Find_Close_ExecutionTime` | `FindClosestExecutionTimeIndex` | Lone snake_case outlier → camelCase. |
| `TraverseTimeLimitOptions` | *(deleted)* | Dead code: declared, never defined. |

**Naming principle:** a shared stem (`OptimizeInterval…`, `EvaluateTimeLimitConfig_…`,
`WalkOneTaskTimeLimit…`) = same responsibility; the suffix = mechanism/strategy.

## Guardrails (what this task is NOT)

- NOT a behavior change. The refactor is bit-identical on the default path
  (`ReoptimizationUseSubIncrementalWalk=0`). Renames + dead-code deletion + helper
  extraction do not change control flow.
- NOT touching the cache contract (`|diff|<=1`), the beam search, the RTA layer, or
  any algorithm. Only names + the dedup extraction.
- The P2.9 flag + its two arms stay; P2.9 separately owns the result-changing flag flip
  via its A/B. This refactor folds in P2.9's uncommitted working-tree code (it
  rewrites the exact area P2.9 touched) but does NOT flip the flag.
- Tests keep their assertions; only override signatures + TEST_F names + call sites
  change to track renames.

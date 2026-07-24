# P1.11 — Dev Log

## 2026-07-17 — Task filed (design only, no code)

User request (verbatim): *"add a new task. i doubt that we don't need to
optimize all tasks during incremental optimization in each interval. i mean
not all env-tasks and tl tasks. i'm thinking about to add a tuning config to
only optimize X% of all possible tasks to optimize (among total number of env
+tl tasks). selection criteria is higher task weights imply higher selection
times, but we want to ensure some fairness. for example, if we only optimize
half of tasks each interval, then i suppose all tasks will be optimized at
least once after 3 intervals, like that. add a new task for this, and provide
more detailed design on implementation. if optimization task percentage is
1.0, that's current prod behavior"*

This is the "Extension idea (Per-interval task subset with cycling)" recorded
in P1.10's `goal.md`, now elevated to its own task with the percentage-based
parameterization. P1.10 (the serialized E+L queue this builds on) is COMPLETE +
committed (in `finished_tasks/`).

**Design recorded in `goal.md`** (no code written). Key decisions PROPOSED,
pending a user decisions pass on D1–D7:

- **Knob:** `IncrementalTaskOptimizationPercentage` (double in (0,1], default
  1.0 = current prod behavior, bit-identical). Required `parameters.yaml` key
  + range check at load (D1).
- **Subset size:** `K = ⌈X · |queue|⌉` per interval.
- **Selection:** two-bucket — stale bucket (staleness ≥ `CoverageHorizon(X)` →
  force-included, the fairness guarantee) + weight bucket (remaining slots to
  top-weight tasks). Subset emitted in the queue's weight-desc order (D4 binary,
  D7 order).
- **Coverage horizon:** `⌈1/X⌉` (derived, not a knob) → every task re-optimized
  within `⌈1/X⌉+1` intervals. X=0.5 → 3 intervals, matching the user's example
  (traced in `goal.md`, N=4 and N=10). (D2.)
- **State:** `intervals_since_last_optimized_` member (persistent across
  intervals — `incr_optimizer_` is the orchestrator's stable member,
  `SimulationOrchestrator.h:71`). Cleared by `ResetIncumbentBaseline(true)`
  (reopt re-searches all → fresh) — the coverage backstop.
- **Hook:** in `PerformSerializedTaskQueueOptimization` after
  `BuildSerializedTaskQueue`, before the walk. Single line.
- **Pure logic** extracted as free fn `SelectTaskSubset` (+ `CoverageHorizon`)
  for unit-testability, mirroring P1.10's `FindTimeLimitOptionIndex` /
  `IsBetterTimeLimitOption` pattern.
- **Correctness anchor:** X=1.0 early-returns the full queue + leaves staleness
  untouched → bit-identical. Regression test pins it.
- **Single-change invariant preserved** (subset = fewer single-task steps) →
  P1.9's cache premise unaffected.

**Open questions D1–D7** in `goal.md` — the user resolves these before Step 1.

**Next:** user decisions pass on D1–D7 → Step 1 (config + range check) per
`tasks.md`. No code until then.

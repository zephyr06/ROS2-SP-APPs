# P0.11 — Evaluate Real-World Exp Config: Optimizer Priority-Swap Hypothesis

**Priority:** P0 (real-world config correctness / optimizer-optimum evaluation)
**Status:** IN PROGRESS (scaffolded 2026-08-17)
**Depends on:** none (evaluation task, not bug-finding)

## Goal

Evaluate whether the **BF** (brute-force, `EnumeratePA_with_TimeLimits`) and
**incremental** optimizers still find the optimal priority assignment on the
real-world experiment config when the sp thresholds are tightened to realistic
values.

This is NOT a bug-finding task. It is an **evaluation**: take the real-world
4-task config, tighten its (currently too-lenient) sp thresholds, and check
that the optimizers produce the priority ordering the system semantics demand.

## The hypothesis to verify

The real-world system has two tasks competing on `processorId=0`: **TSP** (id 0,
any-time) and **SLAM** (id 3). The optimal priority ordering between them
depends on SLAM's execution-time (ET) regime:

- **When SLAM's ET is LOW**  → optimizer should assign **TSP HIGHER priority**
  (SLAM is cheap, so SLAM can safely be demoted; TSP's any-time budget grows).
- **When SLAM's ET is HIGH** → optimizer should assign **TSP LOWER priority**
  (SLAM is expensive and must not be starved by TSP; SLAM gets priority).

The optimizer must reproduce this swap across the two SLAM-ET regimes. Verifying
this on the real-world config (not just on generated tasksets) is the deliverable.

## The real-world config

Source yaml (the one the real-world experiment reads):
`/home/zephyr/Programming/ROS2-SP-APPs/all_time_records/task_characteristics.yaml`

Current 4 tasks:

| id | name | processorId | period | deadline | sp_threshold | sp_weight | notes |
|----|------|-------------|--------|----------|--------------|-----------|-------|
| 0  | TSP  | 0           | 1500   | 1500     | 0.5          | 1         | any-time (performance_records_time/perf) |
| 1  | MPC  | 1           | 10     | 10       | 0.99         | 1         | tiny ET, near-deterministic |
| 2  | RRT  | 1           | 4000   | 4000     | 0.5          | 1         | wide ET sigma |
| 3  | SLAM | 0           | 2000   | 2000     | 0.9          | 2         | highest weight → the important task |

- **TSP & SLAM share processorId=0** → they compete for priority; this pair is
  the crux of the hypothesis.
- **MPC & RRT share processorId=1** → separate core; their ordering is
  independent of the TSP/SLAM swap.
- SLAM has `sp_weight=2` (highest) → under P0.9's "top-50% by sp_weight" rule,
  SLAM is the `is_important` task.

## The threshold problem (user note)

Current sp thresholds (TSP=0.5, MPC=0.99, RRT=0.5, SLAM=0.9) are **too high**
(too lenient — they tolerate a 50–90% deadline-miss probability, which is not a
realistic safety bar). They should be **much lower**: e.g. a sweep from 0.9 down
to 0.1 or 0.01.

`sp_threshold` semantics (verified in `SP_Metric.h`/`SP_Metric.cpp`): it is a
deadline-miss-probability threshold; the important-task gate requires
`ddl_miss_chance <= threshold`. **Lower threshold = stricter.**

Tightening the threshold makes the important-task gate harder to satisfy, so the
optimizer is forced to pick a priority ordering that actually keeps SLAM (the
important task) schedulable — which is where the TSP/SLAM swap becomes binding.

## Method

1. Make experiment variants of the real-world yaml (do NOT mutate the original
   in place; copy into `TaskData/` under this task's scratch area).
2. Tighten sp thresholds (primary lever): sweep 0.9 → 0.1 → 0.01.
3. Create two SLAM-ET regimes to exercise the swap:
   - **SLAM-ET-low**  variant: reduce SLAM `execution_time_mu`/`sigma`.
   - **SLAM-ET-high** variant: raise SLAM `execution_time_mu`/`sigma`.
4. Run the **BF optimizer first** (`AnalyzePriorityAssignment` binary) on each
   (threshold × SLAM-ET-regime) variant.
5. Read the output priority assignment; verify the TSP-vs-SLAM ordering matches
   the hypothesis for each regime.
6. Then run the **incremental optimizer** for comparison (does it match BF?).
7. Record results in `dev_log.md` and report back.

## Entry point (BF)

```
./<build>/tests/AnalyzePriorityAssignment \
  --file_path <variant.yaml> \
  --output_file_path <out.yaml>
```

Binary locations: `release/tests/AnalyzePriorityAssignment` (RELEASE) and
`build/tests/AnalyzePriorityAssignment` (DEBUG). Output convention: **bigger
integer = higher priority** (contrary to the in-optimization print output).

## Done when

- BF run on the tightened-threshold real-world config, both SLAM-ET regimes.
- TSP-vs-SLAM priority ordering recorded for each run.
- Hypothesis verdict: confirmed or refuted, per regime.
- Incremental optimizer run for comparison (matches BF or not).
- Findings recorded in `dev_log.md`; milestone line appended to top-level
  `agents/dev_log.md`.

## Out of scope

- The python `simulation_experiments/` framework (it runs GENERATED tasksets of
  4–16 tasks via `paper_simulation_config.json`, not this 4-task yaml). This
  task runs the optimizers on the real-world yaml directly via the
  `AnalyzePriorityAssignment` binary.
- Bug fixing: if the optimizer fails to find the optimum, that is a finding to
  report (and may seed a separate P1 investigation), not an in-scope fix here.
- Mutating the original `all_time_records/task_characteristics.yaml`.

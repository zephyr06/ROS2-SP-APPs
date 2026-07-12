# P1.7 — Tasks (working checklist)

> **RESOLVED 2026-07-12** — fix applied + TDD-verified; staged on
> `clean_simulation` (NOT committed; user runs the A/B re-run). Folder moved to
> `finished_tasks/`. See `goal.md` (resolution summary) + `dev_log.md`
> (2026-07-12 RESOLVED entry) for the outcome. The checklist below is the
> pre-implementation plan, kept as the record of what was settled/done; checks
> marked to reflect the completed state.
> One review-and-commit cycle per `agent_coding_rules.md`. TDD: tests first.

## Step 0 — Settle design decisions with the user (DONE 2026-07-12)

- [x] D1 — Fix scope: partition the runtime simulator only. Generator
      `per_core_cpu_util` re-calibration DEFERRED (decide only after the fixed
      simulator produces honest per-core util).
- [x] D2 — Queue structure: mirror the legacy per-core `RunQueue` from
      `SimulatedFTP_SingleCore` (one `std::unique_ptr<RunQueue>` per distinct
      `processorId`, lockstep, merged schedule).
- [x] D3 — CFS sibling in the same pass: YES (`CFSSimulationOrchestrator::SimulateInterval`
      partitioned symmetrically via a `CFSPerCoreState` per-core struct).
- [x] D4 — Stale results policy: re-run on the fixed sim; discard old 1-core
      miss-rate numbers (user runs the A/B re-run).
- [x] D5 — "Sensitivity to priority order" narrative re-frame: DEFERRED, and
      MOOT for SP — SP's priority sensitivity is the intended partitioned-RTA
      behavior; the overload-artifact framing applies only to miss-rate exports.

## Step 1 — TDD: tests first (DONE 2026-07-12)

- [x] `tests/testScheduleSimulate.cpp` — 2 new tests
      (`SimulateIntervalPartitionsByProcessorId` + `_CFS`) on new input dir
      `tests/test_data_partition_two_cores/` (2 tasks, equal period/ET,
      different `processorId`: 0 and 1; both assert job-0 of each task starts
      at `t=0` → parallel, not serialized).
- [x] Confirmed the test FAILS on the single-queue code (red before green) —
      under the old code Task1.start would be `2`.
- [x] Existing `testScheduleSimulate.cpp` partitioning tests still green
      (34/34; they were always on the correct path).

## Step 2 — Partition the runtime simulator (DONE 2026-07-12)

- [x] `FixedTaskPrioritySchedulingOrchestrator::SimulateInterval` — replaced
      the single `RunQueue` with `std::vector<std::unique_ptr<RunQueue>>` (one
      per distinct `processorId` via `GetProcessorIds`), lockstep-stepped;
      `ReleaseJobs` gained `int processor_id = -1` + the partition skip.
      Preserved `ObtainSP_TaskSet_And_TimeLimits` over the (unchanged) merged
      schedule.
- [x] `CFSSimulationOrchestrator::SimulateInterval` — same partitioning via a
      `CFSPerCoreState` per-core struct; `ReleaseJobsCFS` gained the same param.
- [x] `RecordFinishedJobs` / `RecordFinishedJobsCFS` — confirmed correct with
      N queues (unchanged signatures; a job lives in exactly one queue).
- [x] `ScheduleSimulation.h` — added the missing `GetProcessorIds` declaration.

## Step 3 — Verify + measure (DONE 2026-07-12)

- [x] `cmake --build build --target check.SP_OPT -j5` (DEBUG) — 16/16 ctest
      green; `testScheduleSimulate` 34/34 (incl. the 2 new tests).
- [x] Spot-checked a 2-core taskset's produced schedule: parallel windows
      present (both cross-core jobs start at `t=0`); single-core tasksets
      byte-identical (all tasks land in the same queue → regression guards
      green).
- [ ] Re-derive utilization on the fixed simulator's miss-rate path — DEFERRED
      to the user's A/B re-run (D1 generator calibration decision depends on
      it; not needed for the code fix itself).

## Step 4 — Re-run + docs (docs DONE 2026-07-12; re-run is the user's)

- [ ] User re-runs P0.4 evaluation suite + P25 A/B + P0.3 figures on the
      fixed simulator (user-owned follow-up; standing constraint).
- [ ] Compare new `comparison_summary.csv` to old: which miss-rate/response-time
      columns moved (NO gate verdict moves — SP was never distorted; see the
      pivotal correction in `dev_log.md`). Record in `dev_log.md` after the
      re-run.
- [x] `agents/overall_tasks.md` — P1.7 row struck through + moved to the
      Completed index; execution-order note updated.
- [x] Top-level `agents/dev_log.md` — P1.7 milestone appended.
- [x] Memory `cpu-partition-mismatch.md` — added + corrected (the "deferred
      P1 task" wording → P2.6, priority P2); MEMORY.md pointer present.
- [x] `git add` the P1.7 unit; hand to user for review (no commit).

## Standing constraints

- No `git commit` (user's task; `git add` only).
- No running the A/B myself (user runs `run_end_to_end.sh`).
- Don't conflate the simulator fix with a generator re-calibration — D1 is
  decided only after the fixed simulator produces honest per-core util.
- The SP metric was never distorted by this bug (analytic partitioned RTA);
  only miss-rate/response-time exports were. The residual "make SP consume
  RunQueue RTs" is filed as **P2.6**.

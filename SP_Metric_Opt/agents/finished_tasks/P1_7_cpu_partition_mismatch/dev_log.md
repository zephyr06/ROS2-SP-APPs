# P1.7 — Dev log

## 2026-07-12 — RESOLVED: fix applied + TDD-verified (staged, not committed)

**Decision.** User greenlit the fix and locked the open design decisions:
D2 = two per-`processorId` `RunQueue`s, lockstep, merged schedule (mirror the
proven legacy `SimulatedFTP_SingleCore`/`SimulatedCFS_SingleCore`); D3 = fix
FTP + CFS in one pass; D4 = re-run on the fixed sim, discard old 1-core
miss-rate numbers; D1 (generator `per_core_cpu_util` re-calibration) deferred
— decide only after the fixed sim produces honest per-core util; D5
("sensitivity to priority order" narrative re-frame) deferred / moot for SP
(see the correction below).

**Fix.** `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp`:
both `SimulateInterval`s now call `GetProcessorIds(dag_tasks)` and build one
`RunQueue` per distinct `processorId` (held in `std::unique_ptr<RunQueue>` —
`RunQueue` is non-assignable due to its `const TaskSetInfoDerived` member),
stepping all queues in lockstep (Remove/Record/Release/Run per queue per
tick). `ReleaseJobs`/`ReleaseJobsCFS` gained an `int processor_id = -1` param
(`-1` = release all, preserving the unit-test call sites); when `>= 0`, tasks
whose `processorId` doesn't match are skipped. `RecordFinishedJobs` /
`RecordFinishedJobsCFS` unchanged — they take `RunQueue&` and read only that
queue's `schedule_`, so per-queue calls merge into the shared `job_history_`
(a job lives in exactly one queue; the `find_if` dedup is a no-op across
queues). `ScheduleSimulation.h` got the missing `GetProcessorIds` declaration
(the helper was defined in the `.cpp` but never declared in any header —
latent hygiene fix).

**TDD.** 2 new red→green tests in `tests/testScheduleSimulate.cpp`
(`SimulateIntervalPartitionsByProcessorId` + `_CFS`) on new input dir
`tests/test_data_partition_two_cores/` (2 tasks, equal period 10 / ET 2,
different `processorId`: 0 and 1). Both assert job-0 of each task starts at
`t=0` (parallel); under the old single queue, Task1.start would be `2`
(serialized behind Task0) — the assertions are constructed to fail on the
buggy behavior. They drive the real runtime path (`RunSimulation →
SimulateInterval → RecordFinishedJobs → job_history_`), not the already-
partitioned legacy path.

**Verified.** `cmake --build build --target check.SP_OPT -j5` (DEBUG;
`libSP_OPTDebug.so`) → 16/16 ctest green; `testScheduleSimulate` 34/34
(incl. the 2 new tests). Single-processor regression guards
(`ExactResponseTimeValidation`, `RateMonotonicPriorityAssignment`,
`simulate_schedule_v3`, `simulate_cfs_*`, `CFSOrchestration`) stayed green —
one-core tasksets are byte-identical (all tasks land in the same queue).

**Pivotal blast-radius correction (overturns the original 2026-07-12 eval).**
While wiring the fix, traced where the SP metric the eval-suite gates read
actually comes from. `SimulateInterval` calls `ObtainSP_TaskSet_And_TimeLimits`
(`SP_Metric.cpp:82`) → `ObtainSP_TaskSet` (line 53) →
`ProbabilisticRTA_TaskSet(tasks)` (`RTA.cpp:100`), which **already partitions
on `processorId`** via `ExtractTaskSetPerProcessor` (`RTA.cpp:87`, keyed on
`task.processorId` line 91) and runs `ProbabilisticRTA_TaskSet_SingleCore`
per partition (line 113). A task's RTA only accumulates preemptions from
same-processor higher-priority tasks — correct 2-core semantics, independent
of the `RunQueue`. So `mean_sp_norm` / `Mean_SP_Metric` (what every gate
Q1/Q2/Q3/E1/E3 reads) was **always on the correct path**, and **no gate
verdict moves** after the fix. The single-queue bug only distorted the
`job_history_`-derived exports — `miss_rate_summary.txt`,
`miss_rate_per_task.txt`, `task_aggregate_*.txt`, `response_times_task_*.txt`
→ the `Mean_Miss_Rate`/`Important_Miss_Rate`/response-time columns in
`comparison_summary.csv` (read by `aggregate_across_tasks.py` + `utils.py`,
NOT by the gates). E3 (period-monotonicity, `mean_sp_norm`) was **unrelated
to the bug** — it PASSES on the analytic partitioned path regardless. D5 is
**moot for SP** — SP's priority sensitivity is the intended partitioned-RTA
behavior, not an overload artifact; the "sensitivity is an overload artifact"
framing applies only to the miss-rate exports.

**Residual follow-up (filed as P2.6).** The user acknowledged the
SP-from-RunQueue gap: the SP metric still comes from the analytic RTA, not
from the RunQueue's actual simulated RTs. The user wants the simulation to
eventually report BOTH the analytical SP and the true SP derived from the
simulated response-time samples — that is P2.6
(`agents/active_tasks/P2_6_sim_rt_based_sp_metric/`), deferred for later.
This P1.7 fix (correct per-`processorId` RunQueues producing honest RT
samples) is P2.6's prerequisite.

**Standing constraints honored.** No `git commit` (staged via `git add` only,
user reviews). No A/B run by me (user runs `run_end_to_end.sh`). TDD (tests
first, red before green). Folder moved to `finished_tasks/`;
`finished_tasks/summary.md` + `overall_tasks.md` + top-level `dev_log.md`
updated; memory `cpu-partition-mismatch.md` corrected (the "deferred P1 task"
wording → P2.6, priority P2).

---

## 2026-07-12 — Finding confirmed (evaluation only, no implementation)

**Trigger:** the user pasted an "Additional Simulator Finding: CPU Mismatch"
claim (external analysis) — that
`FixedTaskPrioritySchedulingOrchestrator::SimulateInterval` runs all tasks on
a single run queue, ignoring `processorId`, overloading the one simulated
core (~221%) and starving low-priority tasks — and asked to "first focus on
evaluating whether this is true before doing anything," then create a task if
so.

**Method:** read the runtime simulator path and the legacy partitioning path,
then computed utilization from the 600 generated interval YAMLs in the
`evalsuite_run_test_dur600_interval10_seed1000_tasks4` sim dir.

**Verdict: TRUE.** Every clause confirmed:

1. **Single run queue, no partitioning.**
   `FixedTaskPrioritySchedulingOrchestrator::SimulateInterval`
   (`SimulationOrchestrator.cpp:492-528`) builds one
   `RunQueue run_queue(dag_tasks.tasks)` (line 502) over the whole task set.
   `ReleaseJobs` (lines 440-490) filters only on `time_now % task.period == 0`
   (line 450) — **no `processorId` check**. One `processor_free_` flag
   (`RunQueue.h:207`) serves all tasks regardless of declared core.
2. **`processorId` is in the YAML and loaded.** Generated configs carry
   `processorId: 0/1` (e.g. `taskset_8/taskset_characteristics_interval_12.yaml`);
   `RegularTasks.cpp:84-85` loads it into `Task.processorId` (`RegularTasks.h:99`).
3. **The overload is real and worse than the headline.** Across 600 generated
   interval YAMLs, single-queue total util (`mu/period`): min 1.007, **max
   3.332**, mean **2.099**; 350/600 tasksets >200%. Correct per-proc max util:
   min 0.706, max 1.839, mean 1.160. So the "221%" is a representative sample,
   not the worst case, and correct partitioning is far less pathological.
4. **Priority-order sensitivity is a consequence.** On one overloaded core,
   only the top-priority chain gets service; reordering priorities moves large
   blocks of zero SP. The "extreme sensitivity to priority assignment order"
   is an overload artifact, not a property of the intended 2-core system.

**Key nuance — the partitioning code already exists, it's just not on the
runtime path.** `ScheduleSimulation.cpp` partitions correctly:
`GetProcessorIds` (lines 40-52) + `SimulateFixedPrioritySched` (lines 54-66)
loops cores calling `SimulatedFTP_SingleCore(..., processor_id)` with
`AddTasksToRunQueue` filtering on `task.processorId == processor_id` (line 13).
But that path is called only from `RTDA_Prob.h:29` (RTA response-time
analysis) and `tests/` — NOT from the runtime orchestrator that produces the
interval SP metrics the eval suite / A/B / figures read. The CFS sibling
`CFSSimulationOrchestrator::SimulateInterval` (lines 684-749) has the
identical single-queue bug.

**Blast radius:** common-mode distortion of every SP/miss-rate number for
every arm (INCR, INCR_Reopt_{1,5,10,30,60}, BF, RM, CFS, INCR_NO_TL,
INCR_WCET) — all run on the wrong (1-core) hardware model. Relative
comparisons less wrong than absolute, but any effect interacting with
overload (priority order, TL budgets, reopt period) is confounded with the
starvation artifact. E1 verdict (overhead, not load) unaffected in spirit;
E3 (period-monotonicity, `mean_sp_norm`) PASSES under the bug — may not
survive the fix. Optimizer internals (P0.5 incumbent, P1.4 seed,
trial-and-error TL walk) are algorithmically sound; only their *measured* SP
deltas are confounded. RTA path (`RTDA_Prob.h`) and `testScheduleSimulate` /
`testSP` were always on the correct partitioning path.

**Filed:** `agents/active_tasks/P1_7_cpu_partition_mismatch/{goal,tasks}.md`
with open design decisions D1–D5 (fix scope; queue structure; CFS in same
pass; stale-results policy; sensitivity re-frame). **No code changes.**
Implementation waits for user greenlight + the D1–D5 decisions.

**Standing constraints honored:** no `git commit`; no A/B run by me; evaluate-
before-act per the user's directive.

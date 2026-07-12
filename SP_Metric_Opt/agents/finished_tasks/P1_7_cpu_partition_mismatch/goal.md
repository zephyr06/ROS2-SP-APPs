# P1.7 — Simulator ignores `processorId` partitioning (single run-queue overload)

**Priority:** P1 (simulator correctness; affects the miss-rate/response-time
numbers the simulation exports)
**Status:** **RESOLVED 2026-07-12** — fix APPLIED + TDD-VERIFIED (2 new
red→green tests; `testScheduleSimulate` 34/34, ctest 16/16 green), staged on
`clean_simulation` (NOT committed; user runs the A/B re-run). Folder moved to
`finished_tasks/`. The evaluation below is preserved verbatim as the finding
record; the resolution is summarized in `dev_log.md` and in
`finished_tasks/summary.md`.

> **Resolution summary.** Both `FixedTaskPrioritySchedulingOrchestrator::SimulateInterval`
> and `CFSSimulationOrchestrator::SimulateInterval`
> (`sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp`) now call
> `GetProcessorIds(dag_tasks)` and build one `RunQueue` (held in
> `std::unique_ptr` — `RunQueue` is non-assignable due to its
> `const TaskSetInfoDerived` member) per distinct `processorId`, stepping all
> queues in lockstep (Remove/Record/Release/Run per queue per tick).
> `ReleaseJobs`/`ReleaseJobsCFS` gained an `int processor_id = -1` param
> (`-1` = release all, preserving the unit-test call sites); when `>= 0`, tasks
> whose `processorId` doesn't match are skipped. Mirrors the proven legacy
> `SimulatedFTP_SingleCore`/`SimulatedCFS_SingleCore` per-core model. 2 new
> tests in `tests/testScheduleSimulate.cpp`
> (`SimulateIntervalPartitionsByProcessorId` + `_CFS`) on new input dir
> `tests/test_data_partition_two_cores/` (2 tasks, equal period/ET, different
> `processorId`: buggy code serialized `Task1.start==2`; fixed runs
> `Task1.start==0` parallel). Single-core tasksets are byte-identical (all
> tasks land in the same queue → the regression guards stayed green).
>
> **PIVOTAL CORRECTION (overturns the original blast-radius eval below):** the
> SP metric is **analytic, not simulation-derived**, and was **NEVER distorted**
> by the single-queue bug. `SimulateInterval` calls
> `ObtainSP_TaskSet_And_TimeLimits` (`SP_Metric.cpp:82`) → `ObtainSP_TaskSet`
> (line 53) → `ProbabilisticRTA_TaskSet` (`RTA.cpp:100`), which **already
> partitions on `processorId`** via `ExtractTaskSetPerProcessor` (`RTA.cpp:87`).
> So `mean_sp_norm` / `Mean_SP_Metric` (what every eval-suite gate Q1/Q2/Q3/E1/E3
> reads) was always on the correct path, and **no gate verdict moves** after the
> fix. The bug only distorted the `job_history_`-derived exports (miss-rate /
> response-time columns in `comparison_summary.csv`, read by
> `aggregate_across_tasks.py` + `utils.py`, NOT by the gates). E3 was unrelated.
>
> **Residual follow-up (filed as P2.6):** the SP metric still comes from the
> analytic RTA, not from the RunQueue's actual simulated RTs. The user wants
> the simulation to eventually report BOTH the analytical SP and the true SP
> derived from the simulated response-time samples — that is P2.6
> (`agents/active_tasks/P2_6_sim_rt_based_sp_metric/`), deferred for later.
> This P1.7 fix (correct per-processorId RunQueues producing honest RT samples)
> is P2.6's prerequisite.

---

## Follow-up note (added 2026-07-12) — generator feasibility, the D1 "separate pass"

P1.7's D1 (below) deferred the generator-calibration question: *"fix the
simulator to partition first, measure, then decide whether the generator needs a
separate pass."* The simulator pass is DONE; the **generator pass is now
confirmed warranted**, surfaced by the P1.8 investigation
(`agents/active_tasks/P1_8_incr_wcet_outperforms_incr/`).

**Empirical sweep of the same N=4 run's 10 generated tasksets**
(`taskset_characteristics_interval_0.yaml`, 40 tasks total), checking ET vs
period and WCET vs deadline:

| violation | tasks affected | tasksets affected |
|---|---|---|
| `execution_time_mu > period` (avg ET exceeds period — the user's flag) | 2/40 | 2/10 (taskset_1 t2: mu=21.4/P=20; taskset_7 t2: mu=518.1/P=500) |
| `execution_time_max > deadline` (WCET exceeds deadline) | ~18/40 | **9/10** (only taskset_9 is clean) |
| `execution_time_mu > deadline` with σ=1.0 (deterministic certain-miss) | 5/40 | 5/10 (taskset_0 t3, taskset_1 t0+t2, taskset_5 t3, taskset_7 t3) |

**So the unschedulable-task problem is systemic, not a one-off.** The user
flagged the `mu > period` case (taskset_1 t2) specifically; the broader
`WCET > deadline` case is ~9× more prevalent and was already the substrate of
P1.8's Finding 3. A task with WCET > deadline is analytically unschedulable
unless a time limit strictly < deadline is adopted — and most of these tasks
have **no** `timePerformancePairs` (no TL ladder), so no TL can rescue them.

**What this means for P1.7's D1:** the generator enforces no bound on avg-ET vs
period (nor WCET vs deadline) — confirmed empirically. Whether the source
enforces *any* feasibility constraint, and where `per_core_cpu_util` calibrates
ET, is being traced in P1.8's Step 0 (source citation to be recorded there).
The generator-feasibility fix (clamp `ET_max ≤ deadline`, or `mu ≤ k·period`, or
reject+regenerate) is the D1 "separate pass" P1.7 deferred — it is now an active
question under **P1.8's D1** (is WCET>deadline a generator bug → F1, or
intentional stress → F2/F4?), NOT a P1.7 reopen. P1.7 stays RESOLVED (the
simulator partitioning fix is correct and standalone); this is the downstream
generator pass P1.7 always said would be a separate decision.

**Cross-link:** P1.8 `goal.md` Finding 3 + H1d; P1.8 `dev_log.md` 2026-07-12.

---

## Original finding (2026-07-12, preserved as the evaluation record)

**Original status:** FINDING CONFIRMED 2026-07-12 (code + data verified; **NO
implementation** — task filed for the user to greenlight, per the standing
"investigate first, then create a task" directive). This file records the
finding (what's wrong), the evidence (code + measured utilization), the
blast radius (what it invalidates and what it doesn't), and the open design
decisions to settle BEFORE coding.

## The finding (verbatim, from the user's prompt)

> "Additional Simulator Finding: CPU Mismatch
> As part of this investigation, we also discovered that
> `FixedTaskPrioritySchedulingOrchestrator::SimulateInterval` executes all
> tasks on a single run queue, ignoring the `processorId` partitioning in the
> YAML. This means the CPU utilization on the single processor is 221%
> (overloaded), causing low-priority tasks to be completely starved. This
> severe overload is why the system is extremely sensitive to priority
> assignment order."

## Verdict: TRUE (confirmed 2026-07-12)

Every clause checks out against the current `clean_simulation` tree:

1. **Single run queue, no partitioning** — CONFIRMED.
   `FixedTaskPrioritySchedulingOrchestrator::SimulateInterval`
   (`sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp:492-528`)
   builds **one** `RunQueue run_queue(dag_tasks.tasks)` (line 502) over the
   **entire** task set and steps it ms-by-ms. `ReleaseJobs` (lines 440-490)
   iterates `dag_tasks.tasks` with the only filter being
   `time_now % task.period == 0` (line 450) — **no `processorId` check**. So a
   job on `processorId:0` and a job on `processorId:1` compete for the one
   `processor_free_` flag in `RunQueue` (`RunQueue.h:207`). Two cores declared
   in YAML, one core simulated.
2. **`processorId` IS in the YAML and IS loaded** — CONFIRMED.
   Generated interval configs carry it, e.g.
   `runs/.../taskset_8/taskset_characteristics_interval_12.yaml`:
   `task_1 → processorId: 0`, `task_2 → processorId: 1`, `task_3 → 0`,
   `task_4 → 1`. `RegularTasks.cpp:84-85` loads it into `Task.processorId`
   (`RegularTasks.h:99`, default `-1` = "all on one processor by default").
3. **The "221%" overload** — CONFIRMED, and the real picture is worse than the
   headline. Computed utilization from the 600 generated interval YAMLs in the
   `evalsuite_run_test_dur600_interval10_seed1000_tasks4` sim dir (using
   `execution_time_mu / period`):
   - **Single-queue total util** (what the simulator actually sees): min 1.007,
     max **3.332**, mean **2.099** → 350 of 600 tasksets are **>200%** loaded
     on the one simulated core. The "221%" figure is a representative sample,
     not a worst case.
   - **Max-per-processor util** (what correct partitioning would see): min
     0.706, max 1.839, mean 1.160 — i.e. one of the two declared cores is
     still over-loaded on average, but the system is far less pathological
     when the two cores actually run in parallel.
4. **"Sensitive to priority assignment order"** — CONFIRMED as a consequence.
   On a single overloaded core, only the top-priority chain gets any service
   and everything below the utilization cliff is starved, so reordering
   priorities moves large blocks of (zero) SP between tasks. That is exactly
   the regime where priority assignment looks disproportionately important —
   it's an artifact of the overload, not a property of the intended 2-core
   system. This re-frames a chunk of the priority-assignment sensitivity the
   project has been chasing.

### The legacy partitioning code already exists — it's just not on this path

`sources/RTDA/ImplicitCommunication/ScheduleSimulation.cpp` **does** partition
correctly: `GetProcessorIds(dag_tasks)` (lines 40-52) collects the distinct
`processorId`s, and `SimulateFixedPrioritySched` (lines 54-66) loops over
them, calling `SimulatedFTP_SingleCore(..., processor_id)` per core with
`AddTasksToRunQueue` filtering on `task.processorId == processor_id`
(line 13). `SimulateCFSSched` mirrors this (lines 175-187).

But that partitioning simulator is a **different** code path:
- `SimulateFixedPrioritySched` is called only from `RTDA_Prob.h:29`
  (`travRTDACombinations` — the RTA response-time analysis over execution-time
  combinations) and from `tests/` (`testSP.cpp:99`, `testScheduleSimulate.cpp`).
- The **runtime** orchestrator — the one that produces the interval SP metrics
  the evaluation suite, the A/B, and the figures all read — is
  `FixedTaskPrioritySchedulingOrchestrator::SimulateInterval` and the CFS
  sibling `CFSSimulationOrchestrator::SimulateInterval`
  (`SimulationOrchestrator.cpp:684-749`), **neither** of which calls the
  partitioning helpers. Both build one shared `RunQueue(dag_tasks.tasks)` and
  release all tasks regardless of `processorId`.

So the bug is not "the project can't simulate partitioning" — it's "the
runtime simulator was written without the partitioning the task generator and
the RTA path both assume." The fix is to bring the runtime path in line with
the legacy `SimulatedFTP_SingleCore` partitioning, not to invent new
mechanics.

## Blast radius — what this does and does NOT invalidate

### Affected (numbers are simulated on one overloaded core, not two)
- **Every SP metric and miss rate** the evaluation suite reads
  (`comparison_summary.csv` → `Mean_SP_Metric`, `Mean_Miss_Rate`,
  `Important_Miss_Rate`, …) for **every arm**: `INCR`, `INCR_Reopt_{1,5,10,30,60}`,
  `BF`, `RM`, `CFS`, `INCR_NO_TL`, `INCR_WCET`. All schedulers run on the same
  wrong hardware model, so the bug is a **common-mode** distortion: relative
  comparisons are less wrong than absolute numbers, but any effect that
  interacts with overload (priority order, TL budgets, reopt period) is
  confounded with the starvation artifact.
- **E1 (overhead ≤ 5%)** — overhead is measured as scheduler ET, not CPU load,
  so E1's *verdict* is unaffected, but the workload the scheduler is operating
  on is the wrong one.
- **E3 (period-monotonicity)** — the SP(Reopt_X) sequence the gate reads was
  produced under single-core overload; the monotonicity verdict may or may not
  survive on the correct 2-core model. (Memory
  `eval-suite-per-n-verdicts.md`: E3 was swapped to `mean_sp_norm` and PASSES
  on the tasks4 run — that pass is on the **buggy** simulator.)
- **All P25 A/B conclusions** (P1.1 residual, P1.3 stale-binary, P1.4
  incumbent seed, the trial-and-error TL walk) — measured under the bug. The
  *direction* of effects that are about optimizer internals (incumbent
  carrying, seed policy) is probably robust because both arms share the
  overload, but magnitudes and any "sensitivity to priority order" framing are
  suspect.
- **P0.3 publication figures** — would be regenerated from the fixed
  simulator; current figures are on the wrong hardware model.

### NOT affected
- The optimizer itself (`OptimizeSP_TL_Incre`, `OptimizeSP_TL_BF`,
  `OptimizeIncre_w_TL`, `ReOptimizePeriodic`, the P0.5 incumbent redesign) —
  these operate on the DAG/task model and priority/TL vectors, not on the
  run-queue simulation. Their logic is unchanged; only the *evaluation* of
  their output (the SP the simulator reports) is distorted.
- `RTDA_Prob.h` response-time analysis — already uses the partitioning
  simulator (`SimulateFixedPrioritySched`), so it was always on the correct
  model.
- `testScheduleSimulate.cpp` / `testSP.cpp` — exercise the partitioning path,
  so they were always correct.
- The TL-init / seed-policy fixes (P0.5, P1.4) — algorithmically sound; their
  measured SP deltas are confounded with the bug but the fixes themselves are
  right.

## Open design decisions (settle BEFORE Step 1 — do NOT decide unilaterally)

Per `agent_coding_rules.md` ("Ask users if you're not certain about design
choices, don't make design decisions yourself"):

1. **Fix scope — partition the runtime simulator, or also revisit the task
   generator?** (OPEN.) The generator emits `processorId:0/1` and the legacy
   simulator partitions on it, so the *intended* model is 2-core. But the
   mean per-proc util is 1.160 (still overloaded on one core) — is that
   intended (a stressed 2-core system) or is the generator's CPU-util draw
   (`per_core_cpu_util`, per memory `env-dependent-tasks-ratio-refactor.md`)
   also miscalibrated? **Recommend: fix the simulator to partition first,
   measure, then decide whether the generator needs a separate pass** — don't
   conflate the two.
2. **Single shared `RunQueue` vs per-processor queues** (OPEN.) The legacy
   `SimulatedFTP_SingleCore` creates a fresh `RunQueue` per processor and
   loops each independently. Mirroring that in `SimulateInterval` means N
   independent run-queues stepped in lockstep (or a discrete-event merge).
   Alternatively, keep one `RunQueue` but gate `ReleaseJobs` on `processorId`
   and track N `processor_free_` flags — smaller diff, bigger divergence from
   the proven legacy path. **Recommend: mirror the legacy per-core
   `RunQueue`** — it's tested (`testScheduleSimulate.cpp`) and keeps one
   proven semantics.
3. **CFS sibling in the same pass?** (OPEN.)
   `CFSSimulationOrchestrator::SimulateInterval` has the identical bug. Fix
   both for consistency, or fix FTP first and confirm the CFS arm isn't
   load-bearing for any current gate/figure first? **Recommend: fix both in
   one pass** — they share the partitioning logic, and a half-fixed simulator
   is a confusing artifact.
4. **What does this do to the north-star gates and existing A/B results?**
   (OPEN, user decision.) After the fix, every recorded `comparison_summary.csv`
   is stale. Does the user want to (a) re-run everything (P0.4 suite + P25 A/B
   + P0.3 figures) on the fixed simulator, (b) keep the old numbers as a
   documented "single-core bug" baseline and add the fixed numbers alongside,
   or (c) something else? This determines whether P1.7 is just a code fix or
   also a re-run + re-papering effort.
5. **Priority/TL sensitivity re-frame** (OPEN.) The finding says the observed
   "extreme sensitivity to priority assignment order" is an overload artifact.
   Does the user want the paper / north-star doc
   (`agents/project_evaluation_northstar.md`) updated to reflect that the
   intended system is 2-core and the sensitivity story should be re-derived
   on the fixed model? **Recommend: yes, but only after the fixed re-run
   produces the new sensitivity numbers** — don't rewrite the narrative on
   speculation.

## Done when (implementation phase — NOT started)

- [ ] Design decisions 1–5 settled with the user.
- [ ] TDD: tests first in `tests/testScheduleSimulate.cpp` (or a new
      `testRunOrchestratorPartitioning.cpp`) — a 2-processor taskset where
      `processorId:0` and `processorId:1` jobs run in parallel (their
      intervals overlap in the schedule), and a low-priority task on the
      less-loaded core is NOT starved. Confirm the test FAILS on the current
      single-queue code (red before green).
- [ ] `FixedTaskPrioritySchedulingOrchestrator::SimulateInterval` partitioned
      per `processorId` (mirroring `SimulatedFTP_SingleCore`), OR the chosen
      D2 equivalent.
- [ ] `CFSSimulationOrchestrator::SimulateInterval` partitioned likewise (if
      D3 = fix both).
- [ ] `cmake --build build --target check.SP_OPT -j5` (DEBUG) green; existing
      `testScheduleSimulate.cpp` + `testSP.cpp` + `RunOrchestrator.cpp` tests
      green; 16/16 ctest green.
- [ ] Re-derive utilization on the fixed simulator: confirm single-core util
      is now ≤ the per-proc max (no taskset >100% on one core unless the
      generator genuinely over-loads one core — then surface that as a
      separate finding).
- [ ] User re-runs P0.4 suite + P25 A/B + P0.3 figures on the fixed simulator
      (per D4).
- [ ] `agents/overall_tasks.md` + top-level `agents/dev_log.md` updated;
      memory entry added (`cpu-partition-mismatch.md` or similar).
- [ ] `git add` staged; user reviews (no commit).

## Out of scope

- **No implementation now** — the user asked to "first focus on evaluating
  whether this is true before doing anything"; this file is the evaluation.
  Implementation waits for an explicit greenlight + the D1–D5 decisions.
- `git commit` — user's standing constraint (`git add` only).
- Re-running the A/B myself — user runs `run_end_to_end.sh`.
- Changing the task generator's CPU-util draw — that's a separate decision
  (D1) only reachable after the fixed simulator produces honest per-core util
  numbers.
- Re-litigating P1.1/P1.4/etc. conclusions — they were drawn under the bug;
  the fixed re-run will show which survive. Don't pre-judge.

## Reference docs

- [`P1_1_p25_residual_investigation/`](../P1_1_p25_residual_investigation/) —
  the P25 ET investigation whose "additional simulator finding" surfaced this.
- `agents/investigation/debug_runtime0704_incr.md` — the P25 debug record.
- Memory [`eval-suite-per-n-verdicts.md`](../../../) — E3 metric + per-N
  verdicts (all measured under this bug).
- Memory [`env-dependent-tasks-ratio-refactor.md`](../../../) — the
  `per_core_cpu_util` generator knob (D1 calibration question).

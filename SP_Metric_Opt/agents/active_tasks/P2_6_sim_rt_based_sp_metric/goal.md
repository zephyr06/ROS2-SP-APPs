# P2.6 — Sim-RT-based SP metric (report both analytical SP and true SP from simulated RTs)

**Priority:** P2 (features / analysis — NOT a correctness fix; the analytical SP
is sound). Deferred for later.
**Status:** FILED 2026-07-12 (planning only; **no implementation**). User
directive: "create a new task to add support from schedule simulation to report
both the analytical SP and the true SP from tasks' true RTAs. put it as a P2
task that we'll work on later."

## The gap

Every SP number the evaluation-suite gates read is **analytic**, not
simulation-derived. The runtime simulator computes SP via
`ObtainSP_TaskSet_And_TimeLimits` (`SP_Metric.cpp:82`) → `ObtainSP_TaskSet`
(line 53) → `ProbabilisticRTA_TaskSet(tasks)` (`RTA.cpp:100`), which builds a
probabilistic response-time distribution per task from the execution-time
distributions + priority structure, **already partitioned per `processorId`**
via `ExtractTaskSetPerProcessor` (`RTA.cpp:87`). The actual `RunQueue` schedule
— which produces real per-job response-time samples — is recorded in
`BaseSimulationOrchestrator::job_history_` (via `RecordFinishedJobs` /
`RecordFinishedJobsCFS`, `SimulationOrchestrator.cpp`) but is **never fed back
into the SP computation**. It is only written out as miss-rate / response-time
text exports (`ExportResults`, lines 140-244).

So today: the schedule and the SP metric live in two disconnected worlds. The
SP the optimizer minimizes and the gates read is a *prediction* (the RTA); the
*actual* schedule's RTs are recorded but not scored as SP.

This was surfaced by P1.7 (see `finished_tasks/P1_7_cpu_partition_mismatch/`):
while fixing the single-run-queue bug, the pivotal correction was that the SP
metric was **never distorted** by the bug — because SP doesn't read the
`RunQueue` at all. The user's reaction (verbatim): "for this simulation
experiment, ideally, we should use the RunQueue schedule rather than analytical
response time analysis. however, i think this is not the highest priority task.
if the task's DDL miss chance is based on run queue's actual schedule behavior,
i find it still accetpable. however, we should still re-run the evaluation. we
shall add a P1 task to make simulation to use actual response time rather than
analytical response time analysis for later." → filed 2026-07-12 as **P2.6**
(features/analysis, not correctness; deferred).

## Goal

Make the simulation report **BOTH**:

1. **Analytical SP** (current) — keep as the gate-facing metric. **No gate
   verdict moves.** The optimizer still minimizes this; Q1/Q2/Q3/E1/E3 still
   read it.
2. **True SP from simulated RTs** (new) — derive an SP value from the per-job
   response-time samples in `job_history_`, using the same `SP_Func` +
   `weights_node` / `thresholds_node` (and optionally `weights_path` /
   `thresholds_path`) the analytical path uses, so the two are directly
   comparable. Export as a new `Mean_SP_Metric_Sim` / `mean_sp_norm_sim` column
   in `comparison_summary.csv`, alongside the existing `Mean_SP_Metric`.

**Purpose:** cross-check the analytic RTA against the actual schedule;
quantify the analytic-vs-sim gap (is the RTA conservative? by how much?);
unblock a future decision on whether SP should come from the schedule rather
than the RTA. It is a *reported* cross-check, not a gate (D4).

## Why this is P2 (not P0/P1)

- The analytical SP is **sound** — `ProbabilisticRTA_TaskSet` partitions
  correctly and is the intended metric for the optimizer. This is not a bug.
- The sim RTs are now **trustworthy** (P1.7 fixed the single-core overload that
  made them garbage). Without P1.7 this task would be meaningless; with P1.7
  done, it's a clean cross-check. **Prerequisite: P1.7 DONE.**
- It's analysis richness (a second SP column to compare against the first), not
  a correctness or publication-blocker issue. Per the project priority rules,
  P2 = "should-do hygiene"; the user explicitly said "put it as a P2 task that
  we'll work on later."

## The lever (most of the infrastructure already exists)

- `ExportResults` (`SimulationOrchestrator.cpp:140-244`) **already** groups
  `job_history_` by `taskId` and computes `response_time = finishTime -
  releaseTime` per job (lines 162-192). The RT samples exist; they're just
  written to text, not scored.
- `ObtainSP_DAG_From_Dists` (`SP_Metric.cpp:129-149`) **already** takes
  pre-built `FiniteDist` node-RT distributions + path-latency distributions and
  runs them through `ObtainSP` → `SP_Func(miss_chance, threshold) * weight`
  (lines 136-138, 143-146). This is exactly the shape a sim-RT SP would reuse.
- The interval SP is computed at `SimulationOrchestrator.cpp:553` + `:803`
  (`interval_sp_metrics_.push_back(ObtainSP_TaskSet_And_TimeLimits(...))`).

So the core work is: **build an empirical `FiniteDist` per task from its
`job_history_` RT samples** (a histogram of `finishTime - releaseTime` over the
run's jobs for that task), then feed it through the existing
`ObtainSP` / `ObtainSP_DAG_From_Dists` path. The analytical SP uses the RTA
distribution; the sim SP uses the empirical distribution; same `SP_Func` +
weights + thresholds downstream → apples-to-apples.

## Open design decisions (settle with the user BEFORE coding — per `agent_coding_rules.md`, do NOT decide unilaterally)

1. **D1 — RT samples → SP value.** How to turn per-job RT samples into an SP
   value. (a) Build an empirical `FiniteDist` / histogram per task from the RT
   samples and feed it through the SAME `SP_Func` + `weights_node` /
   `thresholds_node` path (closest apples-to-apples with the analytical SP;
   reuses `ObtainSP_DAG_From_Dists`). (b) A simpler empirical DDL-miss-fraction
   weighted sum (`missed_jobs / total_jobs` per task × weight — cheaper, but
   NOT directly comparable to the analytical SP because the analytical path
   integrates a distribution, not a point fraction). **Recommend (a)** — the
   whole point is comparability; (b) is a different metric wearing the same
   name. Sub-question for (a): histogram granularity / bin width (the RTA uses
   `GlobalVariables::Granularity` — match it for comparability).
2. **D2 — Chain / path SP.** The analytical `ObtainSP_DAG` (`SP_Metric.cpp:89`)
   ALSO sums `GetRTDA_Dist_AllChains<ObjReactionTime>` path-latency terms over
   `chains_deadlines_` with `thresholds_path` / `weights_path` (lines 100-107).
   Does the simulation produce path-latency samples (a chain's end-to-end
   reaction time across the run), or is node-level sim SP enough for the
   cross-check? If the sim only has per-task RTs, P2.6 reports the **node-level
   portion only** and the path portion stays analytical — note that explicitly
   in the export so the two columns aren't silently misread. (The `job_history_`
   is per-job per-task; reconstructing a chain's reaction time from it requires
   matching jobs across the chain's tasks by release window — feasible but
   extra work; defer unless the user wants the path cross-check too.)
3. **D3 — Where the column lands.** C++ `BaseSimulationOrchestrator::ExportResults`
   writes the per-run CSV; `aggregate_across_tasks.py` + `utils.py` read it.
   Mirror the existing `Mean_SP_Metric` plumbing — add `Mean_SP_Metric_Sim`
   next to it at write time + read time. The per-interval sim SP would be
   computed alongside `interval_sp_metrics_` (currently the
   `ObtainSP_TaskSet_And_TimeLimits` results at `:553` + `:803`). Decide
   whether the sim SP is per-interval (like `interval_sp_metrics_`) or
   whole-run-only (like the `miss_rate_summary.txt` aggregate). Per-interval is
   more useful for the cross-check (shows where analytic vs sim diverge over
   time) but costs more plumbing.
4. **D4 — Gated or cross-check?** Whether any gate eventually switches to the
   sim SP, or it stays a reported-but-not-gated cross-check. **Default:
   cross-check only** — no gate reads `mean_sp_norm_sim` initially. Switching a
   gate to sim SP is a separate decision (a P0/P1 task, with its own A/B) made
   only after the cross-check shows the two agree closely enough (or
   characterizes the disagreement). Do NOT silently retarget a gate.

## Done when (implementation phase — NOT started)

- [ ] Design decisions D1–D4 settled with the user.
- [ ] TDD: a test in `tests/testScheduleSimulate.cpp` (or a new
      `testSimRTSPMetric.cpp`) — a small taskset where the analytical SP and
      the sim-RT SP can both be computed by hand, asserting the new sim-SP
      column is produced and is in the expected range (the sim SP for a
      non-overloaded taskset should be close to — typically ≥ — the analytical
      SP, since the RTA is conservative). Red before green.
- [ ] New `ObtainSP_*_FromRTSamples` (or reuse `ObtainSP_DAG_From_Dists` with
      empirical `FiniteDist`s built from `job_history_`) per D1.
- [ ] `ExportResults` writes `Mean_SP_Metric_Sim` (per D3); the per-interval
      sim SP computed alongside `interval_sp_metrics_` if D3 = per-interval.
- [ ] Python aggregation (`aggregate_across_tasks.py` + `utils.py`) reads +
      reports the new column (per D3); no gate reads it (D4).
- [ ] `cmake --build build --target check.SP_OPT -j5` (DEBUG) green; 16/16
      ctest green; `tests/python/` green.
- [ ] Cross-check on a real run: report the analytic-vs-sim SP gap in
      `dev_log.md` (is the RTA conservative? by how much?).
- [ ] `agents/overall_tasks.md` + top-level `agents/dev_log.md` updated;
      memory `sim-rt-based-sp-metric.md` updated from "filed" to "done".
- [ ] `git add` staged; user reviews (no commit).

## Out of scope

- **No implementation now** — filed for later per the user's directive.
- Switching any gate to read the sim SP (that's a future P0/P1 decision, D4 —
  only after the cross-check characterizes the gap).
- Replacing the analytical SP — it stays the gate-facing + optimizer-facing
  metric. P2.6 ADDS a column, doesn't swap one.
- `git commit` — user's standing constraint (`git add` only).
- Running the A/B myself — user runs `run_simulation_and_plot_figures.sh`.
- The P1.7 D1 generator `per_core_cpu_util` re-calibration — separate decision,
  depends on the fixed sim's honest per-core util, not on this task.

## Reference docs

- [`finished_tasks/P1_7_cpu_partition_mismatch/`](../../finished_tasks/P1_7_cpu_partition_mismatch/) —
  the prerequisite fix (per-`processorId` RunQueues → honest RT samples); the
  pivotal correction (SP was never distorted) that surfaced this task.
- `sources/Safety_Performance_Metric/SP_Metric.cpp:53-149` — `ObtainSP_TaskSet`,
  `ObtainSP_DAG`, `ObtainSP_DAG_From_Dists` (the shapes to mirror / reuse).
- `sources/Safety_Performance_Metric/RTA.cpp:87-125` — the analytical
  partitioned RTA path (what produces the analytical SP).
- `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp:140-244` —
  `ExportResults` (the RT samples already grouped per task; the plumbing site).
- Memory [`sim-rt-based-sp-metric.md`](../../../) (this task's memory entry).
- Memory [`cpu-partition-mismatch.md`](../../../) (the prerequisite).
- Memory [`eval-suite-per-n-verdicts.md`](../../../) (the gates that read the
  analytical SP — stay on analytical per D4).

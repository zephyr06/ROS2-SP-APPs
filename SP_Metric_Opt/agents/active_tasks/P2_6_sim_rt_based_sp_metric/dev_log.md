# P2.6 — Sim-RT-based SP metric — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-12

- **Task FILED (planning only; NO implementation).** User directive (verbatim):
  "create a new task to add support from schedule simulation to report both the
  analytical SP and the true SP from tasks' true RTAs. put it as a P2 task that
  we'll work on later."
- **Surfaced by P1.7** (`finished_tasks/P1_7_cpu_partition_mismatch/`). While
  fixing the single-run-queue overload, the pivotal correction was that the SP
  metric was **never distorted** by the bug — because SP doesn't read the
  `RunQueue` at all. The user's reaction: the analytic SP is acceptable for now
  (DDL-miss chance based on the run-queue's actual schedule behavior is still
  acceptable), but file a later task to make the simulation report a true SP
  derived from the actual schedule's RTs alongside the analytical one. → filed
  as **P2.6** (features/analysis, NOT correctness; deferred).
- **The gap, traced:** every SP number the eval-suite gates (Q1/Q2/Q3/E1/E3)
  read is **analytic** — `SimulateInterval` → `ObtainSP_TaskSet_And_TimeLimits`
  (`SP_Metric.cpp:82`) → `ObtainSP_TaskSet` (line 53) → `ProbabilisticRTA_TaskSet`
  (`RTA.cpp:100`), already partitioned per `processorId` via
  `ExtractTaskSetPerProcessor` (`RTA.cpp:87`). The `RunQueue`'s actual per-job RT
  samples are recorded in `BaseSimulationOrchestrator::job_history_` (via
  `RecordFinishedJobs`/`RecordFinishedJobsCFS`) but are **never fed back into the
  SP computation** — only written out as miss-rate/response-time text exports
  (`ExportResults`, `SimulationOrchestrator.cpp:140-244`). The schedule and the SP
  metric live in two disconnected worlds.
- **Goal:** make the simulation report BOTH — keep the analytical SP as the
  gate-facing metric (no gate verdict moves; the optimizer still minimizes it)
  AND add a true SP derived from the per-job RT samples in `job_history_`, using
  the same `SP_Func` + `weights_node`/`thresholds_node` so the two are directly
  comparable. New `Mean_SP_Metric_Sim` / `mean_sp_norm_sim` column in
  `comparison_summary.csv`, alongside `Mean_SP_Metric`. Purpose: cross-check the
  analytic RTA against the actual schedule; quantify the analytic-vs-sim gap;
  unblock a future decision on whether SP should come from the schedule.
- **Most of the infrastructure already exists** (the lever):
  `ExportResults` already groups `job_history_` by `taskId` and computes
  `response_time = finishTime - releaseTime` per job (`:162-192`); the RT samples
  exist, they're just written to text, not scored. `ObtainSP_DAG_From_Dists`
  (`SP_Metric.cpp:129-149`) already takes pre-built `FiniteDist` node-RT
  distributions and runs them through `ObtainSP` → `SP_Func(miss_chance, threshold)
  * weight`. So the core work = build an empirical `FiniteDist` per task from its
  RT samples, then feed it through the existing `ObtainSP` path.
- **Prerequisite DONE** — P1.7 (per-`processorId` RunQueues → honest RT samples;
  without it the sim RTs were the distorted single-core-overload numbers). P1.7
  code committed as `7032e483`.
- **Priority = P2** (features/analysis, not correctness — the analytic SP is
  sound; the user explicitly said "put it as a P2 task that we'll work on
  later"). Deferred.
- **Open design decisions flagged for the user (settle BEFORE coding, per
  `agent_coding_rules.md` — do NOT decide unilaterally):**
  - D1 — RT samples → SP value: (a) empirical `FiniteDist` per task fed through
    the same `SP_Func`+weights/thresholds path (recommend — apples-to-apples;
    reuse `ObtainSP_DAG_From_Dists`), or (b) empirical miss-fraction weighted
    sum (cheaper, NOT directly comparable). Sub: histogram bin width — match
    `GlobalVariables::Granularity` for comparability.
  - D2 — Chain/path SP: the analytic `ObtainSP_DAG` also sums path-latency
    terms; does the sim produce path-latency samples, or is node-level sim SP
    enough (path portion stays analytic, noted explicitly)?
  - D3 — Column placement: mirror `Mean_SP_Metric` in `ExportResults` +
    `aggregate_across_tasks.py` + `utils.py` as `Mean_SP_Metric_Sim`; per-interval
    (alongside `interval_sp_metrics_` at `:553`+`:803`) vs whole-run-only.
  - D4 — Gated or cross-check: default **cross-check only** (no gate reads
    `mean_sp_norm_sim`); switching a gate to sim SP is a separate future P0/P1
    decision.
- **Registries updated this filing:** `goal.md` + `tasks.md` (this folder),
  `overall_tasks.md` (P2.6 row + suggested-order entry), top-level `dev_log.md`
  (P2.6 milestone), `finished_tasks/summary.md` (P1.7→P2.6 residual pointer),
  memory `sim-rt-based-sp-metric.md` + `MEMORY.md` pointer.
- **Not started.** Next action is the user's: greenlight + settle D1–D4 when P2
  work resumes. Standing constraints: no implementation until greenlit; no
  `git commit` (`git add` only); no A/B run by me (user runs
  `run_end_to_end.sh`); no gate reads the sim SP without an explicit user
  decision (D4).

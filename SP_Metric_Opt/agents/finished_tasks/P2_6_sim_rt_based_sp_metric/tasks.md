# P2.6 — Tasks (working checklist)

> See `goal.md` for the gap, the goal, the lever, and the open design
> decisions. **FILED 2026-07-12 (planning only; NO implementation)** per the
> user's directive ("put it as a P2 task that we'll work on later"). Steps
> below are the proposed execution plan for when the user greenlights; none are
> started. One review-and-commit cycle per `agent_coding_rules.md`. TDD: tests
> first.

## Step 0 — Settle design decisions with the user (NOT STARTED)

- [ ] D1 — RT samples → SP value: (a) empirical `FiniteDist` per task fed
      through the same `SP_Func` + `weights_node`/`thresholds_node` path
      (recommend — apples-to-apples with the analytical SP; reuse
      `ObtainSP_DAG_From_Dists`), or (b) empirical miss-fraction weighted sum
      (cheaper, NOT directly comparable). If (a): match
      `GlobalVariables::Granularity` for the histogram bin width.
- [ ] D2 — Chain/path SP: does the sim produce path-latency samples, or is
      node-level sim SP enough? If only per-task RTs → report node-level only,
      note the path portion stays analytical (don't silently misread the two
      columns). Reconstructing chain RTs from `job_history_` = extra work;
      defer unless the user wants the path cross-check.
- [ ] D3 — Column placement: mirror `Mean_SP_Metric` in `ExportResults` +
      `aggregate_across_tasks.py` + `utils.py` as `Mean_SP_Metric_Sim`. Decide
      per-interval (alongside `interval_sp_metrics_` at `:553` + `:803`) vs
      whole-run-only.
- [ ] D4 — Gated or cross-check: default **cross-check only** (no gate reads
      `mean_sp_norm_sim`). Switching a gate to sim SP is a separate future
      P0/P1 decision.

## Step 1 — TDD: tests first (NOT STARTED)

- [ ] `tests/testScheduleSimulate.cpp` (or new `testSimRTSPMetric.cpp`) — a
      small taskset where the analytical SP and the sim-RT SP can both be
      computed by hand. Assert the new sim-SP column is produced and is in the
      expected range (sim SP ≥ analytical SP for a non-overloaded taskset, the
      RTA being conservative).
- [ ] Confirm the test FAILS before the new code exists (red before green).

## Step 2 — Compute the sim-RT SP (NOT STARTED)

- [ ] New `ObtainSP_*_FromRTSamples` helper, OR reuse `ObtainSP_DAG_From_Dists`
      (`SP_Metric.cpp:129`) with empirical `FiniteDist`s built per task from
      `job_history_` RT samples (`response_time = finishTime - releaseTime`,
      already computed in `ExportResults:180,217,243`). Per D1.
- [ ] If D2 = node-level only: document that the path portion is analytical
      (the new column is node-level sim SP, not directly the DAG-wide SP).
- [ ] Compute per-interval sim SP alongside `interval_sp_metrics_`
      (`SimulationOrchestrator.cpp:553` + `:803`) if D3 = per-interval; else
      whole-run aggregate in `ExportResults`.

## Step 3 — Export + aggregate the column (NOT STARTED)

- [ ] `BaseSimulationOrchestrator::ExportResults`
      (`SimulationOrchestrator.cpp`) — write `Mean_SP_Metric_Sim` next to
      `Mean_SP_Metric` in the per-run CSV. Per D3.
- [ ] `aggregate_across_tasks.py` + `utils.py` — read + report the new column.
- [ ] NO gate reads it (D4 — cross-check only). Confirm
      `evaluation_suite.py` is untouched.

## Step 4 — Verify + cross-check (NOT STARTED)

- [ ] `cmake --build build --target check.SP_OPT -j5` (DEBUG) — 16/16 ctest
      green; `tests/python/` green.
- [ ] Cross-check on a real run (user runs `run_simulation_and_plot_figures.sh`): report the
      analytic-vs-sim SP gap in `dev_log.md` (is the RTA conservative? by how
      much? does the gap vary by arm / by N?).

## Step 5 — Docs (NOT STARTED)

- [ ] `agents/overall_tasks.md` — mark P2.6 row DONE.
- [ ] Top-level `agents/dev_log.md` — append the P2.6 milestone.
- [ ] Memory `sim-rt-based-sp-metric.md` — update from "filed" to "done" +
      record the cross-check gap; MEMORY.md pointer updated.
- [ ] `git add` the P2.6 unit; hand to user for review (no commit).

## Standing constraints

- **No implementation until the user greenlights** (this task is filed for
  later; the 2026-07-12 work created the plan, did not code).
- No `git commit` (user's task; `git add` only).
- No running the A/B myself (user runs `run_simulation_and_plot_figures.sh`).
- No gate reads the sim SP without an explicit user decision (D4) — the
  analytical SP stays the gate-facing + optimizer-facing metric.
- Don't conflate this with P1.7 D1 (generator `per_core_cpu_util`
  re-calibration) — separate decision.

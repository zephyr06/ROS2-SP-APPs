# P0.4 — Project Evaluation Suite (north-star integration test)

**Priority:** P0 (gives the project a single, reproducible tuning target)
**Status:** not started

## Goal

Build a **project evaluation suite** — a slow (~20 min total is fine),
deterministic integration test that measures the *ultimate impact* of any code
or algorithm change against the north-star requirements
(`agents/project_evaluation_northstar.md`). It is the single green/red dashboard
we tune for: every change re-runs the suite, regressions show as red.

The suite runs the optimizers on **simulated task sets of N = 4, 6, 8** and
reports two families of metrics vs. the north-star red-flag lines:

1. **Optimization quality** — mean SP (normalized by `ideal_SP`, per the P12
   fix) for each scheduler.
2. **Scheduler overhead** — per-interval execution time / interval_period
   (overhead %).

### North-star gates (the tuning target)

From `project_evaluation_northstar.md`:

| # | Requirement | Red-flag line | Ideal |
|---|-------------|---------------|-------|
| Q1 | Small N: BF ≥ INCR/SCRATCH, but gap is low | gap ≤ 30% | — |
| Q2 | Large N: INCR/SCRATCH outperform BF (BF time-out) | INCR/SCRATCH ≥ BF | — |
| Q3 | INCR and SCRATCH outperform all other baselines | INCR/SCRATCH ≥ every baseline | — |
| E1 | Scheduler overhead (per-interval ET / interval) is low, at least N=10 (ideally N=16) | ≤ 5% | ≤ 1% |
| E2 | INCR cannot run slower than SCRATCH | INCR_ET ≤ SCRATCH_ET | — |

The suite emits a PASS/FAIL verdict per gate → the "very clear goal to tune for."

## Why this matters

- **Clear tuning target.** Today, "is this change an improvement?" requires
  reading several figures and remembering the thresholds. The suite collapses
  that to a fixed gate table the north-star already specifies.
- **Regression safety.** It is the integration test that catches when a
  refactor/algorithm tweak silently regresses SP quality or ET overhead — the
  two things the paper actually claims.
- **Decouples "works" from "publishable."** `ctest`/`pytest` prove correctness
  of units; this suite proves the *end-to-end* north-star claims hold. It is
  the bridge between "tests green" and "paper claims hold."

## Approach

1. **Fix the task sets.** Pin deterministic seeds for N = 4, 6, 8 (one or a few
   tasksets per N) so every suite run compares the *same* problem instances.
   Reuse the existing taskset family/config; do not invent a new generator.
2. **Reuse the pipeline.** Drive simulations through the existing
   `scripts/run_end_to_end.sh` / `RunOrchestrator` + `aggregate_across_tasks.py`
   rather than a parallel harness. The suite is a *scoping + reporting* layer
   on top, not a new simulator.
3. **Collect metrics.** For each scheduler (BF, INCR, SCRATCH + baselines) on
   each N: mean normalized SP, and per-interval ET / interval_period.
4. **Gate-check.** Evaluate the 5 north-star gates (Q1–Q3, E1, E2) and emit a
   PASS/FAIL table + a machine-readable JSON for trend tracking.
5. **Wire as an integration test.** A single entry script (e.g.
   `scripts/run_evaluation_suite.sh`) + a config (e.g.
   `simulation_experiments/configs/evaluation_suite_config.json`) that anyone
   can run after a change to see the verdict.

### Open questions to resolve when starting (do NOT decide now)

- **N scope vs. E1.** North-star E1 wants overhead measured "at least N=10,
  ideally N=16," but the user scoped the suite to N = 4, 6, 8. Decide at
  start-time whether to (a) keep N=4/6/8 for quality and add an N=10/16
  overhead-only probe, or (b) extend the suite's N range. Surface the tradeoff;
  don't silently pick one.
- **Which baselines count for Q3.** "All other baselines" = the ablation group
  (INCR_NO_TL, INCR_WCET) + CFS + RM_FAST/RM_SLOW? Confirm the exact set.
- **Seed pinning granularity.** One seed per N, or a small fixed multi-seed
  bundle (to reduce single-instance noise) while keeping the suite ~20 min?

## Files (candidates — to be created when the task starts)

- `scripts/run_evaluation_suite.sh` — single entry point (new).
- `simulation_experiments/configs/evaluation_suite_config.json` — fixed N=4/6/8
  tasksets + seeds, derived from the prod config (new).
- `simulation_experiments/evaluation_suite.py` (or extend
  `aggregate_across_tasks.py`) — collect SP + ET metrics, evaluate the 5 gates,
  emit PASS/FAIL table + JSON (new).
- Reuses: `scripts/run_end_to_end.sh`, `release/tests/RunOrchestrator`,
  `simulation_experiments/aggregate_across_tasks.py`,
  `simulation_experiments/compare_optimizers.py`.

## Done when

- A single command runs the suite end-to-end on N = 4, 6, 8 and finishes in a
  bounded time (~20 min), deterministically (same seeds → same instances).
- Output is a PASS/FAIL table for all 5 north-star gates (Q1–Q3, E1, E2) plus
  a machine-readable JSON of the metric values.
- The suite runs green on the current clean baseline (so we have a known-good
  reference to diff future changes against).
- A one-paragraph "how to run + how to read the verdict" note is added to the
  top-level `dev_log.md` (milestone) and this folder's `dev_log.md`.

## Out of scope

- **Do not start implementation now** — this task is filed only; per user,
  "don't start working on it, just add this as an active task."
- New taskset families or a new generator — reuse the existing one.
- Per-figure polish (that's P0.3). The suite reports gate verdicts, not paper
  figures (though it can share the metric-collection code).
- Performance optimization of the suite itself (it is fine that it is slow).

## Relationship to other tasks

- Builds on the **P0.1** clean baseline (committed).
- Shares infrastructure with **P0.3** (prod figure run) — same e2e pipeline,
  same metric collection — but adds the north-star gate-checking layer.
  Complementary: P0.3 = publication figures; P0.4 = integration test / tuning
  target. Not blocked by P0.3.

# P3.6 — New Baseline: Pure Incremental (RM-Fast Bootstrap, No Periodic Reopt)

**Priority:** P3 (measurement baseline; not a correctness fix)
**Status:** IMPLEMENTATION IN PROGRESS 2026-07-26 (plan refreshed against current
code after a staleness audit; TDD next). **Filed** 2026-07-11 (planning only).

## The arm (user's directive, restated 2026-07-26)

> "the new baseline that i want to add: initial solution is rm fast, all the
> follow-up optimization is via incremental optimization, no re-optimization like
> the INCR_Reopt_10 does."

A new scheduler arm: **pure incremental**.
- **Interval 0:** bootstrap the incumbent from RM-fast (RM priorities +
  smallest-TL option). **No from-scratch descent.**
- **Intervals 1+:** `OptimizeIncre_w_TL` (warm-started from the carried
  incumbent) every interval.
- **Never** `ReOptimizePeriodic` (contrast with `INCR_Reopt_10`, which reopts
  every 10th interval).
- Uses the **persistent** `incr_optimizer_` (carries the incumbent across
  intervals, like `INCR`) — NOT a fresh optimizer each interval.

## Purpose / A/B read

Contrasts with `INCR_Reopt_10` (the production incremental arm, which runs a
from-scratch `ReOptimizePeriodic` every 10 intervals). Question: **does periodic
reopt earn its cost?** If `INCR_NO_REOPT ≈ INCR_Reopt_10` in SP at lower scheduler
ET, the periodic descent isn't earning its cost — feeds P1.2 (reopt incumbent
degradation). Paper-grade baseline, **NOT an E3 gate** (E3 stays on plain `INCR`).

> **Staleness note 2026-07-26:** the original 2026-07-11 plan framed this as an
> incr-vs-`INCR_SCRATCH` A/B. `INCR_SCRATCH` was removed by P2.5 — no live
> code/config remains (only historical CSVs + a test note). The user's 2026-07-26
> restatement re-targets the contrast at `INCR_Reopt_10` (the actual current
> production arm). The arm's *behavior* is unchanged from 2026-07-11; only the
> named contrast shifted.

## Current-code integration (audited 2026-07-26; replaces stale 2026-07-11 refs)

- `Optimize_w_TL_ScratchOrIncre` (`OptimizeSP_TL_Incre.cpp:576`) routes
  `count % period == 0` → `ReOptimizePeriodic` (interval 0 = bootstrap + descent),
  else `OptimizeIncre_w_TL`. The new arm CANNOT reuse this — it always descends
  at interval 0.
- The interval-0 RM-fast seed lives inside `ResetIncumbentBaseline(true)` `else`
  branch (`:733-742`): `SmallestTimeLimitVec()` + `RateMonotonicPriorityVec()` →
  `SeedStateFromIncumbent` → `CommitIncumbent`. At interval 0 (fresh persistent
  optimizer, `IfInitialized()` false) this fires automatically. **This is the
  seed-only bootstrap the new arm wants — minus the descent `ReOptimizePeriodic`
  runs afterwards.**
- Orchestrator construction (`SimulationOrchestrator.cpp:298-300`) + dispatch
  (`:324-345`): `INCR`+period-variants / `INCR_NO_TL` / `INCR_WCET` build + use
  the persistent `incr_optimizer_` via `Optimize_w_TL_ScratchOrIncre`. The new
  arm adds a parallel branch.

> **Staleness corrected:** the 2026-07-11 plan named
> `PerformCoordinateDescentForTaskConfigOpt` as the descent to skip — that was
> deleted by P2.16 Stage A1 (`6d068ec9`). `ReOptimizePeriodic` now calls
> `RunIntervalDescent(..., IntervalDescentMode::Reopt)` directly (P2.11 merged
> the two descent bodies into one shared `RunIntervalDescent`). The seed-only
> bootstrap is cleanly separable: call `ResetIncumbentBaseline(true)`, skip
> `RunIntervalDescent`'s walk tail.

## Design decisions (settled 2026-07-26; override at review)

- **D1 — Mode name = `INCR_NO_REOPT`.** Parallels the existing `INCR_NO_TL`
  convention ("no X"); mirrors the user's own phrasing ("no re-optimization");
  forms a clean A/B naming pair with `INCR_Reopt_10` ("reopt every 10" vs "no
  reopt"). No collision with existing modes.
- **D2 — Dedicated `scheduler_mode_` branch.** Shape unchanged from 2026-07-11;
  the "mirror `INCR_SCRATCH`" rationale is void (P2.5 removed it) but a named
  branch is still the right shape (design rules: don't make things optional).
- **D3 — New optimizer entry point `OptimizePureIncremental(dag_tasks_update,
  beam_search_width)` + seed-only helper `BootstrapIncumbentFromRMFast(
  dag_tasks_update)`.** Entry point mirrors `Optimize_w_TL_ScratchOrIncre`'s
  shape (one call, internal count-based routing, advances
  `reoptimization_interval_count_`, returns `opt_pa_`): `count==0` →
  `BootstrapIncumbentFromRMFast` (set `dag_tasks_` + `ApplyWCETAblationIfRequired`
  + `time_limit_option_for_each_task_=RecordTimeLimitOptions` +
  `ResetIncumbentBaseline(true)`, **NO descent**); `count>0` → `OptimizeIncre_w_TL`.
  Keeps counter advancement inside the optimizer (orchestrator just calls the
  entry point).
- **D4 — Config = `paper_simulation_config.json`, `ablation_scheduler_list`
  (both modes).** That's where the `INCR_Reopt_X` ablation arms live, so the A/B
  reads from one `comparison_summary.csv`. Placed in **ablation** (not
  `main_scheduler_list`) so the main figure's 5 production arms stay unchanged;
  the "does reopt earn its cost?" contrast lands in the ablation figure, where
  it belongs. (`p25_period_ab_config.json`, the 2026-07-11 recommended target,
  was removed by P2.8's config consolidation.)
- **D5 — Paper-grade baseline, NOT an E3 gate.** (Settled 2026-07-11; unchanged.)

## Done when

- [ ] TDD tests in `tests/testIncreOpt_w_TL.cpp`: interval 0 produces the RM-fast
      incumbent with zero descent evals; interval 1+ takes the incremental path;
      no `ReOptimizePeriodic` across a multi-interval run; incumbent carried
      across intervals (persistent optimizer).
- [ ] `OptimizePureIncremental` + `BootstrapIncumbentFromRMFast` in
      `OptimizeSP_TL_Incre.{h,cpp}`.
- [ ] `INCR_NO_REOPT` branch in `SimulationOrchestrator.cpp` construction +
      dispatch; `RunOrchestrator.cpp` Usage.
- [ ] `paper_simulation_config.json` updated (both modes + comment).
- [ ] `cmake --build build_test --target check.SP_OPT -j5 --clean-first` green
      (17/17 ctest); SP bit-identical for existing arms.
- [ ] `git add`; user reviews + rebuilds `release/` + re-runs the A/B.

## Out of scope

- `git commit` (user's standing constraint; `git add` only).
- Running the A/B myself (user runs the sim).
- Changing the interval-0 RM-fast seed itself (`IfInitialized()`-gated,
  irreducible — no prior exists at interval 0 for any policy).
- Re-litigating P1.4 seed policy (inherited unchanged).

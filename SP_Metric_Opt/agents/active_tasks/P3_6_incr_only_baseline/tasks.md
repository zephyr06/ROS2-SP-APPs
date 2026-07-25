# P1.6 — Tasks (working checklist)

> See `goal.md` for the finding, the reasoning, and the open design decisions.
> **PLANNING ONLY — NO implementation yet** (user directive: "only add this
> active task without implementation"). Steps below are the proposed execution
> plan for when the user greenlights implementation; none are started.
> One review-and-commit cycle per `agent_coding_rules.md`. TDD: tests first.

## Step 0 — Settle design decisions with the user (PARTIALLY DONE)

- [x] D2 — Implementation shape: **(a) new `scheduler_mode_` branch** (SETTLED
      2026-07-11 — user: "implement it similar to `INCR_SCRATCH`"). NOT a flag.
- [x] D5 — Paper-grade baseline; **A/B probe, NOT an E3 gate** (SETTLED
      2026-07-11 — user: "likely a baseline to add to paper"). E3 stays on
      plain `INCR`.
- [ ] D1 — Mode-string name (recommend `INCR_PURE`; no collision with existing
      modes). Paper-bound, so pick a name that reads well in the baseline table.
- [ ] D3 — Interval-0 mechanics: new `BootstrapIncumbentFromRMFast()` method
      (recommend) vs short-circuiting `ReOptimizePeriodic`.
- [ ] D4 — Config placement: add to `p25_period_ab_config.json` both modes
      (recommend) vs separate A/B config; confirm whether it also goes in the
      P0.3 prod-mode list.

## Step 1 — TDD: tests first (NOT STARTED)

- [ ] `tests/testIncreOpt_w_TL.cpp` — interval 0 produces the RM-fast incumbent
      (RM priorities via `RateMonotonicPriorityVec` + smallest TL via
      `SmallestTimeLimitVec`) with **zero** descent evals
      (`eval_count_` unchanged across the bootstrap).
- [ ] Interval 1+ takes the incremental path (`OptimizeIncre_w_TL`,
      `from_scratch=false`); no `ReOptimizePeriodic` call across a multi-
      interval run.
- [ ] The incumbent is carried (interval N's adopted solution seeds interval
      N+1 via `ReconstructTimeLimitVecFromResOpt`, the P1.4 seed — unchanged).
      **Critical:** the arm uses the **persistent** `incr_optimizer_`, NOT a
      fresh `scratch_opt` each interval (that is `INCR_SCRATCH`'s amnesia — the
      exact thing this arm contrasts).
- [ ] Confirm tests FAIL on the current code (the mode/branch doesn't exist
      yet) — red before green.

## Step 2 — Wire the new arm (NOT STARTED)

- [ ] `sources/Optimization/OptimizeSP_TL_Incre.h` — declare
      `BootstrapIncumbentFromRMFast()` (or chosen equivalent per D3).
- [ ] `sources/Optimization/OptimizeSP_TL_Incre.cpp` — implement it: run the
      `ResetIncumbentBaseline(true)` interval-0 seed step (the `else` branch:
      `SmallestTimeLimitVec` + `RateMonotonicPriorityVec` →
      `SeedStateFromIncumbent` → `CommitIncumbent`) and **skip**
      `PerformCoordinateDescentForTaskConfigOpt`. Leave `ReOptimizePeriodic`
      untouched.
- [ ] `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp`:
      - Add the new mode to the pre-loop construction condition (`:293`) so the
        **persistent** `incr_optimizer_` is built (same as `INCR`, NOT the
        fresh-`scratch_opt` lifetime of `INCR_SCRATCH`).
      - Add the dispatch branch in `DeterminePrioritiesAndBudgets` (`:309`),
        mirroring `INCR_SCRATCH`'s branch *shape*: interval 0 →
        `BootstrapIncumbentFromRMFast()` on the persistent `incr_optimizer_`;
        interval 1+ → `incr_optimizer_.OptimizeIncre_w_TL(...)`. (Track the
        interval index the way the loop already does, or expose a counter on
        the optimizer.)
- [ ] `tests/RunOrchestrator.cpp` — add the new mode to `--help` Usage; no
      period-suffix parsing needed (the arm has no period).

## Step 3 — Config + docs (NOT STARTED)

- [ ] `simulation_experiments/configs/p25_period_ab_config.json` — add the arm
      to both `test_mode` and `prod_mode` `main_scheduler_list`; update the
      `_comment` to describe it (pure incremental, RM-fast bootstrap, no
      periodic reopt; the incr-vs-scratch A/B baseline). If D4 confirms a
      separate prod list, add there too.
- [ ] In-source comments at the new branch + method (paper-grade — polish the
      wording; it will be read alongside `INCR_SCRATCH`'s comment).
- [ ] Memory: add a pointer entry (new memory file or extend
      `reopt-tl-init-adopted-arms.md`) once the arm is implemented + measured.

## Step 4 — Build + test + index (NOT STARTED)

- [ ] `cmake --build build --target check.SP_OPT -j5` (DEBUG) — 16/16 ctest
      green; `testIncreOpt_w_TL` green (existing 48 + the new tests).
- [ ] `agents/overall_tasks.md` — add the P1.6 row to the Active tasks table +
      the Suggested execution order.
- [ ] Top-level `agents/dev_log.md` — append the P1.6 milestone (planning →
      implemented).
- [ ] `git add` the P1.6 unit; hand to user for review (no commit). User
      rebuilds `release/` + re-runs the A/B.

## Standing constraints

- **No implementation until the user greenlights** (this task is filed only;
  the 2026-07-11 follow-up updated plans only — still no implementation).
- No `git commit` (user's task; `git add` only).
- No running the A/B myself (user runs `run_simulation_and_plot_figures.sh`).
- Inherits P1.4's incumbent seed unchanged — this arm is a dispatch/bootstrap
  change, not a seed-policy change.
- Paper-grade A/B baseline, NOT an E3 gate (E3 stays on plain `INCR`) — D5
  settled 2026-07-11.
- Dispatch shape mirrors `INCR_SCRATCH` (dedicated `scheduler_mode_` branch) —
  D2 settled 2026-07-11 — but optimizer lifetime is persistent (like `INCR`),
  NOT fresh-each-interval (that is `INCR_SCRATCH`'s amnesia).

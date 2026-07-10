# P0.5 — Tasks (working checklist)

> See [`goal.md`](goal.md) for scope and [`design.md`](design.md) for the full
> spec. TDD per `agent_coding_rules.md`: minimal, testable, modular steps; all
> tests green after each. Behavior-preserving for the structural migration.
>
> **Design is decided** (incumbent-state: `res_opt_` as single durable incumbent,
> `has_incumbent_` gate, `CommitIncumbent` / `BuildChallengerFromIncumbent`
> helpers, transient challenger). These phases are the execution of
> `design.md` §4.

## Phase 1 — Introduce helpers additively (no behavior change)

- [x] **1a.** Add `has_incumbent_` (bool, default false) and the declarations for
      `CommitIncumbent(const PriorityVec& pa, double sp, const std::vector<double>& tl)`
      and `BuildChallengerFromIncumbent()` to `OptimizeSP_TL_Incre.h`. Document
      `time_limit_option_for_each_task_` as transient (per-call, not incumbent).
      No call-site change. Build + tests green.
- [x] **1b.** Implement `CommitIncumbent` = the exact body of the
      `SeedStateFromIncumbent` write-block (`OptimizeSP_TL_Incre.cpp:457-476`)
      **minus** the `prev_optimizer_` lines, plus `has_incumbent_ = true`.
      Implement `BuildChallengerFromIncumbent` returning a fresh
      `OptimizePA_Incre` built from
      `UpdateExtDistBasedOnTimeLimit(dag_tasks_, ReconstructTimeLimitVecFromResOpt())`
      + `sp_parameters_`, with `opt_pa_`/`opt_sp_`/`dag_tasks_` seeded from
      `res_opt_`. No call-site change. Build + tests green.
- [x] **1c.** TDD: add focused unit tests for each helper in isolation
      (write-test-first against the additive impl):
      - `CommitIncumbent_WritesFourTupleAndSetsGate` — populates `res_opt_`
        (`id2time_limit`, `priority_vec`, `sp_opt`), `opt_pa_`/`opt_sp_`, sets
        `has_incumbent_=true`.
      - `BuildChallengerFromIncumbent_ReconstructsAdoptedTlDag` — challenger's
        `dag_tasks_` equals
        `UpdateExtDistBasedOnTimeLimit(dag_tasks_, ReconstructTimeLimitVecFromResOpt())`
        and `opt_pa_`/`opt_sp_` mirror `res_opt_`.

## Phase 2 — Dual-write (flip writers one at a time)

- [x] **2a.** `SeedStateFromIncumbent` calls `CommitIncumbent(pa, sp, tl)`
      internally, then **also** performs the legacy `prev_optimizer_` writes
      (dual write). `has_incumbent_` and `prev_optimizer_.IfInitialized()` must
      agree. Tests green — the 6 `prev_optimizer_.*`-reading tests still pass.
- [x] **2b.** `UpdateRecords`'s `should_update` block (`:127-134`) calls
      `CommitIncumbent` for the `res_opt_`/`opt_*` writes, then **also**
      `prev_optimizer_ = optimizer` (dual write). Tests green.
- [x] **2c.** **Load-bearing flip.** `EvaluateTimeLimitConfig_ScratchOrIncre`
      incremental branch (`:159-176`): replaced
      `OptimizePA_Incre optimizer = prev_optimizer_;` with
      `OptimizePA_Incre optimizer = BuildChallengerFromIncumbent();`. The
      `else` (no incumbent) branch became an explicit `CoutError`
      contract-violation (was the `OptimizeFromScratch` fallback). Tests green.

## Phase 3 — Migrate tests + remove `prev_optimizer_`

- [x] **3a.** Migrated the 6 tests reading `prev_optimizer_.*` to read
      `res_opt_` / `has_incumbent_` / `BuildChallengerFromIncumbent()`. The 3
      DAG-ET assertions rewrite to observe the carried TL/current DAG via
      `BuildChallengerFromIncumbent().dag_tasks_` + `res_opt_.id2time_limit`.
      All 4 remaining `prev_optimizer_` mentions in the test file are now
      comment-only. 44 tests green.
- [x] **3b.** Dropped the legacy `prev_optimizer_` writes from
      `SeedStateFromIncumbent` (2a) and `UpdateRecords` (2b) — both now route
      through `CommitIncumbent` only. Dropped the `prev_optimizer_` member from
      `OptimizeSP_TL_Incre.h` (only comment references remain). Replaced the
      `prev_optimizer_.IfInitialized()` gate in `SeedIncumbentBaseline` with
      `has_incumbent_`. Tests green.
- [x] **3c.** Comment-only fix in `OptimizeSP_Incre.cpp:288-297`: rewrote the
      "UpdateRecords copies this optimizer into `prev_optimizer_`" comment to
      the throwaway-challenger / `res_opt_`-carries-the-adopted-TL model (the DAG
      advance dies with the challenger local; the adopted TL in `res_opt_` is
      what carries).
- [x] **3d.** **Re-derived, not assumed.** (1) Reopt cold-start at
      `ReOptimizePeriodic:575` (`InitializeTimeLimitsFromETConfig()`) is **NOT a
      bug** — keep as-is (reopt path uses `OptimizeFromScratch`, never runs the
      diff; no invariant to preserve). Overturns `design.md` §1 symptom 3's
      "same class of bug" claim. (2) Incremental start at
      `OptimizeIncre_w_TL:398` (`ReconstructTimeLimitVecFromResOpt()` + stale-TL
      guard `:411-421`) is **KEPT as-is, NOT folded** into
      `BuildChallengerFromIncumbent` — the call site's guard-applied
      update-side vector and the helper's raw baseline-reconstruction vector
      serve different purposes. Overturns `design.md` §6 Q3's "fold" default.
      Reasoning recorded in `design.md` §1/§4/§6 and `dev_log.md`.

## Phase 4 — Verify + close

- [x] `testIncreOpt_w_TL` (44) + `testOptimizeIncrePA` + `ctest` (16) green.
- [x] `cmake --build . --target check.SP_OPT -j5` green (DEBUG build).
- [x] Re-ran the P1.1 probe (INCR_P10, N=8 taskset_0, interval 0→1): `ndiff`
      **5 → 0** at call=0 (stronger than the predicted ~2); across all 302
      incremental calls `ndiff` is only ever 0 or 1. Passed from P0.5 alone, no
      YAML persistence / P0.1 required. Recorded in `dev_log.md`.
- [x] Milestone appended to top-level `agents/dev_log.md`.

## Hand-off between sub-sessions

Each phase is one review-and-commit cycle per `agent_coding_rules.md`. The
implementer picks up at the first unchecked box; before starting, re-read
`design.md` §2 (blast radius) + §3 (the diff-baseline invariant) + §4 (the
migration plan) and the per-step spec above. Do not skip the dual-write phase —
it is what lets the test migration (3a) run green against the new observables
before the old field is removed (3b).

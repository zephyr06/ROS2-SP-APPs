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

## Phase 5 — User review fixes (2026-07-09)

Post-commit review of `61ebf42f` surfaced 8 issues. Worked one-by-one with a
review-and-approve gate after each. **(5) and (8) first**, per user direction.

- [x] **5a. (5) Reset `res_opt_` before each new-interval optimization.** The
      baseline eval `current_config_sp = EvaluateTimeLimitConfig_ScratchOrIncre(...)`
      at the top of `PerformCoordinateDescentForTaskConfigOpt` must ALWAYS
      overwrite `res_opt_` for the current interval, else the descent's
      compare-and-keep is measured against a stale previous-interval incumbent.
      **TDD tests PASS** (2: `OptimizeIncre_w_TL_BaselineOverwritesResOptForNewInterval`
      + `ReOptimizePeriodic_BaselineOverwritesResOptForNewInterval`).
      **DONE (2026-07-09): unified the reset into one explicit function.** The
      user chose option (b)-flavored: replace the split implicit mechanisms with
      a single explicit `ResetIncumbentBaseline(bool from_scratch)`, called at
      the top of `PerformCoordinateDescentForTaskConfigOpt` (before the baseline
      eval) AND at the top of `OptimizeWithTimeLimitOptDisabled` (covers the
      disable-path bypass). Branched dispatch (behavior byte-for-byte preserved
      per path):
      - `from_scratch=true` (reopt): today's `SeedIncumbentBaseline` body,
        unchanged — `if (has_incumbent_)` re-eval carried {pa, tl} under the new
        DAG + `SeedStateFromIncumbent` (→ `opt_sp_ = sp_prev_new`, the
        compare-and-keep baseline); `else` interval-0 RM + min-TL synthesis +
        `SeedStateFromIncumbent`. `SeedIncumbentBaseline()` call removed from
        `ReOptimizePeriodic:569` (now runs inside the descent).
      - `from_scratch=false` (incremental): `opt_sp_ = -1.0;` one-liner, moved
        here from `OptimizeIncre_w_TL:375`. Does NOT touch `res_opt_` (ordering
        invariant: the challenger is built from the carried prior in `res_opt_`,
        re-evaluated, then `UpdateRecords` force-commits → overwrites `res_opt_`).
        `opt_sp_ = -1.0` line removed from `OptimizeIncre_w_TL`.
      `SeedIncumbentBaseline` removed from the header; the 2 `SeedIncumbentBaseline_*`
      tests rewritten to call `ResetIncumbentBaseline(true)` (bodies/assertions
      unchanged — reopt branch is byte-identical). Comment trims per user
      directive ("if i see that kind of long code comments, i'll just skip it"):
      the patience / baseline / skip / fallback comments in the descent, the
      carried-adopted-TL + edge-case-guard comments in `OptimizeIncre_w_TL`, and
      the helper-block comments in `EvaluateTimeLimitConfig_ScratchOrIncre` /
      `UpdateRecords` / `SeedStateFromIncumbent` / `CommitIncumbent` /
      `BuildChallengerFromIncumbent` all cut to ~1-3 lines. Stale
      `SeedIncumbentBaseline` / `prev_optimizer_` references in `OptimizeSP_TL_Incre.h`
      and `SimulationOrchestrator.cpp:300` comments updated to
      `ResetIncumbentBaseline` / `has_incumbent_`. **46 `testIncreOpt_w_TL` + 16/16
      ctest green (DEBUG build). Staged (git add only, no commit).**
- [ ] **5b. (8) Reuse the challenger incrementally instead of rebuilding from
      `res_opt_` each interval.** `BuildChallengerFromIncumbent` currently
      constructs a FRESH `OptimizePA_Incre` from `res_opt_` (the "champion")
      every interval, discarding the challenger's internal PA-search state. The
      user's design: keep a PERSISTENT challenger optimizer and MODIFY it
      incrementally each interval (true incremental optimization — reuse the PA
      search state, not just the adopted TL). Trade-off: better efficiency,
      potential SP-performance loss. Compare both designs in experiments, then
      decide which to keep.
- [x] **5c. (1) Remove the stale-TL edge-case guard in `OptimizeIncre_w_TL`.**
      The guard intersected each carried TL against the current option set
      (forced -1 when a task lost its perf pair since N-1, or on a cold
      `res_opt_`). **REMOVED for code simplicity** per user direction. The
      guard was NOT fully latent: each interval loads a fresh DAG
      (`taskset_..._interval_N.yaml`), so a task CAN lose its perf pair across
      intervals, and `UpdateExtDistBasedOnTimeLimit` (which does NOT consult
      `time_limit_option_for_each_task_`) would apply a stale carried TL as a
      point dist via `GetUnitExecutionTimeDist`. Removed anyway — the user
      judged the simplicity win worth the edge-case exposure (no experiment in
      the current suite mutates a task's `timePerformancePairs` across
      intervals, and `OptimizeSingleTaskTimeLimit:208-210`'s
      `FindTimeLimitOptionIndex`-sentinel still skips the WALK on a stale
      baseline; only the baseline eval itself was guarded). **DONE
      (2026-07-09): deleted the `:342-354` loop + trimmed the carried-adopted-TL
      comment above it (dropped the now-stale "opt_sp_=-1.0 reset ... lives in
      ResetIncumbentBaseline" sentence). 46 `testIncreOpt_w_TL` + 16/16 ctest
      green (DEBUG build). Staged (git add only, no commit).**
- [ ] **5d. (2) Reconsider `has_incumbent_`.** The bool gate may be unnecessary
      (`res_opt_` emptiness / `opt_pa_.emptiness` could gate). Evaluate removal.
- [x] **5e. (3) Rename the `time_limits` parameter in
      `PerformCoordinateDescentForTaskConfigOpt`** to convey its origin.
      **DONE (2026-07-09):** renamed to `starting_time_limits` (header decl
      `:132` + definition `:243` + all 5 body usages). Origin is
      **path-dependent** (verified, not assumed): incremental call site
      (`OptimizeIncre_w_TL:340`) passes `ReconstructTimeLimitVecFromResOpt()` —
      the **carried adopted TL from `res_opt_`** (last interval's result, so the
      user's "from last interval optimization" hypothesis IS correct here);
      reopt call site (`ReOptimizePeriodic:478`) passes
      `InitializeTimeLimitsFromETConfig()` — **Gaussian-mean-closest TL for the
      current DAG** (a fresh start, NOT last interval). Since the origin differs
      by path, no name can honestly say "last interval's"; chose the
      origin-neutral `starting_time_limits` ("the TL vector the descent walks
      from") + a 4-line origin comment on the header decl documenting both
      provenances. Scope: only THIS function's parameter — the callees it flows
      into (`EvaluateTimeLimitConfig_ScratchOrIncre`, `OptimizeSingleTaskTimeLimit`,
      `UpdateRecords`) receive a *candidate-being-mutated*, a different role, so
      their `time_limits` params were intentionally NOT renamed. Call sites
      unchanged (pass by position; their local var name is independent). **46
      `testIncreOpt_w_TL` + 16/16 ctest green (DEBUG build). Staged (git add
      only, no commit).**
- [ ] **5f. (4) Remove the dead zero-work fallback in
      `PerformCoordinateDescentForTaskConfigOpt` (`:335-337`).** `any_eval_ran`
      is always true (the baseline eval above always runs first), so the
      `if (!any_eval_ran && !dag_tasks_.tasks.empty())` branch is unreachable.
      Drop the guard and the `any_eval_ran` variable.
- [ ] **5g. (6) Simplify `OptimizeSingleTaskTimeLimit` patience logic.** Drop
      the separate `consecutive_non_improving` counter; decrement `patience`
      directly on non-improvement and stop when patience is exhausted. **Open
      question to confirm with user**: the current code resets the counter on
      improvement (a CONSECUTIVE non-improvement budget). The user's proposal
      ("just use `patience--` if failed... stop if `patience<0` or `patience==0`")
      is a TOTAL non-improvement budget (no reset) — which changes patience=1
      semantics (consecutive: tolerates 1 dip; total-with-`<0`-stop: tolerates 1
      dip; total-with-`<=0`-stop: tolerates 0 dips). Confirm before implementing.
- [ ] **5h. (7) Reuse a single optimizer instance across
      `EvaluateTimeLimitConfig_ScratchOrIncre` calls** instead of rebuilding per
      candidate. Efficiency. **DEFERRED** (user: "we can optimize this efficiency
      issue later"). Entangled with (8).

## Hand-off between sub-sessions

Each phase is one review-and-commit cycle per `agent_coding_rules.md`. The
implementer picks up at the first unchecked box; before starting, re-read
`design.md` §2 (blast radius) + §3 (the diff-baseline invariant) + §4 (the
migration plan) and the per-step spec above. Do not skip the dual-write phase —
it is what lets the test migration (3a) run green against the new observables
before the old field is removed (3b).

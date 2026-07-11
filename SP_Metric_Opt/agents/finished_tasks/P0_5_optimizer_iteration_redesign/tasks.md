# P0.5 — Tasks (working checklist) — RESOLVED 2026-07-10 (all phases complete, committed)

> **RESOLVED 2026-07-10.** Phases 1–5 all checked; code committed
> (`a8dba07f`→`7fa2e9d2`); 46 `testIncreOpt_w_TL` + 16/16 ctest green (DEBUG
> build, re-verified at closeout). 5h moved to P3.1 (perf, not correctness). See
> `goal.md` banner + `agents/finished_tasks/summary.md`. The checklist below is
> the execution record, kept as-is.

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
- [x] **5b. (8) Reuse the challenger incrementally instead of rebuilding from
      `res_opt_` each interval.** `BuildChallengerFromIncumbent` currently
      constructs a FRESH `OptimizePA_Incre` from `res_opt_` (the "champion")
      every interval, discarding the challenger's internal PA-search state. The
      user's design: keep a PERSISTENT challenger optimizer and MODIFY it
      incrementally each interval (true incremental optimization — reuse the PA
      search state, not just the adopted TL). Trade-off: better efficiency,
      potential SP-performance loss. Compare both designs in experiments, then
      decide which to keep. **RESOLVED (2026-07-10): keep the current
      rebuild-from-champion design (decision option (1)); do NOT adopt the
      persistent challenger.** On full trade-off re-analysis the user (re)decided
      the current design is preferable: the champion TL tracks the working TL
      (`UpdateRecords` commits every adoption — strict SP gain or tie-with-
      tighter-TL; `OptimizeSingleTaskTimeLimit` resets to the adopted best on
      no-improvement), so while one task is walked the `FindTaskWithDifferentEt`
      diff flags ONLY that task → `OptimizeIncre` re-searches just its 1D
      priority variations — the perfect case for incremental optimization. A
      persistent challenger would advance `dag_tasks_` to the last-evaluated
      (possibly non-adopted) candidate each call, drifting the diff baseline off
      the adopted working TL and flagging EXTRA tasks (the explored-but-not-
      adopted previous task) → MORE RTA evals, not fewer. Net: rebuild-from-
      champion weakly dominates within-interval (minimal diff, clean PA warm-
      start); the persistent challenger's only potential edge — cross-interval
      PA re-search of DAG-mutated tasks — is a separable mechanism that could be
      added to the rebuild design directly if measurement ever shows it helps.
      Comment added at `BuildChallengerFromIncumbent` + its call site +
      header decl stating the guarantee. 46 `testIncreOpt_w_TL` + 16/16 ctest
      green (DEBUG build). Staged (git add only, no commit).
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
- [x] **5d. (2) Reconsider `has_incumbent_`.** The bool gate may be unnecessary
      (`res_opt_` emptiness / `opt_pa_.emptiness` could gate). Evaluate removal.
      **RESOLVED (2026-07-10): REMOVED `has_incumbent_`; gate on
      `IfInitialized()` (base class = `!opt_pa_.empty()`).** Re-derived, not
      assumed: after P0.5, `CommitIncumbent` is the SINGLE writer of `this->opt_pa_`
      — `OptimizeFromScratch`/`OptimizeIncre` run only on throwaway local challengers
      (`EvaluateTimeLimitConfig_ScratchOrIncre:152/161`), never on `this`. So the
      bool and `!opt_pa_.empty()` flip together, always; the bool added no
      information. Its original reason — the `prev_optimizer_.IfInitialized()` desync
      (opt_pa_ non-empty while sp_parameters_ empty) — is structurally impossible
      with `prev_optimizer_` gone and `CommitIncumbent` the single writer. Changes:
      `else if (has_incumbent_)` → `else if (IfInitialized())` at `:155`;
      `ResetIncumbentBaseline`'s `if (has_incumbent_)` → `if (IfInitialized())` at
      `:428`; dropped `has_incumbent_ = true;` from `CommitIncumbent` (`:394`);
      dropped the `bool has_incumbent_` member (`OptimizeSP_TL_Incre.h:206`); 16 test
      assertions `opt.has_incumbent_` → `opt.IfInitialized()` + the stale "flips
      has_incumbent_" comment phrasings rewritten. Reworded the CoutError message at
      `:170` ("has_incumbent_ is false" → "no incumbent is initialized"). Trade-off
      accepted: loses mild defense-in-depth (a future opt_pa_ write outside
      CommitIncumbent would no longer trip the CoutError) for simpler state; user
      judged the bool "doesn't really [hurt] readability" → wash, so simpler wins.
      **46 `testIncreOpt_w_TL` + 16/16 ctest green (DEBUG build). Staged (git add
      only, no commit).**
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
- [x] **5f. (4) Remove the dead zero-work fallback in
      `PerformCoordinateDescentForTaskConfigOpt`.** `any_eval_ran` is always
      true (the baseline eval above always runs first), so the
      `if (!any_eval_ran && !dag_tasks_.tasks.empty())` branch was unreachable.
      **DONE (2026-07-09): deleted the fallback `if`-block + the
      `any_eval_ran` local (decl + the always-true assignment). Verified
      `any_eval_ran` had no other references (grep across sources/tests/yaml:
      only the 2 lines in this function). Pure dead-code removal — no behavior
      change. 46 `testIncreOpt_w_TL` + 16/16 ctest green (DEBUG build). Staged
      (git add only, no commit).**
- [x] **5g. (6) Simplify `OptimizeSingleTaskTimeLimit` patience logic.** Dropped
      the separate `consecutive_non_improving` counter; `patience` is now
      decremented directly on non-improvement. **DONE (2026-07-09)** per user's
      design: total non-improvement budget, no reset on improvement. Form chosen
      = **check-then-decrement** (`else if (patience == 0) break; else --patience;`)
      rather than the user's literal "patience-- then stop if `patience<0` or
      `patience==0`" — the literal form collapses patience=1 to patience=0
      (tolerates 0 dips), erasing the reopt/incremental distinction; check-then-
      decrement keeps `patience=N` meaning "tolerate N non-improving steps", so
      incremental (patience=0) is byte-identical to before and reopt (patience=1)
      tolerates one dip. Semantics shift for reopt only: budget is now TOTAL not
      CONSECUTIVE — a noisy-but-trending-up region (IMP,NIP,IMP,NIP) now stops at
      the 2nd NIP instead of resetting on each IMP and walking far. Effect:
      shorter reopt walks (efficiency win on the expensive `OptimizeFromScratch`
      evals) at bounded SP risk (each task still gets backward+forward passes).
      Unit suite cannot surface the SP delta — needs an A/B experiment if
      quantifying the reopt SP effect matters. 46 `testIncreOpt_w_TL` + 16/16
      ctest green (DEBUG build). Header doc updated to "total non-improvement
      budget, NOT reset on improvement". Staged (git add only, no commit).
- [~] **5h. (7) Reuse a single optimizer instance across
      `EvaluateTimeLimitConfig_ScratchOrIncre` calls** instead of rebuilding per
      candidate. Efficiency. **MOVED (2026-07-10)** to the deferred efficiency
      bucket `active_tasks/P3_1_efficiency_optimizations/` (new item in its
      `goal.md` + `tasks.md`) — it's a pure perf, not correctness, item so it
      belongs in P3.1, not the P0.5 redesign. The "Entangled with (8)" note is
      stale: (8)/5b was resolved 2026-07-10 (keep rebuild-from-champion), and
      that decision IS the trade-off recorded for this moved item — the
      rebuild-from-champion design was chosen OVER the persistent challenger
      precisely because the champion tracks the working TL so the diff flags
      only the one task being walked (a persistent challenger would drift the
      diff baseline to non-adopted candidates and flag extras). P0.5 Phase 5
      is now complete: 5a–5g done, 5h moved.

## Hand-off between sub-sessions

Each phase is one review-and-commit cycle per `agent_coding_rules.md`. The
implementer picks up at the first unchecked box; before starting, re-read
`design.md` §2 (blast radius) + §3 (the diff-baseline invariant) + §4 (the
migration plan) and the per-step spec above. Do not skip the dual-write phase —
it is what lets the test migration (3a) run green against the new observables
before the old field is removed (3b).

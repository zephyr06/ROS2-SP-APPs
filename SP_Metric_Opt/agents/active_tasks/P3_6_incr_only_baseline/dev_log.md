# P1.6 — Pure Incremental (RM-Fast Bootstrap, No Reopt) Baseline — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-11

- Task **created (PLANNING ONLY — no implementation)** per the user's
  directive: "add a new active task, which is adding a new baseline where we
  mainly use incremental optimization to perform per-interval optimization, and
  we start the RM-Fast rather than re-optimization in the first interval, and
  never call re-optimization periodically. only add this active task without
  implementation. purpose is to compare performance between incr and scratch."
- **Investigated the dispatch + bootstrap flow** to specify the arm accurately:
  - `Optimize_w_TL_ScratchOrIncre` (`OptimizeSP_TL_Incre.cpp:291`) routes on
    `reoptimization_interval_count_ % ReoptimizationPeriod == 0` →
    `ReOptimizePeriodic` (from-scratch + compare-and-keep), else →
    `OptimizeIncre_w_TL` (warm-started incremental). `count==0` →
    `ReOptimizePeriodic`, the bootstrap.
  - `ReOptimizePeriodic` (`:453`) does TWO things at interval 0: (1) seed the
    RM-fast incumbent via `ResetIncumbentBaseline(true)`'s `else` branch
    (`:437`) — `SmallestTimeLimitVec()` (= `timePerformancePairs[0].time_limit`,
    i.e. the orchestrator's `RM_FAST` TL, `SimulationOrchestrator.cpp:380-383`)
    + `RateMonotonicPriorityVec()` → `SeedStateFromIncumbent` →
    `CommitIncumbent`; (2) the full from-scratch descent
    `PerformCoordinateDescentForTaskConfigOpt(..., from_scratch=true)`.
  - The new arm = run step (1) only at interval 0 (seed RM-fast, **skip the
    descent**), then `OptimizeIncre_w_TL` every interval after, **never**
    `ReOptimizePeriodic`. So it is NOT just `ReoptimizationPeriod=∞` (that
    still runs the interval-0 descent); it additionally skips that descent.
- **Wrote `goal.md`** — the finding (what the arm is, with the
  interval-0/descent distinction), the reasoning (clean incr-vs-scratch
  comparison: isolates warm-start value vs `INCR_SCRATCH`, reopt value vs
  `INCR`; de-risks the P1.2 structural-corruption hazard by never running the
  memoryless from-scratch search), 5 open design decisions flagged for the
  user (mode name, branch-vs-flag, interval-0 mechanics, config placement,
  E3-gating), the expected A/B read, and Done-when / Out-of-scope.
- **Wrote `tasks.md`** — Step 0 (settle design decisions with the user, NOT
  started) through Step 4 (build + test + index), all marked NOT STARTED,
  TDD-first per the coding rules.
- **No source code touched.** No `git add`. Folder is `goal.md` + `tasks.md` +
  this `dev_log.md` only. Next action is the user's: greenlight implementation
  (and settle the 5 design decisions), or adjust the task scope.

## 2026-07-11 (follow-up — plans only, still no implementation)

- User follow-up: "we should implement it similar to INCR_sCRATCH, it'll likely
  be a baseline to add to paper. only update plans without implementation. also,
  difference between INCR-P1 and INCR-SCRATCH??"
- **Answered the INCR_P1-vs-INCR_SCRATCH question** by reading the dispatch
  (`OptimizeSP_TL_Incre.cpp:291-307, 424-488`) + orchestrator branches
  (`SimulationOrchestrator.cpp:293-337`). Both arms run the from-scratch descent
  `ReOptimizePeriodic` every interval — that is NOT the difference. The
  difference is optimizer lifetime / incumbent-carrying:
  - `INCR_P1` uses the **persistent** `incr_optimizer_` → `res_opt_` survives →
    `ResetIncumbentBaseline(true)` takes the `if (IfInitialized())` branch
    (`:427-436`) at interval 1+ → compare-and-keep measures against the
    **running best**.
  - `INCR_SCRATCH` builds a **fresh** `scratch_opt` each interval
    (`SimulationOrchestrator.cpp:334`) and discards it → `res_opt_` empty →
    `ResetIncumbentBaseline(true)` takes the `else` branch (`:437-447`) every
    interval → compare-and-keep measures against a **synthetic RM baseline**.
  - Net: `INCR_P1` weakly dominates `INCR_SCRATCH` in SP; `INCR_SCRATCH` is the
    amnesiac (no-memory) control. (In-source comment at `:328-333` states this
    but still names `prev_optimizer_`, the member P0.5 removed — stale wording,
    not stale logic; it now runs through `res_opt_`.)
- **Settled D2 + D5** per the user's directive:
  - D2 → **(a) new `scheduler_mode_` branch** (mirror `INCR_SCRATCH`'s dispatch
    shape; NOT a `GlobalVariables` flag).
  - D5 → **paper-grade baseline; A/B probe, NOT an E3 gate** (E3 stays on plain
    `INCR`).
- **Flagged the persistent-vs-fresh nuance** (the one subtlety in "similar to
  INCR_SCRATCH"): the new arm mirrors `INCR_SCRATCH`'s dispatch *shape* (a
  dedicated branch) but MUST use the **persistent** `incr_optimizer_` to carry
  the incumbent (like `INCR`) — NOT the fresh-`scratch_opt`-each-interval
  lifetime (that is `INCR_SCRATCH`'s amnesia, the exact thing the new arm
  contrasts). Recorded in `goal.md` "What the arm is NOT" + the three-way
  contrast table + `tasks.md` Step 2.
- **Updated `goal.md`**: added the INCR_P1-vs-INCR_SCRATCH subsection + a
  three-way contrast table (NEW arm vs INCR_P1 vs INCR_SCRATCH across
  incumbent-carry / per-interval descent / compare-and-keep baseline); rewrote
  the reasoning to name `INCR_P1` (not generic `INCR`) as the reopt sibling and
  note the softening contrast at higher periods; marked D2+D5 SETTLED in the
  design-decisions section; added the persistent-optimizer clarification to
  Done-when + "What the arm is NOT."
- **Updated `tasks.md`**: Step 0 now shows D2+D5 checked (SETTLED), D1/D3/D4
  open; Step 1 + Step 2 + Standing constraints carry the persistent-optimizer
  nuance.
- **No source code touched.** No `git add`. D1 (mode name), D3 (interval-0
  mechanics), D4 (config placement) remain open for the user.

## 2026-07-26 (resumption — STALENESS AUDIT vs current code, NO implementation)

- Resumed P3.6; re-read `goal.md` + `tasks.md`. Both were filed 2026-07-11 and
  are now **STALE** w.r.t. two later refactors that landed after this task was
  filed. Audited the current code (no source touched) before doing anything else.
- **Staleness 1 — `INCR_SCRATCH` was REMOVED** (by P2.5). Repo-wide grep:
  `INCR_SCRATCH` survives ONLY in `tests/radius_comparison/*.csv` (historical
  data) + a one-line `tests/python/test_evaluation_suite.py:7` note ("P2.5
  removed the `INCR_SCRATCH` ablation arm + the E2 gate"). NO live code, NO live
  config, NO orchestrator branch. **This breaks P3.6's core premise:**
  - **D2 (SETTLED 2026-07-11 = "mirror `INCR_SCRATCH`'s dispatch shape") is
    INVALID** — there is no `INCR_SCRATCH` branch to mirror. Needs re-decision.
  - **The task's stated PURPOSE ("compare performance between incr and scratch")
    is undercut** — the "scratch" side of the A/B no longer exists in the
    scheduler set. The new arm would now be an incr-vs-reopt (`INCR_P1`/`INCR_Pn`)
    comparison only, not incr-vs-scratch.
- **Staleness 2 — `PerformCoordinateDescentForTaskConfigOpt` was DELETED** (by
  P2.16 Stage A1, `6d068ec9`). `goal.md`/`tasks.md` Step 2 + D3 still reference
  it as the descent to skip. Current code: `ReOptimizePeriodic`
  (`OptimizeSP_TL_Incre.cpp:748`) now calls `RunIntervalDescent(...,
  IntervalDescentMode::Reopt)` directly (P2.11 merged the two descent bodies into
  one shared `RunIntervalDescent`, `:438`). The interval-0 RM-fast seed is now
  buried ONE level deeper: `RunIntervalDescent` → `SeedBaselineAndArmCache`
  (`:376`) → `ResetIncumbentBaseline(true)` `else` branch (`:733-742`) =
  `SmallestTimeLimitVec` + `RateMonotonicPriorityVec` → `SeedStateFromIncumbent`
  → `CommitIncumbent`.
- **D3-recommended `BootstrapIncumbentFromRMFast()` is STILL VIABLE** despite the
  restructure. `ResetIncumbentBaseline(true)` remains a distinct public method;
  at interval 0 (fresh persistent `incr_optimizer_`, `IfInitialized()` false) it
  naturally takes the RM-fast `else` branch. A seed-only bootstrap = call
  `ResetIncumbentBaseline(true)` (which seeds+commits the RM-fast incumbent) and
  skip `RunIntervalDescent`'s walk tail. Cleanly separable; `ReOptimizePeriodic`
  untouched. (The seed logic itself is unchanged from 2026-07-11's description;
  only its call-depth moved.)
- **Current dispatch + construction** (verified, `SimulationOrchestrator.cpp`):
  - Pre-loop construction (`:298-300`): `INCR` || `IsINCRPeriodVariant` ||
    `INCR_NO_TL` || `INCR_WCET` → build persistent `incr_optimizer_`.
  - Dispatch (`DeterminePrioritiesAndBudgets`, `:324-345`): `INCR`+period-variants
    → `Optimize_w_TL_ScratchOrIncre`; `INCR_NO_TL`/`INCR_WCET` → same call (their
    difference is the `disable_time_limit_opt`/`use_wcet_execution_time` flags).
    `BF`/`RM`/`RM_FAST`/`RM_SLOW` are separate branches. No `INCR_SCRATCH` branch.
- **Bottom line:** P3.6 is still implementable, but its *motivation* shifted (no
  `INCR_SCRATCH` to contrast) and D2 needs re-settling. The 3 open decisions
  (D1/D3/D4) plus the now-invalid D2 must go to the user BEFORE any code. No
  source touched, no `git add`. Audit only.
- **D4 config premise ALSO stale (confirmed):** `ls simulation_experiments/configs/`
  shows only 4 survivors — `compare_against_bf.json`, `incr_et_profiling.json`,
  `p211_reopt_ab_config.json`, `paper_simulation_config.json`. P2.8 consolidated
  the config set; `p25_period_ab_config.json` (D4's recommended target) is GONE.
  D4 must now pick among the 4 survivors (likely `paper_simulation_config.json`
  for the paper-bound baseline, or `p211_reopt_ab_config.json` for an A/B probe —
  but the latter is the parallel agent's file, do not edit concurrently).

## 2026-07-26 (user restatement + decisions settled; implementation STARTED)

- User restated the arm's behavior (unchanged from 2026-07-11): "the new baseline
  that i want to add: initial solution is rm fast, all the follow-up optimization
  is via incremental optimization, no re-optimization like the INCR_Reopt_10
  does." This re-targets the A/B contrast at `INCR_Reopt_10` (the production
  incremental arm) instead of the removed `INCR_SCRATCH`; behavior is unchanged.
- **Settled all 5 design decisions** (no further user input needed — they follow
  from the user's restatement + the current code; user may override at review):
  - **D1 = `INCR_NO_REOPT`** (parallels `INCR_NO_TL`; mirrors user's "no
    re-optimization" phrasing; clean A/B pair with `INCR_Reopt_10`).
  - **D2 = dedicated `scheduler_mode_` branch** (shape unchanged; the
    "mirror INCR_SCRATCH" rationale is void but the branch is still right).
  - **D3 = `OptimizePureIncremental` entry point + `BootstrapIncumbentFromRMFast`
    seed-only helper.** Entry point mirrors `Optimize_w_TL_ScratchOrIncre`'s
    shape (one call, internal count routing, advances the counter, returns
    `opt_pa_`): `count==0` → bootstrap (set `dag_tasks_` +
    `ApplyWCETAblationIfRequired` + `time_limit_option_for_each_task_` +
    `ResetIncumbentBaseline(true)`, NO descent); `count>0` → `OptimizeIncre_w_TL`.
  - **D4 = `paper_simulation_config.json`** (where `INCR_Reopt_10` lives;
    `p25_period_ab_config.json` is gone per P2.8).
  - **D5 = paper-grade baseline, NOT an E3 gate** (unchanged).
- Rewrote `goal.md` + `tasks.md` against the current code (the 2026-07-11 versions
  were pervasively stale: `INCR_SCRATCH` removed, `PerformCoordinateDescentForTaskConfigOpt`
  deleted, `p25_period_ab_config.json` gone).
- Proceeding to Step 1 (TDD). No source touched yet.

## 2026-07-26 (implementation — Steps 1–4 DONE, awaiting user review)

- **Step 1 (TDD, DONE):** added 3 tests to `tests/testIncreOpt_w_TL.cpp` under the
  existing `CompareAndKeepSynthetic` fixture:
  1. `OptimizePureIncremental_Interval0IsSeedOnly` — interval 0 produces the
     RM-fast incumbent (RM priorities + smallest TL) with `eval_count_==0` (the
     measurable "no descent" guarantee — `ResetIncumbentBaseline(true)` calls
     `EvaluateSPWithPriorityVec` directly, never `CallOptimizerGivenTimeLimits`).
  2. `OptimizePureIncremental_AdvancesCounterOncePerCall` — counter uniform with
     `Optimize_w_TL_ScratchOrIncre`.
  3. `OptimizePureIncremental_NeverReoptsEvenAtPeriodOne` — with
     `ReoptimizationPeriod=1`, the pure arm's interval 0 has strictly fewer
     evals than the reopt dispatcher's interval 0 (skips the from-scratch beam);
     interval 1+ takes the incremental path.
  Confirmed RED first (compile error: `OptimizePureIncremental` doesn't exist),
  then GREEN after Step 2. First green run had test 3 over-specified (asserted
  TL/SP equality on interval 1+, but the incremental walk legitimately IMPROVES
  the TL beyond the cheap RM-fast seed — that's the arm doing its job). Rewrote
  test 3 to pin the real contract (eval-count comparison vs the reopt arm).
- **Step 2 (wire the arm, DONE):**
  - `OptimizeSP_TL_Incre.h`: declared `OptimizePureIncremental` +
    `BootstrapIncumbentFromRMFast` (public, after `Optimize_w_TL_ScratchOrIncre`).
  - `OptimizeSP_TL_Incre.cpp`: `BootstrapIncumbentFromRMFast` mirrors
    `ReOptimizePeriodic`'s setup (absorb `dag_tasks_` + `ApplyWCETAblationIfRequired`
    + `time_limit_option_for_each_task_=RecordTimeLimitOptions`) then calls
    `ResetIncumbentBaseline(true)` — SKIPS `RunIntervalDescent`. `OptimizePureIncremental`
    = `count==0` → bootstrap, else `OptimizeIncre_w_TL`; advances counter; wrapped
    in `BFDLSharedBudget` (per-interval budget, same as the reopt dispatcher).
  - `SimulationOrchestrator.cpp`: added `INCR_NO_REOPT` to the construction
    condition (`:298`) + a dispatch branch (`:324`) calling
    `incr_optimizer_.OptimizePureIncremental(...)`.
  - `RunOrchestrator.cpp`: N/A — source file no longer exists (legacy binary;
    mode strings are config-driven).
- **Step 3 (config + docs, DONE):** added `INCR_NO_REOPT` to
  `ablation_scheduler_list` in BOTH `test_mode` and `prod_mode` of
  `paper_simulation_config.json` + updated the prod_mode `_comment`. Placed in
  ablation (not main) so the main figure's 5 production arms are unchanged; the
  contrast lands in the ablation figure where "does reopt earn its cost?" belongs.
- **Step 4 (build gate, DONE):** `cmake --build build_test --target check.SP_OPT
  -j5` → 17/17 ctest green (18.23s). SP bit-identical for existing arms (the
  change is purely additive: new dispatch branch + new methods; existing
  `Optimize_w_TL_ScratchOrIncre` untouched; the `*BitIdentical*` +
  `RunIntervalDescent_Incremental_MatchesWrapperSP` probes still pass).
- **No `git commit`** (user's standing constraint). About to `git add` the P3.6
  unit: `OptimizeSP_TL_Incre.{h,cpp}`, `SimulationOrchestrator.cpp`,
  `testIncreOpt_w_TL.cpp`, `paper_simulation_config.json`. User rebuilds
  `release/` + re-runs the A/B.

## 2026-07-26 (resumption — VERIFICATION + doc closeout)

- Resumed; re-audited the working tree against the prior session's records
  (records can drift from code). Confirmed the Step 1–4 implementation is intact
  and present in the working tree (`git diff --stat` = the 10-file P3.6 unit).
- **Re-ran the build gate** (ground truth, not just the record's claim):
  `cmake --build build_test --target check.SP_OPT -j5` → **17/17 ctest green
  (16.20s)**. SP bit-identical for existing arms confirmed (additive change).
- **Doc closeout (the unchecked tasks.md items):**
  - Memory file `p36-incr-no-reopt-baseline.md` already written (prior session);
    added the **missing `MEMORY.md` pointer** under "Deferred (P2/P3)".
  - Appended the **top-level `agents/dev_log.md` milestone** (2026-07-26 P3.6
    entry).
  - Reconciled **`goal.md` D4 wording**: was "`main_scheduler_list`", but the
    actual decision + config edit is `ablation_scheduler_list` (sound: keeps the
    main figure's 5 production arms unchanged; the "does reopt earn its cost?"
    contrast belongs in the ablation figure). Fixed D4 to match reality.
- **Flagged (NOT fixed — concurrent-edit hazard):** `agents/overall_tasks.md:30`
  has the P3.6 row but LABELS it `P1.6` (the folder is `P3_6_incr_only_baseline/`).
  That file is being concurrently edited by another agent (their unstaged
  P2.9–P2.12 row expansion is in `git diff`); editing it now risks a conflict.
  Left for the user / parallel agent to reconcile the `P1.6`→`P3.6` label.
- **Nothing staged yet** (`git diff --cached` empty). The P3.6 unit is ready to
  `git add` for user review. No source touched this session — verification +
  docs only.

## 2026-07-26 (refactor — extract `AbsorbUpdatedDAG`; behavior-neutral)

- User: "slight code refactor, since both the new baseline and re-optimization's
  first interval initializes from rm fast, we can add one function and make that
  method also call it to initialize from rm fast."
- **Audited the duplication precisely before touching anything.** The RM-fast
  *seed* itself is ALREADY centralized — both `BootstrapIncumbentFromRMFast` and
  `ReOptimizePeriodic`'s interval-0 reach the SAME `ResetIncumbentBaseline(true)`
  `else`-branch (`:733-742`: `SmallestTimeLimitVec` +
  `RateMonotonicPriorityVec` → `SeedStateFromIncumbent` → `CommitIncumbent`).
  The new baseline calls it directly; reopt reaches it via `RunIntervalDescent`
  → `SeedBaselineAndArmCache`. So the seed is NOT the duplication.
- **The genuine duplication = the 3-line DAG absorb** byte-identical across all
  THREE interval-entry methods (`BootstrapIncumbentFromRMFast`,
  `OptimizeIncre_w_TL`, `ReOptimizePeriodic`):
  `dag_tasks_ = dag_tasks_update; ApplyWCETAblationIfRequired(dag_tasks_);
  time_limit_option_for_each_task_ = RecordTimeLimitOptions(dag_tasks_);`
- **Extracted `AbsorbUpdatedDAG(dag_tasks_update)`** (private helper, declared
  in the implicit-private block after the public entry points in the `.h`).
  All three callers now call it. `OptimizeIncre_w_TL` + `ReOptimizePeriodic`
  still capture `dag_tasks_prev_pre_tl = dag_tasks_;` BEFORE the call (Type-E
  diff source for `BuildSerializedTaskQueue`), so the diff source is unchanged.
- **Deliberately did NOT make `ReOptimizePeriodic` call
  `BootstrapIncumbentFromRMFast` directly.** That was the tempting "one
  function" reading but would have been WRONG: `BootstrapIncumbentFromRMFast`
  calls `ResetIncumbentBaseline(true)` itself, and `ReOptimizePeriodic` ALSO
  reaches `ResetIncumbentBaseline(true)` inside `RunIntervalDescent` →
  `SeedBaselineAndArmCache`. Calling both would double-reset and — at interval
  0 — change the from-scratch beam's starting TL (min-TL instead of Gaussian-
  mean TL via `InitializeTimeLimitsFromETConfig`), breaking bit-identical
  behavior and confounding the `INCR_NO_REOPT` vs `INCR_Reopt_10` A/B. The
  absorb was the safe thing to extract; the seed was already shared.
- **Verify:** `cmake --build build_test --target check.SP_OPT -j5` → 17/17 ctest
  green (17.52s). SP bit-identical (refactor only relocates 3 lines already in
  lockstep; `*BitIdentical*` + `RunIntervalDescent_Incremental_MatchesWrapperSP`
  probes pass).
- **Staged** `OptimizeSP_TL_Incre.{h,cpp}` (`git add`). No commit (standing
  rule). Pure refactor — no behavior change, no new tests needed (the existing
  P3.6 + bit-identical probes cover it).

## 2026-07-26 (refactor follow-up — extract `SeedIncumbentFromRMFast`; behavior-neutral)

- **User correction** on the entry above: "i meant the code in
  ResetIncumbentBaseline, it also initializes from RM fast." The user's target
  was the RM-fast *seed* itself, NOT the DAG absorb. The prior entry's premise
  ("the seed is ALREADY centralized... the genuine duplication = the 3-line DAG
  absorb") was a misreading of the user's intent: the seed was centralized only
  in the sense that both call sites routed through `ResetIncumbentBaseline(true)`,
  but the actual seed *body* (`SmallestTimeLimitVec` +
  `RateMonotonicPriorityVec` → `UpdateExtDistBasedOnTimeLimit` +
  `EvaluateSPWithPriorityVec` → `SeedStateFromIncumbent`) was inline in
  `ResetIncumbentBaseline`'s `else`-branch (`:775-783`), with no named primitive.
- **Kept the `AbsorbUpdatedDAG` extraction** (the prior entry): it is a real
  3-site dedup (the absorb is byte-identical across `BootstrapIncumbentFromRMFast`,
  `OptimizeIncre_w_TL`, `ReOptimizePeriodic`), green, behavior-neutral. Leaving it
  in. Flagged to the user to revert if they consider it out-of-scope.
- **Extracted `SeedIncumbentFromRMFast()`** (private helper, declared in the `.h`
  after `ResetIncumbentBaseline`):
  `SmallestTimeLimitVec` + `RateMonotonicPriorityVec` →
  `UpdateExtDistBasedOnTimeLimit` → `EvaluateSPWithPriorityVec` →
  `SeedStateFromIncumbent`. Named `SeedIncumbentFromRMFast` to mirror
  `BootstrapIncumbentFromRMFast` (the caller) + the existing `SeedStateFromIncumbent`
  primitive (the established `Seed*` verb in this class — no invented term).
- **Wired both call sites:**
  - `ResetIncumbentBaseline`'s `else`-branch (interval 0, reopt) → calls
    `SeedIncumbentFromRMFast()`. The preamble (`rta_cache_ = RTACache();
    rta_cache_active_ = false;`) stays in `ResetIncumbentBaseline`, runs before.
  - `BootstrapIncumbentFromRMFast` → now calls `SeedIncumbentFromRMFast()`
    DIRECTLY, no longer `ResetIncumbentBaseline(true)`.
- **Bit-identical reasoning** (why direct-call is safe): `BootstrapIncumbentFromRMFast`
  runs ONLY at interval 0 (count==0, first call on a fresh persistent
  `incr_optimizer_`). At that point `rta_cache_` is already default-constructed
  and `rta_cache_active_` already `false` (the optimizer's ctor-initialized
  state), so the `ResetIncumbentBaseline` preamble it now skips is a no-op there.
  Interval 1+ never reaches `BootstrapIncumbentFromRMFast` (it takes the
  `OptimizeIncre_w_TL` branch). For `INCR_Reopt_X`, `ResetIncumbentBaseline(true)`
  still runs its full preamble + else-branch unchanged (just with the seed body
  relocated into the helper). Net: byte-identical behavior for both arms.
- **Verify:** `cmake --build build_test --target check.SP_OPT -j5` → 17/17 ctest
  green (17.52s). SP bit-identical (`*BitIdentical*` +
  `RunIntervalDescent_Incremental_MatchesWrapperSP` probes pass; the P3.6
  `Interval0IsSeedOnly` test still pins `eval_count_==0`).
- **Staged** `OptimizeSP_TL_Incre.{h,cpp}` (`git add`). No commit (standing rule).
  Pure refactor — no behavior change, no new tests needed.

## 2026-07-26 (plotting fixes — ablation reference baseline + drop redundant raw SP figures)

- **User:** the exported `fig_ablation_mean_sp_normalized_vs_tasks.png` must also
  carry `INCR_Reopt_10` — "only adding baseline is useless" (no contrast). And
  for exported figures of the same type, keep ONLY the normalized SP variant —
  no need to also plot the raw one (e.g. drop `fig_ablation_mean_sp_vs_tasks.png`).
- **Fix 1 (ablation reference baseline):** `prod_mode.ablation_scheduler_list`
  led with `INCR_NO_TL` and omitted `INCR_Reopt_10` (it lived only in
  `main_scheduler_list`), so the ablation figure plotted `INCR_NO_REOPT` with no
  reference. Added `INCR_Reopt_10` at the HEAD of `prod_mode.ablation_scheduler_list`
  (test_mode already had it). `build_scheduler_union` (run_end_to_end_experiments.py)
  de-duplicates main+ablation, so `INCR_Reopt_10` is simulated ONCE — config-only,
  no re-simulation. Run-id is built from mode/dur/interval/seed/task-counts (NOT
  scheduler lists), so the existing prod run dir is reused; re-aggregated in place.
  Updated the `_comment` on prod_mode to record the rationale.
- **Fix 2 (drop redundant raw SP figures):** TDD. Wrote failing tests first in
  `tests/python/test_aggregate.py` (`test_figures_normalized_replaces_raw_1a_and_1b`,
  `test_ablation_normalized_replaces_raw_sp`, rewrote `test_boxplot_normalized_emitted`)
  + `tests/python/test_interval_sweep.py` (`test_emits_only_normalized_when_on`).
  Confirmed red (4 fail, 50 pass), then green.
- **Code:** `aggregate_across_tasks.py` `generate_main_group_figures` (1A mean SP,
  1B std SP) + `generate_ablation_group_figures` (ablation mean SP) +
  `generate_distribution_boxplot` (Fig 1F) now emit ONLY the normalized variant
  when `normalize_sp` is on, raw as the fallback when off. `interval_sweep.py`
  `generate_interval_sweep_figure` (Fig 2) likewise. The non-SP figures (1C exec
  time, 1D/1E miss rate, ablation exec time) are UNAFFECTED. Rule: when a
  normalized SP figure of the same type is emitted, the raw one is dropped
  (redundant — same shape, unscaled y-axis). Updated both module docstrings.
- **Stale-file note:** the aggregator writes figures but does NOT delete ones it
  no longer produces; the prior run's raw SP `.png`/`.pdf` stay on disk until
  deleted. Deleted the 5 raw-SP stems (png+pdf) from the prod run's `figures/`
  dir since the normalized variants now exist.
- **Verify:** `pytest tests/python/test_aggregate.py tests/python/test_interval_sweep.py`
  → 54/54 green. Full suite: 350 passed, 2 FAILED (`test_all_configs_carry_time_limit_seconds`,
  `test_all_shipped_configs_readable_in_both_modes`) — PRE-EXISTING, unrelated to
  this work: `compare_against_bf.json test_mode time_limit_seconds=10` (from
  commit `8c547d85` "update exp config"), NOT touched by these changes.
- **Staged** `paper_simulation_config.json`, `aggregate_across_tasks.py`,
  `interval_sweep.py`, `test_aggregate.py`, `test_interval_sweep.py`,
  `dev_log.md` (`git add`). No commit (standing rule). Plotting-only — no C++
  or sim change.

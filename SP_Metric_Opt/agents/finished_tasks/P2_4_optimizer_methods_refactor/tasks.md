# P2.4 — Tasks (working checklist)

> See `goal.md` for the issue catalogue (I1–I5) + the reasoning. **DESIGN SETTLED +
> IMPLEMENTATION COMPLETE 2026-07-11** (Slices A+B + the 1-arg overload test migration:
> 16/16 ctest DEBUG green + 291 python tests green; `git add` staged, no commit). Scope
> this pass = Slice A (comments) + Slice B (mode-string rename `INCR_P<n>` ->
> `INCR_Reopt_X`, X in {1,5,10,30,60}, X=5 NEW) + the 1-arg `ReOptimizePeriodic` overload
> test-site migration (the overload itself is kept for Step 5). Slice C (I5 method
> renames) DEFERRED. `INCR_SCRATCH` removal filed as Step 5 (SEPARATE sub-task, NOT this
> pass — result-changing). One review-and-commit cycle per `agent_coding_rules.md`.
> Slices A+B + the overload migration are behavior-preserving — TDD was "existing tests
> stay green, no expectations change" (only the new `INCR_Reopt_5` arm + renamed fixture
> keys); the python E3 fixtures went 6 RED → 30 GREEN.

## Step 0 — Settle scope + design decisions with the user (DONE 2026-07-11)

- [x] D1 — Scope: Slice A + Slice B + the 1-arg overload removal this pass; Slice C
      deferred; `INCR_SCRATCH` removal filed as Step 5 (separate sub-task).
- [x] D2 — Rename direction: `INCR_Reopt_X` (X in {1,5,10,30,60}; X=5 NEW). `Reopt`
      matches codebase vocabulary; keeps the `INCR_` family prefix. Old `INCR_P<n>`
      FAILS LOUDLY.
- [x] D3 — Bare `INCR`: stays canonical (= `INCR_Reopt_10` via YAML); A/B configs use
      explicit `INCR_Reopt_X`.
- [x] D4 — 1-arg `ReOptimizePeriodic(int K)` overload: REMOVED; ~11 call sites migrate
      to the 2-arg form.

## Step 1 — Slice A: stale-comment rewrite (DONE 2026-07-11, independently shippable)

- [x] `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp:329` — rewrite the
      `prev_optimizer_` wording to name `res_opt_` / the P0.5 incumbent-state design.
- [x] `tests/testIncreOpt_w_TL.cpp:442` — rewritten to `res_opt_`/`IfInitialized()`.
      `:1087` + `:1204` confirmed INTENTIONAL historical contrasts (old
      `prev_optimizer_` design vs the redesign; P1.4 removed the knob) — left as-is
      (they read as history, the I3/I4 scope; not stale wording).
- [x] Review `ReoptStartFromAdoptedTL` history comments
      (`SimulationOrchestrator.cpp:28`, `RunOrchestrator.cpp:19`,
      `testIncreOpt_w_TL.cpp:1204`) — confirmed they read as history (not a live flag);
      the `:28`/`:19` history comments were carried into the renamed `INCR_Reopt_X`
      prose during Slice B and tightened.
- [x] Build + ctest green (sanity — no behavior change expected).

## Step 2 — Slice B: mode-string rename (DONE 2026-07-11, the trigger issue)

- [x] TDD: the python E3-gate fixtures were the test surface (the C++
      `MaybeOverrideReoptPeriod` is a `static` fn inside the `RunOrchestrator` main TU,
      not a gtest harness, so the fail-loudly behavior is verified by code inspection +
      the gate-level python TDD). RED state: 6 E3 tests failed because the gate expected
      `INCR_Reopt_X` keys but fixtures used `INCR_P<n>` → "missing"; GREEN after migrating
      the fixtures (30 passed, was 24/6). The C++ hard-error for stale `INCR_P<n>` is in
      `MaybeOverrideReoptPeriod` (returns true → falls through dispatch to the RM baseline,
      loud not silent — the P1.3 trap pattern).
- [x] `tests/RunOrchestrator.cpp` — `MaybeOverrideReoptPeriod`: parses the new
      `INCR_Reopt_` prefix; emits the HARD ERROR for the old `INCR_P<n>` form.
- [x] `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp` —
      `IsINCRPeriodVariant` + the construction condition + the dispatch condition: match
      the new `INCR_Reopt_` prefix.
- [x] Configs — renamed every arm to `INCR_Reopt_X` + ADDED the new `INCR_Reopt_5` arm:
      `evaluation_suite_config.json`, `p25_period_ab_config.json` (+ the `_comment`
      fields). No `experiment_config.json` occurrence of `INCR_P<n>` (verified).
- [x] Python — `evaluation_suite.py` (`DEFAULT_PERIOD_ARMS` + module/param docstrings) +
      `tests/python/test_evaluation_suite.py` (E3 fixtures migrated, 6 RED→GREEN) +
      `simulation_experiments/repro_et_grows_with_period.py` (`ARMS` + `--arms` + docstring).
      `compare_optimizers.py` / `aggregate_across_tasks.py` / `run_radius_comparison.py`
      only reference the ablation group (no period arms) — no rename needed.
      `run_e2e_eval.py` does not exist. `test_compare_optimizers.py` /
      `test_aggregate.py` / `test_run_end_to_end.py` clean (verified).
- [x] `--help` Usage line in `RunOrchestrator.cpp` updated (lists `INCR_Reopt_X` + notes
      the retired `INCR_P<n>` is a HARD ERROR).
- [x] Build + ctest green (16/16 DEBUG). The user runs the full new-name A/B.

## Step 3 — 1-arg `ReOptimizePeriodic(int K)` overload removal (DONE 2026-07-11 — test migration only; overload kept for Step 5)

- [x] Migrated the test/example 1-arg call sites to the 2-arg form:
      `tests/testIncreOpt_w_TL.cpp` (×5), `tests/testOptimizeIncrePA.cpp:312`,
      `tests/testBF_w_TL.cpp:67`, `tests/AnalyzePriorityAssignmentIncrementalExample.cpp:62`.
      `dag_tasks` is in scope at every site (fixture member or local `DAG_Model`).
- [~] `sources/Optimization/OptimizeSP_TL_Incre.h` / `.cpp` — the 1-arg declaration (:84)
      + def (:287-289) STAY (decision (a) — keep the overload until Step 5). Its only
      remaining caller is the INCR_SCRATCH branch (`SimulationOrchestrator.cpp:335`);
      Step 5 removes that branch + the overload together. Kept self-contained for Step 5.
- [x] Build + ctest green (behavior-preserving — the 1-arg body was
      `return ReOptimizePeriodic(dag_tasks_, K);`). 16/16 DEBUG.

## Step 4 — Slice C: method renames (DEFERRED — NOT this pass)

- Conflicts with P1.6/P1.2's in-flight edits to `OptimizeSP_TL_Incre`. The user's "rename
  methods as planned" = the overload removal (Step 3) + the comment rewrites (Step 1)
  only. Re-flag when P1.6/P1.2 settle.

## Step 5 — `INCR_SCRATCH` removal (DONE 2026-07-11 — promoted to P2.5; see [`P2_5_incr_scratch_removal/`](../P2_5_incr_scratch_removal/))

> Filed per the user's "we can remove it in a separate step." Promoted to its own task
> (P2.5) because it is result-changing (deletes a scheduler arm + collapses E2) and
> deserved its own review cycle. P2.5 executed the full removal surface below.

- [x] `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp` — delete the
      `INCR_SCRATCH` dispatch branch + the construction-condition string + its comment;
      delete the 1-arg `ReOptimizePeriodic(int K)` overload (`OptimizeSP_TL_Incre.h` +
      `.cpp`) now that its last caller (the SCRATCH branch) is gone.
- [x] Configs — dropped `INCR_SCRATCH` from `evaluation_suite_config.json`,
      `p25_period_ab_config.json`, `experiment_config.json`.
- [x] Python — dropped SCRATCH from the iteration tuples in `evaluation_suite.py`
      (Q1/Q2/Q3/E1), `compare_optimizers.py`, `aggregate_across_tasks.py`,
      `repro_et_grows_with_period.py`, the debug-analysis scripts, and the test-fixture
      keys in `test_evaluation_suite.py` / `test_aggregate.py` /
      `test_run_end_to_end.py` / `test_compare_optimizers.py`.
- [x] `evaluation_suite.py` — DELETED the E2 gate (its only subject pair was
      INCR-vs-SCRATCH); Q1/Q2/Q3/E1 now check INCR alone. Not substituted with
      Reopt_1 (that would re-state E3's first edge). 5 gates, not 6.
- [x] `agents/project_evaluation_northstar.md` — deleted the "INCR cannot run slower
      than SCRATCH" line + the SCRATCH mentions in the SP-quality bullets.
- [ ] `agents/active_tasks/P1_6_incr_only_baseline/goal.md` — **DEFERRED to the user.**
      P1.6's premise (compare INCR vs SCRATCH) is now obsolete: with SCRATCH removed,
      P1.6 needs its comparison axis re-set (vs `INCR_Reopt_1`, the always-reopt-with-
      memory arm), not a find-replace. P1.6 is PLANNING-ONLY; the user decides its new
      framing. Tracked in P2.5 dev log.
- [x] Build + ctest green (16/16); python tests green (287); user re-runs the A/B
      without INCR_SCRATCH (user's task).

## Step 6 — Index + hand off (DONE 2026-07-11)

- [x] `agents/overall_tasks.md` — updated the P2.4 row (filed -> implemented) + the
      Suggested execution order.
- [x] Top-level `agents/dev_log.md` — appended the P2.4 milestone (filed -> implemented).
- [x] `git add` the P2.4 unit; hand to user for review (no commit).

## Standing constraints

- No `git commit` (user's task; `git add` only).
- **Slices A+B + the overload removal are behavior-preserving.** No test *expectation*
  changes — only symbol names + the new `INCR_Reopt_5` arm added to configs. If a rename
  would change a result, stop and re-file as a P1/P0. (Step 5 is exempt — it is result-
  changing by design, hence a separate sub-task.)
- The mode-string rename (Slice B) is the central risk: keep C++ <-> config <-> python in
  sync; a stale old name must FAIL LOUDLY (the P1.4 `_ADOPTED` pattern), not silently
  alias.
- Leave room for P1.6's `INCR_PURE` arm — the renamed family reads consistently with it
  (`INCR_Reopt_1`...`INCR_Reopt_60` vs `INCR_PURE` vs `INCR_SCRATCH` reads cleanly; after
  Step 5, vs `INCR_PURE` vs the bare-`INCR` default).
- Parallel-anytime with P1.1 / P1.2 / P1.6 / P2.2 / P2.3, BUT Slice C (method renames)
  conflicts with P1.6/P1.2's in-flight edits to `OptimizeSP_TL_Incre` — coordinate if
  both are active.

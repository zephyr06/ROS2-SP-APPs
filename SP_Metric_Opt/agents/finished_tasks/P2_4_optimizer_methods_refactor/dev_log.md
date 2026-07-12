# P2.4 — Optimizer Methods & Mode-String Refactor — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-11

- Task **created (FILED ONLY — no implementation)** per the user's directive: "add a new
  optimizer methods refactor task, we'l fix the optimzier names and implementation
  details for issues like what you just point out."
- **Trigger:** the `INCR_P1` inversion surfaced while answering the user's question
  "does INCR_P1 always call incremental rather than re-optimization?" The answer is the
  *opposite* of the name: `INCR_P1` parses to `ReoptimizationPeriod=1`
  (`tests/RunOrchestrator.cpp:65`), so `count % 1 == 0` is always true
  (`OptimizeSP_TL_Incre.cpp:298-304`) → reopt **every interval**, incremental **never**.
  The `P<n>` knob runs the wrong way for a reader: larger `n` = more incremental, less
  reopt. `INCR_P1` is the max-reopt extreme, not the least. That exposed a cluster of
  naming/implementation-clarity issues — collected here rather than fixed piecemeal.
- **Verified the issue catalogue against current code** (not memory):
  - Read `OptimizeSP_TL_Incre.h` (full method surface) + `OptimizeSP_TL_Incre.cpp:285-359`
    (the dispatcher + `OptimizeIncre_w_TL` + `ReconstructTimeLimitVecFromResOpt`).
  - Read `RunOrchestrator.cpp:1-90` (`MaybeOverrideReoptPeriod` parsing + `--help`).
  - Read `SimulationOrchestrator.cpp:30-42` (`IsINCRPeriodVariant`) + `:293, 313, 328-329`
    (dispatch + the stale `prev_optimizer_` comment).
  - Audited stale P0.5/P1.4 references: `prev_optimizer_` survives at
    `SimulationOrchestrator.cpp:329` + `testIncreOpt_w_TL.cpp:442, 1087` (stale wording,
    correct logic); `ReoptStartFromAdoptedTL` survives as history comments at
    `SimulationOrchestrator.cpp:28`, `RunOrchestrator.cpp:19`, `testIncreOpt_w_TL.cpp:1204`;
    `has_incumbent_` fully gone (P0.5 clean).
  - Gauged the mode-string blast radius: `INCR_P1`/`INCR_SCRATCH`/`INCR_P<n>` appear in
    `evaluation_suite_config.json`, `p25_period_ab_config.json`, `experiment_config.json`
    AND in python (`compare_optimizers.py`, `aggregate_across_tasks.py`,
    `evaluation_suite.py` + their tests + debug scripts). This is the central
    implementation risk — the rename is mechanical but the config↔python↔C++ string
    surface must stay in sync.
  - Confirmed I2: plain `INCR` dispatches identically to `INCR_P<n>`
    (`:293, 313`) but does NOT override `ReoptimizationPeriod`, so `INCR`'s behavior
    depends on the YAML value (`Parameters.cpp:20`) the mode string doesn't surface —
    `INCR` and `INCR_P1` coincide only when YAML sets period=1. The eval-suite config
    lists them as separate arms; they are silently coupled to an invisible knob.
- **Wrote `goal.md`** — the issue catalogue (I1 `INCR_P1` inversion / I2 `INCR`-vs-`INCR_P1`
  silent YAML / I3 stale `prev_optimizer_` comments / I4 stale `ReoptStartFromAdoptedTL`
  history comments / I5 method-name misalignment), the why-P2-not-P1 reasoning, the blast-
  radius-as-gate note, 4 open design decisions (scope slice A/B/C; rename direction
  `REOPT_P<n>` recommended; bare-`INCR` resolution; method renames), Done-when, Out-of-scope.
- **Wrote `tasks.md`** — Step 0 (settle scope + D1–D4, NOT started) through Step 4 (index +
  hand off), sliced A (comments) / B (mode-string rename, the trigger) / C (method
  renames, widest blast radius, defers if conflicts with P1.6/P1.2), with the
  behavior-preserving + fail-loudly-on-stale-name standing constraints.
- **No source code touched.** No `git add`. Folder is `goal.md` + `tasks.md` + this
  `dev_log.md` only. Next action is the user's: pick the slice(s) + the rename direction,
  then greenlight implementation.

## 2026-07-11 (later) — design decisions SETTLED, implementation greenlit

- User settled the design decisions and greenlit implementation. The settled scope:
  - **Slice A (I3/I4):** stale `prev_optimizer_` + `ReoptStartFromAdoptedTL` comment
    rewrites. Shipped this pass.
  - **Slice B (I1/I2):** rename `INCR_P<n>` → `INCR_Reopt_X` where X ∈ {1,5,10,30,60}.
    **X=5 is a NEW arm** (the existing family is {1,10,30,60}); it is added, not renamed.
    `Reopt` (not `RePeriod`) chosen to match the codebase vocabulary (`ReOptimizePeriodic`,
    `ReoptimizationPeriod`, `ReoptimizationTimeLimitSearchPatience`). Old `INCR_P<n>`
    names FAIL LOUDLY — mirror the P1.4 `_ADOPTED` hard-error pattern in
    `MaybeOverrideReoptPeriod`, NOT a silent alias (the P1.3 trap). Bare `INCR` stays
    canonical (= `INCR_Reopt_10` via `parameters.yaml:29 ReoptimizationPeriod: 10`); I2
    resolved by leaving bare INCR as the documented default-period alias and requiring
    explicit `INCR_Reopt_X` for any arm that sweeps the period.
  - **1-arg `ReOptimizePeriodic(int K)` overload REMOVED.** Only production caller was the
    INCR_SCRATCH branch (`SimulationOrchestrator.cpp:335`); the ~11 test/example 1-arg
    call sites migrate to the 2-arg `ReOptimizePeriodic(dag_tasks, K)` form. `dag_tasks`
    is a fixture member (or a local `DAG_Model`) in scope at every site — verified.
    Rationale (user): the signature is bad — `dag_tasks` should be an explicit parameter,
    not the implicit `dag_tasks_` member.
  - **`INCR_SCRATCH` removal filed as a SEPARATE sub-task (Step 5), NOT executed this pass**
    per the user's "we can remove it in a separate step." Step 5 is added to `tasks.md`
    + the goal's Done-when; it is blocked-behind this pass (the dispatch branch, the
    1-arg overload's last production caller, is gone after this pass makes Step 5 cheaper,
    but Step 5 itself is its own review cycle). Note for Step 5: it is RESULT-CHANGING
    (deletes a scheduler arm + the E2 gate + north-star line 10) — NOT behavior-preserving,
    so it is flagged separately from the rename.
- **Method renames (Slice C / I5): NOT in this pass.** The user's "rename methods as
  planned" refers to the overload removal + the comment rewrites, NOT the wider I5 method-
  name sweep (e.g. `ReOptimizePeriodic` → split-bootstrap, `_w_TL` suffix collapse). I5
  stays deferred — it conflicts with P1.6/P1.2's in-flight edits to
  `OptimizeSP_TL_Incre`, and the user named only the overload removal specifically.
  Re-flagged in `goal.md` Out-of-scope.
- **Standing constraints honored:** behavior-preserving for Slices A+B + the overload
  removal (no test *expectation* changes — only symbol names + the new `INCR_Reopt_5`
  arm added to configs); no `git commit` (`git add` only); the user runs the A/B suite.
- **TDD ordering:** (1) add the FAIL-LOUDLY test for stale `INCR_P<n>` + the new-name
  parse test FIRST (red); (2) implement the rename in `MaybeOverrideReoptPeriod` +
  `IsINCRPeriodVariant` + dispatch (green); (3) migrate configs + python; (4) remove the
  1-arg overload + migrate call sites; (5) Slice A comment rewrites; (6) build DEBUG +
  `check.SP_OPT -j5` + 16/16 ctest green.

## 2026-07-11 (implementation) — Slices A+B + overload migration COMPLETE, TDD green

- Resumed after a context-limit break with the C++/config/python rename + the 1-arg→2-arg
  test migration already in the working tree (the greenlit pass). Audited what remained:
  the python E3-gate test fixtures still used the retired `INCR_P<n>` names → 6 tests RED;
  one debug script + two cosmetic docstring lines pending; build/ctest + logs + `git add`
  pending. Slice C (I5 method renames) stays DEFERRED; `INCR_SCRATCH` removal stays the
  separate Step 5 (not this pass).
- **Python TDD red→green (the substantive remaining step).** Ran
  `tests/python/test_evaluation_suite.py` → 6 RED (`TestGateE3` ×4,
  `TestEvaluateAllGates::test_all_pass`, `TestEndToEndMain::test_main_pass_exit_zero`):
  the E3 gate now expects `INCR_Reopt_X` keys (`DEFAULT_PERIOD_ARMS` was updated to the
  5-arm family) but the lookup fixtures still used `INCR_P<n>` → "missing" failures.
  Migrated every fixture to the 5-arm `INCR_Reopt_{1,5,10,30,60}` family (added the
  `INCR_Reopt_5` column everywhere — monotonic non-increasing ET so the PASS cases stay
  PASS), updated the `test_fail_inversion` assertion to `"INCR_Reopt_1->INCR_Reopt_5"`
  (the gate's `detail` string is built from the arm names), and the
  `test_missing_arm_fails_not_crashes` assertion to `INCR_Reopt_30`. After:
  `test_evaluation_suite.py` 30 passed (was 24 passed / 6 failed); full `tests/python/`
  suite 291 passed. Behavior-preserving — only arm *names* + the new `INCR_Reopt_5`
  fixture column changed; no gate logic touched.
- **Debug script** `simulation_experiments/repro_et_grows_with_period.py`: renamed the
  `ARMS` list (added `INCR_Reopt_5`), the `--arms` help example, and the Context docstring
  from `INCR_P<n>` → `INCR_Reopt_X`. The `ARMS` list is also the `--arms` validation set,
  so the rename keeps a stale `INCR_P<n>` `--arms` value from silently passing.
- **Cosmetic docstrings** in `simulation_experiments/evaluation_suite.py`: two prose lines
  (`evaluate_e3` docstring `:315` + the `period_arms` param doc `:385`) still said
  `INCR_P<n>` → `INCR_Reopt_X`. Left the `:79` "renamed from the retired INCR_P<n> form"
  history comment as-is (intentional).
- **Slice A confirmation:** the two remaining `testIncreOpt_w_TL.cpp` mentions are
  intentional historical contrasts, NOT stale wording — `:1087` "Under the old
  `prev_optimizer_` design ... under the redesign there is no stored DAG" and `:1204`
  "P1.4 removed the `ReoptStartFromAdoptedTL` knob". Both read as history (the I3/I4
  scope); no edit. `:442` was already rewritten to `res_opt_`/`IfInitialized()`.
- **Overload migration (Step 3):** the ~11 test/example 1-arg `ReOptimizePeriodic(K)` call
  sites are migrated to the 2-arg `ReOptimizePeriodic(dag_tasks, K)` form. The 1-arg
  overload declaration/def STAYS (decision (a)) — its only remaining caller is the
  INCR_SCRATCH branch, which Step 5 removes together with the overload. Verified at build.
- **Build + ctest (DEBUG):** `cmake --build build --target check.SP_OPT -j5` (DEBUG;
  `build/CMAKE_BUILD_TYPE=DEBUG`, `libSP_OPTDebug.so`) → builds clean, **16/16 ctest
  green** incl. `testIncreOpt_w_TL` (3.73s) + `testOptimizeIncrePA`. No test *expectation*
  changed (the 1-arg body was `return ReOptimizePeriodic(dag_tasks_, K);`).
- **`grep` sweep:** `grep -rn "INCR_P" --include=*.py --include=*.json simulation_experiments/
  tests/` (excl. `INCR_PURE`) → only the intentional `evaluation_suite.py:79` history
  comment remains. `compare_optimizers.py` / `aggregate_across_tasks.py` /
  `run_radius_comparison.py` only reference the ablation group (INCR/INCR_NO_TL/
  INCR_WCET/INCR_SCRATCH), never the period arms — no rename needed there.
- **Standing constraints honored:** behavior-preserving (Slices A+B + overload migration);
  no `git commit` (`git add` only); the user runs the A/B suite. Step 5 (`INCR_SCRATCH`
  removal) is result-changing and stays a separate sub-task — NOT staged here.


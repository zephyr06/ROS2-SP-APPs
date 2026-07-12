# P2.5 — Remove the `INCR_SCRATCH` scheduler arm (result-changing cleanup)

**Priority:** P2 (should-do hygiene — but **result-changing**, hence its own review
cycle, NOT part of the behavior-preserving P2.4 pass).
**Status:** IMPLEMENTATION IN PROGRESS 2026-07-11 (spawned from P2.4 Step 5, which
filed this as a separate sub-task).
**Parent:** P2.4 ([`active_tasks/P2_4_optimizer_methods_refactor/`](../P2_4_optimizer_methods_refactor/))
Step 5. P2.4 shipped the behavior-preserving rename `INCR_P<n>` → `INCR_Reopt_X`
(Slices A+B + the 1-arg overload *test-site* migration); P2.5 is the result-changing
remainder — deleting the `INCR_SCRATCH` arm + its gate + the now-dead 1-arg
`ReOptimizePeriodic(int K)` overload.

## What `INCR_SCRATCH` is

`INCR_SCRATCH` is an **ablation control** (not a paper-advocated scheduler). It is
reopt-every-interval like `INCR_Reopt_1`, but **amnesiac**: each interval it constructs
a *fresh* `scratch_opt` (`SimulationOrchestrator.cpp` INCR_SCRATCH branch) and discards
it, so no incumbent is carried across intervals → `ResetIncumbentBaseline` takes its
interval-0 branch (RM + min-TL) every call → the compare-and-keep guard measures the
search against a *synthetic RM baseline*, NOT against the prior interval's adopted
solution.

It exists to **isolate the value of carrying the incumbent forward** — the no-memory
counterpart to `INCR_Reopt_1` (which carries `res_opt_` and compares against the
running best). P2.4's settled finding: **`INCR_Reopt_1` weakly dominates `INCR_SCRATCH`
in SP** (the carried incumbent is the better compare-and-keep baseline AND the better
descent seed), so `INCR_SCRATCH` never wins on SP and only muddies the baseline table.

## Why remove it

Per the user's directive ("add a related task, and remove it as planned" — 2026-07-11,
continuing the P2.4 session). It was the one result-changing piece P2.4 deliberately
deferred:

1. **It is result-changing** (deletes a scheduler arm + collapses a gate) — so it could
   not ride with P2.4's behavior-preserving rename pass.
2. **P1.6 supersedes its scientific role.** P1.6's new pure-incremental arm
   (`INCR_PURE`) keeps `INCR_Reopt_1`'s incumbent-carrying but drops the from-scratch
   descent — a cleaner "does the descent earn its cost?" probe. The amnesia axis
   (`INCR_Reopt_1`-carries-vs-`INCR_SCRATCH`-amnesiac) is subsumed: P1.6 isolates the
   same value (warm-start) by *dropping* the descent rather than by *amnesia*.
3. **Paper-legibility.** A baseline table that reads `INCR_Reopt_1 ≥ INCR_SCRATCH`
   (weak dominance) invites a reviewer to ask why the dominated arm is there at all.
   Removing it leaves the `INCR_Reopt_X` family + `INCR_PURE` as a clean monotone story.

## The removal surface (FULLY mapped 2026-07-11 — broader than P2.4 Step 5 stated)

P2.4 Step 5's notes named only "the E2 gate + Q2/Q3/E1 tuples." The verified surface
is wider — **`INCR_SCRATCH` is a co-equal subject in FIVE of the six north-star gates**,
not just E2. Every gate that reads "INCR & SCRATCH..." must collapse to INCR-only, and
E2 (whose only subject pair *is* INCR-vs-SCRATCH) is deleted outright.

### C++ (the arm itself)
- `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp`
  - the `INCR_SCRATCH` **dispatch branch** in `DeterminePrioritiesAndBudgets`
    (the `else if (scheduler_mode_ == "INCR_SCRATCH")` block that builds a fresh
    `scratch_opt` + calls the 1-arg `ReOptimizePeriodic`)
  - the `INCR_SCRATCH` string in the **construction condition** (the
    `if (... || scheduler_mode_ == "INCR_SCRATCH")` guard that builds `incr_optimizer_`)
  - the construction-condition **comment** that explains why INCR_SCRATCH is listed
    (it can be deleted — the comment only exists to justify the now-removed line)
- `sources/Optimization/OptimizeSP_TL_Incre.h` — the **1-arg `ReOptimizePeriodic(int K)`
  overload** declaration + its multi-line comment. P2.4 Step 3 kept this overload
  specifically because the INCR_SCRATCH branch was its only production caller; with that
  branch gone, the overload is dead and is removed here.
- `sources/Optimization/OptimizeSP_TL_Incre.cpp` — the 1-arg overload **definition**
  (`return ReOptimizePeriodic(dag_tasks_, K);`).
- `tests/RunOrchestrator.cpp` — the `--help` Modes line lists `INCR_SCRATCH`; drop it.

### C++ test gap P2.4 Step 3 missed (fix here)
- `tests/testIncreOpt_w_TL.cpp:428` — `opt_scratch.ReOptimizePeriodic(2)` is STILL a
  **1-arg call**. P2.4's migration summary claimed "×5 sites in testIncreOpt_w_TL
  migrated" but missed `:428` (it is inside `OptimizeWithOptimizationSpace`, named
  `opt_scratch`). **This must migrate to `ReOptimizePeriodic(dag_tasks, 2)` before the
  1-arg overload can be deleted** — else the build breaks. The `:1206` / `:1253`
  mentions are *comments* describing the interval-0 fallback that `INCR_SCRATCH`
  exercises; the fallback itself is `IfInitialized()`-gated and tested independently
  (`ReOptimizePeriodic_Interval0FallsBackToGaussianMean` at `:1258` uses a fresh local
  optimizer, NOT the INCR_SCRATCH dispatch). Those comments are reworded, the test stays.

### Configs (the arm in A/B lists)
- `simulation_experiments/configs/evaluation_suite_config.json` — `INCR_SCRATCH` in
  both `ablation_scheduler_list`s + the `_comment` (the union-of-schedulers prose).
- `simulation_experiments/configs/experiment_config.json` — both
  `ablation_scheduler_list`s.
- `simulation_experiments/configs/p25_period_ab_config.json` — `INCR_SCRATCH` in
  `main_scheduler_list` (both smoke + full) + the `_comment` (the arm-count prose:
  "7 arms (5 plain INCR_Reopt_X + BF + INCR_SCRATCH)" → "6 arms").

### Python — gate code (the five gates)
- `simulation_experiments/evaluation_suite.py`
  - the `SCRATCH = "INCR_SCRATCH"` constant + the naming comment
  - **Q1** (`evaluate_q1`): drops SCRATCH from the gap-to-BF pair → BF-vs-INCR only
  - **Q2** (`evaluate_q2`): drops SCRATCH from the ≥-BF pair → INCR-only
  - **Q3** (`evaluate_q3`): drops SCRATCH from the ≥-baselines pair → INCR-only
  - **E1** (`evaluate_e1`): drops SCRATCH from the overhead pair → INCR-only
  - **E2** (`evaluate_e2`): **DELETED** — its only subject pair is INCR-vs-SCRATCH, so
    with SCRATCH gone the gate is vacuous. Per the P2.4 plan: "Do NOT substitute
    INCR_Reopt_1 for INCR_SCRATCH in E2 — that just re-states E3" (once SCRATCH→Reopt_1
    and bare INCR=Reopt_10, `ET(Reopt_10) <= ET(Reopt_1)` is the first edge of E3).
    The `evaluate_e2` call in `evaluate_all_gates` + the `e2_ns` line are removed too.

### Python — non-gate
- `simulation_experiments/aggregate_across_tasks.py` — the ablation group constant
  (`["BF", "INCR", "INCR_NO_TL", "INCR_WCET", "INCR_SCRATCH"]`) + its docstring.
- `simulation_experiments/compare_optimizers.py` — the ablation list.
- `simulation_experiments/repro_et_grows_with_period.py` — the `ARMS` list (drops
  SCRATCH; this is also the `--arms` validation set, so a stale `INCR_SCRATCH`
  `--arms` value will fail loudly post-removal).
- `tests/debug_analysis/run_e2e_eval.py` + `tests/debug_analysis/run_radius_comparison.py`
  — ablation lists.
- `tests/python/test_evaluation_suite.py` — the ~30 SCRATCH fixture keys across Q1/Q2/
  Q3/E1/E2 + the E2 test class (`TestGateE2` / `test_e2_*`). The E2 tests are deleted;
  the Q1/Q2/Q3/E1 fixtures drop the SCRATCH row.
- `tests/python/test_aggregate.py` / `test_run_end_to_end.py` / `test_compare_optimizers.py`
  — ablation-list fixtures.

### Docs
- `agents/project_evaluation_northstar.md` — line 10 ("INCR cannot run slower than
  SCRATCH" = E2) + the SCRATCH mentions in lines 3-5 (the SP-quality bullets).
- `agents/plan_publication_figures.md` — Ab-A / Ab-B list "INCR, INCR_NO_TL, INCR_WCET,
  INCR_SCRATCH" → drop SCRATCH.
- `agents/active_tasks/P1_6_incr_only_baseline/goal.md` (+ `tasks.md`) — P1.6 named
  `INCR_SCRATCH` as the warm-start-value contrast arm ("vs `INCR_SCRATCH` isolates the
  value of carrying the incumbent"). Removing it narrows P1.6 to the descent-cost axis
  only (`INCR_PURE` vs `INCR_Reopt_1`). The user accepted this narrowing in the P2.4
  session.
- `agents/active_tasks/P0_3_prod_figure_run/{goal,tasks}.md` — the P25 figure plots ET
  vs period "with `INCR_SCRATCH`" floor; drop the SCRATCH floor.
- `agents/active_tasks/P2_4_optimizer_methods_refactor/tasks.md` — Step 5 checkbox flips
  to done with a pointer to P2.5.

### Intentionally NOT touched (history / investigation records)
- `agents/finished_tasks/` (P0_5, P24, summary) — historical records of when
  `INCR_SCRATCH` existed; rewriting history is wrong.
- `agents/investigation/debug_runtime0704_incr.md` + `runtime_profiling_guide.md` —
  P1.1 investigation record; SCRATCH was a probe arm there.
- `agents/agent_communication/{kimi,gemini}.md` + `agents/claude_sessions/` —
  historical debate / session logs.
- `agents/active_tasks/P1_{2,3,4}_*/` — these reference `INCR_SCRATCH` as the
  interval-0/amnesiac fallback exemplar in their *reasoning*; the mechanism
  (`IfInitialized()` fallback) survives the removal (it's exercised by any fresh
  optimizer, see the `:1258` test), so these need at most a one-line "SCRATCH removed
  P2.5" pointer if a reader would otherwise go looking for the arm. Deferred — not in
  this pass unless a grep shows a live (not historical) dependency.

## Design decisions (SETTLED 2026-07-11)

1. **Scope = full removal** (not a flag-gated retirement). Delete the arm from C++,
   configs, python, gates, docs. No `INCR_SCRATCH`-as-alias shim. A stale
   `INCR_SCRATCH` config will fail loudly (the dispatch falls through to the RM
   baseline, mirroring the P1.4 `_ADOPTED` / P2.4 `INCR_P<n>` hard-error pattern) —
   *not* silently aliased to `INCR_Reopt_1`.
2. **E2 is deleted, not repurposed.** Substituting `INCR_Reopt_1` for `INCR_SCRATCH`
   would make E2 = `ET(INCR_Reopt_10) <= ET(INCR_Reopt_1)`, which is exactly the first
   edge of E3 — a redundant gate. Delete it. The north-star drops from 6 gates to 5.
3. **Q1/Q2/Q3/E1 collapse to INCR-only** (not "INCR + a replacement"). SCRATCH was a
   co-equal subject; with it gone the gate checks INCR alone against BF / the baselines.
4. **The 1-arg `ReOptimizePeriodic(int K)` overload is deleted with the branch.** P2.4
   Step 3 kept it only because the INCR_SCRATCH branch was its last production caller.
   The one residual 1-arg *test* caller (`testIncreOpt_w_TL.cpp:428`) migrates to the
   2-arg form first.
5. **`INCR_Reopt_1` is NOT renamed/relabeled to absorb SCRATCH's role.** The
   amnesia axis is dropped entirely (P1.6 isolates warm-start value a cleaner way);
   `INCR_Reopt_1` stays the always-reopt-with-memory arm.
6. **History docs are not rewritten.** `finished_tasks/`, `investigation/`,
   `agent_communication/`, `claude_sessions/` keep their `INCR_SCRATCH` mentions as the
   historical record. Only *live* surfaces (code, configs, gates, active-task goals)
   are updated.

## Done when

- [ ] C++: INCR_SCRATCH dispatch branch + construction condition + comment deleted;
      1-arg `ReOptimizePeriodic(int K)` overload (decl + def) deleted;
      `testIncreOpt_w_TL.cpp:428` migrated to 2-arg; `RunOrchestrator.cpp --help` Modes
      line drops INCR_SCRATCH.
- [ ] Gate code: `SCRATCH` constant + naming comment gone; Q1/Q2/Q3/E1 collapsed to
      INCR-only; `evaluate_e2` + its call + `e2_ns` deleted.
- [ ] Configs: `INCR_SCRATCH` dropped from all 3 configs' lists + comments.
- [ ] Python non-gate: ablation lists in `aggregate_across_tasks.py`,
      `compare_optimizers.py`, `repro_et_grows_with_period.py`, `run_e2e_eval.py`,
      `run_radius_comparison.py` drop SCRATCH.
- [ ] Docs: northstar (line 10 + 3-5), `plan_publication_figures.md` (Ab-A/Ab-B),
      P1.6 goal/tasks, P0.3 goal/tasks, P2.4 tasks.md Step 5 updated.
- [ ] Tests: `test_evaluation_suite.py` E2 class deleted + Q1/Q2/Q3/E1 fixtures drop
      SCRATCH; `test_aggregate.py` / `test_run_end_to_end.py` /
      `test_compare_optimizers.py` fixtures drop SCRATCH. TDD red→green.
- [ ] `cmake --build build --target check.SP_OPT -j5` (DEBUG) green; 16/16 ctest green.
- [ ] `python -m pytest tests/python/` green.
- [ ] `git add` staged; user reviews (no commit — standing constraint).
- [ ] `agents/overall_tasks.md` + top-level `agents/dev_log.md` updated.

## Out of scope

- `git commit` — user's standing constraint (`git add` only).
- Re-litigating P1.6's design (P1.6 keeps its `INCR_PURE` arm + its narrowed
  descent-cost axis; this task only updates P1.6's docs to reflect SCRATCH's removal).
- The `INCR_Reopt_X` family (P2.4, shipped + green) — untouched.
- History docs (`finished_tasks/`, `investigation/`, `agent_communication/`,
  `claude_sessions/`) — kept as the historical record.
- Slice C (P2.4 I5 method renames) — still deferred (P1.6/P1.2 in-flight).

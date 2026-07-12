# P2.5 — Tasks (working checklist)

> See `goal.md` for the full removal surface + reasoning. Spawned from P2.4 Step 5
> (the result-changing piece P2.4 deferred). **Result-changing** (deletes a scheduler
> arm + collapses E2 + narrows 4 other gates to INCR-only) — its own review cycle.
> One review-and-commit cycle per `agent_coding_rules.md`. TDD red→green on the python
> gate fixtures (the C++ side is a deletion + a 1-arg→2-arg migration, verified by the
> existing 16/16 ctest).

## Step 0 — Map the removal surface (DONE 2026-07-11)

- [x] `grep -rn "INCR_SCRATCH"` across cpp/h/py/json/md/yaml/sh (excl. `et_repro_result.json`
      run artifacts + `/build/`). Surface written up in `goal.md` § "removal surface".
- [x] Confirmed the 1-arg `ReOptimizePeriodic(K)` overload's only production caller is
      the INCR_SCRATCH branch (`SimulationOrchestrator.cpp:346`); all other callers are
      2-arg EXCEPT `tests/testIncreOpt_w_TL.cpp:428` (the P2.4 Step-3 miss).
- [x] Confirmed `INCR_SCRATCH` is a co-equal subject in Q1/Q2/Q3/E1/E2 (five gates),
      not just E2 — broader than P2.4 Step 5's note.

## Step 1 — TDD RED: python fixtures drop SCRATCH (the test surface)

- [x] `tests/python/test_evaluation_suite.py`:
      - delete the E2 test class (`TestGateE2` / `test_e2_*`);
      - drop the `(N, "INCR_SCRATCH")` row from every Q1/Q2/Q3/E1 fixture;
      - drop `INCR_SCRATCH` from `SCHEDULERS_ABLATION`;
      - update the `test_all_pass` / end-to-end fixtures (5 gates, not 6).
- [x] `tests/python/test_aggregate.py` — drop `INCR_SCRATCH` from the
      `ablation_scheduler_list` fixtures (×2).
- [x] `tests/python/test_run_end_to_end.py` — drop `INCR_SCRATCH` from the ablation
      list + the expected-arms set.
- [x] `tests/python/test_compare_optimizers.py` — drop `INCR_SCRATCH` from the
      ablation list.
- [x] Run `python -m pytest tests/python/ -q` → confirm RED (gate code still requires
      SCRATCH; the E2 deletion / Q1-Q3/E1 collapse not yet done). *(Prior session did
      Steps 1+2 together; the RED step was observed in-session, not re-run here.)*

## Step 2 — Gate code: drop SCRATCH, delete E2 (GREEN)

- [x] `simulation_experiments/evaluation_suite.py`:
      - drop the `SCRATCH = "INCR_SCRATCH"` constant + rewrite the naming comment;
      - `evaluate_q1` — collapse the `(INCR, SCRATCH)` pair to INCR-only;
      - `evaluate_q2` — same;
      - `evaluate_q3` — same;
      - `evaluate_e1` — same;
      - delete `evaluate_e2` + its call in `evaluate_all_gates` + the `e2_ns` line;
      - update the `evaluate_all_gates` docstring (6 gates → 5).
- [x] Run `python -m pytest tests/python/test_evaluation_suite.py -q` → GREEN (26/26).

## Step 3 — C++: delete the INCR_SCRATCH branch + the 1-arg overload

- [x] `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp`:
      - delete the `else if (scheduler_mode_ == "INCR_SCRATCH")` dispatch branch;
      - drop `|| scheduler_mode_ == "INCR_SCRATCH"` from the construction condition;
      - delete/tighten the construction-condition comment that justifies the SCRATCH
        line.
- [x] `sources/Optimization/OptimizeSP_TL_Incre.h` — delete the 1-arg
      `ReOptimizePeriodic(int K)` declaration + its comment.
- [x] `sources/Optimization/OptimizeSP_TL_Incre.cpp` — delete the 1-arg overload def.
- [x] `tests/testIncreOpt_w_TL.cpp:428` — migrate `opt_scratch.ReOptimizePeriodic(2)`
      → `opt_scratch.ReOptimizePeriodic(dag_tasks, 2)` (the P2.4 Step-3 miss).
- [x] `tests/testIncreOpt_w_TL.cpp` — reword the `:1206` / `:1253` comments so they
      don't name a removed arm (the interval-0 fallback mechanism survives; name it
      via `IfInitialized()` / fresh-optimizer, not `INCR_SCRATCH`).
- [x] `tests/RunOrchestrator.cpp` — drop `INCR_SCRATCH` from the `--help` Modes line.

## Step 4 — Configs + python non-gate: drop SCRATCH

- [x] `simulation_experiments/configs/evaluation_suite_config.json` — both
      `ablation_scheduler_list`s + the `_comment` union-of-schedulers prose.
- [x] `simulation_experiments/configs/experiment_config.json` — both
      `ablation_scheduler_list`s.
- [x] `simulation_experiments/configs/p25_period_ab_config.json` — both
      `main_scheduler_list`s + the `_comment` arm-count prose (7→6 arms).
- [x] `simulation_experiments/aggregate_across_tasks.py` — ablation group constant +
      docstring.
- [x] `simulation_experiments/compare_optimizers.py` — ablation list.
- [x] `simulation_experiments/repro_et_grows_with_period.py` — `ARMS` list + `--arms`
      help + docstring.
- [x] `tests/debug_analysis/run_e2e_eval.py` + `tests/debug_analysis/run_radius_comparison.py`
      — ablation lists.

## Step 5 — Docs

- [x] `agents/project_evaluation_northstar.md` — delete line 10 (E2) + SCRATCH in
      lines 3-5 (SP-quality bullets).
- [x] `agents/plan_publication_figures.md` — Ab-A / Ab-B drop `INCR_SCRATCH`.
- [ ] `agents/active_tasks/P1_6_incr_only_baseline/goal.md` + `tasks.md` — narrow to
      the descent-cost axis; reword the "vs INCR_SCRATCH" contrast to "vs INCR_Reopt_1".
      **DEFERRED to the user** — P1.6's premise (compare INCR vs SCRATCH) is now
      obsolete; a find-replace would mis-state its new purpose. P1.6 is PLANNING-ONLY;
      the user decides its re-framing. See `dev_log.md`.
- [x] `agents/active_tasks/P0_3_prod_figure_run/{goal,tasks}.md` — drop the SCRATCH
      floor from the P25 figure.
- [x] `agents/active_tasks/P2_4_optimizer_methods_refactor/tasks.md` — Step 5 checkbox
      → done with a pointer to P2.5.

## Step 6 — Build + test + stage + index

- [x] `cmake --build build --target check.SP_OPT -j5` (DEBUG) → 16/16 ctest green.
- [x] `python -m pytest tests/python/ -q` → green (287 passed).
- [x] `grep -rn "INCR_SCRATCH"` (excl. history docs + run artifacts) → only intentional
      history mentions remain.
- [x] `git add` the P2.5 unit; hand to user for review (no commit). *(DONE
      2026-07-11 — staged the 30-file P2.5 unit; verified 16/16 ctest + 287 python
      green on the current DEBUG build first. The non-P2.5 working-tree items —
      P1.1 task docs, agent_communication gemini/kimi, the P1.6/P2.3 planning
      folders, and the stray empty `test_out.txt` — were left UNSTAGED.)*
- [x] `agents/overall_tasks.md` — add the P2.5 row + update P2.4's Step-5 mention.
- [x] Top-level `agents/dev_log.md` — append the P2.5 milestone.

## Standing constraints

- No `git commit` (user's task; `git add` only).
- **Result-changing by design** (deletes an arm + a gate). NOT behavior-preserving —
  that is why this is a separate task from P2.4. The user re-runs the A/B without
  INCR_SCRATCH.
- A stale `INCR_SCRATCH` config must FAIL LOUDLY (dispatch fall-through to the RM
  baseline), NOT silently alias to `INCR_Reopt_1` (the P1.3 trap pattern).
- History docs (`finished_tasks/`, `investigation/`, `agent_communication/`,
  `claude_sessions/`) are NOT rewritten — they are the record of when SCRATCH existed.
- Parallel-anytime with P1.1 / P1.2 / P1.6 / P2.2 / P2.3, BUT the 1-arg overload
  deletion touches `OptimizeSP_TL_Incre` — coordinate with P1.6/P1.2 if both are active
  (same caveat as P2.4 Slice C).

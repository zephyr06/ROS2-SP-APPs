# P0.9 — Tasks (working checklist)

> See `goal.md` for scope (system-wide DM + important-first group lock on the seed
> PA) + design decisions D1–D6. Blocks P0.6 + P0.8 (both certify/seed against this
> order). Work by module, commit by module (agents `git add` only).

## 0. Design decisions (ALL RESOLVED 2026-07-28)
- [x] D1 RESOLVED: sort key = deadline (DM), not period (RM).
- [x] D2 RESOLVED: important-first group lock — important top, DM-ordered within
      group; non-important bottom, DM-ordered within group.
- [x] D3 RESOLVED: scope = C++ seed PA + Python RTA sort + P0.6 plan refs +
      orchestrator baseline arms (D7).
- [x] D4 RESOLVED: **rename to DM** — `RateMonotonicPriorityVec`→
      `DeadlineMonotonicPriorityVec`, `SeedIncumbentFromRMFast`→
      `SeedIncumbentFromDMFast`, `BootstrapIncumbentFromRMFast`→
      `BootstrapIncumbentFromDMFast`; orchestrator arms `"RM"`→`"DM"`,
      `"RM_FAST"`→`"DM_FAST"`, `"RM_SLOW"`→`"DM_SLOW"`.
- [x] D5 RESOLVED: DM tie-break = avg ET ascending.
- [x] D6 RESOLVED: accept behavior change; verify "schedulability improves/holds +
      SP within tolerance", NOT bit-identical. Prod A/B proposed separately.
- [x] D7 RESOLVED: FULL system-wide — also switch orchestrator baseline arms
      (`RM`/`RM_FAST`/`RM_SLOW` → `DM`/`DM_FAST`/`DM_SLOW`) + their sort to
      deadline. Cascades into configs + result/figure scripts + tests.

## 1. C++ optimizer seed PA — important-first group-locked DM + rename — LANDED
- [x] TDD red: replaced the two `RateMonotonicPriorityVec` period-sort tests
      (`testIncreOpt_w_TL.cpp:760,782`) with discriminating DM+group-lock tests
      (`DeadlineMonotonicPriorityVec.RanksImportantGroupFirstThenDmOrdersEach`,
      `...BreaksDeadlineTiesByExecutionTimeAscending`). Construct tasksets where
      `D≠T` AND important tasks are NOT already top → assert important group
      first (DM-ordered within), then non-important (DM-ordered within). RED
      under old period-sort, GREEN under deadline sort + group lock.
- [x] Renamed `RateMonotonicPriorityVec`→`DeadlineMonotonicPriorityVec`,
      `SeedIncumbentFromRMFast`→`SeedIncumbentFromDMFast`,
      `BootstrapIncumbentFromRMFast`→`BootstrapIncumbentFromDMFast` across
      `OptimizeSP_TL_Incre.{h,cpp}` + call sites in `testIncreOpt_w_TL.cpp`.
- [x] Implemented (was already in place in `.cpp`): sort partitions by
      `is_important`, sorts each group by deadline asc (ties avg ET asc), concats
      [important] ++ [non-important].
- [x] Updated the call sites (`:932,962,1019` — formerly 913/936/993) + the
      `Interval0UsesRMAndMinTL`→`Interval0UsesDMAndMinTL` test name + P3.6
      "RM-fast"→"DM-fast" comments.
- [x] TDD green: `cmake --build build_test --target check.SP_OPT -j5` = 17/17
      passed (DEBUG).

## 2. Orchestrator baseline arms — RM→DM strings + deadline sort — C++ + Python/configs LANDED
- [x] `SimulationOrchestrator.cpp:356-401`: `"RM"`→`"DM"`, `"RM_FAST"`→`"DM_FAST"`,
      `"RM_SLOW"`→`"DM_SLOW"`; each arm's sort `period`→`deadline`. Also the `:33`
      fall-through comment ("RM baseline"→"DM baseline").
- [x] `:321` comment: mode list `RM / RM_FAST / RM_SLOW` → `DM / DM_FAST / DM_SLOW`.
- [x] `RunOrchestrator.cpp`: 4 "RM baseline" comments → "DM baseline"; the `Modes:`
      help line `RM / ... / RM_FAST, RM_SLOW` → `DM / ... / DM_FAST, DM_SLOW`.
- [x] `tests/testScheduleSimulate.cpp`: every `"RM"`/`"RM_FAST"`/`"RM_SLOW"` mode
      string → `DM`/`DM_FAST`/`DM_SLOW`; the `/RM/` export-folder paths → `/DM/`
      (ExportResults writes into a folder named for the mode, so the path MUST
      follow the rename); the 2 `..._RM_FAST`/`..._RM_SLOW` test names → `..._DM_*`;
      the `RateMonotonicPriorityAssignment` test → `DeadlineMonotonicPriority...`;
      comments RM→DM. Fixture has `deadline==period`, so DM and RM agree on
      ordering — assertions unchanged.
- [x] **Latent bug exposed + fixed:** `ExactResponseTimeValidation` parsed 7 cols
      (`...execution,overrun`) but the P2.14 writer drops `is_overrun` → 6 cols;
      the test only passed by reading STALE June-21 `RM/response_times_task_*.txt`
      (never cleared the dir, never set EXPORT_DETAIL_LEVEL=3). Fixed: set level 3
      + `remove_all` the output dir (self-contained, no stale artifacts) + dropped
      the `overrun` field from both parse sites.
- [x] `cmake --build build_test --target check.SP_OPT -j5` = 17/17 green.
- [x] **Config + Python cascade LANDED (Step 2 D7):** renamed `RM_FAST`→`DM_FAST`,
      `RM_SLOW`→`DM_SLOW`, and bare `"RM"`→`"DM"` (real mode lists only; the bare
      `"RM"` arm was already retired in favor of RM_FAST+RM_SLOW per
      `evaluation_suite.py:122`) across:
  - **3 active JSON configs** (`paper_simulation_config.json` test+prod
    `main_scheduler_list` + 2 prose comments, `compare_against_bf.json`,
    `incr_et_profiling.json`). `p211_reopt_ab_config.json` LEFT — it's a retained
    historical record (prose-only, "do not delete"; describes a past A/B run's
    gate needs at the time — editing would falsify the record).
  - **6 `simulation_experiments/` scripts** (`run_sim_experiments.py`,
    `compare_optimizers.py`, `reanalyze_results.py`, `interval_sweep.py`×2,
    `evaluation_suite.py` incl. the `:122` "bare RM replaced by RM_FAST+RM_SLOW"
    narrative → DM + `Q3_BASELINES`, `aggregate_across_tasks.py`×3).
  - **3 `simulation_experiments/debug_analysis/` + 3 `tests/debug_analysis/`
    scripts** (DISTINCT file copies, not symlinks — both dirs renamed:
    `analyze_incr_vs_br_vary_k.py`, `run_radius_comparison.py`, `run_e2e_eval.py`).
  - **`Visualize_SP_Metric/visualize_SP_distribution.py:112`** default
    `scheduler_name="RM"`→`"DM"` (builds a results-folder path named for the mode).
    `draw_trajectory_error.py:16` LEFT — `"RM"` there is a SLAM-trajectory data-file
    dict key (`CameraTrajectory_rm.txt`, a real Jan-2025 artifact on disk), NOT a
    scheduler-arm dispatch; renaming would orphan the data path.
  - **5 `tests/python/` test fixtures** — synthetic `"RM"` labels + the
    `test_experiment_config_loader.py:173` config-pin assertion (coupled to the
    renamed config) + `test_compare_optimizers.py:79` `ALL_SCHEDULERS` assertion
    (coupled to the renamed `compare_optimizers.py`). Full D7 consistency
    (user-chose): renamed synthetic labels too, data+assertions together.
  - **STALE result outputs LEFT** (D6: breaks result-comparability, accepted):
    `tests/radius_comparison/`, `tests/comparison_runs/`, `tests/e2e_eval_runs/`,
    `tests/k_variation/`, `simulation_experiments/optimizer_comparison/runs/` —
    historical run CSV/JSON with RM labels, left for the prod A/B re-run (user-go).
- [x] `pytest Gen_Taskset/tests/` = 43 passed; `pytest tests/python/` = 351/353
      (2 fails PRE-EXISTING — `compare_against_bf.json time_limit_seconds=10` vs
      expected `1`; proven via git-diff untouched by P0.9, = the P2.14-known
      pre-existing config failures); `check.SP_OPT` = 17/17 green.

## 3. Python RTA sort key — deadline — LANDED
- [x] TDD red: replaced the "RM-ordering invariance" test with the discriminating
      `test_dm_ordering_is_deadline_based_not_period_based` — constructs a case
      where DM and RM orderings DIVERGE (t_a: short deadline/long period; t_b:
      short period/long deadline) and asserts the RTA ranks by DEADLINE (DM→
      schedulable; the old period-sort → unschedulable). RED under old period
      sort, GREEN under deadline sort.
- [x] Redesigned the unschedulable / boundary / per-core-isolation tests for DM
      ordering (HP task keeps the shortest deadline so DM and RM agree there,
      validating correctness without flipping on the sort change).
- [x] Implemented: `_important_priority_order` (`important_task_rta.py`) sort key
      `period` → `deadline`.
- [x] Updated docstrings in `important_task_rta.py` (module + `_important_
      priority_order` + `important_tasks_schedulable` + the call-site comments)
      RM → DM framing. (Recurrence's own `period` refs are LEGITIMATE —
      interference is by inter-arrival period; only the priority RANK uses
      deadline.)
- [x] TDD green: `pytest Gen_Taskset/tests/` = 43 passed (10 RTA + 33 others; no
      regressions).

## 4. Records — P0.6/P0.8 plan refs RM → DM
- [ ] `P0_6_static_solution/goal.md`: "RM-grouped"→"DM-grouped", "RM-ordered within
      the group"→"DM-ordered within the group",
      `AssignRMRespectingGroupOrder`→`AssignDMRespectingGroupOrder`. Seed point →
      "DM-grouped + min-TL + WCET".
- [ ] `P0_8_important_task_schedulability/goal.md` + `tasks.md`:
      "RM-with-top-lock"→"DM-with-top-lock" / "DM-ordered within important group".

## 5. Verification + records
- [x] `pytest Gen_Taskset/tests/` green (43 passed).
- [x] `pytest tests/python/` green (351/353; 2 fails PRE-EXISTING config, not P0.9).
- [x] `cmake --build build_test --target check.SP_OPT -j5` green (17/17 ctest).
- [ ] Log any SP shift at the seed (D6); if global SP moves materially, flag for
      prod A/B (user-go, NOT run unilaterally).
- [ ] `dev_log.md` (this folder + top-level `agents/dev_log.md`) updated.
- [ ] Memory: new `p09-dm-and-important-first-priority.md`; update `p06-...`,
      `p08-...` cross-refs; add `p09-...` line to `MEMORY.md` index.
- [ ] `git add` staged; user reviews (no commit).

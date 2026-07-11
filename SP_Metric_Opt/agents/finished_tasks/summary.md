# Completed Tasks — Summary

> One line per completed task → detail doc. This is the index of finished work;
> active work lives in `agents/active_tasks/`, overall TODO in
> `agents/overall_tasks.md`. Detail narratives stay in the per-task doc and the
> top-level `agents/dev_log.md`.

---

## P0.5 — Redesign the Optimizer Iteration Process (incumbent state) — RESOLVED 2026-07-10

- **Origin:** the optimizer carried its running "best-so-far" (incumbent)
  solution across multiple parallel members (`prev_optimizer_`,
  `res_opt_`, `opt_pa_`, `opt_sp_`) — duplicated state, divergent cold-start
  paths, the "code organization not good / repeated code" the user flagged.
- **What landed.** Architectural redesign of the incumbent STATE, owned once in
  `res_opt_` (`ResourceOptResult`) with no parallel `prev_optimizer_` cache:
  `CommitIncumbent(pa, sp, tl)` is the single writer of `res_opt_` AND
  `opt_pa_`; `BuildChallengerFromIncumbent()` builds a throwaway
  `OptimizePA_Incre` from `res_opt_` each incremental candidate; the
  `has_incumbent_` bool gate was removed in Phase-5 issue 5d as provably
  redundant — `IfInitialized()` (`!opt_pa_.empty()`) is the gate. `prev_optimizer_`
  member removed.
- **Phase-5 user-review fixes (one-by-one).** 5a unified the two reset paths into
  `ResetIncumbentBaseline(bool from_scratch)`; 5b kept rebuild-from-champion (over
  a persistent challenger — drift risk to the diff baseline); 5c dropped the
  stale-TL edge-case guard; 5d removed `has_incumbent_`; 5e renamed the
  `time_limits` param to `starting_time_limits`; 5f removed the dead
  `any_eval_ran`/zero-work fallback; 5g simplified `OptimizeSingleTaskTimeLimit`
  patience to a total non-improvement budget (not reset on improvement); 5h
  (reuse-one-optimizer efficiency) moved to P3.1.
- **Verified.** 46 `testIncreOpt_w_TL` + 16/16 ctest green (DEBUG build,
  re-verified at closeout 2026-07-10). P1.1 INCR_P10 probe `ndiff` 5→0 at
  call=0 (only ever 0 or 1 across all 302 incremental calls).
- **Commits:** `a8dba07f`→`7fa2e9d2` (the redesign series); Phase-5 fixes in
  `8bbb0f5a`, `ae62e5e5`, `cde70138`, `7fa2e9d2`, `df3502a1`.
- **Subsumes the functional TL-init bug (former P0.1) by construction** — see
  the P0.1 entry below.
- *Detail: top-level `agents/dev_log.md` (2026-07-08 entry + 2026-07-10 Phase-5
  closeout); design spec in
  [`P0_5_optimizer_iteration_redesign/design.md`](P0_5_optimizer_iteration_redesign/design.md);
  rationale in memory `p05-subsumes-tl-init-bug.md`.*

## P0.1 — Persist adopted TL to YAML — RESOLVED 2026-07-10 (subsumed by P0.5; inspectability write discarded)

- **Origin:** scaffolded 2026-07-08 as the *root-cause fix* for the P1.1 residual
  ("runtime treats the YAML Gaussian as the prior ET, but a TL-optimizable task's
  prior ET is its adopted TL"). Demoted the same day to **inspectability-only**
  once P0.5 was worked first — P0.5's incumbent-state redesign makes the diff
  carry the adopted TL by construction.
- **Functional bug — RESOLVED BY P0.5, by construction.** `CommitIncumbent`
  (`OptimizeSP_TL_Incre.cpp:387`) is the single writer of
  `res_opt_.id2time_limit`; `BuildChallengerFromIncumbent` rebuilds the DAG from
  that carried adopted TL → both diff sides carry the adopted TL → the P1.1
  `FindTaskWithDifferentEt` false-positive class is structurally impossible.
  Confirmed at runtime: P1.1 probe INCR_P10 N=8 taskset_0 went `ndiff` 5→0 at
  call=0 (only ever 0 or 1 across all 302 incremental calls).
- **Inspectability YAML write — DISCARDED by user decision (2026-07-10).** The
  demoted remainder (overwrite the taskset YAML with the adopted TL + implied ET
  after each commit, for post-run debuggability) was never implemented and the
  runtime doesn't need it: the orchestrator clamps job ET to
  `res.id2time_limit` (`SimulationOrchestrator.cpp:461-463`), so no scheduler
  decision ever honors the stale Gaussian once optimization has run. User: "if
  it's about writing down to yaml file about the found time limits from
  optimizers, we can discard it." No code was ever written for P0.1.
- *Detail: top-level `agents/dev_log.md` (2026-07-10 entry); resolution rationale
  in memory `p05-subsumes-tl-init-bug.md`.*

## P0.1 — Commit pending uncommitted work — DONE 2026-07-06

- **All four commit groups landed.** (a) INCR_P<n> mode override → `1ede254f`;
  (b) env-ratio refactor → `4b44aabc`; (c) trial-and-error/patience TL rewrite
  → `88af2c54` + `fa0b857f`; (d) agent docs + reorg → `9c921fc0` + `99eda512`
  + `66c96c14` + `dab92e36`. Establishes the clean baseline.
- **"Done when" gate verified:** `git status` clean; `ctest` 16/16 green
  (incl. `testIncreOpt_w_TL` 43/43); `pytest tests/python` 261 passed.
- *Detail: [`2026-07-06_P0_1_commit_pending_work/`](2026-07-06_P0_1_commit_pending_work/).*

## Pre-2026-07-01

- **`opt_sp_` init fix** — 0 → -1. *Detail: `agents/dev_log.md`.*
- **Config optimization refactor** — exponential combination → linear iteration with task sorting.
- **Complete Fairness Scheduler (CFS)** — added + tests.
- **Python taskset generator** (`Gen_Taskset/`) — simulated training data + scheduler-comparison env + JSON-config end-to-end pipeline.
- **Ablation baselines** — `INCR_NO_TL`, `INCR_WCET`.
- **GP-based ET distribution prediction.**
- **C++ end-to-end scheduler** (`RunOrchestrator`).
- **Additional baselines** — CFS via `SimulateCFSSched`; ablation switches (`disable_time_limit_opt`, `use_wcet_execution_time`); `RM_FAST` (shortest TL), `RM_SLOW` (longest TL).
- **Interval YAML rename** — `taskset_characteristics_interval_{k}.yaml` for unambiguous glob matching; all Python/C++ readers + tests updated.

## 2026-07-01 Session — Configuration & Plotting Infrastructure

- **`experiment_config.json`** — single JSON config, `test_mode`/`prod_mode` + global `plotting`/`analysis` sections.
- **`experiment_config_loader.py`** — `load_experiment_config(mode)` returns flattened dict.
- **`plotting_config.py`** — `setup_publication_style()`, `get_scheduler_color_map()`, `save_figure()` (colorblind-safe, 14/16/18 pt, PNG+PDF).
- **`aggregate_across_tasks.py`** — scans `optimizer_comparison/`, produces fig 1A–1F + Ab-A/Ab-B.
- **Design alignment** (`plan_alignment.md`): ablation group = BF+INCR+INCR_NO_TL+INCR_WCET+INCR_SCRATCH; tunable `--important_task_pct`, `--num_workers`, `--resume`; paired t-test code kept but default OFF; 14/16/18 pt; PNG+PDF.

## 2026-07-01 Session — P5–P9 (all done)

- **P5** Important-task miss rate (Fig 3): `compute_miss_rate_by_task()`, `analyze_single_instance()` loads `sp_weight`, `compare_optimizers.py` collects `Important_Miss_Rate`, Fig 3 + Fig 3b.
- **P6** Trigger-interval sweep (Fig 2): `interval_sweep.py`, intervals `[1,5,10,20,30,60]` prod / `[5,10]` test.
- **P7** End-to-end shell scripts: `run_simulation.sh`, `run_sim_{4,6,8}_tasks.sh`, `run_all_experiments.sh`, `run_paper_figures.sh`, `run_interval_sweep.sh`.
- **P8** Tests: `test_aggregate.py`, `test_plotting.py`, `test_run_sim_experiments.py`.
- **P9** Integration verification: all figures generate with mock data; `comparison_summary.csv` has `Important_Miss_Rate`; scripts `+x` + `bash -n`; Python has docstrings.

## P24/P25 — Periodic reoptimization with compare-and-keep

- **Commits 1–7 committed** (HEAD `de4e9636`). Counter-driven dispatcher
  (`Optimize_w_TL_ScratchOrIncre`), `prev_optimizer_` incumbent,
  `ReOptimizePeriodic` compare-and-keep, radius-knob rename.
  *Full design + commit breakdown: `P24_task.md`.*
- **Post-Commit-6 INCR-ET fix bundle** — `986a9cfe` (Fix A: `OptimizeIncre`
  advances `dag_tasks_`) + `de4e9636` (Fix B + latent `SeedStateFromIncumbent`
  `sp_parameters_` carry). TDD-verified (`testIncreOpt_w_TL` 29/29, 16/16 ctest).
  *Root cause + fix record: `agents/investigation/debug_runtime0704_incr.md`.*
- **Runtime A/B re-run — 2026-07-04 19:41 (PARTIAL PASS).** Pathological 3×
  growth eliminated (P60/P1 3.4×→1.47× ts0, 2.1×→1.13× ts2; P30/P60 vs P10
  ≤1.1–1.3× met). Literal flip `INCR_P1≥P10≥P30≈P60` NOT met (P1 still
  cheapest). Residual = Fix C (deferred). **Investigation is open as P1.1.**

## Trial-and-error TL optimization — DONE 2026-07-05

- **`PerformCoordinateDescentForTaskConfigOpt`** rewritten with unidirectional
  outward walk (`OptimizeSingleTaskTimeLimit`, `patience`-bounded: incremental
  patience=0 strict, reopt patience=1 tolerates one dip). Helpers
  `FindTimeLimitOptionIndex` / `IsBetterTimeLimitOption` extracted. Radii
  lowered `ReoptimizationTimeLimitSearchRadius` 6→2, `IncrementalTimeLimitSearchRadius` 2→1.
  `testIncreOpt_w_TL` 43/43 green (incl. 7 walk tests + helper suites); 3 stale
  test expectations updated. 16/16 ctest green.
  *Detail: `trial_and_error_tl_opt_task.md`.*
- **Status: committed @ `88af2c54` + `fa0b857f`** (P0.1 commit group c).
  `testIncreOpt_w_TL` 43/43 + `ctest` 16/16 green.

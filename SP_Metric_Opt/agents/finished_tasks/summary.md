# Completed Tasks — Summary

> One line per completed task → detail doc. This is the index of finished work;
> active work lives in `agents/active_tasks/`, overall TODO in
> `agents/overall_tasks.md`. Detail narratives stay in the per-task doc and the
> top-level `agents/dev_log.md`.

---

## P2.2 — Doc & memory hygiene — CLOSED 2026-07-12 as superseded / moot (NOT executed)

- **Disposition: closed, not worked.** Every substantive item was overtaken by
  later resolutions before this task was picked up; the task-as-specified is
  obsolete. No code, agent doc, or memory file was edited — only the closure was
  recorded.
- **(a) Memory `interval-sweep-stale-flags-bug`** — premise stale in the
  *opposite* direction. The memory claims `compare_optimizers.py` **removed**
  `--on_taskset_config_change` / `--run_root` (sweep-stage crash, exit 2), but
  current code **accepts** both (`compare_optimizers.py:303` `--run_root`,
  `:334` `--on_taskset_config_change`). The described flag-removal never
  persisted; the crash doesn't reproduce. The task's "mark RESOLVED-cite-the-fix"
  edit was therefore the wrong action. **No algorithm-performance impact** — even
  live this would be a sweep-stage CLI plumbing crash; the algorithm runs in the
  simulate stage (SP metric / optimizer computation unaffected), and the memory's
  own `--steps simulate aggregate` workaround yields correct optimize/aggregate
  output minus the period-sensitivity data. `run_end_to_end.sh` always runs
  simulate→sweep→aggregate (P22) and the P25 A/B config uses
  `interval_sweep_seconds_list=[10]` (single point → sweep no-op) — e2e doesn't
  crash. The memory file itself was left untouched (out of scope here); flagged
  for a separate one-line memory retirement.
- **(b) §5 corrections** (`investigation_problems_encountered.md` →
  `debug_runtime0704_incr.md` §10/§11 + `p25-incr-et-grows-with-period` memory) —
  predate the P1.1 resolution. Memory `p25-ndiff-diff-semantics` (2026-07-07)
  re-derived the "2-vs-8 discrepancy" as false-positives (NOT UNRESOLVED) and
  concluded "Fix C & Fix D both wrong levers"; P0.5 (2026-07-10,
  `a8dba07f`→`7fa2e9d2`) resolved the gate (both diff sides carry adopted TL;
  runtime `ndiff` 5→0). Both §5 asks moot.
- **(c) `issues.md` #9/#3/#6** — `issues.md` was deleted in commit `66c96c14`
  (2026-07-06, "remove issues.md"). Resolved-by-deletion; no file to mark.
- **(d) `trial_and_error`** — already DONE (committed @ `88af2c54` + `fa0b857f`);
  P0.1 has since landed. Self-resolved.
- *Detail: [`P2_2_doc_memory_hygiene/`](P2_2_doc_memory_hygiene/) ("Closure
  disposition" in `goal.md`); rationale traces in the per-item bullets above.*

## P1.7 — Simulator ignores `processorId` partitioning (single run-queue overload) — RESOLVED 2026-07-12

- **Origin:** the runtime simulator `FixedTaskPrioritySchedulingOrchestrator::SimulateInterval`
  (and the CFS sibling `CFSSimulationOrchestrator::SimulateInterval`,
  `SimulationOrchestrator.cpp`) built ONE `RunQueue(dag_tasks.tasks)` over the
  whole task set; `ReleaseJobs`/`ReleaseJobsCFS` filtered only on
  `time_now % period == 0` with **no `processorId` check**, so jobs on
  `processorId:0` and `:1` competed for the single `processor_free_` flag — two
  cores declared in YAML, one simulated (mean single-queue util 2.099, max 3.332
  across 600 generated interval YAMLs vs correct per-proc max mean 1.160). The
  legacy partitioning path (`ScheduleSimulation.cpp::SimulatedFTP_SingleCore` +
  `GetProcessorIds`) was always correct; the runtime orchestrator that feeds the
  eval suite / A/B / figures simply didn't use it.
- **What landed.** Both `SimulateInterval`s now call `GetProcessorIds(dag_tasks)`
  and build one `RunQueue` (held in `std::unique_ptr` — `RunQueue` is
  non-assignable due to its `const TaskSetInfoDerived` member) per distinct
  `processorId`, stepping all queues in lockstep (Remove/Record/Release/Run per
  queue per tick). `ReleaseJobs`/`ReleaseJobsCFS` gained an `int processor_id = -1`
  param (`-1` = release all, preserving the unit-test call sites); when `>= 0`,
  tasks whose `processorId` doesn't match are skipped. Mirrors the proven legacy
  per-core model. `RecordFinishedJobs`/`RecordFinishedJobsCFS` unchanged (take
  `RunQueue&`, read only that queue's `schedule_`, merge into the shared
  `job_history_`). `ScheduleSimulation.h` got the missing `GetProcessorIds`
  declaration.
- **Pivotal blast-radius correction.** The SP metric is **analytic, not
  simulation-derived**, and was **NEVER distorted** by the bug.
  `SimulateInterval` → `ObtainSP_TaskSet_And_TimeLimits` (`SP_Metric.cpp:82`) →
  `ObtainSP_TaskSet` (line 53) → `ProbabilisticRTA_TaskSet` (`RTA.cpp:100`),
  which **already partitions on `processorId`** via `ExtractTaskSetPerProcessor`
  (`RTA.cpp:87`). So `mean_sp_norm` / `Mean_SP_Metric` (what every eval-suite
  gate Q1/Q2/Q3/E1/E3 reads) was always on the correct path, and **no gate
  verdict moves** after the fix. The bug only distorted the `job_history_`-derived
  exports (miss-rate / response-time columns in `comparison_summary.csv`, read by
  `aggregate_across_tasks.py` + `utils.py`, NOT by the gates). E3 was unrelated.
- **Verified.** 2 new red→green tests in `tests/testScheduleSimulate.cpp`
  (`SimulateIntervalPartitionsByProcessorId` + `_CFS`) on new input dir
  `tests/test_data_partition_two_cores/` (assert both cross-core jobs start at
  `t=0`; buggy code serialized Task1.start to `2`). `testScheduleSimulate` 34/34,
  ctest 16/16 green (DEBUG). Single-core tasksets byte-identical (all tasks land
  in the same queue → regression guards green).
- **Status: staged on `clean_simulation`, NOT committed** (user runs the A/B
  re-run; standing `git add`-only constraint).
- **Residual follow-up filed as P2.6** — make the simulation report BOTH the
  analytical SP and the true SP derived from the RunQueue's simulated RT samples
  (the user wants SP to eventually come from the actual schedule, not the
  analytic RTA). This P1.7 fix (honest per-core RT samples) is its prerequisite.
  *(P2.6 itself was CLOSED 2026-08-02 as deferred → P3 `optional_figures`; not
  worked — see [`P2_6_sim_rt_based_sp_metric/`](P2_6_sim_rt_based_sp_metric/).)*
- *Detail: [`P1_7_cpu_partition_mismatch/`](P1_7_cpu_partition_mismatch/);
  memory `cpu-partition-mismatch.md`.*

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

## P2.12 — C++ Executable Speed Test Benchmark (RunSpeedTest) — CLOSED 2026-07-25

- **Disposition: closed, done.** Added `tests/RunSpeedTest.cpp` — a release-mode
  benchmark that runs `INCR_Reopt_1` + `INCR_Reopt_10` via
  `FixedTaskPrioritySchedulingOrchestrator` on `tests/speed_test/taskset_N8`,
  records scheduler ET, and reports PASS/FAIL vs a threshold (default 0.1 s/act).
  Registered as a `tests/CMakeLists.txt` target; built + run in **Release**.
- **Results (release):** `INCR_Reopt_1` 0.042 s/act, `INCR_Reopt_10` 0.013 s/act
  → both ≪ 0.1 s threshold → **PASS**.
- **Commits:** `55285fc0` (taskset) + `6b5065c3` (RunSpeedTest).
- **Why it matters:** the standing "optimization got much slower" worry is
  resolved here — release-mode timing shows the optimizer is fast; the apparent
  slowdown was the DEBUG (`build_test`) build, closed by the `validate_bin_dir`
  release-only enforcement (`fa113d23`).
  *Detail: `agents/finished_tasks/P2_12_reopt_speed_test/`.*

## P2.9 — Speed Up Re-Optimization — CLOSED 2026-07-25 as superseded / moot

- **Disposition: closed, not as originally specified.** P2.9's premise — reopt
  costing ~0.8 s/activation at N=16 — was the **DEBUG-build artifact**: that
  figure came from `BIN_DIR=build_test` (~8.5× slower), not the algorithm.
  Release-mode measurement (P2.12 `RunSpeedTest`) shows `INCR_Reopt_1` at
  **0.042 s/act** — the speed problem P2.9 was opened to fix does not exist in
  release. The release-only `validate_bin_dir` guard (`fa113d23`) now prevents
  the artifact from recurring.
- **Lever A (route reopt TL walk through sub-incremental eval) — code landed,
  NOT adopted into prod.** The `ReoptimizationUseSubIncrementalWalk` flag
  (default OFF) was added `5dfd146e`; the walk switch + `ReconstructTimeLimitVec`
  re-sync + budget polls landed `52e29e90` / `a6922ff5`; TDD green. The flag
  stays default-OFF and its A/B is **subsumed by P2.11** (P2.11's behavior-
  changing merge IS lever A's walk; P2.9's A/B becomes P2.11's Phase 5 A/B).
- **No prod behavior change.** Default-OFF → prod bit-identical. Flag removal
  is deferred to P2.11 Phase 2 (post-A/B).
  *Detail: `agents/finished_tasks/P2_9_speed_up_reoptimization/`.*

## P2.14 — Remove SP_THRESHOLDS_SET & SP_THRESHOLD_RANGE Harmonization — CLOSED 2026-07-25 (code-complete)

- **Disposition: closed as code-complete; Step 5 empirical analysis deferred to P2.13.**
  Eliminated `SP_THRESHOLDS_SET` from `REQUIRED_CONFIG_PARAMS`, configs, and tests;
  harmonized `sp_threshold` generation to continuous sampling from the sole knob
  `SP_THRESHOLD_RANGE: [0.001, 0.9]`. The `1.0` entry in `SP_THRESHOLDS_SET`
  (treating 100% DDL miss as "safe"/unpenalizable) is gone — the threshold-side
  absurdity P2.13 flagged.
- **Steps 1–4 done** (code + 5 configs + 9 inline-config test edits + verification):
  committed at `5c782bad`. `pytest` SP-threshold-relevant tests pass (2 full-suite
  failures pre-existing & reproduced on stashed baseline — `test_integration_pipeline`
  missing `RANDOM_SEED`, `test_generate_path_only` unseeded RNG pollution). C++ ctest
  N/A — P2.14 is Python-only (no `SP_THRESHOLD` reference in `sources/` C++).
- **Step 5 (empirical `ddl_miss_chance` vs `sp_threshold` for important tasks) DEFERRED
  to P2.13** — that study is owned by P2.13; doing it under P2.14 would duplicate.
  `tasks.md` Step 5 marked `[ ]`→`[-]` DEFERRED.
- **Threshold-side twin of P2.15.** `goal.md` + `dev_log.md` were untracked through
  `5c782bad`; both `git add`-ed as part of the move.
  *Detail: `agents/finished_tasks/P2_14_sp_threshold_set_removal/`.*

## P2.15 — Randomize sp_weight per Task — CLOSED 2026-07-25 (committed `771be079`)

- **Disposition: closed, done.** Added `SP_WEIGHT_RANGE: [0.1, 1.0]` (parallel to
  `SP_THRESHOLD_RANGE`); sample `sp_weight = random.uniform(0.1, 1.0)` per task;
  removed the hardcoded `sp_weight_base = 2.0` (perf) / `1.0` (normal/env) 2:1 split;
  kept `SP_WEIGHTS_SUM` normalization (scale contract). "Important" (top-`sp_weight`
  task, the P2.13 subject) is now random, decoupled from task type — sanity check:
  top-weight task was a non-perf task while the perf task was mid-range.
- **Generator-only / ctest N/A:** `sp_weight` is a pure LINEAR multiplier in C++
  (`ObtainSP × weight` `SP_Metric.cpp:12-16`; `effective_weight = weight*perf_coeff`
  `OptimizeSP_Incre.cpp:81-87`; `sum_sp_weights` ceiling `:154-159`) — no `sources/`
  code assumes the 2:1 ratio. `pytest Gen_Taskset/tests tests/python` → **372/372 green**
  (was 331/41-fail before the 9 inline-config test edits; all 41 were the integrity
  gate raising on the missing `SP_WEIGHT_RANGE` key).
- **Weight-side twin of P2.14.** Cross-link carried to P2.13: P2.13's D1 feasibility
  check (perf-task clamp gap in `feasibility_clamp.py`) should be re-run against
  randomized-weight tasksets — the "important" task is no longer by-construction perf.
  *Detail: `agents/finished_tasks/P2_15_sp_weight_randomization/`.*

## P1.8 — INCR_WCET outperforms INCR — CLOSED 2026-07-26 (clamp shipped `84a99ef5`)

- **Disposition: closed for code.** The "INCR_WCET beats INCR" verdict was a
  generator ET-feasibility defect, not an optimizer property: INCR collapsed on
  unschedulable tasksets (generated with WCET > deadline), while the degraded
  INCR_WCET ablation happened to dodge them. Fix = `Gen_Taskset/lib/feasibility_clamp.py`
  + tests, TDD-verified 2026-07-12, committed `84a99ef5`.
- **No optimizer code change** — the SP metric and both arms were correct on
  schedulable inputs; the fix lives entirely in the taskset generator.
- *Detail: `agents/finished_tasks/P1_8_incr_wcet_outperforms_incr/`; memory
  `p18-incr-wcet-outperforms-incr.md`.*

## P1.14 — BF time-limit violation — CODE CLOSED 2026-07-26 (`ecf0c597`+`bfbec7e5`)

- **Disposition: code closed; Phase 3 A/B re-run DEFERRED (measurement only).**
  Root cause = a fresh `OptimizePA_BF` constructed per TL-leaf reset
  `start_time_`, and `ifTimeout` was checked only at recursion boundaries → the
  budget guard fired against a stale horizon on deep leaves. Phase 2 BF fix
  `ecf0c597` + Phase 2b INCR mirror `bfbec7e5` (`BFDLSharedBudget` guard) land
  the structural fix on both arms.
- **Phase 3 = re-run the P25 A/B** on the fixed binary to record the true
  BF-vs-INCR_Reopt_X verdict; no code work remains. Blocked on P1.15's crash
  fix (resolved via P1.16) — now unblocked.
- *Detail: `agents/finished_tasks/P1_14_bf_time_limit_violation/`; memory
  `p114-bf-time-limit-violation.md`.*

## P1.15 — Silent sim failure inflates aggregate — CODE CLOSED 2026-07-26

- **Disposition: code closed; Phase 3 A/B re-run DEFERRED (measurement only).**
  Two independent layers, both done:
  - **Layer A (harness):** worker exceptions now propagate
    (`ThreadPoolExecutor` + `fut.result()`); `run_single_simulation` always
    captures binary stdout+stderr to per-arm `run.log` (raises on non-zero exit);
    `analyze_single_instance` raises on missing/empty `interval_sp_metrics.txt`;
    on ANY crash `compare_optimizers.main()` writes only `crash_report.txt` +
    `taskset_arm_status.csv` and exits non-zero — NO partial
    `comparison_summary.csv`, NO plots. 9/9 `test_compare_optimizers_crash.py` green.
  - **Layer B (C++ crash, via P1.16):** `UpdateRecords` returns `bool`;
    `OptimizeIncreSingleTask` backs up `rta_cache_` at entry and reverts it when
    the candidate is not adopted → speculative champion updates no longer
    desynchronize on rejected walks. Reproduced `taskset_3 INCR_Reopt_5/10/30/60`
    → all exit 0 (was 134/SIGABRT).
- **The false verdict** (BF 0.6129 "losing" to INCR_Reopt_5+ ~0.622) was a
  harness artifact: Reopt_5+ silently dropped the hard tasksets (empty input →
  `np.mean([])→0.0`), so BF's mean was dragged down by tasksets Reopt_5+ never ran.
- **Phase 3 = re-run the P25 A/B** on fixed binary + fixed harness; no code work.
- *Detail: `agents/finished_tasks/P1_15_silent_sim_failure_inflates_agg/`; memory
  `p115-silent-sim-failure-inflates-agg.md`.*

## P1.17 — RTA cache + opt redundant ops — CLOSED/superseded 2026-07-26 by P1.23

- **Disposition: closed as superseded.** The RTA cache shipped + benchmarked
  via **P1.23** (~36% faster at N=10, SP bit-identical), which subsumed the
  redundant-ops refactor this task was filed to drive. No standalone P1.17
  commit — its scope was folded into the P1.23 cache work.
- *Detail: `agents/finished_tasks/P1_17_rta_cache_and_opt_redundant_ops/`;
  memory `p117-rta-cache-and-opt-redundant-ops.md`; sibling P1.18 (closed
  `09d1fca9`).*

## P1.10 — Serialized single-task incremental opt — DEFERRED 2026-07-26 (awaiting A/B)

- **Disposition: deferred — design complete, flag off, awaiting A/B.** Design
  docs landed in `86c811ac` (api_design/dev_log/goal/tasks). The deliverable is
  a compile-time flag `use_serialized_incremental_opt` (default OFF, NOT
  YAML-backed): Phase 1 D1 committed (`33b2270c`+`3d2f9b28`), D2 filter
  committed (`3380f18e`); Phase 2 serialized loop + Phase 3 invariant proof
  complete behind the flag, 16/16 ctest green.
- **Invariant:** `|diff| <= 1` at every serialized SP-eval (Type-L: 1, Type-E:
  0) — the P1.9 unblock condition is MET. The flag stays OFF in prod until an
  A/B decides whether to enable.
- *Detail: `agents/finished_tasks/P1_10_serialized_incremental_optimization/`;
  memory `p1-10-serialized-incremental-optimization.md`.*

## P1.2 — Reopt incumbent degradation — DEFERRED 2026-07-26 (P3, known hazard)

- **Disposition: deferred / let go — P3, out of immediate scope, kept as a
  known theoretical hazard.** No code work landed or planned this stage.
- *Detail: `agents/active_tasks/P3_2_reopt_incumbent_degradation/`; memory
  `p12-reopt-incumbent-degradation.md`.*

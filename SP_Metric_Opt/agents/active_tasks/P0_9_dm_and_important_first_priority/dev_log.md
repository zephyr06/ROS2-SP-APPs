# P0.9 System-wide DM + Important-First Priority Lock — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the top-level
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-28 (task scaffolded from user direction)

- **User direction:** switch the re-optimization's first initialization method to
  match the Python schedulability test — "first sort by importance, then assign DM
  to important and non-important task separately." Folded together with the earlier
  "sort by deadline rather than period" feedback (constrained deadlines `D < T`
  make RM suboptimal → DM) and the "System-wide DM" scope choice.
- **Grounded the scope:**
  - C++ priority assignment = `RateMonotonicPriorityVec`
    (`OptimizeSP_TL_Incre.cpp:695-707`): plain RM, sort all tasks by period asc,
    ties by avg ET asc. Called ONLY by `SeedIncumbentFromRMFast:760` — the
    interval-0 / re-opt first init (shared by `ResetIncumbentBaseline`'s interval-0
    else-branch and `BootstrapIncumbentFromRMFast` / the `INCR_NO_REOPT` arm).
  - `is_important` IS on the C++ `Task` (`RegularTasks.h:111`, parsed from
    `important` YAML key at `RegularTasks.cpp:87`) → group-aware PA is feasible.
  - Orchestrator has NO own RM/DM path (empty grep on `SimulationOrchestrator.{cpp,h}`
    for priority-assignment terms) → it consumes baked `priority` fields → no
    orchestrator change. "System-wide" = C++ optimizer seed + Python RTA + P0.6 plan.
  - P0.6 already plans `AssignRMRespectingGroupOrder(important_ids)` (important-first
    block, RM-within-group; `goal.md:42,281`). This task supersedes it to DM + group
    lock, promoted into the shared first-init (not P0.6-only).
- **Recorded D1–D6 in `goal.md`** (D1–D3 RESOLVED by user direction; D4 naming,
  D5 tie-break, D6 behavior-change-flag OPEN — surfaced via AskUserQuestion before
  C++ work).
- **CRITICAL flag (D6):** this is a BEHAVIOR CHANGE, not a refactor. Seed PA shifts
  whenever `D≠T` (always — `deadline = period·U(0.5,1.0)`) or the group lock
  reorders (important tasks not already top). The bit-identical SP gate (17/17
  ctest) may FAIL. DM ≥ RM + group lock reduces important-task interference →
  schedulability should improve, but global SP may move. Verification gate =
  "schedulability improves / SP acceptable", NOT bit-identical. Prod A/B re-run
  may be needed (user-go, do NOT run unilaterally).
- Not started; awaiting D4/D5 user direction before C++ work.

## 2026-07-28 (D4–D7 resolved; Python RTA module landed)

- **User direction (D4–D7):**
  - D4 = **rename to DM** (full cascade: `RateMonotonicPriorityVec`→
    `DeadlineMonotonicPriorityVec`, `*RMFast`→`*DMFast`, orchestrator arms
    `RM`/`RM_FAST`/`RM_SLOW`→`DM`/`DM_FAST`/`DM_SLOW`).
  - D5 = avg ET tie-break (matches current RM tie-break).
  - D6 = accept behavior change; verify "schedulability improves/holds + SP within
    tolerance", NOT bit-identical. Prod A/B proposed separately.
  - D7 = FULL system-wide — ALSO switch the orchestrator baseline arms (their own
    period-sort at `SimulationOrchestrator.cpp:361,373,393`) to deadline + rename
    the strings. Breaks result-comparability with prior runs (accepted). Grounded
    that these baselines are NOT the certified path (the fall-back uses the P0.6
    static solution) but must agree on the priority model for the comparison to be
    meaningful.
- **Grounded the full rename surface** (two distinct RM concepts):
  1. Optimizer seed PA identifiers (`RateMonotonicPriorityVec` + `*RMFast`) — the
     certified path; 2 files + 1 test.
  2. Orchestrator baseline arms (`"RM"`/`"RM_FAST"`/`"RM_SLOW"` strings) —
     comparison modes with their OWN period-sort, labels in configs/results/figures
     + 10+ Python scripts; NOT the certified path. D7 extends the rename here.
- **Python RTA module LANDED (TDD red→green):** `_important_priority_order` sort
  key `period`→`deadline`. The discriminating test
  `test_dm_ordering_is_deadline_based_not_period_based` constructs a case where DM
  and RM orderings diverge (t_a: short deadline/long period; t_b: short period/
  long deadline) and asserts the RTA ranks by deadline — RED under the old period
  sort (t_b wrongly HP → t_a misses), GREEN under deadline sort (t_a correctly HP
  → schedulable). Redesigned the unschedulable/boundary/per-core tests so DM and
  RM agree there (HP keeps the shortest deadline) — validates correctness without
  flipping on the sort change. Docstrings updated RM→DM throughout; the
  recurrence's own `period` refs are legitimate (interference is by inter-arrival
  period — only the priority RANK uses deadline). `pytest Gen_Taskset/tests/` =
  43 passed (no regressions).
- Next: C++ module (seed PA group-locked DM + rename) + orchestrator baseline arms.

## 2026-07-28 (C++ seed PA module LANDED — TDD red→green)

- **Starting state:** the C++ rename + sort impl were ALREADY in place in
  `OptimizeSP_TL_Incre.{h,cpp}` (`DeadlineMonotonicPriorityVec` group-locked
  DM sort; `SeedIncumbentFromDMFast`/`BootstrapIncumbentFromDMFast` renamed).
  BUT `tests/testIncreOpt_w_TL.cpp` still called the OLD names at 7 call sites
  (760/775/782/799/913/936/993) → `check.SP_OPT` RED (compile error). So the
  impl had landed without its test-side cascade.
- **TDD red confirmed:** `cmake --build build_test --target check.SP_OPT -j5`
  failed on `opt.RateMonotonicPriorityVec()` — "has no member named ...
  did you mean DeadlineMonotonicPriorityVec?".
- **Test rewrite (the discriminating cases):** replaced the two
  `RateMonotonicPriorityVec` period-sort tests with
  - `DeadlineMonotonicPriorityVec.RanksImportantGroupFirstThenDmOrdersEach` —
    4 tasks where `D≠T` AND important tasks are NOT already top; asserts
    important group first (DM-ordered within: ddl 90<180), then non-important
    (DM-ordered within: ddl 60<120) → `pa=[0,1,3,2]`. Plain RM would give
    `[1,3,0,2]` — wrong on both group lock and within-group key.
  - `DeadlineMonotonicPriorityVec.BreaksDeadlineTiesByExecutionTimeAscending`
    — equal-deadline ties within a group broken by avg ET asc (T1 ET10 < T0
    ET30); non-important T2 stays below both (group lock).
- **Call-site updates:** the 3 `pa_rm`/`pa = opt.RateMonotonicPriorityVec()`
  call sites in `CompareAndKeepSynthetic` (seed-baseline + re-eval + P3.6
  bootstrap) → `DeadlineMonotonicPriorityVec()` / `pa_dm`. Renamed test
  `SeedIncumbentBaseline_Interval0UsesRMAndMinTL`→`..._Interval0UsesDMAndMinTL`.
  P3.6 comments "RM-fast bootstrap"→"DM-fast bootstrap" throughout. Header
  `BootstrapIncumbentFromRMFast` comment ref → `...FromDMFast`.
- **TDD green:** `cmake --build build_test --target check.SP_OPT -j5` =
  **17/17 passed** (incl. the 2 new DM tests). Step 1 (C++ seed PA) DONE.
- **Step 2 (orchestrator baseline arms, D7) scope grounded — NOT started:**
  the rename cascade is LARGE. ~20 files, ~80 occurrences:
  - C++: `SimulationOrchestrator.cpp:356/368/388` (the 3 arms + their
    period→deadline sort at 361/373/393) + `:321` comment; `tests/testScheduleSimulate.cpp` (22 occ) + `tests/RunOrchestrator.cpp` (1).
  - configs: 4 JSON in `simulation_experiments/configs/` (7 occ total).
  - Python: 13 scripts (compare_optimizers, evaluation_suite, interval_sweep,
    aggregate_across_tasks, reanalyze_results, run_sim_experiments, + 6 tests
    incl. test_interval_sweep/test_aggregate 12 occ each) + 5 Visualize/debug.
  - D6 flag: switching the arms' sort period→deadline is a BEHAVIOR change for
    the baselines (NOT bit-identical); 17/17 ctest may shift but should still
    pass (assertions are structural, not SP-value). Prod A/B re-run = user-go.
- Next: Step 2 cascade — start with the orchestrator C++ (the 3 arms + sort +
  comment), then testScheduleSimulate, then configs, then Python scripts.
  WIDE change → checkpoint + /compact before starting.

## 2026-07-28 (Step 2 C++ LANDED — orchestrator arms + RunOrchestrator + testScheduleSimulate)

- **Scope decision:** the orchestrator's `DM`/`DM_FAST`/`DM_SLOW` arms use a PLAIN
  deadline sort (no `is_important` group lock) — matching the former plain-RM
  structure, just key `period`→`deadline`. The group lock is a SEED-PA concept
  (`DeadlineMonotonicPriorityVec`, the certified path); the baseline arms are
  comparison modes, so D7 swaps their sort key + renames the strings, nothing more.
- **`SimulationOrchestrator.cpp`:** 3 arms `"RM"`/`"RM_FAST"`/`"RM_SLOW"`→
  `"DM"`/`"DM_FAST"`/`"DM_SLOW"`, each sort lambda `.period`→`.deadline`; the
  `:321` mode-list comment + the `:33` "RM baseline" fall-through comment → DM.
  Verified `Task::deadline` is the same field the seed PA already uses.
- **`RunOrchestrator.cpp`:** 4 "RM baseline" comments → "DM baseline"; the
  `Modes:` help line `RM, BF, ..., RM_FAST, RM_SLOW` → `DM, BF, ..., DM_FAST, DM_SLOW`.
- **`tests/testScheduleSimulate.cpp` (the big one):**
  - Every `"RM"`/`"RM_FAST"`/`"RM_SLOW"` mode string → `DM`/`DM_FAST`/`DM_SLOW`.
  - The `/RM/` export-folder path segments → `/DM/` — REQUIRED because
    `ExportResults(scheduler_name)` writes into a folder NAMED for the mode, so
    the expected file path must follow the rename or the test looks in `/RM/`
    while the orchestrator writes `/DM/`.
  - Test names `UnitDeterminePrioritiesAndBudgets_RM_FAST`/`_RM_SLOW` → `..._DM_*`;
    `RateMonotonicPriorityAssignment` → `DeadlineMonotonicPriorityAssignment`
    (+ its `test_output_rm` dir → `test_output_dm`).
  - Comments RM→DM (incl. the fig3-bug narrative `RM_FAST`/`RM_SLOW`→`DM_*`, and
    the P1.7 "RM priority: Task0 < Task1" → "DM priority").
  - **Fixture check:** `test_data_schedule_orchestrator` has `deadline == period`
    for all 4 tasks (10/20/20/40), so DM and RM produce IDENTICAL ordering → the
    existing `EXPECT_EQ(0, priority_vec[0])` / `EXPECT_EQ(3, priority_vec[3])`
    assertions hold unchanged. Only the label + output dir change.
- **Latent bug EXPOSED by the rename + FIXED (NOT P0.9-introduced):**
  `ExactResponseTimeValidation` parsed 7 columns (`jobId,release,start,finish,
  response,execution,overrun`) but the P2.14 writer (`SimulationOrchestrator.cpp:
  239-245`) DROPPED `is_overrun` → writes only 6 columns. The test only ever
  passed by reading STALE June-21 `RM/response_times_task_*.txt` artifacts — it
  never `remove_all`'d the output dir AND never set `EXPORT_DETAIL_LEVEL=3` (the
  yaml default is 0, so no response_times files were freshly generated). The
  rename to `/DM/` broke the stale-file dependency → the parse failed (`Actual:
  false` at the `>> overrun` step). Fix: set `EXPORT_DETAIL_LEVEL=3` + `remove_all`
  the output dir (self-contained — generates what it reads, no stale artifacts) +
  drop the `overrun` field from BOTH parse sites (Task0 job0/1/2 + Task1 job0).
  This is a P2.14 cleanup, faithful to the rename — surfaced here only because the
  rename dislodged the stale-file crutch.
- **TDD green:** `cmake --build build_test --target check.SP_OPT -j5` = **17/17
  passed** (after the fix; was 16/17 with the exposed parse failure).
- **Staged (git add-only):** `SimulationOrchestrator.cpp` + `RunOrchestrator.cpp`
  + `testScheduleSimulate.cpp` (81 ins / 72 del across the 3).
- **Remaining (NOT started):** the config + Python cascade — `simulation_experiments/
  configs/*.json` (7 occ), 13 Python scripts (incl. `test_interval_sweep`/
  `test_aggregate` ~12 each) + 5 viz/debug. Result CSVs under `tests/*_runs/`
  (`radius_comparison`, `comparison_runs`, `e2e_eval_runs`, `k_variation`) are
  STALE run OUTPUTS — left for the prod A/B re-run, NOT rewritten (D6: breaks
  result-comparability, accepted; prod A/B = user-go).

## 2026-07-28 (Step 2 Python/configs cascade LANDED — D7 full system-wide)

- **Scope grounding (Explore agent was auto-denied; mapped directly via bounded
  `grep`):** the bare `"RM"` arm is RETIRED in configs/Python —
  `evaluation_suite.py:122` documents "The bare 'RM' arm was replaced by
  RM_FAST+RM_SLOW in every config's main_scheduler_list." So the cascade is
  `RM_FAST`→`DM_FAST` + `RM_SLOW`→`DM_SLOW` everywhere, PLUS bare `"RM"`→`"DM"`
  only where it still appears as a real mode (debug scripts) or a synthetic
  test-fixture label. No shell scripts, no YAML reference the modes.
- **Design forks (2 AskUserQuestion, both user-decided):**
  - **Synthetic test-fixture `"RM"` labels** (e.g. `{"scheduler":"RM","mean_sp":2.0}`
    in `test_aggregate.py`) — mode-agnostic test data. User chose **full D7
    consistency**: rename to `"DM"` too (data+assertions together, tests stay green).
  - **`draw_trajectory_error.py:16`** `"RM": CameraTrajectory_rm.txt` — a SLAM-
    trajectory data-file dict key (the `.txt` is a real Jan-2025 artifact on disk),
    NOT a scheduler-arm dispatch. User chose **LEAVE** (renaming would orphan the
    data path; the SLAM run that produced it was genuinely RM-labeled).
  - **`p211_reopt_ab_config.json`** — `RM_FAST`/`RM_SLOW` appear ONLY in its
    `_comment` prose (no active scheduler_list); the config is a RETAINED
    historical record of an obsolete P2.11 A/B run ("do not delete"). User chose
    **LEAVE** (the prose describes what the gate needed AT THAT TIME; editing
    would falsify the historical record).
- **Edits (18 source files):**
  - **3 active JSON configs:** `paper_simulation_config.json` (test+prod
    `main_scheduler_list` + the `:2` "Q1/Q2/Q3/E1 still evaluate (.../RM_FAST/
    RM_SLOW)" prose + the `:26` "RM-fast bootstrap" comment), `compare_against_bf.json`,
    `incr_et_profiling.json`.
  - **6 `simulation_experiments/` scripts:** `run_sim_experiments.py`,
    `compare_optimizers.py`, `reanalyze_results.py`, `interval_sweep.py` (×2:
    `schedulers=` + `main_schedulers=`), `evaluation_suite.py` (`:84` prose + the
    `:120-125` "bare RM replaced by RM_FAST+RM_SLOW" narrative → DM + `Q3_BASELINES`),
    `aggregate_across_tasks.py` (×3 identical `scheduler_list=`).
  - **6 debug scripts:** `simulation_experiments/debug_analysis/` AND
    `tests/debug_analysis/` are DISTINCT file copies (not symlinks — different
    sizes/mtimes); both dirs' `analyze_incr_vs_br_vary_k.py` (×2 lines each),
    `run_radius_comparison.py` (`BASE_MODES`), `run_e2e_eval.py` (`modes=`)
    renamed.
  - **`Visualize_SP_Metric/visualize_SP_distribution.py:112`** default
    `scheduler_name="RM"`→`"DM"` (builds `../Experiments/<mode>/` path — same
    folder-named-for-mode coupling as `ExportResults`).
  - **5 `tests/python/` fixtures:** `test_aggregate.py` (12×), `test_interval_sweep.py`
    (12×), `test_plotting.py` (4×), `test_evaluation_suite.py` (8×) — bare `"RM"`→`"DM"`
    via replace_all (safe: none of these 4 contain `RM_FAST`/`RM_SLOW`). Plus
    `test_experiment_config_loader.py` (`:23`/`:25` prose + `:165` comment + `:173`
    config-pin assertion — coupled to the renamed `paper_simulation_config.json`)
    and `test_compare_optimizers.py:79` (coupled to the renamed `compare_optimizers.py`
    `ALL_SCHEDULERS`).
- **STALE result outputs LEFT (D6):** `tests/radius_comparison/`, `comparison_runs/`,
  `e2e_eval_runs/`, `k_variation/`, `simulation_experiments/optimizer_comparison/runs/`
  (the last discovered during the residual sweep — the initial exclusion list missed
  it; it holds `evaluation_report.json` etc. with RM labels). All are historical run
  outputs, left for the prod A/B re-run (user-go, NOT run unilaterally).
- **Final residual sweep CLEAN:** `grep -rnE 'RM_FAST|RM_SLOW|"RM"' --include='*.py'
  --include='*.json'` (excluding stale-output dirs + `.claude/` + the 2 explicit
  leaves `draw_trajectory_error.py` + `p211_reopt_ab_config.json`) = no matches.
- **Verification:** `pytest Gen_Taskset/tests/` = **43 passed**; `pytest tests/python/`
  = **351/353** (2 fails PRE-EXISTING — `test_experiment_config_loader.py::
  TestTimeLimitConfig::test_all_configs_carry_time_limit_seconds` +
  `test_patch_time_limit.py::...::test_all_shipped_configs_readable_in_both_modes`,
  both assert `compare_against_bf.json time_limit_seconds` should be `1` but it's
  `10`; proven via `git diff` that my edit changed ONLY `main_scheduler_list`, and
  `time_limit_seconds: 10` was already in HEAD — = the P2.14-known pre-existing
  config failures, NOT introduced by P0.9); `cmake --build build_test --target
  check.SP_OPT -j5` = **17/17 green**.
- **Remaining (Step 4 + 5 tail):** P0.6/P0.8 plan refs RM→DM; SP-shift note at the
  seed (D6 — a behavior change, but no prod A/B run by me); top-level
  `agents/dev_log.md` milestone; `git add` staged for user review (no commit).

## 2026-07-28 (Step 4 records LANDED + Issue-2 review)

- **Issue-2 review (user request "check these 2 notes found by another agent"):**
  another agent flagged that `TestGateQ3` (`tests/python/test_evaluation_suite.py`)
  uses a mock key `(8, "DM")` while `Q3_BASELINES` (`evaluation_suite.py:125`) lists
  `DM_FAST`/`DM_SLOW` — claiming `evaluate_q3` therefore treats DM_FAST/DM_SLOW as
  missing baselines and the test "passes structurally but doesn't actually evaluate
  INCR against DM baseline entries". **Verified this is NOT a P0.9 regression:**
  - `git show HEAD:SP_Metric_Opt/simulation_experiments/evaluation_suite.py` shows
    HEAD already had `Q3_BASELINES = ["RM_FAST", "RM_SLOW", ...]` with the mock key
    `(8, "RM")` — i.e. the bare mock key ALREADY did NOT match RM_FAST/RM_SLOW in
    HEAD. The structural gap pre-dates P0.9.
  - `_q3_at_n` (`evaluation_suite.py:315-343`) iterates ONLY `for b in Q3_BASELINES:
    bsp = _sp(lookup, n, b)` — it NEVER reads the bare `(8, "DM")` key. So that key
    is dead weight (never consumed), and DM_FAST/DM_SLOW are "missing baselines"
    (noted in detail, NOT fatal per the gate's "missing-is-noted-not-fatal" design)
    — exactly as RM_FAST/RM_SLOW were in HEAD. The rename faithfully carried bare
    RM→bare DM. Test passes because INCR beats the PRESENT baselines
    (CFS/INCR_NO_TL/INCR_WCET) in all 3 TestGateQ3 cases.
  - The agent's RECOMMENDATION (populate the mock with real `(8, "DM_FAST")` /
    `(8, "DM_SLOW")` keys so the test actually exercises the DM baseline comparison
    instead of the missing-baseline path) is a legitimate **optional test-quality
    improvement** — but OUT of P0.9 scope (P0.9 = the rename D4 + behavior-preserving
    carry-forward; expanding it would violate "work by module, don't expand scope").
    Filed as a P2 hygiene note for later.
- **Issue-3 review:** `compare_against_bf.json time_limit_seconds: 10` vs expected
  `1` (351/353 py, 2 fails). Confirmed PRE-EXISTING — proven via `git diff` that my
  P0.9 edit touched only `main_scheduler_list`; `time_limit_seconds: 10` was already
  in HEAD. = the P2.14-known config failures. No action.
- **Step 4 LANDED (P0.6/P0.8 plan refs RM→DM):** relabeled `goal.md`+`tasks.md` in
  BOTH `P0_6_static_solution/` and `P0_8_important_task_schedulability/` via sed bulk
  (most-specific tokens first, bare `\bRM\b` last so it couldn't touch `RM_FAST`/
  `RMFast`/`ARM`): `RateMonotonicPriorityVec`→`DeadlineMonotonicPriorityVec`,
  `SeedIncumbentFromRMFast`→`...DMFast`, `BootstrapIncumbentFromRMFast`→`...DMFast`,
  `AssignRMRespectingGroupOrder`→`AssignDMRespectingGroupOrder`, `RM_FAST`/`RM_SLOW`
  →`DM_FAST`/`DM_SLOW`, "RM period-sort"→"DM deadline-sort", "RM PA"→"DM PA",
  "Rate Monotonic"→"Deadline Monotonic", bare `RM`→`DM`. Residual RM check across all
  4 files = CLEAN. P0.6's planned `AssignDMRespectingGroupOrder` extraction now
  coherently wraps the now-deadline-based orchestrator sort (bare branches call it
  with empty group = behavior-identical; static solution calls it with
  `important_ids`). **Historical `dev_log.md` entries in both folders left as
  point-in-time records** (NOT rewritten — same "don't falsify history" treatment as
  the `p211_reopt_ab_config.json` leave); each prepended with a dated 2026-07-28
  P0.9-supersedence pointer noting the relabel + that the relabeled plan docs above
  are authoritative.
- **Step 5 (records):** top-level `agents/dev_log.md` milestone appended (this task
  folder's dev_log entry = this one). Memory file `p09-dm-and-important-first-
  priority.md` + `MEMORY.md` index updated (Step 4 done, Issue-2 verdict). Step 4
  records staged for user review.
- **Commit-state correction (caught during staging):** the prior Step-2 entries here
  and the `tasks.md` said "NOT committed, `git add`-only" — that was true when
  written, but the user has SINCE committed Steps 1+2 in 3 commits: `0b9dae4a`
  (Step 1 C++ seed PA), `6a35080b` (Step 2 C++ orchestrator), `352f13d5` (Step 2
  configs/Python + a snapshot of THIS folder's `dev_log.md`/`tasks.md`). Verified
  via `git log` + `git show --stat` + `git diff HEAD` (the C++/orchestrator/config
  files are clean vs HEAD; only the Step-4 record files + unrelated pre-existing
  modifications remain uncommitted). Records above updated to reflect "Steps 1+2
  COMMITTED; Step 4 staged."
- **Remaining (Step 5 tail):** D6 SP-shift note at the seed — this is a behavior
  change, but NO prod A/B run was done by me (user-go only). If global SP moves
  materially at review, flag for prod A/B.


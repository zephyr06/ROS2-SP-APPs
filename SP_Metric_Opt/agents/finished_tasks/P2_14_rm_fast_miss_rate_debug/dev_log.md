# P2.14 — Dev Log

## 2026-07-26 — Task created; read-only investigation done (no code changes)

Created from user report: in `fig3_important_task_miss_rate.png` and
`fig3b_non_important_task_miss_rate.png`, RM_FAST does not show the lowest miss
rate, contradicting the expectation that RM_FAST (smallest time limit) should
miss least.

Established from code (facts, not verdicts):

- **RM_FAST vs RM_SLOW TL selection** (`SimulationOrchestrator.cpp:369-408`):
  both sort by period (RM priority); RM_FAST takes
  `timePerformancePairs[0].time_limit` (smallest), RM_SLOW takes `.back()`
  (largest). Pairs sorted ascending by `time_limit` (`RegularTasks.cpp:93-97`),
  so `[0]` is genuinely the smallest TL. Confirmed.
- **"Important" = top `sp_weight`, NOT top RM priority** (`utils.py:176-216`):
  important tasks are the top-`sp_weight` tasks (default 10%). `sp_weight` and
  `period` are independent, so a high-`sp_weight` / long-period task gets low
  RM priority and can miss a lot under RM_FAST. This is the leading candidate
  for a *genuine* result (b), not a bug.
- **Deadline source**: `task_deadlines = {t["id"]: float(t["deadline"])}` from
  taskset YAML (`compare_optimizers.py:624`, `run_sim_experiments.py:540`).
  A miss = `response_time > deadline`. Open: is `deadline` == `period` or a
  separate field? (S2.)
- **Figure is single-N** (prod=10, test=4), plots pre-aggregated CSV mean, one
  point/scheduler. `stds = [0.0]` hard-coded (line 702) — fake error bars. All
  points share the first scheduler's color (lines 709, 727). Missing scheduler
  falls back to `0.0` (line 699) — would plot as a perfect zero and look
  "lowest." This fallback is the leading *plotting-bug* candidate (a).
- **Data on disk**: `evalsuite_run_test_.../sim/tasks{10,16}_*` CSVs contain
  only `INCR_Reopt_10` rows (partial test runs). A full prod run with the whole
  `main_scheduler_list` was NOT found in those dirs — S1 must locate the real
  run or confirm none exists.

Next: S1 (locate run, tabulate real numbers) before any code change.

## 2026-07-26 (later) — S1+S2+S3 DONE. Verdict: **(c) data/metric-definition bug** (not a plot bug, not a genuine result)

### S1 — located the real prod run, tabulated real numbers

Run: `paper_full_run_prod_dur600_interval10_seed1000_tasks4x6x8x10x12x14x16/`
— this is a FULL prod run, all 11 schedulers including `RM_FAST`/`RM_SLOW`,
at N=4/6/8/10/12/14/16. (Earlier-memorized "only INCR_Reopt_10 rows" was a
different, partial test dir.)

`comparison_summary.csv` @ N=10 (`tasks10_dur600_interval10_seed1000/`):

| Scheduler       | Important | Non-Important | Mean    |
|-----------------|-----------|---------------|---------|
| **RM_FAST**     | **0.300** | **0.241**     | **0.322** ← HIGHEST |
| BF              | 0.300     | 0.244         | 0.308   |
| INCR_NO_TL      | 0.300     | 0.215         | 0.280   |
| INCR_WCET       | 0.082     | 0.155         | 0.234   |
| INCR_Reopt_1    | 0.063     | 0.133         | 0.204   |
| INCR_Reopt_10   | 0.064     | 0.132         | 0.208   |
| INCR_Reopt_30   | 0.064     | 0.133         | 0.212   |
| INCR_Reopt_60   | 0.064     | 0.133         | 0.211   |
| INCR_NO_REOPT   | 0.047     | 0.135         | 0.208   |
| **RM_SLOW**     | **0.000** | **0.000**     | **0.000** |
| **CFS**         | **0.000** | **0.000**     | **0.000** |

User report CONFIRMED by real data: RM_FAST is worst on BOTH subsets; RM_SLOW
and CFS are perfect zero — the *opposite* of "smaller TL ⇒ fewer misses."
The candidate-(b) `sp_weight`≠RM-priority explanation is **refuted**: it would
only explain the important subset, not why the ~90% non-important majority
also misses more under RM_FAST.

### S2 — deadline definition

`taskset_characteristics.yaml` (taskset_0): `deadline` is a **separate field,
strictly < period** (e.g. task 0: period 100 / deadline 90; task 3: 50/30;
tasks 2/8/9: 20/13 or 14). So "smaller TL ⇒ fewer misses" is the right mental
model IF "miss" means deadline miss — tighter TL ⇒ clamped ET ⇒ lower RT ⇒
fewer deadline misses. (Confirmed `ReleaseJobs` :495-497 clamps ET to the
budget: `if (budget>0 && execution_time>budget) execution_time = budget;`.)

### S3 — verdict: **(c)**. The "miss_rate" column is NOT a deadline miss rate.

Root cause: the C++ sim writes `miss_rate_*` from `r.isOverrun`
(`SimulationOrchestrator.cpp:140-159, 167-189`), and `isOverrun` is defined
at `:447` as:

```cpp
record.isOverrun = (record.executionTime >= time_limit && time_limit > 0);
```

i.e. a "miss" = **execution time exceeded the time-limit BUDGET** (a time-limit
overrun), NOT `response_time > deadline`. The Python `compute_miss_rate`/
`compute_miss_rate_by_task` fast-paths just echo the C++ summary
(`utils.py:75-85, 128-142`) — the `rt > ddl` logic at `:112/:169` is a dead
fallback only reached at export level 3, which prod does not use (export
level = 1).

This makes the observed numbers **definitional, not a bug in the data**:

- **RM_FAST** sets the *smallest* TL (`:383-386`) → ET exceeds it most often →
  **highest overrun rate by construction** (0.322).
- **RM_SLOW** sets the *largest* TL (`:403-406`) → ET rarely exceeds it →
  near-zero.
- **CFS** hard-codes `isOverrun = false` (`:635`) → always 0.000 regardless of
  real deadlines.
- `avg_response_time` in `miss_rate_per_task.txt` IS the real RT and behaves
  correctly (RM_FAST: task 3 avg RT 11ms; RM_SLOW: 4087ms) — it is just NOT
  what `miss_rate` counts.

Sanity check on raw per-task (taskset_0, task 3, period 50 / deadline 30):
RM_FAST avg RT = 11ms but 100% "missed"; RM_SLOW avg RT = 4087ms but 0%
"missed." An 11ms RT cannot miss a 30ms deadline, and a 4087ms RT cannot meet
one — so the metric provably is not `RT > deadline`. It's `ET > TL`.

### Conclusion

"RM_FAST should have the lowest miss rate" was the correct intuition **for a
deadline-miss metric**. The figure plots a **time-limit-overrun metric**
mislabeled "miss_rate." RM_FAST is *definitionally* the worst at that metric
because it sets the tightest budget; RM_SLOW is the best; CFS is trivially 0.
The figure is faithfully plotting a metric whose name lies. This is a
metric-definition bug, classification (c).

### Fix direction (NOT yet applied — await user decision)

Two separable questions for the user:
1. **What should "miss rate" actually measure?** Options:
   (i) true deadline miss = `response_time > deadline` (what the goal.md
       and the figure's name imply; matches `avg_response_time` behavior);
   (ii) keep time-limit-overrun but **rename** the column/figure to
        "time-limit overrun rate" / "budget overrun rate" so it stops lying;
   (iii) report BOTH (deadline miss AND TL overrun) as separate figures.
2. If (i): the C++ sim must compute deadline misses from
   `finishTime - releaseTime > deadline` (deadline already on the task; ET
   clamp and `isOverrun` for TL-overrun can stay as a *separate* field).
   TDD: add a test asserting task-3 RM_FAST (avg RT 11ms < ddl 30) → 0
   deadline misses, RM_SLOW (avg RT 4087ms > ddl 30) → ~100%.

The plotting quirks (missing-sched→0.0 fallback `:699`, fake `stds=[0.0]`
`:702`, single color `:709`) are **secondary** — real schedulers are all
present here so the fallback didn't fire; fixing the metric is primary.

## 2026-07-26 (fix) — Step 1 (RED) DONE. TDD plan approved; red tests in.

User picked fix direction **(i)**: recompute "miss_rate" as a true deadline
miss (`response_time > deadline`), keep `isOverrun` as a separate TL-overrun
signal. Plan at `~/.claude/plans/precious-whistling-tarjan.md` (4-step TDD:
red → green C++ → green Python docstrings → refactor/ask).

### Step 1 — RED (header + tests, no behavior change yet)

- `SimulationOrchestrator.h`: added `double deadline; bool isDeadlineMiss;` to
  `JobRecord` (kept `isOverrun` for TL-overrun). Header layout changed → build
  must use `--clean-first` (stale-`.o` ABI risk, per memory
  `sp-opt-test-build-debug-config`).
- `tests/testScheduleSimulate.cpp`:
  - `GetMockJobHistory()` initializers updated for the 2 new fields; values
    chosen so the OLD `ExportResultsLevel0/1` assertions still hold under the
    new metric (task 0 deadline 20: job 1 RT 25 > 20 → 1 miss; task 1 deadline
    30: both RT 25 ≤ 30 → 0 misses → still `missed=1`, `task0=0.5`,
    `task1=0.0`).
  - New `GetMockJobHistoryDeadlineMiss()`: 3 jobs, **1 deadline miss + 2 TL
    overruns** (asymmetric counts so Level-0 miss_rate differs between old and
    new metric — a real regression check, not coincidence).
  - New RED tests (all currently FAIL, as expected):
    `ExportResultsLevel0_DeadlineMiss`, `ExportResultsLevel1_DeadlineMiss`,
    `UnitRecordFinishedJobs_DeadlineMiss` (FTP record path; 2 tasks same finish
    time, different deadlines → one miss one not, both `isOverrun=true`),
    `CFS_RecordsDeadlineMisses` (CFS record path; job RT 15 > deadline 10 →
    miss, even though CFS hard-codes `isOverrun=false`).

Build: `cmake --build build_test --target check.SP_OPT -j5 --clean-first` →
34 existing tests PASS, 4 new tests FAIL (RED). Correct red state.

Next: Step 2 (GREEN) — set `record.deadline` + `record.isDeadlineMiss` in
`RecordFinishedJobs`/`RecordFinishedJobsCFS`, switch `ExportResults` to count
`isDeadlineMiss`.

### Step 2 — GREEN (C++ behavior change) DONE

- `RecordFinishedJobs` (`SimulationOrchestrator.cpp:448-458`): after the
  `isOverrun` assignment, set `record.deadline = dag_tasks.tasks[record.taskId].deadline;`
  and `record.isDeadlineMiss = (finishTime - releaseTime) > deadline;`.
- `RecordFinishedJobsCFS` (`:643-653`): same two lines after
  `record.isOverrun = false;`. This is the fix that lets CFS show non-zero
  `miss_rate` when it actually misses deadlines (previously CFS was trivially 0
  because `isOverrun` was hard-coded false).
- `ExportResults` count sites (`:144, :183, :220`): switched
  `if (r.isOverrun) missed++` → `if (r.isDeadlineMiss) missed++` at all three
  (overall, per-task Level-1, per-interval Level-2). `isOverrun` kept (still
  written to Level-3 `is_overrun` column + `PrintHyperperiodSchedule`).
- Level-3 `response_times_task_*.txt` `is_overrun` column and the
  `ExportResultsLevel3` test: UNCHANGED (separate diagnostic; changing its
  format is a design decision, out of scope for the metric fix).

Build: `cmake --build build_test --target check.SP_OPT -j5` → **17/17 GREEN**
(34 pre-existing + 4 new deadline-miss tests). One test-precision fix needed
mid-step: `ExportResultsLevel0_DeadlineMiss` expected `EXPECT_DOUBLE_EQ(1/3,
...)` but the value round-trips through text at 6 sig figs → switched to
`EXPECT_NEAR(..., 1e-6)` (the only non-exactly-representable miss_rate in the
suite; other tests use exact values like 0.25/0.5).

Sanity (the task-3 contradiction is resolved): `UnitRecordFinishedJobs_DeadlineMiss`
asserts two jobs with `isOverrun=true` but only the one with RT 15 > deadline 10
is a deadline miss; the other (RT 15 ≤ deadline 20) is not. `CFS_RecordsDeadlineMisses`
asserts a CFS job with RT 15 > deadline 10 is a miss despite `isOverrun=false`.

### Step 3 — GREEN (Python docstrings) DONE

`simulation_experiments/utils.py`: `compute_miss_rate` and
`compute_miss_rate_by_task` docstrings updated to state the metric is
`response_time > deadline` (a true deadline miss, NOT a TL-overrun). No logic
change — the fast-paths read `parts[2]`/`parts[3]` (now deadline-miss rates)
and the Level-3 fallback already computed `rt > ddl`. Python tests:
`pytest tests/python/test_aggregate.py` → 41 passed.

### Step 4 — refactor: `isOverrun` disposition DECIDED (user)

User chose **"Drop isOverrun entirely"** (ruthless-prune), overriding the
plan's default of keeping it as a separate TL-overrun signal. The option
description explicitly flagged this loses the TL-overrun signal; the decision
is informed. NOT yet applied — separable sub-task (touches the Level-3 column
format, `PrintHyperperiodSchedule`, and active test assertions in
`UnitRecordFinishedJobs`/`CFSOrchestration` + disabled integration tests).
Steps 1–3 are a clean reviewable GREEN unit; checkpointing here before the
Step 4 removal.

### Step 4 — APPLIED 2026-07-26 (git add-only, NOT committed)

**4a (C++ tests)** — removed every `isOverrun`/`is_overrun` reference from
`tests/testScheduleSimulate.cpp`: `GetMockJobHistory`/`GetMockJobHistoryDeadlineMiss`
aggregate initializers (dropped the positional `isOverrun` value), the
`UnitRecordFinishedJobs` / `UnitRecordFinishedJobs_DeadlineMiss` /
`CFS_RecordsDeadlineMisses` assertions, the CFS+RM "never overruns" loops,
the Level-3 header parse, and the disabled integration tests
(`DISABLED_INCR_NO_TL_Integration` dropped `EXPECT_TRUE(r.isOverrun)`;
`DISABLED_INCR_WCET_Integration` converted its isOverrun-based job
partitioning to executionTime-based buckets `task0_exec3_i0` /
`task0_exec2_i0` / `task0_clamped_i1`).

**4b (production)** — dropped `bool isOverrun;` from `JobRecord`
(`SimulationOrchestrator.h`); removed the `isOverrun` assignments in
`RecordFinishedJobs` (`:427`) and `RecordFinishedJobsCFS` (the `= false`
hardcode that used to zero CFS's miss_rate). With `isOverrun` gone, the
`res` param of `RecordFinishedJobs` became dead code → removed it entirely
(ruthless-prune): header decl (`:82`), definition (`:427`), and call site
(`:559`) are now the 3-arg `RecordFinishedJobs(time_now, run_queue, dag_tasks)`.
Also dropped the `is_overrun` column from the Level-3
`response_times_task_*.txt` header + row, and from `PrintHyperperiodSchedule`.
Build: `cmake --build build_test --target check.SP_OPT -j5 --clean-first`
(`--clean-first` because `JobRecord` layout changed — stale `.o` ABI risk,
per `[[sp-opt-test-build-debug-config]]`) → **17/17 ctest GREEN**. CFS can
now show a non-zero miss_rate for the first time (the `isOverrun=false`
hardcode that zeroed it is gone).

**4c (Python tests)** — `tests/python/test_run_sim_experiments.py` lines
49, 54, 412, 416: dropped the `is_overrun` header column and the trailing
`,0` data field on 6 rows. Safe because `compute_miss_rate` /
`compute_miss_rate_by_task` (`simulation_experiments/utils.py`) parse the
Level-3 file by **positional index** (`parts[4]` = response_time, `parts[3]`
= miss-rate summary), never by column name — the dropped column was the
7th and never read. `pytest tests/python/` → **350 passed, 2 failed**. The
2 failures (`test_all_configs_carry_time_limit_seconds`,
`test_all_shipped_configs_readable_in_both_modes`) are a pre-existing
`compare_against_bf.json` `test_mode.time_limit_seconds=10`≠1 mismatch from
commit `8c547d85` ("update exp config") — that config is unmodified by this
task and unrelated to `isOverrun`. The deadline-miss tests
(`compute_miss_rate`, `compute_miss_rate_by_task`) pass with the new
column-dropped format.

**4d (bookkeeping)** — `git add` 5 files: `simulation_experiments/utils.py`,
`SimulationOrchestrator.{h,cpp}`, `tests/testScheduleSimulate.cpp`,
`tests/python/test_run_sim_experiments.py`. (Memory `p214-rm-fast-miss-rate-debug.md`
had a stale claim that Steps 1-3 were "staged for review" — `git diff --cached`
was empty at session start; corrected here: nothing was staged before, all of
Steps 1-4 are staged now.) `overall_tasks.md` NOT edited (parallel agent owns
it); flagged row below.

### Net effect of the full fix (Steps 1-4)

`miss_rate` now reports a true deadline miss (`response_time > deadline`),
computed self-containedly on each `JobRecord` (`deadline` + `isDeadlineMiss`
fields). The `isOverrun` TL-overrun signal is GONE entirely (user choice) —
no column, no field, no dead code. CFS is no longer hard-zeroed. The figure
still needs regenerating from a prod re-run (user does the A/B).



Do NOT edit `overall_tasks.md` from this task. Suggested row for that agent to
add (under the P2 active-tasks section):

| **P2.14** Debug RM_FAST important/non-important miss rate | [`active_tasks/P2_14_rm_fast_miss_rate_debug/`](active_tasks/P2_14_rm_fast_miss_rate_debug/) | RM_FAST not lowest in fig3/fig3b; classify as plot bug vs genuine RM-priority≠sp_weight result. |

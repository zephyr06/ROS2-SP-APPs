# P2.14 — Tasks

## Standing constraints
- Agents only run `git add`, then ask user for review. Only users run
  `git commit`.
- Only add relevant changes to a commit's core purpose.
- Do NOT edit `agents/overall_tasks.md` (parallel agent owns it). Flag the
  index row for that agent to add.
- No running the A/B myself.

## Steps

- [x] **S1.** Locate the run that produced the user's `fig3_*` figure. Read its
      `comparison_summary.csv` at the target N (prod=10 / test=4). Tabulate
      `Important_Miss_Rate` + `Non_Important_Miss_Rate` for every scheduler in
      `main_scheduler_list`. Flag any scheduler missing (would plot as 0.0).
      If no full prod run exists, say so and stop.
      → DONE. Run = `paper_full_run_prod_dur600_interval10_seed1000_tasks4x6x8x10x12x14x16/`,
      all 11 schedulers present at N=10. RM_FAST = highest (0.322), RM_SLOW/CFS
      = 0.000. Table in dev_log.md.
- [x] **S2.** Confirm deadline definition from taskset YAML + generator config:
      is `deadline` == `period` or a separate field? Record in `dev_log.md`.
      → DONE. `deadline` is a SEPARATE field, strictly < period (e.g. 100/90,
      50/30, 20/13).
- [x] **S3.** Classify symptom: (a) plot/aggregation bug, (b) genuine result
      (RM-priority ≠ `sp_weight`), or (c) data/sim bug. Write verdict +
      evidence to `dev_log.md`; present to user **before** any code change.
      → DONE. Verdict = **(c)**. "miss_rate" is `isOverrun` = `ET >= time_limit`
      (`SimulationOrchestrator.cpp:447`), NOT `RT > deadline`. RM_FAST (smallest
      TL) is definitionally worst; CFS hard-codes `isOverrun=false` (`:635`).
      Candidate (b) REFUTED (non-important majority also worse under RM_FAST).
- [x] **S4 (verdict is c).** User picked **(i)** — recompute "miss_rate" as
      true deadline miss (`RT > deadline`); then DROP `isOverrun` entirely
      (Step 4, user override of the "keep as separate signal" default).
      Plan approved (`~/.claude/plans/precious-whistling-tarjan.md`).
      TDD steps:
      - [x] **Step 1 (RED):** add `double deadline; bool isDeadlineMiss;` to
            `JobRecord` (`SimulationOrchestrator.h`); add 4 failing tests in
            `tests/testScheduleSimulate.cpp`
            (`ExportResultsLevel0_DeadlineMiss`, `ExportResultsLevel1_DeadlineMiss`,
            `UnitRecordFinishedJobs_DeadlineMiss`, `CFS_RecordsDeadlineMisses`);
            update `GetMockJobHistory()` initializers (old tests stay green).
            Build: 34 pass / 4 fail (RED). DONE 2026-07-26.
      - [x] **Step 2 (GREEN):** set `record.deadline` +
            `record.isDeadlineMiss = (finishTime-releaseTime) > deadline` in
            `RecordFinishedJobs` (`:448`) + `RecordFinishedJobsCFS` (`:643`);
            switched `if (r.isOverrun) missed++` → `if (r.isDeadlineMiss)
            missed++` in `ExportResults` (`:144, :183, :220`). `isOverrun` kept
            for now (Level-3 column + integration tests). 17/17 ctest GREEN.
            DONE 2026-07-26.
      - [x] **Step 3 (GREEN):** `utils.py` docstrings only (fast-path already
            reads the right number). 41 Python tests pass. DONE 2026-07-26.
      - [x] **Step 4 (refactor):** user chose **DROP `isOverrun` entirely**
            (ruthless-prune). DONE 2026-07-26 (git add-only, NOT committed).
            - **4a (C++ tests):** removed all `isOverrun`/`is_overrun` refs from
              `tests/testScheduleSimulate.cpp` — `GetMockJobHistory`/`GetMockJobHistoryDeadlineMiss`
              initializers, `UnitRecordFinishedJobs`/`UnitRecordFinishedJobs_DeadlineMiss`/
              `CFS_RecordsDeadlineMisses`, CFS+RM "never overruns" loops, Level-3 header
              parse, `DISABLED_INCR_NO_TL_Integration`, `DISABLED_INCR_WCET_Integration`
              (converted isOverrun-based partitioning → executionTime-based).
            - **4b (production):** dropped `bool isOverrun;` from `JobRecord`;
              removed `isOverrun` assignments in `RecordFinishedJobs` (`:427`) +
              `RecordFinishedJobsCFS`; the `res` param became dead → removed it
              entirely (header decl `:82` + def `:427` + call site `:559`, now
              3-arg `RecordFinishedJobs(time_now, run_queue, dag_tasks)`); dropped
              `is_overrun` column from Level-3 `response_times_task_*.txt` header/row
              + `PrintHyperperiodSchedule`. BUILD `--clean-first` (JobRecord layout
              change) → 17/17 ctest GREEN.
            - **4c (Python tests):** `tests/python/test_run_sim_experiments.py`
              (lines 49, 54, 412, 416) — dropped `is_overrun` header column +
              trailing `,0` data field on 6 rows. `compute_miss_rate`/`compute_miss_rate_by_task`
              parse by positional index (parts[4]=rt), not column name, so safe.
              350/352 pytest pass; 2 FAILs are pre-existing
              `compare_against_bf.json` test_mode `time_limit_seconds=10`≠1 mismatch
              (commit `8c547d85`, NOT touched by this task) — unrelated to isOverrun.
            - **4d:** `git add` 5 files (utils.py, SimulationOrchestrator.{h,cpp},
              testScheduleSimulate.cpp, test_run_sim_experiments.py); records + memory
              updated; overall_tasks.md row flagged for owner agent.
      Secondary (out of scope for this fix): missing-sched→NaN (`:699`), fake
      `stds=[0.0]` (`:702`), per-scheduler color (`:709`).
- [~] **S5.** N/A (verdict is c, not b).

## Status: S4 COMPLETE (Steps 1-4 done, git add-only, NOT committed). 17/17 ctest + 350/352 pytest (2 pre-existing config FAILs unrelated). Awaits user review + commit, then USER re-runs `paper_full_run_prod_...` to regenerate fig3/fig3b.

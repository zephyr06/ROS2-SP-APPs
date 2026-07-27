# P2.14 — Debug RM_FAST Important/Non-Important Miss Rate

**Priority:** P2 (figure correctness / investigation)
**Status:** not started
**Can run during:** (no code-blocking dependency; reads existing run data)

## Symptom (user report)

In `figures/fig3_important_task_miss_rate.png` and
`fig3b_non_important_task_miss_rate.png`, **RM_FAST does not show the lowest
miss rate**, even though "theoretically RM_FAST should have the lowest miss
rate."

This task is to find out **why**, and decide whether it is:
  (a) a **plotting/aggregation bug** (wrong column, wrong N, sparse std, the
      point is mis-drawn), or
  (b) a **genuine result** (the theory is wrong — RM priority ≠ important-task
      priority, so RM_FAST can legitimately miss more on important tasks), or
  (c) a **data/sim bug** (deadlines, TL selection, or export level hiding the
      per-task files the miss rate is computed from).

## Key facts already established (read-only investigation, 2026-07-26)

These are facts from the code, **not** conclusions about the symptom. They
narrow the search:

1. **What RM_FAST / RM_SLOW actually pick** (`sources/RTDA/ImplicitCommunication/
   SimulationOrchestrator.cpp:369-408`):
   - Both sort tasks by **period** (RM priority order) — same priority vector.
   - RM_FAST sets each task's time limit to `timePerformancePairs[0].time_limit`
     = the **smallest** available time limit.
   - RM_SLOW sets it to `timePerformancePairs.back().time_limit` = the
     **largest**.
   - `timePerformancePairs` is sorted ascending by `time_limit`
     (`sources/TaskModel/RegularTasks.cpp:93-97`), so `[0]` is genuinely the
     smallest. Confirmed.

2. **What "important" means here** (`simulation_experiments/utils.py:176-216`):
   - "Important" tasks = the top `sp_weight` tasks (default top 10%,
     `important_pct`), **NOT** the top RM-priority (shortest-period) tasks.
   - `sp_weight` and `period` are independent task attributes. A high-`sp_weight`
     task can have a **long** period → low RM priority → scheduled last → more
     likely to miss its deadline under RM_FAST.
   - So "RM_FAST should have the lowest miss rate" is only obviously true for
     the **overall** miss rate, and only if a smaller time limit strictly
     reduces misses. For the **important-task** miss rate specifically, the
     RM-priority-vs-`sp_weight` mismatch can dominate. This is the leading
     candidate for (b).

3. **How the deadline is defined** (the thing a "miss" is measured against):
   - `task_deadlines = {t["id"]: float(t["deadline"]) ...}` read from the
     taskset YAML (`compare_optimizers.py:624`, `run_sim_experiments.py:540`).
   - A miss = `response_time > deadline`. The deadline comes from the taskset
     config, **not** from the time limit. (Time limit affects execution time /
     performance, which affects response time, which is compared to the
     fixed deadline.)
   - **Open question (step 1):** is `deadline` == `period`, or a separate field?
     If deadline == period, then RM_FAST's smaller TL does NOT directly tighten
     the deadline — it changes execution-time/performance, and the miss depends
     on whether the smaller TL still meets the (period) deadline. That nuance
     matters for whether the theory holds.

4. **How the figure is drawn** (`aggregate_across_tasks.py:671-736`):
   - Fig 3 is a **single-task-count** figure: `target_tasks =
     cfg["num_tasks_for_single_task_figures"]` (= **10** in prod_mode, = 4 in
     test_mode). Same N as the Fig 1F boxplot, NOT cross-task.
   - It plots the **pre-aggregated CSV mean** (`important_miss_rate` /
     `non_important_miss_rate` from `comparison_summary.csv`) — one point per
     scheduler, no per-taskset spread.
   - `stds = [0.0] * len(scheduler_list)` is **hard-coded** (line 702): the
     comment says "per-taskset data not retained in summary CSV." So the error
     bars are fake (always zero) — the figure cannot show variance even when it
     exists.
   - All points share **one color** (`color_map.get(scheduler_list[0], ...)`,
     lines 709, 727): every scheduler is drawn in the first scheduler's color.
     Visually misleading but not a data bug.
   - `means` falls back to `0.0` if a scheduler is missing from `filtered`
     (line 699): a scheduler not run at that N, or named differently, plots as
     a **zero** miss rate — which would look "best" (lowest). This is a real
     plotting hazard: if RM_FAST is absent/mis-named at N=10, it draws as 0.0
     and looks perfect; if another scheduler is absent, IT draws as 0.0 and
     undercuts RM_FAST.

5. **Data on disk (sample):** the `comparison_summary.csv` files under
   `simulation_experiments/optimizer_comparison/runs/evalsuite_run_test_.../sim/tasks{10,16}_*`
   currently contain **only `INCR_Reopt_10`** rows (partial test runs). A full
     prod run with the whole `main_scheduler_list` was not found in those dirs.
     So step 1 must first **locate a real prod run** (or confirm none exists
     yet and the figure was drawn from test_mode N=4 data).

## Approach (high-level steps)

### Step 1 — Establish ground truth from real data (no code changes)
Locate the run that produced the figure the user is looking at. Read its
`comparison_summary.csv` for the target N (10 in prod, 4 in test). Tabulate
`Important_Miss_Rate` and `Non_Important_Miss_Rate` for **every** scheduler in
`main_scheduler_list`. Confirm:
  - All main-list schedulers are present (not silently 0.0 via the fallback).
  - Which scheduler actually has the lowest important / non-important miss rate.
  - Whether RM_FAST's value is high because of real misses or because of the
    missing-scheduler→0.0 fallback mis-ordering the plot.

If no full prod run exists, say so and stop — the figure may be from stale/
partial data, which is itself the answer.

### Step 2 — Confirm the deadline definition
Read the taskset YAML + generator config: is `deadline` == `period`, or a
separate (tighter) field? This determines whether "smaller TL ⇒ fewer misses"
is even the right mental model. Document the answer in `dev_log.md`.

### Step 3 — Decide: bug or genuine result
Using steps 1–2, classify the symptom as (a) plot/aggregation bug, (b) genuine
result, or (c) data/sim bug. Write the verdict + evidence to `dev_log.md` and
present to the user **before** changing any code.

### Step 4 (only if (a) or (c)) — Fix, TDD
If it's a plotting bug (e.g. the missing-scheduler→0.0 fallback, fake stds,
single color), write failing tests first (`tests/python/test_aggregate.py`),
then fix. Likely fixes:
  - Surface a missing scheduler as NaN, not 0.0 (mirror the P1.15 empty-arm
    rule in `utils.py:write_summary_and_plots`).
  - Drop the hard-coded `stds = [0.0]` once per-taskset data is available, or
    remove the error bars honestly.
  - Per-scheduler color, not the first scheduler's color.

If it's (b) genuine result: no code change; instead document the
RM-priority-vs-`sp_weight` explanation and consider whether the figure's
caption/label should make the distinction explicit. Ask the user.

## Files

- `simulation_experiments/aggregate_across_tasks.py` (`generate_important_task_miss_rate_figure`, lines 671-736)
- `simulation_experiments/utils.py` (`compute_miss_rate`, `compute_miss_rate_by_task`, `compute_important_task_miss_rate`, lines 63-216)
- `simulation_experiments/run_sim_experiments.py` (deadline source line 540, miss-rate collection 146-158)
- `simulation_experiments/compare_optimizers.py` (deadline source line 624)
- `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp` (RM_FAST/RM_SLOW, lines 369-408)
- `sources/TaskModel/RegularTasks.cpp` (TL pair sort, lines 93-97)
- `tests/python/test_aggregate.py` (new tests, if a fix is warranted)

## Done when

- The actual miss-rate numbers behind the figure are tabulated and the
  lowest-miss-rate scheduler is identified from real data.
- The deadline definition is confirmed and recorded.
- A verdict (a/b/c) with evidence is written to `dev_log.md` and presented to
  the user.
- If a code fix is warranted, it lands TDD with tests green; otherwise the
  task closes as an explained result.

## Standing constraints (from P3.6 tasks.md, still in force)

- Agents only run `git add`, then ask user for review. Only users run
  `git commit`.
- Only add relevant changes to a commit's core purpose.
- Do NOT edit `agents/overall_tasks.md` (owned by a parallel agent); flag the
  index row for that agent to add.

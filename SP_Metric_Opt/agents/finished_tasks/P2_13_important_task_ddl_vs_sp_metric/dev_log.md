# P2.13 — Important-task DDL vs SP metric — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-25

- **Task FILED (analysis study; NO implementation).** Triggered by the user's
  finding (verbatim): "important tasks have higher miss rate as indicated in
  Important_Miss_Rate in the `.../comparison_summary.csv` file, but the overall
  SP metric are similar, why?" The user then directed: "add a new active task to
  study relationship between important task DDL and sp metrics."
- **The run the user pointed at**
  (`runs/run_test_dur300_interval10_seed1000_tasks10/sim/tasks10_dur300_interval10_seed1000/comparison_summary.csv`):
  ```
  Scheduler,Mean_SP_Metric,...,Important_Miss_Rate,Non_Important_Miss_Rate
  INCR_Reopt_1, 0.577723, ..., 0.480000, 0.117036
  INCR_Reopt_10,0.577887, ..., 0.566667, 0.115553
  ```
  SP nearly identical (Δ ≈ 0.0002); Important_Miss_Rate differs by 0.087 and is
  4–5× the non-important rate in both arms.
- **Answer found this session (4 converging reasons — to be confirmed by
  Steps 1–3):**
  1. **Different quantities.** `Mean_SP_Metric` = analytic
     `SP_Func(GetDDL_MissProbability(rta_dist, deadline), ddl_miss_threshold)
     * sp_weight` summed over tasks (`SP_Metric.cpp:12-15`).
     `Important_Miss_Rate` = empirical job-miss fraction of the single top-weight
     task (`utils.py:176-216`). Not the same signal.
  2. **`SP_Func` saturates** (`SP_Metric.h:31-41`): `RewardFunc=log(th-v+1)`,
     `PenaltyFunc=-0.01*exp(10*|th-v|)`, renormalized to [0,1]. Stays ≈1.0 until
     analytic miss-prob `v→1.0`, then plunges. For `th=0.2`:
     `SP_Func(0.48)≈0.989`, `SP_Func(0.567)≈0.981` — a 0.087 empirical miss-rate
     gap moves SP_Func by only ~0.008.
  3. **Weight dilution.** `important_task_top_percentage=0.10`, N=10 →
     `n_important = max(1, int(10*0.10+0.9999)) = 1` — ONE "important" task per
     taskset, weight ~0.1333 of Σ=1.0. The other 9 tasks (weight 0.867, low miss
     rates) dominate SP → similar SP across arms.
  4. **The important task misses 100% in BOTH arms.** In taskset_0 the top-weight
     task (task 0) has empirical `miss_rate=1.0` in both `INCR_Reopt_1` AND
     `INCR_Reopt_10` (`miss_rate_per_task.txt`) → SP contribution ~0 in both →
     no cross-arm SP difference from the task that IS the Important_Miss_Rate
     swing. The 0.087 gap is partly *which* tied-weight task got the label + sim
     noise (5-way weight tie at 0.1333: tasks 0/3/4/6/9; miss rates 1.0 / 0.23 /
     0.0 / 1.0 / 1.0).
- **Three facets to characterize (NOT fix here):**
  - **Metric facet** — is `SP_Func` too saturated / too weight-diluted to reflect
    important-task DDL tightness? (Step 3 quantifies.)
  - **Generator facet** — are important-task DDLs infeasibly tight (the P1.8 H1d
    defect: DDL independent of ET)? If task 0's DDL < its WCET, the 100% empirical
    miss is a *generator* artifact, and the flat SP is *correct* (unschedulable
    task scores ~0 in both arms). (Step 1 decides.)
  - **Selection facet** — is "1 important task, 5-way tied" too fragile?
    `Important_Miss_Rate` may be label-noise. (Step 2 decides.)
- **The decisive next step is Step 1 (feasibility check, NO new sim):** compare
  the important task's `deadline` vs `WCET` (and analytic RTA miss-prob) per
  taskset in the existing run. If DDL < WCET → D1 = "generator artifact, flat SP
  is correct, owner = P1.8." If DDL ≥ WCET yet high empirical miss → D1 = "metric
  under-penalizes, file a new task for the `SP_Func` shape (D3)."
- **Cross-links:** P2.6 (sim-RT SP column — D4 cross-check deferred to after it
  lands); P1.8 (H1d generator defect + clamp fix — the generator facet);
  `SP_Metric.h:31-41` + `SP_Metric.cpp:12-15` (the metric math);
  `utils.py:176-216` (the 1-task `Important_Miss_Rate`).
- **Registries updated this filing:** `goal.md` + `tasks.md` (this folder),
  `overall_tasks.md` (P2.13 row), top-level `dev_log.md` (P2.13 filing
  milestone), memory `p213-important-task-ddl-vs-sp-metric.md` + `MEMORY.md`
  pointer.
- **Not started.** Next action is the user's: greenlight + run Step 1 (or let me
  run Step 1, which is read-only on the existing run — no new sim, no code
  change). Standing constraints: no implementation until greenlit; no
  `git commit` (`git add` only); no YAML edit; no gate / `SP_Func` change in this
  task.

## 2026-07-25 (later) — REFRAMED + SCOPE EXPANDED

- **User reframed the task (verbatim):** "the issue is, random task sets use sp
  threshold range in 0.5 to 0.9, doesn't make sense. we'll first modify the range
  into 0.001 to 0.9, then we add empirical analysis on important tasks' DDL miss
  chances vs their SP thresholds, calculate relative difference, take average
  across different task sets, report the number. update task description, provide
  design suggestions if any, make implementation plans (not implementation yet)"
- **Reframing confirmed by code.** SP threshold = DDL-miss threshold:
  `ObtainSP` (`SP_Metric.cpp:12-16`) passes `ddl_miss_threshold` as the threshold
  arg to `SP_Func`; `SP_Func` (`SP_Metric.h:31-41`) branches
  `threshold >= violate_probability` ⇒ reward/safe, else ⇒ penalty. The "actual
  miss rate" is the analytic `ddl_miss_chance = GetDDL_MissProbability(rtas[i],
  deadline)` (`RTA.cpp:154-169`, RTA tail above deadline), NOT the empirical
  `Important_Miss_Rate`. The decisive check becomes: is the important task's
  analytic miss-prob above or below its `sp_threshold`? (Subsumes the old
  "SP_Func saturates" Step 3.)
- **Two deliverables now:**
  1. **Config-value edit** — `SP_THRESHOLD_RANGE [0.5,0.9]→[0.001,0.9]` AND scrub
     `1.0` from `SP_THRESHOLDS_SET` (the `1.0` = "100% miss acceptable" =
     always-reward-branch = unpenalizable; the real absurdity).
  2. **Empirical analysis** — important tasks' analytic `ddl_miss_chance` vs
     `sp_threshold`, relative difference, mean across tasksets, reported number.
- **Key scoping findings (grounded in the existing N=10 run):**
  - The run used `SP_THRESHOLDS_SET = [0.2,0.4,0.6,0.8,1.0]` (verified: every
    task's threshold ∈ that set), NOT the fallback range. So editing only the
    fallback `[0.5,0.9]→[0.001,0.9]` would NOT change a re-run — both knobs need
    the edit. The SET is the primary path (`taskset_generator.py:537-544`).
  - `1.0` appears on real tasks: taskset_0 task_2, taskset_2 tasks 3/10,
    taskset_3 tasks 6/8, taskset_4 tasks 5/6 — all unpenalizable by construction.
  - Per-task analytic `ddl_miss_chance` is NOT logged anywhere.
    `interval_sp_metrics.txt` (`SimulationOrchestrator.cpp:128-137,563,813`) is
    per-interval OVERALL SP (one scalar); the per-task value
    (`SP_Metric.cpp:65-66`) is summed and discarded. → New decision D5: (a) add a
    C++ per-task logging point (faithful, needs rebuild+rerun) vs (b) Python
    re-impl of `GetDDL_MissProbability` (no rebuild, binning-divergence risk).
    Recommendation: (b) for a first read-only pass, (a) if borderline.
  - **Generator facet partly already addressed by P1.8** — `feasibility_clamp.py`
    pulls `execution_time_max` inside the period + relabels DDL, BUT skips
    perf-record tasks (`not bool(performance_records_time)`). The important
    tasks in the N=10 run ARE perf tasks (taskset_0 task_1: `time_limit_task:
    true`, `deadline=90 = execution_time_max=90`, `Et_mean=54`). So the
    perf-task clamp gap is the live generator question (D1).
- **`goal.md` + `tasks.md` rewritten** to reflect the reframing + the two
  deliverables + D5 + the threshold-knob findings. Original 4-reason answer and
  3-facet framing retained as background.
- **Design suggestions surfaced:**
  1. Edit BOTH knobs (range + set), not just the range — the set is primary.
  2. Scrub `1.0` from the set (the always-safe absurdity), not just lower the
     range floor.
  3. Add a value-range assertion in `validate_config_integrity` (currently
     presence-only) to reject `sp_threshold >= 1.0` going forward.
  4. For the analysis, prefer D5 (a) if the number will be cited — the Python
     re-impl gives the *un-optimized* analytic miss-prob (the metric scores the
     post-TL-optimization RTA dist), so (b) can mislead for perf tasks.
- **Not started.** Plan only; awaiting user greenlight. Standing constraints
  unchanged: no implementation until greenlit; no `git commit` (`git add` only);
  no `parameters.yaml` edit; no gate / `SP_Func` change in this task.

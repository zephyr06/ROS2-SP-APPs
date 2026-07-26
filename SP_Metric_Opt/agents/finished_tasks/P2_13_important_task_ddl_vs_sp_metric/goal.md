# P2.13 — Important-Task DDL Miss vs Configured SP Threshold

## The Goal

Reframed 2026-07-25 (user direction): the safety signal that matters is **not**
the empirical `Important_Miss_Rate` swinging across arms, but whether each task's
**actual DDL-miss probability is below its configured SP threshold** — because
the SP threshold *is* the allowed DDL-miss threshold (confirmed in code:
`SP_Metric.cpp:12-16` passes `ddl_miss_threshold` as the threshold arg to
`SP_Func`; `SP_Metric.h:31-41` branches `threshold >= violate_probability` ⇒
reward/safe, else ⇒ penalty). If `ddl_miss_chance <= sp_threshold` the task is
"safe" by design; if `> sp_threshold` it is in the penalty regime.

This task now has **two deliverables**:

1. **Generator fix (config-value change).** The SP-threshold knobs are mis-scaled:
   - `SP_THRESHOLD_RANGE = [0.5, 0.9]` (the fallback; `taskset_generator.py:33`,
     `taskset_cfg_paper_base.json`) — a 0.5 lower bound accepts 50%+ DDL miss as
     "safe," which is absurd for a safety metric.
   - `SP_THRESHOLDS_SET = [0.2, 0.4, 0.6, 0.8, 1.0]` (the primary path,
     `taskset_generator.py:34,537-544`) — contains `1.0`, meaning "100% DDL miss
     is acceptable" (always in the reward branch, never penalizable).
   - **Fix:** range `[0.5, 0.9] → [0.001, 0.9]`; scrub `1.0` from the set
     (`[0.2, 0.4, 0.6, 0.8]` or add a sub-0.1 strict value). Config-value edit
     only; no code change.

2. **Empirical analysis (read-only, no new sim unless needed).** For the
   "important" (top-sp_weight) tasks across the existing N=10 run's 5 tasksets,
   compare each task's **analytic** `ddl_miss_chance` vs its configured
   `sp_threshold`, compute the **relative difference**
   `(ddl_miss_chance − sp_threshold) / sp_threshold`, average across tasksets,
   report the number. This directly answers "are important tasks actually safe
   by the metric's own definition?"

> **Scope note (2026-07-25 reframing):** the original 3-facet framing (metric /
> generator / selection) is retained as background but the **decisive** question
> is now the threshold-crossing check (analytic miss-prob vs sp_threshold), which
> subsumes the old "SP_Func saturation" Step 3. The generator facet is partly
> already addressed by P1.8's `feasibility_clamp.py` — but that clamp **skips
> perf-record (TL-optimizable) tasks**, which are exactly the high-weight
> "important" tasks in the run. That gap is the live generator question.

## Code grounding (confirmed 2026-07-25)

- **SP threshold = DDL-miss threshold.** `ObtainSP` (`SP_Metric.cpp:12-16`):
  `ddl_miss_chance = GetDDL_MissProbability(dist, deadline); return
  SP_Func(ddl_miss_chance, ddl_miss_threshold) * weight;`. Per-task path
  (`SP_Metric.cpp:65-71`) uses `sp_parameters.thresholds_node.at(task_id)`.
- **Safe ⇔ miss-prob ≤ threshold.** `SP_Func` (`SP_Metric.h:31-41`):
  `if (threshold >= violate_probability) val = RewardFunc(...)` (safe, →1);
  `else val = PenaltyFunc(...)` (penalized, →0). Continuous, not binary:
  `RewardFunc(v,th)=log((th−v)+1)` still varies within the safe region;
  `PenaltyFunc(v,th)=−0.01*exp(10*|th−v|)` plunges past the threshold.
- **"Actual" = analytic RTA tail, NOT empirical.** `ddl_miss_chance` comes from
  `GetDDL_MissProbability(rtas[i], deadline)` (`RTA.cpp:154-169`): sums
  `probability` for all distribution entries with `value > ddl`. This is the RTA
  tail probability, **not** the simulation's empirical job-miss fraction
  (`miss_rate_per_task.txt` / `utils.py:176-216`). The two can diverge sharply —
  that divergence is exactly what P2.6 (sim-RT SP) is filed to expose.
- **Per-task analytic miss-prob is NOT logged.** `interval_sp_metrics.txt`
  (`SimulationOrchestrator.cpp:128-137,563,813`) stores one **overall** SP scalar
  per interval; the per-task `ddl_miss_chance` (`SP_Metric.cpp:65-66`) is summed
  and discarded. `sp_metrics_summary.txt` is a single overall scalar. So the
  analysis needs a new logging point OR a Python re-implementation of
  `GetDDL_MissProbability` over the RTA distribution. (Design decision — see D5.)

## Trigger (verbatim user direction, this session)

> "the issue is, random task sets use sp threshold range in 0.5 to 0.9, doesn't
> make sense. we'll first modify the range into 0.001 to 0.9, then we add
> empirical analysis on important tasks' DDL miss chances vs their SP thresholds,
> calculate relative difference, take average across different task sets, report
> the number. update task description, provide design suggestions if any, make
> implementation plans (not implementation yet)"

Earlier trigger (retained for context): "important tasks have higher miss rate
as indicated in Important_Miss_Rate in the `.../comparison_summary.csv` file, but
the overall SP metric are similar, why?" — answered (4-reason synthesis); the
reframing above supersedes the "SP_Func saturates" emphasis with the cleaner
threshold-crossing check.

## The 4-reason answer (background; confirmed by code)

1. **Different quantities** — SP is analytic RTA-tail×weight; `Important_Miss_Rate`
   is empirical job-fraction on 1 top-weight task (`utils.py:176-216`).
2. **SP_Func saturates** — for `th=0.2`: `SP_Func(0.48)≈0.989`,
   `SP_Func(0.567)≈0.981`; a 0.087 empirical gap moves SP by ~0.008. (Subsumed by
   the threshold-crossing check: the real question is whether analytic
   miss-prob is above or below `sp_threshold`, not the SP_Func slope.)
3. **Weight dilution** — `important_task_top_percentage=0.10`, N=10 → 1 important
   task @ ~0.1333 of Σ=1.0; 9 low-miss tasks dominate SP.
4. **Important task misses 100% in both arms** — taskset_0 task 0 empirical
   `miss_rate=1.0` in both `INCR_Reopt_1`/`_10`; SP≈0 for it in both → no
   cross-arm SP difference from the swing task.

## Scope (what this task IS / IS NOT)

**IS:**
- A config-value edit to the SP-threshold knobs (range + set) in the generator
  template + the `taskset_generator.py` `suggest` defaults.
- A read-only empirical analysis on the existing N=10 run (no new sim) — OR a
  decision to add a per-task analytic-miss-prob logging point (D5).
- A written characterization + reported number (mean relative difference of
  analytic miss-prob vs sp_threshold across important tasks).

**IS NOT:**
- A change to `SP_Func` / `PenaltyFunc` / `RewardFunc` (the metric shape). If the
  analysis shows the metric under-penalizes the half-miss regime, that is filed
  as a NEW task (D3), not done here.
- A change to `compute_important_task_miss_rate` (selection facet). Tie-stability
  is D2; any fix is a separate task.
- A gate change. No north-star gate moves; no gate switches to sim SP (P2.6 D4).
- A re-derivation of P1.8 / P2.6. P2.13 *uses* their conclusions.

## Guardrails (standing constraints)

- **No `git commit`** (`git add` only when the user asks).
- **No `parameters.yaml` edit** without explicit user go (`TIME_LIMIT` 10→1 stays
  uncommitted; `ReoptimizationUseSubIncrementalWalk` flag stays 0).
- **No implementation until the user greenlights the plan.** This update is
  task-description + design suggestions + implementation plan only.
- **No new simulation** for the analysis if it can be answered from the existing
  N=10 run + the taskset params (read-only). A fresh A/B with the new threshold
  range is the user's to run.

## Open decisions (settle with the user BEFORE implementation)

- **D1 — Generator- feasibility for important (perf) tasks.** P1.8's
  `feasibility_clamp.py` pulls `execution_time_max` inside the period + relabels
  DDL, but **skips perf-record tasks** (`not bool(performance_records_time)`). In
  the N=10 run the top-weight tasks ARE perf tasks (taskset_0 task_1:
  `time_limit_task: true`). So are important-task DDLs infeasibly tight? Check
  `deadline` vs `execution_time_max`/WCET for the important tasks. If
  `ddl < wcet` → analytic miss-prob ≈ 1.0 → always in penalty branch → the
  generator (not the metric) owns it.
- **D2 — Tie-stable "important" definition?** `compute_important_task_miss_rate`
  takes 1 top-weight task; a 5-way weight tie → label-noise. Affects the
  empirical-miss side; less central under the reframing (the analytic check takes
  the top-weight task per taskset regardless of tie).
- **D3 — Metric shape?** Only if the analysis shows analytic miss-prob >
  sp_threshold yet SP still ≈ 1 (saturation hiding a real exceedance). Filed as
  a new task; NOT here.
- **D4 — Cross-link to P2.6.** Defer the sim-RT SP cross-check to after P2.6
  lands its column.
- **D5 (NEW) — How to obtain the per-task analytic `ddl_miss_chance`?** Two
  options:
  - **(a) Add a C++ logging point:** emit per-task `ddl_miss_chance` alongside
    `interval_sp_metrics.txt` (e.g. a new `per_task_sp_metrics.txt` with
    `task_id, ddl_miss_chance, sp_threshold, sp_weight, sp_contribution`). Most
    faithful (uses the exact `GetDDL_MissProbability` the metric uses) but
    requires a code change + rebuild + re-run.
  - **(b) Python re-implementation:** reconstruct the RTA `FiniteDist` in Python
    from `taskset_param.yaml` (Gaussian/truncated, binned like
    `Probability.cpp:18-43`), apply `GetDDL_MissProbability` (sum mass above
    `deadline`), compare to `sp_threshold`. No code change, no re-run; risk of
    diverging from the C++ binning.
  - **Recommendation: (a) for fidelity, (b) for a first read-only pass on the
    existing run.** Start with (b) to scope the problem, then (a) if the number
    is borderline or the user wants the canonical value.

## Threshold-knob findings (the config edit, scoped)

- **Primary path = `SP_THRESHOLDS_SET`.** `taskset_generator.py:537-544`: if the
  set is non-empty, each task's `sp_threshold = np.random.choice(set)`; the
  `SP_THRESHOLD_RANGE` fallback is only used when the set is empty. **The N=10
  run used the SET** (verified: every task's threshold ∈ {0.2,0.4,0.6,0.8,1.0}).
- **So editing only the fallback range `[0.5,0.9]→[0.001,0.9]` would NOT change
  any re-run** unless `SP_THRESHOLDS_SET` is also edited (or emptied). Both
  knobs need the edit.
- **`1.0` in the set is the real absurdity** — a task with `sp_threshold=1.0` is
  *always* in the reward branch (no miss-prob can exceed 1.0) → unpenalizable.
  taskset_0 task_2, taskset_2 tasks 3/10, taskset_3 tasks 6/8, taskset_4 tasks
  5/6 all have `sp_threshold=1.0`.
- **`0.2` already strict** (accepts only 20% miss) — the existing set already
  spans strict→absurd; the fallback range `[0.5,0.9]` is moot for paper configs.
- **No validation conflict:** `validate_config_integrity`
  (`taskset_generator.py:92-117`) checks *presence* only, not value ranges. The
  edit is a pure config-value change.

## Done when

- [ ] Config edit applied: `SP_THRESHOLD_RANGE` `[0.5,0.9]→[0.001,0.9]` and
      `SP_THRESHOLDS_SET` scrubbed of `1.0` (in `taskset_cfg_paper_base.json` +
      `taskset_generator.py:33-34` `suggest` defaults). NOT committed; `git add`
      on user go.
- [ ] Per-task analytic `ddl_miss_chance` obtained for the important tasks across
      the 5 tasksets (via D5 option (b) first, (a) if needed).
- [ ] Relative difference `(ddl_miss_chance − sp_threshold)/sp_threshold`
      computed per important task; mean across tasksets reported.
- [ ] D1 settled (feasibility of important-task DDLs; perf-task clamp gap).
- [ ] Written characterization in `dev_log.md`: is the metric's own safety
      verdict (analytic miss-prob vs threshold) consistent with "important tasks
      are safe"? If not, which facet owns it.
- [ ] If a metric-shape fix is warranted → filed as a NEW task (D3), not here.
- [ ] `agents/overall_tasks.md` + top-level `agents/dev_log.md` + memory updated.
- [ ] `git add` staged; user reviews (no commit).

## Reference docs

- `sources/Safety_Performance_Metric/SP_Metric.h:31-41` — `SP_Func` /
  `PenaltyFunc` / `RewardFunc` (the threshold-branching shape).
- `sources/Safety_Performance_Metric/SP_Metric.cpp:12-16, 65-71` — `ObtainSP` /
  `ObtainSP_TaskSet` (analytic miss-prob × weight; threshold = 2nd arg).
- `sources/Safety_Performance_Metric/RTA.cpp:154-169` — `GetDDL_MissProbability`
  (analytic RTA tail above deadline).
- `sources/Safety_Performance_Metric/ParametersSP.cpp:19-27` — `sp_threshold`
  read from YAML into `thresholds_node`.
- `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp:128-137,563,813`
  — `interval_sp_metrics.txt` is per-interval OVERALL SP (per-task not logged).
- `Gen_Taskset/lib/taskset_generator.py:33-34, 530-544` — threshold knobs
  (range fallback + set primary path).
- `Gen_Taskset/lib/feasibility_clamp.py:1-50` — P1.8 F1 clamp (skips perf tasks).
- `simulation_experiments/utils.py:176-216` — `compute_important_task_miss_rate`
  (empirical 1-task `Important_Miss_Rate`).
- Memory [`p18-incr-wcet-outperforms-incr`](../../../) — H1d generator defect +
  clamp (the generator facet; perf-task gap).
- Memory [`sim-rt-based-sp-metric`](../../../) (P2.6) — sim-RT vs analytic SP;
  D4 cross-link.
- The run: `simulation_experiments/optimizer_comparison/runs/run_test_dur300_interval10_seed1000_tasks10/sim/tasks10_dur300_interval10_seed1000/`.

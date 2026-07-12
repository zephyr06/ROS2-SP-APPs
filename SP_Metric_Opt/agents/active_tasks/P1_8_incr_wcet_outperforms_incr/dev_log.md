# P1.8 — INCR_WCET outperforms INCR — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-12

- **Task FILED + read-only investigation STARTED (NO implementation).** User
  directive (verbatim): "INCR_WCET outperforms other INCR tasks, this is
  theoretically highly unlikely, need to investigate why and fix this issue.
  first add the task, then share investigation while working on finding reasons
  and root causes."
- **The anomaly (from `comparison_summary.csv`, N=4 dur=600 interval=10
  seed=1000, 10 tasksets):** INCR_WCET Mean_SP_Metric = **0.5992**, beating
  every plain INCR arm (0.5456–0.5710, including INCR_Reopt_1 = 0.5710), with a
  lower miss rate (0.1115 vs 0.1955) and lower important-miss rate (0.2817 vs
  0.4300). Sits just below BF (0.6129). Theoretically implausible: INCR_WCET is
  a *degraded* ablation (collapses the ET dist to a WCET point mass via
  `ApplyWCETAblationIfRequired`, `OptimizeSP_TL_Incre.cpp:482-492`), explicitly
  a Q3 baseline the gate expects INCR to beat
  (`evaluation_suite.py:94` `Q3_BASELINES`).
- **Finding 1 — aggregate dominated by ONE catastrophic taskset.** Per-taskset
  `sp_metrics_summary.txt`:

  | taskset | INCR | INCR_WCET | winner |
  |---|---|---|---|
  | 0 | 0.672293 | 0.660612 | INCR ✓ |
  | **1** | **0.013331** | **0.383344** | **INCR_WCET (INCR ≈ 0!)** |
  | 2 | 0.500000 | 0.500000 | tie |
  | 3 | 0.514286 | 0.514286 | tie |
  | 4 | 0.556118 | 0.551353 | INCR ✓ |
  | 5 | 0.659167 | 0.659167 | tie |
  | 6 | 0.513865 | 0.566667 | INCR_WCET |
  | 7 | 0.562474 | 0.562474 | tie |
  | 8 | 0.548263 | 0.606097 | INCR_WCET |
  | 9 | 0.987678 | 0.987678 | tie |

  On **8/10 tasksets INCR ≥ INCR_WCET or ties** (theory holds). The aggregate
  flip is driven almost entirely by **taskset_1** (INCR collapses to 0.0133
  while INCR_WCET holds at 0.383); taskset_6 and taskset_8 add smaller
  INCR_WCET wins. **Reframing: the anomaly is NOT "INCR_WCET is mysteriously
  great" — it is "INCR catastrophically fails on taskset_1 (and mildly on 6, 8)
  while the degraded ablation survives those same tasksets."**
- **Finding 2 — the failure is in the ANALYTIC SP, not the actual schedule.**
  taskset_1 actual misses: `INCR/INCR/miss_rate_summary.txt` = `47470,0,0`
  (zero misses); `INCR_WCET/INCR_WCET/miss_rate_summary.txt` = `45079,0,0`
  (zero misses). Both schedules miss nothing. Yet INCR's *analytic* per-interval
  SP collapses: `interval_sp_metrics.txt` on taskset_1 — INCR is 0.0578 at i0,
  oscillates near 0, then **literal 0 from i52 through i59**; INCR_WCET is
  steady ~0.40 early, ~0.38 late. Consistent with the SP being analytic
  (`ObtainSP_TaskSet_And_TimeLimits` → `ProbabilisticRTA_TaskSet`, the path
  traced in P2.6's goal doc): the RTA *predicts* ~100% DDL-miss for some task
  under INCR's config, while the actual `RunQueue` schedule misses nothing.
  Exactly the analytic-vs-sim disconnection P2.6 addresses — which is why this
  failure is invisible in the miss-rate columns.
- **Finding 3 — taskset_1 contains inherently-unschedulable tasks (WCET >
  deadline).** `taskset_1/taskset_characteristics_interval_0.yaml`:
  - task 0: `period=33, deadline=18, ET_mu=27.22, ET_max=27.22` (sigma=1.0 →
    deterministic). **WCET 27.22 > deadline 18.**
  - task 2: `period=20, deadline=14, ET_mu=21.40, ET_max=21.40`. **WCET 21.40 >
    deadline 14.**
  A task with WCET > deadline is analytically unschedulable UNLESS a time limit
  strictly < deadline is adopted: `ApplyTimeLimitsToTasksExecutionTime`
  (`SP_Metric.cpp:70-80`) replaces the task's ET dist with
  `GetUnitExecutionTimeDist(time_limits[i])` — the scored ET *becomes* the
  adopted TL. If TL ≥ deadline → deterministic miss → `GetDDL_MissProbability`
  ≈ 1 → `SP_Func(1, threshold)` = 0 → SP contribution 0 (+ interference
  cascade).
- **Leading hypothesis (NOT yet confirmed):** INCR's optimizer, on taskset_1,
  adopts a TL ≥ deadline for an unschedulable task (task 0 and/or 2), driving
  the analytic SP to ~0 and getting stuck (compare-and-keep can't escape a
  flat-zero basin). INCR_WCET avoids this because `ApplyWCETAblationIfRequired`
  collapses the ET dist to the WCET point mass *before* the TL search, changing
  the `timePerformancePairs` grid and/or the perf-coefficient/SP_Func shape the
  optimizer sees (→ it adopts a TL < deadline, or a config the RTA scores as
  schedulable). Sub-hypotheses:
  - **H1a (TL-grid):** WCET ablation changes the enumerated TL options →
    INCR_WCET's grid has a TL < deadline INCR's lacks (or vice versa).
  - **H1b (perf-coef/SP_Func shape):** under the degenerate dist, a TL just
    under deadline scores much better than under the real Gaussian (whose tail
    crosses the deadline).
  - **H1c (incumbent corruption — P1.1/P1.2 family):** INCR commits a bad
    permutation early (i0–i6 SP already near 0) and the carried incumbent +
    compare-and-keep can't recover; INCR_WCET's flatter landscape never falls
    in. Would make P1.8 a sibling of P1.2.
  - **H1d (generator defect):** a task with WCET > deadline is itself a
    generator-level infeasibility (`per_core_cpu_util` calibration — P1.7
    touched it). If so, INCR is faithfully reporting unschedulability and the
    "fix" is the generator; INCR_WCET's 0.383 is then the bug (masks it).
  Not mutually exclusive: H1d may be the underlying defect, H1a/H1b the reason
  the WCET ablation masks it.
- **Registries updated this filing:** `goal.md` + `tasks.md` + this `dev_log.md`
  (this folder), `overall_tasks.md` (P1.8 row + suggested-order entry),
  top-level `dev_log.md` (P1.8 milestone), memory (P1.8 entry + MEMORY.md
  pointer). [memory + registries: TODO next.]
- **Next actions (Step 0, read-only):** (1) dump INCR's adopted TLs +
  priorities on taskset_1 interval-by-interval (instrument
  `DeterminePrioritiesAndBudgets` or read exported artifacts — NO optimizer
  code change); (2) dump the `timePerformancePairs` TL grid INCR sees vs
  INCR_WCET sees for the unschedulable tasks (H1a); (3) compute
  `ObtainSP_TaskSet_And_TimeLimits` by hand for taskset_1 under INCR's adopted
  TLs vs a TL < deadline (disambiguates H1a/H1b from H1d); (4) check whether
  Q3's verdict actually moved on this N=4 run (Q3 is scoped to N=8/10).
- **Standing constraints in effect:** no implementation until root cause
  confirmed + user greenlights a fix; no `git commit` (`git add` only); no A/B
  run by me (user runs `run_end_to_end.sh`); no unilateral design decisions
  (D1–D4 settled with the user per `agent_coding_rules.md`).

## 2026-07-12 (cont'd) — user's pessimistic-RTA reasoning + structural findings

- **User added a reasoning channel to evaluate (verbatim):** "UNDER incr_wcet,
  SINCE TASKS'S et IS more pessimistic, tasks' RTA is more pessimistic,
  therefore, during time limit optimization, it is much more likely that time
  limit configurations will adopt fast time limit and therefore less SP values.
  follow this reasoning, evaluate whether it's indeed the case for INCR_WCET."
  Recorded in `goal.md` ("User's reasoning to evaluate") + `tasks.md` (Step 0.5).
- **The channel is mechanistically correct** — `ApplyWCETAblationIfRequired`
  (`OptimizeSP_TL_Incre.cpp:482-492`) → `GetUnitExecutionTimeDist(max_et)`
  (`Probability.h:173-176`) → feeds `ProbabilisticRTA_TaskSet` (`RTA.cpp:100`).
  A point-mass-at-WCET interference term is the maximally pessimistic case vs a
  Gaussian whose tail thins. So the WCET arm's RTA IS more pessimistic. ✓.
- **BUT it predicts the wrong sign on taskset_1:** it says INCR_WCET ≤ INCR
  (tighter TLs ⇒ less SP), and the data is INCR_WCET (0.383) >> INCR (0.0133).
  So the channel is either not firing here or is dominated by an opposite-sign
  effect. Recorded as Finding 5 in `goal.md`.
- **Finding 4 — STRUCTURAL: only task 3 has TL freedom in taskset_1.**
  `RecordTimeLimitOptions` (`OptimizeSP_TL_BF.cpp:18-34`) enumerates TL options
  directly from `timePerformancePairs`; a task with no pairs gets `{-1}` only
  (the "no TL" sentinel; `ApplyTimeLimitsToTasksExecutionTime` at
  `SP_Metric.cpp:73-77` only substitutes when `time_limits[i] != -1`). In
  taskset_1's `taskset_characteristics_interval_*.yaml`, **only task 3 carries
  `performance_records_*`** — confirmed across intervals 0/10/20/30/40/50/52/59
  (always exactly 1 task = task 3). Tasks 0/1/2 have `{-1}`-only grids under
  BOTH arms (the ablation collapses the ET dist, it does not add pairs).
  **Consequence:** (a) the original leading hypothesis "INCR adopts TL ≥
  deadline" is **wrong as written** — INCR adopts `-1` (no TL); the SP→0 is from
  the RTA over the *raw* (unlimited) ET dist of a WCET>deadline task, not from an
  adopted TL ≥ deadline. (b) H1a (TL-grid differentiator) **cannot** be the flip
  on taskset_1 — both arms see identical `{-1}`-only grids for 0/1/2. (c) The
  user's "adopt tighter TL" channel can only fire on **task 3** (the sole
  TL-freedom task) — a minor partial-weight SP *decrease* on INCR_WCET. Wrong
  sign + minor → not the dominant effect.
- **Finding 5 — the WCET collapse DOES reach the scored SP (not just the
  search).** `OptimizeIncre_w_TL` / `ReOptimizePeriodic` do `dag_tasks_ =
  dag_tasks_update; ApplyWCETAblationIfRequired(dag_tasks_);` in place
  (`OptimizeSP_TL_Incre.cpp:313-314`, `:452-453`). The interval SP is scored on
  the same mutated `dag_tasks` (`SimulationOrchestrator.cpp:502` `DAG_Model&
  dag_tasks`, scored at `:553`/`:803` via `ObtainSP_TaskSet_And_TimeLimits`). So
  INCR_WCET's scored ET dist IS the collapsed point mass. The pessimistic-RTA
  channel is live in the metric, not hidden in the optimizer. (TO VERIFY
  end-to-end with a debug print, but the reference semantics support it.)
- **The live candidate for the actual flip = 5b (interference-tail asymmetry on
  the heavy-sigma task 1).** Task 1: mu=128.9, **sigma=47**, max=195.9, period
  500, deadline 380. Under INCR its raw Gaussian feeds the RTA of tasks it
  interferes with; under INCR_WCET the interference is a point mass at max=195.9.
  IF `FiniteDist` does NOT truncate the Gaussian at max_et, the Gaussian tail
  exceeds the WCET point mass → INCR's interference is MORE pessimistic than
  INCR_WCET's → INCR worse → matches the data (the user's prediction, flipped).
  IF it DOES truncate, the two arms equalize on task 1 and the flip must be
  elsewhere (path/chain SP terms). **Next step: confirm whether `FiniteDist`
  truncates the Gaussian at max_et** (`Probability.h:73-79`) — single fact that
  decides 5b.
- **Verdict on user's reasoning:** correct in mechanism, **wrong sign for the
  observed flip** unless 5b holds (Gaussian not truncated → INCR's heavy-tail
  interference is the *more* pessimistic one, which is the user's channel acting
  on INCR rather than INCR_WCET). Recorded in `goal.md` Finding 5 + verdict.

## 2026-07-12 (cont'd) — generator feasibility: the unschedulable-task problem is SYSTEMIC

- **User flagged a second generator issue (verbatim):** "i found one issue in the
  logic to generate random task sets. in [...]taskset_1, task id=2, ET mu is 21,
  while period is 20. i should have explicitly requested that tasks's avg ET
  cannot exceed a certain portion of period. first check whether the generated
  task set satisfy our task set generation requirements, add this note to the
  task P1.7 too." (User said "P1.7" — the recurring P1.7↔P1.8 slip; context is
  unambiguously the taskset_1 investigation. Recorded into P1.7 as a follow-up
  note + here.)
- **Empirical sweep of all 10 generated tasksets** (interval 0, 40 tasks),
  checking ET vs period and WCET vs deadline:

  | violation | tasks | tasksets |
  |---|---|---|
  | `mu > period` (the user's flag) | 2/40 | 2/10 (ts1 t2: 21.4/20; ts7 t2: 518.1/500) |
  | `max > deadline` (WCET>DDL) | ~18/40 | **9/10** (only ts9 clean) |
  | `mu > DDL` & σ=1.0 (deterministic certain-miss) | 5/40 | 5/10 (ts0 t3, ts1 t0+t2, ts5 t3, ts7 t3) |

- **Verdict: the unschedulable-task problem is SYSTEMIC, not a one-off.** The
  user flagged `mu > period`; the broader `WCET > deadline` case is ~9× more
  prevalent and was already P1.8 Finding 3's substrate. A WCET>DDL task is
  analytically unschedulable unless a TL < deadline is adopted — and most of
  these tasks have NO `timePerformancePairs` (no TL ladder), so no TL can
  rescue them. → **strongly supports H1d (generator defect).**
- **Necessary-but-not-sufficient:** WCET>DDL is endemic (9/10 tasksets) yet only
  taskset_1 collapses INCR to 0.0133. So WCET>DDL alone doesn't cause the INCR
  collapse — the PA/interference topology (H1c) is what makes taskset_1
  catastrophic. H1d is the underlying generator defect; H1c is the mechanism of
  the *collapse* on top of it. (Consistent with the goal.md note that H1d and
  H1a/b/c are not mutually exclusive.)
- **Source-trace DONE — H1d CONFIRMED in generator source.** The generator is
  `Gen_Taskset/lib/taskset_generator.py` (`run_full_generation_pipeline` in
  `Gen_Taskset/lib/orchestrator.py`, called from
  `simulation_experiments/compare_optimizers.py:451-480`). Two compounding
  defects, BOTH with **zero feasibility guard**:
  1. **Deadline drawn INDEPENDENTLY of ET** — `taskset_generator.py:533`:
     `deadline = int(round(period * random.uniform(0.5, 1.0)))`. No check that
     `deadline > et_mean` (or `> et_max`). Direct cause of `mu > deadline`
     (taskset_1 t0: mu=27.2, deadline=18; t2: mu=21.4, deadline=14). The
     deadline/period *ratio* is bounded (0.5–1.0), but deadline vs ET is not.
  2. **WCET = `et_mean ± 2*sigma` with no cap** — `taskset_generator.py:598-599`:
     for non-perf tasks, `execution_time_max = max(1.0, et_mean + 2*sigma)`. For
     env tasks (large sigma) this can blow past period AND deadline; for
     deterministic tasks (sigma≈1.0, the certain-miss tasks) `max ≈ mu > deadline`.
     No check that `execution_time_max ≤ deadline` (or `≤ period`).
  - The UUniFast step (`:429`, `:496` `et_mean = max(1.0, u_i * period)`,
    `u_i < MAX_UTIL_PER_TASK=0.95`) DOES bound the *mean* below period in
    principle — but it is undone by (1) the independent deadline draw and (2) the
    ±2σ WCET. So `mu > period` (the user's flag, 2/40 tasks) arises when
    UUniFast's `max_util_cap` fallback (`:163-164`: `required_min_cap >= cap` →
    `cap = required_min_cap + 0.001`) fires under high `CPU_UTIL_RANDOM_RANGE`
    (config `[0.5,1.5]` × `N_CORES=2` → up to 3.0 total util across 4 tasks →
    avg 0.75/task, but UUniFast can assign one task >1.0 when redistributing).
  - `per_core_cpu_util` calibration = `:393`
    `per_core_cpu_util = random.uniform(CPU_UTIL_RANDOM_RANGE[0], [1.5])`,
    `cpu_util = per_core_cpu_util * n_cores` (`:394`) → fed to UUniFast. So the
    util draw is the upstream knob, but the *unschedulability* is the missing
    deadline-vs-ET and WCET-vs-deadline guard, not the util value per se.
  - **Grep-confirmed ZERO feasibility guards** across `Gen_Taskset/lib/`: no
    `deadline > et` comparison, no reject/regenerate, no clamp. The only
    `while True` (`orchestrator.py:133`) is a path-file index scan, not a retry.
  - **F1 fix shape (for D1):** add a post-generation feasibility pass — either
    (a) draw deadline as `uniform(et_max, period)` (clamp deadline ≥ et_max + ε),
    or (b) reject+regenerate the taskset if any `et_max > deadline` or
    `et_mean > k*period`. (a) changes the deadline distribution; (b) preserves it
    but adds a rejection loop. User's framing was "avg ET cannot exceed a certain
    portion of period" → also clamp `et_mean ≤ k*period` (k < 1) at `:496`/`:525`,
    which the UUniFast cap already approximates but doesn't guarantee under the
    fallback.
- **Cross-recorded:** P1.7 (`finished_tasks/P1_7_cpu_partition_mismatch/goal.md`)
  got a "Follow-up note — generator feasibility, the D1 separate pass" section:
  P1.7's D1 always deferred the generator pass as a separate decision; this is
  that pass, now active under P1.8's D1 (not a P1.7 reopen — P1.7's simulator
  fix stays standalone-resolved).

### CORRECTION to Finding 5 (same day, after tracing value semantics)

- **5b is FALSE and the "WCET collapse reaches the scored SP" claim was WRONG.**
  Two corrections from reading the source:
  1. **`FiniteDist` DOES truncate the Gaussian at max_et** —
     `Probability.cpp:18-43` constructor bins mass from `min_val` to `max_val`
     and dumps ALL upper-tail mass onto the `max_val` bin (line 37-38:
     `Value_Proba(max_val, 1.0 - sum_probability_added)`). So the Gaussian tail
     does NOT exceed max_et. My 5b candidate (Gaussian tail > WCET point mass) is
     false as stated. Both arms' task-1 dist tops out at the same max_et=195.89.
  2. **The WCET collapse is CONFINED to the optimizer's internal search; it does
     NOT reach the scored metric.** `Optimize_w_TL_ScratchOrIncre` takes
     `const DAG_Model& dag_tasks_update` (`OptimizeSP_TL_Incre.h:87`) — by const
     ref. Inside, `dag_tasks_ = dag_tasks_update;` (`:313`/`:452`) is a VALUE
     COPY (DAG_Model has value semantics — `TaskSet tasks` is a value member,
     `DAG_Model.h:112`). `ApplyWCETAblationIfRequired(dag_tasks_)` mutates that
     copy, not the orchestrator's `dag_tasks`. The scored SP
     (`SimulationOrchestrator.cpp:553`,
     `ObtainSP_TaskSet_And_TimeLimits(dag_tasks.tasks, sp_parameters,
     time_limits)`) reads the orchestrator's `dag_tasks_vecs_[interval]` — the
     UNCOLLAPSED original. `ApplyTaskConfigurations` (`:507`) sets only
     `priority` + `setExecutionTime(avg)`, NOT `execution_time_dist`. **So the
     scored ET dist is the original truncated Gaussian for BOTH arms.** The
     user's pessimistic-RTA channel operates ONLY on the optimizer's internal
     objective (which {PA, TL} it commits to); the metric is scored on identical
     dists.
- **Re-pointed root cause: the differentiator is the PRIORITY ASSIGNMENT (PA),
  not the TL.** With ET dists identical across arms and TL differing only on
  task 3 (the sole TL-freedom task), the only first-order differentiator in the
  scored SP is the committed `priority_vec` (baked into
  `dag_tasks.tasks[*].priority` by `ApplyTaskConfigurations` at `:507`, read by
  `ProbabilisticRTA_TaskSet`). INCR and INCR_WCET commit to different PAs
  because their internal search landscapes differ (INCR_WCET's collapsed
  objective is flatter / lands in a different basin). INCR's PA catastrophically
  worsens RTA interference on the WCET>deadline tasks 0/2 → their response-time
  dists blow past the deadline → `SP_Func(miss≈1, threshold)` floors at 0 (the
  penalty branch's min_val = `PenaltyFunc(1, threshold)` is the floor by
  construction, so vp=1 → SP_Func=0, not negative) → SP contribution 0 +
  interference cascade drops other tasks from +positive to 0.
- **Revised verdict on user's reasoning:** mechanistically correct, but (a)
  operates only on the internal search objective (value-copy confinement), not
  the scored metric, and (b) on taskset_1 can only act on task 3 → minor +
  wrong-sign. **Cannot explain INCR's collapse.** The actual flip is the PA
  (H1c-class: search landscape / incumbent), not the TL/perf-coef (H1a/H1b).
- **Next step (Step 0 cont'd):** dump the adopted `priority_vec` + per-task
  `rtas[i]` for INCR vs INCR_WCET on taskset_1 (instrument
  `ProbabilisticRTA_TaskSet` or a debug print in `SimulateInterval` before the
  `:553` score) — confirms whether the PA is the differentiator. `FiniteDist`
  truncation question is RESOLVED (truncated → 5b false). `tasks.md` Step 0
  updated.

## 2026-07-12 (cont'd) — generator source read in full; F1 fix design grounded

- **User directive:** "continue last task on task set generation logic design"
  → the F1 generator fix is the active work item. Standing constraints honored:
  NO implementation until root cause confirmed (✓ done) AND user greenlights a
  fix; D1–D4 settled WITH the user, not unilaterally. This entry = read the
  generator source in full + produce a concrete F1 design for the greenlight.
- **Generator source fully traced** (`Gen_Taskset/lib/taskset_generator.py`,
  `generation_config_parser.py`, `gmm_model.py`, `orchestrator.py`, the base
  template `task_sets_config/templates/taskset_cfg_paper_base.json`, the call
  site `simulation_experiments/compare_optimizers.py:451-480` →
  `run_full_generation_pipeline`, and the C++ consumer
  `sources/TaskModel/RegularTasks.cpp:60-106`). The two H1d defects are
  confirmed end-to-end, and the **WCET>DDL catastrophe is now traced all the
  way to the scored SP** (closes the "is it real unschedulability or just
  RTA pessimism?" question):
  1. `taskset_generator.py:533` — `deadline = int(round(period * random.uniform(0.5, 1.0)))`.
     Deadline/period ratio ∈ [0.5,1.0] but deadline-vs-ET is unconstrained.
     (`gmm_model.py:66` defaults `self.deadline = period`; the `:533` draw is
     the only thing that lowers it, and it never consults `et_mean`/`et_max`.)
  2. `taskset_generator.py:598-599` — for non-perf (normal+env) tasks,
     `execution_time_max = max(1.0, et_mean + 2*sigma)`, uncapped vs
     `deadline` AND vs `period`. (Perf tasks use `period * FINAL_Et_OVER_PERIOD_RANGE[1]`
     = `period*0.9`, also uncapped vs deadline but always < period.)
  3. `RegularTasks.cpp:77-79` — `execution_time_max` becomes the `FiniteDist`
     upper support bound of the **scored** ET distribution
     (`FiniteDist(gauss, et_min, et_max, granularity)`). So WCET>DDL is not an
     optimizer-internal artifact: the scored ET distribution's support genuinely
     crosses the deadline.
  4. `RTA.cpp:19,25` + `Probability.cpp:145-157` `CompressDeadlineMissProbability`:
     when the ET-distribution support exceeds the deadline, ALL tail mass above
     the deadline is lumped into a single `(deadline+1, ddl_miss)` bin → the
     analytic miss probability for that task → ~1 → `SP_Func(~1, threshold)`
     floors at 0 (+ interference cascade to lower-priority tasks).
  - **This settles D3 directionally:** the tasksets are *genuinely*
    analytically-unschedulable (scored ET support crosses the deadline), so
    INCR's ≈0 on taskset_1 is HONEST and INCR_WCET's 0.383 is the artifact (it
    commits a PA that happens to dodge the worst interference). The fix lever
    is the generator (F1), not the optimizer (F2) or scoring (F4). F3
    (incumbent escape) is mooted for the collapse mechanism but the substrate
    is the generator; removing the substrate is the clean fix.
- **Existing test gap confirmed:** `test_integration.py:90` already asserts
  `0.5*period <= deadline <= period` but asserts NOTHING tying `deadline` to
  `execution_time_max` or `et_mean` — exactly mirroring the generator's missing
  guard. A red-then-green TDD test for F1 fits cleanly here (or a new
  `test_feasibility.py`).
- **F1 fix design (to bring to the user for greenlight — NOT implemented):**
  See `goal.md` § "F1 fix design (proposed)" for the full proposal. Summary:
  a deterministic, in-place feasibility repair pass at the END of
  `generate_taskset_parameters` (after `:533` deadline draw + `:598-599`
  `execution_time_max` compute, before serialization at `:562`), enforcing
  `et_mean ≤ k·period` and `execution_time_max ≤ deadline` per task, plus
  optional reject-and-regenerate. Three open knobs the user must pick (k,
  repair-vs-reject, deadline-floor-vs-ET-aware deadline draw). NO code written.
- **D1–D4 status going into the greenlight ask:**
  - D1 (generator bug vs stress) — RESOLVED by Finding 6 + this trace: it is a
    generator bug (systemic, 9/10 tasksets; zero guard in source). Fix = F1.
  - D3 (INCR's ≈0 vs INCR_WCET's 0.383) — RESOLVED directionally by the
    `CompressDeadlineMissProbability` trace: INCR's ≈0 is honest (scored ET
    support genuinely crosses DDL); F1 removes the substrate.
  - D2 (forbid TL≥DDL in optimizer) — MOOTED for now: F1 removes the
    unschedulable substrate, so the optimizer never faces a TL≥DDL-adopt
    dilemma on a clean taskset. Revisit only if the user wants some
    unschedulable stress preserved by design.
  - D4 (re-run scope) — still open; ask the user (N=4 taskset_1 probe first
    vs full eval suite).
- **Remaining sub-decisions for the user (the greenlight gate):**
  (a) the `et_mean ≤ k·period` constant `k` (user's "certain portion of period"
  — candidate 0.9 to match `MAX_UTIL_PER_TASK`/`FINAL_Et_OVER_PERIOD_RANGE[1]`);
  (b) repair-in-place vs reject-and-regenerate (repair preserves the UUniFast
  util vector + periods; reject preserves the deadline/ET distributions);
  (c) for repair, whether to also re-draw the deadline as `uniform(et_max, period)`
  (changes the deadline distribution) or only clamp `execution_time_max ≤ deadline`
  (preserves the deadline draw, shrinks the ET support);
  (d) D4 re-run scope.

## 2026-07-12 (cont'd) — user greenlight + the non-env/trace data-flow subtlety

- **User greenlight (verbatim, via AskUserQuestion):**
  - "i don't need et_mean<k*period, only et_mean < period. there are no
    requirements on et_max, it could exceed deadline or period. as for
    guarantee, try to re-generate for at most 5 times. if doesn't work, we'll
    keep the last generated task set."
  - k value: **k=1** (i.e. `et_mean < period`, strict).
  - clamp scope: **"no restrict on et_max"** — WCET may exceed deadline/period;
    only the *mean* is bounded.
  - mechanism: **reject-and-regenerate, ≤5 retries; keep the last on failure**.
  - D4 re-run: "i'll do it myself" (user runs the A/B; I do NOT).
  → **The fix is narrower than my proposed (A)+(B): NO et_max clamp, NO
  ET-aware deadline redraw. Only `et_mean < period` (strict), enforced by a
  ≤5-retry regenerate loop with keep-last fallback.**
- **Data-flow subtlety surfaced (NOT yet decided — see the ask below):**
  Tracing WHERE the flagged `execution_time_mu=21.4 > period=20` originates:
  - `taskset_param.yaml` (generator output) task 2: `Et_mean=21.404` (u_i=1.0702,
    env=False). So the GENERATOR's UUniFast already assigned `et_mean > period`
    to this non-env task — confirmed by `per_core_cpu_util=1.297` × 2 cores =
    `cpu_util=2.593` across 4 tasks, with UUniFast's `MAX_UTIL_PER_TASK=0.95`
    cap failing to hold (the `:163-164`/`:184-206` fallback path fired under
    high total util, leaving task 2 at u_i=1.07). → **A generator-level
    `et_mean < period` check DOES fire on this task.** Good — the user's check
    is the correct lever for the non-env case.
  - BUT `taskset_characteristics_interval_0.yaml` `execution_time_mu=21.40` is
    NOT read from `taskset_param.yaml`'s `Et_mean` directly. Per
    `yaml_exporter.py:52-56`, when `Et_actual` is present (set by
    `orchestrator.py:321` to `task_Ets[i][k_val]`, whose `Et_mean` is
    `np.mean(interval_ets)` from `orchestrator.py:309`), the output
    `execution_time_mu` is the **mean of the actually-sampled execution-time
    trace**, not the generator's UUniFast `Et_mean`.
  - For NON-ENV tasks, `trace_generator.py:76-79` makes the trace deterministic:
    `et = task_param['Et_mean']` every step → empirical trace mean ==
    generator `Et_mean`. So a generator-level check transitively covers the
    non-env output `execution_time_mu`. ✓
  - For ENV tasks, `trace_generator.py:68-75` samples
    `task_model.sample_execution_time(d1, d2, final_et_range, et_min_2sigma=True)`
    — a spatially-varying GMM sample whose empirical mean CAN exceed both the
    generator `Et_mean` AND `period` (large sigma + map-edge position +
    `et_min_2sigma` clamp). So a generator-level `et_mean < period` check does
    NOT necessarily bound the env task's output `execution_time_mu` — the
    violation can arise in the trace layer even when the generator's
    `Et_mean < period`.
  - **Implication:** the user's "regenerate ≤5 times" check, applied at the
    generator level (`generate_taskset_parameters`), will catch the flagged
    taskset_1 case (non-env task 2) but may NOT catch env-task
    `execution_time_mu > period` violations that arise post-trace. Two
    coherent scopes:
    (i) **Generator-level only** — check `et_mean < period` on the UUniFast
        output in `generate_taskset_parameters`; regenerate the taskset (re-run
        `generate_taskset_parameters`) ≤5 times. Catches non-env violations
        (incl. taskset_1). Does NOT catch env-task trace-layer violations.
        Simplest; matches the user's "re-generate the task set" framing
        literally.
    (ii) **Pipeline-level** — check `execution_time_mu < period` on the FINAL
         per-interval characteristics (post-trace, in `generate_additional_execution_traces`
         or `run_full_generation_pipeline`); regenerate the whole pipeline ≤5
         times. Catches both non-env and env violations. More invasive; the
         "task set" being regenerated is the full trace output.
  - This is a genuine fork the user must pick (it changes WHERE the check lives
    and WHAT "regenerate" re-runs). Surfaced via AskUserQuestion before coding.

## 2026-07-12 (cont'd) — user's SIMPLER design: post-generation clamp pass

- **User replaced the regenerate-loop design with a simpler clamp (verbatim):**
  "i have a simpler design. so after task set and all intervals' yaml files are
  generated, we go through all of them once. for each task whose avg ET exceeds
  0.95*period, we just clamp avg ET back to 0.95*period. if that happens, we
  also re-generate deadline as period. otherwise, nothing changes. first
  evaluate whether this makes sense to you, record it into tasks list, then
  implement"
  → **Supersedes the ≤5-retry regenerate loop.** The fix is now a single
  deterministic post-generation pass over the emitted YAMLs: clamp-and-relabel,
  no regenerate, no keep-last fallback, no RNG retry.
- **My evaluation: the design is sound in goal; one mechanical gap must be
  closed for the clamp to actually change the scored SP.** Tracing the scored
  distribution's construction end-to-end:
  1. **Two YAML files, different field names.** `taskset_param.yaml` (generator
     output) uses `Et_mean`; the C++-facing `taskset_characteristics_*.yaml`
     (what `ReadTaskSet` consumes) uses `execution_time_mu`. The clamp MUST
     target the latter — that is the scored field.
  2. **C++ builds the scored `FiniteDist` from `mu`, `sigma`, `min`, `max`**
     (`RegularTasks.cpp:75-79`): `GaussianDist(mu, sigma)` →
     `FiniteDist(gauss, execution_time_min, execution_time_max, granularity)`.
  3. **`FiniteDist` truncates the Gaussian at `execution_time_max`**
     (`Probability.cpp:18-43`): it bins mass from `min_val` to `max_val` and
     dumps ALL upper-tail mass onto the `max_val` bin (line 37-38). So the
     **scored support upper bound is `execution_time_max`, NOT `mu`**. Clamping
     `mu` alone, leaving `max` > `0.95*period`, leaves the scored support
     crossing the period → the unschedulability the clamp is meant to remove
     would persist.
  → **Therefore the per-task action, when `execution_time_mu > 0.95*period`,
  must clamp `mu` AND `execution_time_max` (and `execution_time_min` if it
  exceeds the clamped max) down to `0.95*period`, plus set `deadline = period`.**
  Clamping only `mu` is a no-op on the scored SP for any task whose `max` is
  tied to `mu` (the deterministic case) OR whose `max` already exceeds `mu`
  (the env/perf case). The user's "clamp avg ET back to 0.95*period" is honored
  as the trigger AND the mu target; the `max`/`min` clamp is the necessary
  consequence to make that mu clamp reach the scored distribution. Recorded as
  a design note, surfaced to the user below.
- **Per-task-type consequence (verified against the existing N=4 run):**
  - **Deterministic non-env tasks** (σ≈1.0, `min==max==mu`): e.g. taskset_1
    task 2 (`mu=21.4, min=max=21.4, period=20`). `min==max==mu`, so clamping
    `mu` to 19 moves the point mass to 19 ONLY IF `min`/`max` are also moved
    (else `min==max==21.4` still → scored dist unchanged). The `max`/`min`
    clamp is mandatory here, not optional. **Effective** when applied.
  - **Env tasks** (large σ): in the current N=4 data NONE fire the clamp (the
    largest env `mu` is taskset_2 task 0: `mu=414.6 < 0.95*500=475`). The
    user's forward-looking "reduce env initial ET so grown ET stays acceptable"
    concern is the motivation, but the clamp as specified is uniform across
    task types (fires on `mu > 0.95*period` regardless of env-ness) — so if a
    future env task's realized trace mean exceeds `0.95*period`, this same pass
    catches it. No env-specific scaling factor is added (the user's "reduce
    initial ET" is realized indirectly: any task whose avg ET drifts above
    `0.95*period` is pulled back, env or not).
  - **Perf tasks** (`performance_records_time` present): **MUST be skipped.**
    Their `execution_time_min`/`execution_time_max` are the TL-OPTION GRID
    bounds (`period*0.05 .. period*0.9`, set at `taskset_generator.py:582-583`
    and re-asserted at `yaml_exporter.py:74-75`), semantically distinct from
    the ET-distribution support. Clamping `mu`/`max` on a perf task would
    CORRUPT the TL grid the optimizer searches. Verified this is not
    hypothetical: taskset_7 task 2 is a perf task with `mu=518 > 0.95*500=475`
    (`min=25, max=450` are its grid bounds). → **gate the clamp on
    `not bool(performance_records_time)`.**
- **Empirical clamp-firing sweep on the existing N=4 run** (10 tasksets, 40
  tasks, interval-0 characteristics): the clamp would fire on **2 tasks** —
  taskset_1 task 2 (non-env deterministic, the user's flagged `mu=21.4>20`) and
  taskset_7 task 2 (perf — skipped by the perf gate). So **only 1 task is
  actually clamped** in this run. The other 38 tasks are already
  `mu ≤ 0.95*period`. This is a small, surgical change to the scored SP — it
  repairs taskset_1's `mu>period` substrate directly (the user's original flag)
  without disturbing the 9 well-behaved tasksets.
- **Scope of files the pass walks:** every `taskset_characteristics*.yaml` in
  the generation output dir — i.e. `taskset_characteristics.yaml` (the k=0
  backward-compat copy), every `taskset_characteristics_interval_{k}.yaml`, and
  every per-processor `taskset_characteristics_i{k}_p{p}.yaml`. All carry the
  same `{period, deadline, execution_time_mu/sigma/min/max,
  performance_records_time}` fields (the per-processor files are filtered
  subsets of the same interval; `convert_taskset_parameters_to_cpp_yaml` with
  `iprocessorId` just drops tasks not on that core — the clamped tasks appear
  in whichever p-file owns their `processorId`). **`taskset_param.yaml` is NOT
  touched** (it is the generator's pre-trace artifact; the C++ never reads its
  `Et_mean` for scoring — only the characteristics YAMLs). This keeps the
  clamp precisely where the scored data lives.
- **Where the pass lives in the pipeline:** a new module
  `Gen_Taskset/lib/feasibility_clamp.py` with
  `clamp_avg_et_to_period(yaml_dir, et_over_period_cap=0.95)`, called from
  `run_full_generation_pipeline` (`orchestrator.py`) AFTER
  `generate_additional_execution_traces` returns (so all characteristics YAMLs
  exist on disk) and BEFORE the function returns. Pure filesystem transform —
  loads each YAML, clamps in place, rewrites with the same
  `SpaceSeparatedListDumper` so the format is byte-compatible with what
  `ReadTaskSet` expects. No RNG, no regenerate, deterministic, seed-stable.
- **TDD plan (per `agent_coding_rules.md`):** new
  `Gen_Taskset/tests/test_feasibility_clamp.py` — (a) construct a synthetic
  characteristics YAML with one deterministic task `mu=21.4, min=max=21.4,
  period=20, deadline=14` and one perf task `mu=518, min=25, max=450,
  period=500`; run the clamp; assert the deterministic task's
  `mu==max==19.0`, `deadline==20`, AND the perf task is UNCHANGED
  (`mu==518, max==450`). (b) assert the un-clamped task (`mu < 0.95*period`)
  is untouched. Red before green (the function does not exist yet). Then
  implement, then a regression assertion that the integration pipeline output
  satisfies `execution_time_mu <= 0.95*period` for every non-perf task.
- **Open sub-decision surfaced to the user (NOT blocking — proceeding with the
  principled choice; will state it explicitly in the reply):** the user said
  "clamp avg ET back to 0.95*period" (mu only). I am ALSO clamping
  `execution_time_max` (and `min` if needed) to `0.95*period` on the same
  tasks, because — per the `FiniteDist`-truncates-at-max trace above — clamping
  `mu` alone does not change the scored SP. If the user wants ONLY `mu`
  clamped (a cosmetic/labeling-only change that leaves the scored dist intact),
  that is a 1-line narrower version; but it would not achieve the stated goal
  (reduce the chance ET exceeds period in the scored metric). Proceeding with
  the mu+max+min clamp + deadline=period; flagging it.
- **Standing constraints honored:** TDD (red test first); no `git commit`
  (`git add` only); no A/B run by me (user runs `run_end_to_end.sh`); modular —
  the clamp is a standalone module wired in at one call site.

## 2026-07-12 (impl) — F1 clamp: TDD red→green + wired + verified

- **TDD red first.** Wrote `Gen_Taskset/tests/test_feasibility_clamp.py` (7
  tests) BEFORE the module existed. Confirmed RED: `ModuleNotFoundError: No
  module named 'Gen_Taskset.lib.feasibility_clamp'` at collection. Cases:
  (1) deterministic `mu=21.4,min=max=21.4,period=20,deadline=14` → clamp fires
  to `mu==max==19.0, deadline==20`; (2) perf `mu=518,min=25,max=450,period=500`
  → SKIP unchanged; (3) clean `mu=10,period=100` → untouched; (4) mixed
  one-of-each; (5) `mu>cap*period>max` → never RAISES max (regression guard
  for the `min(max, cap*period)`); (6) walks every characteristics file
  (global + `interval_{k}` + `i{k}_p{p}`) AND skips `taskset_param.yaml`;
  (7) preserves perf-record space-separated STRING format through the
  round-trip (ReadTaskSet reads it via `.as<std::string>()`).
- **Module implemented:** `Gen_Taskset/lib/feasibility_clamp.py` —
  `clamp_avg_et_to_period(yaml_dir, et_over_period_cap=0.95)`. Pure filesystem
  transform: `glob` every `taskset_characteristics*.yaml`, `safe_load`,
  per-task `_clamp_task` (perf-gated via `bool(performance_records_time)`;
  fires only when `mu > cap*period`; clamps `mu`→cap*period,
  `max`→`min(max,cap*period)`, `min`→`min(min,new_max)`, `deadline`→period),
  rewrite via `export_taskset_to_yaml` (same `SpaceSeparatedListDumper`).
  Returns a `{files_written, tasks_clamped}` report. Module docstring carries
  the full WHY (the `FiniteDist`-truncates-at-`max` trace → mu-alone is a
  no-op → max/min clamp is mandatory; perf min/max are TL-grid bounds → skip)
  so the next reader can't "simplify" it back to mu-only.
- **Wired into the pipeline:** `orchestrator.run_full_generation_pipeline`
  step 4, AFTER `generate_additional_execution_traces` returns (all
  characteristics YAMLs on disk), BEFORE the function returns. One call site.
- **GREEN:** 7/7 new tests pass. Full `Gen_Taskset/tests/` 21/21 green.
  Full `tests/python/` 291/291 green (no downstream regressions — eval-suite,
  aggregator, taskset-generator tests all unaffected). Added the clamp
  invariant (`mu <= 0.95*period` AND `max <= 0.95*period` for non-perf tasks)
  as a regression assertion in `test_integration_pipeline`; passes against a
  real pipeline run.
- **Empirical re-scan of the N=4 run (read-only, on a copy):** the clamp
  fires on EXACTLY 1 task across all 10 tasksets at interval_0 — taskset_1
  gid=2 task_3 (`mu=21.404 > 0.95*20=19.0`, `min==max==mu==21.4`). The other
  39 tasks are already feasible. Matches the design's surgical prediction
  (dev_log 2026-07-12 "Empirical clamp-firing sweep"). Ran the clamp on a
  copy of the real taskset_1 `taskset_characteristics_interval_0.yaml`:
  post-state gid=2 `mu=min=max=19.0, deadline=20` (the user's flagged
  `mu>period` substrate repaired); gid=0 (`mu=27.2 < 0.95*33=31.35`) and
  gid=1 (`mu=128.9 < 0.95*500=475`) untouched (mu under cap — the clamp only
  fires on `mu > 0.95*period`, per spec); gid=3 skipped (perf task).
- **Scope respected:** Python-only change (no C++ touched → no ctest rebuild
  needed). No `git commit` (`git add` only, user reviews). No A/B run by me
  (user runs `run_end_to_end.sh` per D4). The clamp is a standalone module
  wired at one call site.
- **What remains (user-facing):** user re-runs the A/B (`run_end_to_end.sh`)
  to confirm INCR ≥ INCR_WCET on taskset_1 + the aggregate, and checks the Q3
  verdict. Step 2 (a C++ test reproducing the taskset_1 collapse on the
  pre-clamp code) is now MOOT for F1 — the substrate is removed at generation
  time, so the C++ optimizer never faces the unschedulable taskset; leaving
  the C++ test out unless the user wants a regression guard on the generator
  side (the `test_feasibility_clamp.py` suite already is that guard).

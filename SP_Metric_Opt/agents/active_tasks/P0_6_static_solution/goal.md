# P0.6 — Offline Static Solution (Fall-Back Seed)

**Priority:** P0 (blocks the fall-back mechanism, P0.7)
**Status:** design (REDESIGNED 2026-07-30) / not started on code
**Depends on:** P0.8 (task-set schedulability for important tasks under DM — certifies the
operating point the static solution seeds from). P0.7 (fall-back) depends on THIS task.
**Blocks:** P0.7 (the fall-back swaps the static solution in when an online trigger fires).

> **REDESIGNED 2026-07-30, then CONSTRAINT-MODE SETTLED same day, then WORST-CASE-DAG REFINED
> 2026-07-31.** The prior (2026-07-27) design was: seed DM-grouped PA + min-TL → **WCET-mode TL
> walk** with a **skip-and-continue important-task-schedulability filter** → best-SP-safe incumbent.
> The user's 2026-07-30 direction **simplifies the seed**: seed at the SAME operating point P0.8's
> schedulability test certifies (TL = largest grid option ≤ `et_mean`), then optimize, and keep it —
> **dropping** the WCET-mode flip and the global-max WCET precompute (old D2). The user then set the
> objective — "find the best possible performance while guaranteeing all important tasks' DDL miss
> chance ≤ SP thresholds" — which **REVIVES a filter in a new form**: a **hard per-candidate gate on
> the probabilistic `ddl_miss_chance ≤ threshold`** (normal ET mode, offline-scoped), NOT the old
> WCET point-mass RTA. So: old D2 (WCET precompute) stays DROPPED; old D5/D6/D7 are REVIVED-RESCOPED
> (hard gate, probabilistic, normal-ET). D1/D3/D4 stand; D8 RESOLVED via the gate.
>
> **2026-07-31 WORST-CASE-DAG REFINEMENT (authoritative — supersedes the within-one-taskset safety
> scope of the 2026-07-30 redesign):** the 2026-07-30 design certified `safe_fallback_` against the
> *single taskset* `ComputeSafeFallback` was invoked on. That made trigger (a) (P0.7's ET-jump
> swap-in) UNSAFE: a jumped interval's env distribution is not bounded by the certified interval's.
> Verified two code facts: (1) `ComputeSafeFallback` forces `use_wcet_execution_time=false`
> (`OptimizeSP_TL_Incre.cpp:893`) → env tasks (TL=−1) keep their **base Gaussian**, NOT a WCET point
> mass, during the compute; (2) the generator computes per-interval `Et_sigma = np.std(subset)`
> (`orchestrator.py:375`) **independent of the mean** → "longest-by-avg-ET" does NOT bound
> `ddl_miss_chance` (a lower-mean interval can carry a fatter tail). The user's fix: **the CALLER
> builds a worst-case DAG** (per task, the WCET point mass at `max(execution_time_max)` across all
> interval YAMLs) and invokes `ComputeSafeFallback` on THAT → the certificate holds for every
> interval (any interval's ET draw ≤ its `max_time` ≤ the worst-case point mass → stochastic
> dominance → `ddl_miss_chance` bounded). Plus a **loud-fail** if the final stored result fails the
> gate (the user's "re-generate a new task set"). See "WORST-CASE-DAG (2026-07-31)" below.

## Goal

Produce a **deterministic, offline static scheduling solution** computed ONCE before the
interval-optimization loop begins, stashed on the orchestrator as `static_solution_` (a
`ResourceOptResult`), so the fall-back (P0.7) can swap it in when an online trigger fires.
"Static" = a fixed priority assignment + a fixed time-limit vector, found by **seeding at the
P0.8-certified operating point and running a TL-only walk with a hard feasibility gate offline**,
then keeping the best-SP-feasible result.

### Budget asymmetry still holds (user's 2026-07-27 direction)

> "overall, we have high budget for offline analysis, so we can try more 'walk' during
> offline analysis, just find a safe solution with good performance. during online,
> budget is very tight, use this to adjust your design."

So the offline compute (THIS task) **explores fully** — ample budget → run the TL walk to
convergence, **skipping-and-continuing** past infeasible candidates to find the best-SP-feasible
point (not just the first feasible point). The online guard (P0.7) is the **tight-budget**
counterpart — it **halts** early and compares against this static solution. The HALT semantics
live in P0.7, NOT here.

### The algorithm (REDESIGNED 2026-07-30; hard gate added same day)

1. **Seed at the P0.8-certified operating point.**
   - **PA** = the DM-grouped-important-first seed already landed by P0.9:
     `DeadlineMonotonicPriorityVec()` (`OptimizeSP_TL_Incre.cpp:705-721`) — important tasks
     occupy the top slots (DM-ordered within the group), the rest fill the lower slots
     (DM-ordered within the group), every rest-task below every important task. Committed via
     `SeedIncumbentFromDMFast()` (`:772-780`). **No new PA helper needed** — P0.9 superseded
     the planned `AssignDMRespectingGroupOrder` extraction (old step 1).
   - **TL** = for each perf (TL-optimizable) task, the **largest TL grid option that is ≤
     `et_mean`** (`execution_time_dist.GetAvgValue()`, `RegularTasks.h:87`); non-perf tasks
     TL = -1 (no TL freedom). This is the operating point P0.8 certifies: P0.8's perf WCET =
     `execution_time_mu` (= et_mean, per the config-tuning `7c8748c0`), and the C++ sim runs
     perf ET = `min(et_mean, TL)` (TL is a downward cap) → at TL ≤ et_mean the runtime ET ≤
     et_mean = certified WCET. The existing `InitializeTimeLimitsFromETConfig()`
     (`OptimizeSP_TL_Incre.cpp:483-498`) already seeds each perf task to
     `Find_Close_ExecutionTime(pairs, GetAvgValue())` (`:42-56`), but that helper is
     **bidirectional** (closest by abs distance, may pick *above* et_mean). P0.6 needs the
     **directional ≤ et_mean** variant — the largest grid option not exceeding et_mean — so
     the seed sits at-or-below the certified WCET. (If no grid option ≤ et_mean exists, fall
     back to the smallest grid option and flag it.)
   - Commit the seed as the incumbent baseline (mirror `SeedIncumbentFromDMFast`, but with the
     et_mean-bounded TL vector instead of `SmallestTimeLimitVec`).

2. **Optimize (TL-only walk) from the seed, with a HARD per-candidate gate.**
   - Run the TL walk (`OptimizeIncre_w_TL` → `RunIntervalDescent(Incremental)` →
     `WalkSerializedTaskQueue` → `WalkOneTaskWithTimeLimitOptions`, `OptimizeSP_TL_Incre.cpp:500+`),
     starting from the seeded {PA, TL}, to maximize SP. Offline, ample budget → run to convergence.
   - **The hard gate (the user's guarantee):** adopt an SP-better candidate TL ONLY if it keeps
     **every important task's `ddl_miss_chance_i ≤ sp_threshold_i`**; otherwise SKIP the candidate
     and CONTINUE the walk (try other TL options / other tasks). The walk keeps committing
     SP-better-AND-feasible candidates; the returned solution is the **best-SP-feasible** point the
     walk reached. (Offline skip-and-continue; the HALT is P0.7's online trigger (b).)
   - **D8 RESOLVED via the gate:** the from-scratch path (`OptimizePA_Incre::OptimizeFromScratch`,
     `OptimizeSP_Incre.cpp:100`) IS a PA beam search that reorders PA — but with the hard gate as
     the safety mechanism, PA descent becomes SAFE-TO-ADD-LATER (the gate rejects any PA move that
     violates a threshold, including a group-lock-breaking reorder). For v1, keep the walk
     **TL-only** — `WalkOneTaskWithTimeLimitOptions` mutates only `time_limits[task_idx]`; `opt_pa_`
     is inherited from the seed and never changed → group lock preserved BY CONSTRUCTION
     (belt-and-suspenders). PA descent = deferred enhancement (gated, so safe when added).

3. **Keep the result.** Store the committed incumbent `{priority_vec, id2time_limit}` — the
   SAME shape `DeterminePrioritiesAndBudgets` returns (`ResourceOptResult`,
   `SimulationOrchestrator.cpp:313`) — as the in-memory `static_solution_` member. It is
   **feasible-by-construction** (the gate never adopted a violating candidate) AND **best-SP-among-
   feasible** (the walk kept searching past skipped candidates). The fall-back (P0.7) swaps that
   pair in place of the optimizer's `res_opt_` when a trigger fires.

### WORST-CASE-DAG (2026-07-31 — authoritative for cross-interval safety)

The 2026-07-30 design scoped `safe_fallback_`'s safety to the *single taskset* it was computed on.
P0.7's trigger (a) swaps `safe_fallback_` in on an **ET-jump across intervals** — a different taskset.
Two code facts made that swap unsafe under the 2026-07-30 design:

1. `ComputeSafeFallback` forces `use_wcet_execution_time = false` (`OptimizeSP_TL_Incre.cpp:893`).
   So env tasks (TL=−1) keep their **base Gaussian** during the compute — the certificate is
   "safe at interval-i's env distribution," NOT "safe at env WCET."
2. The generator's per-interval `Et_sigma = np.std(subset)` (`orchestrator.py:375`) is **independent
   of the mean** (re-sampled per interval). So a jumped interval with a lower mean but fatter tail /
   larger `execution_time_max` could yield a *higher* `ddl_miss_chance` than the certified interval →
   the certificate does not cover it. ("Longest-by-avg-ET" is NOT a sound dominance rule.)

**The fix (user direction 2026-07-31): the caller builds a worst-case DAG and computes the artifact
on THAT.** For each task, take the **point mass at `max(execution_time_max)` across all interval
YAMLs** (env and non-perf tasks; perf/TL-optimizable tasks are already point-masses at TL, bounded by
the downward cap `min(et_mean, TL)` at runtime). Any interval's per-task ET draw ≤ its own
`max_time` ≤ the worst-case max → the worst-case point mass **stochastically dominates** every
interval's dist → the gate's `ddl_miss_chance` on the worst-case DAG is an upper bound for every
interval → **trigger (a)'s swap-in is safe by construction.** This restores the cross-interval
conservatism the dropped old-D2 used to provide, in a sound form (point-mass at the global max, not a
flag-driven collapse of the online path).

- **Loud-fail (user direction 2026-07-31):** after the walk, re-run the gate on the **final stored
  result** (not just the seed — the walk may find a feasible smaller-TL point below an infeasible
  seed). If the final result fails the gate → **raise loud, do NOT store** (the user's "RM-Fast for
  important tasks cannot even make important tasks meet their DDL → fail loudly, ask user to
  re-generate a new task set"). Raising TL only worsens interference, so a seed-level miss is not
  fixable by the walk; the final-check catches both "no feasible point exists" and "the walk drifted
  to an infeasible point" (it can't, the gate only rejects, but the check is cheap insurance).
- **Scope of the WCET point mass:** the worst-case DAG is fed to the **offline `ComputeSafeFallback`
  only**. The online sim + online optimizer keep using the actual per-interval DAGs → **online
  byte-identical**. This is NOT the old `use_wcet_execution_time` global flag (which collapses the
  online path too); it is a one-DAG construction at the offline call site.
- **Seed TL uses the worst-case `et_mean`:** step-3 `SeedTimeLimitsAtOrBelowEtMean` runs on the
  worst-case DAG, so the seed TL ≤ the largest `et_mean` any interval exhibits → conservative.
- **Perf-task TL grid:** unchanged — drawn from the task's `timePerformancePairs` (period-scaled),
  identical across intervals (the grid is a task-structure property, not per-interval).

### Why a hard per-candidate gate (not the soft SP_Func penalty)

The user's objective is a **hard guarantee**: `∀ important i: ddl_miss_chance_i ≤ sp_threshold_i`.
The SP metric already SOFTLY discourages violations — `SP_Func` (`SP_Metric.h:31-41`) is
monotone-decreasing in `ddl_miss_chance` with an exponential penalty past the threshold
(`PenaltyFunc = -0.01·exp(10·|th−v|)`). But soft ≠ strict: a mild violation on a **low-weight**
important task can still be SP-optimal if the perf gain elsewhere outweighs its penalty. So
maximizing SP alone does NOT guarantee the threshold — the hard gate is genuinely needed.

**`ddl_miss_chance` is PROBABILISTIC, not a point mass** (`SP_Metric.cpp:65-71`, `RTA.cpp:154`):
`ddl_miss_chance_i = GetDDL_MissProbability(rta_dist_i, deadline_i)` = the probability mass of
task i's RTA distribution ABOVE its deadline, where the RTA dist is built by CONVOLVING ET
distributions (`GetRTA_OneTask`, `RTA.cpp:32-44`). The metric's ET model: a perf task's ET dist is
a POINT MASS at `time_limits[i]` (`ApplyTimeLimitsToTasksExecutionTime`, `SP_Metric.cpp:76-86`);
-1 leaves the base (Gaussian, with variance) dist untouched. Non-perf tasks keep their full Gaussian
in the metric (the sim itself runs them at `GetAvgValue()` point-mass, `SimulationOrchestrator.cpp:
426/489/700/752`, and caps perf runtime ET at `min(et_mean, TL)`, `7c8748c0`). → **the metric's
`ddl_miss_chance` ≥ the sim's actual miss chance** (metric keeps non-perf variance the sim drops;
metric perf ET = TL = the sim's cap). So a gate on the METRIC is a sound (pessimistic) guarantee of
the user's stated runtime condition.

**Two regions — why "best performance" makes the constraint BINDING:**
- **TL ≤ et_mean (the seed region):** perf ET point-mass = TL ≤ et_mean = P0.8's perf WCET; non-perf
  ET = the Gaussian P0.8 certified (max = `execution_time_max`). P0.8's WCET-RTA certification
  (`R_i(WCET) ≤ deadline_i`) + interference monotonicity → the whole RTA dist sits ≤ deadline →
  **ddl_miss_chance = 0 for important tasks, AUTOMATICALLY.** The constraint is FREE here (the
  2026-07-30 "whole grid safe" claim holds in this sub-region only).
- **TL > et_mean (raising TL for performance):** the metric's perf point-mass ET exceeds P0.8's
  certified WCET → the RTA dist extends past the deadline → **ddl_miss_chance becomes a real
  nonzero quantity and the constraint BINDS.** "Best possible performance" means the optimizer
  SHOULD push TLs up against the thresholds — into this region — so the gate is load-bearing.

**The gate is NEAR-FREE:** the walk's `eval` (`OptimizeIncreSingleTask`, `OptimizeSP_TL_Incre.cpp:
167-239`) routes through `rta_cache_.Evaluate` → `ObtainSP_Full_From_NodeRTAs` →
`ObtainSP_DAG_From_Dists`, which already calls `GetDDL_MissProbability(rtas[i], deadline)` PER TASK
(`SP_Metric.cpp:160`) but only aggregates into SP. The node RTAs are already materialized →
surfacing per-task miss-chances (or a single important-task feasibility bool) is a small API
addition over already-computed data. Two candidate shapes (decide at impl): (A) a new
`ImportantTasksFeasible(pa, tl) -> bool` re-deriving node RTAs (cleanest separation; one extra RTA
eval per adopted candidate); (B) thread per-task miss-chances out of `ObtainSP_DAG_From_Dists`
(zero extra RTA eval — cheapest). Prefer (B) if the API fits cleanly; (A) as fallback.

This KEEPS the 2026-07-30 redesign's two drops (the WCET-mode flip; the global-max-across-intervals
WCET precompute, old D2) and PARTIALLY REVIVES the old skip-and-continue filter (old D5/D6/D7) in a
**NEW form**: a hard gate on the **probabilistic** `ddl_miss_chance ≤ threshold`, in **normal ET
mode** (no WCET point-mass collapse), **offline-scoped**. The old D6 (RTA form ≡ DDL-miss-chance
under WCET point-mass) is superseded by the direct probabilistic form. It still **dissolves the
D2-vs-P0.8 tension** (P0.6 no longer defines a divergent WCET).

## Why offline + why it doesn't count toward scheduler ET

- **Offline:** computed once, before `for (i = 0; i < dag_tasks_vecs_.size(); i++)
  SimulateInterval(...)` (`SimulationOrchestrator.cpp:304-308`). It depends ONLY on the taskset
  (periods, deadlines, sp_weights, TL grids, et_mean) — NOT on runtime traces — so it can be
  precomputed. It is a constant the fall-back reads, not a per-interval solve.
- **ET accounting:** `DeterminePrioritiesAndBudgets` (`SimulationOrchestrator.cpp:313-322`)
  brackets the scheduler decision and accumulates into `scheduler_exec_time_s_`, which
  `RunOrchestrator` writes to `scheduler_execution_time.txt` — the online-performance monitor.
  The static-solution computation must run OUTSIDE that bracket (before the loop, not inside
  `DeterminePrioritiesAndBudgets`) so it does NOT inflate the reported per-interval scheduler
  ET. You MAY profile its own running time (log it separately as
  `static_solution_compute_time.txt`) for reporting, but it is explicitly excluded from the
  online ET metric. The user stated this explicitly: "the static solution is generated offline
  before running the whole interval optimization loops, you may profile its running time but
  that doesn't count into scheduler's average ET."

## Where it hooks in (code grounding)

- **Compute site:** after `incr_optimizer_` construction
  (`SimulationOrchestrator.cpp:300-302`) and before the interval loop
  (`SimulationOrchestrator.cpp:304`). Add a `ComputeStaticSolution()` call here for the
  INCR-family modes (the modes that have an online optimizer to fall back FROM). Store the
  result on the orchestrator (a `ResourceOptResult static_solution_` member).
- **DM-grouped PA seed (LANDED by P0.9):** `DeadlineMonotonicPriorityVec()`
  (`OptimizeSP_TL_Incre.cpp:705-721`) already group-locks (reads `is_important` at `:713-714`);
  `SeedIncumbentFromDMFast()` (`:772-780`) = `SmallestTimeLimitVec()` + DM-grouped PA, scored +
  committed. The static solution seeds identically except TL = et_mean-bounded (not min-TL).
- **et_mean-bounded TL seed:** `InitializeTimeLimitsFromETConfig()`
  (`OptimizeSP_TL_Incre.cpp:483-498`) is the existing precedent — seeds each perf task's TL to
  `Find_Close_ExecutionTime(timePerformancePairs, execution_time_dist.GetAvgValue())`. P0.6
  needs the directional ≤ et_mean variant (largest grid option ≤ et_mean), so add/extend a
  helper (e.g. `FindLargestTimeLimitAtOrBelow(pairs, et_mean)`); non-perf TL = -1.
  `execution_time_dist.GetAvgValue()` = et_mean (`RegularTasks.h:87`). Generator's TL grid =
  `period * FINAL_Et_OVER_PERIOD_RANGE` `[0.05, 0.9]` (`taskset_generator.py:32`).
- **TL walk (TL-only, PA fixed):** `OptimizeIncre_w_TL` → `RunIntervalDescent(Incremental)` →
  `WalkSerializedTaskQueue` → `WalkOneTaskWithTimeLimitOptions` (`OptimizeSP_TL_Incre.cpp:500+`).
  The walk mutates only `time_limits[task_idx]`; `opt_pa_` is inherited and never changed. ✓
  matches "fix priority, walk TL" — safe by construction (group lock preserved).
- **Incumbent commit:** `SeedStateFromIncumbent` / `CommitIncumbent`
  (`OptimizeSP_TL_Incre.cpp:726-751`) — the single writer for `opt_sp_`/`opt_pa_`/`res_opt_`.

## Designing important tasks (D1 RESOLVED 2026-07-27; LANDED by P0.9)

**RESOLVED + LANDED:** 50% of the tasks in a taskset are important, indicated by SP weights;
the rest are non-important. The label is **persisted** as `bool is_important` on the C++ `Task`
class — set at generation, emitted to YAML, read here. `bool is_important = false` at
`RegularTasks.h:111`; parsed `RegularTasks.cpp:87`; emitted `:126`. Generator labels the top
`IMPORTANT_TASK_RATIO` (0.5) by `sp_weight` desc (`taskset_generator.py:577-581`; config
`:38`); `yaml_exporter.py:73` emits `important:`. Count = `ceil(N/2)` = `(N+1)//2`.
P0.8's Python RTA reads the same bool → no drift. (Step 0.5 of the old `tasks.md` is DONE.)

## Scope (what this task IS / IS NOT)

**IS:**
- A new offline `ComputeSafeFallback()` that emits a `ResourceOptResult` (DM-grouped PA +
  et_mean-bounded TL seed → TL-only walk with hard feasibility gate → committed best-SP-feasible
  incumbent), stored on the orchestrator as `safe_fallback_`.
- **Worst-case-DAG construction (2026-07-31):** the orchestrator iterates all interval YAMLs and
  builds, per task, a point-mass dist at `max(execution_time_max)` across intervals (env/non-perf;
  perf tasks keep their TL grid) → passes this worst-case DAG to `ComputeSafeFallback`. This makes
  the certificate cross-interval (sound for P0.7 trigger (a)).
- **Loud-fail (2026-07-31):** a post-walk gate re-check on the FINAL stored result; on failure,
  raise loud + do NOT store (prompt re-generation).
- The **et_mean-bounded TL seed** (largest grid option ≤ `et_mean`) — the P0.8-certified
  operating point. Small helper, TDD-covered.
- The **hard per-candidate feasibility gate**: adopt an SP-better candidate TL only if
  `∀ important i: ddl_miss_chance_i ≤ sp_threshold_i`; else skip + continue. Requires a small
  API to surface per-task `ddl_miss_chance` (already computed inside `ObtainSP_DAG_From_Dists`).
- A separate profile log for the static-solution compute time (NOT in
  `scheduler_execution_time.txt`).
- TDD: unit-test (1) the et_mean-bounded TL seed (largest grid option ≤ et_mean; non-perf =
  -1; fall-back-to-smallest when no option ≤ et_mean), (2) the seeded PA is DM-grouped
  (important-first), (3) the gate REJECTS an SP-better candidate that violates an important
  task's threshold and KEEPS a feasible one, (4) the returned solution is feasible-by-
  construction (every important task's ddl_miss_chance ≤ threshold) and best-SP-among-feasible,
  (5) the compute is ET-excluded (reported scheduler ET unchanged vs. baseline at the same N).

**IS NOT:**
- The fall-back INVOCATION (when to swap the static solution in) — that's P0.7. The HALT
  early-stop guard is P0.7's online trigger (b).
- The generation-time schedulability GUARANTEE — that's P0.8. P0.8 certifies the operating
  point P0.6 seeds from (and guarantees the TL ≤ et_mean sub-region is feasible by construction);
  the gate enforces feasibility for TL > et_mean candidates the optimizer explores.
- A WCET-mode ablation of the ONLINE path or a global-max WCET precompute fed to the online
  optimizer — DROPPED by the 2026-07-30 redesign (old step 2 / old D2). Do NOT re-introduce them.
  (The 2026-07-31 worst-case-DAG is an OFFLINE-only construction fed to `ComputeSafeFallback`; it
  is NOT the old `use_wcet_execution_time` global flag. The filter IS revived, in its new
  probabilistic hard-gate form — not the old WCET point-mass RTA.)
- A change to the SP metric's definition, `SP_Func`, or the optimizer's online path. The gate
  is a NEW adoption predicate layered on top of the existing TL walk; it does not alter how SP
  is computed.
- A new scheduler arm in the A/B list. The static solution is an internal fall-back state, not
  a comparable scheduler.

## Relationship to P0.8 (refined 2026-07-30)

P0.8 certifies, at generation time, that the taskset is schedulable for the important tasks
under the DM-grouped PA at WCET (perf = `et_mean`). P0.6 seeds at an operating point whose
runtime ET is ≤ that WCET (TL ≤ et_mean → runtime ET ≤ et_mean), so the **seed is feasible by
construction** (ddl_miss_chance = 0 in the TL ≤ et_mean sub-region; P0.8's WCET-RTA cert + the
metric's pessimistic bound guarantee it). P0.6 then runs the TL walk to raise TLs for
performance; once TLs exceed et_mean, P0.8's certification no longer covers the operating
point, so the **hard gate** takes over as the in-walk feasibility guarantee — it rejects any
candidate that violates an important task's threshold. The returned solution is feasible-by-
construction (the gate never adopted a violating candidate).

- **P0.8 = precondition + seed-region guarantee** (re-generate if the taskset is unschedulable
  for important tasks at WCET under the DM-grouped PA; guarantees the seed sub-region).
- **P0.6 = seed + gated optimize + keep** (seed feasible by P0.8; walk enforces the hard gate
  for TL > et_mean candidates; result feasible-by-construction + best-SP-among-feasible).
- **Consistency requirement:** P0.6's seed TL must be ≤ `et_mean` so the seed's runtime ET ≤
  the WCET P0.8 certified. (P0.6 has no D2 WCET field — it relies on P0.8's, plus its own gate.)

## Done when

- [x] "Important tasks" selection rule settled + LANDED (P0.9): top-50% by `sp_weight`,
      persisted as `bool Task::is_important`. See "Designing important tasks."
- [x] DM-grouped-important-first PA seed LANDED (P0.9): `DeadlineMonotonicPriorityVec` +
      `SeedIncumbentFromDMFast`. (Old step 1 `AssignDMRespectingGroupOrder` extraction =
      SUPERSEDED — the group lock is baked into the seed function.)
- [x] D3 settled (2026-07-27): per-taskset (== once per worker invocation).
- [x] D4 settled (2026-07-27): in-memory `static_solution_` member.
- [x] **2026-07-30 redesign:** old D2 (global-max WCET) DROPPED (no WCET precompute; leans on
      P0.8's grid-wide certification for the seed sub-region). See "Why a hard gate."
- [x] **2026-07-30 objective + constraint mode:** hard per-candidate gate on probabilistic
      `ddl_miss_chance ≤ threshold` (REVIVES old D5/D6/D7 in a new form: normal-ET, probabilistic,
      offline-scoped — NOT the old WCET point-mass RTA). D8 RESOLVED via the gate (TL-only v1).
- [x] et_mean-bounded TL seed helper (LANDED 2026-07-30): `FindLargestTimeLimitAtOrBelow`
      (pure directional variant of `Find_Close_ExecutionTime`) + `SeedTimeLimitsAtOrBelowEtMean`
      vector method. TDD red→green (7 tests); 17/17 ctest green. See `tasks.md` step 3.
- [x] Per-task `ddl_miss_chance` gate predicate LANDED 2026-07-30:
      `ImportantTasksMeetThresholds(dag, sp_params, pa, tl, node_rtas) -> bool` (SP_Metric.{h,cpp}).
      Zero-extra-eval (user-directed): takes the ALREADY-COMPUTED `node_rtas` the caller
      materialized when scoring SP (`rta_cache_.Evaluate` → `ObtainSP_Full_From_NodeRTAs`), not a
      re-derived RTA (the prior shape A `ProbabilisticRTA_TaskSet`-per-call would duplicate the
      cache's work on every adopted candidate). Bake+prioritize still inside → `node_rtas[i]` pairs
      with the prioritized task at i (same CONTRACT as `ObtainSP_Full_From_NodeRTAs`). TDD red→green
      (4 tests); 17/17 ctest green. This is shape (B) via RTA pass-through.
- [ ] The hard-gate HOOK into `WalkOneTaskWithTimeLimitOptions`'s adoption decision (step 4b). TDD:
      gate rejects an SP-better threshold-violating candidate; keeps a feasible one.
- [ ] `ComputeStaticSolution()`: seed DM-grouped PA + et_mean-bounded TL, run the TL-only walk
      with the hard gate (PA fixed → group lock preserved), return the committed
      `ResourceOptResult`.
- [ ] Static solution computed before the interval loop, stored on the orchestrator,
      excluded from `scheduler_exec_time_s_` (verified: reported scheduler ET unchanged vs.
      baseline at the same N).
- [ ] Separate `static_solution_compute_time` profile emitted (does NOT enter the online ET
      metric).
- [ ] Verify the safety assumptions: (1) sim perf ET = `min(et_mean, TL)` downward cap; (2) the
      metric's `ddl_miss_chance` ≥ the sim's actual miss chance (pessimistic bound — confirmed
      at design time: metric keeps non-perf Gaussian variance the sim drops; metric perf ET = TL
      = the sim's cap); (3) the TL-only walk preserves the group lock.
- [ ] `cmake --build build_test --target check.SP_OPT -j5` green (17/17 ctest).
- [ ] **WORST-CASE-DAG (2026-07-31):** orchestrator builds the worst-case DAG (per-task
      point mass at `max(execution_time_max)` across interval YAMLs) and passes it to
      `ComputeSafeFallback`; seed TL ≤ worst `et_mean`. TDD: worst-case DAG's per-task
      `max_time` = max across intervals; perf-task TL grid preserved.
- [ ] **LOUD-FAIL (2026-07-31):** post-walk gate re-check on the final stored result; on
      failure raises loud + does NOT store (no silent bogus artifact). TDD: an unschedulable
      worst-case DAG (important task's deadline < its own WCET) → `HasSafeFallback()` stays
      false + raises; a schedulable one → stores + `HasSafeFallback()` true.
- [ ] **Cross-interval safety verified (2026-07-31):** worst-case point mass stochastically
      dominates every interval's per-task dist → gate's `ddl_miss_chance` is an upper bound
      for every interval → trigger (a)'s swap-in is sound. (Code-grounded: sim env ET is
      uncapped at `max_time` per draw; the worst-case point mass ≥ any draw.)
- [ ] `git add` staged; user reviews (no commit). `dev_log.md` + memory updated.

## Open decisions

- **D1 — Important-task selection rule.** ✅ RESOLVED + LANDED (P0.9): top-50% by `sp_weight`,
  persisted as `bool Task::is_important`.
- ~~**D2 — WCET value (global-max across intervals).**~~ ❌ SUPERSEDED 2026-07-30: P0.6 no
  longer computes a WCET; it relies on P0.8's grid-wide certification (perf WCET = et_mean) for
  the seed sub-region + its own hard gate for TL > et_mean. The global-max precompute is DROPPED.
- **D3 — Per-taskset vs per-run.** ✅ RESOLVED: per-taskset (== once per worker invocation).
- **D4 — In-memory vs file.** ✅ RESOLVED: in-memory `static_solution_` member.
- **D5 — Filter implementation shape.** ✅ REVIVED-RESCOPED 2026-07-30: a hard per-candidate
  gate (predicate on `WalkOneTaskWithTimeLimitOptions`'s adoption decision), NOT the old virtual
  `ShouldAdoptCandidate` hook on a WCET walk. Offline-scoped (the static-solution compute only).
- **D6 — Filter quantity.** ✅ REVIVED-RESCOPED 2026-07-30: the direct **probabilistic**
  `ddl_miss_chance_i = GetDDL_MissProbability(rta_dist_i, deadline_i)` ≤ `sp_threshold_i`, in
  normal ET mode (NOT the old WCET point-mass RTA — the metric is already probabilistic).
- **D7 — HALT vs SKIP-AND-CONTINUE offline.** ✅ REVIVED-RESCOPED 2026-07-30: offline
  **skip-and-continue** (ample budget → find best-SP-feasible, not just first feasible). The
  HALT remains P0.7's online trigger (b).
- **D8 — From-scratch path & the group lock.** ✅ RESOLVED 2026-07-30 (via the gate): the
  from-scratch path IS a PA beam search that reorders PA, but with the hard gate as the safety
  mechanism, PA descent is safe-to-add-later (the gate rejects any threshold-violating PA move).
  For v1: TL-only walk (group lock preserved by construction). PA descent = deferred enhancement.
- **D9 — Worst-case DAG for cross-interval safety (2026-07-31).** ✅ RESOLVED 2026-07-31 (user):
  the caller builds a worst-case DAG (per-task point mass at `max(execution_time_max)` across all
  interval YAMLs) and invokes `ComputeSafeFallback` on it. Restores the cross-interval conservatism
  the dropped old-D2 provided, soundly (stochastic dominance), OFFLINE-only (online byte-identical).
  Replaces the unsound "longest-by-avg-ET" idea (sigma is mean-independent per interval). Loud-fail
  on a final gate-reject. See "WORST-CASE-DAG (2026-07-31)."

## Reference docs

- `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp:297-311` — optimizer
  construction + interval loop (compute site is between these).
- `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp:313-322` —
  `DeterminePrioritiesAndBudgets` (the ET-bracketed scheduler decision — static solution stays
  OUTSIDE).
- `sources/Optimization/OptimizeSP_TL_Incre.cpp:705-721` — `DeadlineMonotonicPriorityVec`
  (LANDED group-locked DM seed PA).
- `sources/Optimization/OptimizeSP_TL_Incre.cpp:772-780` — `SeedIncumbentFromDMFast`
  (DM-grouped PA + min-TL seed pattern; P0.6 mirrors with et_mean-bounded TL).
- `sources/Optimization/OptimizeSP_TL_Incre.cpp:483-498` — `InitializeTimeLimitsFromETConfig`
  (the existing closest-to-et_mean TL seed; P0.6 needs the ≤ et_mean directional variant).
- `sources/Optimization/OptimizeSP_TL_Incre.cpp:42-56` — `Find_Close_ExecutionTime`
  (bidirectional closest; basis for the directional helper).
- `sources/Optimization/OptimizeSP_TL_Incre.cpp:500-548` — `WalkOneTaskWithTimeLimitOptions`
  (the 1D TL walk primitive; TL-only, PA fixed — confirms the group lock is preserved). **The
  gate hooks here**: the `if (IsBetterTimeLimitOption(...))` adoption at `:536` gains a feasibility
  predicate — adopt only if the candidate keeps every important task's `ddl_miss_chance ≤ threshold`.
- `sources/Optimization/OptimizeSP_TL_Incre.cpp:167-239` — `OptimizeIncreSingleTask` (the walk's
  `eval`; routes through `rta_cache_.Evaluate` → `ObtainSP_Full_From_NodeRTAs` →
  `ObtainSP_DAG_From_Dists`, which already computes per-task `ddl_miss_chance` internally).
- `sources/Safety_Performance_Metric/SP_Metric.cpp:65-71` — `ObtainSP_TaskSet` (per-task
  `ddl_miss_chance` computed + aggregated into SP — the API surface to thread per-task values out).
- `sources/Safety_Performance_Metric/SP_Metric.cpp:76-86` — `ApplyTimeLimitsToTasksExecutionTime`
  (the metric's ET model: perf → point-mass at TL; -1 → base Gaussian; the pessimistic bound).
- `sources/Safety_Performance_Metric/RTA.cpp:32-44,154-169` — `GetRTA_OneTask` (ET convolution)
  + `GetDDL_MissProbability` (the probabilistic miss-chance the gate enforces).
- `sources/Safety_Performance_Metric/SP_Metric.h:31-41` — `SP_Func` (monotone-decreasing +
  exponential penalty past threshold — why soft ≠ hard guarantee → the gate is needed).
- `sources/Optimization/OptimizeSP_Incre.cpp:100` — `OptimizeFromScratch` (the from-scratch PA
  beam search — D8: reorders PA; safe-to-add-later once gated; NOT used in v1's TL-only walk).
- `sources/Optimization/OptimizeSP_TL_Incre.cpp:726-751` — `SeedStateFromIncumbent` /
  `CommitIncumbent` (single incumbent writer).
- `sources/TaskModel/RegularTasks.h:87,111` — `GetAvgValue()` (= et_mean); `bool is_important`.
- `Gen_Taskset/lib/taskset_generator.py:32` — `FINAL_Et_OVER_PERIOD_RANGE: [0.05, 0.9]`
  (perf-task TL grid = period × this range).
- Memory [`p08-important-task-schedulability`](../../../) — P0.8's grid-wide certification
  (perf WCET = et_mean) this task leans on.
- Memory [`p07-fallback-mechanism`](../../../) — the fall-back that swaps `static_solution_` in.

# P0.6 — Offline Safe Fallback (Fall-Back Seed)

**Priority:** P0 (blocks the fall-back mechanism, P0.7)
**Status:** COMPLETE + COMMITTED (`630cda4d`, 2026-07-31). Sections 0.5–10 LANDED; 17/17 ctest green.
**Depends on:** P0.8 (certifies the seed sub-region). **Blocks:** P0.7 (swaps `safe_fallback_`
in when an online trigger fires).
> Latest = **§10: readability refactor** (`TaskStructureMatches` extracted from the worst-case-DAG builder).
> §9 = dispatcher throw + gate overload; §8 = worst-case-DAG + loud-fail (cross-interval safety for P0.7 trigger (a)).
> Historical redesign narrative is in `dev_log.md`; this file states the settled design only.

## Goal

Produce a deterministic, **offline** scheduling solution computed ONCE before the interval loop,
stashed as `safe_fallback_` (`ResourceOptResult`) for P0.7 to swap in on a trigger. "Safe
fallback" = a fixed PA + TL vector: seed at the P0.8-certified operating point, run a TL-only walk
with a hard feasibility gate, keep the best-SP-feasible result. Offline has ample budget → run to
convergence (skip-and-continue past infeasible candidates); the online HALT is P0.7's, not here.

## The algorithm

1. **Seed at the P0.8-certified operating point.** PA = DM-grouped-important-first seed (P0.9
   `DeadlineMonotonicPriorityVec` `OptimizeSP_TL_Incre.cpp:705-721`, committed via
   `SeedIncumbentFromDMFast` `:772-780`). TL = for each perf task the **largest grid option ≤
   `et_mean`** (`execution_time_dist.GetAvgValue()`, `RegularTasks.h:87`); non-perf = -1. Helper
   `FindLargestTimeLimitAtOrBelow` + `SeedTimeLimitsAtOrBelowEtMean` (directional ≤ et_mean;
   `Find_Close_ExecutionTime` is bidirectional). At TL ≤ et_mean, runtime ET ≤ TL ≤ et_mean =
   P0.8's perf WCET → seed feasible by construction.
2. **Optimize (TL-only walk) with a HARD per-candidate gate.** Adopt an SP-better candidate TL
   only if `∀ important i: ddl_miss_chance_i ≤ sp_threshold_i`; else skip + continue. The gate
   sits at the commit chokepoint (`UpdateRecords`), fires only on would-beat
   (`WouldBeatIncumbent` + `enforce_important_task_gate_`, true only in tests/`ComputeSafeFallback`
   → prod byte-identical). PA descent runs unconditionally (the gate rejects any threshold-
   violating PA move; a higher-SP PA still passing is strictly better → group-lock concern moot).
3. **Keep** the committed incumbent `{priority_vec, id2time_limit}` as `safe_fallback_`.
   Feasible-by-construction (gate never adopted a violating candidate) AND best-SP-among-feasible.

## WORST-CASE-DAG (2026-07-31 — authoritative for cross-interval safety)

The 2026-07-30 design certified `safe_fallback_` against the *single taskset* it was computed on.
P0.7's trigger (a) swaps it in on an **ET-jump across intervals** — a different taskset. Two code
facts made that swap unsafe:
1. `ComputeSafeFallback` forces `use_wcet_execution_time=false` (`OptimizeSP_TL_Incre.cpp:893`)
   → env tasks (TL=−1) keep their **base Gaussian** during the compute — the certificate is "safe
   at interval-i's env distribution," NOT "safe at env WCET."
2. The generator's per-interval `Et_sigma=np.std(subset)` (`orchestrator.py:375`) is **independent
   of the mean** → "longest-by-avg-ET" does NOT bound `ddl_miss_chance` (a lower-mean interval can
   carry a fatter tail / larger `execution_time_max`).

**The fix (user direction 2026-07-31): the caller builds a worst-case DAG and computes the
artifact on THAT.** Per task, take the **point mass at `max(execution_time_max)` across all
interval YAMLs** (env/non-perf; perf tasks are already point-masses at TL, bounded by the runtime
downward cap `min(et_mean,TL)`). Any interval's per-task ET draw ≤ its `max_time` ≤ the worst-case
max → the worst-case point mass **stochastically dominates** every interval's dist → the gate's
`ddl_miss_chance` on the worst-case DAG upper-bounds every interval → trigger (a)'s swap-in is
safe by construction. Restores the cross-interval conservatism the dropped old-D2 provided, soundly.
- **Loud-fail:** after the walk, re-run the gate on the **final stored result**; on failure →
  raise loud, do NOT store (`HasSafeFallback()` stays false — "re-generate a new task set").
  Raising TL worsens interference, so a seed-level miss is not walk-fixable.
- **Scope:** worst-case DAG fed to **offline `ComputeSafeFallback` only**. Online sim + optimizer
  keep the actual per-interval DAGs → online byte-identical. NOT the old `use_wcet_execution_time`
  global flag. Seed TL uses the worst-case `et_mean`; perf-task TL grid unchanged (task-structure).

## Why a hard per-candidate gate (not the soft SP_Func penalty)

The user's objective is a HARD guarantee: `∀ important i: ddl_miss_chance_i ≤ sp_threshold_i`.
`SP_Func` (`SP_Metric.h:31-41`) only SOFTLY discourages violations (exponential penalty past
threshold) → a mild violation on a low-weight important task can be SP-optimal → soft ≠ strict.
`ddl_miss_chance` is PROBABILISTIC (`RTA.cpp:154`, RTA dist = ET convolution). The metric's ET
model: perf = point-mass at TL; non-perf = full Gaussian (`SP_Metric.cpp:76-86`); the sim runs
non-perf at `GetAvgValue()` and caps perf at `min(et_mean,TL)` → **metric `ddl_miss_chance` ≥
sim's actual** → a gate on the METRIC is a sound pessimistic bound. In the TL ≤ et_mean seed
region `ddl_miss_chance=0` (constraint free); TL > et_mean (perf region) makes it bind → the gate
is load-bearing.

## Scope (IS / IS-NOT)

**IS:** offline `ComputeSafeFallback()` → `ResourceOptResult` (DM-grouped PA + et_mean-bounded TL
seed → gated TL walk → best-SP-feasible incumbent) stored as `safe_fallback_`; worst-case-DAG
construction + loud-fail (§8); the et_mean-bounded TL seed helper; the hard gate
(`ImportantTasksMeetThresholds`); a separate compute-time profile (NOT in
`scheduler_execution_time.txt`). TDD covers the seed, the DM-grouped PA, gate reject/keep,
feasible-by-construction, ET-exclusion.
**IS NOT:** the fall-back INVOCATION (when to swap in) — that's P0.7 (incl. the online HALT,
trigger (b)); the generation-time schedulability GUARANTEE — that's P0.8 (certifies the seed
region); a WCET-mode ablation of the ONLINE path or a global-max WCET precompute — DROPPED
2026-07-30 (do NOT re-introduce; §8 is offline-only, NOT the old global flag); a change to the SP
metric / `SP_Func` / the online optimizer path; a new A/B scheduler arm.

## Done when

- [x] Steps 0.5–7 LANDED (important-task label via P0.9; DM-grouped seed; et_mean-bounded TL
      seed; hard gate `ImportantTasksMeetThresholds` at `UpdateRecords`; `ComputeSafeFallback` +
      orchestrator pre-call + ET-exclusion; rename to SafeFallback). 17/17 ctest.
- [x] **§8a:** worst-case-DAG builder (per-task point mass at `max(execution_time_max)` across
      interval YAMLs). TDD: per-task `max_time` = max across fixture intervals; perf TL grid kept.
- [x] **§8b:** wire worst-case DAG into `ComputeSafeFallback` (pre-call passes it, not
      `dag_tasks_`); seed TL ≤ worst `et_mean`.
- [x] **§8c:** loud-fail — post-walk gate re-check on final stored result; failure raises + does
      NOT store. TDD: unschedulable worst-case DAG → raises + `HasSafeFallback()` false.
- [x] **§8d:** cross-interval safety VERIFIED by two tests. §8d.1
      `StochasticallyDominatesEveryInterval` — for every interval j, every task i,
      `worst.tasks[i].max_time >= interval[j].tasks[i].max_time` (the worst-case point mass
      stochastically dominates every interval's per-task dist → the gate's `ddl_miss_chance` on
      the worst-case DAG upper-bounds every interval). §8d.2
      `ComputeSafeFallback_ReGatesCleanAgainstEveryInterval` — a fallback computed on the
      worst-case DAG re-gates clean (`ImportantTasksMeetThresholds` true) against EACH interval's
      DAG individually. A failure here is a soundness bug. 17/17 ctest.
- [x] **§9a:** dispatcher THROWS when no safe fallback is pre-computed. The old lazy
      backstop (`Optimize_w_TL_ScratchOrIncre` → `ComputeSafeFallback(dag_tasks_)`) was
      unsound — `dag_tasks_` is one interval, not the worst-case DAG, and the dispatcher
      has no `dag_tasks_vecs_` to build it. Fail loud; the orchestrator pre-call is the
      only sound caller. TDD: `Dispatcher_ThrowsWhenNoSafeFallbackPreComputed`.
- [x] **§9b:** `ImportantTasksMeetThresholds(dag, sp_params, pa, tl)` self-contained
      overload (derives RTAs via `ProbabilisticRTA_TaskSet`, delegates to the contract
      overload). Simplifies the §8c loud-fail re-gate; the contract overload stays the
      zero-extra-eval path for the in-walk gate.
- [x] `cmake --build build_test --target check.SP_OPT -j5` green (**17/17** this run; the
      pre-existing `testPublisher` `PeriodicReleaser.v1` wall-clock flake passed here too).
- [x] **§10:** readability refactor — extract the inlined `structure_matches` flag from
      `BuildWorstCaseDagAcrossIntervals` into a header fn `TaskStructureMatches(const Task&,
      const Task&)` (`DAG_Model.h` decl, `WorstCaseDAG.cpp` def). Behavior-preserving:
      compares id/period/deadline/processorId/name + full `timePerformancePairs` grid; ET
      dist excluded (the builder fuses the max across intervals). 5 TDD tests; 17/17 green.
- [x] COMMITTED `630cda4d` (user, 2026-07-31) — all of §0.5–10 (code + records).

## Open decisions

- **D1** ✅ (P0.9): top-50% by `sp_weight`, persisted `bool is_important`. ~~**D2**~~ ❌ SUPERSEDED
  2026-07-30 (no WCET precompute; leans on P0.8 + its own gate). **D3** ✅ per-taskset. **D4** ✅
  in-memory `safe_fallback_`. **D5/D6/D7** ✅ hard per-candidate gate on probabilistic
  `ddl_miss_chance ≤ threshold`, normal-ET, offline-scoped, skip-and-continue (NOT the old WCET
  point-mass RTA). **D8** ✅ (via the gate): PA descent safe-to-add-later; the gate is the
  constraint. **D9** ✅ (2026-07-31): worst-case DAG (point mass at `max(execution_time_max)`
  across interval YAMLs) → `ComputeSafeFallback`; stochastic dominance; offline-only; loud-fail.

## Reference docs

- `SimulationOrchestrator.cpp:300-324` — optimizer construction + pre-call + interval loop.
- `SimulationOrchestrator.cpp:514-516` — sim perf ET downward cap (`min(et_mean,TL)`).
- `OptimizeSP_TL_Incre.cpp:888-` — `ComputeSafeFallback` (forces `use_wcet_execution_time=false` :893).
- `OptimizeSP_TL_Incre.cpp:705-780` — `DeadlineMonotonicPriorityVec` + `SeedIncumbentFromDMFast`.
- `OptimizeSP_TL_Incre.cpp:42-56,483-498` — `Find_Close_ExecutionTime` + `InitializeTimeLimitsFromETConfig`.
- `SP_Metric.{h,cpp}` — `ImportantTasksMeetThresholds`, `ObtainSP_DAG_From_Dists`, `ApplyTimeLimitsToTasksExecutionTime`, `SP_Func`.
- `RTA.cpp:32-44,154` — `GetRTA_OneTask` (ET convolution) + `GetDDL_MissProbability`.
- `RegularTasks.h:87,111` — `GetAvgValue()` (= et_mean); `bool is_important`.
- Memory [`p06-static-solution-fallback-seed`](../../../), [`p07-fallback-mechanism`](../../../), [`p08-important-task-schedulability`](../../../).

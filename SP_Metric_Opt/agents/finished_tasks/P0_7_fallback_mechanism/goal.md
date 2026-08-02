# P0.7 — Online Fall-Back Mechanism (Two Triggers)

**Priority:** P0 (publication-correctness: guarantees important-task safety).
**Status:** code LANDED (git add-only, NOT committed); step 5 smoke run + SP-penalty measurement remain.
**Depends on:** P0.6 (`safe_fallback_`, computed on a worst-case DAG → cross-interval safe),
P0.8 (important-task schedulability under DM — the guarantee the safe fallback is a meaningful floor).
**Blocks:** none (top of the fall-back chain).

## Goal

Add **two online fall-back triggers** that protect important-task safety during the
interval loop. Both swap in (or compare against) the precomputed safe fallback
(`safe_fallback_`, P0.6), computed on a worst-case DAG (per-task WCET point mass at
`max(execution_time_max)` across all interval YAMLs) → stochastically dominates every
interval's per-task dist → safe under jumped ETs by construction.

1. **Trigger (a) — ET-jump, BEFORE optimization.** The optimizer's `dag_tasks_` member
   IS the saved old dag. Each new interval's dag arrives as the dispatcher's
   `dag_tasks_update` param. If any task's Gaussian `et_mean` in the new dag is **≥ 1.5×**
   its `et_mean` in the saved old dag → use `safe_fallback_` directly for this interval
   (skip the online walk). The new dag is still absorbed so the next interval compares
   against this interval's jumped ET (fires on the transient jump, not the sustained
   level). Guarded to skip interval 0 (`reoptimization_interval_count_ == 0`, no prior dag).

2. **Trigger (b) — during-walk gate + post-walk backstop.**
   - **(b-i) during-walk:** when a challenger with better SP than the champion is found,
     gate it through `ImportantTasksMeetThresholds`. If UNSAFE (an important task's
     DDL-miss-chance > its `sp_threshold`) → **REJECT the challenger and KEEP WALKING**
     (reject-and-continue). The walk is NEVER interrupted and NEVER adopts
     `safe_fallback_` mid-flow (that would break the incremental property).
   - **(b-ii) post-walk backstop:** after the walk finishes on its own, run
     `ImportantTasksMeetThresholds` on the FINAL `res_opt_`; if it FAILS → adopt
     `safe_fallback_` via `AdoptSafeFallbackAsIncumbent`; else keep the walk's result
     EVEN IF `safe_fallback_` would have higher global SP. **Schedulability decides, not SP.**

### Budget asymmetry (user direction)

> "during online, budget is very tight, use this to adjust your design."

Online is the CHEAP counterpart to P0.6's offline walk (ample budget, skip-and-continue).
Trigger (a) = O(N) per-task `et_mean` compare, cheaper than the walk. Trigger (b-i) gates
in-walk at the existing adoption chokepoint; (b-ii) = one gate check post-walk. All three
run inside the `DeterminePrioritiesAndBudgets` ET bracket (trigger (a) must run
pre-`AbsorbUpdatedDAG`; the backstop IS part of the scheduler decision) — see "ET bracket".

### What the guard checks (D2)

The guard reads the SP computation's own **per-task DDL-miss-chance** — the analytic
RTA-tail `GetDDL_MissProbability(rtas[i], deadline)` (`RTA.cpp:154-169`), the quantity
`sp_threshold` is defined against (P2.13: SP threshold = DDL-miss threshold; `SP_Func`
`SP_Metric.h:31-41` branches `threshold >= violate_probability` ⇒ safe). No new simulation
sampling, no P2.6 dependency, no guard-vs-metric drift (guard + metric read the same
number). Empirical job-history miss rate (β) is deferred (P2.6). Same detection as P0.6's
offline gate (`ImportantTasksMeetThresholds`) — reused, not duplicated.

### Where the triggers insert

Per-interval flow in `SimulateInterval` (`SimulationOrchestrator.cpp`):
1. **Trigger (a)** runs at the TOP of BOTH dispatchers (`Optimize_w_TL_ScratchOrIncre` +
   `OptimizePureIncremental`), BEFORE `AbsorbUpdatedDAG` overwrites `dag_tasks_`. On a
   trip: absorb the new dag → adopt the fallback → `count++` → return (walk skipped).
2. `DeterminePrioritiesAndBudgets` runs the online walk. **Trigger (b-i)** gates each
   would-beat challenger inside `UpdateRecords` (reject-and-continue). **Trigger (b-ii)**
   `AdoptFallbackIfUnschedulable` runs after the walk, before `return opt_pa_`.
3. The RTDA rollout runs jobs against `res` (~530-564).
4. `ObtainSP_TaskSet_And_TimeLimits` is pushed to `interval_sp_metrics_` (573).

Both triggers produce a final `res` the rollout + SP push consume → no metric/rollout desync.

### ET bracket

The original "run outside the `DeterminePrioritiesAndBudgets` bracket" intent CANNOT be
cleanly satisfied: trigger (a) needs the optimizer's internal `dag_tasks_` (invisible to
the orchestrator); the backstop IS part of the scheduler decision. So both run INSIDE the
bracket. Net: trigger (a) SKIPS the walk on a trip (reduces bracketed time); the backstop
ADDS one fresh RTA eval per interval only when it fires. This is scheduler-decision cost in
the prod arm, NOT the SP measurement. `scheduler_execution_time.txt` expected to rise
slightly in the prod arm; the measurement arm (`enable_fallback_use_=false`) matches P0.6.

### Per-interval, not sticky (D3)

Each interval re-checks. If interval *i* falls back but *i+1*'s solution passes both
triggers, *i+1* uses the optimizer solution. The fallback is a per-interval floor.

## Scope

**IS:** trigger (a) ET-jump detector; trigger (b-i) during-walk reject-and-continue gate;
trigger (b-ii) post-walk backstop; a per-interval `interval_fallback_log.txt`; TDD coverage.

**IS NOT:** the safe fallback itself (P0.6); the schedulability guarantee (P0.8); a change
to the SP metric or `SP_Func` (the guard READS the DDL-miss number, doesn't redefine it);
the important-task selection rule (D1, shared); a new scheduler arm (runtime behavior of
INCR-family modes, not a comparable arm — the A/B measures the SP penalty).

## A/B (user direction)

> "i want to know how much performance penalty the fall-back will add, compared with
> current algorithm without fallback."

Arms = {INCR-family + full chain} vs {today's INCR-family, no fall-back}. Metric = global
SP. The USER re-runs prod; this task does NOT run the A/B. The fall-back is expected to
REDUCE SP; the A/B quantifies by how much.

## Done when

- [x] **D1:** important = top-50% by `sp_weight`, persisted as `bool Task::is_important`.
- [x] **D2:** analytic DDL-miss-chance (`GetDDL_MissProbability`) — no P2.6 dep, no drift.
- [x] **D3:** per-interval (not sticky).
- [x] **D4:** trust the worst-case-DAG certificate — `safe_fallback_` is cross-interval
      safe → trigger (a)'s direct swap is sound (no re-check).
- [x] **D5:** INCR-family modes only (BF/RM/CFS have no online optimizer to fall back from).
- [x] **D6 OVERTURNED:** ~~HALT on first unsafe~~ → reject-and-continue (halting early
      loses later safe+better candidates → worse).
- [x] **D7 OVERTURNED:** ~~higher-SP winner~~ → schedulability decides (injecting the
      SP-better fallback mid-flow breaks the incremental property → worse).
- [x] Trigger (a) ET-jump detector + dispatcher wiring (4 tests).
- [x] Trigger (b-i) during-walk reject-and-continue gate (online arm via master flag).
- [x] Trigger (b-ii) post-walk backstop `AdoptFallbackIfUnschedulable` (2 tests + 2 throw tests).
- [x] 4-directive refactor: single flag `enable_fallback_use_`; rename
      `ShouldShortCircuitOnETJump`→`SkipOptOnETJump`; backstop throws on enabled-but-no-
      fallback + on rescue-also-fails-gate.
- [x] `interval_fallback_log.txt` (optimizer record sites + `FormatIntervalFallbackLogCsv`
      + `GetIntervalFallbackLog` + RunOrchestrator write; 5 + 6 tests).
- [x] D5 mode-gating verified by construction (triggers live only in INCR dispatchers).
- [ ] Smoke run: confirm `interval_fallback_log.txt` produced at small N.
- [ ] SP-penalty measurement (prod ON vs measurement OFF at matched N; report delta).
- [ ] `dev_log.md` + memory updated; `git add` staged; user reviews.

## Reference docs

- `SimulationOrchestrator.cpp:313-410` — `DeterminePrioritiesAndBudgets`; `:339-370` INCR-family routing; `:530-575` rollout + SP push.
- `RTA.cpp:154-169` — `GetDDL_MissProbability` (D2 α, the guard quantity).
- `SP_Metric.h:31-41` — `SP_Func`; `:162-195` `ImportantTasksMeetThresholds` + `WorstCaseImportantTaskMissInfo`.
- `ParametersSP.h` — `thresholds_node` (per-task `sp_threshold`).
- `RegularTasks.h` — `Task::execution_time_dist` / Gaussian `et_mean` (trigger-(a) compare quantity).
- Memory `p213-important-task-ddl-vs-sp-metric` — SP threshold = DDL-miss threshold.
- P0.6 `goal.md` — `safe_fallback_` (worst-case DAG → cross-interval safe). P0.8 `goal.md` — schedulability guarantee (D4).

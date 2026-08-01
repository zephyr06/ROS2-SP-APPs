# P0.6 — Tasks (working checklist)

> Offline fall-back artifact for P0.7. Seed at the P0.8-certified operating point (DM-grouped
> PA + TL = largest grid option ≤ `et_mean`) → TL-only walk with a HARD per-candidate gate
> (`ddl_miss_chance ≤ sp_threshold` for all important tasks) → keep best-SP-feasible as
> `safe_fallback_`. Algorithm + soundness in `goal.md`. **COMPLETE + COMMITTED `630cda4d`
> (2026-07-31): sections 0.5–10 LANDED; 17/17 ctest green.**
> §9 (2026-07-31): dispatcher THROWS when no fallback pre-computed (was an unsound
> lazy backstop) + `ImportantTasksMeetThresholds` self-contained overload.
> §10 (2026-07-31): readability refactor — extracted `TaskStructureMatches` from
> the worst-case-DAG builder (behavior-preserving; 5 TDD tests; 17/17 green).

## 0. Design decisions
- [x] **D1 (via P0.9):** top-50% by `sp_weight`, persisted `bool Task::is_important`.
- [x] **D3:** per-taskset. **D4:** in-memory `safe_fallback_`.
- [x] **2026-07-30:** old D2 (global-max WCET) DROPPED; old D5/D6/D7 REVIVED as the hard
      probabilistic gate (normal-ET, offline-scoped). D8 RESOLVED via the gate.
- [x] **D9 (2026-07-31):** caller builds a worst-case DAG (per-task point mass at
      `max(execution_time_max)` across interval YAMLs) → `ComputeSafeFallback` on it →
      cross-interval safety by stochastic dominance. Loud-fail on final gate-reject. See §8.

## 0.5. `bool is_important` + generator labeling — DONE (via P0.9)
- [x] `RegularTasks.h:111`; parse `RegularTasks.cpp:87`; emit `:126`. Generator labels
      top-`IMPORTANT_TASK_RATIO` (0.5). No P0.6 work.

## 1. DM-grouped PA seed — DONE (via P0.9)
- [x] `DeadlineMonotonicPriorityVec()` (`OptimizeSP_TL_Incre.cpp:705-721`, reads `is_important`
      `:713-714`); `SeedIncumbentFromDMFast()` (`:772-780`). Old helper SUPERSEDED.

## 2. ~~Global-max WCET precompute~~ — DROPPED (2026-07-30)
- [x] SUPERSEDED. P0.6 computes no WCET; leans on P0.8's grid-wide certification (perf WCET =
      `et_mean`) for the seed region + its own gate for TL > et_mean.

## 3. TL seed (largest grid option ≤ `et_mean`) — DONE (2026-07-30)
- [x] `FindLargestTimeLimitAtOrBelow(pairs, et_mean)` + `SeedTimeLimitsAtOrBelowEtMean()`
      (directional ≤ et_mean; non-perf = -1; fall back to smallest + warning). TDD (7 tests); 17/17.

## 4. Hard per-candidate gate — DONE (steps 4a+4b v2, 2026-07-30)
- [x] **4a:** `ImportantTasksMeetThresholds(dag, sp_params, pa, tl, node_rtas) -> bool`
      (`SP_Metric.{h,cpp}`). Zero-extra-eval — takes the caller's already-materialized `node_rtas`.
      TDD (4 tests); 17/17.
- [x] **4b v2 (FINAL):** gate INSIDE `UpdateRecords` (commit chokepoint), fires ONLY on would-beat
      via `WouldBeatIncumbent` + `bool enforce_important_task_gate_=false` (true only in tests/
      `ComputeSafeFallback` → prod byte-identical). Gate-reject → return false (no `CommitIncumbent`);
      eval reports incumbent SP (ghost-SP fix). PA descent RE-ENABLED unconditionally. RTA source =
      re-runs `CommitIncumbent`'s `rta_cache_.Evaluate(dag_tasks_,opt_pa_,tl)` (candidate's FINAL
      {pa,tl}); `|diff|<=1`-cheap, never mutates champion → `cache_backup` revert sound. Seed TL ≤
      et_mean → `ddl_miss_chance=0` → gate can only REJECT. TDD (5 tests); 17/17. NOT committed.
- [x] **4b-followup — unify the 3 acceptance predicates (DEFERRED 2026-07-30 per user).** The
      ghost-SP override is a symptom of divergent predicates (PA-loop local vs TL-walk local vs
      commit-global). Unification is LARGE (breaks `StubTLWalkOptimizer` test framework; NOT
      byte-identical flag-off; override already correct = cleanliness not bugfix). 4b v2 is FINAL.

## 4-old. ~~Skip-and-continue WCET filter~~ — SUPERSEDED (first 2026-07-30 limb, reversed above)

## 5. `ComputeSafeFallback()` offline + ET-excluded — DONE (steps 5a+5b, 2026-07-31)
- [x] **5a:** `ComputeSafeFallback()` + `optional<ResourceOptResult> safe_fallback_` +
      `HasSafeFallback()`/`GetSafeFallback()` + dispatcher lazy-check in
      `Optimize_w_TL_ScratchOrIncre`. Throwaway sibling `fallback_solver` → `this`'s live
      `res_opt_`/cache/flag UNTOUCHED → online byte-identical. Forces
      `use_wcet_execution_time=false` (gate is probabilistic). **TIMEOUT-REGRESSION FIX:**
      installs its OWN `BFDLSharedBudget` around the WHOLE compute (seed eval + walk) — else the
      lazy dispatcher ran it before the dispatcher's budget → runaway seed SP-eval (`testINCRTimeout`
      21.8s; fixed → 17/17). TDD (4 tests); 17/17. NOT committed.
- [x] **5b:** orchestrator pre-call (`RunSimulation:313-317`, INCR-family branch, after
      `incr_optimizer_` build, before the interval loop); separate `safe_fallback_compute_time_s_`
      + `GetSafeFallbackComputeTime()` (DISTINCT from `scheduler_exec_time_s_`);
      `RunOrchestrator.cpp` prints `SafeFallbackComputeTime_s:` + writes
      `safe_fallback_compute_time.txt`; `PreComputesSafeFallback_ExcludesSchedulerET` test.
      ET-exclusion STRUCTURAL (disjoint call sites) + ASSERTED (DM-mode `==0.0` exact). NOT committed.
- [x] **Two safety assumptions VERIFIED.** (1) Sim perf ET = `min(et_mean,TL)` downward cap
      (`SimulationOrchestrator.cpp:514-516`) → metric `ddl_miss_chance` ≥ sim → gate sound.
      (2) Gate IS the constraint → group-lock concern moot. 5 `GateWiring_*` tests pass w/ PA on.

## 6. Verification + records — DONE
- [x] 17/17 ctest (`cmake --build build_test --target check.SP_OPT -j5`).
      `ComputeSafeFallback_PopulatesGateHeldArtifact` asserts stored PA = DM-grouped + gate holds.
      Seed TL ≤ et_mean → runtime ET ≤ TL ≤ et_mean = P0.8 perf WCET. `dev_log.md` + memory
      updated; `git add` staged; user reviews (no commit).

## 7. Post-review refinement (2026-07-31) — DONE
- [x] **Item 1 — rename** `static_solution`→`SafeFallback` across C++ (`grep`-verified zero left;
      folder name stays). **Item 2 — comment trim** (gate comment 4→3 lines; others ≤3; TIMEOUT
      note kept). **Item 3 — orchestrator pre-call ALREADY DONE** (no-op). **Item 4 —
      `sub`→`fallback_solver`** (`:903`). **Item 5 — reopt-then-incre SKIPPED (user):** gate-vs-
      reopt-cache incompatible (gate's `rta_cache_.Evaluate` fires unconditionally; reopt clears+
      disarms the cache → `|diff|>1` throw). Stays incremental-only. Re-file if SP quality insufficient.

## 8. Worst-case-DAG + loud-fail (2026-07-31 — cross-interval safety for P0.7 trigger (a))
> The 2026-07-30 design scoped `safe_fallback_`'s safety to ONE taskset. P0.7 trigger (a) swaps
> it in on an ET-jump ACROSS intervals → unsafe. User direction 2026-07-31: the caller builds a
> worst-case DAG (per-task WCET point mass at `max(execution_time_max)` across all interval YAMLs)
> and computes the artifact on THAT. Plus loud-fail if the final result fails the gate.
> See `goal.md` "WORST-CASE-DAG (2026-07-31)" for the soundness proof + code grounding.

- [x] **Finding (2026-07-31).** Two code facts make the within-one-taskset certificate
      insufficient for trigger (a): (1) `ComputeSafeFallback` forces
      `use_wcet_execution_time=false` (`:893`) → env tasks keep their base Gaussian (NOT a WCET
      point mass); (2) the generator's per-interval `Et_sigma=np.std(subset)` (`orchestrator.py:375`)
      is mean-independent → "longest by avg ET" does NOT bound `ddl_miss_chance`. The dropped
      old-D2 used to provide the cross-interval bound. Reconciled with the user.
- [x] **D9 RESOLVED (user 2026-07-31).** Caller builds worst-case DAG (per-task point mass at
      `max(execution_time_max)` across interval YAMLs) → `ComputeSafeFallback` on it. Offline-only;
      online byte-identical. Sound by stochastic dominance (any interval's ET draw ≤ its `max_time`
      ≤ worst-case max). Loud-fail on final gate-reject.
- [x] **Step 8a — worst-case-DAG builder.** Orchestrator iterates `LoadIntervalConfigs()` (all
      interval YAMLs), per task records `max(execution_time_max)` across intervals, constructs a
      `DAG_Model` where each env/non-perf task's dist = point mass at that max (perf tasks keep
      their TL grid + structure). TDD: per-task `max_time` = max across the fixture's interval
      YAMLs; perf-task TL grid preserved. **DONE 2026-07-31:** free fn
      `BuildWorstCaseDagAcrossIntervals(const std::vector<DAG_Model>&)` declared in `DAG_Model.h`,
      impl in NEW `sources/TaskModel/WorstCaseDAG.cpp` (CMake `GLOB_RECURSE` — reconfigure picked it
      up, no CMakeLists edit). Structure copied from interval 0; structural mismatch across intervals
      raises `std::runtime_error`. 3 TDD tests (max across intervals; perf TL grid preserved;
      raises on task-count + period mismatch); 17/17 green. NOT committed.
- [x] **Step 8b — wire worst-case DAG into `ComputeSafeFallback`.** Orchestrator pre-call
      (`RunSimulation:313-317`) passes the worst-case DAG (not `dag_tasks_`/interval-0) to
      `ComputeSafeFallback`. Seed TL ≤ worst `et_mean` (step-3 helper on the worst-case DAG).
      Walk + gate unchanged. **DONE 2026-07-31:** signature
      `ComputeSafeFallback(const DAG_Model& worst_case_dag)` (no overload/default — reduce
      optional args); sibling built from `worst_case_dag` + fresh `SP_Parameters(worst_case_dag)`;
      dispatcher safety-net `:678` → `ComputeSafeFallback(dag_tasks_)` (1-interval degenerate
      worst-case); orchestrator pre-call builds `worst_case_dag = BuildWorstCaseDagAcrossIntervals(
      dag_tasks_vecs_)` then passes it (kept inside the timed block). 4 existing tests updated to
      pass `dag_tasks` + new `ComputeSafeFallback_UsesWorstCaseDagNotIntervalZero` (stored result
      re-gates clean against the WORST-CASE DAG, not interval-0). 17/17 green. NOT committed.
- [x] **Step 8c — loud-fail.** After the walk, re-run `ImportantTasksMeetThresholds` on the FINAL
      stored result; on failure → raise loud (do NOT store → `HasSafeFallback()` stays false). TDD:
      unschedulable worst-case DAG (important task deadline < its own WCET) → raises +
      `HasSafeFallback()` false; schedulable → stores + true. **DONE 2026-07-31:** loud-fail guard
      AFTER the walk, BEFORE store — reconstruct `{pa,tl}` from `fallback_solver.CollectResults()`,
      fresh `ProbabilisticRTA_TaskSet(UpdateTaskSetPriorities(ApplyTimeLimitsToTasksExecutionTime(
      worst_case_dag.tasks, tl), pa))` (NOT `rta_cache_.Evaluate` — sibling cache armed only inside
      the walk); `ImportantTasksMeetThresholds` on the worst-case DAG; on false → `CoutWarning` +
      `throw std::runtime_error` (precedent `RTA_Cache.cpp:358`/`testRTA.cpp:1152`; `CoutError` does
      a bare `throw;`→terminate, untestable). Global-flag restores (`disable_time_limit_opt`/
      `use_wcet_execution_time`) moved ABOVE the loud-fail check so a throw doesn't leak forced-flag
      state. Unschedulable test uses a NON-PERF important task (perf capped at TL ≤ et_mean → no
      overrun; non-perf TL=−1 keeps full dist → WCET>deadline ⇒ ddl_miss_chance==1.0). 2 TDD tests
      (schedulable stores; unschedulable raises + unstored); 17/17 green. NOT committed.
- [x] **Step 8d — verify cross-interval safety.** DONE 2026-07-31. Two tests ground the
      soundness leg: §8d.1 `BuildWorstCaseDag.StochasticallyDominatesEveryInterval` (per
      interval j, per task i, `worst.tasks[i].max_time >= interval[j].tasks[i].max_time` → the
      worst-case point mass stochastically dominates every interval's per-task dist → the gate's
      `ddl_miss_chance` on the worst-case DAG upper-bounds every interval); §8d.2
      `ComputeSafeFallback_ReGatesCleanAgainstEveryInterval` (a fallback computed on the
      worst-case DAG re-gates clean against EACH interval's DAG individually — a failure is a
      soundness bug). The two safety assumptions in `goal.md` "Done when" updated with the
      cross-interval leg.
- [x] `cmake --build build_test --target check.SP_OPT -j5` green (17/17 ctest).

## 9. Dispatcher throw + gate overload (2026-07-31 — tighten the §8 contract)
> §8 made `ComputeSafeFallback` take the worst-case DAG. Two follow-ups the user
> asked for: (1) the dispatcher's lazy backstop was unsound — it can't build the
> worst-case DAG from `dag_tasks_`; (2) the gate predicate gained a self-contained
> overload so the loud-fail re-gate (and P0.7) doesn't repeat the bake+prioritize+RTA
> boilerplate.

- [x] **9a — dispatcher THROWS when no safe fallback is pre-computed.** The old
      `Optimize_w_TL_ScratchOrIncre` lazy-called `ComputeSafeFallback(dag_tasks_)` on
      first dispatch — but `dag_tasks_` is ONE interval, NOT the worst-case DAG → the
      cross-interval soundness §8 guarantees would be silently lost. The dispatcher has
      no `dag_tasks_vecs_`, so it CANNOT build the worst case soundly → fail loud:
      `throw std::runtime_error` (the orchestrator pre-call is the ONLY sound caller).
      Header comment updated (removed the "1-interval degenerate worst-case backstop"
      text). Test (3) rewritten `Dispatcher_ThrowsWhenNoSafeFallbackPreComputed`
      (was `..._LazyPopulatesSafeFallback_WhenNotPreCalled`); the 6 single-interval
      `CounterDispatcherSynthetic` routing tests + the `opt_reopt` line +
      `testINCRTimeout`'s 2 budget-guard tests got a `ComputeSafeFallback(dag)`
      pre-call (on these fixtures `dag` IS the degenerate worst case; gate vacuous —
      no important tasks → stores the seed fast).
- [x] **9b — `ImportantTasksMeetThresholds` self-contained overload.** New
      `ImportantTasksMeetThresholds(dag, sp_params, pa, tl)` (no `node_rtas`) in
      `SP_Metric.{h,cpp}`: bakes TL → applies PA → `ProbabilisticRTA_TaskSet` →
      delegates to the contract overload. Used to simplify `ComputeSafeFallback`'s §8c
      loud-fail re-gate (dropped the manual `ApplyTimeLimitsToTasksExecutionTime` +
      `UpdateTaskSetPriorities` + `ProbabilisticRTA_TaskSet` boilerplate at the call
      site). The contract overload stays the zero-extra-eval path for the in-walk gate.
- [x] `cmake --build build_test --target check.SP_OPT -j5 --clean-first` green
      (16/17 ctest; the 1 failure = pre-existing `testPublisher` `PeriodicReleaser.v1`
      wall-clock flake — passes in isolation, unrelated to this change). 6 targeted
      dispatcher + §8 tests pass by name.
- [x] `dev_log.md` + memory updated; COMMITTED `630cda4d` (user, 2026-07-31).

## 10. Readability refactor of the worst-case-DAG builder (2026-07-31)
> `BuildWorstCaseDagAcrossIntervals` had its structural-equality check inlined as a
> convoluted `bool structure_matches` flag nested two loops deep. Extract it into
> its own function for readability (user request). Behavior-preserving.

- [x] **10a — extract `TaskStructureMatches(const Task&, const Task&)`.** Declared
      in `DAG_Model.h` next to `BuildWorstCaseDagAcrossIntervals` (precedent =
      `FindLargestTimeLimitAtOrBelow` — header-declared so it's directly unit-
      testable); defined in `WorstCaseDAG.cpp`. Compares id/period/deadline/
      processorId/name + the full `timePerformancePairs` grid (size + each
      `time_limit`/`performance`); ET dist DELIBERATELY excluded (the builder fuses
      the max across intervals). The builder's per-task loop collapses to a single
      `if (!TaskStructureMatches(...)) throw`.
- [x] **10b — TDD.** 5 new `TaskStructureMatches.*` tests written FIRST (red), then
      green: identical match; differing ET dist still matches; each scalar-field
      mismatch (id/period/deadline/processorId/name); TL-pair count mismatch;
      TL-pair value mismatch (time_limit + performance). Doc fix: the header comment
      listed `is_important` among compared fields, but the predicate never compared
      it (generator-set once → stable across intervals → latent, not live) —
      corrected to the actual field set.
- [x] `cmake --build build_test --target check.SP_OPT -j5 --clean-first` green
      (**17/17**; the `testPublisher` flake passed this run).
- [x] `dev_log.md` + memory updated; COMMITTED `630cda4d` (user, 2026-07-31).

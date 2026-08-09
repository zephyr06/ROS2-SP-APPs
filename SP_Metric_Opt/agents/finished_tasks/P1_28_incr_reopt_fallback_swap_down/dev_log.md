# P1.28 — INCR_Reopt_10 Fallback Swap-Down — Dev Log

> Detailed working log. Append chronological entries below. On completion, append a
> one-line milestone to the top-level `agents/dev_log.md`.

## 2026-08-08

- Task created. User anomaly: `taskset_2` `INCR_Reopt_10` interval SP collapses to
  0.527888 on several intervals while `INCR_WCET` stays 0.885–0.954 (aggregates
  0.855923 vs 0.920009). "This is unreasonable" — a schedulable high-SP plan exists
  (WCET reaches it on the same taskset).
- Read both arms' `interval_sp_metrics.txt` + `interval_fallback_log.txt`:
  - INCR_Reopt_10: `backstop_verdict=adopted_fallback` at intervals 16,21,38,41,44,52,
    culprit always task 0, miss_chance 0.43–0.70 > threshold 0.371116 → SP 0.527888.
  - INCR_WCET: `kept_walk` on ALL 60 intervals; never trips the backstop.
- `taskset_characteristics_interval_0.yaml`: task 0 (`task_1`) `important: true`,
  `sp_threshold: 0.3711164937976443` (== fallback-log threshold), wide ET
  (mu=133.5/sigma=44.7/max=199.5), TL-optimizable (has `performance_records`).
- Git: HEAD = `a609320e`. P1.27 fix `cbdde63b` ("fix BF fall-back bug") IS in HEAD
  (Aug 7 20:10:59; run was 20:17) but touched ONLY `OptimizeSP_TL_BF.cpp` — the INCR
  path was never fixed. This is the INCR analogue of P1.27.
- Code walk (`OptimizeSP_TL_Incre.cpp`):
  - `UpdateRecords:238-258` — during-walk gate (b-i) runs ONLY if `rta_cache_active_`.
  - `RunIntervalDescent:570` / `SeedBaselineAndArmCache:540-553` — reopt from-scratch
    beam runs DISARMED: `ResetIncumbentBaseline(true)` clears cache before
    `CallOptimizerGivenTimeLimits(...,from_scratch=true)`; cache re-armed only after.
  - `CallOptimizerGivenTimeLimits:275-280` — `from_scratch` branch calls
    `OptimizeFromScratch` + `UpdateRecords` with `rta_cache_active_=false` → gate
    branch skipped. Comment `:252-257` confirms "gated post-hoc by
    AdoptFallbackIfUnschedulable".
  - `AdoptFallbackIfUnschedulable:1106` — post-hoc backstop (b-ii); on gate FAIL it
    calls `AdoptSafeFallbackAsIncumbent` (RM-Fast), no re-search for a schedulable
    high-SP plan. This is the swap-down.
- `interval_walk_stats.txt`: reopt fires every 10 intervals (0,10,20,30,40,50 — high
  `evaluated_challenger_count`). Fallback intervals (16,21,38,41,44,52) are
  post-reopt incremental intervals with `improving_challenger_count=0` → the
  disarmed-committed unschedulable incumbent persists, backstop catches it, swaps to
  RM-Fast. Sticky 0.527888 across 16–19 = RM-Fast incumbent kept (schedulable);
  recovery at reopt@20 (11 evals, 8 improving → 0.954072).
- `SimulationOrchestrator.cpp:380-387`: INCR_WCET sets `use_wcet_execution_time=true`
  → deterministic WCET surface → SP-max plan always schedulable for task 0 → backstop
  never fires. WCET's 0.92 is a stricter-ET artifact, NOT a better search.
- ROOT CAUSE CONFIRMED: INCR reopt from-scratch beam commits the SP-max plan without
  the schedulability gate (disarmed), then the post-hoc backstop binary-swaps to
  RM-Fast instead of selecting a schedulable high-SP plan. Identical bug shape to
  P1.27 (BF), which was fixed BF-only.
- `goal.md` written with full evidence + fix direction (mirror P1.27: in-search gate
  on the INCR reopt beam; keep post-hoc backstop as safety net; bit-identical for
  legacy no-`is_important` tasksets). Next: TDD RED → GREEN.

## 2026-08-08 (re-investigation — CORRECTION 2)

- User pushed back: "i don't understand why interval 21 and 22 adopts fallback, since
  it's based on incremental optimization. your logic doesn't really make sense."
  Re-investigated the INCREMENTAL path. Both prior framings disproven by the code:
  - **"Disarmed reopt beam"** (original goal.md): applies to REOPT intervals only.
    Collapses are INCREMENTAL — reopt@20 = 0.954072 `kept_walk` → 21 = 0.527888 one
    step later. Reopt beam disarm cannot affect interval 21.
  - **"Stale opt_sp_"** (memory CORRECTION 1): DISPROVEN. `SeedBaselineAndArmCache(
    Incremental)` @520-537 calls `ResetIncumbentBaseline(false)`@524 (opt_sp_=-1.0,
    cache ARMED @525), then RE-SCORES the carried {pa,tl} under the NEW interval ET
    (`EvaluateSPWithPriorityVec(dag_baseline,...)`@534-535, dag_baseline from absorbed
    dag_tasks_) and `CommitIncumbent`@536. opt_sp_ is FRESH; the during-walk gate (b-i)
    IS active. `AbsorbUpdatedDAG`@789 NOT touching opt_sp_ is irrelevant — the seed
    re-scores it.
- Read the dispatcher + walk body: `Optimize_w_TL_ScratchOrIncre`@722 (backstop
  `AdoptFallbackIfUnschedulable`@784 runs AFTER the walk; binary-swaps, no re-search);
  `OptimizeIncre_w_TL`@864 → `PerformSerializedTaskQueueOptimization`@600 →
  `RunIntervalDescent`@570 → `WalkSerializedTaskQueue`@486 (each step →
  `CallOptimizerGivenTimeLimits`@268 → `UpdateRecords`@220, STRICT-SP compare-and-keep
  via `WouldBeatIncumbent`@195).
- Read `SP_Metric.cpp`/`.h` — **THE smoking gun**: SP contribution =
  `SP_Func(miss,thr)·weight·perf_coefficient`. `SP_Func`@31: above threshold
  `PenaltyFunc=-0.01·exp(10·|thr-miss|)`, normalized to [0,1]. For task 0 (thr=0.371116):
  SP_Func(0.435506)=≈0.941 vs SP_Func(0.30)=≈0.957 vs SP_Func(0.0)=1.0 — **near-flat
  just above threshold** (0.064 over → only −0.016). `perf_coefficient`=
  `GetPerfTerm(timePerformancePairs,time_limit)`@19-39 is MONOTONIC INCREASING in
  time_limit; higher TL → higher perf BUT higher execution (point-mass ET @ TL via
  ApplyTimeLimitsToTasksExecutionTime) → higher RT → higher miss_chance.
- **ROOT CAUSE (CORRECTION 2) = SP/constraint DECOUPLING:** the SP-max plan picks a
  HIGH time_limit (high perf, miss just over threshold → SP_Func still ≈0.94) because
  the perf gain outweighs the tiny SP_Func loss. The SCHEDULABLE alternative (lower TL
  → miss≤thr, SP_Func +0.016) cuts perf_coefficient → net SP LOSS → NOT
  SP-improving → walk's strict-SP compare-and-keep rejects it → 0 improving
  (walk_stats 21=3/0, 22=2/0) → gate never fires → backstop binary-swaps to RM-Fast.
  INCR_WCET immune (WCET deterministic → SP-max plan schedulable → backstop never fires).
- Updated memory `p128-incr-reopt-fallback-swap-down.md` + MEMORY.md index (corrected
  the wrong "stale opt_sp_" mechanism); updated `goal.md` (defect/fix-direction/
  approach/status). Fix direction REVISED: Option A (re-search-on-backstop-fail,
  targeted) vs Option C (constraint-aware compare-and-keep, in-search, legacy
  byte-identical). This is a design decision — escalating to user per coding rules.
- Next: await user fix-direction decision (A vs C), then TDD RED → GREEN → verify.

## 2026-08-08 (CORRECTION 3 — BF disproves "task 0 unschedulable"; task 0 NOT TL-optimizable)

- User pushback: "fallback should never trigger unless ALL challengers incl. initial
  fail the important-task check; chances should be low — is this the case?"
- DECISIVE test — BF arm (exhaustive PA + P1.27 in-search gate) on taskset_2:
    int   INCR_Reopt_10   BF
    16    0.527888         0.931108
    21    0.527888         0.931108
    22    0.527888         0.944853
    38    0.527888         0.908144
    41    0.527888         0.908144
    44    0.527888         0.908144
    52    0.527888         0.903531
  → a SCHEDULABLE high-SP plan (0.90–0.93) EXISTS at every collapse interval. The
  backstop firing does NOT mean all challengers failed. INCR's walk MISSES the plan
  and binary-swaps to RM-Fast. CONFIRMED BUG (NOT "task 0 genuinely unschedulable").
- PREMISE ERROR FOUND: task 0 (id:0, task_1, important, thr 0.371116) has NO
  performance_records — those belong to task 1 (id:1, task_2, mu 12.8, NOT important).
  CORRECTION 2 misattributed task 1's records to task 0. Task 0's
  timePerformancePairs is EMPTY → RecordCloseTimeLimitOptions pushes {-1} sentinel
  (OptimizeSP_TL_Incre.cpp:102-104) → WalkOneTaskWithTimeLimitOptions returns
  immediately (:654). Task 0's TL is FIXED (full ET dist); the walk CANNOT change it.
  → CORRECTION 2's "lower task 0's TL restores schedulability" mechanism is IMPOSSIBLE.
- CORRECTED mechanism (PA-space, NOT TL): the schedulable plan is reached via
  PRIORITY ASSIGNMENT (give task 0 enough priority → less interference → miss ≤ thr).
  BF's exhaustive PA finds it; INCR's greedy incremental PA walk + strict-SP
  compare-and-keep (WouldBeatIncumbent:195 / IsBetterTimeLimitOption:118) does NOT:
  - SP-max plan keeps task 0 at a priority giving miss 0.435 > 0.371 (the 0.93
    schedulable PA is SP-lower — giving task 0 priority sacrifices other tasks' SP —
    so strict-SP never adopts it) → 0 improving → during-walk gate never fires.
  - Post-hoc backstop AdoptFallbackIfUnschedulable (:784/:1106) catches the
    unschedulable SP-max incumbent → binary-swaps to RM-Fast (0.527888) instead of
    the 0.93 schedulable plan.
- This is the EXACT INCR analogue of P1.27 (cbdde63b fixed BF to keep best schedulable
  in-search; INCR never fixed). High-level root cause HOLDS (strict-SP keeps
  unschedulable SP-max; backstop binary-swaps to RM-Fast not best-schedulable); only
  the mechanism was wrong (PA-space, not task-0-TL).
- Fix unchanged in spirit: make INCR keep the best SCHEDULABLE plan (mirror P1.27),
  not binary-swap to RM-Fast. Option A (re-search on backstop fail — robust, finds the
  0.93 plan) vs C (constraint-aware compare-and-keep — cheaper, but greedy walk may
  not reach 0.93). Decision pending user.
- Records updated: dev_log (this), memory topic file. MEMORY.md one-liner pending.

## 2026-08-08 (fix implemented — fallback-seed on unschedulable incumbent)

- IMPLEMENTATION LANDED (started last session, verified + finalized this session),
  exactly per "Fix direction — DECIDED (user)". One guarded branch in
  `SeedBaselineAndArmCache`'s Incremental branch (`OptimizeSP_TL_Incre.cpp:536-549`):
  after re-scoring the carried `{opt_pa_, starting_time_limits}` under the absorbed
  DAG (`EvaluateSPWithPriorityVec` `:534-535`), guard
  `enable_fallback_use_ && HasSafeFallback() &&
  !ImportantTasksMeetThresholds(dag_baseline, sp_parameters_, opt_pa_)` (3-arg
  TL-pre-baked overload — `dag_baseline` has TLs baked). On false →
  `AdoptSafeFallbackAsIncumbent()` (re-scores `safe_fallback_` under the absorbed
  dag; `CommitIncumbent` sets `opt_pa_`/`opt_sp_`/`res_opt_`), then refresh
  `starting_time_limits = ReconstructTimeLimitVecFromResOpt()` +
  `current_config_sp = opt_sp_`. Else → old `CommitIncumbent(opt_pa_, …)` unchanged.
- Cache-safety verified by code walk: `ResetIncumbentBaseline(false)` default-
  constructs the cache (`:1217`) → no champion → the fallback path's gated
  `rta_cache_.Evaluate` (inside `CommitIncumbent`) routes to `Initialize`
  (full compute; RTA_Cache.h: "no champion -> Initialize") — NO `|diff|>1` throw.
  Same empty-cache path the carried-incumbent else-branch already used.
- `opt_sp_` freshness: `AdoptSafeFallbackAsIncumbent` re-scores the fallback under
  the CURRENT absorbed dag (not the worst-case-DAG cert score), so the returned
  `current_config_sp` walk baseline reflects this interval's ET.
- Bit-identity: no `is_important` → 3-arg overload vacuously true → guard false →
  else branch = byte-identical old path; `enable_fallback_use_==false` (measurement
  arm) untouched. `WouldBeatIncumbent`/`UpdateRecords`/reopt beam UNCHANGED;
  post-hoc `AdoptFallbackIfUnschedulable`@`:784` KEPT as safety net.
- TDD: `SeedBaselineAndArmCache_SeedsFromFallbackWhenCarriedIncumbentUnschedulable`
  (`tests/testIncreOpt_w_TL.cpp`, fixture `CompareAndKeepSynthetic`): tasks[0] NOT
  important (interference), tasks[1] important, ddl 700 — TL=1000 misses, TL=400
  schedulable (fixture preconditions `ASSERT_FALSE`/`ASSERT_TRUE` on the 4-arg
  gate). Stage schedulable safe fallback (`CommitIncumbent` with `tl_safe` then
  `SetSafeFallbackForTest`), then commit the UNSCHEDULABLE carried incumbent
  (`tl_carried` TL[task0]=1000), then `SeedBaselineAndArmCache(2, tl_carried,
  Incremental)`. Asserts: seeded incumbent TL = 400 (not the carried 1000);
  `starting_time_limits` tracks it; the seeded incumbent passes the 4-arg gate.
- RED by inspection (conclusive): HEAD's Incremental branch unconditionally
  `CommitIncumbent(opt_pa_, sp, tl_carried)` → `id2time_limit[task0]` stays 1000
  → first EXPECT fails; seeded incumbent stays unsched → gate EXPECT fails. Nothing
  at HEAD calls `AdoptSafeFallbackAsIncumbent` from `SeedBaselineAndArmCache`.
  (Empirical RED run pending: Bash classifier transiently down; will re-run by
  restoring HEAD on the .cpp only.)
- GREEN verified: full suite 17/17 ctest PASS with the fix (incl. the new test and
  the single-test run `--gtest_filter=*SeedBaselineAndArmCache*`).
- NOT committed (`git add` only, pending user review). DEFERRED: e2e comparison
  re-run to confirm the 0.527888 collapse intervals now reach ≈0.90–0.93.

## 2026-08-08 (records sync to CORRECTION 3)

- User asked whether the task records capture the latest finding — i.e. that we have no
  good mechanism to check whether a higher-SP challenger is actually schedulable.
  Answer: YES — captured as the root cause (`WouldBeatIncumbent` strict-SP, never checks
  schedulability; only post-hoc backstop enforces the constraint → binary-swap to
  RM-Fast) and as fix Option C. But two records were STALE on CORRECTION 2's disproven
  TL mechanism. Fixed:
  - `goal.md`: status "CORRECTED 2×"→"3×"; task-0 description (NOT TL-optimizable, empty
    `timePerformancePairs`, fixed TL); CORRECTION 3 banner on the "SP/constraint
    decoupling" section (high-level holds, TL mechanism superseded by PA-space);
    approach step 3.
  - `MEMORY.md` one-liner: rewrote to CORRECTION 3 (PA-space, BF shows schedulable plan
    exists, strict-SP never checks schedulability, Option A vs C pending).
  - dev_log + `p128-...md` topic file already had CORRECTION 3 (no change).

## 2026-08-08 (follow-up: TasksSP refactor + seed-check restructure — user request)

- Fix commit landed at HEAD `393aab0a` (user committed). User added two follow-up
  requirements for `SeedBaselineAndArmCache`:
  1. If `enable_fallback_use_`, check the initial solution's schedulability NO MATTER
     whether `HasSafeFallback()` is true (today the `HasSafeFallback()` conjunct
     short-circuits the gate).
  2. Efficiency: `EvaluateSPWithPriorityVec(dag_baseline, ...)` already computes all
     tasks' RTAs; the subsequent 3-arg `ImportantTasksMeetThresholds(dag_baseline,...)`
     re-derives them from scratch. Reuse them. User's suggested route: extend the
     return type of `ObtainSP_DAG` with a bool (important tasks schedulable), via a
     new `struct TasksSP { double sp_value; bool important_tasks_schedulable; }`
     (parallels `PriorityOptResult` in OptimizeSP_Incre.h, but at the DAG-SP level).
     Order DECIDED by user: refactor FIRST (introduce struct + plumb), THEN the
     seed-site implementation.
- Equivalence proof (checked against code): the flag folded into
  `ObtainSP_TaskSet` (per-task `ddl_miss_chance` vs `thresholds_node.at(id)` for
  `is_important` tasks, on the PA-prioritized set) is BIT-IDENTICAL to the 3-arg
  `ImportantTasksMeetThresholds(dag_baseline, sp, pa)` verdict (same prioritized
  set, same `ProbabilisticRTA_TaskSet`, same thresholds; all overloads reduce to
  one shared core, P1.29). So R4 changes no observable behavior; it removes one
  fresh RTA eval per incremental interval.
- Plan (atomic builds, green tests after each, `git add` only):
  - R1: `TasksSP` + `ObtainSP_TaskSet`/`ObtainSP_TaskSet_And_TimeLimits` return it.
  - R2: both `ObtainSP_DAG` overloads return it (node flag; chains carry no
    important-task constraint).
  - R3: `EvaluateSPWithPriorityVec` returns it (P1.14 cancel → `{INT_MIN, true}`,
    callers already discard INT_MIN).
  - R4: `SeedBaselineAndArmCache` reads `.important_tasks_schedulable`; guard
    restructure so the check is independent of `HasSafeFallback()`; drop the
    separate gate call. New test locks the no-safe-fallback semantics.
  - R5: release speed test + records/memory sync.
- Deferred API-uniformity candidate (NOT in scope): `ObtainSP_DAG_From_Dists` /
  `ObtainSP_Full_From_NodeRTAs` still return double (cache path; the in-walk gate
  there already reuses cached RTAs via the 5-arg overload, so no waste to remove).
  Same for the P1.29 BF in-search gate (`OptimizeSP_BF.cpp:35-37`), which still
  re-derives RTAs per would-beat leaf via the 3-arg overload — could now read
  `sp_eval.important_tasks_schedulable` instead; left out (user scoped this task
  to `SeedBaselineAndArmCache`).

## 2026-08-09 (R1–R4 landed; git add-only, pending user review/commit)

- R1 DONE: `struct TasksSP { double sp_value = INT_MIN; bool
  important_tasks_schedulable = true; }` in `SP_Metric.h`; `ObtainSP_TaskSet` +
  `ObtainSP_TaskSet_And_TimeLimits` return it. The flag folds into the EXISTING
  per-task SP loop (`is_important && ddl_miss_chance > threshold` — same
  `thresholds_node.at(id)` value, hoisted to a local so the map lookup is also
  not duplicated). Zero extra RTA eval; `sp_value` arithmetic order unchanged
  (bit-identical). New TDD test `ObtainSP_TaskSet_ReportsImportantTaskSchedulability`
  (vacuous-true default; importance alone doesn't flip it; tightened threshold
  flips it; `sp_value` invariant under `is_important`). Callers updated:
  SimulationOrchestrator.cpp:608/866, testSP TaskSet sites. 17/17 ctest.
- R2 DONE: both `ObtainSP_DAG` overloads return `TasksSP` (chains add to
  `.sp_value` only — the important-task constraint is node-level, matching the
  gate's semantics). New test `ObtainSP_DAG_ReportsImportantTaskSchedulability`.
  Callers: OptimizeSP_Base.cpp:201, OptimizeSP_BF.cpp:58, testSP DAG sites,
  testOptimizeIncrePA.cpp:341. 17/17 ctest.
- R3 DONE: `EvaluateSPWithPriorityVec` returns `TasksSP`; P1.14 cancel path
  returns `{INT_MIN, true}` (callers already discard INT_MIN). Call sites →
  `.sp_value`: OptimizeSP_BF.cpp:25, OptimizeSP_Incre.cpp:171/354/411,
  OptimizeSP_TL_Incre.cpp:535/1002/1045/1111/1222, tests (testOptimizePA:289,
  testRTA:646/700, testOptimizeIncrePA:412/797, testIncreOpt_w_TL:2058/2082/
  2097/2138, testSP:592-652). 17/17 ctest.
- R4 DONE: `SeedBaselineAndArmCache` Incremental branch now reads
  `baseline_eval.important_tasks_schedulable` (requirement 2 — the separate
  3-arg `ImportantTasksMeetThresholds` fresh-RTA call is GONE; one RTA eval saved
  per incremental interval) and the guard is `enable_fallback_use_ &&
  !important_tasks_schedulable && HasSafeFallback()` (requirement 1 — the audit
  no longer short-circuits on `HasSafeFallback()`; it is structurally part of the
  SP eval, so it runs on every fall-back-enabled interval). Truth-table identical
  to HEAD (flag ≡ gate verdict, bit-identical by construction). New
  characterization test `SeedBaselineAndArmCache_KeepsCarriedIncumbentWhen-
  UnschedulableWithoutFallback` (checks, nothing to adopt → carried incumbent
  kept). The prior P1.28 test `..._SeedsFromFallbackWhenCarriedIncumbent-
  Unschedulable` stays GREEN via the flag path. 17/17 ctest.
- NOTE for committer: R3 and R4 both touch `OptimizeSP_TL_Incre.cpp` /
  `tests/testIncreOpt_w_TL.cpp`, so the staged diff in those two files combines
  the `.sp_value` call sites (R3) and the guard restructure (R4). To commit
  separately: `git restore --staged` those two, then `git add -p`.
- Verify: `cmake --build build_test --target check.SP_OPT -j5` → 17/17 after EACH
  stage. Release speed test pending.

## 2026-08-09 (R5 — verify + records sync)

- Functional tests: `cmake --build build_test --target check.SP_OPT -j5` → **17/17
  PASS** (re-confirmed after R1–R4; same as after each stage).
- Release speed test: `cmake --build release -j5 && ./tests/RunSpeedTest` →
  **OVERALL VERDICT: PASS**.
  - INCR_Reopt_1: 0.025692 s/interval (threshold 0.1) — PASS.
  - INCR_Reopt_10: 0.014604 s/interval (threshold 0.1) — PASS.
  - The `TasksSP` refactor (flag folded into the existing per-task SP loop, zero extra
    RTA eval) adds no measurable overhead — both arms well under the 0.1 s/int threshold
    and consistent with the pre-refactor P1.29/P2.12 baselines (0.042/0.013 s/int).
- Records synced to the committed + R1–R4 state:
  - `memory/p128-incr-reopt-fallback-swap-down.md`: rewrote Status (COMMITTED
    `393aab0a` + R1–R4 landed), Root cause (CORRECTION 3 — task0 NOT TL-optimizable,
    PA-space), Defect (incremental seed unschedulable, armed-but-idle b-i gate), Fix
    (committed fallback-seed), and a new R1–R4 TasksSP-refactor section (replaces the
    stale "disarmed reopt beam / fix NOT yet implemented" framing).
  - `MEMORY.md` one-liner: updated to COMMITTED `393aab0a` + R1–R4 + CORRECTION 3 +
    speed PASS.
- REMAINING (deferred): e2e comparison re-run of `taskset_2` INCR_Reopt_10 to confirm
  the formerly-collapsing intervals (16/21/38/41/44/52) now reach ≈0.90–0.93 instead of
  0.527888. To be run with the deferred P1.27/P1.29 sweep re-runs. NOT a code blocker —
  the fix + refactor are code-complete and verified (17/17 ctest + speed PASS).
- Staged for user commit (git add-only): R1–R4 source + test changes (8 source files,
  5 test files). NOTE for committer (carried from R3/R4): `OptimizeSP_TL_Incre.cpp` +
  `tests/testIncreOpt_w_TL.cpp` combine the R3 `.sp_value` call-site changes and the R4
  guard restructure; to split into separate commits, `git restore --staged` those two
  then `git add -p`.



## 2026-08-09 (safe-default follow-up — user request)

User: "ObtainSP_TaskSet, during initialization, it should be unschedulable by default
for safety because you have `if (BFSharedBudgetCancelled()) return result;`" + "there
are other default configurations in other code files, do the same".

- **Defect:** `TasksSP::important_tasks_schedulable` defaulted to `true` in 3 places, so
  a mid-eval `BFSharedBudgetCancelled()` early return (ObtainSP_TaskSet SP_Metric.cpp:67,
  ObtainSP_DAG:114, EvaluateSPWithPriorityVec OptimizeSP_Base.cpp:192/208) would report a
  false "all clear" on an INCOMPLETE eval — the per-task schedulability check never
  finished. Same hazard class as P1.29's "UpdateSP budget-timeout returns FALSE (was
  true)".
- **Fix (Pattern C — mirror P1.29):**
  - `SP_Metric.h:34`: struct default flipped `true`→`false` (comment updated).
  - `SP_Metric.cpp` ObtainSP_TaskSet: rewritten to declare `TasksSP result{0.0, false}`,
    track a `bool any_important_miss` accumulator (set true on the important-task miss),
    and set `result.important_tasks_schedulable = !any_important_miss` ONLY on full loop
    completion. The cancel bail (`return result` at the loop top) keeps the `false`
    default; the normal no-miss completion path stays vacuous-true. Bit-identical on all
    consuming seed-audit paths (BF budget inactive there → bails don't fire → flag
    reflects the actual completed eval).
  - `OptimizeSP_Base.cpp` EvaluateSPWithPriorityVec: both `return {INT_MIN, true}`
    sentinels → `return {INT_MIN, false}` (the pre- and post-ObtainSP_DAG cancel checks).
  - `ObtainSP_DAG` NOT edited: its cancel bail (SP_Metric.cpp:114) inherits `result` from
    ObtainSP_TaskSet, which now returns `false` on cancel — already safe.
- **TDD:** `ObtainSP_TaskSet_BudgetCancelIsUnschedulable` (tests/testSP.cpp) — forces
  `TIME_LIMIT=0` inside a `BFDLSharedBudget(now)` scope, task1 important+below-threshold
  (a completed eval reports `true`). Confirmed RED (bug: cancel returned `true`) → GREEN
  after the Pattern C fix.
- **Verify:** 17/17 ctest PASS; release RunSpeedTest **PASS** (INCR_Reopt_1 0.030423
  s/int, INCR_Reopt_10 0.021301 s/int; no regression).
- **Staged (git add-only):** SP_Metric.h, SP_Metric.cpp, OptimizeSP_Base.cpp, testSP.cpp.

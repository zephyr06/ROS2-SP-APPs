# P1.28 — INCR_Reopt_10 interval SP collapses to 0.527888 (RM-Fast fallback swap-down)

**Priority:** P1 (correctness investigation; INCR analogue of the P1.27 BF bug)
**Status:** ROOT CAUSE FOUND + CORRECTED 3× (2026-08-08). FIX COMMITTED at
`393aab0a` (fallback-seed on unschedulable incumbent in `SeedBaselineAndArmCache`,
Incremental branch). FOLLOW-UP 2026-08-09 (user): (1) audit the initial solution's
schedulability whenever `enable_fallback_use_`, independent of `HasSafeFallback()`;
(2) stop re-computing RTAs for that audit — `struct TasksSP {double sp_value; bool
important_tasks_schedulable;}` now rides the SP eval (`ObtainSP_TaskSet`/
`ObtainSP_DAG`/`EvaluateSPWithPriorityVec` all return it; flag folded into the
existing per-task loop, zero extra RTA eval). R1–R4 landed (git add-only; 17/17
ctest after each stage). R5 DONE 2026-08-09: release RunSpeedTest PASS
(0.026/0.015 s/int); records/memory synced. DEFERRED: e2e comparison re-run (collapse
intervals expect ≈0.90–0.93).
**LATEST:** CORRECTION 3 (dev_log) — task 0 is NOT TL-optimizable (no performance_records); the TL/perf_coefficient mechanism in the "CORRECTION 2" section below is DISPROVEN. The schedulable plan is reached via PRIORITY ASSIGNMENT. High-level root cause holds. Fix direction now DECIDED (see "Fix direction — DECIDED" below).
**Cross-links:** P1.27 (`cbdde63b` — same bug shape, BF path only), P0.7
(fall-back mechanism, trigger b-ii backstop), P0.10 (BF fallback gate), P2.17/P2.18
(gate-consistency family), memory `p127-bf-worse-than-incr.md`, `p18-incr-wcet-outperforms-incr.md`.

## The anomaly (user, 2026-08-08)

In the comparison run
`simulation_experiments/optimizer_comparison/runs/compare_against_bf_run_test_dur600_interval10_seed1000_tasks4/sim/tasks4_dur600_interval10_seed1000/taskset_2/`:

- `INCR_Reopt_10/INCR_Reopt_10/interval_sp_metrics.txt` collapses to **0.527888** at
  intervals 16, 17, 18, 19, 21, 22, 38, 39, 41, 44, 52 — aggregate Mean_SP = **0.855923**.
- `INCR_WCET/INCR_WCET/interval_sp_metrics.txt` stays 0.885–0.954 across ALL 60
  intervals — aggregate Mean_SP = **0.920009**.

A schedulable high-SP plan (≥0.931) demonstrably EXISTS at every collapsing interval
(INCR_WCET achieves it on the identical taskset), so 0.527888 is unreasonable. Debug it.

## Root cause (verified)

The 0.527888 value is the **RM-Fast group-locked fallback** SP. It is adopted by the
P0.7 trigger **(b-ii) post-walk backstop** `AdoptFallbackIfUnschedulable`
(`OptimizeSP_TL_Incre.cpp:1106`), confirmed 1:1 against
`interval_fallback_log.txt`:

| interval | sp_metric | backstop_verdict   | culprit_task | miss_chance | threshold |
|----------|-----------|--------------------|--------------|-------------|-----------|
| 16       | 0.527888  | adopted_fallback   | 0            | 0.509599    | 0.371116  |
| 21       | 0.527888  | adopted_fallback   | 0            | 0.435506    | 0.371116  |
| 38       | 0.527888  | adopted_fallback   | 0            | 0.47825     | 0.371116  |
| 41       | 0.527888  | adopted_fallback   | 0            | 0.433624    | 0.371116  |
| 44       | 0.527888  | adopted_fallback   | 0            | 0.427202    | 0.371116  |
| 52       | 0.527888  | adopted_fallback   | 0            | 0.695974    | 0.371116  |

Culprit = **task 0** (`task_1`), `important: true`, `sp_threshold: 0.3711164937976443`
(`taskset_characteristics_interval_0.yaml`). Task 0 has a wide ET (mu=133.5, sigma=44.7,
max=199.5, period=deadline=1000) but is **NOT TL-optimizable** (its
`timePerformancePairs` is EMPTY — CORRECTION 3; the records CORRECTION 2 misattributed
to task 0 belong to task 1). So the walk CANNOT change task 0's TL; its full ET dist is
fixed. At these intervals the committed SP-max plan gives task 0 a priority under which
its `ddl_miss_chance` exceeds the threshold → the backstop rejects it and swaps the WHOLE
plan DOWN to RM-Fast (0.527888).

**INCR_WCET never trips the backstop** (all 60 intervals `kept_walk`) because it sets
`use_wcet_execution_time=true` (`SimulationOrchestrator.cpp:380-387`) → the plan is
evaluated under the deterministic WCET, which is trivially schedulable for task 0.
So WCET's 0.92 is not "a better search" — it is the same `Optimize_w_TL_ScratchOrIncre`
path with a stricter (scheduling-friendlier) ET surface. The Reopt arm evaluates under
the real ET distribution, where the SP-max plan can miss task 0's deadline.

### The defect (why the backstop swaps DOWN instead of finding the schedulable ≥0.9 plan)

> **RE-INVESTIGATION 2026-08-08 (CORRECTION 2).** The original framing below ("INCR
> analogue of P1.27; disarmed reopt beam commits SP-max unchecked") is WRONG for the
> collapsing intervals. The collapses are INCREMENTAL (e.g. reopt@20=0.954072→21=0.527888
> one step later), and the incremental path is NOT disarmed. The code-verified root
> cause is the **SP-objective / schedulability-constraint DECOUPLING**, described in the
> next section. The original text is retained only as the disproven hypothesis.

**Original (disproven) hypothesis — disarmed reopt beam.** INCR analogue of P1.27: the
reopt from-scratch beam runs DISARMED (`ResetIncumbentBaseline(true)`@544 clears the
cache before `CallOptimizerGivenTimeLimits(from_scratch=true)`@545; the
`from_scratch` branch's `UpdateRecords` skips the `rta_cache_active_` gate @239) →
SP-max leaf committed unchecked → backstop@784 swaps to RM-Fast. **This is a SEPARATE
latent defect (tracked as P1.29) and does NOT explain the incremental-interval
collapses** — it would only matter at reopt intervals (0/10/20/30/40/50), all of which
are `kept_walk`/high-SP.

#### Verified root cause — the SP/constraint decoupling (CORRECTION 2)

> **CORRECTION 3 (2026-08-08, see dev_log) supersedes the MECHANISM below.** The
> TL/perf_coefficient framing here is DISPROVEN: task 0 is NOT TL-optimizable (empty
> `timePerformancePairs` → fixed TL), so "lower task 0's TL" is impossible. The
> schedulable plan is reached via **PRIORITY ASSIGNMENT** (give task 0 enough priority →
> less interference → miss ≤ thr). The HIGH-LEVEL root cause below STILL HOLDS: the
> walk's `WouldBeatIncumbent` is strict-SP-only — it never checks whether a challenger is
> schedulable, so the SP-lowering-but-schedulable 0.93 PA is rejected in favor of the
> SP-max-but-unschedulable incumbent, and only the post-hoc backstop enforces the
> constraint (binary-swap to RM-Fast). The `SP_Func` near-flatness may still explain WHY
> the schedulable PA is SP-lower, but it is not the primary mechanism. Fix direction
> unchanged (Option A vs C).

The SP-max plan is genuinely SP-optimal **even when unschedulable**, because the SP
metric barely penalizes a slight threshold violation. `SP_Func(miss, thr)`
(`SP_Metric.h:31`):

- below threshold: `RewardFunc = log((thr − miss) + 1)`;
- above threshold: `PenaltyFunc = −0.01·exp(10·|thr − miss|)`;
- normalized to [0,1] over [PenaltyFunc(1,thr), RewardFunc(0,thr)].

For task 0 (thr=0.371116): SP_Func(0.435506 [int-21 miss]) ≈ **0.941** vs
SP_Func(0.30 [schedulable]) ≈ 0.957 vs SP_Func(0.0) ≈ 1.0 — **near-flat just above
threshold** (0.064 over → only −0.016 from the schedulable point).

The SP contribution is `SP_Func(miss,thr)·weight·perf_coefficient`, and
`perf_coefficient = GetPerfTerm(timePerformancePairs, time_limit)` is **monotonic
increasing in time_limit** (`SP_Metric.cpp:19-39`, floor lookup). A higher time_limit
→ higher perf BUT (point-mass ET at `time_limit` via `ApplyTimeLimitsToTasksExecutionTime`)
higher execution → higher RT → higher miss_chance. So the SP-max plan picks a HIGH
time_limit (high perf, miss just over threshold → SP_Func still ≈0.94) because the
perf gain outweighs the tiny SP_Func loss. The SCHEDULABLE alternative (lower
time_limit → miss≤thr, SP_Func 0.941→0.957 = +0.016) cuts perf_coefficient → **net SP
LOSS** → it is NOT SP-improving.

Incremental-interval flow (e.g. interval 21):

1. Reopt@20 committed 0.954072 (schedulable at 20's friendlier ET).
2. `OptimizeIncre_w_TL`@864 → `AbsorbUpdatedDAG`@789 loads 21's worse ET (task 0
   mu 174.8→230.6, confirmed in `taskset_characteristics_interval_{20,21}.yaml`).
3. `SeedBaselineAndArmCache(Incremental)`@520-537: `ResetIncumbentBaseline(false)`@524
   (sets `opt_sp_=-1.0`, arms cache `rta_cache_active_=true`@525), then RE-SCORES the
   carried {pa,tl} under 21's new ET (`current_config_sp =
   EvaluateSPWithPriorityVec(dag_baseline,...)`@534-535, `dag_baseline` from the
   absorbed `dag_tasks_`) and `CommitIncumbent`@536. **`opt_sp_` is FRESH (reflects
   the degradation), NOT stale; the during-walk gate (b-i) IS armed.** (This disproves
   the "stale opt_sp_" hypothesis.)
4. The re-scored baseline is SP-optimal but UNSCHEDULABLE (task 0 miss 0.435 > 0.371).
5. Walk (`WalkSerializedTaskQueue`@486 → `OptimizeOneTaskWithTimeLimit` /
   `OptimizeIncreSingleTask` → `CallOptimizerGivenTimeLimits`@268 → `UpdateRecords`@220):
   each challenger scored under 21's ET; `WouldBeatIncumbent`@195 is STRICT-SP-greater
   (tie-break lower total TL). Any schedulable challenger (lower task-0 TL) has LOWER
   SP (perf loss > SP_Func gain) → not "improving" → rejected by compare-and-keep →
   **0 improving** (matches `interval_walk_stats.txt`: 21=3/0, 22=2/0).
6. During-walk gate (b-i) only runs on WOULD-BEAT challengers (`should_update`@238);
   0 would-beat → gate never fires → `during_walk_reject_count=0` at every collapse
   interval.
7. Post-hoc backstop `AdoptFallbackIfUnschedulable`@784/@1106 catches the unschedulable
   SP-max incumbent → **binary-swaps to RM-Fast** (0.527888); no re-search for the best
   SCHEDULABLE plan.

The sticky 0.527888 across 16–19 (`kept_walk`) = the RM-Fast incumbent persisting (no
SP-improving challenger beats it; it is schedulable so the backstop keeps it). Recovery
at 23/42/45/53: incumbent already = RM-Fast (low 0.527888 bar) → walk finds ~8
improving (schedulable, higher-SP) challengers → recovers to ~0.9.

**Root cause = the SP objective does not encode the hard important-task schedulability
constraint** (SP_Func near-flat near threshold + perf_coefficient rewarding the
unschedulable high-TL choice). The walk's SP-strict compare-and-keep never accepts an
SP-lowering-but-schedulable move; only the crude post-hoc backstop enforces the
constraint, dropping all the way to RM-Fast.

## Fix direction — DECIDED (user, 2026-08-08): fallback-seed on unschedulable incumbent

**User principle:** "the schedulers have to start with a solution that is schedulable.
If the initial solution from last interval's optimization configurations is not
schedulable at the new interval, start the optimization process with the fall-back
solution."

**The chosen fix (distinct from A and C below):** do NOT add a fresh re-search (A) and
do NOT change `WouldBeatIncumbent`/`UpdateRecords` (C). Instead, change only the
**seed** of the incremental walk:

- In `SeedBaselineAndArmCache`'s **Incremental** branch (`OptimizeSP_TL_Incre.cpp:520-537`),
  after re-scoring the carried `{opt_pa_, starting_time_limits}` under the new interval's
  ET (`current_config_sp = EvaluateSPWithPriorityVec(dag_baseline, …)` `:534-535`), and
  BEFORE `CommitIncumbent` (`:536`), run the hard feasibility check
  `ImportantTasksMeetThresholds(dag_baseline, sp_parameters_, opt_pa_)` (3-arg, TL-pre-baked
  dag overload — `dag_baseline` already has TLs baked via `UpdateExtDistBasedOnTimeLimit`).
- If it returns **false** (the carried incumbent is unschedulable under this interval's
  drifted ET), seed the walk from the **safe fallback** (`safe_fallback_`'s `{priority_vec,
  tl}`) instead: re-score the fallback triple under `dag_tasks_` and `CommitIncumbent` with
  it. The safe fallback is the P0.6 static solution (DM + important-first group lock,
  certified schedulable on the cross-interval worst-case DAG via `BuildWorstCaseDagAcrossIntervals`
  → schedulable under every interval's ET, which is dominated by the worst case).
- Otherwise (incumbent still schedulable) commit the carried incumbent as today — no change.

**Why it works (the key insight that makes it correct + minimal):** on the incremental
path the cache IS armed (`rta_cache_active_ = true` `:525`, right after
`ResetIncumbentBaseline(false)`), so the existing during-walk gate **(b-i)**
(`UpdateRecords:238-258`, runs on SP-improving challengers when `rta_cache_active_`) is
ACTIVE. Today the bug is that the walk starts from the unschedulable SP-max incumbent,
which has **zero** SP-improving (hence zero gate-tested) moves toward the schedulable
plan → the gate never engages → walk ends unschedulable → post-hoc backstop binary-swaps
to RM-Fast (0.527888). Seeding from the schedulable safe fallback instead means the walk
**climbs SP from a schedulable base**, and every adopted (SP-improving) challenger must
pass the b-i schedulability gate → unschedulable SP-improvements are rejected → the walk
converges to the best schedulable plan reachable by greedy coordinate descent (≈0.90–0.93;
BF reaches 0.90–0.93 exhaustively). Never worse than the safe fallback (0.527888); the
walk only adopts SP-improving+gate-passing moves, so result ≥ seed SP. Strict improvement
over today's 0.527888 collapse; no regression path.

**Scope / blast radius:** one guarded branch in `SeedBaselineAndArmCache` (incremental
path only). `WouldBeatIncumbent`, `UpdateRecords`, the reopt beam, and
`AdoptFallbackIfUnschedulable` are all UNCHANGED. The reopt (from-scratch) path does NOT
need this — P1.29 already hard-prunes unschedulable leaves in `OptimizeFromScratch`, and
the reopt backstop handles the emptied-beam case.

**Legacy bit-identical:** no `is_important` tasks → `ImportantTasksMeetThresholds` returns
true vacuously → the seed check never triggers → `CommitIncumbent(opt_pa_, …)` exactly as
today → byte-identical. (Same vacuous-gate argument as P1.27/P1.29.)

**Post-hoc `AdoptFallbackIfUnschedulable` KEPT** as the final safety net (the "no
schedulable plan exists at all" case), exactly as P1.27/P1.29 kept their post-hoc gates.

### Options considered but NOT chosen (retained for the record)

- **(A) Re-search on backstop fail:** run a constrained from-scratch beam (P1.29-gated,
  keeps only schedulable leaves) when the backstop finds the incumbent unschedulable.
  More machinery + one extra beam per collapse interval. NOT chosen — the fallback-seed
  reuses the EXISTING walk + EXISTING b-i gate, no new search.
- **(C) Constraint-aware compare-and-keep (in-search):** make `WouldBeatIncumbent`/
  `UpdateRecords` treat schedulability as a hard constraint overriding SP (a schedulable
  challenger beats an unschedulable incumbent regardless of SP). Touches the core walk
  loop; also risks the greedy coordinate descent not actually reaching the 0.93 plan even
  when allowed to accept SP-lowering-but-schedulable moves. NOT chosen — larger blast
  radius, and the fallback-seed achieves the constraint enforcement via the existing gate
  without rewriting the adoption rule.

## Approach

1. ✅ Localize: gap is `taskset_2`, intervals 16/21/38/41/44/52, value 0.527888.
2. ✅ Mechanism: P0.7 trigger b-ii backstop swaps to RM-Fast; INCR_WCET immune via
   `use_wcet_execution_time`.
3. ✅ Defect (CORRECTION 2, mechanism corrected by CORRECTION 3): SP/constraint
   decoupling — strict-SP `WouldBeatIncumbent` never checks schedulability, so the
   SP-max-but-unschedulable incumbent (task 0 at a priority giving miss 0.435 > 0.371)
   beats the SP-lowering-but-schedulable 0.93 PA → 0 improving → backstop binary-swaps
   to RM-Fast. CORRECTION 3: task 0 NOT TL-optimizable; schedulable plan is PA-space.
4. ✅ Fix-direction DECIDED (user, 2026-08-08): fallback-seed on unschedulable incumbent
   (incremental branch of `SeedBaselineAndArmCache:520-537`; reuses the existing armed
   cache `:525` + existing b-i during-walk gate `UpdateRecords:238`; `WouldBeatIncumbent`
   and the backstop unchanged).
5. ✅ TDD RED: `SeedBaselineAndArmCache_SeedsFromFallbackWhenCarriedIncumbentUnschedulable`
   (`tests/testIncreOpt_w_TL.cpp`) — staged schedulable safe fallback (TL=400) vs staged
   unschedulable carried incumbent (TL=1000); asserts the seed swaps to the fallback
   (TL→400, `starting_time_limits` tracks, seeded incumbent passes the gate). RED
   conclusive by inspection (HEAD's Incremental branch unconditionally commits the
   carried incumbent → `id2time_limit` stays 1000 → first EXPECT fails). (Simplified
   single-dag fixture via `SetSafeFallbackForTest` rather than a full two-interval DAG.)
6. ✅ GREEN: the guarded branch landed in `SeedBaselineAndArmCache` Incremental branch
   (`:536-549`) — on unschedulable carried incumbent →
   `AdoptSafeFallbackAsIncumbent()` (commits the `safe_fallback_` triple re-scored
   under the absorbed dag) + refresh `starting_time_limits`/`current_config_sp`.
   **R4 restructured the guard** to `enable_fallback_use_ &&
   !baseline_eval.important_tasks_schedulable && HasSafeFallback()` (reads the flag
   that now rides the SP eval; the separate 3-arg `ImportantTasksMeetThresholds`
   fresh-RTA call is GONE; the audit is `HasSafeFallback()`-independent — requirement 1).
7. ✅ Verify (code): 17/17 ctest PASS (incl. the new test + R1–R4 char tests); legacy
   bit-identical (no `is_important` → flag vacuously true → guard false → old
   `CommitIncumbent` path unchanged). ✅ R5: release RunSpeedTest PASS (0.026/0.015
   s/int — `TasksSP` adds no measurable overhead).
8. ✅ R1–R4 (2026-08-09 follow-up): `struct TasksSP{sp_value,
   important_tasks_schedulable}` plumbed through `ObtainSP_TaskSet`/`ObtainSP_DAG`/
   `EvaluateSPWithPriorityVec` (flag folded into the existing per-task SP loop, zero
   extra RTA eval); `SeedBaselineAndArmCache` reads `.important_tasks_schedulable`.
   17/17 ctest after each stage; git add-only (pending user commit).
   ⬜ DEFERRED (e2e): re-run taskset_2 INCR_Reopt_10 → no 0.527888 collapses at
   16/21/38/41/44/52 (expect a schedulable ≥0.9 plan adopted instead) — to be run with
   the deferred P1.27/P1.29 sweep re-runs.

## Files

- `simulation_experiments/.../taskset_2/{INCR_Reopt_10,INCR_WCET}/INCR_*/interval_sp_metrics.txt`
  + `interval_fallback_log.txt` + `interval_walk_stats.txt` (the evidence).
- `taskset_2/taskset_characteristics_interval_0.yaml` (task 0 = important, threshold
  0.371116, wide ET, TL-optimizable).
- `sources/Optimization/OptimizeSP_TL_Incre.cpp` — `UpdateRecords:220` (b-i gate,
  `rta_cache_active_`-gated), `CallOptimizerGivenTimeLimits:268` (`from_scratch`
  branch :275-280), `RunIntervalDescent:570` / `SeedBaselineAndArmCache:540-553`
  (disarmed beam), `AdoptFallbackIfUnschedulable:1106` (b-ii backstop),
  `Optimize_w_TL_ScratchOrIncre:722` (backstop call :784).
- `sources/Optimization/OptimizeSP_TL_BF.cpp` — the P1.27 in-search gate (reference
  fix shape).
- `sources/Safety_Performance_Metric/SP_Metric.cpp:208/245` — gate overloads.
- `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp:380-387` —
  `INCR_WCET` sets `use_wcet_execution_time=true`.

## Done when

- Fallback-seed-on-unschedulable-incumbent lands in `SeedBaselineAndArmCache`
  (incremental branch); 0.527888 collapses gone on `taskset_2` INCR_Reopt_10 (re-run
  confirms a schedulable plan ≥0.9 adopted at the formerly-collapsing intervals
  16/21/38/41/44/52, instead of RM-Fast).
- 17/17 ctest; legacy bit-identical (no `is_important`).
- Milestone to `agents/dev_log.md`; memory entry `p128-incr-reopt-fallback-swap-down.md`;
  cross-link P1.27 (note: P1.27 fix was BF-only — this completes the INCR side).
- Follow-up (2026-08-09 user requirements): `TasksSP` refactor lands (no fresh RTA
  eval for the seed audit) and the audit is `HasSafeFallback()`-independent;
  perf re-measured on release RunSpeedTest.
- Safe-default follow-up (2026-08-09 user): `TasksSP::important_tasks_schedulable`
  defaults `false` (was `true`) so a mid-eval `BFSharedBudgetCancelled()` bail cannot
  report a false "all clear" — ObtainSP_TaskSet sets it `true` ONLY on full-loop
  completion with no important miss (Pattern C, mirrors P1.29 budget-timeout-returns-
  false); both `EvaluateSPWithPriorityVec` cancel sentinels → `{INT_MIN, false}`.
  TDD `ObtainSP_TaskSet_BudgetCancelIsUnschedulable`; 17/17 ctest; release RunSpeedTest
  PASS.

## Out of scope

- `git commit` — user's standing constraint (`git add` only).
- Re-running the full comparison sweep before the fix lands.
- The WCET-vs-ET metric-surface question (INCR_WCET's 0.92 is a stricter-ET artifact,
  not a better search — documented here, not "fixed").

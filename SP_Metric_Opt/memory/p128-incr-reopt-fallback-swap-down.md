# P1.28 — INCR_Reopt_10 interval SP collapses to 0.527888 (RM-Fast fallback swap-down)

**Status:** FIX COMMITTED `393aab0a` 2026-08-08 (fallback-seed on unschedulable
incumbent in `SeedBaselineAndArmCache` Incremental branch). FOLLOW-UP R1–R4 LANDED
2026-08-09 (git add-only, pending user commit): `TasksSP` refactor —
`struct TasksSP {double sp_value; bool important_tasks_schedulable;}` rides the SP
eval so the seed schedulability audit costs ZERO extra RTA eval and is independent of
`HasSafeFallback()`. 17/17 ctest; release RunSpeedTest PASS (0.026/0.015 s/int).
DEFERRED: e2e comparison re-run (collapse intervals expect ≈0.90–0.93).
**Folder:** `agents/active_tasks/P1_28_incr_reopt_fallback_swap_down/`.
**Cross-links:** P1.27 (`cbdde63b`, BF-only fix — this is the INCR analogue), P0.7
(trigger b-ii backstop), P0.10, P2.17/P2.18 (gate-consistency family), P1.29
(in-search gate, INCR struct + prune).

## Anomaly
`taskset_2` `INCR_Reopt_10` `interval_sp_metrics.txt` collapses to **0.527888** at
intervals 16,17,18,19,21,22,38,39,41,44,52 (aggregate 0.855923); `INCR_WCET` stays
0.885–0.954 on all 60 (aggregate 0.920009). A schedulable ≥0.93 plan exists at every
collapsing interval (WCET reaches it on the identical taskset).

## Root cause (verified — CORRECTION 3, 2026-08-08)
0.527888 = RM-Fast fallback SP. Adopted by P0.7 trigger **(b-ii) post-walk backstop**
`AdoptFallbackIfUnschedulable` (`OptimizeSP_TL_Incre.cpp:1106`), confirmed 1:1 against
`interval_fallback_log.txt`: `adopted_fallback` at 16/21/38/41/44/52, culprit always
**task 0** (`task_1`, `important:true`, `sp_threshold:0.371116`), miss_chance
0.43–0.70 > 0.371116.

**CORRECTION 3 (decisive):** task 0 is **NOT TL-optimizable** — its
`timePerformancePairs` is EMPTY (the records CORRECTION 2 misattributed to task 0
belong to task 1). So the walk CANNOT change task 0's TL; its full ET dist is fixed.
The earlier "disarmed reopt beam" framing (CORRECTION 2's TL mechanism) is DISPROVEN.
The collapses are INCREMENTAL (reopt@20=0.954072→21=0.527888 one step later), and the
incremental path is NOT disarmed. **High-level root cause HOLDS:** the walk's
`WouldBeatIncumbent` (`:195`) is strict-SP-only — it never checks whether a challenger
is schedulable, so the SP-lowering-but-schedulable plan (reached via PRIORITY
ASSIGNMENT: give task 0 enough priority → less interference → miss ≤ thr) is rejected
in favor of the SP-max-but-unschedulable incumbent. Only the post-hoc backstop enforces
the constraint, binary-swapping to RM-Fast (0.527888) instead of the best schedulable
plan. BF proves a schedulable 0.90–0.93 plan EXISTS at every collapse interval
(int 16→0.931, 21→0.931, 22→0.945, 38→0.908, 41→0.908, 44→0.908, 52→0.904).

**INCR_WCET immune** because `SimulationOrchestrator.cpp:380-387` sets
`use_wcet_execution_time=true` → deterministic WCET surface → SP-max plan always
schedulable → backstop never fires. WCET's 0.92 is a stricter-ET artifact, NOT a
better search (same `Optimize_w_TL_ScratchOrIncre` path).

## The defect
On the incremental path, `SeedBaselineAndArmCache(Incremental)` (`:520-537`) re-scores
the carried `{opt_pa_, starting_time_limits}` under the new interval's drifted ET and
`CommitIncumbent`s it — but the re-scored baseline can be SP-max-yet-UNSCHEDULABLE.
The cache IS armed (`rta_cache_active_=true` `:525`), so the during-walk gate (b-i,
`UpdateRecords:238-258`) is active — BUT it only runs on SP-improving challengers, and
from an unschedulable SP-max incumbent there are ZERO SP-improving moves toward the
schedulable plan (the schedulable PA is SP-lower) → gate never engages → walk ends
unschedulable → post-hoc backstop binary-swaps to RM-Fast. No re-search for the best
SCHEDULABLE plan. `during_walk_reject_count=0` and `improving_challenger_count=0` at
every collapse interval confirm this.

## Fix (COMMITTED `393aab0a` — fallback-seed on unschedulable incumbent)
User principle: "the schedulers have to start with a solution that is schedulable. If
the initial solution from last interval's optimization configurations is not
schedulable at the new interval, start the optimization process with the fall-back
solution." Chosen fix (distinct from re-search Option A and constraint-aware
compare-and-keep Option C): change ONLY the **seed** of the incremental walk. In
`SeedBaselineAndArmCache`'s Incremental branch, after re-scoring the carried incumbent
under the new ET, if it is unschedulable, seed the walk from the **safe fallback**
(`safe_fallback_`'s `{priority_vec, tl}` — the P0.6 DM+important-first static solution,
certified schedulable on the cross-interval worst-case DAG) instead. The walk then
climbs SP from a schedulable base; every adopted SP-improving challenger passes the
armed b-i gate → converges to the best schedulable plan (≈0.90–0.93). Never worse than
the safe fallback (walk only adopts SP-improving+gate-passing moves → result ≥ seed SP).
`WouldBeatIncumbent`/`UpdateRecords`/reopt beam/`AdoptFallbackIfUnschedulable`
UNCHANGED (backstop KEPT as final safety net). Legacy bit-identical (no `is_important`
→ gate vacuous → seed check never triggers). TDD
`SeedBaselineAndArmCache_SeedsFromFallbackWhenCarriedIncumbentUnschedulable`.

## Follow-up R1–R4 (LANDED 2026-08-09, git add-only) — TasksSP refactor
User added two requirements for `SeedBaselineAndArmCache`: (1) audit the initial
solution's schedulability whenever `enable_fallback_use_`, independent of
`HasSafeFallback()`; (2) stop re-computing RTAs for that audit. Route (2): new
`struct TasksSP { double sp_value = INT_MIN; bool important_tasks_schedulable = true; }`
in `SP_Metric.h`; the flag folds into the EXISTING per-task SP loop in
`ObtainSP_TaskSet` (`is_important && ddl_miss_chance > threshold`), so it rides the SP
eval — ZERO extra RTA eval. Plumbed through `ObtainSP_DAG` (both overloads; node-level
flag, chains add `.sp_value` only) and `EvaluateSPWithPriorityVec` (P1.14 cancel →
`{INT_MIN, true}`). R4: `SeedBaselineAndArmCache` reads
`baseline_eval.important_tasks_schedulable`; guard restructured to
`enable_fallback_use_ && !important_tasks_schedulable && HasSafeFallback()` (audit
runs every fall-back-enabled interval, no `HasSafeFallback()` short-circuit; the
separate 3-arg `ImportantTasksMeetThresholds` fresh-RTA call is GONE). Truth-table
identical to HEAD (flag ≡ gate verdict, bit-identical by construction). New TDD tests
`ObtainSP_TaskSet_ReportsImportantTaskSchedulability`,
`ObtainSP_DAG_ReportsImportantTaskSchedulability`,
`SeedBaselineAndArmCache_KeepsCarriedIncumbentWhenUnschedulableWithoutFallback`.
17/17 ctest after each stage; release RunSpeedTest PASS (0.026/0.015 s/int).
DEFERRED: e2e comparison re-run (confirm collapses now ≈0.90–0.93); the deferred API-
uniformity candidate (`ObtainSP_DAG_From_Dists`/`ObtainSP_Full_From_NodeRTAs` +
P1.29 BF in-search gate still re-derive RTAs — out of scope here).

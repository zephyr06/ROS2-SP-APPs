# P1.30 — Dev log

## 2026-08-17 — ROOT CAUSE FOUND: INCR RTA-cache SP inflation (BF is correct)

### What the user asked
"why are there scenarios when INCR is better than BF? that's impossible?" →
"no i don't accept this bug... figure out why... maybe check why BF skipped it."

### Verdict
**BF did NOT skip anything. BF is correct. The bug is in INCR: its RTA-cache
SP-scoring path inflates the reported SP above the canonical metric, so INCR
WRONGLY appears to beat BF.** The user's instinct is right — INCR > BF IS
impossible under the canonical SP metric.

### Hard evidence — BF per-leaf trace (mid config, debugMode=1)
BF enumerates all 12 TSP time-limit leaves (one per value in TSP's
`performance_records_time`: 150 250 300 450 850 900 950 1000 1050 1100 1200
1500). Each leaf prints "Optimal SP is: X" = max SP over all 24 PAs at that TL
(via `EvaluateSPWithPriorityVec`):

| TSP TL | BF leaf Optimal SP |
|--------|--------------------|
| 1050   | 4.96058 |
| 1100   | **4.97052**  ← BF's global max → adopted |
| 1200   | **4.96962**  (LOWER than 1100) |
| 1500   | 3.99673 |

BF correctly adopted TL=1100 (4.97052) — the true maximum. TL=1200's true max
is 4.96962 < 4.97052. **No adoption bug, no skip, no budget timeout**
(total run 0.81 s << 10 s `TIME_LIMIT`).

### INCR's impossible number
INCR reported `res_opt_.sp_opt` = **4.98389** at TL=1200. But the canonical
function `EvaluateSPWithPriorityVec(dag_baked_TL1200, pa)` — which BF's TL=1200
leaf used to enumerate ALL 24 PAs — caps at **4.96962**. INCR's adopted PA is
one of those 24, so its canonical SP ≤ 4.96962 < 4.98389. **INCR's reported SP
is inflated by ≥ 0.014 above the canonical metric.** Arithmetic proof; no
instrumentation needed.

### Mechanism — where the inflation comes from
Two SP-scoring paths exist:

1. **Canonical (BF, and INCR's from-scratch seed):** `EvaluateSPWithPriorityVec`
   → `ObtainSP_DAG` → `ObtainSP_TaskSet` → `ProbabilisticRTA_TaskSet(tasks)`
   (fresh full RTA per eval).
2. **Cache path (INCR TL walk):** `ObtainSP_Full_From_NodeRTAs(dag, sp, pa, tl,
   node_rtas)` where `node_rtas = rta_cache_.Evaluate(dag, pa, no_tl)` — reuses
   the RTA cache's node RTAs instead of a fresh `ProbabilisticRTA_TaskSet`.

`ReOptimizePeriodic` → `RunIntervalDescent(Reopt)`:
- **Seed** (`SeedBaselineAndArmCache`, line 557-559): `CallOptimizerGivenTimeLimits
  (from_scratch=true)` → `OptimizeFromScratch` → SP via `EvaluateSPWithPriorityVec`
  (OptimizeSP_Incre.cpp:171). **Correct** (matches BF).
- **Cache arm** (line 566-570): `rta_cache_active_ = true`.
- **TL walk** (`WalkSerializedTaskQueue` → `OptimizeOneTaskWithTimeLimit` →
  `OptimizeIncreSingleTask` → `OptimizeIncre_SingleTask`): challenger eval uses
  the **cache path** (`ObtainSP_Full_From_NodeRTAs`, OptimizeSP_Incre.cpp:351)
  because `rta_cache` is engaged.

So TL=1200 was reached by the TL walk and its SP was scored by the cache path →
inflated to 4.98389. `UpdateRecords` → `CommitIncumbent` wrote that inflated SP
into `res_opt_.sp_opt`.

### Baking is NOT the divergence (both paths bake identically)
- `UpdateExtDistBasedOnTimeLimit` (BF leaf, OptimizeSP_TL_BF.cpp:7) =
  `GetUnitExecutionTimeDist(tl)` when `tl != -1`.
- `ApplyTimeLimitsToTasksExecutionTime` (cache path, SP_Metric.cpp:83) =
  `GetUnitExecutionTimeDist(tl)` when `tl != -1`. Same.
- The cache path is called with `no_tl` (all -1) because `dag_tasks_update` is
  already TL-baked (CallOptimizerGivenTimeLimits:272). So the bake is a no-op
  there; both paths see the same baked ETs. **H1 (baking divergence) ELIMINATED.**

### Where the divergence IS
The **node RTAs** differ: cache `rta_cache_.Evaluate` vs fresh
`ProbabilisticRTA_TaskSet`. The cache was claimed "bit-identical" (P1.23), but
it is NOT bit-identical in the TL-walk scenario (champion adopted at one TL,
eval at a different TL with `no_tl` on an already-baked dag). The cache's
suffix-patching (or its handling of an ET change = TL change) yields RTAs that
inflate SP. (No chains in this config, so the chain term — recomputed fresh in
both paths — is 0 and not the source.)

### Implications
- **All prior INCR > BF observations in P0.11 (mid, pertask) are suspect** —
  likely cache-path SP inflation, not real INCR superiority.
- **Potentially affects production/simulation INCR results**: `OptimizeIncre`
  always engages a local cache (OptimizeSP_Incre.cpp:386) and the TL walk arms
  `rta_cache_active_` (line 566). If the inflation reproduces on synthetic
  tasksets, published INCR SPs may be inflated vs the canonical metric. **Serious
  for PW (paper) — needs verification on the sim tasksets.**
- This is distinct from P1.18/P1.9/P1.25 cache work (those were about SPEED and
  claimed bit-identity; this is a CORRECTNESS hole in the bit-identity claim
  under TL-walk ET changes).

### Next steps (debug, not yet done)
1. Empirically confirm the cache returns wrong node RTAs: instrument
   `ObtainSP_Full_From_NodeRTAs` to ALSO compute a fresh
   `ProbabilisticRTA_TaskSet` and log the per-task RTA diff at TL=1200.
2. Localize: is it the suffix-patch (`Evaluate`'s `ComputeTaskSetDifference` →
   reuse prefix), or the ET-change handling (TL change treated as ET change)?
3. Decide fix: either (a) recompute fresh RTA for SP scoring (kills cache speed
   for SP — but cache can still speed the during-walk GATE), or (b) fix the
   cache's TL/ET-change path to be truly bit-identical.
4. Re-run P0.11 mid/pertask/mpc_important after the fix → expect INCR ≤ BF
   everywhere (BF = ground truth).

### Status
- ROOT CAUSE identified (INCR cache-path SP inflation; BF correct). Not yet
  empirically confirmed at the RTA level (step 1 above) nor fixed.
- `debugMode` toggled 0→1→0 in `sources/parameters.yaml` (restored).
- Nothing committed (agents only `git add`).

## 2026-08-17 — EMPIRICALLY CONFIRMED: cache node-RTAs diverge from fresh RTA

### Instrumentation added (NOT committed; debug-gated, temporary)
Added a debug-gated self-check at the end of `ObtainSP_Full_From_NodeRTAs`
(SP_Metric.cpp): recomputes the node RTAs FRESH via `ProbabilisticRTA_TaskSet`
(the canonical path) and re-scores SP via `ObtainSP_DAG_From_Dists` with the
SAME chain dists (`reaction_time_dists`), so the ONLY variable is node_rtas
(cache) vs fresh. On `|cache_sp - fresh_sp| > 1e-9` it logs the two SPs, the
diff, the vector sizes, and (on the first divergence) per-task miss-prob +
per-task SP contribution for both paths. Gated on `GlobalVariables::debugMode==1`,
capped at 20 reports. Built into `release/tests/AnalyzePriorityAssignmentIncremental`.

### Result — mid config (`rw_baseline_tightened.yaml`), INCR run
The self-check fired. The cache path's SP is INFLATED vs the fresh/canonical
path at EVERY TL step the walk visits:

| TSP TL | cache_sp  | fresh_sp  | diff (cache−fresh) |
|--------|-----------|-----------|--------------------|
| 1050   | 4.93404   | 4.93077   | +0.00326 |
| (mid)  | 4.94401   | 4.94071   | +0.00330 |
| (mid)  | 4.96395   | 4.96058   | +0.00337 |
| 1100   | 4.97392   | 4.97052   | +0.00340 |  ← BF's adopted TL (BF=4.97052 canonical, correct)
| 1200   | 4.98389   | 4.96962   | +0.01427 |  ← INCR's adopted; the big jump

- `fresh_sp` at every TL EXACTLY matches BF's per-leaf canonical SP
  (TL=1100→4.97052, TL=1200→4.96962). So the fresh path == BF == canonical. ✓
- `cache_sp` is inflated at every TL; the inflation is small (~+0.0034) for
  most TLs but JUMPS to +0.0143 at TL=1200 — exactly the 4.98389 INCR reported.
- Vector sizes: node_rtas=fresh=tasks=4 → NOT a length/indexing mismatch.
- **This proves the divergence is in the node RTAs themselves** (cache
  `Evaluate` returns node RTAs that differ from fresh `ProbabilisticRTA_TaskSet`
  for the SAME (dag, pa, tl)), NOT in baking or chain terms (those are identical
  between the two calls by construction).
- Per-task miss-prob / SP-contribution detail captured (see run log); pending
  read to identify WHICH task's RTA carries the divergence (TSP core0 is the
  only TL-changed task; SLAM core0 is its only HP/interferer → suspect SLAM or
  TSP node RTA).

### What this rules in/out
- Baking divergence (H1): RULED OUT (both paths bake identically; same chain dists).
- Chain term: RULED OUT (identical `reaction_time_dists` passed to both).
- Indexing/length mismatch: RULED OUT (sizes all 4).
- The bug IS in `RTACache::Evaluate`'s node-RTA computation for a TL (ET) change
  on one task — either the suffix-patch recompute or the FullReuse reindex.

### Next
1. Read the per-task detail from the run log → localize which task's RTA is
   wrong (suspect: the changed task TSP, or its core-0 neighbor SLAM).
2. Trace `RTACache::Evaluate` Rule-A path for an ET-only change to find the
   exact wrong step.
3. TDD: write a unit test that warms the cache (Initialize at TL_A), Evaluate
   at TL_B (ET change on one task), asserts cache node_rtas == fresh RTA.
4. Fix; re-run; expect cache_sp == fresh_sp at every TL → INCR ≤ BF.

## 2026-08-17 — ROOT CAUSE CONFIRMED: loose 1e-1 ET-equality tolerance

### The actual bug (NOT RollPrefix)
Static re-analysis of `RollPrefix` (RTA_Cache.cpp:24) vs the oracle's rolling
prefix (RTA.cpp:110-112): they are BYTE-IDENTICAL (both `Compress(G*1)` then
`Convolve(et)`). The cache recompute loop (RTA_Cache.cpp:498-517) mirrors
`ProbabilisticRTA_TaskSet_SingleCore` exactly — same `IdentityPrefix` start,
same per-task `RollPrefix`, same 3-arg `GetRTA_OneTask`. The dev_log's prior
"bug is in RollPrefix" hypothesis was WRONG.

The real divergence is UPSTREAM, in the ET-CHANGE DETECTOR:
- `RTACache::IsSingleTaskChange` (RTA_Cache.cpp:272) calls
  `FindTaskWithDifferentEt(champion_.champ_tasks_baked, cand_tasks_baked)`.
- `FindTaskWithDifferentEt` (OptimizeSP_Incre.cpp:217) tests
  `tasks_base[i].execution_time_dist != tasks_updated[i].execution_time_dist`,
  i.e. `FiniteDist::operator!=` → `!FiniteDist::operator==`.
- `FiniteDist::operator==` (Probability.cpp:415) → `approx_equal(other, 1e-1)`,
  and `Value_Proba::operator==` (Probability.cpp:12) uses
  `approx_equal_double(..., 1e-1)` — a LOOSE 10% RELATIVE tolerance.

A TL-baked ET is a POINT MASS at the TL value (`GetUnitExecutionTimeDist` →
`FiniteDist({{tl, 1.0}})`, Probability.h:177). Two close TLs are both size-1
point masses, so `approx_equal` reaches the value compare:
`approx_equal_double(1100, 1200, 0.1) = |100|/|1100| = 0.0909 < 0.1 → EQUAL`.
So the ET change 1100→1200 is MISSED.

### Consequence (matches the empirical divergence exactly)
When the TL walk steps between ADJACENT grid values (1100→1200, 1050→1100,
950→1000, … — all < 10% relative at these magnitudes), `FindTaskWithDifferentEt`
returns EMPTY for the changed task → `IsSingleTaskChange`: `et_diff.empty()` +
same PA → `|diff|==0` → `ClassifyReusePerTask` returns ALL FullReuse →
`RTACache::Evaluate` returns the REINDEXED CHAMPION RTA verbatim, i.e. the
changed task's RTA is FROZEN at the CHAMPION's TL, NOT recomputed at the
candidate TL. That stale RTA carries the wrong (lower) miss-prob → inflated SP.

This matches the instrumented per-task table: ONLY the TL-changed task (TSP,
id0) diverges; its cache miss-prob (0.3618) < fresh (0.4735) — the stale
champion-TL RTA under-estimates the candidate-TL miss-prob. The big +0.014
jump at TL=1200 = cumulative drift once the champion TL lags far enough.

### Why the existing narrow-ET tests PASS
`Evaluate_TLChange_OneTaskPatch` uses TL=3 vs no-TL: champion ET is a 2-point
dist (size 2), candidate is a point mass (size 1) → `approx_equal` returns
false on the SIZE mismatch → change DETECTED → NoReuse → recompute → correct.
The bug only bites when BOTH champion and candidate ETs are the SAME SHAPE
(same support size) with values within 10% relative — i.e. two close TL-baked
point masses, exactly the TL-walk's adjacent-step case.

### TDD RED test added (tests/testRTA.cpp)
`Evaluate_TLChange_SmallRelativeMagnitude_RecomputesChangedTask` in
`TaskSetForTest_4tasks_2cores_cache`: Initialize champion with task1 TL=1100,
Evaluate candidate with task1 TL=1200 (rel 0.091 < 0.1), assert cache RTA ==
oracle RTA. RED on HEAD (cache returns the stale TL=1100 RTA for task1).
Run: `./build_test/tests/testRTA --gtest_filter='*SmallRelativeMagnitude*'`.

### Fix direction (NOT yet applied)
`FindTaskWithDifferentEt`'s purpose is to detect ANY ET change (a TL change,
however small, changes the RTA and MUST trigger recompute for the cache's
bit-identity contract). The loose `FiniteDist::operator!=` (1e-1) is wrong
for this. Fix: detect ET changes with a TIGHT tolerance (e.g.
`!approx_equal(other, 1e-9)`) or exact structural compare, NOT the loose
`operator!=`. Do NOT change `Value_Proba::operator==` globally (its 1e-1 may
be load-bearing elsewhere); localize the fix to the diff detector.

## 2026-08-17 — LOCALIZED to TSP's RTA + TDD RED test added

### TDD test added (tests/testBF_w_TL.cpp)
`TaskSetForTest_p130_rw_mid::BF_NotWorseThan_INCR` — loads the mid config
(`TaskData/p0_11_variants/rw_baseline_tightened.yaml`), runs BF
(`EnumeratePA_with_TimeLimits`) and INCR (`OptimizePA_Incre_with_TimeLimits` +
`ReOptimizePeriodic(dag_tasks, 2)`), asserts `res_bf.sp_opt >= res_incr.sp_opt`.
RED on HEAD: BF=4.97052 < INCR=4.98389. Built into `build_test/tests/testBF_w_TL`.
Run: `./build_test/tests/testBF_w_TL --gtest_filter='*p130_rw_mid*'`. ~8s.

### Per-task divergence (first divergence, TL=1050 step)
The debug self-check's `divergence_reports==1` block printed per-task detail:

| task | pos | dl | thr | w | cache_miss | fresh_miss | contrib_diff |
|------|-----|----|-----|---|------------|------------|--------------|
| SLAM (id3) | 0 | 2000 | 0.1 | 2 | 0 | 0 | 0 |
| MPC (id1)  | 1 | 10 | 0.1 | 1 | 0 | 0 | 0 |
| RRT (id2)  | 2 | 4000| 0.1 | 1 | 0.383375 | 0.383375 | 0 |
| **TSP (id0)** | **3** | 1500| 0.1 | 1 | **0.361772** | **0.473451** | **+0.0034715** |

**LOCALIZED: the divergence is in TSP's RTA (task id=0, candidate priority pos=3).**
Every other task has contrib_diff=0. TSP's cache miss-prob (0.3618) is LOWER
than fresh (0.4735) → cache UNDER-estimates TSP's miss-prob → inflates SP
(higher SP). This is the UNSAFE direction (over-claiming safety).

### What this means
TSP is the TL-changed task (the only task whose ET changes across the walk).
In the candidate PA, TSP is at priority pos 3 (lowest). On core0
({SLAM, TSP}), SLAM > TSP, so TSP's HP set = {SLAM}. TSP is the NoReuse
(changed) task → its RTA is recomputed via 3-arg `GetRTA_OneTask(TSP,
hp_tasks={SLAM}, hp_tasks_et_conv=<rolled SLAM ET prefix>)`. The oracle
(`ProbabilisticRTA_TaskSet`) computes TSP's RTA via the SAME 3-arg form with
the SAME HP set — yet the two diverge. **So the bug is in the `hp_tasks_et_conv`
PREFIX the cache builds vs the oracle's rolling prefix, NOT in GetRTA_OneTask
itself (both call the 3-arg form).**

Suspect: `RollPrefix` (RTA_Cache.cpp:24, `Compress(Granularity*1)` + `Convolve`)
builds the SLAM-ET prefix DIFFERENTLY from the oracle's rolling prefix
(RTA.cpp:110-112). SLAM's ET is EXTREMELY wide (sigma=577, range [68,1560],
~150 support points at granularity 5 → ~30 at granularity 10) → the lossy
`CompressDistributionWithOnlySize` fires hard, so any prefix-build divergence
is amplified into a large RTA diff (matches the +0.014 jump at TL=1200).

### Next
1. Diff `RollPrefix` (RTA_Cache.cpp) vs the oracle's rolling-prefix build
   (RTA.cpp `ProbabilisticRTA_TaskSet_SingleCore`, lines ~87,110-112). Find
   the exact step where the SLAM-ET prefix diverges.
2. Add a focused unit test: Initialize cache at TL_A, Evaluate at TL_B (TSP ET
   change), assert TSP's cache RTA == fresh `ProbabilisticRTA_TaskSet` RTA
   (the `divergence_reports==1` per-task check, promoted to a real assertion).
3. Fix the prefix build; re-run `BF_NotWorseThan_INCR` → expect GREEN.

## 2026-08-18 — FIX APPLIED + VERIFIED (INCR ≤ BF everywhere)

### Fix
`FindTaskWithDifferentEt` (sources/Optimization/OptimizeSP_Incre.cpp:212) now
detects ET changes with a TIGHT tolerance —
`!et_base.approx_equal(et_updated, 1e-9)` — instead of the loose
`FiniteDist::operator!=` (whose underlying `operator==` uses a 1e-1 RELATIVE
tolerance). A TL-baked ET is a point mass at the TL value, so two close TLs
(1100 vs 1200, rel 0.091 < 0.1) compared EQUAL under the loose operator, the
change was MISSED, the changed task was wrongly classified FullReuse, and the
cache returned the STALE champion-TL RTA → SP inflation. The tight compare
flags any genuine TL/env change yet still tolerates identical re-bakes. The
fix is LOCALIZED to the diff detector; `Value_Proba::operator==`'s global 1e-1
is left untouched (may be load-bearing elsewhere).

### TDD verification (RED → GREEN)
- `Evaluate_TLChange_SmallRelativeMagnitude_RecomputesChangedTask`
  (tests/testRTA.cpp): RED on HEAD with the loose operator (cache returned the
  stale TL-11 RTA {12:1.0} for a TL-12 candidate; oracle wants {13:1.0});
  GREEN with the tight-tolerance fix. Confirmed both directions by temporarily
  reverting the one compare line.
- `TaskSetForTest_p130_rw_mid::BF_NotWorseThan_INCR` (tests/testBF_w_TL.cpp):
  end-to-end mid-config check. GREEN: `BF sp=4.97052 INCR sp=4.97052` (was
  BF=4.97052 < INCR=4.98389).

### Full-suite + speed
- `check.SP_OPT`: 17/17 PASS (DEBUG build_test).
- `RunSpeedTest` (release): PASS, 0.056 s/interval (< 0.1 s threshold). No
  regression — the tight compare adds negligible cost (one per-task dist
  compare per Evaluate; the cache still skips the RTA recompute for truly
  unchanged tasks, so the speedup over fresh-only is retained).

### P0.11 re-run (release binaries, post-fix + post-cleanup)
| variant | BF SP | INCR SP (pre-fix) | INCR SP (post-fix) |
|---------|-------|-------------------|--------------------|
| mid (`rw_baseline_tightened`)    | 4.97052 | 4.98389 (+0.0134) | **4.97052** |
| pertask (`rw_pertask_thresholds`)| 4.96487 | 4.98181 (+0.0169) | **4.96487** |
| mpc_important (`rw_mpc_important`)| 4.97052 | (inflated) | **4.97052** |

INCR ≤ BF on ALL three (in fact INCR == BF). The inflation is gone. BF remains
the ground truth; INCR no longer wrongly beats it.

### Cleanup
- Removed the temporary debug self-check from `ObtainSP_Full_From_NodeRTAs`
  (sources/Safety_Performance_Metric/SP_Metric.cpp) — the per-task
  cache-vs-fresh RTA divergence logger that confirmed the root cause. The TDD
  test is the permanent guard; the debug block (gated on debugMode) was
  debug-only scaffolding. Re-verified 17/17 + mid-config after removal.
- Restored `sources/parameters.yaml` `debugMode: 1 -> 0`.

### Status
- FIX APPLIED, VERIFIED, CLEANED. NOT committed (agents only `git add`).
- Awaits user review + commit. Suggested commit split:
  (1) the `FindTaskWithDifferentEt` tight-tolerance fix + its TDD test
      (OptimizeSP_Incre.cpp + testRTA.cpp) — the core correctness fix;
  (2) the end-to-end `BF_NotWorseThan_INCR` regression test (testBF_w_TL.cpp +
      CMakeLists.txt) — guards against regression on the real-world config;
  (3) the `AnalyzePriorityAssignmentIncremental` INCR driver + its `Adopted SP`
      print (AnalyzePriorityAssignmentIncremental.cpp + CMakeLists.txt +
      AnalyzePriorityAssignment.cpp) — the P0.11 eval tooling.
- DEFERRED: verify whether prod/sim INCR SPs were inflated (PW impact) — sample
  a synthetic sim taskset. The fix is in the shared cache path, so any prior
  INCR run that stepped the TL walk across < 10%-relative adjacent grid values
  may have reported inflated SP.

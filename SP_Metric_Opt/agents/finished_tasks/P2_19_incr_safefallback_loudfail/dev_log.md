# P2.19 — Dev Log

## 2026-08-01 — Reproduced + root-caused (H1 confirmed: seed infeasible)

**Repro:** `RunOrchestrator taskset_3 INCR_Reopt_10 10000 1` → core dump +
the `ComputeSafeFallback` loud-fail message. Deterministic. Only
`INCR_Reopt_10` crashes; BF/CFS/DM_FAST/DM_SLOW run clean on taskset_3.

**NOT P2.18:** crashing binary (built 14:05) INCLUDES `f371c543` (P2.18 fix,
13:56). Predates `02f3c8fd` (P0.10 §2, 19:17) but that's IRRELEVANT — the crash
is INCR-side `ComputeSafeFallback`, not the BF path. Genuinely new.

**Diagnostics added** (temporary, `OptimizeSP_TL_Incre.cpp` near the loud-fail):
gate the SEED {DM PA, tl_seed} before the walk + print
`WorstCaseImportantTaskMissInfo` for seed and candidate + the worst-case DAG's
`GetAvgValue()` for task 0.

**Decisive output:**
```
P2.19 DIAG seed gate: ok=0 task_id=0 miss=1.000000 thr=0.500000
                     tl_seed0=45.000000 wc_avg0=45.000000
P2.19 DIAG candidate: pa=[6] tl0=45.000000 task_id=0 miss=1.000000 thr=0.500000
```

→ **H1 confirmed: the SEED itself fails the gate before the walk runs.** Task 0
(`task_1`, perf, important, period=50, deadline=50) misses with probability 1.0.

## Root cause — perf-task WCET mismatch (P0.8 vs ComputeSafeFallback)

`BuildWorstCaseDagAcrossIntervals` (`WorstCaseDAG.cpp:63-70`) sets each task's
worst-case dist to a point mass at `execution_time_dist.max_time` (the max
across intervals). For a **perf task**, `execution_time_dist.max_time` =
`execution_time_max` = the **TL-grid upper bound** (45.0 for task 0), NOT the
faithful ET.

`RegularTasks.cpp:77-79` builds the dist from
`(gauss, execution_time_min, execution_time_max, granularity)` →
`dist.max_time = execution_time_max` = 45.0 for task 0.

But the **faithful perf-task WCET is `et_mean`** (= `execution_time_mu` =
4.225). P0.8's gate (`important_task_rta.py:_wcets_from_loaded_tasks`) uses
EXACTLY this: perf → `execution_time_mu`, non-perf → `execution_time_max`. Its
docstring states it explicitly:

> The on-disk `execution_time_max` for a perf task is a TL-grid BOUND (the
> exporter forces it to the config range), NOT the ET support; the prior
> `period * tl_grid_upper` rule was a conservative bound that over-rejected.

So **P0.8 certifies task 0 at WCET=4.225** (trivially meets deadline 50);
`ComputeSafeFallback` certifies it at WCET=45.0 (misses deadline 50 under
interference). Both gates are "correct" under their OWN WCET — they DISAGREE
because they use different perf-task WCETs. `ComputeSafeFallback`'s is wrong
(over-conservative by 10x for this task).

**Compounded mechanism (why the SEED fails, not just the walk):**
`SeedTimeLimitsAtOrBelowEtMean` (`OptimizeSP_TL_Incre.cpp:610`) calls
`GetAvgValue()` on the worst-case DAG's dist. On a point-mass-at-45.0 dist,
`GetAvgValue()` = 45.0 → it picks the largest TL option ≤ 45.0 = **45.0** (the
max/worst TL option). The comment claims the seed is "TL ≤ et_mean →
miss_chance 0"; but on the worst-case DAG `GetAvgValue()` IS the inflated WCET,
not et_mean, so the seed picks the LEAST safe TL. The walk can only raise TL
further → still 45.0 → still misses.

## Two bugs, one root
1. **`BuildWorstCaseDagAcrossIntervals` over-inflates perf-task WCET** — uses
   `execution_time_dist.max_time` (TL-grid bound) instead of `et_mean`. This is
   the PRIMARY bug; it's why the gate disagrees with P0.8.
2. **`SeedTimeLimitsAtOrBelowEtMean` reads the worst-case DAG's `GetAvgValue()`**
   — on the inflated worst-case DAG this returns the WCET, so the seed picks the
   max TL. Symptom of bug 1, but the direct mechanism of the seed failure.

Fixing bug 1 (use et_mean for perf tasks in the worst-case DAG) should fix both:
with task 0's worst-case dist = point mass at 4.225, `GetAvgValue()` = 4.225 →
seed picks TL ≤ 4.225 → RTA ≈ 4.225 + interference << 50 → gate passes.

## Taskset_3 task 0 params (the trigger)
- period=50, deadline=50, important=True, perf=True, processorId=0
- `execution_time_mu` (et_mean) = 4.225
- `execution_time_max` (TL-grid bound) = 45.0  ← 10x the faithful WCET
- `sp_threshold` = 0.6123
- performance_records_time: 2.5 ... 45.0 (10 TL options, max = 45.0)

## NEXT
Decide the fix scope:
- **(A) Fix `BuildWorstCaseDagAcrossIntervals`** to use et_mean for perf tasks
  (match P0.8's WCET rule). Touches `WorstCaseDAG.cpp` only. This is the
  root-cause fix — makes the worst-case DAG's perf-task WCET faithful.
- **(B) Also fix `SeedTimeLimitsAtOrBelowEtMean`** to read the ORIGINAL
  `et_mean` (not the worst-case dist's `GetAvgValue()`). Defensive, but if (A)
  lands this becomes a no-op (worst-case `GetAvgValue()` == et_mean for perf
  tasks under (A)).
- **(C) Make `ComputeSafeFallback`'s loud-fail a WARN+continue (keep best
  candidate) instead of a throw.** WRONG direction — the loud-fail is the P1.15
  safety net; silencing it re-opens the silent-infeasibility hole. Do NOT do (C).

Recommend (A) as the root-cause fix; (B) optional defensive. Then TDD a test
reproducing a perf task whose `execution_time_max` >> `et_mean` where the
worst-case DAG must use et_mean.

## 2026-08-01 — Diagnosis verified against codebase; fix (A) scoped

**Fix (A) confirmed sound + scoped.** Re-traced the full consumption path:

- `BuildWorstCaseDagAcrossIntervals` (`WorstCaseDAG.cpp:62-70`) overwrites EVERY
  task's dist with a point mass at `execution_time_dist.max_time` (max across
  intervals). For a PERF task (carries `timePerformancePairs`) `max_time` =
  `execution_time_max` = the **TL-grid bound** (45.0 for task 0), NOT the
  faithful WCET. This is bug 1.
- P0.8's authoritative gate (`important_task_rta.py:_wcets_from_loaded_tasks`,
  lines 357–414): perf → `execution_time_mu` (= `et_mean`), non-perf → global
  max `execution_time_max`. Its docstring (lines 363–375) states the on-disk
  `execution_time_max` for a perf task is a TL-grid BOUND, NOT the ET support,
  and "the simulator caps perf runtime ET at min(et_mean, TL), so
  `execution_time_mu` is interval-invariant." So `et_mean` is the SOUND
  perf-task WCET. `ComputeSafeFallback`'s worst-case DAG uses the WRONG one →
  over-inflates perf WCET ~10x → spurious loud-fail.
- Reliable C++ source of `et_mean`: `Task::exec_time_gauss.mu`, set at read time
  (`RegularTasks.cpp:101`, `task.setExecGaussian(GaussianDist(execution_time_mu,
  ...))`). Traced prod path: `SimulationOrchestrator` → `ReadDAG_Tasks` →
  `ReadTaskSet` → `setExecGaussian`. So `exec_time_gauss.mu` is set for every
  YAML-loaded task including perf tasks → the authoritative `et_mean`.

**Soundness of fix (A) for perf tasks:** the old
`StochasticallyDominatesEveryInterval` property (worst.max_time >= every
interval's max_time) was a CONSERVATIVE FICTION for perf tasks — a perf task's
`execution_time_dist.max_time` is a TL-grid bound, not an ET support bound, so
"dominating" it is meaningless. The correct perf soundness property is
worst-case WCET (`et_mean`) >= actual runtime ET (≤ `et_mean` by the sim cap) —
holds trivially. Non-perf dominance is unchanged (still max across intervals).

**Affected tests** (`tests/testIncreOpt_w_TL.cpp`):
- `TakesMaxExecutionTimeMaxAcrossIntervals` (4111), `StochasticallyDominates-
  EveryInterval` (4208): NON-PERF only (`BuildTwoNonPerfDag`) → UNAFFECTED.
- `PreservesPerfTaskTimeLimitGrid` (4133): asserts the BUGGY perf-WCET=max_time
  (line 4178). MUST rewrite to assert perf-WCET == `et_mean`
  (`exec_time_gauss.mu`).
- `ComputeSafeFallback_*` (3999/4040/4073/4235): use `CompareAndKeepSynthetic`
  fixture whose `T_perf` has `et_perf=500`, grid [400..1000], Gaussian mean 500
  → `et_mean==max_time` coincidentally → fix is a no-op there (still pass).

**Plan:** TDD red test (perf task `execution_time_max` >> `et_mean`, current
worst-case DAG rejects → after fix accepts) → implement fix (A) in
`WorstCaseDAG.cpp` (perf → point mass at `exec_time_gauss.mu`) → rewrite
`PreservesPerfTaskTimeLimitGrid` → remove P2.19 temp diagnostics → regression.
Fix scope = `WorstCaseDAG.cpp` + tests only (no `OptimizeSP_TL_Incre.cpp` logic
change beyond removing diagnostics).

## 2026-08-01 — Design SUPERSEDED by user's 3-point design; scope decided

The earlier fix (A) (perf → point mass at `et_mean` / `exec_time_gauss.mu`) is
SUPERSEDED. User's design (verbatim intent):

1. `BuildWorstCaseDagAcrossIntervals` should use the **minimum possible
   time-limit option** (not the longest TL / `max_time`) as the perf-task input
   ET, because the optimizer selects these TLs.
2. `ComputeSafeFallback` should keep running the incremental optimizer until
   convergence (a full pass over all tasks cannot improve the best SP).
3. Rename `BuildWorstCaseDagAcrossIntervals` →
   `BuildDAGForObtainSafeFallBAckAcrossIntervals`.

**Why (1) min-TL is sound (verified against the data flow):** the gate
`ImportantTasksMeetThresholds` (`SP_Metric.cpp:245-256`) BAKES the chosen TL into
the perf dist via `ApplyTimeLimitsToTasksExecutionTime` (`SP_Metric.cpp:76-86`),
then runs `ProbabilisticRTA_TaskSet` on the baked dist. So the worst-case DAG's
stored perf dist NEVER reaches the gate's RTA — it's consumed only by the SEED
selector `SeedTimeLimitsAtOrBelowEtMean` (`OptimizeSP_TL_Incre.cpp:600-624`,
which reads `GetAvgValue()` → picks largest TL ≤ that) and the DM tie-break
(`DeadlineMonotonicPriorityVec:893-894`, also `GetAvgValue()`). Min TL → least
interference → most feasible seed; the walk still has the FULL TL grid
(`RecordTimeLimitOptions`, `OptimizeSP_TL_BF.cpp:19-35`, not a neighborhood) to
climb to the best-SP gate-feasible TL. Non-perf dominance unchanged; perf
soundness holds because the gate certifies the CHOSEN TL, not the stored dist.

**Min-TL vs et_mean:** min-TL is more conservative (e.g. 2.5 vs et_mean 4.225
for task 0). Also more robust: et_mean can exceed the grid, re-triggering the
over-seed bug. Min-TL is a grid option by construction → seed always picks it.

**Scope DECIDED (user-approved):** (1)+(3) land in P2.19 — (1) alone stops the
SIGABRT. (2) is a behavior-change ENHANCEMENT (not the crash fix), deferred to a
SEPARATE task **P2.20** (`agents/active_tasks/P2_20_incr_fallback_convergence_loop/`).
P2.20 design points: `OptimizeIncre_w_TL` is single-pass (`:839-858`) → needs a
convergence wrapper with `ApproxEqualSP` + max-iter cap; the `BFDLSharedBudget`
(`ComputeSafeFallback:974`) must still bound the looped runtime; the in-walk gate
(P0.7, `enable_fallback_use_=true` here) must keep rejecting gate-infeasible
candidates each pass.

**Casing note for review:** user's literal `FallBAck` reads as a typo; my
suggested normalized form is `BuildSafeFallbackDagAcrossIntervals` (matches the
codebase `Dag` convention). Defer to user at git-add review.

**Implementation plan:** TDD red (rewrite `PreservesPerfTaskTimeLimitGrid`
4133 + add `WorstCaseDagUsesMinTimeLimitForPerfTasks`) → implement (1) in
`WorstCaseDAG.cpp` → (3) rename across header/def/caller/tests → remove P2.19
temp diagnostics → build (`--clean-first`, header changed) + regression → repro.

## 2026-08-01 — LANDED (points 1 + 3); regression + repro green

**TDD red→green:** rewrote `PreservesPerfTaskTimeLimitGrid` (asserts perf
worst-case dist = point mass at `timePerformancePairs[0].time_limit`, not the
inflated `max_time`); added `WorstCaseDagUsesMinTimeLimitForPerfTasks` (perf
`max_time`=45.0 >> min-TL=2.5 across 2 intervals → `GetAvgValue()` == 2.5).
Both RED before fix (124/125 + the 2 new failing), GREEN after.

**(1) fix** (`sources/TaskModel/WorstCaseDAG.cpp`, dist-overwrite loop): split
by task type. Perf (`!timePerformancePairs.empty()`) → `GetUnitExecutionTimeDist`
at `timePerformancePairs[0].time_limit` (min TL option, read from `worst` =
interval 0); non-perf → unchanged max `max_time` across intervals. Updated the
function comment to drop the stale "stochastic dominance for perf" fiction
(perf `max_time` is a grid bound, not an ET support bound).

**(3) rename:** `BuildWorstCaseDagAcrossIntervals` →
`BuildDAGForObtainSafeFallBAckAcrossIntervals` (user-specified casing, kept
verbatim) via sed across `DAG_Model.h` (decl + comment), `WorstCaseDAG.cpp`
(def + 3 throw msgs), `SimulationOrchestrator.cpp:319` (prod caller),
`testIncreOpt_w_TL.cpp` (13 refs). Clean rebuild green.

**Removed P2.19 temp diagnostics** from `ComputeSafeFallback`
(`OptimizeSP_TL_Incre.cpp`): deleted the seed-gate print + candidate print
blocks; kept the loud-fail re-gate throw (P1.15 safety net). Also corrected the
now-stale seed comment ("GetAvgValue()==max_time" → "point mass at MIN TL
option → least-interference seed").

**Regression:** `cmake --build build_test --target check.SP_OPT -j5 --clean-first`
→ 16/17 ctest (sole failure pre-existing `OrchestratorTest.CFS_RunOrchestrator_Binary`);
`testIncreOpt_w_TL` 125/125; legacy BF `testBF_w_TL` + `testBFRTimeout` +
`testOptimizePA` all green.

**Repro:** `release/tests/RunOrchestrator <taskset_3> /tmp/p219_repro_out
INCR_Reopt_10 10000 1` → **exit 0** (was SIGABRT + core dump). SP=0.754635,
`SafeFallbackComputeTime_s` ran clean, no loud-fail message. DM_FAST on the
same taskset → exit 0. BF on taskset_3 is slow (pre-existing P1.14 brute-force
runtime; confirmed BF does NOT call `BuildDAGForObtainSafeFallBAckAcrossIntervals`
→ unrelated to this change).

**Casing flag for review:** used the user's exact `FallBAck` casing. Reads as a
typo; suggested normalized `BuildSafeFallbackDagAcrossIntervals` (codebase `Dag`
convention). Defer to user at commit.

**Point (2)** (convergence loop) → P2.20 stub created
(`agents/active_tasks/P2_20_incr_fallback_convergence_loop/`), not started.

## 2026-08-01 — Comment cleanup + re-verify + stage (handoff to user)

The first landing pass left STALE comments describing the OLD "point mass at
max(execution_time_max) / stochastically dominates every interval" behavior at
four sites the rename touched but didn't rewrite:

1. `SimulationOrchestrator.cpp:316` (caller comment above the renamed call).
2. `DAG_Model.h:144` (the function's header doc-comment).
3. `testIncreOpt_w_TL.cpp:3986` (the §8 section header).
4. `testIncreOpt_w_TL.cpp:4252` (the `StochasticallyDominatesEveryInterval`
   comment — that test is NON-PERF only, so dominance still holds, but the
   wording read as a universal claim).

Rewrote each to state perf=min-TL-option + non-perf=max-across-intervals; tagged
the dominance test as the "non-perf soundness leg" with a cross-ref to
`WorstCaseDagUsesMinTimeLimitForPerfTasks`. Comment-only changes.

**Re-verified after the comment edits:**
- `cmake --build build_test --target check.SP_OPT -j5` → 16/17 ctest, sole
  failure the pre-existing `OrchestratorTest.CFS_RunOrchestrator_Binary` (CFS
  binary, unrelated to P2.19).
- `testIncreOpt_w_TL` → **126/126** (was 125 in the prior log entry; the count
  is 126 because the new `WorstCaseDagUsesMinTimeLimitForPerfTasks` test was
  added — the prior log's "125/125" was already slightly stale).
- Release `libSP_OPT.so` carries the new symbol
  `BuildDAGForObtainSafeFallBAck` (6 string hits); `release/tests/RunOrchestrator`
  links it (rebuilt clean, no stale-binary risk).
- Repro: `release/tests/RunOrchestrator TaskData/sim_experiments/taskset_3
  /tmp/p219_repro_out INCR_Reopt_10 10000 1` → **exit 0** (was SIGABRT + core
  dump). `SafeFallbackComputeTime_s` ran clean; no loud-fail message.

**Staged (`git add`) — P2.19 files ONLY:**
`sources/TaskModel/WorstCaseDAG.cpp`, `sources/TaskModel/DAG_Model.h`,
`sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp`,
`sources/Optimization/OptimizeSP_TL_Incre.cpp`, `tests/testIncreOpt_w_TL.cpp`,
`agents/active_tasks/P2_19_incr_safefallback_loudfail/`,
`agents/active_tasks/P2_20_incr_fallback_convergence_loop/`.

Left UNSTAGED (pre-existing P0.10 uncommitted work, NOT part of P2.19):
`agents/active_tasks/P0_10_bf_important_task_fallback/{dev_log,tasks}.md`,
`agents/dev_log.md` (mixed P0.10+P2.19 entries),
`simulation_experiments/configs/compare_against_bf.json` (P0.10 N=[4,6] config).

**Awaiting user review + commit.** Open casing question still: the user's
literal `FallBAck` reads as a typo; suggested normalized
`BuildSafeFallbackDagAcrossIntervals`. Defer to user.

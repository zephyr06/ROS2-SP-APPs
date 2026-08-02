# P2.19 — INCR `ComputeSafeFallback` loud-fail crashes `INCR_Reopt_10`

## Symptom
`compare_against_bf` (N=6, seed 1000+20 step, taskset_3) SIGABRTs on the
`INCR_Reopt_10` arm ONLY. BF / CFS / DM_FAST / DM_SLOW all ran CLEAN on the
same taskset_3. The crash is NOT a segfault — it's a deliberate `throw
std::runtime_error` from `ComputeSafeFallback`'s loud-fail re-gate
(`OptimizeSP_TL_Incre.cpp:1027-1035`):

> `ComputeSafeFallback: the worst-case-DAG result VIOLATES the important-task
> gate (a task's ddl_miss_chance > threshold). No safe fallback exists for this
> task set — regenerate a new task set.`

So the C++ throws (P1.15 layer B: turn silent sim failure into a crash), and the
harness (P1.15 layer A) reports it as SIGABRT.

## Reproduced
Deterministic. `RunOrchestrator <taskset_3> /tmp/out INCR_Reopt_10 10000 1` →
core dump + the message above. (2026-08-01.)

## Timeline (rules out stale-binary)
- `f371c543` (P2.18 RTA-cache-mid-beam fix) committed 13:56.
- Crashing `release/tests/RunOrchestrator` built 14:05 → INCLUDES `f371c543`.
- Crash run.log written 14:47.
- `02f3c8fd` (P0.10 §2 BF gate) committed 19:17 — AFTER the crash → NOT in the
  crashing binary, but IRRELEVANT: the crash is INCR-side `ComputeSafeFallback`,
  not the BF path P0.10 §2 touches.

So this is a GENUINELY NEW failure mode — distinct from P2.18 (which was the
RTA-cache-mid-beam `|diff|>1` throw, fixed by `f371c543`). The P2.18 fix is
present; this still fires.

## What the code does (`OptimizeSP_TL_Incre.cpp:963-1039`)
`ComputeSafeFallback(worst_case_dag)` builds the offline safe-fallback artifact
on the caller-built CROSS-INTERVAL worst-case DAG:
1. Forces real-ET dist + TL-opt-on globals; installs a `BFDLSharedBudget`.
2. Makes a throwaway sibling `fallback_solver` on `worst_case_dag`; forces
   `enable_fallback_use_ = true`.
3. Seeds at DM PA + `SeedTimeLimitsAtOrBelowEtMean()` (TL ≤ et_mean →
   ddl_miss_chance = 0 → "seed gate-feasible, so the walk can only REJECT").
4. Runs `OptimizeIncre_w_TL` (gate-governed TL walk).
5. Loud-fail re-gate on the FINAL stored result via the SELF-CONTAINED
   `ImportantTasksMeetThresholds(worst_case_dag, sp_params_worst,
   candidate.priority_vec, tl_result)` (re-derives RTAs fresh).
6. On FAIL → `CoutWarning` + `throw`. On PASS → store as `safe_fallback_`.

## The puzzle (HYPOTHESIS — to verify)
The comment at `:989-992` + `:1022-1026` claims the seed is
feasibility-by-construction (TL ≤ et_mean → point mass → miss_chance 0) and the
walk "can only REJECT → a miss means the seed was infeasible → not walk-fixable".
Yet the loud-fail fires. So EITHER:
- (H1) The seed is NOT actually feasible on the worst-case DAG —
  `SeedTimeLimitsAtOrBelowEtMean()` does not in fact bound TL ≤ et_mean for this
  taskset (e.g. a task with no `timePerformancePairs`, or a TL option floor above
  et_mean, or an et_mean that's not the max on the worst-case DAG). The "point
  mass → miss 0" reasoning breaks if a TL option exceeds et_mean.
- (H2) The walk ADOPTED a candidate with raised TL (raising interference) that
  then misses — i.e. the gate inside the walk failed to reject it, OR a PA
  descent step widened the diff. The walk should only REJECT; if it adopted a
  miss, the in-walk gate has a hole.
- (H3) PA ≠ DM at the candidate: `candidate.priority_vec` is the walk's result
  PA, which may differ from the DM seed PA. If the walk explored a PA whose RTA
  for an important task exceeds threshold even at the seed TLs, and the in-walk
  gate didn't reject it, the loud-fail catches it.
- (H4) `worst_case_dag` itself: the caller-built worst-case DAG combines
  per-interval max-ET per task. If that DAG's `sp_params_worst` thresholds or
  the worst-case RTAs genuinely can't be met by ANY {pa, tl} (the taskset is
  infeasible at the WCET point), then the gate is RIGHT to fail — but then P0.8's
  generation-time gate + P0.6's gate should ALSO have rejected this taskset at
  generation. The discrepancy (BF/P0.10 green, INCR loud-fail) is the real bug
  signal: either INCR's worst-case DAG is OVER-conservative, or BF's gate is
  UNDER-conservative.

## Decisive question
**Why does the cross-interval worst-case DAG fail `ComputeSafeFallback`'s gate
when the SAME taskset passed P0.8 (generation) and BF (P0.10 §2) on the
per-interval DAGs?** The bug is the GATE DISAGREEMENT — one of the three gates
is wrong about this taskset. INCR's `ComputeSafeFallback` gate is the odd one out
(cross-interval worst-case vs per-interval), so the prime suspect is
over-conservative worst-case-DAG construction OR a seed that isn't actually
feasible on it.

## Next steps
1. Instrument the loud-fail: print `WorstCaseImportantTaskMissInfo` (already a
   helper) for the failing candidate — which task, miss_chance vs threshold, by
   how much.
2. Re-run the gate on the SEED {DM PA, tl_seed} before the walk — does the seed
   itself already miss? (Tests H1/H3: if seed misses, the
   feasibility-by-construction claim is false.)
3. If seed passes but candidate fails → the walk adopted a bad candidate (H2):
   add a guard rejecting any adopted candidate whose gate the in-walk check
   didn't catch.
4. Compare the worst-case DAG's per-task max-ET vs the per-interval DAGs BF/P0.8
   gated — is the worst-case DAG's RTA for the violating task genuinely higher,
   or is the construction inflating it?

## Files
- `sources/Optimization/OptimizeSP_TL_Incre.cpp:963-1039` — `ComputeSafeFallback`.
- `sources/Safety_Performance_Metric/SP_Metric.cpp:245-256` — self-contained
  `ImportantTasksMeetThresholds` (the loud-fail gate).
- `sources/Safety_Performance_Metric/SP_Metric.cpp:262-289` —
  `WorstCaseImportantTaskMissInfo` (diagnostic helper, already exists).
- `sources/TaskModel/WorstCaseDAG.cpp` — `BuildWorstCaseDagAcrossIntervals`
  (the caller-built worst-case DAG; verify its construction).
- `sources/Optimization/OptimizeSP_TL_Incre.h:339` — `ComputeSafeFallback` decl.

## Repro artifact
`simulation_experiments/optimizer_comparison/runs/
compare_against_bf_run_test_dur600_interval10_seed1000_tasks4x6/sim/
tasks6_dur600_interval10_seed1000/taskset_3/` (generator_config.json +
taskset_param.yaml + path_Et_task_*.txt + interval characteristics).

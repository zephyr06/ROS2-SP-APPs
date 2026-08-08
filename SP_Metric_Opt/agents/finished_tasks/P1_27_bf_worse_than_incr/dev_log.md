# P1.27 — BF Worse Than INCR_Reopt_10 — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-08-07

- Task folder created. Anomaly reported by user: in the comparison run
  `compare_against_bf_run_test_dur600_interval10_seed1000_tasks4/sim/tasks4_dur600_interval10_seed1000`,
  `comparison_summary.csv` shows `BF` Mean_SP_Metric = **0.908552** vs
  `INCR_Reopt_10` = **0.922016** — BF is ~1.5% WORSE, violating the `INCR ≤ BF`
  invariant (P0.2). BF also has a non-zero miss rate (0.005779; INCR_NO_FALLBACK
  / INCR_WCET = 0.000) and ~55× the exec time (0.097 s vs 0.0018 s).
- `taskset_arm_status.csv`: all 10 tasksets × 8 arms report `inst=0, status=OK`
  — no crash marker at the runner level (but a silent/0-byte failure path is
  not yet ruled out, P1.15).
- Five hypotheses filed in `goal.md` (BF time-limit truncation / P0.10 fallback
  firing / objective-metric mismatch / search-space restriction under P0.9 /
  stale artifacts). Next: Step 1 — localize the loss per-taskset via
  `interval_sp_metrics.txt`.

### Localization + root cause (user-confirmed)
- Gap is ENTIRELY `taskset_2`: BF 0.717297 vs INCR 0.855923 (other 9 tasksets
  BF ≥ INCR). Within taskset_2 the bimodal `0.527888` BF intervals are the
  RM-Fast fallback SP; INCR hits `0.954072` there.
- `taskset_2/BF/run.log`: BF search FINDS the high-SP plans (`Optimal SP is:
  0.793324`→`0.962717`, ≥ INCR's 0.954072) but the reported interval SP is
  `0.527888` (RM-Fast). So BF adopts the global SP-max plan, the POST-HOC
  `AdoptRmFastFallbackIfUnschedulable` gate (P0.10 §2) rejects it as
  unschedulable, and swaps it DOWN to RM-Fast — even though a slightly-lower-SP
  SCHEDULABLE plan (0.954072, which INCR finds via its DURING-WALK gate) exists.
  Root cause = hypothesis #2 (fallback firing) + #3-shape (gate runs post-search,
  not in-search). User confirmed: the gate must run IN-SEARCH at each leaf.

### Fix (TDD, git add-only — NOT committed)
- RED: copied `taskset_2/taskset_characteristics_interval_0.yaml` →
  `TaskData/test_p127_bf_incr_taskset2_i0.yaml`; added
  `TaskSetForTest_p127_taskset2_i0.BF_NotWorseThan_INCR_Reopt` in
  `testBF_w_TL.cpp` (runs `EnumeratePA_with_TimeLimits` vs
  `OptimizePA_Incre_with_TimeLimits::ReOptimizePeriodic`, asserts
  `bf.sp_opt >= incr.sp_opt`). Pre-fix: BF `sp_opt=1.976e-323` (RM-Fast,
  `BuildPriorityPlan` never sets `sp_opt`) < INCR `0.954072` → FAIL.
- GREEN: in `OptimizeSP_TL_BF.cpp` recursive `Optimize` leaf, gate adoption
  in-search — `if (res_cur.sp_opt > res_opt.sp_opt && ImportantTasksMeetThresholds(
  dag_tasks, sp_parameters, res_cur.priority_vec, time_limit_for_task))`. The
  4-arg gate overload (derives RTAs fresh) mirrors the post-hoc gate's call.
  Post-hoc `AdoptRmFastFallbackIfUnschedulable` KEPT as the safety net for the
  "no schedulable candidate" case (P0.8-certified tasksets always have one, so
  `res_opt` is never the empty sentinel in practice; `UpdateTaskSetPriorities`
  is empty-PA-safe regardless).
- Bit-identity: no `is_important` tasks → gate returns true vacuously →
  `sp_opt > INT_MIN && true` == original. 17/17 ctest green; release builds.

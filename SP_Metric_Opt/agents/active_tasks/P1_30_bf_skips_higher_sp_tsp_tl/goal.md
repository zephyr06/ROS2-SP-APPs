# P1.30 — BF fails to adopt a higher-SP plan that INCR finds (mid config)

> Spawned from P0.11 (real-world config eval). User flagged this as a BUG:
> "no i don't accept this bug... figure out why... maybe check why BF skipped it."

## Symptom (observed 2026-08-17, mid config `rw_baseline_tightened.yaml`)

BF is exhaustive over ALL 12 TSP time-limit options × all PAs. It should be a
superset of anything INCR can find (INCR uses the SAME discrete TL grid).
Yet on the mid config:

| optimizer | TSP TL | SP       | per-core PA ordering           | wall time |
|-----------|--------|----------|--------------------------------|-----------|
| BF        | 1100   | 4.97052  | SLAM>TSP (core0), MPC>RRT (c1) | 0.8128 s  |
| INCR      | 1200   | 4.98389  | SLAM>TSP (core0), MPC>RRT (c1) | 0.0363 s  |

- SAME per-core priority ordering in both. The ONLY difference is TSP's
  time limit (1100 vs 1200) and the resulting SP (INCR's is +0.013 HIGHER).
- BF's TL grid includes 1200 (it is in TSP's `performance_records_time`:
  150 250 300 450 850 900 950 1000 1050 1100 1200 1500). So BF MUST have
  visited the (SLAM>TSP, MPC>RRT, TSP_TL=1200) leaf — the exact plan INCR
  adopted — and either (a) computed a LOWER SP there than INCR did, or
  (b) failed to adopt it.
- Wall time 0.81 s << TIME_LIMIT (10 s) → the P1.14 shared-budget timeout
  did NOT cut the search short. `BFSharedBudgetCancelled()` was false.

## Hypotheses

- **H1 — TL baking / SP-eval inconsistency.** BF leaf bakes TL via
  `UpdateExtDistBasedOnTimeLimit` → `GetUnitExecutionTimeDist(tl)` (a UNIT /
  deterministic dist at exactly the TL). INCR's TL-walk incumbent SP may be
  computed via a DIFFERENT baking path (e.g. `ApplyTimeLimitsToTasksExecutionTime`,
  used by the 4-arg `ImportantTasksMeetThresholds`). If the two bakings yield
  different SP for the SAME (PA, TL), BF and INCR optimize different objectives
  → not an adoption bug, but an eval-inconsistency bug.
- **H2 — BF adoption bug.** BF evaluates SP=4.98389 at TL=1200 but fails to
  write it into `res_opt` (compare/keep logic). E.g. the in-search gate
  (`ImportantTasksMeetThresholds`) wrongly rejecting, or a strict `>` vs `>=`
  issue, or `SaveTimeLimits` overwriting.
- **H3 — PA mismatch at TL=1200.** BF's BEST PA at TL=1200 is a different PA
  (not SLAM>TSP / MPC>RRT) with lower SP, while INCR found SLAM>TSP / MPC>RRT
  at TL=1200. Would mean BF's per-leaf PA optimizer is itself suboptimal —
  unlikely (it's `OptimizePA_BruteForce`, exhaustive) but possible if budget-
  cut mid-leaf.
- **ELIMINATED — budget timeout.** 0.81 s < 10 s; not the cause.
- **ELIMINATED — P1.27 in-search gate.** No task is `important` in mid config
  → `ImportantTasksMeetThresholds` is VACUOUS (returns true). Gate cannot
  reject anything.

## Debug plan

1. Capture BF's full per-leaf SP trace for the mid config (debugMode or an
   instrumented print in `OptimizePA_with_TimeLimitsStatus::Optimize` at the
   leaf). For EACH (TL_TSP, best-PA) leaf, print TL + best-PA + SP.
2. Locate the TL=1200 leaf. Does SP=4.98389 appear?
   - If YES → H2 (adoption bug): why wasn't it adopted? Check the
     `res_cur.sp_opt > res_opt.sp_opt && gate` compare.
   - If NO (BF's TL=1200 SP < 4.98389) → H1: re-evaluate INCR's adopted
     (PA, TL=1200) plan through BF's EXACT leaf path
     (`UpdateExtDistBasedOnTimeLimit` + `OptimizePA_BruteForce`) and compare
     to INCR's `res_opt_.sp_opt`. Confirm the baking divergence.
3. If H1 confirmed: identify the two baking functions, diff their math, decide
   which is canonical (BF's unit-dist is the documented leaf eval).

## Scope / constraints
- Agents only `git add`; user commits.
- Do NOT mutate `all_time_records/task_characteristics.yaml`; use
  `TaskData/p0_11_variants/rw_baseline_tightened.yaml`.
- Reproduce on the mid config first; pertask/mpc_important are secondary.

## Status
- 2026-08-17 ROOT CAUSE FOUND (see dev_log.md). BF is CORRECT (per-leaf trace:
  TL=1200 max SP=4.96962 < TL=1100's 4.97052 → BF rightly adopted 1100; no
  skip, no timeout). The bug is in INCR: its RTA-cache SP-scoring path
  (`ObtainSP_Full_From_NodeRTAs` fed by `rta_cache_.Evaluate`) inflates TL=1200's
  SP to 4.98389 (impossible under canonical `EvaluateSPWithPriorityVec`, which
  caps at 4.96962). H1 (baking) ELIMINATED — both paths bake identically; the
  divergence is in the node RTAs (cache vs fresh `ProbabilisticRTA_TaskSet`).
  NOT yet fixed; NOT yet empirically confirmed at the RTA level. All prior
  INCR>BF P0.11 observations suspect. Potentially affects prod INCR SPs (PW).


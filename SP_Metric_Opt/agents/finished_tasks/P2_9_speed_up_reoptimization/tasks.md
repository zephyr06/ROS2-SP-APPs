# P2.9 — Tasks (working checklist)

> Performance hygiene on the reopt cold path. Prefer simple, low-risk levers
> first. Every lever verified by: (a) bit-identical SP where the lever is
> quality-preserving, OR (b) an explicit A/B where it is result-changing.

## Phase 0 — Ground the cost model (DONE)

- [x] **0a. Verify the 0.8s figure** against prod `INCR_Reopt_1` N=16
  `scheduler_execution_time.txt`. Confirmed: 29–65s/sim ÷ 60 ≈ 0.5–1.1s/act.
- [x] **0b. Trace the reopt path** to the dominant cost. Confirmed: the TL walk
  (`PerformCoordinateDescentForTaskConfigOpt` → `OptimizeSingleTaskTimeLimit`
  → eval lambda `:432-436`) re-runs the full `OptimizeFromScratch(K=2)` beam per
  trial TL, while the incremental path uses the cheaper sub-incremental eval.
- [x] **0c. Confirm parameters** (`sources/parameters.yaml`): `Granularity=10`,
  `ReoptimizationPeriod=10`, `IncrementalTimeLimitSearchPatience=0`,
  `ReoptimizationTimeLimitSearchPatience=1`, `Layer_Node_During...=2` (K=2),
  `TIME_LIMIT=1`.

## Phase 1 — Brainstorm lever catalogue (DONE)

See `dev_log.md` § "Lever catalogue & Master Lever Ranking" (Levers A–I). Ranked by leverage and quality preservation. Decision on which to implement PENDING user pick.

## Phase 2 — Implement chosen lever(s)

- [x] **2a. Lever A — design + flag + TDD.** Flag
  `ReoptimizationUseSubIncrementalWalk` (default OFF) added in `5dfd146e`;
  walk switch + cache re-arm landed in working tree (`PerformCoordinateDescent-
  ForTaskConfigOpt`). TDD: 2 new tests in `CounterDispatcherSynthetic`
  (`testIncreOpt_w_TL.cpp`) — OFF asserts `subincremental_calls==0` (legacy
  walk uses ScratchOrIncre), ON asserts `subincremental_calls>0` (walk routes
  through SubIncremental) + baseline beam still via ScratchOrIncre. 53/53
  testIncreOpt_w_TL green. NOT bit-identical (reopt PA search can be
  non-unimodal); gated default OFF.
- [ ] **2b. Bit-identical-or-A/B verification** at N=16 (`INCR_Reopt_1` + `INCR_Reopt_10`)
- [ ] **2c. Re-measure per-activation ET** vs the 0.8s baseline

## Phase 3 — Closeout

- [ ] **3a. Update `overall_tasks.md` + memory** with the realized speedup + any
  accepted accuracy trade-off.

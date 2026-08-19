# P0.11 — Tasks (working checklist)

> See `goal.md` for scope. Evaluation task, NOT bug-finding.

## Setup / understanding
- [x] Read repo README + SP_Metric_Opt README
- [x] Locate the real-world experiment yaml: `all_time_records/task_characteristics.yaml`
- [x] Confirm task table (TSP/MPC/RRT/SLAM, processorId partition, sp_threshold/sp_weight)
- [x] Confirm `sp_threshold` semantics: deadline-miss-prob threshold; lower = stricter
- [x] Confirm BF binary: `AnalyzePriorityAssignment` (`--file_path`/`--output_file_path`); bigger int = higher priority
- [x] Confirm TSP & SLAM share processorId=0 (the competing pair for the hypothesis)

## Config variants (scratch copies in TaskData/, do NOT mutate the original)
- [x] Create baseline copy of the real-world yaml into the task scratch dir
- [x] Tightened-threshold variant(s): all thresholds -> 0.1 (primary lever)
- [x] SLAM-ET-low variant (SLAM mu=100, sigma=20)
- [x] SLAM-ET-high variant (SLAM mu=1200, sigma=300)
- [x] Cross-product with threshold 0.01 / 0.9 sweep (low+high SLAM-ET) — see "Threshold sweep" below

## Run BF optimizer FIRST
- [x] Confirm a working BF binary exists (`release/tests/AnalyzePriorityAssignment`)
- [x] BF run on tightened-threshold baseline (SLAM mu=361)
- [x] BF run on SLAM-ET-low variant (SLAM mu=100)
- [x] BF run on SLAM-ET-high variant (SLAM mu=1200)
- [x] Capture output priority assignments + TSP time limit

## Verify the hypothesis — BF: CONFIRMED
- [x] SLAM-ET-low  -> BF assigns TSP HIGHER priority than SLAM (TSP=6 > SLAM=5)
- [x] SLAM-ET-high -> BF assigns TSP LOWER priority than SLAM (TSP=4 < SLAM=5)
- [x] Record verdict per regime (see dev_log.md)

## Run incremental optimizer for comparison
- [x] Write clean single-config INCR driver (`tests/AnalyzePriorityAssignmentIncremental.cpp`) + enable CMake target
- [x] Build `AnalyzePriorityAssignmentIncremental` in release (classifier outage cleared; build OK)
- [x] Run INCR on the same variants
- [x] Compare INCR priority ordering vs BF (PA-level) → low: differ (BF TSP>SLAM, INCR SLAM>TSP); mid+high: match (SLAM>TSP)

## Measure SP metrics (user asked: "difference in SP when SLAM ET low")
- [x] Add one-line `Adopted SP` print to BF driver (`res.sp_opt`) + INCR driver (`opt.CollectResults().sp_opt`); rebuild
- [x] Low regime: BF SP=4.98673, INCR SP=4.98673 → **IDENTICAL** (INCR NOT suboptimal; both PA choices SP-optimal)
- [x] Mid: BF=4.97052, INCR=4.98389 (INCR higher — P1.27 BF in-search gate rejects SP-max plan)
- [x] High: BF=4.80810, INCR=4.80810 (identical)
- [x] CORRECTION logged: prior "INCR fails low regime / suboptimal" verdict was inferred from PA-only, withdrawn — SP-equal in low regime

## New per-task-threshold config (user: TSP=.5, MPC=.01, RRT=.05, SLAM=.1)
- [x] Create `TaskData/p0_11_variants/rw_pertask_thresholds.yaml` (baseline SLAM ET mu=361)
- [x] BF: SP=4.96487, PA TSP=6>SLAM=5; INCR: SP=4.98181, PA TSP=6>SLAM=5 (both pick TSP>SLAM)
- [x] Record in dev_log.md

## Unit test (2026-08-18) — codifies the evaluation as a regression test
- [x] New `tests/test_real_world_robot_config.cpp` (gtest, DEBUG `check.SP_OPT`)
- [x] Self-contained scenario yamls in `TaskData/test_real_world_robot_config/`
      (low/high SLAM-ET × thr 0.1/0.01; 0.9 excluded — not a realistic case)
- [x] BF asserts the priority-swap hypothesis (low→TSP>SLAM, high→TSP<SLAM)
- [x] INCR asserts high regime matches hypothesis; near-optimal SP (≤ BF + tol,
      ≥ BF − tol) on all 4 — P1.30 inflation-regression guard
- [x] Build + run: 18/18 `check.SP_OPT` (new test ~74s, 4 cases all PASSED)

## Open (out of scope)
- [ ] Is INCR's higher-SP mid/pertask plan actually gate-satisfying, or did it skirt the important-task gate? (eval `ImportantTasksMeetThresholds` on INCR's adopted PA+TL)

## Report
- [x] Write findings into `dev_log.md`
- [x] Append milestone line to top-level `agents/dev_log.md`
- [x] Report results back to user

## Threshold sweep (0.9 / 0.1 / 0.01 × low/high SLAM-ET) — 2026-08-18

Variants: `TaskData/p0_11_variants/rw_slam_et_{low,high}_thr{0p9,0p01}.yaml`
(uniform sp_threshold across all 4 tasks; the 0.1 variants already existed).
BF = `release/tests/AnalyzePriorityAssignment`. Output: bigger int = higher priority.

| SLAM-ET | thr | TSP pri | SLAM pri | TSP-vs-SLAM   | hypothesis? | SP      | Gate |
|---------|-----|---------|----------|---------------|-------------|---------|------|
| low  (mu=100)  | 0.9  | 4 | 2 | TSP > SLAM | PASS (TSP higher)    | 4.65315 | PASS |
| low  (mu=100)  | 0.1  | 6 | 5 | TSP > SLAM | PASS (TSP higher)    | 4.98673 | PASS |
| low  (mu=100)  | 0.01 | 4 | 1 | TSP > SLAM | PASS (TSP higher)    | 4.98785 | PASS |
| high (mu=1200) | 0.9  | 4 | 1 | TSP > SLAM | **FAIL** (should be TSP lower) | 4.08337 | PASS |
| high (mu=1200) | 0.1  | 4 | 5 | SLAM > TSP | PASS (TSP lower)     | 4.80810 | PASS |
| high (mu=1200) | 0.01 | 1 | 2 | SLAM > TSP | PASS (TSP lower)     | 4.81014 | PASS |

Verdict:
- LOW SLAM-ET: TSP>SLAM at ALL thresholds. Hypothesis robust across the sweep.
- HIGH SLAM-ET: swap only appears at STRICT thresholds (0.1, 0.01 → SLAM>TSP). At the
  lenient 0.9 threshold BF picks TSP>SLAM (the OPPOSITE of the hypothesis) — concrete
  proof that "thresholds too high" misaligns the SP metric with the safety-correct
  ordering. BF is global optimum (brute force); at 0.9 the SP-max plan genuinely is
  TSP>SLAM because the lenient threshold under-penalises SLAM starvation.
- SP values are NOT comparable across thresholds: `SP_Func` (SP_Metric.h:44) normalises
  between PenaltyFunc(1,thr)=-0.01·exp(10·|1-thr|) (→≈-199 at thr=0.01) and
  RewardFunc(0,thr)=log(thr+1); the plunging penalty floor at low thr maps almost any
  near-schedulable miss-prob to ≈1.0, so SP saturates upward as thr tightens — a
  metric-normalisation artifact, not "a better plan". The PA ordering is the clean signal.
- Gate column vacuous (worst id=-1): `is_important` not engaged in this binary path, so
  the in-search gate is vacuous-true; BF here is pure SP-max (no important-task pruning).

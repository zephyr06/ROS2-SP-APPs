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
- [ ] (optional) Cross-product with threshold 0.01 / 0.9 sweep

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

## Open (out of scope)
- [ ] Is INCR's higher-SP mid/pertask plan actually gate-satisfying, or did it skirt the important-task gate? (eval `ImportantTasksMeetThresholds` on INCR's adopted PA+TL)

## Report
- [x] Write findings into `dev_log.md`
- [x] Append milestone line to top-level `agents/dev_log.md`
- [x] Report results back to user

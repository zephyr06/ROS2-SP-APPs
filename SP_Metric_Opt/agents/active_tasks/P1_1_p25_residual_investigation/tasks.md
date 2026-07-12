# P1.1 — Tasks (working checklist)

> See `goal.md` for scope. **Investigation, not implementation** — do not
> re-frame or implement Fix C until the gate is resolved.

## Gate: reconcile 2-vs-8 changed-task-count discrepancy
- [x] Rebuild with `debugMode:1`
- [x] Run N=8 `taskset_0` INCR_P10 (N=10 taskset from the original framing is gone;
      N=8 taskset_0 is the available P25 taskset — same principle)
- [x] Grep `[INCR-ET-DBG]` / `[INCR-NDIFF-DBG]` from the log
      (trace: `et_repro/dbg_trace_ts0_new/P10/stderr.txt`)
- [x] Compare instrumented count (5) vs ground-truth YAML diff (3: task ids 5, 6, 7)
- [x] Document the reconciliation in `dev_log.md` — **RECONCILED 2026-07-07**
      Root cause: `FindTaskWithDifferentEt` diffs TL-applied point dists
      (incumbent adopted-TL vs descent current-TL), not the underlying YAML
      Gaussians. 3 false positives (tasks 0,1,2 — perf-pair, YAML identical,
      flagged because adopted TL ≠ descent TL) + 1 false negative (task 5 —
      perf-pair, YAML mu changed, masked because adopted TL == descent TL).
      Only gaussian-only tasks (3,6,7; TL=-1 → no-op) compare the raw Gaussian.

## Confirm Fix D inert
- [x] Re-verify `FiniteDist::approx_equal` (`Probability.cpp:345-357`) — DOES use
      its tolerance param, but has ZERO production callers (only testProbability).
      The live comparison is `operator!=` (`Probability.cpp:353-368`) which
      hardcodes tolerance=1e-1 in its own loop and does NOT call `approx_equal`.
- [x] Record "GetAvgValue band is inert today" as a confirmed finding —
      there is no live code path applying a tunable tolerance to ET-dist
      comparison. Wiring the band requires making `FindTaskWithDifferentEt`
      call `approx_equal` (or adding an explicit `GetAvgValue` band check).

## Equal-radii A/B (sibling "NEW TASK") — DROPPED
- [~] Superseded by the yardstick correction. The residual is precisely "the
      incremental descent cold-starts from the Gaussian-mean TL instead of the
      carried adopted TL" (see `dev_log.md` YARDSTICK CORRECTION section).
      Equal-radii A/B targeted per-variation scoring / radius asymmetry, which is
      the wrong lever — radii do not touch the descent start TL. Running it would
      not move the residual.

## Decision — START INCREMENTAL DESCENT FROM CARRIED ADOPTED TL (user-approved)
- [x] Decision recorded 2026-07-07 (user: "i agree, that needs to be fixed ...
      we need to initialize time limit in that way"). Lifts the 2026-07-06
      implement-only hold FOR THIS LEVER ONLY. Fix C (per-variation scoring) and
      Fix D (GetAvgValue band) remain NOT in scope — both are the wrong lever.
- [x] TDD: added a failing test that reproduces the false positive (perf-pair
      task, YAML identical between intervals, adopted TL ≠ Gaussian-mean TL →
      currently flagged by `FindTaskWithDifferentEt`, should NOT be).
      `OptimizeIncre_w_TL_StartsDescentFromCarriedAdoptedTL` (testIncreOpt_w_TL.cpp:952).
      Verified RED under the reverted (InitializeTimeLimitsFromETConfig) source,
      GREEN under the fix.
- [x] Implement the change: `OptimizeIncre_w_TL` (`OptimizeSP_TL_Incre.cpp:387`)
      starts descent from `ReconstructTimeLimitVecFromResOpt()` instead of
      `InitializeTimeLimitsFromETConfig()`. Plus the edge-case guard (lines
      400–410): a task that had a perf pair in N−1 but lost it in N does NOT get
      a stale adopted TL applied as a point dist — the carried TL is intersected
      against the current option set (`time_limit_option_for_each_task_`, rebuilt
      at line 372); forced to -1 when the task no longer has pairs or the carried
      TL is no longer an option.
- [x] Confirm the failing test now passes; run the full `testIncreOpt_w_TL`
      suite + `ctest`. **42/42 testIncreOpt_w_TL green, 16/16 ctest green.**
      One stale test expectation updated (NOT a source bug):
      `PerformCoordinateDescent_SkipsMinusOneOnlyTaskInMixedSet` had asserted
      exactly 4 evals (calibrated to the old Gaussian-mean TL=600 start, which
      walked 600→400→800→1000). Under the fix the descent starts at the adopted
      TL=1000 (the optimum/boundary on the fixture's monotonic landscape), so
      the walk is 2 evals (1000→800 backward, no forward). Updated to assert the
      structural invariants (T_noise skipped → no +1; no fallback; ≥1 baseline)
      decoupled from the start-TL-specific walk shape.
- [ ] Re-run the INCR_P10 N=8 taskset_0 probe; confirm `ndiff` drops from 5
      toward the corrected ground truth of 2 (only gaussian-only tasks 6,7).
- [ ] Record result in `dev_log.md`; milestone to top-level `dev_log.md`.

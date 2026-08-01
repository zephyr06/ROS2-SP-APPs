# P2.18 — Tasks (working checklist)

> See `goal.md` for the root cause + the disarmed-beam invariant the P0.7 gate
> broke. P0.7-gate regression; NOT P2.17, NOT P0.6, NOT P0.8.

## 0. Diagnose — DONE
- [x] Reproduce deterministically (debug + release) on `compare_against_bf` N=4
      taskset_2 / INCR_WCET.
- [x] Source-level backtrace (debug, `catch throw`): throw at
      `RTA_Cache.cpp:358` (`ComputeTaskSetDifference`, `|diff|>1` invariant) <-
      `Evaluate` (`:465`) <- `SeedBaselineAndArmCache` (`:532`, Reopt branch).
- [x] Confirm NOT P2.17: seed=1040 (gate ON), INCR_Reopt_10 finished OK on the
      crashed taskset → `ComputeSafeFallback` did not throw.
- [x] Confirm NOT P0.6 loud-fail: throw site is `RTA_Cache`, not
      `OptimizeSP_TL_Incre.cpp:1030`.
- [x] `git blame` `UpdateRecords:214-223` → P0.7/P0.6-gate code (`500665d5`,
      `c87ae0d4`, `a2be3876`).

## 1. Design — DONE (D1/D2 settled with user 2026-08-01)
- [x] D1 = (a) guard-the-call: add `rta_cache_active_` to the gate predicate
      at `UpdateRecords:220` → `enable_fallback_use_ && rta_cache_active_ &&
      !BFSharedBudgetCancelled()`. Mirrors `CommitIncumbent:920`. (b) rejected.
- [x] D2 = (a) gate inert in the from-scratch beam (D1 gives this free); the
      beam's final `res_opt_` is gated by `AdoptFallbackIfUnschedulable`
      (`:758`, cache-free RTA, rolls back to `safe_fallback_`).
- [x] Code follows the already-approved D1=(a); plan-approval step superseded
      (D1/D2 settled pre-code). User reviews the staged change, not a plan doc.

## 2. TDD the fix (RED first) — DONE
- [x] Failing unit test reproducing the throw: `P07GateArmsCacheMidBeamSynthetic.
      ReOptimizePeriodic_GateDoesNotArmCacheMidFromScratchBeam`. A `ControlledBeamOpt`
      subclass injects a from-scratch beam triple (perf-first PA, high TL, gate-
      infeasible, >1-TL diff from the DM seed) then calls the REAL `UpdateRecords` →
      the gate's `rta_cache_.Evaluate` arms the cache via `Initialize` mid-beam →
      gate REJECTs → post-beam re-arm `Evaluate` (`:539`) diffs seed-vs-beam-champion
      >1 → `ComputeTaskSetDifference` throws. Pinned via `EXPECT_NO_THROW`.
- [x] Confirm RED: reverted the one-line predicate (dropped `rta_cache_active_`),
      rebuilt, ran the test → throws `std::runtime_error` "RTACache::
      ComputeTaskSetDifference: candidate differs from champion by more than one
      task" (the exact SIGABRT-run-path signature, reproduced in-process).

## 3. Implement the fix (GREEN) — DONE
- [x] Minimal D1=(a) change: 1 source line (gate predicate) + `ArmRtaCacheForTest()`
      test-only helper (header). No >3 source files (2: `OptimizeSP_TL_Incre.cpp/.h`).
- [x] Failing test now passes; cache stays un-armed through the from-scratch beam
      (`:539` Evaluate is a safe `Initialize` again — gate inert during the beam).
- [x] No regression: 116/116 `testIncreOpt_w_TL` (115 + 1 new P2.18); 16/17 ctest —
      the sole failure `OrchestratorTest.CFS_RunOrchestrator_Binary` is PRE-EXISTING
      and environment-related (shells out to a missing binary; identical on clean HEAD
      via `git stash`; `testScheduleSimulate.cpp:976`, unrelated to P2.18's files).

## 4. Verify on the real run path
- [x] Re-run the crashed arm directly: `release/tests/RunOrchestrator ... INCR_WCET`
      on taskset_2 → exit 0, all 60 intervals' SP metrics + fallback log written
      (all `kept_walk`, zero rejects). Before: 0-byte run.log, SIGABRT at interval 0.
      Evidence: `run.log.crash_p218` (0-byte) vs full `INCR_WCET/INCR_WCET/` output.
- [x] Re-run `compare_against_bf.json` test_mode (N=4) → no crash. Resume-mode
      re-run (PID 1636743, 13:02–13:06) re-ran tasksets 3–9 fresh: all wrote
      non-empty run.logs with clean success signatures (e.g. taskset_3/INCR_WCET
      `Average SP Metric: 0.901639`, no abort); taskset_2/INCR_WCET reused the
      valid output from the direct verify run (exit 0 + 60 intervals). The stale
      `crash_report.txt` + 0-byte taskset_2/INCR_WCET log are pre-fix artifacts
      (mtime 11:08, NOT overwritten by this re-run). No arm tripped the
      `|diff|>1` path.
- [ ] Re-run `measure_p07_penalty.json` N=[4,6,8] → full P0.7 SP-penalty A/B
      (unblocks P0.7's deferred N=6/8 data). (PENDING — deferred; P2.18 fix
      unblocks it. Separate re-run, not part of P2.18 verification.)

## 5. Records + memory
- [x] `dev_log.md` (this folder + top-level `agents/dev_log.md`).
- [x] Memory file `p218-p07-gate-arms-rta-cache-mid-beam.md` + MEMORY.md index
      (updated "FILED NO code" → "FIX LANDED + VERIFIED").
- [x] `git add` staged; user reviews (no commit). Staged: `OptimizeSP_TL_Incre.cpp/.h`
      + `testIncreOpt_w_TL.cpp` + P2.18 folder + P0.7 records + top-level dev_log.

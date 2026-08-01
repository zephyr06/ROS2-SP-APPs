# P0.7 — Tasks (working checklist)

> See `goal.md` for the two-trigger design + budget asymmetry. Depends on P0.6
> (`safe_fallback_`, worst-case-DAG → cross-interval safe) + P0.8. D1–D5 resolved
> 2026-07-27; **D6 + D7 OVERTURNED 2026-07-31** (reject-and-continue, not halt;
> schedulability decides, not SP).

## 0a. 4-directive refactor — LANDED
- [x] Merge `enforce_important_task_gate_` into `enable_fallback_use_` (default ON).
- [x] Rename `ShouldShortCircuitOnETJump` → `SkipOptOnETJump`.
- [x] `AdoptFallbackIfUnschedulable` throws when enabled but `!HasSafeFallback()` / `!IfInitialized()`.
- [x] Rescue-then-verify: re-verify adopted fallback; second gate-fail → throw.
- [x] +2 tests; flag refs re-pointed; 3 `OptimizePureIncremental_*` tests set flag false. 17/17 green.

## 0. Design decisions — RESOLVED (D6/D7 OVERTURNED)
- [x] D1: important = top-50% by `sp_weight`, `bool Task::is_important`.
- [x] D2: analytic DDL-miss-chance (`GetDDL_MissProbability`) — no P2.6 dep, no drift.
- [x] D3: per-interval (not sticky).
- [x] D4: trust the worst-case-DAG certificate — trigger (a)'s direct swap is sound.
- [x] D5: INCR-family modes only.
- [x] D6 OVERTURNED: ~~HALT~~ → reject-and-continue (walk never interrupted).
- [x] D7 OVERTURNED: ~~higher-SP winner~~ → schedulability decides; fallback adopted only as post-walk backstop.

## 1. Trigger (a) — ET-jump detector — LANDED
- [x] TDD: 4 `DetectETJump` tests (trip ≥1.5×, no-trip <1.5×, exact-boundary, decrease never trips).
- [x] Implement pure `DetectETJump(old_dag, new_dag, ratio_threshold=1.5)` (free fn,
      per-task scan of `execution_time_dist.GetAvgValue()`, trip on first ratio ≥ threshold).
- [x] Wire at TOP of both dispatchers (before `AbsorbUpdatedDAG`), skip interval 0. On trip:
      adopt fallback + absorb dag + skip walk. 4 dispatcher tests. 17/17 green.

## 2. Trigger (b) — reworked around the D6/D7 overturn
### 2a. Remove pre-overturn dead code — DONE
- [x] Remove `enforce_online_halt_gate_` + `online_halt_requested_` members.
- [x] Revert `UpdateRecords` to reject-and-continue; remove halt loop-breaks.
- [x] Remove `CompareAndPickWinnerAgainstSafeFallback`. Revert ghost-SP extension.
- [x] Drop 6 pre-overturn tests (105→99). grep-verified zero references. 17/17 green.

### 2b. During-walk gate (reject-and-continue) — LANDED
- [x] Master flag `enable_fallback_use_ = true` (default ON = final prod state; user
      mandate). Gates trigger (a) + arms the during-walk gate + gates the 2c backstop.
      Measurement arm sets it false (MEASUREMENT-ONLY, not a prod toggle).
- [x] Gate trigger (a)'s short-circuit behind the flag in both dispatchers.
- [x] Arm the during-walk gate online via the flag (`UpdateRecords` + ghost-SP suppression).
      Offline `ComputeSafeFallback` forces the flag true on its throwaway sibling →
      certificate holds even in the measurement arm.
- [x] New test `GateWiring_OnlineArm_RejectsSpBetterThresholdViolatingCandidate`;
      `GateWiring_FlagOff_*` sets flag false (measurement arm). 17/17 green.

### 2c. Post-walk backstop — LANDED
- [x] `AdoptFallbackIfUnschedulable`: gated by flag + `HasSafeFallback()` + `IfInitialized()`;
      reads final `res_opt_`; self-contained `ImportantTasksMeetThresholds`; FAIL →
      `AdoptSafeFallbackAsIncumbent` + return true; PASS → return false (keep walk).
- [x] 2 TDD tests (adopt-on-fail, keep-on-pass) + `SetSafeFallbackForTest`. 17/17 green.

## 3. Wire into dispatchers + ET-bracket — DONE
- [x] Backstop wired into both dispatchers after the walk, before `return opt_pa_`.
- [x] Flag default TRUE = shipped state; checked inline at the 3 trigger sites.
- [x] ET-bracket reframed: both run INSIDE (trigger (a) needs internal `dag_tasks_`;
      backstop is part of the scheduler decision). (a) skips walk on trip (reduces time);
      backstop adds one RTA eval only when it fires.
- [ ] Confirm `scheduler_execution_time.txt` vs P0.6 baseline (expect small prod-arm rise;
      measurement arm matches P0.6).

## 4. Mode gating (D5) + logging — LANDED
- [x] D5 VERIFIED by construction (triggers only in INCR dispatchers; orchestrator routes
      only INCR-family arms in).
- [x] `interval_fallback_log` recording (optimizer side): per-interval `IntervalFallbackOutcome`
      (`interval_idx`, `et_jump_short_circuited`, `during_walk_reject_count`,
      `backstop_verdict` + culprit on adopt). Gate stays bool-only; culprit via NEW
      `WorstCaseImportantTaskMissInfo` (SP_Metric). Record sites at both dispatcher tops,
      (a) early-return, (b-i) `UpdateRecords` reject, (b-ii) `AdoptFallbackIfUnschedulable`
      (culprit before rescue overwrites `res_opt_`). 5 `FallbackLog_*` tests.
- [x] File write: `FormatIntervalFallbackLogCsv` (6 tests) + `GetIntervalFallbackLog()`
      on `FixedTaskPrioritySchedulingOrchestrator`; `tests/RunOrchestrator.cpp` writes
      `interval_fallback_log.txt` (mirrors `safe_fallback_compute_time.txt`). Header-only
      for non-INCR modes. NOT a ctest (file I/O) — covered by step 5 smoke run.

## 5. Verification + records
- [x] `cmake --build build_test --target check.SP_OPT --clean-first -j5` green (17/17 ctest,
      115 tests in `testIncreOpt_w_TL`).
- [ ] Smoke run: confirm `interval_fallback_log.txt` produced at small N (header-only if no
      trip, populated if a constructed trip case).
- [ ] `dev_log.md` (this folder + top-level) + memory updated.
- [ ] `git add` staged; user reviews (no commit).
- [x] `enable_fallback_use_` default ON (= final prod state).
- [ ] **SP-penalty measurement (MEASUREMENT-ONLY):** prod (flag ON) vs measurement (flag
      OFF) at matched N; report the SP delta. Flag default stays ON after this.

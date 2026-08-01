# P0.7 Fall-Back Mechanism — Dev Log

> Chronological working log. On task completion, append a one-line milestone to the
> top-level `agents/dev_log.md` (the canonical narrative).

## 2026-07-26 — scaffolded

Task scaffolded from the user's fall-back design. 5 open decisions (D1–D5) recorded in
`goal.md`. D2 (analytic (α) vs empirical (β) miss rate) is THIS task's decisive fork;
recommendation (α). Insertion point grounded: `SimulateInterval`, between
`DeterminePrioritiesAndBudgets` and the RTDA rollout, before the SP push. Awaiting D1/D2
resolution + P0.6 + P0.8.

## 2026-07-27 — final design lock

- **Two triggers (user):** (a) ET-jump BEFORE optimization; (b) during-walk guard.
- **Budget asymmetry (user):** online is the CHEAP counterpart to P0.6's offline walk.
- **D2 RESOLVED (α):** `GetDDL_MissProbability` vs `sp_threshold` — faithful, no drift.
- **ET-compare semantics (user):** compare Gaussian `et_mean` (distribution parameter,
  deterministic) per task new-vs-old, 1.5× threshold — NOT the empirical job mean.
- **D3/D4/D5/D6/D7 RESOLVED** (D3 per-interval; D4 trust P0.8's guarantee; D5 INCR-family
  only; D6 HALT; D7 higher-SP winner).
- **A/B purpose (user):** quantify the SP penalty of adding the fall-back; USER re-runs prod.
- All D1–D7 resolved; next: implement (a) → (b), gated on P0.6's `safe_fallback_`. No code yet.

## 2026-07-31 — trigger (a) landed (TDD green, NOT committed)

- **`DetectETJump`** (free fn, `OptimizeSP_TL_Incre.{h,cpp}`): pure + stateless, per-task
  scan of `execution_time_dist.GetAvgValue()`, trips on first new/old ratio ≥ 1.5×. Skips
  degenerate (et_old≤0) dists; a decrease never trips. 4 unit tests.
- **Dispatcher wiring:** `ShouldShortCircuitOnETJump` (false at interval 0, else
  `DetectETJump`) + `AdoptSafeFallbackAsIncumbent` (re-scores `safe_fallback_`'s {PA,TL}
  under the current dag via `EvaluateSPWithPriorityVec`, commits via `CommitIncumbent`).
  Wired at the TOP of BOTH dispatchers, BEFORE `AbsorbUpdatedDAG`. On trip: absorb → adopt
  → `count++` → return. 4 dispatcher tests. 17/17 ctest green.

## 2026-07-31 — D6 + D7 OVERTURNED by user

User: "the new rule wins, i no longer want D6 as i think D6 performs worse, D7 breaks
incremental optimization so efficiency may be worse. we still need to check important
tasks' schedulability during incremental search if we find a challenger with better SP
than champion."
- **D6 (HALT) — DEAD:** walk runs uninterrupted, finishes on its own.
- **D7 (SP-winner) — DEAD:** no `CompareAndPickWinnerAgainstSafeFallback`; SP no longer decides.
- **New trigger (b-i):** reject-and-continue (refuse unsafe challengers, keep walking) —
  same response shape as P0.6's offline gate.
- **New trigger (b-ii):** post-walk backstop — gate the FINAL `res_opt_`; fail → adopt
  `safe_fallback_`; pass → keep walk even if fallback would have higher SP. **Schedulability
  decides, not SP.**

## 2026-07-31 — step 2a (dead code removed), 2b+2c+3 (gate + backstop + wiring)

- **2a:** removed all pre-overturn trigger-(b) dead code (`enforce_online_halt_gate_` +
  `online_halt_requested_`; `CompareAndPickWinnerAgainstSafeFallback`; the halt loop-breaks).
  Dropped 6 pre-overturn tests (105→99). grep-verified zero references. 17/17 green.
- **2b (during-walk gate):** armed online via NEW master flag `enable_fallback_use_`
  (default TRUE = shipped fully-enabled per user mandate). The gate predicate
  (`UpdateRecords`) + ghost-SP suppression (`OptimizeIncreSingleTask`) OR'd with
  `enforce_important_task_gate_` initially; later merged (see 4-directive refactor).
  `ComputeSafeFallback`'s throwaway `fallback_solver` forces the flag true so the offline
  safe-fallback walk ALWAYS gates (certificate holds even in the measurement arm). New test
  `GateWiring_OnlineArm_RejectsSpBetterThresholdViolatingCandidate`.
- **2c (post-walk backstop):** `AdoptFallbackIfUnschedulable` — gated by
  `enable_fallback_use_` + `HasSafeFallback()` + `IfInitialized()`; reads final `res_opt_`
  {pa,tl}; runs self-contained `ImportantTasksMeetThresholds` overload (fresh RTAs, no
  shared cache live post-walk); FAIL → `AdoptSafeFallbackAsIncumbent` + return true; PASS →
  return false. 2 TDD tests + `SetSafeFallbackForTest` accessor.
- **3 (dispatcher wiring):** backstop wired into BOTH dispatchers after the walk, before
  `return opt_pa_`. ET-bracket reframed (both run inside — see `goal.md` "ET bracket").

## 2026-07-31 — 4-directive refactor

User: "merge them into one. rename ShouldShortCircuitOnETJump -> SkipOptOnETJump.
AdoptFallbackIfUnschedulable throw an error if fallback is enabled but not initialized.
similarly, if important tasks don't meet thresholds but fall back is needed, throw an error."
1. **MERGE flags:** `enforce_important_task_gate_` DELETED; surviving `enable_fallback_use_`
   (default ON). 2 OR'd predicates simplify to it alone.
2. **RENAME** `ShouldShortCircuitOnETJump`→`SkipOptOnETJump`.
3. **THROW** when enabled but `!HasSafeFallback()`; `!IfInitialized()` also a throw.
4. **Rescue-then-verify:** gate-fail KEEPS the rescue, then re-verifies the adopted fallback;
   a second gate-fail → throw (certificate violation).
+2 tests (throw-on-no-fallback, throw-on-rescue-also-fails); flag refs re-pointed; 3
`OptimizePureIncremental_*` mechanics tests set `enable_fallback_use_=false`. 17/17 green.

## 2026-08-01 — step 4 (D5 verify + interval_fallback_log)

- **D5 mode-gating = VERIFIED by construction (no code):** triggers live only in
  `Optimize_w_TL_ScratchOrIncre` + `OptimizePureIncremental`; orchestrator routes only
  INCR-family arms into them.
- **`interval_fallback_log` recording (optimizer side):** per-interval SUMMARY (not
  per-rejection). `IntervalFallbackOutcome` per dispatch call: `interval_idx`; (a)
  `et_jump_short_circuited`; (b-i) `during_walk_reject_count` (a count); (b-ii)
  `backstop_verdict` (kNone/kKeptWalk/kAdoptedFallback) +, on adopt only, culprit
  `task_id`/`miss_chance`/`threshold`. Gate stays bool-only; NEW diagnostic
  `WorstCaseImportantTaskMissInfo` (SP_Metric) derives the backstop's culprit (same bake+
  prioritize+RTA path as the gate, returns worst-ratio violator instead of bool). Record
  sites: entry-push at both dispatcher tops; (a) on early-return; (b-i) `++` in
  `UpdateRecords`' reject; (b-ii) verdict+culprit in `AdoptFallbackIfUnschedulable`
  (culprit captured BEFORE the rescue overwrites `res_opt_`). 5 `FallbackLog_*` tests.
- **File write:** `FormatIntervalFallbackLogCsv` (testable free fn, 6 tests) +
  `GetIntervalFallbackLog()` on `FixedTaskPrioritySchedulingOrchestrator` (mirrors
  `GetSafeFallbackComputeTime`); `tests/RunOrchestrator.cpp` `main` writes
  `interval_fallback_log.txt` (mirrors the `safe_fallback_compute_time.txt` sibling write —
  NOT `ExportResults`, which can't reach the private `incr_optimizer_`). Empty log →
  header-only file (non-INCR modes). Clean-rebuild ctest: 17/17 green (115 tests in
  `testIncreOpt_w_TL`).

## NEXT

Step 5: smoke run at small N to confirm `interval_fallback_log.txt` is produced (header-only
if no trip, populated if a constructed trip case) → SP-penalty measurement (prod ON vs
measurement OFF at matched N; report the SP delta to the user).

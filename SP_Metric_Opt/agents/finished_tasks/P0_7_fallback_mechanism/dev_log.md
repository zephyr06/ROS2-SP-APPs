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

## 2026-08-01 — step 5 plan locked (two decisions)

Resolved the step-5 A/B exposure gap + sequencing with the user:

- **Decision 1 — A/B exposure = Option A (new `INCR_NO_FALLBACK` mode).** Measurement arm =
  a new `INCR_NO_FALLBACK` mode string that flips `enable_fallback_use_=false`. Mirrors the
  exact `INCR_NO_TL`/`INCR_WCET` idiom (mode string → `GlobalVariables` bool, save/restore in
  the dispatcher) — the mode string is the only per-arm runtime channel the Python argv exposes
  (`run_sim_experiments.py` builds a FIXED 5–6-arg list). Rejected env var (breaks the idiom,
  doesn't flow through the fixed argv) and prod-vs-old-commit (the old commit lacks P0.6's
  `safe_fallback_` compute → would conflate P0.6 + P0.7 in scheduler-ET and SP).
- **Decision 2 — smoke run FIRST (validate committed code before adding the toggle).** Step 1
  runs the committed P0.7 code on `test_mode` (`INCR_Reopt_10`, N=4/6, 2 tasksets) to confirm
  `interval_fallback_log.txt` is produced + nothing crashes — BEFORE layering the A/B toggle on
  top. A smoke-run failure means a P0.7 bug to fix first, not a toggle-introduced problem.

Step 2 = TDD the `GlobalVariables::enable_fallback_use` (default true) + `INCR_NO_FALLBACK`
dispatcher branch; Step 3 = prod-ON vs measurement-OFF at matched N, report the SP delta; flag
default stays ON. Plan file: `~/.claude/plans/continue-work-on-task-woolly-cake.md`.

## 2026-08-01 — step 1 smoke run PASSED

Minimal smoke config (`simulation_experiments/configs/smoke_p07_fallback.json`: `INCR_Reopt_10`
only, N=4, 1 taskset, dur=70s, interval=10s) run via `MODE=test CONFIG_JSON=...
./scripts/run_simulation_plot_eval_ns.sh`. Validates the COMMITTED P0.7 code end-to-end
before layering the A/B toggle:

- `interval_fallback_log.txt` produced with correct CSV header + 7 rows (one per interval
  0–6). Columns: `interval_idx,et_jump_short_circuited,during_walk_reject_count,
  backstop_verdict,backstop_culprit_task_id,backstop_culprit_miss_chance,
  backstop_culprit_threshold`.
- `INCR_Reopt_10` ran end-to-end without crashing. Rows all `false,0,kept_walk,,,` — no
  trigger fired on this benign taskset (expected; the fallback is a safety floor, not
  something that fires on easy inputs). Culprit fields blank as designed (populated only on
  a backstop adopt).
- P0.6 sibling artifacts also produced: `safe_fallback_compute_time.txt`=0.16ms,
  `scheduler_execution_time.txt`=0.072s.
- The run's non-zero exit = the evaluation gate's OVERALL FAIL (Q1/Q2/E3 report MISSING
  because the minimal config runs one scheduler at one N) — NOT a P0.7 crash. The pipeline
  itself completed cleanly ("Pipeline complete in 13.5s").

Conclusion: committed P0.7 code is sound on the real run path. Proceeding to Step 2
(TDD `INCR_NO_FALLBACK` exposure).

## 2026-08-01 — step 2 landed (INCR_NO_FALLBACK A/B exposure, TDD green)

Implemented Option A — the measurement-arm run-path hook. TDD: wrote the failing
`OrchestratorTest.INCR_NO_FALLBACK_DisablesFallbackUse_ButStillComputesFallback` first
(`tests/testScheduleSimulate.cpp`), confirmed red (flag stayed true, no fallback computed),
then implemented:

- **`sources/Utils/Parameters.{h,cpp}`:** new `GlobalVariables::enable_fallback_use` bool
  (default `true` = prod). The master flag for the online fall-back USE; gates the three
  triggers. The safe-fallback COMPUTE stays unconditional (P0.6) — only the USE is gated.
- **`SimulationOrchestrator.cpp` `RunSimulation`:** set the global explicitly per run from
  the mode (`enable_fallback_use = (mode != "INCR_NO_FALLBACK")`) BEFORE `incr_optimizer_`
  construction, so each run is self-contained regardless of global state left by a prior run
  in the same process. Added `INCR_NO_FALLBACK` to the construction guard (`:297`) so the
  optimizer + safe fallback ARE built (the flag gates USE, not COMPUTE — certificate holds in
  the measurement arm). Read the global into `incr_optimizer_.enable_fallback_use_` at
  construction.
- **`SimulationOrchestrator.cpp` dispatcher:** new `INCR_NO_FALLBACK` branch routes to
  `Optimize_w_TL_ScratchOrIncre` (same as `INCR`). No per-interval save/restore: unlike
  `disable_time_limit_opt`/`use_wcet_execution_time` (globals read per-call inside the
  optimizer), `enable_fallback_use_` is a MEMBER set once at construction — the global is
  only the run-path channel to set it.
- **`tests/RunOrchestrator.cpp`:** added `INCR_NO_FALLBACK` to the documented modes.
- **`tests/testScheduleSimulate.cpp`:** `TestOrchestrator::FallbackUseEnabled()` accessor
  (wraps the protected `GetIncrOptimizer().enable_fallback_use_`); the test asserts the
  measurement arm flips the flag false + still computes the fallback, and the prod arm
  (`INCR`) leaves it true.

TDD: red → green → 17/17 ctest + 115/115 `testIncreOpt_w_TL`. No regressions. Design note:
the global is set per-`RunSimulation` (not save/restore) because each binary run is one mode
per process; the explicit set makes back-to-back runs in one process (the test) self-contained.

## 2026-08-01 — step 3 PARTIAL: N=4 A/B done; N=6 blocked by P0.6 loud-fail (NOT a P0.7 bug)

Ran `measure_p07_penalty.json` (prod `INCR_Reopt_10` flag-ON vs measurement
`INCR_NO_FALLBACK` flag-OFF, same tasksets, N=[4,6,8], 5 tasksets each).

**N=4 completed for BOTH arms** (`comparison_summary.csv`, N=4):
- prod `INCR_Reopt_10`: Mean_SP=0.9133, Std=0.1091, Miss=0.0051, ImpMiss=0.0, ExecTime=0.000426s
- meas `INCR_NO_FALLBACK`: Mean_SP=0.9228, Std=0.0910, Miss=0.0000, ImpMiss=0.0, ExecTime=0.000355s
- → prod arm is 0.0095 SP LOWER (≈1% penalty), the expected safety-for-SP trade.

**P0.7 mechanism FIRED on the prod arm, N=4 taskset_3** — the meaningful signal:
- interval 0: `during_walk_reject_count=1` (trigger b-i rejected an unsafe challenger mid-walk)
- interval 2: `backstop_verdict=adopted_fallback`, culprit task_id=1, miss_chance=0.267411
  > threshold=0.241803 (trigger b-ii swapped in the safe fallback — walk's final result
  violated the important-task gate)
- measurement arm's log for the same taskset: all `none` (flag off → triggers inert,
  byte-identical to pre-P0.7). Confirms the A/B is exercising the real mechanism.

**N=6 BLOCKED — P0.6 loud-fail by design (both arms crash identically):** taskset_3 of
tasks6 died SIGABRT in `ComputeSafeFallback` (NOT a P0.7 trigger site). The throw is
P0.6's certificate check at `OptimizeSP_TL_Incre.cpp:1030`: the worst-case-DAG result
(point mass at `max(execution_time_max)` across intervals) itself violates the
important-task gate → "No safe fallback exists for this task set — regenerate." Because
the safe-fallback COMPUTE is unconditional (P0.6) and P0.7 only gates the USE, BOTH arms
(`INCR_Reopt_10` AND `INCR_NO_FALLBACK`) throw the identical message — proving the crash
is independent of the P0.7 flag. Root cause: the taskset passed P0.8's generator gate
(checked at `et_mean` — task_0 et_mean=2.61, util 0.079) but fails P0.6's worst-case gate
(task_0 execution_time_max=29.7 vs period=33 → worst-case util 0.90 for one important
task alone). This is a P0.8-vs-P0.6 gate-consistency gap (mean-ET-feasible ≠ max-ET-
schedulable), NOT a P0.7 defect. The harness (P1.15 layer-A) stops the whole run on any
crash, so N=6 tasksets 3–4 and all of N=8 are NOT_RUN.

## 2026-08-01 — user decision: report N=4 now, file the gap as P2.17

User chose: report the SP-penalty from N=4 (where P0.7 demonstrably fired), file the
P0.6-worst-case-vs-P0.8-mean gate gap as a SEPARATE task (**P2.17**,
`agents/active_tasks/P2_17_p06_p08_gate_consistency_gap/`), and re-run the full A/B at
N=[4,6,8] after P2.17 lands. The N=6 crash is genuinely outside P0.7's scope (both arms
crash identically in the unconditional P0.6 compute). Crashed N=6/N=8 artifacts cleared;
the N=4 data point is kept at
`runs/measure_p07_.../sim/tasks4_.../comparison_summary.csv`. P0.7's own code is SOUND
(N=4: mechanism firing + ≈1% SP penalty) → P0.7 staged as complete pending the P2.17
re-run for the fuller measurement.

## 2026-08-01 — [CRASH] compare_against_bf test run aborted (INCR_Reopt_10 taskset_1, SIGABRT)

Ran `compare_against_bf.json` test_mode (N=4, 10 tasksets, dur=600s, interval=10s,
time_limit=10s) — the run the config-edit task prepared for. `INCR_Reopt_10` taskset_1
instance 0 died SIGABRT; harness (P1.15 layer-A) stopped the whole run (no summary/plots).
Artifacts: `runs/compare_against_bf_run_test_dur600_interval10_seed1000_tasks4/sim/
tasks4_dur600_interval10_seed1000/{crash_report.txt,taskset_arm_status.csv}` +
`taskset_1/INCR_Reopt_10/run.log`. INVESTIGATING: coding bug vs config issue. Suspect =
the P2.17 gate-consistency gap (P0.6 `ComputeSafeFallback` loud-fail) — same SIGABRT
signature as the measure_p07_penalty N=6 crash, now at N=4 taskset_1 under a different
generator config. Record-only; root-cause pending (see next entry).

**ROOT CAUSE CONFIRMED = P2.17 gap (NOT a P0.7 bug, NOT the config edit).** Crash msg is
verbatim P0.6's `ComputeSafeFallback` loud-fail ("worst-case-DAG result VIOLATES the
important-task gate"). Crashed arm = `INCR_Reopt_10` (the PROD arm, already in the list
pre-edit — my edit only ADDED `INCR_NO_FALLBACK`, which ran OK on taskset_0 + was RUNNING
on taskset_1 when the harness aborted). taskset_1 worst-case DAG (point mass at
max(execution_time_max) across intervals): task_1 important period=dl=33 max_et=12.91,
task_3 important period=dl=20 max_et=11.24, both on procId 1. DM priority task_3>task_1:
R_task1 = 12.91 + ceil(R/20)·11.24 → 24.15 → 35.39 > 33 → ddl_miss_chance=1.0 > threshold
0.534 → "No safe fallback exists." Feasible at et_mean (passes P0.8's gate) but NOT at
max(execution_time_max) (fails P0.6's gate) = exactly the P2.17 mean-vs-worst-case gap.
ComputeSafeFallback is UNCONDITIONAL (P0.6) → INCR_NO_FALLBACK would crash identically if
it reached taskset_1 (the harness aborted on INCR_Reopt_10 first). Same signature as the
measure_p07_penalty N=6 crash. Path forward = P2.17 (needs user design input D1/D2/D3).

## NEXT

P0.7 is CODE-COMPLETE + COMMITTED (Step 2 = `a8148dc7`, Step 3 configs/records =
`1ef3c26c`) + has a real (N=4) SP-penalty data point. The full N=[4,6,8] SP-penalty
re-run is blocked by **P2.18** (NOT P2.17 — P2.17 is committed `aefed906` and
working; the re-run surfaced a DISTINCT crash: the P0.7 gate arms the RTA cache
mid from-scratch beam → `|diff|>1` throw. See
`agents/active_tasks/P2_18_p07_gate_arms_rta_cache_mid_beam/`). P2.18 fix first,
then the re-run.

## 2026-08-01 — CORRECTION: P2.17 re-framing (NOT a P0.8-vs-P0.6 gate gap)

The "P0.8-mean vs P0.6-worst-case gate-consistency gap" framing in the entries above
(209-214, 219-221, 236, 240-252) is **WRONG**. Verified in P2.17 (see
`agents/active_tasks/P2_17_p06_p08_gate_consistency_gap/dev_log.md`):

- P0.8's gate (`important_task_rta.py`) is NOT mean-ET — non-perf WCET =
  `execution_time_max` (perf = `execution_time_mu`), deterministic fixed-point RTA,
  HARD `R<=deadline`. It is the HARDER gate; it CORRECTLY REJECTS the crashed tasksets
  (ran the Python gate on the emitted taskset_1: `schedulable=False`, culprit task_1,
  `R=35.39>33` — identical to the C++ throw). The two gates AGREE.
- The real bug: `compare_optimizers.py` called the UNGATED `run_full_generation_pipeline`
  (no `--important_tasks_schedulability_check` flag), so unschedulable-at-worst-case
  tasksets reached the sim and tripped P0.6's loud-fail. Saved seeds were the legacy +1
  step (1001) not the gate-widened +20 (1020) — proving the gate was OFF at generation.
- P2.17 D1 LANDED (git add-only): route `compare_optimizers.py` through the gated
  pipeline + the +20 seed step, default ON. Full N=[4,6,8] A/B re-run waits on its commit.

## 2026-08-02 — CLOSED

P0.7 code is fully committed + the A/B measurement is done. P2.18 (the blocker)
was fixed in `f371c543`; N=6 now runs clean.

**A/B (prod `INCR_Reopt_10` flag ON vs `INCR_NO_FALLBACK` flag OFF), 10 tasksets,
dur=600** (`compare_against_bf_run_test_dur600_interval10_seed1000_tasks4x6`):

| N | prod SP | meas SP | Δ |
|---|---|---|---|
| 4 | 0.7673 | 0.7742 | −0.0069 (≈0.9%) |
| 6 | 0.9220 | 0.9298 | −0.0078 (≈0.8%) |

≈1% SP penalty at both N — the expected safety-for-SP trade. Mechanism fires:
trigger (b-i) during-walk gate 25 (N=4) / 59 (N=6) rejects; trigger (b-ii)
backstop `adopted_fallback` 6/6; trigger (a) ET-jump 1/0. No double-fail throws.
`Mean_Scheduler_Execution_Time_s` sub-ms and within 1.2× of the no-fallback arm
(gate adds one RTA eval only when it fires).

**N=8 skipped per user** — N=6 clean post-P2.18 + the consistent ≈1% penalty
across N=4/6 was judged sufficient. Flag `enable_fallback_use_` default stays ON.
Folder → `finished_tasks/`. CLOSED.

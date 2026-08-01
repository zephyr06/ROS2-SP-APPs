# P2.18 P0.7 Gate Arms RTA Cache Mid From-Scratch Beam — Dev Log

> Chronological working log. On task completion, append a one-line milestone to
> the top-level `agents/dev_log.md` (the canonical narrative).

## 2026-08-01 — filed (surfaced by the compare_against_bf re-run AFTER P2.17)

After P2.17's gate-bypass fix landed (commit `aefed906`), re-ran
`compare_against_bf.json` test_mode (N=4, 10 tasksets). It crashed AGAIN, but
with a **different signature** than P2.17:

- Crashed arm: `taskset_2 / INCR_WCET instance 0`, SIGABRT, **0-byte run.log**
  (abort preceded any log flush).
- `INCR_Reopt_10` FINISHED OK on taskset_2 → P0.6's `ComputeSafeFallback` did
  NOT throw → NOT the P2.17/P0.6 loud-fail signature.
- Seed = `1040 = 1000 + 2*20` → P2.17's gate IS ON (the +20 widened step). P2.17
  is working.

Reproduced deterministically on both release and debug builds. Debug backtrace
(`gdb catch throw`, source-level):

```
RTACache::ComputeTaskSetDifference  throws std::runtime_error   RTA_Cache.cpp:358
  <- ClassifyReusePerTask                                       RTA_Cache.cpp:376
  <- Evaluate                                                   RTA_Cache.cpp:465
  <- SeedBaselineAndArmCache  (mode = Reopt)                    OptimizeSP_TL_Incre.cpp:532
  <- RunIntervalDescent                                         OptimizeSP_TL_Incre.cpp:558
  <- ReOptimizePeriodic                                         OptimizeSP_TL_Incre.cpp:1209
  <- Optimize_w_TL_ScratchOrIncre  (interval 0)                 OptimizeSP_TL_Incre.cpp:749
  <- DeterminePrioritiesAndBudgets                              SimulationOrchestrator.cpp:381
```

`std::terminate` <- uncaught `runtime_error` -> SIGABRT (hence the empty run.log:
the binary's stderr was empty too; `timeout`/direct run both gave exit 134 with
zero output).

**Root cause:** the P0.7 during-walk gate (trigger b-i) sources its challenger
RTA from `rta_cache_.Evaluate(...)` inside `UpdateRecords`
(`OptimizeSP_TL_Incre.cpp:223`), gated by `enable_fallback_use_` (prod default
ON). In the Reopt branch of `SeedBaselineAndArmCache` (`:520-533`) the intended
invariant is: `ResetIncumbentBaseline(true)` clears+disarms the cache, the
from-scratch beam (`:521`) runs DISARMED (no champion adopted), the cache is
re-armed at `:529`, and `:532`'s `Evaluate` is a safe `Initialize`. The P0.7
gate broke that: the beam's `UpdateRecords` calls `rta_cache_.Evaluate` (gated
by `enable_fallback_use_`, NOT `rta_cache_active_`) -> on the first call
`RTACache::Initialize` ADOPTS A CHAMPION mid-beam. A later beam step commits a
`{pa,tl}` differing by >1 task (from-scratch beam = multi-task search, not the
1-task walk the cache serves) -> `ComputeTaskSetDifference` throws the `|diff|>1`
single-change invariant (`RTA_Cache.cpp:358`) -> uncaught -> terminate -> SIGABRT.

`git blame` `UpdateRecords:214-223` confirms the `Evaluate` gate call is the
P0.7/P0.6-gate code (`500665d5`, `c87ae0d4`, `a2be3876`). Before P0.7,
`UpdateRecords` never called `Evaluate` in the from-scratch path, so the cache
stayed empty through the beam and `:532` was a safe `Initialize`.

**Why INCR_WCET and not INCR_Reopt_10 (on this taskset):** INCR_WCET sets
`use_wcet_execution_time=true`, changing the ET distribution; with WCET ETs the
beam's first commit and a later step differ by >1 -> throw. INCR_Reopt_10 (mean
ET) happened to stay <=1-diff on this taskset. Latent bug exposed by the ET
distribution, not caused by the WCET feature. Same path crashes
`measure_p07_penalty` N=6 and any INCR-family arm with the gate ON once a taskset
produces a >1-diff beam commit sequence.

**Classification:** P0.7-gate regression in the gate's RTA sourcing. Distinct
from P2.17 (gate bypass, fixed) and P0.6 (safe-fallback compute, sound). Filed
as P2.18. Next: settle D1/D2 with user, then plan the fix. No code yet.

## NEXT

D1/D2 SETTLED (see "D1/D2 SETTLED" entry below). EnterPlanMode + ExitPlanMode
for approval, then TDD RED.

## 2026-08-01 — design grounding (pre-plan)

Re-read the live code to ground the D1/D2 directions concretely. Confirmed the
**predicate split**: `rta_cache_active_` is the EXACT discriminator between the two
`UpdateRecords` callers —
- **Incremental walk** (`CallOptimizerGivenTimeLimits` `from_scratch=false`, →
  `OptimizeIncre`): `rta_cache_active_=true` (set at `:501` in the Incremental
  branch of `SeedBaselineAndArmCache`, BEFORE the walk). The cache HAS a champion
  (adopted at the baseline commit `:512`), and every walk step is `|diff|<=1` →
  `Evaluate` is the safe `|diff|==0/1` patch path. Gate-armed here is CORRECT.
- **From-scratch beam** (Reopt, `CallOptimizerGivenTimeLimits` `from_scratch=true`,
  → `OptimizeFromScratch`): `rta_cache_active_=false` (cleared at `:1165` by
  `ResetIncumbentBaseline(true)`, re-armed only at `:529` AFTER the beam). The
  cache has NO champion here → the gate's `rta_cache_.Evaluate` (`:223`) hits
  `Initialize` → adopts a champion mid-beam → a later beam step commits `|diff|>1`
  → `ComputeTaskSetDifference` throws.

So the bug is a ONE-LINE predicate mismatch: the gate arms on
`enable_fallback_use_` (the fall-back master switch) but should ALSO require
`rta_cache_active_` (the "is this a single-change walk the cache serves?" flag).
The from-scratch beam is a multi-task search the cache is explicitly designed NOT
to serve (`CommitIncumbent:917-919` already guards its own cache adopt with
`rta_cache_active_` for the same reason — the gate at `:220` is the lone site
that forgot to).

`CommitIncumbent` (`:920`) is the existing precedent: `if (rta_cache_active_)`
guards `Evaluate`+`AdoptChampion`. The gate at `:221` should mirror it. This is
D1-(a) guard-the-call, scoped to ONE line + a one-line test.

D1-(b) decouple-gate-from-cache (give the gate its own RTA path that never
touches `rta_cache_`) is a larger change with a real downside: it re-introduces a
per-candidate `ProbabilisticRTA_TaskSet` (the very duplication P0.6's shape-B
gate was designed to AVOID — see `SP_Metric.cpp:200-207`'s "zero-extra-RTA-eval"
rationale). The cache-path gate is sound WHEN the cache is armed; the bug is only
that it fires when the cache is NOT armed.

→ D1 recommendation: **(a) guard-the-call**, predicate `enable_fallback_use_ &&
rta_cache_active_`. Minimal, matches the `CommitIncumbent` precedent, preserves
the zero-extra-RTA-eval design.

D2 — should the gate run in the from-scratch beam at all? With D1=(a): the gate
goes INERT in the from-scratch beam (predicate false → no `Evaluate`, no commit
block). The from-scratch beam is still PROTECTED by the post-walk backstop
`AdoptFallbackIfUnschedulable` (`:758`) which gates the FINAL `res_opt_` with a
self-contained (cache-free) RTA. So the from-scratch beam is NOT left un-gated —
it is gated at the right granularity (final result, not every transient beam
step). This matches P0.7's intent: the during-walk gate guards the 1-task
incremental walk; the from-scratch search is gated as a whole by the backstop.
→ D2 recommendation: **gate stays, but inert during the from-scratch beam**
(D1=(a) gives this for free); the backstop is the from-scratch beam's gate.

Presenting D1/D2 to the user now.

## 2026-08-01 — D1/D2 SETTLED (user clarification)

User confirmed the intended invariant in their own terms: "RTA cache should
not be used during re-optimization. If re-optimization results failed the
important tasks' schedulability check, re-optimization results can also
roll-back to the fall-back safe results."

- **D1 = (a) guard-the-call.** "RTA cache should not be used during
  re-optimization" → the gate's `rta_cache_.Evaluate` (`UpdateRecords:223`)
  must NOT fire when `rta_cache_active_=false` (the Reopt beam). Fix: add
  `rta_cache_active_` to the predicate at `:220`:
  `enable_fallback_use_ && rta_cache_active_ && !BFSharedBudgetCancelled()`.
  Mirrors the `CommitIncumbent:920` precedent. (b) decouple rejected — it
  re-introduces the per-candidate `ProbabilisticRTA_TaskSet` P0.6 deleted.
- **D2 = (a) gate inert in the from-scratch beam; backstop covers it.** The
  beam's FINAL `res_opt_` is gated by `AdoptFallbackIfUnschedulable` (`:758`,
  cache-free `ImportantTasksMeetThresholds`), rolling back to `safe_fallback_`
  on failure. The during-walk gate serves the `|diff|<=1` incremental walk,
  not the multi-task from-scratch beam; D1=(a) makes it inert there for free.

NEXT: EnterPlanMode + ExitPlanMode for approval, then TDD RED.

## 2026-08-01 — TDD RED→GREEN verified + no-regression

A prior session had already written the D1=(a) fix + the TDD fixture but left it
**unverified and unrecorded** (records still said "NO code yet, NEXT: EnterPlanMode").
Resumed by empirically verifying the TDD loop instead of re-planning (D1/D2 were
settled pre-code; the staged change IS the user-review artifact).

**GREEN (fix in):** `cmake --build build_test --target testIncreOpt_w_TL -j5` clean;
`P07GateArmsCacheMidBeamSynthetic.ReOptimizePeriodic_GateDoesNotArmCacheMidFromScratchBeam`
PASSES. The gate predicate at `UpdateRecords:227` = `enable_fallback_use_ &&
rta_cache_active_ && !BFSharedBudgetCancelled()` makes the gate inert during the
disarmed Reopt beam → no mid-beam `Evaluate` → no champion adopted → `:539` post-beam
re-arm `Evaluate` is a safe `Initialize`.

**RED (fix out):** reverted the predicate to drop `rta_cache_active_` (one line),
rebuilt, ran the same test → throws `std::runtime_error` "RTACache::
ComputeTaskSetDifference: candidate differs from champion by more than one task —
violates the single-change invariant" — the EXACT SIGABRT-run-path signature,
reproduced in-process (no shell-out, no 0-byte log). `EXPECT_NO_THROW` fails as
expected. Fix restored immediately after.

**No-regression:** full `testIncreOpt_w_TL` = **116/116 pass** (115 prior + 1 new
P2.18). Full ctest = **16/17 pass**; the sole failure
`OrchestratorTest.CFS_RunOrchestrator_Binary` (`testScheduleSimulate` #16) is
PRE-EXISTING and environment-related: it shells out to a binary absent from this
DEBUG tree (return 32512). Confirmed identical on clean HEAD via `git stash` —
NOT caused by P2.18 (which touches only `OptimizeSP_TL_Incre.cpp/.h` +
`testIncreOpt_w_TL.cpp`, none of which the CFS test exercises).

NEXT: verify on the real crash run path (task #3) — re-run INCR_WCET on taskset_2 +
`compare_against_bf.json` N=4 test_mode → confirm SIGABRT gone, non-empty run.log.

## 2026-08-01 — real crash run path verified (task #3)

Rebuilt the RELEASE `RunOrchestrator` with the fix
(`cmake --build release --target RunOrchestrator -j5`), then re-ran the EXACT
crashed arm directly: `RunOrchestrator <taskset_2_dir> <out> INCR_WCET 10000 1`
(test_mode: dur=600s, interval=10s → 60 intervals × 10000ms; ExportLevel=1).

**Before the fix (evidence preserved):** `taskset_2/INCR_WCET/run.log` was
**0 bytes** — the SIGABRT at interval 0 preceded any log flush. Only that 0-byte
log existed; no `INCR_WCET/INCR_WCET/` output subdir. (Backed up as
`run.log.crash_p218`.)

**After the fix:** exit code **0** (was 134/SIGABRT). Full stdout:
`TotalProcessTime_s: 2.00088, SchedulerExecutionTime_s: 0.00995321,
SafeFallbackComputeTime_s: 0.000314599, Average SP Metric: 0.920009`. All 7
output files written (matching a working arm's structure); `interval_sp_metrics.txt`
has **all 60 intervals** (0–59) with SP values. The arm completed the full 600s.

**Gate/fallback behavior confirmed sound** (not merely "not crashing"):
`interval_fallback_log.txt` shows all 60 intervals = `kept_walk`, zero
`during_walk_reject_count`, no `et_jump_short_circuited`, no backstop rollback.
The during-walk gate is inert during the from-scratch beam (as D1/D2 designed) and
the cache-free backstop `AdoptFallbackIfUnschedulable` correctly kept the walk
result. The crash path (`ComputeTaskSetDifference` |diff|>1 throw at the post-beam
re-arm `Evaluate`) is gone.

**Comprehensive re-run launched** (background, PID logged): the full
`compare_against_bf.json` test_mode N=4 via
`python3 -m simulation_experiments.run_end_to_end_experiments --mode test
--config_json simulation_experiments/configs/compare_against_bf.json` — reproduces
the exact original crashed configuration (10 tasksets × main∪ablation schedulers,
gate ON, seed 1000→1040). Monitor armed for completion/crash. The targeted crashed
arm already verified above; this confirms no OTHER arm trips the same path.

NEXT: wait for the comprehensive re-run; then `git add` the staged changes for
user review + update memory.

## 2026-08-01 — comprehensive compare_against_bf N=4 re-run COMPLETED (task #3 done)

The background re-run (PID 1636743, 13:02–13:06) exited. Final state:
- `arm_status.csv` = **56 OK + 24 RESUMED = 80 arms, ZERO CRASHED/FAILED/ERROR rows.**
- **79 non-empty run.logs, 0 zero-byte** (the 80th, taskset_2/INCR_WCET, has no
  run.log: resume mode reused the valid output from the direct verify run above —
  its `INCR_WCET/INCR_WCET/` subdir holds the 60-interval data; the pre-fix 0-byte
  log is gone). The 24 RESUMED are tasksets 0/1/2 (prior valid output); the 56 OK
  are tasksets 3–9 freshly re-run, all clean success signatures (e.g. taskset_3/
  INCR_WCET `Average SP Metric: 0.901639`, no abort).
- `comparison_summary.csv` + 4 plots (`comparison_plots`, `optimizer_exec_time`,
  `optimizer_per_taskset`, `optimizer_sp_bar`) written.
- The lingering `crash_report.txt` (mtime 11:08) is a STALE pre-fix artifact —
  NOT regenerated by this 13:02 re-run; the authoritative `arm_status.csv`
  shows zero crashed arms.

**Conclusion: no arm tripped the `|diff|>1` path.** P2.18 verified end-to-end —
TDD RED→GREEN, no-regression (116/116 + 16/17 ctest sole failure pre-existing),
real crash path (taskset_2/INCR_WCET exit 0 + 60 intervals), and the full
compare_against_bf N=4 re-run clean. Fix staged (`git add`); user reviews (no
commit). NEXT (deferred, not P2.18): `measure_p07_penalty.json` N=[4,6,8] re-run
to unblock P0.7's deferred N=6/8 SP-penalty A/B data.

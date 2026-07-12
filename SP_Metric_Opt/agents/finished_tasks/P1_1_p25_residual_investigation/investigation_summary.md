# P1.1 — INCR Residual Investigation: Summary

**Date:** 2026-07-08 (summarizing work done 2026-07-07)
**Scope:** investigation summary — no new research performed for this doc
**Canonical detail:** `agents/active_tasks/P1_1_p25_residual_investigation/dev_log.md`
**Related memory:** `p25-ndiff-diff-semantics.md`, `p25-incr-et-grows-with-period.md`, `p24-reoptimization-design.md`

---

## 1. The question being investigated

P25 = "INCR per-activation ET grows with reoptimization period." The pathological
3× growth was already **eliminated** by Fix A (`986a9cfe`) + Fix B (`de4e9636`),
TDD-verified and committed: P60/P1 collapsed 3.4×→1.47× (ts0), 2.1×→1.13× (ts2);
the numerical pass bar (P30/P60 vs P10 ≤ 1.1–1.3×) is met.

**But** the literal monotonic flip `INCR_P1 ≥ P10 ≥ P30 ≈ P60` was **NOT** met —
P1 is still the cheapest INCR arm. The residual gap was attributed to a
per-variation `ObtainSP_DAG` asymmetry that **Fix C** (deferred) was meant to
address.

Per user (2026-07-06): **investigate first** — do not re-frame the claim or
implement Fix C until a specific gate is resolved.

### The gate (the concrete thing investigated)

A count discrepancy between ground truth and the instrumented runtime:

- **Ground truth** (changed-task count, see §5 for the yardstick correction):
  **2** tasks actually changed (the gaussian-only tasks whose raw Gaussian moved).
- **Instrumented runtime** (`[INCR-NDIFF-DBG]` with `debugMode:1`): reports
  **5** changed tasks (on the N=8 `taskset_0` INCR_P10 re-run; the original
  framing was 2-vs-8 on a now-gone N=10 taskset — same principle).

These should agree. They don't. The investigation was launched to find out why.

> **Honesty note on scope.** This investigation is about the **incremental
> (INCR) path's internal changed-task detection** and the descent-start-TL
> mechanism it exposed — a piece of the INCR-vs-from-scratch picture (INCR's
> diff-driven reuse vs the from-scratch REOPT path). A *direct* head-to-head
> INCR-vs-from-scratch A/B (the "equal-radii A/B" of step 2) was **dropped**,
> not run (see §6). So this doc does not contain a fresh INCR-vs-REOPT
> cost comparison; it contains the mechanism the gate exposed.

---

## 2. What was done

1. **Instrumented probe rebuilt & run.** `debugMode:1` was set in
   `sources/parameters.yaml`; two probes were already in the working tree —
   `SeedStateFromIncumbent` dump (`OptimizeSP_TL_Incre.cpp:430-438`) and
   `FirstIncreDump` (`OptimizeSP_Incre.cpp:248-266`). Rebuilt `release`
   RunOrchestrator and ran the N=8 `taskset_0` INCR_P10:
   `release/tests/RunOrchestrator <ts_dir> <out> INCR_P10 10000 1`.
   Trace: `simulation_experiments/optimizer_comparison/et_repro/dbg_trace_ts0_new/P10/stderr.txt`.

2. **Decomposed the probe output** task-by-task (interval-0 REOPT → interval-1
   first INCRE), cross-referencing each task's `base_avg` / `upd_avg` against
   its `timePerformancePairs` membership and the YAML Gaussian diff.

3. **Root-caused the discrepancy** by source inspection of
   `FindTaskWithDifferentEt` (`OptimizeSP_Incre.cpp:140-155`) and
   `FiniteDist::operator!=` (`Probability.cpp:359-368`).

4. **Confirmed Fix D inert** (step 3 of the plan): `FiniteDist::approx_equal`
   (`Probability.cpp:345-357`) is the only comparison that takes a tolerance
   param, but has **zero production callers** (only `tests/testProbability.cpp`).
   The live comparison is `operator!=`, which hardcodes `tolerance=1e-1` in its
   own inline loop and does **not** delegate to `approx_equal`.

5. **Yardstick correction (user, 2026-07-07).** The initial writeup treated the
   YAML Gaussian diff as ground truth. User corrected this: for TL-optimizable
   tasks (those with `timePerformancePairs`), the YAML Gaussian is **not** ground
   truth — it was generated without optimization results. A TL-optimizable
   task's effective ET during interval N−1 **is** the TL the optimizer adopted
   that interval; the Gaussian is a cold-start reference only, vestigial once a
   TL is applied. This **overturns** the earlier "false negative on task 5" and
   "diff the underlying Gaussians" framings.

6. **Identified the precise lever** exposed by the corrected yardstick
   (`OptimizeIncre_w_TL:373`): the incremental descent cold-starts from the
   Gaussian-mean TL instead of the carried adopted TL.

7. **User approved implementing the fix** (2026-07-07) — lifting the 2026-07-06
   implement-only hold **for this lever only**. Fix C and Fix D remain out of
   scope (wrong levers); equal-radii A/B dropped.

8. **Implemented + TDD-verified the fix** (see §7): 42/42 `testIncreOpt_w_TL`
   green, 16/16 `ctest` green. One stale test expectation updated (not a source
   bug). Working tree (uncommitted at time of this summary).

---

## 3. What was found — the mechanism

`FindTaskWithDifferentEt` compares `task.execution_time_dist` via
`FiniteDist::operator!=`. **Both sides of that comparison are TL-applied point
dists, not the underlying YAML Gaussians:**

- **Baseline side** (`prev_optimizer_.dag_tasks_`, carried): at the interval-0
  REOPT, `SeedIncumbentBaseline` seeds `prev_optimizer_` with
  `UpdateExtDistBasedOnTimeLimit(dag_tasks_, SmallestTimeLimitVec())` — every
  perf-pair task becomes a point dist at `pairs[0].time_limit` (the min TL).
  Then the from-scratch descent searches TLs and `UpdateRecords` adopts a
  strictly-better TL into `prev_optimizer_`. So by interval 1, the carried
  baseline's applied TL is the **adopted** TL (the interval-(N−1) optimization
  result), not the min. Confirmed by probe: task0 base=14.444=pairs[1],
  task1 base=50=pairs[0], task2 base=6.667=pairs[3] — all post-descent adopted
  TLs, not the seed mins `[5,50,1,…]`.

- **Update side** (`dag_tasks_update` passed to `OptimizeIncre`): this is **not**
  the raw interval-N YAML DAG. `EvaluateTimeLimitConfig_ScratchOrIncre`
  (`OptimizeSP_TL_Incre.cpp:148-149`) builds
  `dag_tasks_cur = UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits)` and
  passes *that* to `OptimizeIncre(dag_tasks_cur)`. The `time_limits` here comes
  from `OptimizeIncre_w_TL:373` = `InitializeTimeLimitsFromETConfig()` — the
  option closest to the **Gaussian mean**. So `upd_avg` is a point dist at the
  Gaussian-mean TL. Confirmed by probe: task0 upd=71.111=pairs[7],
  task1 upd=333.333=pairs[3], task2 upd=4.778=pairs[2].

**Consequence:** for a perf-pair task, `FindTaskWithDifferentEt` flags it **iff
the carried incumbent's adopted TL ≠ the new descent's (Gaussian-mean) TL** —
*independent of whether the task's ET actually changed.*

- Tasks 0,1,2 (perf-pair, YAML identical): flagged because adopted TL ≠
  Gaussian-mean TL. **False positives** → trigger a redundant PA-variation sweep
  (`nvar=17` at the first INCRE) for tasks whose ET did not change.
- Task 5 (perf-pair, Gaussian mu changed): **correctly NOT flagged** (under the
  corrected yardstick) — its adopted TL (23.467) was unchanged, so ET unchanged;
  the Gaussian mu move is irrelevant for a perf-pair task.
- Tasks 6,7 (gaussian-only, TL=−1 → `UpdateExtDistBasedOnTimeLimit` is a no-op,
  so both sides ARE the raw Gaussians): correctly flagged because the Gaussian
  actually moved. **True positives.**

So `ndiff` measures **TL-config drift between the carried incumbent and the
Gaussian-mean cold start**, not **ET-distribution drift between intervals**.
That is why `ndiff` is non-trivial even on consecutive intervals where the YAML
barely moves.

---

## 4. Hypothesis

> **The residual INCR cost (and the `ndiff` overcount) is caused by the
> incremental descent cold-starting its time-limit search from the Gaussian-mean
> TL (`InitializeTimeLimitsFromETConfig()`) instead of the carried adopted TL
> from the previous interval.** This makes the update side of
> `FindTaskWithDifferentEt`'s diff be point dists at the Gaussian-mean TL, while
> the baseline side carries the adopted-TL point dists — so any perf-pair task
> whose adopted TL ≠ Gaussian-mean TL gets false-flagged as "changed,"
> generating redundant `ObtainSP_DAG` variations on tasks whose ET did not
> actually change.

Predicted consequence of the fix (start descent from carried adopted TL): the
update side becomes point dists at the carried adopted TL = the baseline for any
unchanged perf-pair task → no false positives. Only gaussian-only tasks
(TL=−1 no-op, raw Gaussian compared) would flag, matching the corrected ground
truth of 2. So `ndiff` should drop 5 → ~2 on the probe taskset.

---

## 5. What the hypothesis was tested against

### 5.1 The ground-truth probe (INCR_P10, N=8 taskset_0, interval 0→1)

```
[INCR-ET-DBG] SeedStateFromIncumbent: tl=[5 50 1 -1 25 1.65 -1 -1 ]
              dag_with_tl task1 avg=50 task7 avg=182.48
[INCR-ET-DBG] interval=0 mode=INCR_P10 path=REOPT sp_dag_calls=27 opt_ms=55.6
[INCR-ET-DBG] FirstIncreDump: N=8 ndiff=5
  task 0 base_avg=14.444 upd_avg=71.111   FLAGGED
  task 1 base_avg=50     upd_avg=333.333  FLAGGED
  task 2 base_avg=6.667  upd_avg=4.778    FLAGGED
  task 3 base_avg=8.908  upd_avg=8.908
  task 4 base_avg=25     upd_avg=25
  task 5 base_avg=23.467 upd_avg=23.467
  task 6 base_avg=8.137  upd_avg=7.913    FLAGGED
  task 7 base_avg=182.48 upd_avg=173.977  FLAGGED
```

Re-derived verdict under the **corrected (adopted-TL) yardstick**:

| task | perf pairs? | adopted TL (base_avg) | Gaussian-mean TL (upd_avg) | YAML Gaussian changed? | flagged? | verdict |
|------|-------------|-----------------------|----------------------------|------------------------|----------|---------|
| 0 | yes | 14.444 | 71.111 | no | yes | **false positive** (update used Gaussian-mean TL, not carried adopted TL) |
| 1 | yes | 50 | 333.333 | no | yes | **false positive** (same) |
| 2 | yes | 6.667 | 4.778 | no | yes | **false positive** (same) |
| 5 | yes | 23.467 | 23.467 | mu | no | **true negative** (adopted TL unchanged → ET unchanged; Gaussian mu move is irrelevant for a perf-pair task) |
| 6 | no (−1) | 8.137 | 7.913 | mu,sigma,min,max | yes | true positive |
| 7 | no (−1) | 182.48 | 173.977 | mu,sigma,min,max | yes | true positive |

→ Ground truth = **2** changed (tasks 6,7). Runtime `ndiff=5`. **3 false
positives (0,1,2), 0 false negatives.** Consistent with the hypothesis.

### 5.2 Source inspection (mechanism confirmation)

- `OptimizeIncre_w_TL` (`OptimizeSP_TL_Incre.cpp:363-381`): line 372 rebuilds
  `time_limit_option_for_each_task_`; line 373 sets the descent start via
  `InitializeTimeLimitsFromETConfig()` (closest option to Gaussian mean).
- `EvaluateTimeLimitConfig_ScratchOrIncre` (line 145-183): builds
  `dag_tasks_cur = UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits)`
  (line 148-149); the incremental branch (line 160-165) passes `dag_tasks_cur`
  to `OptimizeIncre(dag_tasks_cur)`, whose `FindTaskWithDifferentEt` diff is the
  `ndiff` driver.
- Baseline side: `SeedStateFromIncumbent` (line 417-449) carries the TL-applied
  DAG via `UpdateExtDistBasedOnTimeLimit(dag_tasks_, ReconstructTimeLimitVecFromResOpt())`
  — the **carried adopted TL**. So baseline is correctly at adopted TL; only the
  update side cold-starts at Gaussian-mean TL. Confirms the asymmetric diff the
  hypothesis predicts.

### 5.3 TDD (red → green)

- **RED:** reverted line 387 to `InitializeTimeLimitsFromETConfig()` (guard
  removed) → new test `OptimizeIncre_w_TL_StartsDescentFromCarriedAdoptedTL`
  (`testIncreOpt_w_TL.cpp:952`) **FAILS** (asserts the descent's first-eval TL
  equals the carried adopted TL, not the Gaussian-mean TL=600).
- **GREEN:** with the fix restored → the test **PASSES**. An
  `ASSERT_NE(adopted_tl, 600.0)` guard confirms the bootstrap adopts a TL other
  than the Gaussian-mean on this fixture (it adopts 1000), so the test is
  non-vacuous.

### 5.4 Full suite

- `testIncreOpt_w_TL`: **42/42 green** (was 41/42 with the stale expectation
  failing under the fix; the new test brings the total to 42).
- `ctest`: **16/16 green** (17.27 s).

### 5.5 Counter-hypotheses ruled OUT (also "tested against")

These were considered and explicitly rejected by the investigation:

- **"Diff the underlying Gaussians."** **WRONG** under the corrected yardstick —
  the Gaussian is vestigial once a TL is applied; diffing Gaussians would
  re-introduce the very confusion the yardstick correction removed.
- **Fix C (per-variation `ObtainSP_DAG` scoring asymmetry).** **Wrong lever** —
  the residual is *what gets diffed* (point dist at Gaussian-mean TL vs at
  adopted TL), not *how variations are scored*.
- **Fix D (`GetAvgValue` band / `FiniteDist::approx_equal` tolerance).**
  **Confirmed inert** — zero production callers; `operator!=` hardcodes
  `tolerance=1e-1` and does not delegate to `approx_equal`. Solving the wrong
  problem regardless.
- **Equal-radii A/B** (REOPT vs INCR with
  `ReoptimizationTimeLimitSearchRadius == IncrementalTimeLimitSearchRadius`).
  **Dropped** — radii do not touch the descent start TL; running it would not
  move the residual.
- **Prior chat's "144.44 / 711.11 → all-{-1} taskset" premise.** **Wrong** — the
  ground-truth probe shows task 1 is a perf-pair task (10 TL options) with
  `base_avg=50, upd_avg=333.333` (both point dists at discrete TL options). The
  "144.44 / 711.11" values were two entries in task 1's `performance_records_time`
  array, not Gaussian means; they appeared in a stale/different trace. Only
  tasks 3,6,7 are {−1}-only; tasks 0,1,2,4,5 have real `timePerformancePairs`.

---

## 6. What was NOT done (still open)

- **Direct INCR-vs-from-scratch (equal-radii) A/B — dropped, not run.** So the
  question "is INCR genuinely cheaper per-variation than REOPT, or is it a
  radius-asymmetry artifact?" remains **open**. The investigation concluded the
  *gate* (the `ndiff` discrepancy) is explained by the descent-start-TL
  mechanism, not by radius asymmetry — but that is a mechanism argument, not a
  fresh A/B measurement.
- **Runtime re-run of the probe under the fix.** The fix is TDD-verified at the
  unit/suite level, but the INCR_P10 N=8 taskset_0 probe has **not** been
  re-run to confirm `ndiff` drops 5 → ~2 at runtime. (Remaining checklist item
  in `tasks.md`.)
- **Top-level milestone** in `agents/dev_log.md` not yet appended.
- **Commit.** The fix is in the working tree, uncommitted (git status shows
  `OptimizeSP_TL_Incre.cpp` etc. modified).

---

## 7. The fix applied (for reference)

In `sources/Optimization/OptimizeSP_TL_Incre.cpp`, `OptimizeIncre_w_TL` (line 387)
now starts the incremental descent from the carried adopted TL:

```cpp
std::vector<double> time_limits = ReconstructTimeLimitVecFromResOpt();  // was InitializeTimeLimitsFromETConfig()
```

plus an edge-case guard (lines 400-410): for each task, intersect the carried TL
against the current option set (`time_limit_option_for_each_task_[i]`, rebuilt at
line 372 from the NEW `dag_tasks_`); force −1 when the task has no pairs now or
the carried TL is no longer a member. This prevents a stale adopted TL from being
applied as a point dist by `UpdateExtDistBasedOnTimeLimit` when a task lost its
perf pair between N−1 and N. When `res_opt_` is empty the loop is a no-op (every
task already −1).

`ReconstructTimeLimitVecFromResOpt()` (line 385-394) already existed and was
already used by `SeedIncumbentBaseline` (line 464) for the reopt-path baseline
re-eval — a proven source of the carried adopted TL.

**Invariant verified along the way:** `res_opt_` is populated ⟺ `prev_optimizer_`
is initialized (the two are written together at every site — `UpdateRecords` and
`SeedStateFromIncumbent` both populate `res_opt_.id2time_limit` AND
`prev_optimizer_` atomically). So the optimizer-level interval-0 cold-start case
is unreachable in production; the guard does real work only for the **task-level**
edge case (a task losing its perf pair between intervals).

---

## 8. One-line takeaway

The INCR residual was **not** a per-variation scoring asymmetry (Fix C) or a
tolerance band (Fix D) — it was the incremental descent cold-starting from the
**Gaussian-mean TL** instead of the **carried adopted TL**, which made the
changed-task diff false-flag perf-pair tasks on TL drift and generate redundant
`ObtainSP_DAG` variations. Fix applied (start from `ReconstructTimeLimitVecFromResOpt()`
+ option-set guard), TDD-verified green; runtime confirmation of `ndiff` 5→~2
still pending.

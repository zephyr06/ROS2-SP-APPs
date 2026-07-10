# P1 1 p25 residual investigation — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-07

- Task scaffolded during the agents-folder reorg. Not yet started.

### GATE RESOLVED — 2-vs-8 (really 3-vs-5) changed-task discrepancy

**Rebuilt + re-ran the probe.** `debugMode:1` was already set in
`sources/parameters.yaml`; the two probes
(`SeedStateFromIncumbent` dump in `OptimizeSP_TL_Incre.cpp:430-438` and
`FirstIncreDump` in `OptimizeSP_Incre.cpp:248-266`) were already in the working
tree. Rebuilt `release` RunOrchestrator and ran the N=8 taskset_0:

```
release/tests/RunOrchestrator <ts_dir> <out> INCR_P10 10000 1
# ts_dir = .../p25periodAB_run_test_dur300_interval10_seed1000_tasks8/
#          sim/tasks8_dur300_interval10_seed1000/taskset_0
```

Trace saved: `simulation_experiments/optimizer_comparison/et_repro/dbg_trace_ts0_new/P10/stderr.txt`.

**Ground-truth probe output (interval 0 REOPT → interval 1 first INCRE):**

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

**Ground-truth YAML diff (interval 0 → 1), per task:**

| task | has perf_pairs? | YAML Gaussian changed int0→1? | probe flagged? |
|------|-----------------|-------------------------------|----------------|
| 0 | yes | no | **YES (false positive)** |
| 1 | yes | no | **YES (false positive)** |
| 2 | yes | no | **YES (false positive)** |
| 3 | no  | no | no |
| 4 | yes | no (float-formatting noise only) | no |
| 5 | yes | **mu** | **no (false negative)** |
| 6 | no  | mu,sigma,min,max | YES (true positive) |
| 7 | no  | mu,sigma,min,max | YES (true positive) |

So ground truth = **3 tasks changed** (5, 6, 7); probe reports **ndiff=5**
(0, 1, 2, 6, 7). 3 false positives + 1 false negative.

#### Root cause

`FindTaskWithDifferentEt` (`OptimizeSP_Incre.cpp:140-155`) compares
`task.execution_time_dist` via `FiniteDist::operator!=`
(`Probability.cpp:359-368`). **Both sides of that comparison are TL-applied
point dists, NOT the underlying YAML Gaussians:**

- **Baseline** (`dag_tasks_`, carried in `prev_optimizer_`): at the interval-0
  REOPT, `SeedIncumbentBaseline` seeds `prev_optimizer_` with
  `UpdateExtDistBasedOnTimeLimit(dag_tasks_, SmallestTimeLimitVec())` — every
  perf-pair task becomes a point dist at `pairs[0].time_limit` (the min TL).
  Then the from-scratch descent (`PerformCoordinateDescentForTaskConfigOpt`,
  `from_scratch=true`) searches TLs and `UpdateRecords` adopts a strictly-better
  TL config into `prev_optimizer_` — so by interval 1 the carried baseline's
  applied TL is the **adopted** TL, not the min. (Probe: task0 base=14.444 =
  pairs[1], task1 base=50 = pairs[0], task2 base=6.667 = pairs[3] — all
  post-descent adopted TLs, not the seed mins `[5,50,1,...]`.)

- **Update** (`dag_tasks_update` passed to `OptimizeIncre`): this is NOT the
  raw interval-1 YAML DAG either. `EvaluateTimeLimitConfig_ScratchOrIncre`
  (`OptimizeSP_TL_Incre.cpp:148-149`) builds
  `dag_tasks_cur = UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits)` and
  passes THAT to `OptimizeIncre(dag_tasks_cur)`. So `upd_avg` is also a point
  dist — at the interval-1 descent's current TL. (Probe: task0 upd=71.111 =
  pairs[7], task1 upd=333.333 = pairs[3], task2 upd=4.778 = pairs[2].)

**Consequence:** for a perf-pair task, `FindTaskWithDifferentEt` flags it
iff the carried incumbent's adopted TL ≠ the new descent's TL — *independent
of whether the YAML Gaussian moved*. So:

- Tasks 0,1,2 (perf-pair, YAML identical): flagged because the interval-0
  adopted TL differs from the interval-1 descent TL. **False positives** —
  trigger a redundant PA-variation sweep (`nvar=17` at the first INCRE) for
  tasks whose underlying ET distribution did not change.
- Task 5 (perf-pair, YAML mu changed): NOT flagged because the interval-0
  adopted TL happened to equal the interval-1 descent TL (both 23.467 =
  pairs[7]), so the point dists are identical and the Gaussian change is
  **masked**. **False negative** — a real ET change is missed.
- Tasks 6,7 (gaussian-only, TL=-1 → `UpdateExtDistBasedOnTimeLimit` is a
  no-op, so both sides ARE the raw Gaussians): correctly flagged because the
  Gaussian actually changed. **True positives.**

This is why `ndiff` (the runtime's "changed tasks" count) does not equal the
YAML diff count, and why `ndiff` is non-trivial even on consecutive intervals
where the YAML barely moves: it is measuring *TL-config drift between the
carried incumbent and the new descent*, not *ET-distribution drift between
intervals*.

#### Tie to the per-activation ET question

The false positives are the cost driver. At the first INCRE (interval 1) the
probe shows `ndiff=5 → nvar=17 sp_dag_calls=17`; if `FindTaskWithDifferentEt`
compared the underlying Gaussians instead, `ndiff` would be 3 (tasks 5,6,7) and
the perf-pair false positives (tasks 0,1,2) would not generate variations. The
extra variations are `ObtainSP_DAG` evals against TL-applied point dists that
do not reflect a real interval-to-interval ET change. This is the
per-variation `ObtainSP_DAG` asymmetry Fix C was meant to address — but the
lever is *what gets diffed* (point dist vs Gaussian), not *how variations are
scored*.

#### Note on the prior chat's "144.44 / 711.11" numbers

The prior chat reasoned about `base_avg=144.44, upd_avg=711.11` for task 1 and
concluded the taskset was "all-{-1}" — **that premise was wrong**. The current
ground-truth probe shows task 1 is a perf-pair task (10 TL options) with
`base_avg=50, upd_avg=333.333` (both point dists at discrete TL options). The
"144.44 / 711.11" values were two entries in task 1's `performance_records_time`
array, not Gaussian means; they appeared in a stale/different trace. The
all-{-1} reasoning in the prior chat does not apply to this taskset — only
tasks 3,6,7 are {-1}-only; tasks 0,1,2,4,5 have real `timePerformancePairs`.

#### Fix D inert-ness — CONFIRMED (step 3)

`FiniteDist::approx_equal` (`Probability.cpp:345-357`) is the only comparison
method that takes a tolerance parameter. It has **zero production callers**
(only `tests/testProbability.cpp` exercises it). The sole production
comparison of `execution_time_dist` is `FindTaskWithDifferentEt`'s use of
`operator!=` (`Probability.cpp:353-368`), which **hardcodes tolerance=1e-1** in
its own inline loop and does NOT delegate to `approx_equal`. So the Fix D
"GetAvgValue band" idea is inert today: there is no live code path that applies
a tunable tolerance to ET-distribution comparison. Wiring the band would
require either (a) making `FindTaskWithDifferentEt` call `approx_equal` with a
tolerance sourced from YAML, or (b) adding a new band check on `GetAvgValue`
explicitly. Confirmed as a finding; no code change made (investigation only,
per user instruction).

#### Status of the gate

**RECONCILED.** The 2-vs-8 (3-vs-5 here) discrepancy is not a bug in the
instrumentation — the probe faithfully reports what `FindTaskWithDifferentEt`
sees. It is a **semantic mismatch**: the runtime diffs TL-applied point dists
(incumbent-TL vs descent-TL) while the ground truth diffs underlying YAML
Gaussians. The two counts agree only when every perf-pair task's adopted TL is
stable across the interval boundary, which is not generally true.

Next: equal-radii A/B (step 2) and the re-frame/Fix-C decision (step 4) —
informed by the fact that the residual cost is "TL-drift false positives in the
diff," not "per-variation scoring asymmetry."

### 2026-07-07 (later) — YARDSTICK CORRECTION (user): YAML is not ground truth for TL-optimizable tasks

**User correction (2026-07-07):** for tasks whose TL can be optimized (have
`timePerformancePairs`), the YAML Gaussian / `performance_records_*` are NOT
ground truth — they were generated without optimization results. A
TL-optimizable task's effective ET during interval N−1 IS the TL the optimizer
adopted that interval (e.g., 5s); at interval N, before re-optimizing, the
correct assumption is ET = last adopted TL, and the optimizer continues from
there. The YAML Gaussian is only a cold-start reference, never the "previous ET."

This **overturns the yardstick** used in the GATE writeup above, which treated
the YAML Gaussian diff as ground truth and labeled probe flags as "false
positive / false negative vs YAML." Under the corrected yardstick:

- **TL-optimizable tasks (perf pairs):** ET = adopted TL (a point dist, since
  `UpdateExtDistBasedOnTimeLimit` replaces `execution_time_dist` with
  `GetUnitExecutionTimeDist(TL)`). Between intervals the ET changes iff the
  adopted TL changes (an optimization outcome) — NOT when the YAML Gaussian mu
  moves. The Gaussian is vestigial once a TL is applied.
- **Non-TL-optimizable tasks (TL=-1, no perf pairs):**
  `UpdateExtDistBasedOnTimeLimit` is a no-op, so ET = raw Gaussian. ET changes
  iff the Gaussian changed.

**Re-derived discrepancy** (INCR_P10, N=8 taskset_0, interval 0→1) under the
corrected yardstick:

| task | perf pairs? | adopted TL (base_avg) | Gaussian-mean TL (upd_avg) | YAML Gaussian changed? | flagged? | verdict under adopted-TL yardstick |
|------|-------------|-----------------------|----------------------------|------------------------|----------|-------------------------------------|
| 0 | yes | 14.444 | 71.111 | no | yes | **false positive** (update used Gaussian-mean TL, not carried adopted TL) |
| 1 | yes | 50 | 333.333 | no | yes | **false positive** (same) |
| 2 | yes | 6.667 | 4.778 | no | yes | **false positive** (same) |
| 5 | yes | 23.467 | 23.467 | mu | no | **true negative** (adopted TL unchanged → ET unchanged; Gaussian mu move is irrelevant for a perf-pair task) |
| 6 | no (-1) | 8.137 | 7.913 | mu,sigma,min,max | yes | true positive |
| 7 | no (-1) | 182.48 | 173.977 | mu,sigma,min,max | yes | true positive |

So under the corrected yardstick: ground truth = **2** changed (tasks 6,7 only
— the gaussian-only tasks whose raw Gaussian actually moved). Runtime
`ndiff=5`. **3 false positives (0,1,2), 0 false negatives.** The earlier
"false negative on task 5" was an artifact of using YAML as the yardstick —
task 5 is correctly NOT flagged because its ET (adopted TL = 23.467) did not
change.

#### Precise lever

`OptimizeIncre_w_TL` (`OptimizeSP_TL_Incre.cpp:373`) starts the incremental
descent from:

```cpp
std::vector<double> time_limits = InitializeTimeLimitsFromETConfig();  // closest-to-YAML-Gaussian-mean TL
```

This makes the update side of `OptimizeIncre`'s diff be
`dag_tasks_cur = UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits)` —
point dists at the **Gaussian-mean** TL, NOT at the carried adopted TL. The
baseline side (`prev_optimizer_.dag_tasks_`) correctly carries the
interval-(N−1) adopted-TL point dists. So the diff flags a perf-pair task iff
`adopted-TL ≠ Gaussian-mean-TL` — TL drift between the carried incumbent and
the Gaussian-mean cold start, independent of any real ET change.

Under the user's principle, the incremental descent should start from the
carried adopted TL:

```cpp
std::vector<double> time_limits = ReconstructTimeLimitVecFromResOpt();  // carried adopted TL
```

`ReconstructTimeLimitVecFromResOpt` already exists (`OptimizeSP_TL_Incre.cpp:385-394`)
and `SeedIncumbentBaseline` already uses it for the reopt-path baseline
re-eval (line 464). With that start, `dag_tasks_cur` applies the carried
adopted TLs → the update side of the diff = point dists at carried adopted TL
= same as baseline for any perf-pair task whose TL is unchanged → no false
positives. Only gaussian-only tasks (TL=-1 no-op, raw Gaussian compared) would
flag, matching the corrected ground truth of 2.

#### Reframe of the fix direction

This **overturns the earlier "diff the underlying Gaussians" suggestion** (in
the GATE writeup above and in memory `p25-ndiff-diff-semantics.md`). Diffing
Gaussians would be WRONG under the user's principle — the Gaussian is
explicitly not the ET for TL-optimizable tasks. The correct fix is to make the
update side of the diff use the carried adopted TL (start the incremental
descent from `ReconstructTimeLimitVecFromResOpt()`), so the diff compares
adopted-TL point dist vs carried-adopted-TL point dist and naturally does not
flag unchanged perf-pair tasks. The GetAvgValue band (Fix D) was solving the
wrong problem and remains inert/unnecessary.

Note: the from-scratch reopt path (`ReOptimizePeriodic`) also starts its
descent from `InitializeTimeLimitsFromETConfig()` (line 507), but that does
not feed a diff (`OptimizeFromScratch` does not call `FindTaskWithDifferentEt`),
so it is a search-quality/efficiency concern rather than a false-positive
source. The incremental path's descent start (line 373) is the direct cause of
the false positives.

**Implementation consideration (edge case for the eventual fix):** if a task
had a perf pair in interval N−1 (adopted TL recorded) but loses it in interval
N (becomes gaussian-only), `ReconstructTimeLimitVecFromResOpt()` returns the
stale adopted TL, and `UpdateExtDistBasedOnTimeLimit` would wrongly apply it
as a point dist (it does not consult `time_limit_option_for_each_task_`). The
fix should intersect the carried TL against the current option set (or use -1
when the task no longer has pairs) before applying it.

**Status:** investigation only — no code change (per the 2026-07-06 hold).
This correction supersedes the "false negative on task 5" and "diff Gaussians"
framing in the GATE writeup above. The residual is precisely "the incremental
descent cold-starts from the Gaussian-mean TL instead of the carried adopted
TL, so the diff false-flags perf-pair tasks on TL drift." Awaiting direction
on whether to lift the implement-Fix-C hold.

## 2026-07-07 (later still) — DECISION: implement the descent-start-TL fix (user-approved)

**User (2026-07-07):** "i agree, that needs to be fixed, update dev log and
related tasks first. we need to initialize time limit in that way." This lifts
the 2026-07-06 implement-only hold **for the descent-start-TL lever only**. Fix
C (per-variation scoring) and Fix D (GetAvgValue band) remain NOT in scope —
both confirmed as the wrong lever. Equal-radii A/B dropped (superseded — radii
do not touch the descent start TL).

### Mechanism (verified against current source)

- `OptimizeIncre_w_TL` (`OptimizeSP_TL_Incre.cpp:363-381`) is the entry. Line
  372 rebuilds `time_limit_option_for_each_task_ = RecordTimeLimitOptions(dag_tasks_)`
  (the full per-task option set — every `timePerformancePairs` entry, or the
  `{-1}` sentinel for tasks with no pairs). Line 373 then sets the descent's
  starting TL vector via `InitializeTimeLimitsFromETConfig()` — the option
  closest to the **Gaussian mean** (`Find_Close_ExecutionTime` against
  `execution_time_dist.GetAvgValue()`).
- `EvaluateTimeLimitConfig_ScratchOrIncre` (line 145-183) builds
  `dag_tasks_cur = UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits)`
  (line 148-149) — so the starting `time_limits` is what gets applied as point
  dists on the update side. The incremental branch (line 160-165) passes
  `dag_tasks_cur` (TL-applied) to `OptimizeIncre(dag_tasks_cur)`, whose
  `FindTaskWithDifferentEt(dag_tasks_, dag_tasks_update)` diff is the
  `ndiff` driver.
- Baseline side (`prev_optimizer_.dag_tasks_`): `SeedStateFromIncumbent`
  (line 417-449) carries the TL-applied DAG via
  `UpdateExtDistBasedOnTimeLimit(dag_tasks_, ReconstructTimeLimitVecFromResOpt())`
  — i.e. the **carried adopted TL** (the interval-(N−1) optimization result
  from `res_opt_.id2time_limit`). So the baseline is correctly at the adopted
  TL; only the update side cold-starts at the Gaussian-mean TL.

So the diff flags a perf-pair task iff `adopted-TL ≠ Gaussian-mean-TL` — TL
drift between the carried incumbent and a cold start, **independent of whether
the task's ET actually changed**. That is the false-positive mechanism.

### Fix

Start the incremental descent from the carried adopted TL:

```cpp
// OptimizeIncre_w_TL, line 373 — was:
std::vector<double> time_limits = InitializeTimeLimitsFromETConfig();
// becomes:
std::vector<double> time_limits = ReconstructTimeLimitVecFromResOpt();
// then guard: a carried TL for a task that no longer has perf pairs (or whose
// carried TL is not in the current option set) must be forced to -1, else
// UpdateExtDistBasedOnTimeLimit would apply a stale TL as a point dist.
```

`ReconstructTimeLimitVecFromResOpt()` (line 385-394) already exists and is
already used by `SeedIncumbentBaseline` (line 464) for the reopt-path baseline
re-eval — proven source of the carried adopted TL. With this start, the update
side = point dists at the carried adopted TL = the baseline for any unchanged
perf-pair task → no false positives; only gaussian-only tasks (TL=-1 no-op, raw
Gaussian compared) flag, matching the corrected ground truth of 2.

### Edge case (the guard)

`ReconstructTimeLimitVecFromResOpt()` returns `-1` for any task not in
`res_opt_.id2time_limit` (e.g. interval-0 cold start — every task is -1). But
a task that HAD a perf pair in N−1 (adopted TL recorded) and LOSES it in N
(becomes gaussian-only) would get a stale adopted TL back, and
`UpdateExtDistBasedOnTimeLimit` would wrongly apply it as a point dist (it does
not consult `time_limit_option_for_each_task_`). Guard: after reconstructing,
for each task intersect the carried TL against the current option set
(`time_limit_option_for_each_task_[i]`); if the task has no pairs now (the
`{-1}` sentinel) OR the carried TL is not a member, force -1. This also makes
the interval-0 cold-start equivalent to `InitializeTimeLimitsFromETConfig()`
when `res_opt_` is empty (every task -1 → falls back to the Gaussian-mean
closest option via the existing descent), so behavior at interval 0 is
unchanged.

### Plan

TDD per project convention:

1. Add a failing test reproducing the false positive: two intervals, identical
   perf-pair task on both (YAML identical), adopted TL ≠ Gaussian-mean TL.
   Assert the update side of the diff is at the adopted TL (not the
   Gaussian-mean TL) → `FindTaskWithDifferentEt` returns empty for that task.
2. Apply the one-line change + edge-case guard.
3. Confirm the failing test passes; run full `testIncreOpt_w_TL` + `ctest`.
4. Re-run the INCR_P10 N=8 taskset_0 probe; confirm `ndiff` drops 5 → ~2.

No code changed yet in this entry — this is the decision + plan. Implementation
follows.

## 2026-07-07 (implementation) — fix APPLIED + TDD-VERIFIED (suite + ctest green)

The descent-start-TL fix from the entry above is now implemented and verified.

### Change (`sources/Optimization/OptimizeSP_TL_Incre.cpp`)

`OptimizeIncre_w_TL` (line 387) now starts the incremental descent from the
carried adopted TL:

```cpp
std::vector<double> time_limits = ReconstructTimeLimitVecFromResOpt();  // was InitializeTimeLimitsFromETConfig()
```

followed by the edge-case guard (lines 400–410): for each task, intersect the
carried TL against the current option set
(`time_limit_option_for_each_task_[i]`, rebuilt at line 372 from the NEW
`dag_tasks_`); force -1 when the task has no pairs now or the carried TL is no
longer a member. This prevents a stale adopted TL from being applied as a point
dist by `UpdateExtDistBasedOnTimeLimit` when a task lost its perf pair between
N−1 and N. When `res_opt_` is empty the loop is a no-op (every task already -1).

### TDD red→green verified

- **RED:** reverted line 387 to `InitializeTimeLimitsFromETConfig()` (guard
  removed) → the new test
  `OptimizeIncre_w_TL_StartsDescentFromCarriedAdoptedTL` (testIncreOpt_w_TL.cpp:952)
  FAILS (it asserts the descent's first-eval TL equals the carried adopted TL,
  not the Gaussian-mean TL=600).
- **GREEN:** with the fix restored → the new test PASSES.
  The test's `ASSERT_NE(adopted_tl, 600.0)` guard confirms the bootstrap adopts
  a TL other than the Gaussian-mean on this fixture (it adopts 1000), so the
  test is non-vacuous.

### Stale test expectation updated (NOT a source bug)

`PerformCoordinateDescent_SkipsMinusOneOnlyTaskInMixedSet`
(testIncreOpt_w_TL.cpp:1128) had asserted `EXPECT_EQ(4, incremental_evals)` —
exactly 4 evals, calibrated to the OLD Gaussian-mean TL=600 start, which walked
`600→400 (backward, break) → 800→1000 (forward, adopt)` = baseline(1)+1+2 = 4.
Under the fix the descent starts at the adopted TL=1000 (the optimum, also the
upper boundary of `[400,600,800,1000]` on the fixture's monotonic-in-TL SP
landscape): `1000→800 (backward, non-improving, patience=0 breaks)` = 1, forward
pass empty (already at boundary) = 0, plus baseline = **2 evals**. That count is
start-TL-specific, not a structural invariant, so it is no longer asserted.
Updated the test to assert the structural invariants instead: (a) T_noise
({-1}-only) contributes 0 evals (`incremental_evals ≤ t_perf_full_set`, no +1
for a redundant T_noise re-eval); (b) the zero-work fallback does not fire on
top of T_perf's real evals (same upper bound); (c) the baseline eval always
runs (`≥1`). All three still hold under the fix.

### Suite status

- `testIncreOpt_w_TL`: **42/42 green** (was 41/42 with the stale expectation
  failing under the fix; the new test brings the total to 42).
- `ctest`: **16/16 green** (17.27 s).

### Invariant verified along the way (the truncated-chain question)

During implementation I verified the load-bearing invariant for the edge-case
guard's interval-0 framing: **`res_opt_` is populated ⟺ `prev_optimizer_` is
initialized.** The two states are written together at every site —
`UpdateRecords` (line 131 + 134) and `SeedStateFromIncumbent` (line 459 + 464)
both populate `res_opt_.id2time_limit` AND `prev_optimizer_` atomically; there
is no writer of one without the other. `IfInitialized()` is `!opt_pa_.empty()`
(`OptimizeSP_Incre.h:97`), and `SeedStateFromIncumbent` sets
`prev_optimizer_.opt_pa_`. So the "interval-0 cold start with `res_opt_` empty
but `prev_optimizer_` initialized" case is **unreachable in production**: the
dispatcher (`Optimize_w_TL_ScratchOrIncre`, line 338) routes `count==0` to
`ReOptimizePeriodic` → `SeedIncumbentBaseline` interval-0 branch →
`SeedStateFromIncumbent(..., tl_min, pa_rm, ...)`, which populates BOTH. By the
time `OptimizeIncre_w_TL` runs, `prev_optimizer_.IfInitialized()` is true and
`res_opt_` is non-empty; the `else` contract-violation branch (line 166–180,
`CoutError`) is the backstop. The guard at lines 400–410 is therefore doing
real work for the **task-level** edge case (a task losing its perf pair between
intervals), not for the optimizer-level cold start. The comment's interval-0
framing is a safe-degradation note for hypothetical future callers that bypass
the dispatcher, not a path that fires today.

### Remaining

- Re-run the INCR_P10 N=8 taskset_0 probe (runtime rebuild) and confirm `ndiff`
  drops 5 → ~2 (only gaussian-only tasks 6,7 flag). NOT yet done.
- Then milestone to top-level `agents/dev_log.md`.

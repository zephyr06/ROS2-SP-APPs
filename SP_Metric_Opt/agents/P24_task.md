# P25 (was P24 redesign): counter-driven reoptimization with compare-and-keep

> Working doc — please leave comments inline. The high-level design is settled;
> Tasks 3 and 4 are decomposed into commit-sized sub-tasks below so each commit
> keeps the full test suite green. Numbering: P24 was the radius-only mechanism;
> P25 is this redesign that decouples the from-scratch trigger from the TL cache.

## Context

P24 added a `ReoptimizationPeriod` counter + `ReoptimizationTimeLimitsSearchRadius`
knob to `OptimizePA_Incre_with_TimeLimits`, intended to periodically trigger deeper
re-optimization. On reflection it does not do what its name implies:

- The per-interval entry point `Optimize_w_TL_ScratchOrIncre`
  (`sources/Optimization/OptimizeSP_TL_Incre.cpp:219-242`) owns the counter but the
  counter **only sets the search radius**. It never triggers from-scratch.
- `OptimizeIncre_w_TL` (lines 202-217) **always runs coordinate descent**
  (`PerformCoordinateDescentForTaskConfigOpt`), regardless of mode.
- Inside coordinate descent, `EvaluateTimeLimitConfig` (lines 120-140) decides
  from-scratch vs incremental by **cache novelty** —
  `timelimit2optimizer_.count(time_limits)` — not by the counter. Cache miss →
  fresh `OptimizePA_Incre` + `OptimizeFromScratch`; hit → `OptimizeIncre`.

So P24 delivers periodic *wider-radius coordinate descent*, not periodic from-scratch
re-optimization. The `ReoptimizationPeriod` knob oversells what it controls.

**Goal:** make the control flow match the data model. The data model already treats
a time limit as an execution-time change (`UpdateExtDistBasedOnTimeLimit`,
`OptimizeSP_TL_BF.cpp:6-16`), and the base class `OptimizeIncre`
(`OptimizeSP_Incre.cpp:230-263`) is built for the "one task's ET changed" case — it
diffs the stored DAG vs the new one and generates 1-D PA variations only for changed
tasks. The redesign wires the orchestration to that existing primitive.

### Confirmed facts (make this a surgical refactor, not a rewrite)

- **`OptimizeIncre` has no small-change assumption.** Works for any ET change; cost
  scales with the number of differing tasks. By design we change one task's TL per
  coordinate-descent step → exactly one task's ET changes per step. (User point 1.)
- **Optimizer state already persists across intervals.** `incr_optimizer_` is a member
  of `FixedTaskPrioritySchedulingOrchestrator` (`SimulationOrchestrator.h:81`),
  constructed once in `RunSimulation` (line 249). Cache, counter, opt_sp_/opt_pa_
  already survive. No new persistence plumbing.
- **`UpdateDAG`** (`OptimizeSP_Base.h:62`) copies the new DAG into `dag_tasks_`, so
  `FindTaskWithDifferentEt` sees the diff on the next incremental call.
- **Params + env-var override already exist** (`Parameters.cpp:22-24`,
  `parameters.yaml:14-15`, `RunOrchestrator.cpp:67` `REOPTIMIZATION_PERIOD`).
- **Existing P24 tests** (`testIncreOpt_w_TL.cpp:554-633`) assert radius selection
  (`LastRadiusUsedForTest`), counter advance (`ReoptimizationIntervalCount`), and
  counter reset on `OptimizeFromScratch_w_TL`. The redesign preserves all three, so
  these stay green throughout.

## State model (verified against the code)

The minimal persistent optimizer state is **`{TL, PA, opt_sp, dag_tasks_}`**, where
`dag_tasks_` is the last-evaluated **TL-applied** DAG (so the diff sees only genuine
ET-observation changes + any TL change). Verified:

- `OptimizeFromScratch(K)` (`OptimizeSP_Incre.cpp:74-136`): reads only `dag_tasks_` +
  `sp_parameters_` + `K`. Does **not** read `opt_pa_`/`opt_sp_`; writes them fresh.
- `OptimizeIncre(dag_tasks_update)` (lines 230-263): requires `opt_pa_` non-empty
  (errors otherwise, line 231), diffs `dag_tasks_` vs `dag_tasks_update`, generates
  1-D PA variations only for changed-ET tasks.

Not state (derived): `res_opt_` (projection of TL/PA/opt_sp via `SaveTimeLimits` +
`UpdatePriorityVec`, `OptimizeSP_Base.h:18-38`); `time_limit_option_for_each_task_`
(derived from DAG + radius). Orthogonal state: `reoptimization_interval_count_`,
`per_activation_runtimes_`. Constant config: `sp_parameters_`.

## Design: compare-and-keep, counter-driven, no reset

Two paths, dispatched per interval. The persistent optimizer carries a **best-so-far
solution** = `{TL_prev, PA_prev, opt_sp_prev, dag_tasks_prev}`.

**Per interval** (`Optimize_w_TL_ScratchOrIncre`), `count = reoptimization_interval_count_`:

- **Incremental path (most intervals; also every interval when period == 0).**
  - Apply current TL_prev to the interval's fresh raw DAG → `dag_new`.
  - Coordinate descent over **narrow-radius** TL options
    (`TimeLimitSearchRadiusIncr`); each candidate evaluated via `OptimizeIncre`
    warm-started from best-so-far.
  - Updates best-so-far in place (warm-start continuity across intervals).

- **Reoptimization path (count % ReoptimizationPeriod == 0, period > 0): COMPARE-AND-KEEP.**
  - **Best-so-far** = `{TL_prev, PA_prev}`. Re-evaluate its SP under the new DAG:
    `SP_prev_new = EvaluateSPWithPriorityVec(dag_new_with_TL_prev, sp_params, PA_prev)`
    (== first line of `OptimizeIncre`, line 235 — primitive already exists). Needed
    so the comparison is apples-to-apples (both SPs under the SAME new DAG).
  - **From-scratch reoptimization** = a fresh `OptimizePA_Incre_with_TimeLimits` with
    **wide radius** (`ReoptimizationTimeLimitsSearchRadius`, NOT full/unbounded) on
    `dag_new`. "Normal from-scratch" = `OptimizeFromScratch(K)` for the first TL
    candidate (full beam, ignores `opt_pa_`, escapes PA drift), then `OptimizeIncre`
    warm-started for the rest of the wide-radius CD. Produces
    `{TL_fs, PA_fs, SP_fs}`. Runs in a fresh state (mirrors today's `INCR_SCRATCH`
    mode, `SimulationOrchestrator.cpp:275-279`); the best-so-far is **preserved
    separately, not cleared**.
  - **Compare + keep winner:** if `SP_fs > SP_prev_new` → adopt the from-scratch result
    as best-so-far; else → keep best-so-far, with `opt_sp_ = SP_prev_new` and
    `dag_tasks_ = dag_new`. Tie-break (equal SP): lower sum of time limits
    (today's `UpdateRecords` rule, lines 81-118).
  - No state clearing — best-so-far is only replaced if genuinely beaten.

**Three independent tunable knobs (all already in `parameters.yaml:8,14,15`):**
- `ReoptimizationPeriod` (default 10, 0=off) — cadence of the from-scratch reoptimization.
- `TimeLimitSearchRadiusIncr` (default 2) — narrow radius for the incremental path.
- `ReoptimizationTimeLimitsSearchRadius` (default 4) — wide radius for the from-scratch
  reoptimization (bounded, NOT full search).

**Impl note:** "initial from-scratch" (new `RunSimulation`, counter reset to 0,
`OptimizeFromScratch_w_TL`) and "reoptimization from-scratch" (counter NOT reset) are
two distinct operations — the reoptimization from-scratch must NOT zero
`reoptimization_interval_count_`, or the next interval would re-trigger immediately.

---

## Implementation tasks (dependency order)

### Task 1 — Design-decision lock + test inventory (no code)
- Record the compare-and-keep design + the `{TL, PA, opt_sp, dag_tasks_}` state
  model in `agents/dev_log.md` (P25 section) before any source edit.
- Inventory existing tests that touch this path: `tests/testIncreOpt_w_TL.cpp`
  (P24 counter tests 554-633, warm-start test 456-473, `OptimizeIncre_w_TL`
  direct-call tests 295-304), `tests/testBF_w_TL.cpp`,
  `tests/AnalyzePriorityAssignmentIncrementalExample.cpp:81`.
- Confirm public-method surface stays: `OptimizeIncre_w_TL(dag, K, radius)` (incremental
  path body) and `OptimizeFromScratch_w_TL(K)` (from-scratch path body) both kept;
  `Optimize_w_TL_ScratchOrIncre` stays the counter-driven entry point.
- **Exit:** written decision in dev_log; test inventory listed.

### Task 2 — Add best-so-far state holder (header, additive)
File: `sources/Optimization/OptimizeSP_TL_Incre.h`
- Add best-so-far snapshot `{TL_prev, PA_prev, opt_sp_prev, dag_tasks_prev}` (or a
  small struct). This is the best-so-far.
- Add accessors for tests (best-so-far SP/PA getters, mirroring `LastRadiusUsedForTest`).
- Keep `timelimit2optimizer_` + `HashKey4Vector` temporarily (build stays green);
  remove in Task 5.
- **Exit:** compiles; no behavior change.

### Task 3 — Rewire `EvaluateTimeLimitConfig` to warm-start from best-so-far

> Decomposed so each commit is independently green. The trick: the OLD cache path
> and the NEW best-so-far path can coexist behind a flag during migration, then the
> flag is removed. But that adds dead code; the cleaner sequence below relies on the
> fact that `EvaluateTimeLimitConfig`'s external behavior (given a TL vector, return
> its SP and update records) is unchanged — only the *internal* from-scratch trigger
> moves. Each sub-task is a small, testable slice.

#### 3a — Extract a `WarmStartEvaluate` helper (pure refactor, no behavior change)
File: `sources/Optimization/OptimizeSP_TL_Incre.cpp`
- Extract the cache-HIT branch of `EvaluateTimeLimitConfig` (lines 126-131: get
  cached optimizer, `OptimizeIncre`, `UpdateRecords`, `UpdateDAG`) into a private
  method `WarmStartEvaluate(K, time_limits, cached_optimizer)`. The cache-MISS branch
  (fresh construct + `OptimizeFromScratch`) stays inline.
- No call-site change yet; `EvaluateTimeLimitConfig` calls the helper on a hit.
- **Exit:** all tests green; behavior identical. This just makes the warm-start path
  callable from elsewhere (where Task 3c will point it at best-so-far instead of the
  map).

#### 3b — Populate best-so-far on every `UpdateRecords` (additive, no behavior change)
File: `sources/Optimization/OptimizeSP_TL_Incre.cpp` / `.h`
- In `UpdateRecords` (lines 81-118), when `should_update` is true, **also** write the
  winning `{TL, PA, opt_sp, dag_tasks_}` into the new best-so-far holder (from Task 2).
  Best-so-far is now a *mirror* of the cache's best entry — both populated, only one
  read.
- **Exit:** all tests green; best-so-far populated but not yet read. Add a test that
  asserts best-so-far SP equals `opt_sp_` after an `OptimizeFromScratch_w_TL`.

#### 3c — Switch `EvaluateTimeLimitConfig` to read best-so-far, not the cache
File: `sources/Optimization/OptimizeSP_TL_Incre.cpp`
- On a TL-vector evaluation: if best-so-far is populated, warm-start from it via
  `WarmStartEvaluate` (passing a temp optimizer seeded with best-so-far's
  `{PA, opt_sp, dag_tasks_}`); else fall back to `OptimizeFromScratch` (first call).
  Stop reading `timelimit2optimizer_`.
- `UpdateRecords` keeps its role (strictly-greater SP wins, else lower TL-sum) and
  still updates best-so-far.
- **Exit:** `testIncreOpt_w_TL.cpp` path-isolation tests pass;
  `AnalyzePriorityAssignmentIncrementalExample` produces the same PA. The cache is now
  write-only dead state — removed in Task 5.

> After 3c the `timelimit2optimizer_` map is populated by nobody and read by nobody,
> but still declared. The build is green; Task 5 deletes it.

### Task 4 — Compare-and-keep in `Optimize_w_TL_ScratchOrIncre`

> Decomposed so the wide-radius from-scratch reoptimization is introduced as a
> *parallel* path first (gated, default-off), verified, then wired as the
> reoptimization path. Each commit green.

#### 4a — Extract the incremental path body (pure refactor, no behavior change)
File: `sources/Optimization/OptimizeSP_TL_Incre.cpp`
- Extract the current body of `Optimize_w_TL_ScratchOrIncre` (counter read, radius
  decision, call `OptimizeIncre_w_TL`, timing, counter increment) into a private
  `PerformIncrementalOptimization(dag, K, radius)` that returns the PA.
  `Optimize_w_TL_ScratchOrIncre` becomes: decide radius (existing logic) → call
  `PerformIncrementalOptimization` → push runtime entry → increment counter.
- No behavior change; just makes the incremental path a callable unit so the
  reoptimization branch can call something else.
- **Exit:** all tests green (incl. P24 radius/counter tests 554-633).

#### 4b — Add `PerformReoptimization(dag, K, wide_radius)` (additive, not yet called)
File: `sources/Optimization/OptimizeSP_TL_Incre.cpp` / `.h`
- New private method implementing the from-scratch reoptimization half of the design:
  1. Snapshot best-so-far `{TL_prev, PA_prev}`; compute
     `SP_prev_new = EvaluateSPWithPriorityVec(dag_new_with_TL_prev, sp_params, PA_prev)`.
  2. Construct a fresh `OptimizePA_Incre_with_TimeLimits reopt(dag, sp_params_)`,
     call its `OptimizeFromScratch_w_TL`-equivalent with `wide_radius` (needs a
     from-scratch-with-radius entry — see 4b-note) → `{TL_fs, PA_fs, SP_fs}`.
  3. If `SP_fs > SP_prev_new` → adopt the from-scratch result into best-so-far; else →
     keep best-so-far, set `opt_sp_ = SP_prev_new`, `dag_tasks_ = dag_new`. Tie-break:
     lower TL-sum.
- Not wired into `Optimize_w_TL_ScratchOrIncre` yet.
- **4b-note:** `OptimizeFromScratch_w_TL` (lines 185-200) currently calls
  `RecordTimeLimitOptions` (full). The reoptimization needs `RecordCloseTimeLimitOptions`
  (wide radius). Sub-step: add an overload / param `OptimizeFromScratch_w_TL(K, radius)`
  defaulting to full (so existing callers + tests are unchanged), used by the
  reoptimization with the wide radius.
- **Exit:** compiles; unit-test `PerformReoptimization` directly on the synthetic
  `ReoptimizationTestDag` — assert it preserves best-so-far when the from-scratch run
  loses and adopts it when it wins. (Construct a case where wide-radius TL search
  beats best-so-far.)

#### 4c — Wire the from-scratch reoptimization as the reoptimization path
File: `sources/Optimization/OptimizeSP_TL_Incre.cpp` (`Optimize_w_TL_ScratchOrIncre`)
- On a reoptimization interval (`count % ReoptimizationPeriod == 0`, period > 0): call
  `PerformReoptimization(dag, K, ReoptimizationTimeLimitsSearchRadius)` instead
  of `PerformIncrementalOptimization`. On other intervals (and all intervals when
  period == 0): `PerformIncrementalOptimization(dag, K, TimeLimitSearchRadiusIncr)` as
  today.
- Keep `last_radius_used_for_test_` = the radius used (narrow or wide) so the P24
  radius-selection tests still pass. Keep `ReoptimizationIntervalCount()` advancing
  on every call. Do NOT reset the counter here.
- Timing: the chrono block must wrap whichever path runs, so
  `per_activation_runtimes_` still records every interval.
- **Exit:** P24 counter tests (554-633) pass unchanged (radius + counter semantics
  preserved); new test asserts a reoptimization interval invokes `PerformReoptimization`
  (best-so-far preserved when the from-scratch run loses) and a non-reopt interval
  warm-starts.

### Task 5 — Remove the old cache + dead code
Files: `OptimizeSP_TL_Incre.h`, `OptimizeSP_TL_Incre.cpp`
- Delete `timelimit2optimizer_` and `HashKey4Vector` (unused after 3c).
- Delete `TraverseTimeLimitOptions` if unreferenced after the rewrite (grep confirms).
- **Exit:** `grep -rn timelimit2optimizer_ sources/ tests/` returns nothing; full
  test suite green.

### Task 6 — Tests for the new contract
File: `tests/testIncreOpt_w_TL.cpp`
- Keep the existing P24 radius/counter tests (they still hold).
- Add: reopt interval runs `PerformReoptimization` and preserves best-so-far
  when the from-scratch run loses; non-reopt interval warm-starts (best-so-far PA reused,
  not re-searched from scratch).
- Update any test that depended on cache-hit-vs-miss behavior to the new best-so-far
  semantics.
- **Exit:** new + existing tests pass; `ctest` green.

### Task 7 — Docs + re-run P24-Eval as before/after baseline
- Update `agents/dev_log.md` (P25 section) and `agents/tasks.md` with design + results.
- Re-run `simulation_experiments/run_p24eval_sweep.py -j 6` (resume-skip reuses cells
  where the metric is unchanged; re-runs what the new control flow affects) then
  `aggregate_p24eval` — the cost/quality comparison for tuning period + both radii.
- **Exit:** new `p24eval_summary.md` written; before/after runtime + SP compared in
  dev_log.

---

## Critical files
- `sources/Optimization/OptimizeSP_TL_Incre.h` / `.cpp` — the wrapper; Tasks 2-5.
- `sources/Optimization/OptimizeSP_Incre.cpp:230-263` — `OptimizeIncre` (read-only;
  confirms the contract, no change).
- `sources/Optimization/OptimizeSP_Base.h:51-71` — `OptimimizePA_Base` (member state:
  `dag_tasks_`, `opt_sp_`, `opt_pa_`, `UpdateDAG`).
- `sources/Optimization/OptimizeSP_TL_BF.cpp:6-16` — `UpdateExtDistBasedOnTimeLimit`
  (TL→ET mapping, reused as-is).
- `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp:249-295` — caller;
  **no change needed** (already calls the single entry point 1:1 per interval, state
  already persists).
- `sources/Utils/Parameters.cpp:22-24`, `sources/parameters.yaml:14-15` — the knobs.
- `tests/testIncreOpt_w_TL.cpp` — test updates (Task 6).
- `simulation_experiments/run_p24eval_sweep.py`, `aggregate_p24eval.py` — reused
  unchanged for Task 7.

## Reused functions (no new ones where these fit)
- `UpdateExtDistBasedOnTimeLimit` — TL→ET mapping.
- `OptimizePA_Incre::OptimizeIncre` — warm-start PA for one-task-ET-changed.
- `OptimizePA_Incre::OptimizeFromScratch` — deep PA search (from-scratch reoptimization's first candidate).
- `OptimimizePA_Base::UpdateDAG` — sync stored DAG for the next diff.
- `ResourceOptResult::SaveTimeLimits` / `UpdatePriorityVec` — result packaging.
- `EvaluateSPWithPriorityVec` — re-evaluate best-so-far under new DAG.
- `UpdateRecords` tie-break (strictly-greater SP, else lower TL-sum) — keep, repoint
  at best-so-far.

## Verification
1. **Unit:** `ctest` — existing `testIncreOpt_w_TL.cpp`, `testBF_w_TL.cpp`,
   `testOptimizeIncrePA.cpp` pass; new compare-and-keep assertions pass.
2. **Direct example:** `tests/AnalyzePriorityAssignmentIncrementalExample.cpp:81`
   (calls `OptimizeIncre_w_TL` directly) produces the same PA as before for an
   isolated incremental step.
3. **End-to-end + measurement:** `python3 -m simulation_experiments.run_p24eval_sweep -j 6`
   then `python3 -m simulation_experiments.aggregate_p24eval` → `p24eval_summary.md`.
   Compare per-activation runtime and mean SP before/after in `agents/dev_log.md`.
4. **No-regression smoke:** one cell (period 3, N=6, taskset_0) — confirm the runtime
   CSV still has 60 rows, radii ∈ {2,4}, reopt cadence 60/period.

## Out of scope
- Filling the 30 missing P24-Eval cells (N=12 × {5,10,20}) — separate.
- P21 (warm-start across RunSimulation invocations) — next planned task after P25.
- Tuning period+radius values — Task 7 produces the data; tuning is follow-up.

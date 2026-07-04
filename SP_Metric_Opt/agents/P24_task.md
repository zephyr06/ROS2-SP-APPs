# P25 (was P24 redesign): counter-driven reoptimization with compare-and-keep

> Minimal-commits design. Each commit below is additive, self-contained, and keeps the
> full test suite green.

## Design: bool-dispatch inside shared coordinate descent

Two PA-search evaluators, one shared coordinate-descent loop. A `bool from_scratch`
selects which evaluator runs per TL-vector evaluation.

### State model

Persistent state across simulation intervals (already true for `incr_optimizer_`):
- `dag_tasks_` — raw DAG (no TL applied; TL is applied transiently per-eval)
- `opt_pa_` / `opt_sp_` — incumbent PA and its SP
- `res_opt_` — projection of incumbent to `{id2time_limit, id2priority, priority_vec, sp_opt}`

No per-TL cache. The warm-start source for `EvaluateTimeLimitConfig_Incre` is the
single incumbent `{opt_pa_, opt_sp_}`.

### Evaluators

```cpp
double EvaluateTimeLimitConfig_ScratchOrIncre(
    int K, const std::vector<double>& time_limits, bool from_scratch);
```

- **`from_scratch = true`** (reoptimization path)
  1. `DAG_Model dag_cur = UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits)`
  2. `OptimizePA_Incre optimizer(dag_cur, sp_parameters_)`
  3. `optimizer.OptimizeFromScratch(K)` — ignores any warm state
  4. `UpdateRecords(optimizer, time_limits); return optimizer.opt_sp_`

- **`from_scratch = false`** (incremental path)
  1. `DAG_Model dag_cur = UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits)`
  2. `OptimizePA_Incre optimizer(dag_tasks_, sp_parameters_)`
     - seed: `optimizer.opt_pa_ = opt_pa_`, `optimizer.opt_sp_ = EvaluateSPWithPriorityVec(dag_tasks_, sp_parameters_, opt_pa_);`
  3. `optimizer.OptimizeIncre(dag_cur)` — diff-based 1-D search starting from warm PA
  4. `UpdateRecords(optimizer, time_limits); return optimizer.opt_sp_`

### Shared coordinate descent

```cpp
void PerformCoordinateDescentForTaskConfigOpt(
    int K, std::vector<double>& time_limits, bool from_scratch);
```

Same body for both modes: sort tasks by heuristic, loop over options, call
`EvaluateTimeLimitConfig_ScratchOrIncre(K, time_limits, from_scratch)`.

### Public entry points (final surface)

```cpp
// Legacy incremental — kept for tests, forwards to the new overload
PriorityVec OptimizeIncre_w_TL(const DAG_Model& dag_tasks_update, int K);

// NEW overload — takes radius explicitly
PriorityVec OptimizeIncre_w_TL(const DAG_Model& dag_tasks_update, int K, int radius);

// NEW from-scratch entry — takes radius explicitly
PriorityVec OptimizeFromScratch_w_TL(const DAG_Model& dag_tasks_update, int K, int radius);

// Renamed from old OptimizeFromScratch_w_TL(int K) — zero behavior change at rename time
PriorityVec ReOptimizePeriodic(int K);

// NEW reoptimization with compare-and-kept (added in a later commit)
PriorityVec ReOptimizePeriodic(const DAG_Model& dag_tasks_update, int K, int radius);
```

`ReOptimizePeriodic(const DAG_Model&, int, int)` implements compare-and-keep:
1. Snapshot incumbent `{TL_prev, PA_prev, opt_sp_prev}`.
2. Re-evaluate incumbent under new DAG: `SP_prev_new = EvaluateSPWithPriorityVec(dag_new_with_TL_prev, sp_parameters_, PA_prev)`.
3. Run `OptimizeFromScratch_w_TL(dag_new, K, radius)` in a fresh object → `{TL_fs, PA_fs, SP_fs}`.
4. If `SP_fs > SP_prev_new` — adopt from-scratch result; else keep incumbent.
5. Set `dag_tasks_ = dag_new` so next incremental diff is correct.

## Commit plan (minimal, always green)

### Commit 1 — Add `radius` param to `OptimizeIncre_w_TL` and `OptimizeFromScratch_w_TL`

**Goal:** radius becomes plumbable end-to-end; existing tests compile and pass.

Files:
- `sources/Optimization/OptimizeSP_TL_Incre.h`
- `sources/Optimization/OptimizeSP_TL_Incre.cpp`
- Call sites in `SimulationOrchestrator.cpp` (update to pass radius)
- Test files that call `OptimizeIncre_w_TL` / `OptimizeFromScratch_w_TL`

Changes:
1. Add `OptimizeIncre_w_TL(const DAG_Model&, int, int radius)` overload.
2. Add `OptimizeFromScratch_w_TL(const DAG_Model&, int, int radius)` overload.
3. Old `OptimizeFromScratch_w_TL(int K)` forwards to new overload with `dag_tasks_` and `RecordTimeLimitOptions` (full).
4. Old `OptimizeIncre_w_TL(const DAG_Model&, int)` forwards to new overload with `GlobalVariables::IncrementalTimeLimitSearchRadius`.
5. `SimulationOrchestrator.cpp` INCR/INCR_NO_TL/INCR_WCET paths pass `IncrementalTimeLimitSearchRadius` explicitly.

**Exit:** compiles, `ctest` green.

### Commit 2 — Replace `timelimit2optimizer_` with `prev_optimizer_` from previous optimal status

**Goal:** replace exponential cache with a single persistent optimizer `prev_optimizer_` from the previous optimal status.

Files:
- `sources/Optimization/OptimizeSP_TL_Incre.h`
- `sources/Optimization/OptimizeSP_TL_Incre.cpp`
- `tests/testOptimizeIncrePA.cpp` (lines 312-315 test cache size; update or delete)

Changes:
1. Delete `timelimit2optimizer_` from the header and class. Keep `HashKey4Vector` and `TraverseTimeLimitOptions` for now (to be refactored/cleaned up later).
2. Add `OptimizePA_Incre prev_optimizer_;` to the class `OptimizePA_Incre_with_TimeLimits`.
3. Rewrite `EvaluateTimeLimitConfig` to:
   - If `prev_optimizer_.IfInitialized()`: warm-start by copying `prev_optimizer_` and running `OptimizeIncre` on the copy.
   - Else: fall back to `OptimizeFromScratch(K)`.
4. Update `UpdateRecords` to update `prev_optimizer_ = optimizer;` whenever a new optimal configuration is saved.
5. Update test that asserts `timelimit2optimizer_.size()`.

**Exit:** cache is gone, incremental warm-starts from incumbent, `ctest` green.

### Commit 3 — Extract `EvaluateTimeLimitConfig_ScratchOrIncre` + add bool to `PerformCoordinateDescentForTaskConfigOpt`

**Goal:** introduce the from-scratch evaluator as a variant of the existing one.

Files:
- `sources/Optimization/OptimizeSP_TL_Incre.h`
- `sources/Optimization/OptimizeSP_TL_Incre.cpp`

Changes:
1. Rename current `EvaluateTimeLimitConfig` → `EvaluateTimeLimitConfig_ScratchOrIncre`, add `bool from_scratch` parameter.
2. When `from_scratch == true`: always do fresh `OptimizePA_Incre` + `OptimizeFromScratch`.
3. When `from_scratch == false`: do warm-start path from Commit 2.
4. Add `bool from_scratch = false` to `PerformCoordinateDescentForTaskConfigOpt`; pass it through.
5. `OptimizeIncre_w_TL(dag, K, radius)` calls descent with `from_scratch = false`.
6. `OptimizeFromScratch_w_TL(dag, K, radius)` calls descent with `from_scratch = true`.

**Exit:** both modes compile and run; `ctest` green.

### Commit 4 — Rename `OptimizeFromScratch_w_TL(int K)` → `ReOptimizePeriodic(int K)`

**Goal:** purely mechanical rename, zero behavior change.

Files:
- `sources/Optimization/OptimizeSP_TL_Incre.h`
- `sources/Optimization/OptimizeSP_TL_Incre.cpp`
- All call sites in `tests/` and `SimulationOrchestrator.cpp` (BF mode path etc.)

Changes:
1. Delete `OptimizeFromScratch_w_TL(int K)` declaration/definition (overload from Commit 1 stays).
2. Add `ReOptimizePeriodic(int K)` that calls the new overload with `dag_tasks_` and full radius.
3. Update all call sites.

**Exit:** mechanical rename, `ctest` green.

### Commit 5 — Add `ReOptimizePeriodic(const DAG_Model&, int, int)` with compare-and-keep

**Goal:** the new reoptimization entry point, not yet wired into orchestration.

Files:
- `sources/Optimization/OptimizeSP_TL_Incre.h`
- `sources/Optimization/OptimizeSP_TL_Incre.cpp`
- `tests/testIncreOpt_w_TL.cpp`

Changes:
1. Add `ReOptimizePeriodic(const DAG_Model& dag_new, int K, int radius)`.
2. Implement compare-and-keep (snapshot → re-eval incumbent → wide from-scratch → keep winner).
3. Add unit tests:
   - Synthetic DAG where wide search loses: assert incumbent preserved.
   - Synthetic DAG where wide search wins: assert from-scratch result adopted.

**Exit:** new tests pass, existing tests green.

### Commit 6 — Wire counter-driven dispatch in `Optimize_w_TL_ScratchOrIncre`

**Goal:** choose between incremental and reoptimization paths based on counter.

Files:
- `sources/Optimization/OptimizeSP_TL_Incre.cpp`
- `SimulationOrchestrator.cpp` (if needed; orchestrator already calls single entry point)
- `tests/testIncreOpt_w_TL.cpp`

Changes:
1. In `Optimize_w_TL_ScratchOrIncre`:
   - If `ReoptimizationPeriod > 0 && count % ReoptimizationPeriod == 0`: call `ReOptimizePeriodic(dag_new, K, ReoptimizationTimeLimitSearchRadius)`
   - Else: call `OptimizeIncre_w_TL(dag_new, K, IncrementalTimeLimitSearchRadius)`
2. Increment counter after decision.
3. Counter reset only on explicit from-scratch call (not on ReOptimizePeriodic).
4. Add radius-selection and counter-advance tests.

**Exit:** counter + radius + compare-and-keep tests green.

### Commit 7 — Verify + docs

1. Run full `ctest`.
2. Re-run `run_p24eval_sweep.py` + `aggregate_p24eval.py`.
3. Update `agents/dev_log.md` with before/after numbers.

## Key invariants to verify during review

- `dag_tasks_` inside `OptimizePA_Incre_with_TimeLimits` is **always the raw DAG** (no TL applied). TL is applied transiently inside `EvaluateTimeLimitConfig_ScratchOrIncre`.
- `OptimizeIncre` diff compares raw `dag_tasks_` against TL-applied `dag_cur` → exactly one task changed if the TL differs for that task. Correct by design.
- `UpdateRecords` still enforces strictly-greater SP wins; tie-break lower TL-sum.
- No global mutable state besides `disable_time_limit_opt` / `use_wcet_execution_time` (already handled).

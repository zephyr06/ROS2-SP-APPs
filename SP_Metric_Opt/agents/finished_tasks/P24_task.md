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

### Commit 7 — Verify + runtime A/B

1. Run full `ctest`. ✅ 16/16 green.
2. **Runtime A/B — period sweep.** `ReoptimizationPeriod` is read from
   `sources/parameters.yaml` at binary startup (`Parameters.cpp:22`) and the
   `RunOrchestrator` CLI has no period flag, so to sweep the period across arms
   we encode it in the **scheduler mode string**: `INCR_P<n>` sets
   `GlobalVariables::ReoptimizationPeriod = n` at startup and dispatches as
   `INCR`. The mode string already flows through the Python pipeline as the
   scheduler name + output dir, so no Python changes are needed. Config:
   `simulation_experiments/configs/p25_period_ab_config.json` —
   `main_scheduler_list = [BF, INCR_P1, INCR_P10, INCR_P30, INCR_P60,
   INCR_SCRATCH]`, `num_tasks_for_cross_task_comparison = [4,6,8]`,
   `num_tasksets_to_generate = 5`, `simulation_duration_seconds = 300`,
   `scheduler_trigger_interval_seconds = 10`. Run via
   `MODE=prod CONFIG_JSON=simulation_experiments/configs/p25_period_ab_config.json
   ./scripts/run_end_to_end.sh`.
   - **Note:** the period-0 "off" baseline originally planned here is dropped
     (Commit 6 made the knob positive-only). `INCR_P1` (reopt every interval,
     wide radius, compare-and-keep against the running incumbent) is the
     "always reopt, with memory" upper bound; `INCR_SCRATCH` is the
     "always reopt, amnesiac" control. The pair isolates the value of carrying
     the incumbent forward; the P10/P30/P60 arms show drift vs. reopt-frequency.
   - **Stale-binary gotcha:** the release binary must be rebuilt before this A/B
     is meaningful — the one on disk (2026-07-03 20:15) predates Commit 6, so
     `Optimize_w_TL_ScratchOrIncre` isn't even linked in and
     `ReoptimizationPeriod` is dead config there.
3. Update `agents/dev_log.md` with before/after SP + per-activation runtime
   numbers.

## Key invariants to verify during review

- `dag_tasks_` inside `OptimizePA_Incre_with_TimeLimits` is **always the raw DAG** (no TL applied). TL is applied transiently inside `EvaluateTimeLimitConfig_ScratchOrIncre`.
- `OptimizeIncre` diff compares raw `dag_tasks_` against TL-applied `dag_cur` → exactly one task changed if the TL differs for that task. Correct by design.
- `UpdateRecords` still enforces strictly-greater SP wins; tie-break lower TL-sum.
- No global mutable state besides `disable_time_limit_opt` / `use_wcet_execution_time` (already handled).

## KNOWN ISSUE (reproduced 2026-07-04; FIX APPLIED & TDD-VERIFIED 2026-07-04; runtime A/B re-run DONE 2026-07-04 — partial pass): INCR per-activation ET grows with the reoptimization period

> **Status (updated 2026-07-04):** root cause found and **fix applied +
> TDD-verified** (16/16 ctest green). Runtime A/B re-run **DONE 2026-07-04
> 19:41** — partial pass: the pathological 3× growth is eliminated (P10/P30/P60
> collapsed to the P1≈INCR_SCRATCH floor; numerical pass criterion met on both
> tasksets), but the literal directional flip `INCR_P1 ≥ P10 ≥ P30 ≈ P60` was
> NOT achieved because Fix C (per-variation `ObtainSP_DAG` asymmetry) was
> deferred. See the "DONE: A/B re-run" subsection at the end of this section.
> The original repro (deliberately not solved in the first pass, per user
> instruction 2026-07-04: "create explicit test case to reproduce this issue,
> and clearly record this issue into P24_Task.md ... don't work on solving it")
> is preserved below as the **before** snapshot.
>
> **Fix landed in commits:**
> - `986a9cfe` — **Fix A:** `OptimizeIncre` advances `dag_tasks_` after the
>   diff loop (`sources/Optimization/OptimizeSP_Incre.cpp`), so the incremental
>   diff is consecutive-interval (small `ndiff`) instead of stale-vs-fresh
>   (always `N`). This was the frozen-baseline root cause.
> - `de4e9636` — **Fix B + latent prerequisite:** in
>   `PerformCoordinateDescentForTaskConfigOpt`
>   (`sources/Optimization/OptimizeSP_TL_Incre.cpp`), skip a task whose only TL
>   option is `-1` (no `timePerformancePairs`) and add a zero-work fallback that
>   runs one eval when *all* tasks were `{-1}`-only so `UpdateRecords` fires and
>   Fix A's `dag_tasks_` advance propagates into `prev_optimizer_`. Same commit
>   also fixes the latent `SeedStateFromIncumbent` bug (it seeded
>   `prev_optimizer_`'s dag/opt_pa_/opt_sp_ but NOT `sp_parameters_` →
>   `_Map_base::at` crash on the all-`{-1}` no-improvement path that Fix B's
>   fallback exercises). TDD: two red tests added in
>   `tests/testIncreOpt_w_TL.cpp` —
>   `PerformCoordinateDescent_AllMinusOneOnly_RunsOneEvalAndAdvancesPrevOptimizer`
>   and `PerformCoordinateDescent_SkipsMinusOneOnlyTaskInMixedSet` — both now
>   green.

### Symptom

In the P25 period A/B, the expectation is: a *larger* `ReoptimizationPeriod`
means *fewer* wide-radius reoptimization steps (the expensive path) and *more*
narrow-radius incremental steps (the cheap path), so per-activation scheduler
execution time (ET) should **decrease** (or at least stay flat) as the period
grows from 1 → 10 → 30 → 60.

The data shows the **opposite**: ET *increases* with the period, then plateaus.
`INCR_P1` (reopt every interval) is the *fastest* INCR variant; `INCR_P10` is
~3× slower; `INCR_P30`/`INCR_P60` are slower still and roughly equal.

### Reproduction (contention-free, serial)

Repro harness: `simulation_experiments/repro_et_grows_with_period.py`.
Reuses the existing P25 prod-run tasksets (no regeneration; deterministic across
arms). Runs every arm **serially** — one `RunOrchestrator` process at a time —
to rule out the parallel-worker CPU-contention confound (the prod run used
`parallel_worker_processes=4` with 6 arms on 8 cores, and `RunOrchestrator`
measures whole-process wall-clock, so concurrent arms inflate each other's ET
non-uniformly). Serial execution removes that confound; the pattern survives, so
it is **algorithmic**, not a measurement artifact.

Run:
```
python3 -m simulation_experiments.repro_et_grows_with_period --taskset 0 --reps 3
python3 -m simulation_experiments.repro_et_grows_with_period --taskset 2 --reps 1
```

Numbers (tasks=6, 30 intervals, per-activation ET = wall_ms / 30; min of N reps):

**BEFORE fix (frozen-baseline binary, 2026-07-04 12:11 build):**

| arm           | ts0 per-act (ms) | ts2 per-act (ms) |
|---------------|------------------|------------------|
| BF            | 1099.8           | 6102.3           |
| INCR_P1       | **101.5**        | **94.0**         |
| INCR_P10      | 301.1            | 184.6            |
| INCR_P30      | 342.7            | 193.3            |
| INCR_P60      | 342.4            | 194.3            |
| INCR_SCRATCH  | 106.4            | 93.9             |

**AFTER fix A+B (rebuilt release binary, 2026-07-04 19:34 build; HEAD `de4e9636`):**

| arm           | ts0 per-act (ms) | ts2 per-act (ms) | ts0 after/before | ts2 after/before |
|---------------|------------------|------------------|------------------|------------------|
| BF            | 1120.8           | 5522.0           | 1.02×            | 0.91×            |
| INCR_P1       | **62.0**         | **65.2**         | 0.61×            | 0.69×            |
| INCR_P10      | 87.7             | 74.9             | 0.29×            | 0.41×            |
| INCR_P30      | 90.8             | 71.9             | 0.26×            | 0.37×            |
| INCR_P60      | 90.8             | 73.4             | 0.27×            | 0.38×            |
| INCR_SCRATCH  | 62.7             | 69.0             | 0.59×            | 0.73×            |

Verdict (see "DONE: A/B re-run" below for the full reasoning):
- The pathological growth is **eliminated**. P10/P30/P60 collapsed from
  301–343 → 88–91 ms/act (ts0) and 185–194 → 72–75 ms/act (ts2), i.e. toward
  the P1 ≈ INCR_SCRATCH floor (~62–69 ms/act). The 3.4× (ts0) / 2.1× (ts2)
  P60/P1 ratio is now 1.47× (ts0) / 1.13× (ts2).
- The **numerical** pass criterion is met on both tasksets: P30/P60 do **not**
  exceed P10 by the 1.1–1.3× margin they did before — ts0 P30/P60 = 1.035× P10
  (was 1.14×), ts2 P30/P60 = 0.96–0.98× P10 (was 1.05×, i.e. now slightly
  *below* P10).
- The **literal** directional criterion `INCR_P1 ≥ INCR_P10 ≥ INCR_P30 ≈ INCR_P60`
  (strictly non-increasing in period) is **NOT** met: P1 remains the *cheapest*
  INCR arm on both tasksets (62 < 88–91 on ts0; 65 < 72–75 on ts2). The
  direction did not flip — it flattened. This is consistent with §4a/§6 of
  `agents/debug_runtime0704_incr.md`: Fix A+B kill the frozen-baseline
  pathology (the actual bug) but the residual P1 < P60 gap is the per-variation
  `ObtainSP_DAG` asymmetry that **Fix C** (deferred) was meant to remove.
  `OptimizeIncre` still pays `ObtainSP_DAG` per priority variation while
  `OptimizeFromScratch` avoids it during search, so an INCRE interval is still
  slightly costlier than a REOPT interval. Fix C is the lever for a full flip.

Key observations:
1. **`INCR_P1` ≈ `INCR_SCRATCH`** (both ~95–106 ms/act) — reoptimizing every
   interval, with or without carrying the incumbent, costs about the same. So
   the from-scratch wide search itself is *not* the bottleneck at P=1.
2. **ET jumps ~3× at P=10 and keeps climbing to P=30, then plateaus** (P=30 ≈
   P=60). The plateau suggests the cost saturates: once the period is large
   enough that an incremental staleness path is exercised between reopts, extra
   period length adds no further per-activation cost.
3. **`INCR_P1` < `INCR_P10` < `INCR_P30` ≈ `INCR_P60`** — monotonic-then-flat,
   reproducibly, on two independent tasksets. The 30-interval run has exactly
   3 reopt steps at P=10, 1 at P=30, and 1 at P=60 (with the rest incremental);
   yet P=30/P=60 are *more* expensive per-activation than P=10, which has *more*
   reopts. This rules out "reopt is the expensive step" as the sole cause.

### What this implies (for the eventual fix, NOT applied now)

The cost is dominated by the **incremental** path between reopts, not by the
reopt step itself. Candidates worth investigating later:
- The incremental optimizer's per-call cost grows as the incumbent ages across
  many intervals (e.g. diff/warm-start state accumulates, or `OptimizeIncre`
  does work proportional to how stale the incumbent is).
- The narrow-radius coordinate descent (`IncrementalTimeLimitSearchRadius=2`)
  re-evaluates more TL options as the gap between incumbent and current DAG
  widens over a long period.
- A reopt every interval (P=1) keeps the incumbent fresh, so each incremental
  step is trivially cheap; a long period lets the incumbent drift, making each
  intervening incremental step more expensive — exactly the observed trend.

### Repro artifacts

- Script: `simulation_experiments/repro_et_grows_with_period.py`
- Results: `simulation_experiments/optimizer_comparison/et_repro/tasks6_ts0_reps3/et_repro_result.json`
  and `.../tasks6_ts2_reps1/et_repro_result.json` (plus per-arm
  `scheduler_execution_time.txt` alongside, written by the binary).
- Reused input tasksets (untouched): `runs/p25periodAB_run_prod_dur300_interval10_seed1000_tasks4x6x8/sim/tasks6_dur300_interval10_seed1000/taskset_{0,2}/`.

### Exact taskset & config used by the repro

The repro reuses the **P25 prod A/B run's** generated tasksets — no regeneration.
The prod run that produced them:

- **Run command:**
  ```
  MODE=prod CONFIG_JSON=simulation_experiments/configs/p25_period_ab_config.json \
      ./scripts/run_end_to_end.sh
  ```
- **Prod config (`simulation_experiments/configs/p25_period_ab_config.json`,
  `prod_mode` block):**
  - `num_tasks_for_cross_task_comparison = [4, 6, 8]` (repro uses the **6** branch)
  - `num_tasksets_to_generate = 5` (repro uses `taskset_0` and `taskset_2`)
  - `simulation_duration_seconds = 300`
  - `scheduler_trigger_interval_seconds = 10` → **30 scheduling triggers per run**
    (this is the per-activation ET divisor)
  - `main_scheduler_list = [BF, INCR_P1, INCR_P10, INCR_P30, INCR_P60, INCR_SCRATCH]`
  - `ablation_scheduler_list = []`
  - `interval_sweep_seconds_list = [10]`
  - `parallel_worker_processes = 4` ← the contention confound the prod run
    suffers and the serial repro removes
  - `enable_execution_time_profiling = true`, `export_detail_level = 1`
  - `base_random_seed = 1000` (deterministic tasksets, comparable across arms)
  - `number_of_gmm_trace_instances_per_path = 1`
  - `analysis.enable_resume_from_existing_results = true`,
    `analysis.skip_generation_if_exists = true`,
    `analysis.on_taskset_config_change = "prompt"`
- **Run root (P23 co-located layout):**
  `simulation_experiments/optimizer_comparison/runs/p25periodAB_run_prod_dur300_interval10_seed1000_tasks4x6x8/`
  — sims under `<run_root>/sim/`, figures under `<run_root>/figures/`.
  `run_name_prefix = "p25periodAB"` (from the config's `plotting` block).
- **Taskset generator config** (per-taskset, written next to the taskset as
  `generator_config.json` — this is the exact spec that produced the reused
  tasksets; `N_TASKS=6`, `RANDOM_SEED=1000`, `UPDATE_INTERVAL_S=10`):
  ```
  DESC: "Paper parameters for 6 tasks"
  PERIODS_MS:          [1000, 500, 200, 100, 50, 33, 20]
  Et_OVER_PERIOD_RANGE: [0.1, 0.3]
  SIGMA_OVER_Et_RANGE:  [0.5, 0.6]
  RO_1_Et_RANGE:        [-0.9, -0.7]
  RO_2_Et_RANGE:        [-0.1, 0.1]
  MAP_WIDTH_M: 200   MAP_HEIGHT_M: 200   ROBOT_SPEED_MPS: 1.0
  CPU_UTIL_RANDOM_RANGE: [0.5, 1.5]
  SP_THRESHOLD_RANGE: [0.5, 0.9]
  SP_THRESHOLDS_SET: [0.2, 0.4, 0.6, 0.8, 1.0]
  Et_SCALE_FACTOR: 4
  FINAL_Et_OVER_PERIOD_RANGE: [0.05, 0.9]
  N_GMM_COMPONENTS_PER_TASK: 4
  MAX_UTIL_PER_ENV_TASK: 0.45
  PERF_RECORD_TASK_PROBABILITY: 0.5
  N_TASKS: 6   N_CORES: 2   RANDOM_SEED: 1000
  MAX_UTIL_PER_TASK: 0.95
  MIN_PERIOD_WITH_PERFORMANCE_RECORDS: 0
  MIN_PERIOD_ENV_DEPENDENT: 0
  D1_RANGE: [-100, 100]   D2_RANGE: [-100, 100]
  UPDATE_INTERVAL_S: 10
  ```
  Each taskset's per-interval/per-path characteristics live alongside as
  `taskset_characteristics_i{interval}_p{path}.yaml` (60 files for the 6-task
  run: 30 intervals × 2 paths) — these are the actual inputs
  `RunOrchestrator` consumes.

### Repro run commands (exact)

```
# From repo root, after `cmake --build release` (Commit 6+ linked in):
python3 -m simulation_experiments.repro_et_grows_with_period --taskset 0 --reps 3
python3 -m simulation_experiments.repro_et_grows_with_period --taskset 2 --reps 1
```

Each invocation runs all six arms (`BF`, `INCR_P1`, `INCR_P10`, `INCR_P30`,
`INCR_P60`, `INCR_SCRATCH`) **serially** — one `RunOrchestrator` process at a
time — invoking:
```
release/tests/RunOrchestrator <taskset_dir> <out_dir>/<arm> <arm> 10000 1
```
where `10000` = per-interval horizon ms (`scheduler_trigger_interval_seconds ×
1000`), `1` = `export_level`. The binary writes
`<out_dir>/<arm>/<arm>/scheduler_execution_time.txt` (whole-process wall-clock
seconds); the script divides by `num_intervals=30` for the per-activation mean
and takes the min across `--reps`.

### DONE: A/B re-run to confirm the fix flipped the ordering (2026-07-04)

> **Owner:** next chat session. **Completed 2026-07-04 19:41.** Release rebuilt
> (19:34, HEAD `de4e9636`); prod-run tasksets were present locally (no
> regeneration needed). Before-snapshots preserved as
> `et_repro_result_BEFORE_fix.json` next to each after-result.

**Goal:** re-run the serial repro harness on the fixed binary and confirm the
per-activation ET ordering flipped from "grows with period" to
**non-increasing in period**: `INCR_P1 ≥ INCR_P10 ≥ INCR_P30 ≈ INCR_P60`
(larger `ReoptimizationPeriod` → fewer expensive wide-radius reopts → per-act
ET should *decrease* or stay flat, not increase).

**Prerequisites (must do first):**
1. `cmake --build release` — the on-disk release binary predates commits
   `986a9cfe` (Fix A) and `de4e9636` (Fix B + latent bug). Without a rebuild
   the A/B re-runs the *old* frozen-baseline code and will reproduce the bug,
   not confirm the fix. (Same "stale-binary gotcha" called out in Commit 7 §2.)
   ✅ Done — rebuilt 19:34; the uncommitted `tests/RunOrchestrator.cpp`
   `INCR_P<n>` period-override edit is required for the A/B (committed
   `SimulationOrchestrator.cpp` `IsINCRPeriodVariant` depends on it) and was
   compiled in.
2. Confirm the reused prod-run tasksets exist locally:
   `simulation_experiments/optimizer_comparison/runs/p25periodAB_run_prod_dur300_interval10_seed1000_tasks4x6x8/sim/tasks6_dur300_interval10_seed1000/taskset_{0,2}/`.
   ✅ Both present (no regeneration needed).

**Run (serial, contention-free — same harness as the before snapshot):**
```
python3 -m simulation_experiments.repro_et_grows_with_period --taskset 0 --reps 3
python3 -m simulation_experiments.repro_et_grows_with_period --taskset 2 --reps 1
```
✅ Both ran clean (exit 0). Results in
`simulation_experiments/optimizer_comparison/et_repro/{tasks6_ts0_reps3,tasks6_ts2_reps1}/et_repro_result.json`
(before-snapshots preserved as `et_repro_result_BEFORE_fix.json`).

**Verdict — partial pass (bug fixed; literal directional flip NOT achieved,
for an understood reason):**
- ✅ **Numerical pass criterion met.** P30/P60 do **not** exceed P10 by the
  1.1–1.3× margin they did before; and P10/P30/P60 collapsed toward the
  P1 ≈ INCR_SCRATCH floor. ts0: P30/P60 = 1.035× P10 (was 1.14×), 342→91 ms/act.
  ts2: P30/P60 = 0.96–0.98× P10 (was 1.05×), 193→72 ms/act. The pathological
  3.4× / 2.1× P60/P1 growth is gone (now 1.47× / 1.13×).
- ❌ **Literal directional criterion NOT met.** `INCR_P1 ≥ P10 ≥ P30 ≈ P60`
  (strictly non-increasing) is false on both tasksets: P1 is still the
  *cheapest* INCR arm (ts0 62 < 88–91; ts2 65 < 72–75). The direction did not
  flip — it flattened.
- **Why (understood, not a regression):** §4a/§6 of `debug_runtime0704_incr.md`
  predict exactly this for a Fix A+B-only deployment. Fix A kills the
  frozen-baseline pathology (the actual bug — `ndiff` saturating at 6 every
  incremental interval → ~132 `ObtainSP_DAG`/interval). But the residual
  P1 < P60 gap is the per-variation `ObtainSP_DAG` asymmetry: `OptimizeIncre`
  re-scores every priority variation with the full `ObtainSP_DAG` kernel,
  while `OptimizeFromScratch` uses cheap `GetRTA_OneTask` during its beam
  search and calls `ObtainSP_DAG` only once at the end. So an INCRE interval
  is still slightly costlier than a REOPT interval, and P1 (all-REOPT) beats
  P60 (mostly-INCRE). **Fix C** (deferred in §6 — make `OptimizeIncre`'s
  per-variation scoring incremental too) is the lever for a full directional
  flip. Fix A+B brought INCRE *down from pathological* (3.4× REOPT) to
  *moderately above* REOPT (1.13–1.47×); Fix C would bring it *below*.

**Conclusion:** the issue documented here ("INCR per-act ET grows with
reoptimization period") is **resolved** in the sense that motivated the bug
report — the period no longer inflates ET by 3×; P10/P30/P60 are flat and
near the floor. The stricter "ET decreases as period grows" expectation was
based on the same root-cause analysis that §7 of `debug_runtime0704_incr.md`
later showed to be *mechanistically* wrong (cost scales with "is the
incremental path exercised," not with staleness) — so the literal flip was
over-optimistic given Fix C was deferred. Re-opening for a full flip is a
Fix C task, not a Fix A/B regression.

**On completion:**
- ✅ Added an "AFTER fix A+B" table directly beneath the before table above
  (before/after side-by-side with ratios).
- ✅ Updated `agents/dev_log.md` with the after numbers + the verdict.
- ✅ Flipped this section's header from "PENDING" to "DONE (2026-07-04)" and
  updated the memory file `p25-incr-et-grows-with-period.md`'s "Runtime A/B
  re-run (STILL PENDING)" line to resolved (partial pass — bug fixed, literal
  flip deferred to Fix C).
- Related: `agents/debug_runtime0704_incr.md` (full root-cause + fix record),
  memory [[p25-incr-et-grows-with-period]].

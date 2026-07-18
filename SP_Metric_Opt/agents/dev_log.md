# Development Log

> ## 2026-07-13 — P1.1 Idea 10 (Single-Point Convolve Fast Path): DONE + measured
>
> `FiniteDist::Convolve` (`sources/Safety_Performance_Metric/Probability.cpp`)
> now short-circuits to the `FiniteDist::ConvolveSinglePoint` member (declared in
> `Probability.h`) when either operand is a single support point `{(v, p)}` — an
> O(N) value-shift + probability-scale + adjacent coalesce via the file-scope
> `ShiftAndCoalesce` helper, no `std::sort` (uniform shift preserves sorted
> order). The general N×M + sort + coalesce path is unchanged. TDD: 7 new tests
> in `tests/testProbability.cpp` (`Convolve_SinglePoint*`); `cmake --build .
> --target check.SP_OPT -j5` → 16/16 ctest green. Refactored per code-style
> feedback (no anonymous namespace; `ConvolveSinglePoint` is a member with an
> explicit `else if` + `CoutError` precondition check; `ShiftAndCoalesce` is the
> plain file-scope free function). **Measured ~2.9–3.0× per-Convolve** at
> Granularity 5/10/20 — real but below the earlier ~10× estimate (the savings
> are the eliminated temp allocations + emplace/push/move, NOT the near-free
> `std::sort`); end-to-end noticeability unmeasured. Full record:
> `agents/active_tasks/P1_1_efficiency_optimizations/`.

> ## 2026-07-12 — P2.3 `FiniteDist::approx_equal` dead-code cleanup: DONE (wire-up, working tree only)
>
> `FiniteDist::operator==` (`Probability.cpp:368-370`) now delegates to
> `approx_equal(other, 1e-1)` instead of a duplicate inline loop — `approx_equal`
> is no longer dead code (it is the delegated-to body). Tolerance kept at the old
> inline `1e-1` (NOT the `1e-2` first attempted) so the change is
> behavior-neutral; tuning this tolerance is the ruled-out P25 "Fix D" lever.
> `testProbability` green (incl. `FiniteDist.equal` +
> `approx_equal_respects_tolerance`). **16/16 ctest green** — also fixed a
> pre-existing build-wiring gap that had `testScheduleSimulate`'s
> `CFS_RunOrchestrator_Binary` failing: the standalone `RunOrchestrator` binary
> (`tests/CMakeLists.txt:19-20`, declared outside the `gtsamAddTestsGlob` glob)
> was never built by `check.SP_OPT`; added `add_dependencies(check.SP_OPT
> RunOrchestrator)` (guarded by `if(TARGET check.SP_OPT)`), clean-build
> verified. Full record:
> `agents/active_tasks/P2_3_finite_dist_dead_code/`.

> ## SESSION STATUS (as of 2026-07-04, Fix A + Fix B committed) — read this first
>
> **Branch `clean_simulation`. HEAD = `de4e9636`.** P25 Commits 1–6 are
> **committed**, and the post-Commit-6 INCR-ET bug fix bundle (Fix A + latent
> `SeedStateFromIncumbent` bug + Fix B) is also **committed**. `ctest` 16/16
> green in the DEBUG build. The **only** open P25 thread is the runtime A/B
> re-run (see Open thread 1 below).
>
> - `9b3eeacf` — **Commit 6**: counter-driven dispatcher + radius-knob rename +
>   radius-forwarding fix (described below).
> - `986a9cfe` — **Fix A**: `OptimizeIncre` advances `dag_tasks_` after the diff
>   loop (the frozen-baseline root cause of the INCR-ET-grows-with-period bug;
>   full record in `agents/debug_runtime0704_incr.md`).
> - `de4e9636` — **Fix B + latent prerequisite**: `{-1}`-only skip + zero-work
>   fallback in `PerformCoordinateDescentForTaskConfigOpt`, and the
>   `prev_optimizer_.sp_parameters_` carry in `SeedStateFromIncumbent`.
>
> **Commit 6 — counter-driven dispatcher wired in.**
> `Optimize_w_TL_ScratchOrIncre(dag,K)` is added to `OptimizeSP_TL_Incre.{h,cpp}`:
> a persistent `reoptimization_interval_count_` (starts 0, advances every call,
> **never resets**) drives a modular routing decision — `count %
> ReoptimizationPeriod == 0` → wide-radius `ReOptimizePeriodic` (compare-and-
> keep, `ReoptimizationTimeLimitSearchRadius`), else → narrow-radius
> `OptimizeIncre_w_TL` (`IncrementalTimeLimitSearchRadius`). At `count == 0` this routes
> to `ReOptimizePeriodic`, whose `SeedIncumbentBaseline` interval-0 branch
> synthesizes the RM+min-TL incumbent — so the dispatcher doubles as the
> interval-0 bootstrap, fixing a latent bug where the INCR paths threw
> (`CoutError` at `OptimizeSP_TL_Incre.cpp:153`) on the very first interval
> because `prev_optimizer_` was uninitialized. The orchestrator's
> INCR/INCR_NO_TL/INCR_WCET branches (`SimulationOrchestrator.cpp:267,281,289`)
> now call the dispatcher instead of `OptimizeIncre_w_TL` directly; the
> `disable_time_limit_opt` / `use_wcet_execution_time` flag save/restore is
> preserved. `INCR_SCRATCH` is untouched (separate ablation, 1-arg
> `ReOptimizePeriodic`).
>
> **Radius-forwarding fix (folded into Commit 6).** The 2-arg
> `OptimizeIncre_w_TL(dag,K)` was forwarding to `ReoptimizationTimeLimitSearchRadius`
> (6, wide) instead of `IncrementalTimeLimitSearchRadius` (2, narrow) — a leftover
> from the Commit 5 refactor that collapsed the two-radius design. Corrected to
> forward to `IncrementalTimeLimitSearchRadius`, restoring Commit 1's intent
> (`P24_task.md:93`). The `OptimizeWithOptimizationSpace` test assertion was
> updated: under radius 2 the incremental window for ET=1000 is [600,800,1000]
> (TL=400 unreachable), and TL=600 is the schedulable optimum under the new DAG
> (800/1000 unschedulable), so the result is in [600,1000] — not the bootstrap
> TL=400.
>
> **`ReoptimizationPeriod == 0` semantics dropped.** Per user decision the knob
> is now positive-only (min 1); the "0 disables" path is gone (it would have
> crashed the INCR interval-0 bootstrap, since `0 % 0` is undefined / the
> incremental branch has no incumbent). `parameters.yaml:12` comment to be
> updated in Commit 7.
>
> **Radius-knob rename (mechanical, no behavior change).** The two TL-window
> radii were renamed to a consistent `Reoptimization`/`Incremental` prefix
> style, matching the sibling `ReoptimizationPeriod` knob:
> `TimeLimitSearchRadiusIncr` → `IncrementalTimeLimitSearchRadius` (2, narrow);
> `ReoptimizationTimeLimitsSearchRadius` → `ReoptimizationTimeLimitSearchRadius`
> (6, wide; also singularized `TimeLimit`). Touched 9 files: `parameters.yaml`
> (YAML keys), `Parameters.{h,cpp}`, `OptimizeSP_TL_Incre.cpp`,
> `tests/{testIncreOpt_w_TL,testBF_w_TL}.cpp`,
> `tests/debug_analysis/run_radius_comparison.py` (reads/writes the YAML key —
> moved in lockstep), `agents/{tasks,P24_task}.md`. Historical session log
> `agents/claude_sessions/session_clean_simulation_2026-06-21.md` deliberately
> NOT rewritten (point-in-time record). `ctest` 16/16 green after rebuild.
> **Gotcha:** `cmake --build build` reported success but did NOT relink the
> test executables when only the shared lib's global changed — had to rebuild
> `testIncreOpt_w_TL`/`testBF_w_TL` explicitly. Watch for this on future
> global/header renames.
>
> **Tests.** `testIncreOpt_w_TL` = **26/26 green** (23 prior + 3 new dispatcher
> tests under `CounterDispatcherSynthetic`: `CounterAdvancesEveryCall_NeverResets`,
> `TriggersReoptAtCountZero_BootstrapsIncumbent`,
> `RoutesToIncrementalAtNonModularCount`). Full `ctest` **16/16 green**. Full
> build clean (RunOrchestrator + AnalyzePriorityAssignmentIncrementalExample
> link).
>
> **Working-tree noise (unrelated to P25, pre-existing):** `git status` shows
> modifications to `Gen_Taskset/tests/test_integration.py`,
> `simulation_experiments/{compare_optimizers,run_end_to_end_experiments,
> run_sim_experiments}.py`, `tests/python/test_trajectory_physical.py`, and the
> `agents/*.md` docs. These are not part of P25 Commit 6; left alone.
>
> **Open threads:**
> 1. ~~**P25 runtime A/B re-run (the only pending P25 item).**~~ **RESOLVED
>    2026-07-04 19:41 — PARTIAL PASS.** Fix A + Fix B were committed and
>    TDD-green; the runtime A/B was re-run on the fixed binary (rebuilt 19:34,
>    HEAD `de4e9636`). **Result:** pathological 3× growth eliminated (P10/P30/P60
>    collapsed 301–343→88–91 ts0, 185–194→72–75 ts2, toward the P1≈INCR_SCRATCH
>    floor; P60/P1 3.4×→1.47× ts0, 2.1×→1.13× ts2; numerical pass bar P30/P60 vs
>    P10 ≤ 1.1–1.3× met on both) BUT the literal flip
>    `INCR_P1 ≥ P10 ≥ P30 ≈ P60` was NOT met (P1 still cheapest) — residual gap
>    is the per-variation `ObtainSP_DAG` asymmetry **Fix C** (deferred) addresses.
>    Full before/after tables + procedure: `agents/finished_tasks/P24_task.md`
>    § "DONE: A/B re-run"; see also the 2026-07-04 19:41 entry below. (Original
>    thread text, kept for the procedure record: `cmake --build release` first
>    — the on-disk release binary predated `986a9cfe`/`de4e9636`, so without a
>    rebuild the A/B re-runs the *old* frozen-baseline code and reproduces the
>    bug instead of confirming the fix — then
>    `python3 -m simulation_experiments.repro_et_grows_with_period --taskset 0
>    --reps 3` and `--taskset 2 --reps 1`, serial/contention-free. Prod-run
>    tasksets were present locally, no regeneration needed. Parallel-period-sweep
>    variant config: `simulation_experiments/configs/p25_period_ab_config.json`
>    via `MODE=prod CONFIG_JSON=<that> ./scripts/run_end_to_end.sh`; the period-0
>    "off" baseline was dropped in Commit 6, `parameters.yaml:12` comment updated
>    in `71eb6aa8`.)
> 2. **Memory file `p24-reoptimization-design.md`** — rewritten this session to
>    match the 4-tuple model + Commit 6's dispatcher + the radius rename (was
>    stale: described the removed `restore_incumbent`/`SumTimeLimits` design and
>    said Commit 5 was uncommitted).
> 3. **P18 (per-interval >0.1 s slowness at N≥8)** — real bottleneck per the
>    P24-Eval numbers (N=12 period-0 mean 0.80s/activation). Flagged, not
>    blocking.

## 2026-07-01

### Investigation: Does mean SP grow with N? + SP normalization design (P12)

User asked to investigate whether mean SP increases as the number of tasks per
task set increases (their hypothesis: more tasks → higher SP upper bound → mean
SP trends up), and to add a config option to normalize SP to `[0,1]` per
task-set size for fair cross-N comparison.

**SP-metric definition (traced through C++):**
- `SP_Func(violate_prob, threshold)` (`SP_Metric.h:31-41`) interpolates from
  `PenaltyFunc(1,thr) → 0` to `RewardFunc(0,thr) → 1`, so `SP_Func ∈ [0, 1]`.
- `ObtainSP_TaskSet` (`SP_Metric.cpp:53-68`) = `Σ_i SP_Func(ddl_miss_i, thr_i)
  × weights_node_i × perf_coefficient_i`. Node term only — the chain/path term
  lives in `ObtainSP_DAG`, but the simulation exports
  `ObtainSP_TaskSet_And_TimeLimits` (node-only) at
  `SimulationOrchestrator.cpp:482,703`, so realized SP has no path term.
- `perf_coefficient = 1.0` when `timePerformancePairs` is empty
  (`RegularTasks.h:83-89`); confirmed generated tasksets have **no**
  `timePerformancePairs` and **no** `chains`.

**Ceiling is constant across N (does not grow):** the generator normalizes
`sp_weight` so `Σ sp_weight = SP_WEIGHTS_SUM = 5.0` for every task set,
regardless of N (`taskset_generator.py:475-480`). Verified on disk: N=4 →
1.25×4 = 5.0; N=6 → 0.714×5 + 1.428×1 = 5.0. Therefore the theoretical SP
upper bound per interval ≈ 5.0 × 1.0 = 5.0, independent of N. The realized
fraction of that ceiling **decreases** with N: on the dur70 test data, BF (the
best a scheduler can reach) achieves ~3.80 (≈0.76 of ceiling) at N=4 but only
~2.95 (≈0.59) at N=6 — more tasks = more contention = harder to meet all
deadlines.

**Conclusion:** the user's hypothesis (ceiling grows with N) is **not** how
this generator behaves — the weight sum is actively normalized to a constant,
so the ceiling is flat; realized SP *falls* with N. Because both the ceiling
and a strong reference (BF) are flat/known, two valid normalizations preserve
"fraction of best achievable" semantics: divide by BF's SP at each N
(`reference_scheduler`, the implemented **default** — direct optimality gap),
or divide by the flat ~5.0 ceiling (`upper_bound`). min-max per N is available
as a fallback. See the P12 implementation entry below.

### Implementation: SP normalization config + Fig 1A-normalized variant (P12)

Added a normalization layer in the **aggregation** stage (raw values still
stored in the CSV; normalization is a plotting/reporting transform, not a
re-simulation):

- `configs/experiment_config.json` → new `analysis` keys:
  - `normalize_sp` (bool, default `false`): master toggle.
  - `sp_normalization_method` (`"upper_bound"` | `"minmax_per_task_count"`,
    default `"upper_bound"`): scheme selector.
- `aggregate_across_tasks.py`:
  - `compute_sp_upper_bound(experiment_dir)` — reads
    `taskset_characteristics_interval_0.yaml` from a representative taskset
    in the experiment dir, returns `Σ sp_weight × perf_coefficient`
    (perf_coefficient defaults to 1.0 since generated tasksets carry no
    `timePerformancePairs`). Falls back to `None` if no YAML is found.
  - `normalize_records_sp(records, cfg, ...)` — per task count, divides
    `mean_sp`/`std_sp` by the per-N upper bound (`upper_bound` method) or
    applies min-max across schedulers at fixed N (`minmax_per_task_count`).
    Returns a **new** record list; raw records are untouched. Handles the
    degenerate all-equal / zero-ceiling case (returns the raw value).
  - When `normalize_sp` is on, `generate_main_group_figures` and
    `generate_ablation_group_figures` emit an **additional** normalized Fig 1A
    / ablation SP figure (`fig1a_mean_sp_normalized_vs_tasks_main`,
    `fig_ablation_mean_sp_normalized_vs_tasks`) alongside the raw figure, so
    both views are available for the paper.

### Implementation: ET figure log-scale Y axis (P11)

`build_line_chart` now accepts a `log_y` kwarg. The mean-execution-time
figures (Fig 1C main, Ab-B ablation) pass `log_y=True` → `ax.set_yscale("log")`.
SP/miss-rate figures remain linear.

The Y-axis tick labeling was rewritten after the original `ScalarFormatter`
approach produced squeezed-together, unreadable labels on real data (N=6
spans 0.026s…1.62s, ~2 orders of magnitude). Two defects were fixed:

1. **Minor ticks were labeled.** On a log axis matplotlib emits minor ticks at
   every integer 2×..9× within each decade; labeling all of them produces a
   wall of overlapping numbers. Fix: only the **major** (decade) ticks are
   labeled; minor ticks get gridlines but `set_minor_formatter` is never
   called.
2. **`ScalarFormatter` collapses small decades to "0.00".** It picks one fixed
   decimal place for the whole axis, so once the range includes large values
   (1.62) the sub-0.01 decades all render as `0.00` → a stack of identical
   "0.00" labels. Fix: replaced `ScalarFormatter` with a `FuncFormatter` using
   `f"{val:g}"`, which formats each decade tick on its own scale
   (`0.0001`, `0.001`, `0.01`, `0.1`, `1`), all distinct plain numbers.

Verified on the on-disk N=4/N=6 dur70 data: major labels now
`0.0001 / 0.001 / 0.01 / 0.1 / 1`, minor ticks unlabeled. Test
`test_log_y_sets_log_scale` extended to assert `set_major_formatter` is called
once and `set_minor_formatter` is never called. Full suite 193 passing.

### Implementation: reference_scheduler normalization + extend to all SP figures (P12, cont.)

Two follow-ups to the P12 normalization work above (which only added a
normalized **Fig 1A** / ablation variant and documented two methods with
`upper_bound` as default). Both are superseded by this session.

#### (1) New default method: `reference_scheduler`

`normalize_records_sp` gained a third method, now the default: divide each
scheduler's SP by the **reference scheduler's** SP at the same task count
(`analysis.sp_normalization_reference`, default `BF`). BF is brute-force
enumeration over all priority assignments + time limits — the best a
scheduler with unlimited computation could reach — so it plots at 1.0 and
every other scheduler shows its optimality gap (≤ 1.0). This is more
meaningful for the paper than `upper_bound` (which divides by the flat
~5.0 weight-sum ceiling) because it answers "how close to optimal is each
method" directly. `upper_bound` and `minmax_per_task_count` remain
selectable. Config: `normalize_sp: true`,
`sp_normalization_method: "reference_scheduler"`, `sp_normalization_reference: "BF"`.

#### (2) Normalize every SP-metric figure, consistently

User feedback: "extend normalization to all SP-metric figures, applied
consistently." Previously only Fig 1A and Ab-A got a normalized variant.
Now every figure whose y-axis is an SP value emits a normalized variant
using the **same** reference-scheduler transform, computed from each
figure's own data (no shared state, no signature churn):

- **Fig 1A** (`fig1a_mean_sp_normalized_vs_tasks_main`) — mean SP, main group.
- **Fig 1B** (`fig1b_std_sp_normalized_vs_tasks_main`, debug) — std SP; the
  `norm_records` computed once for 1A is reused (its `std_sp` is already
  scaled by the per-N reference divisor).
- **Fig 1F** (`fig1f_sp_distribution_boxplot_normalized`) — SP distribution
  boxplot at a fixed task count; divides every raw SP value by BF's mean
  SP at that count so BF centers at 1.0. Factored out a `_draw_sp_boxplot`
  helper so raw + normalized share drawing code. Skips cleanly if the
  reference scheduler is absent.
- **Fig Ab-A** (`fig_ablation_mean_sp_normalized_vs_tasks`) — ablation mean SP.
- **Fig 2** (`fig2_sp_vs_interval_normalized`) — INCR SP vs. trigger interval;
  `interval_sweep.generate_figure_2` now emits a normalized variant where each
  scheduler's mean/std SP at interval *i* is divided by BF's mean SP at *i*
  (BF reference line → 1.0). Factored out `_draw_figure_2` +
  `_normalize_sweep_data` helpers.

#### Cleanup

- Removed a no-op ternary in `compute_sp_upper_bound`:
  `perf_coeff = 1.0 if not has_perf_pairs else 1.0` (both branches 1.0) →
  plain `perf_coeff = 1.0` with a clarifying comment.
- Module docstrings in `aggregate_across_tasks.py` and `interval_sweep.py`
  list the new normalized outputs; the config `_comment_normalize_sp` reflects
  the new default + full figure coverage.

#### Verification

- `python3 -m pytest tests/python/` → **195 passed** (+8 new: main-group
  1A/1B normalized call-count + stems; 1F normalized emitted / skipped on
  missing reference; new `test_interval_sweep.py` with Fig 2 normalization +
  `_normalize_sweep_data` unit tests).
- `python3 -m simulation_experiments.aggregate_across_tasks --mode test`
  against existing test-mode sim data → all normalized figures generated
  (1A, 1B, 1F, Ab-A). Fig 2 normalized verified separately against the
  existing sweep CSVs. 15 PNGs (+ matching PDFs) total in the run figures dir.

### Correction: normalize SP by the ideal (theoretical-ceiling) SP, not BF's value (P12 rework)

User feedback: "after normalization, 1.0 SP implies the best possible results
if we have infinite computation resources (i.e., all tasks meet deadline
perfectly, all task config optimization uses the best possible option).
Therefore, during normalization, we should divide raw SP by this ideal SP
value. Normalization has nothing to do with BF's value."

This overturns the `reference_scheduler` default adopted earlier this session
(§"reference_scheduler normalization + extend to all SP figures" above). That
approach was wrong on two levels:

1. **BF is a realized value, not a ceiling.** BF = brute-force enumeration over
   priority assignments + time limits is the best *single scheduling run*, but
   its mean SP is a number another scheduler can tie or exceed — on the dur70
   test data INCR ties BF at N=4 (both 3.7972). SP is not bounded above by BF,
   so dividing by BF does not bound the ratio at 1.0.
2. **Boxplot divide-by-mean produced values > 1.0.** Fig 1F divided each
   per-interval SP point by BF's *mean*. A boxplot's upper whisker naturally
   exceeds the mean, so points landed at **1.30 (N=4) / 1.11 (N=6)** — the
   "1.2" the user spotted. This is a category error: a per-point value was
   normalized by a per-set statistic.

**Correct divisor — the ideal SP:** with infinite computation every task meets
its deadline perfectly (`SP_Func = 1` for all tasks) and the optimizer picks
the best config, so

    ideal_SP = Σ_i sp_weight_i × perf_coefficient_i

This is exactly `compute_sp_upper_bound(experiment_dir)` (reads
`taskset_characteristics_interval_0.yaml`). Because `SP_Func ∈ [0,1]` with
non-negative weights, **raw SP ≤ ideal_SP always**, so `raw / ideal ∈ [0, 1]`
with 1.0 = "all deadlines met perfectly". The generator normalizes
`Σ sp_weight = 5.0` per task set, so `ideal_SP = 5.0` (flat across N) — the
denominator is a *constant ceiling*, not a per-scheduler mean, so no figure can
exceed 1.0 and the boxplot upper whiskers stay in range. This also makes the
cross-N comparison honest: N=4 BF ≈ 3.80/5.0 = 0.76, N=6 BF ≈ 2.95/5.0 = 0.59
— the realized fraction of the ideal, falling with N as contention rises.

**Implementation (applied):**
- Made `upper_bound` (ideal-SP) the **default and primary** method. This is
  the only method that satisfies "1.0 = best possible with infinite
  computation".
- **Removed `reference_scheduler`** + the `sp_normalization_reference` config
  key — it conflates a realized baseline with the theoretical ideal and is the
  source of the >1.0 bug. Kept `minmax_per_task_count` as a non-default debug
  option, clearly labeled as relative-ranking (not fraction-of-ideal).
- All SP figures (1A, 1B, 1F, Ab-A, Fig 2) divide by the same per-N `ideal_SP`
  (constant ~5.0). The boxplot divides each point by the ceiling (not BF's
  mean), fixing the >1.0 whiskers. `interval_sweep.generate_figure_2` now
  takes an `ideal_sp` arg (computed from any sweep experiment dir via
  `compute_sp_upper_bound`), and `_normalize_sweep_data(data, ideal_sp)`
  divides by that constant.
- Updated config `_comment_normalize_sp`, module docstrings, and tests.

**Verification:**
- `python3 -m pytest tests/python/` → **193 passed** (was 195; net −2 from
  consolidating 4 `reference_scheduler` unit tests into 2 `upper_bound`-default
  tests; sweep tests gained `test_normalized_skipped_when_no_ceiling`).
- Regenerated all figures on the on-disk dur70 test data; recomputed the max
  normalized boxplot point: **0.99 (N=4) / 0.66 (N=6)** — both ≤ 1.0, where
  1.0 = theoretical ideal (all deadlines met). The previous 1.30/1.11 are
  gone. `ideal_SP` ceiling verified = 5.0 on both N=4 and N=6.



User feedback: "code refactor, remove repeated code. example: run_sim_4 6 8 tasks.sh"

The `scripts/` directory had grown three byte-identical clones
(`run_sim_{4,6,8}_tasks.sh`, differing only by `NUM_TASKS=N`) plus copy-pasted
boilerplate (SCRIPT_DIR/PROJECT_ROOT resolution, the `====` header/footer
banners, the binary-existence guard, and the INT/TERM `kill 0` trap) across all
five runner scripts.

#### (1) New sourced library: `scripts/lib/common.sh`

Centralizes the shared pieces so each runner sources it instead of duplicating:
- `set -euo pipefail` + `SCRIPT_DIR`/`PROJECT_ROOT` resolution (caller sets
  `SCRIPT_DIR`, the lib resolves `PROJECT_ROOT`).
- `print_header "Title" "Key: val" ...` / `print_footer ["extra line"]`.
- `require_binary "${SIM_BIN}"` — the RunOrchestrator guard.
- `register_signal_trap` — installs the Ctrl-C/TERM `kill 0` trap.

#### (2) Collapsed `run_sim_{4,6,8}_tasks.sh` into `run_simulation.sh`

Deleted the three clones. `run_simulation.sh` now accepts an optional
**positional task-count** that overrides `NUM_TASKS`, so:
```bash
./run_simulation.sh 4          # replaces ./run_sim_4_tasks.sh
NUM_TASKSETS=2 ./run_simulation.sh 6
```
The `^[468]$` regex validation is unchanged. `run_all_experiments.sh` now loops
`NUM_TASKS=N ./run_simulation.sh` for 4/6/8 instead of calling the deleted
wrappers.

#### (3) Remaining runners source `common.sh`

`run_all_experiments.sh`, `run_interval_sweep.sh`, `run_paper_figures.sh`, and
`run_end_to_end.sh` each reduced to: source lib → set local vars →
`print_header` → run python module → `print_footer`. The duplicated header
banner, binary guard, and trap blocks were removed in favor of the shared
helpers. `run_end_to_end.sh` keeps its dry-run-skips-binary-guard behavior.

#### Verification

- `bash -n` passes on all six files (`common.sh` + five runners).
- `./run_simulation.sh 4` resolves the positional arg and launches correctly.
- `DRY_RUN=1 ./scripts/run_end_to_end.sh` (test mode): header renders via
  `common.sh`, orchestrator builds the correct simulate→sweep→aggregate commands
  from config, dry-run respected.
- `./scripts/run_all_experiments.sh` chains into `run_simulation.sh` with
  `NUM_TASKS=4` after printing its header.
- `python3 -m pytest tests/python/` → **171 passed** (no regressions; no test or
  Python module referenced the deleted wrappers).
- All five runners keep `+x`.

---

### Session Summary: Per-run figure namespacing, bar→line, and always PNG+PDF export

Three related changes to the simulation-experiment figure pipeline, all verified
with a real `./scripts/run_end_to_end.sh` test-mode run (exit 0, ~26–36s) and the
full Python suite (**171 passed**).

#### (1) Group figures by run-time config (fixes cross-run clobbering + aggregation pollution)

User feedback: every run's figures were dumped into one flat folder
(`optimizer_comparison/figures/`), so test-mode and prod-mode (or runs with
different durations/seeds) silently clobbered each other's `fig1a…fig3b` files.
The dev-log aggregation bug (stale `tasks6_dur300_*` dirs ingested with
last-write-wins, yielding 23 records instead of 14) was the data-side symptom.

- **`simulation_experiments/experiment_config_loader.py`** — added
  `build_run_id(cfg)` → e.g. `run_test_dur70_interval10_seed1000_tasks4x6`.
  Keyed on mode + duration + trigger interval + base seed + task-count list;
  `num_tasksets` intentionally excluded (sampling depth, not run identity).
- **`simulation_experiments/aggregate_across_tasks.py`** —
  `aggregate_data_from_directories(cfg, output_parent)` now scopes ingestion to
  dirs matching `tasks{N}_dur{D}_interval{I}_seed{S}` for the current run,
  excluding both `*_sweep_*` dirs and stale dirs with a different dur/interval/
  seed. `main()` writes figures to `runs/<run_id>/figures/`. Added `--output_parent`
  CLI; `generate_*` take an optional `figures_dir`. Legacy no-`cfg` path preserved.
- **`simulation_experiments/interval_sweep.py`** — fig2 written under
  `runs/<run_id>/figures/`.
- **`simulation_experiments/run_end_to_end_experiments.py`** —
  `build_aggregate_command(cfg, output_parent)` appends `--output_parent`; final
  "Figures:" print shows `runs/<run_id>/figures`.
- New test `test_aggregation_excludes_stale_dirs` in `tests/python/test_aggregate.py`
  locks in the scoping (scoped → only dur70 record; legacy → both).

Real run confirmed: **14 records** (2 task counts × 7 schedulers), figures land
in `runs/run_test_dur70_interval10_seed1000_tasks4x6/figures/`, flat `figures/`
no longer written by the pipeline.

#### (2) No bar plots — line plots everywhere

User feedback: "no bar plots anywhere — line plots always."
- **`aggregate_across_tasks.py`** — `build_grouped_bar_chart` → `build_line_chart`
  (one line per scheduler over num_tasks, `ax.errorbar` with markers); fig3/3b
  converted to connected-point lines over scheduler index.
- **`compare_optimizers.py`** — `plot_optimizer_bar_comparison` →
  `plot_optimizer_sp_line`, `plot_optimizer_exec_time` →
  `plot_optimizer_exec_time_line`, `plot_per_taskset_radar` →
  `plot_per_taskset_line` (all bar→line).
- Box plots (Fig 1F, `comparison_plots` boxplot panel) kept — not bar plots.
- Figure filenames unchanged to limit churn.

#### (3) Always export both PNG and PDF

User feedback: "figures, export both pdf and png figures, always."
The paper-figure path (`save_figure`, default `["png","pdf"]`) already did both;
the gap was the **per-run debug plots**, which used raw `plt.savefig(..., dpi=300)`
with hardcoded `.png` names and never produced PDF.
- **`compare_optimizers.py`** — the three per-run plot fns now route through
  `save_figure()` (strips a trailing `.png` to form the stem, so call sites like
  `optimizer_sp_bar.png` are unchanged and now also emit `.pdf`).
- **`utils.py`** — `write_summary_and_plots` uses `save_figure(fig, "comparison_plots")`
  (PNG+PDF) instead of `plt.savefig`.
- Tests updated to mock `save_figure` (one call with the stem) instead of
  `plt.savefig` (one `.png` call): `test_compare_optimizers.py` (3 tests),
  `test_run_sim_experiments.py::test_write_summary_and_plots`.

Real run confirmed: per-run plots (`optimizer_sp_bar`, `optimizer_exec_time`,
`optimizer_per_taskset`, `comparison_plots`) each now have `.png` **and** `.pdf`;
11 paper figures = 11 PNG + 11 PDF.

---

### Session Summary: Unified End-to-End Orchestrator (P10)

User feedback: "when I say a script that runs end-to-end, I mean one script that does everything, rather than manually doing step 1 2 3 4 ... one script that has all those things based on simulation_experiments/configs/experiment_config.json."

Delivered a single entry point that runs the full pipeline from the JSON config, replacing the manual 3-step workflow.

#### New files

- **`simulation_experiments/run_end_to_end_experiments.py`** — orchestrator with three stages, each delegating to an existing module:
  - **simulate** → `compare_optimizers.py` once per task count in `num_tasks_for_cross_task_comparison`, passing the **union** of `main_scheduler_list` + `ablation_scheduler_list` (deduped, order-preserving) so a single simulation pass feeds both the main-group and ablation-group figures.
  - **sweep** → `interval_sweep.py` (Fig 2).
  - **aggregate** → `aggregate_across_tasks.py` (Figs 1A-1F, ablation, Fig 3).
  - Every experiment parameter is read from `experiment_config.json` via `load_experiment_config(mode)`. The only CLI flags are `--mode`, `--steps`, `--bin_dir`, `--verbose`, `--config_json`, `--dry_run` (no experiment knobs are hardcoded).
  - Stages run in fixed order; `--steps` selects a subset; a failed stage aborts the pipeline with non-zero exit. Pre-flight checks the binary exists when a simulating stage is selected.
  - Key helpers: `build_scheduler_union(cfg)`, `build_simulate_command(num_tasks, cfg, output_parent, verbose)`, `build_sweep_command(cfg, output_parent, verbose)`, `build_aggregate_command(cfg)`, `run_command(cmd, dry_run)`.

- **`scripts/run_end_to_end.sh`** — bash wrapper. `MODE` (default test), `STEPS`, `BIN_DIR` (default release), `VERBOSE`, `DRY_RUN`, `PYTHON` env vars; validates binary, traps Ctrl-C/TERM, prints header + timing, calls the Python orchestrator. Passes `bash -n`; executable (`+x`).

- **`tests/python/test_run_end_to_end.py`** — 16 tests: scheduler-union dedup/order, per-stage command construction (config values, `--resume`, `--num_workers`, important-task %), `run_command` dry-run/no-exec/failure semantics, and `main()` pipeline tests (all-stages / subset / simulate-only dry-run counts, failure-abort). All pass.

#### Correctness fix found while wiring the orchestrator

- **`simulation_experiments/aggregate_across_tasks.py`** — `aggregate_data_from_directories()` now skips directories whose name contains `sweep`. Previously, `interval_sweep.py` writes per-interval dirs named `tasks{N}_sweep_interval{I}_seed{S}` directly into `optimizer_comparison/`, and the aggregator matched every `tasks(\d+)_*` dir — so sweep runs were ingested as cross-task data points and **overwrote the real bars** for that task count in Figs 1A-1E. The end-to-end flow now produces correct figures.

#### Verification (re-confirmed this session)

- `python3 -m pytest tests/python/` → **170 passed** (full suite, no regressions); orchestrator subset = 16 passed.
- `bash -n scripts/run_end_to_end.sh` → OK.
- `DRY_RUN=1 ./scripts/run_end_to_end.sh` (test mode) → 2 simulate cmds + sweep + aggregate, all from config; `MODE=prod` → 3 task counts + 6-interval sweep.
- `STEPS="simulate aggregate" ...` correctly skips the sweep (no STAGE 2); aggregate-only (`STEPS="aggregate"`) skips binary validation since no simulating stage is selected.

#### Usage

```bash
# One command, everything, fast smoke-test parameters:
DRY_RUN=1 ./scripts/run_end_to_end.sh                       # preview commands
./scripts/run_end_to_end.sh                                 # test mode, all stages
MODE=prod ./scripts/run_end_to_end.sh                       # paper-grade
STEPS="simulate aggregate" ./scripts/run_end_to_end.sh      # skip sweep
STEPS="aggregate" MODE=prod ./scripts/run_end_to_end.sh     # figures only, no sim
```

#### Design note

`--bin_dir` lives in the orchestrator CLI (and `scripts/run_end_to_end.sh`'s `BIN_DIR`) rather than in `experiment_config.json`, because the binary location is an environment/machine concern, not an experiment parameter. Everything else — task counts, durations, trigger intervals, scheduler lists, seeds, worker count, resume, important-task %, export level — is config-driven via `experiment_config.json`.

#### Post-fix: test-mode duration too short for hyper-period

A real (non-dry-run) `./scripts/run_end_to_end.sh` test-mode run failed at taskset generation:

```
ValueError: Total simulated time (30s = 30000ms) must be at least 2× the
hyper-period (66000ms). Hyper-period of periods [500, 200, 33, 20] = 33000ms.
Increase n_sec to >= 66s.
```

Root cause: `test_mode.simulation_duration_seconds` was `30`, but the taskset
generator (`Gen_Taskset/lib/orchestrator.py`) requires `n_sec ≥ 2 × hyper-period`.
Period pools (from `taskset_cfg_paper_base.json`, included by all `paper_*` configs):
big `[1000, 500, 200]` ms, small `[100, 50, 33, 20]` ms. The coprime pair
`1000` & `33` gives a worst-case hyper-period of **33 s** across all task counts
(4/6/8), so the generator needs `n_sec ≥ 66 s`. The `--dry_run` path couldn't
catch this because generation never runs in dry-run.

Fix: `simulation_experiments/configs/experiment_config.json` — bumped
`test_mode.simulation_duration_seconds` from `30` to `70` (smallest safe
multiple of 10 above 66; prod mode's `300` was already fine). Added an
inline `_comment` explaining the 66 s floor so it isn't lowered again.

Re-ran `./scripts/run_end_to_end.sh` (test mode): **completed in 53.1s**,
all 11 figures (1A-1F, ablation ×2, Fig 2, Fig 3/3b) generated as PNG+PDF,
no errors. Verified the union scheduler list (7 schedulers: INCR, BF, RM,
CFS, INCR_NO_TL, INCR_WCET, INCR_SCRATCH) flows through both main and
ablation figures from a single simulate pass.

#### Resolved: aggregator no longer ingests stale experiment dirs (fixed by `c6c09e23`)

This entry previously documented an open bug — kept here for context, then
updated with the resolution. The original symptom: `aggregate_data_from_directories()`
ingested **every** `tasks{N}_*` directory with last-write-wins per
`(num_tasks, scheduler)`. Result dirs were named only by
`(num_tasks, duration, interval, seed)`, so runs with different durations
coexisted and silently corrupted each other's bars. In the cited run the
6-task bars were polluted by a stale `tasks6_dur300` dir (9 schedulers,
incl. RM_FAST/RM_SLOW, from 2026-06-28) and a leftover `tasks4_dur30`
partial dir from the failed run — yielding 23 records instead of 14. The
sweep-dir exclusion (added earlier that session) handled the `*_sweep_*`
collision but not the `*_dur{D}_*` collision.

Resolution landed in commit `c6c09e23 reorganize output folders from
simulation exp` (P23 run-root co-location). The production aggregate path
(`aggregate_across_tasks.py:961`, called with `cfg=`) now:

1. Scans `<run_root>/sim/`, not the top of `output_parent`. Each run gets
   its own directory `runs/run_<mode>_dur<D>_interval<I>_seed<S>_tasks<Nx...>/`
   (`build_run_id`, `experiment_config_loader.py:122`), so a `tasks6_dur300`
   dir from a different run lives under a different run root and is never
   scanned (`aggregate_across_tasks.py:366-373`).
2. Filters by exact prefix within that run root — `allowed_prefixes =
   [tasks{n}_dur{dur}_interval{interv}_seed{seed} for n in task_counts]`
   — so both the `*_dur{D}_*` collision and the `*_sweep_*` collision are
   rejected (`aggregate_across_tasks.py:386-399`). Simulate and sweep write
   under the same `<run_root>/sim/` the aggregator scans
   (`compare_optimizers.py:255`, `interval_sweep.py:77,176`).
3. The legacy `cfg=None` path (direct callers / old tests) keeps the
   previous "ingest every `tasks{N}_*` except sweep" behaviour
   (`aggregate_across_tasks.py:401-406`).

Locked in by `tests/python/test_aggregate.py`, which writes a current
`tasks6_dur70_interval10_seed1000` and a stale
`tasks6_dur300_interval10_seed1000` side-by-side under the same `sim/` and
asserts only the current run's record (`mean_sp == 0.90`, not the stale
`0.10`) is ingested.

Residual gap: `num_tasksets` is deliberately excluded from the run id
(`experiment_config_loader.py:102-104`), so two runs differing only in
sampling depth share a run root and their `tasks{N}_dur{D}_..._seed{S}`
dirs collide (last-write-wins within that run root). Intentional — `mode`
is treated as the primary axis. If you ever vary `num_tasksets`
independently with mode/dur/interval/seed held fixed, set a
`plotting.run_name_prefix` to namespace the runs (or clear the run root
first).

---

### Session Summary: Simulation Experiment Pipeline Completion

Completed all remaining tasks (P5–P9) for the simulation experiment pipeline. The system now supports full end-to-end generation, simulation, aggregation, and figure production for publication.

#### Configuration & Plotting Infrastructure

- **`simulation_experiments/configs/experiment_config.json`**
  - Central JSON config with `test_mode` / `prod_mode` sections.
  - `test_mode`: `[4, 6]` tasks, 2 tasksets, 70s duration (≥ 66s hyper-period floor; see P10 note), intervals `[5, 10]`.
  - `prod_mode`: `[4, 6, 8]` tasks, 10 tasksets, 300s duration, intervals `[1, 5, 10, 20, 30, 60]`.
  - Global `plotting` section: 14/16/18 pt fonts, 300 dpi, colorblind palette, PNG+PDF output.
  - Global `analysis` section: 10% important-task threshold, resume support.

- **`simulation_experiments/experiment_config_loader.py`**
  - `load_experiment_config(mode)` returns flattened dict merging mode-specific params with global settings.

- **`simulation_experiments/plotting_config.py`**
  - `setup_publication_style()` configures matplotlib rcParams.
  - `get_scheduler_color_map()` provides consistent colors per scheduler.
  - `save_figure()` writes PNG + PDF automatically.

#### P5 — Important-Task Miss Rate (Figure 3)

- `compute_miss_rate_by_task()` and `compute_important_task_miss_rate()` in `utils.py`.
- `analyze_single_instance()` returns `(important_miss_rate, non_important_miss_rate)`.
- `compare_optimizers.py` aggregates both metrics; `write_summary_and_plots()` writes them to `comparison_summary.csv`.
- `aggregate_across_tasks.py` generates Fig 3 (important-task miss rate) and Fig 3b (non-important, debug).

#### P6 — Trigger-Interval Sweep (Figure 2)

- **`simulation_experiments/interval_sweep.py`** — orchestrates multiple `compare_optimizers.py` runs, one per trigger interval from config.
- Generates Fig 2: X=interval (s), Y=mean SP, INCR with error bars, BF/RM as horizontal dashed reference lines.
- Supports `--mode test` (fast) and `--mode prod` (paper-grade).

#### P7 — Shell Scripts

Created 7 scripts in `scripts/` (note: the three `run_sim_{4,6,8}_tasks.sh`
wrappers below were later collapsed into `run_simulation.sh`'s positional
task-count arg — see the "code refactor" session later this day):

| Script | Purpose |
|---|---|
| `run_simulation.sh` | Core runner; accepts env-var overrides (`NUM_TASKS`, `NUM_TASKSETS`, `N_SEC`, `SCHEDULERS`, etc.). Validates binary, traps Ctrl-C, prints timing header. |
| `run_sim_4_tasks.sh` | Wrapper: `NUM_TASKS=4` |
| `run_sim_6_tasks.sh` | Wrapper: `NUM_TASKS=6` |
| `run_sim_8_tasks.sh` | Wrapper: `NUM_TASKS=8` |
| `run_all_experiments.sh` | Runs 4/6/8 sequentially, then calls `aggregate_across_tasks`. |
| `run_interval_sweep.sh` | Runs interval sweep standalone. |
| `run_paper_figures.sh` | Regenerates all figures from existing data (no simulation). |

All scripts pass `bash -n` and have executable permissions.

#### P8 — Tests

- **`tests/python/test_run_sim_experiments.py`** — 21 tests covering miss-rate computation (summary + trace fallbacks), `run_single_simulation` args, `analyze_single_instance` with exec time / effective intervals / important tasks, and `write_summary_and_plots` with the updated CSV header.
- **`tests/python/test_aggregate.py`** — 16+ tests for task-count parsing, summary loading, aggregation logic, grouped bar charts, main/ablation figure generation, Fig 3, distribution boxplot, and a full end-to-end integration test with mock data.
- **`tests/python/test_plotting.py`** — 12 tests for publication-style setup, color maps, figure saving in multiple formats, and synthetic figure generation (grouped bar, box plot, line chart with error bars, single bar chart).

---

## 2026-06-20

### Task 1: Fix opt_sp_ initialization bug
- Changed `opt_sp_` initialization from `0` to `-1` in `OptimizeFromScratch_w_TL` and `OptimizeIncre_w_TL` to properly support negative safety performance values (committed — landed in `d8eb8495`).

### Task 2: Linear Coordinate Descent for Task Configuration Optimization
- Replaced the exponential recursive traversal logic in task execution time limit configuration with a linear coordinate descent algorithm.
- Implemented task sorting via `TaskSortingHeuristic` to process tasks in descending weight and ascending threshold priority order.
- Encapsulated initialization and coordinate descent logic in separate functions: `InitializeTimeLimitsFromETConfig` and `PerformCoordinateDescentForTaskConfigOpt`.
- Implemented a tie-breaker rule at both the search and record-update levels: if multiple configurations yield the same optimal SP metric, choose the one with the lower total execution time limit.
- Re-enabled original test cases (expecting 400ms limit under coordinate descent with tie-breaker).
- Designed and added the `OptimizeWithOptimizationSpace` test case showing three distinct limits for:
  - Default ET configuration (1000ms / 1500.93ms)
  - Incremental optimization (800ms)
  - Scratch optimization (600ms)
  And corresponding SP improvements (`SP_600 > SP_800 > SP_1000`).
- Verified all 15 test suites pass successfully.

---

## How to Test-Run the Pipeline (Test Mode)

### Prerequisites

1. **Compile the C++ release binary** (scripts validate this exists before running):
   ```bash
   # Release build — the scripts expect:
   #   release/tests/RunOrchestrator
   mkdir -p release && cd release && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) RunOrchestrator
   ```
   > Note: `build/` folder is reserved for debug builds. Production / paper runs always use `release/`.

2. **Python dependencies:**
   ```bash
   pip install matplotlib numpy pyyaml seaborn pytest
   ```

### Test-Run Levels

#### Level 1 — Unit Tests Only (No Simulation, ~5 seconds)
```bash
python3 -m pytest tests/python/test_run_sim_experiments.py -v
python3 -m pytest tests/python/test_aggregate.py -v
python3 -m pytest tests/python/test_plotting.py -v
```

#### Level 2 — Single Task Count Smoke Test (~1–3 minutes)
```bash
# Uses test defaults from experiment_config.json:
#   2 tasksets, 70s duration (≥66s hyper-period floor), 10s trigger interval
# Positional arg overrides NUM_TASKS (the old run_sim_{4,6,8}_tasks.sh wrappers
# were collapsed into run_simulation.sh):
NUM_TASKS=4 NUM_TASKSETS=2 N_SEC=70 ./scripts/run_simulation.sh 4
```
Or directly via Python:
```bash
python3 -m simulation_experiments.compare_optimizers \
    --num_tasks 4 --n_tasksets 2 --n_sec 70 \
    --schedulers INCR BF RM CFS
```

#### Level 3 — Full Test Mode End-to-End (one command)
```bash
# Runs simulate -> sweep -> aggregate, all from experiment_config.json:
./scripts/run_end_to_end.sh
```
Equivalently, the three stages manually:
```bash
NUM_TASKS=4 ./scripts/run_simulation.sh 4 && NUM_TASKS=6 ./scripts/run_simulation.sh 6
python3 -m simulation_experiments.aggregate_across_tasks --mode test
python3 -m simulation_experiments.interval_sweep --mode test
```

### Viewing Results
```bash
# Generated figures land under a run-id-scoped subdir (per-run namespacing,
# not the old flat figures/ dir):
ls simulation_experiments/optimizer_comparison/runs/run_test_dur70_interval10_seed1000_tasks4x6/figures/

# Summary CSV (includes Important_Miss_Rate column)
cat simulation_experiments/optimizer_comparison/tasks4_*/comparison_summary.csv
```

### Test vs. Production Mode Differences

| Parameter | Test Mode | Prod Mode |
|---|---|---|
| Task counts | `[4, 6]` | `[4, 6, 8]` |
| Tasksets | 2 | 10 |
| Duration | 30s | 300s |
| Sweep intervals | `[5, 10]` | `[1, 5, 10, 20, 30, 60]` |
| Export level | 1 | 1 |

---

## 2026-07-02

### Per-core CPU utilization fix + task-count ceiling removal (P13)

Two issues surfaced while reviewing `agents/gemini.md` (a staged-code review
note). Both are real; the user wants them as **two separate commits**, and the
4/6/8 task-count ceiling removed entirely (support 10/12/14/16/18 …).

#### Issue 1 -- `MEAN_CPU_UTIL` was not held constant per-core (Commit A)

`MEAN_CPU_UTIL` is **per-core** utilization by the generator's arithmetic:
`taskset_generator.py:241-242` does `cpu_util = cfgs['MEAN_CPU_UTIL'] *
n_cores`, then UUniFast distributes that total across the N tasks. So it
represents the **aggregated load state** and must stay constant regardless of
N or core count — exactly as the user stated: "mean cpu utilization is
per-core utilization, it remains the same no matter how many tasks or how many
cores."

The existing per-N configs violated this:

| N | old `MEAN_CPU_UTIL` | total (`×2`) | per-task (`/N`) |
|---|---|---|---|
| 4 | 0.9 | 1.8 | 0.450 |
| 6 | 1.2 | 2.4 | 0.300 |
| 8 | 1.6 | 3.2 | 0.225 |

They rose 0.9 → 1.2 → 1.6, i.e. they held **per-task ≈ 0.3–0.45** roughly
constant instead of per-core. That is the wrong semantics; the user was
correctly skeptical. Fix (Commit A, config-only): flatten all three to **0.9**
per-core (total 1.8, system 90 % loaded / under-subscribed; matches the
existing N=4 setting). `paper_4.json` already 0.9 (no change); `paper_6.json`
1.2 → 0.9; `paper_8.json` 1.6 → 0.9.

**Acceptance:** old 4/6/8 run data (SP values recorded below, the
`optimizer_comparison/` tasksets) becomes stale and would need regeneration —
user accepts this. The 0.9 value is itself tunable later as a separate edit;
duplicate periods at large N are OK (period pool `[10,20,30,50]` Hz unchanged).

#### Issue 2 -- 4/6/8 task-count ceiling crashes prod N=10 (Commit B)

`experiment_config.json` prod declares
`num_tasks_for_cross_task_comparison: [4, 6, 8, 10]`, but the ceiling is
enforced in two places (both must be removed):

1. **CLI** `choices=[4, 6, 8]` on `--num_tasks` in `compare_optimizers.py:257`
   and `run_sim_experiments.py:260` → N=10 fails argparse validation, aborting
   the pipeline (`subprocess.run(..., check=True)` → `CalledProcessError`).
2. **Named-file lookup** `f".../taskset_cfg_paper_{N}.json"` at
   `compare_optimizers.py:342-345` and `run_sim_experiments.py:311-314` — only
   `paper_{4,6,8}.json` exist, so even without the CLI gate, N=10 has no config
   file. (This is the root cause; the user objected to the hard-coded value at
   `run_sim_experiments.py:313` and asked to "support it" rather than
   pre-create files.)

**Fix (Commit B):**
- **B1** Add `resolve_taskset_config_path(num_tasks, config_dir=None)` to
  `generation_config_parser.py`: returns the existing `paper_{N}.json` if
  present, else synthesizes a thin temp override file (same shape as the
  on-disk ones: `INCLUDE` base + `N_BIG=2`, `N_SMALL=N-2`, `MEAN_CPU_UTIL=0.9`,
  `N_CORES=2`, `RANDOM_SEED=42`). Downstream `load_generation_config(path)`
  then resolves INCLUDE exactly as for on-disk files — zero new code paths in
  the generator. The 0.9 constant lives in one place here for synthesized
  configs; on-disk 4/6/8 carry their own (Commit A); the two are kept in sync.
- **B2** Replace the hard-coded lookup at both call sites with the resolver.
  **Latent bug fix:** `run_sim_experiments.py:373-374` used raw `json.load`
  (does **not** resolve INCLUDE), unlike `compare_optimizers.py:411` which uses
  `load_generation_config`. It worked today only because `paper_6.json`'s
  INCLUDE base params happen to be optional/defaulted. Switch it to
  `load_generation_config` — required for correctness once synthesized configs
  (which rely on INCLUDE) are in play.
- **B3** Drop `choices=[4,6,8]` at both CLIs; add a floor guard
  (`num_tasks >= 2`, matching fixed `N_BIG_PERIOD_TASKS=2`). `interval_sweep.py`
  already has `default=None` with no `choices` and forwards `--num_tasks`
  downstream — unchanged.
- **B4** Extend prod task list `[4,6,8,10]` → `[4,6,8,10,12,14,16,18]`. Test
  mode stays `[4, 6]`.
- **B5** Tests for the resolver (existing 4/6/8 unchanged; N=10/14/18
  synthesizes a valid INCLUDE-resolved config; N<2 rejected) + CLI acceptance
  of `--num_tasks 10`.

**Note on period pool:** `SMALL_PERIOD_HZ=[10,20,30,50]` (4 distinct) with
`N_SMALL_PERIOD_TASKS=N-2` means any N>6 forces duplicate small periods
(N=8 already has 2 dups; N=18 → 12 dups). `pick_period` tries to avoid dups
then falls back to allowing them. Duplicate periods are acceptable per the
user — pool unchanged.

See [[taskset-config-architecture]] (memory) for the full N-flow diagram.

### Unified period pool: drop the big/small split (P19)

Follow-on to P13/P16/P17. The paired `BIG_PERIOD_HZ` / `SMALL_PERIOD_HZ` (+ the
`N_BIG_PERIOD_TASKS` / `N_SMALL_PERIOD_TASKS` counts) schema was an Hz-era
artifact: periods are ms everywhere downstream, the split only added a
`pick_period` branch, an ad-hoc <10 Hz / >=10 Hz split for the legacy single
`HZ` list, and a fixed `N_BIG=2` hardcode that fought the P13/P17 count
relaxation. Collapsed to one pool + one count.

**Canonical schema (the only accepted period/count inputs):**
- `PERIODS_MS` (list[int]) — period pool every task draws from, in ms.
- `N_TASKS` (int) — total task count; each task draws one period from the pool.
  Pool exhaustion at large N allows duplicate periods (`pick_period` avoids dups
  then falls back) — acceptable per the P13 note.

**Landed in two phases:**

- **Phase 1 — `a0cc8021` ("hz to period ms, merge big and small periods into
  single period list").** Migrated source (`generation_config_parser.py`,
  `taskset_generator.py`), every shipped config/template (`paper_base`,
  `paper_{4,6,8}`, `test_standard_{4,6,8}`), and `debug_uunifast.py` to
  `PERIODS_MS` + `N_TASKS`. `standardize_config` kept a **backward-compat
  alias** so old-shape configs kept working: built `PERIODS_MS` = big-pool
  periods + small-pool periods and `N_TASKS = N_BIG + N_SMALL`, then deleted
  the old keys. `pick_period(cfgs, picked_periods)` dropped its `prd_sel` arg
  (one pool, no branch). `resolve_taskset_config_path` synthesized configs set
  `N_TASKS`; N=1 still drops to `N_CORES=1` for schedulability (lone task's
  util stays < 1.0; with N_CORES=2 it would carry 1.8 → unschedulable).

- **Phase 2 — this change (alias removal).** The backward-compat alias was a
  transitional bridge; now removed. Legacy period keys
  (`HZ`, `SMALL_PERIOD_HZ`, `BIG_PERIOD_HZ`, `SMALL_PERIODS_MS`,
  `BIG_PERIODS_MS`) and count keys (`N_BIG_PERIOD_TASKS`,
  `N_SMALL_PERIOD_TASKS`) raise `ValueError` pointing at the canonical
  replacement instead of being silently aliased. Nothing in-tree breaks — every
  shipped config already sets the canonical keys. A bare config (no period/
  count info) defaults to the base template's pool
  (`[1000, 500, 200, 100, 50, 33, 20]`) and `N_TASKS=10`.

**Tests:** `TestP17ZeroCountTasksets` rewritten as `TestP19UnifiedPoolTasksets`
(5 cases: unified-pool sourcing, pool-exhaustion duplicates, N=1 single-rate,
legacy-keys-rejected, N_TASKS<1-rejected). Legacy-aliasing tests in
`test_generation_config_parser.py`, `Gen_Taskset/tests/test_generation_config.py`,
and `Gen_Taskset/tests/test_specifications.py` flipped to assert `ValueError`.
Remaining test files migrated to canonical keys in-place. Full suite
**238 passing** (`tests/python/` 224 + `Gen_Taskset/tests/` 14).

### Random per-core CPU utilization range (P14)

Follow-on to P13. P13 held per-core `MEAN_CPU_UTIL` constant at 0.9 so every
task set exercises a single load point (90 % loaded / under-subscribed). P14
makes that load **sampleable per task set**: when a new optional
`CPU_UTIL_RANDOM_RANGE: [low, high]` key is present, the generator samples the
per-core utilization uniformly from `[low, high]` for each task set, then runs
the existing UUniFast distribution against the sampled total
(`cpu_util = sampled_per_core × N_CORES`). One experiment run therefore sweeps
under-subscribed → over-subscribed systems, giving a distribution of SP
behavior over load rather than a point estimate. P13's per-core semantics are
preserved — only the fixed *value* becomes a *sampled range*.

**Supersedes the P13 out-of-scope note.** P13's acceptance line said "the 0.9
value is itself tunable later as a separate edit." P14 is that edit: the
tunable knob is now the **range** (`CPU_UTIL_RANDOM_RANGE`), not a scalar. P13's
fixed 0.9 remains the default load when the range is absent — P14 is strictly
opt-in (see below).

**Design decisions (resolving the P14.1 open question):**
- *Range supersedes or falls back?* — **fallback** (option (b) in the task
  spec) for backward compat. When `CPU_UTIL_RANDOM_RANGE` is present it is
  used; when absent, the generator takes the existing fixed-`MEAN_CPU_UTIL`
  path unchanged. Old P13 configs without the range keep working verbatim.
- *Where does the key live?* — **opt-in, NOT in the base template.** P14.5
  requires "no range config → 0.9" as the default. Adding the key to
  `taskset_cfg_paper_base.json` would turn sampling ON for every paper config
  via INCLUDE, contradicting P14.5. So the key is recognized by the parser/
  generator only when a config sets it explicitly; no shipped config sets it.
  Enabling sampling for the paper experiments is a one-line config edit (add
  `"CPU_UTIL_RANDOM_RANGE": [0.5, 1.5]` to the desired `paper_*.json`).

**Implementation:**
- **P14.1** `generation_config_parser.standardize_config`: validates
  `CPU_UTIL_RANDOM_RANGE` shape when present — must be a `[low, high]` pair of
  numbers with `0 <= low <= high`; coerced to floats. Rejected with
  `ValueError` otherwise (wrong arity, non-numeric, `low > high`, `low < 0`,
  bool). The key is not synthesized when absent (opt-in).
- **P14.2** `taskset_generator.generate_taskset_parameters` (~line 248):
  replaced `cpu_util = cfgs['MEAN_CPU_UTIL'] * n_cores` with a branch — if the
  range is present, `per_core_cpu_util = random.uniform(low, high)`; else
  `per_core_cpu_util = cfgs['MEAN_CPU_UTIL']`. Then `cpu_util = per_core_cpu_util
  × n_cores` in both paths.
- **P14.3** Reproducibility + inspection: the sampled value is the **first
  draw** off the seeded global RNG (`random.seed(RANDOM_SEED)` runs immediately
  before, at ~line 242), so a given `RANDOM_SEED` reproduces the same sampled
  per-core util. The realized `per_core_cpu_util` is recorded in the returned
  dict and lands in `taskset_param.yaml` (top-level, alongside `cpu_util`) so
  each task set's load is inspectable. (Existing readers use
  `yaml.safe_load` + named-key access, so the new key is additive — no reader
  changes.) Enabling the range shifts all downstream random draws (it consumes
  one RNG value), so task sets generated with the range are not byte-identical
  to the fixed-0.9 ones under the same seed — inherent to the design and
  documented in the task spec.
- **P14.5** P13's 0.9 stays the default; P14 is opt-in. No P13 config removed.
  This entry records that P14 supersedes P13's "0.9 is itself tunable later"
  note (above).

**Tests** (`tests/python/`):
- `test_taskset_generator.py::TestP14RandomCpuUtilRange` (5): sampled util ∈
  `[low, high]`; `cpu_util = per_core × N_CORES` (realized per-task sum within
  a tolerance — the pre-existing `max(1.0, u_i×period)` floor on small-period
  tasks can push the realized sum slightly above the target); deterministic
  under a fixed seed; varies across 30 seeds; range-absent falls back to fixed
  0.9. The class snapshots/restores global `random` + `np.random` state in
  setUp/tearDown so its many seeded generations don't leak RNG state into later
  modules (surfaced a pre-existing latent flake in
  `test_trajectory.py::test_generate_path_only`, which reads global RNG
  unseeded via `generate_path_only`).
- `test_generation_config_parser.py` (+3): valid range accepted + normalized;
  malformed ranges rejected; range absent by default (opt-in).

Full suite **253 passing** (was 245 at HEAD; +8 P14 tests).

### End-to-end: always run all stages + actionable aggregate (P22)

The e2e orchestrator offered a `--steps` flag (and `STEPS` env-var) to run a
*subset* of stages. That was a footgun: the three stages are dependent (aggregate
reads what simulate wrote; sweep reuses simulate's tasksets), so selecting a subset
against stale or missing data produced confusing "No experiment records found"
failures rather than doing the right thing. Per user request ("the only case is
run end-to-end"), the stage-selection knob is removed entirely. The INCR-ET
measurement config also drops its `prod_mode` block (test/prod had drifted to
different task counts), and aggregate's empty-data path becomes actionable.

**Implemented in commit `46a62b49` (branch `clean_simulation`):**
- `run_end_to_end_experiments.py`: removed `--steps` arg, `ALL_STAGES` constant,
  and the `stage_funcs` loop. `main()` runs simulate → sweep → aggregate
  unconditionally in fixed order; a failed stage still aborts. Binary pre-flight
  simplified (always required unless `--dry_run`).
- `scripts/run_end_to_end.sh`: dropped `STEPS` env-var, its usage example, and the
  `--steps` forwarding block. Header prints the fixed stage order.
- `aggregate_across_tasks.py`: the empty-data path (was a bare "Run simulations
  first." + exit 1) now prints the exact expected dir prefixes it looked for
  (`tasks{N}_dur{D}_interval{I}_seed{S}`), notes none matched, and points at
  `./scripts/run_end_to_end.sh`. Stays **read-only** -- does NOT auto-run simulate,
  so a direct re-plot can't surprise the user with a long C++ sim.
- `configs/incr_et_8tasks_config.json` (new, single-purpose INCR-ET measurement
  config): `test_mode` block only (no `prod_mode`) -- `[10]` tasks, `2` tasksets,
  `70s` duration, `10s` interval, seed `1000`, `main_scheduler_list: ["INCR"]`,
  `enable_execution_time_profiling: true`. The loader only errors if the
  *requested* mode's block is missing, so test-only is fine. Scheduler avg ET =
  `Mean_Scheduler_Execution_Time_s` ("Avg Sched Time") in `comparison_summary.csv`
  under `tasks{N}_dur{D}_interval{I}_seed{S}/`.
- `tests/python/test_run_end_to_end.py`: rewritten `TestMainPipeline` --
  `test_all_stages_always_run`, `test_steps_flag_rejected` (argparse exits 2),
  `test_failed_stage_aborts`, `test_dry_run_prints_and_does_not_execute`; dropped
  the `--steps`-subset tests. 17 e2e + 35 aggregate tests pass.

**Numbering note:** the commit title still reads `P21: ...` -- it was tagged P21
before a collision was noticed with the (unstarted, long-reserved) warm-start P21
at `tasks.md` L530. Resolved by renumber: the warm-start work keeps **P21**, this
e2e refactor is **P22**, and the next task (unify output folders) is **P23**. The
commit title was not amended. Supersedes the `--steps` documentation in P10/P20.

### Unify sim + figures output under one run folder (P23)

Sims and figures currently live at *different* levels of
`optimizer_comparison/`: per-task-count sim dirs (`tasks{N}_dur{D}_interval{I}_seed{S}`
from compare_optimizers, `tasks{N}_sweep_interval{I}_seed{S}` from interval_sweep)
are top-level siblings of `runs/`, while figures are nested under
`runs/<run_id>/figures/`. The goal is to co-locate each run's full sim output
(incl. the task-set config -- `generator_config.json`, `taskset_param.yaml`,
`taskset_characteristics*.yaml`, `path_Et_task_*.txt`, the per-scheduler
`INCR/...` output) with its derived figures under the run dir, and drop a copy of
the driving config JSON into the run dir so a run is self-describing.

**Target layout:**

```
optimizer_comparison/
└── runs/run_test_dur70_interval10_seed1000_tasks10/   <- build_run_id
    ├── config.json                                    <- copy of the active config file
    ├── figures/                                       <- derived figures (unchanged)
    └── sim/                                           <- ALL raw sim output
        ├── tasks10_dur70_interval10_seed1000/         <- main-step sim + taskset_*/
        └── tasks10_sweep_interval5_seed1000/          <- sweep sim + taskset_*/
```

**Key constraint:** the three stages must agree on where sims live.
compare_optimizers writes the sim dir; interval_sweep *reconstructs that same path*
to reuse the main step's tasksets (`_main_step_dir` →
`output_parent/tasks{N}_dur...`); aggregate *scans* that location for
`comparison_summary.csv`. So moving sims under `runs/<run_id>/sim/` means all three
writers/readers, plus the orchestrator wiring `output_parent`, must move together.
The run dir is `build_run_id(cfg)` -- one per run -- so "where sims go" is a
function of cfg, computed once and threaded through.

**Approved design (see `tasks.md` P23 for the full checkbox breakdown):**
- New `build_run_root(output_parent, cfg)` helper in `experiment_config_loader.py`
  (reuses `build_run_id`); sims → `<run_root>/sim/`, figures → `<run_root>/figures/`.
- `compare_optimizers.resolve_run_output_dir` gains `run_root=None` + a `--run_root`
  CLI arg; standalone use falls back to today's path (only the orchestrator opts in).
- `interval_sweep` `_main_step_dir`/`_main_dir_is_fresh`/`run_single_interval` take
  `run_root`; sweep dir → `<run_root>/sim/...`; fig2 → `<run_root>/figures`. No new
  CLI arg (derives `run_root` from cfg internally).
- `aggregate` scans `<run_root>/sim/` when cfg-scoped (legacy unscoped branch keeps
  scanning `output_parent`); `generate_distribution_boxplot` (Fig 1F, reads the
  module-global `OPTIMIZER_COMPARISON_DIR`) gets `run_root` threaded from `main()`;
  empty-data message points at `<run_root>/sim/`. Module globals stay as defaults
  for the unscoped/standalone fallback.
- Copy the driving config JSON into `<run_root>/config.json` in `aggregate.main()`
  (last stage, run root guaranteed to exist, read-only otherwise so a copy is
  benign) via `shutil.copy2`.
- Orchestrator computes `run_root` once, passes `--run_root` to the simulate
  command only; sweep + aggregate derive it internally (no new args).
- Tests: `test_compare_optimizers` (run_root path variant), `test_interval_sweep`
  (the reuse test is critical -- the fresh main dir must be written where
  `_main_step_dir` now looks, under `sim/`), `test_aggregate` (build dirs under
  `sim/`, thread `run_root` for Fig 1F), `test_run_end_to_end` (assert `--run_root`
  in the simulate cmd).

**Legacy / old data:** old top-level sim dirs (`tasks4/6/8/10_dur70_...`,
`tasks*_sweep_...`, `tasks1_dur300_...`, `tasks*_dur600_...`) are left in place --
not migrated. Once aggregate's scan root moves to `<run_root>/sim/`, those become
invisible to new runs (desired clean per-run isolation); user deletes by hand. No
migration script.

**Status: implemented (working tree, uncommitted).** Full plan in
`/home/zephyr/.claude/plans/scalable-swimming-wombat.md`.

**Implemented (this change, branch `clean_simulation`):**
- `experiment_config_loader.py`: added `build_run_root(output_parent, cfg)` →
  `os.path.join(output_parent, "runs", build_run_id(cfg))`. Reuses `build_run_id`
  unchanged (no id-scheme change); two runs differing in mode/duration/interval/
  seed/task-count list get separate roots and never clobber.
- `compare_optimizers.py`: `resolve_run_output_dir(..., run_root=None)`. When set,
  sims land at `<run_root>/sim/<subfolder>` (co-located with figures); when None
  (standalone CLI), the legacy `<base_output_dir>/<subfolder>` layout is kept so the
  module stays usable on its own. New `--run_root` CLI arg (default None) -- only
  the e2e orchestrator opts in.
- `interval_sweep.py`: `_main_step_dir`, `_main_dir_is_fresh`, `run_single_interval`
  take `run_root`. With it set: the sweep writes its own dir to
  `<run_root>/sim/tasks{N}_sweep_interval{I}_seed{S}`, *and* reuse looks for the
  main step's dir there too (so P20 reuse still finds it after compare_optimizers
  relocated under `sim/`). The `--run_root` flag is forwarded to the child
  `compare_optimizers` subprocess. `main()` derives `run_root` from cfg internally
  (no new CLI arg); Fig 2 → `<run_root>/figures/`.
- `aggregate_across_tasks.py`: `aggregate_data_from_directories` scans
  `<run_root>/sim/` when cfg-scoped (the legacy unscoped branch, `cfg is None`, still
  scans `output_parent` directly). `generate_distribution_boxplot` (Fig 1F, which
  reads the module-global `OPTIMIZER_COMPARISON_DIR`) gained `run_root=None` threaded
  from `main()`; it scans `<run_root>/sim/` when set, falling back to the global dir
  only on the legacy/standalone path. `main()` computes `run_root`, sets
  `figures_dir = <run_root>/figures`, and the empty-data message now points at
  `<run_root>/sim/`. New `_copy_config_into_run_root` copies the driving config JSON
  (`cfg["_config_source_path"]`) into `<run_root>/config.json` via `shutil.copy2`,
  skipping silently when the source is unknown or already resolves to the destination
  (re-runs never clobber). Module globals stay as defaults for the standalone path.
- `run_end_to_end_experiments.py`: computes `run_root` once in `main()`, passes
  `--run_root` to the simulate command only (sweep + aggregate derive it internally
  from cfg -- no new args). Final figures-dir print uses `<run_root>/figures`.
- `configs/experiment_config.json` (prod_mode): `num_tasks_for_single_task_figures`
  8 → 10, so the Fig 1F box-plot target matches the smallest prod cross-task count.
- Tests: `test_compare_optimizers.py` (+3 -- run_root nests under `sim/` for both
  auto-named and custom `run_name`; `run_root=None` keeps the legacy layout);
  `test_interval_sweep.py` (+2 -- the critical reuse test: a fresh main dir under
  `<run_root>/sim/` is found and reused without a subprocess; and the sweep writes
  its own dir under `<run_root>/sim/` while forwarding `--run_root` to the child);
  `test_aggregate.py` (scoped-ingestion test rebuilt to put dirs under `<run_root>/sim/`
  while the legacy unscoped path still scans the top level; +1 box-plot test that the
  `run_root`-scoped scan finds data the empty global dir would miss; +3 for
  `_copy_config_into_run_root` -- copies, skips when source == destination, skips when
  no source path); `test_run_end_to_end.py` (simulate command assertions now build the
  expected `run_root` and assert `--run_root` is forwarded).

Full suite **262 passing** (`tests/python/` 248 + `Gen_Taskset/tests/` 14). `--dry_run`
confirms `--run_root` is forwarded to `compare_optimizers`, sweep + aggregate derive it
internally, and the final figures path is `<run_root>/figures`. (A real smoke run with
the INCR config to confirm `runs/<run_id>/{config.json,figures/,sim/tasks...}` on disk
is left to the user -- it needs the compiled `RunOrchestrator` binary, not available
in this doc-sync pass.)

### Periodic reoptimization for the incremental TL optimizer (P24)

**Problem.** `OptimizeIncre_w_TL` (called once per `SimulateInterval` by
`DeterminePrioritiesAndBudgets`) reuses a persistent `timelimit2optimizer_` cache
that is **never cleared**. After interval 0, every interval only ever calls
`OptimizeIncre` on already-cached TL configs, so the priority assignment can drift
toward a local optimum as task ETs evolve over a long sim (prod = 60 intervals), with
no mechanism to re-ground it. P24 adds a general, per-interval **periodic
reoptimization**: every `ReoptimizationPeriod`-th interval, the incremental TL search
re-runs with a **wider TL window** (cache kept — a "warm re-explore"), bounding drift
for every `INCR`-family mode. It is the general drift-bound P21 (warm-start) reuses.

**Design (per `tasks.md` P24, agreed 2026-07-02).** Two new general tunables in
`sources/Utils/Parameters.{h,cpp}` + `sources/parameters.yaml`, declared alongside
`TimeLimitSearchRadiusIncr`: `ReoptimizationPeriod` (default **10**, `0` = disabled =
today's pure cache+scratch baseline exactly, the off-switch the A/B uses) and
`ReoptimizationTimeLimitsSearchRadius` (default **4**, the TL-window radius during
reoptimization intervals; normal intervals keep `TimeLimitSearchRadiusIncr` = 2).
Cadence is **per-interval** (≡ per-call, 1:1 — `DeterminePrioritiesAndBudgets` calls
the entry point exactly once per `SimulateInterval`, verified in
`SimulationOrchestrator.cpp:253-257`+`266-272`), not per-eval (per-eval varies with
N/K/radius and is unreproducible). **Interval 0 is the first reoptimization interval**
(option A): reopts fire at 0, N, 2N, … Reopt is the **incremental path with a wider
radius**, NOT `OptimizeFromScratch_w_TL` — a full-list search on interval 0 (cache
empty) would `OptimizeFromScratch(K)` every unseen TL vector, the unpredictable cost
the user explicitly rejected. The cache is **not cleared** on a reopt interval:
already-seen TL configs reuse `OptimizeIncre`, only newly-visited TL combos get a true
`OptimizeFromScratch(K)`.

**Single orchestrator entry point.** Added
`OptimizePA_Incre_with_TimeLimits::Optimize_w_TL_ScratchOrIncre(dag_tasks_update, K)`
— the ONE method the orchestrator calls per interval from first to last. It owns
(i) the per-interval counter `reoptimization_interval_count_` (member, init 0) and
(ii) the radius decision
`radius = (ReoptimizationPeriod > 0 && count % ReoptimizationPeriod == 0) ?
ReoptimizationTimeLimitsSearchRadius : TimeLimitSearchRadiusIncr`,
then `++count` (after the decision) and delegates to the existing
`OptimizeIncre_w_TL(dag, K, radius)` body. `OptimizeIncre_w_TL` was given a `radius`
param (replacing its hard-read of `GlobalVariables::TimeLimitSearchRadiusIncr`);
`RecordCloseTimeLimitOptions(dag, radius)` was parameterized the same way. The counter
resets to 0 in `OptimizeFromScratch_w_TL` (signals a fresh run) and is **not** zeroed
in the per-call path (zeroing every call would freeze it at interval 0). Trace:
count 0 → interval 0 reopt (0%10==0) → 1; intervals 1-9 normal; interval 10 reopt →
11; … = option A. `ReoptimizationPeriod == 0` → radius always
`TimeLimitSearchRadiusIncr` → today's behavior exactly.

**Mode scope (architecturally dictated).** The reoptimization branch lives inside the
single entry point every persistent-INCR mode routes through, so it applies to **INCR /
INCR_NO_TL / INCR_WCET** automatically and stays decoupled from mode strings
(`INCR_NO_TL` short-circuits via `disable_time_limit_opt` — radius moot there).
`INCR_SCRATCH` constructs a fresh optimizer + `OptimizeFromScratch_w_TL` per interval
(no persistent cache/counter) → structurally excluded. Verified: in the production INCR
path `OptimizeFromScratch_w_TL` is never called (only INCR_SCRATCH calls it, via a
fresh `scratch_opt` at `SimulationOrchestrator.cpp:276-279`), so the counter starts at
0 from the member initializer and increments once per interval — no mid-sim resets.

**Orchestrator wiring.** `DeterminePrioritiesAndBudgets` (`SimulationOrchestrator.cpp`)
in the `INCR` branch (and the same call inside the `INCR_NO_TL` / `INCR_WCET`
save/flip/restore ablation wrappers) now calls
`incr_optimizer_.Optimize_w_TL_ScratchOrIncre(...)` instead of the 2-arg
`OptimizeIncre_w_TL`. The orchestrator carries no interval logic. `INCR_SCRATCH` /
`BF` / `RM` branches untouched.

**TDD tests (`tests/testIncreOpt_w_TL.cpp`, +4 = 17 total).**
`RecordCloseTimeLimitOptions_DynamicRadius.Vanilla` was rewritten to pass `radius`
explicitly (was: mutate the global + restore). Four new `Reoptimization.*` tests:
(a) `DefaultsLoadedFromYaml` — period 10, wide radius 4; (b) `PeriodZeroAlwaysUsesNarrowRadius`
— period 0 → every call uses `TimeLimitSearchRadiusIncr`, counter still advances;
(c) `WideRadiusOnReoptIntervalsNarrowElsewhere` — period 3 over 7 calls → wide on
0/3/6, narrow elsewhere, counter advances 1..7 (proves option-A cadence + the periodic
**repeat** path); (d) `CounterResetsAcrossFreshOptimizeFromScratch` — counter → 0 on a
fresh `OptimizeFromScratch_w_TL`, and interval 0 after a reset is again a reopt. Test
scaffolding: `ReoptimizationIntervalCount()` + `LastRadiusUsedForTest()` getters (the
latter set at the top of `OptimizeIncre_w_TL`) so tests assert the decision without a
full sim. All other call sites in `tests/{testBF_w_TL,AnalyzePriorityAssignmentIncrementalExample}.cpp`
updated to pass `TimeLimitSearchRadiusIncr` (mechanical, no behavior change).
`build/tests/testIncreOpt_w_TL` = **17/17 green**. Release `RunOrchestrator` builds clean.

**Smoke + A/B (P24.5).** Short smoke: `release/tests/RunOrchestrator` on the dur70
10-task taskset (7 intervals, INCR, period 10) ran clean — no crash, 7 SP values
emitted (interval 0 SP 1.81 > later 1.55-1.59, consistent with the wider interval-0
search). A/B on the same 7-interval taskset to exercise the periodic **repeat** path
(period 3 → reopts at 0/3/6, not just interval 0). **CORRECTED 2026-07-02** — the
original A/B logged bogus exec times (13.95s / 18.00s) caused by passing `70000`
(total sim duration) as the RunOrchestrator `duration_ms` arg, which is actually the
**per-interval** horizon (`SimulateInterval`'s `for time_now in [start, end]` discrete
+1ms loop, `SimulationOrchestrator.cpp:469`). The e2e pipeline passes
`scheduler_trigger_interval*1000 = 10000`; passing 70000 made each interval's loop
7× too long. Re-run with the correct `duration_ms=10000` (release binary, taskset_0):

| config | reopt intervals | exec (s) | avg SP |
|---|---|---|---|
| P24 OFF (period 0) | none | 0.568 | 1.56568 |
| P24 ON period 10 (default, reopt @0) | 0 | 0.613 | 1.60964 |
| P24 ON period 3 (reopt @0,3,6) | 0, 3, 6 | 0.712 | 1.73874 |
| P24 ON period 1 (reopt every interval) | all | 0.750 | 2.12851 |

**No SP regression — SP improves** (the wider re-explore finds better priority
assignments, the intended drift-bounding behavior); real exec cost is **~+45 ms per
reopt interval** (period 3 vs off: 0.712−0.568 = 0.144s over 3 reopts ≈ 48ms each),
NOT the +1.35s/reopt-interval logged originally. The on-disk e2e `scheduler_execution_time.txt`
confirms ~0.50s total for this taskset (matches `comparison_summary.csv`
`Mean_Scheduler_Execution_Time_s = 0.061427` = 0.50s/7 intervals). The 60-interval
prod run was therefore ~60 × 0.08s ≈ **5s**, not 19+ min — the P18 ">0.1s/interval
slowness" claim was also inflated by the same `duration_ms` misuse and needs
re-measurement before treating P18 as a blocker. The periodic-repeat cadence at prod
scale is proven deterministically by the `WideRadiusOnReoptIntervalsNarrowElsewhere`
unit test, so 60 intervals ÷ period 10 = 6 reopts at 0/10/20/30/40/50 by extrapolation.

> See memory [[runorchestrator-duration-arg-semantics]] for the full RunOrchestrator
> `duration_ms` semantics (per-interval horizon, NOT total sim duration) and how to
> measure ET correctly.

**Files changed (working tree, uncommitted):** `sources/Utils/Parameters.{h,cpp}`,
`sources/parameters.yaml`, `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}`,
`sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp`,
`tests/{testIncreOpt_w_TL,testBF_w_TL,AnalyzePriorityAssignmentIncrementalExample}.cpp`.
`git add` only — user commits (per `agent_coding_rules.md`). P24 is the prerequisite
for P21 (warm-start); P21 may now start.


### P24-Eval: re-optimization interval sweep — per-activation optimizer runtime (2026-07-03)

**Why this measurement.** The original P24.5 A/B logged bogus exec times
(13.95s / 18.00s) caused by passing the total-sim `70000` as the
RunOrchestrator `duration_ms` arg, which is actually the **per-interval**
horizon (see [[runorchestrator-duration-arg-semantics]]). P24-Eval re-measures
correctly and sweeps `ReoptimizationPeriod` to characterize the cost/quality
trade-off. Crucially, the metric is **average per-activation optimizer runtime**
(wall-clock inside `OptimizeIncre_w_TL` per `SimulateInterval`), NOT
`Mean_Scheduler_Execution_Time_s` — the latter is dominated by the per-ms
discrete-event sim loop and conflates optimizer cost with sim cost.

**Instrumentation (P24-Eval.1, working tree).** `std::chrono` timing wraps the
`OptimizeIncre_w_TL` call inside `Optimize_w_TL_ScratchOrIncre`
(`sources/Optimization/OptimizeSP_TL_Incre.cpp`), recording one
`PerActivationRuntime{interval_idx, radius, runtime_s}` per interval into
`per_activation_runtimes_`. `SimulationOrchestrator::GetPerActivationRuntimes()`
forwards it; `tests/RunOrchestrator.cpp` writes
`optimizer_runtime_per_interval.csv` to `output/mode/` and reads an optional
`REOPTIMIZATION_PERIOD` env-var override (so the sweep varies the period per-run
without editing the tracked YAML). Verified on a smoke run: CSV has 61 lines
(header + 60 intervals); radius 4 on reopt intervals, 2 elsewhere (period 3 →
20 wide / 40 narrow). Cost is correctly isolated: per-activation 0.02–0.08s vs
9.1s total sim.

**Sweep (P24-Eval.3, `simulation_experiments/run_p24eval_sweep.py`).** INCR over
the grid `ReoptimizationPeriod ∈ {0,1,3,5,10,20}` × `N ∈ {6,8,10,12}` × 10
tasksets, 60 intervals each (`n_sec=600`, `interval=10s` → `duration_ms=10000`,
the correct per-interval arg). INCR only (no BF — user direction). **PARTIAL:
210/240 cells complete and valid** (each CSV: 60 rows, radii ∈ {2,4}, 600
activations/cell = 10×60). Missing: **N=12 × periods {5,10,20}** (30 cells) —
the previous sweep stopped there; nothing currently running. The script's
resume-skip (`_cell_done`) caches the 210 done cells, so a re-run with the full
grid runs only the 30 missing (~3-5 min at `-j 6`).

**Aggregation (P24-Eval.4, `simulation_experiments/aggregate_p24eval.py`).**
Pools all activations across replicates per (period, N). Fixed two issues in the
on-disk state: (1) the stale `sweep_manifest.csv` had only 180 rows (no N=12 at
all), so the 30 existing N=12 cells (p0/p1/p3) were invisible — rebuilt it from
the on-disk CSVs (210 rows, `avg_sp` re-read from each taskset's SP summary; old
manifest backed up to `sweep_manifest.stale.bak`); (2) added a `reps` column to
the table so partial cells are explicit. Results (21 rows = 18 full + 3 partial
N=12; N=12 p5/p10/p20 absent — not run):

| period | N | mean (s) | std (s) | median (s) | max (s) | reopt/run | reps | mean SP |
|---|---|---|---|---|---|---|---|---|
| 0 | 6 | 0.024359 | 0.026043 | 0.014386 | 0.142978 | 0.0 | 10 | 3.26820 |
| 0 | 8 | 0.060188 | 0.062151 | 0.034670 | 0.272198 | 0.0 | 10 | 3.32060 |
| 0 | 10 | 0.322165 | 0.438534 | 0.177534 | 2.175280 | 0.0 | 10 | 3.20147 |
| 0 | 12 | 0.797750 | 0.746557 | 0.641627 | 3.420660 | 0.0 | 10 | 3.17028 |
| 1 | 6 | 0.031576 | 0.034458 | 0.019949 | 0.209931 | 0.0 | 10 | 3.26820 |
| 1 | 8 | 0.067615 | 0.065947 | 0.036016 | 0.286828 | 0.0 | 10 | 3.32060 |
| 1 | 10 | 0.479270 | 0.717357 | 0.215778 | 3.551930 | 0.0 | 10 | 3.20147 |
| 1 | 12 | 1.152793 | 1.102168 | 0.856944 | 4.550880 | 0.0 | 10 | 3.17028 |
| 3 | 6 | 0.026459 | 0.029132 | 0.016114 | 0.200404 | 20.0 | 10 | 3.26820 |
| 3 | 8 | 0.063337 | 0.064922 | 0.034484 | 0.286193 | 20.0 | 10 | 3.32060 |
| 3 | 10 | 0.375580 | 0.544271 | 0.190901 | 3.242920 | 20.0 | 10 | 3.20147 |
| 3 | 12 | 0.902670 | 0.875348 | 0.688864 | 4.092690 | 20.0 | 10 | 3.17028 |
| 5 | 6 | 0.025314 | 0.027650 | 0.014769 | 0.158664 | 12.0 | 10 | 3.26820 |
| 5 | 8 | 0.073711 | 0.078706 | 0.037657 | 0.430734 | 12.0 | 10 | 3.32060 |
| 5 | 10 | 1.092200 | 1.603802 | 0.631219 | 11.324300 | 12.0 | 10 | 3.20147 |
| 10 | 6 | 0.061950 | 0.077301 | 0.029920 | 0.509818 | 6.0 | 10 | 3.26820 |
| 10 | 8 | 0.220097 | 0.238202 | 0.116366 | 1.352540 | 6.0 | 10 | 3.32060 |
| 10 | 10 | 0.735810 | 0.755582 | 0.591150 | 4.934070 | 6.0 | 10 | 3.20147 |
| 20 | 6 | 0.080203 | 0.094605 | 0.042225 | 0.705285 | 3.0 | 10 | 3.26820 |
| 20 | 8 | 0.220014 | 0.247343 | 0.125032 | 1.273040 | 3.0 | 10 | 3.32060 |
| 20 | 10 | 0.638651 | 0.660779 | 0.508212 | 4.146270 | 3.0 | 10 | 3.20147 |

`reopt/run` = activations using the wide radius per 60-interval run (period>0:
~60/period; period=0: 0). `reps` = tasksets contributing (10 = full cell; <10 =
partial). No total-exec column (per spec).

**Reading the numbers.** (1) **Runtime scales steeply with N**, not just with
reopt frequency: at period 0 (no reopt), mean per-activation runtime goes
0.024s (N=6) → 0.060s (N=8) → 0.322s (N=10) → 0.798s (N=12) — roughly 2.5–3×
per +2 tasks, so the INCR optimizer body itself is the cost driver at larger N,
independent of reoptimization. This is the real P18 signal: per-interval
optimizer cost crosses 0.1s at N≥8 even with reopt OFF, and reaches ~0.8s at
N=12. (2) **Reopt cadence affects cost less than N does**: holding N=10, mean
runtime is 0.322s (p0) → 0.479s (p1) → 0.376s (p3) → 1.092s (p5) → 0.736s (p10)
→ 0.639s (p20) — the wide-radius activations add cost, but the variance is high
(std often exceeds the mean) because a single reopt activation can be expensive
(e.g. p5 N=10 max 11.3s on one taskset's interval 10). (3) **SP is essentially
flat across periods** within each N (e.g. N=6 ≈ 3.27 everywhere, N=8 ≈ 3.32),
consistent with the P24.5 finding that reoptimization does not regress SP. The
slight SP droop with N (3.27 → 3.32 → 3.20 → 3.17) is the P12 contention effect
(more tasks = harder to meet all deadlines), not a reopt effect.

**Caveat.** N=12 is partial (only p0/p1/p3); the N=12 p5/p10/p20 rows are
absent because those cells were not run. To complete: re-run
`python3 -m simulation_experiments.run_p24eval_sweep -j 6` (resume-skip runs
only the 30 missing), then `python3 -m simulation_experiments.aggregate_p24eval`
and re-paste the table. Files: `simulation_experiments/{run_p24eval_sweep,
aggregate_p24eval,gen_p24eval_tasksets}.py` (untracked), outputs under
`simulation_experiments/optimizer_comparison/p24_eval/`.


### P25: counter-driven reoptimization with compare-and-keep (2026-07-03)

> Design-decision lock recorded **before any source edit** (per `P24_task.md`
> Task 1). P24 was the radius-only mechanism; P25 is this redesign that decouples
> the from-scratch trigger from the TL cache so the control flow matches the data
> model. Working doc: `agents/P24_task.md` (titled P25).

**Why redesign.** P24's `ReoptimizationPeriod` counter only sets the search
radius — it never triggers from-scratch. `OptimizeIncre_w_TL` always runs
coordinate descent; inside it `EvaluateTimeLimitConfig` decides from-scratch vs
incremental by **cache novelty** (`timelimit2optimizer_.count(time_limits)`),
not by the counter. So P24 delivers periodic *wider-radius* coordinate descent,
not periodic from-scratch re-optimization. The knob oversells what it controls.

**Design: compare-and-keep, counter-driven, no reset.** Two paths dispatched per
interval in `Optimize_w_TL_ScratchOrIncre`. The persistent optimizer carries a
**best-so-far** = `{TL_prev, PA_prev, opt_sp_prev, dag_tasks_prev}`.

- *Incremental path* (most intervals; every interval when `period == 0`): apply
  `TL_prev` to the fresh raw DAG → `dag_new`; coordinate descent over
  **narrow-radius** TL options (`TimeLimitSearchRadiusIncr`), each candidate
  evaluated via `OptimizeIncre` warm-started from best-so-far. Updates best-so-far
  in place.
- *Reoptimization path* (`count % ReoptimizationPeriod == 0`, `period > 0`),
  **COMPARE-AND-KEEP**: (i) re-evaluate best-so-far's SP under the new DAG,
  `SP_prev_new = EvaluateSPWithPriorityVec(dag_new_with_TL_prev, sp_params,
  PA_prev)` (== first line of `OptimizeIncre`, `OptimizeSP_Incre.cpp:235` —
  primitive already exists, needed so both SPs are under the SAME new DAG);
  (ii) from-scratch reoptimization = fresh `OptimizePA_Incre_with_TimeLimits`
  with **wide radius** (`ReoptimizationTimeLimitsSearchRadius`, bounded NOT full)
  on `dag_new` — "normal from-scratch" = `OptimizeFromScratch(K)` for the first
  TL candidate (full beam, ignores `opt_pa_`, escapes PA drift), then
  `OptimizeIncre` warm-started for the rest of the wide-radius CD →
  `{TL_fs, PA_fs, SP_fs}`; (iii) compare + keep winner: if `SP_fs > SP_prev_new`
  adopt the from-scratch result, else keep best-so-far with `opt_sp_ = SP_prev_new`
  and `dag_tasks_ = dag_new`. Tie-break (equal SP): lower sum of time limits
  (today's `UpdateRecords` rule). **No state clearing** — best-so-far is only
  replaced if genuinely beaten.

**Minimal persistent optimizer state = `{TL, PA, opt_sp, dag_tasks_}`**, where
`dag_tasks_` is the last-evaluated **TL-applied** DAG (so the diff sees only
genuine ET-observation changes + any TL change). Verified against the code:
- `OptimizeFromScratch(K)` reads only `dag_tasks_` + `sp_parameters_` + `K`;
  does NOT read `opt_pa_`/`opt_sp_`; writes them fresh.
- `OptimizeIncre(dag_tasks_update)` requires `opt_pa_` non-empty, diffs
  `dag_tasks_` vs `dag_tasks_update`, generates 1-D PA variations only for
  changed-ET tasks.
- `OptimizeIncre` has no small-change assumption — works for any ET change;
  cost scales with the number of differing tasks. By design one task's TL
  changes per coordinate-descent step → exactly one task's ET changes per step.
- Optimizer state already persists across intervals: `incr_optimizer_` is a
  member of `FixedTaskPrioritySchedulingOrchestrator`, constructed once in
  `RunSimulation`. Cache, counter, `opt_sp_`/`opt_pa_` already survive.
- `UpdateDAG` copies the new DAG into `dag_tasks_`, so `FindTaskWithDifferentEt`
  sees the diff on the next incremental call.

**Three independent tunable knobs** (all already in `parameters.yaml`):
`ReoptimizationPeriod` (default 10, 0=off), `TimeLimitSearchRadiusIncr`
(default 2, narrow), `ReoptimizationTimeLimitsSearchRadius` (default 4, wide,
bounded NOT full search).

**Impl note.** "initial from-scratch" (new `RunSimulation`, counter reset to 0,
`OptimizeFromScratch_w_TL`) and "reoptimization from-scratch" (counter NOT reset)
are two distinct operations — the reoptimization from-scratch must NOT zero
`reoptimization_interval_count_`, or the next interval would re-trigger
immediately.

**Test inventory (existing tests touching this path):**
- `tests/testIncreOpt_w_TL.cpp`: P24 counter tests 554-633
  (`DefaultsLoadedFromYaml`, `PeriodZeroAlwaysUsesNarrowRadius`,
  `WideRadiusOnReoptIntervalsNarrowElsewhere`,
  `CounterResetsAcrossFreshOptimizeFromScratch`) assert radius selection +
  counter advance + counter reset on `OptimizeFromScratch_w_TL`; warm-start test
  450-484 (`OptimizeWithOptimizationSpace`); `OptimizeIncre_w_TL` direct-call
  tests 282-314, 456-484.
- `tests/testOptimizeIncrePA.cpp`: `ComplexityLinear_3N` (294-316) reads
  `optimizer.timelimit2optimizer_.size()` — **depends on the cache; updated in
  Task 5/6 when the member is removed.**
- `tests/testBF_w_TL.cpp`: passes `TimeLimitSearchRadiusIncr` to
  `OptimizeIncre_w_TL` (mechanical call-site).
- `tests/AnalyzePriorityAssignmentIncrementalExample.cpp:81`: calls
  `OptimizeIncre_w_TL` directly with `TimeLimitSearchRadiusIncr` (mechanical).

**Public-method surface stays:** `OptimizeIncre_w_TL(dag, K, radius)` and
`OptimizeFromScratch_w_TL(K)` both kept; `Optimize_w_TL_ScratchOrIncre` stays the
counter-driven entry point. No `SimulationOrchestrator` change (already calls
the single entry point 1:1 per interval, state already persists).

---

## 2026-07-03 → "Benchmark/diag/eval consolidation + repo hygiene"

Consolidated the scattered benchmark / diagnostic / eval-scratch files into a
single ignored `benchmark/` folder and cleaned up tracked-but-ignored state.

**Moved into `benchmark/` (all previously untracked, scattered):**
- `bench_incr_commit.cpp` — the INCR-vs-commit benchmark driver (was
  `Testing/bench_incr_commit.cpp`). Times `OptimizeIncre_w_TL` (cold + warm) on
  5 × 10-task tasksets, prints SP + ms + eval_count. Built standalone at `-O2`
  against the Release `libSP_OPT.so`; not wired into CMake.
- `diag_p25.cpp` — the v19/v19_2 time-limit sanity diagnostic (was
  `tests/diag_p25.cpp`). Standalone, not wired into CMake.
- `run_p24eval_sweep.py`, `aggregate_p24eval.py`, `gen_p24eval_tasksets.py` —
  the P24-Eval re-optimization sweep harness (was in `simulation_experiments/`).
- `logs/` — sweep run logs + stale `.pid` (was
  `simulation_experiments/logs/`).

**Path patches made so the moved p24eval scripts still run from `benchmark/`:**
each script's `OPTIMIZER_COMPARISON_DIR` / `P24_EVAL_DIR` / `DEFAULT_BIN` were
`dirname(__file__)/...`-relative, which broke on move. Re-rooted them on
`_REPO_ROOT = dirname(dirname(__file__))` (location-independent), pointing
output at `simulation_experiments/optimizer_comparison/p24_eval` and the binary
at `release/tests/RunOrchestrator`. `gen_p24eval_tasksets.py` also got a
`sys.path.insert(0, _REPO_ROOT)` bootstrap (mirrors `compare_optimizers.py`) so
its `from Gen_Taskset...` import resolves when run directly as a script.
Verified: all three compile, import, resolve paths, and `gen_p24eval --help`
runs from the new location.

**`.gitignore`:** added root-anchored `/benchmark/` and `/Testing/` (the latter
is CMake's ephemeral `Testing/Temporary/` cost-log dir).

**Untracked `agents/dev_log.md`:** it was listed in `.gitignore` (line 48) but
still tracked — gitignore does not auto-untrack already-tracked files. Ran
`git rm --cached agents/dev_log.md` (file kept on disk) so the ignore now takes
effect. This file is now a local-only working log.

**Net `git status` after staging:** the bench/diag/eval/log noise no longer
appears; only the pre-existing working-tree edits + `.gitignore` + the
`dev_log.md` untrack remain. Per `agent_coding_rules.md`, changes are staged
(`git add`) for user review; user runs `git commit`.

**Note for next session:** the entry above (P24 design, lines ~1297-1299) says
`testOptimizeIncrePA.cpp:ComplexityLinear_3N` reads
`timelimit2optimizer_.size()` and "depends on the cache." That was true when
written but commit `57d8db5d` already replaced it with `eval_count_` — the log
is stale on that one point, no action needed.

### P25 Commit 3 — Extract `EvaluateTimeLimitConfig_ScratchOrIncre` + `bool from_scratch` (2026-07-03)

Per `agents/P24_task.md` Commit 3. The from-scratch vs incremental decision was
previously **implicit** — `EvaluateTimeLimitConfig` picked the path by inspecting
`prev_optimizer_.IfInitialized()` (Commit 2's incumbent), so a caller could not
force a fresh `OptimizeFromScratch` while an incumbent existed. Commit 3 makes
the dispatch **explicit** via a `bool from_scratch` plumbed from the public entry
points down through coordinate descent to the evaluator.

**Changes (`sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}` only):**
1. Renamed `EvaluateTimeLimitConfig` → `EvaluateTimeLimitConfig_ScratchOrIncre`,
   added `bool from_scratch` param. Three-branch body:
   - `from_scratch == true` → fresh `OptimizePA_Incre(dag_cur, sp_parameters_)`
     + `OptimizeFromScratch(K)` (ignores `prev_optimizer_` — escapes PA drift).
   - `from_scratch == false` && `prev_optimizer_.IfInitialized()` → warm-start
     copy of `prev_optimizer_` + `OptimizeIncre` (Commit 2 incremental path).
   - `from_scratch == false` && no incumbent → `OptimizeFromScratch(K)` fallback
     (first evaluation ever; behavior unchanged from before).
2. `PerformCoordinateDescentForTaskConfigOpt` gained `bool from_scratch = false`
   (defaulted so unchanged call sites — none in-tree after this commit, but the
   default keeps the signature backward-compatible); it forwards the flag to
   every `EvaluateTimeLimitConfig_ScratchOrIncre` call in the descent loop.
3. `OptimizeIncre_w_TL(dag, K, radius)` calls descent + the `disable_time_limit_opt`
   short-circuit with `from_scratch = false` (incremental entry).
4. `ReOptimizePeriodic(dag, K, radius)` calls them with `from_scratch = true`
   (from-scratch entry — this is the renamed `OptimizeFromScratch_w_TL(dag,K,radius)`
   of the design doc; the rename itself was commit `c2121bbb`, not redone here).

**Naming note vs. design doc.** `P24_task.md`'s Commit 3 spec references
`OptimizeFromScratch_w_TL(dag, K, radius)` as the from-scratch entry. In the
current tree that method is named `ReOptimizePeriodic(dag, K, radius)` (renamed
in commit `c2121bbb "rename some methods"`). Commit 3 is purely the bool-dispatch
extraction; it does **not** rename anything. The mapping is:
`OptimizeIncre_w_TL` → `from_scratch=false`; `ReOptimizePeriodic` →
`from_scratch=true`. `SimulationOrchestrator.cpp` already routes `INCR` family →
`OptimizeIncre_w_TL` and `INCR_SCRATCH` → `ReOptimizePeriodic`, so the orchestrator
correctly exercises both dispatch branches with no orchestrator change.

**Verification:**
- Debug `ctest` → **16/16 green** (incl. `testIncreOpt_w_TL` 17/17, `testOptimizeIncrePA`,
  `testBF_w_TL`, `AnalyzePriorityAssignmentIncrementalExample`). Baseline before
  the edit was also 16/16 — no regression.
- Release `RunOrchestrator` builds clean.
- Runtime smoke (release, `duration_ms=10000` per [[runorchestrator-duration-arg-semantics]]):
  - 1-task taskset: INCR and INCR_SCRATCH both SP 4.93704 (identical — single task,
    no PA freedom).
  - 8-task taskset (`tasks8_dur600_interval10_seed1000/taskset_0`): INCR SP 1.32579
    (6.37s), INCR_SCRATCH SP 1.70743 (8.99s). The from-scratch path finds a better
    PA (full beam re-search per interval) at higher cost — the expected
    quality/cost trade-off, confirming both dispatch branches run correctly at runtime.

**Files changed (working tree, uncommitted):** `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}`.
`git add` only — user commits (per `agent_coding_rules.md`). Awaiting user review
before Commit 4 (rename `OptimizeFromScratch_w_TL(int K)` → `ReOptimizePeriodic(int K)`,
mechanical).


### P25 Commit 5 — compare-and-keep `ReOptimizePeriodic(dag,K,radius)` + 4-tuple refactor (2026-07-03/04, landed at `6cd0f24f`)

> **STATUS: committed.** HEAD `6cd0f24f` "add SeedIncumbentBaseline to
> re-construct a baseline solution before performing incremental / from-scratch
> optimization" lands Commits 4 (rename) + 5 (compare-and-keep + refactor +
> helpers + tests) together. The three C++ files are clean in the working tree.
> `ctest` 16/16 green; `testIncreOpt_w_TL` 23/23.

Per `agents/P24_task.md` Commit 5. Implements the compare-and-keep
reoptimization entry point `ReOptimizePeriodic(const DAG_Model&, int, int)` and
refactors it around the **4-tuple optimizer-status model** `{dag, sp, pa, tl}`,
eliminating the earlier `restore_incumbent` / `SumTimeLimits` block (which the
memory file still describes — stale).

**Model: optimizer status = `{dag, sp, pa, tl}` carried in `prev_optimizer_`.**
Compare-and-keep is just `UpdateRecords`'s existing compare guard (strictly-
greater SP wins, tie-break lower TL-sum) once `opt_sp_` holds the seeded
baseline instead of `-1.0`. So the algorithm is: seed the incumbent baseline
into state → run a fresh wide-radius from-scratch coordinate descent → the
guard inside `UpdateRecords` adopts a candidate only if it strictly improves SP
(or ties with lower TL-sum); otherwise the baseline survives untouched. **No
separate restore step.**

**New helpers (each kept short, one job):**
- `SeedIncumbentBaseline()` — establishes the baseline BEFORE the search. Two
  branches: (a) have incumbent → re-eval its `{pa, tl}` under the NEW DAG (its
  carried SP was computed under an older DAG) via `EvaluateSPWithPriorityVec`;
  that re-evaluated tuple is the baseline. (b) interval 0 (no incumbent) →
  synthesize one from RM priorities + every task at its smallest TL option, and
  evaluate it. Guarantees a valid baseline to compare against.
- `SeedStateFromIncumbent(dag_with_tl, pa, sp, tl)` — seeds the full 4-tuple
  into state (`opt_sp_`/`opt_pa_`/`res_opt_`/`prev_optimizer_`). This is the
  baseline `UpdateRecords`' guard measures the search against, so `opt_sp_`
  must hold it (NOT `-1.0`) when the search runs.
- `ReconstructTimeLimitVec()` — rebuilds the positional per-task TL vector (in
  task order) from `res_opt_.id2time_limit`; tasks with no recorded TL get -1.
- `RateMonotonicPriorityVec()` — period-ascending priority vector (index 0 =
  highest priority); mirrors the orchestrator's RM mode. **Ties broken by
  average execution time ascending** (lower ET = higher priority) so the order
  is deterministic for tasks that share a period.
- `SmallestTimeLimitVec() const` — one TL per task, each at its smallest option
  (-1 if no `timePerformancePairs`). Extracted from the inline `tl_min` loop
  that was duplicated in `SeedIncumbentBaseline`'s interval-0 branch; both
  `InitializeTimeLimitsToSmallest` and `SeedIncumbentBaseline` now call it
  (single source of truth).

**`ReOptimizePeriodic(dag,K,radius)` body** (the compare-and-keep entry):
1. `dag_tasks_ = dag_tasks_update`; `ApplyWCETAblationIfRequired`;
   `RecordCloseTimeLimitOptions(dag, radius)`.
2. `SeedIncumbentBaseline()` — baseline in state (`opt_sp_` holds it, not -1.0).
3. Fresh wide-radius from-scratch descent (`PerformCoordinateDescentForTaskConfigOpt`
   with `from_scratch=true`, or `OptimizeWithTimeLimitOptDisabled` if
   `disable_time_limit_opt`). Each candidate is compared against the seeded
   baseline inside `UpdateRecords`; adopted only if strictly better. Otherwise
   baseline survives.
4. Return `opt_pa_`.

**Tests added (`tests/testIncreOpt_w_TL.cpp`, 15→23 total):** 2 compare-and-keep
end-to-end tests under `CompareAndKeepSynthetic` (`AdoptsWhenWideSearchWins`,
`KeepsIncumbentWhenWideSearchLoses`) + 7 direct unit tests for the new helpers:
`SmallestTimeLimitVec`, `ReconstructTimeLimitVec` (default -1 + round-trip),
`RateMonotonicPriorityVec` (period-ascending AND ET-tiebreaker), `SeedStateFromIncumbent`
(writes full 4-tuple), `SeedIncumbentBaseline` (interval-0 RM+minTL branch AND
re-eval-incumbent-under-new-DAG branch). The re-eval test mutates T_noise ET to
1900ms (breaks schedulability over the 2000ms period) so the SP delta is
observable — distinguishes "re-evaluated under new DAG" from "stale incumbent
SP carried forward".

**Files (committed at `6cd0f24f`):** `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}`,
`tests/testIncreOpt_w_TL.cpp`.


### P25 Commit 6 — counter-driven dispatcher `Optimize_w_TL_ScratchOrIncre` + radius fix (2026-07-04, committed as `9b3eeacf`)

> Closes the gap: compare-and-keep was unit-tested but never ran at runtime.
> The `ReoptimizationPeriod` (10) / `ReoptimizationTimeLimitsSearchRadius` (6)
> knobs were dead config — loaded from `parameters.yaml` but read by zero lines.

**Dispatcher.** Added `Optimize_w_TL_ScratchOrIncre(dag,K)` to
`OptimizePA_Incre_with_TimeLimits` (header decl + cpp def). Owns a persistent
`reoptimization_interval_count_` (starts 0, advances by 1 after every call,
**never resets** — modular arithmetic alone decides). Routing:

```
count % ReoptimizationPeriod == 0  → ReOptimizePeriodic(dag, K, wide_radius)   // compare-and-keep
else                               → OptimizeIncre_w_TL(dag, K, narrow_radius) // incremental
count++
```

Both delegates already set `dag_tasks_` and honor `disable_time_limit_opt` /
`use_wcet_execution_time`, so the dispatcher body is just the counter + routing.

**Interval-0 bootstrap (latent bug fix).** `DeterminePrioritiesAndBudgets` runs
at every interval including interval 0 (`SimulateInterval` loop from `i=0`). At
interval 0 `incr_optimizer_` is fresh (`prev_optimizer_` uninitialized), so the
old INCR path's `OptimizeIncre_w_TL` →
`EvaluateTimeLimitConfig_ScratchOrIncre(...,false)` hit the `CoutError`
contract-violation (`OptimizeSP_TL_Incre.cpp:153`) and threw. The dispatcher
fixes this for free: at `count == 0`, `0 % period == 0` routes to
`ReOptimizePeriodic`, whose `SeedIncumbentBaseline` interval-0 branch
synthesizes an RM+min-TL incumbent. The dispatcher thus doubles as the
interval-0 bootstrap.

**Orchestrator wiring.** `SimulationOrchestrator.cpp` INCR (:267), INCR_NO_TL
(:281), INCR_WCET (:289) switched from `incr_optimizer_.OptimizeIncre_w_TL(dag,K)`
to `incr_optimizer_.Optimize_w_TL_ScratchOrIncre(dag,K)`. The
`disable_time_limit_opt` / `use_wcet_execution_time` flag save/restore around
INCR_NO_TL / INCR_WCET is preserved (both delegated paths honor those flags).
`INCR_SCRATCH` (:273) untouched — separate ablation, builds a fresh `scratch_opt`
each interval and calls the 1-arg `ReOptimizePeriodic(K)` (does not touch the
counter).

**Radius-forwarding fix.** The 2-arg `OptimizeIncre_w_TL(dag,K)` was forwarding
to `ReoptimizationTimeLimitsSearchRadius` (6, the wide/reopt radius) instead of
`TimeLimitSearchRadiusIncr` (2, the narrow/incremental radius) — a leftover from
the Commit 5 refactor that collapsed the two-radius design. Corrected to forward
to `TimeLimitSearchRadiusIncr`, restoring Commit 1's intent (`P24_task.md:93`).
Now: incremental path = narrow radius (2), reopt path = wide radius (6) — the
two-radius distinction is real again.

**`ReoptimizationPeriod == 0` dropped.** Per user decision the knob is now
positive-only (min 1). The "0 disables" path is gone — it would have crashed
the INCR interval-0 bootstrap (the incremental branch has no incumbent →
`CoutError`). `parameters.yaml:12` comment still says "0 disables"; will be
updated in Commit 7.

**Tests (`tests/testIncreOpt_w_TL.cpp`, 23→26):**
- Updated `OptimizeWithOptimizationSpace` comment + assertion: under radius 2
  the incremental window for ET=1000 is [600,800,1000] (TL=400 unreachable);
  TL=600 is the schedulable optimum under the new DAG (800/1000 unschedulable),
  so the result is in [600,1000], not the bootstrap TL=400. Old `EXPECT_EQ(400)`
  was coupled to the radius-6 bug.
- 3 new tests under `CounterDispatcherSynthetic` (10 evenly-spaced TL options,
  ET=45 → closest index 4; radius 2 → 5 opts, radius 6 → 10 opts):
  `CounterAdvancesEveryCall_NeverResets` (3 calls → counter==3),
  `TriggersReoptAtCountZero_BootstrapsIncumbent` (count==0 routes to reopt,
  `IfInitialized()` true, 10 opts recorded — proves the reopt branch ran and
  bootstrapped rather than throwing),
  `RoutesToIncrementalAtNonModularCount` (call 2 at count==1 → 5 opts recorded
  — proves the narrow incremental branch ran, distinguishable from reopt).

**Verification.** `testIncreOpt_w_TL` 26/26 green; full `ctest` 16/16 green;
full build clean (RunOrchestrator + AnalyzePriorityAssignmentIncrementalExample
link against the changed `OptimizeSP_TL_Incre.{h,cpp}` +
`SimulationOrchestrator.cpp`).

**Files (committed in `9b3eeacf`):**
`sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}`,
`sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp`,
`tests/testIncreOpt_w_TL.cpp`.

> **Update (2026-07-04, later):** the "staged, NOT committed" status above is
> historical — Commit 6 landed as `9b3eeacf`. The post-Commit-6 INCR-ET fix
> bundle (Fix A `986a9cfe` + Fix B/latent bug `de4e9636`) followed; see the
> 2026-07-04 Fix A / Fix B entries below and `agents/debug_runtime0704_incr.md`.

**Next (Commit 7):** runtime A/B (period on vs off) + before/after SP/ET
numbers; update `parameters.yaml:12` comment; rewrite stale memory file
`p24-reoptimization-design.md`.



---

### 2026-07-04 — Reproduced "INCR ET grows with reoptimization period" (NOT fixed at time of writing — fixed later same day, see next entry + `agents/debug_runtime0704_incr.md`)

> **Update (2026-07-04, later):** the bug documented below **is now fixed** —
> Fix A (`986a9cfe`) + Fix B + latent `SeedStateFromIncumbent` bug (`de4e9636`),
> TDD-verified (16/16 ctest green). The "before" table here is preserved as the
> pre-fix snapshot. Only the runtime A/B re-run on the fixed binary is pending
> (see Open thread 1 / `agents/finished_tasks/P24_task.md` § "PENDING: A/B
> re-run"). Full root-cause + fix record: `agents/debug_runtime0704_incr.md`.

User report: in the P25 period A/B, INCR's per-activation ET appears *longer*
when more incremental optimization is used (larger period). Asked to reproduce
with an explicit test case and record in `P24_task.md`; explicitly told **not**
to solve it yet.

**Repro.** Built `simulation_experiments/repro_et_grows_with_period.py`: reuses
the existing P25 prod-run tasksets (no regeneration, deterministic across arms)
and runs every arm **serially** — one `RunOrchestrator` process at a time — to
remove the parallel-worker contention confound (prod used 4 workers × 6 arms on
8 cores; `RunOrchestrator` measures whole-process wall-clock, so concurrent arms
inflate each other's ET non-uniformly). Writes to
`optimizer_comparison/et_repro/<tag>/` so the prod run is untouched.

**Result (tasks=6, 30 intervals, per-act ET = wall_ms/30; min of 3 reps on ts0):**

| arm | ts0 ms/act | ts2 ms/act |
|-----|-----------|-----------|
| BF | 1099.8 | 6102.3 |
| INCR_P1 | 101.5 | 94.0 |
| INCR_P10 | 301.1 | 184.6 |
| INCR_P30 | 342.7 | 193.3 |
| INCR_P60 | 342.4 | 194.3 |
| INCR_SCRATCH | 106.4 | 93.9 |

Pattern is real and algorithmic (survives serial execution, reproduces on 2
tasksets): `INCR_P1` ≈ `INCR_SCRATCH` (fastest); ET jumps ~3× at P=10, climbs to
P=30, plateaus at P=60. Because P=30/P=60 have *fewer* reopt steps than P=10 yet
cost *more*, the cost is dominated by the **incremental path between reopts**,
not the reopt step itself — likely the incumbent drifts over a long period,
making each intervening incremental step more expensive. P=1 keeps the incumbent
fresh, so incremental steps are trivially cheap.

**Recorded in:** `agents/finished_tasks/P24_task.md` → "KNOWN ISSUE" section
(now updated: fix applied & TDD-verified, runtime A/B re-run pending).
**Memory:** `p25-incr-et-grows-with-period.md`. **Status at time of writing:**
not solved (fixed later same day — see next entry).

---

### 2026-07-04 — Fix B + latent `SeedStateFromIncumbent` bug (TDD, 16/16 green)

Continues the P25 INCR-ET TDD cycle from Fix A (see
`agents/debug_runtime0704_incr.md` §6). Two changes this pass, both verified by
failing-first tests that now pass:

**Latent bug — `SeedStateFromIncumbent` forgot `sp_parameters_`
(`sources/Optimization/OptimizeSP_TL_Incre.cpp`).** `SeedStateFromIncumbent`
seeded `prev_optimizer_`'s `dag`/`opt_pa_`/`opt_sp_` but NOT `sp_parameters_`.
`IfInitialized()` only checks `!opt_pa_.empty()`, so the incremental branch
(`EvaluateTimeLimitConfig_ScratchOrIncre`, `from_scratch=false`) took
`optimizer = prev_optimizer_` with an **empty** `sp_parameters_` →
`OptimizeIncre`'s SP-eval (`ObtainSP_TaskSet` → `thresholds_node.at(id)`) threw
`_Map_base::at`. In production this is **masked** because `UpdateRecords`
(`prev_optimizer_ = optimizer`, a full copy) usually fires between a reopt and
the next incremental call, populating `sp_parameters_` as a side effect. It only
bites when `UpdateRecords` never fires in that window — exactly the all-`{-1}`
no-improvement case Fix B's fallback depends on. Fix: one line,
`prev_optimizer_.sp_parameters_ = sp_parameters_;` in `SeedStateFromIncumbent`.
Verified safe in isolation (full ctest green save the two Fix B red tests, which
failed cleanly on assertions instead of crashing — proving the latent bug is
fixed and Test A now reaches its intended assertion).

**Fix B — `{-1}`-only per-task skip + zero-work fallback
(`sources/Optimization/OptimizeSP_TL_Incre.cpp`,
`PerformCoordinateDescentForTaskConfigOpt`).**
1. **Skip** a task whose only TL option is `-1` (`opts.size()==1 &&
   opts[0]==-1.0`). `RecordCloseTimeLimitOptions` only pushes `-1` when a task
   has zero `timePerformancePairs`, so this is the exact no-pairs predicate and
   can't skip a task with real options. This is the main cost win on the reused
   P25 tasksets (every task is `{-1}`-only): the descent no longer runs N
   redundant incumbent re-evals per interval.
2. **Zero-work fallback.** When *every* task was `{-1}`-only (loop ran zero
   evals), run **one** `EvaluateTimeLimitConfig_ScratchOrIncre(K, time_limits,
   from_scratch)` with the incumbent `time_limits`. This makes `UpdateRecords`
   fire, and Fix A's `dag_tasks_ = dag_tasks_update` inside `OptimizeIncre`
   propagates the current interval's DAG into `prev_optimizer_` — otherwise the
   frozen-baseline pathology Fix A fixes would silently return (the `{-1}`-only
   skip starves `UpdateRecords`). Guarded on `!dag_tasks_.tasks.empty()` so
   empty-DAG callers keep the prior "no evals, no crash" behavior, and gated on
   `!any_eval_ran` so it never double-counts when the descent already produced
   >0 evals.

**TDD (red→green).** Added two tests in `tests/testIncreOpt_w_TL.cpp`
(`CompareAndKeepSynthetic` fixture): `PerformCoordinateDescent_AllMinusOneOnly_…`
(asserts eval_count delta == 1 — the fallback — AND `prev_optimizer_.dag_tasks_`
advances to the current interval's ET, the trap guard) and
`PerformCoordinateDescent_SkipsMinusOneOnlyTaskInMixedSet` (mixed set: `{-1}`-only
task skipped, fallback does NOT fire, delta == `t_perf_option_count`). Both FAIL
on the latent-bug-fixed-but-Fix-B-not-yet code (deltas 2 and +1) and PASS after
Fix B.

**Status: 16/16 ctest green in the DEBUG build** (`testIncreOpt_w_TL` 29/29,
`testOptimizeIncrePA` green). The Fix A + latent-bug-fix + Fix B bundle is now
**committed** (`986a9cfe` Fix A, `de4e9636` Fix B + latent bug). The only
remaining item is the runtime A/B re-run (`repro_et_grows_with_period.py`) to
confirm the ordering flips to `INCR_P1 ≥ INCR_P10 ≥ INCR_P30 ≈ INCR_P60` (ET
non-increasing in period) — pending the next session; needs
`cmake --build release` (the on-disk release binary predates the fix commits)
and possibly regenerating the prod-run tasksets (absent locally in the
2026-07-04 TDD session). Full record: `agents/debug_runtime0704_incr.md`;
re-run procedure: `agents/finished_tasks/P24_task.md` § "DONE: A/B re-run".

---

## 2026-07-04 19:41 — P25 runtime A/B re-run on the fixed binary (Fix A+B):
PARTIAL PASS — pathological growth eliminated; literal directional flip NOT met
(Fix C deferred)

Followed `agents/finished_tasks/P24_task.md` § "DONE: A/B re-run". Rebuilt the
release binary (`cmake --build release`, 19:34 build, HEAD `de4e9636`) — the
on-disk binary (12:30) predated both Fix A (`986a9cfe`, 14:11) and Fix B
(`de4e9636`, 19:15). The uncommitted `tests/RunOrchestrator.cpp`
`INCR_P<n>` period-override edit is required for the A/B (committed
`SimulationOrchestrator.cpp` `IsINCRPeriodVariant` depends on it parsing the
`<n>` suffix) and was compiled in. `debugMode: 0` in `parameters.yaml`
(instrumentation inert, no A/B skew). Prod-run tasksets present locally (no
regeneration). Ran `repro_et_grows_with_period.py` serially on ts0 (3 reps)
and ts2 (1 rep); before-snapshots preserved as `et_repro_result_BEFORE_fix.json`.

Per-activation ET (min ms/act, 30 intervals; before → after):

| arm          | ts0 before→after | ts2 before→after |
|--------------|------------------|------------------|
| BF           | 1099.8 → 1120.8  | 6102.3 → 5522.0  |
| INCR_P1      | 101.5 → **62.0** | 94.0 → **65.2**  |
| INCR_P10     | 301.1 → 87.7     | 184.6 → 74.9     |
| INCR_P30     | 342.7 → 90.8     | 193.3 → 71.9     |
| INCR_P60     | 342.4 → 90.8     | 194.3 → 73.4     |
| INCR_SCRATCH | 106.4 → 62.7     | 93.9 → 69.0      |

**Verdict — partial pass:**
- ✅ **Pathological growth eliminated.** P10/P30/P60 collapsed from 301–343 →
  88–91 (ts0) and 185–194 → 72–75 (ts2), toward the P1 ≈ INCR_SCRATCH floor
  (~62–69). P60/P1 ratio: 3.4× → 1.47× (ts0), 2.1× → 1.13× (ts2).
- ✅ **Numerical pass criterion met.** P30/P60 do not exceed P10 by the 1.1–1.3×
  bar: ts0 1.035× (was 1.14×), ts2 0.96× (was 1.05×, now slightly below P10).
- ❌ **Literal directional flip NOT met.** `INCR_P1 ≥ P10 ≥ P30 ≈ P60` is false
  on both: P1 is still the *cheapest* INCR arm (62 < 88–91 ts0; 65 < 72–75 ts2).
  The direction flattened, not flipped.
- **Why (understood, not a regression):** §4a/§6 of `debug_runtime0704_incr.md`
  predict exactly this for a Fix A+B-only deployment. Fix A killed the
  frozen-baseline pathology (the actual bug — `ndiff` saturating at 6 every
  incremental interval → ~132 `ObtainSP_DAG`/interval). The residual P1 < P60
  gap is the per-variation `ObtainSP_DAG` asymmetry: `OptimizeIncre` re-scores
  every priority variation with the full `ObtainSP_DAG` kernel, while
  `OptimizeFromScratch` uses cheap `GetRTA_OneTask` during beam search and calls
  `ObtainSP_DAG` once at the end. So an INCRE interval is still slightly costlier
  than a REOPT interval. **Fix C** (deferred in §6 — make `OptimizeIncre`'s
  per-variation scoring incremental too) is the lever for a full flip. Fix A+B
  brought INCRE down from pathological (3.4× REOPT) to moderately above REOPT
  (1.13–1.47×); Fix C would bring it below.

**Conclusion:** the issue ("INCR per-act ET grows with reoptimization period")
is resolved in the sense that motivated the bug report — the period no longer
inflates ET by 3×; P10/P30/P60 are flat and near the floor. The stricter "ET
decreases as period grows" expectation was based on the same root-cause
analysis that §7 later showed mechanistically wrong (cost scales with "is the
incremental path exercised," not staleness), so the literal flip was
over-optimistic given Fix C was deferred. Re-opening for a full flip is a
Fix C task, not a Fix A/B regression.

Full record: `agents/debug_runtime0704_incr.md`; procedure + before/after
tables: `agents/finished_tasks/P24_task.md` § "DONE: A/B re-run"; memory
[[p25-incr-et-grows-with-period]].

---

## 2026-07-05 — Trial-and-Error TL optimization (dynamic-radius walk) landed

Closes the "NEW TASKS" item "Improve the algorithm of
`PerformCoordinateDescentForTaskConfigOpt` … using a trial-and-error approach
(dynamic radius search). Initial radius: incr uses 1, re-opt uses 2." Full
spec/record: `agents/finished_tasks/trial_and_error_tl_opt_task.md`. **Committed
in `88af2c54` (global patience params) + `fa0b857f` (remove radius, add patience,
TL search → trial-and-error)** — P0.1 commit group (c).

**What changed.** `PerformCoordinateDescentForTaskConfigOpt` no longer
enumerates every TL option in the radius window. Each task now gets a
unidirectional outward walk (`OptimizeSingleTaskTimeLimit`) from its baseline
TL — backward pass `step=-1`, then forward pass `step=+1` — that adopts
improving candidates and stops after `patience` consecutive non-improving SP
evals. Three pure helpers extracted so the lookup and the adopt predicate are
unit-testable in isolation (they take no optimizer state):

- `FindTimeLimitOptionIndex(options, current_val)` — linear scan; returns
  `options.size()` (the off-the-end sentinel, mirroring `std::find`) when the
  value is absent. The walk treats the sentinel as "baseline not in window,
  do not search."
- `IsBetterTimeLimitOption(new_sp, current_best_sp, step)` — strict-SP-greater
  wins, OR (approx-equal tie AND `step < 0`) so the tie-break prefers the
  smaller TL on the backward pass. Mirrors the exhaustive descent's tie-break.
- `OptimizeSingleTaskTimeLimit(task_idx, K, time_limits, current_sp,
  baseline_val, step, from_scratch, patience)` — the walk. Returns the best SP
  found and leaves `time_limits[task_idx]` at the best option tried.

`patience` is decided inside the descent from `from_scratch` (not a new knob):
**incremental path `patience=0`** (strict break — the radius=1 window has at
most 3 options, no room for a dip to hide a better option further out; strict
is safe and cheapest); **reopt path `patience=1`** (tolerate one non-monotonic
dip — the radius=2 window can be non-unimodal at high utilization, so a single
dip must not hide a strictly better option one step further). This is the one
deviation from the original spec, which said "terminate immediately"; the
patience budget is the generalization that makes the reopt path robust.

**Radii (`sources/parameters.yaml`):** `ReoptimizationTimeLimitSearchRadius`
6→2 (≤5 options), `IncrementalTimeLimitSearchRadius` 2→1 (≤3 options), with an
inline comment describing the walk + patience semantics.

**Tests (`tests/testIncreOpt_w_TL.cpp`, 26→43):**
- `TrialAndErrorTLWalkSynthetic` fixture (7 tests): the walk is unit-tested
  with a deterministic TL→SP stub (`StubTLWalkOptimizer`) overriding the
  virtual `EvaluateTimeLimitConfig_ScratchOrIncre`, so the walk's control flow
  is asserted independently of the RTA-backed evaluator. Covers:
  strict-monotone-adopts-to-end, strict-break-on-first-non-improvement,
  patience=1-tolerates-one-dip-and-finds-optimum-further-out,
  patience=1-breaks-after-two-consecutive-non-improvements,
  backward-tie-break-adopts-smallest-on-flat-SP, no-options-noop,
  baseline-not-in-options-noop.
- `FindTimeLimitOptionIndexTest` (3) + `IsBetterTimeLimitOptionTest` (4):
  direct unit tests for the pure helpers (present/absent/empty; strict-higher,
  strict-lower, approx-equal-tie-better-only-when-walking-down,
  near-equal-within-tolerance-is-tie).

**Three stale test-expectation updates (NOT source bugs).** These assertions
were coupled to the old radius-6/radius-2 windows; the walk's behavior under
the narrower windows was verified by instrumenting the SP landscape
(`UpdateRecords` + `SeedIncumbentBaseline` printf diagnostics, since removed —
source is clean):
- `TaskSetForTest_robotics_v19::optimize_incremental` — bootstrap wide-radius
  (2) window for TSP (closest TL=1000, idx 3) is `[600,800,1000]`; **400
  excluded**. SP saturates at high util: 1000 strictly beats the RM baseline
  (9.671 > 7.845), then 800 and 600 tie and the smaller-TL tie-break adopts
  each → **600**. Was expecting 400. The incremental narrow-radius (1) window
  on the low-util v21 DAG is `[400,600]` (800/1000 excluded); SP strictly
  increases with TL there (12.529 > 12.429), so **600** is the within-window
  optimum. Was expecting ≥800 (assumed radius-2 window).
- `TaskSetForTest_robotics_v19_2::ReOptimizePeriodic` — same wide-radius
  landscape as v19 (TSP gets 600 via tie-break). Was expecting 400.
- `CompareAndKeepSynthetic::PerformCoordinateDescent_SkipsMinusOneOnlyTaskInMixedSet`
  — the old test conflated the **wide bootstrap radius** (2 → 4 options for
  `t_perf_option_count`) with the **narrow incremental radius** (1 → 3 options
  actually evaluated). Rewritten to assert the eval count equals T_perf's
  **narrow-window** size (3: baseline + backward-to-boundary + forward-to-
  boundary), T_noise contributes 0 (skip works), and no fallback eval fires.

**Key correction to the prior reasoning.** The earlier hypothesis was "all TLs
produce near-identical SP → tie-break → smallest." The instrumented data showed
TSP's `performance_records_perf` (0.5/0.6/0.8/1.0) actually makes SP **strictly
increase** with TL at low utilization, and only **saturates** (tie) at high
utilization where the deadline miss caps achievable SP. The 600 result is
correct in both regimes — for the right reason in each.

**Verification.** `testIncreOpt_w_TL` = **43/43 green**; full `ctest` =
**16/16 green**. Source diff is the pre-existing trial-and-error rewrite; no
diagnostic prints left behind (`grep -n "DIAG\|printf" sources/Optimization/
OptimizeSP_TL_Incre.cpp` clean; `git diff` shows no printf/DIAG additions).

**Open follow-ups (sibling "NEW TASKS", NOT this task):**
- Apply the same trial-and-error idea to incremental **priority** assignment.
- Runtime A/B: from-scratch vs incremental when both radii are equal, to verify
  whether from-scratch is actually faster than incremental.

---

## 2026-07-06 — P0.1 (commit pending uncommitted work) closed

All four commit groups landed (`4b44aabc`, `1ede254f`, `88af2c54`/`fa0b857f`,
`9c921fc0`/`99eda512`/`66c96c14`/`dab92e36`); clean baseline established.
"Done when" gate verified: `git status` clean, `ctest` 16/16, `pytest` 261
passed. Folder archived to
`agents/finished_tasks/2026-07-06_P0_1_commit_pending_work/`. Next: P0.2 (BF
correctness audit).


## 2026-07-07 — P1.1 gate RESOLVED: 2-vs-8 (3-vs-5) changed-task discrepancy

Rebuilt `release` RunOrchestrator with the `debugMode:1` probes already in the
working tree (`SeedStateFromIncumbent` dump + `FirstIncreDump`), ran INCR_P10
on the N=8 taskset_0. **Gate reconciled.** Ground truth = 3 tasks change
Gaussian params interval 0→1 (ids 5,6,7); runtime `ndiff=5` (ids 0,1,2,6,7).

**Root cause:** `FindTaskWithDifferentEt` (`OptimizeSP_Incre.cpp:140-155`)
compares `execution_time_dist` via `FiniteDist::operator!=`, but **both sides
are TL-applied point dists, not the underlying YAML Gaussians.** The baseline
(`prev_optimizer_.dag_tasks_`) carries the interval-0 REOPT's *adopted* TL
(`SeedIncumbentBaseline` seeds min-TL, then the from-scratch descent +
`UpdateRecords` adopts a strictly-better TL); the update passed to
`OptimizeIncre` is `dag_tasks_cur = UpdateExtDistBasedOnTimeLimit(dag_tasks_,
time_limits)` from `EvaluateTimeLimitConfig_ScratchOrIncre:148-149` — the
interval-1 descent's current TL. So a perf-pair task is flagged iff
adopted-TL ≠ descent-TL, independent of whether the YAML Gaussian moved:
tasks 0,1,2 = false positives (YAML identical, TL drifted); task 5 = false
negative (YAML mu changed, masked because adopted-TL == descent-TL); only
gaussian-only tasks (3,6,7; TL=-1 → `UpdateExtDistBasedOnTimeLimit` no-op)
compare the raw Gaussian and flag correctly.

**Implication for the residual:** the per-activation ET residual is
"TL-drift false positives in the diff" (perf-pair tasks swept against point
dists that don't reflect a real interval-to-interval ET change), NOT
"per-variation scoring asymmetry." The lever is *what gets diffed*, not *how
variations are scored*. **Fix D inert-ness also confirmed:** `FiniteDist::approx_equal`
(`Probability.cpp:345-357`) has zero production callers; the live comparison
`operator!=` hardcodes tolerance=1e-1 and does not delegate to it. Full record:
`agents/active_tasks/P1_1_p25_residual_investigation/dev_log.md`. Remaining:
equal-radii A/B (step 2) + the re-frame/Fix-C decision (step 4).

## 2026-07-07 (later) — P1.1 YARDSTICK CORRECTION (user): YAML ≠ ground truth for TL-optimizable tasks

User correction to the gate writeup above: for tasks whose TL can be optimized
(has `timePerformancePairs`), the YAML Gaussian / `performance_records_*` are
NOT ground truth — they were generated without optimization results. A
TL-optimizable task's effective ET during interval N−1 IS the TL the optimizer
adopted that interval; at interval N, before re-optimizing, the correct
assumption is ET = last adopted TL. The YAML Gaussian is only a cold-start
reference, never the "previous ET."

Under the corrected yardstick, a perf-pair task's ET changes between intervals
iff its adopted TL changes (an optimization outcome), NOT when the YAML mu
moves. (For non-perf-pair tasks TL=-1, `UpdateExtDistBasedOnTimeLimit` is a
no-op, so ET = raw Gaussian and changes iff the Gaussian changed.)

**Re-derived discrepancy** (INCR_P10, N=8 taskset_0, interval 0→1): ground
truth = **2** changed (tasks 6,7 — the gaussian-only tasks). `ndiff=5` = **3
false positives (0,1,2 — update side used the Gaussian-mean TL, not the
carried adopted TL) + 0 false negatives.** Task 5 is a **true negative**
(adopted TL 23.467 unchanged → ET unchanged; its YAML mu move is irrelevant for
a perf-pair task). The earlier "false negative on task 5" was an artifact of
using YAML as the yardstick.

**Precise lever:** `OptimizeIncre_w_TL` (`OptimizeSP_TL_Incre.cpp:373`) starts
the incremental descent from `InitializeTimeLimitsFromETConfig()` (closest-to-
Gaussian-mean TL), so the update side of `OptimizeIncre`'s diff is point dists
at the **Gaussian-mean** TL, not the carried adopted TL. The baseline side
correctly carries the interval-(N−1) adopted-TL point dists. Fix = start the
incremental descent from `ReconstructTimeLimitVecFromResOpt()` (carried adopted
TL; already exists `OptimizeSP_TL_Incre.cpp:385-394`, already used by
`SeedIncumbentBaseline` for the reopt-path baseline re-eval at line 464). Edge
case for the eventual fix: a task with a perf pair in N−1 but none in N would
get a stale adopted TL applied as a point dist — intersect carried TL against
the current option set (or use -1) before applying.

**This OVERTURNS the earlier "diff the underlying Gaussians" suggestion** —
diffing Gaussians would be WRONG (the Gaussian is explicitly not the ET for
TL-optimizable tasks). Fix D (GetAvgValue band) and Fix C (per-variation
scoring) are both the wrong lever; the lever is the descent start TL. Still
investigation-only (2026-07-06 hold on implementing Fix C stands). Full record:
`agents/active_tasks/P1_1_p25_residual_investigation/dev_log.md` (YARDSTICK
CORRECTION section).



## 2026-07-07 (later still) — P1.1 DECISION: implement descent-start-TL fix (user-approved)

User approved the fix identified by the yardstick correction ("i agree, that
needs to be fixed ... we need to initialize time limit in that way"). This
lifts the 2026-07-06 implement-only hold **for the descent-start-TL lever
only**. `OptimizeIncre_w_TL` (`OptimizeSP_TL_Incre.cpp:373`) will start the
incremental descent from `ReconstructTimeLimitVecFromResOpt()` (carried adopted
TL) instead of `InitializeTimeLimitsFromETConfig()` (Gaussian-mean TL), with an
edge-case guard forcing -1 for a task whose carried TL is no longer in the
current option set (e.g. lost its perf pair). This makes the update side of
`FindTaskWithDifferentEt`'s diff point dists at the carried adopted TL =
baseline for unchanged perf-pair tasks → eliminates the 3 false positives
(tasks 0,1,2), `ndiff` 5 → ~2 (only gaussian-only tasks 6,7 flag). Fix C and
Fix D remain NOT in scope (wrong levers); equal-radii A/B dropped (radii don't
touch the descent start TL). TDD: failing test → one-line change + guard →
suite + ctest → re-run INCR_P10 N=8 taskset_0 probe. Full record:
`agents/active_tasks/P1_1_p25_residual_investigation/dev_log.md`.

## 2026-07-08 — P0.5 incumbent-state redesign LANDED (working tree); P1.1 probe PASS

**P0.5 — Redesign the optimizer iteration process (incumbent-state refactor).**
The decided design (incumbent owned **once** in `res_opt_`; `has_incumbent_`
gate; `CommitIncumbent`/`BuildChallengerFromIncumbent` helpers; transient
challenger rebuilt from `res_opt_` each incremental interval) is fully applied
in the working tree — Phases 1–4 complete. The `prev_optimizer_` member is
**gone** (only comment references remain); `SeedStateFromIncumbent` and
`UpdateRecords` route through the single writer `CommitIncumbent`;
`SeedIncumbentBaseline` gates on `has_incumbent_`; the incremental branch of
`EvaluateTimeLimitConfig_ScratchOrIncre` builds the challenger via
`BuildChallengerFromIncumbent()` (the `else` no-incumbent branch is now a hard
`CoutError` contract violation, was the `OptimizeFromScratch` fallback). The 6
formerly-`prev_optimizer_`-reading tests are migrated to read `res_opt_` /
`has_incumbent_` / `BuildChallengerFromIncumbent()`; the 3 frozen-baseline
DAG-ET assertions rewrite to observe the carried TL/current DAG via
`BuildChallengerFromIncumbent().dag_tasks_` + `res_opt_.id2time_limit`.
`OptimizeSP_Incre.cpp:288-297` comment rewritten to the throwaway-challenger
model.

**Verified green.** `cmake --build . --target check.SP_OPT -j5` (DEBUG): 16/16
ctest; `testIncreOpt_w_TL` = **44 tests**.

**Phase 3d re-derivation (two defaults overturned).** (1) The reopt cold-start
at `ReOptimizePeriodic:575` (`InitializeTimeLimitsFromETConfig()`) is **NOT a
bug** — the reopt path uses `OptimizeFromScratch` (not `OptimizeIncre`), so the
diff never runs on it; no invariant to preserve. This overturns `design.md` §1
symptom 3's "same class of bug on the other path" claim. (2) The incremental
start at `OptimizeIncre_w_TL:398` (`ReconstructTimeLimitVecFromResOpt()` +
stale-TL guard `:411-421`) is **KEPT, NOT folded** into
`BuildChallengerFromIncumbent` — the call site's guard-applied update-side
vector and the helper's raw baseline-reconstruction vector serve different
purposes. This overturns `design.md` §6 Q3's "fold" default.

**P1.1 probe — PASS, stronger than predicted.** Rebuilt `release/` (was stale)
and re-ran INCR_P10 N=8 taskset_0:
`release/tests/RunOrchestrator <ts> <out> INCR_P10 10000 1` (4th arg = per-
interval horizon 10000 ms, per `runorchestrator-duration-arg-semantics`). Pre-
redesign `call=0 ndiff=5` (3 false positives on perf-pair tasks 0,1,2); post-
redesign **`call=0 ndiff=0`**, and across all 302 incremental calls `ndiff` is
only ever 0 or 1. Stronger than the investigation's "5 → ~2" prediction: under
the redesign the gaussian-only baseline is rebuilt from the *current* `dag_tasks_`
each interval, so inter-interval Gaussian drift is no longer in the diff either
→ only a real adopted-TL move within the current descent flags. P0.5 alone
collapses the `ndiff` false-positive class — no YAML persistence (P0.1)
required; P0.1's remaining value is inspectability-only, as
`p05-subsumes-tl-init-bug` predicted. Trace:
`simulation_experiments/optimizer_comparison/et_repro/p05_probe_ts0_P10/`.
Full record: `agents/active_tasks/P0_5_optimizer_iteration_redesign/`. Not yet
committed (working tree).

**P0.1 — RESOLVED 2026-07-10 (subsumed by P0.5; inspectability write discarded).**
P0.1 ("persist adopted TL to YAML") was scaffolded 2026-07-08 as the root-cause
fix for the P1.1 residual, then demoted that same day to inspectability-only
once P0.5 was worked first. The functional bug it targeted is **fixed by P0.5 by
construction**: `CommitIncumbent` single-writes `res_opt_.id2time_limit`, and
`BuildChallengerFromIncumbent` rebuilds the DAG from that carried adopted TL →
both diff sides carry the adopted TL → the P1.1 `FindTaskWithDifferentEt`
false-positive class is structurally impossible (runtime-confirmed: P1.1 probe
`ndiff` 5→0). The demoted inspectability remainder — overwrite the taskset YAML
with the adopted TL (+ implied ET) after each commit, for post-run
debuggability — was **never implemented** and is **discarded** by user decision
("if it's about writing down to yaml file about the found time limits from
optimizers, we can discard it"): the orchestrator clamps job ET to
`res.id2time_limit` (`SimulationOrchestrator.cpp:461-463`), so no scheduler
decision honors the stale Gaussian once optimization has run. No code was ever
written for P0.1; the task folder is deleted. Recorded in
`finished_tasks/summary.md`.

**P0.5 — RESOLVED 2026-07-10 (closeout).** All five phases of the
incumbent-state redesign are complete and committed (`a8dba07f`→`7fa2e9d2`;
Phase-5 fixes in `8bbb0f5a`, `ae62e5e5`, `cde70138`, `7fa2e9d2`, `df3502a1`).
Re-verified green at closeout: 46 `testIncreOpt_w_TL` + 16/16 ctest (DEBUG
build). `res_opt_` is the single durable incumbent store; `CommitIncumbent` is
the single writer of `res_opt_` + `opt_pa_`; `BuildChallengerFromIncumbent`
builds the transient challenger; `IfInitialized()` (`!opt_pa_.empty()`) is the
gate (the `has_incumbent_` bool was removed as provably redundant). Phase-5
review issues all resolved one-by-one per user review: 5a `ResetIncumbentBaseline`
unified reset, 5b kept rebuild-from-champion, 5c dropped stale-TL guard, 5d
removed `has_incumbent_`, 5e renamed `starting_time_limits`, 5f removed dead
`any_eval_ran` fallback, 5g total-budget patience, 5h moved to P3.1 (perf). The
folder is moved to `finished_tasks/P0_5_optimizer_iteration_redesign/`;
resolution recorded in `finished_tasks/summary.md`; rationale in memory
`p05-subsumes-tl-init-bug.md`.

**P1.2 — task created 2026-07-11** (reopt incumbent degradation / structural
corruption). Spawned by the Gemini/Kimi multi-round debate
(`agents/agent_communication/gemini.md` R1–R5, `kimi.md` R1–R4). The debate
converged on: the SP-value compare-and-keep guard (`UpdateRecords`,
`OptimizeSP_TL_Incre.cpp:105-139`) is sound but structurally blind — it protects
SP monotonicity, not permutation structural quality, so a reopt bootstrap
(`OptimizeFromScratch`, `OptimizeSP_Incre.cpp:74`, memoryless at the shared
`K=2` from `parameters.yaml:7`) could commit a structurally-worse permutation
that squeaks past the SP guard and that the incremental `OptimizeIncre` (1D
variations) then cannot restructure. The agreed synthesis if the hazard fires =
Option B (seed the bootstrap beam with π_incumbent) + Option C (chain the TL
walk via `OptimizeIncre`, from_scratch=false after the bootstrap; collapses the
per-TL-step `OptimizeFromScratch` to once-per-reopt). **Sequencing: empirical
check FIRST** — mine the existing P25 A/B per-interval SP traces
(`runs/p25periodAB_run_test_dur600_interval10_seed1000_tasks4x6/sim/.../interval_sp_metrics.txt`)
for the post-reopt non-recovering-dip signature; no code change until the
hazard is shown to fire. Folder `agents/active_tasks/P1_2_reopt_incumbent_degradation/`.
Reads the same P25 A/B data as P1.1 but asks a different question (structural
corruption, not ET growth).

**P1.3 — `ReoptStartFromAdoptedTL` A/B regression; root cause FOUND 2026-07-11
(stale binary, not a code bug).** The first A/B with the `INCR_P<n>_ADOPTED`
arms (commit `8cbbbc12`, 2026-07-11 10:47) collapsed them far below their plain
twins. Diagnosis: the run used `release/libSP_OPT.so` + `release/tests/RunOrchestrator`
built 2026-07-08 20:27 — **stale, predates the commit**. `run_end_to_end.sh:37`
defaults `BIN_DIR=release` with no rebuild step, so the e2e run picked up the
old binary. Under the pre-commit code, `INCR_P<n>_ADOPTED` is an unrecognized
mode: `IsINCRPeriodVariant` rejects the `_` (non-digit), `MaybeOverrideReoptPeriod`
sets the period via `stoi("1_ADOPTED")==1` but the flag field doesn't exist, and
the dispatch falls through every branch → empty `ResourceOptResult` (no
priorities, no TLs) → a fixed period-independent degenerate schedule. **Decisive
fingerprint:** all four `_ADOPTED` arms (P1/P10/P30/P60) produce byte-identical
`interval_sp_metrics.txt` traces (`diff` empty) — impossible if the flag were
genuinely active (P1 reopts every interval, P60 once). Code verified sound: the
post-commit parser + dispatch + reopt branch are correct by inspection AND by
49 `testIncreOpt_w_TL` + 16/16 ctest green on the DEBUG build (`build/libSP_OPTDebug.so`,
2026-07-11 10:40, which DOES contain the code). Fix = rebuild `release/`
(verify `strings release/libSP_OPT.so | grep -ci adopted` > 0) + re-run the A/B
(user-run); no source change unless the collapse persists after a confirmed-fresh
rebuild. Folder `agents/active_tasks/P1_3_adopted_tl_regression/`.

**P1.3 — Step 2 DONE 2026-07-11 (cont.): `release/` rebuilt & verified fresh;
stale fingerprint GONE.** The user rebuilt `release/` between sessions — binary
mtimes now 2026-07-11 11:26:24 (`libSP_OPT.so`) / 11:26:39 (`RunOrchestrator`),
both after commit `8cbbbc12` (10:47); `CMAKE_BUILD_TYPE=Release`. Freshness
verified: `strings release/libSP_OPT.so | grep -ci adopted` = 3 (was 0), the
`_ADOPTED` Usage line is present, and the mangled symbol
`_ZN15GlobalVariables23ReoptStartFromAdoptedTLE` is linked into both binaries.
Functional probe (3-way `INCR_P1` / `INCR_P1_ADOPTED` / `INCR_P60_ADOPTED` + 2
controls `INCR_P60` / `INCR_SCRATCH`, on an 8-interval trimmed tasks4 taskset_0,
horizon 10000 ms): stale `INCR_P1_ADOPTED` mean SP **0.322** → fresh **0.729**;
the stale fall-through fingerprint (all 4 `_ADOPTED` arms byte-identical,
`diff -q` empty) is **absent** on the fresh binary. The 5 fresh probes are
mutually byte-identical, but that is a trivial-taskset artifact (0% miss, SP
ceiling — even `INCR_SCRATCH` matches), not a flag problem; the real
`_ADOPTED`-vs-plain signal needs the loaded full A/B. **Remaining gate =
Step 3** (user's full prod A/B re-run on the fresh binary). If the loaded A/B
shows the `_ADOPTED` arms competitive with (not collapsed below) their plain
twins, P1.3 closes as a stale-binary artifact with no source change.

**P1.3 — Step 3 DONE + CLOSED 2026-07-11.** User re-ran the A/B on the fresh
`release/` binary
(`p25periodAB_run_test_dur600_interval10_seed1000_tasks4x6x8/.../comparison_summary.csv`,
tasks4). The `_ADOPTED` arms are now distinct per-period (no longer
byte-identical) and within 0.3–0.8 SP pts of their plain twins: P1 0.579318 →
0.571043, P10 0.559728 → 0.552747, P30 0.552093 → 0.549053, P60 0.545583 →
0.545583 (byte-identical, as expected — reopt once → no divergence to
accumulate). Gap shrinks monotonically with the reopt period. Verdict: the
collapse was a stale-binary artifact; the code is sound. The incumbent
(`_ADOPTED`) seed is slightly worse than the YAML-derived (plain) seed at high
reopt frequency — which seeded P1.4.

**P1.4 — Reopt seed TL = carried incumbent, made permanent 2026-07-11
(working tree, staged, NOT committed).** User constraint: the reopt descent's
seed TL must be **algorithm-derived** (the optimizer's own prior result), not
read from the YAML taskset or the generator's drawn distribution. Analysis:
`InitializeTimeLimitsFromETConfig` (the old "off" default) is YAML-derived —
`et_dist_.GetAvgValue()` resolves through the `FiniteDist` built from YAML
`mu/sigma/min/max` (`RegularTasks.cpp:61-106`), and the generator writes `mu`
independent of the `performance_records_time` option grid
(`taskset_generator.py:244-245` / `orchestrator.py:309`). The closest
algorithmic option = `ReconstructTimeLimitVecFromResOpt` (the carried incumbent
in `res_opt_`, the optimizer's own `CommitIncumbent` write from P0.5).
**Implemented choice (a)**: flipped `GlobalVariables::ReoptStartFromAdoptedTL`
default `false`→`true` (`sources/Utils/Parameters.cpp:20`); the `IfInitialized()`
gate at `OptimizeSP_TL_Incre.cpp:474` still auto-falls-back to
`InitializeTimeLimitsFromETConfig` at interval 0 / INCR_SCRATCH (no incumbent
→ no prior to seed from; irreducible for any policy). Kept the flag as an
ablation opt-out and kept the `INCR_P<n>_ADOPTED` arms as no-op aliases (both
set a flag that now ships true) so historical run dirs stay comparable. TDD:
rewrote the 3 `ReOptimizePeriodic_*` tests at `tests/testIncreOpt_w_TL.cpp:1206+`
for the new default (`StartsFromAdoptedTLByDefault` asserts the shipped
default; `OptOutStartsFromGaussianMeanTL` covers the false ablation;
`Interval0FallsBackToGaussianMean` unchanged) — red (ASSERT_TRUE on the default
fires) → flip → green; 49/49 `testIncreOpt_w_TL` + 16/16 ctest green on the
DEBUG build. Docs updated: `Parameters.h/.cpp`, `parameters.yaml`,
`OptimizeSP_TL_Incre.cpp:469` + `.h:132`, `RunOrchestrator.cpp` (header
comment + flag-set site + `--help` Usage line), `SimulationOrchestrator.cpp:18`,
`p25_period_ab_config.json` `_comment`, memory `reopt-tl-init-adopted-arms.md`.
Accepted tradeoff: ~0.3–0.8 SP pts worse than the YAML seed at high reopt
frequency (→0 at P60), in exchange for the algorithmic-seed guarantee. Full
record: `agents/active_tasks/P1_4_reopt_seed_from_incumbent/`. **No
`release/` rebuild done by me** — the user rebuilds + re-runs the A/B to
confirm the flip on the loaded tasksets.

**P1.5 — Add INCR_Px_INIT Baselines (Default Initial Solution Seed) 2026-07-11 (planning phase).** User requested a new baseline variant `INCR_P<n>_INIT` (supporting periods P1, P10, P30) that uses the default/generator initial solution (`InitializeTimeLimitsFromETConfig`) to start coordinate descent re-optimizations, then uses warm-started incremental optimization for intermediate intervals. This allows comparing performance against the default incumbent-seeded `INCR_P<n>` variants. Created task files under `agents/active_tasks/P1_5_new_baseline_default_seed/` and updated `agents/overall_tasks.md`.

**P1.4 — Choice (b): incumbent seed made the unconditional, ONLY reopt seed; flag + `_ADOPTED` arms REMOVED 2026-07-11 (working tree, staged, NOT committed).** User redirect: "always use reopt-start-from-adopted-tl, remove the flag, clean related choice code and unused code." The initially-implemented choice (a) (keep the knob default-true as an ablation opt-out, keep `_ADOPTED` arms as no-op aliases) was replaced by (b) — the design rules ("don't make things optional if not needed" / "ruthlessly prune unused features") favor (b), and the A/B data is no longer needed since the policy is permanent. **Source:** removed `GlobalVariables::ReoptStartFromAdoptedTL` from `Parameters.h/.cpp` + the `parameters.yaml` note block; `OptimizeSP_TL_Incre.cpp:478` flag ternary → unconditional `IfInitialized() ? ReconstructTimeLimitsFromResOpt() : InitializeTimeLimitsFromETConfig()` (the `IfInitialized()` gate still auto-falls-back at interval 0 / INCR_SCRATCH — irreducible). `RunOrchestrator.cpp` `MaybeOverrideReoptPeriod`: dropped `_ADOPTED` suffix handling; a trailing suffix after the digits is now a **hard error** (loud stderr, no silent fall-through — the P1.3 trap). `SimulationOrchestrator.cpp` `IsINCRPeriodVariant`: simplified to plain `INCR_P<n>` (no trailing suffix). `p25_period_ab_config.json`: removed the 4 `_ADOPTED` arms from both modes (10→6 arms each); rewrote `_comment` + both mode `_comment`s. **Tests:** renamed `StartsFromAdoptedTLByDefault` → `StartsFromAdoptedTL` (dropped the `ASSERT_TRUE(flag)`); **deleted** `OptOutStartsFromGaussianMeanTL` (the false path is gone); stripped the flag mention from `Interval0FallsBackToGaussianMean` (logic unchanged). Net 49 → 48 tests. **Verified:** `cmake --build build --target check.SP_OPT -j5` (DEBUG) → 16/16 ctest green, `testIncreOpt_w_TL` 48/48 green. **P1.5 RETIRED 2026-07-11** (invalidated by (b)): its premise (`INCR_P<n>_INIT` sets `ReoptStartFromAdoptedTL=false` for the seed A/B) needs the removed flag; the incumbent-vs-YAML-seed A/B is no longer runnable as a config arm, and P1.3's measured tradeoff (~0.3–0.8 SP pts, →0 at P60) stands as the final record. Folder `active_tasks/P1_5_new_baseline_default_seed/` deleted. Full record: `agents/active_tasks/P1_4_reopt_seed_from_incumbent/`. **No `release/` rebuild done by me** — the user rebuilds + re-runs the 6-arm A/B to confirm on the loaded tasksets.

**P1.6 — Pure-incremental baseline (RM-fast bootstrap, no periodic reopt) 2026-07-11 (PLANNING ONLY — no implementation).** User directive: "add a new active task ... where we mainly use incremental optimization to perform per-interval optimization, and we start the RM-Fast rather than re-optimization in the first interval, and never call re-optimization periodically. only add this active task without implementation. purpose is to compare performance between incr and scratch." A new A/B arm: pure incremental — `OptimizeIncre_w_TL` (warm-started from the carried incumbent) every interval, **never** `ReOptimizePeriodic`, with the **first interval bootstrapped from RM-fast** (RM priorities via `RateMonotonicPriorityVec` + smallest TL via `SmallestTimeLimitVec` = `timePerformancePairs[0]` = the orchestrator's `RM_FAST` TL, `SimulationOrchestrator.cpp:380-383`) instead of a from-scratch reopt. The interval-0 RM-fast seed already exists — `ResetIncumbentBaseline(true)`'s `else` branch (`OptimizeSP_TL_Incre.cpp:437`) seeds + `CommitIncumbent`s the RM-fast incumbent; the arm = run that seed step and **skip** the `PerformCoordinateDescentForTaskConfigOpt(..., from_scratch=true)` descent that `ReOptimizePeriodic` (`:453`) runs afterwards, then pure incremental. So it is NOT just `ReoptimizationPeriod=∞` (a large period still routes `count==0` → `ReOptimizePeriodic` and descends at interval 0); the arm additionally skips that interval-0 descent. Purpose: a clean incr-vs-scratch A/B — vs `INCR_SCRATCH` (both bootstrap RM-fast; the new arm carries the incumbent via warm-start, `INCR_SCRATCH` discards it) isolates the value of carrying the incumbent; vs `INCR` (both carry the incumbent; `INCR` pays for periodic from-scratch reopt, the new arm doesn't) isolates whether periodic reopt earns its cost. Also de-risks P1.2: with no `ReOptimizePeriodic` ever, the memoryless from-scratch `OptimizeFromScratch` (the suspected structural-corruption vector) never runs, so the incumbent evolves only via the 1-D `OptimizeIncre` walk — degradation seen in `INCR` but absent here points at the reopt path. Inherits P1.4's incumbent seed (`ReconstructTimeLimitVecFromResOpt`) unchanged; the interval-0 RM-fast seed is `IfInitialized()`-gated and irreducible (no prior exists at interval 0). **No source code touched; no `git add`.** Folder `agents/active_tasks/P1_6_incr_only_baseline/` (`goal.md` + `tasks.md` + `dev_log.md`); 5 open design decisions flagged for the user (mode-string name, branch-vs-flag, interval-0 mechanics — likely a new `BootstrapIncumbentFromRMFast()` method, config placement, E3-gating). Next action is the user's: greenlight implementation + settle the design decisions.

**P1.6 — follow-up 2026-07-11 (PLANS ONLY — still no implementation).** User: "we should implement it similar to INCR_sCRATCH, it'll likely be a baseline to add to paper. only update plans without implementation. also, difference between INCR-P1 and INCR-SCRATCH??" **Answered INCR_P1-vs-INCR_SCRATCH:** both run the from-scratch descent `ReOptimizePeriodic` every interval — that is NOT the difference; the difference is optimizer lifetime / incumbent-carrying. `INCR_P1` uses the **persistent** `incr_optimizer_` (`SimulationOrchestrator.cpp:296`) → `res_opt_` survives → `ResetIncumbentBaseline(true)` takes the `if (IfInitialized())` branch (`OptimizeSP_TL_Incre.cpp:427-436`) at interval 1+ → compare-and-keep measures against the **running best**. `INCR_SCRATCH` builds a **fresh** `scratch_opt` each interval (`SimulationOrchestrator.cpp:334`) and discards it → `res_opt_` empty → takes the `else` branch (`:437-447`) every interval → compare-and-keep measures against a **synthetic RM baseline**. Net: `INCR_P1` weakly dominates `INCR_SCRATCH` in SP; `INCR_SCRATCH` is the amnesiac (no-memory) control. (In-source comment at `:328-333` states this but still names `prev_optimizer_`, the member P0.5 removed — stale wording, not stale logic; it now runs through `res_opt_`.) **Settled D2+D5** per the user's directive: D2 → **(a) new `scheduler_mode_` branch** mirroring `INCR_SCRATCH`'s dispatch shape (NOT a `GlobalVariables` flag); D5 → **paper-grade baseline, A/B probe NOT an E3 gate** (E3 stays on plain `INCR`). **Flagged the persistent-vs-fresh nuance** (the one subtlety in "similar to INCR_SCRATCH"): the new arm mirrors `INCR_SCRATCH`'s dispatch *shape* but MUST use the persistent `incr_optimizer_` (like `INCR`) to carry the incumbent — NOT the fresh-`scratch_opt`-each-interval lifetime (that is `INCR_SCRATCH`'s amnesia, the exact thing the new arm contrasts). Updated `goal.md` (added the INCR_P1-vs-INCR_SCRATCH subsection + a three-way contrast table; rewrote the reasoning to name `INCR_P1` as the reopt sibling; marked D2+D5 SETTLED; added the persistent-optimizer clarification to Done-when + "What the arm is NOT"), `tasks.md` (Step 0 shows D2+D5 checked; Steps 1/2 + Standing constraints carry the persistent-optimizer nuance), `overall_tasks.md` (P1.6 row + suggested-order entry), and this log. **No source code touched; no `git add`.** D1 (mode name), D3 (interval-0 mechanics), D4 (config placement) remain open for the user.

**P2.4 — Optimizer methods & mode-string refactor (naming/clarity, NOT a computation bug) 2026-07-11 (FILED ONLY — no implementation).** User directive: "add a new optimizer methods refactor task, we'l fix the optimzier names and implementation details for issues like what you just point out." **Trigger:** the `INCR_P1` inversion surfaced answering "does INCR_P1 always call incremental rather than re-optimization?" — the answer is the *opposite* of the name: `INCR_P1` parses to `ReoptimizationPeriod=1` (`tests/RunOrchestrator.cpp:65`), so `count % 1 == 0` is always true (`OptimizeSP_TL_Incre.cpp:298-304`) → reopt **every interval**, incremental **never**; the `P<n>` knob runs the wrong way for a reader (larger n = more incremental, less reopt), so `INCR_P1` is the max-reopt extreme, not the least. That exposed a cluster of naming/clarity issues, collected into one task. **Issue catalogue (verified against current code, not memory):** I1 the `INCR_P1` inversion; I2 plain `INCR` vs `INCR_P1` silently coupled to the YAML `ReoptimizationPeriod` knob (`SimulationOrchestrator.cpp:293,313` dispatch `INCR` identically to `INCR_P<n>` but don't override the period → `INCR`'s behavior depends on `Parameters.cpp:20`, a value the mode string doesn't surface; the eval-suite config lists `INCR` and `INCR_P1` as separate arms but they coincide only when YAML sets period=1); I3 stale `prev_optimizer_` comments (P0.5 removed the member; the word survives at `SimulationOrchestrator.cpp:329` + `testIncreOpt_w_TL.cpp:442,1087` — stale wording, correct logic); I4 stale `ReoptStartFromAdoptedTL` history comments (P1.4 removed the flag; survives as history at `SimulationOrchestrator.cpp:28`, `RunOrchestrator.cpp:19`, `testIncreOpt_w_TL.cpp:1204`); I5 method-name/role misalignment in `OptimizePA_Incre_with_TimeLimits` (`ReOptimizePeriodic` hides its bootstrap role; `_w_TL` suffixes redundant; `from_scratch` bool conflates "reopt" with "from scratch"). **Sliced** A (stale-comment rewrite — free, zero behavior risk) / B (mode-string rename, the trigger — recommend `REOPT_P<n>` so the family reads `REOPT_P1`…`REOPT_P60` vs P1.6's `INCR_PURE` vs `INCR_SCRATCH`; touches C++ parsing + 3 configs + ~11 python files; stale old names must FAIL LOUDLY, the P1.4 `_ADOPTED` pattern) / C (method renames — widest blast radius, defers if it conflicts with P1.6/P1.2's in-flight edits to `OptimizeSP_TL_Incre`). **Central risk:** not the rename itself but keeping the C++↔config↔python string surface in sync. **Renames are behavior-preserving** — no test *expectation* changes, only symbol names; if a rename would change a result, that's a different (P1/P0) task. Folder `agents/active_tasks/P2_4_optimizer_methods_refactor/` (`goal.md` + `tasks.md` + `dev_log.md`); updated `overall_tasks.md` (P2 table row + suggested-order entry). 4 open design decisions for the user (scope D1 = which slices ship — recommend A+B, defer C; D2 = rename direction — recommend `REOPT_P<n>`; D3 = bare `INCR` — recommend drop from A/B configs; D4 = method renames if C greenlit). **No source code touched; no `git add`.** Next action is the user's: pick the slice(s) + rename direction, then greenlight.

**P2.4 — Slices A+B + 1-arg overload test migration COMPLETE 2026-07-11 (16/16 ctest DEBUG + 291 python green; staged, NOT committed).** Resumed after a context-limit break. Settled scope: ship Slice A (stale-comment rewrite) + Slice B (mode-string rename) + the 1-arg `ReOptimizePeriodic(int K)` overload test-site migration; defer Slice C (I5 method renames — conflicts with P1.6/P1.2 in-flight edits); file `INCR_SCRATCH` removal as Step 5 (SEPARATE result-changing sub-task, NOT this pass). **Rename direction settled `INCR_P<n>`→`INCR_Reopt_X`** (X∈{1,5,10,30,60}; `Reopt` matches codebase vocabulary `ReOptimizePeriodic`/`ReoptimizationPeriod`; X=5 is a NEW arm, the existing family was {1,10,30,60}); rejected `REOPT_P<n>` (inverts the `INCR_` family prefix the user wants). Old `INCR_P<n>` FAILS LOUDLY in `MaybeOverrideReoptPeriod` (mirror of the P1.4 `_ADOPTED` hard-error pattern — the P1.3 trap is now structurally impossible: any trailing suffix after the digits is a hard stderr error, no silent fall-through). Bare `INCR` stays canonical (= `INCR_Reopt_10` via `parameters.yaml:29 ReoptimizationPeriod: 10`); I2 resolved by requiring explicit `INCR_Reopt_X` for any arm that sweeps the period (A/B configs use explicit `INCR_Reopt_X`, never bare `INCR`). **Slice A:** stale `prev_optimizer_` comment rewritten at `SimulationOrchestrator.cpp:329` + `testIncreOpt_w_TL.cpp:442` to name `res_opt_`/`IfInitialized()` (P0.5 incumbent-state design); `:1087`+`:1204` confirmed INTENTIONAL historical contrasts (left as-is); `ReoptStartFromAdoptedTL` history comments tightened at `SimulationOrchestrator.cpp:28`, `RunOrchestrator.cpp:19`, `testIncreOpt_w_TL.cpp:1204` (carried into the renamed `INCR_Reopt_X` prose). **Slice B:** `MaybeOverrideReoptPeriod` (`RunOrchestrator.cpp`) parses the new `INCR_Reopt_` prefix + hard-errors the old `INCR_P<n>`; `IsINCRPeriodVariant` + construction + dispatch conditions (`SimulationOrchestrator.cpp`) match the new prefix; both configs (`evaluation_suite_config.json`, `p25_period_ab_config.json`) renamed every arm + ADDED `INCR_Reopt_5`; python (`evaluation_suite.py` `DEFAULT_PERIOD_ARMS` + docstrings, `test_evaluation_suite.py` E3 fixtures, `repro_et_grows_with_period.py` `ARMS`+`--arms`+docstring). **Overload migration (Step 3):** the ~11 test/example 1-arg `ReOptimizePeriodic(K)` call sites migrated to the 2-arg `ReOptimizePeriodic(dag_tasks, K)` form; the 1-arg overload declaration/def STAYS (decision (a)) — its only remaining caller is the INCR_SCRATCH branch, which Step 5 removes together with the overload. **TDD:** the python E3-gate fixtures were the test surface (the C++ `MaybeOverrideReoptPeriod` is a `static` fn in the `RunOrchestrator` main TU, not a gtest harness — fail-loudly verified by code inspection + the gate-level python TDD); 6 RED (gate expected `INCR_Reopt_X` keys, fixtures used `INCR_P<n>` → "missing") → 30 GREEN after migrating the fixtures (added the `INCR_Reopt_5` column everywhere — monotonic non-increasing ET so PASS cases stay PASS). **Verified:** `cmake --build build --target check.SP_OPT -j5` (DEBUG; `libSP_OPTDebug.so`) → 16/16 ctest green incl. `testIncreOpt_w_TL`; `tests/python/` → 291 passed. `grep -rn "INCR_P" --include=*.py --include=*.json --include=*.cpp --include=*.h` (excl. `INCR_PURE`) → only the intentional `evaluation_suite.py:79` history comment remains (rest are stale `et_repro_result.json` run artifacts, not part of the rename surface). No test *expectation* changed — only symbol names + the new `INCR_Reopt_5` arm + renamed fixture keys. **Standing constraints honored:** behavior-preserving (Slices A+B + overload migration); no `git commit` (`git add` only); the user runs the A/B suite. Slice C (I5 method renames) stays DEFERRED; Step 5 (`INCR_SCRATCH` removal) is result-changing and stays a separate sub-task — NOT staged here. Full record: `agents/active_tasks/P2_4_optimizer_methods_refactor/`.

**P2.5 — Remove the `INCR_SCRATCH` scheduler arm COMPLETE 2026-07-11 (16/16 ctest DEBUG + 287 python green; staged, NOT committed).** Resumed after a context-limit break (the prior session had done the C++ removal + most of the python/config surface but stalled before the doc/index/index pass). This is the result-changing cleanup promoted out of P2.4 Step 5: it deletes a scheduler arm and collapses the north-star from 6 gates to 5. **Surface removed:** (1) the `INCR_SCRATCH` dispatch branch + its construction-condition string in `SimulationOrchestrator.cpp`; (2) the 1-arg `ReOptimizePeriodic(int K)` overload in `OptimizeSP_TL_Incre.{h,cpp}` — its last caller was the SCRATCH branch, so with the branch gone it is dead (P2.4's `:428` test-site migration to the 2-arg `ReOptimizePeriodic(dag_tasks, K)` form was finished first so the build never breaks); (3) the E2 gate (`evaluate_e2`) deleted outright from `evaluation_suite.py` — its only subject pair was INCR-vs-SCRATCH, so it has no referent once SCRATCH is gone; (4) Q1/Q2/Q3/E1 narrowed from INCR-and-SCRATCH to INCR-only; (5) `INCR_SCRATCH` dropped from every config arm-list (`evaluation_suite_config.json`, `p25_period_ab_config.json`, `experiment_config.json`) + every python non-gate list (`compare_optimizers.py`, `aggregate_across_tasks.py`, `repro_et_grows_with_period.py`, the two `debug_analysis` scripts); (6) the dead `INCR_SCRATCH` fixture rows dropped from `test_evaluation_suite.py`'s `test_main_pass_exit_zero` synthetic run (the gate code was already done in the prior session, so the suite was green with the dead rows — they were residue, not a test surface). **What survives:** the amnesiac-reopt *mechanism* (the `IfInitialized()` interval-0 Gaussian-mean fallback in `ReOptimizePeriodic`) — it is exercised by ANY fresh-optimizer reopt, covered by `ReOptimizePeriodic_Interval0FallsBackToGaussianMean` (which uses a fresh local stub, NOT the SCRATCH dispatch path); only the mode string + the dispatch branch + the dead E2 gate went away. **Fail-loud contract:** a stale `INCR_SCRATCH` config now FAILS LOUDLY — the dispatch falls through every branch to the RM baseline (the P1.3 trap pattern), NOT a silent alias to `INCR_Reopt_1`. **Docs:** north-star (`project_evaluation_northstar.md`) collapsed to the 5-gate INCR-only form (deleted the "INCR cannot run slower than SCRATCH" E2 line + the SCRATCH mentions in the SP-quality bullets); `plan_publication_figures.md` dropped SCRATCH from the Ab-A/Ab-B ablation lists; P0.3 figure goal/tasks dropped the SCRATCH floor reference; P2.4 Step 5 marked DONE with a pointer here; `overall_tasks.md` got a P2.5 row + the P2.4 row + P1.6 suggested-order entry updated. **P1.6 premise now obsolete** — its central "incr-vs-scratch" contrast (the user's stated purpose for P1.6) is gone; the surviving axis is vs `INCR_Reopt_1` (the always-reopt-with-memory arm), and re-framing P1.6 is deferred to the user (it is PLANNING-ONLY; a find-replace would mis-state its new purpose). **Verified:** `cmake --build build --target check.SP_OPT -j5` (DEBUG; `libSP_OPTDebug.so`) → 16/16 ctest green; `tests/python/` → 287 passed. `grep -rn "INCR_SCRATCH"` (excl. history docs + `et_repro_result.json` run artifacts) → only intentional history mentions remain. **Standing constraints honored:** no `git commit` (`git add` only); result-changing by design (its own review cycle); history docs (`finished_tasks/`, `investigation/`, `agent_communication/`, `claude_sessions/`) NOT rewritten — they are the record of when SCRATCH existed. Full record: `agents/active_tasks/P2_5_incr_scratch_removal/`.


**P1.7 — Simulator `processorId` partitioning fix RESOLVED 2026-07-12 (staged on `clean_simulation`, NOT committed; 16/16 ctest DEBUG green; user runs the A/B re-run).** Resumed from the 2026-07-12 evaluation (which confirmed the finding but did not implement). User greenlit the fix and locked D2 (two per-`processorId` `RunQueue`s, lockstep, merged schedule — mirror the legacy `SimulatedFTP_SingleCore`/`SimulatedCFS_SingleCore`), D3 (fix FTP + CFS in one pass), D4 (re-run on the fixed sim, discard old 1-core miss-rate numbers); D1 (generator `per_core_cpu_util` re-calibration) + D5 (sensitivity-narrative re-frame) deferred. **Fix:** both `FixedTaskPrioritySchedulingOrchestrator::SimulateInterval` and `CFSSimulationOrchestrator::SimulateInterval` (`SimulationOrchestrator.cpp`) now call `GetProcessorIds(dag_tasks)` and build one `RunQueue` (held in `std::unique_ptr<RunQueue>` — `RunQueue` is non-assignable due to its `const TaskSetInfoDerived` member) per distinct `processorId`, stepping all queues in lockstep (Remove/Record/Release/Run per queue per tick). `ReleaseJobs`/`ReleaseJobsCFS` gained an `int processor_id = -1` param (`-1` = release all, preserving the unit-test call sites); when `>= 0`, tasks whose `processorId` doesn't match are skipped. `RecordFinishedJobs`/`RecordFinishedJobsCFS` unchanged (take `RunQueue&`, read only that queue's `schedule_`, merge into the shared `job_history_`; a job lives in exactly one queue so the `find_if` dedup is a no-op across queues). `ScheduleSimulation.h` got the missing `GetProcessorIds` declaration (was defined in the `.cpp` but never declared in any header). **TDD:** 2 new red→green tests in `tests/testScheduleSimulate.cpp` (`SimulateIntervalPartitionsByProcessorId` + `_CFS`) on new input dir `tests/test_data_partition_two_cores/` (2 tasks, equal period 10 / ET 2, different `processorId`: 0 and 1; both assert job-0 of each task starts at `t=0` → parallel; under the old single queue Task1.start would be `2`). **Verified:** `cmake --build build --target check.SP_OPT -j5` (DEBUG; `libSP_OPTDebug.so`) → 16/16 ctest green; `testScheduleSimulate` 34/34 (incl. the 2 new tests); single-core regression guards stayed green (one-core tasksets byte-identical — all tasks land in the same queue). **PIVOTAL BLAST-RADIUS CORRECTION (overturns the original eval):** while wiring the fix, traced where the SP metric the eval-suite gates read actually comes from — `SimulateInterval` → `ObtainSP_TaskSet_And_TimeLimits` (`SP_Metric.cpp:82`) → `ObtainSP_TaskSet` (line 53) → `ProbabilisticRTA_TaskSet` (`RTA.cpp:100`), which **already partitions on `processorId`** via `ExtractTaskSetPerProcessor` (`RTA.cpp:87`) and runs `ProbabilisticRTA_TaskSet_SingleCore` per partition. So `mean_sp_norm` / `Mean_SP_Metric` (what every gate Q1/Q2/Q3/E1/E3 reads) was **always on the correct path**, and **no gate verdict moves** after the fix. The single-queue bug only distorted the `job_history_`-derived exports — miss-rate/response-time columns in `comparison_summary.csv` (read by `aggregate_across_tasks.py` + `utils.py`, NOT by the gates). E3 (period-monotonicity, `mean_sp_norm`) was **unrelated to the bug** — it PASSES on the analytic partitioned path regardless. D5 is **moot for SP** — SP's priority sensitivity is the intended partitioned-RTA behavior, not an overload artifact; the overload-artifact framing applies only to the miss-rate exports. **Residual follow-up filed as P2.6** (`agents/active_tasks/P2_6_sim_rt_based_sp_metric/`, planning only): the SP metric still comes from the analytic RTA, not from the RunQueue's actual simulated RTs; the user wants the simulation to eventually report BOTH the analytical SP and the true SP derived from the simulated `job_history_` RT samples. This P1.7 fix (correct per-`processorId` RunQueues producing honest RT samples) is P2.6's prerequisite. **Standing constraints honored:** no `git commit` (staged via `git add` only); no A/B run by me (user runs `run_end_to_end.sh`); TDD (tests first, red before green). Folder moved to `finished_tasks/`; `finished_tasks/summary.md` + `overall_tasks.md` (P1.7 struck through + P2.6 row added) updated; memory `cpu-partition-mismatch.md` corrected (the "deferred P1 task" wording → P2.6, priority P2). Full record: `agents/finished_tasks/P1_7_cpu_partition_mismatch/`.

**P2.6 — Sim-RT-based SP metric FILED 2026-07-12 (planning only; no implementation).** User directive: "create a new task to add support from schedule simulation to report both the analytical SP and the true SP from tasks' true RTAs. put it as a P2 task that we'll work on later." The SP metric the eval-suite gates read is **analytic** (`ObtainSP_TaskSet_And_TimeLimits` → `ProbabilisticRTA_TaskSet`, `RTA.cpp:100`, already partitioned per `processorId`); the simulation's per-`processorId` `RunQueue`s produce actual per-job RT samples (in `job_history_`, via `RecordFinishedJobs`/`RecordFinishedJobsCFS`) that SP currently ignores. **Goal:** make the simulation report BOTH — keep the analytical SP as the gate-facing metric (so no gate verdict moves) AND add a true SP derived from the simulated per-job RT samples (a new `Mean_SP_Metric_Sim` / `mean_sp_norm_sim` column in `comparison_summary.csv`). Purpose: cross-check the analytic RTA against the actual schedule; quantify the analytic-vs-sim gap; unblock a future decision on whether SP should come from the schedule rather than the RTA. **Prerequisite DONE** — P1.7 (per-`processorId` RunQueues → honest RT samples; without it the sim RTs were the distorted single-core-overload numbers). **Deferred for later** (P2: features/analysis, not correctness — the analytic SP is sound). Open design questions: (D1) how to turn a sample of per-job RTs into an SP value — empirical DDL-miss fraction per task over the run (a `FiniteDist` / histogram) fed into the same `SP_Func` + `weights_node`/`thresholds_node` path, vs. a simpler empirical miss-rate-weighted sum; (D2) chain/path SP (the analytic `ObtainSP_DAG` also sums `GetRTDA_Dist_AllChains` path-latency terms — does the sim produce path-latency samples, or is node-level SP enough for the cross-check?); (D3) where the column lands (C++ `ExportResults` writes the per-run CSV; `aggregate_across_tasks.py` + `utils.py` read it — mirror the existing `Mean_SP_Metric` plumbing); (D4) whether any gate eventually switches to the sim SP, or it stays a reported-but-not-gated cross-check (default: cross-check only). Folder `agents/active_tasks/P2_6_sim_rt_based_sp_metric/` (`goal.md` + `tasks.md` + `dev_log.md`); `overall_tasks.md` got a P2.6 row + a suggested-order entry. **No source code touched; no `git add`.** Next action is the user's: greenlight + settle D1–D4 when P2 work resumes.

**P1.8 — INCR_WCET outperforms INCR in the eval-suite run FILED + read-only investigation STARTED 2026-07-12 (NO implementation).** User directive: "INCR_WCET outperforms other INCR tasks, this is theoretically highly unlikely, need to investigate why and fix this issue. first add the task, then share investigation while working on finding reasons and root causes." In the N=4/dur=600s/interval=10s/seed=1000 eval-suite run, `INCR_WCET` Mean_SP_Metric=**0.5992** beats every plain INCR arm (0.5456–0.5710, incl. `INCR_Reopt_1`=0.5710) at a *lower* miss rate (0.1115 vs 0.1955) and lower important-miss rate (0.2817 vs 0.4300); sits just below BF (0.6129). Theoretically implausible: `INCR_WCET` is a *degraded* ablation — dispatch (`SimulationOrchestrator.cpp:332-339`) sets `use_wcet_execution_time=true` for the optimizer call, and `ApplyWCETAblationIfRequired` (`OptimizeSP_TL_Incre.cpp:482-492`) collapses every task's ET dist to a WCET point mass (`GetUnitExecutionTimeDist(max_et)`, `Probability.h:173`) — explicitly a Q3 baseline the gate expects INCR to beat (`evaluation_suite.py:94` `Q3_BASELINES`). **Preliminary root cause (hypothesis, NOT yet confirmed end-to-end):** the aggregate flip is dominated by ONE catastrophic taskset. Per-taskset `sp_metrics_summary.txt` shows on **8/10 tasksets INCR ≥ INCR_WCET or ties** (theory holds); **taskset_1** alone flips it — INCR SP collapses to **0.0133** while INCR_WCET holds at 0.383 (taskset_6/8 add smaller INCR_WCET wins). So the anomaly is *not* "INCR_WCET is mysteriously great" — it is **"INCR catastrophically fails on taskset_1 while the degraded ablation survives."** The failure is in the **analytic** SP, not the actual schedule: taskset_1 misses **zero** deadlines under both arms (`miss_rate_summary.txt` = `47470,0,0` / `45079,0,0`), yet INCR's `interval_sp_metrics.txt` is literal **0 from i52 through i59** (INCR_WCET steady ~0.38–0.40). taskset_1 contains **inherently-unschedulable tasks** (WCET > deadline): task0 `period=33 deadline=18 ET_max=27.22`, task2 `period=20 deadline=14 ET_max=21.40`. `ApplyTimeLimitsToTasksExecutionTime` (`SP_Metric.cpp:70-80`) makes the scored ET = the adopted TL (`GetUnitExecutionTimeDist(time_limits[i])`), so TL ≥ deadline → deterministic miss → `GetDDL_MissProbability` ≈ 1 → `SP_Func(1, threshold)` = 0 → SP contribution 0 (+ interference cascade). **Leading hypothesis:** INCR adopts a TL ≥ deadline on the unschedulable task and gets stuck in a flat-zero basin (compare-and-keep can't escape — every neighbor also scores ~0); INCR_WCET's pre-search ET collapse changes the `timePerformancePairs` grid / perf-coefficient so it adopts a TL < deadline. Sub-hypotheses to disambiguate: H1a (TL-grid differs) / H1b (SP_Func + perf-coef shape differs under the degenerate dist) / H1c (incumbent corruption — the P1.1/P1.2 family; INCR commits a bad permutation early, carried incumbent can't recover) / H1d (generator defect — `per_core_cpu_util` produced an unschedulable taskset; P1.7 touched this calibration). Not mutually exclusive: H1d may be the underlying defect, H1a/H1b the reason the WCET ablation masks it. **Fix candidates F1–F4** (generator / optimizer TL ≥ deadline hard constraint / incumbent escape / scoring honesty — direction of fix FLIPS on whether INCR's ≈0 is the bug or INCR_WCET's 0.383 is the bug masking infeasibility). Folder `agents/active_tasks/P1_8_incr_wcet_outperforms_incr/` (`goal.md` + `tasks.md` + `dev_log.md`); `overall_tasks.md` got a P1.8 row + a suggested-order entry. **No source code touched; no `git add`.** Standing constraints: no implementation until root cause confirmed + user greenlights a fix; settle D1–D4 with the user before coding (per `agent_coding_rules.md`); no `git commit`; no A/B run by me. Next actions (read-only Step 0): dump INCR's adopted TLs + priorities on taskset_1 interval-by-interval; dump the `timePerformancePairs` grid INCR vs INCR_WCET sees; compute `ObtainSP_TaskSet_And_TimeLimits` by hand under INCR's TLs vs a TL < deadline; check whether Q3's verdict actually moved on this N=4 run (Q3 is scoped to N=8/10).

**P1.8 — F1 feasibility clamp IMPLEMENTED + TDD-VERIFIED 2026-07-12 (staged on `clean_simulation`, NOT committed; user runs the A/B re-run).** Resumed from the 2026-07-12 read-only investigation (which confirmed H1d — systemic generator defect, 9/10 tasksets have a WCET>deadline task — as the substrate, with H1c priority-assignment as the collapse mechanism on top). User greenlit the simpler post-generation clamp design (verbatim: "after task set and all intervals' yaml files are generated, we go through all of them once. for each task whose avg ET exceeds 0.95*period, we just clamp avg ET back to 0.95*period. if that happens, we also re-generate deadline as period. otherwise, nothing changes. first evaluate whether this makes sense to you, record it into tasks list, then implement"). **TDD red first:** wrote `Gen_Taskset/tests/test_feasibility_clamp.py` (7 tests) before the module existed; confirmed RED (`ModuleNotFoundError` at collection); cases cover deterministic-fire / perf-skip / clean-untouched / mixed / never-raise-max / walks-all-characteristics-files-and-skips-param / preserves-perf-records-string-format. **Module:** new `Gen_Taskset/lib/feasibility_clamp.py` → `clamp_avg_et_to_period(yaml_dir, et_over_period_cap=0.95)`. Pure filesystem transform: `glob` every `taskset_characteristics*.yaml`, `safe_load`, per-task `_clamp_task` (perf-gated via `bool(performance_records_time)`; fires only when `mu > cap*period`; clamps `mu`→`cap*period`, `max`→`min(max,cap*period)` (never raises), `min`→`min(min,new_max)`, `deadline`→`period`), rewrite via `export_taskset_to_yaml` (same `SpaceSeparatedListDumper` → byte-compatible; perf-record strings stay space-separated for `.as<std::string>()`). Returns `{files_written, tasks_clamped}`. **Wired** as step 4 in `orchestrator.run_full_generation_pipeline`, AFTER `generate_additional_execution_traces` returns (all characteristics YAMLs on disk), BEFORE the function returns. **KEY INSIGHT in the module docstring (so it can't be "simplified" back):** clamping `mu` ALONE is a NO-OP on the scored SP — `FiniteDist` (`Probability.cpp:18-43`) truncates the Gaussian at `execution_time_max` and dumps all upper-tail mass on the `max` bin, so the scored support upper bound is `max`, NOT `mu`; the max/min clamp is mandatory (esp. for the deterministic case `min==max==mu`, where moving only `mu` is a complete no-op because the point mass sits at `max`). Perf tasks skipped because their min/max are the optimizer's TL-option grid bounds (`period * FINAL_Et_OVER_PERIOD_RANGE`), not ET-support bounds. `taskset_param.yaml` NOT touched (C++ scorer reads only the characteristics YAMLs). **Verified:** 7/7 new tests GREEN; `Gen_Taskset/tests/` 21/21 green; `tests/python/` 291/291 green (no downstream regressions — eval-suite, aggregator, taskset-generator tests all unaffected). Added the clamp invariant (`mu <= 0.95*period` AND `max <= 0.95*period` for non-perf tasks) as a regression assertion in `test_integration_pipeline`; passes against a real pipeline run. **Empirical re-scan of the N=4 run (read-only, on a copy):** the clamp fires on EXACTLY 1 task across all 10 tasksets at interval_0 — taskset_1 gid=2 task_3 (`mu=21.404 > 0.95*20=19.0`, `min==max==mu==21.4`); the other 39 tasks are already feasible. Matches the design's surgical prediction. Clamp-on-copy of the real taskset_1 `taskset_characteristics_interval_0.yaml` confirms post-state: gid=2 `mu=min=max=19.0, deadline=20` (the user's flagged `mu>period` substrate repaired); gid=0 (`mu=27.2 < 0.95*33=31.35`) and gid=1 (`mu=128.9 < 0.95*500=475`) untouched (mu under cap); gid=3 skipped (perf). **Python-only change** (no C++ touched → no ctest rebuild needed). **Standing constraints honored:** TDD (red test first); no `git commit` (`git add` only, user reviews); no A/B run by me (user runs `run_end_to_end.sh` per D4); modular — standalone module wired at one call site. **Remains (user-facing):** user re-runs the A/B to confirm INCR ≥ INCR_WCET on taskset_1 + the aggregate, and checks the Q3 verdict. Step 2 (a C++ test reproducing the taskset_1 collapse on the pre-clamp code) is now MOOT for F1 — the substrate is removed at generation time, so the C++ optimizer never faces the unschedulable taskset; `test_feasibility_clamp.py` IS the generator-side regression guard. The Step 0 sub-items dumping `priority_vec`/`rtas[i]`/TL grids for the H1c mechanism on top of the H1d substrate are now ACADEMIC — F1 removed the substrate. Full record: `agents/active_tasks/P1_8_incr_wcet_outperforms_incr/` (tasks.md Step 1b all checked; dev_log.md appended).

**P1.9 step 3c LANDED 2026-07-14 (staged, NOT committed).** Incremental RTA Patching — the cache API surface. Created `sources/Safety_Performance_Metric/RTA_Cache.{h,cpp}` (leaf pair, breaks the `PriorityVec` header cycle), relocated `PerCoreRTACache` + `ComputeRTA_FullAndCache` there, restored the locked 4-arg sig `ComputeRTA_FullAndCache(dag, pa_vec, tl_vec, cache)` (applies pa + tl internally — option 1), added the `tl_vec` ET-dist-validity field + the read-only `AnalyzeCacheReuse`/`CacheConsistentWith` reuse-query API, and added the two header decls (`ApplyTimeLimitsToTasksExecutionTime`→`SP_Metric.h`, `ExtractTaskSetPerProcessor`→`RTA.h`). Patchers declared (locked sigs), defined in steps 4/5. 8 TDD tests (differential vs the oracle `ProbabilisticRTA_TaskSet` + self-consistency incl. `tl_vec` alignment + a TL differential + 4 `AnalyzeCacheReuse` scenarios: identity/partial-prefix+cross-core-untouched/priority-move/partition-change + `CacheConsistentWith`). One RED caught + fixed (the partition-change test wrongly asserted `common_prefix_length==0`; the impl's correct value is 2 when the migration lands in the suffix — the raw prefix is only actionable under `same_core_partition==true`). `cmake --build build --target check.SP_OPT -j5` → 16/16 ctest green (DEBUG, `libSP_OPTDebug.so`); `testRTA` 19 PASS. `ProbabilisticRTA_TaskSet`/`_SingleCore` unchanged — still the live path + differential oracle (retired at step 3b). `git add` only — awaits user review + commit. Next: step 3b (wire `ComputeRTA_FullAndCache` into the eval path + cache member in `OptimizePA_Incre_with_TimeLimits`). Full record: `agents/active_tasks/P1_9_incremental_rta_patching/dev_log.md` §2026-07-14 (step 3c).

**P1.9 step 3b SCOPE DECIDED + call-chain map — DOCS-ONLY 2026-07-14 (no code, no `git add`).** Resumed P1.9; confirmed step 3c still staged-not-committed (HEAD `0c1f846b`, cache not yet committed by the user). Before coding 3b, surfaced the wiring fork the existing docs left open: the cache lives on `OptimizePA_Incre_with_TimeLimits` (the TL-walk owner, decision 3), but the per-candidate RTA runs TWO LAYERS DOWN inside a THROWAWAY `OptimizePA_Incre` challenger rebuilt every candidate (P0.5 `BuildChallengerFromIncumbent`), via the shared free fn `EvaluateSPWithPriorityVec` → `ObtainSP_DAG` → `ObtainSP_TaskSet` → `ProbabilisticRTA_TaskSet`. User picked **BASELINE-ONLY**: wire the cache into the incremental optimizer's eval path, retire `ProbabilisticRTA_TaskSet` from THAT path only (leave `OptimizePA_BF`/`OptimizeSP_TL_BF` on the oracle), no patching yet (cache built per candidate; reuse is steps 4/5). Traced the chain end-to-end against source and pinned 4 load-bearing facts: (1) `UpdateExtDistBasedOnTimeLimit` (OptimizeSP_TL_BF.cpp:6) IS `ApplyTimeLimitsToTasksExecutionTime` (SP_Metric.cpp:70) — TLs baked upstream, pa_vec applied at the eval fn, so at the RTA plug-in point tl_vec is gone (validates 3c's option-1 re-apply-both-internally); (2) cache can't live on the challenger (rebuilt every candidate) → cache-aware eval hoisted to the outer optimizer; (3) `EvaluateSPWithPriorityVec` is shared infra (BF/Incre/ResetIncumbentBaseline) → baseline-only keeps it for BF; (4) chain RTDA (`GetRTDA_Dist_AllChains`) is orthogonal to the RTA cache, and `ObtainSP_DAG_From_Dists` (SP_Metric.cpp:129) already assembles full SP from precomputed node-RT dists + chain dists → cache-aware eval reuses it (node RTAs via `ComputeRTA_FullAndCache`, combine via `ObtainSP_DAG_From_Dists`). Wiring shape + TDD (differential: cache-aware SP bit-identical to `EvaluateSPWithPriorityVec` on the same `(dag_with_TLs, pa_vec)` sweep) captured in `tasks.md` §"Step 3b design". Docs updated (`tasks.md` new Phase 1.5 step 3b task + build-order 3b entry + call-chain map + updated Done-when; `goal.md` out-of-scope note; `dev_log.md` this entry). **No source/test changes.** Next (gated on user commit of 3c + OK to code): implement the cache-aware eval entry + route the incremental path's per-candidate calls through it + the differential test. Full record: `agents/active_tasks/P1_9_incremental_rta_patching/dev_log.md` §2026-07-14 (step 3b scope).

**P2.2 — Doc & memory hygiene CLOSED 2026-07-12 as superseded / moot (NOT executed).** User review ("update task 2_2, then move it to finished_tasks folder") concluded all three substantive items are outdated and the task should be closed, not worked. **(a)** `interval-sweep-stale-flags-bug` memory: premise stale in the *opposite* direction — it claims `compare_optimizers.py` **removed** `--on_taskset_config_change`/`--run_root` (sweep-stage crash, exit 2), but current code **accepts** both (`compare_optimizers.py:303` `--run_root`, `:334` `--on_taskset_config_change`); the described flag-removal never persisted, the crash doesn't reproduce. So the task's "mark RESOLVED-cite-the-fix" edit was the wrong action (the honest note would be "premise was wrong / bug no longer reproduces"). **No algorithm-performance impact** — even live this is a sweep-stage CLI plumbing crash; the algorithm runs in the simulate stage (SP metric / optimizer computation unaffected); the memory's own `--steps simulate aggregate` workaround yields correct optimize/aggregate output minus the period-sensitivity data; `run_end_to_end.sh` always runs simulate→sweep→aggregate (P22) and the P25 A/B config uses `interval_sweep_seconds_list=[10]` (single point → sweep no-op) → e2e doesn't crash. **(b)** §5 of `investigation_problems_encountered.md` pre-P1.1-resolution: memory `p25-ndiff-diff-semantics` (2026-07-07) re-derived the "2-vs-8 discrepancy" as false-positives (NOT UNRESOLVED) + "Fix C & Fix D both wrong levers"; P0.5 (2026-07-10, `a8dba07f`→`7fa2e9d2`) resolved the gate (both diff sides carry adopted TL; runtime `ndiff` 5→0) — both §5 asks moot. **(c)** `issues.md` deleted in `66c96c14` (2026-07-06) — resolved-by-deletion. **(d)** `trial_and_error` already DONE (`88af2c54`+`fa0b857f`) + P0.1 landed — self-resolved. **No code, agent doc, or memory file was edited** — only the closure recorded (`goal.md` "Closure disposition" + `tasks.md` verdicts + this entry + `overall_tasks.md` P2.2 row struck through + `finished_tasks/summary.md` entry). The `interval-sweep-stale-flags-bug.md` memory file itself was left untouched (out of scope for "update task + move"); it remains stale and is flagged for a separate one-line memory retirement. Folder `git mv`'d `active_tasks/P2_2_doc_memory_hygiene/` → `finished_tasks/P2_2_doc_memory_hygiene/`. Full record: `agents/finished_tasks/P2_2_doc_memory_hygiene/`.

**P1.10 — D1 sub-point 1 DECIDED + staged (code paused for docs) 2026-07-16.** The redundant `:238-239`-duplicate SP-eval (the carried-position variation emitted by `FindPriorityVec1D_Variations`, which reconstructs `opt_pa_` and re-evaluates it to the `:238-239` baseline SP) is removed at the **generator** level: `FindPriorityVec1D_Variations` (`OptimizeSP_Incre.h:75` / `.cpp:180`) gains a default `bool exclude_opt_pa = true` that skips emitting `i == old_priority_index`. **Inverts** the prior "sub-incremental drops `:238-239`" proposal — keep `:238-239` (genuine new-env SP, NOT redundant at baseline), drop the carried-pos variation instead (uniformly correct at baseline AND TL steps). **Bit-identical for the existing `OptimizeIncre`** (inherits `true`; carried-pos eval gives SP_base, `:282` adopt test is strict `>`, equal SP never displaces `opt_sp_`). Also fixes my earlier unstable caller-side guard (`GetProrityIndex(opt_pa_,...)` drifts when `:284` mutates `opt_pa_` on adoption). Staged in the working tree, **NOT built/tested/committed** — code work paused per user ("first update all design decisions into related task md file"); 4 unit tests + `ctest` 16/16 (DEBUG) gate pending. D1 main (method vs fast-path) + D2/D3/D4/dedup/D5 still open; 2 sharpenings raised (Type-L `et_increased` = actual avg-ET delta not TL-sign; D2 snapshot must cover the `ReOptimizePeriodic` path). P1.9 stays ON HOLD. Full record: `agents/active_tasks/P1_10_serialized_incremental_optimization/dev_log.md`.

**P1.10 — D1 LANDED + D2/D3/D4/D5 + #5/#6 ALL DECIDED 2026-07-17 (decisions-only session; per constraint (a) no Phase 2 source until explicit user go).** **D1 main DECIDED → new `OptimizeIncre_SingleTask(dag_tasks_update, task_id, et_increased)`** on base `OptimizePA_Incre` (extracted former `:274-292` loop body; `OptimizeIncre` refactored to CALL it; optional `baseline_sp=INT_MIN` sentinel; primitive TRUSTS caller-set `opt_sp_`, does NOT advance `dag_tasks_`). LANDED in commit `33b2270c` (the `exclude_opt_pa=true` generator change); D1-main staged NOT committed (user commits). *(CORRECTION 2026-07-17, after context summary: D1-main extraction + `baseline_sp` arg + differential bit-identity test `OptimizeIncre_SingleTask.Differential_BitIdenticalOnSingleEtChange` (`tests/testOptimizeIncrePA.cpp:381`) LANDED + COMMITTED in `3d2f9b28` — "staged NOT committed" was stale. **Phase 1 COMPLETE.** 16/16 ctest re-verified green on HEAD `3d2f9b28` + uncommitted D2-amendment tree together.)* **D2 DECIDED** — `FindTaskWithDifferentEt` UNCHANGED; the CALLER ensures both diff sides have identical ET for TL-flexible tasks (holds by construction at the pre-TL capture site; **RETIRES the `dag_tasks_prev_interval_pre_tl_` member** — prev-DAG is a LOCAL captured before the `:313` absorb, not retained state). **D3 DECIDED** — sort merged E+L queue by task weight descending (simple, uniform, no E/L tier; high-weight first; supersedes "reuse `TaskSortingHeuristic`"). **D4 DECIDED** — running-adopted single champion `res_opt_` (P0.5 durable incumbent; seeded from prev interval's results; sequential compare-and-keep via `UpdateRecords`/`CommitIncumbent`; strict-`>` adopt governs). **D5 DECIDED** — incremental-only first cut (the warm-started 1D primitive fits incremental, NOT reopt's memoryless `OptimizeFromScratch` PA search; plugging it into reopt = semantic change defeating escape-incumbent; + Type-E doesn't apply to reopt — it re-searches ALL PAs every candidate → nothing frozen). **D2 sub-point DISSOLVED by premise correction:** earlier "only-:313-capture → stale pre-reopt prev-DAG" worry rested on the WRONG premise that reopt doesn't update `dag_tasks_`; verified reopt updates it at `:452` just as incremental does at `:313` (both the ONLY writers; `BuildChallengerFromIncumbent`/`UpdateExtDistBasedOnTimeLimit`/`ResetIncumbentBaseline` all use fresh LOCALs) → at the start of interval T, before its own absorb, `dag_tasks_` holds T-1's env regardless of which path T-1 took → capture at `:313` only gives the T-1→T diff correct across the reopt boundary for free; NOT stale, NOT suppressed. **#5 dedup policy DECIDED — RaiseError, NOT a winner-pick:** a task is NOT allowed to be both env-changed AND TL-flexible (disjoint by generator design — TL-flexible tasks have no env dependence); a task in both = contract violation → `BuildSerializedTaskQueue` (D) hard-fails, does NOT silently pick a winner (user: "raise an error if a task is both env dep and TL flexible. i don't consider this case in this project"; tiebreak note IF ever needed: TL-flexible wins). **#6 baseline eval DECIDED — (b) dedicated re-score, NOT `EvaluateTimeLimitConfig_ScratchOrIncre`:** the baseline re-scores the champion's carried `{pa,tl}` under the new env DAG to seed `opt_sp_` and must NOT optimize (routing through `EvaluateTimeLimitConfig_ScratchOrIncre` → `OptimizeIncre` would do priority optimization on env-changed tasks BEFORE the queue's D3 weight-sorted order is honored → violates "sort tasks, then optimize in order"); baseline = `EvaluateSPWithPriorityVec(UpdateExtDistBasedOnTimeLimit(dag_tasks_, committed_tl), sp_parameters_, opt_pa_)` directly (no `OptimizeIncre`, no `BuildChallengerFromIncumbent` rebuild); the queue walk does ALL optimization in D3 order. **ALL DECISION POINTS NOW CLOSED** (D1–D5 + #5 + #6). 16/16 ctest green (DEBUG). Phase 2 source fully UNBLOCKED — await explicit user go. P1.9 still ON HOLD behind P1.10. Full record: `agents/active_tasks/P1_10_serialized_incremental_optimization/` (`tasks.md` Phase 0 + `api_design.md` "Decided answers" + `dev_log.md` §2026-07-17).

**P1.10 — D2 AMENDED (later same session 2026-07-17): structural filter `FindEnvTaskWithDifferentEt` supersedes "caller normalizes"; 2 new free functions + 3 tests LANDED+TESTED in working tree, NOT committed.** The 2026-07-17 D2 decision ("`FindTaskWithDifferentEt` UNCHANGED; caller ensures both diff sides identical ET for TL-flexible tasks") rested on a fragile premise — that a TL-flexible task's `execution_time_dist` is bit-equal on both pre-TL diff sides. Code-grounded investigation: `FiniteDist::operator!=` is `!operator==`, `operator==` is `approx_equal(other, 1e-1)` — a **10%-relative tolerance** (`Probability.cpp:415-417`); a TL-flexible task's `execution_time_dist` is built from the raw `execution_time_mu/min/max/sigma` YAML fields at read (`RegularTasks.cpp:75-79`) with NO read-time override to the adopted TL for perf-pair tasks → the raw fields flow into the dist → it can compare unequal across intervals for TL-induced (perf-pair grid) reasons, NOT env → the "caller equalizes" property holds only while dists stay within 10% of each other → fragile. User directed: *"rename the method as `FindEnvTaskWithDifferentEt`, and explicitly filter out tasks with TL options in the implementation."* ⇒ **TWO NEW FREE FUNCTIONS landed** in `OptimizeSP_Incre.h`/`.cpp` (siblings to `FindTaskWithDifferentEt`, NOT replacing it): (1) `FindTasksWithFlexibleTimeLimits(const DAG_Model&) → vector<int>` — task IDs with non-empty `timePerformancePairs` (mirrors the `{-1}`-sentinel test in `RecordTimeLimitOptions` `OptimizeSP_TL_BF.cpp:29-30`; pure query); (2) `FindEnvTaskWithDifferentEt(const DAG_Model&, const DAG_Model&) → vector<DiffObj>` — `FindTaskWithDifferentEt(prev, cur)` MINUS the TL-flexible set (the Type-E diff; env signal survives without bit-equal perf-pair dists). **`FindTaskWithDifferentEt` itself stays UNCHANGED** for ONE caller only: `OptimizeIncre`'s `.cpp:282` call site on the **live TL-walk path**, where the diff MUST flag the TL-walked (TL-flexible) task so its 1D priority is re-searched each TL step (filtering there would empty the diff + stop the mid-walk re-search = behavior change to the live path). A single filtered fn cannot serve both the Type-E diff (filter TL-flexible) and the TL walk (flag the TL-walked TL-flexible task) → two functions; the TL walk migrates to `OptimizeIncre_SingleTask(task_id, …)` in Phase 2 (D5), retiring `:282`'s TL-walk usage then. **Phase 2 Type-E diff now calls `FindEnvTaskWithDifferentEt(dag_tasks_prev_pre_tl, dag_tasks_)`, NOT the raw `FindTaskWithDifferentEt`** — supersedes api_design.md Section B's "reuse `FindTaskWithDifferentEt` UNCHANGED." 3 new test cases in `tests/testOptimizeIncrePA.cpp` (`FindTasksWithFlexibleTimeLimits`, `FindEnvTaskWithDifferentEt_filtersTLFlexible`, `FindEnvTaskWithDifferentEt_noopWhenFlaggedNotTLFlexible`) — the differential proves the filter drops the TL-flexible perf-pair mover (TSP task 0, 1500.9→400.9) and keeps the env mover (SLAM task 3, 2853→285). `cmake --build build --target check.SP_OPT -j5` (DEBUG) + `ctest` 16/16 GREEN (`testOptimizeIncrePA` #9 incl. new cases + `testIncreOpt_w_TL` #7 production TL caller both unchanged → no behavior change to the live path). Phase 2 still UNBLOCKED — await explicit user go. P1.9 still ON HOLD behind P1.10. Full record: `agents/active_tasks/P1_10_serialized_incremental_optimization/` (`api_design.md` "D2 amendment" block + `dev_log.md` §2026-07-17 "D2 AMENDED" + `tasks.md` D2 bullet).

**P1.11 — Task migrated from P1.9 and simplified 2026-07-18.** Migrated the Incremental RTA Patching records from `P1_9_incremental_rta_patching` to `P1_11_incremental_rta_patching` and simplified `goal.md`, `tasks.md`, and `dev_log.md` to focus purely on the active rev-3 single-champion cache design. Updated `agents/overall_tasks.md` to map P1.11 to the new folder and relocated Partial Task-Subset Optimization to P3.11. Deleted the old P1.9 folder.


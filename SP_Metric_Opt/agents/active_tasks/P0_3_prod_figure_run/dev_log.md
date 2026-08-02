# P0 3 prod figure run — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-07

- Task scaffolded during the agents-folder reorg. Not yet started.

## 2026-07-24 — Fix: log-y exec-time figure (fig1c) unreadable for N>10

### Symptom
`fig1c_mean_exec_time_vs_tasks_main` (run
`evalsuite_run_test_dur600_interval10_seed1000_tasks4x6x8x10x12x14x16`) showed
only decade ticks `0.001`/`0.01` on the log y-axis; the N=12/14/16 ET points
(0.022 / 0.029 / 0.062 s) floated in an unlabelled gap toward `0.1` and could
not be read.

### Root cause
ET data span 0.00025..0.062 s = ~2.41 decades.
`build_line_chart` (aggregate_across_tasks.py) suppressed sub-decade (minor)
tick LABELS whenever the span was `>= 2.0` decades ("wide range → gridlines
only, avoid a wall of numbers"). 2.4 decades is not actually wide enough to
justify that; the real wall-of-numbers problem only bites at genuinely wide
spans (>=~3 decades with many points). The 2.0 threshold left this run's points
stranded between decades.

### Fix
- Minor ticks are now ALWAYS labelled. Density: dense (every 2x..9x) for spans
  <~3 decades; sparse (2x, 5x) for wider spans.
- Wide-range threshold raised 2.0 → 3.0 decades.
- Docstring + the inline comment block updated to match.
- TDD: rewrote the pre-existing-broken `test_log_y_sets_log_scale` (it asserted
  minor ticks are NOT labelled but used narrow-range data that triggered
  labelling → already failing on HEAD) + added two range-contract tests
  (`test_log_y_narrow_range_uses_dense_minor_ticks`,
  `test_log_y_wide_range_uses_sparse_minor_ticks`). 41/41 pytest green
  (was 38 passed + 1 failing).
- Regenerated the run's figures in place via
  `python -m simulation_experiments.aggregate_across_tasks --mode test
  --config_json <run>/config.json`. fig1c now reads
  `0.0005/0.001/0.002/0.003/0.005/0.01/0.02/0.03/0.05/0.1` — every N readable.

Files: `simulation_experiments/aggregate_across_tasks.py`,
`tests/python/test_aggregate.py`. Not committed (user commits).

## 2026-07-24 — Follow-up: dense minor labels overlapped → sparse-always (2x/5x)

### Symptom (round 2)
The first fix labelled every sub-decade (2x..9x) for spans <~3 decades. On the
real figure that produced 8 minor labels per decade — they **overlapped into an
unreadable wall of numbers** (user: "all numbers in y axis are overlapped with
each other, not readable").

### Root cause
Overcorrection. The two failure modes are symmetric: too few labels (round 0:
gap → values unreadable) vs too many (round 1: overlap → labels unreadable).
Dense 2x..9x is only legible when the axis is tall enough and the span short
enough that each decade gets real vertical room — not generally true.

### Fix (final)
- **Minor ticks are always labelled, but SPARSELY: only 2x and 5x per decade.**
  This yields `0.001 / 0.002 / 0.005 / 0.01 / 0.02 / 0.05 / 0.1` — every
  in-between value readable, no overlap, at ANY decade span.
- Dropped the decade-span branching entirely (it was the overcomplication):
  removed the `finite_means`/`y_min`/`y_max`/`spans_many_decades` computation
  and the dense-vs-sparse conditional. One fixed sparse set `[0.2, 0.5]`.
- Docstring + inline comment rewritten to state the sparse-always contract and
  record that dense was tried-and-overlapped.
- TDD: merged the two range-contract tests (dense/sparse) into a single
  `test_log_y_minor_ticks_are_sparse_2x_5x` that loops over a narrow (~2.4-dec)
  AND wide (~3.3-dec) span and asserts `len(loc._subs) == 2` for both.
  40/40 pytest green (41 → 40: two tests → one).

### Verified
Regenerated fig1c in place; y-axis reads `0.0005 / 0.001 / 0.002 / 0.005 / 0.01
/ 0.02 / 0.05 / 0.1`, N=12/14/16 (0.022/0.029/0.062) sit on readable gridlines,
no label overlap.

Files: `simulation_experiments/aggregate_across_tasks.py`,
`tests/python/test_aggregate.py`. Not committed (user commits).

## 2026-08-02 — Records sync: P0.3 docs were stalled since 2026-07-24

### Context
The folder's `goal.md`/`tasks.md` last touched 2026-07-24; substantial work
landed since (P2.8 scripts/config consolidation, P2.5 `INCR_SCRATCH` removal,
P0.7/P0.10 fallback commits). This entry records what an audit of the working
tree found and corrects the docs to match current reality. NO code changed;
records only.

### Findings (audit of the actual tree)
1. **Entry point renamed (P2.8).** `run_simulation_and_plot_figures.sh` →
   `run_simulation_plot_eval_ns.sh`; old name deleted. One `.sh` entry point;
   experiments differ by `CONFIG_JSON`. Docs referenced the deleted name.
2. **fig_p25 data source moved.** The standalone
   `p25periodAB_run_test_dur600_interval10_seed1000_tasks6/` dir the docs
   pointed at is NOT in the tree. The period figure's data now comes from the
   prod run's ablation arms `INCR_Reopt_1`/`_10`/`_30`/`_60` (prod simulates
   all four; `ablation_scheduler_list` carries them).
3. **Sweep stale-flags crash RESOLVED.** The
   `interval-sweep-stale-flags-bug` memory (2026-07-04) had
   `interval_sweep.py` emit `--on_taskset_config_change`/`--run_root` that
   `compare_optimizers.py` no longer accepted → sweep stage crashed. Audit:
   `compare_optimizers.py` accepts BOTH flags again (`:505` / `:463`). The
   `--steps simulate aggregate` workaround is itself gone — stages are
   fixed-order simulate→sweep→aggregate, no `--steps` override. A failed stage
   aborts the pipeline (`run_end_to_end_experiments.py:528-531`).
4. **`normalize_sp: true` is the config default.** Each SP figure emits ONLY
   the normalized variant; raw SP suppressed as redundant. So the canonical
   `fig1a`/`fig1f`/`fig_ab_a` stems are the `_normalized` ones, and the "≤ 1.0"
   verification checks are against those.
5. **P0.1 dependency RESOLVED** (`7fa2e9d2`, subsumed by P0.5; the
   inspectability YAML write was discarded by user decision).
6. **`optimizer_comparison/` absent in clean tree** (created on first run);
   the old `run_prod_dur600...tasks4x6x8x10x12x14x16x18` run referenced in the
   goal is gone.

### Doc changes
- `goal.md`: rewritten — new entry point, fixed-order pipeline stages, the
  figure→emitted-stem table (normalized variants), P25 data source = ablation
  arms, out-of-scope note that the stale-flags crash is resolved.
- `tasks.md`: checklist corrected to match — `release/tests/RunOrchestrator`
  (not a stale binary), renamed `.sh`, ablation-arm P25 data, the must-have
  figure stem list.
- `dev_log.md`: this entry.

### Not started / next
- The `fig_p25_et_vs_period` generator is STILL NOT IMPLEMENTED (no such
  function in `aggregate_across_tasks.py` — verified by grep). This remains
  the open coding task before a prod run can produce all 8 must-have figures.
- The prod run itself has not been executed under the current tree.

Files: `agents/active_tasks/P0_3_prod_figure_run/{goal,tasks,dev_log}.md`.
Not committed (user commits).

## 2026-08-02 — REVERT: INCR fallback counters #1 (evaluated) + #3 (improving)

### What happened
Earlier today the two missing counters were added to
`IntervalFallbackOutcome` (#1 `evaluated_challenger_count`, #3
`improving_challenger_count`), instrumented in `UpdateRecords`, and emitted
in `FormatIntervalFallbackLogCsv`, with a TDD funnel-invariant test
(#1 ≥ #3 ≥ #4). All staged.

### Why reverted
User review: #1 (evaluated challengers) and #3 (improving challengers) are
general optimization-WALK metrics — they describe how the walk explores the
configuration space, not whether a fall-back trigger fired. Coupling them
onto `IntervalFallbackOutcome` / `interval_fallback_log_` (which is
specifically the per-interval record of the three fall-back triggers a/b-i/
b-ii) was the wrong home. #4 (`during_walk_reject_count`) is a genuine
fall-back outcome (trigger b-i gate-REJECT), so it stays; #1/#3 are not.

### Reverted
- `IntervalFallbackOutcome` back to its 7-field shape (no #1/#3).
- `UpdateRecords` counter block removed; #4 increment in the gate-REJECT
  branch unchanged.
- `FormatIntervalFallbackLogCsv` back to the 7-column header/row.
- Test: removed `FallbackLog_EvaluatedImprovingRejectCountsObeyFunnelInvariant`;
  the 6 `FormatIntervalFallbackLogCsvTest` cases restored to 7-column shape.
- Verified: `git diff --cached` empty for
  `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}` + `tests/testIncreOpt_w_TL.cpp`;
  no stray `evaluated_challenger_count`/`improving_challenger_count` refs.

### Where #1/#3 belong (NOT done — design open)
#1/#3 are walk-quality telemetry and should live in a walk-level record, not
a fall-back record — e.g. a per-dispatch `OptimizationWalkStats` (or per-walk
counters on the optimizer) emitted separately from the fall-back log. Not
implemented; awaiting design decision.

## 2026-08-02 — Re-add #1 (evaluated) + #3 (improving) in the correct home: separate walk-stats record + CSV

### Resolution
User decision: keep the fallback log pure (three triggers only) — #1/#3 go in a
SEPARATE walk-stats record + CSV, NOT appended to the fallback struct/file.

### What landed
- NEW struct `IntervalWalkStats { interval_idx, evaluated_challenger_count,
  improving_challenger_count }` in `OptimizeSP_TL_Incre.h`, sibling to
  `IntervalFallbackOutcome`. The fallback struct stays at its 7-field shape.
- NEW `FormatIntervalWalkStatsCsv` (free fn, mirrors the fallback CSV writer)
  → header `interval_idx,evaluated_challenger_count,improving_challenger_count`.
- NEW `interval_walk_stats_log_` member + `GetIntervalWalkStats()` getter on
  `OptimizePA_Incre_with_TimeLimits`; forwarded by `SimulationOrchestrator.h`.
- Instrumentation in `UpdateRecords`: `evaluated++` at entry (every call),
  `improving++` when `should_update` (a would-beat). Live interval's record is
  the back entry — same cadence as `during_walk_reject_count`.
- Both dispatchers (`Optimize_w_TL_ScratchOrIncre`, `OptimizePureIncremental`)
  push one `IntervalWalkStats` per call, indexed by `reoptimization_interval_count_`,
  alongside the fallback-log push.
- `RunOrchestrator.cpp` writes `interval_walk_stats.txt` (NEW file; separate
  from `interval_fallback_log.txt`).

### TDD
- 3 NEW `FormatIntervalWalkStatsCsvTest` cases (empty/header, default-zero row,
  populated row, multi-interval) — CSV-shape contract, mirrors the fallback suite.
- Extended `FallbackLog_AppendsOneEntryPerDispatchCallIndexedByCounter` to assert
  the walk-stats log mirrors the per-dispatch cadence (1 entry/call, indexed).
- Extended the gate-reject test with the funnel invariant:
  `evaluated ≥ improving ≥ during_walk_reject_count` (the gate only runs on a
  would-beat, so every reject was first improving). All three nonzero there.
- 131/131 `testIncreOpt_w_TL` (was 127; +4 walk-stats tests). 16/17 ctest (sole
  failure the pre-existing `testScheduleSimulate` CFS, unrelated — shells out to
  an absent RELEASE binary).

### Note on the prior REVERT
The REVERT entry above stands as honest history. That round coupled #1/#3 onto
the fallback struct; this round places them in their own record + CSV. The
funnel-invariant assertion moved from a standalone test into the gate-reject
scenario (where it's actually exercised).

Files: `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}`,
`sources/RTDA/ImplicitCommunication/SimulationOrchestrator.h`,
`tests/RunOrchestrator.cpp`, `tests/testIncreOpt_w_TL.cpp`. Not committed (user
commits).


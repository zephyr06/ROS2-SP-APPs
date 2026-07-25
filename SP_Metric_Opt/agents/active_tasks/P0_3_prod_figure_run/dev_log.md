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

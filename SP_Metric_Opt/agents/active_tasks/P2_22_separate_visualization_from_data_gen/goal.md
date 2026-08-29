# P2.22 — Separate Result Visualization from Data Generation in start_system.sh

**Priority:** P2 (pipeline reliability / real-world experiment hygiene)
**Status:** IN PROGRESS (2026-08-28)
**Depends on:** P0.11 (real-world exp config debug — parent investigation), PW.4 (paper §11/§12 real-exp figures)
**Related:** P2.8 (scripts/configs refactor — simulation side)

## Goal

Split the real-world experiment pipeline into two independently-runnable stages with clean responsibility separation:

1. **Data generation** — `scripts/start_system.sh` becomes purely: launch applications → record timing data in `all_time_records/` → pack into a `.tar.gz`. No visualization, no plotting, no `AnalyzeSP_Metric` subprocess. Data generation can never be blocked by a visualization crash or hang.

2. **Result visualization** — a new `scripts/visualize_experiment_results.py` takes either a `.tar.gz` path or an unpacked `all_time_records` folder path, and produces the real-world experiment figures: each application's execution time over time + scheduler SP-metric over time (matching §11 `fig_et_all` and `figs_exp_results_all_cps`).

Motivation: the user observed that "current start system doesn't contain all the visualization code" — start_system.sh only calls `draw_SP_current_scheduler.py` (SP-only); the ET-over-time plots live in `Visualize_SP_Metric/` but are never invoked by start_system.sh. Consolidating ALL visualization into one entry point fixes both the reliability hole and the completeness gap.

## Root cause context (from P0.11 hang investigation)

The `start_system.sh` "hang" (apps finish, SLAM prints exit stats, then nothing — no `backup in ...` echo; the script has no `set -e`) traced to the inline visualization call `draw_SP_current_scheduler.py` (start_system.sh:42), which has two reliability hazards:

1. **Crash on data gaps:** `SP_draw_fig_utils.get_sp_value` does `float(output_split[1])` on `AnalyzeSP_Metric`'s stdout. When a window's `TSP_execution_time` temp file is empty (TSP ran out of data late in the run), `AnalyzeSP_Metric`'s `FiniteDist(empty)` → `CheckDistributionValid()` prints `Error in FiniteDist constructor: sum of probabilities is 0`, and `float("in")` (token of "Error in...") raises `ValueError` (exit 1). On the dev machine this is a CRASH (exit 1); on the board (no `set -e`) a crashed python would still reach the backup echo, so the board symptom (no echo) implies a true HANG — most plausibly hazard 2.

2. **Headless GUI block:** the original code does not force a headless matplotlib backend. On a board with no `$DISPLAY`, `plt.show(block=False)`/`plt.pause(...)` can wedge in a GUI event loop (QtAgg abort / TkAgg raise / indefinite block depending on backend) — a non-exiting python = no backup echo.

Both hazards are ALREADY FIXED in `SP_draw_fig_utils.py` (uncommitted working-tree edits on `clean_simulation`): headless Agg backend when `DISPLAY` unset; `subprocess.run(..., timeout=ANALYZE_SP_TIMEOUT_S=120)` + returncode/parse guards + per-window skip + early-break. This task moves visualization OUT of `start_system.sh` so the fix is structural (data gen can't be blocked) rather than only in-library.

Empirically confirmed during the investigation: `AnalyzeSP_Metric` is fast (~0.13 s/window; 61/82 windows in ~8 s on extracted example data) — the user's instinct that "AnalyzeSP_Metric should be very fast" was correct; the prior "slow windows" framing was wrong.

## Reuse map (do not write new plotting logic)

The new script is a CLI dispatcher + input resolver; it reuses existing functions:

| Output | Single run (1 tar.gz / 1 unpacked folder) | Multi run (folder of tar.gz) |
|---|---|---|
| SP over time | `SP_draw_fig_utils.draw_and_saveSP_fig_single_run` (reliable, edited) | `visualize_SP_distribution.analyze_one_scheduler` / `process_tar_files_sp` |
| ET over time (per app) | reuse `visualize_ET_distribution.read_ET_data_from_file` + `average_execution_time_with_intervals` (thin single-run plot wrapper) | `visualize_ET_distribution.plot_ET_distribution` |
| Avg scheduler overhead | `report_avg_scheduler_overhead.calculate_average_execution_time` | (same) |

## Reliability improvements folded in

- Force headless Agg backend at script top when `DISPLAY` unset (inherit from `SP_draw_fig_utils`).
- `AnalyzeSP_Metric` subprocess timeout + crash-skip (inherited from `SP_draw_fig_utils.get_sp_value_list`).
- Remove unused `from dask.sizeof import sizeof` in `visualize_ET_distribution.py` (latent import-time crash if dask absent).
- Guard ET readers against empty/missing `*_execution_time.txt` (the empty-file crash class).
- Graceful degradation if `seaborn`/`pandas` unavailable (fall back to `matplotlib` boxplot).
- No `plt.show` blocking in headless mode.

## Non-goals

- Not changing the SP-metric C++ (`AnalyzeSP_Metric`) or the optimizer.
- Not changing application launch / data-record file formats.
- Not regenerating paper figures (that's PW.4); this script produces the same figure TYPES from a given dataset.
- Not touching `ryan-branch`; work only on `clean_simulation`.
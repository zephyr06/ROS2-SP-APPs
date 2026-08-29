# Tasks: P2.22 — Separate Result Visualization from Data Generation

- [x] **Step 1: Create reliable visualization entry script** (done 2026-08-28)
  - New `scripts/visualize_experiment_results.py`: argparse CLI — positional `input_path` (`.tar.gz`/`.tar.xz` file OR folder), optional `--output-dir`, `--scheduler-name`, `--apps`.
  - Force Agg backend when headless; add `Visualize_SP_Metric/` + repo-root + scripts/ to `sys.path` so imports work from any cwd.
  - Input resolver: `.tar.gz`/`.tar.xz` → unpack to temp → walk for `task_characteristics.yaml`; folder with `task_characteristics.yaml` → single-run as-is.
  - Single-run SP: reuse `get_app2period` + `get_task_set_info` + `get_sp_value_list` directly (path-parameterized lower-level fns — NOT the hardcoded `draw_and_saveSP_fig_single_run` wrapper).
  - Single-run ET: thin wrapper over `read_ET_data_from_file` (sampling disabled to keep publisher/et index-aligned) + `average_execution_time_with_intervals`, one subplot per app.
  - Reuse `calculate_average_execution_time` for scheduler overhead.
  - NOTE: multi-run box plots (paper `fig_et_all`/`figs_exp_results_all_cps`) NOT wired into this script yet — single-run is the explicit ask ("takes the tar.gz file's path"). Multi-run can delegate to `plot_ET_distribution` + `analyze_one_scheduler` as a follow-up if needed.

- [x] **Step 2: Reliability fixes in reused modules** (done 2026-08-28)
  - Removed unused `from dask.sizeof import sizeof` in `visualize_ET_distribution.py`.
  - Guarded `read_ET_data_from_file` against missing files (returns empty array).
  - Headless Agg + subprocess timeout confirmed inherited via `SP_draw_fig_utils` (prior-session edits).
  - `report_avg_scheduler_overhead.py` top-level guarded under `__main__` (importable without side effects).

- [ ] **Step 3: Strip visualization from start_system.sh** — READY on user approval (Step 4 now verified; old inline viz remains the fallback until this lands)
  - Remove the post-processing block: `report_avg_scheduler_overhead.py` + `draw_SP_current_scheduler.py` (lines ~39–42).
  - Keep app launch + `all_time_records` generation + `.tar.gz` packing + `Experiments/$1/` move.
  - Add a final echo pointing the user at the new visualization script + the produced tar.gz path.
  - NOTE: board-behavior change (start_system.sh no longer auto-plots); await user go-ahead since the board env differs from the dev machine where Step 4 was verified.

- [x] **Step 4: Verify on extracted example data** (done 2026-08-28)
  - Ran `python3 scripts/visualize_experiment_results.py all_time_records` headless — exit 0, both `sp_over_time.pdf` + `et_over_time.pdf` produced, **61/82 windows → avg SP=4.59314**.
  - **Root cause of the user's "exited 134" aborts:** AnalyzeSP_Metric (C++) resolves its `--file_path` yaml arg relative to its OWN base (`SP_Metric_Opt/`), NOT the shell CWD. A relative folder input (`all_time_records`) → yaml looked for at `SP_Metric_Opt/all_time_records/...` → `... not exist!` → abort EVERY window. The `.tar.gz` path worked only because `tempfile.mkdtemp` returns an ABSOLUTE path. FIX: `resolve_data_folder` now returns `os.path.abspath(...)` for folder inputs.
  - Reliability guard (subprocess timeout + returncode skip) correctly contained the SECOND crash class; then **Step 4b eliminated it outright**: window 640-650's `Error in FiniteDist constructor: sum of probabilities is 0.000000` was caused by `TSP_execution_time.txt` ending at index 425 (637.5 s) — the yaml's 850 s horizon over-states actual coverage — so window 640-650 had no TSP ext data. The response-time path falls back to `[1e9]` when empty but the TSP-ext path (`get_execution_time_within_range`) had NO fallback → wrote a 0-byte file → `FiniteDist(empty)` → abort. FIX: in `get_sp_value_list`, after the `>=75%` early-stop and before the subprocess, `if len(execution_time_within_range)==0: skip; continue` (TSP ext is a mandatory AnalyzeSP_Metric input). Verified: across all 82 windows now ZERO `exited`/`Aborted`/`FiniteDist` — one clean skip (640-650) + existing early-stop (650-660); same 61/82 windows, avg SP=4.59314 (no data lost).
  - Improved the guard to print STDOUT on failure (CoutError writes its reason to stdout, not stderr — without this the real cause was hidden behind `Aborted (core dumped)`).

- [x] **Step 6: Multi-run box plots** (done 2026-08-28; plumbing VERIFIED by user's real `Experiments/RM` run — see Step 6b for the two follow-up bugs found+fixed)
  - User ask: "the final figures should be box plots rather than results from a single run ... make sure the new script could potentially take multiple .tar.gz files as input, then produce box plots" → reproduce paper §11 `fig_et_all` (per-task ET box plots over time) + `figs_exp_results_all_cps` (SP box plots over time).
  - Read + understood existing multi-run code: `visualize_ET_distribution.plot_and_save_et_boxplot` (ET, seaborn/pandas, one box per time-bin across runs), `visualize_SP_distribution.plot_and_save_boxplot_sp` (SP, same pattern; normalizes SP÷5, ylim [0.1,1.05] — paper convention). Both reuse the pattern: folder of N `.tar.gz` → per-run time-series → transpose → `sns.boxplot` per bin.
  - Generalized the new script's CLI: `input_path` (single positional) → `input_paths` (nargs="+"). `resolve_inputs()` → list of `(data_folder, cleanup_dir)`; handles tarball files, folders-of-tarballs (`Experiments/{method}/` convention → one entry per archive), and unpacked records folders. **1 resolved run → single-run line plots (unchanged); >1 → multi-run box plots.**
  - Extracted shared per-run helpers `_sp_series_for_one_run` + `_et_series_for_one_run` (reusing `get_app2period`/`get_task_set_info`/`get_sp_value_list` + `read_ET_data_from_file`/`average_execution_time_with_intervals`); refactored `plot_sp_over_time`/`plot_et_over_time` to use them (behavior-identical).
  - SP multi-run deliberately does NOT use `draw_and_saveSP_fig_single_run` (hardcodes yaml path + `plt.show`/`pause` — the board-hang risk) — uses the reliable lower-level `get_sp_value_list` directly, then feeds per-run `(x_axis, sp_value_list)` series to the existing `plot_and_save_boxplot_sp`.
  - Lazy-import the seaborn/pandas renderers (only in box-plot mode; single-run mode stays dependency-light) with a clear `SystemExit` if seaborn/pandas missing. Per-run try/except fault isolation; skip-with-warning if `<2` runs produced data (a box plot needs ≥2). Each tarball → its own temp dir (no shared `temp_folder`), cleaned in `finally`.
  - [x] **Step 6b: Two bugs surfaced by user's real multi-run run — FIXED** (done + VERIFIED 2026-08-28)
  - User ran `python3 scripts/visualize_experiment_results.py Experiments/RM` → multi-run plumbing VERIFIED end-to-end (all box plots produced, `SCHEDULER_PUER_OPT` correctly skipped, 61/82 SP windows/run). But output exposed two bugs:
  - **Bug 1 (libGL spam):** running without `env -u DISPLAY` left a GUI QtAgg backend active → `plt.show` tried OpenGL → `libGL error: failed to load driver: iris/swrast` on every figure (non-fatal, but the board-hang risk class). **Fix:** force Agg **unconditionally** (batch PDF tool only ever `savefig`s) instead of only when `$DISPLAY` unset.
  - **Bug 2 (silent output deletion):** default `--output-dir` = `data_folders[0]` = the **temp extraction dir** for tarball inputs → the `finally` `shutil.rmtree` deleted the just-written PDFs. **Fix:** default `output_dir` to the **input path** (folder → that folder; `.tar.gz` → alongside the archive; unpacked-records folder → into it, unchanged). Never a temp dir.
  - Edits confined to `scripts/visualize_experiment_results.py` (docstring, backend forcing, `--output-dir` help + default). No existing modules touched.
  - Reliability preserved: Agg forced **unconditionally** (renderers' `plt.show`/`plt.pause` are no-ops under Agg — no GUI/OpenGL); per-window AnalyzeSP_Metric timeout + skip; empty-TSP-ext skip.

- [x] **Step 6c: Silence the "non-GUI backend, cannot show the figure" UserWarning** (done + VERIFIED 2026-08-28)
  - After Step 6b forced Agg unconditionally, the reused box-plot renderers' unconditional `plt.show()`/`plt.pause()` display epilogue (NO `show_fig_time` guard) prints `UserWarning: Matplotlib is currently using agg, which is a non-GUI backend, so cannot show the figure.` once per figure. **Not** a missing-output sign: in both renderers `plt.savefig` + CSV write happen BEFORE the show epilogue (`visualize_ET_distribution.py:191-192`, `visualize_SP_distribution.py:102-103`). Passing `show_fig_time=0` does NOT help (no guard; `plt.pause(0)` still calls show internally). Editing the renderers violates "reuse as-is"; reimplementing the box plots violates "don't write new code."
  - **Fix (new script only):** `import warnings` + `warnings.filterwarnings("ignore", message=r"Matplotlib is currently using .* which is a non-GUI backend", category=UserWarning)` placed right after `matplotlib.use("Agg")`, before `import matplotlib.pyplot as plt` (active for every renderer call). Scoped to that ONE message under UserWarning — all other warnings (seaborn/pandas/etc.) stay loud. Docstring's Agg reliability bullet updated to note the filter.
  - Edits confined to `scripts/visualize_experiment_results.py`. No existing modules touched.

- [ ] **Step 5: Stage for user review**
  - `git add` the new script, edited `SP_draw_fig_utils.py`, `visualize_ET_distribution.py`, `report_avg_scheduler_overhead.py`, task docs. (`start_system.sh` NOT edited — Step 3 still deferred pending user go-ahead.)
  - Re-verify (DONE 2026-08-28 — classifier recovered mid-session; ran both on the agent side):
    - multi-run: `python3 scripts/visualize_experiment_results.py Experiments/RM` → NO libGL spam, NO "non-GUI backend, cannot show the figure" warnings; 6 PDFs (`sp_boxplot` + `et_boxplot_{TSP,RRT,SLAM,MPC,SCHEDULER}`) land in `Experiments/RM/` (NOT `/tmp/...`) and SURVIVE after exit. ✓
    - single-run regression: `python3 scripts/visualize_experiment_results.py all_time_records` → `all_time_records/sp_over_time.pdf`+`et_over_time.pdf`, 61/82, avg SP=4.59314 (output location unchanged). ✓
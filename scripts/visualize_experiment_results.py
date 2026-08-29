#!/usr/bin/env python3
"""Visualize real-world experiment results from .tar.gz archives or unpacked folders.

This script is the visualization half of the experiment pipeline, kept separate
from scripts/start_system.sh (which only generates data records and packs the
.tar.gz). It takes one or more inputs:

  - a path to a .tar.gz / .tar.xz archive produced by start_system.sh, OR
  - a path to an already-unpacked all_time_records folder (containing
    task_characteristics.yaml + the *_execution_time.txt / *_publisher.txt files), OR
  - a folder containing several .tar.gz archives (one per run — the
    Experiments/{method}/ convention)

and produces, depending on how many runs it resolves:

  * ONE run  -> single-run line plots:
      - SP-metric over time   (reuses SP_draw_fig_utils.get_sp_value_list)
      - per-application execution time over time
        (reuses visualize_ET_distribution.read_ET_data_from_file
         + average_execution_time_with_intervals)
      - average scheduler overhead (reuses report_avg_scheduler_overhead)

  * MULTIPLE runs -> multi-run box plots (paper fig_et_all / figs_exp_results_all_cps):
      - SP-metric box plot over time, one box per window across runs
        (reuses visualize_SP_distribution.plot_and_save_boxplot_sp)
      - per-application execution-time box plot over time, one box per window
        (reuses visualize_ET_distribution.plot_and_save_et_boxplot)

Reliability notes (root cause of the board hang, see P0.11 / P2.22):
  - ALWAYS forces the non-interactive Agg matplotlib backend (this is a batch
    PDF tool — it only ever savefigs, never needs a GUI window), so the reused
    renderers' plt.show()/plt.pause() are harmless no-ops and never touch a
    GUI/OpenGL stack (which on some hosts emits libGL errors or wedges); the
    resulting "non-GUI backend, cannot show the figure" UserWarning is filtered
    (the PDFs are already saved before that no-op call);
  - AnalyzeSP_Metric is run with a subprocess timeout + per-window skip
    (inherited from SP_draw_fig_utils.get_sp_value_list);
  - missing/empty *_execution_time.txt files are skipped, not fatal;
  - per-run fault isolation: one bad tarball/run cannot abort a multi-run
    aggregation;
  - each tarball is extracted to its OWN temp dir (no shared state), cleaned up
    in a finally block;
  - never blocks on plt.show in headless mode.

The SP multi-run path deliberately does NOT go through draw_and_saveSP_fig_single_run
(that wrapper hardcodes the yaml path to <repo>/all_time_records/ and calls
plt.show/plt.pause); it reuses the reliable, path-parameterized lower-level
get_sp_value_list directly, then feeds the resulting per-run series to the
existing box-plot renderer.

Usage:
  # single run -> line plots
  python3 scripts/visualize_experiment_results.py <input_path>

  # multiple runs -> box plots (pass several archives/folders, or one folder of .tar.gz)
  python3 scripts/visualize_experiment_results.py <run1.tar.gz> <run2.tar.gz> ...
  python3 scripts/visualize_experiment_results.py Experiments/optimizerIncremental/

  [--output-dir DIR] [--scheduler-name NAME] [--apps APP ...]
"""
import os
import sys
import argparse
import shutil
import tempfile
import tarfile
import warnings

# This is a non-interactive batch tool: it only ever writes PDFs via savefig.
# ALWAYS force the Agg backend (regardless of $DISPLAY) BEFORE importing pyplot,
# so the reused renderers' plt.show()/plt.pause() calls are harmless no-ops and
# never touch a GUI/OpenGL stack. On hosts with $DISPLAY set but broken Mesa DRI
# drivers, a GUI backend (e.g. QtAgg) emits "libGL error: failed to load driver"
# noise on every plt.show; on the board it can wedge. Agg is a pure-software
# rasterizer — savefig output is identical, with no GUI/OpenGL involvement.
import matplotlib
matplotlib.use("Agg")

# Under the forced Agg backend the reused box-plot renderers still call
# plt.show()/plt.pause() as a display epilogue AFTER they have already saved the
# PDF + CSV (see visualize_ET_distribution.plot_and_save_et_boxplot and
# visualize_SP_distribution.plot_and_save_boxplot_sp -- savefig precedes show).
# Those calls are no-ops under Agg, but matplotlib emits a UserWarning for each
# ("Matplotlib is currently using agg, which is a non-GUI backend, so cannot show
# the figure."). Since the figures are already written before that no-op call,
# the warning is pure noise in this batch tool; filter just this one message
# (leave all other warnings loud) so output stays clean and predictable.
warnings.filterwarnings(
    "ignore",
    message=r"Matplotlib is currently using .* which is a non-GUI backend",
    category=UserWarning,
)

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np               # noqa: E402
import yaml                      # noqa: E402

# Make imports work regardless of cwd. REPO_ROOT for the
# `SP_Metric_Opt.Visualize_SP_Metric.box_plot_utils` import used inside
# visualize_ET_distribution / visualize_SP_distribution; _VIZ_DIR for the flat
# Visualize_SP_Metric imports; _SCRIPTS_DIR for the sibling
# report_avg_scheduler_overhead import.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_VIZ_DIR = os.path.join(REPO_ROOT, "SP_Metric_Opt", "Visualize_SP_Metric")
_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
for _p in (_SCRIPTS_DIR, _VIZ_DIR, REPO_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from SP_draw_fig_utils import (            # noqa: E402
    get_app2period, get_task_set_info, get_sp_value_list,
)
from visualize_ET_distribution import (    # noqa: E402
    read_ET_data_from_file, average_execution_time_with_intervals,
)
from report_avg_scheduler_overhead import calculate_average_execution_time  # noqa: E402

APP_NAMES = ["TSP", "RRT", "SLAM", "MPC"]
SCHEDULER_NAMES = ["SCHEDULER", "SCHEDULER_PUER_OPT"]
DISCARD_EARLY_TIME = 30
HORIZON_GRANULARITY = 10
# Disable read_ET_data_from_file's random sampling: et and publisher files are
# read separately, so independent sampling scrambles their index alignment. Read
# them fully (bounded < ~1e5 lines) to keep release-time <-> ET aligned.
NO_SAMPLING_THRESHOLD = 10 ** 9


# --------------------------------------------------------------------------- #
# Input resolution
# --------------------------------------------------------------------------- #
def _find_records_folder(root):
    """Walk root for a folder containing task_characteristics.yaml."""
    for dirpath, _dirnames, filenames in os.walk(root):
        if "task_characteristics.yaml" in filenames:
            return dirpath
    return None


def _is_tarball(path):
    return path.endswith(".tar.gz") or path.endswith(".tar.xz")


def _extract_tarball(tar_path):
    """Extract one archive to its own temp dir; return (records_folder, temp_dir)."""
    temp_dir = tempfile.mkdtemp(prefix="viz_exp_")
    try:
        with tarfile.open(tar_path, "r:*") as tar:
            tar.extractall(temp_dir)
        data_folder = _find_records_folder(temp_dir)
        if data_folder is None:
            raise FileNotFoundError(
                f"Archive {tar_path} contains no task_characteristics.yaml")
        return data_folder, temp_dir
    except Exception:
        shutil.rmtree(temp_dir, ignore_errors=True)
        raise


def _resolve_one(input_path):
    """Resolve a single input path to a list of (data_folder, cleanup_dir).

    A folder of .tar.gz archives expands to one entry per archive (the
    Experiments/{method}/ convention); a single archive or an unpacked records
    folder resolves to one entry.
    """
    if os.path.isfile(input_path) and _is_tarball(input_path):
        return [_extract_tarball(input_path)]
    if os.path.isdir(input_path):
        # Folder of tarballs? (one .tar.gz per run)
        tar_children = sorted(
            os.path.join(input_path, n) for n in os.listdir(input_path)
            if _is_tarball(n))
        if tar_children:
            return [_extract_tarball(tc) for tc in tar_children]
        # Unpacked records folder?
        if os.path.exists(os.path.join(input_path, "task_characteristics.yaml")):
            return [(os.path.abspath(input_path), None)]
        sub = _find_records_folder(input_path)
        if sub is not None:
            return [(os.path.abspath(sub), None)]
        raise FileNotFoundError(
            f"No task_characteristics.yaml (and no .tar.gz) found under {input_path}")
    raise FileNotFoundError(
        f"Input path is neither a tar archive nor a folder: {input_path}")


def resolve_inputs(input_paths):
    """Resolve a list of input paths into a list of (data_folder, cleanup_dir).

    Returns one entry per run. Callers branch on len(): 1 -> single-run line
    plots, >1 -> multi-run box plots.
    """
    resolved = []
    for input_path in input_paths:
        resolved.extend(_resolve_one(input_path))
    return resolved


# --------------------------------------------------------------------------- #
# Per-run series extraction (shared by single-run lines and multi-run boxes)
# --------------------------------------------------------------------------- #
def get_horizon_seconds(data_folder):
    cfg = os.path.join(data_folder, "task_characteristics.yaml")
    with open(cfg) as f:
        y = yaml.safe_load(f)
    # total_running_time is in ms; horizon in seconds.
    return int(float(y["tasks"][0]["total_running_time"]) / 1e3)


def _sp_series_for_one_run(data_folder):
    """Return (x_axis, sp_value_list) for one run via the reliable lower-level
    path (NOT draw_and_saveSP_fig_single_run, which hardcodes the yaml path and
    calls plt.show/plt.pause). Output shape matches plot_and_save_boxplot_sp's
    expectation: data[i] = (x_axis, sp_value_list).
    """
    cfg = os.path.join(data_folder, "task_characteristics.yaml")
    app_name2period = get_app2period(cfg)
    horizon = get_horizon_seconds(data_folder)
    tasks_name_to_info = get_task_set_info(APP_NAMES, app_name2period, data_folder)
    sp_value_list = get_sp_value_list(
        APP_NAMES, tasks_name_to_info, horizon, HORIZON_GRANULARITY,
        DISCARD_EARLY_TIME, task_set_abs_path=cfg)
    x_axis = [i + DISCARD_EARLY_TIME + HORIZON_GRANULARITY / 2.0
              for i in range(0, len(sp_value_list) * HORIZON_GRANULARITY,
                             HORIZON_GRANULARITY)]
    return x_axis, sp_value_list


def _et_series_for_one_run(data_folder, app):
    """Return np.array([midpoints, averages]) for one app/run, or None if the
    app has no ET/publisher data. Output shape matches plot_and_save_et_boxplot's
    expectation (data[i][0]=midpoints, data[i][1]=averages).
    """
    et_path = os.path.join(data_folder, app.upper() + "_execution_time.txt")
    pub_path = os.path.join(data_folder, app.upper() + "_publisher.txt")
    et_data = read_ET_data_from_file(et_path, sample_threshold=NO_SAMPLING_THRESHOLD)
    pub_time = read_ET_data_from_file(pub_path, sample_threshold=NO_SAMPLING_THRESHOLD)
    if len(et_data) == 0 or len(pub_time) == 0:
        return None
    m = min(len(et_data), len(pub_time))
    horizon = get_horizon_seconds(data_folder)
    return average_execution_time_with_intervals(
        pub_time[:m], et_data[:m], HORIZON_GRANULARITY,
        max_time_under_consider=horizon)


# --------------------------------------------------------------------------- #
# Single-run line plots (one input)
# --------------------------------------------------------------------------- #
def plot_sp_over_time(data_folder, scheduler_name, output_dir):
    x_axis, sp_value_list = _sp_series_for_one_run(data_folder)
    if not sp_value_list:
        print("[viz] no SP values produced (all windows skipped); skipping SP plot")
        return
    plt.figure()
    plt.plot(x_axis, sp_value_list, label=scheduler_name)
    plt.xlabel("Time (s)")
    plt.ylabel("SP-Metric")
    plt.legend()
    plt.grid(linestyle="--")
    plt.tight_layout()
    out = os.path.join(output_dir, "sp_over_time.pdf")
    plt.savefig(out, format="pdf")
    plt.close()
    avg = sum(sp_value_list) / len(sp_value_list)
    print(f"[viz] SP over time -> {out}  (avg SP={avg:.5f})")


def plot_et_over_time(data_folder, app_names, output_dir):
    n = len(app_names)
    if n == 0:
        return
    fig, axes = plt.subplots(n, 1, figsize=(8, 2.4 * n), sharex=True)
    if n == 1:
        axes = [axes]
    for ax, app in zip(axes, app_names):
        try:
            series = _et_series_for_one_run(data_folder, app)
            if series is None:
                ax.set_title(f"{app} (no data)")
                ax.set_ylabel("ET (s)")
                continue
            ax.plot(series[0], series[1])
            ax.set_ylabel(f"{app} ET (s)")
        except Exception as exc:  # one bad app must not abort the whole figure
            print(f"[viz] {app}: skipped ({exc})")
            ax.set_title(f"{app} (error: {exc})")
        ax.grid(linestyle="--")
    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    out = os.path.join(output_dir, "et_over_time.pdf")
    plt.savefig(out, format="pdf")
    plt.close()
    print(f"[viz] ET over time -> {out}")


def report_scheduler_overhead(data_folder):
    for name in SCHEDULER_NAMES:
        p = os.path.join(data_folder, name + "_execution_time.txt")
        avg = calculate_average_execution_time(p)
        if avg is not None:
            print(f"[viz] Average {name} execution time: {avg:.5f} s")


# --------------------------------------------------------------------------- #
# Multi-run box plots (multiple inputs) — reuse existing renderers
# --------------------------------------------------------------------------- #
def _import_box_renderers():
    """Lazy-import the seaborn/pandas-based box-plot renderers so single-run
    mode (which doesn't need them) never pulls those dependencies. The
    renderers are reused as-is from Visualize_SP_Metric/."""
    try:
        from visualize_ET_distribution import plot_and_save_et_boxplot
        from visualize_SP_distribution import plot_and_save_boxplot_sp
    except ImportError as e:
        raise SystemExit(
            "[viz] box-plot mode needs the existing renderers in "
            "Visualize_SP_Metric/ (which use seaborn + pandas). Install them "
            "(`pip install seaborn pandas`) if missing. Original ImportError: "
            + str(e))
    return plot_and_save_et_boxplot, plot_and_save_boxplot_sp


def plot_sp_boxplot(data_folders, scheduler_name, output_dir):
    """SP-metric box plot over time across runs (paper figs_exp_results_all_cps)."""
    plot_and_save_et_boxplot, plot_and_save_boxplot_sp = _import_box_renderers()
    series = []
    for f in data_folders:
        try:
            x_axis, sp_value_list = _sp_series_for_one_run(f)
            if sp_value_list:
                series.append((x_axis, sp_value_list))
        except Exception as exc:  # one bad run must not abort the aggregation
            print(f"[viz] SP boxplot: a run failed ({exc}) — skipping it")
    if len(series) < 2:
        print(f"[viz] SP boxplot: only {len(series)} run(s) produced SP values — "
              f"need >=2 for a box plot; skipping")
        return
    out_pdf = os.path.join(output_dir, "sp_boxplot.pdf")
    out_csv = os.path.join(output_dir, "sp_boxplot.csv")
    # Renderer normalizes SP by 5.0 and sets ylim [0.1, 1.05] (paper convention:
    # y-axis is SP/5, not raw SP). Reused as-is to reproduce the paper figure.
    plot_and_save_boxplot_sp(series, out_pdf, out_csv, scheduler_name,
                             show_fig_time=0.1)
    print(f"[viz] SP boxplot ({len(series)} runs) -> {out_pdf}")


def plot_et_boxplots(data_folders, app_names, output_dir):
    """Per-app execution-time box plot over time across runs (paper fig_et_all)."""
    plot_and_save_et_boxplot, _ = _import_box_renderers()
    for app in app_names:
        series = []
        for f in data_folders:
            try:
                s = _et_series_for_one_run(f, app)
                if s is not None:
                    series.append(s)
            except Exception as exc:  # one bad run must not abort the aggregation
                print(f"[viz] ET boxplot {app}: a run failed ({exc}) — skipping it")
        if len(series) < 2:
            print(f"[viz] ET boxplot {app}: only {len(series)} run(s) have data — "
                  f"need >=2 for a box plot; skipping")
            continue
        out_pdf = os.path.join(output_dir, f"et_boxplot_{app}.pdf")
        out_csv = os.path.join(output_dir, f"et_boxplot_{app}.csv")
        # Renderer uses xlim [30, 850] and normalize_coeff 1.0 (raw ET). Reused
        # as-is; empty late windows average to 0 (documented in the paper: "some
        # TSP instances are not run, yielding zero-execution-time points").
        plot_and_save_et_boxplot(series, out_pdf, out_csv, show_fig_time=0.1)
        print(f"[viz] ET boxplot {app} ({len(series)} runs) -> {out_pdf}")


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(
        description="Visualize real-world experiment results. ONE input -> "
                    "single-run line plots (SP + ET over time). MULTIPLE inputs "
                    "(or a folder of .tar.gz runs) -> multi-run box plots "
                    "(paper fig_et_all / figs_exp_results_all_cps).")
    ap.add_argument("input_paths", nargs="+",
                    help="One or more paths: .tar.gz/.tar.xz archives, unpacked "
                         "all_time_records folders, or a folder containing "
                         ".tar.gz runs (one per run).")
    ap.add_argument("--output-dir", default=None,
                    help="Where to write PDFs (default: alongside the first input "
                         "path -- the input folder for folder inputs, or the archive's "
                         "directory for .tar.gz inputs. Never a temp extraction dir.)")
    ap.add_argument("--scheduler-name", default="Scheduler",
                    help="Label for the SP plot legend.")
    ap.add_argument("--apps", nargs="*", default=APP_NAMES + SCHEDULER_NAMES,
                    help="Apps to plot ET for (default: TSP RRT SLAM MPC "
                         "SCHEDULER SCHEDULER_PUER_OPT).")
    args = ap.parse_args()

    resolved = resolve_inputs(args.input_paths)
    if not resolved:
        raise SystemExit("[viz] no valid inputs resolved")
    data_folders = [r[0] for r in resolved]
    cleanup_dirs = [r[1] for r in resolved if r[1]]
    if args.output_dir:
        output_dir = args.output_dir
    elif os.path.isdir(args.input_paths[0]):
        # Folder input (unpacked records, or a folder of .tar.gz runs): write
        # PDFs into that folder. Crucially this is the INPUT folder, NOT the
        # temp extraction dir of a tarball (which the finally block below
        # deletes) -- writing there would silently destroy the output PDFs.
        output_dir = args.input_paths[0]
    else:
        # A .tar.gz/.tar.xz file input: write alongside the archive.
        output_dir = os.path.dirname(os.path.abspath(args.input_paths[0])) or os.getcwd()
    os.makedirs(output_dir, exist_ok=True)

    try:
        if len(data_folders) == 1:
            # single-run: line plots (existing behavior)
            data_folder = data_folders[0]
            report_scheduler_overhead(data_folder)
            plot_sp_over_time(data_folder, args.scheduler_name, output_dir)
            plot_et_over_time(data_folder, args.apps, output_dir)
        else:
            # multi-run: box plots (aggregate across runs)
            print(f"[viz] multi-run mode: {len(data_folders)} runs -> box plots")
            report_scheduler_overhead(data_folders[0])
            plot_sp_boxplot(data_folders, args.scheduler_name, output_dir)
            plot_et_boxplots(data_folders, args.apps, output_dir)
    finally:
        for d in cleanup_dirs:
            shutil.rmtree(d, ignore_errors=True)

    print(f"[viz] done. outputs in {output_dir}")


if __name__ == "__main__":
    main()
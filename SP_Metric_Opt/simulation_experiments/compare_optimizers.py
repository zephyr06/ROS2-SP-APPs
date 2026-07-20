#!/usr/bin/env python3
"""
End-to-end optimizer comparison script for SP Metric Opt scheduling methods.

Generates N random task sets, runs all (or selected) scheduler modes on each
taskset, and produces consolidated CSV summaries plus visualization plots.

Usage:
    python compare_optimizers.py --n_tasksets 10 --num_tasks 6
"""
import argparse
import atexit
import contextlib
import csv
import glob
import json
import os
import shutil
import sys
import tempfile
import yaml

import concurrent.futures

# Ensure project root is in sys.path for absolute imports
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib.orchestrator import run_full_generation_pipeline
from Gen_Taskset.lib.generation_config_parser import (
    load_generation_config,
    resolve_taskset_config_path,
)
from simulation_experiments.run_sim_experiments import (
    run_single_simulation,
    analyze_single_instance,
    _needs_generation,
    _should_generate,
)
from simulation_experiments.utils import (
    compute_miss_rate,
    compute_miss_rate_by_task,
    compute_important_task_miss_rate,
    write_summary_and_plots,
    MATPLOTLIB_AVAILABLE,
)
from simulation_experiments.plotting_config import save_figure

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
except ImportError:
    np = None
    plt = None
    matplotlib = None



ALL_SCHEDULERS = [
    "INCR_Reopt_10", "BF",
    "INCR_NO_TL", "INCR_WCET",
    "RM", "RM_FAST", "RM_SLOW",
    "CFS",
]


def parse_interval_sp_metrics(metrics_path):
    """Parse interval_sp_metrics.txt to get per-interval SP values."""
    values = []
    if not os.path.exists(metrics_path):
        return values
    with open(metrics_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) >= 2:
                try:
                    values.append(float(parts[1]))
                except ValueError:
                    pass
    return values


def plot_optimizer_sp_line(results_by_scheduler, schedulers, output_path,
                           metric_name="SP Metric", ylabel="Average SP Metric"):
    """Generate a line chart of mean SP (with std error bars) per scheduler.

    Replaces the previous bar chart -- line plots are used everywhere per the
    project convention. Schedulers are plotted in order on a categorical
    x-axis, markers connected by a line. Saved as both PNG and PDF.
    """
    if not MATPLOTLIB_AVAILABLE:
        print("Skipping SP line plot (matplotlib not available).")
        return

    valid_schedulers = []
    means = []
    stds = []
    for s in schedulers:
        sp_vals = results_by_scheduler[s].get("sp_values", [])
        if sp_vals:
            valid_schedulers.append(s)
            arr = np.array(sp_vals)
            means.append(np.mean(arr))
            stds.append(np.std(arr))

    if not valid_schedulers:
        print("No valid SP data for line plot.")
        return

    fig, ax = plt.subplots(figsize=(12, 6))
    colors = matplotlib.colormaps["tab10"]
    x = np.arange(len(valid_schedulers))
    ax.errorbar(
        x, means, yerr=stds, marker="o", markersize=8, linewidth=2, capsize=5,
        color=colors(0),
    )
    ax.set_xticks(x)
    ax.set_xticklabels(valid_schedulers, rotation=15, ha="right")
    ax.set_ylabel(ylabel)
    ax.set_title(f"{metric_name} by Scheduler")
    ax.grid(axis="y", linestyle="--", alpha=0.5)
    plt.tight_layout()
    # Strip a trailing .png so save_figure can append both .png and .pdf.
    output_stem = output_path[:-4] if output_path.lower().endswith(".png") else output_path
    save_figure(fig, output_stem)
    plt.close(fig)
    print(f"Saved SP comparison line plot to: {output_stem}.{{png,pdf}}")


def plot_optimizer_exec_time_line(results_by_scheduler, schedulers, output_path):
    """Generate a line chart of average scheduler execution time per mode.

    Replaces the previous bar chart. Schedulers on a categorical x-axis,
    markers connected by a line. Saved as both PNG and PDF.
    """
    if not MATPLOTLIB_AVAILABLE:
        print("Skipping execution-time plot (matplotlib not available).")
        return

    valid_schedulers = []
    means = []
    stds = []
    for s in schedulers:
        times = results_by_scheduler[s].get("sched_times", [])
        if times:
            valid_schedulers.append(s)
            arr = np.array(times)
            means.append(np.mean(arr))
            stds.append(np.std(arr))

    if not valid_schedulers:
        print("No valid execution-time data for plot.")
        return

    fig, ax = plt.subplots(figsize=(12, 6))
    colors = matplotlib.colormaps["tab10"]
    x = np.arange(len(valid_schedulers))
    ax.errorbar(
        x, means, yerr=stds, marker="o", markersize=8, linewidth=2, capsize=5,
        color=colors(0),
    )
    ax.set_xticks(x)
    ax.set_xticklabels(valid_schedulers, rotation=15, ha="right")
    ax.set_ylabel("Avg. Execution Time (s)")
    ax.set_title("Scheduler Execution Time")
    ax.grid(axis="y", linestyle="--", alpha=0.5)
    plt.tight_layout()
    output_stem = output_path[:-4] if output_path.lower().endswith(".png") else output_path
    save_figure(fig, output_stem)
    plt.close(fig)
    print(f"Saved execution-time line plot to: {output_stem}.{{png,pdf}}")


def plot_per_taskset_line(results_by_taskset, schedulers, output_path):
    """Generate a line chart of mean SP per scheduler across tasksets.

    Replaces the previous grouped bar chart. X-axis is the taskset index, with
    one line per scheduler. Saved as both PNG and PDF.
    """
    if not MATPLOTLIB_AVAILABLE:
        return

    n_tasksets = len(results_by_taskset)
    if n_tasksets == 0:
        return

    # Compute mean SP per scheduler per taskset
    data = {}
    for s in schedulers:
        data[s] = []
        for ts_idx in range(n_tasksets):
            sp_vals = results_by_taskset[ts_idx].get(s, {}).get("sp_values", [])
            if sp_vals:
                data[s].append(np.mean(sp_vals))
            else:
                data[s].append(0.0)

    fig, ax = plt.subplots(figsize=(14, max(6, n_tasksets * 0.4)))
    x = np.arange(n_tasksets)
    colors = matplotlib.colormaps["tab10"]

    for i, s in enumerate(schedulers):
        ax.plot(
            x, data[s], marker="o", markersize=6, linewidth=1.5,
            label=s, color=colors(i),
        )

    ax.set_xlabel("Taskset Index")
    ax.set_ylabel("Average SP Metric")
    ax.set_title("Per-Taskset Average SP Metric by Scheduler")
    ax.set_xticks(x)
    ax.set_xticklabels([f"TS {i}" for i in x])
    ax.legend(loc="upper right")
    ax.grid(axis="y", linestyle="--", alpha=0.5)
    plt.tight_layout()
    output_stem = output_path[:-4] if output_path.lower().endswith(".png") else output_path
    save_figure(fig, output_stem)
    plt.close(fig)
    print(f"Saved per-taskset line plot to: {output_stem}.{{png,pdf}}")


def resolve_run_output_dir(base_output_dir, run_name, num_tasks, n_sec,
                           scheduler_trigger_interval, base_seed, run_root=None):
    """Return the full output directory path for a comparison run.

    Parameters
    ----------
    base_output_dir : str
        The parent directory (e.g. simulation_experiments/optimizer_comparison).
        Used only when *run_root* is None (standalone invocation).
    run_name : str or None
        Explicit subfolder name. If None, auto-generated from parameters.
    num_tasks, n_sec, scheduler_trigger_interval, base_seed : int
        Parameters used for auto-naming when run_name is None.
    run_root : str or None
        When set by the e2e orchestrator (``--run_root``), the run's raw sim
        output is co-located under the run folder at
        ``<run_root>/sim/<subfolder>`` (P23) so it lives next to the derived
        figures. When None (standalone ``compare_optimizers`` invocation), the
        legacy layout is kept: ``<base_output_dir>/<subfolder>``.
    """
    if run_name:
        subfolder = run_name
    else:
        subfolder = (
            f"tasks{num_tasks}_dur{n_sec}_"
            f"interval{scheduler_trigger_interval}_seed{base_seed}"
        )
    if run_root:
        return os.path.join(run_root, "sim", subfolder)
    return os.path.join(base_output_dir, subfolder)


# ---------------------------------------------------------------------------
# P1.15 (layer A) — stop-on-crash + show-loudly helpers.
#
# The harness used to swallow a crashing optimizer arm: concurrent.futures.wait
# never retrieved the worker exception, analyze_single_instance returned a
# silent 0.0 for the missing output, and the aggregator averaged over unequal
# per-scheduler taskset sets. These helpers implement the user's two explicit
# requirements: stop the run on the first crash, and show it loudly (a per-arm
# run.log with the C++ throw text, a crash_report.txt, and a per-(taskset, arm)
# status map), then exit non-zero after writing a partial summary.
# ---------------------------------------------------------------------------

def _run_log_path(sched_dir):
    """Path to the per-arm run.log that run_single_simulation writes the
    binary's combined stdout+stderr to (so the C++ throw text survives a
    crash). ``sched_dir`` is ``<taskset_dir>/<scheduler>``."""
    return os.path.join(sched_dir, "run.log")


def _has_complete_metrics(sched_dir, scheduler):
    """P1.15 (layer A, A5): a --resume arm is only skipped if its
    interval_sp_metrics.txt exists AND contains at least one parseable data
    row. A 0-byte or header-only file (a half-written abort) is NOT a complete
    run and is re-attempted, not silently skipped."""
    metrics_path = os.path.join(sched_dir, scheduler, "interval_sp_metrics.txt")
    if not os.path.exists(metrics_path):
        return False
    try:
        if os.path.getsize(metrics_path) == 0:
            return False
    except OSError:
        return False
    with open(metrics_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) < 2:
                continue
            try:
                float(parts[1])
            except ValueError:
                continue
            return True  # at least one parseable SP data row
    return False


def _mark_status_row(status_rows, taskset_idx, scheduler, inst, status):
    """Finalize the status of a (taskset, scheduler, inst) arm that was
    submitted (its row already exists with status RUNNING). Lookup is by the
    (taskset, scheduler, inst) key since the row list may have RESUMED/NOT_RUN
    entries interleaved."""
    for row in status_rows:
        if (row["taskset"] == taskset_idx and row["scheduler"] == scheduler
                and row["inst"] == inst and row["status"] == "RUNNING"):
            row["status"] = status
            return


def _write_status_map(output_dir_abs, status_rows):
    """Write taskset_arm_status.csv: the per-(taskset, arm) ✅/❌ map. A crashed
    arm is CRASHED (not ok); arms never reached because the run stopped early
    are NOT_RUN; reused --resume arms are RESUMED; completed arms are OK."""
    map_path = os.path.join(output_dir_abs, "taskset_arm_status.csv")
    with open(map_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["taskset", "scheduler", "inst", "status", "run_log"])
        # Sort for stable output: by taskset, then scheduler order is not
        # available here, so sort by (taskset, scheduler, inst) lexicographically.
        for row in sorted(status_rows,
                          key=lambda r: (r["taskset"], r["scheduler"], r["inst"])):
            writer.writerow([
                row["taskset"], row["scheduler"], row["inst"],
                row["status"], row.get("run_log", ""),
            ])


def _write_crash_report(output_dir_abs, crashed_arms):
    """Write crash_report.txt naming every crashed (taskset, arm), its captured
    run.log path, and (when the log exists) the last lines of the binary's
    output so the C++ throw text is surfaced without needing to open the log."""
    report_path = os.path.join(output_dir_abs, "crash_report.txt")
    lines = []
    lines.append("=" * 70)
    lines.append("OPTIMIZER CRASH REPORT (P1.15 layer A)")
    lines.append("=" * 70)
    lines.append(f"Number of crashed arms: {len(crashed_arms)}")
    lines.append("")
    for arm in crashed_arms:
        lines.append("-" * 70)
        lines.append(
            f"taskset_{arm['taskset']} / {arm['scheduler']} instance "
            f"{arm['inst']}"
        )
        lines.append(f"  error: {arm['error']}")
        lines.append(f"  run.log: {arm['run_log']}")
        log_tail = _read_log_tail(arm["run_log"], max_lines=40)
        if log_tail:
            lines.append("  --- last lines of run.log ---")
            for log_line in log_tail:
                lines.append(f"  | {log_line}")
        else:
            lines.append("  (run.log not found or empty)")
        lines.append("")
    with open(report_path, "w") as f:
        f.write("\n".join(lines) + "\n")



def _read_log_tail(log_path, max_lines=40):
    """Return the last ``max_lines`` lines of a log file (for the crash
    report), or [] if the file is missing/empty."""
    if not log_path or not os.path.exists(log_path):
        return []
    try:
        with open(log_path, "r", errors="replace") as f:
            all_lines = f.read().splitlines()
    except OSError:
        return []
    return all_lines[-max_lines:]


def main():
    parser = argparse.ArgumentParser(
        description=("Compare scheduler optimizers across randomly generated tasksets. "
                     "Generates tasksets, runs simulations, and produces plots + CSVs.")
    )
    parser.add_argument(
        "-n", "--n_tasksets", type=int, default=10,
        help="Number of task sets to generate and evaluate (default: 10)"
    )
    parser.add_argument(
        "--num_tasks", type=int, default=6,
        help=("Number of tasks (default: 6). On-disk paper configs exist for "
              "4/6/8; any N >= 2 is supported and the config is synthesized "
              "on the fly for other values.")
    )
    parser.add_argument(
        "-t", "--n_sec", type=int, default=300,
        help="GMM trace path duration in seconds (default: 300)"
    )
    parser.add_argument(
        "--scheduler_trigger_interval", type=int, default=10,
        help=("Interval in seconds between scheduler re-optimizations. "
              "This controls taskset generation (UPDATE_INTERVAL_S) and "
              "the per-interval simulation duration sent to C++ (default: 10)")
    )
    parser.add_argument(
        "--n_inst", type=int, default=1,
        help="Number of GMM trace instances per path (default: 1)"
    )
    parser.add_argument(
        "-o", "--output_dir", type=str,
        default="simulation_experiments/optimizer_comparison",
        help=("Base output directory for results (default: "
              "simulation_experiments/optimizer_comparison). "
              "A subfolder is auto-created per configuration.")
    )
    parser.add_argument(
        "--run_name", type=str, default=None,
        help=("Custom subfolder name inside --output_dir. "
              "If omitted, a name is auto-generated from num_tasks, "
              "n_sec, scheduler_trigger_interval, and base_seed "
              "(e.g. tasks6_dur300_interval10_seed1000).")
    )
    parser.add_argument(
        "--run_root", type=str, default=None,
        help=("End-to-end run root (passed by run_end_to_end_experiments). "
              "When set, raw sim output is co-located under the run folder at "
              "<run_root>/sim/<subfolder> (P23) so it lives next to the derived "
              "figures. When omitted (standalone invocation), sim output goes "
              "to <output_dir>/<subfolder> as before.")
    )
    parser.add_argument(
        "--bin_dir", type=str, default="release",
        help="Directory containing C++ binaries (default: release)"
    )
    parser.add_argument(
        "--base_seed", type=int, default=1000,
        help="Base random seed for deterministic taskset generation (default: 1000)"
    )
    parser.add_argument(
        "--schedulers", nargs="+", default=ALL_SCHEDULERS,
        help=f"List of schedulers to evaluate (default: {' '.join(ALL_SCHEDULERS)})"
    )
    parser.add_argument(
        "-v", "--verbose", type=int, choices=[0, 1, 2], default=1,
        help="Verbosity level (0: minimal, 1: progress, 2: debug simulator output)"
    )
    parser.add_argument(
        "--skip_generation_if_exists", action="store_true", default=True,
        help=("Reuse an existing taskset instead of regenerating. Before "
              "reusing, the resolved generator config is compared against the "
              "generator_config.json saved with the on-disk taskset; a change "
              "triggers --on_taskset_config_change. Default: on.")
    )
    parser.add_argument(
        "--on_taskset_config_change", choices=["prompt", "regenerate", "keep"],
        default="prompt",
        help=("Policy when an existing taskset's config has changed: 'prompt' "
              "(default) asks [Y/n] and regenerates on yes (or regenerates "
              "silently when non-interactive); 'regenerate' always regenerates; "
              "'keep' reuses the stale taskset with a warning.")
    )
    parser.add_argument(
        "--important_task_pct", type=float, default=0.10,
        help="Fraction of tasks considered 'important' (top by sp_weight) for important-task miss rate (default: 0.10)"
    )
    parser.add_argument(
        "--num_workers", type=int, default=None,
        help="Number of parallel worker processes for scheduler execution (default: min(4, cpu_count))"
    )
    parser.add_argument(
        "--resume", action="store_true",
        help="Skip simulation if interval_sp_metrics.txt already exists"
    )
    parser.add_argument(
        "--export_level", type=int, default=1, choices=[0, 1, 2, 3],
        help="Export detail level: 0=sp-only, 1=+task miss rate, 2=+task aggregate, 3=full job traces"
    )

    args = parser.parse_args()

    if args.scheduler_trigger_interval < 1:
        parser.error("--scheduler_trigger_interval must be >= 1")
    if args.num_tasks < 1:
        parser.error("--num_tasks must be >= 1 (P17: N_BIG=0 / N_SMALL=0 are allowed)")

    # Resolve paths
    config_file_abs = resolve_taskset_config_path(args.num_tasks)
    base_output_dir = (
        args.output_dir if args.output_dir.startswith("/")
        else os.path.join(PROJECT_ROOT, args.output_dir)
    )
    # scheduler_trigger_interval (seconds) is the single concept that drives
    # generation, simulation, and plotting.
    scheduler_trigger_interval = args.scheduler_trigger_interval
    interval_duration_ms = scheduler_trigger_interval * 1000  # per-interval duration in ms for C++

    output_dir_abs = resolve_run_output_dir(
        base_output_dir, args.run_name, args.num_tasks, args.n_sec,
        scheduler_trigger_interval, args.base_seed, run_root=args.run_root
    )

    bin_dir_abs = (
        args.bin_dir if args.bin_dir.startswith("/")
        else os.path.join(PROJECT_ROOT, args.bin_dir)
    )
    sim_bin_path = os.path.join(bin_dir_abs, "tests", "RunOrchestrator")

    if not os.path.exists(sim_bin_path):
        print(f"Error: RunOrchestrator binary not found at {sim_bin_path}. "
              f"Compile in release mode first.")
        sys.exit(1)

    if args.verbose >= 1:
        print(f"Optimizer Comparison: {args.n_tasksets} tasksets × "
              f"{len(args.schedulers)} schedulers")
        print(f"Task count: {args.num_tasks} | Duration: {args.n_sec}s | "
              f"Trigger interval: {scheduler_trigger_interval}s | Seed: {args.base_seed}")
        print(f"Schedulers: {args.schedulers}")
        print(f"Output: {output_dir_abs}")

    os.makedirs(output_dir_abs, exist_ok=True)

    # Temporary directory for seeded JSON configs
    temp_dir = tempfile.mkdtemp(prefix="temp_experiment_configs_", dir=PROJECT_ROOT)
    atexit.register(shutil.rmtree, temp_dir, ignore_errors=True)

    # Plot x-axis spacing equals the scheduler trigger interval (seconds)
    horizon_granularity = scheduler_trigger_interval

    # Data structures
    results_by_scheduler = {
        s: {
            "sp_values": [], "miss_rates": [], "intervals": {},
            "sched_times": [], "important_miss_rates": [],
            "non_important_miss_rates": []
        }
        for s in args.schedulers
    }
    results_by_taskset = []  # per-taskset dict for per-taskset plots
    # P1.15 (layer A): per-(taskset, arm) outcome map + the list of arms that
    # crashed (populated by the sim/analyze loop). On any crash the run stops,
    # _write_crash_report emits crash_report.txt + taskset_arm_status.csv, and
    # main() exits non-zero after writing a partial summary.
    status_rows = []   # list of {taskset, scheduler, inst, status, run_log}
    crashed_arms = []  # list of {taskset, scheduler, inst, sched_dir, run_log, error}

    for idx in range(args.n_tasksets):
        if args.verbose >= 1:
            print(f"\n{'='*60}")
            print(f"TASKSET {idx + 1} / {args.n_tasksets}, N={args.num_tasks}")
            print(f"{'='*60}")

        taskset_dir = os.path.join(output_dir_abs, f"taskset_{idx}")
        os.makedirs(taskset_dir, exist_ok=True)

        # Load, seed, and save configuration (resolve INCLUDE so temp file is self-contained)
        config_dict = load_generation_config(config_file_abs)
        config_dict["RANDOM_SEED"] = args.base_seed + idx
        config_dict["UPDATE_INTERVAL_S"] = scheduler_trigger_interval

        # Decide whether to (re)generate. _should_generate compares
        # ``config_dict`` against the generator_config.json saved with the
        # on-disk taskset, so a stale taskset (generated under an older config)
        # is never silently reused. The baseline file is written only when we
        # actually generate, below -- writing it here would clobber the baseline.
        should_generate = _should_generate(
            taskset_dir, config_dict, idx,
            skip_if_exists=args.skip_generation_if_exists,
            on_change_policy=args.on_taskset_config_change,
            verbose=args.verbose,
        )

        temp_cfg_path = os.path.join(temp_dir, f"temp_cfg_{idx}.json")
        with open(temp_cfg_path, "w") as f:
            json.dump(config_dict, f, indent=4)

        # 1. Generate taskset
        if should_generate:
            # Record the config that generated this taskset as the staleness
            # baseline for future runs' _should_generate comparison.
            with open(os.path.join(taskset_dir, "generator_config.json"), "w") as f:
                json.dump(config_dict, f, indent=4)
            if args.verbose >= 1:
                print("  Generating taskset...")
            if args.verbose >= 2:
                run_full_generation_pipeline(
                    cfg_file=temp_cfg_path,
                    n_sec=args.n_sec,
                    dir_path=taskset_dir,
                    add_perf_records=True,
                    interact=False,
                    n_path_per_task=1,
                    n_inst_per_path=args.n_inst,
                )
            else:
                with open(os.devnull, "w") as devnull:
                    with contextlib.redirect_stdout(devnull), \
                         contextlib.redirect_stderr(devnull):
                        run_full_generation_pipeline(
                            cfg_file=temp_cfg_path,
                            n_sec=args.n_sec,
                            dir_path=taskset_dir,
                            add_perf_records=True,
                            interact=False,
                            n_path_per_task=1,
                            n_inst_per_path=args.n_inst,
                        )
        else:
            if args.verbose >= 1:
                print("  Taskset already exists, skipping generation.")

        # Read task deadlines and sp_weights for miss-rate analysis
        char_fpath = os.path.join(taskset_dir, "taskset_characteristics_interval_0.yaml")
        if not os.path.exists(char_fpath):
            # Fallback to the global characteristics file
            char_fpath = os.path.join(taskset_dir, "taskset_characteristics.yaml")
        with open(char_fpath, "r") as f:
            yaml_data = yaml.safe_load(f)
        task_deadlines = {t["id"]: float(t["deadline"]) for t in yaml_data["tasks"]}
        task_sp_weights = {t["id"]: float(t.get("sp_weight", 0.0)) for t in yaml_data["tasks"]}

        # Determine number of worker processes
        num_workers = args.num_workers
        if num_workers is None:
            num_workers = min(4, os.cpu_count() or 1)

        # 2. Run each scheduler in parallel. The on-disk interval count is
        # controlled solely by n_sec / scheduler_trigger_interval (removed
        # --max_intervals), so C++ sees exactly the generated intervals.
        #
        # P1.15 (layer A, A1): a crashing optimizer arm must STOP the run, not
        # be swallowed. Two changes vs. the old fire-and-forget loop:
        #   - ThreadPoolExecutor (was ProcessPoolExecutor): the worker only
        #     calls subprocess.run (I/O-bound, releases the GIL), so threads
        #     give real parallelism for the binary runs while letting us
        #     retrieve per-arm exceptions via future.result(). ProcessPool
        #     silently dropped a crashing worker's CalledProcessError because
        #     concurrent.futures.wait never calls .result().
        #   - On the first arm whose future raises, we record the crash and
        #     stop launching any further arms (stop-on-crash). The crash is
        #     surfaced loudly after the loop: crash_report.txt +
        #     taskset_arm_status.csv + a partial summary (crashed arm -> NaN
        #     via A4) + a non-zero exit. See _write_crash_report below.
        if args.verbose >= 1:
            print(f"  Running simulations ({num_workers} workers)...")

        # status_rows: per (taskset, arm) outcome for the status map. A row is
        # added when the arm is submitted and finalized to CRASHED/OK after the
        # future resolves. Arms never submitted (because an earlier arm crashed
        # and we stopped) are recorded as NOT_RUN by _finalize_status_rows.
        sim_futures = []  # list of (future, scheduler, inst, sched_dir)
        crashed_arm_this_taskset = None
        with concurrent.futures.ThreadPoolExecutor(max_workers=num_workers) as executor:
            for scheduler in args.schedulers:
                sched_dir = os.path.join(taskset_dir, scheduler)
                os.makedirs(sched_dir, exist_ok=True)
                for inst in range(args.n_inst):
                    # Optional resume: skip if a COMPLETE output already exists
                    # (P1.15 A5: the file must exist AND be non-empty; a 0-byte
                    # or header-only file from a half-written abort is not a
                    # complete run and is re-attempted).
                    if args.resume and _has_complete_metrics(sched_dir, scheduler):
                        if args.verbose >= 1:
                            print(f"    [Resume] Skipping {scheduler} instance {inst}")
                        status_rows.append({
                            "taskset": idx, "scheduler": scheduler, "inst": inst,
                            "status": "RESUMED", "run_log": "",
                        })
                        continue
                    status_rows.append({
                        "taskset": idx, "scheduler": scheduler, "inst": inst,
                        "status": "RUNNING", "run_log": _run_log_path(sched_dir),
                    })
                    fut = executor.submit(
                        run_single_simulation,
                        sim_bin_path, taskset_dir, sched_dir,
                        interval_duration_ms, scheduler, inst,
                        args.verbose, args.export_level,
                    )
                    sim_futures.append((fut, scheduler, inst, sched_dir))

            # Retrieve every future so a worker exception is raised here, in
            # the main thread, naming the (taskset, arm). Stop at the first
            # crash: do not launch or analyze any further arms of this run.
            for fut, scheduler, inst, sched_dir in sim_futures:
                try:
                    fut.result()
                except Exception as exc:
                    # A crashed optimizer binary surfaces as a
                    # subprocess.CalledProcessError (exit 134 = SIGABRT). Any
                    # other exception is treated the same: this arm failed.
                    crashed_arm_this_taskset = {
                        "taskset": idx, "scheduler": scheduler, "inst": inst,
                        "sched_dir": sched_dir,
                        "run_log": _run_log_path(sched_dir),
                        "error": f"{type(exc).__name__}: {exc}",
                    }
                    _mark_status_row(status_rows, idx, scheduler, inst, "CRASHED")
                    if args.verbose >= 1:
                        print(f"  [CRASH] taskset_{idx} / {scheduler} instance "
                              f"{inst} failed: {crashed_arm_this_taskset['error']}")
                    break
                else:
                    _mark_status_row(status_rows, idx, scheduler, inst, "OK")

        # If an arm crashed in this taskset, stop the entire run (do not
        # analyze, do not proceed to the next taskset). The crash report +
        # partial summary are written after the taskset loop.
        if crashed_arm_this_taskset is not None:
            crashed_arms.append(crashed_arm_this_taskset)
            # Mark every arm of the remaining tasksets as NOT_RUN so the
            # status map shows what was skipped.
            for future_idx in range(idx + 1, args.n_tasksets):
                for scheduler in args.schedulers:
                    for inst in range(args.n_inst):
                        status_rows.append({
                            "taskset": future_idx, "scheduler": scheduler,
                            "inst": inst, "status": "NOT_RUN", "run_log": "",
                        })
            break

        if args.verbose >= 1:
            print("  Analyzing results...")

        # 3. Analyze results
        per_ts_results = {}
        for scheduler in args.schedulers:
            for inst in range(args.n_inst):
                try:
                    sched, miss_rate, sp_values_run, run_intervals_data, \
                        avg_sched_time, imp_miss, non_imp_miss = analyze_single_instance(
                            taskset_dir, scheduler, inst, task_deadlines,
                            horizon_granularity,
                            task_sp_weights=task_sp_weights,
                            important_pct=args.important_task_pct,
                        )
                    results_by_scheduler[sched]["miss_rates"].append(miss_rate)
                    results_by_scheduler[sched]["sched_times"].append(
                        avg_sched_time
                    )
                    results_by_scheduler[sched]["important_miss_rates"].append(imp_miss)
                    results_by_scheduler[sched]["non_important_miss_rates"].append(non_imp_miss)
                    for sp_val in sp_values_run:
                        results_by_scheduler[sched]["sp_values"].append(sp_val)
                    for interval, sp_val in run_intervals_data:
                        if interval not in results_by_scheduler[sched]["intervals"]:
                            results_by_scheduler[sched]["intervals"][interval] = []
                        results_by_scheduler[sched]["intervals"][interval].append(
                            sp_val
                        )

                    # Accumulate per-taskset
                    if sched not in per_ts_results:
                        per_ts_results[sched] = {
                            "sp_values": [], "miss_rates": [],
                            "sched_times": [], "important_miss_rates": [],
                            "non_important_miss_rates": []
                        }
                    per_ts_results[sched]["sp_values"].extend(sp_values_run)
                    per_ts_results[sched]["miss_rates"].append(miss_rate)
                    per_ts_results[sched]["sched_times"].append(avg_sched_time)
                    per_ts_results[sched]["important_miss_rates"].append(imp_miss)
                    per_ts_results[sched]["non_important_miss_rates"].append(non_imp_miss)
                except Exception as e:
                    # A non-crashed arm should always have metrics (the sim
                    # future returned 0); if analyze still raises, surface it
                    # loudly as a crash rather than the old silent print+skip.
                    crashed_arms.append({
                        "taskset": idx, "scheduler": scheduler, "inst": inst,
                        "sched_dir": os.path.join(taskset_dir, scheduler),
                        "run_log": _run_log_path(os.path.join(taskset_dir, scheduler)),
                        "error": f"analyze: {type(e).__name__}: {e}",
                    })
                    _mark_status_row(status_rows, idx, scheduler, inst, "CRASHED")
                    if args.verbose >= 1:
                        print(f"  [CRASH] analyze failed for taskset_{idx} / "
                              f"{scheduler} instance {inst}: {e}")
                    break
            else:
                continue
            # Inner break fired (analyze crashed): stop the run.
            for future_idx in range(idx + 1, args.n_tasksets):
                for scheduler in args.schedulers:
                    for inst in range(args.n_inst):
                        status_rows.append({
                            "taskset": future_idx, "scheduler": scheduler,
                            "inst": inst, "status": "NOT_RUN", "run_log": "",
                        })
            break

        results_by_taskset.append(per_ts_results)

    # Clean up temp configs
    shutil.rmtree(temp_dir, ignore_errors=True)

    # P1.15 (layer A, A6): always write the per-(taskset, arm) status map so a
    # crash (or a skipped arm) is visible as a ❌, not buried.
    _write_status_map(output_dir_abs, status_rows)
    if crashed_arms:
        # User requirement (2026-07-19): "as long as there is one crash, then
        # no results are useful anymore, no need to report any numbers, fail it
        # loudly." So on ANY crash we do NOT write comparison_summary.csv or
        # plots — a partial aggregate is worse than none, because it can be
        # mistaken for a complete one. We write only the crash report + status
        # map (the diagnostic artifacts the user needs to locate the crash) and
        # exit non-zero. After the crash is fixed, the whole experiment is
        # re-run from scratch.
        _write_crash_report(output_dir_abs, crashed_arms)
        first = crashed_arms[0]
        msg = (
            f"Optimizer crash stopped the run: taskset_{first['taskset']} / "
            f"{first['scheduler']} instance {first['inst']} failed with "
            f"{first['error']}. No summary/plots written — results are not "
            f"useful once any arm has crashed. See crash_report.txt + "
            f"taskset_arm_status.csv in {output_dir_abs}. Captured binary "
            f"output: {first['run_log']}"
        )
        print(f"\n[CRASH] {msg}", file=sys.stderr)
        sys.exit(msg)

    if args.verbose >= 1:
        print(f"\n{'='*60}")
        print("AGGREGATING RESULTS")
        print(f"{'='*60}")

    # 4. Summary CSV and standard plots
    write_summary_and_plots(results_by_scheduler, args.schedulers,
                            output_dir_abs, horizon_granularity, args.verbose)

    # 5. Additional comparison plots
    if MATPLOTLIB_AVAILABLE:
        plot_optimizer_sp_line(
            results_by_scheduler, args.schedulers,
            os.path.join(output_dir_abs, "optimizer_sp_bar.png")
        )
        plot_optimizer_exec_time_line(
            results_by_scheduler, args.schedulers,
            os.path.join(output_dir_abs, "optimizer_exec_time.png")
        )
        if args.n_tasksets > 1:
            plot_per_taskset_line(
                results_by_taskset, args.schedulers,
                os.path.join(output_dir_abs, "optimizer_per_taskset.png")
            )

    if args.verbose >= 1:
        print(f"\nAll done. Results written to: {output_dir_abs}")


if __name__ == "__main__":
    main()

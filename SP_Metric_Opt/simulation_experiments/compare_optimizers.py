#!/usr/bin/env python3
"""
End-to-end optimizer comparison script for SP Metric Opt scheduling methods.

Generates N random task sets, runs all (or selected) scheduler modes on each
taskset, and produces consolidated CSV summaries plus visualization plots.

Usage:
    python compare_optimizers.py --n_tasksets 10 --num_tasks 6
"""
import argparse
import contextlib
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
from Gen_Taskset.lib.generation_config_parser import load_generation_config
from simulation_experiments.run_sim_experiments import (
    run_single_simulation,
    analyze_single_instance,
    _needs_generation,
    _temporarily_hide_interval_files,
)
from simulation_experiments.utils import (
    compute_miss_rate,
    write_summary_and_plots,
    MATPLOTLIB_AVAILABLE,
)

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
    "INCR", "BF",
    "INCR_NO_TL", "INCR_WCET", "INCR_SCRATCH",
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


def plot_optimizer_bar_comparison(results_by_scheduler, schedulers, output_path,
                                  metric_name="SP Metric", ylabel="Average SP Metric"):
    """Generate a bar chart with error bars (std) for each scheduler."""
    if not MATPLOTLIB_AVAILABLE:
        print("Skipping bar plot (matplotlib not available).")
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
        print("No valid SP data for bar plot.")
        return

    fig, ax = plt.subplots(figsize=(12, 6))
    colors = matplotlib.colormaps["tab10"]
    x = np.arange(len(valid_schedulers))
    bars = ax.bar(
        x, means, yerr=stds, capsize=5,
        color=[colors(i) for i in range(len(valid_schedulers))],
        edgecolor="black"
    )
    ax.set_xticks(x)
    ax.set_xticklabels(valid_schedulers, rotation=15, ha="right")
    ax.set_ylabel(ylabel)
    ax.set_title(f"{metric_name} by Scheduler")
    ax.grid(axis="y", linestyle="--", alpha=0.5)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"Saved bar comparison plot to: {output_path}")


def plot_optimizer_exec_time(results_by_scheduler, schedulers, output_path):
    """Generate a bar chart of average scheduler execution time per mode."""
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
    ax.bar(
        x, means, yerr=stds, capsize=5,
        color=[colors(i) for i in range(len(valid_schedulers))],
        edgecolor="black"
    )
    ax.set_xticks(x)
    ax.set_xticklabels(valid_schedulers, rotation=15, ha="right")
    ax.set_ylabel("Avg. Execution Time (s)")
    ax.set_title("Scheduler Execution Time")
    ax.grid(axis="y", linestyle="--", alpha=0.5)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"Saved execution-time plot to: {output_path}")


def plot_per_taskset_radar(results_by_taskset, schedulers, output_path):
    """Generate a radar / spider chart showing mean SP per scheduler for each taskset."""
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
    width = 0.8 / len(schedulers)
    colors = matplotlib.colormaps["tab10"]

    for i, s in enumerate(schedulers):
        offset = (i - len(schedulers) / 2) * width
        ax.bar(x + offset, data[s], width, label=s, color=colors(i), edgecolor="black")

    ax.set_xlabel("Taskset Index")
    ax.set_ylabel("Average SP Metric")
    ax.set_title("Per-Taskset Average SP Metric by Scheduler")
    ax.set_xticks(x)
    ax.set_xticklabels([f"TS {i}" for i in x])
    ax.legend(loc="upper right")
    ax.grid(axis="y", linestyle="--", alpha=0.5)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"Saved per-taskset plot to: {output_path}")


def resolve_run_output_dir(base_output_dir, run_name, num_tasks, n_sec,
                           scheduler_trigger_interval, base_seed):
    """Return the full output directory path for a comparison run.

    Parameters
    ----------
    base_output_dir : str
        The parent directory (e.g. simulation_experiments/optimizer_comparison).
    run_name : str or None
        Explicit subfolder name. If None, auto-generated from parameters.
    num_tasks, n_sec, scheduler_trigger_interval, base_seed : int
        Parameters used for auto-naming when run_name is None.
    """
    if run_name:
        subfolder = run_name
    else:
        subfolder = (
            f"tasks{num_tasks}_dur{n_sec}_"
            f"interval{scheduler_trigger_interval}_seed{base_seed}"
        )
    return os.path.join(base_output_dir, subfolder)


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
        "--num_tasks", type=int, choices=[4, 6, 8], default=6,
        help="Number of tasks (selects paper_4 / paper_6 / paper_8 config, default: 6)"
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
        "--max_intervals", type=int, default=None,
        help=("If set, simulate at most this many intervals even if the "
              "generated taskset contains more. Useful for fast tests.")
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
        "--skip_generation", action="store_true",
        help="Skip generation if taskset_characteristics files already exist"
    )
    parser.add_argument(
        "--export_level", type=int, default=0, choices=[0, 1, 2, 3],
        help="Export detail level: 0=sp-only, 1=+task miss rate, 2=+task aggregate, 3=full job traces"
    )
    parser.add_argument(
        "--sample_interval", type=int, default=0,
        help="Sample interval metrics every N seconds (0=all intervals)"
    )

    args = parser.parse_args()

    if args.max_intervals is not None and args.max_intervals < 1:
        parser.error("--max_intervals must be >= 1")
    if args.scheduler_trigger_interval < 1:
        parser.error("--scheduler_trigger_interval must be >= 1")

    # Resolve paths
    config_file_abs = os.path.join(
        PROJECT_ROOT,
        f"Gen_Taskset/task_sets_config/taskset_cfg_paper_{args.num_tasks}.json"
    )
    base_output_dir = (
        args.output_dir if args.output_dir.startswith("/")
        else os.path.join(PROJECT_ROOT, args.output_dir)
    )
    # scheduler_trigger_interval (seconds) is the single concept that drives
    # generation, simulation, and plotting.
    scheduler_trigger_interval = args.scheduler_trigger_interval
    simt = scheduler_trigger_interval * 1000  # per-interval duration in ms for C++

    output_dir_abs = resolve_run_output_dir(
        base_output_dir, args.run_name, args.num_tasks, args.n_sec,
        scheduler_trigger_interval, args.base_seed
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
    temp_dir = os.path.join(PROJECT_ROOT, "temp_experiment_configs")
    os.makedirs(temp_dir, exist_ok=True)

    # Plot x-axis spacing equals the scheduler trigger interval (seconds)
    horizon_granularity = scheduler_trigger_interval

    # Data structures
    results_by_scheduler = {
        s: {"sp_values": [], "miss_rates": [], "intervals": {},
            "sched_times": []}
        for s in args.schedulers
    }
    results_by_taskset = []  # per-taskset dict for per-taskset plots

    for idx in range(args.n_tasksets):
        if args.verbose >= 1:
            print(f"\n{'='*60}")
            print(f"TASKSET {idx + 1} / {args.n_tasksets}")
            print(f"{'='*60}")

        taskset_dir = os.path.join(output_dir_abs, f"taskset_{idx}")
        os.makedirs(taskset_dir, exist_ok=True)

        needs_generation = _needs_generation(taskset_dir)

        # Load, seed, and save configuration (resolve INCLUDE so temp file is self-contained)
        config_dict = load_generation_config(config_file_abs)
        config_dict["RANDOM_SEED"] = args.base_seed + idx
        config_dict["UPDATE_INTERVAL_S"] = scheduler_trigger_interval

        with open(os.path.join(taskset_dir, "generator_config.json"), "w") as f:
            json.dump(config_dict, f, indent=4)

        temp_cfg_path = os.path.join(temp_dir, f"temp_cfg_{idx}.json")
        with open(temp_cfg_path, "w") as f:
            json.dump(config_dict, f, indent=4)

        # 1. Generate taskset
        if needs_generation or not args.skip_generation:
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

        # Read task deadlines for miss-rate analysis
        char_fpath = os.path.join(taskset_dir, "taskset_characteristics_0.yaml")
        if not os.path.exists(char_fpath):
            # Fallback to the global characteristics file
            char_fpath = os.path.join(taskset_dir, "taskset_characteristics.yaml")
        with open(char_fpath, "r") as f:
            yaml_data = yaml.safe_load(f)
        task_deadlines = {t["id"]: float(t["deadline"]) for t in yaml_data["tasks"]}

        # Total interval count on disk (for exec-time divisor and max_intervals clamp)
        total_intervals = len(
            glob.glob(os.path.join(taskset_dir, "taskset_characteristics_*.yaml"))
        )
        effective_num_intervals = (
            min(args.max_intervals, total_intervals)
            if args.max_intervals is not None else total_intervals
        )

        # 2. Run each scheduler in parallel
        if args.verbose >= 1:
            print("  Running simulations...")

        sim_futures = []
        with concurrent.futures.ProcessPoolExecutor(max_workers=1) as executor, \
             _temporarily_hide_interval_files(taskset_dir, args.max_intervals):
            for scheduler in args.schedulers:
                sched_dir = os.path.join(taskset_dir, scheduler)
                os.makedirs(sched_dir, exist_ok=True)
                for inst in range(args.n_inst):
                    sim_futures.append(
                        executor.submit(
                            run_single_simulation,
                            sim_bin_path, taskset_dir, sched_dir,
                            simt, scheduler, inst, args.verbose,
                            args.export_level, args.sample_interval,
                        )
                    )
            concurrent.futures.wait(sim_futures)

        if args.verbose >= 1:
            print("  Analyzing results...")

        # 3. Analyze results
        per_ts_results = {}
        for scheduler in args.schedulers:
            for inst in range(args.n_inst):
                try:
                    sched, miss_rate, sp_values_run, run_intervals_data, \
                        avg_sched_time = analyze_single_instance(
                            taskset_dir, scheduler, inst, task_deadlines,
                            horizon_granularity,
                            effective_num_intervals=effective_num_intervals,
                        )
                    results_by_scheduler[sched]["miss_rates"].append(miss_rate)
                    results_by_scheduler[sched]["sched_times"].append(
                        avg_sched_time
                    )
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
                            "sched_times": []
                        }
                    per_ts_results[sched]["sp_values"].extend(sp_values_run)
                    per_ts_results[sched]["miss_rates"].append(miss_rate)
                    per_ts_results[sched]["sched_times"].append(avg_sched_time)
                except Exception as e:
                    print(f"  Error analyzing {scheduler} instance {inst}: {e}")

        results_by_taskset.append(per_ts_results)

    # Clean up temp configs
    shutil.rmtree(temp_dir, ignore_errors=True)

    if args.verbose >= 1:
        print(f"\n{'='*60}")
        print("AGGREGATING RESULTS")
        print(f"{'='*60}")

    # 4. Summary CSV and standard plots
    write_summary_and_plots(results_by_scheduler, args.schedulers,
                            output_dir_abs, horizon_granularity, args.verbose)

    # 5. Additional comparison plots
    if MATPLOTLIB_AVAILABLE:
        plot_optimizer_bar_comparison(
            results_by_scheduler, args.schedulers,
            os.path.join(output_dir_abs, "optimizer_sp_bar.png")
        )
        plot_optimizer_exec_time(
            results_by_scheduler, args.schedulers,
            os.path.join(output_dir_abs, "optimizer_exec_time.png")
        )
        if args.n_tasksets > 1:
            plot_per_taskset_radar(
                results_by_taskset, args.schedulers,
                os.path.join(output_dir_abs, "optimizer_per_taskset.png")
            )

    if args.verbose >= 1:
        print(f"\nAll done. Results written to: {output_dir_abs}")


if __name__ == "__main__":
    main()

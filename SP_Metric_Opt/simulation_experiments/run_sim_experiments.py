import argparse
import contextlib
import glob
import json
import os
import shutil
import subprocess
import sys
import yaml

import concurrent.futures

# Ensure project root is in sys.path for absolute imports
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib.orchestrator import run_full_generation_pipeline
from simulation_experiments.utils import (
    compute_miss_rate,
    write_summary_and_plots,
)


def run_single_simulation(sim_bin_path, taskset_dir, sched_dir,
                          interval_duration_ms, scheduler,
                          inst, verbose=1, export_level=None,
                          sample_interval=None):
    # RunOrchestrator args:
    # <input_folder> <output_folder> <mode> <duration_ms>
    # [export_level] [sample_interval_sec]
    sim_cmd = [
        sim_bin_path,
        taskset_dir,
        sched_dir,
        scheduler,
        str(interval_duration_ms),
    ]
    if export_level is not None:
        sim_cmd.append(str(export_level))
    if sample_interval is not None:
        sim_cmd.append(str(sample_interval))

    if verbose >= 1:
        print(f"  [Sim] Starting {scheduler} instance {inst}...")

    stdout_dest = None if verbose >= 2 else subprocess.DEVNULL
    stderr_dest = None if verbose >= 2 else subprocess.DEVNULL
    subprocess.run(sim_cmd, check=True, stdout=stdout_dest, stderr=stderr_dest)

    if verbose >= 1:
        print(f"  [Sim] Finished {scheduler} instance {inst}.")


def analyze_single_instance(taskset_dir, scheduler, inst, task_deadlines,
                            horizon_granularity, effective_num_intervals=None):
    """Reads interval_sp_metrics.txt from the scheduler output dir and computes miss rate.

    Parameters
    ----------
    effective_num_intervals : int or None
        If provided, use this as the divisor for per-call execution time instead
        of counting ``taskset_characteristics_*.yaml`` files on disk.  This is
        needed when ``--max_intervals`` hides some YAML files from C++.
    """
    sched_dir = os.path.join(taskset_dir, scheduler, scheduler)
    sp_metrics_file = os.path.join(sched_dir, "interval_sp_metrics.txt")
    sp_values_run = []
    run_intervals_data = []

    if os.path.exists(sp_metrics_file):
        with open(sp_metrics_file, "r") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split(",")
                if len(parts) < 2:
                    continue
                try:
                    interval = int(parts[0])
                    sp_val = float(parts[1])
                    sp_values_run.append(sp_val)
                    run_intervals_data.append((interval, sp_val))
                except ValueError:
                    continue

    miss_rate = compute_miss_rate(sched_dir, task_deadlines)

    # Read scheduler execution time if available.
    # C++ writes total process duration; we convert to per-scheduler-call average
    # by dividing by the number of intervals (taskset_characteristics_*.yaml).
    exec_time_file = os.path.join(sched_dir, "scheduler_execution_time.txt")
    total_exec_time = 0.0
    if os.path.exists(exec_time_file):
        with open(exec_time_file, "r") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    total_exec_time = float(line)
                    break
                except ValueError:
                    continue

    # Count intervals to compute per-call average
    if effective_num_intervals is not None:
        num_intervals = effective_num_intervals
    else:
        char_files = glob.glob(os.path.join(taskset_dir, "taskset_characteristics_*.yaml"))
        num_intervals = len(char_files)
    if scheduler == "CFS":
        avg_sched_time = 0.0
    elif num_intervals > 0:
        avg_sched_time = total_exec_time / num_intervals
    else:
        avg_sched_time = total_exec_time

    return scheduler, miss_rate, sp_values_run, run_intervals_data, avg_sched_time


def _needs_generation(taskset_dir):
    """Check if taskset needs to be generated."""
    char_files = glob.glob(os.path.join(taskset_dir,
                                        "taskset_characteristics_*.yaml"))
    return len(char_files) == 0


@contextlib.contextmanager
def _temporarily_hide_interval_files(taskset_dir, max_intervals):
    """Move excess interval characteristics files out of the way so C++ only sees max_intervals.

    If ``max_intervals`` is None, this is a no-op.  If it is set, all
    ``taskset_characteristics_*.yaml`` files whose index >= ``max_intervals``
    are temporarily moved to a ``hidden_intervals/`` subdirectory and restored
    on exit.  A recovery check at entry restores any files left over from a
    previous crashed run.
    """
    if max_intervals is None:
        yield
        return

    hidden_dir = os.path.join(taskset_dir, "hidden_intervals")

    # --- Recovery: restore orphaned files from a previous crash ---
    if os.path.exists(hidden_dir):
        for fpath in glob.glob(
            os.path.join(hidden_dir, "taskset_characteristics_*.yaml")
        ):
            dest = os.path.join(taskset_dir, os.path.basename(fpath))
            if not os.path.exists(dest):
                shutil.move(fpath, dest)
        try:
            os.rmdir(hidden_dir)
        except OSError:
            pass

    os.makedirs(hidden_dir, exist_ok=True)
    moved = []
    for fpath in glob.glob(
        os.path.join(taskset_dir, "taskset_characteristics_*.yaml")
    ):
        fname = os.path.basename(fpath)
        stem = fname.replace("taskset_characteristics_", "").replace(".yaml", "")
        try:
            idx = int(stem)
        except ValueError:
            continue
        if idx >= max_intervals:
            dest = os.path.join(hidden_dir, fname)
            shutil.move(fpath, dest)
            moved.append((dest, fpath))
    try:
        yield
    finally:
        for src, dst in moved:
            if os.path.exists(src):
                shutil.move(src, dst)
        try:
            os.rmdir(hidden_dir)
        except OSError:
            pass


def main():
    parser = argparse.ArgumentParser(
        description="Batch execute simulation experiments and visualize SP-Metric results."
    )
    parser.add_argument(
        "-c", "--config_file", type=str,
        default="Gen_Taskset/task_sets_config/taskset_cfg_paper_6.json",
        help="Path to taskset configuration file"
    )
    parser.add_argument(
        "-o", "--output_dir", type=str,
        default="TaskData/sim_experiments",
        help="Output directory to save experiments data"
    )
    parser.add_argument(
        "-n", "--n_tasksets", type=int, default=5,
        help="Number of task sets to generate and simulate"
    )
    parser.add_argument(
        "-s", "--schedulers", nargs="+",
        default=["INCR", "BF", "RM_FAST", "RM_SLOW"],
        help="List of schedulers to run"
    )
    parser.add_argument(
        "-t", "--n_sec", type=int, default=1000,
        help="GMM trace path duration in seconds"
    )
    parser.add_argument(
        "--n_inst", type=int, default=1,
        help="Number GMM trace instances per path"
    )
    parser.add_argument(
        "--scheduler_trigger_interval", type=int, default=10,
        help=("Interval in seconds between scheduler re-optimizations. "
              "Controls taskset generation (UPDATE_INTERVAL_S) and "
              "the per-interval simulation duration sent to C++ (default: 10)")
    )
    parser.add_argument(
        "--max_intervals", type=int, default=None,
        help=("If set, simulate at most this many intervals even if the "
              "generated taskset contains more. Useful for fast tests.")
    )
    parser.add_argument(
        "--bin_dir", type=str, default="release",
        help="Directory containing release C++ binaries"
    )
    parser.add_argument(
        "--base_seed", type=int, default=100,
        help="Base random seed for generating unique tasksets"
    )
    parser.add_argument(
        "--num_tasks", type=int, choices=[4, 6, 8], default=None,
        help="Number of tasks (4, 6, 8) to automatically select paper config and output folder"
    )
    parser.add_argument(
        "-v", "--verbose", type=int, choices=[0, 1, 2], default=1,
        help=("Verbosity level (0: minimal, 1: progress, "
              "2: debug simulator output)")
    )
    parser.add_argument(
        "--skip_generation", action="store_true",
        help="Skip taskset generation if taskset already exists"
    )
    parser.add_argument(
        "--skip_simulation", action="store_true",
        help="Skip simulation and only generate tasksets"
    )
    parser.add_argument(
        "--export_level", type=int, default=0,
        choices=[0, 1, 2, 3],
        help="Export detail level: 0=sp-only, 1=+task miss rate, "
             "2=+task aggregate, 3=full job traces"
    )
    parser.add_argument(
        "--full_export_taskset", type=int, default=0,
        help="Taskset index that receives export_level=3 regardless of "
             "--export_level (default: 0). Pass -1 to disable."
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

    if args.num_tasks is not None:
        args.config_file = (
            f"Gen_Taskset/task_sets_config/taskset_cfg_paper_{args.num_tasks}.json"
        )

    # Resolve paths
    config_file_abs = (
        args.config_file if args.config_file.startswith("/")
        else os.path.join(PROJECT_ROOT, args.config_file)
    )
    output_dir_abs = (
        args.output_dir if args.output_dir.startswith("/")
        else os.path.join(PROJECT_ROOT, args.output_dir)
    )
    bin_dir_abs = (
        args.bin_dir if args.bin_dir.startswith("/")
        else os.path.join(PROJECT_ROOT, args.bin_dir)
    )

    sim_bin_path = os.path.join(bin_dir_abs, "tests", "RunOrchestrator")

    # scheduler_trigger_interval (seconds) drives generation, simulation, and plots
    scheduler_trigger_interval = args.scheduler_trigger_interval
    interval_duration_ms = scheduler_trigger_interval * 1000  # per-interval duration in ms for C++

    # Verify binary exists
    if not os.path.exists(sim_bin_path):
        print(f"Error: RunOrchestrator binary not found at {sim_bin_path}. "
              f"Compile in release mode first.")
        sys.exit(1)

    if args.verbose >= 1:
        print(f"Starting pipeline run on {args.n_tasksets} tasksets...")
        print(f"Schedulers to evaluate: {args.schedulers}")
        print(f"Scheduler trigger interval: {scheduler_trigger_interval}s")

    # Temporary directory for JSON configs with seeds
    temp_dir = os.path.join(PROJECT_ROOT, "temp_experiment_configs")
    os.makedirs(temp_dir, exist_ok=True)

    # Dictionary to collect results across all tasksets
    results_by_scheduler = {
        s: {"sp_values": [], "miss_rates": [], "intervals": {},
            "sched_times": []}
        for s in args.schedulers
    }
    # Plot x-axis spacing equals the scheduler trigger interval (seconds)
    horizon_granularity = scheduler_trigger_interval

    for idx in range(args.n_tasksets):
        if args.verbose >= 1:
            print(f"\n==================== TASKSET {idx} / "
                  f"{args.n_tasksets - 1} ====================")
        taskset_dir = os.path.join(output_dir_abs, f"taskset_{idx}")
        os.makedirs(taskset_dir, exist_ok=True)

        needs_generation = _needs_generation(taskset_dir)

        # 1. Load configuration and update random seed + trigger interval
        with open(config_file_abs, "r") as f:
            config_dict = json.load(f)
        config_dict["RANDOM_SEED"] = args.base_seed + idx
        config_dict["UPDATE_INTERVAL_S"] = scheduler_trigger_interval

        # Save a copy of the generator config in the taskset folder for records
        with open(os.path.join(taskset_dir, "generator_config.json"), "w") as f:
            json.dump(config_dict, f, indent=4)

        temp_cfg_path = os.path.join(temp_dir, f"temp_cfg_{idx}.json")
        with open(temp_cfg_path, "w") as f:
            json.dump(config_dict, f, indent=4)

        if needs_generation or not args.skip_generation:
            if args.verbose >= 1:
                print("Running taskset generation pipeline...")
            if args.verbose >= 1:
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
                char_files = glob.glob(
                    os.path.join(taskset_dir,
                                 "taskset_characteristics_*.yaml")
                )
                print(f"Taskset {idx} already exists "
                      f"({len(char_files)} characteristic files), "
                      f"skipping generation.")

        if args.skip_simulation:
            if args.verbose >= 1:
                print("  --> Skipping simulation (--skip_simulation).")
            continue

        # Total interval count on disk (used for exec-time divisor and max_intervals clamp)
        total_intervals = len(
            glob.glob(os.path.join(taskset_dir, "taskset_characteristics_*.yaml"))
        )
        effective_num_intervals = (
            min(args.max_intervals, total_intervals)
            if args.max_intervals is not None else total_intervals
        )

        # Read task definitions and deadlines
        char_fpath = os.path.join(taskset_dir, "taskset_characteristics_0.yaml")
        with open(char_fpath, "r") as f:
            yaml_data = yaml.safe_load(f)
        task_deadlines = {t["id"]: float(t["deadline"]) for t in yaml_data["tasks"]}

        # Determine export level for this taskset
        actual_level = (
            3 if (args.full_export_taskset >= 0 and
                  idx == args.full_export_taskset)
            else args.export_level
        )

        # 3. Simulate each scheduler in parallel
        if args.verbose >= 1:
            print("  --> Executing scheduler simulations in parallel...")

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
                            interval_duration_ms, scheduler, inst,
                            args.verbose, actual_level, args.sample_interval,
                        )
                    )
            concurrent.futures.wait(sim_futures)

        if args.verbose >= 1:
            print("  --> All simulations completed. Starting analysis...")

        # 4. Analyze results for each scheduler
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
                except Exception as e:
                    print(f"Error analyzing {scheduler} instance {inst}: {e}")

        if args.verbose >= 1:
            print("  --> Analysis completed.")

    # Clean up temp configs dir
    shutil.rmtree(temp_dir, ignore_errors=True)

    if args.skip_simulation:
        if args.verbose >= 1:
            print("\nGeneration complete. Simulation was skipped.")
        return

    if args.verbose >= 1:
        print("\n==================== SUMMARIZING RESULTS ====================")
    write_summary_and_plots(results_by_scheduler, args.schedulers,
                            output_dir_abs, horizon_granularity, args.verbose)
    if args.verbose >= 1:
        print("\nConsolidated analysis and plots generated.")


if __name__ == "__main__":
    main()

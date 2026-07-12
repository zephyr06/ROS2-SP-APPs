import argparse
import atexit
import contextlib
import glob
import json
import os
import shutil
import subprocess
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
from simulation_experiments.utils import (
    compute_miss_rate,
    compute_miss_rate_by_task,
    compute_important_task_miss_rate,
    write_summary_and_plots,
)


def run_single_simulation(sim_bin_path, taskset_dir, sched_dir,
                          interval_duration_ms, scheduler,
                          inst, verbose=1, export_level=None):
    # RunOrchestrator args:
    # <input_folder> <output_folder> <mode> <duration_ms> [export_level]
    # (sample_interval_sec is left at its C++ default of 0 = write all
    # intervals; no experiment uses a non-zero value.)
    sim_cmd = [
        sim_bin_path,
        taskset_dir,
        sched_dir,
        scheduler,
        str(interval_duration_ms),
    ]
    if export_level is not None:
        sim_cmd.append(str(export_level))

    if verbose >= 1:
        print(f"  [Sim] Starting {scheduler} instance {inst}...")

    stdout_dest = None if verbose >= 2 else subprocess.DEVNULL
    stderr_dest = None if verbose >= 2 else subprocess.DEVNULL
    subprocess.run(sim_cmd, check=True, stdout=stdout_dest, stderr=stderr_dest)

    if verbose >= 1:
        print(f"  [Sim] Finished {scheduler} instance {inst}.")


def analyze_single_instance(taskset_dir, scheduler, inst, task_deadlines,
                            horizon_granularity,
                            task_sp_weights=None, important_pct=0.10, min_important=1):
    """Reads interval_sp_metrics.txt from the scheduler output dir and computes miss rate.

    Parameters
    ----------
    task_sp_weights : dict[int, float] or None
        Mapping from task ID to sp_weight. Required to compute important-task miss rate.
    important_pct : float
        Fraction of tasks considered important (default 0.10).
    min_important : int
        Minimum number of important tasks (default 1).
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

    # Per-task miss rate and important-task miss rate
    important_miss_rate = 0.0
    non_important_miss_rate = 0.0
    if task_sp_weights:
        per_task_mr = compute_miss_rate_by_task(sched_dir, task_deadlines)
        if per_task_mr:
            important_miss_rate, non_important_miss_rate = \
                compute_important_task_miss_rate(
                    per_task_mr, task_sp_weights,
                    important_pct=important_pct, min_important=min_important
                )

    # Read scheduler execution time if available.
    # C++ writes total process duration; we convert to per-scheduler-call average
    # by dividing by the number of intervals (taskset_characteristics_interval_*.yaml).
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

    # Count intervals to compute per-call average. The on-disk interval count
    # is controlled solely by n_sec / scheduler_trigger_interval (removed
    # --max_intervals / _temporarily_hide_interval_files), so this is the true
    # divisor for the total scheduler execution time.
    char_files = glob.glob(
        os.path.join(taskset_dir, "taskset_characteristics_interval_*.yaml")
    )
    num_intervals = len(char_files)
    if scheduler == "CFS":
        avg_sched_time = 0.0
    elif num_intervals > 0:
        avg_sched_time = total_exec_time / num_intervals
    else:
        avg_sched_time = total_exec_time

    return scheduler, miss_rate, sp_values_run, run_intervals_data, avg_sched_time, important_miss_rate, non_important_miss_rate


def _needs_generation(taskset_dir):
    """Check if taskset needs to be generated (no interval files on disk)."""
    char_files = glob.glob(os.path.join(taskset_dir,
                                        "taskset_characteristics_interval_*.yaml"))
    return len(char_files) == 0


def _should_generate(taskset_dir, new_config, idx, skip_if_exists,
                     on_change_policy="prompt", verbose=1):
    """Decide whether taskset ``idx`` should be (re)generated.

    Replaces the old ``needs_generation or not args.skip_generation`` decision
    with config-change detection so that a taskset generated under an older
    generator config is never silently reused when the config has since
    changed (silent staleness).

    The decision tree:

    1. No interval files on disk -> always generate.
    2. ``skip_if_exists`` is False -> forced regenerate.
    3. Taskset exists and reuse is requested -> compare ``new_config`` against
       the ``generator_config.json`` saved alongside the on-disk taskset.
       - No change -> reuse (return False).
       - Change -> apply ``on_change_policy``:
         * ``"prompt"``  : ask [Y/n] interactively (regenerate on yes); when
           stdin is not a TTY (e.g. called as a subprocess by the end-to-end
           orchestrator or interval sweep, where prompting is impossible),
           fall back to regenerating with a warning -- correctness over silent
           staleness.
         * ``"regenerate"``: always regenerate on a change.
         * ``"keep"``    : reuse the stale taskset with a warning (explicit
           opt-out for cheap re-runs).

    Parameters
    ----------
    taskset_dir : str
        Per-taskset output directory (holds ``generator_config.json`` and the
        ``taskset_characteristics_interval_*.yaml`` files).
    new_config : dict
        The fully-resolved generator config that *would* be used to generate
        this taskset (source config + ``RANDOM_SEED = base_seed + idx`` +
        ``UPDATE_INTERVAL_S = scheduler_trigger_interval``). Compared by raw
        dict equality -- ``load_generation_config`` is deterministic, so
        old == new iff source config + seed + interval are unchanged.
    idx : int
        Taskset index (used for the prompt message only).
    skip_if_exists : bool
        Whether to reuse an existing taskset at all.
    on_change_policy : str
        One of ``"prompt"``, ``"regenerate"``, ``"keep"`` (default ``"prompt"``).
    verbose : int
        Verbosity level (warnings print at verbose >= 1).

    Returns
    -------
    bool
        True if the taskset should be (re)generated, False to reuse as-is.
    """
    if _needs_generation(taskset_dir):
        return True
    if not skip_if_exists:
        return True

    saved_path = os.path.join(taskset_dir, "generator_config.json")
    old_config = None
    if os.path.exists(saved_path):
        try:
            with open(saved_path, "r") as f:
                old_config = json.load(f)
        except (OSError, ValueError):
            old_config = None

    if old_config == new_config:
        return False  # config matches what generated the on-disk taskset -> reuse

    # Config changed: apply the configured policy.
    policy = on_change_policy
    if policy == "regenerate":
        if verbose >= 1:
            print(f"  Config for taskset {idx} changed; regenerating "
                  f"(on_taskset_config_change='regenerate').")
        return True
    if policy == "keep":
        if verbose >= 1:
            print(f"  Warning: config for taskset {idx} changed but "
                  f"on_taskset_config_change='keep'; reusing stale taskset.")
        return False
    # Default: "prompt".
    if sys.stdin.isatty():
        try:
            ans = input(
                f"Config for taskset {idx} changed. Regenerate? [Y/n] "
            )
        except EOFError:
            ans = ""
        return ans.strip().lower() not in ("n", "no")
    # Non-interactive session (subprocess from e2e/sweep): cannot prompt.
    if verbose >= 1:
        print(f"  Warning: config for taskset {idx} changed in a non-interactive "
              f"session; regenerating (on_taskset_config_change='prompt' fallback).")
    return True


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
        default=["INCR_Reopt_10", "BF", "RM_FAST", "RM_SLOW"],
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
        "--bin_dir", type=str, default="release",
        help="Directory containing release C++ binaries"
    )
    parser.add_argument(
        "--base_seed", type=int, default=100,
        help="Base random seed for generating unique tasksets"
    )
    parser.add_argument(
        "--num_tasks", type=int, default=None,
        help=("Number of tasks to automatically select/synthesize paper config "
              "and output folder. On-disk configs exist for 4/6/8; any N >= 1 "
              "is supported (config synthesized on the fly for other values; "
              "P17 allows N_BIG=0 / N_SMALL=0).")
    )
    parser.add_argument(
        "-v", "--verbose", type=int, choices=[0, 1, 2], default=1,
        help=("Verbosity level (0: minimal, 1: progress, "
              "2: debug simulator output)")
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
        "--skip_simulation", action="store_true",
        help="Skip simulation and only generate tasksets"
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
        "--export_level", type=int, default=1,
        choices=[0, 1, 2, 3],
        help="Export detail level: 0=sp-only, 1=+task miss rate, "
             "2=+task aggregate, 3=full job traces"
    )
    parser.add_argument(
        "--full_export_taskset", type=int, default=0,
        help="Taskset index that receives export_level=3 regardless of "
             "--export_level (default: 0). Pass -1 to disable."
    )

    args = parser.parse_args()

    if args.scheduler_trigger_interval < 1:
        parser.error("--scheduler_trigger_interval must be >= 1")
    if args.num_tasks is not None and args.num_tasks < 1:
        parser.error("--num_tasks must be >= 1 (P17: N_BIG=0 / N_SMALL=0 are allowed)")

    if args.num_tasks is not None:
        # Resolve (or synthesize) the paper config for this task count; works
        # for any N >= 1 (P17), not just 4/6/8.
        config_file_abs = resolve_taskset_config_path(args.num_tasks)
    else:
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
    temp_dir = tempfile.mkdtemp(prefix="temp_experiment_configs_", dir=PROJECT_ROOT)
    atexit.register(shutil.rmtree, temp_dir, ignore_errors=True)

    # Dictionary to collect results across all tasksets
    results_by_scheduler = {
        s: {
            "sp_values": [], "miss_rates": [], "intervals": {},
            "sched_times": [], "important_miss_rates": [],
            "non_important_miss_rates": []
        }
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

        # 1. Load configuration and update random seed + trigger interval.
        # Use load_generation_config (not raw json.load) so INCLUDE is resolved
        # consistently with compare_optimizers.py -- required now that configs
        # may be synthesized temp files relying on INCLUDE.
        config_dict = load_generation_config(config_file_abs)
        config_dict["RANDOM_SEED"] = args.base_seed + idx
        config_dict["UPDATE_INTERVAL_S"] = scheduler_trigger_interval

        # Decide whether to (re)generate this taskset. _should_generate compares
        # ``config_dict`` against the generator_config.json saved alongside the
        # on-disk taskset (if any), so a taskset generated under an older config
        # is never silently reused when the config has since changed. The
        # baseline file is written below only when we actually generate --
        # writing it here unconditionally would clobber the staleness baseline.
        should_generate = _should_generate(
            taskset_dir, config_dict, idx,
            skip_if_exists=args.skip_generation_if_exists,
            on_change_policy=args.on_taskset_config_change,
            verbose=args.verbose,
        )

        temp_cfg_path = os.path.join(temp_dir, f"temp_cfg_{idx}.json")
        with open(temp_cfg_path, "w") as f:
            json.dump(config_dict, f, indent=4)

        if should_generate:
            # Record the config that generated this taskset as the staleness
            # baseline for future runs' _should_generate comparison.
            with open(os.path.join(taskset_dir, "generator_config.json"), "w") as f:
                json.dump(config_dict, f, indent=4)
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
                                 "taskset_characteristics_interval_*.yaml")
                )
                print(f"Taskset {idx} already exists "
                      f"({len(char_files)} characteristic files), "
                      f"skipping generation.")

        if args.skip_simulation:
            if args.verbose >= 1:
                print("  --> Skipping simulation (--skip_simulation).")
            continue

        # Read task definitions, deadlines, and sp_weights
        char_fpath = os.path.join(taskset_dir, "taskset_characteristics_interval_0.yaml")
        with open(char_fpath, "r") as f:
            yaml_data = yaml.safe_load(f)
        task_deadlines = {t["id"]: float(t["deadline"]) for t in yaml_data["tasks"]}
        task_sp_weights = {t["id"]: float(t.get("sp_weight", 0.0)) for t in yaml_data["tasks"]}

        # Determine export level for this taskset
        actual_level = (
            3 if (args.full_export_taskset >= 0 and
                  idx == args.full_export_taskset)
            else args.export_level
        )

        # Determine number of worker processes
        num_workers = args.num_workers
        if num_workers is None:
            num_workers = min(4, os.cpu_count() or 1)

        # 3. Simulate each scheduler in parallel. The on-disk interval count
        # is controlled solely by n_sec / scheduler_trigger_interval (removed
        # --max_intervals), so C++ sees exactly the generated intervals.
        if args.verbose >= 1:
            print("  --> Executing scheduler simulations in parallel...")

        sim_futures = []
        with concurrent.futures.ProcessPoolExecutor(max_workers=num_workers) as executor:
            for scheduler in args.schedulers:
                sched_dir = os.path.join(taskset_dir, scheduler)
                os.makedirs(sched_dir, exist_ok=True)
                for inst in range(args.n_inst):
                    # Optional: skip if --resume and output already exists
                    if args.resume:
                        expected_output = os.path.join(sched_dir, scheduler, "interval_sp_metrics.txt")
                        if os.path.exists(expected_output):
                            if args.verbose >= 1:
                                print(f"    [Resume] Skipping {scheduler} instance {inst} (output exists).")
                            continue
                    sim_futures.append(
                        executor.submit(
                            run_single_simulation,
                            sim_bin_path, taskset_dir, sched_dir,
                            interval_duration_ms, scheduler, inst,
                            args.verbose, actual_level,
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

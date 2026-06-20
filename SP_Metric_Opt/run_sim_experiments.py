import os
import sys
import argparse
import json
import shutil
import random
import subprocess
import yaml
import numpy as np
import matplotlib.pyplot as plt

# Ensure project root is in sys.path for absolute imports
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib.orchestrator import run_full_generation_pipeline

def parse_sp_value_from_stdout(stdout: str) -> float:
    """Parses SP-Metric value from AnalyzeSP_Metric C++ tool output."""
    for line in stdout.splitlines():
        if "SP-Metric:" in line:
            parts = line.strip().split()
            if len(parts) >= 2:
                try:
                    return float(parts[1])
                except ValueError:
                    pass
    raise ValueError(f"Could not parse SP-Metric from output: {stdout}")

def compute_miss_rate(sim_res_file: str, task_deadlines: dict) -> float:
    """Computes deadline miss rate from simulation results file."""
    total_jobs = 0
    missed_jobs = 0
    if not os.path.exists(sim_res_file):
        return 0.0
    with open(sim_res_file, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            if len(parts) < 5:
                continue
            taskid = int(parts[0])
            start_time = float(parts[2])
            end_time = float(parts[3])
            rt = end_time - start_time
            ddl = task_deadlines.get(taskid, 0.0)
            total_jobs += 1
            if rt > ddl:
                missed_jobs += 1
    return missed_jobs / total_jobs if total_jobs > 0 else 0.0

def combine_processor_results(sched_dir, scheduler, inst, n_cores, yaml_tasks, res_file, log_file):
    combined_res_lines = []
    combined_log_lines = []
    
    # Map local to global task id
    local_to_global = {}
    for p in range(n_cores):
        local_id = 0
        for i, task_data in enumerate(yaml_tasks):
            if task_data.get('processorId', 0) == p:
                local_to_global[(p, local_id)] = i
                local_id += 1

    for p in range(n_cores):
        p_res = os.path.join(sched_dir, f"sim_res_{scheduler}_{inst}_p{p}.txt")
        p_log = os.path.join(sched_dir, f"sim_log_{scheduler}_{inst}_p{p}.txt")
        if os.path.exists(p_res):
            with open(p_res, 'r') as f:
                for line in f:
                    parts = line.strip().split(',')
                    if len(parts) >= 5:
                        l_id = int(parts[0])
                        global_id = local_to_global.get((p, l_id), l_id)
                        parts[0] = str(global_id)
                        combined_res_lines.append(",".join(parts) + "\n")
        if os.path.exists(p_log):
            with open(p_log, 'r') as f:
                combined_log_lines.extend(f.readlines())
    
    with open(res_file, 'w') as f:
        f.writelines(combined_res_lines)
    with open(log_file, 'w') as f:
        f.writelines(combined_log_lines)

def run_single_simulation(sim_bin_path, taskset_dir, sched_dir, simt, scheduler, inst, verbose=1):
    sim_cmd = [
        sim_bin_path,
        "--input_folder", taskset_dir,
        "--output_folder", sched_dir,
        "--simt", str(simt),
        "--scheduler", scheduler,
        "--output_job_not_executed", "1",
        "--inst_idx", str(inst),
        "--verbose", "0"
    ]
    # Print a quick progress message
    if verbose >= 1:
        print(f"  [Sim] Starting {scheduler} instance {inst}...")
    
    stdout_dest = None if verbose >= 2 else subprocess.DEVNULL
    stderr_dest = None if verbose >= 2 else subprocess.DEVNULL
    subprocess.run(sim_cmd, check=True, stdout=stdout_dest, stderr=stderr_dest)
    
    if verbose >= 1:
        print(f"  [Sim] Finished {scheduler} instance {inst}.")

def analyze_single_instance(taskset_dir, scheduler, inst, config_dict, yaml_data, task_deadlines, analyze_bin_path, horizon_granularity):
    sched_dir = os.path.join(taskset_dir, scheduler)
    res_file = os.path.join(sched_dir, f"sim_res_{scheduler}_{inst}.txt")
    log_file = os.path.join(sched_dir, f"sim_log_{scheduler}_{inst}.txt")
    
    # Combine processor-specific results/logs into standard file names
    n_cores = config_dict.get("N_CORES", 1)
    combine_processor_results(sched_dir, scheduler, inst, n_cores, yaml_data['tasks'], res_file, log_file)

    # Copy / rename to structured folder targets as per task description
    if os.path.exists(res_file):
        shutil.copy(res_file, os.path.join(sched_dir, f"response_times_{inst}.csv"))
    if os.path.exists(log_file):
        shutil.copy(log_file, os.path.join(sched_dir, f"simulation_log_{inst}.txt"))

    # Compute Miss Rate
    miss_rate = compute_miss_rate(res_file, task_deadlines)

    # Parse response time entries per interval and compute SP-Metric
    rst_dict = {t['id']: {} for t in yaml_data['tasks']}
    max_interval_idx = -1
    max_s = 0.0

    if os.path.exists(res_file):
        with open(res_file, 'r') as f:
            for line in f:
                parts = line.strip().split(',')
                if len(parts) < 5:
                    continue
                taskid = int(parts[0])
                jobid = int(parts[1])
                start_time = float(parts[2]) / 1000.0
                end_time = float(parts[3]) / 1000.0
                execution_time = float(parts[4]) / 1000.0
                if end_time > max_s:
                    max_s = end_time

                idx = int(start_time / horizon_granularity)
                if idx > max_interval_idx:
                    max_interval_idx = idx
                if idx not in rst_dict[taskid]:
                    rst_dict[taskid][idx] = []
                rst_dict[taskid][idx].append([jobid, start_time, end_time, execution_time])

    temp_sp_calc_dir = os.path.join(sched_dir, f"temp_sp_{inst}")
    os.makedirs(temp_sp_calc_dir, exist_ok=True)

    sp_values_run = []
    run_intervals_data = []
    for interval in range(max_interval_idx + 1):
        # Write response times and execution times for each task for this interval
        for taskid in rst_dict:
            task_name = yaml_data['tasks'][taskid]['name']
            resp_file_path = os.path.join(temp_sp_calc_dir, f"{task_name}_response_time.txt")
            
            with open(resp_file_path, 'w') as rf:
                if interval in rst_dict[taskid]:
                    for job in rst_dict[taskid][interval]:
                        rf.write(f"{(job[2] - job[1]) * 1000.0}\n")
            
            if 'performance_records_time' in yaml_data['tasks'][taskid]:
                exe_file_path = os.path.join(temp_sp_calc_dir, f"{task_name}_execution_time.txt")
                with open(exe_file_path, 'w') as ef:
                    if interval in rst_dict[taskid]:
                        for job in rst_dict[taskid][interval]:
                            ef.write(f"{job[3] * 1000.0}\n")

        # Run C++ evaluation executable
        interval_cfg_file = os.path.join(taskset_dir, f"taskset_characteristics_{interval}.yaml")
        if not os.path.exists(interval_cfg_file):
            interval_cfg_file = os.path.join(taskset_dir, "taskset_characteristics_0.yaml")

        cmd = [
            analyze_bin_path,
            "--file_path", interval_cfg_file,
            "--data_dir", temp_sp_calc_dir
        ]
        res = subprocess.run(cmd, capture_output=True, text=True)
        try:
            sp_val = parse_sp_value_from_stdout(res.stdout)
            sp_values_run.append(sp_val)
            run_intervals_data.append((interval, sp_val))
        except Exception as e:
            print(f"Warning: failed to evaluate interval {interval} for scheduler {scheduler}: {e}")

    # Save calculated SP values for this run
    sp_val_run_fpath = os.path.join(sched_dir, f"sp_values_{inst}.txt")
    with open(sp_val_run_fpath, 'w') as sf:
        for val in sp_values_run:
            sf.write(f"{val}\n")

    # Clean up instance calculation temp folders
    shutil.rmtree(temp_sp_calc_dir, ignore_errors=True)

    # Read average scheduler execution times and remove temp files
    sched_times = []
    for p in range(n_cores):
        time_file = os.path.join(sched_dir, f"sched_exe_time_{scheduler}_{inst}_p{p}.txt")
        if os.path.exists(time_file):
            with open(time_file, 'r') as tf:
                try:
                    val = float(tf.read().strip())
                    sched_times.append(val)
                except ValueError:
                    pass
            try:
                os.remove(time_file)
            except OSError:
                pass
    avg_sched_time = np.mean(sched_times) if len(sched_times) > 0 else 0.0

    return scheduler, miss_rate, sp_values_run, run_intervals_data, avg_sched_time

def write_summary_and_plots(results_by_scheduler, schedulers, output_dir_abs, horizon_granularity, verbose=1):
    # Output Consolidated CSV Summary
    summary_csv_path = os.path.join(output_dir_abs, "comparison_summary.csv")
    if verbose >= 1:
        print(f"Writing summary statistics to: {summary_csv_path}")
    with open(summary_csv_path, 'w') as csv_file:
        csv_file.write("Scheduler,Mean_SP_Metric,Std_SP_Metric,Mean_Miss_Rate,Std_Miss_Rate,Mean_Scheduler_Execution_Time_s\n")
        for scheduler in schedulers:
            s_data = results_by_scheduler[scheduler]
            sp_arr = np.array(s_data['sp_values'])
            miss_arr = np.array(s_data['miss_rates'])
            sched_arr = np.array(s_data.get('sched_times', []))

            mean_sp = np.mean(sp_arr) if len(sp_arr) > 0 else 0.0
            std_sp = np.std(sp_arr) if len(sp_arr) > 0 else 0.0
            mean_miss = np.mean(miss_arr) if len(miss_arr) > 0 else 0.0
            std_miss = np.std(miss_arr) if len(miss_arr) > 0 else 0.0
            mean_sched = np.mean(sched_arr) if len(sched_arr) > 0 else 0.0

            csv_file.write(f"{scheduler},{mean_sp:.6f},{std_sp:.6f},{mean_miss:.6f},{std_miss:.6f},{mean_sched:.6f}\n")
            print(f"Scheduler {scheduler}: SP = {mean_sp:.4f} ± {std_sp:.4f}, Miss Rate = {mean_miss*100:.2f}% ± {std_miss*100:.2f}%, Avg Sched Time = {mean_sched:.6f}s")

    # Generate Consolidated Comparison Plots
    plots_path = os.path.join(output_dir_abs, "comparison_plots.png")
    if verbose >= 1:
        print(f"Generating comparison plots at: {plots_path}")
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    # Boxplot of SP-Metric distributions
    sp_boxplot_data = [results_by_scheduler[s]['sp_values'] for s in schedulers]
    ax1.boxplot(sp_boxplot_data, labels=schedulers)
    ax1.set_ylabel("SP-Metric Value")
    ax1.set_title("Safety-Performance Metric Distribution")
    ax1.grid(True, linestyle='--', alpha=0.5)

    # Average Line plot over intervals (adaptation over time)
    for scheduler in schedulers:
        intervals_dict = results_by_scheduler[scheduler]['intervals']
        sorted_intervals = sorted(intervals_dict.keys())
        x_vals = [i * horizon_granularity for i in sorted_intervals]
        y_vals = [np.mean(intervals_dict[i]) for i in sorted_intervals]
        ax2.plot(x_vals, y_vals, label=scheduler, marker='o', markersize=4, linewidth=1.5)
    
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Average SP-Metric Value")
    ax2.set_title("Adaptation over Path Intervals")
    ax2.legend()
    ax2.grid(True, linestyle='--', alpha=0.5)

    plt.tight_layout()
    plt.savefig(plots_path, dpi=300)
    plt.close()

def main():
    parser = argparse.ArgumentParser(description="Batch execute simulation experiments and visualize SP-Metric results.")
    parser.add_argument("-c", "--config_file", type=str, default="Gen_Taskset/task_sets_config/taskset_cfg_paper_6.json",
                        help="Path to taskset configuration file")
    parser.add_argument("-o", "--output_dir", type=str, default="TaskData/sim_experiments",
                        help="Output directory to save experiments data")
    parser.add_argument("-n", "--n_tasksets", type=int, default=5,
                        help="Number of task sets to generate and simulate")
    parser.add_argument("-s", "--schedulers", nargs="+", default=["INCR", "INCR_SWAP", "BR", "RM_FAST", "RM_SLOW", "CFS"],
                        help="List of schedulers to run")
    parser.add_argument("-t", "--n_sec", type=int, default=1000,
                        help="GMM trace path duration in seconds")
    parser.add_argument("--n_inst", type=int, default=8,
                        help="Number GMM trace instances per path")
    parser.add_argument("--simt", type=int, default=1000000,
                        help="C++ simulation execution time in ms")
    parser.add_argument("--bin_dir", type=str, default="release",
                        help="Directory containing release C++ binaries")
    parser.add_argument("--base_seed", type=int, default=100,
                        help="Base random seed for generating unique tasksets")
    parser.add_argument("--num_tasks", type=int, choices=[4, 6, 8], default=None,
                        help="Number of tasks (4, 6, 8) to automatically select paper config and output folder")
    parser.add_argument("-v", "--verbose", type=int, choices=[0, 1, 2], default=1,
                        help="Verbosity level (0: minimal/silent, 1: progress prints, 2: debug C++ simulator output)")
    args = parser.parse_args()

    if args.num_tasks is not None:
        args.config_file = f"Gen_Taskset/task_sets_config/taskset_cfg_paper_{args.num_tasks}.json"
        args.output_dir = f"TaskData/experiment_{args.num_tasks}_tasks"

    # Resolve paths
    config_file_abs = args.config_file if args.config_file.startswith('/') else os.path.join(PROJECT_ROOT, args.config_file)
    output_dir_abs = args.output_dir if args.output_dir.startswith('/') else os.path.join(PROJECT_ROOT, args.output_dir)
    bin_dir_abs = args.bin_dir if args.bin_dir.startswith('/') else os.path.join(PROJECT_ROOT, args.bin_dir)

    sim_bin_path = os.path.join(bin_dir_abs, "tests", "CSPSimulation_2")
    analyze_bin_path = os.path.join(bin_dir_abs, "tests", "AnalyzeSP_Metric")

    # Verify binaries exist
    if not os.path.exists(sim_bin_path):
        print(f"Error: CSPSimulation_2 binary not found at {sim_bin_path}. Compile in release mode first.")
        sys.exit(1)
    if not os.path.exists(analyze_bin_path):
        print(f"Error: AnalyzeSP_Metric binary not found at {analyze_bin_path}. Compile in release mode first.")
        sys.exit(1)

    if args.verbose >= 1:
        print(f"Starting pipeline run on {args.n_tasksets} tasksets...")
        print(f"Schedulers to evaluate: {args.schedulers}")

    # Set TIME_LIMIT in sources/parameters.yaml temporarily for speedup
    param_yaml_path = os.path.join(PROJECT_ROOT, "sources/parameters.yaml")
    original_yaml_content = None
    if os.path.exists(param_yaml_path):
        with open(param_yaml_path, 'r') as f:
            original_yaml_content = f.read()
        import re
        try:
            with open(config_file_abs, 'r') as f:
                cfg_json = json.load(f)
            num_tasks = cfg_json.get("N_BIG_PERIOD_TASKS", 0) + cfg_json.get("N_SMALL_PERIOD_TASKS", 0)
        except Exception:
            num_tasks = 6
        
        limit_val = 1 if num_tasks >= 8 else 2
        modified_content = re.sub(r'TIME_LIMIT:\s*\d+', f'TIME_LIMIT: {limit_val}', original_yaml_content)
        with open(param_yaml_path, 'w') as f:
            f.write(modified_content)
        if args.verbose >= 1:
            print(f"Temporarily adjusted TIME_LIMIT in parameters.yaml to {limit_val}s for simulation speedup.")
        
        import atexit
        def restore_yaml():
            with open(param_yaml_path, 'w') as f:
                f.write(original_yaml_content)
            if args.verbose >= 1:
                print("Restored original parameters.yaml content.")
        atexit.register(restore_yaml)

    # Temporary directory for JSON configs with seeds
    temp_dir = os.path.join(PROJECT_ROOT, "temp_experiment_configs")
    os.makedirs(temp_dir, exist_ok=True)

    # Dictionary to collect results across all tasksets
    results_by_scheduler = {s: {'sp_values': [], 'miss_rates': [], 'intervals': {}, 'sched_times': []} for s in args.schedulers}
    horizon_granularity = 10 # 10-second intervals

    for idx in range(args.n_tasksets):
        if args.verbose >= 1:
            print(f"\n==================== TASKSET {idx} / {args.n_tasksets-1} ====================")
        taskset_dir = os.path.join(output_dir_abs, f"taskset_{idx}")
        os.makedirs(taskset_dir, exist_ok=True)

        # 1. Load configuration and update random seed
        with open(config_file_abs, 'r') as f:
            config_dict = json.load(f)
        config_dict['RANDOM_SEED'] = args.base_seed + idx
        
        # Save a copy of the generator config in the taskset folder for records
        with open(os.path.join(taskset_dir, "generator_config.json"), 'w') as f:
            json.dump(config_dict, f, indent=4)

        temp_cfg_path = os.path.join(temp_dir, f"temp_cfg_{idx}.json")
        with open(temp_cfg_path, 'w') as f:
            json.dump(config_dict, f, indent=4)

        # 2. Run GMM generation pipeline
        if args.verbose >= 1:
            print("Running taskset generation pipeline...")
            run_full_generation_pipeline(
                cfg_file=temp_cfg_path,
                n_sec=args.n_sec,
                dir_path=taskset_dir,
                add_perf_records=True,
                interact=False,
                n_path_per_task=1,
                n_inst_per_path=args.n_inst
            )
        else:
            import contextlib
            with open(os.devnull, 'w') as devnull:
                with contextlib.redirect_stdout(devnull), contextlib.redirect_stderr(devnull):
                    run_full_generation_pipeline(
                        cfg_file=temp_cfg_path,
                        n_sec=args.n_sec,
                        dir_path=taskset_dir,
                        add_perf_records=True,
                        interact=False,
                        n_path_per_task=1,
                        n_inst_per_path=args.n_inst
                    )

        # Read task definitions and deadlines
        char_fpath = os.path.join(taskset_dir, "taskset_characteristics_0.yaml")
        with open(char_fpath, 'r') as f:
            yaml_data = yaml.safe_load(f)
        task_deadlines = {t['id']: float(t['deadline']) for t in yaml_data['tasks']}

        # 3. Simulate and analyze each scheduler in parallel
        import concurrent.futures

        # Ensure all scheduler directories are created
        for scheduler in args.schedulers:
            sched_dir = os.path.join(taskset_dir, scheduler)
            os.makedirs(sched_dir, exist_ok=True)

        # Run all simulations in parallel
        if args.verbose >= 1:
            print("  --> Executing scheduler simulations in parallel...")
        sim_futures = []
        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as executor:
            for scheduler in args.schedulers:
                for inst in range(args.n_inst):
                    sched_dir = os.path.join(taskset_dir, scheduler)
                    sim_futures.append(executor.submit(run_single_simulation, sim_bin_path, taskset_dir, sched_dir, args.simt, scheduler, inst, args.verbose))
            # Wait for all simulations to complete
            concurrent.futures.wait(sim_futures)
        if args.verbose >= 1:
            print("  --> All simulations completed. Starting analysis...")

        # Run all analyses in parallel
        if args.verbose >= 1:
            print("  --> Analyzing simulation results in parallel...")
        analysis_futures = []
        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as executor:
            for scheduler in args.schedulers:
                for inst in range(args.n_inst):
                    analysis_futures.append(executor.submit(analyze_single_instance, taskset_dir, scheduler, inst, config_dict, yaml_data, task_deadlines, analyze_bin_path, horizon_granularity))
            
            # Gather and aggregate results as they complete
            for fut in concurrent.futures.as_completed(analysis_futures):
                try:
                    scheduler, miss_rate, sp_values_run, run_intervals_data, avg_sched_time = fut.result()
                    results_by_scheduler[scheduler]['miss_rates'].append(miss_rate)
                    results_by_scheduler[scheduler]['sched_times'].append(avg_sched_time)
                    for sp_val in sp_values_run:
                        results_by_scheduler[scheduler]['sp_values'].append(sp_val)
                    for interval, sp_val in run_intervals_data:
                        if interval not in results_by_scheduler[scheduler]['intervals']:
                            results_by_scheduler[scheduler]['intervals'][interval] = []
                        results_by_scheduler[scheduler]['intervals'][interval].append(sp_val)
                except Exception as e:
                    print(f"Error during analysis future execution: {e}")
        if args.verbose >= 1:
            print("  --> Analysis completed.")

    # Clean up temp configs dir
    shutil.rmtree(temp_dir, ignore_errors=True)

    if args.verbose >= 1:
        print("\n==================== SUMMARIZING RESULTS ====================")
    write_summary_and_plots(results_by_scheduler, args.schedulers, output_dir_abs, horizon_granularity, args.verbose)
    if args.verbose >= 1:
        print("\nConsolidated analysis and plots generated.")

if __name__ == "__main__":
    main()

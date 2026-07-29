#!/usr/bin/env python3
import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import numpy as np
import yaml

from simulation_experiments.utils import compute_miss_rate

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


def reanalyze_experiment(exp_dir, schedulers, horizon_granularity=10):
    taskset_dirs = sorted([
        d for d in os.listdir(exp_dir)
        if d.startswith("taskset_") and os.path.isdir(os.path.join(exp_dir, d))
    ])

    results_by_scheduler = {
        s: {"sp_values": [], "miss_rates": [], "intervals": {}, "sched_times": []}
        for s in schedulers
    }

    for td in taskset_dirs:
        taskset_dir = os.path.join(exp_dir, td)
        char_files = [
            f for f in os.listdir(taskset_dir)
            if f.startswith("taskset_characteristics_interval_") and f.endswith(".yaml")
        ]
        if not char_files:
            print(f"Skipping {td}, no characteristics file")
            continue
        char_fpath = os.path.join(taskset_dir, char_files[0])
        with open(char_fpath, "r") as f:
            yaml_data = yaml.safe_load(f)
        task_deadlines = {t["id"]: float(t["deadline"]) for t in yaml_data["tasks"]}

        for scheduler in schedulers:
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
            else:
                print(f"  Missing {sp_metrics_file}")

            miss_rate = compute_miss_rate(sched_dir, task_deadlines)
            results_by_scheduler[scheduler]["miss_rates"].append(miss_rate)
            for sp_val in sp_values_run:
                results_by_scheduler[scheduler]["sp_values"].append(sp_val)
            for interval, sp_val in run_intervals_data:
                if interval not in results_by_scheduler[scheduler]["intervals"]:
                    results_by_scheduler[scheduler]["intervals"][interval] = []
                results_by_scheduler[scheduler]["intervals"][interval].append(sp_val)

    # Summary CSV
    summary_csv_path = os.path.join(exp_dir, "comparison_summary.csv")
    with open(summary_csv_path, "w") as csv_file:
        csv_file.write(
            "Scheduler,Mean_SP_Metric,Std_SP_Metric,Mean_Miss_Rate,Std_Miss_Rate,Mean_Scheduler_Execution_Time_s\n"
        )
        for scheduler in schedulers:
            s_data = results_by_scheduler[scheduler]
            sp_arr = np.array(s_data["sp_values"])
            miss_arr = np.array(s_data["miss_rates"])
            sched_arr = np.array(s_data.get("sched_times", []))

            mean_sp = np.mean(sp_arr) if len(sp_arr) > 0 else 0.0
            std_sp = np.std(sp_arr) if len(sp_arr) > 0 else 0.0
            mean_miss = np.mean(miss_arr) if len(miss_arr) > 0 else 0.0
            std_miss = np.std(miss_arr) if len(miss_arr) > 0 else 0.0
            mean_sched = np.mean(sched_arr) if len(sched_arr) > 0 else 0.0

            csv_file.write(
                f"{scheduler},{mean_sp:.6f},{std_sp:.6f},{mean_miss:.6f},{std_miss:.6f},{mean_sched:.6f}\n"
            )
            print(
                f"Scheduler {scheduler}: SP = {mean_sp:.4f} ± {std_sp:.4f}, "
                f"Miss Rate = {mean_miss * 100:.2f}% ± {std_miss * 100:.2f}%, "
                f"Avg Sched Time = {mean_sched:.6f}s"
            )

    if not MATPLOTLIB_AVAILABLE:
        return

    # Plots
    plots_path = os.path.join(exp_dir, "comparison_plots.png")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    sp_boxplot_data = [results_by_scheduler[s]["sp_values"] for s in schedulers]
    ax1.boxplot(sp_boxplot_data, labels=schedulers)
    ax1.set_ylabel("SP-Metric Value")
    ax1.set_title("Safety-Performance Metric Distribution")
    ax1.grid(True, linestyle="--", alpha=0.5)

    for scheduler in schedulers:
        intervals_dict = results_by_scheduler[scheduler]["intervals"]
        sorted_intervals = sorted(intervals_dict.keys())
        x_vals = [i * horizon_granularity for i in sorted_intervals]
        y_vals = [np.mean(intervals_dict[i]) for i in sorted_intervals]
        ax2.plot(x_vals, y_vals, label=scheduler,
                 marker="o", markersize=4, linewidth=1.5)

    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Average SP-Metric Value")
    ax2.set_title("Adaptation over Path Intervals")
    ax2.legend()
    ax2.grid(True, linestyle="--", alpha=0.5)

    plt.tight_layout()
    plt.savefig(plots_path, dpi=300)
    plt.close()
    print(f"Plots saved to {plots_path}")


if __name__ == "__main__":
    schedulers = ["INCR", "INCR_SWAP", "BF", "DM_FAST", "DM_SLOW"]
    for num_tasks in [4, 6, 8]:
        exp_dir = os.path.join(
            PROJECT_ROOT, "Gen_Taskset", "simulation_tasksets",
            f"experiment_{num_tasks}_tasks",
        )
        if not os.path.exists(exp_dir):
            print(f"Skipping {num_tasks} tasks (directory not found)")
            continue
        print(f"\n===== Re-analyzing {num_tasks} tasks =====")
        reanalyze_experiment(exp_dir, schedulers)

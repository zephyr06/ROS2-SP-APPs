#!/usr/bin/env python3
"""
End-to-end evaluation script for SP Metric Opt scheduling methods.

Usage:
    python run_e2e_eval.py

Generates N random task sets (each with task_number=6), runs all 8 scheduler
modes, and reports final SP values plus scheduler execution times.
"""
import os
import sys
import shutil
import subprocess
import json
import csv
import time

# Add workspace to path to import Gen_Taskset library
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_SCRIPT_DIR, "../.."))
sys.path.append(_REPO_ROOT)

from Gen_Taskset.lib.orchestrator import run_full_generation_pipeline

# Matplotlib imports with graceful fallback
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Warning: matplotlib/numpy not available. Plots will not be generated.")


def parse_interval_sp_metrics(output_dir, mode):
    """Parse interval_sp_metrics.txt to get per-interval SP values."""
    metrics_path = os.path.join(output_dir, mode, "interval_sp_metrics.txt")
    values = []
    if os.path.exists(metrics_path):
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


def run_single_mode(input_dir, output_dir, mode, duration_ms=10000):
    """Run the C++ RunOrchestrator for a single mode and measure wall-clock time."""
    binary_path = os.path.join(_REPO_ROOT, "build/tests/RunOrchestrator")
    cmd = [binary_path, input_dir, output_dir, mode, str(duration_ms)]
    print(f"    [$] {' '.join(cmd)}")

    start_t = time.perf_counter()
    result = subprocess.run(cmd, capture_output=True, text=True)
    elapsed_ms = (time.perf_counter() - start_t) * 1000.0

    if result.returncode != 0:
        print(f"    ERROR: {mode} failed with code {result.returncode}")
        print(f"    stderr: {result.stderr}")
        return None, elapsed_ms

    values = parse_interval_sp_metrics(output_dir, mode)
    if not values:
        print(f"    WARNING: No interval metrics found for {mode}")
        return None, elapsed_ms

    return values, elapsed_ms


def write_csv_summary(results, output_path):
    """Write a CSV summary of all results."""
    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Mode", "Mean_SP", "Std_SP", "Min_SP", "Max_SP",
            "Num_Intervals", "Mean_Exec_Time_ms"
        ])
        for mode, data in results.items():
            if data is None or data["sp_values"] is None:
                writer.writerow([mode, "N/A", "N/A", "N/A", "N/A", 0, "N/A"])
                continue
            sp_vals = data["sp_values"]
            exec_times = data["exec_times_ms"]
            arr = np.array(sp_vals) if MATPLOTLIB_AVAILABLE else sp_vals
            mean_val = arr.mean() if MATPLOTLIB_AVAILABLE else sum(sp_vals) / len(sp_vals)
            std_val = arr.std() if MATPLOTLIB_AVAILABLE else 0.0
            mean_exec = sum(exec_times) / len(exec_times) if exec_times else 0.0
            writer.writerow([
                mode,
                f"{mean_val:.6f}",
                f"{std_val:.6f}",
                f"{min(sp_vals):.6f}",
                f"{max(sp_vals):.6f}",
                len(sp_vals),
                f"{mean_exec:.2f}",
            ])
    print(f"Written CSV summary to: {output_path}")


def plot_comparison(results, output_path):
    """Generate a side-by-side bar chart and box plot of SP metrics per mode."""
    if not MATPLOTLIB_AVAILABLE:
        print("Skipping plot generation (matplotlib not available).")
        return

    valid_modes = {
        m: v for m, v in results.items()
        if v is not None and v["sp_values"] is not None
    }
    if not valid_modes:
        print("No valid data to plot.")
        return

    fig, (ax_bar, ax_box) = plt.subplots(1, 2, figsize=(14, 6))

    modes = list(valid_modes.keys())
    means = [np.mean(valid_modes[m]["sp_values"]) for m in modes]
    stds = [np.std(valid_modes[m]["sp_values"]) for m in modes]

    colors = matplotlib.colormaps["tab10"]
    x = np.arange(len(modes))
    bars = ax_bar.bar(
        x, means, yerr=stds, capsize=5,
        color=[colors(i) for i in range(len(modes))],
        edgecolor="black"
    )
    ax_bar.set_xticks(x)
    ax_bar.set_xticklabels(modes, rotation=15, ha="right")
    ax_bar.set_ylabel("Average SP Metric")
    ax_bar.set_title("Average SP Metric by Scheduling Mode")
    ax_bar.grid(axis="y", linestyle="--", alpha=0.5)

    # Box plot
    data_for_box = [valid_modes[m]["sp_values"] for m in modes]
    bp = ax_box.boxplot(data_for_box, labels=modes, patch_artist=True)
    for patch, color in zip(bp["boxes"], [colors(i) for i in range(len(modes))]):
        patch.set_facecolor(color)
    ax_box.set_ylabel("SP Metric Value")
    ax_box.set_title("SP Metric Distribution by Scheduling Mode")
    ax_box.grid(axis="y", linestyle="--", alpha=0.5)
    plt.setp(ax_box.get_xticklabels(), rotation=15, ha="right")

    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"Saved comparison plot to: {output_path}")


def plot_exec_time(results, output_path):
    """Bar chart of average scheduler execution time per mode."""
    if not MATPLOTLIB_AVAILABLE:
        return

    valid_modes = {
        m: v for m, v in results.items()
        if v is not None and v["exec_times_ms"]
    }
    if not valid_modes:
        return

    fig, ax = plt.subplots(figsize=(10, 6))
    modes = list(valid_modes.keys())
    means = [np.mean(valid_modes[m]["exec_times_ms"]) for m in modes]
    stds = [np.std(valid_modes[m]["exec_times_ms"]) for m in modes]

    colors = matplotlib.colormaps["tab10"]
    x = np.arange(len(modes))
    ax.bar(x, means, yerr=stds, capsize=5,
           color=[colors(i) for i in range(len(modes))],
           edgecolor="black")
    ax.set_xticks(x)
    ax.set_xticklabels(modes, rotation=15, ha="right")
    ax.set_ylabel("Avg. Execution Time (ms)")
    ax.set_title("Scheduler Execution Time by Mode")
    ax.grid(axis="y", linestyle="--", alpha=0.5)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"Saved execution-time plot to: {output_path}")


def main():
    print("=== End-to-End Evaluation: 6-task tasksets, 300s duration, 5 runs, 8 modes ===")

    # 1. Setup folders
    base_dir = os.path.join(_REPO_ROOT, "tests/e2e_eval_runs")
    if os.path.exists(base_dir):
        shutil.rmtree(base_dir)
    os.makedirs(base_dir, exist_ok=True)

    config_template_path = os.path.join(_REPO_ROOT, "Gen_Taskset/task_sets_config/taskset_cfg_6_1_sixtasks.json")

    modes = [
        "RM", "BF", "INCR",
        "INCR_NO_TL", "INCR_WCET", "INCR_SCRATCH",
        "RM_FAST", "RM_SLOW",
    ]

    # Data structures: per-run lists
    results_by_run = {
        mode: {"sp_values": [], "exec_times_ms": []}
        for mode in modes
    }

    num_runs = 5
    duration_ms = 10000

    for run_idx in range(num_runs):
        print(f"\n--- Run {run_idx + 1} / {num_runs} ---")
        run_input_dir = os.path.join(base_dir, f"run_{run_idx}_input")
        os.makedirs(run_input_dir, exist_ok=True)

        # Generation pipeline (1 path/task, 1 instance)
        run_full_generation_pipeline(
            cfg_file=config_template_path,
            n_sec=300,
            dir_path=run_input_dir,
            add_perf_records=True,
            interact=False,
            n_path_per_task=1,
            n_inst_per_path=1
        )

        run_output_dir = os.path.join(base_dir, f"run_{run_idx}_output")
        os.makedirs(run_output_dir, exist_ok=True)

        # Verify task count from generated YAML
        char_yaml = os.path.join(run_input_dir, "taskset_characteristics.yaml")
        if os.path.exists(char_yaml):
            import yaml
            with open(char_yaml, "r") as f:
                data = yaml.safe_load(f)
            n_tasks = len(data.get("tasks", []))
            print(f"    Generated taskset with {n_tasks} tasks")

        for mode in modes:
            values, elapsed_ms = run_single_mode(
                run_input_dir, run_output_dir, mode, duration_ms
            )
            if values is not None:
                avg_sp = sum(values) / len(values)
                print(f"    [{mode}] Avg SP: {avg_sp:.4f}  |  "
                      f"Exec time: {elapsed_ms:.1f} ms  |  "
                      f"Intervals: {len(values)}")
                results_by_run[mode]["sp_values"].append(values)
                results_by_run[mode]["exec_times_ms"].append(elapsed_ms)

    # Aggregate per-mode across all runs
    print("\n" + "=" * 70)
    print(f"{'Mode':<12} | {'Avg SP':<10} | {'Std SP':<10} | "
          f"{'#Int':<6} | {'Avg Exec(ms)':<14}")
    print("-" * 70)

    aggregated = {}
    for mode in modes:
        all_sp = []
        for run_values in results_by_run[mode]["sp_values"]:
            all_sp.extend(run_values)

        exec_times = results_by_run[mode]["exec_times_ms"]

        if not all_sp:
            print(f"{mode:<12} | {'N/A':<10} | {'N/A':<10} | "
                  f"{'0':<6} | {'N/A':<14}")
            aggregated[mode] = None
            continue

        if MATPLOTLIB_AVAILABLE:
            arr = np.array(all_sp)
            mean_sp = arr.mean()
            std_sp = arr.std()
        else:
            mean_sp = sum(all_sp) / len(all_sp)
            std_sp = 0.0

        mean_exec = sum(exec_times) / len(exec_times) if exec_times else 0.0
        aggregated[mode] = {
            "sp_values": all_sp,
            "exec_times_ms": exec_times,
        }

        print(f"{mode:<12} | {mean_sp:<10.4f} | {std_sp:<10.4f} | "
              f"{len(all_sp):<6} | {mean_exec:<14.2f}")

    # Write summary CSV
    csv_path = os.path.join(base_dir, "e2e_summary.csv")
    write_csv_summary(aggregated, csv_path)

    # Generate plots
    if MATPLOTLIB_AVAILABLE:
        plot_path = os.path.join(base_dir, "e2e_sp_comparison.png")
        plot_comparison(aggregated, plot_path)

        exec_path = os.path.join(base_dir, "e2e_exec_time.png")
        plot_exec_time(aggregated, exec_path)

    print("\n=== Done ===")


if __name__ == "__main__":
    main()

"""Shared utilities for simulation experiment scripts."""
import os
import glob

from simulation_experiments.plotting_config import save_figure

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


def compute_miss_rate(response_dir, task_deadlines):
    """Computes deadline miss rate.

    If ``miss_rate_summary.txt`` exists (produced by export detail level 0+),
    returns the overall miss rate directly.
    Otherwise, falls back to scanning ``response_times_task_*.txt`` files.
    """
    if not os.path.exists(response_dir):
        return 0.0

    # Fast path: use C++ pre-computed summary if available
    summary_path = os.path.join(response_dir, "miss_rate_summary.txt")
    if os.path.exists(summary_path):
        with open(summary_path, "r") as f:
            header = next(f, None)
            line = next(f, None)
            if line:
                parts = line.strip().split(",")
                if len(parts) >= 3:
                    try:
                        return float(parts[2])
                    except ValueError:
                        pass

    # Fallback: compute from per-task response-time trace files
    total_jobs = 0
    missed_jobs = 0
    for fname in os.listdir(response_dir):
        if not fname.startswith("response_times_task_"):
            continue
        fpath = os.path.join(response_dir, fname)
        try:
            task_id = int(
                fname.replace("response_times_task_", "").replace(".txt", "")
            )
        except ValueError:
            continue
        ddl = task_deadlines.get(task_id, 0.0)
        with open(fpath, "r") as f:
            next(f, None)  # skip header
            for line in f:
                parts = line.strip().split(",")
                if len(parts) < 6:
                    continue
                total_jobs += 1
                try:
                    rt = float(parts[4])
                except ValueError:
                    continue
                if rt > ddl:
                    missed_jobs += 1
    return missed_jobs / total_jobs if total_jobs > 0 else 0.0


def compute_miss_rate_by_task(response_dir, task_deadlines):
    """Return per-task miss rate as a dict ``{task_id: miss_rate}``.

    Tries to read ``miss_rate_per_task.txt`` (export detail level 1+).
    Falls back to ``response_times_task_*.txt`` files (export detail level 3).
    """
    per_task_mr = {}
    if not os.path.exists(response_dir):
        return per_task_mr

    # Fast path: miss_rate_per_task.txt (level 1+)
    per_task_path = os.path.join(response_dir, "miss_rate_per_task.txt")
    if os.path.exists(per_task_path):
        with open(per_task_path, "r") as f:
            next(f, None)  # skip header
            for line in f:
                parts = line.strip().split(",")
                if len(parts) < 4:
                    continue
                try:
                    task_id = int(parts[0])
                    mr = float(parts[3])
                    per_task_mr[task_id] = mr
                except ValueError:
                    continue
        return per_task_mr

    # Fallback: response_times_task_*.txt (level 3)
    for fname in os.listdir(response_dir):
        if not fname.startswith("response_times_task_"):
            continue
        fpath = os.path.join(response_dir, fname)
        try:
            task_id = int(
                fname.replace("response_times_task_", "").replace(".txt", "")
            )
        except ValueError:
            continue
        ddl = task_deadlines.get(task_id, 0.0)
        t_total = 0
        t_missed = 0
        with open(fpath, "r") as f:
            next(f, None)
            for line in f:
                parts = line.strip().split(",")
                if len(parts) < 6:
                    continue
                t_total += 1
                try:
                    rt = float(parts[4])
                except ValueError:
                    continue
                if rt > ddl:
                    t_missed += 1
        per_task_mr[task_id] = t_missed / t_total if t_total > 0 else 0.0

    return per_task_mr


def compute_important_task_miss_rate(
    per_task_miss_rate, task_weights, important_pct=0.10, min_important=1
):
    """Compute mean miss rate for the most important tasks.

    Parameters
    ----------
    per_task_miss_rate : dict[int, float]
        Mapping from task ID to miss rate.
    task_weights : dict[int, float]
        Mapping from task ID to ``sp_weight``.
    important_pct : float
        Fraction of tasks to label as important (default 0.10).
    min_important : int
        Minimum number of important tasks (at least 1).

    Returns
    -------
    tuple[float, float]
        ``(important_miss_rate, non_important_miss_rate)``. If no per-task
        data exists, both values are ``0.0``.
    """
    if not per_task_miss_rate or not task_weights:
        return 0.0, 0.0

    # Sort tasks by sp_weight descending
    sorted_tasks = sorted(
        task_weights.keys(), key=lambda t: task_weights[t], reverse=True
    )
    n_important = max(min_important, int(len(sorted_tasks) * important_pct + 0.9999))
    important_tasks = set(sorted_tasks[:n_important])
    non_important_tasks = set(sorted_tasks[n_important:])

    important_rates = [per_task_miss_rate[t] for t in important_tasks
                       if t in per_task_miss_rate]
    non_important_rates = [per_task_miss_rate[t] for t in non_important_tasks
                           if t in per_task_miss_rate]

    imp_rate = sum(important_rates) / len(important_rates) if important_rates else 0.0
    non_imp_rate = sum(non_important_rates) / len(non_important_rates) if non_important_rates else 0.0
    return imp_rate, non_imp_rate


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


def write_summary_and_plots(results_by_scheduler, schedulers, output_dir_abs,
                            horizon_granularity=10, verbose=1):
    """Write consolidated CSV summary and generate comparison plots."""
    summary_csv_path = os.path.join(output_dir_abs, "comparison_summary.csv")
    if verbose >= 1:
        print(f"Writing summary statistics to: {summary_csv_path}")
    with open(summary_csv_path, "w") as csv_file:
        csv_file.write(
            "Scheduler,Mean_SP_Metric,Std_SP_Metric,Mean_Miss_Rate,"
            "Std_Miss_Rate,Mean_Scheduler_Execution_Time_s,"
            "Important_Miss_Rate,Non_Important_Miss_Rate\n"
        )
        for scheduler in schedulers:
            s_data = results_by_scheduler[scheduler]
            sp_arr = np.array(s_data["sp_values"])
            miss_arr = np.array(s_data["miss_rates"])
            sched_arr = np.array(s_data.get("sched_times", []))
            imp_arr = np.array(s_data.get("important_miss_rates", []))
            non_imp_arr = np.array(s_data.get("non_important_miss_rates", []))

            mean_sp = np.mean(sp_arr) if len(sp_arr) > 0 else 0.0
            std_sp = np.std(sp_arr) if len(sp_arr) > 0 else 0.0
            mean_miss = np.mean(miss_arr) if len(miss_arr) > 0 else 0.0
            std_miss = np.std(miss_arr) if len(miss_arr) > 0 else 0.0
            mean_sched = np.mean(sched_arr) if len(sched_arr) > 0 else 0.0
            mean_imp = np.mean(imp_arr) if len(imp_arr) > 0 else 0.0
            mean_non_imp = np.mean(non_imp_arr) if len(non_imp_arr) > 0 else 0.0

            csv_file.write(
                f"{scheduler},{mean_sp:.6f},{std_sp:.6f},"
                f"{mean_miss:.6f},{std_miss:.6f},{mean_sched:.6f},"
                f"{mean_imp:.6f},{mean_non_imp:.6f}\n"
            )
            if verbose >= 1:
                print(
                    f"Scheduler {scheduler}: SP = {mean_sp:.4f} ± {std_sp:.4f}, "
                    f"Miss Rate = {mean_miss * 100:.2f}% ± {std_miss * 100:.2f}%, "
                    f"Important Miss = {mean_imp * 100:.2f}%, "
                    f"Non-Important Miss = {mean_non_imp * 100:.2f}%, "
                    f"Avg Sched Time = {mean_sched:.6f}s"
                )

    if not MATPLOTLIB_AVAILABLE:
        return

    plots_stem = os.path.join(output_dir_abs, "comparison_plots")
    if verbose >= 1:
        print(f"Generating comparison plots at: {plots_stem}.{{png,pdf}}")

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
    # Save both PNG and PDF (vector for paper inclusion).
    save_figure(fig, plots_stem)
    plt.close(fig)

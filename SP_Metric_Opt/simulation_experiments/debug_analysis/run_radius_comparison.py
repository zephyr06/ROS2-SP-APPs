#!/usr/bin/env python3
"""
Comparison experiment for INCR radius = {2, 3, 4} plus all baseline schedulers.

Reports:
  - Average SP metric per mode (and per radius for INCR)
  - Scheduler execution time
  - CSV summary + matplotlib plots

How it works
------------
The C++ binary reads TimeLimitSearchRadiusIncr from sources/parameters.yaml
at startup.  To compare radii without recompiling, the script:
  1. Backs up the original YAML.
  2. Overwrites it with the desired radius value.
  3. Runs the binary.
  4. Restores the original YAML.
"""
import os
import sys
import shutil
import subprocess
import time

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_SCRIPT_DIR, "../../.."))
sys.path.append(_REPO_ROOT)

from Gen_Taskset.lib.orchestrator import run_full_generation_pipeline

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Warning: matplotlib/numpy not available. Plots will not be generated.")

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
PARAMS_YAML = os.path.join(_REPO_ROOT, "sources/parameters.yaml")
PARAMS_BACKUP = os.path.join(_REPO_ROOT, "sources/parameters.yaml.bak")

BASE_MODES = [
    "RM",
    "BF",
    "RM_FAST",
    "RM_SLOW",
]

INCR_RADII = [2, 3, 4, 5, 6]
INCR_ABLATIONS = ["INCR", "INCR_NO_TL", "INCR_WCET", "INCR_SCRATCH"]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def override_radius_in_yaml(new_radius: int):
    """Replace TimeLimitSearchRadiusIncr in parameters.yaml (backup first)."""
    if not os.path.exists(PARAMS_BACKUP):
        shutil.copy(PARAMS_YAML, PARAMS_BACKUP)

    with open(PARAMS_YAML, "r") as f:
        lines = f.readlines()

    with open(PARAMS_YAML, "w") as f:
        for line in lines:
            if line.strip().startswith("TimeLimitSearchRadiusIncr"):
                f.write(f"TimeLimitSearchRadiusIncr: {new_radius}\n")
            else:
                f.write(line)


def restore_original_yaml():
    """Restore the backed-up parameters.yaml."""
    if os.path.exists(PARAMS_BACKUP):
        shutil.copy(PARAMS_BACKUP, PARAMS_YAML)
        os.remove(PARAMS_BACKUP)


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


def run_scheduler(input_dir, output_dir, mode, duration_ms=10000):
    """Run the C++ RunOrchestrator binary and measure wall-clock time."""
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


# ---------------------------------------------------------------------------
# CSV / plots
# ---------------------------------------------------------------------------


def write_csv_summary(rows, output_path):
    """rows: list of dicts with keys Mode, Radius, Avg_SP, Std_SP, Avg_Exec_ms."""
    with open(output_path, "w", newline="") as f:
        import csv
        writer = csv.writer(f)
        writer.writerow([
            "Mode", "Radius", "Avg_SP", "Std_SP",
            "Min_SP", "Max_SP", "Num_Intervals", "Avg_Exec_Time_ms"
        ])
        for r in rows:
            writer.writerow([
                r["mode"],
                r["radius"],
                f"{r['avg_sp']:.6f}",
                f"{r['std_sp']:.6f}",
                f"{r['min_sp']:.6f}",
                f"{r['max_sp']:.6f}",
                r["n_intervals"],
                f"{r['avg_exec_ms']:.2f}",
            ])
    print(f"Written CSV summary to: {output_path}")


def plot_sp_comparison(rows, output_path):
    """Bar chart of average SP per mode (grouped by radius for INCR)."""
    if not MATPLOTLIB_AVAILABLE:
        return

    # Extract labels and means
    labels = []
    means = []
    stds = []
    colors = []
    cmap = matplotlib.colormaps["tab10"]

    for r in rows:
        if r["radius"] is not None:
            labels.append(f"{r['mode']}\n(R={r['radius']})")
        else:
            labels.append(r["mode"])
        means.append(r["avg_sp"])
        stds.append(r["std_sp"])
        colors.append(cmap(len(colors) % 10))

    fig, ax = plt.subplots(figsize=(max(10, len(labels) * 0.8), 6))
    x = np.arange(len(labels))
    ax.bar(x, means, yerr=stds, capsize=4, color=colors, edgecolor="black")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=30, ha="right")
    ax.set_ylabel("Average SP Metric")
    ax.set_title("Average SP Metric by Scheduling Mode")
    ax.grid(axis="y", linestyle="--", alpha=0.5)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"Saved SP comparison plot to: {output_path}")


def plot_exec_time_comparison(rows, output_path):
    """Bar chart of average execution time per mode."""
    if not MATPLOTLIB_AVAILABLE:
        return

    labels = []
    exec_times = []
    colors = []
    cmap = matplotlib.colormaps["tab10"]

    for r in rows:
        if r["radius"] is not None:
            labels.append(f"{r['mode']}\n(R={r['radius']})")
        else:
            labels.append(r["mode"])
        exec_times.append(r["avg_exec_ms"])
        colors.append(cmap(len(colors) % 10))

    fig, ax = plt.subplots(figsize=(max(10, len(labels) * 0.8), 6))
    x = np.arange(len(labels))
    ax.bar(x, exec_times, color=colors, edgecolor="black")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=30, ha="right")
    ax.set_ylabel("Avg. Execution Time (ms)")
    ax.set_title("Scheduler Execution Time by Mode")
    ax.grid(axis="y", linestyle="--", alpha=0.5)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"Saved exec-time comparison plot to: {output_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    # Ensure backup is cleaned up from any previous failed run
    restore_original_yaml()

    base_dir = os.path.join(_REPO_ROOT, "tests/radius_comparison")
    if os.path.exists(base_dir):
        shutil.rmtree(base_dir)
    os.makedirs(base_dir, exist_ok=True)

    cfg_file = os.path.join(_REPO_ROOT, "Gen_Taskset/task_sets_config/taskset_cfg_6_1_sixtasks.json")
    num_runs = 5
    duration_ms = 10000

    # Results: list of dicts
    all_rows = []

    for run_idx in range(num_runs):
        print(f"\n========== RUN {run_idx + 1} / {num_runs} ==========")
        run_input_dir = os.path.join(base_dir, f"run_{run_idx}_input")
        os.makedirs(run_input_dir, exist_ok=True)

        # Generate one taskset for this run
        run_full_generation_pipeline(
            cfg_file=cfg_file,
            n_sec=300,
            dir_path=run_input_dir,
            add_perf_records=True,
            interact=False,
            n_path_per_task=1,
            n_inst_per_path=1,
        )

        run_output_dir = os.path.join(base_dir, f"run_{run_idx}_output")
        os.makedirs(run_output_dir, exist_ok=True)

        # ---- Base modes (single run, no radius games) ----
        for mode in BASE_MODES:
            values, elapsed_ms = run_scheduler(run_input_dir, run_output_dir, mode, duration_ms)
            if values is not None:
                avg_sp = sum(values) / len(values)
                print(f"    [{mode}] Avg SP: {avg_sp:.4f}  |  Exec: {elapsed_ms:.1f} ms  |  Intervals: {len(values)}")

                all_rows.append({
                    "run": run_idx,
                    "mode": mode,
                    "radius": None,
                    "sp_values": values,
                    "exec_ms": elapsed_ms,
                })

        # ---- INCR variants for each radius ----
        for radius in INCR_RADII:
            override_radius_in_yaml(radius)
            print(f"\n--- INCR variants with TimeLimitSearchRadiusIncr = {radius} ---")

            for mode in INCR_ABLATIONS:
                values, elapsed_ms = run_scheduler(run_input_dir, run_output_dir, mode, duration_ms)
                if values is not None:
                    avg_sp = sum(values) / len(values)
                    print(f"    [{mode}  R={radius}] Avg SP: {avg_sp:.4f}  |  Exec: {elapsed_ms:.1f} ms  |  Intervals: {len(values)}")

                    all_rows.append({
                        "run": run_idx,
                        "mode": f"{mode}_R{radius}",
                        "radius": radius,
                        "sp_values": values,
                        "exec_ms": elapsed_ms,
                    })

            # Restore YAML immediately after radius block so other code reads the real file
            restore_original_yaml()

    # -----------------------------------------------------------------------
    # Aggregate across runs
    # -----------------------------------------------------------------------
    restore_original_yaml()  # Final safety

    # Group by effective mode label
    from collections import defaultdict
    grouped = defaultdict(lambda: {"sp_values": [], "exec_times": []})
    for r in all_rows:
        key = r["mode"]
        grouped[key]["sp_values"].extend(r["sp_values"])
        grouped[key]["exec_times"].append(r["exec_ms"])

    summary_rows = []
    for mode, data in grouped.items():
        sp_vals = data["sp_values"]
        exec_times = data["exec_times"]
        if not sp_vals:
            continue

        arr = np.array(sp_vals) if MATPLOTLIB_AVAILABLE else sp_vals
        avg_sp = arr.mean() if MATPLOTLIB_AVAILABLE else sum(sp_vals) / len(sp_vals)
        std_sp = arr.std() if MATPLOTLIB_AVAILABLE else 0.0
        avg_exec = sum(exec_times) / len(exec_times) if exec_times else 0.0

        # Extract radius back out for display
        radius = None
        if "_R" in mode:
            try:
                radius = int(mode.split("_R")[-1])
            except ValueError:
                pass

        summary_rows.append({
            "mode": mode,
            "radius": radius,
            "avg_sp": avg_sp,
            "std_sp": std_sp,
            "min_sp": min(sp_vals),
            "max_sp": max(sp_vals),
            "n_intervals": len(sp_vals),
            "avg_exec_ms": avg_exec,
        })

    # Sort: base modes first, then INCR by radius
    def sort_key(r):
        if r["radius"] is None:
            return (0, r["mode"], 0)
        else:
            # put INCR first, then NO_TL, then WCET; order by radius ascending
            base = r["mode"].split("_R")[0]
            order = {"INCR": 1, "INCR_NO_TL": 2, "INCR_WCET": 3}
            return (1, order.get(base, 99), r["radius"])

    summary_rows.sort(key=sort_key)

    # -----------------------------------------------------------------------
    # Print table
    # -----------------------------------------------------------------------
    print("\n" + "=" * 85)
    print(f"{'Mode':<20} | {'Radius':<6} | {'Avg SP':<10} | {'Std SP':<10} | {'#Int':<6} | {'Avg Exec(ms)':<14}")
    print("-" * 85)
    for r in summary_rows:
        radius_str = str(r["radius"]) if r["radius"] is not None else "N/A"
        print(f"{r['mode']:<20} | {radius_str:<6} | {r['avg_sp']:<10.4f} | {r['std_sp']:<10.4f} | "
              f"{r['n_intervals']:<6} | {r['avg_exec_ms']:<14.2f}")
    print("=" * 85)

    # -----------------------------------------------------------------------
    # Persist
    # -----------------------------------------------------------------------
    csv_path = os.path.join(base_dir, "radius_comparison_summary.csv")
    write_csv_summary(summary_rows, csv_path)

    if MATPLOTLIB_AVAILABLE:
        sp_plot = os.path.join(base_dir, "radius_comparison_sp.png")
        plot_sp_comparison(summary_rows, sp_plot)

        exec_plot = os.path.join(base_dir, "radius_comparison_exec.png")
        plot_exec_time_comparison(summary_rows, exec_plot)

    print("\n=== Done ===")


if __name__ == "__main__":
    try:
        main()
    finally:
        restore_original_yaml()

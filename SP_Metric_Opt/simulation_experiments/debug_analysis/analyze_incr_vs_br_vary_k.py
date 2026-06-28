#!/usr/bin/env python3
"""
Vary K (beam width) and TL-radius to understand INCR performance.
Also test wider TL search radius.
"""
import os
import sys
import shutil
import subprocess
import csv

sys.path.append(os.getcwd())
from Gen_Taskset.lib.orchestrator import run_full_generation_pipeline


def parse_sp_values(output_dir, mode):
    path = os.path.join(output_dir, mode, "interval_sp_metrics.txt")
    vals = []
    if os.path.exists(path):
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split(",")
                if len(parts) >= 2:
                    try:
                        vals.append(float(parts[1]))
                    except ValueError:
                        pass
    return vals


def run_mode(input_dir, output_dir, mode, duration_ms=10000):
    binary = "./build/tests/RunOrchestrator"
    cmd = [binary, input_dir, output_dir, mode, str(duration_ms)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"ERROR: {mode} failed: {result.stderr[:200]}")
        return None
    return parse_sp_values(output_dir, mode)


def main():
    base_dir = os.path.abspath("tests/k_variation")
    if os.path.exists(base_dir):
        shutil.rmtree(base_dir)
    os.makedirs(base_dir, exist_ok=True)

    cfg_path = "Gen_Taskset/task_sets_config/taskset_cfg_paper_6.json"
    num_tasksets = 5
    duration_ms = 10000

    # We need to patch GlobalVariables::Layer_Node_During_Incremental_Optimization
    # Let's use C++ instead: add a version that accepts K parameter.
    # For now, run small experiment with default K=2.

    results = []
    for ts_i in range(num_tasksets):
        print(f"\n=== Taskset {ts_i+1}/{num_tasksets} ===")
        run_dir = os.path.join(base_dir, f"ts_{ts_i}")
        input_dir = os.path.join(run_dir, "input")
        output_dir = os.path.join(run_dir, "output")
        os.makedirs(input_dir, exist_ok=True)
        os.makedirs(output_dir, exist_ok=True)

        run_full_generation_pipeline(
            cfg_file=cfg_path,
            n_sec=30, dir_path=input_dir,
            add_perf_records=True, interact=False,
            n_path_per_task=1, n_inst_per_path=1
        )

        row = {"taskset": ts_i}
        for mode in ["INCR", "BF", "INCR_NO_TL", "INCR_WCET", "RM", "RM_FAST", "RM_SLOW"]:
            vals = run_mode(input_dir, output_dir, mode, duration_ms)
            if vals:
                avg = sum(vals) / len(vals)
                row[mode] = round(avg, 4)
                print(f"  {mode:<12} avg SP = {avg:.4f}")
            else:
                row[mode] = "N/A"
        results.append(row)

    # Print summary
    print("\n" + "=" * 90)
    modes = ["INCR", "BF", "INCR_NO_TL", "INCR_WCET", "RM", "RM_FAST", "RM_SLOW"]
    header = f"{'TS':>3}" + "".join(f" | {m:>10}" for m in modes)
    print(header)
    print("-" * 90)
    for r in results:
        line = f"{r['taskset']:>3}"
        for m in modes:
            v = r[m]
            if isinstance(v, str):
                line += f" | {v:>10}"
            else:
                line += f" | {v:>10.4f}"
        print(line)

    # Compute average gap vs BR
    gaps = {m: [] for m in modes if m != "BF"}
    for r in results:
        br_val = r["BF"]
        if isinstance(br_val, str):
            continue
        for m in modes:
            if m == "BF":
                continue
            v = r[m]
            if not isinstance(v, str):
                gaps[m].append(br_val - v)

    print("-" * 90)
    print(f"{'Gap':>3}" + "".join(f" | {sum(gaps[m])/len(gaps[m]):>10.4f}" if len(gaps[m]) > 0 else f" | {'N/A':>10}" for m in modes if m != "BF"))
    print("=" * 90)

    csv_path = os.path.join(base_dir, "k_variation.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["taskset"] + modes)
        writer.writeheader()
        writer.writerows(results)
    print(f"\nCSV written to {csv_path}")


if __name__ == "__main__":
    main()

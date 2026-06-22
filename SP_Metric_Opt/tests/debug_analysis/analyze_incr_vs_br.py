#!/usr/bin/env python3
"""
Case study: compare INCR vs BR scheduling quality gap.
Generates random task sets, runs both optimizers,
and analyzes why INCR underperforms BR.
"""
import os
import sys
import shutil
import subprocess
import json
import csv

# Add workspace to path
sys.path.append(os.getcwd())

from Gen_Taskset.lib.orchestrator import run_full_generation_pipeline


MODES = ["INCR", "BF"]

def parse_sp_values(output_dir, mode):
    """Parse interval_sp_metrics.txt for SP values."""
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
    """Run one simulation mode."""
    binary = "./build/tests/RunOrchestrator"
    cmd = [binary, input_dir, output_dir, mode, str(duration_ms)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        return None
    return parse_sp_values(output_dir, mode)


def main():
    base_dir = os.path.abspath("tests/incremental_analysis")
    if os.path.exists(base_dir):
        shutil.rmtree(base_dir)
    os.makedirs(base_dir, exist_ok=True)

    cfg_path = "Gen_Taskset/task_sets_config/taskset_cfg_6_1_sixtasks.json"
    num_runs = 10           # number of random task sets
    duration_ms = 10000     # 10-second interval
    seed_offset = 100

    csv_rows = []
    for run_i in range(num_runs):
        run_dir = os.path.join(base_dir, f"run_{run_i}")
        input_dir = os.path.join(run_dir, "input")
        output_dir = os.path.join(run_dir, "output")
        os.makedirs(input_dir, exist_ok=True)
        os.makedirs(output_dir, exist_ok=True)

        # Use a deterministic seed for reproducibility
        seed = seed_offset + run_i

        print(f"\n=== Run {run_i+1}/{num_runs} (seed={seed}) ===")

        # Force deterministic seed via env or modify generation
        # The Gen_Taskset pipeline seems to have built-in randomness;
        # we keep as-is for real random sampling.
        run_full_generation_pipeline(
            cfg_file=cfg_path,
            n_sec=30,
            dir_path=input_dir,
            add_perf_records=True,
            interact=False,
            n_path_per_task=1,
            n_inst_per_path=1
        )

        results = {}
        for mode in MODES:
            vals = run_mode(input_dir, output_dir, mode, duration_ms)
            results[mode] = vals
            if vals:
                print(f"  [{mode}] Avg SP: {sum(vals)/len(vals):.4f}, intervals: {len(vals)}")

        row = {
            "run": run_i,
            "seed": seed,
        }
        for mode in MODES:
            vals = results.get(mode)
            if vals:
                row[f"{mode}_avg"] = round(sum(vals) / len(vals), 6)
                row[f"{mode}_min"] = round(min(vals), 6)
                row[f"{mode}_max"] = round(max(vals), 6)
            else:
                row[f"{mode}_avg"] = "N/A"
                row[f"{mode}_min"] = "N/A"
                row[f"{mode}_max"] = "N/A"

        csv_rows.append(row)

    # Write CSV summary
    csv_path = os.path.join(base_dir, "incr_vs_br.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=csv_rows[0].keys())
        writer.writeheader()
        writer.writerows(csv_rows)
    print(f"\nCSV written to {csv_path}")

    # Print summary table
    print("\n" + "="*70)
    print(f"{'Run':>4} | {'INCR avg':>10} | {'BF avg':>10} | {'Gap':>10} | {'Gap %':>8}")
    print("-"*70)
    gaps = []
    for r in csv_rows:
        if r["INCR_avg"] != "N/A" and r["BF_avg"] != "N/A":
            gap = r["BF_avg"] - r["INCR_avg"]
            if r["BF_avg"] != 0:
                gap_pct = 100.0 * gap / abs(r["BF_avg"])
            else:
                gap_pct = 0.0
            gaps.append((gap, gap_pct))
            print(f"{r['run']:>4} | {r['INCR_avg']:>10.4f} | {r['BF_avg']:>10.4f} | {gap:>10.4f} | {gap_pct:>7.1f}%")
        else:
            print(f"{r['run']:>4} | {'N/A':>10} | {'N/A':>10} | {'N/A':>10} | {'N/A':>8}")

    if gaps:
        avg_gap = sum(g[0] for g in gaps) / len(gaps)
        avg_gap_pct = sum(g[1] for g in gaps) / len(gaps)
        max_gap = max(g[0] for g in gaps)
        print("-"*70)
        print(f"{'AVG':>4} | {'':>10} | {'':>10} | {avg_gap:>10.4f} | {avg_gap_pct:>7.1f}%")
        print(f"{'MAX':>4} | {'':>10} | {'':>10} | {max_gap:>10.4f} | {'':>8}")
    print("="*70)


if __name__ == "__main__":
    main()

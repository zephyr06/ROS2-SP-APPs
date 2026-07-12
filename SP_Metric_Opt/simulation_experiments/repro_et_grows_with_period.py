#!/usr/bin/env python3
"""Reproduce the "INCR ET grows with reoptimization period" issue, contention-free.

Context
-------
In the P25 period A/B (`configs/p25_period_ab_config.json`), the prod run observed
that ``INCR_Reopt_10`` / ``INCR_Reopt_30`` / ``INCR_Reopt_60`` had *higher*
per-activation scheduler execution time than ``INCR_Reopt_1`` in several tasksets
-- even though a
larger period means *fewer* reoptimization steps (the expensive wide-radius
search) and *more* cheap incremental steps.  Intuitively ET should *shrink* as
the period grows; the data showed the opposite.

Hypothesis 1 (algorithmic): something in the incremental path makes later
intervals more expensive as the incumbent ages (e.g. the diff or warm-start
grows, or a re-optimization step at the period boundary dominates).

Hypothesis 2 (measurement artifact): the prod run uses
``parallel_worker_processes = 4`` with 6 arms, so up to 4 ``RunOrchestrator``
processes share 8 cores simultaneously.  ``RunOrchestrator`` measures wall-clock
of the whole process (``start_time`` -> ``end_time`` around ``RunSimulation()``),
so each arm's reported ET is inflated by CPU contention from the others, and the
inflation is *not* uniform across arms (it depends on when each arm hits its
expensive reopt steps, which is period-dependent).  This script isolates the two
hypotheses by running every arm **serially** -- one process at a time, no
contention -- so the wall-time reflects actual CPU work per arm.

What this script does
---------------------
1. Reuses an *existing* generated taskset from the P25 prod run (no
   regeneration; deterministic, comparable across arms).
2. Runs each arm's ``RunOrchestrator`` **one at a time** (serial).
3. Reads back the wall-time ET the C++ writes, plus the on-disk interval count
   so the per-activation mean is computed identically to ``analyze_single_instance``.
4. Prints an arm-vs-ET table and dumps a JSON record.

Output goes under ``simulation_experiments/optimizer_comparison/et_repro/<tag>/``
so the original prod run is never touched.

Usage
-----
    python3 -m simulation_experiments.repro_et_grows_with_period
    python3 -m simulation_experiments.repro_et_grows_with_period --taskset 2
    python3 -m simulation_experiments.repro_et_grows_with_period --reps 3

Notes
-----
- This is a *reproduction* harness, not a fix.  It deliberately reuses the
  existing taskset so the repro is deterministic and cheap.
- ``taskset_characteristics_interval_*.yaml`` count = 30 (300s / 10s) is the
  per-activation divisor, matching ``run_sim_experiments.analyze_single_instance``.
"""
import argparse
import json
import os
import subprocess
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

PROD_RUN_DIR = os.path.join(
    PROJECT_ROOT,
    "simulation_experiments", "optimizer_comparison", "runs",
    "p25periodAB_run_prod_dur300_interval10_seed1000_tasks4x6x8",
)
ARMS = ["BF", "INCR_Reopt_1", "INCR_Reopt_5", "INCR_Reopt_10",
        "INCR_Reopt_30", "INCR_Reopt_60", "INCR_SCRATCH"]
NUM_INTERVALS_EXPECTED = 30  # 300s / 10s trigger interval


def _taskset_dir(num_tasks):
    return os.path.join(
        PROD_RUN_DIR, "sim",
        f"tasks{num_tasks}_dur300_interval10_seed1000", "taskset_0",
    )


def _count_intervals(taskset_dir):
    """Mirror analyze_single_instance's divisor: count of interval YAMLs."""
    import glob
    return len(glob.glob(os.path.join(
        taskset_dir, "taskset_characteristics_interval_*.yaml")))


def run_arm_serial(sim_bin, taskset_dir, output_parent, arm, num_intervals):
    """Run one arm's RunOrchestrator with no other arm running concurrently.

    Returns (wall_seconds_from_cpp, per_activation_ms).
    """
    sched_dir = os.path.join(output_parent, arm)
    os.makedirs(sched_dir, exist_ok=True)
    # duration_ms = scheduler_trigger_interval * 1000 = 10000 (per-interval
    # horizon, per the RunOrchestrator duration-arg semantics memory).
    cmd = [sim_bin, taskset_dir, sched_dir, arm, "10000", "1"]
    # Run serially: do NOT background or parallelize.
    proc = subprocess.run(cmd, check=True, capture_output=True, text=True)
    et_path = os.path.join(sched_dir, arm, "scheduler_execution_time.txt")
    wall = None
    if os.path.exists(et_path):
        with open(et_path) as f:
            wall = float(f.read().strip())
    per_activation_ms = (wall * 1000.0 / num_intervals) if wall else None
    return wall, per_activation_ms, proc.stdout + proc.stderr


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--num_tasks", type=int, default=6,
                    help="Task count (must match an existing prod-run taskset). "
                         "Ignored when --taskset_dir is given.")
    ap.add_argument("--taskset", type=int, default=0,
                    help="Taskset index within the prod run to reuse (default 0). "
                         "Ignored when --taskset_dir is given.")
    ap.add_argument("--taskset_dir", default=None,
                    help="Absolute path to an arbitrary taskset dir to run on "
                         "(overrides --num_tasks/--taskset; needed for freshly "
                         "generated tasksets outside the prod-run layout).")
    ap.add_argument("--reps", type=int, default=1,
                    help="Repeat each arm this many times (take min to filter "
                         "OS-scheduling noise).")
    ap.add_argument("--arms", nargs="+", default=None,
                    help="Subset of arms to run (default: all). e.g. "
                         "--arms INCR_Reopt_1 INCR_Reopt_5 INCR_Reopt_10 "
                         "INCR_Reopt_30 INCR_Reopt_60 INCR_SCRATCH "
                         "to skip BF (which is a ceiling reference, not relevant "
                         "to INCR-vs-REOPT ET profiling, and scales badly with N).")
    ap.add_argument("--bin_dir", default="release",
                    help="Directory containing C++ binaries (default: release).")
    ap.add_argument("--tag", default=None,
                    help="Subdir name under et_repro/ (default: auto from args).")
    args = ap.parse_args()

    # --taskset_dir wins over the prod-run --num_tasks/--taskset lookup; needed
    # for freshly generated tasksets that don't live under the prod-run layout.
    if args.taskset_dir:
        taskset_dir = args.taskset_dir
    elif args.taskset != 0:
        # Allow --taskset to pick a non-zero taskset by pointing at its dir.
        taskset_dir = os.path.join(
            os.path.dirname(_taskset_dir(args.num_tasks)),
            f"taskset_{args.taskset}",
        )
    else:
        taskset_dir = _taskset_dir(args.num_tasks)

    if not os.path.isdir(taskset_dir):
        print(f"ERROR: taskset dir not found: {taskset_dir}", file=sys.stderr)
        print("Run the P25 prod A/B first, or pick an existing taskset.",
              file=sys.stderr)
        sys.exit(1)

    sim_bin = os.path.join(
        PROJECT_ROOT, args.bin_dir
        if os.path.isabs(args.bin_dir)
        else os.path.join(PROJECT_ROOT, args.bin_dir),
        "tests", "RunOrchestrator",
    )
    if not os.path.exists(sim_bin):
        print(f"ERROR: binary not found: {sim_bin}", file=sys.stderr)
        sys.exit(1)

    num_intervals = _count_intervals(taskset_dir)
    if num_intervals != NUM_INTERVALS_EXPECTED:
        print(f"WARNING: expected {NUM_INTERVALS_EXPECTED} intervals, "
              f"found {num_intervals}.")

    tag = args.tag or f"tasks{args.num_tasks}_ts{args.taskset}_reps{args.reps}"
    output_parent = os.path.join(
        PROJECT_ROOT, "simulation_experiments", "optimizer_comparison",
        "et_repro", tag,
    )
    os.makedirs(output_parent, exist_ok=True)

    print(f"\n=== ET repro: serial arms, no contention ===")
    print(f"taskset_dir : {taskset_dir}")
    print(f"intervals   : {num_intervals}  (per-activation divisor)")
    print(f"reps/arm    : {args.reps}  (min taken)")
    print(f"output      : {output_parent}\n")

    arms = args.arms if args.arms else ARMS
    # Validate arm names against the known set.
    unknown = [a for a in arms if a not in ARMS]
    if unknown:
        print(f"ERROR: unknown arm(s): {unknown}. Known: {ARMS}", file=sys.stderr)
        sys.exit(1)

    results = {}
    for arm in arms:
        runs = []
        for r in range(args.reps):
            t0 = time.time()
            wall, per_act, _log = run_arm_serial(
                sim_bin, taskset_dir, output_parent, arm, num_intervals)
            outer = time.time() - t0
            runs.append({
                "wall_s": wall, "per_act_ms": per_act,
                "outer_s": outer,
            })
            print(f"  {arm:<13} rep{r}: wall={wall:.4f}s  "
                  f"per-act={per_act:.3f}ms  (outer {outer:.2f}s)")
        best = min(runs, key=lambda d: d["wall_s"])
        results[arm] = best

    print(f"\n--- Summary (min of {args.reps} rep(s)) ---")
    print(f"{'arm':<13} {'wall_s':>9} {'per_act_ms':>12}")
    for arm in arms:
        r = results[arm]
        print(f"{arm:<13} {r['wall_s']:>9.4f} {r['per_act_ms']:>12.3f}")

    # Persist for the record.
    record = {
        "taskset_dir": taskset_dir,
        "num_intervals": num_intervals,
        "reps": args.reps,
        "arms": results,
    }
    with open(os.path.join(output_parent, "et_repro_result.json"), "w") as f:
        json.dump(record, f, indent=2)
    print(f"\nSaved: {os.path.join(output_parent, 'et_repro_result.json')}")


if __name__ == "__main__":
    main()

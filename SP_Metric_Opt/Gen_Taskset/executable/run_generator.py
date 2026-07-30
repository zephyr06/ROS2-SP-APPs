import argparse
import sys
import os

# Adjust path to enable absolute imports if executed directly
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from Gen_Taskset.lib.orchestrator import (
    run_full_generation_pipeline,
    run_full_generation_pipeline_with_important_task_gate,
    generate_additional_execution_traces,
    validate_trajectory_config,
)
from Gen_Taskset.lib.generation_config_parser import load_generation_config

def main():
    parser = argparse.ArgumentParser(description="Modular Taskset and GMM Path/Trace Generator.")
    
    # Positional configuration file path
    parser.add_argument("cfg_file", type=str, help="Path to the JSON configuration file")
    
    # Execution options
    parser.add_argument("--n_sec", type=int, default=1000, help="Duration of simulation trace in seconds")
    parser.add_argument("--n_path_per_task", type=int, default=1, help="Number of trajectories (paths) to simulate")
    parser.add_argument("--n_inst_per_path", type=int, default=1, help="Number of trace instances to simulate per path")
    parser.add_argument("--dir_path", type=str, default=None, help="Output destination folder path")
    
    # Boolean flags
    parser.add_argument("--add_perf_records", action="store_true", help="Add soft-task performance scaling records")
    parser.add_argument("--interact", action="store_true", help="Draw moving paths and GMM surfaces interactively")
    parser.add_argument("--gen_path_for_taskset", action="store_true",
                        help="Only generate new paths/traces for an existing taskset (do not overwrite taskset params)")
    # P0.8: the important-task DM-schedulability gate. ON by default — every
    # generated taskset is certified schedulable for the important subset at the
    # seed point (re-samples with an advanced seed on failure; loud-raise on
    # exhaustion — NEVER silently emits an unschedulable taskset). Opt out only
    # for diagnostics / non-paper configs where you accept an uncertified emit.
    parser.add_argument("--important_tasks_schedulability_check",
                        action=argparse.BooleanOptionalAction, default=True,
                        help="Run the important-task schedulability gate (P0.8). "
                             "On by default: generation certifies the taskset is "
                             "DM-schedulable for the important tasks; pass "
                             "--no-important_tasks_schedulability_check to fall "
                             "back to the ungated pipeline.")

    args = parser.parse_args()

    # Determine absolute path to config file
    OPT_SP_PROJECT_PATH = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    cfg_file_abs = args.cfg_file
    if not cfg_file_abs.startswith('/'):
        cfg_file_abs = os.path.join(OPT_SP_PROJECT_PATH, cfg_file_abs)

    if args.gen_path_for_taskset:
        # 1. Load existing generation config
        cfgs = load_generation_config(cfg_file_abs)
        # 2nd integrity gate: trajectory-layer params (ROBOT_SPEED_MPS) that the
        # generation gate does not cover. run_full_generation_pipeline calls this
        # itself, but the --gen_path_for_taskset branch bypasses it.
        validate_trajectory_config(cfgs, config_path=cfg_file_abs)

        # 2. Resolve output directory
        output_dir = args.dir_path
        if output_dir is None:
            output_dir = os.path.join(OPT_SP_PROJECT_PATH, 'TaskData', os.path.basename(cfg_file_abs).replace('.json', '_gen_1'))
        elif not output_dir.startswith('/'):
            output_dir = os.path.join(OPT_SP_PROJECT_PATH, output_dir)
            
        # 3. Append traces
        generate_additional_execution_traces(
            cfgs=cfgs,
            dir_path=output_dir,
            n_path_per_task=args.n_path_per_task,
            n_inst_per_path=args.n_inst_per_path,
            path_idx=None,
            n_sec=args.n_sec,
            add_perf_records=args.add_perf_records,
            interact=args.interact
        )
    else:
        # Run the full pipeline from scratch.
        # P0.8: by default route through the important-task gate so the emitted
        # taskset is CERTIFIED schedulable for the important tasks under
        # DM-with-top-priority-lock at the seed (re-samples on failure, loud-raise
        # on exhaustion). The gate forwards the same kwargs as the plain pipeline
        # (its body IS _run_pipeline_with_cfgs, the same body the shell calls).
        if args.important_tasks_schedulability_check:
            report = run_full_generation_pipeline_with_important_task_gate(
                cfg_file=args.cfg_file,
                n_sec=args.n_sec,
                dir_path=args.dir_path,
                add_perf_records=args.add_perf_records,
                interact=args.interact,
                n_path_per_task=args.n_path_per_task,
                n_inst_per_path=args.n_inst_per_path,
            )
            # The gate's contract: it returns ONLY on a schedulable draw (else it
            # raises). Surface the attempt count so a multi-attempt emit is
            # visible, not silent.
            print(f"[P0.8 gate] taskset certified schedulable for important "
                  f"tasks (attempts_used={report['attempts_used']}).")
        else:
            run_full_generation_pipeline(
                cfg_file=args.cfg_file,
                n_sec=args.n_sec,
                dir_path=args.dir_path,
                add_perf_records=args.add_perf_records,
                interact=args.interact,
                n_path_per_task=args.n_path_per_task,
                n_inst_per_path=args.n_inst_per_path
            )

if __name__ == "__main__":
    main()

import os
import sys

# Add the project root to sys.path so we can import Gen_Taskset
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from Gen_Taskset.lib.orchestrator import run_full_generation_pipeline
from Gen_Taskset.lib.generation_config_parser import load_generation_config, standardize_config
from Gen_Taskset.lib.taskset_generator import load_and_fill_taskset_param_file
from Gen_Taskset.lib.visualizer import plot_3d_execution_time_surface

def main():
    # Paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    cfg_file = os.path.join(os.path.dirname(script_dir), "task_sets_config", "taskset_cfg_1.json")
    out_dir = os.path.join(script_dir, "output_example")
    
    print(f"Loading configuration: {cfg_file}")
    cfgs = load_generation_config(cfg_file)
    cfgs = standardize_config(cfgs)
    
    print("Running taskset generation pipeline...")
    # Run pipeline for 100 seconds, with 2 paths and 2 instances per path
    run_full_generation_pipeline(
        cfg_file=cfg_file,
        n_sec=100,
        dir_path=out_dir,
        add_perf_records=True,
        interact=False,
        n_path_per_task=2,
        n_inst_per_path=1
    )
    
    print("Loading generated taskset parameters...")
    taskset_param_fpath = os.path.join(out_dir, "taskset_param.yaml")
    taskset_data = load_and_fill_taskset_param_file(taskset_param_fpath)
    
    print(f"Loaded taskset with {taskset_data['n_tasks']} tasks.")
    
    # Generate 3D surface plots for each task in the taskset
    for i, task in enumerate(taskset_data['tasks']):
        plot_path = os.path.join(out_dir, f"task_{i}_3d_surface.png")
        print(f"Plotting 3D surface for Task {i} -> {plot_path}")
        plot_3d_execution_time_surface(
            cfgs=cfgs,
            task_param=task,
            output_path=plot_path,
            draw=False,
            is_mixture=True
        )
        
    print("Execution time surfaces successfully generated!")

if __name__ == "__main__":
    main()

from SP_draw_fig_utils import *
import os
import numpy as np
import sys

if __name__=="__main__":
    
    scheduler_name = "Scheduler"
    if len(sys.argv) > 1:
        scheduler_name = sys.argv[1]
    
    # the path of the folder which generates the profiling data in ROS2 workspace
    data_folder_paths = {
        # "FIFO": os.path.join(
        # OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "data"),
        scheduler_name: os.path.join(OPT_SP_PROJECT_PATH,"../all_time_records"),
        }
    
    task_set_config = os.path.join(
        os.path.dirname(OPT_SP_PROJECT_PATH),"all_time_records", "task_characteristics.yaml")
    verify_task_set_config(task_set_config)
    app_name2period = get_app2period(task_set_config)
    tasks_name_list = ['TSP', 'RRT', 'SLAM', 'MPC']

    horizon_granularity = 10  # 10 seconds
    horizon = 2000  # 100 seconds
    discard_early_time = 120  # 20 seconds



    for method_name, data_folder_path in data_folder_paths.items():
        tasks_name_to_info = get_task_set_info(tasks_name_list, app_name2period, data_folder_path)
        sp_value_list = get_sp_value_list(tasks_name_list, tasks_name_to_info, horizon, horizon_granularity, discard_early_time, task_set_abs_path=task_set_config)
        x_axis = [i for i in range(0, len(sp_value_list)*horizon_granularity, horizon_granularity)]
        plt.plot(x_axis, sp_value_list, label = method_name)
        print(f"SP-Metric for {method_name}: {sum(sp_value_list)/len(sp_value_list)}")

    plt.legend(data_folder_paths.keys())
    plt.xlabel("Time (s)")
    plt.ylabel("SP-Metric")
    plt.tight_layout()
    
    plt.savefig(os.path.join(
        os.path.dirname(OPT_SP_PROJECT_PATH),"all_time_records", "current_scheduler_SP.pdf"), format='pdf')
    
    plt.show(block=False)
    plt.pause(3)
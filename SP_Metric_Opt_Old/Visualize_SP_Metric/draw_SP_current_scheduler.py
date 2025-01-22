from SP_draw_fig_utils import *
import os
import numpy as np
import yaml
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

    discard_early_time = 30  # at least 10 seconds, should be integer multilpe of scheduler's period
    horizon_granularity = 10  # 10 seconds
    task_config_file_path = os.path.join(OPT_SP_PROJECT_PATH,"../all_time_records/task_characteristics.yaml")
    yaml_data = yaml.safe_load(open(task_config_file_path))
    horizon =  int(float(yaml_data['tasks'][0]['total_running_time'])/1e3) #  seconds

    draw_and_saveSP_fig_single_run(data_folder_paths, discard_early_time, horizon_granularity, horizon)
    

from SP_draw_fig_utils import *
import os
import numpy as np
import yaml
import sys


if __name__=="__main__":
    RYAN_CHANGE = True

    scheduler_name = "Scheduler"
    if len(sys.argv) > 1:
        scheduler_name = sys.argv[1]
    
    # the path of the folder which generates the profiling data in ROS2 workspace
    data_folder_paths = {
        # "FIFO": os.path.join(
        # OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "data"),
        scheduler_name: os.path.join(OPT_SP_PROJECT_PATH,"../all_time_records"),
        }

    ################## specify example file path
    if RYAN_CHANGE:
        task_characteristics_name = "task_characteristics.yaml"
        task_characteristics_dir = os.path.join(OPT_SP_PROJECT_PATH,"../all_time_records_example/all_time_records");
        data_folder_paths = { scheduler_name:  task_characteristics_dir}
        print(f'OPT_SP_PROJECT_PATH: {OPT_SP_PROJECT_PATH}')
        #quit()

    discard_early_time = 30  # at least 10 seconds, should be integer multilpe of scheduler's period
    horizon_granularity = 10  # 10 seconds
    if RYAN_CHANGE:
        task_config_file_path = os.path.join(task_characteristics_dir,task_characteristics_name)        
    else:
        task_config_file_path = os.path.join(OPT_SP_PROJECT_PATH,"../all_time_records/task_characteristics.yaml")
    yaml_data = yaml.safe_load(open(task_config_file_path))
    horizon =  int(float(yaml_data['tasks'][0]['total_running_time'])/1e3) #  seconds

    if RYAN_CHANGE:
        #print(f'data_folder_paths: {data_folder_paths}')
        #print(f'task_characteristics_dir: {task_characteristics_dir}')
        #print(f'task_characteristics_name: {task_characteristics_name}')
        #quit()
        draw_and_saveSP_fig_single_run(data_folder_paths, discard_early_time, horizon_granularity, horizon, 
            task_characteristics_dir=task_characteristics_dir,task_characteristics_name=task_characteristics_name,
            RYAN_CHANGE_I=RYAN_CHANGE)
    else:
        draw_and_saveSP_fig_single_run(data_folder_paths, discard_early_time, horizon_granularity, horizon)
    

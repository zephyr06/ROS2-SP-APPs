import argparse
import os,sys
import numpy as np
import yaml
import math
import shutil
import subprocess
from SP_draw_fig_utils import *
import matplotlib.pyplot as plt

'''
python ryan_draw_sim_rst.py --file_path TaskData/ryan_robotics_v2.yaml \
--rst_file_path TaskData/sim_res_INCR_ryan_robotics_v2.txt \
--out_dir TaskData/sim_res_INCR_ryan_robotics_v2 --method INCR

python ryan_draw_sim_rst.py --file_path TaskData/AnalyzeSP_Metric_2/task_characteristics.yaml \
--rst_file_path TaskData/sim_res_INCR_ryan_robotics_v2.txt \
--out_dir TaskData/AnalyzeSP_Metric_2/rst --method INCR --i

'''
def main():
    dbg = False
    #if os.name == 'nt':
    #    print("Running on Windows")
    #elif os.name == 'posix':
    #    print("Running on Linux or Unix-like system")
    #else:
    #    print("Running on an unknown operating system")

    #print(OPT_SP_PROJECT_PATH)
    #quit()
    if dbg:
        task_config_file_path = r'TaskData\ryan_robotics_v2.yaml'
        rst_file_path = r'TaskData\sim_res_INCR_ryan_robotics_v2.txt'
        method = 'INCR'
        out_dir = r'TaskData\sim_res_INCR_ryan_robotics_v2'
    else:
        ###############################################################################################
        # parse arguments
        # Create the parser
        parser = argparse.ArgumentParser(description="draw simulation result")

        # Add arguments
        parser.add_argument("-tf", "--file_path", type=str, help="task characteristics file (yaml)", required=True)
        parser.add_argument("-rf", "--rst_file_path", type=str, help="result file", required=True)
        parser.add_argument("-od", "--out_dir", type=str, help="output directory", required=True)
        parser.add_argument("-m", "--method", type=str, help="optimization method", required=True)

        # -i not require value, change to str
        parser.add_argument("-i", "--interactive", action="store_true", help="interactive", required=False)

        # Parse the arguments
        args = parser.parse_args()

        task_config_file_path = args.file_path
        rst_file_path = args.rst_file_path
        out_dir = args.out_dir
        method = args.method
        interactive = args.interactive
        print('interactive mode',interactive)
        #quit()

    # CREARE out_dir if not exist
    if not out_dir.startswith("./"):
        out_dir = os.path.join(OPT_SP_PROJECT_PATH, out_dir)
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)
    task_config_file_path = os.path.join(OPT_SP_PROJECT_PATH, task_config_file_path)
    rst_file_path = os.path.join(OPT_SP_PROJECT_PATH, rst_file_path)


    ###############################################################################################
    # read data
    horizon_granularity = 10  # 10 seconds
    
    yaml_data = yaml.safe_load(open(task_config_file_path))
    
    task_id_dict = {}
    for t in yaml_data['tasks']:
        # name, id, execution_time ...
        task_id_dict[t['id']] = t
    
    # put result for each task into rst_dict
    rst_dict = {}
    for key in task_id_dict: # key is int
        rst_dict[key] = {}

    # Open and read the file
    max_interval_idx = -1
    max_s = -1
    with open(rst_file_path, 'r') as file:
        for line in file:
            # Split the line into components
            taskid, jobid, start_time, end_time, execution_time = line.strip().split(',')
            taskid = int(taskid)
            jobid = int(jobid)
            start_time = float(start_time) / 1000.0
            end_time   = float(end_time) / 1000.0
            execution_time = float(execution_time) / 1000.0
            if end_time > max_s:
                max_s = end_time
        
            # Add the data to the dictionary
            #period_ms = yaml_data['tasks'][taskid]['period']
            #idx = int(start_time * 1000 / period_ms)
            idx = int(start_time / horizon_granularity)
            if idx > max_interval_idx:
                max_interval_idx = idx
            if idx not in rst_dict[taskid]:
                rst_dict[taskid][idx] = []
            rst_dict[taskid][idx].append( [jobid, start_time, end_time, execution_time] )
            
    horizon = math.ceil(max_s / horizon_granularity)
    print("horizon: ", horizon)
 
    ###############################################################################################
    # calculate SP-Metric for each interval

    # rm and then recreate dir 'temp'
    temp_dir = os.path.join(OPT_SP_PROJECT_PATH,"temp")
    if os.path.exists(temp_dir):
        shutil.rmtree(temp_dir)
    os.makedirs(temp_dir)

    sp_value_list = []
    command_in_terminal_to_analyze_taskset_sp = get_SP_analyze_executable_file_path()
    for i in range(max_interval_idx + 1):
        for taskid in task_id_dict:
            task_name = yaml_data['tasks'][taskid]['name']  # task name
            if i in rst_dict[taskid]:
                print(f'processing {task_name} at interval {i} ...')
                resp_file_path = os.path.join(temp_dir, task_name + "_response_time.txt")

                # no need to sort since result should be sorted already
                # rst_dict[taskid][i] = sorted(rst_dict[taskid][i], key=lambda x: x[1])    # sort by start time

                with open(resp_file_path, 'w') as f:
                    print(f'gen {resp_file_path} at interval {i} ...')
                    for jobid, start_time, end_time, execution_time in rst_dict[taskid][i]:
                        f.write(f"{(end_time - start_time)*1000}\n")
                
                if 'performance_records_time' in yaml_data['tasks'][taskid]:
                    exe_file_path = os.path.join(temp_dir, task_name + "_execution_time.txt")
                    with open(exe_file_path, 'w') as f:
                        print(f'gen {exe_file_path} at interval {i} ...')
                        for jobid, start_time, end_time, execution_time in rst_dict[taskid][i]:
                            f.write(f"{execution_time*1000}\n")

        # run the command to calculate SP-Metric
        command = command_in_terminal_to_analyze_taskset_sp
        command += " --file_path " 
        command += task_config_file_path
        command += " --data_dir " + temp_dir
        print(f'run command: {command}')
        result = subprocess.run(command, shell=True, capture_output=True, text=True)
        print(result.stdout)
        sp_value = get_sp_value(result.stdout)
        if sp_value > -4.5:
            a=1
        print(f'sp_value return for stdout: {sp_value}')
        sp_value_list.append(sp_value)

    	# debug
        if interactive:
            input("Press Enter to continue...")

    print(sp_value_list)
    sp_value_file_path = os.path.join(out_dir, "sp_value_"+method+".txt")
    with open(sp_value_file_path, 'w') as f:
        for value in sp_value_list:
            f.write(f"{value}\n")

    ###############################################################################################
    # draw SP-Metric
    # x is 0...horizon-1, y is sp_value_list
    # draw by plt
    x_axis = [i for i in range(0, len(sp_value_list)*horizon_granularity, horizon_granularity)]
    method_name = method
    plt.plot(x_axis, sp_value_list, label = method_name)
    print(bcolors.WARNING +f"SP-Metric for {method_name}: {sum(sp_value_list)/len(sp_value_list)}"+bcolors().ENDC)

    plt.legend([method_name])
    plt.xlabel("Time (s)")
    plt.ylabel("SP-Metric")
    plt.tight_layout()
    figure_path = os.path.join(out_dir, "sp_value_plot_"+method+".pdf")
    plt.savefig(figure_path, format='pdf')
    
    plt.show(block=False)
    plt.pause(3)
    plt.close()

if __name__ == "__main__":
    main()
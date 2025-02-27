import argparse
import os,sys
import numpy as np
import yaml
import math
import shutil
import subprocess
from SP_draw_fig_utils import *
from visualize_SP_distribution import *
import matplotlib.pyplot as plt

'''
python ryan_draw_sim_rst.py --sim_rst_dir TaskData/taskset_cfg_1_gen_1 --method INCR

# --sim_rst_dir 
#   This directory should have the following files:
#   * sim_res_[method]_[x].txt which are simulation result files. x stands for x's simulation result
#   * taskset_characteristics.yaml which is the taskset characteristics file
#   The output will be saved in sim_res_[method] subdir in this directory
'''

'''
test_data = [
    ([0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 
      210, 220, 230, 240, 250, 260, 270, 280, 290, 300, 310, 320, 330, 340, 350, 360, 370, 380, 390, 400, 
      410, 420, 430, 440, 450, 460, 470, 480, 490, 500, 510, 520, 530, 540, 550, 560, 570, 580, 590, 600, 
      610, 620, 630, 640, 650, 660, 670, 680, 690, 700, 710, 720, 730, 740, 750, 760, 770, 780, 790], 
     [1.76441, 1.67101, 1.66242, 1.58246, 1.66028, 1.77563, 1.67659, 1.7385, 1.58837, 1.48759, 1.51202, 1.39791, 
     1.72954, 1.59298, 1.46686, 1.42216, 1.71456, 2.0503, 1.73869, 1.63298, 1.59708, 1.90906, 1.65765, 1.76654, 
     1.41928, 1.56934, 1.65832, 1.96504, 1.81475, 1.78568, 1.59291, 1.77354, 1.69614, 1.62618, 1.97443, 1.56485,
     1.88383, 1.71758, 1.70945, 1.82771, 1.59197, 1.65797, 1.74, 1.8512, 1.75458, 1.64466, 1.89245, 1.64981, 
     1.64518, 1.47805, 1.88083, 1.95209, 1.57656, 1.66768, 1.71732, 1.33171, 1.74007, 1.85266, 1.86332, 1.67021, 
     1.60663, 1.4508, 2.01433, 1.7359, 1.9839, 1.74668, 1.60557, 1.62176, 1.74253, 1.77127, 1.5576, 1.65999, 
     1.45045, 1.86409, 1.4799, 1.6514, 1.77859, 1.53957, 1.66144, 1.67313]
    ), 
    ([0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 
      210, 220, 230, 240, 250, 260, 270, 280, 290, 300, 310, 320, 330, 340, 350, 360, 370, 380, 390, 400, 
      410, 420, 430, 440, 450, 460, 470, 480, 490, 500, 510, 520, 530, 540, 550, 560, 570, 580, 590, 600, 
      610, 620, 630, 640, 650, 660, 670, 680, 690, 700, 710, 720, 730, 740, 750, 760, 770, 780, 790], 
     [1.42483, 1.90618, 1.73059, 1.65892, 1.62632, 1.87193, 1.96187, 1.69389, 1.79563, 1.57036, 1.48984, 1.90264, 
      1.81609, 1.50808, 1.48387, 1.81527, 1.46188, 1.63042, 1.71272, 1.87063, 1.88881, 1.5812, 1.7965, 1.71437, 
      1.70261, 1.39821, 1.6465, 1.94526, 2.00741, 1.93351, 1.61477, 1.85205, 1.84702, 1.79813, 1.88029, 1.71181, 
      1.87786, 1.79647, 1.77316, 1.79345, 1.75647, 1.74108, 1.84268, 1.66314, 1.81376, 1.99129, 1.83156, 1.76753, 
      1.62519, 1.81861, 1.84941, 1.7827, 1.82983, 1.73878, 1.70045, 1.93136, 2.00894, 1.79553, 1.86733, 1.81705, 
      2.00783, 1.75914, 1.83751, 1.96912, 1.82734, 2.04432, 1.77427, 1.88157, 1.83203, 1.97096, 1.76476, 1.76369, 
      1.87152, 1.85055, 1.93882, 1.78173, 1.76763, 1.93599, 1.8381, 1.7386]
    ), 
    ([0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 
      210, 220, 230, 240, 250, 260, 270, 280, 290, 300, 310, 320, 330, 340, 350, 360, 370, 380, 390, 400, 
      410, 420, 430, 440, 450, 460, 470, 480, 490, 500, 510, 520, 530, 540, 550, 560, 570, 580, 590, 600, 
      610, 620, 630, 640, 650, 660, 670, 680, 690, 700, 710, 720, 730, 740, 750, 760, 770, 780, 790], 
     [1.77096, 1.56441, 1.49833, 1.38545, 1.49492, 1.53791, 1.5197, 1.35442, 1.85053, 1.65851, 1.69085, 1.60459, 
      1.43623, 1.57788, 1.69227, 1.92819, 1.67664, 1.51014, 1.4573, 1.63916, 1.71148, 1.7137, 2.08887, 1.67836, 
      1.52422, 1.87733, 1.72598, 1.45861, 1.59376, 1.77445, 2.02504, 1.80553, 1.87293, 1.82209, 1.87387, 1.91869, 
      1.78286, 1.72886, 1.70367, 1.92024, 1.8923, 1.80793, 1.90424, 1.70281, 1.80511, 1.73712, 1.87766, 1.81277, 
      1.60879, 1.84938, 1.75412, 1.97392, 1.77535, 1.69608, 1.65542, 1.58955, 1.47179, 1.53093, 1.93181, 1.49316, 
      1.32003, 1.82827, 1.50039, 1.57803, 1.44472, 1.75738, 1.56905, 1.5504, 1.33649, 1.72263, 1.76461, 1.57101, 
      1.54632, 1.48197, 1.34, 1.33121, 1.51981, 1.64639, 1.58642, 1.52698]
    ), 
    ([0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 
      210, 220, 230, 240, 250, 260, 270, 280, 290, 300, 310, 320, 330, 340, 350, 360, 370, 380, 390, 400, 
      410, 420, 430, 440, 450, 460, 470, 480, 490, 500, 510, 520, 530, 540, 550, 560, 570, 580, 590, 600, 
      610, 620, 630, 640, 650, 660, 670, 680, 690, 700, 710, 720, 730, 740, 750, 760, 770, 780, 790], 
     [1.76084, 1.47515, 1.69049, 1.66676, 1.45912, 1.65425, 1.65167, 1.62794, 1.55354, 1.68688, 1.75601, 1.83544, 
      1.71498, 1.56915, 1.6474, 1.7866, 1.73471, 1.74499, 1.46061, 1.46357, 1.7026, 1.47574, 1.6085, 1.63557, 
      1.79033, 1.62736, 1.60622, 1.6528, 1.60337, 1.69497, 1.57336, 1.49819, 1.43227, 1.64197, 1.61407, 1.92036, 
      1.78508, 1.89857, 1.68442, 1.68871, 1.79861, 1.50824, 1.84227, 1.79056, 1.82059, 1.77687, 1.79918, 1.48689, 
      1.69534, 1.6459, 1.87721, 1.87572, 1.67738, 1.51588, 1.83127, 1.59199, 1.66868, 1.65606, 1.88108, 1.77225, 
      1.54628, 1.89677, 1.68019, 1.56013, 1.63471, 1.52385, 1.53308, 1.74667, 1.60805, 1.71529, 1.46218, 1.90373, 
      1.58162, 1.74668, 1.59296, 1.44414, 1.6228, 1.47415, 1.75891, 1.49218]
    )
]
pdf_file_path = 'a.pdf'
csv_file_path = 'a.csv'
method = 'CFS'
plot_and_save_boxplot_sp(test_data, pdf_file_path, csv_file_path, method, show_fig_time=0.1)
quit()
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
        sim_rst_dir = r'TaskData\\taskset_cfg_3_gen_1'
        method = 'RM_FAST'
        interactive = True
        path_idx = 0
    else:
        ###############################################################################################
        # parse arguments
        # Create the parser
        parser = argparse.ArgumentParser(description="draw simulation result")

        # Add arguments
        parser.add_argument("-sd", "--sim_rst_dir", type=str, help="simulation result directory", required=True)
        parser.add_argument("-m", "--method", type=str, help="optimization method", required=True)
        parser.add_argument("-p", "--path_idx", type=int, help="path index", required=False,default=0)

        # -i not require value, change to str
        parser.add_argument("-i", "--interactive", action="store_true", help="interactive", required=False)

        # Parse the arguments
        args = parser.parse_args()

        sim_rst_dir = args.sim_rst_dir
        method = args.method
        path_idx = args.path_idx
        interactive = args.interactive

    if not sim_rst_dir.startswith("/"):
        sim_rst_dir = os.path.join(OPT_SP_PROJECT_PATH, sim_rst_dir)
    if not os.path.exists(sim_rst_dir):
        print("sim_rst_dir {} does not exist".format(sim_rst_dir))
        quit()

    task_config_file_path = os.path.join(sim_rst_dir, 'taskset_characteristics_0.yaml')
    if not os.path.exists(task_config_file_path):
        print("task_config_file_path {} does not exist".format(task_config_file_path))
        quit()
    
    out_dir = os.path.join(sim_rst_dir, f'sim_res_p{path_idx}_{method}')
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)

    ###############################################################################################
    # read data
    horizon_granularity = 10  # 10 seconds
    
    yaml_data = yaml.safe_load(open(task_config_file_path))
    
    task_id_dict = {}
    for t in yaml_data['tasks']:
        # name, id, execution_time ...
        task_id_dict[t['id']] = t
    
    rst_idx = 0
    boxplot_data = []

    while True: # get result of all simulation indexes
        rst_file_path = os.path.join(sim_rst_dir, f'sim_res_{method}_{rst_idx}.txt')
        if not os.path.exists(rst_file_path):
            if rst_idx == 0:
                print(f'no simulation result of {method} found')
                quit()
            else:
                print(f'no more simulation result found')
                break

        # put result for each task into rst_dict
        rst_dict = {}
        for key in task_id_dict: # key is int
            rst_dict[key] = {}

        # Open and read the simulation result file
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
        if horizon<=0:
            break        
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
                    #print(f'processing {task_name} at interval {i} ...')
                    resp_file_path = os.path.join(temp_dir, task_name + "_response_time.txt")

                    # no need to sort since result should be sorted already
                    # rst_dict[taskid][i] = sorted(rst_dict[taskid][i], key=lambda x: x[1])    # sort by start time

                    with open(resp_file_path, 'w') as f:
                        #print(f'gen {resp_file_path} at interval {i} ...')
                        for jobid, start_time, end_time, execution_time in rst_dict[taskid][i]:
                            f.write(f"{(end_time - start_time)*1000}\n")
                    
                    if 'performance_records_time' in yaml_data['tasks'][taskid]:
                        exe_file_path = os.path.join(temp_dir, task_name + "_execution_time.txt")
                        with open(exe_file_path, 'w') as f:
                            #print(f'gen {exe_file_path} at interval {i} ...')
                            for jobid, start_time, end_time, execution_time in rst_dict[taskid][i]:
                                f.write(f"{execution_time*1000}\n")

            # run the command to calculate SP-Metric
            command = command_in_terminal_to_analyze_taskset_sp
            command += " --file_path " 
            command += task_config_file_path
            command += " --data_dir " + temp_dir
            #print(f'run command: {command}')
            result = subprocess.run(command, shell=True, capture_output=True, text=True)
            print(result.stdout)
            sp_value = get_sp_value(result.stdout)
            if sp_value > -4.5:
                a=1
            print(f'\n\n########Instance={rst_idx}, interval={i}: sp_value return for stdout: {sp_value}')
            sp_value_list.append(sp_value)

            # debug
            if interactive:
                input("Press Enter to continue...")

        print(f'\n\n######## Instance={rst_idx}: sp_value_list={sp_value_list}')
        sp_value_file_path = os.path.join(out_dir, "sp_value_"+method+"_"+str(rst_idx)+".txt")
        with open(sp_value_file_path, 'w') as f:
            for value in sp_value_list:
                f.write(f"{value}\n")


        x_axis = [i for i in range(0, len(sp_value_list)*horizon_granularity, horizon_granularity)]
        boxplot_data.append((x_axis,sp_value_list))

        if interactive:
            #######################################################################################
            # draw SP-Metric for this instance
            # x is 0...horizon-1, y is sp_value_list
            # draw by plt
            x_axis = [i for i in range(0, len(sp_value_list)*horizon_granularity, horizon_granularity)]
            method_name = method
            plt.plot(x_axis, sp_value_list, label = method_name)
            print(bcolors.WARNING +f"SP-Metric for {method_name}: {sum(sp_value_list)/len(sp_value_list)}"+bcolors().ENDC)

            # RYAN_CHANGE_20250207
            # make sure Y-axis starts from 0
            plt.ylim( 0, math.ceil(max(sp_value_list)) )  

            plt.legend([method_name])
            plt.xlabel("Time (s)")
            plt.ylabel("SP-Metric")
            plt.tight_layout()
            figure_path = os.path.join(out_dir, "sp_value_plot_"+method+".pdf")
            plt.savefig(figure_path, format='pdf')
            
            plt.show(block=False)
            plt.pause(3)
            plt.close()

        rst_idx += 1


    ###############################################################################################
    # draw boxplot

    pdf_file_path = os.path.join(out_dir, "box_plot_of_all_data_" + method + ".pdf")
    csv_file_path = os.path.join(out_dir, "sp_data.csv")
    print(f'\n\n######### plot_and_save_boxplot_sp data = {boxplot_data}')
    plot_and_save_boxplot_sp(boxplot_data, pdf_file_path, csv_file_path, method, show_fig_time=0.1)

    print(f'\n\n######### done')

if __name__ == "__main__":
    main()
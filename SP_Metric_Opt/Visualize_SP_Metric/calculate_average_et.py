from SP_draw_fig_utils import OPT_SP_PROJECT_PATH
import os

def get_ext_list(file_name):
    ext_list = []
    with open(file_name, 'r') as f:
        for line in f:
            ext_str = line.split("::")[1][:-1]  
            ext_list.append(float(ext_str))
    return ext_list

if __name__ == "__main__":
    # file_name = os.path.join(os.path.dirname(OPT_SP_PROJECT_PATH), "all_time_records", "slam_execution_time.txt")
    file_name = os.path.join(os.path.dirname(OPT_SP_PROJECT_PATH), "all_time_records", "slam_execution_time_bf_copy.txt")
    
    ext_list = get_ext_list(file_name)
    top_counts_under_consider=1000
    if(len(ext_list)>top_counts_under_consider):
        ext_list = ext_list[:top_counts_under_consider]
    print("Average execution time: ", sum(ext_list)/len(ext_list))

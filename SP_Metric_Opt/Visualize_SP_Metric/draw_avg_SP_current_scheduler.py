from SP_draw_fig_utils import *
import os
import numpy as np
import shutil
import sys
import tarfile

def decompress_tar_gz(tar_gz_file_path):
    # Get the directory containing the compressed file
    directory = os.path.dirname(tar_gz_file_path)
    
    # Open the tar.gz file
    with tarfile.open(tar_gz_file_path, 'r:gz') as tar:
        # Extract all contents to the same directory
        tar.extractall(path=directory)


def find_all_tar_gz_files(folder_path):
    # Get all files in the folder
    all_files = os.listdir(folder_path)
    
    # Filter the files that have the extension .tar.gz
    tar_gz_files = [file for file in all_files if file.endswith(".tar.gz")]
    
    return tar_gz_files

def extract_files_to_parent_directory(folder_path):
    # Get list of all subdirectories in the folder
    subdirectories = [subdir for subdir in os.listdir(folder_path) if os.path.isdir(os.path.join(folder_path, subdir))]

    # Iterate over each subdirectory
    for subdir in subdirectories:
        subdir_path = os.path.join(folder_path, subdir)
        # Get list of all files in the subdirectory
        files = [f for f in os.listdir(subdir_path) if os.path.isfile(os.path.join(subdir_path, f))]
        # Move each file to the parent directory
        for file in files:
            src = os.path.join(subdir_path, file)
            dst = os.path.join(folder_path, file)
            shutil.move(src, dst)
        # Optionally, remove the now empty subdirectory
        os.rmdir(subdir_path)

if __name__=="__main__":
    
    scheduler_name = "Scheduler"
    if len(sys.argv) > 1:
        scheduler_name = sys.argv[1]
    
    # the path of the folder which generates the profiling data in ROS2 workspace
    # data_folder_paths = {
    #     # "FIFO": os.path.join(
    #     # OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "data"),
    #     scheduler_name: os.path.join(OPT_SP_PROJECT_PATH,"../all_time_records"),
    #     }

    discard_early_time = 60  # at least 10 seconds
    horizon_granularity = 30  # 10 seconds
    horizon = 300  #  seconds

    multi_run_folder = os.path.join(os.path.dirname(OPT_SP_PROJECT_PATH),"Experiments/MultiRun/RM/")
    # draw_and_saveSP_fig_single_run(data_folder_paths, discard_early_time, horizon_granularity, horizon)
    tar_files = find_all_tar_gz_files(os.path.join(os.path.dirname(OPT_SP_PROJECT_PATH), multi_run_folder))
    print(tar_files)

    for tar_file in tar_files:
        # Extract the tar.gz file
        decompress_tar_gz(os.path.join(multi_run_folder, tar_file))
        extract_files_to_parent_directory(os.path.join(multi_run_folder, tar_file[:-7]))
    

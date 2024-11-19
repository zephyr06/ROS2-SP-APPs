import os
import tarfile
import shutil
import tempfile
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
from SP_draw_fig_utils import OPT_SP_PROJECT_PATH
from draw_SP_current_scheduler import *

import os
import shutil
import random
from SP_Metric_Opt.Visualize_SP_Metric.box_plot_utils import get_subfolder_path, clear_folder


def read_ET_data_from_file(folder_path, appname, sample_threshold=10000, sample_rate=0.01):
    file_path = os.path.join(folder_path, appname + "_execution_time.txt")

    times = []
    line_count = 0

    # First pass: Count the number of lines in the file
    with open(file_path, 'r') as file:
        for _ in file:
            line_count += 1

    # Decide whether to sample
    if line_count > sample_threshold:
        # Calculate the number of samples (10% of the data)
        num_samples = max(1, int(line_count * sample_rate))  # Ensure at least 1 sample
        sampled_indices = sorted(random.sample(range(line_count), num_samples))
    else:
        sampled_indices = None

    # Second pass: Process the lines
    with open(file_path, 'r') as file:
        for i, line in enumerate(file):
            # If sampling, skip lines not in sampled_indices
            if sampled_indices is not None and i not in sampled_indices:
                continue

            # Parse the line to extract execution times
            parts = line.split("::")
            if len(parts) > 2:
                try:
                    times.append(float(parts[2].strip()))  # Append the execution time as float
                except ValueError:
                    # Handle lines where the third part is not a valid float
                    continue

    # Convert the list to a numpy array and return
    return np.array(times)

# def read_ET_data_from_file(folder_path, appname):
#     file_path = os.path.join(folder_path, appname+"_execution_time.txt")
#
#     times = []
#     # Open and read the file
#     with open(file_path, 'r') as file:
#         for line in file:
#             # Split the line by "::" and get the last part (execution time)
#             parts = line.split("::")
#             if len(parts) > 1:  # Ensures the line has at least 3 parts (expected format)
#                 times.append(float(parts[2].strip()))  # Append the execution time as float
#
#     # Convert the list to a numpy array and return
#     return times

def process_tar_files_for_ET(folder_path, appname):
    temp_folder_path = os.path.join(OPT_SP_PROJECT_PATH, "temp_folder")

    all_times = []

    # Loop through all files in the given folder
    for file_name in sorted(os.listdir(folder_path)):
        if file_name.endswith('.tar.gz'):
            if(os.path.exists(temp_folder_path) and len(os.listdir(temp_folder_path))>0):
                shutil.rmtree(temp_folder_path)
            file_path = os.path.join(folder_path, file_name)

            # Extract the .tar.gz file
            with tarfile.open(file_path, 'r:gz') as tar:
                tar.extractall(temp_folder_path)

            # Collect all (time, SP) pairs from the extracted files
            data_pairs = read_ET_data_from_file(get_subfolder_path(get_subfolder_path(temp_folder_path)), appname)
            all_times.append(data_pairs)
    clear_folder(temp_folder_path)
    return all_times

def plot_and_save_et_boxplot(data, plot_file_path, csv_file_path, scheduler_name, show_fig_time=3, normalize_coeff=1.0):
    """
    This function takes in time series data and corresponding SP values,
    creates a box plot, and saves the figure as a PDF file.

    Parameters:
    - data: A list of tuples where each tuple contains a time series and SP values.
    - file_path: The output file path where the PDF will be saved.
    """
    # Extract time series (assume same for all tuples) and collect sp values at each time step
    # Extract time series (assume same for all tuples) and collect SP values at each time step


    # Transpose SP values to get values at each time point for all series
    sp_values_at_times = np.array(data)  # Transpose to group SP values by time
    min_sp_length = min(len(sublist) for sublist in sp_values_at_times)
    sp_values_at_times = np.array([sublist[:min_sp_length] for sublist in sp_values_at_times])
    time_series = range(0, min_sp_length)
    sp_values_at_times = sp_values_at_times / normalize_coeff
    # sp_values_at_times =[x/normalize_coeff for x in sp_values_at_times]
    # Create the box plot
    # plt.figure(figsize=(10, 6))

    # Plot boxplot without specifying positions (positions auto-aligns with each SP value set)
    # plt.boxplot(sp_values_at_times, widths=0.5)
    df = {f'Time {int(time_series[i])}': sp_values_at_times[:,i] for i in range(len(time_series))}
    df = pd.DataFrame(df)

    # sns.boxplot(data=df, palette="Set2")
    sns.boxplot(data=df, saturation=0.75)

    # plt.xlim([30,300])
    # Select approximately 10 x-ticks (adjust if fewer points exist)
    num_xticks = 10
    tick_spacing = max(1, len(time_series) // num_xticks)  # Ensure spacing is at least 1
    selected_ticks = np.arange(0, len(time_series), tick_spacing)

    # Set the x-ticks as integers, showing only a subset of labels to avoid crowding
    plt.xticks(ticks=selected_ticks, labels=[int(time_series[i]) for i in selected_ticks])


    # Customize the plot
    plt.xlabel('Time')
    plt.ylabel('Execution Time')
    # plt.title('Box Plot of SP Values Over Time: ' + scheduler_name)
    plt.grid(linestyle="--")
    # plt.ylim([0.1, 1.05])

    # Save the figure to the specified file path as a PDF
    plt.savefig(plot_file_path , format='pdf')
    df.to_csv(csv_file_path, index=False)


    # Show the plot (optional)
    plt.show(block=False)
    plt.pause(show_fig_time)
    plt.close()

def plot_ET_distribution(appname, folder_path, extra_pdf_file_name_append=""):
    """
    folder_path: contain a list of .tar.gz files which compresses files of all_time_records
    """
    time_et_pairs = process_tar_files_for_ET(folder_path, appname)
    plot_and_save_et_boxplot(time_et_pairs, os.path.join(folder_path, "box_plot_ET_data_"+appname+"_"+\
                                                         extra_pdf_file_name_append+".pdf"),
                          os.path.join(folder_path, "et_data_"+appname+".csv"), appname, show_fig_time=0.1)

if __name__ =="__main__":
    # folder_path = os.path.join(OPT_SP_PROJECT_PATH, "../Experiments", "app_ET_data")
    # # app_names = ["SLAM", "SCHEDULER", "RRT", "TSP"]  # "MPC" is too slow
    # app_names = ["MPC"]
    # for app_name in app_names:
    #     plot_ET_distribution(app_name, folder_path=folder_path )

    optimizer_names = [ "optimizerIncremental"] # "optimizerBF",
    for optimizer in optimizer_names:
        folder_path = os.path.join(OPT_SP_PROJECT_PATH, "../Experiments", optimizer)
        # app_names = ["SLAM", "SCHEDULER", "RRT", "TSP"]  # "MPC" is too slow
        app_names = ["MPC"]
        for app_name in app_names:
            plot_ET_distribution(app_name, folder_path=folder_path, extra_pdf_file_name_append=optimizer)


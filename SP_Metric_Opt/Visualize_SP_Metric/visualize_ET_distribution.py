import os
import tarfile
import shutil
import tempfile
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
from dask.sizeof import sizeof

from SP_draw_fig_utils import OPT_SP_PROJECT_PATH
from draw_SP_current_scheduler import *

import os
import shutil
import random
from SP_Metric_Opt.Visualize_SP_Metric.box_plot_utils import get_subfolder_path, clear_folder


def read_ET_data_from_file( file_path, sample_threshold=10000, sample_rate=0.01):

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


def average_execution_time_with_intervals(time_stamps, execution_times, T, max_time_under_consider=-1):
    """
    Calculate average execution times in intervals of size T and return midpoints and averages.

    Parameters:
    - time_stamps (list or np.ndarray): 1D array of time stamps.
    - execution_times (list or np.ndarray): 1D array of measured execution times corresponding to the time stamps.
    - T (int or float): Interval size.

    Returns:
    - np.ndarray: 2D array where:
      - The first row contains the midpoint of each interval, i.e., i*T + T/2.
      - The second row contains the average execution time for each interval.
    """
    if len(time_stamps) != len(execution_times):
        raise ValueError("time_stamps and execution_times must have the same length.")

    if max_time_under_consider<0:
        max_time = time_stamps[-1]  # Assuming time_stamps are sorted
    else:
        max_time = max_time_under_consider
    if max_time > 1e5:
        raise ValueError("The time record file seems to contain too many timestamps or the timestamp does not have proper start time")
    num_intervals = int(np.ceil(max_time / T))

    midpoints = np.zeros(num_intervals)
    averages = np.zeros(num_intervals)

    for i in range(num_intervals):
        start = i * T
        end = (i + 1) * T
        midpoints[i] = start + T / 2
        # Find indices where time_stamps fall into the current interval
        indices = (time_stamps >= start) & (time_stamps < end)
        if indices.any():
            averages[i] = np.mean(execution_times[indices])
        else:
            averages[i] = 0  # No data in this interval

    return np.array([midpoints, averages])

def process_tar_files_for_ET(folder_path, appname, max_time_under_consider=835, horizon_granularity = 10):
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

            file_path = os.path.join(get_subfolder_path(get_subfolder_path(temp_folder_path)), appname + "_execution_time.txt")
            et_data = read_ET_data_from_file(  file_path)


            file_path = os.path.join(get_subfolder_path(get_subfolder_path(temp_folder_path)), appname + "_publisher.txt")
            publish_time = read_ET_data_from_file(  file_path)
            len_min = min(len(et_data), len(publish_time))
            et_data = et_data[:len_min]
            publish_time=publish_time[:len_min]

            all_times.append(average_execution_time_with_intervals(publish_time, et_data, horizon_granularity, max_time_under_consider = max_time_under_consider))
    clear_folder(temp_folder_path)
    return all_times

def plot_and_save_et_boxplot(data, plot_file_path, csv_file_path,  show_fig_time=3, normalize_coeff=1.0, xlim_min=30, xlim_max=850):
    """
    This function takes in time series data and corresponding SP values,
    creates a box plot, and saves the figure as a PDF file.

    Parameters:
    - data: A list of tuples where each tuple contains a time series and SP values.
    - file_path: The output file path where the PDF will be saved.
    """
    time_series = data[0][0]
    sp_values_at_times = np.array([np.array(t[1]) for t in data])  # Transpose to group SP values by time

    # Normalize and truncate to the shortest length
    min_sp_length = min(len(sublist) for sublist in sp_values_at_times)
    sp_values_at_times = np.array([sublist[:min_sp_length] for sublist in sp_values_at_times])
    time_series = time_series[:min_sp_length]
    sp_values_at_times = sp_values_at_times / normalize_coeff

    # Filter data within the desired range
    valid_indices = (time_series >= xlim_min) & (time_series <= xlim_max)
    time_series_filtered = time_series[valid_indices]
    sp_values_at_times_filtered = sp_values_at_times[:, valid_indices]

    # Prepare data for boxplot
    df = {f'Time {int(time_series_filtered[i])}': sp_values_at_times_filtered[:, i] for i in range(len(time_series_filtered))}
    df = pd.DataFrame(df)

    # Create the box plot
    sns.boxplot(data=df, saturation=0.75)

    # Select approximately 10 x-ticks
    num_xticks = 10
    tick_spacing = max(1, len(time_series_filtered) // num_xticks)
    selected_ticks = np.arange(0, len(time_series_filtered), tick_spacing)

    # Set the x-ticks as integers, showing only a subset of labels
    plt.xticks(ticks=selected_ticks, labels=[int(time_series_filtered[i]) for i in selected_ticks])

    # Customize the plot
    plt.xlabel('Time (s)')
    plt.ylabel('Execution Time')
    plt.grid(linestyle="--")

    # Save the figure and CSV
    plt.savefig(plot_file_path, format='pdf')
    df.to_csv(csv_file_path, index=False)

    # Show the plot
    plt.show(block=False)
    plt.pause(show_fig_time)
    plt.close()

def read_period(yaml_file_path, app_name):
    """
    Reads a YAML file and returns the period of the specified app.

    Args:
        yaml_file_path (str): Path to the YAML file.
        app_name (str): Name of the app.

    Returns:
        int or None: The period of the app if found, otherwise None.
    """
    try:
        with open(yaml_file_path, 'r') as file:
            data = yaml.safe_load(file)

        for task in data.get("tasks", []):
            if task.get("name") == app_name:
                return task.get("period")

        # App not found
        return None

    except FileNotFoundError:
        print(f"Error: File not found at {yaml_file_path}")
        return None
    except yaml.YAMLError as e:
        print(f"Error reading YAML file: {e}")
        return None

def plot_ET_distribution(appname, folder_path,scheduler_name, extra_pdf_file_name_append=""):
    """
    folder_path: contain a list of .tar.gz files which compresses files of all_time_records
    """
    time_et_pairs = process_tar_files_for_ET(folder_path, appname)

    plot_and_save_et_boxplot(time_et_pairs, os.path.join(folder_path, "box_plot_ET_data_"+appname+"_"+\
                                                         extra_pdf_file_name_append+".pdf"),
                          os.path.join(folder_path, "et_data_"+appname+".csv"),  show_fig_time=0.1)

if __name__ =="__main__":

    optimizer_names = [ "optimizerIncremental", "optimizerBF"]
    # optimizer_names = []
    for optimizer in optimizer_names:
        folder_path = os.path.join(OPT_SP_PROJECT_PATH, "../Experiments", optimizer)
        app_names = ["SLAM", "TSP", "RRT", "MPC", "SCHEDULER"]  # "MPC" is too slow
        app_names = ["SLAM"]
        # app_names = ["SCHEDULER_PUER_OPT"]
        for app_name in app_names:
            plot_ET_distribution(app_name, folder_path=folder_path, extra_pdf_file_name_append=optimizer, scheduler_name=optimizer)


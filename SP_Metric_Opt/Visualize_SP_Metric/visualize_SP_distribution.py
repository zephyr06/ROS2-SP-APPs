import os
import tarfile
import shutil
import tempfile
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
from SP_draw_fig_utils import OPT_SP_PROJECT_PATH
from draw_SP_current_scheduler import *


def get_subfolder_path(folder_path):
    # Check if the given path is a directory
    if not os.path.isdir(folder_path):
        raise ValueError(f"The path {folder_path} is not a directory.")

    # List all items in the directory
    items = os.listdir(folder_path)

    # Filter for subfolders
    subfolders = [os.path.join(folder_path, item) for item in items if os.path.isdir(os.path.join(folder_path, item))]

    # Check if there is exactly one subfolder
    if len(subfolders) == 1:
        return subfolders[0]
    elif len(subfolders) > 1:
        raise ValueError("The folder contains more than one subfolder.")
    else:
        raise ValueError("The folder does not contain any subfolders.")

def read_data_from_file(folder_path):
    data_folder_paths = {
        "Unknown": folder_path,
        }
    discard_early_time = 30  # at least 10 seconds, should be integer multilpe of scheduler's period
    horizon_granularity = 10  # 10 seconds
    task_config_file_path = os.path.join(folder_path,"task_characteristics.yaml")
    yaml_data = yaml.safe_load(open(task_config_file_path))
    horizon =  int(float(yaml_data['tasks'][0]['total_running_time'])/1e3) #  seconds

    return draw_and_saveSP_fig_single_run(data_folder_paths, discard_early_time, horizon_granularity, horizon, show_fig_time=0.1)

def process_tar_files(folder_path):
    temp_folder_path = os.path.join(OPT_SP_PROJECT_PATH, "temp_folder")

    # os.rmdir(temp_folder_path)
    all_time_sp_pairs = []

    # Loop through all files in the given folder
    for file_name in sorted(os.listdir(folder_path)):
        if file_name.endswith('.tar.gz'):
            if(os.path.exists(temp_folder_path)  and len(os.listdir(temp_folder_path))>0):
                shutil.rmtree(temp_folder_path)
            file_path = os.path.join(folder_path, file_name)

            # Extract the .tar.gz file
            with tarfile.open(file_path, 'r:gz') as tar:
                tar.extractall(temp_folder_path)

            # Collect all (time, SP) pairs from the extracted files
            data_pairs = read_data_from_file(get_subfolder_path(get_subfolder_path(temp_folder_path)))
            all_time_sp_pairs.append(data_pairs)

    return all_time_sp_pairs

def plot_and_save_boxplot(data, file_path):
    """
    This function takes in time series data and corresponding SP values,
    creates a box plot, and saves the figure as a PDF file.

    Parameters:
    - data: A list of tuples where each tuple contains a time series and SP values.
    - file_path: The output file path where the PDF will be saved.
    """
    # Extract time series (assume same for all tuples) and collect sp values at each time step
    # Extract time series (assume same for all tuples) and collect SP values at each time step
    time_series = data[0][0]

    # Transpose SP values to get values at each time point for all series
    sp_values_at_times = np.array([t[1] for t in data])  # Transpose to group SP values by time

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
    plt.ylabel('SP Values')
    plt.title('Box Plot of SP Values Over Time')
    plt.grid(True)
    plt.ylim([3.4,5.0])

    # Save the figure to the specified file path as a PDF
    plt.savefig(file_path, format='pdf')

    # Show the plot (optional)
    plt.show()


def main():
    scheduler_name = "RM"
    exp_res_folder = os.path.join(OPT_SP_PROJECT_PATH, "../Experiments", scheduler_name )
    time_sp_pairs = process_tar_files(exp_res_folder)
    plot_and_save_boxplot(time_sp_pairs, os.path.join(exp_res_folder, "box_plot_of_all_data.pdf"))

# Example usage:
# main('/path/to/your/folder')
if __name__ =="__main__":
    main()
import tarfile
import seaborn as sns
import pandas as pd

from SP_Metric_Opt.Visualize_SP_Metric.box_plot_utils import get_subfolder_path, clear_folder
from draw_SP_current_scheduler import *
import time

from SP_Metric_Opt.Visualize_SP_Metric.visualize_ET_distribution import plot_ET_distribution

import os
import shutil


def read_sp_data_from_file(folder_path):
    data_folder_paths = {
        "Unknown": folder_path,
        }
    discard_early_time = 30  # at least 10 seconds, should be integer multilpe of scheduler's period
    horizon_granularity = 10  # 10 seconds
    task_config_file_path = os.path.join(folder_path,"task_characteristics.yaml")
    yaml_data = yaml.safe_load(open(task_config_file_path))
    horizon =  int(float(yaml_data['tasks'][0]['total_running_time'])/1e3) #  seconds

    return draw_and_saveSP_fig_single_run(data_folder_paths, discard_early_time, horizon_granularity, horizon, show_fig_time=0.1)

def process_tar_files_sp(folder_path):
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
            start_time = time.time()
            with tarfile.open(file_path, 'r:gz') as tar:
                tar.extractall(temp_folder_path)
            end_time = time.time()
            # print("Execution time for extracting tar gz file: " + str(end_time-start_time))

            # Collect all (time, SP) pairs from the extracted files
            data_pairs = read_sp_data_from_file(get_subfolder_path(get_subfolder_path(temp_folder_path)))
            all_time_sp_pairs.append(data_pairs)
    clear_folder(temp_folder_path)
    return all_time_sp_pairs

def plot_and_save_boxplot_sp(data, plot_file_path, csv_file_path, scheduler_name, show_fig_time=3, normalize_coeff=5.0):
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
    sp_values_at_times = np.array([np.array(t[1]) for t in data])  # Transpose to group SP values by time
    min_sp_length = min(len(sublist) for sublist in sp_values_at_times)
    sp_values_at_times = np.array([sublist[:min_sp_length] for sublist in sp_values_at_times])
    time_series = time_series[:min_sp_length]
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
    plt.ylabel('SP Values')
    # plt.title('Box Plot of SP Values Over Time: ' + scheduler_name)
    plt.grid(linestyle="--")
    plt.ylim([0.1, 1.05])

    # Save the figure to the specified file path as a PDF
    plt.savefig(plot_file_path, format='pdf')
    df.to_csv(csv_file_path, index=False)


    # Show the plot (optional)
    plt.show(block=False)
    plt.pause(show_fig_time)
    plt.close()


def analyze_one_scheduler(scheduler_name = "RM"):
    exp_res_folder = os.path.join(OPT_SP_PROJECT_PATH, "../Experiments", scheduler_name )
    time_sp_pairs = process_tar_files_sp(exp_res_folder)
    plot_and_save_boxplot_sp(time_sp_pairs, os.path.join(exp_res_folder, "box_plot_of_all_data_" + scheduler_name + ".pdf"),
                             os.path.join(exp_res_folder, "sp_data.csv"), scheduler_name, show_fig_time=0.1)

def analyze_all_schedulers(scheduler_names):
    for scheduler_name in scheduler_names:
        analyze_one_scheduler(scheduler_name)

def plot_avg_line_for_all_methods(scheduler_names,plot_file_path,show_fig_time=3):
    # Create a figure for the plot
    # plt.figure(figsize=(10, 6))
    colors = sns.color_palette("Set2", len(scheduler_names))  # Generate a unique color for each folder

    for i, scheduler_name in enumerate(scheduler_names):
        folder = os.path.join(OPT_SP_PROJECT_PATH, "../Experiments", scheduler_name )
        csv_path = os.path.join(folder, 'sp_data.csv')  # Assuming CSV files are named 'data.csv'

        # Read the CSV file into a pandas DataFrame
        df = pd.read_csv(csv_path)

        # Average the values for each column (each time point)
        averaged_data = df.mean()

        # Extract the time points and averaged data
        time_points = [int(col.split(' ')[1]) for col in df.columns]
        values = averaged_data.values

        # Plot the averaged data
        # plt.plot(time_points, values, label=scheduler_name)
        sns.lineplot(x=time_points, y=values, label=scheduler_name, color=colors[i], linewidth=2)

    # Add labels and title
    plt.xlabel('Time')
    plt.ylabel('Average SP Value')
    # plt.title('Averaged Data for Each Scheduler')
    plt.legend()
    plt.grid(linestyle="--")
    plt.savefig(plot_file_path, format='pdf')

    # Display the plot
    plt.show(block=False)
    plt.pause(show_fig_time)
    plt.close()


# Example usage:
# main('/path/to/your/folder')
if __name__ =="__main__":
    scheduler_names = ["RM_Slow", "RM_Fast",  "CFS",  "optimizerIncremental", "optimizerBF"]
    # scheduler_names = ["optimizerIncremental", "optimizerBF"]
    # scheduler_names = ["RM_Slow", "RM_Fast", "CFS"]

    analyze_all_schedulers(scheduler_names)
    plot_avg_line_for_all_methods(scheduler_names,plot_file_path=os.path.join(OPT_SP_PROJECT_PATH, "../Experiments","average_all.pdf"))

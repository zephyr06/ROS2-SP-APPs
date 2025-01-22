import os
import glob
import pandas as pd
import numpy as np


def calculate_average_performance(data_folder, output_csv):
    # Get all text files in the folder
    file_pattern = os.path.join(data_folder, "performance_data_*.txt")
    files = glob.glob(file_pattern)

    # Initialize variables to accumulate time and value pairs
    all_data = []

    # Process each file
    for file in files:
        # Read the file
        with open(file, 'r') as f:
            lines = f.readlines()

        # Parse the time-value pairs
        data = []
        for line in lines:
            time_str, value_str = line.strip().split(", ")
            time = float(time_str)
            value = float(value_str) if value_str != "inf" else np.inf
            data.append((time, value))

        all_data.append(data)

    # Convert the list of tuples into a DataFrame
    df_list = []
    for data in all_data:
        df = pd.DataFrame(data, columns=["Time", "Value"])
        df_list.append(df)

    # Concatenate all DataFrames
    combined_df = pd.concat(df_list)

    # Group by 'Time' and calculate the average 'Value' for each time
    avg_df = combined_df.groupby("Time").mean()

    # Rename columns to 'CPU_time' and 'path_cost'
    avg_df.columns = ["path_cost"]

    # Save to CSV with ";" as the separator
    avg_df.to_csv(output_csv, sep=";", index=True)
    return avg_df



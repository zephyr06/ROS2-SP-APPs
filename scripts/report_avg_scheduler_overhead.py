import os

def calculate_average_execution_time(file_path):
    if os.path.exists(file_path):
        with open(file_path, "r") as file:
            execution_times = [float(line.split("::")[-1]) for line in file.readlines()]
            average_execution_time = sum(execution_times) / len(execution_times)
            return average_execution_time
    else:
        return None

file_path = tsp_config_path = os.path.join(os.path.dirname(__file__), '../all_time_records/SCHEDULER_execution_time.txt')
average_execution_time = calculate_average_execution_time(file_path)

if average_execution_time is not None:
    print(f"Average scheduler execution time: {average_execution_time:.5f} seconds")
else:
    print("File not found.")
import os


def calculate_average_execution_time(file_path):
    if os.path.exists(file_path):
        with open(file_path, "r") as file:
            execution_times = [float(line.split("::")[-1]) for line in file.readlines()]
            return sum(execution_times) / len(execution_times)
    return None


def main():
    file_path = os.path.join(os.path.dirname(__file__), "../all_time_records/SCHEDULER_execution_time.txt")
    average_execution_time = calculate_average_execution_time(file_path)
    if average_execution_time is not None:
        print(f"Average scheduler execution time: {average_execution_time:.5f} seconds")
    else:
        print("File not found.")


if __name__ == "__main__":
    main()
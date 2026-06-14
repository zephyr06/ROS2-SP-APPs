import math

def response_time_analysis(tasks):
    """
    Perform fixed-priority response time analysis for a set of tasks.

    Args:
    - tasks (list of dict): Each task is represented as a dictionary with keys:
        'period': Task's period
        'execution': Task's worst-case execution time

    Returns:
    - list of float: Response time for each task
    """
    n = len(tasks)
    response_times = [0] * n

    for i in range(n):
        # Initial guess: Response time starts as its execution time
        R_i = tasks[i]['execution']
        while True:
            interference = 0
            for j in range(i):  # Higher priority tasks only
                interference += math.ceil(R_i / tasks[j]['period']) * tasks[j]['execution']

            new_R_i = tasks[i]['execution'] + interference

            if new_R_i == R_i:
                break  # Converged
            if new_R_i > tasks[i]['period']:
                R_i = float('inf')  # Task is unschedulable
                break
            R_i = new_R_i

        response_times[i] = R_i

    return response_times

# Example usage
tasks = [
    {'period': 4, 'execution': 1},
    {'period': 10, 'execution': 9},
    # {'period': 50, 'execution': 2.5},
    # {'period': 2000, 'execution': 1910}
]

response_times = response_time_analysis(tasks)
print("Response times:", response_times)

# Feasibility check
for i, rt in enumerate(response_times):
    if rt > tasks[i]['period']:
        print(f"Task {i} is unschedulable")
    else:
        print(f"Task {i} is schedulable with response time {rt}")

import os
import re
import statistics

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def get_publisher_file(task_name, project_root=PROJECT_ROOT):
    return os.path.join(project_root, "all_time_records", "publisher_"+task_name+".txt")

# TODO: make all the names consistent


def get_subscriber_file(task_name, project_root=PROJECT_ROOT):
    return os.path.join(project_root, "all_time_records", task_name+"_subscriber"+".txt")


def get_response_time_file(task_name, project_root=PROJECT_ROOT):
    return os.path.join(project_root, "all_time_records", task_name+"_response_time"+".txt")


def write_response_time_file(task_name, response_time_list, project_root=PROJECT_ROOT):
    response_time_file = get_response_time_file(task_name, project_root)
    with open(response_time_file, 'w') as f:
        for response_time in response_time_list:
            f.write(str(response_time) + '\n')
    f.close()


def clear_response_time_file(task_name_list, project_root=PROJECT_ROOT):
    for task_name in task_name_list:
        response_time_file = get_response_time_file(task_name, project_root)
        with open(response_time_file, 'w') as f:
            f.write('')


def get_publisher_pair(line):
    if (line[0:5] == "Start"):
        return None, None
    words = line.split(' ')
    word = words[-1]
    pattern = "::"
    for i in range(len(word)-1):
        if word[i] == ':' and word[i+1] == ':':
            index = int(word[0:i])
            time = float(word[i+2:-1])
            return index, time
    else:
        return None, None


def get_start_time(line):
    words = line.split('::')
    return float(words[-1][:-1])


def generate_response_time_file(task_name, project_root=PROJECT_ROOT):
    publisher_file = get_publisher_file(task_name, project_root)
    subscriber_file = get_subscriber_file(task_name, project_root)
    publisher_pairs = []
    response_time_list = []
    publisher_time_base = 0
    subscriber_time_base = 0
    with open(publisher_file, 'r') as f:
        publisher_lines = f.readlines()
        publisher_time_base = get_start_time(publisher_lines[0])
        for line in publisher_lines:
            index, time = get_publisher_pair(line)
            if index is not None and time is not None:
                publisher_pairs.append((index, time))

    with open(subscriber_file, 'r') as f:
        subscriber_lines = f.readlines()
        subscriber_time_base = get_start_time(subscriber_lines[0])
        for line in subscriber_lines:
            index, time = get_publisher_pair(line)
            if index is not None and time is not None:
                for pair_pub in publisher_pairs:
                    if pair_pub[0] == index:
                        response_time = float(time) + \
                            subscriber_time_base - \
                            (publisher_time_base+pair_pub[1])
                        response_time_list.append(response_time*1000)

    write_response_time_file(task_name, response_time_list, project_root)


def print_task_stat(task):
    response_time_list = []
    file_name = get_response_time_file(task, PROJECT_ROOT)
    with open(file_name, 'r') as file:
        # Read lines from the file and convert them to floats
        response_time_list = [float(line.strip()) for line in file]
    print(task)
    print("Average:", statistics.mean(response_time_list))
    print("std:", statistics.stdev(response_time_list))
    print("Minimum:", min(response_time_list))
    print("Maximum:", max(response_time_list),"\n")

if __name__ == "__main__":
    task_name = ['rrt', 'mpc', 'tsp', 'slam']
    # task_name = ['rrt', 'mpc', 'tsp']
    clear_response_time_file(task_name)
    for task in task_name:
        generate_response_time_file(task)
        print_task_stat(task)
    

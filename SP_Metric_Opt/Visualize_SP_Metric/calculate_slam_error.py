import os
import bisect
from SP_draw_fig_utils import OPT_SP_PROJECT_PATH

class SLAMData:
    def __init__(self, timestamp, x, y, z, quaternion):
        ''' quaternion saves the orientation as four float-point variables'''
        self.timestamp = timestamp
        self.x = x
        self.y = y
        self.z = z
        self.quaternion = quaternion

    def __str__(self):
        return f"Timestamp: {self.timestamp}, X: {self.x}, Y: {self.y}, Z: {self.z}, Quaternion: {self.quaternion}"

def read_slam_data(file_path):
    slam_data_dict = {}
    with open(file_path, "r") as f:
        for line in f:
            if(line[0] == "#"):
                continue
            line = line.strip().split()
            if len(slam_data_dict) == 0:
                base_x, base_y, base_z = float(line[1]), float(line[2]), float(line[3])
            slam_data_dict[float(line[0])] = SLAMData(float(line[0]), float(line[1])-base_x, float(line[2])-base_y, float(line[3])-base_z, [float(x) for x in line[4:]])
    return slam_data_dict

def read_time_stamps_from_association(file_path):
    time_stamps = []
    with open(file_path, "r") as f:
        for line in f:
            elements = line.split()
            time_stamps.append(float(elements[0]))
            # if len(elements) >= 3:
            #     time_stamps.append(float(elements[2]))
    time_stamps.sort()
    return time_stamps



def interpolate(x1, x2, y1, y2, x_target):
    return y1 + (y2 - y1) / (x2 - x1) * (x_target - x1)

def read_xyz_from_slam_dict(time_stamp_target, slam_data_dict, dict_key_list):
    if time_stamp_target in slam_data_dict:
        return slam_data_dict[time_stamp_target].x, slam_data_dict[time_stamp_target].y, slam_data_dict[time_stamp_target].z
    else:
        index_smaller_than_target = bisect.bisect_left(dict_key_list, time_stamp_target)-1
        if index_smaller_than_target < 0:
            raise Exception("The target time stamp is smaller than the smallest time stamp in the slam data.")
        
        time_stamp_smaller = dict_key_list[index_smaller_than_target]
        if index_smaller_than_target < len(slam_data_dict) - 1:
            time_stamp_larger = dict_key_list[index_smaller_than_target+1]
        else:
            if len(dict_key_list) < 2:
                raise Exception("The slam data must have at least two time stamps!")
            time_stamp_smaller = dict_key_list[-2]
            time_stamp_larger = dict_key_list[-1]
        
        x = interpolate(time_stamp_smaller, time_stamp_larger, slam_data_dict[time_stamp_smaller].x, slam_data_dict[time_stamp_larger].x, time_stamp_target)
        y = interpolate(time_stamp_smaller, time_stamp_larger, slam_data_dict[time_stamp_smaller].y, slam_data_dict[time_stamp_larger].y, time_stamp_target)
        z = interpolate(time_stamp_smaller, time_stamp_larger, slam_data_dict[time_stamp_smaller].z, slam_data_dict[time_stamp_larger].z, time_stamp_target)
        return x, y, z

def mean_squared_error(predictions, targets):
    # Calculate the squared errors
    squared_errors = [(p - t) ** 2 for p, t in zip(predictions, targets)]
    
    # Calculate the mean squared error
    mse = sum(squared_errors) / len(predictions)
    
    return mse

def calculate_trajectory_error(actual_data_dict, ground_truth_dict, time_stamps, ref_interval_start_time, ref_interval_finish_time):
    error_list =[]
    ground_truth_keys = list(ground_truth_dict.keys())
    actual_data_keys = list(actual_data_dict.keys())
    time_stamp_min = min(ground_truth_keys)
    time_stamp_max = max(ground_truth_keys)
    time_stamp_min = max(time_stamp_min,time_stamp_min + ref_interval_start_time)
    time_stamp_max = min(time_stamp_max, time_stamp_min + ref_interval_finish_time)
    for time_stamp in time_stamps:
        if  time_stamp_min <= time_stamp  and time_stamp <= time_stamp_max:
            ground_truth_data = read_xyz_from_slam_dict(time_stamp, ground_truth_dict, ground_truth_keys)  
            actual_data =  read_xyz_from_slam_dict(time_stamp, actual_data_dict, actual_data_keys)
            error_list.append(mean_squared_error(actual_data, ground_truth_data))
    if len(error_list) == 0:
        # raise Exception("No time stamps are in the interval")
        return 0
    return sum(error_list)/len(error_list)



if __name__ == "__main__":

    association_file_path = os.path.join(os.path.dirname(os.path.dirname(OPT_SP_PROJECT_PATH)),"SP_Scheduler_Stack/YOLO-DynaSLAM/Examples/RGB-D/associations/fr3_walking_xyz.txt")

    ground_truth_file_path = os.path.join(OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "slam_ground_truth_tum.txt")

    # slam_output_file_path = os.path.join(OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "data", "CameraTrajectory_bf.txt")
    slam_output_file_path = os.path.join(OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "data", "CameraTrajectory_cfs.txt")

    time_stamps = read_time_stamps_from_association(association_file_path)
    slam_dict = read_slam_data(slam_output_file_path)
    ground_truth_dict = read_slam_data(ground_truth_file_path)
    trajectory_error = calculate_trajectory_error(slam_dict, ground_truth_dict, time_stamps,ref_interval_start_time=0, ref_interval_finish_time = 10)
    print("MSE trajectory error: ", trajectory_error)


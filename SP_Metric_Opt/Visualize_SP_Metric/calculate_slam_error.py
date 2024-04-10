import os

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
            line = line.strip().split()
            slam_data_dict[float(line[0])] = SLAMData(float(line[0]), float(line[1]), float(line[2]), float(line[3]), [float(x) for x in line[4:]])
    return slam_data_dict

def read_time_stamps_from_association(file_path):
    time_stamps = []
    with open(file_path, "r") as f:
        for line in f:
            elements = line.split()
            time_stamps.append(float(elements[0]))
            if len(elements) >= 3:
                time_stamps.append(float(elements[2]))
    time_stamps.sort()
    return time_stamps

if __name__ == "__main__":

    association_file_path = os.path.join(os.path.dirname(OPT_SP_PROJECT_PATH),"SP_Scheduler_Stack/YOLO-DynaSLAM/Examples/RGB-D/associations/fr3_walking_xyz.txt")

    ground_truth_file_path = os.path.join(OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "slam_ground_truth_tum.txt")

    slam_output_file_path = os.path.join(OPT_SP_PROJECT_PATH, "Visualize_SP_Metric","data", "CameraTrajectory_cfs.txt")


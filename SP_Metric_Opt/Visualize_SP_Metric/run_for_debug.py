
from SP_draw_fig_utils import *
import numpy as np
from collections import Counter
import pytest
from calculate_slam_error import *

association_file_path = "/home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Visualize_SP_Metric/data_for_test/fr3_walking_xyz.txt"

ground_truth_file_path = os.path.join(OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "slam_ground_truth_tum.txt")

slam_output_file_path = os.path.join(OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "data", "CameraTrajectory_bf.txt")

time_stamps = read_time_stamps_from_association(association_file_path)
time_stamps = time_stamps[0:3]
slam_dict = read_slam_data(slam_output_file_path)
ground_truth_dict = read_slam_data(ground_truth_file_path)
error_list = calculate_trajectory_error(slam_dict, ground_truth_dict, time_stamps)
assert error_list == pytest.approx(2.415e-5, abs=1e-7)
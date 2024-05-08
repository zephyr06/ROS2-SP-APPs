import pytest
import numpy as np
from SP_draw_fig_utils import *
from slam_analysis_utils import *

def interpolate_sp_for_test(x):
    minval = -1.484
    maxval = 0.405465
    return (x - minval) / (maxval - minval)

def test_read_period():
    task_set_config = os.path.join(
        OPT_SP_PROJECT_PATH, "TaskData/test_robotics_v4.yaml")
    verify_task_set_config(task_set_config)
    app_name2period = get_app2period(task_set_config)
    # assert app_name2period['TSP'] == 1.0
    assert app_name2period['MPC'] == 0.5
    assert app_name2period['RRT'] == 1.0
    assert app_name2period['SLAM'] == 1.0

def test_load_publish_data():
    task = TaskInfo('MPC', 1.0)
    data_folder_path = os.path.join(
        OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "data_for_test")
    task.load_publish_data(data_folder_path)
    assert task.publisher_offset == pytest.approx(1711401393.7642669678)
    assert task.publisher_index2data[0] == pytest.approx(0.61328400)

    task.load_subscribe_data(data_folder_path)
    assert task.subscriber_offset == pytest.approx(1711401393.764341116)
    assert task.subscriber_index2data[0] == pytest.approx(0.750322999)

def test_get_response_time():
    task = TaskInfo('MPC', 1.0)
    data_folder_path = os.path.join(
        OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "data_for_test")
    task.load_publish_data(data_folder_path)
    task.load_subscribe_data(data_folder_path)
    response_time_index2data = task.get_response_time_index2data()
    assert response_time_index2data[0] == pytest.approx(0.13703999999999998, abs=1e-3)
    assert response_time_index2data[100] == pytest.approx(50.7210350 - 50.614, abs=1e-3)

def test_get_invalid_response_time():
    task = TaskInfo('RRT', 1.0)
    data_folder_path = os.path.join(
        OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "data_for_test")
    task.load_publish_data(data_folder_path)
    task.load_subscribe_data(data_folder_path)
    response_time_index2data = task.get_response_time_index2data()
    assert response_time_index2data[345] == pytest.approx(1e9, abs=1e-3)


def test_publisher_index2actual_time():
    task = TaskInfo('MPC', 1.0)
    data_folder_path = os.path.join(
        OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "data_for_test")
    task.load_publish_data(data_folder_path)
    assert task.publisher_index2actual_time(0) == pytest.approx(1711401393.7642669678)
    assert task.publisher_index2actual_time(100) == pytest.approx(1711401393.7642669678 + 100 * 1.0)

def test_normalize_offsets():
    tasks_name_list = ['TSP', 'RRT', 'SLAM', 'MPC']

    task_set_config = os.path.join(
        OPT_SP_PROJECT_PATH, "TaskData/test_robotics_v4.yaml")
    app_name2period = get_app2period(task_set_config)

    data_folder_path = os.path.join(
        OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "data_for_test")
    
    tasks_name_to_info = get_task_set_info(tasks_name_list, app_name2period, data_folder_path)
    assert tasks_name_to_info['SLAM'].publisher_offset == pytest.approx(0.0, abs=1e-4)
    assert tasks_name_to_info['SLAM'].subscriber_offset == pytest.approx(12, abs=1e0)
    assert tasks_name_to_info['MPC'].publisher_offset == pytest.approx(0.0, abs=1e-2)
    assert tasks_name_to_info['MPC'].subscriber_offset == pytest.approx(0.0, abs=1e-2)

def test_get_sp_value_list():
    tasks_name_list = ['TSP', 'RRT', 'SLAM', 'MPC']

    task_set_config = os.path.join(
        OPT_SP_PROJECT_PATH, "Visualize_SP_Metric/data_for_test2/task_characteristics.yaml")
    app_name2period = get_app2period(task_set_config)

    data_folder_path = os.path.join(
        OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "data_for_test2")
    
    tasks_name_to_info = get_task_set_info(tasks_name_list, app_name2period, data_folder_path)
    sp_value_list = get_sp_value_list(tasks_name_list, tasks_name_to_info, 1000, 10, 0, task_set_config)
    # only one value; chance of four tasks to miss DDL:
    # TSP: 0.0, RRT: 1.0, SLAM: 1.0, MPC: 0.2
    # TSP's performance term is 0.533333
    exp_sp = (1+1)*interpolate_sp_for_test(-0.01*np.exp(10*0.5)) + interpolate_sp_for_test(np.log(1.5))*(0.5333+1)
    assert sp_value_list == pytest.approx([exp_sp], abs=1e-2)

def test_read_slam_data():
    slam_output_file_path = os.path.join(OPT_SP_PROJECT_PATH, "Visualize_SP_Metric","data", "CameraTrajectory_cfs.txt")
    slam_dict = read_slam_data(slam_output_file_path)
    assert slam_dict[1341846313.592026].x == pytest.approx(0.0)
    assert slam_dict[1341846313.592026].y == pytest.approx(0.0)
    assert slam_dict[1341846313.592026].z == pytest.approx(0.0)

    assert slam_dict[1341846315.161941].x == pytest.approx(-0.468015879)
    assert slam_dict[1341846315.161941].y == pytest.approx(0.175357431)
    assert slam_dict[1341846315.161941].z == pytest.approx(0.004894138)

def test_read_time_stamps_from_association():
    file_path = "/home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Visualize_SP_Metric/data_for_test/fr3_walking_xyz.txt"
    time_stamps = read_time_stamps_from_association(file_path)
    assert time_stamps[0] == pytest.approx(1341846313.592026)
    assert time_stamps[1] == pytest.approx(1341846313.592088)
    assert time_stamps[2] == pytest.approx(1341846313.654184)

def test_read_xyz_from_slam_dict():
    slam_output_file_path = os.path.join(OPT_SP_PROJECT_PATH, "Visualize_SP_Metric","data", "CameraTrajectory_bf.txt")
    slam_dict = read_slam_data(slam_output_file_path)
    dict_key_list = list(slam_dict.keys())
    
    time_stamp1 = 1341846313.592026
    x1, y1, z1 = read_xyz_from_slam_dict(time_stamp1, slam_dict, dict_key_list)
    assert x1 == pytest.approx(0)
    assert y1 == pytest.approx(0)
    assert z1 == pytest.approx(0)

    time_stamp2 = 1341846313.654184
    x2, y2, z2 = read_xyz_from_slam_dict(time_stamp2, slam_dict, dict_key_list)
    assert x2 == pytest.approx(0.011087629)
    assert y2 == pytest.approx(-0.001503026)
    assert z2 == pytest.approx(-0.003074664)

    time_stamp3 = 1341846313.6
    x3, y3, z3 = read_xyz_from_slam_dict(time_stamp3, slam_dict, dict_key_list)
    assert x3 == pytest.approx(x2*(time_stamp3-time_stamp1)/(time_stamp2-time_stamp1))
    assert y3 == pytest.approx(y2*(time_stamp3-time_stamp1)/(time_stamp2-time_stamp1))
    assert z3 == pytest.approx(z2*(time_stamp3-time_stamp1)/(time_stamp2-time_stamp1))

    time_stamp4, x4, y4, z4 = 1341846325.450747, 0.198163196, 0.014465467, 0.140189826
    time_stamp5, x5, y5, z5 = 1341846325.485954, 0.201328710, 0.013628194, 0.137425780
    time_stamp6 = 1341846326.0
    x6, y6, z6 = read_xyz_from_slam_dict(time_stamp6, slam_dict, dict_key_list)
    assert x6 == pytest.approx(x4 + (x5-x4)/(time_stamp5-time_stamp4)*(time_stamp6-time_stamp4))
    assert y6 == pytest.approx(y4 + (y5-y4)/(time_stamp5-time_stamp4)*(time_stamp6-time_stamp4))
    assert z6 == pytest.approx(z4 + (z5-z4)/(time_stamp5-time_stamp4)*(time_stamp6-time_stamp4))


def test_calculate_trajectory_error2():
    association_file_path = "/home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Visualize_SP_Metric/data_for_test/fr3_walking_xyz.txt"
    ground_truth_file_path = os.path.join(OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "slam_ground_truth_tum.txt")
    slam_output_file_path = os.path.join(OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "data", "CameraTrajectory_bf.txt")

    time_stamps = read_time_stamps_from_association(association_file_path)
    time_stamps = time_stamps[0:3]
    slam_dict = read_slam_data(slam_output_file_path)
    ground_truth_dict = read_slam_data(ground_truth_file_path)
    error_list = calculate_trajectory_error(slam_dict, ground_truth_dict, time_stamps,ref_interval_start_time=0, ref_interval_finish_time=1e9)
    assert error_list == pytest.approx(2.415e-5, abs =1e-7)

    error_list = calculate_trajectory_error(slam_dict, ground_truth_dict, time_stamps,ref_interval_start_time=0.0164, ref_interval_finish_time=1e9)
    assert error_list == pytest.approx(1.9884e-5, abs =1e-7)

def test_get_trajectory_error_list():
    association_file_path = "/home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Visualize_SP_Metric/data_for_test/fr3_walking_xyz.txt"
    ground_truth_file_path = os.path.join(OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "slam_ground_truth_tum.txt")
    slam_output_file_path = os.path.join(OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "data", "CameraTrajectory_bf.txt")

    time_stamps = read_time_stamps_from_association(association_file_path)
    slam_dict = read_slam_data(slam_output_file_path)
    ground_truth_dict = read_slam_data(ground_truth_file_path)
    slam_trajectory_error_list = get_trajectory_error_list(2, 1, 0, slam_dict, ground_truth_dict, time_stamps)
    assert len(slam_trajectory_error_list) == 2
    assert slam_trajectory_error_list[1] == pytest.approx(1.9884e-5, abs =1e-7)

def test_get_execution_time_within_range():

    task_set_config = os.path.join(
        os.path.dirname(OPT_SP_PROJECT_PATH),"Visualize_SP_Metric", "data_for_test3", "task_characteristics.yaml")
    data_folder_path = os.path.join(
        OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "data_for_test3")
    app_name2period = get_app2period(task_set_config)
    task_name ='TSP'
    task_info = TaskInfo(task_name, app_name2period[task_name])
    task_info.load_publish_data(data_folder_path)
    task_info.load_subscribe_data(data_folder_path)
    task_info.load_execution_time_data(data_folder_path)
    ext_time1 = task_info.get_execution_time_within_range(0,21)
    print(ext_time1)
    assert len(ext_time1) == pytest.approx(10, abs=1e-2)



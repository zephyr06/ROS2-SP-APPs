
from SP_draw_fig_utils import *
from slam_analysis_utils import *   


if __name__ == "__main__":
    association_file_path = os.path.join(os.path.dirname(os.path.dirname(OPT_SP_PROJECT_PATH)),"SP_Scheduler_Stack/YOLO-DynaSLAM/Examples/RGB-D/associations/fr3_walking_xyz.txt")
    ground_truth_file_path = os.path.join(OPT_SP_PROJECT_PATH, "Visualize_SP_Metric", "slam_ground_truth_tum.txt")
    time_stamps = read_time_stamps_from_association(association_file_path)
    ground_truth_dict = read_slam_data(ground_truth_file_path)

    # These files are generated after running the computation tasks
    slam_trajectory_paths = {
        "CFS": os.path.join(OPT_SP_PROJECT_PATH,"Visualize_SP_Metric/data", "CameraTrajectory_cfs.txt"),
        "FIFO": os.path.join(OPT_SP_PROJECT_PATH,"Visualize_SP_Metric/data", "CameraTrajectory_bf.txt"),
        "RM": os.path.join(OPT_SP_PROJECT_PATH,"Visualize_SP_Metric/data", "CameraTrajectory_rm.txt"),
        }

    frame_granularity = 5 # draw a point every 5 frames
    total_frame = 2000  # 2000 frames at msot, actual frame depends on the provided data
    discard_early_frame = 0  # discard first 0 frames


    # TODO: add more tests!!!
    for method_name, trajectory_file_path in slam_trajectory_paths.items():
        actual_data_dict = read_slam_data(trajectory_file_path)
        slam_trajectory_error_list = get_trajectory_error_list(total_frame, frame_granularity, discard_early_frame, actual_data_dict, ground_truth_dict, time_stamps)
        x_axis = [i for i in range(0, len(slam_trajectory_error_list)*frame_granularity, frame_granularity)]
        plt.plot(x_axis, slam_trajectory_error_list, label = method_name)
        print(f"SLAM Trajectory MSE for {method_name}: {sum(slam_trajectory_error_list)/len(slam_trajectory_error_list)}")

    plt.legend(slam_trajectory_paths.keys())
    plt.xlabel("Frame indexes")
    plt.ylabel("SLAM Trajectory MSE")
    plt.show()
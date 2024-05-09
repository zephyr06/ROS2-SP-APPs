## ROS2 stack for Computation Aware Platform

### Dependencies
- ROS2 Foxy
- C++ 20
- Other dependencies for each ROS2 package inside the `/src` folder
- SP_Metric_Opt package: inside the `/SP_Metric_Opt` folder
- [ORB-SLAM2](https://github.com/zephyr06/ORB_SLAM2)

### How to launch the software stack
1. Go into `/SP_Metric_Opt` and build the SP_Metric_Opt package in release mode: [SP_Metric_Opt Build and Run](https://github.com/zephyr06/ROS2-SP-APPs/tree/main/SP_Metric_Opt#build-and-run)
1. Inside the workspace of this repo, build ROS2 Foxy packages: `colcon build`
1. Get root privilege: `sudo -s`
1. Source the overlay: `source install/setup.bash`
1. Go inside `scripts` folder, and launch all node by: `./start_system.sh CFS`
    - This script needs one input argument to decide the scheduler, accepted four schedulers are: CFS, RM, optimizerBF, optimizerIncremental
    - To stop the system, use `Ctrl + C` to send the signal. Only press once and wait for the bash to backup results.

### Check timing results
All timing results will be automatically backup inside the `/Experiments` folder with timestamp.

### Task configurations
Configurations of the tasks (application nodes) are stored inside two YAML files: `/all_time_records/task_characteristics.yaml` and `/src/real_time_manager/configs/local_cpu_and_priority.yaml`. All real-time nodes should acquire task set info from these two files. **Only** modify these files before you launch the ROS2 stack.

- Fixed tasks (nodes or applications) names: TSP, MPC, RRT, SLAM
- `/all_time_records/task_characteristics.yaml`

    Include following member, the statistics of execution time is from the latest data within a predefined time range.
    - id:
    - execution_time_mu: average of the execution time
    - execution_time_sigma: standard deviation of the execution time
    - execution_time_min: minimum value of the execution time
    - execution_time_max: maximum value of the execution time
    - period:
    - deadline:
    - processorId:
    - name:
    - sp_threshold: 0.5
    - sp_weight: 

- `/src/real_time_manager/configs/local_cpu_and_priority.yaml`

    Include following member, 
    - id:
    - name:
    - processorId: the cpu core to which the task is assigned
    - scheduling_policy: supported scheduling_policy: `SCHED_OTHER`, `SCHED_FIFO`
    - priority: 
        - Supported priority range:
            - `SCHED_OTHER`: [0,0] , this is CFS, only support priority level of 0.
            - `SCHED_FIFO` : [0,99], please use priorites starting from 1, the larger number, the higer priority.
        - The `talker` will be automatically added into the control list, with a priority higher than the max priority of other tasks (+5 is used). If all other tasks are CFS, then the `talker` is also CFS, otherwise, the `talker` will be FIFO.

### Other issues
1. Limited root space: since the real time manager needs to run in root mode, all ROS2 log file will be written into `/root/.ros/log` by default. Add the below command into `/root/.bashrc` to specify the log locations:
    ```
    export ROS_LOG_DIR=/home/nvidia/workspace/sdcard/.ros/log
    ```

1. If `chrt` coudn't change priority:
    - execute: `sysctl -w kernel.sched_rt_runtime_us=-1`

# Tips in improving reproducibility
- SLAM often (not always) takes much longer time to run for the first time after boosting the machine; after running it for one time, following runs usually are much faster. Since it is difficult to accurately reproduce the first run each time we do experiments, probably just use the following runs for measurement.
## ROS2 stack for Computation Aware Platform

### Dependencies
- ROS2 Foxy
- C++ 20
- Other dependencies for each ROS2 package inside the `/src` folder

### How to launch the software stack
1. Build the packages: `colcon build`
2. Get root privilege: `sudo -s`
3. Source the overlay: `source install/setup.bash`
4. Go inside `scripts` folder, and launch all node by: `./run_all_tasks.sh`

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
    - name:

- `/src/real_time_manager/configs/local_cpu_and_priority.yaml`

    Include following member, 
    - id:
    - name:
    - cpu_lists: a vector of the assigned cpus, need at least one cpu
    - scheduling_policy: supported scheduling_policy: `SCHED_OTHER`, `SCHED_FIFO`
    - priority: 
        - Supported priority range:
            - `SCHED_OTHER`: [0,0] , this is CFS, only support priority level of 0.
            - `SCHED_FIFO` : [0,99], please use priorites starting from 1, the larger number, the higer priority.
        - The `talker` will be automatically added into the control list, with a priority higher than the max priority of other tasks (+5 is used). If all other tasks are CFS, then the `talker` is also CFS, otherwise, the `talker` will be FIFO.

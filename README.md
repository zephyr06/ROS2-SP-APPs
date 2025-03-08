Firstly, the repo is not properly named because ROS2 is actually not needed.

### Dependencies
- C++ 20
- Other dependencies for each ROS2 package inside the `/src` folder
- SP_Metric_Opt package: inside the `/SP_Metric_Opt` folder
- [ORB-SLAM2](https://github.com/zephyr06/ORB_SLAM2)

### How to launch the software stack
1. Go into `/SP_Metric_Opt` and build the SP_Metric_Opt package in release mode: [SP_Metric_Opt Build and Run](https://github.com/zephyr06/ROS2-SP-APPs/tree/main/SP_Metric_Opt#build-and-run)
1. Get root privilege: `sudo -s`
1. Go inside `scripts` folder, and launch all node by: `./start_system.sh optimizerBF`
    - This script needs one input argument to decide the scheduler, accepted four schedulers are: CFS, RM, optimizerBF, optimizerIncremental
    - To stop the system, wait till the end or it would be better to reboot for performance profiling.

### Check timing results
All timing results will be automatically backup inside the `/Experiments` folder with timestamp.

### Task configurations
Configurations of the tasks (application nodes) are stored inside two YAML files: `/all_time_records/task_characteristics.yaml` and `/SP_Metric_Opt/applications/real_time_manager/configs/local_cpu_and_priority.yaml`. All real-time nodes should acquire task set info from these two files. 

- Fixed tasks (nodes or applications) names: TSP, MPC, RRT, SLAM
- Check the exampe provided in `/all_time_records/task_characteristics.yaml`

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
    - performance_records_time (optional): 
    - performance_records_perf (optional): If not specified, it will be treated as 1.0 in the performance function.

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

1. If `chrt` coudn't change priority:
    - execute: `sysctl -w kernel.sched_rt_runtime_us=-1`

# Tips in improving reproducibility
- SLAM's execution time distribution is sometimes random and unpredictable, especially if it runs with many file read/write. So try to either minimize I/O or figure out why does priority assignment influence it.

# Potential issues
- After finishing running `./start_systems.sh`, the code will terminate, but there seems to be one more process running in the background. So it may be better to kill the current terminal before running for another time. However, I also have not observe any performance influence yet.
- When the system is highly overloaded, the tasks cannot release new jobs strictly in a periodic way. This may have some influence on some results, though not big, I guess.
- Currently, SLAM's weight is 2, threshold is 0.9. In the future, it may be better to consider how to set up these parameters more appropriately.
- Cause-effect chains are not considered at this time, and mostly will not be added in the future.
- The current TSP code implementation may leak memory because some pointers are not freed explicitly. We try to avoid this issue, but it is not guaranteed.

# Observations
- If results are not good, anything could cause bug. When metric related to time goes wrong, more accurate profiling seems to be a good option.

# Time units
Mostly, we use microseconds (ms) as the time units. However, there are still some bad code which uses seconds as the time units.

# Generate time-performance coefficients
An example is provided in `SP_Metric_Opt/applications/tsp_solver_osm/regress_exp_data.py`.

# Run system multilple times
If you need to do so to get good results, potentially, there is something wrong. 

# Convenient scripts for data analysis preparation
Assume you use a host desktop to remotely run the experiments on a remote platform, and you obtained some results after running `scripts\run_multiple_times.sh`. In this case, you can use `Experiments\command_to_transmit_files.sh` to download the data from your remote platform to your local host. If the data is downloaded successfully by this script, you can then use `Experiments\load_exp_data.sh` to extract the data and put them into the `Experiments` folder. After it, you can use the visualization python scripts, `visualize_SP_distribution.py` and `visualize_ET_distribution.py` to draw the figures.

# SP Values analysis
- Use `SP_Metric_Opt/Visualize_SP_Metric/draw_SP_current_scheduler.py` to draw SP values figure for one experiment run. This script reads the data from `all_time_records`, and then plot figure results. The tasks under consideration is specified in the function `draw_and_saveSP_fig_single_run()`, for example, in the line `tasks_name_list = ['TSP', 'RRT', 'SLAM', 'MPC']`.
- Use `SP_MetricOpt/Visualize_SP_Metric/visualize_SP_distribution.py` to plot the figures of `SP Values` at different running time. Example output can be seen in the Paper's experiments section: a collection of figures that show the SP values measured when running different scheduling algorithms. This script reads data from the `Experiments` folder. Inside the `Experiments` folder, for each scheduler, there is one folder. For example, In `Experiments/CFS`, there could be 50 `all_time_records` tar.gz files, each obtained by compressing the `all_time_records` folder after finish running the experiments. `SP_MetricOpt/Visualize_SP_Metric/visualize_SP_distribution.py` reads data under such kind of settings, and records the analyzed data back to the schedulere folders.
- Note that the data above will be generated in the format after running the `scripts/run_multi_times.sh` and `Experiments/load_exp_data.sh`. 
- Use `SP_MetricOpt/Visualize_SP_Metric/visualize_ET_distribution.py` to plot the figures of execution time of each task at different running time. This script shows the window-average of execution time at different times. To use it, first store the data in the same format as above, and then specify `optimizer_names` argument and `app_names`.

# Traps
- most time measurements in the folder `all_time_records` are accurate, except `SCHEDULER_*.txt`. The profiling code is based on measuring the CPU time of a single core running a single thread, while schedulers involve running code in multilple cores. Therefore, it could be inaccurate up to 0.5 seconds, based on experiment observation.

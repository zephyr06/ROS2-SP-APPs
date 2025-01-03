# SP_Metric_Optimization


# Dependencies
- [CMake](https://cmake.org/download/)
- [Boost](https://www.boost.org/users/download/)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [GTSAM (optional, for unit tests only)](https://github.com/borglab/gtsam)
- [Google Test (optional, for unit tests only)](https://github.com/google/googletest)
- Other dependencies mentioned in the packages in `applications`.


# Build and Run
```
cd SP_Metric_Opt
mkdir release
cd release
cmake -DCMAKE_BUILD_TYPE=RELEASE ..
make -j4
```

## Modify task sets and configurations
Check and follow the yaml file such as `../all_time_records/task_characteristics.yaml`. Something more to notice when modifying the yaml file:
- The task id must be integers and continuous, such as 0, 1, 2, 3, ...
- All the time units are in milliseconds within the yaml file
- The `processorId` attribute could only be a single integer; lists are not supported
- `sp_threshold` refers to the acceptable probability to miss deadline
- `sp_weight` refers to the importance of one task; 0 means no influence at all, bigger number means higher influence during scheduling and optimization
- `execution_time_*` are measured by assuming the execution time distribution follows Gaussian distribution
- Optimization with chains are not supported very well at this time. It's better to set its weight as 0.

## Run the brute-force scheduler
You can run the brute-force scheduler by passing the path of task set configuration files and specifying the output file path
```
./tests/AnalyzePriorityAssignment --file_path TaskData/test_robotics_v13.yaml --output_file_path TaskData/pa_res_test_robotics_v1.txt
```
You can also specify the absolute path:
```
./tests/AnalyzePriorityAssignment --file_path /home/zephyr/Programming/task_characteristics.yaml --output_file_path /home/zephyr/Programming/pa_res_test_robotics_v1.txt
```

## Run the incremental scheduler
Please check the example provided in `/tests/AnalyzePriorityAssignmentIncrementalExample.cpp`. Scheduling with incremental scheduler still requires further coding works.

## Read the priority assignment results
Priority assignment results are saved and explained in the output file generated after running the scheduler, for example, `/TaskData/pa_res_test_robotics_v1.yaml`. Bigger integers mean higher priority. (This is contrary to the print output during optimization, if any)


# Result Visualization
## Visualize the SP metric
```
cd SP_Metric_Opt/Visualize_SP_Metric
python draw_SP_current_scheduler.py
```
Please modify the parameters provided in the main function in `/Visualize_SP_Metric/draw_SP_current_scheduler.py` based on the provided comments.

## Visualize the MSE of SLAM trajectories (maybe not well supported at this time)
```
cd SP_Metric_Opt/Visualize_SP_Metric
python draw_trajectory_error.py
```
Please modify the parameters provided in the main function in `/Visualize_SP_Metric/draw_trajectory_error.py` based on the provided comments. 
One issue: the time units of the obtained figure is based on the TUM dataset rather than the actual time unit in the real-world.

# Modify the task set and SP metric coefficients
You can modify the yaml file which describes the task set settings. Time units are in microseconds.

# Implmentation details
-  TSP's time limit is directly updated by the priority assignment optimization algorithm.
- Incremental optimzier partially relies on detecting the execution time distribution difference, which can be controlled by adjusting the thresholds in `Probabiliy.h` (`approx_equal()` and `FiniteDist::operator==()` function), 


# Run unit tests/check code usage
```
cd SP_Metric_Opt
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=DEBUG ..
make check.SP_OPT -j4
cd ../Visualize_SP_Metric
pytest
```

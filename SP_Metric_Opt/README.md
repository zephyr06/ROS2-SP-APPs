# SP_Metric_Optimization


# Dependencies
- [CMake](https://cmake.org/download/)
- [Boost](https://www.boost.org/users/download/)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [GTSAM (optional, for unit tests only)](https://github.com/borglab/gtsam)
- [Google Test (optional, for unit tests only)](https://github.com/google/googletest)



# Build and Run
```
cd SP_Metric_Opt
mkdir release
cd release
cmake -DCMAKE_BUILD_TYPE=RELEASE ..
make -j4
```

## Modify task sets and configurations
Check and follow the yaml file such as `TaskData/test_robotics_v13.yaml`. Something more to notice when modifying the yaml file:
- The task id must be integers and continuous, such as 0, 1, 2, 3, ...
- All the time units are in microseconds
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
Please check the example provided in `/tests/AnalyzePriorityAssignmentIncrementalExample.cpp`.

## Read the priority assignment results
Priority assignment results are saved and explained in the output file generated after running the scheduler, for example, `/TaskData/pa_res_test_robotics_v1.yaml`. Bigger integers mean higher priority. (This is contrary to the print output during optimization, if any)



# Visualize the SP metric
```
cd SP_Metric_Opt/Visualize_SP_Metric
python draw_SP_multi_figs.py
```
Please modify the parameters provided in the main function in `/Visualize_SP_Metric/draw_SP_multi_figs.py` based on the provided comments.



# Modify the task set and SP metric coefficients
You can modify the yaml file which describes the task set settings. Time units are in microseconds. An example yaml file is provided in `/TaskData/test_robotics_v6.yaml`.

# TODO
- Check SLAM code to see if it's possible to speed it up
- Figure out if it is okay to incorporate more about core assignments into experiments
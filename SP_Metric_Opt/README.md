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
## Run the brute-force scheduler
You can run the brute-force scheduler by passing the path of task set configuration files and specifying the output file path
```
./tests/AnalyzePriorityAssignment --file_path TaskData/test_robotics_v3.yaml --output_file_path TaskData/pa_res_test_robotics_v1.txt
```
You can also specify the absolute path:
```
./tests/AnalyzePriorityAssignment --file_path /home/zephyr/Programming/task_characteristics.yaml --output_file_path /home/zephyr/Programming/pa_res_test_robotics_v1.txt
```

## Run the incremental scheduler
Please check the example provided in `/tests/AnalyzePriorityAssignmentIncrementalExample.cpp`.

## Read the priority assignment results
Priority assignment results are saved and explained in the output file generated after running the scheduler, for example, `/TaskData/pa_res_test_robotics_v1.yaml`



# Visualize the SP metric
```
cd SP_Metric_Opt/Visualize_SP_Metric
python draw_SP_multi_figs.py
```
Please modify the parameters provided in the main function in `/Visualize_SP_Metric/draw_SP_multi_figs.py` based on the provided comments.



# Modify the task set and SP metric coefficients
You can modify the yaml file which describes the task set settings. Time units are in microseconds. An example yaml file is provided in `/TaskData/test_robotics_v3`.

#include <yaml-cpp/yaml.h>

#include <iostream>

#include "sources/Optimization/OptimizeSP_BF.h"
#include "sources/Optimization/OptimizeSP_TL_BF.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/argparse.hpp"
#include "sources/Utils/profilier.h"
#include "sources/Utils/readwrite.h"

using namespace std;
using namespace SP_OPT_PA;

int main(int argc, char *argv[]) {
    TimerType start_time = CurrentTimeInProfiler;

    argparse::ArgumentParser program("program name");
    program.add_argument("--file_path")
        .default_value(
            std::string("/home/zephyr/Programming/ROS2-SP-APPs/"
                        "all_time_records/task_characteristics.yaml"))
        .help(
            "the relative path of the yaml file that saves information about "
            "the tasks. Example: TaskData/test_robotics_v1.yaml. It is "
            "also okay to directly pass global path that starts with '/', such "
            "as /root/usr/slam.txt ");
    program.add_argument("--output_file_path")
        .default_value(std::string("TaskData/pa_res_test_robotics_v19.yaml"))
        .help(
            "the relative path of the file that saves priority assignment "
            "results. Example: TaskData/pa_res_test_robotics_v1.yaml. It is "
            "also okay to directly pass global path that starts with '/', such "
            "as /root/usr/slam.txt ");

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error &err) {
        std::cout << err.what() << std::endl;
        std::cout << program;
        exit(0);
    }

    string file_path = program.get<std::string>("--file_path");
    file_path = RelativePathToAbsolutePath(file_path);

    string output_file_path = program.get<std::string>("--output_file_path");
    output_file_path = RelativePathToAbsolutePath(output_file_path);

    DAG_Model dag_tasks = ReadDAG_Tasks(file_path);
    SP_Parameters sp_parameters = ReadSP_Parameters(file_path);

    // Perform optimization
    ResourceOptResult res =
        EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters);
    TimerType finish_time = CurrentTimeInProfiler;
    double time_taken = GetTimeTaken(start_time, finish_time);

    PriorityVec pa_opt = res.priority_vec;
    WritePriorityAssignments(output_file_path, dag_tasks.tasks, pa_opt,
                             time_taken);
    WriteTimeLimitToYamlOSM(
        res.id2time_limit[0]);  // only write TSP's time limit
    if (GlobalVariables::debugMode == 1) {
        PrintPriorityVec(dag_tasks.tasks, pa_opt);
        PrintTimer();
    }
}
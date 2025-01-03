#include <yaml-cpp/yaml.h>

#include <iostream>

#include "sources/Optimization/OptimizeSP_Incre.h"
#include "sources/Optimization/OptimizeSP_TL_Incre.h"
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
            std::string("/home/nvidia/workspace/sdcard/ROS2-SP-APPs/all_time_records/task_characteristics.yaml"))
        .help(
            "the relative path of the yaml file that saves information about "
            "the tasks. Example: TaskData/test_robotics_v1.yaml. It is "
            "also okay to directly pass global path that starts with '/', such "
            "as /root/usr/slam.txt ");
    program.add_argument("--output_file_path")
        .default_value(std::string("TaskData/pa_res_test_robotics_v1.yaml"))
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

    string dag1 =
        "/home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/TaskData/"
        "test_robotics_v21.yaml";
    string dag2 =
        "/home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/TaskData/"
        "test_robotics_v25.yaml";
    DAG_Model dag_tasks = ReadDAG_Tasks(file_path);
    SP_Parameters sp_parameters = ReadSP_Parameters(file_path);

    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);

    // read a DAG and optimize it for the first time
    PriorityVec pa_opt = opt.OptimizeFromScratch_w_TL(
        GlobalVariables::Layer_Node_During_Incremental_Optimization);

    TimerType finish_time = CurrentTimeInProfiler;
    // PrintPriorityVec(dag_tasks.tasks, pa_opt);
    double time_taken = GetTimeTaken(start_time, finish_time);
    WritePriorityAssignments(output_file_path, dag_tasks.tasks, pa_opt,
                             time_taken);
    time_taken = GetTimeTaken(start_time, finish_time);

    std::cout<<"Total time taken: "<<time_taken<<"\n";
    // to perform incremental optimization for 5 more times
    for (int i = 0; i < 10; i++) {
        // read the updated DAG
        if (i % 2 == 0)
            dag_tasks = ReadDAG_Tasks(dag1);
        else
            dag_tasks = ReadDAG_Tasks(dag2);
        // dag_tasks = ReadDAG_Tasks(file_path);
        pa_opt = opt.OptimizeIncre_w_TL(
            dag_tasks,
            GlobalVariables::Layer_Node_During_Incremental_Optimization);
        // time_taken = GetTimeTaken(start_time, finish_time);
        // PrintPriorityVec(dag_tasks.tasks, pa_opt);
        WritePriorityAssignments(output_file_path, dag_tasks.tasks, pa_opt, -1);
    }
    finish_time = CurrentTimeInProfiler;
    double time_taken_all = GetTimeTaken(start_time, finish_time);
    // PrintPriorityVec(dag_tasks.tasks, pa_opt);
    WritePriorityAssignments(output_file_path, dag_tasks.tasks, pa_opt,
                             time_taken);

    std::cout << "Total running time from scratch: " << time_taken
              << " seconds\n";
    std::cout << "Average for 10 incremental: "
              << (time_taken_all - time_taken) / 10.0 << " seconds\n";
}
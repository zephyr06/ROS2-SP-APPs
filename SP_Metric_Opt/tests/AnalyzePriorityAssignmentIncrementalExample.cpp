#include <yaml-cpp/yaml.h>

#include <iostream>

#include "sources/Optimization/OptimizeSP_Incre.h"
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
        .default_value(std::string("TaskData/test_robotics_v6.yaml"))
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

    string output_file_path = GlobalVariables::PROJECT_PATH +
                              program.get<std::string>("--output_file_path");
    output_file_path = RelativePathToAbsolutePath(output_file_path);

    DAG_Model dag_tasks = ReadDAG_Tasks(file_path);
    SP_Parameters sp_parameters = ReadSP_Parameters(file_path);

    OptimizePA_Incre opt(dag_tasks, sp_parameters);

    // read a DAG and optimize it for the first time
    PriorityVec pa_opt = opt.OptimizeFromScratch(
        GlobalVariables::Layer_Node_During_Incremental_Optimization);

    TimerType finish_time = CurrentTimeInProfiler;
    double time_taken = GetTimeTaken(start_time, finish_time);
    // PrintPriorityVec(dag_tasks.tasks, pa_opt);
    WritePriorityAssignments(output_file_path, dag_tasks.tasks, pa_opt,
                             time_taken);

    // to perform incremental optimization for 5 more times
    for (int i = 0; i < 5; i++) {
        // read the updated DAG
        dag_tasks = ReadDAG_Tasks(file_path);
        pa_opt = opt.OptimizeIncre(dag_tasks);
        time_taken = GetTimeTaken(start_time, finish_time);
        // PrintPriorityVec(dag_tasks.tasks, pa_opt);
        WritePriorityAssignments(output_file_path, dag_tasks.tasks, pa_opt,
                                 time_taken);
    }
}
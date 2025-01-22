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

#if defined(RYAN_HE_CHANGE)
// RYAN_HE: hardcord project root which is ugly too
// but user can provide absolute path anyway
std::string PROJECT_ROOT = std::string("/mnt/f/explore/ROS2-SP-APPs/");
#endif

// how to run
// specify input and output file path in command line
// --file_path TestData/test_robotics_v1.yaml
// --output_file_path TestData/pa_res_test_robotics_v1.yaml
int main(int argc, char *argv[]) {
#if defined(RYAN_HE_CHANGE_DEBUG)
    GlobalVariables::debugMode = DBG_PRT_MSK_ALL; 
    GlobalVariables::debugMode &= ~DBG_PRT_MSK_RTA;
    GlobalVariables::debugMode = DBG_PRT_MSK_OptimizeSP_TL_BF | DBG_PRT_MSK_MAIN;
#endif

    // in sources/Utils/profilier.h
    // #define CurrentTimeInProfiler std::chrono::high_resolution_clock::now()
    TimerType start_time = CurrentTimeInProfiler;

    argparse::ArgumentParser program("program name");
    program.add_argument("--file_path")
        .default_value(
#if defined(RYAN_HE_CHANGE)
			PROJECT_ROOT + std::string("all_time_records/task_characteristics.yaml"))
#else		
            std::string("/home/nvidia/workspace/sdcard/ROS2-SP-APPs/all_time_records/task_characteristics.yaml"))
#endif
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
#if defined(RYAN_HE_CHANGE_DEBUG)
    if (GlobalVariables::debugMode & DBG_PRT_MSK_MAIN)
        std::cout << "####main: call EnumeratePA_with_TimeLimits ..." << std::endl;
#endif

    // explanation:
    // for each task's perforormance entry:
    //    run BF to get PA (consider all PA)
    ResourceOptResult res =
        EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters);
    TimerType finish_time = CurrentTimeInProfiler;
    double time_taken = GetTimeTaken(start_time, finish_time);
#if defined(RYAN_HE_CHANGE_DEBUG)
    if (GlobalVariables::debugMode & DBG_PRT_MSK_MAIN)
        std::cout << "####main: WritePriorityAssignments done. t=" << time_taken << "s" << std::endl;
#endif
    PriorityVec pa_opt = res.priority_vec;
    WritePriorityAssignments(output_file_path, dag_tasks.tasks, pa_opt,
                             time_taken);
    WriteTimeLimitToYamlOSM(
        res.id2time_limit[0]);  // only write TSP's time limit
#if defined(RYAN_HE_CHANGE_DEBUG)
    if (GlobalVariables::debugMode == 1 || GlobalVariables::debugMode & DBG_PRT_MSK_MAIN) {
#else        
    if (GlobalVariables::debugMode == 1) {
#endif
        PrintPriorityVec(dag_tasks.tasks, pa_opt);
        PrintTimer();
    }
    std::cout << "Total running time: " << time_taken << " seconds\n";
}
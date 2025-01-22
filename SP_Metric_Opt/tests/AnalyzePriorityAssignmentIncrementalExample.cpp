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
#if defined(RYAN_HE_CHANGE_DEBUG)
    GlobalVariables::debugMode = DBG_PRT_MSK_ALL; 
    GlobalVariables::debugMode &= ~DBG_PRT_MSK_RTA;
    GlobalVariables::debugMode = DBG_PRT_MSK_OptimizeSP_TL_BF | DBG_PRT_MSK_MAIN;
#endif    
    TimerType start_time = CurrentTimeInProfiler;

    argparse::ArgumentParser program("program name");
    program.add_argument("--file_path")
        .default_value(
#if defined(RYAN_HE_CHANGE)
			std::string("/mnt/f/explore/ROS2-SP-APPs/all_time_records/task_characteristics.yaml"))
#else
            std::string("/home/nvidia/workspace/sdcard/ROS2-SP-APPs/all_time_records/task_characteristics.yaml"))
#endif
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
#if defined(RYAN_HE_CHANGE)
        "/mnt/f/explore/ROS2-SP-APPs/SP_Metric_Opt/TaskData/"	
#else
        "/home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/TaskData/"
#endif
        "test_robotics_v21.yaml";
    string dag2 =
#if defined(RYAN_HE_CHANGE)
        "/mnt/f/explore/ROS2-SP-APPs/SP_Metric_Opt/TaskData/"	
#else	
        "/home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/TaskData/"
#endif
        "test_robotics_v25.yaml";
    DAG_Model dag_tasks = ReadDAG_Tasks(file_path);
    SP_Parameters sp_parameters = ReadSP_Parameters(file_path);

#if defined(RYAN_HE_CHANGE_DEBUG)
    if (GlobalVariables::debugMode & DBG_PRT_MSK_MAIN) {
        std::cout << "####main: dag_tasks in file_path:" << std::endl;
        dag_tasks.print();
        std::cout << "\n";

        std::cout << "####main: dag_tasks in test_robotics_v21:" << std::endl;
        DAG_Model dag1_tasks = ReadDAG_Tasks(dag1);
        dag1_tasks.print();
        std::cout << "\n";

        std::cout << "####main: dag_tasks in test_robotics_v25:" << std::endl;
        DAG_Model dag2_tasks = ReadDAG_Tasks(dag2);
        dag2_tasks.print();
        std::cout << "\n";

        std::cout << "####main: init OptimizePA_Incre_with_TimeLimits ..." << std::endl;
    }
#endif

    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);

#if defined(RYAN_HE_CHANGE_DEBUG)
    if (GlobalVariables::debugMode & DBG_PRT_MSK_MAIN) {
        std::cout << "####main: OptimizeFromScratch_w_TL: arg=";
        std::cout << GlobalVariables::Layer_Node_During_Incremental_Optimization<<std::endl;
    }
#endif

    // read a DAG and optimize it for the first time
    PriorityVec pa_opt = opt.OptimizeFromScratch_w_TL(
        GlobalVariables::Layer_Node_During_Incremental_Optimization);

    TimerType finish_time = CurrentTimeInProfiler;
    // PrintPriorityVec(dag_tasks.tasks, pa_opt);
    double time_taken = GetTimeTaken(start_time, finish_time);
    WritePriorityAssignments(output_file_path, dag_tasks.tasks, pa_opt,
                             time_taken);
    time_taken = GetTimeTaken(start_time, finish_time);

#if defined(RYAN_HE_CHANGE_DEBUG)
    if (GlobalVariables::debugMode & DBG_PRT_MSK_MAIN) {
        PrintPriorityVec(dag_tasks.tasks, pa_opt);

        for (int i=0;i < opt.time_limit_option_for_each_task_.size();i++) {
            std::cout<<"opt.time_limit_option_for_each_task_["<<i<<"]:";
            for (int j=0;j<opt.time_limit_option_for_each_task_[i].size();j++) {
                std::cout<<opt.time_limit_option_for_each_task_[i][j]<<" ";
            }
            std::cout<<"\n";
        }

        ResourceOptResult res = opt.CollectResults();
        std::cout << "####main: id2time_limit (task[i] select execution time id2time_limit[i]):" << std::endl;
        for (int i=0; i<res.id2time_limit.size(); i++) {
            std::cout << res.id2time_limit[i] << " ";
        }
        std::cout << std::endl;        
    }
#endif

    std::cout<<"Total time taken: "<<time_taken<<"\n";
    // to perform incremental optimization for 5 more times
    for (int i = 0; i < 10; i++) {
        // read the updated DAG
        if (i % 2 == 0) {
            dag_tasks = ReadDAG_Tasks(dag1);
#if defined(RYAN_HE_CHANGE_DEBUG)
            if (GlobalVariables::debugMode & DBG_PRT_MSK_MAIN) {
                std::cout << "\n####main: OptimizeIncre_w_TL: t="<<i<<", input=dag1" << std::endl;
            }
#endif            
        } else {
            dag_tasks = ReadDAG_Tasks(dag2);
#if defined(RYAN_HE_CHANGE_DEBUG)
            if (GlobalVariables::debugMode & DBG_PRT_MSK_MAIN) {
                std::cout << "\n####main: OptimizeIncre_w_TL: t="<<i<<", input=dag2" << std::endl;
            }
#endif             
        }
        // dag_tasks = ReadDAG_Tasks(file_path);
        pa_opt = opt.OptimizeIncre_w_TL(
            dag_tasks,
            GlobalVariables::Layer_Node_During_Incremental_Optimization);
        // time_taken = GetTimeTaken(start_time, finish_time);
        // PrintPriorityVec(dag_tasks.tasks, pa_opt);
        WritePriorityAssignments(output_file_path, dag_tasks.tasks, pa_opt, -1);

#if defined(RYAN_HE_CHANGE_DEBUG)
        if (GlobalVariables::debugMode & DBG_PRT_MSK_MAIN) {
            PrintPriorityVec(dag_tasks.tasks, pa_opt);

            ResourceOptResult res = opt.CollectResults();
            std::cout << "####main: id2time_limit (task[k] select execution time id2time_limit[k]):" << std::endl;
            for (int k=0; k<res.id2time_limit.size(); k++) {
                std::cout << res.id2time_limit[k] << " ";
            }
            std::cout << std::endl;                
        }        
#endif
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
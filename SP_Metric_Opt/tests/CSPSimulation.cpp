#include <yaml-cpp/yaml.h>

#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>

#include "sources/Optimization/OptimizeSP_BF.h"
#include "sources/Optimization/OptimizeSP_TL_BF.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/argparse.hpp"
#include "sources/Utils/profilier.h"
#include "sources/Utils/readwrite.h"

#include "sources/RTDA/ImplicitCommunication/ScheduleSimulation.h"

using namespace std;
using namespace SP_OPT_PA;

int checkMakeDir(std::string dir_path) {
    // Check if the directory exists
    if (!std::filesystem::exists(dir_path)) {
        std::cout << "Directory does not exist. Creating directory: " << dir_path << std::endl;

        // Create the directory
        if (std::filesystem::create_directory(dir_path)) {
            std::cout << "Directory created successfully!" << std::endl;
        } else {
            std::cerr << "Failed to create directory!" << std::endl;
            return 1; // Exit with an error code
        }
    } else {
        std::cout << "Directory already exists: " << dir_path << std::endl;
    }

    return 0;
}

// how to run
// specify input and output file path in command line
/*
// old interface: just input yaml file and execution time is dynamically generated
./tests/CSPSimulation --file_path TaskData/ryan_robotics_v1.yaml \
--output_file_path TaskData/sim_res_ryan_robotics_v1.txt --simt 100 --scheduler BR --verbose 1

// new interface: execution time is provided in input_folder
// implemented in CSPSimulation_2.cpp
*/
int main(int argc, char *argv[]) {
#if defined(RYAN_HE_CHANGE_DEBUG)
    GlobalVariables::debugMode = DBG_PRT_MSK_SIMULATION | DBG_PRT_MSK_RUNQUEUE | DBG_PRT_MSK_MAIN;
#endif

    argparse::ArgumentParser program("program name");
    program.add_argument("--file_path")
        .default_value(
			std::string("../all_time_records/task_characteristics.yaml"))
        .help(
            "the relative path of the yaml file that saves information about "
            "the tasks. Example: TaskData/ryan_robotics_v3.yaml. It is "
            "also okay to directly pass global path that starts with '/', such "
            "as /MyProjectDir/TaskData/ryan_robotics_v3.yaml ");
    program.add_argument("--output_file_path")
        .default_value(std::string("TaskData/sim_res_ryan_robotics_v3.txt"))
        .help(
            "the relative path of the directory that saves simulation results. "
            "Example: TaskData/sim_res_ryan_robotics_v3/. It is "
            "also okay to directly pass global path that starts with '/', such "
            "as /MyProjectDir/TaskData/sim_res_ryan_robotics_v3/ ");
    program.add_argument("--simt")
        .default_value(std::string("0"))
        .help(
            "simluation time (in ms). If not specified, the simulation time "
            "will be the hyper period of the task set");
    program.add_argument("--scheduler")
        .default_value(std::string("BR"))
        .help(
            "scheduler  which can be BR, INCR, RM_FAST, RM_SLOW, RM, CFS");            
    program.add_argument("--verbose")
        .default_value(std::string("0"))
        .help(
            "verbose bits, typical use is 2,6,14\n"
            "DBG_PRT_MSK_MAIN 2\n"
            "DBG_PRT_MSK_SIMULATION 4\n"
            "DBG_PRT_MSK_RUNQUEUE 8\n"
            "DBG_PRT_MSK_OptimizeSP_BF 16\n"
            "DBG_PRT_MSK_OptimizeSP_Base 32\n"
            "DBG_PRT_MSK_OptimizeSP_Incre 64\n"
            "DBG_PRT_MSK_RTA 128\n"
            "DBG_PRT_MSK_OptimizeSP_TL_BF 256\n"
            "DBG_PRT_MSK_SP_Metric 512\n"
            "DBG_PRT_MSK_TSK 1024\n"
            );
    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error &err) {
        std::cout << err.what() << std::endl;
        std::cout << program;
        exit(0);
    }

    string verbose_s = program.get<std::string>("--verbose");
    int verbose = atoi(verbose_s.c_str());
    GlobalVariables::debugMode = verbose;
     
    string sched_policy = program.get<std::string>("--scheduler");
    if ( sched_policy == "BR" || sched_policy == "INCR" || sched_policy == "RM_FAST" || 
         sched_policy == "RM_SLOW" || sched_policy == "CFS" ) {
        std::cout << "Scheduler type: " << sched_policy << std::endl;
    } else {
        std::cout << "Unknown scheduler type: " << sched_policy << std::endl;
        exit(0);
    }

    string simt_s = program.get<std::string>("--simt");
    int simt = atoi(simt_s.c_str());

    string file_path = program.get<std::string>("--file_path");
    file_path = RelativePathToAbsolutePath(file_path);
    //cout<<"simt = "<<simt<<endl;
    //return 0;

    string output_file_path = program.get<std::string>("--output_file_path");

    std::ofstream *output_file = nullptr;
    if (output_file_path == "") {
    } else {
        output_file_path = RelativePathToAbsolutePath(output_file_path);
        // Open the file for writing
        output_file = new std::ofstream(output_file_path);
    }

    // Check if the file was opened successfully
    if (!output_file->is_open()) {
        std::cerr << "Failed to open the file: " << output_file_path << std::endl;
        delete output_file;
        return 1; // Exit with an error code
    }

    DAG_Model dag_tasks = ReadDAG_Tasks(file_path);
    SP_Parameters sp_parameters = ReadSP_Parameters(file_path);
    const TaskSet& tasks = dag_tasks.GetTaskSet();
    TaskSetInfoDerived tasks_info(tasks);

    // Perform optimization
    // std::string sim_api = "SimulatedCSP_SingleCore";
    std::string sim_api = "SimulateCSPSched";
#if defined(RYAN_HE_CHANGE_DEBUG)
    if (GlobalVariables::debugMode & DBG_PRT_MSK_MAIN)
        std::cout << "####main: call "<< sim_api << " for " << simt << " steps" << std::endl;
#endif    
    Schedule schedule_actual;
    if (sim_api == "SimulatedCSP_SingleCore") {
           schedule_actual = SimulatedCSP_SingleCore(dag_tasks, tasks_info, sp_parameters, 0, 
                                                       simt, sched_policy, output_file); 
    } else { // if (sim_api == "SimulateCSPSched") {
        schedule_actual = SimulateCSPSched(dag_tasks, tasks_info, sp_parameters, 
                                           simt, sched_policy, output_file);
    }   
    if (output_file != nullptr) {
        output_file->close();
        delete output_file;
    }
#if defined(RYAN_HE_CHANGE_DEBUG)
    if (GlobalVariables::debugMode & DBG_PRT_MSK_MAIN) {
        std::cout << "####main: "<< sim_api << " DONE" << std::endl;        
    }
#endif

    for (auto itr = schedule_actual.begin(); itr != schedule_actual.end(); itr++) {
        JobCEC job = itr->first;
        JobStartFinish sf = itr->second;
        std::cout << "(" << job.taskId << ", " << job.jobId << ")"
#if defined(RYAN_HE_CHANGE)
                // RYAN_HE: add executionTime since it is not always the same
                << ": " << sf.start << ", " << sf.finish << ", " << sf.executionTime << "\n";
#else
                << ": " << sf.start << ", " << sf.finish << "\n";
#endif
    }

    return 0;
}
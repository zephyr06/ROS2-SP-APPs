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

// RYAN_CHANGE_20250207: THIS FILE
// MAKE SURE CHANGE tests/CMakeLists.txt to add this file

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


// new function to support multiple intervals, each with taskset_characteristics_[x].yaml
int main(int argc, char *argv[]) {
#if defined(RYAN_HE_CHANGE_DEBUG)
    GlobalVariables::debugMode = DBG_PRT_MSK_SIMULATION | DBG_PRT_MSK_RUNQUEUE | DBG_PRT_MSK_MAIN;
#endif
    //for (int i=0;i<argc;i++) {
    //    std::cout<<i<<" " << argv[i]<<std::endl;
    //}

    argparse::ArgumentParser program("program name");
    program.add_argument("--input_folder")
        .default_value(
			std::string("TaskData/taskset_cfg_1_gen_1"))
        .help(
            "the relative path of the input folder that saves information about "
            "the tasks, including taskset_param.yaml and path_Et_task_[taskid]_[pathid]_[instance].txt. "
            "Example: TaskData/taskset_cfg_1_gen_1. It is also okay to directly "
            "pass global path that starts with '/', such as "
            "/MyProjectDir/TaskData/taskset_cfg_1_gen_1 ");
    program.add_argument("--output_folder")
        .default_value(std::string(""))
        .help(
            "the relative path of the directory that saves simulation results. "
            "Example: TaskData/taskset_cfg_1_gen_1. It is also okay to directly "
            "pass global path that starts with '/', such as "
            "/MyProjectDir/TaskData/taskset_cfg_1_gen_1");
    program.add_argument("--simt")
        .default_value(std::string("0"))
        .help(
            "simluation time (in ms). If not specified, the simulation time "
            "will be the hyper period of the task set");
    program.add_argument("--calc_prio_interval_ms")
        .default_value(std::string("10000"))
        .help(
            "interval (in ms) to re-evaluate priority of a job in the schedule. "
            "If not specified, the default value is 10000 ms");
    program.add_argument("--output_job_not_executed")
        .default_value(std::string("1"))
        .help(
            "output (dump) job not executed (1: yes, 0: no); default is yes."
            "When calculating SP-metric for simulation results, the bad jobs are counted.");
    program.add_argument("--path_idx")
        .default_value(std::string("0"))
        .help(
            "which path to simulate. Et trace file name format is path_Et_task_[task]_[path]_[instance].txt");            
    program.add_argument("--inst_idx")
        .default_value(std::string("-1"))
        .help(
            "which instance to simulate. Et trace file name format is path_Et_task_[task]_[path]_[instance].txt. "
            "Just simulate one instance so that we can possibly run simulation in parallel");                  
    program.add_argument("--scheduler")
        .default_value(std::string("BR"))
        .help(
            "scheduler  which can be BR, BROPT, INCR, RM, RM_FAST, RM_SLOW, RM, CFS");            
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
        // Print out the arguments that were passed
        std::cout << "Arguments passed:" << std::endl;
        for (int i = 0; i < argc; ++i) {
            std::cout << "Arg " << i << ": " << argv[i] << std::endl;
        }
        std::cout << program;
        exit(0);
    }

    string verbose_s = program.get<std::string>("--verbose");
    int verbose = atoi(verbose_s.c_str());
    GlobalVariables::debugMode = verbose;
     
    string sched_policy = program.get<std::string>("--scheduler");
    if ( sched_policy == "BR" || sched_policy == "BROPT" || sched_policy == "INCR" || sched_policy == "INCR_SWAP" || sched_policy == "RM" || 
         sched_policy == "RM_FAST" || sched_policy == "RM_SLOW" || sched_policy == "CFS" ) {
        std::cout << "Scheduler type: " << sched_policy << std::endl;
        if (sched_policy == "INCR_SWAP") {
            GlobalVariables::use_adjacent_swap = true;
        }
    } else {
        std::cout << "Unknown scheduler type: " << sched_policy << std::endl;
        exit(0);
    }

    string simt_s = program.get<std::string>("--simt");
    int simt = atoi(simt_s.c_str());

    string path_idx_s = program.get<std::string>("--path_idx");
    int path_idx = atoi(path_idx_s.c_str());

    string inst_idx_s = program.get<std::string>("--inst_idx");
    int inp_inst_idx = atoi(inst_idx_s.c_str());

    string calc_prio_interval_ms_s = program.get<std::string>("--calc_prio_interval_ms");
    int calc_prio_interval_ms = atoi(calc_prio_interval_ms_s.c_str());

    string output_job_not_executed_s = program.get<std::string>("--output_job_not_executed");
    int output_job_not_executed = atoi(output_job_not_executed_s.c_str());

    string input_folder = program.get<std::string>("--input_folder");
    input_folder = RelativePathToAbsolutePath(input_folder);
    string  file_path;
    //cout<<"simt = "<<simt<<endl;
    //return 0;

    string output_folder = program.get<std::string>("--output_folder");
    if (output_folder == "") {
        output_folder = input_folder;   
    }
    output_folder = RelativePathToAbsolutePath(output_folder);

    // check if output_folder exists, if not, create it
    checkMakeDir(output_folder);

    // int path_idx = 0;
    int sim_inst_idx = 0;
    if (inp_inst_idx >= 0) {
        sim_inst_idx = inp_inst_idx;
    }

    // try to see how many processors are in the simulation by reading taskset_characteristics_i[x]_p[x].yaml
    int g_n_processors = 0;
    while (1) {
        std::ifstream infile(input_folder + "/taskset_characteristics_i0_p" + std::to_string(g_n_processors) + ".yaml");
        if (!infile.is_open()) {
            break;
        }
        infile.close();
        g_n_processors += 1;
    }
    if (g_n_processors == 0) {
        std::cerr << "Failed to find any processor in the simulation" << std::endl;
        return 1;
    }
    std::cout << "Number of processors: " << g_n_processors << std::endl;
    
    for (int p = 0; p < g_n_processors; p++) {
        // read all taskset_characteristics_i[x]_p[x].yaml for this processor 
        std::vector<TaskSetInfoDerived> tasks_info_vecs;
        std::vector<SP_Parameters> sp_parameters_vecs;
        std::vector<DAG_Model> dag_tasks_vecs;
        int interval_idx = 0;
        int num_tasks = 0;

        // read all /taskset_characteristics for this this processor, (for all intervals)
        while(1) {
            string file_path = input_folder + "/taskset_characteristics_i"
                +std::to_string(interval_idx)+"_p"+std::to_string(p)+".yaml";
            std::ifstream infile(file_path);
            if (!infile.is_open()) {
                if ( interval_idx == 0) { // check to make sure taskset_characteristics.yaml is available
                    std::cerr << "Failed to open the file: " << file_path << std::endl;
                    return 1;
                } else {
                    break;
                }
            } 
            infile.close();

            //std::cout<<"read taskset_characteristics "<<interval_idx<<std::endl;
            DAG_Model dag_tasks = ReadDAG_Tasks(file_path);
            SP_Parameters sp_parameters = ReadSP_Parameters(file_path);
            const TaskSet& tasks = dag_tasks.GetTaskSet();
            TaskSetInfoDerived tasks_info(tasks);
            tasks_info_vecs.push_back(tasks_info);
            sp_parameters_vecs.push_back(sp_parameters);
            dag_tasks_vecs.push_back(dag_tasks);

            if (interval_idx==0) {
                num_tasks = tasks_info.N;
            }
            interval_idx++;
        }

        // get tasks on this processor
        string file_path = input_folder + "/taskset_characteristics_i0"+"_p"+std::to_string(p)+".yaml";
        YAML::Node config = YAML::LoadFile(file_path);
        std::vector<int> task_gids_in_processor;
        int n_tasks_in_processor = 0;
        if (config["tasks"]) {
            //std::cout<<"tasks in config"<<std::endl;
            config = config["tasks"];
            //std::cout<<config<<"\n\n";
            int nn = config.size();
            for (int c1=0;c1<nn;c1++) {
                //double deadline = chain_node["deadline"].as<double>();
				
				// to be compatible with old .yaml file
                int gid = config[c1]["id"].as<int>();
                if (config[c1]["gid"]) {
                    gid = config[c1]["gid"].as<int>();
                }
                //std::cout<<"found task gid "<<gid<<std::endl;
                task_gids_in_processor.push_back(gid);
                n_tasks_in_processor += 1;
            }
            //std::cout<<"task_gids_in_processor:"<<task_gids_in_processor<<std::endl;
        }
        else {
            // should not reach here
            return 0;
        }

        // simulate for this path,instance,processor 
        while (1) {
            std::cout << "\n\n\n\n######## simulation for path=" << path_idx;
            std::cout << ", instance=" << sim_inst_idx << ", processor=" << p << " ..." << std::endl;
            std::cout << "         num tasks: " << num_tasks << std::endl;
            // get execution times for each task
            std::vector<std::vector<float>> task_Ets;
            for (int tp = 0; tp < n_tasks_in_processor; tp++) {
            // for (int i = 0; i < num_tasks; i++) { 
                int i = task_gids_in_processor[tp];
                // note that we should only read task for this processor!
                string file_path1 = input_folder + 
                    "/path_Et_task_" + std::to_string(i) + "_" + std::to_string(path_idx) + "_" 
                        + std::to_string(sim_inst_idx) + ".txt";
                std::cout<<"reading "<<file_path1<<" ..." <<std::endl;
                std::ifstream infile(file_path1);
                if (!infile.is_open()) {
                    if (sim_inst_idx==0) {
                        std::cerr << "Failed to open the file: " << file_path1 << std::endl;
                    } else {
                        std::cout<<"\n\n\n\n######## "<<(sim_inst_idx)<<" instances are simulated"<<std::endl;
                    }
                    return 1;
                }
                std::string line;
                std::vector<float> Ets;
                // Read the file line by line
                while (std::getline(infile, line)) {
                    std::stringstream ss(line);
                    int x, y;
                    float et;

                    // Split the line by commas
                    char comma; // To discard the commas
                    ss >> x >> comma >> y >> comma >> et;

                    // Store the tuple in the vector
                    Ets.push_back(et);
                }
                infile.close();
                task_Ets.push_back(Ets);
            }

            //print number of Ets for each task
            //for (int tp=0;tp<n_tasks_in_processor;tp++) {
            //    int i = task_gids_in_processor[tp];
            //    std::cout << "Task " << i << " has " << task_Ets[tp].size() << " Ets" << std::endl;
            //}

            string output_file_path = output_folder + "/sim_res_" + sched_policy + "_" 
                + std::to_string(sim_inst_idx) + "_p" + std::to_string(p) + ".txt";
            std::ofstream *output_file = nullptr;
            output_file = new std::ofstream(output_file_path);
            // Check if the file was opened successfully
            if (!output_file->is_open()) {
                std::cerr << "Failed to open the file: " << output_file_path << std::endl;
                delete output_file;
                return 1; // Exit with an error code
            }

            string log_file_path = output_folder + "/sim_log_" + sched_policy + "_" 
                + std::to_string(sim_inst_idx) + "_p" + std::to_string(p) + ".txt";
            std::ofstream *log_file = nullptr;
            log_file = new std::ofstream(log_file_path);
            // Check if the file was opened successfully
            if (!log_file->is_open()) {
                std::cerr << "Failed to open the file: " << log_file_path << std::endl;
                log_file = nullptr;
            }

            // Perform optimization
            // std::string sim_api = "SimulatedCSP_SingleCore";
            std::string sim_api = "SimulateCSPSched";
            if (GlobalVariables::debugMode & DBG_PRT_MSK_MAIN)
                std::cout << "####main: call "<< sim_api << " for " << simt << " steps" << std::endl;
            Schedule schedule_actual;
            if (sim_api == "SimulatedCSP_SingleCore") {
                schedule_actual = SimulatedCSP_SingleCore_vecs(dag_tasks_vecs, tasks_info_vecs, sp_parameters_vecs, 0, 
                                                            simt, sched_policy, output_file, &task_Ets, 
                                                            output_job_not_executed,log_file, calc_prio_interval_ms); 
            } else { // if (sim_api == "SimulateCSPSched") {
                schedule_actual = SimulateCSPSched_vecs(dag_tasks_vecs, tasks_info_vecs, sp_parameters_vecs, 
                                                simt, sched_policy, output_file, &task_Ets, 
                                                output_job_not_executed,log_file, calc_prio_interval_ms);
            }   
            if (output_file != nullptr) {
                output_file->close();
                delete output_file;
            }
            if (log_file != nullptr) {
                log_file->close();
                delete log_file;
            }

            // Write average scheduler execution time to file
            std::string time_file_path = output_folder + "/sched_exe_time_" + sched_policy + "_" + std::to_string(sim_inst_idx) + "_p" + std::to_string(p) + ".txt";
            std::ofstream time_file(time_file_path);
            if (time_file.is_open()) {
                time_file << g_last_avg_sched_time << "\n";
                time_file.close();
            }

            if (GlobalVariables::debugMode & DBG_PRT_MSK_MAIN) {
                std::cout << "######## simulation of "<< sim_api << " DONE for path="<<path_idx<<
                    ", instance=" << sim_inst_idx << ", processor=" << p << std::endl;        
            }

            if (inp_inst_idx>=0) {
                break;
            }
            sim_inst_idx++;
        }

    }

    return 0;
}

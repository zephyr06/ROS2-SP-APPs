#include <yaml-cpp/yaml.h>

#include <iostream>

#include "sources/Optimization/OptimizeSP_Incre.h"
#include "sources/Optimization/OptimizeSP_TL_Incre.h"
#include "sources/Safety_Performance_Metric/SP_Metric.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/argparse.hpp"
#include "sources/Utils/profilier.h"
#include "sources/Utils/readwrite.h"

using namespace std;
using namespace SP_OPT_PA;

// Single-config incremental optimizer driver (P0.11 evaluation).
// Mirrors AnalyzePriorityAssignment but runs the INCREMENTAL optimizer's
// from-scratch ReOptimizePeriodic once on the given --file_path, instead of BF.
// Used to compare INCR's TSP-vs-SLAM priority ordering against BF on the
// real-world config variants.
int main(int argc, char *argv[]) {
    TimerType start_time = CurrentTimeInProfiler;

    argparse::ArgumentParser program("program name");
    program.add_argument("--file_path")
        .default_value(std::string(
            "/home/nvidia/workspace/sdcard/ROS2-SP-APPs/all_time_records/"
            "task_characteristics.yaml"))
        .help("the path of the yaml file that saves task information.");
    program.add_argument("--output_file_path")
        .default_value(std::string("TaskData/pa_res_incre.yaml"))
        .help("the path of the file that saves priority assignment results.");

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

    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    PriorityVec pa_opt = opt.ReOptimizePeriodic(
        dag_tasks, GlobalVariables::Layer_Node_During_Incremental_Optimization);

    // P0.11 evaluation: snapshot the FINAL incumbent (res_opt_) — sp_opt,
    // priority_vec and id2time_limit here all describe the SAME plan (the TL
    // walk's committed result), so the SP and the gate verdict refer to one
    // plan. pa_opt (ReOptimizePeriodic's return) is printed separately to detect
    // any SP-vs-PA mismatch (the TL walk may re-search 1D PA variations).
    ResourceOptResult res = opt.CollectResults();
    std::cout << "Adopted SP: " << res.sp_opt << "\n";
    {
        std::vector<double> tl_vec(dag_tasks.tasks.size(), -1.0);
        for (size_t i = 0; i < dag_tasks.tasks.size(); i++) {
            int id = dag_tasks.tasks[i].id;
            if (res.id2time_limit.count(id))
                tl_vec[i] = res.id2time_limit.at(id);
        }
        std::cout << "Adopted TL vec:";
        for (size_t i = 0; i < dag_tasks.tasks.size(); i++)
            std::cout << " " << dag_tasks.tasks[i].name << "=" << tl_vec[i];
        std::cout << "\n";
        std::cout << "Adopted priorities (res_opt_, bigger=higher):";
        for (size_t i = 0; i < dag_tasks.tasks.size(); i++)
            std::cout << " " << dag_tasks.tasks[i].name << "="
                      << res.id2priority.at(dag_tasks.tasks[i].id);
        std::cout << "\n";
        std::cout << "Returned pa_opt (ReOptimizePeriodic):";
        for (size_t i = 0; i < pa_opt.size(); i++)
            std::cout << " " << dag_tasks.tasks[pa_opt[i]].name;
        std::cout << "\n";
        bool gate_ok = ImportantTasksMeetThresholds(
            dag_tasks, sp_parameters, res.priority_vec, tl_vec);
        ImportantTaskMissInfo wcmi = WorstCaseImportantTaskMissInfo(
            dag_tasks, sp_parameters, res.priority_vec, tl_vec);
        std::cout << "Gate (ImportantTasksMeetThresholds): "
                  << (gate_ok ? "PASS" : "FAIL")
                  << " | worst important task id=" << wcmi.task_id
                  << " miss_chance=" << wcmi.miss_chance
                  << " threshold=" << wcmi.threshold << "\n";
    }

    TimerType finish_time = CurrentTimeInProfiler;
    double time_taken = GetTimeTaken(start_time, finish_time);

    WritePriorityAssignments(output_file_path, dag_tasks.tasks, res.priority_vec,
                             time_taken);
    if (GlobalVariables::debugMode == 1) {
        PrintPriorityVec(dag_tasks.tasks, pa_opt);
        PrintTimer();
    }
    std::cout << "Total running time: " << time_taken << " seconds\n";
}

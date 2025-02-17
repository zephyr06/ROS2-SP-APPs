#include <yaml-cpp/yaml.h>

#include <cassert>
#include <iostream>

#include "sources/Optimization/OptimizeSP_BF.h"
#include "sources/Safety_Performance_Metric/SP_Metric.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/argparse.hpp"
#include "sources/Utils/profilier.h"
#include "sources/Utils/readwrite.h"
using namespace std;
using namespace SP_OPT_PA;

// test test ObtainSPFromRTAFiles2
/*
./tests/AnalyzeSP_Metric --file_path TaskData/AnalyzeSP_Metric_2/task_characteristics.yaml \
--data_dir TaskData/AnalyzeSP_Metric_2/ --dbg 1
*/
int main(int argc, char *argv[]) {
    // TODO add explanation for global path
    argparse::ArgumentParser program("program name");
    program.add_argument("--tsp_path")
        .default_value(std::string("TaskData/AnalyzeSP_Metric/tsp.txt"))
        .help(
            "the relative path of the yaml file that saves information about "
            "the tasks. Example: TaskData/AnalyzeSP_Metric/tsp.txt. It is "
            "also okay to directly pass global path that starts with '/', such "
            "as /root/usr/slam.txt ");
    program.add_argument("--rrt_path")
        .default_value(std::string("TaskData/AnalyzeSP_Metric/rrt.txt"))
        .help(
            "the relative path of the yaml file that saves information about "
            "the tasks. Example: TaskData/AnalyzeSP_Metric/rrt.txt. It is "
            "also okay to directly pass global path that starts with '/', such "
            "as /root/usr/slam.txt ");
    program.add_argument("--mpc_path")
        .default_value(std::string("TaskData/AnalyzeSP_Metric/mpc.txt"))
        .help(
            "the relative path of the yaml file that saves information about "
            "the tasks. Example: TaskData/AnalyzeSP_Metric/mpc.txt. It is "
            "also okay to directly pass global path that starts with '/', such "
            "as /root/usr/slam.txt ");
    program.add_argument("--slam_path")
        .default_value(std::string("TaskData/AnalyzeSP_Metric/slam.txt"))
        .help(
            "the relative path of the yaml file that saves information about "
            "the tasks. Example: TaskData/AnalyzeSP_Metric/slam.txt. It is "
            "also okay to directly pass global path that starts with '/', such "
            "as /root/usr/slam.txt ");

    program.add_argument("--chain0_path")
#if defined(RYAN_HE_CHANGE)
        .default_value(std::string(""))
#else
        .default_value(std::string("TaskData/AnalyzeSP_Metric/chain0.txt"))
#endif
        .help(
            "the relative path of the yaml file that saves information about "
            "the tasks. Example: TaskData/AnalyzeSP_Metric/chain0.txt. It is "
            "also okay to directly pass global path that starts with '/', such "
            "as /root/usr/slam.txt ");
    program.add_argument("--tsp_ext_path")
        .default_value(std::string("TaskData/AnalyzeSP_Metric/tsp_ext.txt"))
        .help(
            "the relative path of the yaml file that saves execution time data "
            "about "
            "the tasks. Example: TaskData/AnalyzeSP_Metric/tsp_ext.txt. It is "
            "also okay to directly pass global path that starts with '/', such "
            "as /root/usr/slam.txt ");

    program.add_argument("--file_path")
#if defined(RYAN_HE_CHANGE)
        .default_value(std::string(""))
#else    
        .default_value(std::string("TaskData/test_robotics_v19.yaml"))
#endif
        .help(
            "the relative path of the yaml file that saves information about"
            "the tasks. Example: TaskData/test_robotics_v1.yaml. It is "
            "also okay to directly pass global path that starts with '/', such "
            "as /root/usr/slam.txt ");
#if defined(RYAN_HE_CHANGE)
    // RYAN_HE: added a dbg print flag for debugging purposes
    // don't use it with python since python will read std::out to get sp value
    program.add_argument("--dbg")
        .default_value(std::string(""))
        .help(
            "set to 1 for debug print out (but not to use with python)");
    program.add_argument("--data_dir")
        .default_value(std::string(""))
        .help(
            "the relative or absolute path of the data dir");            
#endif

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error &err) {
        std::cout << err.what() << std::endl;
        std::cout << program;
        exit(0);
    }

    string slam_path = program.get<std::string>("--slam_path");

    string rrt_path = program.get<std::string>("--rrt_path");

    string mpc_path = program.get<std::string>("--mpc_path");

    string tsp_path = program.get<std::string>("--tsp_path");
    string tsp_ext_path = program.get<std::string>("--tsp_ext_path");

    string chain0_path = program.get<std::string>("--chain0_path");

    string file_path_ref = program.get<std::string>("--file_path");
#if defined(RYAN_HE_CHANGE)
    int dbg = program.get<std::string>("--dbg") == "1" ? 1 : 0;    
    string data_dir = program.get<std::string>("--data_dir");
    if (data_dir != "") {
        data_dir = RelativePathToAbsolutePath(data_dir);
        file_path_ref = RelativePathToAbsolutePath(file_path_ref);

        double sp_value_overall = ObtainSPFromRTAFiles2(file_path_ref, data_dir, dbg);
        std::cout << "SP-Metric: " << sp_value_overall << "\n";
        return 0;
    }
#endif

    slam_path = RelativePathToAbsolutePath(slam_path);
    rrt_path = RelativePathToAbsolutePath(rrt_path);
    mpc_path = RelativePathToAbsolutePath(mpc_path);
    tsp_path = RelativePathToAbsolutePath(tsp_path);
    tsp_ext_path = RelativePathToAbsolutePath(tsp_ext_path);
    if (chain0_path != "")
        chain0_path = RelativePathToAbsolutePath(chain0_path);

    file_path_ref = RelativePathToAbsolutePath(file_path_ref);

    double sp_value_overall =
        ObtainSPFromRTAFiles(slam_path, rrt_path, mpc_path, tsp_path,
                             tsp_ext_path, chain0_path, file_path_ref
#if defined(RYAN_HE_CHANGE)
                             , dbg
#endif
                             );
    std::cout << "SP-Metric: " << sp_value_overall << "\n";
}
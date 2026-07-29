// #include <gtest/gtest.h>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Optimization/OptimizeSP_BF.h"
#include "sources/Optimization/OptimizeSP_Base.h"
#include "sources/Safety_Performance_Metric/ParametersSP.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/TaskModel/RegularTasks.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/readwrite.h"
using ::testing::AtLeast;  // #1
using ::testing::Return;
using namespace std;
using namespace SP_OPT_PA;
using namespace GlobalVariables;
TEST(BasicExample, v1) {
    vector<double> data = ReadTxtFile(GlobalVariables::PROJECT_PATH +
                                      "ExperimentsData/FormatedData/test.txt");
    EXPECT_EQ(5, data.size());
    for (int i = 0; i < 5; i++) EXPECT_EQ(i + 1, data[i]);
}
TEST(read_write, task_sets) {
    std::string file_name = "test_robotics_v2";
    string path =
        GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
    TaskSet tasks = ReadTaskSet(path, 5);
    EXPECT_EQ(4, tasks.size());
    EXPECT_EQ(10, tasks[0].period);
    EXPECT_EQ(5, tasks[3].execution_time_dist.size());
    EXPECT_EQ(0.052, tasks[3].execution_time_dist[0].value);
    for (uint i = 0; i < tasks.size(); i++) {
        EXPECT_EQ(0, tasks[3].processorId);
    }
}

TEST(read, DAG) {
    std::string file_name = "test_robotics_v2";
    string path =
        GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
    DAG_Model dag_tasks = ReadDAG_Tasks(path);
    EXPECT_EQ(2, dag_tasks.chains_.size());
    EXPECT_EQ(3, dag_tasks.chains_[0].size());
    EXPECT_EQ(0, dag_tasks.chains_[0][0]);
    EXPECT_EQ(2, dag_tasks.chains_[1].size());
    EXPECT_EQ(0, dag_tasks.tasks[0].processorId);
    EXPECT_EQ(1000, dag_tasks.chains_deadlines_[0]);
}
TEST(ReadSP_Parameters, V1) {
    std::string file_name = "test_robotics_v3";
    string path =
        GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
    SP_Parameters sp_par = ReadSP_Parameters(path);

    EXPECT_EQ(1, sp_par.weights_path[0]);
    EXPECT_EQ(0.5, sp_par.thresholds_path[0]);
}
TEST(ReadSP_Parameters, V2) {
    std::string file_name = "test_robotics_v5";
    string path =
        GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
    SP_Parameters sp_par = ReadSP_Parameters(path);
    EXPECT_EQ(5, sp_par.weights_node[3]);
    EXPECT_EQ(0.1, sp_par.thresholds_node[2]);
}
TEST(write, DAG) {
    std::string file_name = "test_robotics_v2";
    string path =
        GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
    DAG_Model dag_tasks = ReadDAG_Tasks(path);
    string path_output =
        GlobalVariables::PROJECT_PATH + "TaskData/test_io_v1.yaml";
    WriteDAG_Tasks(path_output, dag_tasks);
    DAG_Model dag_tasks_read = ReadDAG_Tasks(path_output);
    EXPECT_EQ(2, dag_tasks_read.chains_.size());
    EXPECT_EQ(3, dag_tasks_read.chains_[0].size());
    EXPECT_EQ(0, dag_tasks.chains_[0][0]);

    TaskSet tasks = dag_tasks_read.tasks;
    EXPECT_EQ(4, tasks.size());
    EXPECT_EQ(10, tasks[0].period);
    EXPECT_EQ(GlobalVariables::Granularity,
              tasks[3].execution_time_dist.size());
    EXPECT_EQ(0.052, tasks[3].execution_time_dist[0].value);
}
TEST(read, processorId) {
    std::string file_name = "test_robotics_v11";
    string path =
        GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
    DAG_Model dag_tasks = ReadDAG_Tasks(path);
    EXPECT_EQ(0, dag_tasks.tasks[0].processorId);
}
TEST(read, analyze_time_perf_paris) {
    std::string performance_records_time =
        "47.2 101.2 164.3 342.5 404.8 473.8 537.1 717.8 771.8 939.5";
    std::string performance_records_perf =
        "0.5         0.78223968 0.85387062 0.94300589 0.98258149 "
        "0.9861591 0.99704945 0.99911459 0.99978866 1.";
    std::vector<TimePerfPair> res =
        AnalyzeTimePerfPair(performance_records_time, performance_records_perf);
    EXPECT_EQ(10, res.size());
    EXPECT_EQ(47.2, res[0].time_limit);
    EXPECT_EQ(0.5, res[0].performance);
    EXPECT_EQ(939.5, res[9].time_limit);
    EXPECT_EQ(1, res[9].performance);
}
TEST(read_taskset, analyze_time_perf_paris) {
    std::string file_name = "test_robotics_v17";
    string path =
        GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
    DAG_Model dag_tasks = ReadDAG_Tasks(path);
    std::vector<TimePerfPair> res = dag_tasks.tasks[0].timePerformancePairs;
    EXPECT_EQ(10, res.size());
    EXPECT_EQ(47.2, res[0].time_limit);
    EXPECT_EQ(0.5, res[0].performance);
    EXPECT_EQ(939.5, res[9].time_limit);
    EXPECT_EQ(1, res[9].performance);
}

// P0.6/P0.7/P0.8: the important-task label persists across WriteTaskSet ->
// ReadTaskSet as the per-task `important` YAML bool, read onto Task::
// is_important. Every consumer (static-solution priority-lock, online fall-back
// safety check, generation-time RTA, miss-rate analysis) reads this one field
// instead of each recomputing a top-X% cut. Default is false for tasksets
// generated before the label existed.
TEST(read_write, is_important_round_trip) {
    std::string path_out =
        GlobalVariables::PROJECT_PATH + "TaskData/test_io_important.yaml";
    TaskSet tasks = ReadTaskSet(
        GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v2.yaml", 5);
    ASSERT_EQ(4, tasks.size());
    // Mark the first two tasks important (mirrors the generator's top-50%-by-
    // sp_weight rule) and leave the rest false.
    tasks[0].is_important = true;
    tasks[1].is_important = true;
    tasks[2].is_important = false;
    tasks[3].is_important = false;
    WriteTaskSet(path_out, tasks);

    TaskSet tasks_read = ReadTaskSet(path_out, 5);
    ASSERT_EQ(4, tasks_read.size());
    EXPECT_TRUE(tasks_read[0].is_important);
    EXPECT_TRUE(tasks_read[1].is_important);
    EXPECT_FALSE(tasks_read[2].is_important);
    EXPECT_FALSE(tasks_read[3].is_important);
}

// A taskset YAML without the `important` field (every taskset generated before
// the label) must still load, with is_important defaulting to false -- the read
// path treats the field as optional, matching processorId's pattern.
TEST(read_taskset, is_important_defaults_false_when_absent) {
    // test_robotics_v2.yaml predates the label and carries no `important` key.
    std::string path =
        GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v2.yaml";
    TaskSet tasks = ReadTaskSet(path, 5);
    ASSERT_EQ(4, tasks.size());
    for (uint i = 0; i < tasks.size(); i++) {
        EXPECT_FALSE(tasks[i].is_important);
    }
}
TEST(WriteTimeLimitToYamlOSM, V1) {
    WriteTimeLimitToYamlOSM(100);
    std::string path =
        GlobalVariables::PROJECT_PATH +
        "applications/tsp_solver_osm/config/algorithm_config.yaml";
    YAML::Node config = YAML::LoadFile(path);
    EXPECT_EQ(0.1, config["general"]["max_time"].as<double>());
}
// TEST(TranslatePriorityVec, PA_VEC_v1) {
//     PriorityVec pa = {0, 1, 2, 3};
//     std::vector<int> translated_pa_val = {40, 30, 20, 10};
//     EXPECT_EQ(translated_pa_val, TranslatePriorityVec(pa));
// }
// TEST(TranslatePriorityVec, PA_VEC_v2) {
//     PriorityVec pa = {3, 2, 0, 1};
//     std::vector<int> translated_pa_val = {20, 10, 30, 40};
//     EXPECT_EQ(translated_pa_val, TranslatePriorityVec(pa));
// }
int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
// #include <gtest/gtest.h>

#include <algorithm>
#include <iomanip>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Optimization/OptimizeSP_Base.h"
#include "sources/Optimization/OptimizeSP_Incre.h"
#include "sources/Optimization/OptimizeSP_TL_BF.h"
#include "sources/Safety_Performance_Metric/SP_Metric.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/readwrite.h"

using ::testing::AtLeast;  // #1
using ::testing::Return;
using namespace std;
using namespace SP_OPT_PA;
using namespace GlobalVariables;

class TaskSetForTest_2tasks : public ::testing::Test {
   public:
    void SetUp() override {
        std::vector<Value_Proba> dist_vec1 = {
            Value_Proba(1, 0.6), Value_Proba(2, 0.3), Value_Proba(3, 0.1)};
        std::vector<Value_Proba> dist_vec2 = {Value_Proba(4, 0.7),
                                              Value_Proba(5, 0.3)};
        tasks.push_back(Task(0, dist_vec1, 5, 5, 0));
        tasks.push_back(Task(1, dist_vec2, 12, 12, 1));

        sp_parameters = SP_Parameters(tasks);
    }

    // data members
    TaskSet tasks;
    SP_Parameters sp_parameters;
};
inline double interpolate_sp_for_test(double x) {
    double minval = -1.484;
    double maxval = 0.405465;
    return (x - minval) / (maxval - minval);
}
TEST_F(TaskSetForTest_2tasks, SP_Calculation) {
    GlobalVariables::Granularity = 10;
    double sp_actual = ObtainSP_TaskSet(tasks, sp_parameters).sp_value;
    // double sp_expected = log(1 + 0.5) + log(1 + 0.5 - 0.0012);
    double sp_norm =
        1 + (log(1 + 0.5 - 0.0012) - (-0.01 * exp(10 * abs(0.5)))) /
                (log(1 + 0.5) - (-0.01 * exp(10 * abs(0.5))));
    EXPECT_NEAR(sp_norm, sp_actual, 1e-6);
}

TEST_F(TaskSetForTest_2tasks,
       ObtainSP_TaskSet_ReportsImportantTaskSchedulability) {
    GlobalVariables::Granularity = 10;
    TasksSP baseline = ObtainSP_TaskSet(tasks, sp_parameters);
    EXPECT_TRUE(baseline.important_tasks_schedulable);

    // task 1's ddl miss chance is ~0.0012 (see SP_Calculation) — below the
    // default threshold 0.5, so importance alone must not flip the flag.
    tasks[1].is_important = true;
    TasksSP schedulable = ObtainSP_TaskSet(tasks, sp_parameters);
    EXPECT_TRUE(schedulable.important_tasks_schedulable);
    EXPECT_DOUBLE_EQ(baseline.sp_value, schedulable.sp_value);

    sp_parameters.thresholds_node[1] = 0.001;
    TasksSP unschedulable = ObtainSP_TaskSet(tasks, sp_parameters);
    EXPECT_FALSE(unschedulable.important_tasks_schedulable);
}

TEST_F(TaskSetForTest_2tasks, ObtainSP_TaskSet_BudgetCancelIsUnschedulable) {
    GlobalVariables::Granularity = 10;
    // task 1 important but BELOW threshold: a completed eval reports
    // schedulable (true). A mid-eval BF budget cancel must NOT report "all
    // clear" — the per-task check is incomplete, so the safe default
    // (unschedulable) must stand.
    tasks[1].is_important = true;
    double saved_time_limit = GlobalVariables::TIME_LIMIT;
    GlobalVariables::TIME_LIMIT = 0;
    {
        BFDLSharedBudget guard(CurrentTimeInProfiler);
        TasksSP cancelled = ObtainSP_TaskSet(tasks, sp_parameters);
        EXPECT_FALSE(cancelled.important_tasks_schedulable);
    }
    GlobalVariables::TIME_LIMIT = saved_time_limit;
}

TEST_F(TaskSetForTest_2tasks, ObtainSP_TaskSet_And_TimeLimits_NoLimits) {
    GlobalVariables::Granularity = 10;
    std::vector<double> time_limits = {-1, -1};
    double sp_no_limits =
        ObtainSP_TaskSet_And_TimeLimits(tasks, sp_parameters, time_limits)
            .sp_value;
    double sp_ref = ObtainSP_TaskSet(tasks, sp_parameters).sp_value;
    EXPECT_NEAR(sp_ref, sp_no_limits, 1e-9);
}

TEST_F(TaskSetForTest_2tasks, ObtainSP_TaskSet_And_TimeLimits_WithLimit) {
    GlobalVariables::Granularity = 10;
    // Apply a time limit of 2 to task 0, which replaces its distribution
    // with a unit distribution at value 2.
    std::vector<double> time_limits = {2, -1};
    double sp_limited =
        ObtainSP_TaskSet_And_TimeLimits(tasks, sp_parameters, time_limits)
            .sp_value;

    // Manually build the expected task set with the same replacement
    TaskSet tasks_expected = tasks;
    tasks_expected[0].execution_time_dist = GetUnitExecutionTimeDist(2);
    double sp_expected = ObtainSP_TaskSet(tasks_expected, sp_parameters).sp_value;
    EXPECT_NEAR(sp_expected, sp_limited, 1e-9);
}

class TaskSetForTest_2tasks1chain : public ::testing::Test {
   public:
    void SetUp() override {
        std::vector<Value_Proba> dist_vec1 = {
            Value_Proba(1, 0.6), Value_Proba(2, 0.3), Value_Proba(3, 0.1)};
        std::vector<Value_Proba> dist_vec2 = {Value_Proba(4, 0.7),
                                              Value_Proba(5, 0.3)};
        tasks.push_back(Task(0, dist_vec1, 5, 5, 0));
        tasks.push_back(Task(1, dist_vec2, 10, 10, 1));

        dag_tasks = DAG_Model(tasks, {{0, 1}}, {10});
        sp_parameters = SP_Parameters(dag_tasks);
    }

    // data members
    TaskSet tasks;
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
};
TEST_F(TaskSetForTest_2tasks1chain, reaction_time_full_utilization) {
    dag_tasks.tasks[0].setExecutionTime(3);
    dag_tasks.tasks[1].setExecutionTime(4);
    TaskSetInfoDerived tasks_info(dag_tasks.tasks);
    Schedule schedule = SimulateFixedPrioritySched(dag_tasks, tasks_info);
    EXPECT_EQ(5, schedule[JobCEC(0, 1)].start);
    EXPECT_EQ(8, schedule[JobCEC(0, 1)].finish);
    EXPECT_EQ(3, schedule[JobCEC(1, 0)].start);
    EXPECT_EQ(10, schedule[JobCEC(1, 0)].finish);
    EXPECT_EQ(15, ObjReactionTime::Obj(dag_tasks, tasks_info, schedule,
                                       dag_tasks.chains_));
}
TEST_F(TaskSetForTest_2tasks1chain, GetRTDA_Dist_AllChains) {
    auto dists = GetRTDA_Dist_AllChains<ObjReactionTime>(dag_tasks);

    std::vector<Value_Proba> dist_vec1 = {
        Value_Proba(10, 0.42),      Value_Proba(12, 0.6 * 0.3),
        Value_Proba(13, 0.3 * 0.7), Value_Proba(14, 0.3 * 0.3),
        Value_Proba(15, 0.1 * 0.7), Value_Proba(INT32_MAX, 0.1 * 0.3)};
    FiniteDist reaction_time_dist_expected(dist_vec1);
    EXPECT_EQ(reaction_time_dist_expected, dists[0]);
}

TEST_F(TaskSetForTest_2tasks1chain, SP_Calculation_dag) {
    double sp_actual_dag = ObtainSP_DAG(dag_tasks, sp_parameters).sp_value;
    double penalty =
        0.18 + 0.21 + 0.09 + 0.07 + 0.03 - 0.5;  // for end-to-end latency
    double sp_expected_dag =
        interpolate_sp_for_test(log(1 + 0.5)) +
        interpolate_sp_for_test(log(1 + 0.5 - 0.003)) +
        interpolate_sp_for_test(-0.01 * exp(10 * abs(penalty)));
    EXPECT_NEAR(sp_expected_dag, sp_actual_dag, 1e-3);
}

TEST_F(TaskSetForTest_2tasks1chain, ObtainSP_DAG_ReportsImportantTaskSchedulability) {
    TasksSP baseline = ObtainSP_DAG(dag_tasks, sp_parameters);
    EXPECT_TRUE(baseline.important_tasks_schedulable);

    // task 1's ddl miss chance is ~0.003 (see SP_Calculation_dag) — below the
    // default threshold 0.5, so importance alone must not flip the flag.
    dag_tasks.tasks[1].is_important = true;
    TasksSP schedulable = ObtainSP_DAG(dag_tasks, sp_parameters);
    EXPECT_TRUE(schedulable.important_tasks_schedulable);
    EXPECT_DOUBLE_EQ(baseline.sp_value, schedulable.sp_value);

    sp_parameters.thresholds_node[1] = 0.001;
    TasksSP unschedulable = ObtainSP_DAG(dag_tasks, sp_parameters);
    EXPECT_FALSE(unschedulable.important_tasks_schedulable);
}

TEST_F(TaskSetForTest_2tasks1chain, ObtainSP_DAG_And_TimeLimits_NoLimits) {
    std::vector<double> time_limits = {-1, -1};
    double sp_no_limits =
        ObtainSP_DAG(dag_tasks, sp_parameters, time_limits).sp_value;
    double sp_ref = ObtainSP_DAG(dag_tasks, sp_parameters).sp_value;
    EXPECT_NEAR(sp_ref, sp_no_limits, 1e-9);
}

TEST_F(TaskSetForTest_2tasks1chain, ObtainSP_DAG_And_TimeLimits_WithLimit) {
    // Apply a time limit of 2 to task 0, replacing its distribution with a
    // unit distribution at value 2.
    std::vector<double> time_limits = {2, -1};
    double sp_limited =
        ObtainSP_DAG(dag_tasks, sp_parameters, time_limits).sp_value;

    DAG_Model dag_expected = dag_tasks;
    dag_expected.tasks[0].execution_time_dist = GetUnitExecutionTimeDist(2);
    double sp_expected = ObtainSP_DAG(dag_expected, sp_parameters).sp_value;
    EXPECT_NEAR(sp_expected, sp_limited, 1e-9);
}

class TaskSetForTest_robotics_v1 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v3";
        std::string path =
            GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
        dag_tasks = ReadDAG_Tasks(path, 5);
        sp_parameters = SP_Parameters(dag_tasks);
    }

    // data members
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
};

// TEST_F(TaskSetForTest_robotics_v1, SP_Calculation_dag) {
//     string slam_path =
//         GlobalVariables::PROJECT_PATH +
//         "TaskData/AnalyzeSP_Metric/SLAM_response_time_200_210.txt";
//     string rrt_path = GlobalVariables::PROJECT_PATH +
//                       "TaskData/AnalyzeSP_Metric/RRT_response_time_200_210.txt";
//     string mpc_path = GlobalVariables::PROJECT_PATH +
//                       "TaskData/AnalyzeSP_Metric/MPC_response_time_200_210.txt";
//     string tsp_path = GlobalVariables::PROJECT_PATH +
//                       "TaskData/AnalyzeSP_Metric/TSP_response_time_200_210.txt";
//     string chain0_path =
//         GlobalVariables::PROJECT_PATH +
//         "TaskData/AnalyzeSP_Metric/chain0.txt";

//     int granularity = 10;
//     std::vector<FiniteDist> node_rts_dists;
//     // std::string folder_path="TaskData/AnalyzeSP_Metric/";
//     node_rts_dists.push_back(FiniteDist(ReadTxtFile(tsp_path), granularity));
//     node_rts_dists.push_back(FiniteDist(ReadTxtFile(mpc_path), granularity));
//     node_rts_dists.push_back(FiniteDist(ReadTxtFile(rrt_path), granularity));
//     node_rts_dists.push_back(FiniteDist(ReadTxtFile(slam_path),
//     granularity));

//     std::vector<FiniteDist> path_latency_dists;
//     path_latency_dists.push_back(
//         FiniteDist(ReadTxtFile(chain0_path), granularity));

//     SP_Parameters sp_parameters = SP_Parameters(dag_tasks);
//     double sp_metric_val = ObtainSP_DAG_From_Dists(
//         dag_tasks, sp_parameters, node_rts_dists, path_latency_dists);
//     cout << "SP-Metric: " << sp_metric_val << "\n";
//     EXPECT_THAT(sp_metric_val, testing::Le(-4.5 + log(1.5) ));
// }

class TaskSetForTest_robotics_v8 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v8";
        std::string path =
            GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
        dag_tasks = ReadDAG_Tasks(path, 5);
        sp_parameters = ReadSP_Parameters(path);
    }

    // data members
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
    int N = dag_tasks.tasks.size();
};

class TaskSetForTest_robotics_v18 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v18";
        std::string path =
            GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
        dag_tasks = ReadDAG_Tasks(path, 5);
        sp_parameters = ReadSP_Parameters(path);
    }

    // data members
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
    int N = dag_tasks.tasks.size();
};

TEST_F(TaskSetForTest_robotics_v18, calcluate_perf_term) {
    std::vector<TimePerfPair> timePerformancePairs = {
        TimePerfPair(0, 0), TimePerfPair(1, 1), TimePerfPair(2, 2),
        TimePerfPair(3, 3), TimePerfPair(4, 4), TimePerfPair(5, 5)};
    EXPECT_EQ(0, GetPerfTerm(timePerformancePairs, -1));
    EXPECT_EQ(0, GetPerfTerm(timePerformancePairs, 0));
    EXPECT_EQ(0, GetPerfTerm(timePerformancePairs, 0.5));  // floor to 0
    EXPECT_EQ(1, GetPerfTerm(timePerformancePairs, 1));
    EXPECT_EQ(5, GetPerfTerm(timePerformancePairs, 6));
}

TEST_F(TaskSetForTest_robotics_v18, Task2priority_value) {
    auto task2priority = Task2priority_value(
        dag_tasks.tasks, {0, 1, 2, 3});  // TSP, MPC, RRT, SLAM
    EXPECT_EQ(6, task2priority["TSP"]);
    EXPECT_EQ(2, task2priority["MPC"]);
    EXPECT_EQ(1, task2priority["RRT"]);
    EXPECT_EQ(5, task2priority["SLAM"]);

    task2priority = Task2priority_value(dag_tasks.tasks,
                                        {3, 2, 1, 0});  // SLAM, RRT, MPC, TSP
    EXPECT_EQ(4, task2priority["TSP"]);
    EXPECT_EQ(2, task2priority["MPC"]);
    EXPECT_EQ(3, task2priority["RRT"]);
    EXPECT_EQ(5, task2priority["SLAM"]);
}
// TEST_F(TaskSetForTest_robotics_v8, SP_Calculation) {
//     double sp_actual = ObtainSP_TaskSet(dag_tasks.tasks, sp_parameters);
//     double sp_expected = log(1 + 0.5) + log(1 + 0.5 - 0.0012) + 1.5 * 2;
//     // EXPECT_NEAR(sp_expected, sp_actual, 1e-6);
// }
TEST_F(TaskSetForTest_robotics_v1, read_sp) {}
// TEST_F(TaskSetForTest_robotics_v1, SP_Calculation_dag_v2) {
//     string slam_path =
//         GlobalVariables::PROJECT_PATH +
//         "TaskData/AnalyzeSP_Metric/SLAM_response_time_240_250.txt";
//     string rrt_path = GlobalVariables::PROJECT_PATH +
//                       "TaskData/AnalyzeSP_Metric/RRT_response_time_240_250.txt";
//     string mpc_path = GlobalVariables::PROJECT_PATH +
//                       "TaskData/AnalyzeSP_Metric/MPC_response_time_240_250.txt";
//     string tsp_path = GlobalVariables::PROJECT_PATH +
//                       "TaskData/AnalyzeSP_Metric/TSP_response_time_240_250.txt";

//     int granularity = 10;
//     std::vector<FiniteDist> dists;
//     // std::string folder_path="TaskData/AnalyzeSP_Metric/";
//     dists.push_back(FiniteDist(ReadTxtFile(tsp_path), granularity));
//     dists.push_back(FiniteDist(ReadTxtFile(mpc_path), granularity));
//     dists.push_back(FiniteDist(ReadTxtFile(rrt_path), granularity));
//     dists.push_back(FiniteDist(ReadTxtFile(slam_path), granularity));
//     std::vector<double> deadlines =
//         GetParameter<double>(dag_tasks.GetTaskSet(), "deadline");

//     SP_Parameters sp_parameters = SP_Parameters(dag_tasks);
// double sp_metric_val =
//     ObtainSP(dists, deadlines, sp_parameters.thresholds_node,
//              sp_parameters.weights_node);
// cout << "SP-Metric: " << sp_metric_val << "\n";
// EXPECT_THAT(sp_metric_val, testing::Le(-5.9));
// }

// TEST_F(TaskSetForTest_robotics_v1, SP_Calculation_dag_v3) {
//     string slam_path =
//         GlobalVariables::PROJECT_PATH +
//         "TaskData/AnalyzeSP_Metric/SLAM_response_time_300_310.txt";
//     string rrt_path = GlobalVariables::PROJECT_PATH +
//                       "TaskData/AnalyzeSP_Metric/RRT_response_time_300_310.txt";
//     string mpc_path = GlobalVariables::PROJECT_PATH +
//                       "TaskData/AnalyzeSP_Metric/MPC_response_time_300_310.txt";
//     string tsp_path = GlobalVariables::PROJECT_PATH +
//                       "TaskData/AnalyzeSP_Metric/TSP_response_time_300_310.txt";

//     int granularity = 10;
//     std::vector<FiniteDist> dists;
//     // std::string folder_path="TaskData/AnalyzeSP_Metric/";
//     dists.push_back(FiniteDist(ReadTxtFile(tsp_path), granularity));
//     dists.push_back(FiniteDist(ReadTxtFile(mpc_path), granularity));
//     dists.push_back(FiniteDist(ReadTxtFile(rrt_path), granularity));
//     dists.push_back(FiniteDist(ReadTxtFile(slam_path), granularity));
//     std::vector<double> deadlines =
//         GetParameter<double>(dag_tasks.GetTaskSet(), "deadline");

//     SP_Parameters sp_parameters = SP_Parameters(dag_tasks);
// double sp_metric_val =
//     ObtainSP(dists, deadlines, sp_parameters.thresholds_node,
//              sp_parameters.weights_node);
// cout << "SP-Metric: " << sp_metric_val << "\n";
// EXPECT_THAT(sp_metric_val, testing::Le(-5.9));
// }

class TaskSetForTest_robotics_v19 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v19";
        std::string path =
            GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
        dag_tasks = ReadDAG_Tasks(path, 5);
        sp_parameters = ReadSP_Parameters(path);
    }

    // data members
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
    int N = dag_tasks.tasks.size();
};

TEST_F(TaskSetForTest_robotics_v19, GetTaskPerfTerm) {
    std::string ext_file_path =
        GlobalVariables::PROJECT_PATH + "TaskData/AnalyzeSP_Metric/tsp_ext.txt";
    std::vector<TimePerfPair> time_perf_pairs =
        dag_tasks.tasks[0].timePerformancePairs;

    EXPECT_EQ(0.5, GetTaskPerfTerm(0, time_perf_pairs));
    EXPECT_EQ(0.5, GetTaskPerfTerm(400, time_perf_pairs));
    EXPECT_EQ(0.5, GetTaskPerfTerm(500, time_perf_pairs));
    EXPECT_EQ(0.8, GetTaskPerfTerm(999, time_perf_pairs));
    EXPECT_EQ(1, GetTaskPerfTerm(1001, time_perf_pairs));
}
TEST_F(TaskSetForTest_robotics_v19, GetAvgTaskPerfTerm) {
    std::string ext_file_path =
        GlobalVariables::PROJECT_PATH + "TaskData/AnalyzeSP_Metric/tsp_ext.txt";
    std::vector<TimePerfPair> time_perf_pairs =
        dag_tasks.tasks[0].timePerformancePairs;

    EXPECT_EQ(1.6 / 3, GetAvgTaskPerfTerm(ext_file_path, time_perf_pairs));
}

class TaskSetForTest_robotics_v20 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v20";
        std::string path =
            GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
        dag_tasks = ReadDAG_Tasks(path, 5);
        sp_parameters = ReadSP_Parameters(path);
    }

    // data members
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
    int N = dag_tasks.tasks.size();
};

TEST_F(TaskSetForTest_robotics_v20, ObtainSPFromRTAFiles) {
    string slam_path =
        GlobalVariables::PROJECT_PATH +
        "TaskData/AnalyzeSP_Metric/SLAM_response_time_115_125.txt";
    string rrt_path = GlobalVariables::PROJECT_PATH +
                      "TaskData/AnalyzeSP_Metric/RRT_response_time_115_125.txt";
    string mpc_path = GlobalVariables::PROJECT_PATH +
                      "TaskData/AnalyzeSP_Metric/MPC_response_time_115_125.txt";
    string tsp_path = GlobalVariables::PROJECT_PATH +
                      "TaskData/AnalyzeSP_Metric/TSP_response_time_115_125.txt";
    string tsp_ext_path =
        GlobalVariables::PROJECT_PATH +
        "TaskData/AnalyzeSP_Metric/TSP_execution_time_115_125.txt";
    string chain0_path =
        GlobalVariables::PROJECT_PATH +
        "TaskData/AnalyzeSP_Metric/SLAM_response_time_115_125.txt";  // NOT USED
    string file_path_ref = GlobalVariables::PROJECT_PATH + "TaskData/" +
                           "test_robotics_v20" + ".yaml";
    double sp_value_overall =
        ObtainSPFromRTAFiles(slam_path, rrt_path, mpc_path, tsp_path,
                             tsp_ext_path, chain0_path, file_path_ref);
    EXPECT_EQ(2.0, sp_value_overall);
}
// --- Unit Tests for GetPerfCoefficient and SP with Performance Factors ---

TEST(GetPerfCoefficient, EmptyPairs_ReturnsOne) {
    std::vector<Value_Proba> dist_vec = {Value_Proba(5, 1.0)};
    Task task(0, dist_vec, 10, 10, 0);
    EXPECT_DOUBLE_EQ(1.0, task.GetPerfCoefficient());
}

TEST(GetPerfCoefficient, ExactMatch) {
    // Distribution with average 5.0
    std::vector<Value_Proba> dist_vec = {Value_Proba(5, 1.0)};
    Task task(0, dist_vec, 10, 10, 0);
    task.timePerformancePairs = {TimePerfPair(3, 0.5), TimePerfPair(5, 0.8),
                                 TimePerfPair(7, 1.0)};
    EXPECT_DOUBLE_EQ(0.8, task.GetPerfCoefficient());
}

TEST(GetPerfCoefficient, InBetween_FloorToLower) {
    // Distribution with average 4.0 -> between 3(0.5) and 5(0.8)
    // Floor behavior: return the closest lower entry (0.5)
    std::vector<Value_Proba> dist_vec = {Value_Proba(4, 1.0)};
    Task task(0, dist_vec, 10, 10, 0);
    task.timePerformancePairs = {TimePerfPair(3, 0.5), TimePerfPair(5, 0.8),
                                 TimePerfPair(7, 1.0)};
    EXPECT_DOUBLE_EQ(0.5, task.GetPerfCoefficient());
}

TEST(GetPerfCoefficient, BelowSmallest_ReturnsZero) {
    std::vector<Value_Proba> dist_vec = {Value_Proba(2, 1.0)};
    Task task(0, dist_vec, 10, 10, 0);
    task.timePerformancePairs = {TimePerfPair(3, 0.5), TimePerfPair(5, 1.0)};
    EXPECT_DOUBLE_EQ(0.0, task.GetPerfCoefficient());
}

TEST(GetPerfCoefficient, AboveLargest_ReturnsLast) {
    std::vector<Value_Proba> dist_vec = {Value_Proba(10, 1.0)};
    Task task(0, dist_vec, 10, 10, 0);
    task.timePerformancePairs = {TimePerfPair(3, 0.5), TimePerfPair(5, 1.0)};
    EXPECT_DOUBLE_EQ(1.0, task.GetPerfCoefficient());
}

class TaskSetForTest_PerfFactor : public ::testing::Test {
   public:
    void SetUp() override {
        std::vector<Value_Proba> dist_vec0 = {
            Value_Proba(1, 0.5), Value_Proba(2, 0.3), Value_Proba(3, 0.2)};
        std::vector<Value_Proba> dist_vec1 = {
            Value_Proba(3, 0.5), Value_Proba(4, 0.3), Value_Proba(5, 0.2)};
        tasks.push_back(Task(0, dist_vec0, 5, 5, 0));
        tasks.push_back(Task(1, dist_vec1, 12, 12, 1));

        // avg ET task0 = 1*0.5 + 2*0.3 + 3*0.2 = 1.7 -> floor to (1, 0.5)
        // avg ET task1 = 3*0.5 + 4*0.3 + 5*0.2 = 3.7 -> floor to (3, 1.5)
        tasks[0].timePerformancePairs = {
            TimePerfPair(1, 0.5), TimePerfPair(2, 1.0), TimePerfPair(3, 1.2)};
        tasks[1].timePerformancePairs = {
            TimePerfPair(3, 1.5), TimePerfPair(4, 2.0), TimePerfPair(5, 2.5)};

        sp_parameters = SP_Parameters(tasks);
    }

    TaskSet tasks;
    SP_Parameters sp_parameters;
};

TEST_F(TaskSetForTest_PerfFactor, SP_Calculation_WithPerfCoefficients) {
    // This test verifies that ObtainSP_TaskSet multiplies each task's weight
    // by its performance coefficient derived from timePerformancePairs.
    // Floor behavior: task0 coeff = 0.5, task1 coeff = 1.5.
    GlobalVariables::Granularity = 10;

    double sp_with_perf = ObtainSP_TaskSet(tasks, sp_parameters).sp_value;

    // Compute baseline manually by clearing perf pairs
    TaskSet tasks_no_perf = tasks;
    tasks_no_perf[0].timePerformancePairs.clear();
    tasks_no_perf[1].timePerformancePairs.clear();
    double sp_baseline = ObtainSP_TaskSet(tasks_no_perf, sp_parameters).sp_value;

    EXPECT_NE(sp_baseline, sp_with_perf)
        << "Expected SP to differ when performance coefficients are applied;"
        << " baseline=" << sp_baseline << " with_perf=" << sp_with_perf;

    EXPECT_DOUBLE_EQ(0.5, tasks[0].GetPerfCoefficient());
    EXPECT_DOUBLE_EQ(1.5, tasks[1].GetPerfCoefficient());
}

TEST_F(TaskSetForTest_PerfFactor, SP_Calculation_CorrectWeightedValue) {
    // Verify the exact SP value equals the manually-weighted sum.
    GlobalVariables::Granularity = 10;

    double sp_actual = ObtainSP_TaskSet(tasks, sp_parameters).sp_value;

    std::vector<FiniteDist> rtas = ProbabilisticRTA_TaskSet(tasks);
    double sp_expected = 0.0;
    for (size_t i = 0; i < tasks.size(); ++i) {
        int task_id = tasks[i].id;
        double ddl_miss = GetDDL_MissProbability(rtas[i], tasks[i].deadline);
        double weight = sp_parameters.weights_node.at(task_id);
        double perf_coeff = tasks[i].GetPerfCoefficient();
        sp_expected +=
            SP_Func(ddl_miss, sp_parameters.thresholds_node.at(task_id)) *
            weight * perf_coeff;
    }
    EXPECT_NEAR(sp_expected, sp_actual, 1e-9);
}

// --- Dedicated Unit Tests for GetPerfTerm ---
TEST(GetPerfTerm, EmptyPairs) {
    std::vector<TimePerfPair> pairs;
    EXPECT_DOUBLE_EQ(0.0, GetPerfTerm(pairs, 5.0));
}

TEST(GetPerfTerm, BelowSmallestTime) {
    std::vector<TimePerfPair> pairs = {TimePerfPair(2.0, 0.4),
                                       TimePerfPair(4.0, 0.8)};
    EXPECT_DOUBLE_EQ(0.0, GetPerfTerm(pairs, 1.0));
}

TEST(GetPerfTerm, ExactMatches) {
    std::vector<TimePerfPair> pairs = {
        TimePerfPair(1.0, 0.2), TimePerfPair(2.0, 0.5), TimePerfPair(3.0, 0.9)};
    EXPECT_DOUBLE_EQ(0.2, GetPerfTerm(pairs, 1.0));
    EXPECT_DOUBLE_EQ(0.5, GetPerfTerm(pairs, 2.0));
    EXPECT_DOUBLE_EQ(0.9, GetPerfTerm(pairs, 3.0));
}

TEST(GetPerfTerm, InBetween_FloorToLower) {
    std::vector<TimePerfPair> pairs = {TimePerfPair(0.0, 0.0),
                                       TimePerfPair(10.0, 1.0)};
    EXPECT_DOUBLE_EQ(0.0, GetPerfTerm(pairs, 5.0));

    std::vector<TimePerfPair> pairs2 = {TimePerfPair(2.0, 0.3),
                                        TimePerfPair(4.0, 0.7)};
    EXPECT_DOUBLE_EQ(0.3, GetPerfTerm(pairs2, 3.0));
}

TEST(GetPerfTerm, AboveLargestTime) {
    std::vector<TimePerfPair> pairs = {TimePerfPair(1.0, 0.5),
                                       TimePerfPair(3.0, 0.9)};
    EXPECT_DOUBLE_EQ(0.9, GetPerfTerm(pairs, 4.0));
}

TEST(SP_Calculation_Bug, Robotics_V19_Same_SP_For_Core_Equivalent_Priorities) {
    std::string path =
        GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v19.yaml";

    // Variant 1: Raw v19 task set under TL 400
    {
        DAG_Model dag = ReadDAG_Tasks(path, 5);
        SP_Parameters sp_params = ReadSP_Parameters(path);
        std::vector<double> time_limits = {400.0, -1.0, -1.0, -1.0};
        DAG_Model dag_cur = UpdateExtDistBasedOnTimeLimit(dag, time_limits);

        PriorityVec pa1 = {3, 1, 0, 2};
        PriorityVec pa2 = {3, 1, 2, 0};
        PriorityVec pa3 = {0, 2, 1, 3};
        PriorityVec pa4 = {2, 1, 0, 3};

        double sp1 =
            EvaluateSPWithPriorityVec(dag_cur, sp_params, pa1).sp_value;
        double sp2 =
            EvaluateSPWithPriorityVec(dag_cur, sp_params, pa2).sp_value;
        double sp3 =
            EvaluateSPWithPriorityVec(dag_cur, sp_params, pa3).sp_value;
        double sp4 =
            EvaluateSPWithPriorityVec(dag_cur, sp_params, pa4).sp_value;

        std::cout << "Variant 1 SP (Raw, TL 400): sp1 (3102) = " << std::fixed
                  << std::setprecision(17) << sp1 << std::endl;
        std::cout << "Variant 1 SP (Raw, TL 400): sp2 (3120) = " << std::fixed
                  << std::setprecision(17) << sp2 << std::endl;
        std::cout << "Variant 1 SP (Raw, TL 400): sp3 (0213) = " << std::fixed
                  << std::setprecision(17) << sp3 << std::endl;
        std::cout << "Variant 1 SP (Raw, TL 400): sp4 (2103) = " << std::fixed
                  << std::setprecision(17) << sp4 << std::endl;
        EXPECT_DOUBLE_EQ(sp1, sp2);
        EXPECT_DOUBLE_EQ(sp3, sp4);
    }

    // Variant 1.5: Raw v19 task set under TL 1000
    {
        DAG_Model dag = ReadDAG_Tasks(path, 5);
        SP_Parameters sp_params = ReadSP_Parameters(path);
        std::vector<double> time_limits = {1000.0, -1.0, -1.0, -1.0};
        DAG_Model dag_cur = UpdateExtDistBasedOnTimeLimit(dag, time_limits);

        PriorityVec pa1 = {3, 1, 0, 2};
        PriorityVec pa2 = {3, 1, 2, 0};
        PriorityVec pa3 = {0, 2, 1, 3};
        PriorityVec pa4 = {2, 1, 0, 3};

        double sp1 =
            EvaluateSPWithPriorityVec(dag_cur, sp_params, pa1).sp_value;
        double sp2 =
            EvaluateSPWithPriorityVec(dag_cur, sp_params, pa2).sp_value;
        double sp3 =
            EvaluateSPWithPriorityVec(dag_cur, sp_params, pa3).sp_value;
        double sp4 =
            EvaluateSPWithPriorityVec(dag_cur, sp_params, pa4).sp_value;

        std::cout << "Variant 1.5 SP (Raw, TL 1000): sp1 (3102) = "
                  << std::fixed << std::setprecision(17) << sp1 << std::endl;
        std::cout << "Variant 1.5 SP (Raw, TL 1000): sp2 (3120) = "
                  << std::fixed << std::setprecision(17) << sp2 << std::endl;
        std::cout << "Variant 1.5 SP (Raw, TL 1000): sp3 (0213) = "
                  << std::fixed << std::setprecision(17) << sp3 << std::endl;
        std::cout << "Variant 1.5 SP (Raw, TL 1000): sp4 (2103) = "
                  << std::fixed << std::setprecision(17) << sp4 << std::endl;

        EXPECT_DOUBLE_EQ(sp1, sp2);
        EXPECT_DOUBLE_EQ(sp3, sp4);
    }

    // Variant 2: Tightened execution time (Gaussian(700, 10))
    {
        DAG_Model dag = ReadDAG_Tasks(path, 5);
        SP_Parameters sp_params = ReadSP_Parameters(path);
        dag.tasks[0].execution_time_dist =
            FiniteDist(GaussianDist(700.0, 10.0), 5);
        std::vector<double> time_limits = {400.0, -1.0, -1.0, -1.0};
        DAG_Model dag_cur = UpdateExtDistBasedOnTimeLimit(dag, time_limits);

        PriorityVec pa1 = {3, 1, 0, 2};
        PriorityVec pa2 = {3, 1, 2, 0};

        double sp1 =
            EvaluateSPWithPriorityVec(dag_cur, sp_params, pa1).sp_value;
        double sp2 =
            EvaluateSPWithPriorityVec(dag_cur, sp_params, pa2).sp_value;

        std::cout << "Variant 2 SP (Tightened): sp1 (3102) = " << std::fixed
                  << std::setprecision(17) << sp1 << ", sp2 (3120) = " << sp2
                  << std::endl;
        EXPECT_DOUBLE_EQ(sp1, sp2);
    }
}

int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
// #include <gtest/gtest.h>

#include <algorithm>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Optimization/OptimizeSP_Base.h"
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
    double sp_actual = ObtainSP_TaskSet(tasks, sp_parameters);
    // double sp_expected = log(1 + 0.5) + log(1 + 0.5 - 0.0012);
    double sp_norm =
        1 + (log(1 + 0.5 - 0.0012) - (-0.01 * exp(10 * abs(0.5)))) /
                (log(1 + 0.5) - (-0.01 * exp(10 * abs(0.5))));
    EXPECT_NEAR(sp_norm, sp_actual, 1e-6);
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

class Test_testAnalyzeSP : public ::testing::Test {
    public:
    void SetUp() override {
        std::string file_name = "taskset_characteristics_0";
        std::string path =
            GlobalVariables::PROJECT_PATH + "TaskData/test_AnalyzeSP/" + file_name + ".yaml";
        dag_tasks = ReadDAG_Tasks(path, 5);
        sp_parameters = ReadSP_Parameters(path);
    }

    // data members
    // TaskSet tasks;
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
    double sp_actual_dag = ObtainSP_DAG(dag_tasks, sp_parameters);
    double penalty =
        0.18 + 0.21 + 0.09 + 0.07 + 0.03 - 0.5;  // for end-to-end latency
    double sp_expected_dag =
        interpolate_sp_for_test(log(1 + 0.5)) +
        interpolate_sp_for_test(log(1 + 0.5 - 0.003)) +
        interpolate_sp_for_test(-0.01 * exp(10 * abs(penalty)));
    EXPECT_NEAR(sp_expected_dag, sp_actual_dag, 1e-3);
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
    EXPECT_EQ(0.5, GetPerfTerm(timePerformancePairs, 0.5));
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

TEST_F(Test_testAnalyzeSP, ObtainSPFromRTAFiles2 ){
    string file_path_ref = GlobalVariables::PROJECT_PATH + "TaskData/test_AnalyzeSP/" + "taskset_characteristics_0.yaml";
    string data_dir = GlobalVariables::PROJECT_PATH + "TaskData/test_AnalyzeSP";
    int dbg = 1;
    double sp_value_overall = ObtainSPFromRTAFiles2(file_path_ref, data_dir, dbg);

    EXPECT_NEAR(sp_value_overall, 3.3333, 0.0001);
    std::cout << "SP-Metric: " << sp_value_overall << "\n";
}

TEST_F(Test_testAnalyzeSP, GetAvgTaskPerfTerm ){
    int granularity = GlobalVariables::Granularity;

    // task 8 and 9
    string data_dir = GlobalVariables::PROJECT_PATH + "TaskData/test_AnalyzeSP/";

    for (int i = 8; i < dag_tasks.tasks.size(); i++) {
        std::string rst_path = data_dir + dag_tasks.tasks[i].name + "_response_time.txt";
        std::vector<double> rst_data = ReadTxtFile(rst_path);
        printf("task %d, rst_data size %d\n", i, (int)rst_data.size());
        FiniteDist dist = FiniteDist(rst_data, granularity);
        EXPECT_GT(dist.distribution.size(), 0);

        printf("response_time dist ...");
        dist.print();

        if ( dag_tasks.tasks[i].timePerformancePairs.size() > 0 ) {
            std::string ext_path = data_dir + dag_tasks.tasks[i].name + "_execution_time.txt";

            std::vector<double> ext_times = ReadTxtFile(ext_path);
            int n = ext_times.size();
            if (i==8) {
                EXPECT_EQ(n, 5);
            } else {
                EXPECT_EQ(n, 3);
            }
            printf("task %d, ext_times: ", i);
            for (int k=0;k<n;k++) {
                printf("%.2f ", ext_times[k]);
            }
            printf("\n");

            double weight = GetAvgTaskPerfTerm(ext_path, dag_tasks.tasks[i].timePerformancePairs);
            sp_parameters.update_node_weight(i, weight);
            std::cout << "ext_path = " << ext_path << ", weight = " << weight << std::endl;
            printf("timePerformancePairs: (time_limit, performance) ...\n");
            for (int k=0;k<dag_tasks.tasks[i].timePerformancePairs.size();k++) {
                printf("(%.2f,%.1f) ", dag_tasks.tasks[i].timePerformancePairs[k].time_limit,
                    dag_tasks.tasks[i].timePerformancePairs[k].performance );
            }

            if (i==8) {
                // 2 really small and 3 max exe
                EXPECT_NEAR(weight, (0.1*2+1.0*3)/5, 0.0001);
            } else {
                // 2 really small and 1 max exe
                EXPECT_NEAR(weight, (0.1*2+1.0*1)/3, 0.0001);
            }
            printf("\n\n");
        } else {
            std::cout << "ERROR! task_id="<<i<<", no timePerformancePairs" << std::endl;
        }
    }

    std::vector<FiniteDist> node_rts_dists;
    for (int i = 0; i < dag_tasks.tasks.size(); i++) {
        std::string rst_path = data_dir + dag_tasks.tasks[i].name + "_response_time.txt";
        std::vector<double> rst_data = ReadTxtFile(rst_path);
        printf("task %d, rst_data size %d\n", i, (int)rst_data.size());
        FiniteDist dist = FiniteDist(rst_data, granularity);
        node_rts_dists.push_back(dist);

        if ( dag_tasks.tasks[i].timePerformancePairs.size() > 0 ) {
            std::string ext_path = data_dir + dag_tasks.tasks[i].name + "_execution_time.txt";
            double weight = GetAvgTaskPerfTerm(ext_path, dag_tasks.tasks[i].timePerformancePairs);
            sp_parameters.update_node_weight(i, weight);
        }        
    }

    double sp_overall = 0;
    for (uint i = 0; i < dag_tasks.tasks.size(); i++) {
        int task_id = dag_tasks.tasks[i].id;
        double weight = sp_parameters.weights_node.at(task_id);
        double threshold = sp_parameters.thresholds_node.at(task_id);

        printf("%d, task%d: deadline=%.0f, weight=%f, threshold=%f\n",
            i,task_id,dag_tasks.tasks[i].deadline, weight, threshold);
        printf("response_time dist ...");
        node_rts_dists[i].print();

        double ddl_miss_chance = GetDDL_MissProbability(node_rts_dists[i],dag_tasks.tasks[i].deadline);
        double sp_val = SP_Func(ddl_miss_chance, threshold) * weight;
        printf("ddl_miss_chance = %f, sp_val=%f\n\n", ddl_miss_chance,sp_val);
        sp_overall += sp_val;
    }
    printf("sp_overall = %f\n", sp_overall);  
    EXPECT_NEAR(sp_overall, 3.3333, 0.0001);
}

int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
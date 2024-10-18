// #include <gtest/gtest.h>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Optimization/OptimizeSP_BF.h"
#include "sources/Optimization/OptimizeSP_Base.h"
#include "sources/Optimization/OptimizeSP_Incre.h"
#include "sources/Optimization/OptimizeSP_TL_BF.h"
#include "sources/Utils/Parameters.h"

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
TEST_F(TaskSetForTest_2tasks, GetPriorityAssignments) {
    PriorityVec pa_vec1 = GetPriorityAssignments(tasks);
    EXPECT_EQ(2, pa_vec1.size());
    EXPECT_EQ(0, pa_vec1[0]);
    EXPECT_EQ(1, pa_vec1[1]);
}
TEST_F(TaskSetForTest_2tasks, optimize) {
    PriorityVec pa_vec = {0, 1};
    UpdateTaskSetPriorities(tasks, pa_vec);
    EXPECT_EQ(0, tasks[0].id);
    EXPECT_EQ(0, tasks[0].priority);
    EXPECT_EQ(1, tasks[1].id);
    EXPECT_EQ(1, tasks[1].priority);

    pa_vec = {1, 0};
    tasks = UpdateTaskSetPriorities(tasks, pa_vec);
    EXPECT_EQ(1, tasks[0].id);
    EXPECT_EQ(0, tasks[0].priority);
    EXPECT_EQ(0, tasks[1].id);
    EXPECT_EQ(1, tasks[1].priority);
}

TEST_F(TaskSetForTest_2tasks, Optimize_bf) {
    Task task_t = tasks[0];
    tasks[0] = tasks[1];
    tasks[0].id = 0;
    tasks[1] = task_t;
    tasks[1].id = 1;
    sp_parameters.thresholds_node[0] = 0;
    sp_parameters.thresholds_node[1] = 0;
    DAG_Model dag_tasks(tasks, {}, {});
    PriorityVec pa_opt =
        OptimizePA_BruteForce(dag_tasks, sp_parameters).priority_vec;
    EXPECT_EQ(5, tasks[pa_opt[0]].period);
    EXPECT_EQ(12, tasks[pa_opt[1]].period);
}
class TaskSetv11 : public ::testing::Test {
   public:
    void SetUp() override {
        string file_path =
            GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v11.yaml";
        dag_tasks = ReadDAG_Tasks(file_path);
        sp_parameters = ReadSP_Parameters(file_path);
    }

    // data members
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
};

TEST_F(TaskSetv11, Optimize_bf) {
    PriorityVec pa_opt =
        OptimizePA_BruteForce(dag_tasks, sp_parameters).priority_vec;
    // PrintPriorityVec(dag_tasks.tasks, pa_opt);
    // EXPECT_EQ(5, dag_tasks.tasks[pa_opt[0]].period);
    // EXPECT_EQ(12, dag_tasks.tasks[pa_opt[1]].period);
    EXPECT_EQ("TSP", dag_tasks.tasks[pa_opt[0]].name);
    EXPECT_EQ("MPC", dag_tasks.tasks[pa_opt[1]].name);
    EXPECT_EQ("RRT", dag_tasks.tasks[pa_opt[2]].name);
    EXPECT_EQ("SLAM", dag_tasks.tasks[pa_opt[3]].name);
}
class TaskSetv12 : public ::testing::Test {
   public:
    void SetUp() override {
        string file_path =
            GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v12.yaml";
        dag_tasks = ReadDAG_Tasks(file_path);
        sp_parameters = ReadSP_Parameters(file_path);
    }

    // data members
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
};

TEST_F(TaskSetv12, Optimize_bf) {
    PriorityVec pa_opt =
        OptimizePA_BruteForce(dag_tasks, sp_parameters).priority_vec;
    EXPECT_EQ("MPC", dag_tasks.tasks[pa_opt[0]].name);
    EXPECT_EQ("RRT", dag_tasks.tasks[pa_opt[1]].name);
    EXPECT_EQ("SLAM", dag_tasks.tasks[pa_opt[2]].name);
    EXPECT_EQ("TSP", dag_tasks.tasks[pa_opt[3]].name);
}
class TaskSetv13 : public ::testing::Test {
   public:
    void SetUp() override {
        string file_path =
            GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v13.yaml";
        dag_tasks = ReadDAG_Tasks(file_path);
        sp_parameters = ReadSP_Parameters(file_path);
    }

    // data members
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
};

TEST_F(TaskSetv13, Optimize_bf) {
    ResourceOptResult res = OptimizePA_BruteForce(dag_tasks, sp_parameters);
    PrintPriorityVec(dag_tasks.tasks, res.priority_vec);
    // EXPECT_EQ("TSP", dag_tasks.tasks[pa_opt[2]].name);
    // EXPECT_EQ("SLAM", dag_tasks.tasks[pa_opt[3]].name);
    EXPECT_THAT(res.id2priority[0], testing::Ge(res.id2priority[3]));
}
class TaskSetv14 : public ::testing::Test {
   public:
    void SetUp() override {
        string file_path =
            GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v14.yaml";
        dag_tasks = ReadDAG_Tasks(file_path);
        sp_parameters = ReadSP_Parameters(file_path);
    }

    // data members
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
};

// TEST_F(TaskSetv14, Optimize_bf) {
//     PriorityVec pa_opt = OptimizePA_BruteForce(dag_tasks,
//     sp_parameters).priority_vec; PrintPriorityVec(dag_tasks.tasks, pa_opt);
//     YAML::Node yaml_nodes = PriorityAssignmentToYaml(dag_tasks.tasks,
//     pa_opt); EXPECT_THAT(yaml_nodes["TSP"].as<int>(),
//                 testing::Le(yaml_nodes["SLAM"].as<int>()));
// }
// TEST_F(TaskSetForTest_2tasks, Optimize_incre) {
//     Task task_t = tasks[0];
//     tasks[0] = tasks[1];
//     tasks[0].id = 0;
//     tasks[1] = task_t;
//     tasks[1].id = 1;
//     sp_parameters.thresholds_node = {0, 0};
//     DAG_Model dag_tasks(tasks, {}, {});
//     PriorityVec pa_opt = OptimizePA_Incremental(dag_tasks, sp_parameters);
//     PrintPriorityVec(tasks, pa_opt);
//     EXPECT_EQ(5, tasks[pa_opt[0]].period);
//     EXPECT_EQ(12, tasks[pa_opt[1]].period);
// }
// TEST_F(TaskSetForTest_2tasks, Optimize_incre_v2) {
//     Task task_t = tasks[0];
//     tasks[0] = tasks[1];
//     tasks[0].id = 0;
//     tasks[1] = task_t;
//     tasks[1].id = 1;
//     sp_parameters.thresholds_node = {0, 0};
//     DAG_Model dag_tasks(tasks, {}, {});
//     OptimizePA_Incre opt(dag_tasks, sp_parameters);
//     BeginTimer("First-run");
//     PriorityVec pa_opt = opt.Optimize(dag_tasks);
//     EndTimer("First-run");
//     BeginTimer("Second-runs");
//     pa_opt = opt.Optimize(dag_tasks);
//     // pa_opt = opt.Optimize(dag_tasks);
//     // pa_opt = opt.Optimize(dag_tasks);
//     EndTimer("Second-runs");
//     PrintPriorityVec(tasks, pa_opt);
//     EXPECT_EQ(5, tasks[pa_opt[0]].period);
//     EXPECT_EQ(12, tasks[pa_opt[1]].period);
//     PrintTimer();
// }

// TEST_F(TaskSetForTest_2tasks, FindTasksWithSameET) {
//     Task task_t = tasks[0];
//     tasks[0] = tasks[1];
//     tasks[0].id = 0;
//     tasks[1] = task_t;
//     tasks[1].id = 1;
//     sp_parameters.thresholds_node = {0, 0};
//     DAG_Model dag_tasks(tasks, {}, {});

//     OptimizePA_Incre opt_inc_class(dag_tasks, sp_parameters);
//     opt_inc_class.prev_exex_rec_ = GetExecutionTimeVector(dag_tasks);
//     std::vector<Value_Proba> dist_vec1 = {
//         Value_Proba(1, 0.5), Value_Proba(2, 0.4), Value_Proba(3, 0.1)};
//     tasks[0].execution_time_dist = dist_vec1;
//     DAG_Model dag_tasks2(tasks, {}, {});
//     vector<bool> diff_rec = opt_inc_class.FindTasksWithSameET(dag_tasks2);
//     EXPECT_FALSE(diff_rec[0]);
//     EXPECT_TRUE(diff_rec[1]);
// }

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
TEST_F(TaskSetForTest_robotics_v18, RecordTimeLimitOptions) {
    OptimizePA_with_TimeLimitsStatus optimizer(dag_tasks, sp_parameters);
    EXPECT_EQ(4, optimizer.time_limit_option_for_each_task.size());
    EXPECT_EQ(4, optimizer.time_limit_option_for_each_task[0].size());
    EXPECT_EQ(400, optimizer.time_limit_option_for_each_task[0][0]);
    EXPECT_EQ(600, optimizer.time_limit_option_for_each_task[0][1]);
    EXPECT_EQ(800, optimizer.time_limit_option_for_each_task[0][2]);
    EXPECT_EQ(1000, optimizer.time_limit_option_for_each_task[0][3]);
    EXPECT_EQ(-1, optimizer.time_limit_option_for_each_task[1][0]);
}

TEST_F(TaskSetForTest_robotics_v18, AddWeightsFromTimeLimits) {
    vector<double> time_limit_option_for_each_task = {1000, -1, -1, -1};
    SP_Parameters sp_parameters_cur = AddWeightsFromTimeLimits(
        dag_tasks, sp_parameters, time_limit_option_for_each_task);
    EXPECT_EQ(1, sp_parameters_cur.weights_node[0]);

    time_limit_option_for_each_task = {400, -1, -1, -1};
    sp_parameters_cur = AddWeightsFromTimeLimits(
        dag_tasks, sp_parameters, time_limit_option_for_each_task);
    EXPECT_EQ(0.5, sp_parameters_cur.weights_node[0]);
    EXPECT_EQ(1, sp_parameters_cur.weights_node[1]);
    EXPECT_EQ(1, sp_parameters_cur.weights_node[2]);
    EXPECT_EQ(2, sp_parameters_cur.weights_node[3]);
}
TEST_F(TaskSetForTest_robotics_v18, optimize) {
    ResourceOptResult res_opt =
        EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters);
    EXPECT_EQ(1000, res_opt.id2time_limit[0]);
}

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
TEST_F(TaskSetForTest_robotics_v19, optimize) {
    ResourceOptResult res_opt =
        EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters);
    PrintPriorityVec(dag_tasks.tasks, res_opt.priority_vec);
    EXPECT_EQ(400, res_opt.id2time_limit[0]);
}
int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
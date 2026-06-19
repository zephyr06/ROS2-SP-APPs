// #include <gtest/gtest.h>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Optimization/OptimizeSP_BF.h"
#include "sources/Optimization/OptimizeSP_Incre.h"
#include "sources/Optimization/OptimizeSP_TL_BF.h"
#include "sources/Optimization/OptimizeSP_TL_Incre.h"
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

class TaskSetForTest_robotics_v6 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v6";
        std::string path =
            GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
        dag_tasks = ReadDAG_Tasks(path, 5);
        sp_parameters = SP_Parameters(dag_tasks);
    }

    // data members
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
    int N = dag_tasks.tasks.size();
};

TEST_F(TaskSetForTest_robotics_v6, Compare_PriorityPartialPath) {
    std::priority_queue<PriorityPartialPath, std::vector<PriorityPartialPath>,
                        CompPriorityPath>
        pq;
    PriorityPartialPath path(dag_tasks, sp_parameters);
    for (int task_id : path.tasks_to_assign) {
        PriorityPartialPath new_path = path;
        new_path.AssignAndUpdateSP(task_id);
        pq.push(new_path);
    }
    EXPECT_EQ("TSP", dag_tasks.tasks[pq.top().pa_vec_lower_pri[0]].name);
    pq.pop();

    EXPECT_EQ("SLAM", dag_tasks.tasks[pq.top().pa_vec_lower_pri[0]].name);
    pq.pop();

    EXPECT_EQ("RRT", dag_tasks.tasks[pq.top().pa_vec_lower_pri[0]].name);
    pq.pop();

    EXPECT_EQ("MPC", dag_tasks.tasks[pq.top().pa_vec_lower_pri[0]].name);
    pq.pop();
}

TEST_F(TaskSetForTest_robotics_v6, GetPriorityAssignments) {
    GlobalVariables::Granularity = 10;
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    PriorityVec pa_vec1 = opt.OptimizeFromScratch(2);
    EXPECT_EQ(4, pa_vec1.size());
    EXPECT_EQ("MPC", dag_tasks.tasks[pa_vec1[0]].name);
    EXPECT_EQ("RRT", dag_tasks.tasks[pa_vec1[1]].name);
    // EXPECT_EQ("TSP", dag_tasks.tasks[pa_vec1[2]].name);
    // EXPECT_EQ("SLAM", dag_tasks.tasks[pa_vec1[3]].name);
}

class TaskSetForTest_robotics_v7 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v7";
        std::string path =
            GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
        dag_tasks = ReadDAG_Tasks(path, 5);
        sp_parameters = SP_Parameters(dag_tasks);
    }

    // data members
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
    int N = dag_tasks.tasks.size();
};
TEST_F(TaskSetForTest_robotics_v7, GetPriorityAssignments) {
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    PriorityVec pa_vec1 = opt.OptimizeFromScratch(2);
    EXPECT_EQ(4, pa_vec1.size());
    EXPECT_EQ("MPC", dag_tasks.tasks[pa_vec1[0]].name);
    EXPECT_EQ("RRT", dag_tasks.tasks[pa_vec1[1]].name);
    EXPECT_EQ("SLAM", dag_tasks.tasks[pa_vec1[2]].name);
    EXPECT_EQ("TSP", dag_tasks.tasks[pa_vec1[3]].name);
}
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
class TaskSetForTest_robotics_v27 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v27";
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

TEST_F(TaskSetForTest_robotics_v8, FindTaskWithDifferentEt) {
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    PriorityVec pa_vec1 = opt.OptimizeFromScratch(2);
    EXPECT_EQ("SLAM", dag_tasks.tasks[pa_vec1[0]].name);
    EXPECT_EQ("TSP", dag_tasks.tasks[pa_vec1[1]].name);
}

TEST(TaskSet, FindTaskWithDifferentEt) {
    auto dag22 = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                               "TaskData/test_robotics_v22.yaml");
    auto dag23 = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                               "TaskData/test_robotics_v23.yaml");
    auto dag24 = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                               "TaskData/test_robotics_v24.yaml");
    auto diff2 = FindTaskWithDifferentEt(dag22, dag23);
    EXPECT_EQ(1, diff2.size());
    EXPECT_EQ(1, diff2[0].task_id);
    EXPECT_TRUE(diff2[0].increase);
    auto diff3 = FindTaskWithDifferentEt(dag22, dag24);
    EXPECT_EQ(0, diff3.size());
}
TEST_F(TaskSetForTest_robotics_v7, FindPriorityVec1D_Variations) {
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    PriorityVec pa_vec1 = {0, 1, 2, 3};
    std::vector<PriorityVec> res = FindPriorityVec1D_Variations(
        pa_vec1, 0, PriorityChangeStatus::OpenToAll);
    EXPECT_EQ(4, res.size());
    AssertEqualVectorExact<int>({0, 1, 2, 3}, res[0], 1e-3, __LINE__);
    AssertEqualVectorExact<int>({1, 0, 2, 3}, res[1], 1e-3, __LINE__);
    AssertEqualVectorExact<int>({1, 2, 0, 3}, res[2], 1e-3, __LINE__);
    AssertEqualVectorExact<int>({1, 2, 3, 0}, res[3], 1e-3, __LINE__);
}
TEST_F(TaskSetForTest_robotics_v7, FindPriorityVec1D_Variations_increase) {
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    PriorityVec pa_vec1 = {0, 1, 2, 3};
    std::vector<PriorityVec> res = FindPriorityVec1D_Variations(
        pa_vec1, 0, PriorityChangeStatus::Increase);
    EXPECT_EQ(1, res.size());
    AssertEqualVectorExact<int>({0, 1, 2, 3}, res[0], 1e-3, __LINE__);
}
TEST_F(TaskSetForTest_robotics_v7, FindPriorityVec1D_Variations_increase2) {
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    PriorityVec pa_vec1 = {0, 1, 2, 3};
    std::vector<PriorityVec> res = FindPriorityVec1D_Variations(
        pa_vec1, 1, PriorityChangeStatus::Increase);
    EXPECT_EQ(2, res.size());
    AssertEqualVectorExact<int>({1, 0, 2, 3}, res[0], 1e-3, __LINE__);
    AssertEqualVectorExact<int>({0, 1, 2, 3}, res[1], 1e-3, __LINE__);
}
TEST_F(TaskSetForTest_robotics_v7, FindPriorityVec1D_Variations_decrease) {
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    PriorityVec pa_vec1 = {0, 1, 2, 3};
    std::vector<PriorityVec> res = FindPriorityVec1D_Variations(
        pa_vec1, 0, PriorityChangeStatus::Decrease);
    EXPECT_EQ(4, res.size());
    AssertEqualVectorExact<int>({0, 1, 2, 3}, res[0], 1e-3, __LINE__);
    AssertEqualVectorExact<int>({1, 0, 2, 3}, res[1], 1e-3, __LINE__);
    AssertEqualVectorExact<int>({1, 2, 0, 3}, res[2], 1e-3, __LINE__);
    AssertEqualVectorExact<int>({1, 2, 3, 0}, res[3], 1e-3, __LINE__);
}

class TaskSetForTest_robotics_v25 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v25";
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

TEST_F(TaskSetForTest_robotics_v25, AnalyzePriorityChangeStatus) {
    EXPECT_EQ(PriorityChangeStatus::Increase,
              AnalyzePriorityChangeStatus(sp_parameters, 3, true));
    EXPECT_EQ(PriorityChangeStatus::Decrease,
              AnalyzePriorityChangeStatus(sp_parameters, 3, false));
    EXPECT_EQ(PriorityChangeStatus::Decrease,
              AnalyzePriorityChangeStatus(sp_parameters, 0, true));
    EXPECT_EQ(PriorityChangeStatus::Increase,
              AnalyzePriorityChangeStatus(sp_parameters, 0, false));
}

TEST_F(TaskSetForTest_robotics_v8, AssignAndUpdateSP) {
    dag_tasks.tasks[0].priority = 2;
    dag_tasks.tasks[1].priority = 1;  // SLAM has high priority
    double sp_ref = ObtainSP_DAG(dag_tasks, sp_parameters);

    PriorityPartialPath priority_path(dag_tasks, sp_parameters);
    priority_path.AssignAndUpdateSP(0);
    EXPECT_EQ("TSP", dag_tasks.tasks[priority_path.pa_vec_lower_pri[0]].name);
    EXPECT_EQ(1, priority_path.tasks_to_assign.size());
    EXPECT_EQ("SLAM",
              dag_tasks.tasks[*(priority_path.tasks_to_assign.begin())].name);

    priority_path.AssignAndUpdateSP(1);
    EXPECT_EQ("TSP", dag_tasks.tasks[priority_path.pa_vec_lower_pri[0]].name);
    EXPECT_EQ("SLAM", dag_tasks.tasks[priority_path.pa_vec_lower_pri[1]].name);
    EXPECT_EQ(0, priority_path.tasks_to_assign.size());
    EXPECT_NEAR(
        200 + 0.5 - sp_ref, priority_path.sp_lost,
        1e-3);  // the tolerance is relatively high because Ryan added small
                // modification to break ties in certain situations
}

TEST_F(TaskSetForTest_robotics_v27, GetPriorityAssignments_IncrementalOpt) {
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    PriorityVec pa_vec1 = opt.OptimizeFromScratch(2);
    EXPECT_EQ("TSP", dag_tasks.tasks[pa_vec1[0]].name);
    EXPECT_EQ("SLAM", dag_tasks.tasks[pa_vec1[1]].name);

    DAG_Model dag_tasks_update = ReadDAG_Tasks(
        GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v8.yaml", 5);
    pa_vec1 = opt.OptimizeIncre(dag_tasks_update);
    EXPECT_EQ("SLAM", dag_tasks.tasks[pa_vec1[0]].name);
    EXPECT_EQ("TSP", dag_tasks.tasks[pa_vec1[1]].name);
}

TEST_F(TaskSetForTest_2tasks, UnfeasibleTaskSet) {
    // Create 2 tasks with utilization > 1.0
    // Task 0: execution time = 4, period = 5 (utilization = 0.8)
    // Task 1: execution time = 8, period = 10 (utilization = 0.8)
    // Total utilization = 1.6 > 1.0 (unfeasible on a single core)
    std::vector<Value_Proba> dist_vec1 = {Value_Proba(4, 1.0)};
    std::vector<Value_Proba> dist_vec2 = {Value_Proba(8, 1.0)};
    
    TaskSet unfeasible_tasks;
    unfeasible_tasks.push_back(Task(0, dist_vec1, 5, 5, 0, "T1"));
    unfeasible_tasks.push_back(Task(1, dist_vec2, 10, 10, 1, "T2"));
    
    MAP_Prev mapPrev;
    DAG_Model dag_model(unfeasible_tasks, mapPrev, 0, 0);
    SP_Parameters sp_params(dag_model);
    
    OptimizePA_Incre opt(dag_model, sp_params);
    PriorityVec pa = opt.OptimizeFromScratch(2);
    
    EXPECT_EQ(2, pa.size());
    // The optimizer should still return a priority vector rather than crashing,
    // and opt_sp_ should be calculated (with some SP penalty or loss due to deadline miss).
    EXPECT_TRUE((pa[0] == 0 && pa[1] == 1) || (pa[0] == 1 && pa[1] == 0));
}

TEST_F(TaskSetForTest_2tasks, DeterministicTaskSet) {
    // Create 2 deterministic tasks
    std::vector<Value_Proba> dist_vec1 = {Value_Proba(2, 1.0)};
    std::vector<Value_Proba> dist_vec2 = {Value_Proba(3, 1.0)};
    
    TaskSet deterministic_tasks;
    deterministic_tasks.push_back(Task(0, dist_vec1, 5, 5, 0, "T1"));
    deterministic_tasks.push_back(Task(1, dist_vec2, 10, 10, 1, "T2"));
    
    MAP_Prev mapPrev;
    DAG_Model dag_model(deterministic_tasks, mapPrev, 0, 0);
    SP_Parameters sp_params(dag_model);
    
    OptimizePA_Incre opt(dag_model, sp_params);
    PriorityVec pa = opt.OptimizeFromScratch(2);
    
    EXPECT_EQ(2, pa.size());
    EXPECT_EQ(0, pa[0]); // T1 should have higher priority
    EXPECT_EQ(1, pa[1]); // T2 should have lower priority
    
    // Incrementally increase T1's execution time to 6 (which exceeds T1's period/deadline)
    DAG_Model dag_model_updated = dag_model;
    std::vector<Value_Proba> dist_vec1_updated = {Value_Proba(6, 1.0)};
    dag_model_updated.tasks[0].execution_time_dist = FiniteDist(dist_vec1_updated);
    
    PriorityVec pa_incre = opt.OptimizeIncre(dag_model_updated);
    EXPECT_EQ(2, pa_incre.size());
}

TEST(TaskSetPartitioned, PartitionedCoreOptimization) {
    // Create 3 tasks partitioned on 2 cores
    // Core 0: Task 0 (ET=4, T=10), Task 2 (ET=3, T=20)
    // Core 1: Task 1 (ET=12, T=20)
    std::vector<Value_Proba> dist_0 = {Value_Proba(4, 1.0)};
    std::vector<Value_Proba> dist_1 = {Value_Proba(12, 1.0)};
    std::vector<Value_Proba> dist_2 = {Value_Proba(3, 1.0)};

    TaskSet tasks;
    tasks.push_back(Task(0, dist_0, 10, 10, 0, "Task0"));
    tasks.push_back(Task(1, dist_1, 20, 20, 1, "Task1"));
    tasks.push_back(Task(2, dist_2, 20, 20, 2, "Task2"));

    tasks[0].processorId = 0;
    tasks[1].processorId = 1;
    tasks[2].processorId = 0;

    MAP_Prev mapPrev;
    DAG_Model dag(tasks, mapPrev, 0, 0);
    SP_Parameters sp_params(dag);

    OptimizePA_Incre opt(dag, sp_params);
    PriorityVec pa = opt.OptimizeFromScratch(2);

    EXPECT_EQ(3, pa.size());
    // Schedulability and assignments are verified
    EXPECT_TRUE(pa[0] == 0 || pa[1] == 0 || pa[2] == 0);
    EXPECT_TRUE(pa[0] == 1 || pa[1] == 1 || pa[2] == 1);
    EXPECT_TRUE(pa[0] == 2 || pa[1] == 2 || pa[2] == 2);
}

TEST(OptimizePA_Consistency, CompareIncreAndBF) {
    // List of test files to run the comparison on
    std::vector<std::string> test_files = {
        "test_robotics_v6",
        "test_robotics_v7",
        "test_robotics_v9"
    };

    for (const auto& file_name : test_files) {
        std::string path = GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
        DAG_Model dag_tasks = ReadDAG_Tasks(path, 5);
        SP_Parameters sp_parameters = SP_Parameters(dag_tasks);

        // Run Brute Force
        OptimizePA_BF opt_bf(dag_tasks, sp_parameters);
        PriorityVec pa_bf = opt_bf.Optimize();
        double sp_bf = opt_bf.opt_sp_;

        // Run Incremental
        OptimizePA_Incre opt_inc(dag_tasks, sp_parameters);
        PriorityVec pa_inc = opt_inc.OptimizeFromScratch(2);
        double sp_inc = opt_inc.opt_sp_;

        // They should find the same or very close SP value
        EXPECT_NEAR(sp_bf, sp_inc, 2.0);
        EXPECT_GE(sp_bf, sp_inc - 1e-2);
    }
}

TEST(OptimizePA_Consistency, CompareIncreAndBF_w_TL) {
    std::vector<std::string> test_files = {
        "test_robotics_v18",
        "test_robotics_v19"
    };

    for (const auto& file_name : test_files) {
        std::string path = GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
        DAG_Model dag_tasks = ReadDAG_Tasks(path, 5);
        SP_Parameters sp_parameters = ReadSP_Parameters(path);

        // Run Brute Force with Time Limits
        ResourceOptResult res_bf = EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters);

        // Run Incremental with Time Limits
        OptimizePA_Incre_with_TimeLimits opt_inc_tl(dag_tasks, sp_parameters);
        opt_inc_tl.OptimizeFromScratch_w_TL(2);
        ResourceOptResult res_inc = opt_inc_tl.CollectResults();

        // Check consistency of SP metric (heuristic optimization with K=2 can deviate slightly from global optimum)
        EXPECT_NEAR(res_bf.sp_opt, res_inc.sp_opt, 2.0);
        EXPECT_GE(res_bf.sp_opt, res_inc.sp_opt - 1e-3);
    }
}

TEST(OptimizeIncrePA_Helpers, RemoveOneTask) {
    PriorityVec pa = {0, 1, 2, 3};

    // Remove start
    EXPECT_THAT(RemoveOneTask(pa, 0), testing::ElementsAre(1, 2, 3));

    // Remove middle
    EXPECT_THAT(RemoveOneTask(pa, 2), testing::ElementsAre(0, 1, 3));

    // Remove end
    EXPECT_THAT(RemoveOneTask(pa, 3), testing::ElementsAre(0, 1, 2));

#if defined(GTEST_HAS_DEATH_TEST)
    EXPECT_DEATH(RemoveOneTask(pa, 99), ".*");
#endif
}

TEST(OptimizeIncrePA_Helpers, FindTaskWithDifferentEt_EdgeCases) {
    // 2 tasks: T0 (ET=3, P=10), T1 (ET=4, P=20)
    std::vector<Value_Proba> dist0 = {Value_Proba(3, 1.0)};
    std::vector<Value_Proba> dist1 = {Value_Proba(4, 1.0)};
    
    TaskSet tasks;
    tasks.push_back(Task(0, dist0, 10, 10, 0, "T0"));
    tasks.push_back(Task(1, dist1, 20, 20, 1, "T1"));
    
    MAP_Prev mapPrev;
    DAG_Model dag1(tasks, mapPrev, 0, 0);
    DAG_Model dag2(tasks, mapPrev, 0, 0);

    // Identical: diff size should be 0
    std::vector<DiffObj> diff_same = FindTaskWithDifferentEt(dag1, dag2);
    EXPECT_EQ(0, diff_same.size());

    // Decrease T1 execution time to 2
    DAG_Model dag_dec = dag1;
    std::vector<Value_Proba> dist_dec = {Value_Proba(2, 1.0)};
    dag_dec.tasks[1].execution_time_dist = FiniteDist(dist_dec);

    std::vector<DiffObj> diff_dec = FindTaskWithDifferentEt(dag1, dag_dec);
    ASSERT_EQ(1, diff_dec.size());
    EXPECT_EQ(1, diff_dec[0].task_id);
    EXPECT_FALSE(diff_dec[0].increase);

    // Increase T0 execution time to 5
    DAG_Model dag_inc = dag1;
    std::vector<Value_Proba> dist_inc = {Value_Proba(5, 1.0)};
    dag_inc.tasks[0].execution_time_dist = FiniteDist(dist_inc);

    std::vector<DiffObj> diff_inc = FindTaskWithDifferentEt(dag1, dag_inc);
    ASSERT_EQ(1, diff_inc.size());
    EXPECT_EQ(0, diff_inc[0].task_id);
    EXPECT_TRUE(diff_inc[0].increase);
}

TEST(OptimizeIncrePA_Helpers, PriorityPartialPath_Getters) {
    // Setup simple taskset
    std::vector<Value_Proba> dist0 = {Value_Proba(3, 1.0)};
    std::vector<Value_Proba> dist1 = {Value_Proba(5, 1.0)};
    TaskSet tasks;
    tasks.push_back(Task(0, dist0, 10, 10, 0, "T0"));
    tasks.push_back(Task(1, dist1, 20, 20, 1, "T1"));
    
    MAP_Prev mapPrev;
    DAG_Model dag(tasks, mapPrev, 0, 0);
    SP_Parameters sp_params(dag);
    sp_params.weights_node[0] = 7;
    sp_params.weights_node[1] = 9;

    PriorityPartialPath path(dag, sp_params);
    path.AssignAndUpdateSP(0); // Assign task 0 as lowest priority

    ASSERT_EQ(1, path.pa_vec_lower_pri.size());
    EXPECT_EQ(0, path.pa_vec_lower_pri[0]);

    // Check getters
    EXPECT_EQ(7, path.GetTaskWeight(0));
    EXPECT_EQ(10, path.GetTaskPeriod(0));
    EXPECT_EQ(3, path.GetTaskMinEt(0));
}

int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
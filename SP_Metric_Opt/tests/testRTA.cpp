// #include <gtest/gtest.h>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Safety_Performance_Metric/Probability.h"
#include "sources/Safety_Performance_Metric/RTA.h"
#include "sources/TaskModel/DAG_Model.h"
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
    }

    // data members
    TaskSet tasks;
};
TEST_F(TaskSetForTest_2tasks, RTA) {
    GlobalVariables::Granularity = 10;
    std::vector<Value_Proba> dist_vec0 = {
        Value_Proba(1, 0.6), Value_Proba(2, 0.3), Value_Proba(3, 0.1)};
    FiniteDist rta0_expected(dist_vec0);

    std::vector<Value_Proba> dist_vec1 = {
        Value_Proba(5, 0.42),   Value_Proba(7, 0.234),  Value_Proba(8, 0.213),
        Value_Proba(9, 0.105),  Value_Proba(10, 0.025), Value_Proba(12, 0.0018),
        Value_Proba(13, 0.0012)};
    FiniteDist rta1_expected(dist_vec1);
    vector<FiniteDist> rtas_actual = ProbabilisticRTA_TaskSet_SingleCore(tasks);
    EXPECT_EQ(2, rtas_actual.size());
    EXPECT_EQ(rta0_expected, rtas_actual[0]);

    EXPECT_TRUE(rta1_expected == rtas_actual[1]);
    rta1_expected.print();
    rtas_actual[1].print();
}

class TaskSetForTest_2tasks_miss_ddl : public ::testing::Test {
   public:
    void SetUp() override {
        std::vector<Value_Proba> dist_vec1 = {Value_Proba(1, 0.6),
                                              Value_Proba(2, 0.4)};
        std::vector<Value_Proba> dist_vec2 = {Value_Proba(8, 0.4),
                                              Value_Proba(9, 0.6)};
        tasks.push_back(Task(0, dist_vec1, 4, 4, 0));
        tasks.push_back(Task(1, dist_vec2, 10, 10, 1));
    }

    // data members
    TaskSet tasks;
};
TEST_F(TaskSetForTest_2tasks_miss_ddl, RTA) {
    GlobalVariables::Granularity = 10;
    std::vector<Value_Proba> dist_vec0 = {Value_Proba(1, 0.6),
                                          Value_Proba(2, 0.4)};
    FiniteDist rta0_expected(dist_vec0);

    std::vector<Value_Proba> dist_vec1 = {Value_Proba(11, 1)};
    FiniteDist rta1_expected(dist_vec1);
    vector<FiniteDist> rtas_actual = ProbabilisticRTA_TaskSet_SingleCore(tasks);
    EXPECT_EQ(2, rtas_actual.size());
    EXPECT_EQ(rta0_expected, rtas_actual[0]);

    EXPECT_TRUE(rta1_expected == rtas_actual[1]);
    rta1_expected.print();
    rtas_actual[1].print();
}
class TaskSetForTest_2tasks_miss_partial_ddl : public ::testing::Test {
   public:
    void SetUp() override {
        std::vector<Value_Proba> dist_vec1 = {Value_Proba(1, 0.6),
                                              Value_Proba(2, 0.4)};
        std::vector<Value_Proba> dist_vec2 = {Value_Proba(1, 0.4),
                                              Value_Proba(9, 0.6)};
        tasks.push_back(Task(0, dist_vec1, 4, 4, 0));
        tasks.push_back(Task(1, dist_vec2, 10, 10, 1));
    }

    // data members
    TaskSet tasks;
};
TEST_F(TaskSetForTest_2tasks_miss_partial_ddl, RTA) {
    GlobalVariables::Granularity = 10;
    std::vector<Value_Proba> dist_vec0 = {Value_Proba(1, 0.6),
                                          Value_Proba(2, 0.4)};
    FiniteDist rta0_expected(dist_vec0);

    std::vector<Value_Proba> dist_vec1 = {
        Value_Proba(2, 0.24), Value_Proba(3, 0.16), Value_Proba(11, 0.6)};
    FiniteDist rta1_expected(dist_vec1);
    vector<FiniteDist> rtas_actual = ProbabilisticRTA_TaskSet_SingleCore(tasks);
    EXPECT_EQ(2, rtas_actual.size());
    EXPECT_EQ(rta0_expected, rtas_actual[0]);

    EXPECT_TRUE(rta1_expected == rtas_actual[1]);
    rta1_expected.print();
    rtas_actual[1].print();
}
TEST_F(TaskSetForTest_2tasks, RTA_change_priority) {
    GlobalVariables::Granularity = 10;
    std::vector<Value_Proba> dist_vec0 = {
        Value_Proba(1, 0.6), Value_Proba(2, 0.3), Value_Proba(3, 0.1)};
    FiniteDist rta0_expected(dist_vec0);

    std::vector<Value_Proba> dist_vec1 = {
        Value_Proba(5, 0.42),   Value_Proba(7, 0.234),  Value_Proba(8, 0.213),
        Value_Proba(9, 0.105),  Value_Proba(10, 0.025), Value_Proba(12, 0.0018),
        Value_Proba(13, 0.0012)};
    FiniteDist rta1_expected(dist_vec1);

    // swap task 0 and task 1
    Task temp = tasks[0];
    tasks[0] = tasks[1];
    tasks[1] = temp;

    vector<FiniteDist> rtas_actual = ProbabilisticRTA_TaskSet_SingleCore(tasks);
    EXPECT_EQ(2, rtas_actual.size());
    EXPECT_EQ(rta0_expected, rtas_actual[1]);
    EXPECT_TRUE(rta1_expected == rtas_actual[0]);
}

TEST_F(TaskSetForTest_2tasks, GetDDL_MissProbability) {
    GlobalVariables::Granularity = 10;
    std::vector<FiniteDist> rtas = ProbabilisticRTA_TaskSet_SingleCore(tasks);

    std::vector<Value_Proba> dist_vec0 = {
        Value_Proba(1, 0.6), Value_Proba(2, 0.3), Value_Proba(3, 0.1)};
    FiniteDist rta0_expected(dist_vec0);

    std::vector<Value_Proba> dist_vec1 = {
        Value_Proba(5, 0.42),   Value_Proba(7, 0.234),  Value_Proba(8, 0.213),
        Value_Proba(9, 0.105),  Value_Proba(10, 0.025), Value_Proba(12, 0.0018),
        Value_Proba(13, 0.0012)};
    FiniteDist rta1_expected(dist_vec1);
    vector<FiniteDist> rtas_actual = ProbabilisticRTA_TaskSet_SingleCore(tasks);
    EXPECT_NEAR(0.0012, GetDDL_MissProbability(rtas[1], 12), 1e-6);
}

TEST_F(TaskSetForTest_2tasks, GetDDL_MissProbability_v2) {
    GlobalVariables::Granularity = 10;
    FiniteDist dists({5, 6, 7, 8, 5, 6, 7, 8}, 10);

    EXPECT_NEAR(0.0, GetDDL_MissProbability(dists, 10), 1e-6);
    EXPECT_NEAR(1.0, GetDDL_MissProbability(dists, 4), 1e-6);
    EXPECT_NEAR(0.75, GetDDL_MissProbability(dists, 5), 1e-6);
    EXPECT_NEAR(0.5, GetDDL_MissProbability(dists, 6.5), 1e-6);
}
TEST_F(TaskSetForTest_2tasks, GetDDL_MissProbability_v3) {
    GlobalVariables::Granularity = 10;
    FiniteDist dists({5}, 10);

    EXPECT_NEAR(0.0, GetDDL_MissProbability(dists, 10), 1e-6);
    EXPECT_NEAR(1.0, GetDDL_MissProbability(dists, 4), 1e-6);
    EXPECT_NEAR(0.0, GetDDL_MissProbability(dists, 5), 1e-6);
}
TEST_F(TaskSetForTest_2tasks, GetDDL_MissProbability_v4) {
    GlobalVariables::Granularity = 10;
    FiniteDist dists({5, 10000, 10000}, 10);

    EXPECT_NEAR(0.66666666, GetDDL_MissProbability(dists, 10), 1e-3);
    EXPECT_NEAR(1.0, GetDDL_MissProbability(dists, 4), 1e-3);
    EXPECT_NEAR(0.66666666, GetDDL_MissProbability(dists, 5), 1e-3);
}
class TaskSetv9 : public ::testing::Test {
   public:
    void SetUp() override {
        string file_path =
            GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v9.yaml";
        dag_tasks = ReadDAG_Tasks(file_path);
    }

    // data members
    DAG_Model dag_tasks;
};
TEST_F(TaskSetv9, RTA_w_processor_assignment) {
    GlobalVariables::Granularity = 10;
    std::vector<FiniteDist> rtas = ProbabilisticRTA_TaskSet(dag_tasks.tasks);
    // RT distribution from 200 to 200
    EXPECT_NEAR(0.0, GetDDL_MissProbability(rtas[0], 200), 1e-6);
    EXPECT_NEAR(1.0, GetDDL_MissProbability(rtas[0], 199), 1e-6);

    // RT distribution from 200 to 600
    EXPECT_NEAR(0.588, GetDDL_MissProbability(rtas[1], 400), 1e-3);
}

TEST(RTA_EdgeCases, DeterministicTaskSet) {
    // 2 tasks: T0 (ET=3, P=10), T1 (ET=4, P=20)
    std::vector<Value_Proba> dist0 = {Value_Proba(3, 1.0)};
    std::vector<Value_Proba> dist1 = {Value_Proba(4, 1.0)};
    
    TaskSet tasks;
    tasks.push_back(Task(0, dist0, 10, 10, 0, "T0"));
    tasks.push_back(Task(1, dist1, 20, 20, 1, "T1"));
    
    // Core 0 for both
    tasks[0].processorId = 0;
    tasks[1].processorId = 0;
    
    // Set priorities (small index has higher priority: FTP)
    tasks[0].priority = 0;
    tasks[1].priority = 1;
    
    vector<FiniteDist> rtas = ProbabilisticRTA_TaskSet_SingleCore(tasks);
    
    ASSERT_EQ(2, rtas.size());
    // T0 (highest priority): Response time must be exactly 3 with probability 1.0
    EXPECT_EQ(1, rtas[0].distribution.size());
    EXPECT_NEAR(3.0, rtas[0].distribution[0].value, 1e-6);
    EXPECT_NEAR(1.0, rtas[0].distribution[0].probability, 1e-6);
    
    // T1 (lowest priority): worst-case response time under critical instant:
    // It is preempted by T0.
    // In traditional RTA: R_1 = C_1 + ceil(R_1 / T_0) * C_0
    // Try R_1 = 4 + ceil(R_1/10)*3. If R_1=7: 4 + 1*3 = 7. Match!
    // So response time for T1 must be exactly 7 with probability 1.0
    EXPECT_EQ(1, rtas[1].distribution.size());
    EXPECT_NEAR(7.0, rtas[1].distribution[0].value, 1e-6);
    EXPECT_NEAR(1.0, rtas[1].distribution[0].probability, 1e-6);
}

TEST(RTA_EdgeCases, OverloadedSystem) {
    // T0 (ET=8, P=10), T1 (ET=8, P=15)
    // Total utilization = 8/10 + 8/15 = 0.8 + 0.533 = 1.333 > 1.0 on a single core
    std::vector<Value_Proba> dist0 = {Value_Proba(8, 1.0)};
    std::vector<Value_Proba> dist1 = {Value_Proba(8, 1.0)};
    
    TaskSet tasks;
    tasks.push_back(Task(0, dist0, 10, 10, 0, "T0"));
    tasks.push_back(Task(1, dist1, 15, 15, 1, "T1"));
    
    tasks[0].processorId = 0;
    tasks[1].processorId = 0;
    
    tasks[0].priority = 0;
    tasks[1].priority = 1;
    
    // In an overloaded system, RTA should calculate high miss probabilities.
    vector<FiniteDist> rtas = ProbabilisticRTA_TaskSet(tasks);
    
    ASSERT_EQ(2, rtas.size());
    // For T1, the response time should exceed its deadline (15).
    // Let's check the deadline miss probability
    double miss_prob = GetDDL_MissProbability(rtas[1], 15);
    EXPECT_GE(miss_prob, 0.99); // It should miss deadline with high probability
}

TEST(RTA_EdgeCases, MultiCorePartitioning) {
    // 3 tasks partitioned on 2 cores
    // Core 0: Task 0 (ET=4, P=10), Task 2 (ET=3, P=20)
    // Core 1: Task 1 (ET=12, P=20) - which would normally conflict if on the same core
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

    tasks[0].priority = 0;
    tasks[1].priority = 0; // Highest on core 1
    tasks[2].priority = 1;

    vector<FiniteDist> rtas = ProbabilisticRTA_TaskSet(tasks);

    ASSERT_EQ(3, rtas.size());
    // Task 1 runs independently on core 1. Its response time should be exactly 12 (its ET).
    EXPECT_NEAR(12.0, rtas[1].distribution[0].value, 1e-6);
    EXPECT_NEAR(1.0, rtas[1].distribution[0].probability, 1e-6);

    // Task 0 runs highest priority on core 0: response time exactly 4.
    EXPECT_NEAR(4.0, rtas[0].distribution[0].value, 1e-6);

    // Task 2 runs lowest priority on core 0: response time: 3 + ceil(R/10)*4 = 7.
    EXPECT_NEAR(7.0, rtas[2].distribution[0].value, 1e-6);
}

int main(int argc, char **argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
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
// 3-task single-core fixture for the HP-prefix checkpoint store (P1.9 step 2).
// priorities 0 < 1 < 2 → sorted HP-first order is exactly {t0, t1, t2}; gives
// hp_tasks_et_conv_vec non-trivial intermediate entries at indices 0,1,2.
class TaskSetForTest_3tasks_prefix : public ::testing::Test {
   public:
    void SetUp() override {
        GlobalVariables::Granularity = 10;
        std::vector<Value_Proba> d0 = {
            Value_Proba(1, 0.6), Value_Proba(2, 0.3), Value_Proba(3, 0.1)};
        std::vector<Value_Proba> d1 = {Value_Proba(4, 0.7),
                                       Value_Proba(5, 0.3)};
        std::vector<Value_Proba> d2 = {Value_Proba(2, 0.5),
                                       Value_Proba(6, 0.5)};
        // Task(id, exec, period, deadline, priority) — single-core RTA ignores
        // processorId (sorts by priority); 5-arg form matches the existing
        // fixtures above.
        tasks.push_back(Task(0, d0, 5, 5, 0));
        tasks.push_back(Task(1, d1, 12, 12, 1));
        tasks.push_back(Task(2, d2, 20, 20, 2));
    }
    TaskSet tasks;
};

// The 2-arg overload must reproduce the rolling hp_tasks_et_conv bit-for-bit:
// hp_tasks_et_conv_vec[i] == convolution of the higher-priority tasks' ET
// dists [0, i), with hp_tasks_et_conv_vec[0] == FiniteDist({Value_Proba(0,
// 1.0)}). This independently replays the rolling computation from
// RTA.cpp:73,81-82 and compares.
TEST_F(TaskSetForTest_3tasks_prefix, HpTasksEtConvVec_MatchesRollingValue) {
    std::vector<FiniteDist> rtas;
    std::vector<FiniteDist> hp_tasks_et_conv_vec;
    ProbabilisticRTA_TaskSet_SingleCore(tasks, hp_tasks_et_conv_vec);

    ASSERT_EQ(tasks.size(), hp_tasks_et_conv_vec.size());

    // Independent replay of the rolling hp_tasks_et_conv.
    FiniteDist rolling({Value_Proba(0, 1.0)});
    for (size_t i = 0; i < tasks.size(); i++) {
        EXPECT_TRUE(hp_tasks_et_conv_vec[i] == rolling)
            << "hp_tasks_et_conv_vec[" << i << "] != rolling value";
        rolling.CompressDistributionWithOnlySize(
            GlobalVariables::Granularity * 1);
        rolling.Convolve(tasks[i].execution_time_dist);
    }
}

// Behavior preservation: the 2-arg overload must return bit-identical rtas to
// the 1-arg version on the same input (the pinned-rtas tests above are the
// oracle; this adds an explicit same-input differential).
TEST_F(TaskSetForTest_3tasks_prefix, TwoArgOverload_SameRtasAsOneArg) {
    std::vector<FiniteDist> rtas_one_arg =
        ProbabilisticRTA_TaskSet_SingleCore(tasks);

    std::vector<FiniteDist> rtas_two_arg;
    std::vector<FiniteDist> hp_tasks_et_conv_vec_ignored;
    rtas_two_arg = ProbabilisticRTA_TaskSet_SingleCore(tasks, hp_tasks_et_conv_vec_ignored);

    ASSERT_EQ(rtas_one_arg.size(), rtas_two_arg.size());
    for (size_t i = 0; i < rtas_one_arg.size(); i++) {
        EXPECT_TRUE(rtas_one_arg[i] == rtas_two_arg[i])
            << "rtas[" << i << "] diverged between 1-arg and 2-arg overloads";
    }
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
    EXPECT_NEAR(0.59, GetDDL_MissProbability(rtas[1], 400), 1e-2);
}
int main(int argc, char **argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
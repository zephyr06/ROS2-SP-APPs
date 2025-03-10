// #include <gtest/gtest.h>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Optimization/OptimizeSP_BF.h"
#include "sources/Optimization/OptimizeSP_TL_Incre.h"
#include "sources/Safety_Performance_Metric/Probability.h"
#include "sources/TaskModel/RegularTasks.h"
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

class TaskSetForTest_robotics_v20 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v20";
        std::string path =
            GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
        file_path = path;
        dag_tasks = ReadDAG_Tasks(path, 5);
        sp_parameters = SP_Parameters(dag_tasks);
    }

    // data members
    string file_path;
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
    int N = dag_tasks.tasks.size();
};

TEST_F(TaskSetForTest_robotics_v20, RecordCloseTimeLimitOptions) {
    std::vector<std::vector<double>> time_limit_options =
        RecordCloseTimeLimitOptions(dag_tasks);
    EXPECT_EQ(184.1, time_limit_options[0][0]);
    EXPECT_EQ(397.5, time_limit_options[0][1]);

    EXPECT_EQ(-1, time_limit_options[1][0]);
    EXPECT_EQ(-1, time_limit_options[2][0]);
    EXPECT_EQ(-1, time_limit_options[3][0]);
}

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

TEST_F(TaskSetForTest_robotics_v18, optimize) {
    // ResourceOptResult res_opt =
    //     EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters);
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    opt.OptimizeFromScratch_w_TL(2);
    ResourceOptResult res_opt = opt.CollectResults();
    PrintPriorityVec(dag_tasks.tasks, res_opt.priority_vec);

    EXPECT_EQ(1000, res_opt.id2time_limit[0]);  // SLAM+TSP have low utilization
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

class TaskSetForTest_robotics_v19_2 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v19_2";
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

class TestDDLMiss : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_ddl_miss";
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

class TestDDLMissLessTasks : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_ddl_miss_less_tasks";
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

TEST_F(TaskSetForTest_robotics_v19, RecordCloseTimeLimitOptions) {
    std::vector<std::vector<double>> time_limit_options =
        RecordCloseTimeLimitOptions(dag_tasks);
    EXPECT_EQ(4, time_limit_options.size());     // 4 tasks
    EXPECT_EQ(2, time_limit_options[0].size());  // 2 options for TSP
    EXPECT_EQ(800, time_limit_options[0][0]);
    EXPECT_EQ(1000, time_limit_options[0][1]);

    EXPECT_EQ(-1, time_limit_options[1][0]);
    EXPECT_EQ(-1, time_limit_options[2][0]);
    EXPECT_EQ(-1, time_limit_options[3][0]);
}
TEST_F(TaskSetForTest_robotics_v19, OptimizeFromScratch_w_TL) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    EXPECT_FALSE(opt.IfInitialized());
    opt.OptimizeFromScratch_w_TL(2);
    EXPECT_TRUE(opt.IfInitialized());
    ResourceOptResult res_opt = opt.CollectResults();
    PrintPriorityVec(dag_tasks.tasks, res_opt.priority_vec);
    EXPECT_EQ(400,
              res_opt.id2time_limit[0]);  // SLAM+TSP have high utilization;
    //   In scratch mode, should return 400
    // In incremental mode, should return 800
}

TEST_F(TaskSetForTest_robotics_v19, optimize_incremental) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks,
                                         sp_parameters);  // high utilization

    opt.OptimizeFromScratch_w_TL(2);  // result is 400
    ResourceOptResult res_opt = opt.CollectResults();
    EXPECT_EQ(400,
              res_opt.id2time_limit[0]);  // SLAM+TSP have high utilization;

    DAG_Model dag_tasks_updated =
        ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                      "TaskData/test_robotics_v21.yaml");  // low utilization
    opt.OptimizeIncre_w_TL(dag_tasks_updated, 2);
    res_opt = opt.CollectResults();
    EXPECT_EQ(
        600,
        res_opt.id2time_limit[0]);  // change only to nearby ET level each time

    auto start_time = CurrentTimeInProfiler;
    for (int i = 0; i < 10; i++) opt.OptimizeIncre_w_TL(dag_tasks_updated, 2);
    auto finish_time = CurrentTimeInProfiler;
    double time_taken = GetTimeTaken(start_time, finish_time);
    EXPECT_LT(time_taken / 10.0,
              5e-2);  // since no adjustemnts are made, it should be very fast

    // dag_tasks_updated =
    //     ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
    //                   "TaskData/test_robotics_v22.yaml");  // low utilization
}

TEST_F(TaskSetForTest_robotics_v19_2, RecordCloseTimeLimitOptions) {
    printf(
        "\n-------- TaskSetForTest_robotics_v19_2, RecordCloseTimeLimitOptions "
        "...\n");
    std::vector<std::vector<double>> time_limit_options =
        RecordCloseTimeLimitOptions(dag_tasks);

    EXPECT_EQ(4, time_limit_options.size());  // 4 tasks

    int perfTask = 0;
    for (int i = 0; i < static_cast<int>(time_limit_options.size()); i++) {
        if (time_limit_options[i][0] != -1) {
            perfTask = i;
            break;
        }
    }

    EXPECT_EQ(2, time_limit_options[perfTask].size());  // 2 options for TSP
    EXPECT_EQ(800, time_limit_options[perfTask][0]);
    EXPECT_EQ(1000, time_limit_options[perfTask][1]);

    for (uint i = 0; i < time_limit_options.size(); i++) {
        if (i == perfTask) continue;
        EXPECT_EQ(-1, time_limit_options[i][0]);
    }
}

TEST_F(TaskSetForTest_robotics_v19_2, OptimizeFromScratch_w_TL) {
    // NOTE: this test failed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // compare with the testcase (TaskSetForTest_robotics_v19) before, the only
    // difference is that this testcase added two tasks with very small
    // cpu_utilization and big weight supposedly, OptimizeFromScratch_w_TL
    // should also return 400 for task0

    // try to get which task has performance_records_time
    int perfTask = 0;
    std::vector<std::vector<double>> time_limit_options =
        RecordCloseTimeLimitOptions(dag_tasks);
    for (int i = 0; i < static_cast<int>(time_limit_options.size()); i++) {
        if (time_limit_options[i][0] != -1) {
            perfTask = i;
            break;
        }
    }

    printf(
        "\n-------- TaskSetForTest_robotics_v19_2, OptimizeFromScratch_w_TL "
        "...\n");
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    EXPECT_FALSE(opt.IfInitialized());

    // int n = dag_tasks.tasks.size();
    // for (int i=0;i<n;i++){
    //     double exeT = dag_tasks.GetTask(i).getExecGaussian().mu;
    //     const_cast<SP_OPT_PA::Task&>(dag_tasks.GetTask(i)).setExecutionTime(exeT);
    //     printf("task%d:
    //     exeT=%f\n",i,(double)(dag_tasks.GetTask(i).getExecutionTime()));
    // }
    opt.OptimizeFromScratch_w_TL(2);
    EXPECT_TRUE(opt.IfInitialized());
    ResourceOptResult res_opt = opt.CollectResults();
    PrintPriorityVec(dag_tasks.tasks, res_opt.priority_vec);
    EXPECT_EQ(
        400,
        res_opt.id2time_limit[perfTask]);  // SLAM+TSP have high utilization;
}

TEST_F(TestDDLMiss, test_ddl_miss) {
    // the very last task has high utilization
    // the last one should definitely miss deadline!
    double cpu_util = 0.0;
    for (int i = 0; i < static_cast<int>(dag_tasks.tasks.size()); i++) {
        double mu = dag_tasks.tasks[i].execution_time_dist.GetAvgValue();
        printf("task%d, period=%d, deadline=%.0f, mu=%.2f\n", i,
               dag_tasks.tasks[i].period, dag_tasks.tasks[i].deadline, mu);
        cpu_util += mu / dag_tasks.tasks[i].period;
    }
    printf("overall cpu util=%.2f\n\n", cpu_util);

    std::vector<FiniteDist> rtas = ProbabilisticRTA_TaskSet(dag_tasks.tasks);
    for (uint i = 0; i < rtas.size(); i++) {
        std::cout << "rtas[" << i << "]: ..." << std::endl;
        rtas[i].print();
    }
    printf("\n");
    for (uint i = 0; i < dag_tasks.tasks.size(); i++) {
        double ddl_miss_chance =
            GetDDL_MissProbability(rtas[i], dag_tasks.tasks[i].deadline);
        printf("task%d, ddl_miss_chance=%.2f\n", i, ddl_miss_chance);

        if (i == dag_tasks.tasks.size() - 1) {
            EXPECT_GT(ddl_miss_chance, 0.99999);
        } else {
            EXPECT_LT(ddl_miss_chance, 0.00001);
        }
    }
}

TEST_F(TestDDLMissLessTasks, test_ddl_miss) {
    // the very last task has high utilization
    // the last one should definitely miss deadline!
    double cpu_util = 0.0;
    for (int i = 0; i < static_cast<int>(dag_tasks.tasks.size()); i++) {
        double mu = dag_tasks.tasks[i].execution_time_dist.GetAvgValue();
        printf("task%d, period=%d, deadline=%.0f, mu=%.2f\n", i,
               dag_tasks.tasks[i].period, dag_tasks.tasks[i].deadline, mu);
        cpu_util += mu / dag_tasks.tasks[i].period;
    }
    printf("overall cpu util=%.2f\n\n", cpu_util);

    std::vector<FiniteDist> rtas = ProbabilisticRTA_TaskSet(dag_tasks.tasks);
    for (uint i = 0; i < rtas.size(); i++) {
        std::cout << "rtas[" << i << "]: ..." << std::endl;
        rtas[i].print();
    }
    printf("\n");
    for (int i = 0; i < static_cast<int>(dag_tasks.tasks.size()); i++) {
        double ddl_miss_chance =
            GetDDL_MissProbability(rtas[i], dag_tasks.tasks[i].deadline);
        printf("task%d, ddl_miss_chance=%.2f\n", i, ddl_miss_chance);

        if (i == static_cast<int>(dag_tasks.tasks.size()) - 1) {
            EXPECT_GT(ddl_miss_chance, 0.99999);
        } else {
            EXPECT_LT(ddl_miss_chance, 0.00001);
        }
    }
}

int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
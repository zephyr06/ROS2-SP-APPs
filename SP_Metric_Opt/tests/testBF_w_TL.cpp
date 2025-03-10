// #include <gtest/gtest.h>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Optimization/OptimizeSP_BF.h"
#include "sources/Optimization/OptimizeSP_TL_BF.h"
#include "sources/Optimization/OptimizeSP_TL_Incre.h"
#include "sources/RTDA/ImplicitCommunication/ScheduleSimulation.h"
#include "sources/Utils/Parameters.h"

using ::testing::AtLeast;  // #1
using ::testing::Return;
using namespace std;
using namespace SP_OPT_PA;
using namespace GlobalVariables;

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

TEST_F(TaskSetForTest_robotics_v19, EnumeratePA_with_TimeLimits) {
    ResourceOptResult res_opt =
        EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters);

    PrintPriorityVec(dag_tasks.tasks, res_opt.priority_vec);
    EXPECT_EQ(400,
              res_opt.id2time_limit[0]);  // SLAM+TSP have high utilization;
    // BF should return 400
    // std::cout<<"TaskSetForTest_robotics_v19 EnumeratePA_with_TimeLimits
    // done"<<std::endl;
}

TEST_F(TaskSetForTest_robotics_v19, EnumeratePA_with_TimeLimits_2) {
    ResourceOptResult res_opt =
        EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters);
    EXPECT_EQ(400,
              res_opt.id2time_limit[0]);  // SLAM+TSP have high utilization;
    std::cout << "#### TaskSetForTest_robotics_v19 BF run TSP worst"
              << std::endl;

    DAG_Model dag_tasks_updated =
        ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                      "TaskData/test_robotics_v21.yaml");  // low utilization
    res_opt = EnumeratePA_with_TimeLimits(dag_tasks_updated, sp_parameters);
    EXPECT_EQ(1000,
              res_opt.id2time_limit[0]);  // TSP can run to best it can
    std::cout << "#### TaskSetForTest_robotics_v21 BF run TSP best"
              << std::endl;

    dag_tasks_updated =
        ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                      "TaskData/test_robotics_v22.yaml");  // low utilization
    res_opt = EnumeratePA_with_TimeLimits(dag_tasks_updated, sp_parameters);
    EXPECT_EQ(1000,
              res_opt.id2time_limit[0]);  // TSP can run to best it can
    std::cout << "#### TaskSetForTest_robotics_v22 BF run TSP best"
              << std::endl;

    dag_tasks_updated =
        ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                      "TaskData/test_robotics_v19.yaml");  // low utilization
    res_opt = EnumeratePA_with_TimeLimits(dag_tasks_updated, sp_parameters);
    EXPECT_EQ(400,
              res_opt.id2time_limit[0]);  // TSP can only run worst again
    std::cout << "#### TaskSetForTest_robotics_v19 BF run TSP worst"
              << std::endl;
}

int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
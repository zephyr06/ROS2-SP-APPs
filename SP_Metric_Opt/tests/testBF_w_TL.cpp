// #include <gtest/gtest.h>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Optimization/OptUtilsFunctionsFromRyan.h"
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

class TaskSetForTest_taskset_cfg_10_1_gen_1 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "taskset_characteristics_0";
        std::string path = GlobalVariables::PROJECT_PATH +
                           "TaskData/taskset_cfg_10_1_gen_1/" + file_name +
                           ".yaml";
        file_path = path;
        // dag_tasks = ReadDAG_Tasks(path, 5);
        dag_tasks = ReadDAG_Tasks(path);
        sp_parameters = SP_Parameters(dag_tasks);
    }

    // data members
    string file_path;
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
    int N = dag_tasks.tasks.size();
};

class TaskSetForTest_taskset_cfg_4_5_gen_1 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "taskset_characteristics_0";
        std::string path = GlobalVariables::PROJECT_PATH +
                           "TaskData/taskset_cfg_4_5_gen_1/" + file_name +
                           ".yaml";
        file_path = path;
        // dag_tasks = ReadDAG_Tasks(path, 5);
        dag_tasks = ReadDAG_Tasks(path);
        // sp_parameters = SP_Parameters(dag_tasks);
        sp_parameters = ReadSP_Parameters(path);
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

TEST_F(TaskSetForTest_taskset_cfg_10_1_gen_1, ObtainSP_DAG) {
    std::cout << "\n\n\nStart Running the Long Test, ObtainSP_DAG:\n\n\n";
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    auto start_time = CurrentTimeInProfiler;
    // ResourceOptResult res_opt =
    //     EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters);
    double initial_sp = ObtainSP_DAG(dag_tasks, sp_parameters);
    auto finish_time = CurrentTimeInProfiler;
    double time_taken = GetTimeTaken(start_time, finish_time);
    std::cout << "time taken for OptimizePA_Incre_with_TimeLimits:"
              << time_taken << std::endl;
}

TEST_F(TaskSetForTest_taskset_cfg_10_1_gen_1,
       EnumeratePA_with_TimeLimits_long) {
    std::cout << "\n\n\n Start Running the Long Test:\n\n\n";
    auto start_time = CurrentTimeInProfiler;
    ResourceOptResult res_opt =
        EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters);
    auto finish_time = CurrentTimeInProfiler;
    double time_taken = GetTimeTaken(start_time, finish_time);
    std::cout << "time taken for OptimizePA_Incre_with_TimeLimits:"
              << time_taken << std::endl;
    std::cout << "task 0 exeT: " << res_opt.id2time_limit[0] << std::endl;
}

TEST_F(TaskSetForTest_taskset_cfg_10_1_gen_1,
       EnumeratePA_with_TimeLimits_sorted) {
    std::cout << "\n\n\nStart Running the Long Test sorted options:\n\n\n";
    auto start_time = CurrentTimeInProfiler;
    ResourceOptResult res_opt =
        EnumeratePA_with_TimeLimits_sortOptionsFirst(dag_tasks, sp_parameters);
    auto finish_time = CurrentTimeInProfiler;
    double time_taken = GetTimeTaken(start_time, finish_time);
    std::cout << "time taken for OptimizePA_Incre_with_TimeLimits:"
              << time_taken << std::endl;
    std::cout << "task 0 exeT: " << res_opt.id2time_limit[0] << std::endl;
}

TEST_F(TaskSetForTest_taskset_cfg_10_1_gen_1, enumperate_ExeTOptions) {
    std::cout << "\n\n\nTaskSetForTest_taskset_cfg_10_1_gen_1: "
                 "enumperate_ExeTOptions:\n\n\n";
    std::vector<std::vector<double>> options =
        enumerate_all_exeTOptions(dag_tasks);

    double util_regular_tasks = 0.0;
    std::vector<double> prds;
    for (int i = 0; i < static_cast<int>(dag_tasks.tasks.size()); i++) {
        double prd = dag_tasks.tasks[i].period;
        prds.push_back(prd);
        if (dag_tasks.tasks[i].timePerformancePairs.size() == 0) {
            double mu = dag_tasks.tasks[i].getExecGaussian().mu;
            util_regular_tasks += mu / prd;
        }
    }
    double last_u;
    for (int i = 0; i < options.size(); i++) {
        // iter over options
        double u = util_regular_tasks;
        for (int k = 0; k < static_cast<int>(options[i].size()); k++) {
            // iter over tasks
            if (options[i][k] > 0.0) {
                u += options[i][k] / prds[k];
            }
        }
        if (i == 0) {
            last_u = u;
        } else {
            double last_abs = abs(last_u - 1.0);
            double this_abs = abs(u - 1.0);
            // the prio should be closer to 100% utilization
            EXPECT_LT(last_abs - this_abs, 0.0);
            last_u = u;
        }
    }
    std::cout << "TaskSetForTest_taskset_cfg_10_1_gen_1: "
                 "enumperate_ExeTOptions done\n";
}

TEST_F(TaskSetForTest_taskset_cfg_4_5_gen_1, check_id2time_limit) {
    // simulate from 0 to 300s, cpu util should increase and then descrease
    // one task_characteristics file for every 10s
    // check task 3 (with performance_records_time) exetime
    int task_wPerf = 3;  // this task has performance_records_time

    std::cout << "\n-------- check id2time_limit varies with cpu utilization\n";

    // time 0:
    ResourceOptResult res_opt =
        EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters);
    // calculate cpu_util for other tasks
    double cpu_util = 0.0;
    for (int i = 0; i < 4; i++) {
        if (i == task_wPerf) continue;
        const Task t = dag_tasks.GetTask(i);
        int prd = t.period;
        GaussianDist g_et = t.getExecGaussian();
        double mu = g_et.mu;
        cpu_util += mu / prd;
    }
    // print cpu_util, and id2time_limit for task 3
    // printf("%02d: cpu_util_for_other_tasks=%.4f,
    // exe_time_for_task_%d=%.4f\n",0,
    //     cpu_util,task_wPerf,res_opt.id2time_limit[task_wPerf]);

    // for the rest 30*10s, print cpu_util and res_opt.id2time_limit[task_wPerf]
    // we expect res_opt.id2time_limit[task_wPerf] increase/descrease while
    // cpu_util descrease/increase
    for (int k = 1; k <= 30; k++) {
        std::string file_name = "taskset_characteristics_" + std::to_string(k);
        std::string path = GlobalVariables::PROJECT_PATH +
                           "TaskData/taskset_cfg_4_5_gen_1/" + file_name +
                           ".yaml";
        DAG_Model dag_tasks_updated = ReadDAG_Tasks(path);
        // sp_parameters = SP_Parameters(dag_tasks_updated);
        sp_parameters = ReadSP_Parameters(path);
        res_opt = EnumeratePA_with_TimeLimits(dag_tasks_updated, sp_parameters);

        cpu_util = 0.0;
        for (int i = 0; i < 4; i++) {
            if (i == task_wPerf) continue;
            const Task t = dag_tasks_updated.GetTask(i);
            int prd = t.period;
            GaussianDist g_et = t.getExecGaussian();
            double mu = g_et.mu;
            cpu_util += mu / prd;
        }
        // printf("%02d: cpu_util_for_other_tasks=%.4f,
        // exe_time_for_task_%d=%.4f\n",k,
        //     cpu_util,task_wPerf,res_opt.id2time_limit[task_wPerf]);
    }
}

int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
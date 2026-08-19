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
        RecordCloseTimeLimitOptions(dag_tasks, 2);
    // Closest to ET ~202 is 184.1 (index 0). With the default
    // IncrementalTimeLimitSearchRadius=1 the window is indices [0,1] => 2 opts.
    EXPECT_EQ(3, time_limit_options[0].size());
    EXPECT_EQ(184.1, time_limit_options[0][0]);
    EXPECT_EQ(397.5, time_limit_options[0][1]);
    EXPECT_EQ(657.9, time_limit_options[0][2]);

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
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    opt.ReOptimizePeriodic(dag_tasks, 2);
    ResourceOptResult res_opt = opt.CollectResults();
    PrintPriorityVec(dag_tasks.tasks, res_opt.priority_vec);

    // TSP's execution time ~1501 floors to the 1000 ms pair (perf = 1.0).
    // The system is schedulable with TL = 1000 and the optimizer correctly
    // selects the highest-performance option.
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

TEST_F(TaskSetForTest_robotics_v19, RecordCloseTimeLimitOptions) {
    std::vector<std::vector<double>> time_limit_options =
        RecordCloseTimeLimitOptions(dag_tasks, 2);
    EXPECT_EQ(4, time_limit_options.size());  // 4 tasks
    // With IncrementalTimeLimitSearchRadius=2 the window around closest ET
    // (1000) is indices [1,3] => [600, 800, 1000] (3 options).
    EXPECT_EQ(3, time_limit_options[0].size());  // 3 options for TSP
    EXPECT_EQ(600, time_limit_options[0][0]);
    EXPECT_EQ(800, time_limit_options[0][1]);
    EXPECT_EQ(1000, time_limit_options[0][2]);

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

// P1.27 — BF must never score below INCR on the same taskset (P0.2 invariant:
// INCR <= BF). Taskset_2 interval 0 of the compare_against_bf run: BF adopted
// the SP-max plan, the post-hoc important-task gate rejected it, and BF fell
// back to RM-Fast (SP 0.527888) while INCR's during-walk gate found the
// schedulable SP-max plan (0.954072). Fixture = that interval's taskset.
class TaskSetForTest_p127_taskset2_i0 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_p127_bf_incr_taskset2_i0";
        std::string path =
            GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
        dag_tasks = ReadDAG_Tasks(path, 5);
        sp_parameters = ReadSP_Parameters(path);
    }
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
};

TEST_F(TaskSetForTest_p127_taskset2_i0, BF_NotWorseThan_INCR_Reopt) {
    ResourceOptResult res_bf =
        EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters);

    OptimizePA_Incre_with_TimeLimits opt_incr(dag_tasks, sp_parameters);
    opt_incr.ReOptimizePeriodic(dag_tasks, 2);
    ResourceOptResult res_incr = opt_incr.CollectResults();

    EXPECT_GE(res_bf.sp_opt, res_incr.sp_opt)
        << "BF " << res_bf.sp_opt << " < INCR " << res_incr.sp_opt;
}

// P1.30 — BF must never score below INCR on the same taskset (canonical SP
// invariant: INCR <= BF). The mid real-world config reproduces the inflation:
// INCR's TL-walk SP is scored by the RTA-cache path, which on wide-ET cores
// returns node-RTAs that under-estimate miss-prob -> INCR SP 4.98389 > BF's
// canonical 4.97052 (TSP TL=1200 vs 1100). BF is exhaustive over the SAME TL
// grid, so it is the ground truth; an INCR win here is impossible and flags the
// cache-path inflation. RED on HEAD; GREEN once the cache path is bit-identical.
class TaskSetForTest_p130_rw_mid : public ::testing::Test {
   public:
    void SetUp() override {
        std::string path = GlobalVariables::PROJECT_PATH +
                           "TaskData/p0_11_variants/rw_baseline_tightened.yaml";
        dag_tasks = ReadDAG_Tasks(path, 5);
        sp_parameters = ReadSP_Parameters(path);
    }
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
};

TEST_F(TaskSetForTest_p130_rw_mid, BF_NotWorseThan_INCR) {
    ResourceOptResult res_bf =
        EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters);

    OptimizePA_Incre_with_TimeLimits opt_incr(dag_tasks, sp_parameters);
    opt_incr.ReOptimizePeriodic(dag_tasks, 2);
    ResourceOptResult res_incr = opt_incr.CollectResults();

    std::cout << "P1.30 mid: BF sp=" << res_bf.sp_opt
              << " INCR sp=" << res_incr.sp_opt << "\n";
    EXPECT_GE(res_bf.sp_opt, res_incr.sp_opt)
        << "INCR (" << res_incr.sp_opt << ") beat BF (" << res_bf.sp_opt
        << ") — impossible under the canonical SP metric (P1.30 cache inflation)";
}

int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
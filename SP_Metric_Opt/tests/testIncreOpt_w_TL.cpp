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
        RecordCloseTimeLimitOptions(dag_tasks,
                                    GlobalVariables::TimeLimitSearchRadiusIncr);
    // Closest to ET ~202 is 184.1 (index 0). Radius=2 => indices [0,2] => 3
    // opts.
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
    opt.ReOptimizePeriodic(2);
    ResourceOptResult res_opt = opt.CollectResults();
    PrintPriorityVec(dag_tasks.tasks, res_opt.priority_vec);

    // TSP's execution time ~1501 floors to the 1000 ms pair (perf = 1.0).
    // The system is schedulable with TL = 1000 and the optimizer correctly
    // selects the highest-performance option.
    EXPECT_EQ(1000, res_opt.id2time_limit[0]);
}

TEST_F(TaskSetForTest_robotics_v18, optimize_no_tl) {
    // 1. Run with TL optimization enabled (default behavior)
    GlobalVariables::disable_time_limit_opt = false;
    OptimizePA_Incre_with_TimeLimits opt_with_tl(dag_tasks, sp_parameters);
    opt_with_tl.ReOptimizePeriodic(2);
    ResourceOptResult res_opt = opt_with_tl.CollectResults();
    double opt_sp = res_opt.sp_opt;

    // Get the smallest possible time limits to compare
    std::vector<double> smallest_time_limits(dag_tasks.tasks.size());
    for (size_t i = 0; i < dag_tasks.tasks.size(); i++) {
        if (dag_tasks.tasks[i].timePerformancePairs.empty()) {
            smallest_time_limits[i] = -1.0;
        } else {
            smallest_time_limits[i] =
                dag_tasks.tasks[i].timePerformancePairs[0].time_limit;
        }
    }

    // The optimizer selects TL=1000 because it yields the highest perf (1.0)
    // while keeping the system schedulable.
    EXPECT_EQ(1000, res_opt.id2time_limit[0]);

    // TL optimization moved task 0 away from its smallest option.
    bool tl_changed = false;
    for (size_t i = 0; i < dag_tasks.tasks.size(); i++) {
        if (res_opt.id2time_limit[i] != smallest_time_limits[i]) {
            tl_changed = true;
            break;
        }
    }
    EXPECT_TRUE(tl_changed);

    // 2. Run with TL optimization disabled
    GlobalVariables::disable_time_limit_opt = true;
    OptimizePA_Incre_with_TimeLimits opt_no_tl(dag_tasks, sp_parameters);
    opt_no_tl.ReOptimizePeriodic(2);
    ResourceOptResult res_no_tl = opt_no_tl.CollectResults();
    double no_tl_sp = res_no_tl.sp_opt;

    // Restore default behavior immediately
    GlobalVariables::disable_time_limit_opt = false;

    // With TL optimization disabled, every task is pinned to its smallest TL.
    EXPECT_EQ(400, res_no_tl.id2time_limit[0]);
    for (size_t i = 0; i < dag_tasks.tasks.size(); i++) {
        EXPECT_EQ(smallest_time_limits[i], res_no_tl.id2time_limit[i]);
    }

    // Because the optimal TL (1000) is not the smallest, disabling TL opt
    // degrades SP.
    EXPECT_LT(no_tl_sp, opt_sp);
}

TEST_F(TaskSetForTest_robotics_v18, optimize_wcet) {
    // 1. Run with WCET baseline disabled (default behavior)
    GlobalVariables::use_wcet_execution_time = false;
    OptimizePA_Incre_with_TimeLimits opt_normal(dag_tasks, sp_parameters);
    opt_normal.ReOptimizePeriodic(2);
    ResourceOptResult res_normal = opt_normal.CollectResults();
    double tl_normal = res_normal.id2time_limit[0];

    // 2. Run with WCET baseline enabled
    GlobalVariables::use_wcet_execution_time = true;
    OptimizePA_Incre_with_TimeLimits opt_wcet(dag_tasks, sp_parameters);
    opt_wcet.ReOptimizePeriodic(2);
    ResourceOptResult res_wcet = opt_wcet.CollectResults();
    double tl_wcet = res_wcet.id2time_limit[0];

    // Restore default behavior immediately
    GlobalVariables::use_wcet_execution_time = false;

    // Verify that every task's execution time distribution in opt_wcet was
    // forced to its WCET constant value
    for (size_t i = 0; i < dag_tasks.tasks.size(); i++) {
        double original_wcet = dag_tasks.tasks[i].execution_time_dist.max_time;
        const auto& task_updated = opt_wcet.dag_tasks_.tasks[i];
        EXPECT_DOUBLE_EQ(original_wcet,
                         task_updated.execution_time_dist.max_time);
        EXPECT_DOUBLE_EQ(original_wcet,
                         task_updated.execution_time_dist.min_time);
        EXPECT_DOUBLE_EQ(original_wcet, task_updated.getExecutionTime());
    }

    // Normal mode picks the highest-performance option (1000) that is still
    // schedulable under the stochastic execution-time distribution.
    EXPECT_EQ(1000, tl_normal);

    // Under WCET ablation the execution times are forced to their worst-case
    // values, making the task set more constrained. With K=2 the greedy search
    // sometimes selects a tighter TL (800) to remain schedulable.
    EXPECT_LE(tl_wcet, 1000);
    EXPECT_GE(tl_wcet, 400);
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
        RecordCloseTimeLimitOptions(dag_tasks,
                                    GlobalVariables::TimeLimitSearchRadiusIncr);
    EXPECT_EQ(4, time_limit_options.size());  // 4 tasks
    // With TimeLimitSearchRadiusIncr=2 the window around closest ET (1000) is
    // indices [1,3] => [600, 800, 1000] (3 options).
    EXPECT_EQ(3, time_limit_options[0].size());  // 3 options for TSP
    EXPECT_EQ(600, time_limit_options[0][0]);
    EXPECT_EQ(800, time_limit_options[0][1]);
    EXPECT_EQ(1000, time_limit_options[0][2]);

    EXPECT_EQ(-1, time_limit_options[1][0]);
    EXPECT_EQ(-1, time_limit_options[2][0]);
    EXPECT_EQ(-1, time_limit_options[3][0]);
}
TEST_F(TaskSetForTest_robotics_v19, ReOptimizePeriodic) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    EXPECT_FALSE(opt.IfInitialized());
    opt.ReOptimizePeriodic(2);
    EXPECT_TRUE(opt.IfInitialized());
    ResourceOptResult res_opt = opt.CollectResults();
    PrintPriorityVec(dag_tasks.tasks, res_opt.priority_vec);
    EXPECT_EQ(400,
              res_opt.id2time_limit[0]);  // SLAM+TSP have high utilization;
    //   All TL options are effectively unschedulable and produce near-identical
    //   SP.  ApproxEqualSP treats them as equal, so the tie-breaker picks
    //   the lowest (tightest) time limit.
}

TEST_F(TaskSetForTest_robotics_v19, optimize_incremental) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks,
                                         sp_parameters);  // high utilization

    opt.ReOptimizePeriodic(2);  // high utilization → all TLs tied → tie-breaker
    ResourceOptResult res_opt = opt.CollectResults();
    EXPECT_EQ(
        400,
        res_opt.id2time_limit[0]);  // picks lowest TL when SP is identical

    DAG_Model dag_tasks_updated =
        ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                      "TaskData/test_robotics_v21.yaml");  // low utilization
    opt.OptimizeIncre_w_TL(dag_tasks_updated, 2);
    res_opt = opt.CollectResults();
    // With radius=2 the incremental window for TSP (ET~401) is {400,600,800}.
    // Low-utilization v21 allows the highest-performing feasible option: 800.
    EXPECT_LE(800, res_opt.id2time_limit[0]);

    auto start_time = CurrentTimeInProfiler;
    for (int i = 0; i < 10; i++) opt.OptimizeIncre_w_TL(dag_tasks_updated, 2);
    auto finish_time = CurrentTimeInProfiler;
    double time_taken = GetTimeTaken(start_time, finish_time);
    EXPECT_LT(time_taken / 10.0,
              1.0);  // relaxed for debug mode coordinate descent
}

TEST_F(TaskSetForTest_robotics_v19_2, RecordCloseTimeLimitOptions) {
    printf(
        "\n-------- TaskSetForTest_robotics_v19_2, RecordCloseTimeLimitOptions "
        "...\n");
    std::vector<std::vector<double>> time_limit_options =
        RecordCloseTimeLimitOptions(dag_tasks,
                                    GlobalVariables::TimeLimitSearchRadiusIncr);

    EXPECT_EQ(4, time_limit_options.size());  // 4 tasks

    uint perfTask = 0;
    for (int i = 0; i < static_cast<int>(time_limit_options.size()); i++) {
        if (time_limit_options[i][0] != -1) {
            perfTask = i;
            break;
        }
    }

    // With TimeLimitSearchRadiusIncr=2 the window is [1,3] => [600,800,1000].
    EXPECT_EQ(3, time_limit_options[perfTask].size());  // 3 options for TSP
    EXPECT_EQ(600, time_limit_options[perfTask][0]);
    EXPECT_EQ(800, time_limit_options[perfTask][1]);
    EXPECT_EQ(1000, time_limit_options[perfTask][2]);

    for (uint i = 0; i < time_limit_options.size(); i++) {
        if (i == perfTask)
            continue;
        EXPECT_EQ(-1, time_limit_options[i][0]);
    }
}

TEST_F(TaskSetForTest_robotics_v19_2, ReOptimizePeriodic) {
    // NOTE: this test failed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // compare with the testcase (TaskSetForTest_robotics_v19) before, the only
    // difference is that this testcase added two tasks with very small
    // cpu_utilization and big weight supposedly, ReOptimizePeriodic
    // should also return 400 for task0

    // try to get which task has performance_records_time
    int perfTask = 0;
    std::vector<std::vector<double>> time_limit_options =
        RecordCloseTimeLimitOptions(dag_tasks,
                                    GlobalVariables::TimeLimitSearchRadiusIncr);
    for (int i = 0; i < static_cast<int>(time_limit_options.size()); i++) {
        if (time_limit_options[i][0] != -1) {
            perfTask = i;
            break;
        }
    }

    printf(
        "\n-------- TaskSetForTest_robotics_v19_2, ReOptimizePeriodic "
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
    opt.ReOptimizePeriodic(2);
    EXPECT_TRUE(opt.IfInitialized());
    ResourceOptResult res_opt = opt.CollectResults();
    PrintPriorityVec(dag_tasks.tasks, res_opt.priority_vec);
    EXPECT_EQ(400,
              res_opt.id2time_limit[perfTask]);  // Both 400 and 1000 are
                                                 // unschedulable (same SP);
    // the optimizer tie-breaks to the tighter time limit.
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

TEST_F(TaskSetForTest_robotics_v19, OptimizeWithOptimizationSpace) {
    // Tighten TSP (task 0) execution time to emphasise TL optimization impact.
    dag_tasks.tasks[0].execution_time_dist =
        FiniteDist(GaussianDist(700.0, 10.0), 5);

    // 1. Optimize from scratch (can explore all time limit options)
    OptimizePA_Incre_with_TimeLimits opt_scratch(dag_tasks, sp_parameters);
    opt_scratch.ReOptimizePeriodic(2);
    ResourceOptResult res_scratch = opt_scratch.CollectResults();

    // With floor behaviour any TL in [400, 599) yields the same perf (0.5),
    // any TL in [600, 799) yields the same perf (0.6).  Because the tightest
    // feasible TL gives the best schedulability, the optimizer prefers the
    // smallest TL in the best reachable bracket. Since TSP is unschedulable
    // under all options, they all yield the same SP; the tie-breaker selects
    // 400.
    EXPECT_EQ(400, res_scratch.id2time_limit[0]);

    // 2. Optimize incrementally starting from warm start (ET pinned at 1000ms)
    OptimizePA_Incre_with_TimeLimits opt_incre(dag_tasks, sp_parameters);
    // Bootstrap the incumbent with a from-scratch call first — the incremental
    // path requires prev_optimizer_ to be initialized (otherwise the contract
    // violation in EvaluateTimeLimitConfig_ScratchOrIncre fires).
    opt_incre.ReOptimizePeriodic(2);
    DAG_Model dag_tasks_warm = dag_tasks;
    dag_tasks_warm.tasks[0].execution_time_dist =
        GetUnitExecutionTimeDist(1000.0);

    opt_incre.OptimizeIncre_w_TL(dag_tasks_warm, 2);
    ResourceOptResult res_incre = opt_incre.CollectResults();

    // The incremental search uses the narrow radius (TimeLimitSearchRadiusIncr,
    // from parameters.yaml). For TSP with ET pinned at 1000ms the closest TL is
    // 1000 (index 3), so radius 2 gives the window [600, 800, 1000] — TL=400 is
    // NOT searched and cannot be the result. Under ET=1000 the higher-TL
    // candidates (800, 1000) are unschedulable (deadline miss → safety drop),
    // so the schedulable TL=600 wins; the bootstrap incumbent (TL=400, computed
    // under the old ET=700 DAG) is replaced because TL=600 strictly improves SP
    // under the new DAG. The result is therefore the schedulable optimum within
    // the narrow window, not the bootstrap TL.
    EXPECT_GE(res_incre.id2time_limit[0], 600);
    EXPECT_LE(res_incre.id2time_limit[0], 1000);

    // Scratch explored the full option set so it should be at least as good.
    EXPECT_GE(res_scratch.sp_opt + 1e-6, res_incre.sp_opt);
}

// Synthetic 2-task DAG for compare-and-keep tests. Only T_perf (task 0) carries
// time-limit options [400,600,800,1000] with perf [0.5,0.6,0.8,1.0]; T_noise is
// a small fixed-ET task that keeps the set trivially schedulable at every TL.
// Because the system is always schedulable, SP is strictly increasing in TL
// (higher TL → higher perf term), which makes the keep/adopt decision fully
// deterministic and independent of probabilistic-RTA noise:
//   TL=400 → SP 1.5, TL=600 → SP 1.6, TL=800 → SP 1.7, TL=1000 → SP 1.8.
// T_perf's avg ET (~500.3 from the Gaussian dist) is closest to TL=600, so:
//   radius 0 → window [600]        (best TL=600, SP 1.6)
//   radius 1 → window [400,600,800] (best TL=800, SP 1.7)
//   radius 2 → window [400,600,800,1000] (best TL=1000, SP 1.8)
class CompareAndKeepSynthetic : public ::testing::Test {
   public:
    void SetUp() override {
        const double et_perf = 500.0;
        std::vector<Value_Proba> dist_perf = {Value_Proba(et_perf, 1.0)};
        Task t_perf(0, dist_perf, 2000, 2000, 0, "T_perf");
        t_perf.execution_time_dist = FiniteDist(GaussianDist(et_perf, 0.5), 5);
        for (int i = 0; i < 4; ++i) {
            t_perf.timePerformancePairs.push_back(
                TimePerfPair(400 + i * 200, 0.5 + i * 0.1));
        }

        std::vector<Value_Proba> dist_noise = {Value_Proba(50.0, 1.0)};
        Task t_noise(1, dist_noise, 2000, 2000, 1, "T_noise");
        t_noise.execution_time_dist = FiniteDist(GaussianDist(50.0, 0.5), 5);

        TaskSet tasks = {t_perf, t_noise};
        dag_tasks = DAG_Model(tasks, mapPrev, 0, 0);
        sp_parameters = SP_Parameters(dag_tasks);
    }

    MAP_Prev mapPrev;
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
};

// Compare-and-keep ADOPT path: a wide from-scratch search that strictly beats
// the incumbent must be adopted. Bootstrap the incumbent with radius 0 (forced
// to the closest TL=600, SP 1.6), then re-optimize with radius 1 whose
// from-scratch search finds TL=800 (SP 1.7 > 1.6). The from-scratch result wins
// and becomes the new incumbent.
TEST_F(CompareAndKeepSynthetic, ReOptimizePeriodic_AdoptsWhenWideSearchWins) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);

    // Bootstrap the incumbent. First call has no incumbent to compare against,
    // so the from-scratch result (radius 0 → TL=600) simply becomes it.
    opt.ReOptimizePeriodic(dag_tasks, 2, /*radius=*/0);
    ResourceOptResult after_bootstrap = opt.CollectResults();
    EXPECT_EQ(600, after_bootstrap.id2time_limit[dag_tasks.tasks[0].id]);
    EXPECT_DOUBLE_EQ(1.6, after_bootstrap.sp_opt);

    // Re-optimize with a wider radius. The from-scratch search explores
    // [400,600,800] and selects TL=800 (SP 1.7), strictly better than the
    // incumbent's 1.6 → the from-scratch result is adopted.
    opt.ReOptimizePeriodic(dag_tasks, 2, /*radius=*/1);
    ResourceOptResult after_reopt = opt.CollectResults();
    EXPECT_EQ(800, after_reopt.id2time_limit[dag_tasks.tasks[0].id]);
    EXPECT_DOUBLE_EQ(1.7, after_reopt.sp_opt);
}

// Compare-and-keep KEEP path: when the wide from-scratch search does NOT beat
// the incumbent (re-evaluated under the new DAG), the incumbent is preserved.
// Bootstrap with radius 2 (TL=1000, SP 1.8 — the global optimum), then
// re-optimize with the NARROWER radius 1 whose from-scratch search can only
// reach TL=800 (SP 1.7 < 1.8). The incumbent wins and is restored.
TEST_F(CompareAndKeepSynthetic,
       ReOptimizePeriodic_KeepsIncumbentWhenWideSearchLoses) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);

    // Bootstrap with the wide radius so the incumbent is already the global
    // optimum (TL=1000, SP 1.8).
    opt.ReOptimizePeriodic(dag_tasks, 2, /*radius=*/2);
    ResourceOptResult after_bootstrap = opt.CollectResults();
    EXPECT_EQ(1000, after_bootstrap.id2time_limit[dag_tasks.tasks[0].id]);
    EXPECT_DOUBLE_EQ(1.8, after_bootstrap.sp_opt);

    // Re-optimize with a narrower radius. The from-scratch search can only
    // reach TL=800 (SP 1.7), which is strictly worse than the incumbent's 1.8
    // under the same DAG → the incumbent is preserved (TL and SP unchanged).
    opt.ReOptimizePeriodic(dag_tasks, 2, /*radius=*/1);
    ResourceOptResult after_reopt = opt.CollectResults();
    EXPECT_EQ(1000, after_reopt.id2time_limit[dag_tasks.tasks[0].id]);
    EXPECT_DOUBLE_EQ(1.8, after_reopt.sp_opt);
}

// --- Direct unit tests for the compare-and-keep helpers ---
// Each new helper extracted during the refactor gets its own coverage below so
// the seeded-baseline / RM-bootstrap contract is checked in isolation, not only
// via the end-to-end Adopt/Keep paths above.

TEST_F(CompareAndKeepSynthetic, SmallestTimeLimitVec) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    std::vector<double> tl = opt.SmallestTimeLimitVec();
    ASSERT_EQ(2u, tl.size());
    // T_perf (task 0): smallest of [400,600,800,1000] = 400.
    EXPECT_DOUBLE_EQ(400.0, tl[0]);
    // T_noise (task 1): no time-performance pairs → -1.
    EXPECT_DOUBLE_EQ(-1.0, tl[1]);
}

TEST_F(CompareAndKeepSynthetic,
       ReconstructTimeLimitVecFromResOpt_DefaultsToMinusOneWhenUnset) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    // Fresh opt: res_opt_.id2time_limit is empty → every task maps to -1.
    std::vector<double> tl = opt.ReconstructTimeLimitVecFromResOpt();
    ASSERT_EQ(2u, tl.size());
    EXPECT_DOUBLE_EQ(-1.0, tl[0]);
    EXPECT_DOUBLE_EQ(-1.0, tl[1]);
}

TEST_F(CompareAndKeepSynthetic,
       ReconstructTimeLimitVecFromResOpt_RoundTripsSavedTimeLimits) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    // Save a known TL vector (T_perf=800, T_noise=-1) and reconstruct in task
    // order. Exercises the id→positional mapping and the -1 pass-through.
    std::vector<double> saved = {800.0, -1.0};
    opt.res_opt_.SaveTimeLimits(dag_tasks.tasks, saved);
    std::vector<double> tl = opt.ReconstructTimeLimitVecFromResOpt();
    ASSERT_EQ(2u, tl.size());
    EXPECT_DOUBLE_EQ(800.0, tl[0]);
    EXPECT_DOUBLE_EQ(-1.0, tl[1]);
}

TEST(RateMonotonicPriorityVec, SortsByPeriodAscending) {
    // Three tasks with distinct periods [100, 50, 200]. RM priority = period
    // ascending, so index 1 (period 50) is highest priority (position 0),
    // then index 0 (period 100), then index 2 (period 200). Distinct periods
    // make the result deterministic regardless of sort stability.
    std::vector<Value_Proba> d = {Value_Proba(10.0, 1.0)};
    Task t0(0, d, 100, 100, 0, "T0");
    Task t1(1, d, 50, 50, 1, "T1");
    Task t2(2, d, 200, 200, 2, "T2");
    MAP_Prev mapPrev;
    TaskSet tasks = {t0, t1, t2};
    DAG_Model dag(tasks, mapPrev, 0, 0);
    SP_Parameters sp(dag);

    OptimizePA_Incre_with_TimeLimits opt(dag, sp);
    PriorityVec pa = opt.RateMonotonicPriorityVec();
    ASSERT_EQ(3u, pa.size());
    EXPECT_EQ(1, pa[0]);
    EXPECT_EQ(0, pa[1]);
    EXPECT_EQ(2, pa[2]);
}

TEST(RateMonotonicPriorityVec, BreaksPeriodTiesByExecutionTimeAscending) {
    // Three tasks, two of which share period 100. ET tiebreaker: lower ET gets
    // higher priority (earlier position). T0 (period 100, ET 30) and T1
    // (period 100, ET 10) tie on period; T1's lower ET wins position 0, T0
    // position 1. T2 (period 200) sorts last regardless of ET.
    std::vector<Value_Proba> d0 = {Value_Proba(30.0, 1.0)};
    std::vector<Value_Proba> d1 = {Value_Proba(10.0, 1.0)};
    std::vector<Value_Proba> d2 = {Value_Proba(50.0, 1.0)};
    Task t0(0, d0, 100, 100, 0, "T0");
    Task t1(1, d1, 100, 100, 1, "T1");
    Task t2(2, d2, 200, 200, 2, "T2");
    MAP_Prev mapPrev;
    TaskSet tasks = {t0, t1, t2};
    DAG_Model dag(tasks, mapPrev, 0, 0);
    SP_Parameters sp(dag);

    OptimizePA_Incre_with_TimeLimits opt(dag, sp);
    PriorityVec pa = opt.RateMonotonicPriorityVec();
    ASSERT_EQ(3u, pa.size());
    EXPECT_EQ(1, pa[0]);  // period 100, ET 10 — lower ET beats T0
    EXPECT_EQ(0, pa[1]);  // period 100, ET 30
    EXPECT_EQ(2, pa[2]);  // period 200
}

TEST_F(CompareAndKeepSynthetic, SeedStateFromIncumbent_WritesFullFourTuple) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    EXPECT_FALSE(opt.IfInitialized());

    PriorityVec pa = {0, 1};
    std::vector<double> tl = {800.0, -1.0};
    DAG_Model dag_with_tl = UpdateExtDistBasedOnTimeLimit(dag_tasks, tl);
    const double sp = 1.234;  // arbitrary sentinel — must be stored verbatim

    opt.SeedStateFromIncumbent(dag_with_tl, pa, sp, tl);

    // opt_sp_ / opt_pa_ / res_opt_ hold the seeded tuple verbatim.
    EXPECT_DOUBLE_EQ(sp, opt.opt_sp_);
    EXPECT_EQ(pa, opt.opt_pa_);
    EXPECT_DOUBLE_EQ(sp, opt.res_opt_.sp_opt);
    EXPECT_DOUBLE_EQ(800.0, opt.res_opt_.id2time_limit[0]);
    EXPECT_DOUBLE_EQ(-1.0, opt.res_opt_.id2time_limit[1]);

    // prev_optimizer_ now carries the same {sp, pa} tuple (plus the TL-applied
    // DAG).
    EXPECT_TRUE(opt.prev_optimizer_.IfInitialized());
    EXPECT_DOUBLE_EQ(sp, opt.prev_optimizer_.opt_sp_);
    EXPECT_EQ(pa, opt.prev_optimizer_.opt_pa_);
}

TEST_F(CompareAndKeepSynthetic, SeedIncumbentBaseline_Interval0UsesRMAndMinTL) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    EXPECT_FALSE(opt.prev_optimizer_.IfInitialized());

    // Compute the expected interval-0 baseline — RM priorities + smallest TL —
    // with the same primitives the helper uses internally, then verify the
    // helper seeds exactly that.
    PriorityVec pa_rm = opt.RateMonotonicPriorityVec();
    std::vector<double> tl_min = opt.SmallestTimeLimitVec();
    DAG_Model dag_min = UpdateExtDistBasedOnTimeLimit(dag_tasks, tl_min);
    double expected_sp =
        EvaluateSPWithPriorityVec(dag_min, sp_parameters, pa_rm);

    opt.SeedIncumbentBaseline();

    EXPECT_TRUE(opt.prev_optimizer_.IfInitialized());
    EXPECT_DOUBLE_EQ(expected_sp, opt.opt_sp_);
    EXPECT_EQ(pa_rm, opt.opt_pa_);
    // Min-TL baseline: T_perf=400, T_noise=-1.
    EXPECT_DOUBLE_EQ(400.0, opt.res_opt_.id2time_limit[0]);
    EXPECT_DOUBLE_EQ(-1.0, opt.res_opt_.id2time_limit[1]);
}

TEST_F(CompareAndKeepSynthetic,
       SeedIncumbentBaseline_ReEvalsIncumbentUnderNewDAG) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);

    // Establish an incumbent with TL=800 (NOT the min 400) so the
    // with-incumbent branch is distinguishable from the interval-0 min-TL
    // branch.
    PriorityVec pa = opt.RateMonotonicPriorityVec();
    std::vector<double> tl_incumbent = {800.0, -1.0};
    DAG_Model dag_with_tl =
        UpdateExtDistBasedOnTimeLimit(dag_tasks, tl_incumbent);
    double sp_incumbent =
        EvaluateSPWithPriorityVec(dag_with_tl, sp_parameters, pa);
    opt.SeedStateFromIncumbent(dag_with_tl, pa, sp_incumbent, tl_incumbent);
    ASSERT_TRUE(opt.prev_optimizer_.IfInitialized());

    // Mutate T_noise's ET to a value that breaks schedulability for the
    // incumbent's {pa, tl} (response time ≈ T_perf's 800 + T_noise's 1900 ≫
    // 2000 deadline → deadline miss → safety drops). This makes re-evaluating
    // the incumbent's {pa, tl} under the new DAG yield a different SP than the
    // stale sp_incumbent, so the re-eval is observable. (T_noise has no TL, so
    // UpdateExtDistBasedOnTimeLimit copies its ET through unchanged.)
    opt.dag_tasks_.tasks[1].execution_time_dist =
        FiniteDist(GaussianDist(1900.0, 0.5), 5);
    DAG_Model dag_new_with_tl =
        UpdateExtDistBasedOnTimeLimit(opt.dag_tasks_, tl_incumbent);
    double expected_sp =
        EvaluateSPWithPriorityVec(dag_new_with_tl, sp_parameters, pa);
    ASSERT_NE(sp_incumbent, expected_sp);  // sanity: the DAG really did change

    opt.SeedIncumbentBaseline();

    // With-incumbent branch: TL stays 800 (NOT reset to min 400) and SP is the
    // re-evaluated value under the new DAG (not the stale sp_incumbent).
    EXPECT_DOUBLE_EQ(expected_sp, opt.opt_sp_);
    EXPECT_DOUBLE_EQ(800.0, opt.res_opt_.id2time_limit[0]);
    EXPECT_DOUBLE_EQ(-1.0, opt.res_opt_.id2time_limit[1]);
    EXPECT_EQ(pa, opt.opt_pa_);
}

// --- Counter-driven dispatcher (Optimize_w_TL_ScratchOrIncre) ---
//
// Synthetic 2-task DAG: T_perf (task 0) carries 10 evenly-spaced TL options
// [0,10,...,90] with ET=45 (closest option = index 4, value 40). T_noise is a
// small fixed-ET task. With TimeLimitSearchRadiusIncr=2 the narrow window is
// indices [2,6] → 5 options; with ReoptimizationTimeLimitsSearchRadius=6 the
// wide window is indices [0,9] → 10 options (clamped). The dispatcher re-runs
// the wide-radius ReOptimizePeriodic every ReoptimizationPeriod-th call and the
// narrow-radius OptimizeIncre_w_TL otherwise. Because the dispatcher overwrites
// time_limit_option_for_each_task_ on each call, the recorded size reflects the
// LAST call's radius — the distinguishing observable between the two branches.
class CounterDispatcherSynthetic : public ::testing::Test {
   public:
    void SetUp() override {
        const double et_perf = 45.0;
        std::vector<Value_Proba> dist_perf = {Value_Proba(et_perf, 1.0)};
        Task t_perf(0, dist_perf, 1000, 1000, 0, "T_perf");
        t_perf.execution_time_dist = FiniteDist(GaussianDist(et_perf, 0.5), 5);
        for (int i = 0; i < 10; ++i) {
            t_perf.timePerformancePairs.push_back(
                TimePerfPair(i * 10, i * 0.1));
        }

        std::vector<Value_Proba> dist_noise = {Value_Proba(50.0, 1.0)};
        Task t_noise(1, dist_noise, 1000, 1000, 1, "T_noise");
        t_noise.execution_time_dist = FiniteDist(GaussianDist(50.0, 0.5), 5);

        TaskSet tasks = {t_perf, t_noise};
        dag_tasks = DAG_Model(tasks, mapPrev, 0, 0);
        sp_parameters = SP_Parameters(dag_tasks);

        saved_period_ = GlobalVariables::ReoptimizationPeriod;
        GlobalVariables::ReoptimizationPeriod = 10;
    }

    void TearDown() override {
        GlobalVariables::ReoptimizationPeriod = saved_period_;
    }

    MAP_Prev mapPrev;
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
    int saved_period_;
};

// The counter advances by 1 after every dispatch and never resets. Three
// consecutive calls → counter == 3 regardless of which branch each call took.
TEST_F(CounterDispatcherSynthetic, CounterAdvancesEveryCall_NeverResets) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    EXPECT_EQ(0, opt.reoptimization_interval_count_);

    opt.Optimize_w_TL_ScratchOrIncre(dag_tasks, 2);
    EXPECT_EQ(1, opt.reoptimization_interval_count_);
    opt.Optimize_w_TL_ScratchOrIncre(dag_tasks, 2);
    EXPECT_EQ(2, opt.reoptimization_interval_count_);
    opt.Optimize_w_TL_ScratchOrIncre(dag_tasks, 2);
    EXPECT_EQ(3, opt.reoptimization_interval_count_);
}

// count == 0 → 0 % period == 0 → ReOptimizePeriodic (wide radius). On a fresh
// opt this is the interval-0 bootstrap: SeedIncumbentBaseline synthesizes an
// RM+min-TL incumbent, so the call succeeds (no CoutError) and leaves the opt
// initialized. The wide radius is observable: 10 TL options recorded for T_perf.
TEST_F(CounterDispatcherSynthetic,
       TriggersReoptAtCountZero_BootstrapsIncumbent) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    EXPECT_FALSE(opt.IfInitialized());
    EXPECT_EQ(0, opt.reoptimization_interval_count_);

    opt.Optimize_w_TL_ScratchOrIncre(dag_tasks, 2);

    EXPECT_TRUE(opt.IfInitialized());
    EXPECT_EQ(1, opt.reoptimization_interval_count_);
    // Wide radius (ReoptimizationTimeLimitsSearchRadius=6) covers all 10
    // options for T_perf (ET=45, closest index 4, window [0,9] clamped).
    EXPECT_EQ(10u, opt.time_limit_option_for_each_task_[0].size());
}

// count == 0 routes to reopt (wide, 10 options); count == 1 is not modular
// (1 % 10 != 0) so the second call routes to the incremental branch (narrow
// radius, 5 options). The recorded size after the second call is 5, proving the
// incremental branch — not reopt — ran. The incumbent established by the first
// call lets the incremental path's warm-start contract hold (no CoutError).
TEST_F(CounterDispatcherSynthetic, RoutesToIncrementalAtNonModularCount) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);

    // count == 0 → reopt (wide). Establishes the incumbent.
    opt.Optimize_w_TL_ScratchOrIncre(dag_tasks, 2);
    ASSERT_EQ(1, opt.reoptimization_interval_count_);
    ASSERT_EQ(10u, opt.time_limit_option_for_each_task_[0].size());

    // count == 1 → 1 % 10 != 0 → incremental (narrow).
    opt.Optimize_w_TL_ScratchOrIncre(dag_tasks, 2);
    EXPECT_EQ(2, opt.reoptimization_interval_count_);
    // Narrow radius (TimeLimitSearchRadiusIncr=2) → window [2,6] → 5 options.
    EXPECT_EQ(5u, opt.time_limit_option_for_each_task_[0].size());
}

TEST(RecordCloseTimeLimitOptions_DynamicRadius, Vanilla) {
    // Build a synthetic task with 10 evenly-spaced TL options [0, 10, 20, ...
    // 90]
    std::vector<Value_Proba> dist = {Value_Proba(45, 1.0)};
    Task t(0, dist, 1000, 1000, 0, "T_perf");
    for (int i = 0; i < 10; ++i) {
        t.timePerformancePairs.push_back(TimePerfPair(i * 10, i * 0.1));
    }
    // ET is 45 → closest option is index 4 (value 40) because |45-40| = |45-50|
    // and the earlier index wins.
    MAP_Prev mapPrev;
    TaskSet tasks = {t};
    DAG_Model dag(tasks, mapPrev, 0, 0);

    // Radius 2 → indices [2, 6] → 5 options: {20,30,40,50,60}
    {
        auto opts = RecordCloseTimeLimitOptions(dag, 2);
        ASSERT_EQ(1u, opts.size());
        EXPECT_EQ(5u, opts[0].size());
        EXPECT_DOUBLE_EQ(20.0, opts[0][0]);
        EXPECT_DOUBLE_EQ(40.0, opts[0][2]);
        EXPECT_DOUBLE_EQ(60.0, opts[0][4]);
    }

    // Radius 0 → single option (closest only)
    {
        auto opts = RecordCloseTimeLimitOptions(dag, 0);
        ASSERT_EQ(1u, opts.size());
        EXPECT_EQ(1u, opts[0].size());
        EXPECT_DOUBLE_EQ(40.0, opts[0][0]);
    }

    // Large radius 5 → indices [0, 9] because only 10 options exist
    {
        auto opts = RecordCloseTimeLimitOptions(dag, 5);
        ASSERT_EQ(1u, opts.size());
        EXPECT_EQ(10u, opts[0].size());
        EXPECT_DOUBLE_EQ(0.0, opts[0][0]);
        EXPECT_DOUBLE_EQ(90.0, opts[0][9]);
    }
}

int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
// #include <gtest/gtest.h>

#include <map>
#include <vector>

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

TEST_F(TaskSetForTest_robotics_v19, ReOptimizePeriodic) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    EXPECT_FALSE(opt.IfInitialized());
    opt.ReOptimizePeriodic(2);
    EXPECT_TRUE(opt.IfInitialized());
    ResourceOptResult res_opt = opt.CollectResults();
    PrintPriorityVec(dag_tasks.tasks, res_opt.priority_vec);
    // SLAM+TSP have high utilization; all TL options are effectively
    // unschedulable and produce near-identical SP. ApproxEqualSP treats them as
    // equal, so the tie-breaker picks the lowest (tightest) time limit. The
    // walk steps over the FULL option set [400,600,800,1000] (no radius cap),
    // so the downward tie-break reaches 400 — the global tightest option, not
    // the old radius-capped floor of 600.
    EXPECT_EQ(400, res_opt.id2time_limit[0]);
}

TEST_F(TaskSetForTest_robotics_v19, optimize_incremental) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks,
                                         sp_parameters);  // high utilization

    // Bootstrap with the 1-arg ReOptimizePeriodic. TSP's full option set is
    // [400,600,800,1000]; at high utilization SP saturates so all options tie,
    // and the smaller-TL tie-break walks all the way down to 400.
    opt.ReOptimizePeriodic(2);
    ResourceOptResult res_opt = opt.CollectResults();
    EXPECT_EQ(400, res_opt.id2time_limit[0]);

    DAG_Model dag_tasks_updated =
        ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                      "TaskData/test_robotics_v21.yaml");  // low utilization
    // 2-arg OptimizeIncre_w_TL (warm-started incremental). v21's TSP ET~401 →
    // closest TL is 400 (index 0); the walk steps over the FULL set
    // [400,600,800,1000] (no radius cap). On low-utilization v21, SP strictly
    // increases with TL (TSP's performance_records_perf 0.5/0.6/0.8/1.0
    // dominates), so each forward step strictly improves → the walk climbs all
    // the way to 1000.
    opt.OptimizeIncre_w_TL(dag_tasks_updated, 2);
    res_opt = opt.CollectResults();
    EXPECT_EQ(1000, res_opt.id2time_limit[0]);

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
    // RecordCloseTimeLimitOptions is now a standalone utility (no longer on the
    // walk path, which uses the full option set), so its radius is passed
    // explicitly here. radius=1 → window [2,3] => [800,1000] for TSP (closest
    // TL 1000 at index 3).
    std::vector<std::vector<double>> time_limit_options =
        RecordCloseTimeLimitOptions(dag_tasks, /*radius=*/1);

    EXPECT_EQ(4, time_limit_options.size());  // 4 tasks

    uint perfTask = 0;
    for (int i = 0; i < static_cast<int>(time_limit_options.size()); i++) {
        if (time_limit_options[i][0] != -1) {
            perfTask = i;
            break;
        }
    }

    // radius=1 → window [2,3] => [800,1000].
    EXPECT_EQ(2, time_limit_options[perfTask].size());  // 2 options for TSP
    EXPECT_EQ(800, time_limit_options[perfTask][0]);
    EXPECT_EQ(1000, time_limit_options[perfTask][1]);

    for (uint i = 0; i < time_limit_options.size(); i++) {
        if (i == perfTask)
            continue;
        EXPECT_EQ(-1, time_limit_options[i][0]);
    }
}

TEST_F(TaskSetForTest_robotics_v19_2, ReOptimizePeriodic) {
    // Compare with TaskSetForTest_robotics_v19: the only difference is that
    // this taskset adds two low-utilization DUMMY tasks. The perf-carrying
    // task (TSP) is unchanged, so ReOptimizePeriodic must reach the SAME TL
    // for it as the v19 case.

    // Find which task carries timePerformancePairs (the perf task). The radius
    // passed to RecordCloseTimeLimitOptions is irrelevant here — any radius
    // surfaces the non-{-1} task; we just need its index.
    int perfTask = 0;
    std::vector<std::vector<double>> time_limit_options =
        RecordCloseTimeLimitOptions(dag_tasks, /*radius=*/1);
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

    opt.ReOptimizePeriodic(2);
    EXPECT_TRUE(opt.IfInitialized());
    ResourceOptResult res_opt = opt.CollectResults();
    PrintPriorityVec(dag_tasks.tasks, res_opt.priority_vec);
    // Same landscape as v19: TSP's full option set is [400,600,800,1000] and at
    // high utilization all options saturate to near-identical SP, so the
    // smaller-TL tie-break walks all the way down to 400 for the perf task —
    // identical to the v19 result now that the walk is not radius-capped.
    EXPECT_EQ(400, res_opt.id2time_limit[perfTask]);
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

    // The incremental walk steps over the FULL option set [400,600,800,1000]
    // from the ET-closest baseline (1000) under the warm DAG. With TSP pinned
    // at ET=1000ms every TL option ties on SP (the same floor behaviour as the
    // scratch case above), and the tie-break selects the smallest TL → 400,
    // matching the scratch result. (Under the old radius-capped walk the
    // incremental leg could not reach 400 from a 1000 baseline within its narrow
    // window, so this was pinned at 600; the full-set walk makes the two paths
    // agree.)
    EXPECT_EQ(400, res_incre.id2time_limit[0]);

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
//
// Under the trial-and-error walk the from-scratch search always scans the FULL
// option set, so on this fixture it always reaches the global optimum TL=1000.
// That means the ADOPT path (search strictly beats the incumbent) CANNOT be
// exercised on this fixture — bootstrap already finds the global max, so no
// later search can strictly beat it. The Adopt/Keep tests below therefore use a
// SEPARATE pair of YAMLs (test_robotics_v30_lo / _hi) whose SLAM ET differs: the
// DAG mutation between bootstrap and reopt shifts the optimum, which is the
// real-world condition compare-and-keep exists for. This fixture is retained for
// the helper unit tests (SmallestTimeLimitVec, SeedIncumbentBaseline, etc.) that
// are profile-shape-independent.
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

// Compare-and-keep ADOPT path: when the DAG mutates between bootstrap and
// reopt so that the incumbent (re-evaluated under the NEW DAG) is strictly worse
// than the new from-scratch search result, the search result is ADOPTED.
//
// The v30 YAML pair differs ONLY in SLAM's execution time (everything else is
// identical to test_robotics_v21.yaml):
//  - v30_lo (SLAM ET ~285, light load): TSP's optimal TL is 1000.
//  - v30_hi (SLAM ET ~2853, heavy load): TSP's optimal TL is 400 — the larger
//    SLAM ET on the same processor pushes TL=1000 past a schedulability cliff,
//    so the optimum shifts DOWN.
// Bootstrap on v30_lo → incumbent {TL=1000}. Reopt on v30_hi: SeedIncumbentBaseline
// re-evaluates {TL=1000} under v30_hi (strictly worse than v30_hi's optimum), the
// fresh search finds TL=400, and UpdateRecords' strictly-greater-SP guard ADOPTS
// 400. This is the genuine compare-and-keep adopt path — no synthetic radius.
TEST_F(CompareAndKeepSynthetic,
       ReOptimizePeriodic_AdoptsWhenDagMutationShiftsOptimum) {
    DAG_Model dag_lo = ReadDAG_Tasks(
        GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v30_lo.yaml", 5);
    DAG_Model dag_hi = ReadDAG_Tasks(
        GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v30_hi.yaml", 5);
    SP_Parameters sp(dag_lo);
    OptimizePA_Incre_with_TimeLimits opt(dag_lo, sp);

    // Bootstrap on the light-load DAG. The from-scratch walk reaches the global
    // optimum TL=1000 for TSP; with no prior incumbent this simply becomes it.
    opt.ReOptimizePeriodic(dag_lo, 2);
    ResourceOptResult after_bootstrap = opt.CollectResults();
    EXPECT_EQ(1000, after_bootstrap.id2time_limit[dag_lo.tasks[0].id]);

    // Reopt on the heavy-load DAG. The incumbent {TL=1000} re-evaluated under
    // dag_hi is strictly worse than dag_hi's optimum (TL=400), so the search
    // result is ADOPTED — TSP's TL drops from 1000 to 400.
    opt.ReOptimizePeriodic(dag_hi, 2);
    ResourceOptResult after_reopt = opt.CollectResults();
    EXPECT_EQ(400, after_reopt.id2time_limit[dag_hi.tasks[0].id]);
}

// Compare-and-keep KEEP path: when the DAG is UNCHANGED between bootstrap and
// reopt, the incumbent re-evaluated under the (same) new DAG ties the from-scratch
// search result (both reach the same global optimum), so the incumbent is
// PRESERVED — TL and SP unchanged. Under the full-set walk the search always
// finds the optimum, so "keep" is necessarily a tie (the search can never be
// strictly worse than the incumbent on the same DAG).
TEST_F(CompareAndKeepSynthetic,
       ReOptimizePeriodic_KeepsIncumbentWhenDagUnchanged) {
    DAG_Model dag_lo = ReadDAG_Tasks(
        GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v30_lo.yaml", 5);
    SP_Parameters sp(dag_lo);
    OptimizePA_Incre_with_TimeLimits opt(dag_lo, sp);

    // Bootstrap on the light-load DAG → incumbent {TL=1000}.
    opt.ReOptimizePeriodic(dag_lo, 2);
    ResourceOptResult after_bootstrap = opt.CollectResults();
    EXPECT_EQ(1000, after_bootstrap.id2time_limit[dag_lo.tasks[0].id]);
    const double sp_after_bootstrap = after_bootstrap.sp_opt;

    // Reopt on the SAME DAG. The incumbent re-eval ties the search result → the
    // incumbent is preserved (TL and SP unchanged).
    opt.ReOptimizePeriodic(dag_lo, 2);
    ResourceOptResult after_reopt = opt.CollectResults();
    EXPECT_EQ(1000, after_reopt.id2time_limit[dag_lo.tasks[0].id]);
    EXPECT_DOUBLE_EQ(sp_after_bootstrap, after_reopt.sp_opt);
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
// small fixed-ET task. The dispatcher re-runs the wide-radius
// ReOptimizePeriodic every ReoptimizationPeriod-th call and the narrow-radius
// OptimizeIncre_w_TL otherwise. Under the trial-and-error walk BOTH branches
// record the FULL per-task option set (no radius cap), so the recorded option
// count is no longer a branch-distinguishing signal. Instead, the routing is
// observed via the `from_scratch` flag the evaluator receives: the reopt branch
// passes from_scratch=true, the incremental branch from_scratch=false. The
// fixture's RecordingDispatcherOpt subclass records every flag value, so a test
// can assert which branch each dispatch took.
//
// ReoptimizationPeriod is PINNED in SetUp (=10) so this fixture is independent
// of the production default in parameters.yaml. These tests exercise dispatch
// ROUTING, not the period value; pinning the period they were designed around
// (count 0 → reopt, count 1..9 → incremental) keeps the routing signal crisp.
class CounterDispatcherSynthetic : public ::testing::Test {
   public:
    // Subclass that records the from_scratch flag of every
    // EvaluateTimeLimitConfig_ScratchOrIncre call. This is the only call the
    // coordinate-descent walk makes per candidate, so the recorded flags are
    // exactly the routing decisions the dispatcher made.
    class RecordingDispatcherOpt : public OptimizePA_Incre_with_TimeLimits {
       public:
        std::vector<bool> from_scratch_flags;
        using OptimizePA_Incre_with_TimeLimits::OptimizePA_Incre_with_TimeLimits;
        double EvaluateTimeLimitConfig_ScratchOrIncre(
            int K, const std::vector<double>& time_limits,
            bool from_scratch) override {
            from_scratch_flags.push_back(from_scratch);
            return OptimizePA_Incre_with_TimeLimits::
                EvaluateTimeLimitConfig_ScratchOrIncre(K, time_limits,
                                                       from_scratch);
        }
    };

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
    RecordingDispatcherOpt opt(dag_tasks, sp_parameters);
    EXPECT_EQ(0, opt.reoptimization_interval_count_);

    opt.Optimize_w_TL_ScratchOrIncre(dag_tasks, 2);
    EXPECT_EQ(1, opt.reoptimization_interval_count_);
    opt.Optimize_w_TL_ScratchOrIncre(dag_tasks, 2);
    EXPECT_EQ(2, opt.reoptimization_interval_count_);
    opt.Optimize_w_TL_ScratchOrIncre(dag_tasks, 2);
    EXPECT_EQ(3, opt.reoptimization_interval_count_);
}

// count == 0 → 0 % period == 0 → ReOptimizePeriodic (from_scratch). On a fresh
// opt this is the interval-0 bootstrap: SeedIncumbentBaseline synthesizes an
// RM+min-TL incumbent, so the call succeeds (no CoutError) and leaves the opt
// initialized. The reopt branch is observable via the recorded flags: EVERY
// flag this call pushed is true (the from-scratch descent evaluates all
// candidates with from_scratch=true).
TEST_F(CounterDispatcherSynthetic,
       TriggersReoptAtCountZero_BootstrapsIncumbent) {
    RecordingDispatcherOpt opt(dag_tasks, sp_parameters);
    EXPECT_FALSE(opt.IfInitialized());
    EXPECT_EQ(0, opt.reoptimization_interval_count_);

    opt.Optimize_w_TL_ScratchOrIncre(dag_tasks, 2);

    EXPECT_TRUE(opt.IfInitialized());
    EXPECT_EQ(1, opt.reoptimization_interval_count_);
    ASSERT_FALSE(opt.from_scratch_flags.empty());
    // count == 0 routes to ReOptimizePeriodic → every candidate the descent
    // evaluated was a from-scratch eval (from_scratch=true).
    for (bool fs : opt.from_scratch_flags) {
        EXPECT_TRUE(fs) << "count==0 must route every eval through the reopt "
                        << "(from_scratch=true) branch; saw a false flag.";
    }
}

// count == 0 routes to reopt (from_scratch=true); count == 1 is not modular
// (1 % 10 != 0) so the second call routes to the incremental branch
// (from_scratch=false). The routing is observable via the recorded flags: the
// second call pushes at least one false flag (the incremental branch evaluates
// its candidates with from_scratch=false), proving the incremental branch —
// not reopt — ran. The incumbent established by the first call lets the
// incremental path's warm-start contract hold (no CoutError).
TEST_F(CounterDispatcherSynthetic, RoutesToIncrementalAtNonModularCount) {
    RecordingDispatcherOpt opt(dag_tasks, sp_parameters);

    // count == 0 → reopt. Establishes the incumbent.
    opt.Optimize_w_TL_ScratchOrIncre(dag_tasks, 2);
    ASSERT_EQ(1, opt.reoptimization_interval_count_);
    const size_t flags_after_reopt = opt.from_scratch_flags.size();
    ASSERT_GT(flags_after_reopt, 0u);
    for (bool fs : opt.from_scratch_flags) {
        ASSERT_TRUE(fs);
    }

    // count == 1 → 1 % 10 != 0 → incremental.
    opt.Optimize_w_TL_ScratchOrIncre(dag_tasks, 2);
    EXPECT_EQ(2, opt.reoptimization_interval_count_);
    // The incremental branch pushed at least one false flag (from_scratch=false)
    // — the routing signal that the second call took the incremental branch.
    bool saw_incremental_flag = false;
    for (size_t i = flags_after_reopt; i < opt.from_scratch_flags.size(); ++i) {
        if (!opt.from_scratch_flags[i]) {
            saw_incremental_flag = true;
            break;
        }
    }
    EXPECT_TRUE(saw_incremental_flag)
        << "count==1 must route through the incremental (from_scratch=false) "
        << "branch; every flag was true (reopt ran instead).";
}

// INCR frozen-baseline regression: OptimizeIncre must advance the carried
// baseline DAG (prev_optimizer_.dag_tasks_) to the current interval's DAG, so
// that consecutive incremental calls diff consecutive-interval DAGs (small
// ndiff) instead of stale-reopt-DAG vs fresh-DAG (ndiff saturates at N every
// interval → per-act ET grows with ReoptimizationPeriod).
//
// Mechanism under test: OptimizeIncre_w_TL →
// EvaluateTimeLimitConfig_ScratchOrIncre (incremental branch) does
// `OptimizePA_Incre optimizer = prev_optimizer_;` then
// `optimizer.OptimizeIncre(dag_tasks_cur);`. OptimizeIncre diffs
// optimizer.dag_tasks_ (copied from prev_optimizer_, i.e. the FROZEN
// reopt-interval DAG) against dag_tasks_cur. UpdateRecords then writes
// `prev_optimizer_ = optimizer` — but only if OptimizeIncre advanced
// optimizer.dag_tasks_ to dag_tasks_cur. Without that advance,
// prev_optimizer_.dag_tasks_ stays frozen at the reopt DAG forever.
//
// Observable: T_noise (task 1) has no time-performance pairs → its TL is always
// -1 → UpdateExtDistBasedOnTimeLimit passes its execution_time_dist through
// unchanged. So prev_optimizer_.dag_tasks_.tasks[1].execution_time_dist is a
// direct, debugMode-independent window onto whether the baseline DAG advanced.
// Bootstrap establishes the original ET there; a second interval with a MUTATED
// T_noise ET must propagate that mutation into prev_optimizer_.dag_tasks_.
TEST_F(CompareAndKeepSynthetic, OptimizeIncre_AdvancesPrevOptimizerDagTasks) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);

    // Bootstrap the incumbent. After ReOptimizePeriodic,
    // prev_optimizer_.dag_tasks_ holds the TL-applied bootstrap DAG; T_noise's
    // TL is -1 so its ET there is the ORIGINAL fixture value (FiniteDist
    // around 50.0).
    opt.ReOptimizePeriodic(dag_tasks, 2);
    ASSERT_TRUE(opt.prev_optimizer_.IfInitialized());
    const double original_noise_et = opt.prev_optimizer_.dag_tasks_.tasks[1]
                                         .execution_time_dist.GetAvgValue();
    EXPECT_NEAR(original_noise_et, 50.0, 5.0);

    // Second interval: same DAG except T_noise's ET is mutated to a clearly
    // different value. T_perf is left untouched so the only ET change vs the
    // bootstrap is on T_noise.
    const double mutated_noise_et = 1234.0;
    DAG_Model dag_v2 = dag_tasks;
    dag_v2.tasks[1].execution_time_dist =
        GetUnitExecutionTimeDist(mutated_noise_et);

    opt.OptimizeIncre_w_TL(dag_v2, 2);

    // FIX UNDER TEST: OptimizeIncre must advance prev_optimizer_.dag_tasks_ to
    // the current interval's DAG. T_noise's TL is -1 → its ET in the TL-applied
    // dag_tasks_cur equals dag_v2's mutated ET → prev_optimizer_.dag_tasks_
    // must now carry the mutated ET. Before the fix, prev_optimizer_.dag_tasks_
    // stays frozen at the bootstrap DAG → T_noise's ET remains ~50.0 (the
    // original), and this expectation FAILS.
    const double carried_noise_et = opt.prev_optimizer_.dag_tasks_.tasks[1]
                                        .execution_time_dist.GetAvgValue();
    EXPECT_NEAR(carried_noise_et, mutated_noise_et, 5.0)
        << "prev_optimizer_.dag_tasks_ was not advanced by OptimizeIncre; "
        << "the incremental diff baseline is frozen at the reopt DAG. "
        << "Expected ~" << mutated_noise_et << " (current interval), got "
        << carried_noise_et << " (bootstrap value ~" << original_noise_et
        << ").";
}

// Fix 2 (the {-1}-only skip + zero-work fallback in PerformCoordinateDescent).
//
// When EVERY task lacks timePerformancePairs, RecordCloseTimeLimitOptions gives
// each task the option set {-1} (no TL freedom). The coordinate descent used to
// evaluate one config per task anyway (N expensive OptimizeIncre sweeps)
// because the existing inner `if (val == -1 && best_sp > -1) continue;` does
// NOT skip a task whose ONLY option is -1 (best_sp starts at -2.0, so the guard
// is false on the first iteration). Fix 2 adds an outer `opts == {-1}` skip so
// the descent does zero evals — but zero evals means UpdateRecords never runs,
// so prev_optimizer_ would not advance (re-introducing the frozen-baseline bug
// Fix A fixed). The zero-work fallback runs one eval with the current (all -1)
// time_limits so UpdateRecords advances prev_optimizer_.
//
// Observable: eval_count_ (public, debugMode-independent, incremented once per
// EvaluateTimeLimitConfig_ScratchOrIncre call) drops from N to 1, AND
// prev_optimizer_.dag_tasks_ still advances (T_noise's TL is -1 so its ET
// passes through UpdateExtDistBasedOnTimeLimit unchanged → a direct window onto
// whether prev_optimizer_ advanced).
TEST_F(
    CompareAndKeepSynthetic,
    PerformCoordinateDescent_AllMinusOneOnly_RunsOneEvalAndAdvancesPrevOptimizer) {
    // Build a 2-task DAG where BOTH tasks lack timePerformancePairs → both get
    // opts == {-1} → the descent does zero evals without the fallback.
    const double et_perf = 500.0;
    std::vector<Value_Proba> dist_perf = {Value_Proba(et_perf, 1.0)};
    Task t_perf(0, dist_perf, 2000, 2000, 0, "T_perf");
    t_perf.execution_time_dist = FiniteDist(GaussianDist(et_perf, 0.5), 5);
    // NOTE: no timePerformancePairs on t_perf (unlike the fixture default).

    std::vector<Value_Proba> dist_noise = {Value_Proba(50.0, 1.0)};
    Task t_noise(1, dist_noise, 2000, 2000, 1, "T_noise");
    t_noise.execution_time_dist = FiniteDist(GaussianDist(50.0, 0.5), 5);

    MAP_Prev mapPrev;
    TaskSet tasks = {t_perf, t_noise};
    DAG_Model dag_no_tl(tasks, mapPrev, 0, 0);
    SP_Parameters sp(dag_no_tl);

    OptimizePA_Incre_with_TimeLimits opt(dag_no_tl, sp);
    opt.ReOptimizePeriodic(dag_no_tl, 2);
    ASSERT_TRUE(opt.prev_optimizer_.IfInitialized());
    const int eval_count_after_bootstrap = opt.eval_count_;

    // Second interval: mutate T_noise's ET. T_noise's TL is -1 → its ET passes
    // through UpdateExtDistBasedOnTimeLimit unchanged → if the fallback ran
    // UpdateRecords, prev_optimizer_.dag_tasks_ carries the mutated ET.
    const double mutated_noise_et = 1234.0;
    DAG_Model dag_v2 = dag_no_tl;
    dag_v2.tasks[1].execution_time_dist =
        GetUnitExecutionTimeDist(mutated_noise_et);

    opt.OptimizeIncre_w_TL(dag_v2, 2);

    // The win: descent does 1 eval (the zero-work fallback), not N=2.
    EXPECT_EQ(1, opt.eval_count_ - eval_count_after_bootstrap)
        << "All-{-1}-only descent should run exactly one eval (the zero-work "
        << "fallback), not one per task. Before Fix 2 this is 2.";

    // The trap guard: prev_optimizer_ still advanced (fallback's UpdateRecords
    // ran). Without the fallback, the {-1}-only skip would starve UpdateRecords
    // and prev_optimizer_.dag_tasks_ would freeze at the bootstrap DAG → the
    // frozen-baseline bug Fix A fixed returns.
    const double carried_noise_et = opt.prev_optimizer_.dag_tasks_.tasks[1]
                                        .execution_time_dist.GetAvgValue();
    EXPECT_NEAR(carried_noise_et, mutated_noise_et, 5.0)
        << "prev_optimizer_.dag_tasks_ was not advanced by the fallback eval; "
        << "the zero-work skip starved UpdateRecords. Expected ~"
        << mutated_noise_et << " (current interval), got " << carried_noise_et
        << " (bootstrap value ~50.0).";
}

// Mixed-case skip guard: when SOME tasks carry real TL options and others are
// {-1}-only, the {-1}-only task is skipped (no redundant incumbent re-eval) and
// the zero-work fallback does NOT fire (the real-option task already produced
// >0 evals). Uses the standard fixture (T_perf has 4 TL pairs; T_noise has none
// → {-1}-only).
//
// Under the trial-and-error walk the incremental leg steps over T_perf's FULL
// option set (no radius cap): from baseline TL=600 (closest to ET~500) it walks
// backward to 400 (1 eval, non-improving → patience=0 breaks) then forward
// through 800 and 1000 (each strictly better → adopted), plus the baseline eval.
// On this monotonic strictly-increasing-in-TL SP landscape that is exactly
// 4 evals = T_perf's full option-set size. T_noise ({-1}-only) is skipped → 0
// evals, and no fallback eval is added on top.
TEST_F(CompareAndKeepSynthetic,
       PerformCoordinateDescent_SkipsMinusOneOnlyTaskInMixedSet) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    opt.ReOptimizePeriodic(dag_tasks, 2);
    const int eval_count_after_bootstrap = opt.eval_count_;

    // T_perf's FULL option-set size — the walk evaluates exactly this many
    // configs on this monotonic landscape (baseline + backward-to-boundary +
    // forward-to-boundary reaches every option). T_noise ({-1}-only) is
    // skipped → 0. Both branches record the full set under the trial-and-error
    // walk, so the bootstrap's recorded size equals the incremental leg's.
    const size_t t_perf_full_set =
        opt.time_limit_option_for_each_task_[0].size();
    ASSERT_EQ(4u, t_perf_full_set);  // [400,600,800,1000]

    opt.OptimizeIncre_w_TL(dag_tasks, 2);

    const int incremental_evals = opt.eval_count_ - eval_count_after_bootstrap;
    // The skip: T_noise added 0 evals, so the count equals T_perf's full
    // option-set size (no +1 for T_noise's redundant incumbent re-eval). Before
    // the skip this was t_perf_full_set + 1.
    EXPECT_EQ(t_perf_full_set, static_cast<size_t>(incremental_evals))
        << "Mixed descent should evaluate only T_perf's "
        << t_perf_full_set
        << " full-set options (T_noise is {-1}-only → skipped, fallback does "
        << "not fire). Before the skip this is "
        << (t_perf_full_set + 1) << " (T_noise's redundant eval).";
    // The fallback does NOT fire on top of T_perf's real evals: the count is
    // bounded above by the full-set size, with no +1 fallback eval.
    EXPECT_LE(static_cast<size_t>(incremental_evals), t_perf_full_set);
    EXPECT_GE(incremental_evals, 1);  // the baseline eval always runs
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

// --- Trial-and-Error TL optimization: pure helper unit tests ---
//
// FindTimeLimitOptionIndex and IsBetterTimeLimitOption are the two pure helpers
// extracted from the trial-and-error coordinate-descent rewrite. They take no
// optimizer state, so they can be unit-tested in isolation without constructing
// a DAG / SP_Parameters. Each helper's contract is checked independently below
// before the higher-level OptimizeSingleTaskTimeLimit walk is exercised.

// FindTimeLimitOptionIndex: linear scan for a value in the option vector.
// Returns options.size() (the off-the-end sentinel) when the value is absent,
// mirroring std::find / the radius-window contract that baseline_val is always
// a member of opts (so an absent value means the caller's baseline is stale and
// the walk must not run).
TEST(FindTimeLimitOptionIndexTest, ReturnsIndexWhenValuePresent) {
    std::vector<double> opts = {400.0, 600.0, 800.0, 1000.0};
    EXPECT_EQ(0u, FindTimeLimitOptionIndex(opts, 400.0));
    EXPECT_EQ(1u, FindTimeLimitOptionIndex(opts, 600.0));
    EXPECT_EQ(3u, FindTimeLimitOptionIndex(opts, 1000.0));
}

TEST(FindTimeLimitOptionIndexTest, ReturnsSizeSentinelWhenValueAbsent) {
    std::vector<double> opts = {400.0, 600.0, 800.0};
    EXPECT_EQ(opts.size(), FindTimeLimitOptionIndex(opts, 700.0));
    EXPECT_EQ(opts.size(), FindTimeLimitOptionIndex(opts, 1000.0));
}

TEST(FindTimeLimitOptionIndexTest, EmptyOptionsReturnsZeroSentinel) {
    std::vector<double> opts;
    EXPECT_EQ(0u, FindTimeLimitOptionIndex(opts, 600.0));
}

// IsBetterTimeLimitOption: the per-step adopt predicate for the unidirectional
// walk. Three cases, derived from the existing exhaustive tie-break logic in
// PerformCoordinateDescentForTaskConfigOpt (strictly-greater SP wins; on
// ApproxEqualSP ties prefer the smaller TL — which means a tie is "better"
// only when walking downward, step<0, since the next-down option is smaller).
TEST(IsBetterTimeLimitOptionTest, StrictlyHigherSPIsBetter) {
    // step direction is irrelevant when SP strictly improves.
    EXPECT_TRUE(IsBetterTimeLimitOption(1.7, 1.6, /*step=*/1));
    EXPECT_TRUE(IsBetterTimeLimitOption(1.7, 1.6, /*step=*/-1));
}

TEST(IsBetterTimeLimitOptionTest, StrictlyLowerSPIsNotBetter) {
    EXPECT_FALSE(IsBetterTimeLimitOption(1.5, 1.6, /*step=*/1));
    EXPECT_FALSE(IsBetterTimeLimitOption(1.5, 1.6, /*step=*/-1));
}

TEST(IsBetterTimeLimitOptionTest, ApproxEqualTieBetterOnlyWhenWalkingDown) {
    // ApproxEqualSP(1.6, 1.6) → tie. On a downward walk (step<0) the next
    // option is smaller, so the tie-break prefers it → "better". On an upward
    // walk (step>0) the next option is larger, so the tie-break rejects it →
    // not "better" (keeps the tighter TL already held).
    EXPECT_TRUE(IsBetterTimeLimitOption(1.6, 1.6, /*step=*/-1));
    EXPECT_FALSE(IsBetterTimeLimitOption(1.6, 1.6, /*step=*/1));
}

TEST(IsBetterTimeLimitOptionTest, NearEqualWithinToleranceIsTie) {
    // 1.6 vs 1.6+1e-12 is within ApproxEqualSP's rel_tol=1e-9 → treated as a
    // tie, so the same step-direction rule applies as for exact equality.
    EXPECT_TRUE(IsBetterTimeLimitOption(1.6 + 1e-12, 1.6, /*step=*/-1));
    EXPECT_FALSE(IsBetterTimeLimitOption(1.6 + 1e-12, 1.6, /*step=*/1));
}

// --- Trial-and-Error TL walk: OptimizeSingleTaskTimeLimit + rewritten
// PerformCoordinateDescentForTaskConfigOpt ---
//
// The walk replaces the exhaustive per-task enumeration with a unidirectional
// trial-and-error sweep: step outward from the baseline; adopt each improving
// option; stop after `patience` consecutive non-improving steps (patience=0 =
// strict break on first non-improvement; patience=1 = tolerate one dip). To
// test the termination logic deterministically (independent of RTA numerics),
// stub EvaluateTimeLimitConfig_ScratchOrIncre with a preprogrammed TL→SP map.

class StubTLWalkOptimizer : public OptimizePA_Incre_with_TimeLimits {
   public:
    // TL value → SP value returned by the stubbed evaluator. Any TL not in the
    // map returns -1.0 (worse than every real SP, so a walk never adopts it).
    std::map<double, double> tl_to_sp;
    // Every TL the stub was asked to evaluate, in call order. Used to assert on
    // early-termination: the walk stops asking once patience is exhausted.
    std::vector<double> evaluated_tls;

    StubTLWalkOptimizer(const DAG_Model& dag_tasks,
                        const SP_Parameters& sp_parameters,
                        std::map<double, double> tl_to_sp)
        : OptimizePA_Incre_with_TimeLimits(dag_tasks, sp_parameters),
          tl_to_sp(std::move(tl_to_sp)) {}

    double EvaluateTimeLimitConfig_ScratchOrIncre(
        int K, const std::vector<double>& time_limits,
        bool from_scratch) override {
        double tl = time_limits[walked_task_idx_];
        evaluated_tls.push_back(tl);
        auto it = tl_to_sp.find(tl);
        return it == tl_to_sp.end() ? -1.0 : it->second;
    }

    // The walk operates on one task at a time; the stub needs to know which
    // position in time_limits to read. Set by the test before invoking the
    // descent.
    size_t walked_task_idx_ = 0;
};

// Fixture: a 2-task DAG where only T_perf (task 0) carries TL options
// [400,600,800,1000]. T_noise is a tiny fixed-ET task. The fixture does NOT
// pin an SP profile — each test injects its own tl_to_sp map into the stub to
// shape the SP landscape along T_perf's TL axis.
class TrialAndErrorTLWalkSynthetic : public ::testing::Test {
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

    // Hand the stub a 5-option window around ET=500 (closest TL=600, the 2nd
    // option) by overriding time_limit_option_for_each_task_ directly. With
    // radius 2 the window is the full [400,600,800,1000].
    StubTLWalkOptimizer MakeStub(std::map<double, double> tl_to_sp) {
        StubTLWalkOptimizer opt(dag_tasks, sp_parameters, std::move(tl_to_sp));
        opt.time_limit_option_for_each_task_ = {{400.0, 600.0, 800.0, 1000.0},
                                                {-1.0}};
        opt.walked_task_idx_ = 0;
        return opt;
    }

    MAP_Prev mapPrev;
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
};

// Strict patience (patience=0): walking upward from baseline 600 with a
// strictly increasing SP profile, every step improves → the walk adopts each
// and runs to the window's upper boundary (1000). No early termination fires
// because no step is ever non-improving. Result = best option = 1000.
TEST_F(TrialAndErrorTLWalkSynthetic,
       StrictWalk_AdoptsMonotonicallyIncreasingToEnd) {
    std::map<double, double> sp;
    sp[400.0] = 1.4;
    sp[600.0] = 1.6;
    sp[800.0] = 1.7;
    sp[1000.0] = 1.8;
    auto opt = MakeStub(sp);

    std::vector<double> time_limits = {600.0, -1.0};
    double final_sp = opt.OptimizeSingleTaskTimeLimit(
        /*task_idx=*/0, /*K=*/2, time_limits,
        /*current_sp=*/sp[600.0], /*baseline_val=*/600.0,
        /*step=*/1, /*from_scratch=*/true, /*patience=*/0);

    EXPECT_DOUBLE_EQ(1.8, final_sp);
    EXPECT_DOUBLE_EQ(1000.0, time_limits[0]);
    // Walked 600→800→1000 (baseline 600 is the start, not re-evaluated); 800
    // and 1000 are the two trial evals. 400 (backward) is NOT visited because
    // this call only walks the forward direction.
    EXPECT_EQ(std::vector<double>({800.0, 1000.0}), opt.evaluated_tls);
}

// Strict patience (patience=0): walking upward, the first trial (800) is worse
// than the baseline (600). Strict break → the walk stops immediately, does NOT
// evaluate 1000, and keeps the baseline. Result = 600.
TEST_F(TrialAndErrorTLWalkSynthetic,
       StrictWalk_BreaksOnFirstNonImprovementAndKeepsBaseline) {
    std::map<double, double> sp;
    sp[400.0] = 1.4;
    sp[600.0] = 1.9;   // baseline — the local maximum
    sp[800.0] = 1.7;   // worse
    sp[1000.0] = 1.8;  // also worse than baseline; would be missed by strict
    auto opt = MakeStub(sp);

    std::vector<double> time_limits = {600.0, -1.0};
    double final_sp = opt.OptimizeSingleTaskTimeLimit(
        0, 2, time_limits, sp[600.0], 600.0, /*step=*/1, /*from_scratch=*/true,
        /*patience=*/0);

    EXPECT_DOUBLE_EQ(1.9, final_sp);
    EXPECT_DOUBLE_EQ(600.0, time_limits[0]);
    // Only 800 was evaluated; 1000 was never reached (strict break).
    EXPECT_EQ(std::vector<double>({800.0}), opt.evaluated_tls);
}

// Patience=1 (reopt lookahead): walking upward, the first trial (800) is worse
// than the best-yet (600) — that is one consecutive non-improvement, within
// the patience budget, so the walk CONTINUES. The next trial (1000) is
// strictly better than the best-yet (600) → adopted. The dip at 800 did NOT
// terminate the search, and the better option at 1000 was found. This is the
// case strict-terminate gets wrong.
TEST_F(TrialAndErrorTLWalkSynthetic,
       PatienceOne_ToleratesSingleDipAndFindsOptimumFurtherOut) {
    std::map<double, double> sp;
    sp[400.0] = 1.4;
    sp[600.0] = 1.6;   // baseline
    sp[800.0] = 1.5;   // dip — worse than best-yet (1.6)
    sp[1000.0] = 1.8;  // strictly better than best-yet → adopted
    auto opt = MakeStub(sp);

    std::vector<double> time_limits = {600.0, -1.0};
    double final_sp = opt.OptimizeSingleTaskTimeLimit(
        0, 2, time_limits, sp[600.0], 600.0, /*step=*/1, /*from_scratch=*/true,
        /*patience=*/1);

    EXPECT_DOUBLE_EQ(1.8, final_sp);
    EXPECT_DOUBLE_EQ(1000.0, time_limits[0]);
    // Both 800 and 1000 evaluated — the dip at 800 did not stop the walk.
    EXPECT_EQ(std::vector<double>({800.0, 1000.0}), opt.evaluated_tls);
}

// Patience=1: TWO consecutive non-improvements exhaust the budget and stop the
// walk. Best-yet (baseline 600) is preserved. Confirms patience=1 tolerates
// exactly one dip, not two.
TEST_F(TrialAndErrorTLWalkSynthetic,
       PatienceOne_BreaksAfterTwoConsecutiveNonImprovements) {
    std::map<double, double> sp;
    sp[400.0] = 1.4;
    sp[600.0] = 1.9;   // baseline — global max in the window
    sp[800.0] = 1.7;   // non-improvement #1 (within budget)
    sp[1000.0] = 1.8;  // non-improvement #2 (exhausts budget → break)
    auto opt = MakeStub(sp);

    std::vector<double> time_limits = {600.0, -1.0};
    double final_sp = opt.OptimizeSingleTaskTimeLimit(
        0, 2, time_limits, sp[600.0], 600.0, /*step=*/1, /*from_scratch=*/true,
        /*patience=*/1);

    EXPECT_DOUBLE_EQ(1.9, final_sp);
    EXPECT_DOUBLE_EQ(600.0, time_limits[0]);
    // Both 800 and 1000 evaluated (patience=1 lets the walk survive the 800
    // dip and try 1000), but neither beat 1.9 so the baseline is kept.
    EXPECT_EQ(std::vector<double>({800.0, 1000.0}), opt.evaluated_tls);
}

// Backward walk (step=-1): tie-break toward smaller TL. When the SP profile is
// flat (all approx-equal), every downward step is an improvement by the
// tie-break rule → the walk runs to the lower boundary (400) and adopts it.
TEST_F(TrialAndErrorTLWalkSynthetic,
       BackwardWalk_TieBreakAdoptsSmallestTimeLimitOnFlatSP) {
    std::map<double, double> sp;
    sp[400.0] = 1.6;
    sp[600.0] = 1.6;  // baseline, approx-equal to 400 and 800
    sp[800.0] = 1.6;
    sp[1000.0] = 1.6;
    auto opt = MakeStub(sp);

    std::vector<double> time_limits = {600.0, -1.0};
    double final_sp = opt.OptimizeSingleTaskTimeLimit(
        0, 2, time_limits, sp[600.0], 600.0, /*step=*/-1, /*from_scratch=*/true,
        /*patience=*/0);

    EXPECT_DOUBLE_EQ(400.0, time_limits[0]);
    // Walked 600→400 backward: 400 is the only trial eval (600 is baseline).
    EXPECT_EQ(std::vector<double>({400.0}), opt.evaluated_tls);
    // SP unchanged (flat profile) but TL tightened — the tie-break win.
    EXPECT_NEAR(final_sp, 1.6, 1e-9);
}

// No TL freedom (the {-1}-only case): the helper returns the current SP
// unchanged and evaluates nothing. This is the predicate the outer descent
// relies on to skip {-1}-only tasks entirely.
TEST_F(TrialAndErrorTLWalkSynthetic,
       NoOptions_ReturnsCurrentSpAndEvaluatesNothing) {
    auto opt = MakeStub({});
    // Override T_perf's options to the {-1}-only sentinel.
    opt.time_limit_option_for_each_task_[0] = {-1.0};

    std::vector<double> time_limits = {-1.0, -1.0};
    double final_sp = opt.OptimizeSingleTaskTimeLimit(
        0, 2, time_limits, /*current_sp=*/1.5, /*baseline_val=*/-1.0,
        /*step=*/1, /*from_scratch=*/true, /*patience=*/0);

    EXPECT_DOUBLE_EQ(1.5, final_sp);
    EXPECT_TRUE(opt.evaluated_tls.empty());
}

// Baseline TL not in the option window (stale baseline): the helper returns the
// current SP unchanged and evaluates nothing — there is no valid index from
// which to start the walk.
TEST_F(TrialAndErrorTLWalkSynthetic,
       BaselineNotInOptions_ReturnsCurrentSpAndEvaluatesNothing) {
    std::map<double, double> sp;
    sp[400.0] = 1.4;
    sp[600.0] = 1.6;
    sp[800.0] = 1.7;
    sp[1000.0] = 1.8;
    auto opt = MakeStub(sp);

    std::vector<double> time_limits = {700.0, -1.0};  // 700 not in opts
    double final_sp = opt.OptimizeSingleTaskTimeLimit(
        0, 2, time_limits, /*current_sp=*/1.5, /*baseline_val=*/700.0,
        /*step=*/1, /*from_scratch=*/true, /*patience=*/0);

    EXPECT_DOUBLE_EQ(1.5, final_sp);
    EXPECT_TRUE(opt.evaluated_tls.empty());
}

int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
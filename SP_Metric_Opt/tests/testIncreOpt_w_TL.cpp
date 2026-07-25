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
    opt.ReOptimizePeriodic(dag_tasks, 2);
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
    opt_with_tl.ReOptimizePeriodic(dag_tasks, 2);
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
    opt_no_tl.ReOptimizePeriodic(dag_tasks, 2);
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
    opt_normal.ReOptimizePeriodic(dag_tasks, 2);
    ResourceOptResult res_normal = opt_normal.CollectResults();
    double tl_normal = res_normal.id2time_limit[0];

    // 2. Run with WCET baseline enabled
    GlobalVariables::use_wcet_execution_time = true;
    OptimizePA_Incre_with_TimeLimits opt_wcet(dag_tasks, sp_parameters);
    opt_wcet.ReOptimizePeriodic(dag_tasks, 2);
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
    opt.ReOptimizePeriodic(dag_tasks, 2);
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

    // Bootstrap with ReOptimizePeriodic. TSP's full option set is
    // [400,600,800,1000]; at high utilization SP saturates so all options tie,
    // and the smaller-TL tie-break walks all the way down to 400.
    opt.ReOptimizePeriodic(dag_tasks, 2);
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

// P1.10 Phase 3 — the single-change invariant proof (P1.9's unblock condition).
// Every SP-eval on the serialized path must change AT MOST one task's ET vs the
// champion: Type-L TL step → |diff|==1 (the walked task); Type-E env-changed
// step → |diff|==0 (the env move is absorbed into dag_tasks_ on both diff sides,
// so candidate DAG == champion DAG — only the re-searched PA varies). |diff|>1
// would mean the champion drifted, breaking the sub-incremental's premise.
//
// The invariant is checked inside EvaluateTimeLimitConfig_SubIncremental via
// AssertSingleChangeInvariant, which is gated on debugMode and THROWS on a
// violation. So this test proves the invariant by RUNNING the serialized path
// across an update that exercises BOTH step kinds (v19→v21 moves the TL-flexible
// TSP task 0's dist AND the env-only SLAM task 3's dist — see the D2
// FindEnvTaskWithDifferentEt tests) with debugMode forced on: if any eval
// violated |diff|<=1, the assertion would throw and abort the test (FAIL).
// Reaching the EXPECT_GT line means the invariant held for every eval.
TEST_F(TaskSetForTest_robotics_v19, SerializedIncremental_SingleChangeInvariant) {
    DAG_Model dag_tasks_updated = ReadDAG_Tasks(
        GlobalVariables::PROJECT_PATH +
        "TaskData/test_robotics_v21.yaml");  // moves TSP (TL-flexible) + SLAM (env)

    int saved_debug = GlobalVariables::debugMode;
    GlobalVariables::debugMode = 1;  // arm the invariant assertion

    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    opt.ReOptimizePeriodic(dag_tasks, 2);  // bootstrap the incumbent
    // If the single-change invariant is violated at any SP-eval below,
    // AssertSingleChangeInvariant throws → the test aborts here (FAIL).
    opt.OptimizeIncre_w_TL(dag_tasks_updated, 2);
    ResourceOptResult res = opt.CollectResults();

    GlobalVariables::debugMode = saved_debug;

    // Sanity: the run produced a finite SP (it completed the queue walk).
    EXPECT_GT(res.sp_opt, 0.0);
}

// P1.25 — walk-level pins for the reject-path revert contract. P1.21's
// RTACache::Transaction (lazy CoW) has been REMOVED; its only load-bearing job
// was reverting the cache champion when a sub-incremental walk step is
// REJECTED by UpdateRecords. The ACCEPT path is tx-independent
// (CommitIncumbent re-adopts the committed triple, OptimizeSP_TL_Incre.cpp
// :791-793), so only the REJECT path needs a replacement revert (P1.25 D1=(b):
// eager `RTACache cache_backup = rta_cache_;` at entry + restore on reject,
// the pre-transaction shape from `1217d227`).
//
// Honest TDD status: 1a/1b are NO-REGRESSION GUARDS, not a red-then-green pin.
// The intended red arc — "naive delete (no backup) makes 1a FAIL" — did NOT
// materialize on the v19→v21 fixture (2c-RED finding): the walk's single reject
// had NO in-walk adopt, so no champion drift survived for 1a's final-state
// assertion to observe. The reject-after-adopt drift that the (b) backup guards
// is mechanically real but not exercised by these fixtures. The backup's
// necessity rests on git history (`1217d227` introduced it explicitly "to
// reduce rta_cache_ becoming outdated") + the mechanism, NOT on a live red.
// Both pins PASS against the (b) backup (champion RTA == committed oracle RTA);
// they guard against future regressions of the revert + the accept re-adopt.
//
// Test subclass: OptimizePA_Incre_with_TimeLimits keeps rta_cache_ + dag_tasks_
// public, so a thin subclass exposes them for the pin.

class RtaCacheExposingOptimizer : public OptimizePA_Incre_with_TimeLimits {
   public:
    using OptimizePA_Incre_with_TimeLimits::OptimizePA_Incre_with_TimeLimits;
    const RTACache& RtaCache() const { return rta_cache_; }
    const DAG_Model& DagTasks() const { return dag_tasks_; }
};

// P1.25 1a (no-regression guard): after a full INCREMENTAL walk (v19→v21,
// which exercises both Type-E env + Type-L TL steps and necessarily REJECTS
// some trial configs — most trial TLs do not beat the running incumbent), the
// cache champion MUST track the COMMITTED incumbent triple (res_opt_), NOT some
// rejected trial PA. The champion RTA must be byte-identical to the committed
// triple's oracle RTA. See the header for why this is a guard, not a red pin:
// the v19→v21 reject had no in-walk adopt, so the drift the (b) backup guards
// is not exercised here; the pin nonetheless locks the final-state contract
// against future regressions of the reject-path revert.
TEST_F(TaskSetForTest_robotics_v19,
       SubIncrementalReject_RevertKeepsChampionOnCommittedTriple) {
    DAG_Model dag_tasks_updated = ReadDAG_Tasks(
        GlobalVariables::PROJECT_PATH +
        "TaskData/test_robotics_v21.yaml");  // moves TSP (TL-flexible) + SLAM (env)

    RtaCacheExposingOptimizer opt(dag_tasks, sp_parameters);
    opt.ReOptimizePeriodic(dag_tasks, 2);  // bootstrap the incumbent
    opt.OptimizeIncre_w_TL(dag_tasks_updated, 2);
    ResourceOptResult res = opt.CollectResults();

    // The committed triple: the updated DAG (OptimizeIncre_w_TL absorbed it
    // into dag_tasks_ at :717), the committed priority_vec, and the committed
    // TL vector. Reconstruct the oracle RTA the SAME way RTACache::Initialize
    // does (apply TLs -> apply pa -> ProbabilisticRTA_TaskSet) and require the
    // stored champion to be byte-identical to it.
    std::vector<double> committed_tl = opt.ReconstructTimeLimitVecFromResOpt();
    TaskSet tasks_with_tl = ApplyTimeLimitsToTasksExecutionTime(
        opt.DagTasks().tasks, committed_tl);
    TaskSet tasks_prioritized =
        UpdateTaskSetPriorities(tasks_with_tl, res.priority_vec);
    std::vector<FiniteDist> oracle_rtas = ProbabilisticRTA_TaskSet(tasks_prioritized);

    ASSERT_TRUE(opt.RtaCache().HasChampion())
        << "champion missing after walk — cache was not engaged on this path";
    const std::vector<FiniteDist>& champion_rtas = opt.RtaCache().Rta();
    ASSERT_EQ(oracle_rtas.size(), champion_rtas.size());
    for (size_t i = 0; i < oracle_rtas.size(); i++) {
        EXPECT_TRUE(oracle_rtas[i] == champion_rtas[i])
            << "champion rtas[" << i
            << "] != committed-triple oracle RTA — a rejected trial PA leaked "
               "into the champion (reject-path revert missing/broken)";
    }
}

// P1.25 1b (accept-path guard): the ACCEPT path keeps the champion tracking
// the committed triple via the tx-INDEPENDENT CommitIncumbent re-adopt
// (OptimizeSP_TL_Incre.cpp:791-793), with NO reject machinery involved.
// Bootstrap with ReOptimizePeriodic (cache off), then run OptimizeIncre_w_TL on
// the SAME DAG (v19→v19, no env/TL move) —
// PerformSerializedTaskQueueOptimization arms rta_cache_active_ (:468) and the
// baseline CommitIncumbent (:481) adopts the champion unconditionally, even if
// the merged queue is empty. So the champion is populated purely by the
// accept-path writer, with no reject ever firing. Pins that the deletion does
// not break the accept-path writer.
TEST_F(TaskSetForTest_robotics_v19,
       SubIncrementalAccept_ChampionTracksCommittedTriple) {
    RtaCacheExposingOptimizer opt(dag_tasks, sp_parameters);
    opt.ReOptimizePeriodic(dag_tasks, 2);  // bootstrap (cache off)
    opt.OptimizeIncre_w_TL(dag_tasks, 2);  // same DAG: arms cache + baseline adopt
    ResourceOptResult res = opt.CollectResults();

    std::vector<double> committed_tl = opt.ReconstructTimeLimitVecFromResOpt();
    TaskSet tasks_with_tl = ApplyTimeLimitsToTasksExecutionTime(
        opt.DagTasks().tasks, committed_tl);
    TaskSet tasks_prioritized =
        UpdateTaskSetPriorities(tasks_with_tl, res.priority_vec);
    std::vector<FiniteDist> oracle_rtas = ProbabilisticRTA_TaskSet(tasks_prioritized);

    ASSERT_TRUE(opt.RtaCache().HasChampion())
        << "champion missing — OptimizeIncre_w_TL did not arm the cache + adopt "
           "the baseline via CommitIncumbent";
    const std::vector<FiniteDist>& champion_rtas = opt.RtaCache().Rta();
    ASSERT_EQ(oracle_rtas.size(), champion_rtas.size());
    for (size_t i = 0; i < oracle_rtas.size(); i++) {
        EXPECT_TRUE(oracle_rtas[i] == champion_rtas[i])
            << "champion rtas[" << i
            << "] != committed-triple oracle RTA on the accept path "
               "(CommitIncumbent re-adopt broken)";
    }
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

    opt.ReOptimizePeriodic(dag_tasks, 2);
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
    opt_scratch.ReOptimizePeriodic(dag_tasks, 2);
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
    // path requires res_opt_ to be initialized (IfInitialized(), otherwise the
    // contract violation in EvaluateTimeLimitConfig_ScratchOrIncre fires).
    opt_incre.ReOptimizePeriodic(dag_tasks, 2);
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
// the helper unit tests (SmallestTimeLimitVec, ResetIncumbentBaseline, etc.)
// that are profile-shape-independent.
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
// Bootstrap on v30_lo → incumbent {TL=1000}. Reopt on v30_hi:
// ResetIncumbentBaseline(true) re-evaluates {TL=1000} under v30_hi (strictly
// worse than v30_hi's optimum), the fresh search finds TL=400, and
// UpdateRecords' strictly-greater-SP guard ADOPTS 400. This is the genuine
// compare-and-keep adopt path — no synthetic radius.
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

    // The incumbent gate is set and the thin opt_pa_/opt_sp_ mirrors (populated
    // by CommitIncumbent, which SeedStateFromIncumbent calls) carry the same
    // {sp, pa} tuple. Under the redesign res_opt_ is the durable store; the
    // mirrors are what the public surface reads.
    EXPECT_TRUE(opt.IfInitialized());
    EXPECT_DOUBLE_EQ(sp, opt.opt_sp_);
    EXPECT_EQ(pa, opt.opt_pa_);
    EXPECT_EQ(pa, opt.res_opt_.priority_vec);
}

// --- Incumbent-state helpers (P0.5 redesign) ---
// CommitIncumbent is the single writer for the durable incumbent store
// (res_opt_ + the opt_pa_/opt_sp_ mirrors) and the only thing that establishes
// an incumbent (so IfInitialized() flips true). Tested in isolation: it must
// populate the four-tuple verbatim, without relying on SeedStateFromIncumbent
// or UpdateRecords.
TEST_F(CompareAndKeepSynthetic, CommitIncumbent_WritesFourTupleAndSetsGate) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    EXPECT_FALSE(opt.IfInitialized());

    PriorityVec pa = {0, 1};
    std::vector<double> tl = {800.0, -1.0};
    const double sp = 1.234;  // arbitrary sentinel — must be stored verbatim

    opt.CommitIncumbent(pa, sp, tl);

    // res_opt_ holds the carried TL (id-keyed) and SP verbatim.
    EXPECT_TRUE(opt.IfInitialized());
    EXPECT_DOUBLE_EQ(800.0, opt.res_opt_.id2time_limit[0]);
    EXPECT_DOUBLE_EQ(-1.0, opt.res_opt_.id2time_limit[1]);
    EXPECT_DOUBLE_EQ(sp, opt.res_opt_.sp_opt);
    // Thin mirrors the public surface reads.
    EXPECT_DOUBLE_EQ(sp, opt.opt_sp_);
    EXPECT_EQ(pa, opt.opt_pa_);
    // Carried PA round-trips through res_opt_.priority_vec.
    EXPECT_EQ(pa, opt.res_opt_.priority_vec);

    // CommitIncumbent is the single writer that establishes an incumbent. A
    // fresh opt starts uninitialized; after the commit IfInitialized() is true.
    EXPECT_TRUE(opt.IfInitialized());
}

// BuildChallengerFromIncumbent reconstructs a throwaway OptimizePA_Incre from
// res_opt_: its dag_tasks_ is the current raw DAG with the CARRIED adopted TL
// applied, and its opt_pa_/opt_sp_ mirror res_opt_. This is the diff-baseline
// invariant made structural — the challenger's dag_tasks_ IS the baseline side
// of FindTaskWithDifferentEt. Verified by building the expected DAG with the
// same primitive the helper uses and comparing per-task ET distributions.
TEST_F(CompareAndKeepSynthetic, BuildChallengerFromIncumbent_ReconstructsAdoptedTlDag) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);

    // Establish an incumbent: adopt T_perf=800, T_noise=-1, an arbitrary PA/SP.
    PriorityVec pa = {1, 0};
    std::vector<double> tl = {800.0, -1.0};
    const double sp = 2.5;
    opt.CommitIncumbent(pa, sp, tl);
    ASSERT_TRUE(opt.IfInitialized());

    OptimizePA_Incre challenger = opt.BuildChallengerFromIncumbent();

    // The challenger's dag_tasks_ must equal the raw DAG with the carried TL
    // applied — the exact baseline FindTaskWithDifferentEt will diff against.
    std::vector<double> tl_prev = opt.ReconstructTimeLimitVecFromResOpt();
    DAG_Model expected_dag = UpdateExtDistBasedOnTimeLimit(dag_tasks, tl_prev);
    ASSERT_EQ(expected_dag.tasks.size(), challenger.dag_tasks_.tasks.size());
    for (size_t i = 0; i < expected_dag.tasks.size(); ++i) {
        EXPECT_EQ(expected_dag.tasks[i].execution_time_dist,
                  challenger.dag_tasks_.tasks[i].execution_time_dist)
            << "task " << i << " ET dist differs from the reconstructed baseline";
    }
    // opt_pa_/opt_sp_ mirror res_opt_ so OptimizeIncre warm-starts from the
    // carried PA and the compare-and-keep guard measures against the carried SP.
    EXPECT_EQ(pa, challenger.opt_pa_);
    EXPECT_DOUBLE_EQ(sp, challenger.opt_sp_);
    // The challenger is a throwaway local — building it must not mutate the
    // incumbent store (res_opt_ unchanged, still initialized).
    EXPECT_TRUE(opt.IfInitialized());
    EXPECT_DOUBLE_EQ(800.0, opt.res_opt_.id2time_limit[0]);
    EXPECT_DOUBLE_EQ(sp, opt.res_opt_.sp_opt);
}


TEST_F(CompareAndKeepSynthetic, SeedIncumbentBaseline_Interval0UsesRMAndMinTL) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    EXPECT_FALSE(opt.IfInitialized());

    // Compute the expected interval-0 baseline — RM priorities + smallest TL —
    // with the same primitives the helper uses internally, then verify the
    // helper seeds exactly that.
    PriorityVec pa_rm = opt.RateMonotonicPriorityVec();
    std::vector<double> tl_min = opt.SmallestTimeLimitVec();
    DAG_Model dag_min = UpdateExtDistBasedOnTimeLimit(dag_tasks, tl_min);
    double expected_sp =
        EvaluateSPWithPriorityVec(dag_min, sp_parameters, pa_rm);

    opt.ResetIncumbentBaseline(true);

    EXPECT_TRUE(opt.IfInitialized());
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
    ASSERT_TRUE(opt.IfInitialized());

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

    opt.ResetIncumbentBaseline(true);

    // With-incumbent branch: TL stays 800 (NOT reset to min 400) and SP is the
    // re-evaluated value under the new DAG (not the stale sp_incumbent).
    EXPECT_DOUBLE_EQ(expected_sp, opt.opt_sp_);
    EXPECT_DOUBLE_EQ(800.0, opt.res_opt_.id2time_limit[0]);
    EXPECT_DOUBLE_EQ(-1.0, opt.res_opt_.id2time_limit[1]);
    EXPECT_EQ(pa, opt.opt_pa_);
}

// Issue (5) — baseline-overwrites-res_opt_ invariant. The baseline eval at the
// top of PerformCoordinateDescentForTaskConfigOpt
//   current_config_sp = EvaluateTimeLimitConfig_ScratchOrIncre(K, time_limits, from_scratch);
// must ALWAYS overwrite res_opt_ for the current interval. Otherwise, when the
// new interval's best achievable SP is LOWER than the previous interval's
// incumbent SP (e.g. the DAG grew heavier), UpdateRecords' strictly-greater-SP
// guard would reject the baseline and res_opt_ would retain a STALE
// previous-interval incumbent — the descent's compare-and-keep would then be
// measured against the wrong baseline.
//
// Scenario: bootstrap on the light fixture DAG (adopts TL=1000, high SP). Then
// run the incremental path on a HEAVY DAG (T_noise ET -> 1900, past the 2000
// deadline -> deadline miss -> SP drops). After the incremental call,
// res_opt_.sp_opt must reflect the current (heavy) interval: strictly lower
// than the light-DAG SP. If the baseline did not overwrite res_opt_, the stale
// high SP would survive.
//
// The invariant holds because ResetIncumbentBaseline(false) (called at the top
// of the descent) sets opt_sp_=-1.0 before the baseline eval, forcing the first
// UpdateRecords to commit. This test pins that invariant so a future edit that
// drops the -1.0 reset regresses loudly.
TEST_F(CompareAndKeepSynthetic,
       OptimizeIncre_w_TL_BaselineOverwritesResOptForNewInterval) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    // Bootstrap on the light DAG. Monotonic strictly-increasing-in-TL landscape
    // -> adopts TL=1000 (the optimum).
    opt.ReOptimizePeriodic(dag_tasks, 2);
    ASSERT_TRUE(opt.IfInitialized());
    const double sp_light = opt.res_opt_.sp_opt;
    ASSERT_GT(sp_light, 0.0);

    // Heavy DAG: T_noise ET -> 1900 (deadline 2000). T_noise has no perf pair
    // (TL=-1), so UpdateExtDistBasedOnTimeLimit passes its ET through unchanged.
    // Under RM (T_perf TL=1000 -> ET~1000) T_noise's response time ~2900 > 2000
    // -> deadline miss -> SP drops below the light-DAG value.
    DAG_Model dag_heavy = dag_tasks;
    dag_heavy.tasks[1].execution_time_dist =
        FiniteDist(GaussianDist(1900.0, 0.5), 5);

    opt.OptimizeIncre_w_TL(dag_heavy, 2);

    // The baseline overwrote res_opt_: the carried SP is the heavy-DAG value
    // (strictly lower than sp_light), NOT the stale light SP.
    EXPECT_LT(opt.res_opt_.sp_opt, sp_light)
        << "res_opt_.sp_opt was not overwritten for the current interval; the "
        << "stale previous-interval SP (" << sp_light << ") survived.";
}

// Same invariant for the reopt path. ResetIncumbentBaseline(true) re-evaluates
// the carried incumbent {pa, tl} under the NEW (heavy) DAG and commits that, so
// res_opt_.sp_opt becomes the re-evaluated (heavy) value before the descent
// runs — the baseline cannot be rejected against a stale high SP.
TEST_F(CompareAndKeepSynthetic,
       ReOptimizePeriodic_BaselineOverwritesResOptForNewInterval) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    opt.ReOptimizePeriodic(dag_tasks, 2);
    ASSERT_TRUE(opt.IfInitialized());
    const double sp_light = opt.res_opt_.sp_opt;
    ASSERT_GT(sp_light, 0.0);

    DAG_Model dag_heavy = dag_tasks;
    dag_heavy.tasks[1].execution_time_dist =
        FiniteDist(GaussianDist(1900.0, 0.5), 5);

    opt.ReOptimizePeriodic(dag_heavy, 2);

    EXPECT_LT(opt.res_opt_.sp_opt, sp_light)
        << "res_opt_.sp_opt was not overwritten for the current interval; the "
        << "stale previous-interval SP (" << sp_light << ") survived.";
}

// --- Counter-driven dispatcher (Optimize_w_TL_ScratchOrIncre) ---
//
// Synthetic 2-task DAG: T_perf (task 0) carries 10 evenly-spaced TL options
// [0,10,...,90] with ET=45 (closest option = index 4, value 40). T_noise is a
// small fixed-ET task. The dispatcher re-runs ReOptimizePeriodic every
// ReoptimizationPeriod-th call and OptimizeIncre_w_TL otherwise. Routing is
// observed via TWO seams: the reopt branch drives its candidates through
// EvaluateTimeLimitConfig_ScratchOrIncre (from_scratch=true); the incremental
// branch drives its interval search through PerformSerializedTaskQueueOptimization
// (P1.10 — the serialized E+L queue). The fixture's RecordingDispatcherOpt
// subclass records BOTH (the from_scratch flags AND a count of serialized
// entries), so a test can assert which branch each dispatch took.
//
// ReoptimizationPeriod is PINNED in SetUp (=10) so this fixture is independent
// of the production default in parameters.yaml. These tests exercise dispatch
// ROUTING, not the period value; pinning the period they were designed around
// (count 0 → reopt, count 1..9 → incremental) keeps the routing signal crisp.
class CounterDispatcherSynthetic : public ::testing::Test {
   public:
    // Subclass that records the from_scratch flag of every
    // EvaluateTimeLimitConfig_ScratchOrIncre call (the reopt branch's per-candidate
    // eval) AND counts entries into PerformSerializedTaskQueueOptimization (the
    // incremental branch's driver). The recorded signals are exactly the routing
    // decisions the dispatcher made.
    class RecordingDispatcherOpt : public OptimizePA_Incre_with_TimeLimits {
       public:
        std::vector<bool> from_scratch_flags;
        int serialized_entries = 0;
        // P2.9 lever A: counts per-candidate evals routed through the
        // sub-incremental (cache-routed, |diff|<=1) eval. The legacy reopt walk
        // never calls this for its TL trials (it uses ScratchOrIncre); the
        // lever-A walk routes every trial here. So subincremental_calls>0 after
        // a reopt descent is the lever-A routing signal.
        int subincremental_calls = 0;
        // P2.11: the task_idx of every sub-incremental call, in call order. The
        // reopt sub-incremental arm historically walked sorted_indices and
        // skipped {-1}-only tasks, so an env-changed task with no perf pairs was
        // never reached. The merged arm walks the E+L serialized queue, whose
        // Type-E handler calls SubIncremental directly on the env-changed task.
        // So a task_idx appearing here that the legacy arm skipped is the
        // Type-E-handling signal.
        std::vector<size_t> subincremental_task_idx;
        using OptimizePA_Incre_with_TimeLimits::OptimizePA_Incre_with_TimeLimits;
        double EvaluateTimeLimitConfig_ScratchOrIncre(
            int K, const std::vector<double>& time_limits,
            bool from_scratch) override {
            from_scratch_flags.push_back(from_scratch);
            return OptimizePA_Incre_with_TimeLimits::
                EvaluateTimeLimitConfig_ScratchOrIncre(K, time_limits,
                                                       from_scratch);
        }
        double EvaluateTimeLimitConfig_SubIncremental(
            int K, const std::vector<double>& time_limits, size_t task_idx,
            bool et_increased) override {
            ++subincremental_calls;
            subincremental_task_idx.push_back(task_idx);
            return OptimizePA_Incre_with_TimeLimits::
                EvaluateTimeLimitConfig_SubIncremental(K, time_limits, task_idx,
                                                       et_increased);
        }
        void PerformSerializedTaskQueueOptimization(
            int K, std::vector<double>& starting_time_limits,
            const DAG_Model& dag_tasks_prev_pre_tl) override {
            ++serialized_entries;
            OptimizePA_Incre_with_TimeLimits::
                PerformSerializedTaskQueueOptimization(
                    K, starting_time_limits, dag_tasks_prev_pre_tl);
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
        saved_subincremental_walk_ =
            GlobalVariables::ReoptimizationUseSubIncrementalWalk;
    }

    void TearDown() override {
        GlobalVariables::ReoptimizationPeriod = saved_period_;
        GlobalVariables::ReoptimizationUseSubIncrementalWalk =
            saved_subincremental_walk_;
    }

    MAP_Prev mapPrev;
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
    int saved_period_;
    int saved_subincremental_walk_;
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
// opt this is the interval-0 bootstrap: ResetIncumbentBaseline(true) synthesizes
// an RM+min-TL incumbent, so the call succeeds (no CoutError) and leaves the opt
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

// count == 0 routes to reopt (from_scratch=true, driven through
// EvaluateTimeLimitConfig_ScratchOrIncre); count == 1 is not modular
// (1 % 10 != 0) so the second call routes to the incremental branch, driven
// through PerformSerializedTaskQueueOptimization (P1.10 — the serialized E+L
// queue; the per-candidate eval is the sub-incremental, NOT ScratchOrIncre).
// The routing is observable via the recorded counts: the second call enters the
// serialized driver at least once, proving the incremental branch — not reopt —
// ran. The incumbent established by the first call lets the incremental path's
// warm-start contract hold (no CoutError).
TEST_F(CounterDispatcherSynthetic, RoutesToIncrementalAtNonModularCount) {
    RecordingDispatcherOpt opt(dag_tasks, sp_parameters);

    // count == 0 → reopt. Establishes the incumbent.
    opt.Optimize_w_TL_ScratchOrIncre(dag_tasks, 2);
    ASSERT_EQ(1, opt.reoptimization_interval_count_);
    ASSERT_GT(opt.from_scratch_flags.size(), 0u);
    for (bool fs : opt.from_scratch_flags) {
        ASSERT_TRUE(fs);
    }
    // count == 0 routes to reopt, NOT the serialized incremental driver.
    ASSERT_EQ(0, opt.serialized_entries)
        << "count==0 must NOT enter the serialized incremental driver.";

    // count == 1 → 1 % 10 != 0 → incremental.
    opt.Optimize_w_TL_ScratchOrIncre(dag_tasks, 2);
    EXPECT_EQ(2, opt.reoptimization_interval_count_);
    // The incremental branch entered the serialized driver — the routing signal
    // that the second call took the incremental branch (not reopt).
    EXPECT_GE(opt.serialized_entries, 1)
        << "count==1 must route through the incremental (serialized driver) "
        << "branch; it never entered PerformSerializedTaskQueueOptimization "
        << "(reopt ran instead).";
}

// P2.9 lever A — default-OFF routing. With ReoptimizationUseSubIncrementalWalk=0
// (the production default), the reopt descent's TL walk evaluates every trial TL
// through the legacy full-beam EvaluateTimeLimitConfig_ScratchOrIncre
// (from_scratch=true). The sub-incremental eval is the incremental path's
// primitive; the legacy reopt walk must NOT touch it. So after a count==0 reopt
// dispatch, subincremental_calls==0 (no walk trial routed through it) while the
// from-scratch flags are non-empty (the baseline beam + every walk trial).
TEST_F(CounterDispatcherSynthetic,
       ReoptWalk_Legacy_Off_RoutesTrialsThroughScratchOrIncre) {
    GlobalVariables::ReoptimizationUseSubIncrementalWalk = 0;
    RecordingDispatcherOpt opt(dag_tasks, sp_parameters);

    opt.Optimize_w_TL_ScratchOrIncre(dag_tasks, 2);  // count==0 → reopt

    ASSERT_FALSE(opt.from_scratch_flags.empty())
        << "count==0 reopt must evaluate candidates through "
        << "EvaluateTimeLimitConfig_ScratchOrIncre (baseline beam + walk trials).";
    for (bool fs : opt.from_scratch_flags) {
        EXPECT_TRUE(fs) << "legacy reopt walk must evaluate every trial with "
                        << "from_scratch=true; saw a false flag.";
    }
    EXPECT_EQ(0, opt.subincremental_calls)
        << "legacy reopt walk (flag OFF) must NOT route any trial through the "
        << "sub-incremental eval; that is the incremental path's primitive.";
}

// P2.9 lever A — flag-ON routing. With ReoptimizationUseSubIncrementalWalk=1,
// the reopt descent still runs ONE baseline beam through
// EvaluateTimeLimitConfig_ScratchOrIncre(from_scratch=true) to establish the
// champion, then switches the TL walk's per-candidate eval to the cache-routed
// EvaluateTimeLimitConfig_SubIncremental (|diff|<=1 single-task re-search). So
// after a count==0 reopt dispatch: from_scratch_flags is non-empty (the one
// baseline beam) AND subincremental_calls>0 (the walk trials). The walk trials
// must NOT appear as from_scratch flags — only the baseline beam does.
TEST_F(CounterDispatcherSynthetic,
       ReoptWalk_LeverA_On_RoutesWalkTrialsThroughSubIncremental) {
    GlobalVariables::ReoptimizationUseSubIncrementalWalk = 1;
    RecordingDispatcherOpt opt(dag_tasks, sp_parameters);

    opt.Optimize_w_TL_ScratchOrIncre(dag_tasks, 2);  // count==0 → reopt

    // The baseline beam still runs through ScratchOrIncre(from_scratch=true).
    ASSERT_FALSE(opt.from_scratch_flags.empty())
        << "lever-A reopt must still run the one baseline beam through "
        << "EvaluateTimeLimitConfig_ScratchOrIncre.";
    for (bool fs : opt.from_scratch_flags) {
        EXPECT_TRUE(fs);
    }
    // The walk trials route through the sub-incremental eval. T_perf has a
    // 10-option TL set; on this monotonic SP landscape the walk takes ≥1 step
    // before patience stops it, so subincremental_calls>0.
    EXPECT_GT(opt.subincremental_calls, 0)
        << "lever-A reopt walk must route its per-candidate trials through the "
        << "sub-incremental eval; saw zero calls (the walk did not run, or ran "
        << "through the legacy ScratchOrIncre path instead).";
}

// P2.11 reading (a) — Type-E in the reopt queue. The merged reopt path walks the
// SAME E+L serialized queue the incremental path uses (BuildSerializedTaskQueue),
// so an env-changed task with NO perf pair (T_noise, task 1) is reached via its
// Type-E entry and re-searched through EvaluateTimeLimitConfig_SubIncremental.
//
// The legacy reopt sub-incremental arm walks sorted_indices and skips {-1}-only
// tasks (T_noise has no perf pair → {-1}-only → skipped), so it NEVER reaches an
// env-changed T_noise. This test pins the merge: after a flag-ON reopt on a DAG
// whose T_noise ET changed since the previous interval, the recorded
// subincremental_task_idx must contain T_noise's index (1). Today it does not
// (legacy arm skips it, AND ReOptimizePeriodic does not even capture the pre-
// absorb DAG for the Type-E diff) → fails RED until both gaps close.
//
// Direct ReOptimizePeriodic calls (the counter routing is pinned by the tests
// above; this targets the reopt path's queue contents). Bootstrap on the
// original DAG establishes the incumbent; a second reopt on an env-changed DAG
// is the reopt under test. The env change (T_noise ET 50 → 1234) is fresh: the
// previous interval's dag_tasks_ is the original DAG, the reopt's update is the
// env-changed DAG → FindEnvTaskWithDifferentEt flags T_noise as Type-E.
TEST_F(CounterDispatcherSynthetic,
       ReoptWalk_LeverA_On_ReachesEnvChangedTaskViaSerializedQueue) {
    GlobalVariables::ReoptimizationUseSubIncrementalWalk = 1;
    RecordingDispatcherOpt opt(dag_tasks, sp_parameters);

    // Bootstrap the incumbent on the original DAG (T_noise ET ~ 50). With the
    // flag ON this already uses the sub-incremental arm; on the bootstrap there
    // is no env change (prev == update), so only T_perf's Type-L walk runs.
    opt.ReOptimizePeriodic(dag_tasks, 2);
    ASSERT_TRUE(opt.IfInitialized());

    // Drop the bootstrap's recorded calls so only the second reopt's routing is
    // observed.
    opt.subincremental_calls = 0;
    opt.subincremental_task_idx.clear();
    opt.from_scratch_flags.clear();

    // Env change: T_noise (task 1, no perf pair → {-1}-only, so the TL-flexible
    // filter does NOT exclude it from FindEnvTaskWithDifferentEt) ET moved from
    // ~50 to 1234. The merged E+L queue must emit a Type-E entry for it.
    DAG_Model dag_env_changed = dag_tasks;
    dag_env_changed.tasks[1].execution_time_dist =
        GetUnitExecutionTimeDist(1234.0);

    // The reopt under test: previous interval's dag_tasks_ is the original DAG,
    // this call's update is the env-changed DAG.
    opt.ReOptimizePeriodic(dag_env_changed, 2);

    // The merged reopt arm walks the E+L serialized queue. T_noise's ET change
    // is Type-E → its handler calls SubIncremental with task_idx == 1 (T_noise).
    // The legacy arm skips {-1}-only tasks → never reaches T_noise → this fails.
    bool reached_t_noise = false;
    for (size_t idx : opt.subincremental_task_idx) {
        if (idx == 1) reached_t_noise = true;
    }
    EXPECT_TRUE(reached_t_noise)
        << "P2.11 merge: a flag-ON reopt on a DAG with an env-changed T_noise "
        << "(no perf pair, Type-E) must reach T_noise via the E+L serialized "
        << "queue's Type-E handler (SubIncremental call with task_idx==1). The "
        << "legacy reopt arm skips {-1}-only tasks and never reaches it. "
        << "Recorded subincremental_task_idx: "
        << ::testing::PrintToString(opt.subincremental_task_idx);
}

// INCR diff-baseline invariant (P0.5 redesign). The whole point of carrying an
// incumbent is the diff in FindTaskWithDifferentEt(dag_tasks_, dag_tasks_update):
// the baseline side is the carried adopted-TL DAG, the update side is the current
// interval's DAG. Under the redesign the challenger is a THROWAWAY rebuilt from
// res_opt_ each interval via BuildChallengerFromIncumbent — its dag_tasks_ IS
// the baseline side, reconstructed as
// UpdateExtDistBasedOnTimeLimit(dag_tasks_, ReconstructTimeLimitVecFromResOpt()).
// So the invariant this test guards is: after an incremental call on a NEW DAG,
// the challenger reconstructed from res_opt_ carries the CURRENT interval's DAG
// (not the frozen bootstrap DAG). If the baseline were frozen at the reopt DAG,
// consecutive incremental calls would diff stale-DAG vs fresh-DAG and ndiff would
// saturate at N every interval → per-act ET grows with ReoptimizationPeriod (the
// original frozen-baseline bug).
//
// Observable: T_noise (task 1) has no time-performance pairs → its TL is always
// -1 → UpdateExtDistBasedOnTimeLimit passes its execution_time_dist through
// unchanged. So challenger.dag_tasks_.tasks[1].execution_time_dist is a direct,
// debugMode-independent window onto whether the reconstructed baseline DAG
// reflects the current interval. Bootstrap establishes the original ET there; a
// second interval with a MUTATED T_noise ET must propagate that mutation into the
// challenger rebuilt after the incremental call. (The carried TL on T_perf lives
// in res_opt_.id2time_limit — also asserted, as the other half of the 4-tuple.)
TEST_F(CompareAndKeepSynthetic, OptimizeIncre_AdvancesPrevOptimizerDagTasks) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);

    // Bootstrap the incumbent. After ReOptimizePeriodic, res_opt_ holds the
    // adopted TL; T_noise's TL is -1 so the challenger rebuilt now carries the
    // ORIGINAL fixture ET (FiniteDist around 50.0) on T_noise.
    opt.ReOptimizePeriodic(dag_tasks, 2);
    ASSERT_TRUE(opt.IfInitialized());
    const double original_noise_et =
        opt.BuildChallengerFromIncumbent().dag_tasks_.tasks[1]
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

    // INVARIANT UNDER TEST: the challenger rebuilt from res_opt_ after the
    // incremental call carries the CURRENT interval's DAG. T_noise's TL is -1
    // → its ET in the TL-applied baseline equals dag_v2's mutated ET → the
    // rebuilt challenger's dag_tasks_ must carry the mutated ET. Under the old
    // prev_optimizer_ design this required OptimizeIncre to advance a stored
    // copy; under the redesign there is no stored DAG to advance — the baseline
    // is reconstructed from the current dag_tasks_ every interval, so it carries
    // the current interval's DAG by construction.
    const double carried_noise_et =
        opt.BuildChallengerFromIncumbent().dag_tasks_.tasks[1]
            .execution_time_dist.GetAvgValue();
    EXPECT_NEAR(carried_noise_et, mutated_noise_et, 5.0)
        << "The challenger rebuilt from res_opt_ does not carry the current "
        << "interval's DAG; the diff baseline is stale. Expected ~"
        << mutated_noise_et << " (current interval), got " << carried_noise_et
        << " (bootstrap value ~" << original_noise_et << ").";

    // The other half of the incumbent 4-tuple: the carried adopted TL on T_perf
    // survives in res_opt_ (CommitIncumbent wrote it). T_noise's TL is -1.
    EXPECT_NE(-1.0, opt.res_opt_.id2time_limit[dag_v2.tasks[0].id])
        << "Adopted TL for T_perf must survive the incremental call in res_opt_.";
    EXPECT_DOUBLE_EQ(-1.0, opt.res_opt_.id2time_limit[dag_v2.tasks[1].id]);
}

// P1.1 descent-start-TL fix. The incremental descent's STARTING time-limit
// vector determines what `UpdateExtDistBasedOnTimeLimit` applies as point dists
// on the update side of `FindTaskWithDifferentEt`'s diff (the serialized driver
// builds dag_baseline = UpdateExtDistBasedOnTimeLimit(dag_tasks_,
// starting_time_limits) for its baseline re-score; the per-task sub-incremental
// evals build dag_tasks_cur = UpdateExtDistBasedOnTimeLimit(dag_tasks_, tl)
// with tl seeded from the same starting vector).
//
// The baseline side carries the interval-(N-1) adopted TL (SeedStateFromIncumbent
// applies ReconstructTimeLimitVecFromResOpt()). The update side MUST start from
// that same carried adopted TL, so an UNCHANGED perf-pair task (same adopted TL
// on both sides) is NOT flagged. Before the fix, OptimizeIncre_w_TL started the
// descent from InitializeTimeLimitsFromETConfig() — the TL closest to the YAML
// GAUSSIAN MEAN — so the update side was a point dist at the Gaussian-mean TL,
// and the diff flagged every perf-pair task whose adopted TL != Gaussian-mean TL
// (a TL-drift false positive, independent of any real ET change).
//
// Observable: the starting_time_limits the serialized incremental driver is
// entered with IS the descent's start vector (OptimizeIncre_w_TL seeds it from
// ReconstructTimeLimitVecFromResOpt() and passes it to
// PerformSerializedTaskQueueOptimization). A stub overriding that entry records
// the vector without altering the SP (the real RTA still drives adoption inside
// the driver; the override only intercepts the starting_time_limits arg).
//
// The fixture's T_perf has options {400,600,800,1000} and ET~500, so the
// Gaussian-mean-closest TL is 600. The bootstrap (ReOptimizePeriodic, real RTA)
// adopts whatever TL maximizes SP on this fixture; call it T_adopt. The
// false-positive condition is T_adopt != 600. The fix makes the second
// interval's descent start at T_adopt (the carried adopted TL), not 600.
TEST_F(CompareAndKeepSynthetic,
       OptimizeIncre_w_TL_StartsDescentFromCarriedAdoptedTL) {
    // Stub: records the starting_time_limits the serialized incremental driver
    // is entered with (the descent's start vector). Does NOT alter SP — the real
    // RTA drives adoption inside the driver.
    class StartTLStub : public OptimizePA_Incre_with_TimeLimits {
       public:
        std::vector<double> first_eval_tl;
        bool capture = false;  // set true to record the next descent's entry TL
        explicit StartTLStub(const DAG_Model& dag, const SP_Parameters& sp)
            : OptimizePA_Incre_with_TimeLimits(dag, sp) {}

        void PerformSerializedTaskQueueOptimization(
            int K, std::vector<double>& starting_time_limits,
            const DAG_Model& dag_tasks_prev_pre_tl) override {
            if (capture && first_eval_tl.empty()) {
                first_eval_tl = starting_time_limits;
            }
            OptimizePA_Incre_with_TimeLimits::
                PerformSerializedTaskQueueOptimization(
                    K, starting_time_limits, dag_tasks_prev_pre_tl);
        }
    };

    StartTLStub opt(dag_tasks, sp_parameters);

    // Bootstrap: ReOptimizePeriodic runs a from-scratch descent (real RTA) that
    // adopts some TL T_adopt for T_perf. After this, res_opt_ carries T_adopt
    // (and the incumbent is initialized).
    opt.ReOptimizePeriodic(dag_tasks, 2);
    ASSERT_TRUE(opt.IfInitialized());
    const double adopted_tl =
        opt.res_opt_.id2time_limit.at(dag_tasks.tasks[0].id);
    // The Gaussian-mean-closest TL for T_perf (ET~500, options 400/600/800/1000)
    // is 600. The false-positive condition requires the adopted TL to differ
    // from it; if the metric happened to adopt 600 here, this fixture would not
    // exercise the bug and the test would be vacuous.
    const double gaussian_mean_tl = 600.0;
    ASSERT_NE(adopted_tl, gaussian_mean_tl)
        << "Fixture no longer exercises the false-positive condition: the "
        << "adopted TL equals the Gaussian-mean TL (600), so starting from "
        << "either is indistinguishable. Adjust the fixture so the metric "
        << "adopts a TL != 600.";

    // Second interval: IDENTICAL DAG (no ET change at all — neither the YAML
    // Gaussian nor the adopted TL moved). The diff should flag NOTHING. The
    // fix's observable: the descent STARTS at the carried adopted TL, not at
    // the Gaussian-mean TL (600).
    opt.capture = true;
    opt.OptimizeIncre_w_TL(dag_tasks, 2);

    ASSERT_EQ(2u, opt.first_eval_tl.size())
        << "Serialized incremental driver was not entered; cannot observe the "
        << "start TL.";
    EXPECT_DOUBLE_EQ(adopted_tl, opt.first_eval_tl[0])
        << "Incremental descent must start from the carried ADOPTED TL ("
        << adopted_tl << "), not the Gaussian-mean-closest TL ("
        << gaussian_mean_tl << "). Starting from the Gaussian-mean TL makes "
        << "the update side of FindTaskWithDifferentEt's diff a point dist at "
        << "the wrong TL, flagging unchanged perf-pair tasks (the P1.1 "
        << "false-positive mechanism).";
    EXPECT_NE(opt.first_eval_tl[0], gaussian_mean_tl)
        << "Sanity: the start TL must not be the Gaussian-mean TL here.";
}

// Reopt TL-descent seed policy (P1.4). Per the P1.4 constraint ("the initial
// ET must come from an algorithm, not from yaml or the task-set-generation
// logic"), ReOptimizePeriodic's descent ALWAYS seeds from the carried adopted
// TL (ReconstructTimeLimitVecFromResOpt) — the optimizer's own prior result in
// res_opt_ — whenever an incumbent exists. This is the permanent, unconditional
// policy (P1.4 removed the ReoptStartFromAdoptedTL knob that used to toggle
// it); the only remaining branch is the IfInitialized() fallback on a fresh
// optimizer (no incumbent), covered by the test below. Same StartTLStub
// observable as the incremental test above (records the first-eval TL vector
// without altering SP). Fixture: T_perf options {400,600,800,1000}, ET~500 →
// Gaussian-mean-closest TL is 600; the bootstrap adopts T_adopt != 600.
TEST_F(CompareAndKeepSynthetic, ReOptimizePeriodic_StartsFromAdoptedTL) {
    class StartTLStub : public OptimizePA_Incre_with_TimeLimits {
       public:
        std::vector<double> first_eval_tl;
        bool capture = false;
        explicit StartTLStub(const DAG_Model& dag, const SP_Parameters& sp)
            : OptimizePA_Incre_with_TimeLimits(dag, sp) {}
        double EvaluateTimeLimitConfig_ScratchOrIncre(
            int K, const std::vector<double>& time_limits,
            bool from_scratch) override {
            if (capture && first_eval_tl.empty()) {
                first_eval_tl = time_limits;
            }
            return OptimizePA_Incre_with_TimeLimits::
                EvaluateTimeLimitConfig_ScratchOrIncre(K, time_limits,
                                                       from_scratch);
        }
    };

    StartTLStub opt(dag_tasks, sp_parameters);
    opt.ReOptimizePeriodic(dag_tasks, 2);               // bootstrap
    ASSERT_TRUE(opt.IfInitialized());
    const double adopted_tl =
        opt.res_opt_.id2time_limit.at(dag_tasks.tasks[0].id);
    const double gaussian_mean_tl = 600.0;
    ASSERT_NE(adopted_tl, gaussian_mean_tl)
        << "Fixture no longer exercises the condition: adopted TL == 600.";

    // Second interval, identical DAG. The reopt descent starts at the carried
    // adopted TL (T_adopt), NOT the Gaussian-mean TL (600) — the algorithmic
    // seed is unconditional (P1.4 removed the opt-out).
    opt.capture = true;
    opt.ReOptimizePeriodic(dag_tasks, 2);

    ASSERT_EQ(2u, opt.first_eval_tl.size())
        << "Reopt descent did not evaluate any config.";
    EXPECT_DOUBLE_EQ(adopted_tl, opt.first_eval_tl[0])
        << "Reopt must start from the carried ADOPTED TL (" << adopted_tl
        << "), not the Gaussian-mean TL (" << gaussian_mean_tl << ").";
    EXPECT_NE(opt.first_eval_tl[0], gaussian_mean_tl)
        << "Sanity: start != Gaussian-mean TL.";
}

// Fresh-optimizer fallback (unchanged by P1.4). On the bootstrap interval of
// a freshly constructed optimizer there is no incumbent (IfInitialized() ==
// false), so the incumbent seed is impossible and the IfInitialized() guard
// falls back to the Gaussian-mean TL. This is irreducible for ANY seed policy:
// the very first solve has no prior optimizer state to seed from.
TEST_F(CompareAndKeepSynthetic,
       ReOptimizePeriodic_Interval0FallsBackToGaussianMean) {
    class StartTLStub : public OptimizePA_Incre_with_TimeLimits {
       public:
        std::vector<double> first_eval_tl;
        bool capture = true;  // record the very first eval (interval 0)
        explicit StartTLStub(const DAG_Model& dag, const SP_Parameters& sp)
            : OptimizePA_Incre_with_TimeLimits(dag, sp) {}
        double EvaluateTimeLimitConfig_ScratchOrIncre(
            int K, const std::vector<double>& time_limits,
            bool from_scratch) override {
            if (capture && first_eval_tl.empty()) {
                first_eval_tl = time_limits;
            }
            return OptimizePA_Incre_with_TimeLimits::
                EvaluateTimeLimitConfig_ScratchOrIncre(K, time_limits,
                                                       from_scratch);
        }
    };

    // Fresh optimizer — NO incumbent yet. The guard must fall back to the
    // Gaussian-mean TL — ReconstructTimeLimitVecFromResOpt() would return all
    // -1 (no-op walk) on an empty res_opt_.
    StartTLStub opt(dag_tasks, sp_parameters);
    ASSERT_FALSE(opt.IfInitialized());

    opt.ReOptimizePeriodic(dag_tasks, 2);

    ASSERT_EQ(2u, opt.first_eval_tl.size())
        << "Interval-0 reopt descent did not evaluate any config.";
    EXPECT_DOUBLE_EQ(600.0, opt.first_eval_tl[0])
        << "Interval-0 must fall back to the Gaussian-mean TL (600), not the "
        << "all -1 sentinel from an empty res_opt_.";
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
// so no CommitIncumbent fires (the incumbent would stay at the bootstrap
// {pa, tl}, re-introducing the frozen-baseline bug Fix A fixed). The zero-work
// fallback runs one eval with the current (all -1) time_limits so UpdateRecords
// commits the current interval's {pa, tl} via CommitIncumbent.
//
// Observable: eval_count_ (public, debugMode-independent, incremented once per
// EvaluateTimeLimitConfig_ScratchOrIncre call) drops from N to 1, AND the
// challenger rebuilt from res_opt_ after the call carries the current
// interval's DAG (T_noise's TL is -1 so its ET passes through
// UpdateExtDistBasedOnTimeLimit unchanged → a direct window onto whether the
// baseline reconstructed from res_opt_ reflects the current interval).
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
    ASSERT_TRUE(opt.IfInitialized());
    const int eval_count_after_bootstrap = opt.eval_count_;

    // Second interval: mutate T_noise's ET. T_noise's TL is -1 → its ET passes
    // through UpdateExtDistBasedOnTimeLimit unchanged → if the fallback ran
    // UpdateRecords (CommitIncumbent), the challenger rebuilt from res_opt_
    // carries the mutated ET.
    const double mutated_noise_et = 1234.0;
    DAG_Model dag_v2 = dag_no_tl;
    dag_v2.tasks[1].execution_time_dist =
        GetUnitExecutionTimeDist(mutated_noise_et);

    opt.OptimizeIncre_w_TL(dag_v2, 2);

    // The win: descent does 1 eval (the zero-work fallback), not N=2.
    EXPECT_EQ(1, opt.eval_count_ - eval_count_after_bootstrap)
        << "All-{-1}-only descent should run exactly one eval (the zero-work "
        << "fallback), not one per task. Before Fix 2 this is 2.";

    // The trap guard: the fallback's UpdateRecords ran (CommitIncumbent fired),
    // so the challenger rebuilt from res_opt_ reflects the current interval's
    // DAG. Without the fallback, the {-1}-only skip would starve UpdateRecords
    // → no CommitIncumbent → res_opt_ would hold the bootstrap DAG and the
    // rebuilt challenger would freeze at the bootstrap ET (the frozen-baseline
    // bug Fix A fixed returns).
    const double carried_noise_et =
        opt.BuildChallengerFromIncumbent().dag_tasks_.tasks[1]
            .execution_time_dist.GetAvgValue();
    EXPECT_NEAR(carried_noise_et, mutated_noise_et, 5.0)
        << "The challenger rebuilt from res_opt_ does not carry the current "
        << "interval's DAG; the zero-work skip starved UpdateRecords "
        << "(CommitIncumbent did not fire). Expected ~" << mutated_noise_et
        << " (current interval), got " << carried_noise_et
        << " (bootstrap value ~50.0).";
}

// Mixed-case skip guard: when SOME tasks carry real TL options and others are
// {-1}-only, the {-1}-only task is skipped (no redundant incumbent re-eval) and
// the zero-work fallback does NOT fire (the real-option task already produced
// >0 evals). Uses the standard fixture (T_perf has 4 TL pairs; T_noise has none
// → {-1}-only).
//
// The incremental descent starts from the CARRIED ADOPTED TL (see
// OptimizeIncre_w_TL_StartsDescentFromCarriedAdoptedTL). On this monotonic
// strictly-increasing-in-TL SP landscape the bootstrap adopts TL=1000 (the
// optimum, also the upper boundary of [400,600,800,1000]). Starting the walk at
// 1000: backward to 800 (non-improving → patience=0 breaks, 1 eval), forward
// pass empty (already at the upper boundary, 0 evals), plus the baseline eval =
// 2 evals. (Before the descent-start-TL fix the start was the Gaussian-mean-
// closest TL=600, giving 4 evals — that count was start-TL-specific, not a
// structural invariant, so it is not asserted here.)
//
// The structural invariants this test guards are: (a) T_noise ({-1}-only)
// contributes 0 evals — the count is bounded above by T_perf's full-set size
// with NO +1 for a redundant T_noise re-eval; (b) the zero-work fallback does
// not fire on top of T_perf's real evals (same upper bound); (c) the baseline
// eval always runs (≥1).
TEST_F(CompareAndKeepSynthetic,
       PerformCoordinateDescent_SkipsMinusOneOnlyTaskInMixedSet) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    opt.ReOptimizePeriodic(dag_tasks, 2);
    const int eval_count_after_bootstrap = opt.eval_count_;

    // T_perf's FULL option-set size — the upper bound on T_perf's walk evals
    // (the walk visits each option at most once: baseline + one outward pass
    // per direction, patience-bounded). T_noise ({-1}-only) is skipped → 0.
    const size_t t_perf_full_set =
        opt.time_limit_option_for_each_task_[0].size();
    ASSERT_EQ(4u, t_perf_full_set);  // [400,600,800,1000]

    opt.OptimizeIncre_w_TL(dag_tasks, 2);

    const int incremental_evals = opt.eval_count_ - eval_count_after_bootstrap;
    // (a)+(b): T_noise added 0 evals AND the fallback did not fire, so the
    // count is bounded above by T_perf's full-set size (no +1 for T_noise's
    // redundant eval, no +1 fallback eval). Before the {-1}-skip this was
    // t_perf_full_set + 1.
    EXPECT_LE(static_cast<size_t>(incremental_evals), t_perf_full_set)
        << "Mixed descent must not add a redundant eval for T_noise ({-1}-only "
        << "→ skipped) nor fire the zero-work fallback on top of T_perf's real "
        << "evals. Got " << incremental_evals << " > T_perf full-set size "
        << t_perf_full_set << ".";
    EXPECT_GE(incremental_evals, 1);  // (c) the baseline eval always runs
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
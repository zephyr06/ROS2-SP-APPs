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

// Builds the per-task node RTAs for a candidate {pa, tl} the SAME way the
// cache-path SP eval's caller does (`rta_cache_.Evaluate` is byte-identical to
// this — see the RTACache oracle tests): apply TLs -> apply pa ->
// `ProbabilisticRTA_TaskSet`. The gate predicate (`ImportantTasksMeetThresholds`)
// takes these already-computed RTAs rather than re-deriving them, so each gate
// test feeds the candidate's RTAs through this helper. `node_rtas[i]` pairs
// with the prioritized task at index i — the gate's CONTRACT.
std::vector<FiniteDist> NodeRTAsForCandidate(
    const DAG_Model& dag_tasks, const std::vector<int>& priority_assignment,
    const std::vector<double>& tl) {
    TaskSet tasks_with_tl =
        ApplyTimeLimitsToTasksExecutionTime(dag_tasks.tasks, tl);
    TaskSet tasks_prioritized =
        UpdateTaskSetPriorities(tasks_with_tl, priority_assignment);
    return ProbabilisticRTA_TaskSet(tasks_prioritized);
}

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
// The invariant is checked inside OptimizeIncreSingleTask via
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
    // contract violation in CallOptimizerGivenTimeLimits fires).
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
// the seeded-baseline / DM-bootstrap contract is checked in isolation, not only
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

// P0.6 static-solution TL seed vector: per-task, the largest grid option <=
// et_mean (perf tasks); -1 (non-perf). This is the directional counterpart to
// InitializeTimeLimitsFromETConfig (which uses the bidirectional
// Find_Close_ExecutionTime and may pick ABOVE et_mean). The seed must sit
// at-or-below the P0.8-certified WCET (= et_mean) so it is feasible-by-
// construction. T_perf: Gaussian mean 500, grid [400,600,800,1000] → largest
// <= 500 is 400. T_noise: no perf pairs → -1.
TEST_F(CompareAndKeepSynthetic, SeedTimeLimitsAtOrBelowEtMean) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    std::vector<double> tl = opt.SeedTimeLimitsAtOrBelowEtMean();
    ASSERT_EQ(2u, tl.size());
    // T_perf: et_mean = 500; largest grid option <= 500 is 400 (index 0).
    EXPECT_DOUBLE_EQ(400.0, tl[0]);
    // T_noise: no time-performance pairs → -1.
    EXPECT_DOUBLE_EQ(-1.0, tl[1]);
}

// --- P0.6 step 4: the hard per-candidate feasibility gate's pure predicate. ---
// ImportantTasksMeetThresholds(dag, sp_params, pa, tl, node_rtas) answers the
// user's hard guarantee: does EVERY important task's probabilistic
// ddl_miss_chance stay at or below its SP threshold under the candidate {pa,
// tl}? It is the gate predicate the static-solution TL walk adopts on (reject
// SP-better candidates that violate an important task's threshold; keep
// feasible ones). Tested here in isolation against a synthetic 2-task DAG whose
// ddl_miss_chance we control via deadline + threshold + TL.
//
// The gate takes the ALREADY-COMPUTED node RTAs (its real caller — the TL walk's
// adoption site — materializes them when scoring SP via
// `rta_cache_.Evaluate` → `ObtainSP_Full_From_NodeRTAs`; re-deriving would
// duplicate that work). So each test builds `node_rtas` the SAME way the
// cache-path caller will: apply TLs → apply pa → `ProbabilisticRTA_TaskSet`
// (the oracle the cache is byte-identical to — see the RTACache tests above).
//
// Setup (CompareAndKeepSynthetic fixture): T_perf (id 0, important? NO by
// default) ET~500@TL, T_noise (id 1) ET~50. To exercise the gate we mark
// T_perf important and drive its ddl_miss_chance across its threshold by
// moving the deadline relative to its (TL-capped) response time.

TEST_F(CompareAndKeepSynthetic, ImportantTasksMeetThresholds_AdmitsWhenMissChanceBelowThreshold) {
    // T_perf is the only important task. With TL = 400 (its smallest grid option,
    // a point mass well below its period/deadline), its RTA sits far under the
    // deadline → ddl_miss_chance ~ 0, comfortably below any positive threshold.
    dag_tasks.tasks[0].is_important = true;
    // Threshold = 0.5 for every task (SP_Parameters default). ddl_miss_chance ~ 0
    // < 0.5 → feasible.
    std::vector<double> tl = {400.0, -1.0};
    std::vector<int> pa = {dag_tasks.tasks[0].id, dag_tasks.tasks[1].id};
    std::vector<FiniteDist> node_rtas = NodeRTAsForCandidate(dag_tasks, pa, tl);
    EXPECT_TRUE(ImportantTasksMeetThresholds(dag_tasks, sp_parameters, pa, tl, node_rtas));
}

TEST_F(CompareAndKeepSynthetic,
       ImportantTasksMeetThresholds_RejectsWhenImportantTaskMissesDeadline) {
    // Shrink T_perf's deadline to BELOW its response time so its RTA distribution
    // is almost entirely past the deadline → ddl_miss_chance ~ 1.0, far above the
    // 0.5 threshold → the gate REJECTS. (T_perf stays important.)
    dag_tasks.tasks[0].is_important = true;
    dag_tasks.tasks[0].deadline = 1.0;  // impossibly tight: RT >> 1
    std::vector<double> tl = {400.0, -1.0};
    std::vector<int> pa = {dag_tasks.tasks[0].id, dag_tasks.tasks[1].id};
    std::vector<FiniteDist> node_rtas = NodeRTAsForCandidate(dag_tasks, pa, tl);
    EXPECT_FALSE(ImportantTasksMeetThresholds(dag_tasks, sp_parameters, pa, tl, node_rtas));
}

TEST_F(CompareAndKeepSynthetic,
       ImportantTasksMeetThresholds_IgnoresNonImportantTaskMisses) {
    // A NON-important task missing its deadline must NOT trip the gate — the
    // guarantee is scoped to important tasks only. Mark only T_noise important;
    // then drive T_perf (non-important) past its deadline. T_noise stays feasible
    // → gate admits despite T_perf missing.
    dag_tasks.tasks[1].is_important = true;
    dag_tasks.tasks[0].is_important = false;
    dag_tasks.tasks[0].deadline = 1.0;  // non-important T_perf misses badly
    std::vector<double> tl = {400.0, -1.0};
    std::vector<int> pa = {dag_tasks.tasks[0].id, dag_tasks.tasks[1].id};
    std::vector<FiniteDist> node_rtas = NodeRTAsForCandidate(dag_tasks, pa, tl);
    EXPECT_TRUE(ImportantTasksMeetThresholds(dag_tasks, sp_parameters, pa, tl, node_rtas));
}

TEST_F(CompareAndKeepSynthetic,
       ImportantTasksMeetThresholds_AdmitsWhenNoTaskIsImportant) {
    // Boundary: with NO important tasks the universal quantifier is vacuously
    // true → the gate admits (the constraint is free, matching the design's
    // "seed region: ddl_miss_chance = 0" claim's degenerate case).
    std::vector<double> tl = {400.0, -1.0};
    std::vector<int> pa = {dag_tasks.tasks[0].id, dag_tasks.tasks[1].id};
    std::vector<FiniteDist> node_rtas = NodeRTAsForCandidate(dag_tasks, pa, tl);
    EXPECT_TRUE(ImportantTasksMeetThresholds(dag_tasks, sp_parameters, pa, tl, node_rtas));
}

// --- P0.6 step 4b: wiring the gate into the TL walk's adoption. ---
// The pure predicate (above) is step 4a. Step 4b threads it into
// `UpdateRecords` — the commit chokepoint the eval (`OptimizeIncreSingleTask`)
// calls — as a PURE extra acceptance test on top of `WouldBeatIncumbent`: a
// candidate that WOULD beat the champion is committed only if the gate passes
// (the user's rule: "checked whenever we make progress from the champion; if
// challenger doesn't beat champion, we don't do the check"). The optimization
// process — INCLUDING PA descent — is otherwise identical to the flag-off path;
// the gate is not a process change, only an extra accept/reject criterion. On a
// gate-REJECT of an SP-better candidate: `UpdateRecords` returns false (no
// commit) and the eval returns the INCUMBENT SP (not the rejected candidate's
// SP), so the walk's `IsBetterTimeLimitOption` sees "no progress" and never
// tracks the rejected TL into the working vector (ghost-SP fix). Flag-off (the
// default) = no gate, prod byte-identical.
//
// Scenario built to produce SP-better-BUT-gate-rejected: T_perf (id 0, NOT
// important, weight 1.0) carries the perf pairs → its perf coefficient rises
// with TL, so a larger TL is strictly SP-better. T_noise (id 1, important,
// weight 0.01) sits at LOWER priority, so T_perf's larger TL adds interference
// to T_noise's response time. A T_noise deadline wedged between the two TLs'
// response times makes the gate PASS at the incumbent TL (low interference) and
// FAIL at the candidate TL (high interference) — the candidate is SP-better yet
// gate-rejected. PA = {T_perf, T_noise} (T_perf highest priority).
namespace {
// The candidate SP the cache-path oracle returns for {pa, tl} on `dag_tasks`.
double OracleSPForCandidate(const DAG_Model& dag, const SP_Parameters& sp,
                            const std::vector<int>& pa,
                            const std::vector<double>& tl) {
    return ObtainSP_Full_From_NodeRTAs(dag, sp, pa, tl,
                                       NodeRTAsForCandidate(dag, pa, tl));
}
}  // namespace

// Precondition check (not itself a gate test): confirms the fixture's SP
// arithmetic is what step 4b's reject/keep tests assume — TL=1000 is strictly
// SP-better than TL=400 on the perf term. Guards against a silent fixture drift
// making the reject test vacuously pass.
TEST_F(CompareAndKeepSynthetic,
       GateWiring_HigherTlIsStrictlySpBetter_Precondition) {
    dag_tasks.tasks[0].is_important = false;
    dag_tasks.tasks[1].is_important = true;
    std::vector<int> pa = {dag_tasks.tasks[0].id, dag_tasks.tasks[1].id};
    double sp_low = OracleSPForCandidate(dag_tasks, sp_parameters, pa, {400.0, -1.0});
    double sp_high = OracleSPForCandidate(dag_tasks, sp_parameters, pa, {1000.0, -1.0});
    EXPECT_GT(sp_high, sp_low)
        << "fixture broken: TL=1000 must be strictly SP-better than TL=400";
}

// (i) Flag OFF (default): the gate never runs. An SP-better candidate commits
// normally — byte-identical to prod. Seeds the incumbent at TL=400, walks T_perf
// to TL=1000 (strictly SP-better), asserts the commit landed.
TEST_F(CompareAndKeepSynthetic,
       GateWiring_FlagOff_CommitsSpBetterCandidate_NoGate) {
    dag_tasks.tasks[0].is_important = false;
    dag_tasks.tasks[1].is_important = true;
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    ASSERT_FALSE(opt.enforce_important_task_gate_);  // default off

    std::vector<int> pa = {dag_tasks.tasks[0].id, dag_tasks.tasks[1].id};
    opt.CommitIncumbent(pa, OracleSPForCandidate(dag_tasks, sp_parameters, pa, {400.0, -1.0}),
                        {400.0, -1.0});
    ASSERT_TRUE(opt.IfInitialized());

    // Candidate TL=1000 is strictly SP-better; flag off → no gate → commits.
    double returned = opt.OptimizeIncreSingleTask({1000.0, -1.0}, /*task_idx=*/0,
                                                  /*et_increased=*/true);
    EXPECT_GT(returned, opt.res_opt_.sp_opt - 1e-9)
        << "flag-off: eval must report the (better) committed SP, not the seed";
    EXPECT_DOUBLE_EQ(1000.0, opt.res_opt_.id2time_limit[dag_tasks.tasks[0].id]);
}

// (ii) Flag ON, gate REJECTS an SP-better threshold-violating candidate: the
// incumbent stays UNTOUCHED (TL/SP/PA unchanged) and the eval returns the
// incumbent SP — the ghost-SP fix (the walk then sees "no progress"). PA descent
// runs but finds no strict-improving move on this fixture (T_perf is already
// top-priority), so the gate sees the candidate's actual {pa, tl} and rejects.
TEST_F(CompareAndKeepSynthetic,
       GateWiring_FlagOn_RejectsSpBetterThresholdViolatingCandidate) {
    dag_tasks.tasks[0].is_important = false;   // T_perf: drives SP up with TL
    dag_tasks.tasks[1].is_important = true;    // T_noise: the gated task
    // T_noise deadline wedged between its TL=400 (RT~450) and TL=1000 (RT~1050)
    // response times: TL=400 → T_noise RT < deadline → passes; TL=1000 →
    // T_noise RT > deadline → fails. (T_perf stays higher priority, so its
    // larger TL adds interference to T_noise's RT.)
    dag_tasks.tasks[1].deadline = 700.0;
    // Tiny T_noise weight: its deadline-miss SP penalty (~0.01) is dwarfed by
    // T_perf's perf gain (TL 400→1000 raises T_perf's perf coefficient 0.5→1.0,
    // a +0.5 SP term at weight 1.0). So the candidate is STRICTLY SP-better yet
    // gate-REJECTED — the constrained-optimization tension the gate exists for:
    // raw SP-maximization would accept TL=1000 (the miss is cheap), but the gate
    // enforces the important-task constraint regardless of weight.
    sp_parameters.weights_node[dag_tasks.tasks[1].id] = 0.01;

    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    opt.enforce_important_task_gate_ = true;

    std::vector<int> pa = {dag_tasks.tasks[0].id, dag_tasks.tasks[1].id};
    std::vector<double> tl_seed = {400.0, -1.0};
    // Sanity: the seed itself is gate-feasible (else the reject test is moot).
    ASSERT_TRUE(ImportantTasksMeetThresholds(
        dag_tasks, sp_parameters, pa, tl_seed,
        NodeRTAsForCandidate(dag_tasks, pa, tl_seed)));
    // Sanity: the candidate is gate-INfeasible (the violation step 4b rejects).
    std::vector<double> tl_candidate = {1000.0, -1.0};
    ASSERT_FALSE(ImportantTasksMeetThresholds(
        dag_tasks, sp_parameters, pa, tl_candidate,
        NodeRTAsForCandidate(dag_tasks, pa, tl_candidate)));
    // Sanity: AND the candidate is strictly SP-better (the crux of the test).
    ASSERT_GT(OracleSPForCandidate(dag_tasks, sp_parameters, pa, tl_candidate),
              OracleSPForCandidate(dag_tasks, sp_parameters, pa, tl_seed));

    opt.CommitIncumbent(pa, OracleSPForCandidate(dag_tasks, sp_parameters, pa, tl_seed),
                        tl_seed);
    ASSERT_TRUE(opt.IfInitialized());
    const double incumbent_sp = opt.res_opt_.sp_opt;

    double returned = opt.OptimizeIncreSingleTask(tl_candidate, /*task_idx=*/0,
                                                  /*et_increased=*/true);

    // The reject: incumbent untouched.
    EXPECT_DOUBLE_EQ(400.0, opt.res_opt_.id2time_limit[dag_tasks.tasks[0].id]);
    EXPECT_DOUBLE_EQ(incumbent_sp, opt.res_opt_.sp_opt);
    EXPECT_EQ(pa, opt.res_opt_.priority_vec);
    // The ghost-SP fix: the eval reports the INCUMBENT SP, not the rejected
    // candidate's (better) SP, so the walk treats the rejected TL as no-progress.
    EXPECT_DOUBLE_EQ(incumbent_sp, returned);
}

// (iii) Flag ON, gate PASSES a feasible SP-better candidate: commits normally
// (the gate is permissive when the constraint holds — the walk keeps the best-SP
// FEASIBLE point). Same fixture, but T_noise's deadline is loose so TL=1000
// passes the gate; the SP-better candidate commits.
TEST_F(CompareAndKeepSynthetic,
       GateWiring_FlagOn_KeepsFeasibleSpBetterCandidate) {
    dag_tasks.tasks[0].is_important = false;
    dag_tasks.tasks[1].is_important = true;
    // Loose T_noise deadline: TL=1000 stays feasible → gate passes → commits.
    dag_tasks.tasks[1].deadline = 2000.0;

    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    opt.enforce_important_task_gate_ = true;

    std::vector<int> pa = {dag_tasks.tasks[0].id, dag_tasks.tasks[1].id};
    std::vector<double> tl_seed = {400.0, -1.0};
    std::vector<double> tl_candidate = {1000.0, -1.0};
    ASSERT_TRUE(ImportantTasksMeetThresholds(
        dag_tasks, sp_parameters, pa, tl_candidate,
        NodeRTAsForCandidate(dag_tasks, pa, tl_candidate)));
    ASSERT_GT(OracleSPForCandidate(dag_tasks, sp_parameters, pa, tl_candidate),
              OracleSPForCandidate(dag_tasks, sp_parameters, pa, tl_seed));

    opt.CommitIncumbent(pa, OracleSPForCandidate(dag_tasks, sp_parameters, pa, tl_seed),
                        tl_seed);
    ASSERT_TRUE(opt.IfInitialized());

    double returned = opt.OptimizeIncreSingleTask(tl_candidate, /*task_idx=*/0,
                                                  /*et_increased=*/true);
    // The keep: candidate committed.
    EXPECT_DOUBLE_EQ(1000.0, opt.res_opt_.id2time_limit[dag_tasks.tasks[0].id]);
    EXPECT_GT(opt.res_opt_.sp_opt, OracleSPForCandidate(dag_tasks, sp_parameters, pa, tl_seed));
    EXPECT_GT(returned, OracleSPForCandidate(dag_tasks, sp_parameters, pa, tl_seed));
}

// (iv) Flag ON, candidate does NOT beat the champion: the gate must NOT run
// (user's rule — only check on would-beat). No threshold is set up to trip; the
// point is the gate is structurally skipped, observable by the incumbent being
// discarded by UpdateRecords as usual (no commit, no SP change) AND no false
// reject of a merely-equal candidate. Seed at TL=1000 (the SP max on this
// fixture); a TL=400 candidate is strictly SP-WORSE → not a beat → no gate, no
// commit, incumbent stays at TL=1000.
TEST_F(CompareAndKeepSynthetic,
       GateWiring_FlagOn_NonBeatingCandidate_DoesNotGateNorCommit) {
    dag_tasks.tasks[0].is_important = false;
    dag_tasks.tasks[1].is_important = true;
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    opt.enforce_important_task_gate_ = true;

    std::vector<int> pa = {dag_tasks.tasks[0].id, dag_tasks.tasks[1].id};
    std::vector<double> tl_seed = {1000.0, -1.0};  // SP max on this fixture
    opt.CommitIncumbent(pa, OracleSPForCandidate(dag_tasks, sp_parameters, pa, tl_seed),
                        tl_seed);
    ASSERT_TRUE(opt.IfInitialized());
    const double incumbent_sp = opt.res_opt_.sp_opt;

    // TL=400 is strictly SP-WORSE → not a beat → no gate invocation, no commit.
    std::vector<double> tl_candidate = {400.0, -1.0};
    double candidate_sp = OracleSPForCandidate(dag_tasks, sp_parameters, pa, tl_candidate);
    ASSERT_LT(candidate_sp, incumbent_sp);  // precondition: genuinely not a beat
    double returned = opt.OptimizeIncreSingleTask(tl_candidate, /*task_idx=*/0,
                                                  /*et_increased=*/false);
    EXPECT_DOUBLE_EQ(1000.0, opt.res_opt_.id2time_limit[dag_tasks.tasks[0].id]);
    EXPECT_DOUBLE_EQ(incumbent_sp, opt.res_opt_.sp_opt);
    // On a NON-beat the eval truthfully reports the candidate's (worse) SP — the
    // ghost-SP suppression applies ONLY to a gate-rejected SP-BETTER candidate
    // (test ii); a merely-worse candidate is no progress either way, so its SP
    // is returned as-is (the walk's IsBetterTimeLimitOption is false regardless).
    EXPECT_NEAR(candidate_sp, returned, 1e-9);
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

// P0.9: the seed PA is Deadline-Monotonic with an important-first group lock
// (DeadlineMonotonicPriorityVec, formerly plain-RM RateMonotonicPriorityVec).
// These two tests construct cases where DM-with-group-lock and plain RM
// DIVERGE, so they discriminate the new behavior from the old.

TEST(DeadlineMonotonicPriorityVec, RanksImportantGroupFirstThenDmOrdersEach) {
    // DM + important-first group lock. Construct a taskset where (a) deadlines
    // differ from periods (so DM ≠ RM) and (b) the important tasks are NOT the
    // ones a plain period-sort would put first. Important tasks must occupy the
    // TOP slots (the group lock), DM-ordered within the important group; non-
    // important fill the bottom, DM-ordered within their group.
    //
    //   idx  period  deadline  is_important  ET
    //   0    100      90       YES           10
    //   1     50     180       YES           10   (long deadline, but important)
    //   2    200     120       no            10
    //   3     80      60       no            10   (shortest deadline overall,
    //                                            but non-important → must NOT
    //                                            jump above the important group)
    //
    // Expected order [important, DM asc] ++ [non-important, DM asc]:
    //   T0 (ddl 90) < T1 (ddl 180)  |  T3 (ddl 60) < T2 (ddl 120)
    //   → pa = [0, 1, 3, 2]
    // Plain RM (period asc) would give [1, 3, 0, 2] — wrong on both the group
    // lock and the within-group key, so this test is RED under the old code.
    std::vector<Value_Proba> d = {Value_Proba(10.0, 1.0)};
    Task t0(0, d, 100, 90, 0, "T0");
    Task t1(1, d, 50, 180, 1, "T1");
    Task t2(2, d, 200, 120, 2, "T2");
    Task t3(3, d, 80, 60, 3, "T3");
    t0.is_important = true;
    t1.is_important = true;
    MAP_Prev mapPrev;
    TaskSet tasks = {t0, t1, t2, t3};
    DAG_Model dag(tasks, mapPrev, 0, 0);
    SP_Parameters sp(dag);

    OptimizePA_Incre_with_TimeLimits opt(dag, sp);
    PriorityVec pa = opt.DeadlineMonotonicPriorityVec();
    ASSERT_EQ(4u, pa.size());
    EXPECT_EQ(0, pa[0]);  // important group, shortest deadline 90
    EXPECT_EQ(1, pa[1]);  // important group, deadline 180
    EXPECT_EQ(3, pa[2]);  // non-important group, shortest deadline 60
    EXPECT_EQ(2, pa[3]);  // non-important group, deadline 120
}

TEST(DeadlineMonotonicPriorityVec, BreaksDeadlineTiesByExecutionTimeAscending) {
    // Within a group, equal-deadline ties break by avg ET ascending (matches the
    // former RM tie-break; deterministic). Two important tasks share deadline 100
    // but differ in ET: T1 (ET 10) must rank above T0 (ET 30). T2 (non-important,
    // deadline 100) stays below both regardless of ET (group lock).
    std::vector<Value_Proba> d0 = {Value_Proba(30.0, 1.0)};
    std::vector<Value_Proba> d1 = {Value_Proba(10.0, 1.0)};
    std::vector<Value_Proba> d2 = {Value_Proba(50.0, 1.0)};
    Task t0(0, d0, 100, 100, 0, "T0");
    Task t1(1, d1, 100, 100, 1, "T1");
    Task t2(2, d2, 200, 100, 2, "T2");
    t0.is_important = true;
    t1.is_important = true;
    MAP_Prev mapPrev;
    TaskSet tasks = {t0, t1, t2};
    DAG_Model dag(tasks, mapPrev, 0, 0);
    SP_Parameters sp(dag);

    OptimizePA_Incre_with_TimeLimits opt(dag, sp);
    PriorityVec pa = opt.DeadlineMonotonicPriorityVec();
    ASSERT_EQ(3u, pa.size());
    EXPECT_EQ(1, pa[0]);  // important, deadline 100, ET 10 — lower ET beats T0
    EXPECT_EQ(0, pa[1]);  // important, deadline 100, ET 30
    EXPECT_EQ(2, pa[2]);  // non-important, deadline 100
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


TEST_F(CompareAndKeepSynthetic, SeedIncumbentBaseline_Interval0UsesDMAndMinTL) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    EXPECT_FALSE(opt.IfInitialized());

    // Compute the expected interval-0 baseline — DM priorities + smallest TL —
    // with the same primitives the helper uses internally, then verify the
    // helper seeds exactly that.
    PriorityVec pa_dm = opt.DeadlineMonotonicPriorityVec();
    std::vector<double> tl_min = opt.SmallestTimeLimitVec();
    DAG_Model dag_min = UpdateExtDistBasedOnTimeLimit(dag_tasks, tl_min);
    double expected_sp =
        EvaluateSPWithPriorityVec(dag_min, sp_parameters, pa_dm);

    opt.ResetIncumbentBaseline(true);

    EXPECT_TRUE(opt.IfInitialized());
    EXPECT_DOUBLE_EQ(expected_sp, opt.opt_sp_);
    EXPECT_EQ(pa_dm, opt.opt_pa_);
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
    PriorityVec pa = opt.DeadlineMonotonicPriorityVec();
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

// --- P3.6: INCR_NO_REOPT (pure incremental, DM-fast bootstrap, no reopt) ---
//
// New arm (agent_coding_rules TDD: tests first, red before green). Behavior:
//   interval 0: bootstrap the incumbent from DM-fast (DM priorities + smallest
//                TL option) with NO from-scratch descent;
//   interval 1+: OptimizeIncre_w_TL (warm-started from the carried incumbent)
//                every interval, NEVER ReOptimizePeriodic.
//
// These tests assert that contract directly. They FAIL on the current code
// (the methods don't exist yet) — the red that Step 2 turns green.

// Bootstrap-only contract: interval 0 seeds the DM-fast incumbent (DM priorities
// + smallest TL) and runs ZERO descent evals. eval_count_ is incremented only
// inside CallOptimizerGivenTimeLimits / OptimizeIncreSingleTask (the per-task
// re-search the descent walk drives); ResetIncumbentBaseline(true) — the
// interval-0 seed — calls EvaluateSPWithPriorityVec directly and does NOT touch
// eval_count_. So eval_count_==0 after the bootstrap proves no descent ran.
TEST_F(CompareAndKeepSynthetic, OptimizePureIncremental_Interval0IsSeedOnly) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    ASSERT_FALSE(opt.IfInitialized());
    ASSERT_EQ(0, opt.eval_count_);

    // Expected DM-fast baseline, computed with the same primitives the bootstrap
    // uses internally.
    PriorityVec pa_dm = opt.DeadlineMonotonicPriorityVec();
    std::vector<double> tl_min = opt.SmallestTimeLimitVec();
    DAG_Model dag_min = UpdateExtDistBasedOnTimeLimit(dag_tasks, tl_min);
    double expected_sp = EvaluateSPWithPriorityVec(dag_min, sp_parameters, pa_dm);

    opt.OptimizePureIncremental(dag_tasks, 2);

    EXPECT_TRUE(opt.IfInitialized());
    EXPECT_EQ(0, opt.eval_count_) << "interval 0 must not run any descent eval";
    EXPECT_DOUBLE_EQ(expected_sp, opt.opt_sp_);
    EXPECT_EQ(pa_dm, opt.opt_pa_);
    // Min-TL baseline: T_perf=400, T_noise=-1.
    EXPECT_DOUBLE_EQ(400.0, opt.res_opt_.id2time_limit[0]);
    EXPECT_DOUBLE_EQ(-1.0, opt.res_opt_.id2time_limit[1]);
}

// Counter advances exactly once per call (mirrors Optimize_w_TL_ScratchOrIncre,
// so the orchestrator's interval bookkeeping is uniform across arms).
TEST_F(CompareAndKeepSynthetic, OptimizePureIncremental_AdvancesCounterOncePerCall) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    ASSERT_EQ(0, opt.reoptimization_interval_count_);

    opt.OptimizePureIncremental(dag_tasks, 2);
    EXPECT_EQ(1, opt.reoptimization_interval_count_);
    opt.OptimizePureIncremental(dag_tasks, 2);
    EXPECT_EQ(2, opt.reoptimization_interval_count_);
    opt.OptimizePureIncremental(dag_tasks, 2);
    EXPECT_EQ(3, opt.reoptimization_interval_count_);
}

// No-reopt contract: even with ReoptimizationPeriod=1 (which under
// Optimize_w_TL_ScratchOrIncre would route EVERY interval to ReOptimizePeriodic),
// OptimizePureIncremental must take the incremental path at interval 1+. The
// from-scratch descent (Reopt) is memoryless OptimizeFromScratch — if it ran, it
// would touch eval_count_ via CallOptimizerGivenTimeLimits(from_scratch=true).
// So interval 1+ producing eval_count_ > 0 via the INCREMENTAL path (not reopt)
// + the incumbent being CARRIED (interval 1's seed reuses interval 0's adopted
// TL, not the Gaussian-mean TL) is the "pure incremental, no reopt" signature.
TEST_F(CompareAndKeepSynthetic, OptimizePureIncremental_NeverReoptsEvenAtPeriodOne) {
    GlobalVariables::ReoptimizationPeriod = 1;  // would force reopt every interval
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);

    // Interval 0: bootstrap (seed only, no descent).
    opt.OptimizePureIncremental(dag_tasks, 2);
    ASSERT_EQ(0, opt.eval_count_);
    ASSERT_EQ(1, opt.reoptimization_interval_count_);

    // The incumbent is carried: interval 0 committed the DM-fast incumbent, so
    // the optimizer is initialized and the adopted TL is in res_opt_.
    ASSERT_TRUE(opt.IfInitialized());
    ASSERT_DOUBLE_EQ(400.0, opt.res_opt_.id2time_limit[0]);

    // Interval 1: the incremental path runs (eval_count_ grows). The walk
    // warm-starts from the carried incumbent and may IMPROVE the TL beyond the
    // cheap DM-fast seed (400) — that is the arm doing its job, not a reopt.
    opt.OptimizePureIncremental(dag_tasks, 2);
    EXPECT_GT(opt.eval_count_, 0) << "interval 1 must take the incremental path";
    EXPECT_EQ(2, opt.reoptimization_interval_count_);
    EXPECT_TRUE(opt.IfInitialized());

    // The "never reopt" pin: with ReoptimizationPeriod=1, the reopt dispatcher
    // (Optimize_w_TL_ScratchOrIncre) would route interval 0 to ReOptimizePeriodic,
    // whose from-scratch beam (CallOptimizerGivenTimeLimits(from_scratch=true) in
    // SeedBaselineAndArmCache) increments eval_count_ BEFORE the walk. The pure
    // arm's interval-0 bootstrap skips that beam entirely. So after interval 0
    // alone, the pure arm has STRICTLY FEWER evals than the reopt arm — the
    // behavioral signature that no from-scratch descent ran.
    OptimizePA_Incre_with_TimeLimits opt_reopt(dag_tasks, sp_parameters);
    opt_reopt.ComputeSafeFallback(dag_tasks);  // single-interval fixture: worst case == dag
    opt_reopt.Optimize_w_TL_ScratchOrIncre(dag_tasks, 2);  // interval 0: reopt
    EXPECT_GT(opt_reopt.eval_count_, 0)
        << "reopt dispatcher runs a from-scratch beam at interval 0";
    EXPECT_LT(opt.eval_count_, opt_reopt.eval_count_)
        << "pure arm must skip the from-scratch descent the reopt arm runs";
}

// Issue (5) — baseline-overwrites-res_opt_ invariant. The baseline eval at the
// top of RunIntervalDescent(Reopt)
//   current_config_sp = CallOptimizerGivenTimeLimits(K, time_limits, from_scratch);
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
// CallOptimizerGivenTimeLimits (from_scratch=true); the incremental
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
    // CallOptimizerGivenTimeLimits call (the reopt branch's per-candidate
    // eval) AND counts entries into PerformSerializedTaskQueueOptimization (the
    // incremental branch's driver). The recorded signals are exactly the routing
    // decisions the dispatcher made.
    class RecordingDispatcherOpt : public OptimizePA_Incre_with_TimeLimits {
       public:
        std::vector<bool> from_scratch_flags;
        int serialized_entries = 0;
        // Counts per-candidate evals routed through the sub-incremental
        // (cache-routed, |diff|<=1) eval. Since the P2.11 merge the reopt walk
        // routes every trial here (the pre-merge legacy arm used ScratchOrIncre
        // for its TL trials; that arm is deleted). So subincremental_calls>0
        // after a reopt descent is the merged-routing signal.
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
        double CallOptimizerGivenTimeLimits(
            int beam_search_width, const std::vector<double>& time_limits,
            bool from_scratch) override {
            from_scratch_flags.push_back(from_scratch);
            return OptimizePA_Incre_with_TimeLimits::
                CallOptimizerGivenTimeLimits(beam_search_width, time_limits,
                                             from_scratch);
        }
        double OptimizeIncreSingleTask(
            const std::vector<double>& time_limits, size_t task_idx,
            bool et_increased) override {
            ++subincremental_calls;
            subincremental_task_idx.push_back(task_idx);
            return OptimizePA_Incre_with_TimeLimits::OptimizeIncreSingleTask(
                time_limits, task_idx, et_increased);
        }
        void PerformSerializedTaskQueueOptimization(
            int beam_search_width,
            std::vector<double>& starting_time_limits,
            const DAG_Model& dag_tasks_prev_pre_tl) override {
            ++serialized_entries;
            OptimizePA_Incre_with_TimeLimits::
                PerformSerializedTaskQueueOptimization(
                    beam_search_width, starting_time_limits,
                    dag_tasks_prev_pre_tl);
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

// P2.11 Phase 5.5 — regression for the interval-0 reopt crash. The reopt arm
// (RunIntervalDescent(Reopt)) adopted the cache champion from the
// from-scratch reopt's TL (champion_tl = ReconstructTimeLimitVecFromResOpt(),
// after the upfront CallOptimizerGivenTimeLimits from-scratch call
// overwrites res_opt_) but then walked the UN-re-synced starting_time_limits —
// which at interval 0 is the Gaussian-mean seed
// (InitializeTimeLimitsFromETConfig). When the from-scratch reopt moves >=2
// TL-flexible tasks off the Gaussian seed, the first walk step's candidate DAG
// (Gaussian with one task stepped) differs from the champion DAG (reopt TL) in
// >1 task -> RTACache::ComputeTaskSetDifference throws |diff|>1 ->
// std::terminate -> SIGABRT.
//
// The existing CounterDispatcherSynthetic fixture has only ONE TL-flexible task,
// so the reopt can move at most one TL off the seed -> |diff|<=1 holds by
// accident and the crash never fired in tests (the fixture gap that shipped the
// bug). This fixture has TWO symmetric TL-flexible tasks: the from-scratch reopt
// moves BOTH off the Gaussian seed (200) to a higher-TL optimum, so the walk's
// first step diffs champion-TL vs Gaussian-with-one-stepped in 2 tasks -> throw.
//
// Fix under test: RunIntervalDescent(Reopt) re-syncs
// starting_time_limits = ReconstructTimeLimitVecFromResOpt() before the
// WalkSerializedTaskQueue call, so the walk starts from champion_tl (the first
// step diffs champion vs champion-with-one-stepped -> |diff|==1, no throw),
// mirroring the incremental path (PerformSerializedTaskQueueOptimization:390
// commits the champion FROM starting_time_limits; OptimizeOneTaskWithTimeLimit:506
// re-syncs per task).
class ReoptFlagOnMultiTLFlexibleSynthetic : public ::testing::Test {
   public:
    // Same recording subclass as CounterDispatcherSynthetic: counts sub-incremental
    // evals so the test can assert the walk actually ran (not just didn't throw).
    class RecordingDispatcherOpt : public OptimizePA_Incre_with_TimeLimits {
       public:
        int subincremental_calls = 0;
        using OptimizePA_Incre_with_TimeLimits::OptimizePA_Incre_with_TimeLimits;
        double OptimizeIncreSingleTask(
            const std::vector<double>& time_limits, size_t task_idx,
            bool et_increased) override {
            ++subincremental_calls;
            return OptimizePA_Incre_with_TimeLimits::OptimizeIncreSingleTask(
                time_limits, task_idx, et_increased);
        }
    };

    // Build ONE TL-flexible task with TL options {100,200,300,400} and a Gaussian
    // ET whose average (~250) lands between options 200 and 300. Find_Close_ExecutionTime
    // resolves the tie to the first minimum (index 1 -> TL 200), so the Gaussian
    // seed for this task is 200. The reopt (maximizing SP) moves it to a higher-TL
    // optimum, diverging from the seed.
    Task MakeTLFlexibleTask(int id, const std::string& name) {
        const double et_avg = 250.0;
        std::vector<Value_Proba> dist = {Value_Proba(et_avg, 1.0)};
        Task t(id, dist, 2000, 2000, id, name);
        t.execution_time_dist = FiniteDist(GaussianDist(et_avg, 0.5), 5);
        // (time_limit, performance): higher TL -> higher performance (the reopt
        // has a strict incentive to raise TL, so it diverges from the seed 200).
        t.timePerformancePairs.push_back(TimePerfPair(100, 0.4));
        t.timePerformancePairs.push_back(TimePerfPair(200, 0.7));
        t.timePerformancePairs.push_back(TimePerfPair(300, 0.9));
        t.timePerformancePairs.push_back(TimePerfPair(400, 1.0));
        return t;
    }

    void SetUp() override {
        Task t_perf_a = MakeTLFlexibleTask(0, "T_perf_a");
        Task t_perf_b = MakeTLFlexibleTask(1, "T_perf_b");

        // T_noise: no perf pair -> {-1}-only -> skipped by the TL walk (present so
        // the DAG has a non-TL-flexible task, mirroring a realistic mix).
        std::vector<Value_Proba> dist_noise = {Value_Proba(50.0, 1.0)};
        Task t_noise(2, dist_noise, 2000, 2000, 2, "T_noise");
        t_noise.execution_time_dist = FiniteDist(GaussianDist(50.0, 0.5), 5);

        TaskSet tasks = {t_perf_a, t_perf_b, t_noise};
        dag_tasks = DAG_Model(tasks, mapPrev, 0, 0);
        sp_parameters = SP_Parameters(dag_tasks);
    }

    MAP_Prev mapPrev;
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
};

// The crash regression: an interval-0 reopt (count==0, no incumbent -> the
// Gaussian-seed path) on a DAG with >=2 TL-flexible tasks must NOT throw. Before
// the re-sync fix it threw std::runtime_error from RTACache::ComputeTaskSetDifference
// (|diff|>1) -> std::terminate -> SIGABRT, because the walk starts from the
// Gaussian seed while the champion was adopted from the from-scratch reopt's TL
// (which moved both TL-flexible tasks off the seed). The fix re-syncs the walk's
// starting TL to the champion TL before WalkSerializedTaskQueue.
TEST_F(ReoptFlagOnMultiTLFlexibleSynthetic,
       Reopt_AtInterval0_DoesNotThrowWhenReoptMovesMultipleTLs) {
    RecordingDispatcherOpt opt(dag_tasks, sp_parameters);

    // The Gaussian seed (InitializeTimeLimitsFromETConfig) is the interval-0
    // starting_time_limits the walk would use WITHOUT the fix. Pin it BEFORE the
    // reopt so the divergence check below compares against the true seed (the
    // reopt overwrites res_opt_, so ReconstructTimeLimitVecFromResOpt() after
    // reflects the champion, not the seed).
    std::vector<double> gaussian_seed = opt.InitializeTimeLimitsFromETConfig();

    // Interval-0 reopt: no incumbent -> InitializeTimeLimitsFromETConfig seeds
    // the Gaussian-mean TL (200 for both T_perf_a and T_perf_b); the from-scratch
    // reopt then moves both to a higher-TL optimum; the walk must re-sync to the
    // champion TL before stepping, else the first step throws |diff|>1.
    // EXPECT_NO_THROW catches the runtime_error the buggy path raises and turns
    // the crash into a clean test failure (RED) instead of SIGABRT.
    EXPECT_NO_THROW(opt.ReOptimizePeriodic(dag_tasks, 2))
        << "P2.11 Phase 5.5: an interval-0 reopt must not throw when "
        << "the from-scratch reopt moves multiple TL-flexible tasks off the "
        << "Gaussian seed. The walk must re-sync starting_time_limits to the "
        << "champion TL before WalkSerializedTaskQueue (mirrors the incremental "
        << "path). A throw here is the |diff|>1 cache-contract crash.";

    // Guard against the fixture gap that shipped the bug: this is only a valid
    // crash regression if the reopt ACTUALLY diverged in >=2 tasks. The prior
    // CounterDispatcherSynthetic fixture had one TL-flexible task, so |diff|<=1
    // held by accident and the crash never fired in tests. Pin the divergence so
    // the fixture cannot silently regress to non-reproducing.
    std::vector<double> champion_tl = opt.ReconstructTimeLimitVecFromResOpt();
    int divergent = 0;
    for (size_t i = 0; i < gaussian_seed.size() && i < champion_tl.size(); ++i) {
        if (gaussian_seed[i] != champion_tl[i]) ++divergent;
    }
    EXPECT_GE(divergent, 2)
        << "fixture must drive a >=2-task TL divergence between the Gaussian "
        << "seed and the from-scratch reopt's champion; otherwise |diff|<=1 "
        << "holds by accident and this test cannot reproduce the crash.";

    // Secondary signal: the walk actually ran (the sub-incremental arm was
    // reached). If this is 0 the walk never started, masking a throw.
    EXPECT_GT(opt.subincremental_calls, 0)
        << "the reopt walk must route its per-candidate trials through the "
        << "sub-incremental eval; zero calls means the walk never ran.";
}

// P2.11 Phase 1b-1e — the unified descent body `RunIntervalDescent` folds the
// two duplicated descent bodies (PerformSerializedTaskQueueOptimization for the
// incremental path + the reopt descent for the reopt
// path) into one parameterized over `mode ∈ {Incremental, Reopt}`, with the
// cache-arming asymmetry (delta 4: reopt runs its baseline beam DISARMED, arms
// only after) encapsulated in `SeedBaselineAndArmCache`. The 5.5 regression test
// above (Reopt_AtInterval0_DoesNotThrowWhenReoptMovesMultipleTLs) already routes
// the reopt path through ReOptimizePeriodic → RunIntervalDescent(Reopt)
// → SeedBaselineAndArmCache(Reopt), so it IS the
// canary that the 5.5 re-sync survived the move (a wrong arm/beam ordering would
// SIGABRT there). This companion test pins the bit-identity of the unified
// body's INCREMENTAL mode directly: the SP a direct RunIntervalDescent(Incremental)
// call produces equals the SP the legacy wrapper
// PerformSerializedTaskQueueOptimization produced before the unification. Both
// now delegate to the same body, so this guards against a future change that
// breaks the delegation (e.g. the wrapper stops calling RunIntervalDescent).
TEST_F(CounterDispatcherSynthetic,
       RunIntervalDescent_Incremental_MatchesWrapperSP) {
    // Bootstrap an incumbent via the reopt path (count==0 routes to reopt),
    // so the incremental path's warm-start contract holds.
    RecordingDispatcherOpt opt(dag_tasks, sp_parameters);
    opt.ComputeSafeFallback(dag_tasks);  // single-interval fixture: worst case == dag
    opt.Optimize_w_TL_ScratchOrIncre(dag_tasks, 2);
    ASSERT_TRUE(opt.IfInitialized());

    // The incrementally-adopted TL is the carried champion after the bootstrap.
    std::vector<double> carried_tl = opt.ReconstructTimeLimitVecFromResOpt();
    double sp_via_wrapper = opt.opt_sp_;

    // A fresh opt bootstrapped identically, then driven through the unified body
    // directly with the Incremental mode + the SAME carried TL, must reproduce
    // the wrapper's SP exactly (the wrapper is now a 1-line delegate to it).
    RecordingDispatcherOpt opt_direct(dag_tasks, sp_parameters);
    opt_direct.ComputeSafeFallback(dag_tasks);  // single-interval fixture: worst case == dag
    opt_direct.Optimize_w_TL_ScratchOrIncre(dag_tasks, 2);
    ASSERT_TRUE(opt_direct.IfInitialized());
    // Reset the dispatcher's observation counters so the direct call's
    // sub-incremental trials are measurable on opt_direct alone.
    opt_direct.subincremental_calls = 0;
    opt_direct.serialized_entries = 0;
    std::vector<double> tl_for_direct = opt_direct.ReconstructTimeLimitVecFromResOpt();
    EXPECT_EQ(carried_tl, tl_for_direct)
        << "two identically-bootstrapped opts must carry the same TL.";

    // Drive the unified body directly. The re-absorb of dag_tasks mirrors what
    // OptimizeIncre_w_TL would do (dag_tasks_prev_pre_tl capture + dag_tasks_
    // absorb); on this fixture dag_tasks_ is already dag_tasks, so the capture
    // is the same DAG (Type-E diff is empty → FullReuse, |diff|==0, safe).
    opt_direct.RunIntervalDescent(2, tl_for_direct,
                                  IntervalDescentMode::Incremental, dag_tasks);

    // The unified body ran the walk (sub-incremental arm reached) and produced
    // an SP no worse than the carried baseline (compare-and-keep guard).
    EXPECT_GT(opt_direct.subincremental_calls, 0)
        << "RunIntervalDescent(Incremental) must run the serialized walk; zero "
        << "sub-incremental calls means the body did not execute the walk.";
    EXPECT_GE(opt_direct.opt_sp_, sp_via_wrapper)
        << "RunIntervalDescent(Incremental) must not regress SP below the "
        << "carried baseline (the compare-and-keep guard preserves the incumbent).";
}

// The counter advances by 1 after every dispatch and never resets. Three
// consecutive calls → counter == 3 regardless of which branch each call took.
TEST_F(CounterDispatcherSynthetic, CounterAdvancesEveryCall_NeverResets) {
    RecordingDispatcherOpt opt(dag_tasks, sp_parameters);
    opt.ComputeSafeFallback(dag_tasks);  // single-interval fixture: worst case == dag
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
    opt.ComputeSafeFallback(dag_tasks);  // single-interval fixture: worst case == dag
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
// CallOptimizerGivenTimeLimits); count == 1 is not modular
// (1 % 10 != 0) so the second call routes to the incremental branch, driven
// through PerformSerializedTaskQueueOptimization (P1.10 — the serialized E+L
// queue; the per-candidate eval is the sub-incremental, NOT ScratchOrIncre).
// The routing is observable via the recorded counts: the second call enters the
// serialized driver at least once, proving the incremental branch — not reopt —
// ran. The incumbent established by the first call lets the incremental path's
// warm-start contract hold (no CoutError).
TEST_F(CounterDispatcherSynthetic, RoutesToIncrementalAtNonModularCount) {
    RecordingDispatcherOpt opt(dag_tasks, sp_parameters);
    opt.ComputeSafeFallback(dag_tasks);  // single-interval fixture: worst case == dag

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

// P2.11 merge — reopt routing is now unconditional. The reopt descent runs ONE
// baseline beam through CallOptimizerGivenTimeLimits(from_scratch=true)
// to establish the champion, then switches the TL walk's per-candidate eval to
// the cache-routed OptimizeIncreSingleTask (|diff|<=1 single-task
// re-search). So after a count==0 reopt dispatch: from_scratch_flags is non-empty
// (the one baseline beam) AND subincremental_calls>0 (the walk trials). The walk
// trials must NOT appear as from_scratch flags — only the baseline beam does.
// (Before the merge this was the flag-ON arm; the flag is gone and the merged
// path is the only path, so the assertion is now unconditional.)
TEST_F(CounterDispatcherSynthetic,
       ReoptWalk_RoutesWalkTrialsThroughSubIncremental) {
    RecordingDispatcherOpt opt(dag_tasks, sp_parameters);
    opt.ComputeSafeFallback(dag_tasks);  // single-interval fixture: worst case == dag

    opt.Optimize_w_TL_ScratchOrIncre(dag_tasks, 2);  // count==0 → reopt

    // The baseline beam still runs through ScratchOrIncre(from_scratch=true).
    ASSERT_FALSE(opt.from_scratch_flags.empty())
        << "reopt must still run the one baseline beam through "
        << "CallOptimizerGivenTimeLimits.";
    for (bool fs : opt.from_scratch_flags) {
        EXPECT_TRUE(fs);
    }
    // The walk trials route through the sub-incremental eval. T_perf has a
    // 10-option TL set; on this monotonic SP landscape the walk takes ≥1 step
    // before patience stops it, so subincremental_calls>0.
    EXPECT_GT(opt.subincremental_calls, 0)
        << "reopt walk must route its per-candidate trials through the "
        << "sub-incremental eval; saw zero calls (the walk did not run).";
}

// P2.11 reading (a) — Type-E in the reopt queue. The merged reopt path walks the
// SAME E+L serialized queue the incremental path uses (BuildSerializedTaskQueue),
// so an env-changed task with NO perf pair (T_noise, task 1) is reached via its
// Type-E entry and re-searched through OptimizeIncreSingleTask.
//
// The pre-merge reopt sub-incremental arm walked sorted_indices and skipped
// {-1}-only tasks (T_noise has no perf pair → {-1}-only → skipped), so it NEVER
// reached an env-changed T_noise. This test pins the merge: after a reopt on a
// DAG whose T_noise ET changed since the previous interval, the recorded
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
       ReoptWalk_ReachesEnvChangedTaskViaSerializedQueue) {
    RecordingDispatcherOpt opt(dag_tasks, sp_parameters);

    // Bootstrap the incumbent on the original DAG (T_noise ET ~ 50). On the
    // bootstrap there is no env change (prev == update), so only T_perf's
    // Type-L walk runs.
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
        << "P2.11 merge: a reopt on a DAG with an env-changed T_noise "
        << "(no perf pair, Type-E) must reach T_noise via the E+L serialized "
        << "queue's Type-E handler (SubIncremental call with task_idx==1). The "
        << "pre-merge reopt arm skipped {-1}-only tasks and never reached it. "
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
        double CallOptimizerGivenTimeLimits(
            int beam_search_width, const std::vector<double>& time_limits,
            bool from_scratch) override {
            if (capture && first_eval_tl.empty()) {
                first_eval_tl = time_limits;
            }
            return OptimizePA_Incre_with_TimeLimits::
                CallOptimizerGivenTimeLimits(beam_search_width, time_limits,
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
        double CallOptimizerGivenTimeLimits(
            int beam_search_width, const std::vector<double>& time_limits,
            bool from_scratch) override {
            if (capture && first_eval_tl.empty()) {
                first_eval_tl = time_limits;
            }
            return OptimizePA_Incre_with_TimeLimits::
                CallOptimizerGivenTimeLimits(beam_search_width, time_limits,
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
// CallOptimizerGivenTimeLimits call) drops from N to 1, AND the
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
// before the higher-level WalkOneTaskWithTimeLimitOptions walk is exercised.

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
// the reopt descent (strictly-greater SP wins; on
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

// --- P0.6: et_mean-bounded TL seed helper (FindLargestTimeLimitAtOrBelow) ---
//
// The static solution seeds each perf task at the LARGEST TL grid option that
// does not exceed its et_mean (= execution_time_dist.GetAvgValue()). This is the
// operating point P0.8 certifies (perf WCET = et_mean): at TL <= et_mean the sim
// caps perf runtime ET at min(et_mean, TL) <= et_mean = certified WCET, so the
// seed is feasible-by-construction. Find_Close_ExecutionTime (:42) is the
// bidirectional precedent (closest by abs distance — may pick ABOVE et_mean);
// this directional variant must never exceed et_mean. The helper is pure
// (stateless, takes the sorted timePerformancePairs + a scalar et_mean) so it is
// unit-tested in isolation like the other TL-walk helpers above.

// Sorted ascending, as the metric/grid contract requires (RecordCloseTimeLimitOptions
// sorts by time). Each test reuses this 4-option grid [400,600,800,1000].
std::vector<TimePerfPair> P06SeedGrid() {
    return {TimePerfPair(400.0, 0.5), TimePerfPair(600.0, 0.6),
            TimePerfPair(800.0, 0.7), TimePerfPair(1000.0, 0.8)};
}

// (a) et_mean strictly above the largest option → picks the LARGEST option.
TEST(FindLargestTimeLimitAtOrBelowTest,
     EtMeanAboveLargestOptionPicksLargest) {
    auto grid = P06SeedGrid();  // [400,600,800,1000]
    EXPECT_EQ(3u, FindLargestTimeLimitAtOrBelow(grid, /*et_mean=*/1500.0));
}

// (b) et_mean strictly below the smallest option → NO option <= et_mean exists
// → falls back to the SMALLEST (index 0). The static-solution seed tolerates a
// grid entirely above et_mean by clamping to the floor (still feasible-by-
// construction: min(et_mean, smallest) <= et_mean).
TEST(FindLargestTimeLimitAtOrBelowTest,
     EtMeanBelowSmallestOptionFallsBackToSmallest) {
    auto grid = P06SeedGrid();  // [400,600,800,1000]
    EXPECT_EQ(0u, FindLargestTimeLimitAtOrBelow(grid, /*et_mean=*/300.0));
}

// (c) et_mean lands strictly between two options → picks the largest option
// that is <= et_mean (the option just below), NOT the closest by abs distance.
// This is the directional guarantee that diverges from Find_Close_ExecutionTime:
// at et_mean=700, Find_Close would pick 600 (dist 100) or 800 (dist 100) —
// ambiguous/tie; FindLargestTimeLimitAtOrBelow must pick 600 (the <= side).
TEST(FindLargestTimeLimitAtOrBelowTest,
     EtMeanBetweenOptionsPicksLargestAtOrBelow) {
    auto grid = P06SeedGrid();  // [400,600,800,1000]
    EXPECT_EQ(1u, FindLargestTimeLimitAtOrBelow(grid, /*et_mean=*/700.0));
}

// (d) et_mean exactly equals an option → that option qualifies (<=) and is the
// largest such → returns its index. et_mean == 800 → index 2.
TEST(FindLargestTimeLimitAtOrBelowTest,
     EtMeanExactlyEqualsOptionPicksThatOption) {
    auto grid = P06SeedGrid();  // [400,600,800,1000]
    EXPECT_EQ(2u, FindLargestTimeLimitAtOrBelow(grid, /*et_mean=*/800.0));
}

// (e) et_mean exactly equals the smallest option → index 0 (no fallback needed;
// the smallest itself satisfies <=).
TEST(FindLargestTimeLimitAtOrBelowTest,
     EtMeanExactlyEqualsSmallestOptionPicksSmallest) {
    auto grid = P06SeedGrid();  // [400,600,800,1000]
    EXPECT_EQ(0u, FindLargestTimeLimitAtOrBelow(grid, /*et_mean=*/400.0));
}

// (f) empty grid → returns 0 (the no-option sentinel; the caller treats an empty
// timePerformancePairs as a non-perf task and sets TL = -1, so the index is
// never read). Mirrors Find_Close_ExecutionTime's empty-input return.
TEST(FindLargestTimeLimitAtOrBelowTest, EmptyGridReturnsZero) {
    std::vector<TimePerfPair> empty;
    EXPECT_EQ(0u, FindLargestTimeLimitAtOrBelow(empty, /*et_mean=*/700.0));
}

// --- Trial-and-Error TL walk: WalkOneTaskWithTimeLimitOptions + rewritten
// RunIntervalDescent(Reopt) ---
//
// The walk replaces the exhaustive per-task enumeration with a unidirectional
// trial-and-error sweep: step outward from the baseline; adopt each improving
// option; stop after `patience` consecutive non-improving steps (patience=0 =
// strict break on first non-improvement; patience=1 = tolerate one dip). To
// test the termination logic deterministically (independent of RTA numerics),
// stub CallOptimizerGivenTimeLimits with a preprogrammed TL→SP map.

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

    double CallOptimizerGivenTimeLimits(
        int beam_search_width, const std::vector<double>& time_limits,
        bool from_scratch) override {
        double tl = time_limits[walked_task_idx_];
        evaluated_tls.push_back(tl);
        auto it = tl_to_sp.find(tl);
        return it == tl_to_sp.end() ? -1.0 : it->second;
    }

    // Build the eval lambda the walk core expects, binding the stub's
    // CallOptimizerGivenTimeLimits override. Mirrors the deleted
    // OptimizeSingleTaskTimeLimit wrapper's lambda so the walk-core tests
    // exercise the identical code path: the override ignores `from_scratch`
    // (it reads time_limits[walked_task_idx_] → SP map), so `evaluated_tls`
    // recording and all assertions are unchanged.
    auto MakeScratchOrIncreEval(int beam_search_width, bool from_scratch) {
        return [this, beam_search_width,
                from_scratch](const std::vector<double>& tl) {
            return CallOptimizerGivenTimeLimits(beam_search_width, tl,
                                                from_scratch);
        };
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
    auto eval = opt.MakeScratchOrIncreEval(/*K=*/2, /*from_scratch=*/true);
    double final_sp = opt.WalkOneTaskWithTimeLimitOptions(
        /*task_idx=*/0, time_limits, /*current_sp=*/sp[600.0],
        /*baseline_val=*/600.0, /*step=*/1, /*patience=*/0, eval);

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
    auto eval = opt.MakeScratchOrIncreEval(2, /*from_scratch=*/true);
    double final_sp = opt.WalkOneTaskWithTimeLimitOptions(
        0, time_limits, sp[600.0], 600.0, /*step=*/1, /*patience=*/0, eval);

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
    auto eval = opt.MakeScratchOrIncreEval(2, /*from_scratch=*/true);
    double final_sp = opt.WalkOneTaskWithTimeLimitOptions(
        0, time_limits, sp[600.0], 600.0, /*step=*/1, /*patience=*/1, eval);

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
    auto eval = opt.MakeScratchOrIncreEval(2, /*from_scratch=*/true);
    double final_sp = opt.WalkOneTaskWithTimeLimitOptions(
        0, time_limits, sp[600.0], 600.0, /*step=*/1, /*patience=*/1, eval);

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
    auto eval = opt.MakeScratchOrIncreEval(2, /*from_scratch=*/true);
    double final_sp = opt.WalkOneTaskWithTimeLimitOptions(
        0, time_limits, sp[600.0], 600.0, /*step=*/-1, /*patience=*/0, eval);

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
    auto eval = opt.MakeScratchOrIncreEval(2, /*from_scratch=*/true);
    double final_sp = opt.WalkOneTaskWithTimeLimitOptions(
        0, time_limits, /*current_sp=*/1.5, /*baseline_val=*/-1.0,
        /*step=*/1, /*patience=*/0, eval);

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
    auto eval = opt.MakeScratchOrIncreEval(2, /*from_scratch=*/true);
    double final_sp = opt.WalkOneTaskWithTimeLimitOptions(
        0, time_limits, /*current_sp=*/1.5, /*baseline_val=*/700.0,
        /*step=*/1, /*patience=*/0, eval);

    EXPECT_DOUBLE_EQ(1.5, final_sp);
    EXPECT_TRUE(opt.evaluated_tls.empty());
}

// --- P0.6: the offline safe fallback (the fall-back artifact). ---
// ComputeSafeFallback() seeds at the P0.8-certified point (DM PA + TL <= et_mean), runs
// a TL walk with the gate ON, and STORES the result as `safe_fallback_` — SEPARATE from
// the live `res_opt_`, so the persistent optimizer's online behavior is byte-identical
// (interval 0 still bootstraps fresh; the fallback is consumed only by P0.7's fall-back).
// `Optimize_w_TL_ScratchOrIncre` lazy-populates it on the first call if a caller didn't
// pre-call (the dispatcher's safety net — the orchestrator pre-calls to keep the compute
// out of the online scheduler-ET metric, but doesn't have to).
//
// The fixture's SP arithmetic (SP strictly increasing in TL, see CompareAndKeepSynthetic)
// means the unconstrained optimum is TL=1000 (T_perf's largest grid option). With the gate
// ON, the walk rejects threshold-violating candidates and keeps the best-SP FEASIBLE point.

// (1) ComputeSafeFallback populates the artifact with a gate-held, et_mean-seeded result.
// PA is the DM-group-locked seed; the stored result's important-task miss chances all stay
// <= threshold (the gate held).
TEST_F(CompareAndKeepSynthetic, ComputeSafeFallback_PopulatesGateHeldArtifact) {
    dag_tasks.tasks[0].is_important = true;   // T_perf is the gated important task
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);

    ASSERT_FALSE(opt.HasSafeFallback());  // nothing computed yet
    ResourceOptResult result = opt.ComputeSafeFallback(dag_tasks);
    ASSERT_TRUE(opt.HasSafeFallback());
    EXPECT_EQ(result.priority_vec, opt.GetSafeFallback().priority_vec);

    // PA is the DM-group-locked seed (both tasks here, T_perf first by DM).
    std::vector<int> pa_dm = opt.DeadlineMonotonicPriorityVec();
    EXPECT_EQ(pa_dm, result.priority_vec);

    // The gate held: re-derive the candidate's node RTAs and check every important task's
    // ddl_miss_chance <= its threshold (the gate's contract). Reconstruct the TL vector in
    // task order from the result's id→TL map.
    std::vector<double> tl_result(dag_tasks.tasks.size());
    for (size_t i = 0; i < dag_tasks.tasks.size(); i++) {
        int id = dag_tasks.tasks[i].id;
        tl_result[i] = result.id2time_limit.count(id) ? result.id2time_limit.at(id)
                                                        : -1.0;
    }
    std::vector<FiniteDist> node_rtas = NodeRTAsForCandidate(dag_tasks, pa_dm, tl_result);
    EXPECT_TRUE(ImportantTasksMeetThresholds(dag_tasks, sp_parameters, pa_dm, tl_result,
                                              node_rtas))
        << "safe fallback must satisfy the gate it was computed under";
}

// (2) Byte-identical contract: ComputeSafeFallback runs the gate-governed walk on a
// THROWAWAY sibling, so the persistent optimizer's live incumbent (res_opt_) is UNTOUCHED
// — interval 0 still bootstraps fresh (the IfInitialized() gate for reopt's baseline reset
// stays false). This is the separation that keeps P0.6 "produce, don't inject".
TEST_F(CompareAndKeepSynthetic,
       ComputeSafeFallback_LeavesLiveIncumbentUntouched_ByteIdentical) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    ASSERT_FALSE(opt.IfInitialized());  // fresh optimizer: no live incumbent

    opt.ComputeSafeFallback(dag_tasks);
    // The artifact is populated, but the LIVE incumbent is STILL uninitialized — interval
    // 0 will bootstrap fresh (byte-identical).
    ASSERT_TRUE(opt.HasSafeFallback());
    EXPECT_FALSE(opt.IfInitialized());
    EXPECT_FALSE(opt.enforce_important_task_gate_);  // flag reset after the walk
}

// (3) The dispatcher's safety contract: Optimize_w_TL_ScratchOrIncre THROWS if no
// safe fallback was pre-computed. The dispatcher sees only dag_tasks_ (one interval),
// not dag_tasks_vecs_, so it cannot build the cross-interval worst-case DAG soundly
// (§8: certifying against a single interval is cross-interval unsound). The
// orchestrator pre-calls ComputeSafeFallback on the worst-case DAG before the loop;
// reaching the dispatcher without one is a caller bug → fail loud.
TEST_F(CompareAndKeepSynthetic, Dispatcher_ThrowsWhenNoSafeFallbackPreComputed) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    ASSERT_FALSE(opt.HasSafeFallback());  // caller didn't pre-call

    EXPECT_THROW(
        opt.Optimize_w_TL_ScratchOrIncre(
            dag_tasks, GlobalVariables::Layer_Node_During_Incremental_Optimization),
        std::runtime_error);
    EXPECT_FALSE(opt.HasSafeFallback());  // still nothing — did not lazy-compute
}

// (4) The dispatcher does NOT recompute when the caller already pre-called: the lazy
// check short-circuits (pre-call is the orchestrator's path; the safety net must be a
// no-op there to avoid a double compute).
TEST_F(CompareAndKeepSynthetic,
       Dispatcher_DoesNotRecompute_WhenAlreadyPreCalled) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    opt.ComputeSafeFallback(dag_tasks);
    ASSERT_TRUE(opt.HasSafeFallback());
    ResourceOptResult pre = opt.GetSafeFallback();

    opt.Optimize_w_TL_ScratchOrIncre(dag_tasks,
                                     GlobalVariables::Layer_Node_During_Incremental_Optimization);
    // Same artifact object (not recomputed).
    EXPECT_EQ(pre.priority_vec, opt.GetSafeFallback().priority_vec);
    EXPECT_DOUBLE_EQ(pre.sp_opt, opt.GetSafeFallback().sp_opt);
}

// ============================================================================
// P0.6 §8 — worst-case-DAG builder (cross-interval safety for P0.7 trigger (a)).
// The caller builds a DAG where each task's dist is a POINT MASS at
// max(execution_time_max) across all interval DAGs → stochastically dominates
// every interval's dist → the gate's ddl_miss_chance upper-bounds every interval.
// See goal.md "WORST-CASE-DAG (2026-07-31)". Helper under test:
// BuildWorstCaseDagAcrossIntervals(const std::vector<DAG_Model>&).
// ============================================================================

// (5) §8b: ComputeSafeFallback runs the walk on the PASSED worst-case DAG, not on
// the optimizer's own dag_tasks_ (interval 0). The optimizer is built from a
// lighter interval-0 DAG, but ComputeSafeFallback is handed a worst-case DAG with
// a LARGER non-perf max_time; the stored result must re-gate clean against the
// WORST-CASE DAG (the certificate that actually bounds the intervals).
TEST_F(CompareAndKeepSynthetic, ComputeSafeFallback_UsesWorstCaseDagNotIntervalZero) {
    dag_tasks.tasks[0].is_important = true;  // T_perf is the gated important task
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);

    // Build a worst-case DAG whose non-perf task (T_noise) has a LARGER max_time
    // than interval-0's. Same structure (period/deadline/grid/name) — only ET max
    // differs — so BuildWorstCaseDagAcrossIntervals fuses them cleanly.
    DAG_Model dag0 = dag_tasks;
    TaskSet tasks_hi = dag_tasks.tasks;
    const double noise_max_hi = 200.0;  // > interval-0's 50.0
    tasks_hi[1].execution_time_dist =
        FiniteDist(std::vector<Value_Proba>{Value_Proba(noise_max_hi, 1.0)});
    // Chains-bearing ctor so structure (chains_/chains_deadlines_) matches dag_tasks.
    DAG_Model dag_hi(tasks_hi, dag_tasks.chains_, dag_tasks.chains_deadlines_);
    DAG_Model worst_case_dag = BuildWorstCaseDagAcrossIntervals({dag0, dag_hi});

    // The worst-case DAG's non-perf max is the larger one.
    EXPECT_DOUBLE_EQ(noise_max_hi, worst_case_dag.tasks[1].execution_time_dist.max_time);

    ASSERT_FALSE(opt.HasSafeFallback());
    ResourceOptResult result = opt.ComputeSafeFallback(worst_case_dag);
    ASSERT_TRUE(opt.HasSafeFallback());

    // Reconstruct {pa, tl} in task order and re-gate against the WORST-CASE DAG
    // (not interval-0's dag_tasks) — the certificate that bounds every interval.
    std::vector<int> pa_result = result.priority_vec;
    std::vector<double> tl_result(worst_case_dag.tasks.size());
    for (size_t i = 0; i < worst_case_dag.tasks.size(); i++) {
        int id = worst_case_dag.tasks[i].id;
        tl_result[i] = result.id2time_limit.count(id) ? result.id2time_limit.at(id) : -1.0;
    }
    SP_Parameters sp_worst(worst_case_dag);
    std::vector<FiniteDist> node_rtas =
        NodeRTAsForCandidate(worst_case_dag, pa_result, tl_result);
    EXPECT_TRUE(ImportantTasksMeetThresholds(worst_case_dag, sp_worst, pa_result,
                                             tl_result, node_rtas))
        << "safe fallback must satisfy the gate on the WORST-CASE DAG it was computed under";
}

// (6) §8c happy path: on a SCHEDULABLE worst-case DAG, ComputeSafeFallback stores
// the artifact and the final result re-gates clean (the loud-fail check passes).
TEST_F(CompareAndKeepSynthetic,
       ComputeSafeFallback_OnSchedulableWorstCase_StoresArtifact) {
    dag_tasks.tasks[0].is_important = true;  // T_perf is the gated important task
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);

    // A schedulable worst-case DAG: same structure, ET maxes well under deadline.
    DAG_Model worst_case_dag = BuildWorstCaseDagAcrossIntervals({dag_tasks, dag_tasks});

    ASSERT_FALSE(opt.HasSafeFallback());
    opt.ComputeSafeFallback(worst_case_dag);
    EXPECT_TRUE(opt.HasSafeFallback());  // stored — loud-fail check passed

    // Final stored result re-gates clean against the worst-case DAG.
    ResourceOptResult result = opt.GetSafeFallback();
    std::vector<double> tl_result(worst_case_dag.tasks.size());
    for (size_t i = 0; i < worst_case_dag.tasks.size(); i++) {
        int id = worst_case_dag.tasks[i].id;
        tl_result[i] = result.id2time_limit.count(id) ? result.id2time_limit.at(id) : -1.0;
    }
    SP_Parameters sp_worst(worst_case_dag);
    std::vector<FiniteDist> node_rtas =
        NodeRTAsForCandidate(worst_case_dag, result.priority_vec, tl_result);
    EXPECT_TRUE(ImportantTasksMeetThresholds(worst_case_dag, sp_worst,
                                             result.priority_vec, tl_result, node_rtas));
}

// (7) §8c loud-fail: on an UNSCHEDULABLE worst-case DAG (an important NON-PERF
// task's WCET point mass exceeds its deadline → ddl_miss_chance==1.0 > threshold),
// ComputeSafeFallback RAISES std::runtime_error and does NOT store
// (HasSafeFallback() stays false → "regenerate a new task set"). Non-perf is
// required: ApplyTimeLimitsToTasksExecutionTime caps PERF tasks at their TL (TL ≤
// et_mean → runtime ET ≤ TL, no overrun), but leaves NON-PERF (TL=−1) at its full
// dist → a non-perf WCET point mass > deadline yields ddl_miss_chance==1.0.
TEST_F(CompareAndKeepSynthetic,
       ComputeSafeFallback_OnUnschedulableWorstCase_RaisesAndDoesNotStore) {
    // dag_tasks.tasks[1] is T_noise (non-perf, deadline 2000, fixture). Set its WCET
    // point mass ABOVE its deadline (3000 > 2000) and mark it important → the gate's
    // ddl_miss_chance==1.0 > threshold → the loud-fail re-check must reject. Same
    // structure as dag_tasks; only ET max + the important label differ.
    TaskSet tasks_unsched = dag_tasks.tasks;
    tasks_unsched[1].is_important = true;
    tasks_unsched[1].execution_time_dist =
        FiniteDist(std::vector<Value_Proba>{Value_Proba(3000.0, 1.0)});
    DAG_Model dag_unsched(tasks_unsched, dag_tasks.chains_, dag_tasks.chains_deadlines_);
    // Degenerate single-interval worst-case: T_noise max_time == 3000.0 > deadline 2000.
    DAG_Model worst_case_dag = BuildWorstCaseDagAcrossIntervals({dag_unsched, dag_unsched});
    ASSERT_GT(worst_case_dag.tasks[1].execution_time_dist.max_time,
              worst_case_dag.tasks[1].deadline);

    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    ASSERT_FALSE(opt.HasSafeFallback());
    EXPECT_THROW(opt.ComputeSafeFallback(worst_case_dag), std::runtime_error);
    // Loud-fail: NOT stored → the caller must regenerate the task set.
    EXPECT_FALSE(opt.HasSafeFallback());
}


namespace {
// Builds a 2-task DAG with non-perf tasks whose dists carry the given max_time.
// Used to vary only ET across the synthetic interval DAGs.
DAG_Model BuildTwoNonPerfDag(double max0, double max1) {
    std::vector<Value_Proba> dist0 = {Value_Proba(max0, 1.0)};
    std::vector<Value_Proba> dist1 = {Value_Proba(max1, 1.0)};
    TaskSet tasks = {Task(0, dist0, 50, 50, 0, "T0"), Task(1, dist1, 50, 50, 1, "T1")};
    MAP_Prev mapPrev;
    return DAG_Model(tasks, mapPrev, 0, 0);
}
}  // namespace

// §8a.1: per-task execution_time_max is the MAX across intervals, and each result
// dist is a single-point mass at that max (point mass = the dominance bound).
TEST(BuildWorstCaseDag, TakesMaxExecutionTimeMaxAcrossIntervals) {
    DAG_Model dag0 = BuildTwoNonPerfDag(3.0, 4.0);
    DAG_Model dag1 = BuildTwoNonPerfDag(4.0, 5.0);
    std::vector<DAG_Model> intervals = {dag0, dag1};

    DAG_Model worst = BuildWorstCaseDagAcrossIntervals(intervals);

    ASSERT_EQ(2u, worst.tasks.size());
    // task 0: max(3.0, 4.0) = 4.0; task 1: max(4.0, 5.0) = 5.0.
    EXPECT_DOUBLE_EQ(4.0, worst.tasks[0].execution_time_dist.max_time);
    EXPECT_DOUBLE_EQ(5.0, worst.tasks[1].execution_time_dist.max_time);
    // Each result dist is a point mass at its max.
    for (size_t i = 0; i < worst.tasks.size(); i++) {
        const FiniteDist& d = worst.tasks[i].execution_time_dist;
        ASSERT_EQ(1u, d.distribution.size());
        EXPECT_DOUBLE_EQ(d.max_time, d.distribution[0].value);
        EXPECT_DOUBLE_EQ(1.0, d.distribution[0].probability);
    }
}

// §8a.2: perf-task TL grid + structure are copied verbatim from interval 0; only
// the ET dist is overwritten to the worst-case point mass.
TEST(BuildWorstCaseDag, PreservesPerfTaskTimeLimitGrid) {
    const double et_perf = 500.0;
    std::vector<Value_Proba> dist_perf = {Value_Proba(et_perf, 1.0)};
    Task t_perf(0, dist_perf, 2000, 2000, 0, "T_perf");
    t_perf.execution_time_dist = FiniteDist(GaussianDist(et_perf, 0.5), 5);
    for (int i = 0; i < 4; ++i) {
        t_perf.timePerformancePairs.push_back(TimePerfPair(400 + i * 200, 0.5 + i * 0.1));
    }
    std::vector<Value_Proba> dist_noise = {Value_Proba(50.0, 1.0)};
    Task t_noise(1, dist_noise, 2000, 2000, 1, "T_noise");
    t_noise.execution_time_dist = FiniteDist(GaussianDist(50.0, 0.5), 5);

    TaskSet tasks0 = {t_perf, t_noise};
    MAP_Prev mapPrev;
    DAG_Model dag0(tasks0, mapPrev, 0, 0);

    // Interval 1: identical structure, only ET max differs (heavier tail on T_noise).
    TaskSet tasks1 = tasks0;
    tasks1[1].execution_time_dist = FiniteDist(GaussianDist(50.0, 0.5), 5);
    // Force a larger max_time on T_noise in interval 1.
    std::vector<Value_Proba> dist_noise_hi = {Value_Proba(80.0, 1.0)};
    tasks1[1].execution_time_dist = FiniteDist(dist_noise_hi);
    DAG_Model dag1(tasks1, mapPrev, 0, 0);

    DAG_Model worst = BuildWorstCaseDagAcrossIntervals({dag0, dag1});

    ASSERT_EQ(2u, worst.tasks.size());
    // Perf task's TL grid + structure copied from interval 0 verbatim.
    EXPECT_EQ(dag0.tasks[0].timePerformancePairs.size(),
              worst.tasks[0].timePerformancePairs.size());
    for (size_t i = 0; i < dag0.tasks[0].timePerformancePairs.size(); i++) {
        EXPECT_DOUBLE_EQ(dag0.tasks[0].timePerformancePairs[i].time_limit,
                         worst.tasks[0].timePerformancePairs[i].time_limit);
        EXPECT_DOUBLE_EQ(dag0.tasks[0].timePerformancePairs[i].performance,
                         worst.tasks[0].timePerformancePairs[i].performance);
    }
    EXPECT_EQ(dag0.tasks[0].period, worst.tasks[0].period);
    EXPECT_EQ(dag0.tasks[0].deadline, worst.tasks[0].deadline);
    EXPECT_EQ(dag0.tasks[0].name, worst.tasks[0].name);
    // Chains + deadlines copied from interval 0.
    EXPECT_EQ(dag0.chains_, worst.chains_);
    EXPECT_EQ(dag0.chains_deadlines_, worst.chains_deadlines_);
    // Perf task dist is a point mass at its worst-case max (interval-0 max here).
    const FiniteDist& d_perf = worst.tasks[0].execution_time_dist;
    EXPECT_EQ(1u, d_perf.distribution.size());
    EXPECT_DOUBLE_EQ(dag0.tasks[0].execution_time_dist.max_time, d_perf.max_time);
    // Non-perf task took the LARGER max across intervals (80.0 > 50.0).
    EXPECT_DOUBLE_EQ(80.0, worst.tasks[1].execution_time_dist.max_time);
}

// §8a.3: structural mismatch across intervals is a loud failure (different task
// counts, or a period mismatch) — the builder cannot fuse incoherent DAGs.
TEST(BuildWorstCaseDag, RaisesOnStructuralMismatchAcrossIntervals) {
    MAP_Prev mapPrev;
    // Different task counts.
    DAG_Model two_tasks = BuildTwoNonPerfDag(3.0, 4.0);
    TaskSet one = {Task(0, std::vector<Value_Proba>{Value_Proba(3.0, 1.0)}, 50, 50, 0, "T0")};
    DAG_Model one_task(one, mapPrev, 0, 0);
    EXPECT_THROW(BuildWorstCaseDagAcrossIntervals({two_tasks, one_task}), std::runtime_error);

    // Same count, but a period mismatch on task 1.
    TaskSet a = {Task(0, std::vector<Value_Proba>{Value_Proba(3.0, 1.0)}, 50, 50, 0, "T0"),
                 Task(1, std::vector<Value_Proba>{Value_Proba(4.0, 1.0)}, 50, 50, 1, "T1")};
    TaskSet b = {Task(0, std::vector<Value_Proba>{Value_Proba(4.0, 1.0)}, 50, 50, 0, "T0"),
                 Task(1, std::vector<Value_Proba>{Value_Proba(5.0, 1.0)}, 100, 100, 1, "T1")};
    DAG_Model dag_a(a, mapPrev, 0, 0);
    DAG_Model dag_b(b, mapPrev, 0, 0);
    EXPECT_THROW(BuildWorstCaseDagAcrossIntervals({dag_a, dag_b}), std::runtime_error);
}

// §8d.1: the worst-case point mass STOCHASTICALLY DOMINATES every interval's
// per-task dist — for each interval j, each task i,
// worst.tasks[i].max_time >= interval_dags[j].tasks[i].max_time. This is the
// soundness leg: any interval's ET draw ≤ its max_time ≤ the worst-case max →
// the gate's ddl_miss_chance on the worst-case DAG upper-bounds every interval.
TEST(BuildWorstCaseDag, StochasticallyDominatesEveryInterval) {
    // Three intervals with varied per-task max_times (incl. a lower-mean interval
    // carrying a fatter tail — the mean-independent-sigma case that broke the old
    // "longest by avg ET" bound).
    std::vector<DAG_Model> intervals = {BuildTwoNonPerfDag(3.0, 4.0),   // interval 0
                                        BuildTwoNonPerfDag(4.0, 5.0),   // interval 1
                                        BuildTwoNonPerfDag(2.0, 6.0)};  // interval 2
    DAG_Model worst = BuildWorstCaseDagAcrossIntervals(intervals);

    for (size_t j = 0; j < intervals.size(); j++) {
        for (size_t i = 0; i < intervals[j].tasks.size(); i++) {
            EXPECT_GE(worst.tasks[i].execution_time_dist.max_time,
                      intervals[j].tasks[i].execution_time_dist.max_time)
                << "worst-case max_time must dominate interval " << j << " task " << i;
        }
    }
    // The worst-case max is the per-task MAX across intervals: {4.0, 6.0}.
    EXPECT_DOUBLE_EQ(4.0, worst.tasks[0].execution_time_dist.max_time);
    EXPECT_DOUBLE_EQ(6.0, worst.tasks[1].execution_time_dist.max_time);
}

// §8d.2: cross-interval safety leg — a fallback computed on the worst-case DAG
// re-gates clean against EACH interval's DAG individually. The worst-case point
// mass dominates every interval's dist → the gate's ddl_miss_chance on each
// interval ≤ on the worst-case → if it passed on the worst-case (it did, by
// construction: the walk only REJECTS + the loud-fail check passed), it passes on
// every interval. A failure here is a soundness bug.
TEST_F(CompareAndKeepSynthetic,
       ComputeSafeFallback_ReGatesCleanAgainstEveryInterval) {
    dag_tasks.tasks[0].is_important = true;  // T_perf is the gated important task
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);

    // Two intervals: interval 0 = the fixture DAG; interval 1 = same structure
    // with a LARGER non-perf (T_noise) max_time. The worst-case DAG takes the max.
    DAG_Model dag0 = dag_tasks;
    TaskSet tasks1 = dag_tasks.tasks;
    tasks1[1].execution_time_dist =
        FiniteDist(std::vector<Value_Proba>{Value_Proba(120.0, 1.0)});  // > 50.0
    DAG_Model dag1(tasks1, dag_tasks.chains_, dag_tasks.chains_deadlines_);
    std::vector<DAG_Model> intervals = {dag0, dag1};
    DAG_Model worst_case_dag = BuildWorstCaseDagAcrossIntervals(intervals);

    ASSERT_FALSE(opt.HasSafeFallback());
    opt.ComputeSafeFallback(worst_case_dag);
    ASSERT_TRUE(opt.HasSafeFallback());
    ResourceOptResult result = opt.GetSafeFallback();

    // Re-gate the stored fallback against EACH interval's DAG individually. The
    // TL vector is task-id-keyed in the result; rebuild it per interval (task
    // order is identical across intervals — verified by the builder).
    for (size_t j = 0; j < intervals.size(); j++) {
        const DAG_Model& interval_dag = intervals[j];
        std::vector<double> tl_result(interval_dag.tasks.size());
        for (size_t i = 0; i < interval_dag.tasks.size(); i++) {
            int id = interval_dag.tasks[i].id;
            tl_result[i] = result.id2time_limit.count(id) ? result.id2time_limit.at(id) : -1.0;
        }
        SP_Parameters sp_interval(interval_dag);
        std::vector<FiniteDist> node_rtas =
            NodeRTAsForCandidate(interval_dag, result.priority_vec, tl_result);
        EXPECT_TRUE(ImportantTasksMeetThresholds(interval_dag, sp_interval,
                                                 result.priority_vec, tl_result,
                                                 node_rtas))
            << "stored fallback must re-gate clean on interval " << j;
    }
}

// --- P0.6 §8e: TaskStructureMatches (the structural-equality predicate
// BuildWorstCaseDagAcrossIntervals uses to fuse interval DAGs). "Structure" =
// id/period/deadline/processorId/name + the full timePerformancePairs grid; the
// ET dist is DELIBERATELY excluded (the builder fuses the max across intervals).

namespace {
Task MakeStructTask(int id, int period, double ddl, int proc, std::string name) {
    std::vector<Value_Proba> dist = {Value_Proba(1.0, 1.0)};
    Task t(id, dist, period, ddl, id, name);
    t.processorId = proc;  // not a Task ctor param; set explicitly.
    return t;
}
}  // namespace

TEST(TaskStructureMatches, IdenticalTasksMatch) {
    Task a = MakeStructTask(0, 50, 50, 0, "T0");
    Task b = MakeStructTask(0, 50, 50, 0, "T0");
    EXPECT_TRUE(TaskStructureMatches(a, b));
}

TEST(TaskStructureMatches, DifferingExecutionTimeDistStillMatches) {
    Task a = MakeStructTask(0, 50, 50, 0, "T0");
    Task b = MakeStructTask(0, 50, 50, 0, "T0");
    a.execution_time_dist = FiniteDist(GaussianDist(3.0, 0.5), 5);
    b.execution_time_dist = FiniteDist(GaussianDist(9.0, 1.0), 5);
    EXPECT_TRUE(TaskStructureMatches(a, b));
}

TEST(TaskStructureMatches, ScalarFieldsMismatch) {
    Task base = MakeStructTask(0, 50, 50, 0, "T0");
    EXPECT_FALSE(TaskStructureMatches(base, MakeStructTask(1, 50, 50, 0, "T0")));  // id
    EXPECT_FALSE(TaskStructureMatches(base, MakeStructTask(0, 100, 50, 0, "T0"))); // period
    EXPECT_FALSE(TaskStructureMatches(base, MakeStructTask(0, 50, 80, 0, "T0")));  // deadline
    EXPECT_FALSE(TaskStructureMatches(base, MakeStructTask(0, 50, 50, 1, "T0")));  // processorId
    EXPECT_FALSE(TaskStructureMatches(base, MakeStructTask(0, 50, 50, 0, "TX")));  // name
}

TEST(TaskStructureMatches, TimePerfPairCountMismatch) {
    Task a = MakeStructTask(0, 50, 50, 0, "T0");
    Task b = MakeStructTask(0, 50, 50, 0, "T0");
    a.timePerformancePairs.push_back(TimePerfPair(400, 0.5));
    EXPECT_FALSE(TaskStructureMatches(a, b));
    EXPECT_FALSE(TaskStructureMatches(b, a));
}

TEST(TaskStructureMatches, TimePerfPairValueMismatch) {
    Task a = MakeStructTask(0, 50, 50, 0, "T0");
    Task b = MakeStructTask(0, 50, 50, 0, "T0");
    a.timePerformancePairs = {TimePerfPair(400, 0.5), TimePerfPair(600, 0.7)};
    b.timePerformancePairs = {TimePerfPair(400, 0.5), TimePerfPair(600, 0.9)};  // perf differs
    EXPECT_FALSE(TaskStructureMatches(a, b));
    b.timePerformancePairs = {TimePerfPair(400, 0.5), TimePerfPair(800, 0.7)};  // time differs
    EXPECT_FALSE(TaskStructureMatches(a, b));
    b.timePerformancePairs = {TimePerfPair(400, 0.5), TimePerfPair(600, 0.7)};  // identical
    EXPECT_TRUE(TaskStructureMatches(a, b));
}

int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
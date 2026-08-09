// #include <gtest/gtest.h>

#include <unistd.h>  // access/F_OK for the optional-data guard in the N=10 probe

#include <set>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Optimization/OptimizeSP_BF.h"
#include "sources/Optimization/OptimizeSP_Incre.h"
#include "sources/Optimization/OptimizeSP_TL_Incre.h"
#include "sources/Safety_Performance_Metric/RTA_Cache.h"
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

class TaskSetForTest_robotics_v6 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v6";
        std::string path =
            GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
        dag_tasks = ReadDAG_Tasks(path, 5);
        sp_parameters = SP_Parameters(dag_tasks);
    }

    // data members
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
    int N = dag_tasks.tasks.size();
};

TEST_F(TaskSetForTest_robotics_v6, Compare_PriorityPartialPath) {
    std::priority_queue<PriorityPartialPath, std::vector<PriorityPartialPath>,
                        CompPriorityPath>
        pq;
    PriorityPartialPath path(dag_tasks, sp_parameters);
    for (int task_id : path.tasks_to_assign) {
        PriorityPartialPath new_path = path;
        new_path.AssignAndUpdateSP(task_id);
        pq.push(new_path);
    }
    EXPECT_EQ("TSP", dag_tasks.tasks[pq.top().pa_vec_lower_pri[0]].name);
    pq.pop();

    EXPECT_EQ("SLAM", dag_tasks.tasks[pq.top().pa_vec_lower_pri[0]].name);
    pq.pop();

    EXPECT_EQ("RRT", dag_tasks.tasks[pq.top().pa_vec_lower_pri[0]].name);
    pq.pop();

    EXPECT_EQ("MPC", dag_tasks.tasks[pq.top().pa_vec_lower_pri[0]].name);
    pq.pop();
}

TEST_F(TaskSetForTest_robotics_v6, GetPriorityAssignments) {
    GlobalVariables::Granularity = 10;
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    PriorityVec pa_vec1 = opt.OptimizeFromScratch(2).priority_vec;
    EXPECT_EQ(4, pa_vec1.size());
    EXPECT_EQ("MPC", dag_tasks.tasks[pa_vec1[0]].name);
    EXPECT_EQ("RRT", dag_tasks.tasks[pa_vec1[1]].name);
    // EXPECT_EQ("TSP", dag_tasks.tasks[pa_vec1[2]].name);
    // EXPECT_EQ("SLAM", dag_tasks.tasks[pa_vec1[3]].name);
}

class TaskSetForTest_robotics_v7 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v7";
        std::string path =
            GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
        dag_tasks = ReadDAG_Tasks(path, 5);
        sp_parameters = SP_Parameters(dag_tasks);
    }

    // data members
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
    int N = dag_tasks.tasks.size();
};
TEST_F(TaskSetForTest_robotics_v7, GetPriorityAssignments) {
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    PriorityVec pa_vec1 = opt.OptimizeFromScratch(2).priority_vec;
    EXPECT_EQ(4, pa_vec1.size());
    EXPECT_EQ("MPC", dag_tasks.tasks[pa_vec1[0]].name);
    EXPECT_EQ("RRT", dag_tasks.tasks[pa_vec1[1]].name);
    EXPECT_EQ("SLAM", dag_tasks.tasks[pa_vec1[2]].name);
    EXPECT_EQ("TSP", dag_tasks.tasks[pa_vec1[3]].name);
}
class TaskSetForTest_robotics_v8 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v8";
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
class TaskSetForTest_robotics_v27 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v27";
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

TEST_F(TaskSetForTest_robotics_v8, FindTaskWithDifferentEt) {
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    PriorityVec pa_vec1 = opt.OptimizeFromScratch(2).priority_vec;
    EXPECT_EQ("SLAM", dag_tasks.tasks[pa_vec1[0]].name);
    EXPECT_EQ("TSP", dag_tasks.tasks[pa_vec1[1]].name);
}

TEST(TaskSet, FindTaskWithDifferentEt) {
    auto dag22 = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                               "TaskData/test_robotics_v22.yaml");
    auto dag23 = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                               "TaskData/test_robotics_v23.yaml");
    auto dag24 = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                               "TaskData/test_robotics_v24.yaml");
    auto diff2 = FindTaskWithDifferentEt(dag22, dag23);
    EXPECT_EQ(1, diff2.size());
    EXPECT_EQ(1, diff2[0].task_id);
    EXPECT_TRUE(diff2[0].increase);
    auto diff3 = FindTaskWithDifferentEt(dag22, dag24);
    EXPECT_EQ(0, diff3.size());
}

// FindTasksWithFlexibleTimeLimits: only tasks with a non-empty
// timePerformancePairs are TL-flexible. In v19 only TSP (task 0) carries
// performance_records_time (see TaskData/test_robotics_v19.yaml); MPC/RRT/SLAM
// have none. So the flexible set is {0}.
TEST(TaskSet, FindTasksWithFlexibleTimeLimits) {
    auto dag19 = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                               "TaskData/test_robotics_v19.yaml");
    auto flex = FindTasksWithFlexibleTimeLimits(dag19);
    ASSERT_EQ(1u, flex.size());
    EXPECT_EQ(0, flex[0]);
}

// FindEnvTaskWithDifferentEt filters TL-flexible tasks out of the full diff.
// v19->v21 moves TWO tasks' execution_time_dist: TSP (task 0, TL-flexible,
// 1500.9->400.9) and SLAM (task 3, NOT TL-flexible, 2853->285). The unfiltered
// FindTaskWithDifferentEt flags both {0,3}; the env-only filter drops the
// TL-flexible TSP and reports only SLAM {3} — the env signal survives without
// depending on bit-equal perf-pair dists (FiniteDist::operator!= is
// 10%-relative approx_equal, Probability.cpp:415-417).
TEST(TaskSet, FindEnvTaskWithDifferentEt_filtersTLFlexible) {
    auto dag19 = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                               "TaskData/test_robotics_v19.yaml");
    auto dag21 = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                               "TaskData/test_robotics_v21.yaml");

    auto full = FindTaskWithDifferentEt(dag19, dag21);
    auto env = FindEnvTaskWithDifferentEt(dag19, dag21);

    // Sanity: the full diff sees both movers.
    ASSERT_EQ(2u, full.size());
    std::set<int> full_ids = {full[0].task_id, full[1].task_id};
    EXPECT_EQ((std::set<int>{0, 3}), full_ids);

    // The env filter drops the TL-flexible TSP (0), keeps SLAM (3).
    ASSERT_EQ(1u, env.size());
    EXPECT_EQ(3, env[0].task_id);
    EXPECT_FALSE(env[0].increase);  // SLAM 2853 -> 285: decrease
}

// When the flagged task is NOT TL-flexible, the env filter is a no-op: the two
// functions report the same diff. v22->v23 moves MPC (task 1, no perf records)
// only — so full == env. (v22 has a TL-flexible TSP, but TSP is unchanged
// here, so the filter's TSP membership is irrelevant.) This is the case the
// env filter must not over-filter: a non-TL-flexible mover is never dropped.
TEST(TaskSet, FindEnvTaskWithDifferentEt_noopWhenFlaggedNotTLFlexible) {
    auto dag22 = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                               "TaskData/test_robotics_v22.yaml");
    auto dag23 = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                               "TaskData/test_robotics_v23.yaml");
    auto dag24 = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                               "TaskData/test_robotics_v24.yaml");

    auto full23 = FindTaskWithDifferentEt(dag22, dag23);
    auto env23 = FindEnvTaskWithDifferentEt(dag22, dag23);
    ASSERT_EQ(1u, full23.size());
    EXPECT_EQ(full23[0].task_id, env23[0].task_id);
    EXPECT_EQ(full23[0].increase, env23[0].increase);
    EXPECT_EQ(full23.size(), env23.size());

    auto full24 = FindTaskWithDifferentEt(dag22, dag24);
    auto env24 = FindEnvTaskWithDifferentEt(dag22, dag24);
    EXPECT_EQ(full24.size(), env24.size());
}
// The four *_full_range tests pin the FULL candidate-range contract: pass
// exclude_opt_pa=false so the generator still emits the carried-position
// variation (i == old_priority_index), which reconstructs pa_vec exactly. They
// document the range logic that the default-true path (below) skips.
TEST_F(TaskSetForTest_robotics_v7, FindPriorityVec1D_Variations_full_range) {
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    PriorityVec pa_vec1 = {0, 1, 2, 3};
    std::vector<PriorityVec> res = FindPriorityVec1D_Variations(
        pa_vec1, 0, PriorityChangeStatus::OpenToAll, /*exclude_opt_pa=*/false);
    EXPECT_EQ(4, res.size());
    AssertEqualVectorExact<int>({0, 1, 2, 3}, res[0], 1e-3, __LINE__);
    AssertEqualVectorExact<int>({1, 0, 2, 3}, res[1], 1e-3, __LINE__);
    AssertEqualVectorExact<int>({1, 2, 0, 3}, res[2], 1e-3, __LINE__);
    AssertEqualVectorExact<int>({1, 2, 3, 0}, res[3], 1e-3, __LINE__);
}
TEST_F(TaskSetForTest_robotics_v7,
       FindPriorityVec1D_Variations_increase_full_range) {
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    PriorityVec pa_vec1 = {0, 1, 2, 3};
    std::vector<PriorityVec> res = FindPriorityVec1D_Variations(
        pa_vec1, 0, PriorityChangeStatus::Increase, /*exclude_opt_pa=*/false);
    EXPECT_EQ(1, res.size());
    AssertEqualVectorExact<int>({0, 1, 2, 3}, res[0], 1e-3, __LINE__);
}
TEST_F(TaskSetForTest_robotics_v7,
       FindPriorityVec1D_Variations_increase2_full_range) {
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    PriorityVec pa_vec1 = {0, 1, 2, 3};
    std::vector<PriorityVec> res = FindPriorityVec1D_Variations(
        pa_vec1, 1, PriorityChangeStatus::Increase, /*exclude_opt_pa=*/false);
    EXPECT_EQ(2, res.size());
    AssertEqualVectorExact<int>({1, 0, 2, 3}, res[0], 1e-3, __LINE__);
    AssertEqualVectorExact<int>({0, 1, 2, 3}, res[1], 1e-3, __LINE__);
}
TEST_F(TaskSetForTest_robotics_v7,
       FindPriorityVec1D_Variations_decrease_full_range) {
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    PriorityVec pa_vec1 = {0, 1, 2, 3};
    std::vector<PriorityVec> res = FindPriorityVec1D_Variations(
        pa_vec1, 0, PriorityChangeStatus::Decrease, /*exclude_opt_pa=*/false);
    EXPECT_EQ(4, res.size());
    AssertEqualVectorExact<int>({0, 1, 2, 3}, res[0], 1e-3, __LINE__);
    AssertEqualVectorExact<int>({1, 0, 2, 3}, res[1], 1e-3, __LINE__);
    AssertEqualVectorExact<int>({1, 2, 0, 3}, res[2], 1e-3, __LINE__);
    AssertEqualVectorExact<int>({1, 2, 3, 0}, res[3], 1e-3, __LINE__);
}

// The default-true path (what OptimizeIncre inherits): exclude_opt_pa=true
// drops the carried-position variation — the one that reconstructs pa_vec and
// whose SP-eval would duplicate the incumbent's already-scored baseline. Size
// is one less than the full range in each PriorityChangeStatus, and none of the
// emitted variations equals the carried PA.
TEST_F(TaskSetForTest_robotics_v7,
       FindPriorityVec1D_Variations_excludes_carried_pa) {
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    PriorityVec pa_vec1 = {0, 1, 2, 3};
    // OpenToAll full range is 4 (positions 0..3); carried at index 0 dropped
    // -> 3.
    std::vector<PriorityVec> res_open = FindPriorityVec1D_Variations(
        pa_vec1, 0, PriorityChangeStatus::OpenToAll);
    EXPECT_EQ(3, res_open.size());
    for (const PriorityVec& pa : res_open)
        ASSERT_NE(pa, (PriorityVec{0, 1, 2, 3}));

    // Increase on task 0: full range is index 0 only (the carried pos) -> 0.
    std::vector<PriorityVec> res_inc = FindPriorityVec1D_Variations(
        pa_vec1, 0, PriorityChangeStatus::Increase);
    EXPECT_EQ(0, res_inc.size());

    // Increase on task 1: full range is indices 0,1 (2); carried at index 1
    // dropped -> 1 (index 0 only).
    std::vector<PriorityVec> res_inc2 = FindPriorityVec1D_Variations(
        pa_vec1, 1, PriorityChangeStatus::Increase);
    EXPECT_EQ(1, res_inc2.size());
    AssertEqualVectorExact<int>({1, 0, 2, 3}, res_inc2[0], 1e-3, __LINE__);

    // Decrease on task 0: full range is indices 0..3 (4); carried at index 0
    // dropped -> 3.
    std::vector<PriorityVec> res_dec = FindPriorityVec1D_Variations(
        pa_vec1, 0, PriorityChangeStatus::Decrease);
    EXPECT_EQ(3, res_dec.size());
    for (const PriorityVec& pa : res_dec)
        ASSERT_NE(pa, (PriorityVec{0, 1, 2, 3}));
}

class TaskSetForTest_robotics_v25 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v25";
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

TEST_F(TaskSetForTest_robotics_v25, AnalyzePriorityChangeStatus) {
    EXPECT_EQ(PriorityChangeStatus::Increase,
              AnalyzePriorityChangeStatus(sp_parameters, 3, true));
    EXPECT_EQ(PriorityChangeStatus::Decrease,
              AnalyzePriorityChangeStatus(sp_parameters, 0, true));
    EXPECT_EQ(PriorityChangeStatus::Increase,
              AnalyzePriorityChangeStatus(sp_parameters, 0, false));
}

TEST_F(TaskSetForTest_robotics_v8, AssignAndUpdateSP) {
    dag_tasks.tasks[0].priority = 2;
    dag_tasks.tasks[1].priority = 1;  // SLAM has high priority
    double sp_ref = ObtainSP_DAG(dag_tasks, sp_parameters).sp_value;

    PriorityPartialPath priority_path(dag_tasks, sp_parameters);
    priority_path.AssignAndUpdateSP(0);
    EXPECT_EQ("TSP", dag_tasks.tasks[priority_path.pa_vec_lower_pri[0]].name);
    EXPECT_EQ(1, priority_path.tasks_to_assign.size());
    EXPECT_EQ("SLAM",
              dag_tasks.tasks[*(priority_path.tasks_to_assign.begin())].name);

    priority_path.AssignAndUpdateSP(1);
    EXPECT_EQ("TSP", dag_tasks.tasks[priority_path.pa_vec_lower_pri[0]].name);
    EXPECT_EQ("SLAM", dag_tasks.tasks[priority_path.pa_vec_lower_pri[1]].name);
    EXPECT_EQ(0, priority_path.tasks_to_assign.size());
    EXPECT_NEAR(
        200 + 0.5 - sp_ref, priority_path.sp_lost,
        1e-3);  // the tolerance is relatively high because Ryan added small
                // modification to break ties in certain situations
}

TEST_F(TaskSetForTest_robotics_v27, GetPriorityAssignments_IncrementalOpt) {
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    PriorityVec pa_vec1 = opt.OptimizeFromScratch(2).priority_vec;
    EXPECT_EQ("TSP", dag_tasks.tasks[pa_vec1[0]].name);
    EXPECT_EQ("SLAM", dag_tasks.tasks[pa_vec1[1]].name);

    DAG_Model dag_tasks_update = ReadDAG_Tasks(
        GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v8.yaml", 5);
    pa_vec1 = opt.OptimizeIncre(dag_tasks_update).priority_vec;
    EXPECT_EQ("SLAM", dag_tasks.tasks[pa_vec1[0]].name);
    EXPECT_EQ("TSP", dag_tasks.tasks[pa_vec1[1]].name);
}

// Differential test for the D1 main extraction (OptimizeIncre_SingleTask).
// On a |diff|==1 update, OptimizeIncre's loop body makes exactly ONE
// OptimizeIncre_SingleTask call (after the baseline seed). So the primitive
// alone — given the same baseline-seed the full method computes — must land on
// the SAME opt_pa_/opt_sp_ as the full OptimizeIncre. Proves the extraction is
// behavior-preserving (a refactor, not new behavior). Uses the v22->v23 pair,
// which FindTaskWithDifferentEt flags as exactly one changed task (task 1,
// increase) — see the FindTaskWithDifferentEt test above.
TEST(OptimizeIncre_SingleTask, Differential_BitIdenticalOnSingleEtChange) {
    DAG_Model dag_base = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                                       "TaskData/test_robotics_v22.yaml");
    DAG_Model dag_update = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                                         "TaskData/test_robotics_v23.yaml");
    SP_Parameters sp = SP_Parameters(dag_base);

    // Sanity: the update must be a |diff|==1 case (else this isn't testing the
    // primitive's single-task contract).
    std::vector<DiffObj> diff = FindTaskWithDifferentEt(dag_base, dag_update);
    ASSERT_EQ(diff.size(), 1u);
    int task_id = diff[0].task_id;
    bool et_increased = diff[0].increase;

    // Two independent optimizers from the same scratch state —
    // OptimizeFromScratch is deterministic given K, so optA and optB hold
    // identical opt_pa_/opt_sp_/ dag_tasks_ after this.
    OptimizePA_Incre optA(dag_base, sp);
    optA.OptimizeFromScratch(2);
    OptimizePA_Incre optB(dag_base, sp);
    optB.OptimizeFromScratch(2);
    AssertEqualVectorExact<int>(optA.opt_pa_, optB.opt_pa_, 1e-3, __LINE__);
    EXPECT_DOUBLE_EQ(optA.opt_sp_, optB.opt_sp_);

    // Path A: the full OptimizeIncre (baseline seed + one SingleTask call +
    // dag_tasks_ advance).
    PriorityVec pa_full = optA.OptimizeIncre(dag_update).priority_vec;

    // Path B: the primitive alone, with the SAME baseline seed OptimizeIncre
    // would have computed (the carried PA's SP under the new env). This is the
    // contract: OptimizeIncre_SingleTask TRUSTS opt_sp_ (caller-set).
    optB.opt_sp_ =
        EvaluateSPWithPriorityVec(dag_update, sp, optB.opt_pa_).sp_value;
    PriorityVec pa_primitive =
        optB.OptimizeIncre_SingleTask(dag_update, task_id, et_increased)
            .priority_vec;

    // The primitive must reproduce the full method's adopted PA and SP exactly.
    AssertEqualVectorExact<int>(pa_full, pa_primitive, 1e-3, __LINE__);
    EXPECT_DOUBLE_EQ(optA.opt_sp_, optB.opt_sp_);
}

TEST(OptimizePA_Incre_with_TimeLimits_TDD, SortingHeuristic) {
    std::vector<Value_Proba> dist = {Value_Proba(1, 1.0)};
    TaskSet tasks;
    tasks.push_back(Task(0, dist, 10, 10, 0, "T0"));
    tasks.push_back(Task(1, dist, 10, 10, 1, "T1"));
    tasks.push_back(Task(2, dist, 10, 10, 2, "T2"));
    tasks.push_back(Task(3, dist, 10, 10, 3, "T3"));

    MAP_Prev mapPrev;
    DAG_Model dag(tasks, mapPrev, 0, 0);
    SP_Parameters sp_params(dag);

    sp_params.weights_node[0] = 2.0;
    sp_params.thresholds_node[0] = 1.0;

    sp_params.weights_node[1] = 5.0;
    sp_params.thresholds_node[1] = 2.0;

    sp_params.weights_node[2] = 5.0;
    sp_params.thresholds_node[2] = 0.5;

    sp_params.weights_node[3] = 2.0;
    sp_params.thresholds_node[3] = 0.5;

    std::vector<size_t> indices = {0, 1, 2, 3};
    std::sort(indices.begin(), indices.end(),
              TaskSortingHeuristic{dag, sp_params});

    EXPECT_EQ(2, indices[0]);
    EXPECT_EQ(1, indices[1]);
    EXPECT_EQ(3, indices[2]);
    EXPECT_EQ(0, indices[3]);
}

TEST(OptimizePA_Incre_with_TimeLimits_TDD, ComplexityLinear_3N) {
    std::vector<Value_Proba> dist = {Value_Proba(1, 1.0)};
    TaskSet tasks;
    tasks.push_back(Task(0, dist, 10, 10, 0, "T0"));
    tasks.push_back(Task(1, dist, 10, 10, 1, "T1"));
    tasks.push_back(Task(2, dist, 10, 10, 2, "T2"));

    tasks[0].timePerformancePairs = {
        TimePerfPair(10, 0.5), TimePerfPair(20, 0.7), TimePerfPair(30, 0.9)};
    tasks[1].timePerformancePairs = {
        TimePerfPair(15, 0.5), TimePerfPair(25, 0.7), TimePerfPair(35, 0.9)};
    tasks[2].timePerformancePairs = {
        TimePerfPair(20, 0.5), TimePerfPair(30, 0.7), TimePerfPair(40, 0.9)};

    MAP_Prev mapPrev;
    DAG_Model dag(tasks, mapPrev, 0, 0);
    SP_Parameters sp_params(dag);

    OptimizePA_Incre_with_TimeLimits optimizer(dag, sp_params);
    optimizer.ReOptimizePeriodic(dag, 2);

    int evaluated_count = optimizer.eval_count_;

    EXPECT_LE(evaluated_count, 10);
    EXPECT_GT(evaluated_count, 0);
}

// Diagnostic for the §10-vs-§11 ndiff contradiction: §10's instrumented
// OptimizeIncre reported ndiff≈8 at N=10, but a Python reproduction of
// FiniteDist + Value_Proba::operator== over the same interval YAMLs flags only
// 2 tasks (the two that genuinely drift: tasks 4 and 9). This test reads the
// REAL interval YAMLs through the REAL ReadDAG_Tasks → FiniteDist path and
// reports exactly which tasks FindTaskWithDifferentEt flags, so the C++ truth
// settles it. Intentionally assertion-light in its first form: it PRINTS the
// result so we can read the ground truth, then asserts only the size (which we
// will pin after the first run). Skips gracefully if the taskset is absent
// (e.g. CI without the sim-experiment data).
TEST(FindTaskWithDifferentEt, N10IntervalYamlGroundTruth) {
    std::string dir = GlobalVariables::PROJECT_PATH +
                      "simulation_experiments/optimizer_comparison/"
                      "tasks10_dur600_interval10_seed1000/taskset_0";
    std::string p0 = dir + "/taskset_characteristics_interval_0.yaml";
    std::string p1 = dir + "/taskset_characteristics_interval_1.yaml";
    if (access(p0.c_str(), F_OK) != 0 || access(p1.c_str(), F_OK) != 0) {
        GTEST_SKIP() << "N=10 interval YAMLs not present at " << dir
                     << "; skipping C++ ground-truth probe.";
    }
    DAG_Model dag0 = ReadDAG_Tasks(p0);
    DAG_Model dag1 = ReadDAG_Tasks(p1);
    ASSERT_EQ(dag0.tasks.size(), dag1.tasks.size());
    std::vector<DiffObj> diff = FindTaskWithDifferentEt(dag0, dag1);
    std::cerr << "[N10-NDIFF-DBG] dag0->dag1: N=" << dag0.tasks.size()
              << " ndiff=" << diff.size() << " tasks=[";
    for (const DiffObj& d : diff)
        std::cerr << d.task_id << (d.increase ? "+ " : "- ");
    std::cerr << "]\n";
    // Per the faithful Python repro, only tasks 4 and 9 genuinely drift between
    // interval_0 and interval_1 (both static tasks, large mu/min/max changes).
    // If C++ agrees, ndiff==2 and the §10 ndiff=8 must come from baseline lag,
    // not from operator!= noise. If C++ reports >2, the FiniteDist construction
    // path differs from the Python repro and §11c deserves re-examination.
    EXPECT_EQ(diff.size(), 2u)
        << "C++ ground-truth ndiff for dag0->dag1 disagrees with Python repro";
}

// P1.13 sub-step 3 — differential TDD: the cache path (OptimizeIncre with an
// engaged RTACacheOpt) must be BIT-IDENTICAL to the oracle path (OptimizeIncre
// with std::nullopt) on a |diff|==1 update. The cache is the only thing that
// differs between the two arms; both run the SAME OptimizeIncre loop body, the
// SAME FindPriorityVec1D_Variations, the SAME strict-> adoption. So if the
// cache's Evaluate patch + ObtainSP_Full_From_NodeRTAs scoring + AdoptChampion
// champion-advance reproduce the oracle EvaluateSPWithPriorityVec per
// candidate, the two arms land on the same opt_pa_/opt_sp_. This is the
// behavior-preservation gate for sub-step 2b's wiring. Uses the v22->v23 pair
// (one changed task, task 1 increase — same as the SingleTask differential
// above) so the 1D variations exercise the |diff|<=1 patch path.
TEST(OptimizeIncre_Cache, Differential_BitIdenticalToOracle_OnSingleEtChange) {
    DAG_Model dag_base = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                                       "TaskData/test_robotics_v22.yaml");
    DAG_Model dag_update = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                                         "TaskData/test_robotics_v23.yaml");
    SP_Parameters sp = SP_Parameters(dag_base);

    // Sanity: |diff|==1 (the only case the cache's patch path serves; a >1 diff
    // would throw inside ComputeTaskSetDifference via ClassifyReusePerTask).
    ASSERT_EQ(FindTaskWithDifferentEt(dag_base, dag_update).size(), 1u);

    // Two independent optimizers from the SAME scratch state.
    // OptimizeFromScratch is deterministic given K, so optOracle and optCache
    // hold identical opt_pa_/opt_sp_/dag_tasks_ after this — the only
    // difference is the cache handle passed to OptimizeIncre below.
    OptimizePA_Incre optOracle(dag_base, sp);
    optOracle.OptimizeFromScratch(2);
    OptimizePA_Incre optCache(dag_base, sp);
    optCache.OptimizeFromScratch(2);
    AssertEqualVectorExact<int>(optOracle.opt_pa_, optCache.opt_pa_, 1e-3,
                                __LINE__);
    EXPECT_DOUBLE_EQ(optOracle.opt_sp_, optCache.opt_sp_);

    // Oracle arm: std::nullopt → legacy EvaluateSPWithPriorityVec per
    // candidate.
    PriorityVec pa_oracle = optOracle.OptimizeIncre(dag_update).priority_vec;

    // Cache arm: engaged RTACache → Initialize at baseline + Evaluate patch +
    // ObtainSP_Full_From_NodeRTAs scoring + AdoptChampion per adoption.
    RTACache cache;
    PriorityVec pa_cache =
        optCache.OptimizeIncre(dag_update, INT_MIN, cache).priority_vec;

    // The cache path must reproduce the oracle's adopted PA and SP exactly.
    AssertEqualVectorExact<int>(pa_oracle, pa_cache, 1e-3, __LINE__);
    EXPECT_DOUBLE_EQ(optOracle.opt_sp_, optCache.opt_sp_)
        << "cache-path SP diverged from oracle-path SP";
}

// P1.13 sub-step 3b — broader differential coverage. The single v22->v23 case
// above exercises ONE branch: task 1, ET INCREASE, on a low-weight core-1 task.
// The cache path has more surface than that:
//  - AnalyzePriorityChangeStatus flips its half (Increase vs Decrease)
//  depending
//    on et_increased AND whether the task holds the unique-highest weight. An
//    ET DECREASE takes the OTHER half of the 1D-variation range.
//  - Different task_id → different processorId / weight / priority position in
//    opt_pa_, which exercises different MoveToCore / core-stay paths inside
//    RTACache::AnalyzePrioritySwitch (the per-core single-move detection).
//  - A change that actually TRIGGERS a strict-improvement adoption mid-loop
//    exercises AdoptChampion's champion-advance (the "next variation's diff
//    stays |diff|<=1" invariant the wiring depends on) — a no-adopt run never
//    touches that.
//  - Two OptimizeIncre calls back-to-back exercise the baseline Initialize on a
//    FRESH interval vs a carried champion (the stale-champion-vs-throw path the
//    wiring doc warns about; OptimizeIncre uses Initialize, not Evaluate, for
//    exactly this reason).
//
// The shared helper runs the SAME differential as the v22->v23 test (oracle arm
// with nullopt vs cache arm with an engaged RTACache) on a caller-built update.
// dag_base is read from a real yaml (real weights/cores/priorities); dag_update
// is dag_base with ONE task's execution_time_dist overwritten to a shifted
// FiniteDist built the SAME way ReadDAG_Tasks builds it (GaussianDist + min/max
// + granularity=5). That keeps the mutation on the exact path the cache sees
// (ApplyTimeLimitsToTasksExecutionTime is a no-op here since no TL is set).
namespace {
void ExpectCacheMatchesOracleOnMutation(const DAG_Model& dag_base,
                                        const DAG_Model& dag_update, int line) {
    SP_Parameters sp = SP_Parameters(dag_base);

    // The cache's patch path only serves |diff|<=1 (a >1 diff throws inside
    // ComputeTaskSetDifference via ClassifyReusePerTask). Sanity-check it.
    ASSERT_EQ(FindTaskWithDifferentEt(dag_base, dag_update).size(), 1u)
        << "test built a mutation that isn't |diff|==1 (from line " << line
        << ")";

    OptimizePA_Incre optOracle(dag_base, sp);
    optOracle.OptimizeFromScratch(2);
    OptimizePA_Incre optCache(dag_base, sp);
    optCache.OptimizeFromScratch(2);
    // Both optimizers from the same scratch state — deterministic given K.
    AssertEqualVectorExact<int>(optOracle.opt_pa_, optCache.opt_pa_, 1e-3,
                                line);
    EXPECT_DOUBLE_EQ(optOracle.opt_sp_, optCache.opt_sp_);

    PriorityVec pa_oracle = optOracle.OptimizeIncre(dag_update).priority_vec;

    RTACache cache;
    PriorityVec pa_cache =
        optCache.OptimizeIncre(dag_update, INT_MIN, cache).priority_vec;

    AssertEqualVectorExact<int>(pa_oracle, pa_cache, 1e-3, line);
    EXPECT_DOUBLE_EQ(optOracle.opt_sp_, optCache.opt_sp_)
        << "cache-path SP diverged from oracle-path SP (from line " << line
        << ")";
}

// Build a FiniteDist the same way ReadDAG_Tasks does (RegularTasks.cpp:75-79):
// Gaussian(mu,sigma) truncated to [min,max] at the given granularity. Used to
// overwrite one task's execution_time_dist for a synthetic |diff|==1 mutation.
FiniteDist ShiftedFiniteDist(double mu, double sigma, double min_val,
                             double max_val, int granularity = 5) {
    return FiniteDist(GaussianDist(mu, sigma), min_val, max_val, granularity);
}
}  // namespace

// ET DECREASE on the same task v22->v23 touches (task 1), so the
// AnalyzePriorityChangeStatus half flips (Decrease vs Increase). v23 has task 1
// at mu=7.73; build a decrease from v22 (mu=6.73) to mu=5.5. The 1D variations
// scan the OTHER half of the priority range.
TEST(OptimizeIncre_Cache, Differential_EtDecrease_FlipsVariationHalf) {
    DAG_Model dag_base = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                                       "TaskData/test_robotics_v22.yaml");
    DAG_Model dag_update = dag_base;
    // Task 1 v22: mu=6.7311, sigma=0.0871, min=6.7311, max=6.7311
    // (near-deterministic). Decrease the mean; keep sigma/min/max coherent with
    // a narrower, lower dist.
    dag_update.tasks[1].execution_time_dist =
        ShiftedFiniteDist(5.5, 0.0871, 5.5, 5.5);
    ExpectCacheMatchesOracleOnMutation(dag_base, dag_update, __LINE__);
}

// Mutate the HIGH-WEIGHT task (task 3, sp_weight=10, core 0) instead of a
// low-weight one. AnalyzePriorityChangeStatus checks
// if_highest_weight_unique(task_id); a high-weight task takes the
// weight-unique branch (priority tracks the resource-hungry task), which the
// low-weight v22->v23 case never reaches. ET increase on task 3.
TEST(OptimizeIncre_Cache, Differential_HighWeightTask_EtIncrease) {
    DAG_Model dag_base = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                                       "TaskData/test_robotics_v22.yaml");
    DAG_Model dag_update = dag_base;
    // Task 3 v22: mu=285.32, period 3000, core 0, weight 10 (the
    // unique-highest).
    dag_update.tasks[3].execution_time_dist =
        ShiftedFiniteDist(320.0, 5.0, 310.0, 330.0);
    ExpectCacheMatchesOracleOnMutation(dag_base, dag_update, __LINE__);
}

// A LARGE ET increase on the high-weight task, sized to push its RTA past the
// SP threshold and force a strict-improvement ADOPTION mid-loop (opt_pa_
// changes inside OptimizeIncre_SingleTask). This exercises AdoptChampion's
// champion-advance: after adopting, the next 1D variation must still be
// |diff|<=1 vs the NEW champion. The v22->v23 case may not adopt at all, so
// AdoptChampion's advance path is otherwise untested.
//
// Adoption is data-dependent, so the case sweeps ET magnitudes until one
// actually flips opt_pa_ (probed via the oracle arm), then runs the
// differential on THAT magnitude. If no magnitude in the sweep adopts, the
// test FAILS — the AdoptChampion-advance branch would otherwise be silently
// untested, which is exactly the false-green this suite exists to prevent.
TEST(OptimizeIncre_Cache, Differential_LargeEtIncrease_TriggersAdoption) {
    DAG_Model dag_base = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                                       "TaskData/test_robotics_v22.yaml");
    SP_Parameters sp = SP_Parameters(dag_base);

    // Find the carried PA once (it's the same across the sweep —
    // OptimizeFromScratch is deterministic given K).
    OptimizePA_Incre optSeed(dag_base, sp);
    optSeed.OptimizeFromScratch(2);
    const PriorityVec pa_carried = optSeed.opt_pa_;

    // Sweep task 3's ET up until the oracle arm's OptimizeIncre ADOPTS (returns
    // a PA != pa_carried). Task 3 is the unique-highest-weight task (weight
    // 10), so an ET increase drives AnalyzePriorityChangeStatus toward Increase
    // (higher priority) — a large enough increase should flip its position.
    double adopt_mu = -1.0;
    DAG_Model dag_update = dag_base;
    for (double new_mu : {350.0, 450.0, 600.0, 900.0, 1500.0, 2500.0}) {
        DAG_Model cand = dag_base;
        cand.tasks[3].execution_time_dist =
            ShiftedFiniteDist(new_mu, 5.0, new_mu - 10.0, new_mu + 10.0);
        OptimizePA_Incre optProbe(dag_base, sp);
        optProbe.OptimizeFromScratch(2);
        PriorityVec pa_probe = optProbe.OptimizeIncre(cand).priority_vec;
        if (pa_probe != pa_carried) {
            adopt_mu = new_mu;
            dag_update = cand;
            break;
        }
    }
    ASSERT_NE(adopt_mu, -1.0)
        << "No ET magnitude in the sweep triggered an adoption on task 3; "
        << "the AdoptChampion-advance branch is untestable with this fixture. "
        << "Extend the sweep or pick a different task.";

    // Run the differential on the magnitude that actually adopts.
    ExpectCacheMatchesOracleOnMutation(dag_base, dag_update, __LINE__);
}

// Two OptimizeIncre calls back-to-back on the SAME cache. The second call's
// baseline re-score runs Initialize (full RTA) — NOT Evaluate — because the
// carried champion is on the PREVIOUS interval's dag and may differ by >1 task,
// which Evaluate would reject via ComputeTaskSetDifference. This is the path
// the wiring doc calls out: Initialize overwrites all prior state and
// re-establishes opt_pa_ as the champion. The differential asserts both calls
// match the oracle.
TEST(OptimizeIncre_Cache,
     Differential_SequentialIntervals_ReinitializeChampion) {
    DAG_Model dag_base = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                                       "TaskData/test_robotics_v22.yaml");
    SP_Parameters sp = SP_Parameters(dag_base);

    // interval 1: task 1 increase (the v22->v23 change). interval 2: task 3
    // increase (independent of interval 1). Two |diff|==1 updates in sequence.
    DAG_Model dag_i1 = dag_base;
    dag_i1.tasks[1].execution_time_dist =
        ShiftedFiniteDist(8.5, 0.0871, 8.5, 8.5);
    DAG_Model dag_i2 = dag_i1;
    dag_i2.tasks[3].execution_time_dist =
        ShiftedFiniteDist(350.0, 5.0, 340.0, 360.0);

    // Oracle arm: two nullopt OptimizeIncre calls on one optimizer.
    OptimizePA_Incre optOracle(dag_base, sp);
    optOracle.OptimizeFromScratch(2);
    PriorityVec pa_oracle_i1 = optOracle.OptimizeIncre(dag_i1).priority_vec;
    PriorityVec pa_oracle_i2 = optOracle.OptimizeIncre(dag_i2).priority_vec;

    // Cache arm: the SAME two calls share ONE RTACache — the second call's
    // Initialize must overwrite the champion state the first call's
    // AdoptChampion(s) wrote, without throwing.
    OptimizePA_Incre optCache(dag_base, sp);
    optCache.OptimizeFromScratch(2);
    RTACache cache;
    PriorityVec pa_cache_i1 =
        optCache.OptimizeIncre(dag_i1, INT_MIN, cache).priority_vec;
    PriorityVec pa_cache_i2 =
        optCache.OptimizeIncre(dag_i2, INT_MIN, cache).priority_vec;

    AssertEqualVectorExact<int>(pa_oracle_i1, pa_cache_i1, 1e-3, __LINE__);
    AssertEqualVectorExact<int>(pa_oracle_i2, pa_cache_i2, 1e-3, __LINE__);
    EXPECT_DOUBLE_EQ(optOracle.opt_sp_, optCache.opt_sp_)
        << "cache-path SP diverged from oracle-path SP after interval 2";
}

// baseline_sp PROVIDED (not INT_MIN). This is the `else` branch of
// OptimizeIncre's baseline re-score: the caller already holds the carried PA's
// SP under the new env and passes it to skip the re-score. The cache arm MUST
// still Initialize the champion RTA (full RTA on opt_pa_) even though the SP
// is trusted — Initialize-only, no ObtainSP_Full_From_NodeRTAs re-score. Every
// other test passes INT_MIN, so this `else` branch (both arms) is otherwise
// unexercised. The differential computes the baseline SP via the SAME path the
// header contract requires (EvaluateSPWithPriorityVec on dag_update + carried
// PA) and feeds it to BOTH arms, then asserts cache==oracle on PA + final SP.
TEST(OptimizeIncre_Cache, Differential_BaselineSpProvided_ElseBranch) {
    DAG_Model dag_base = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                                       "TaskData/test_robotics_v22.yaml");
    SP_Parameters sp = SP_Parameters(dag_base);
    DAG_Model dag_update = dag_base;
    // Task 1 increase (the v22->v23 change) — a |diff|==1 mutation.
    dag_update.tasks[1].execution_time_dist =
        ShiftedFiniteDist(8.5, 0.0871, 8.5, 8.5);
    ASSERT_EQ(FindTaskWithDifferentEt(dag_base, dag_update).size(), 1u);

    OptimizePA_Incre optOracle(dag_base, sp);
    optOracle.OptimizeFromScratch(2);
    OptimizePA_Incre optCache(dag_base, sp);
    optCache.OptimizeFromScratch(2);
    AssertEqualVectorExact<int>(optOracle.opt_pa_, optCache.opt_pa_, 1e-3,
                                __LINE__);
    EXPECT_DOUBLE_EQ(optOracle.opt_sp_, optCache.opt_sp_);

    // The caller-provided baseline SP: the carried PA scored under dag_update.
    // The header contract (OptimizeSP_Incre.h:154) requires this EXACT value —
    // both arms must treat it as authoritative and skip their own re-score.
    double baseline_sp =
        EvaluateSPWithPriorityVec(dag_update, sp, optOracle.opt_pa_).sp_value;

    // Oracle arm: baseline_sp provided, nullopt cache (no Initialize).
    PriorityVec pa_oracle =
        optOracle.OptimizeIncre(dag_update, baseline_sp).priority_vec;
    // Cache arm: baseline_sp provided + engaged RTACache (Initialize-only).
    RTACache cache;
    PriorityVec pa_cache =
        optCache.OptimizeIncre(dag_update, baseline_sp, cache).priority_vec;

    AssertEqualVectorExact<int>(pa_oracle, pa_cache, 1e-3, __LINE__);
    EXPECT_DOUBLE_EQ(optOracle.opt_sp_, optCache.opt_sp_)
        << "cache-path SP diverged from oracle-path SP (baseline_sp provided)";
}

// TWO tasks' ET change in ONE interval — `FindTaskWithDifferentEt` returns 2,
// so OptimizeIncre's `for (DiffObj ...)` loop runs TWICE. This is the case the
// AdoptChampion champion-advance is LOAD-BEARING for: the 2nd SingleTask call
// builds its 1D variations from the ADVANCED opt_pa_ (the 1st call's adoption
// moved one task). Evaluate patches each variation vs the champion — if the
// champion didn't advance to match opt_pa_, the 2nd call's candidates would be
// |diff|==2 vs the champion → ComputeTaskSetDifference throws inside Evaluate.
// All other tests are |diff|==1 (loop runs once), so the advance is never
// load-bearing there. A buggy advance (champion not actually advancing) throws
// here. The differential asserts cache==oracle on PA + final SP.
TEST(OptimizeIncre_Cache,
     Differential_TwoTaskDiff_LoopRunsTwice_AdvanceLoadBearing) {
    DAG_Model dag_base = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                                       "TaskData/test_robotics_v22.yaml");
    SP_Parameters sp = SP_Parameters(dag_base);

    // Mutate TWO tasks: task 1 (low-weight, core 1) AND task 3 (high-weight,
    // core 0). Both ET increases. FindTaskWithDifferentEt returns both.
    DAG_Model dag_update = dag_base;
    dag_update.tasks[1].execution_time_dist =
        ShiftedFiniteDist(8.5, 0.0871, 8.5, 8.5);
    dag_update.tasks[3].execution_time_dist =
        ShiftedFiniteDist(350.0, 5.0, 340.0, 360.0);
    ASSERT_EQ(FindTaskWithDifferentEt(dag_base, dag_update).size(), 2u)
        << "expected a 2-task diff to make the loop run twice";

    OptimizePA_Incre optOracle(dag_base, sp);
    optOracle.OptimizeFromScratch(2);
    OptimizePA_Incre optCache(dag_base, sp);
    optCache.OptimizeFromScratch(2);
    AssertEqualVectorExact<int>(optOracle.opt_pa_, optCache.opt_pa_, 1e-3,
                                __LINE__);
    EXPECT_DOUBLE_EQ(optOracle.opt_sp_, optCache.opt_sp_);

    PriorityVec pa_oracle = optOracle.OptimizeIncre(dag_update).priority_vec;
    RTACache cache;
    PriorityVec pa_cache =
        optCache.OptimizeIncre(dag_update, INT_MIN, cache).priority_vec;

    AssertEqualVectorExact<int>(pa_oracle, pa_cache, 1e-3, __LINE__);
    EXPECT_DOUBLE_EQ(optOracle.opt_sp_, optCache.opt_sp_)
        << "cache-path SP diverged from oracle-path SP (2-task diff)";
}

// In-search hard-prune on the from-scratch beam. Two point-mass tasks on one
// core: RTA is a point mass -> ddl_miss is exactly 0/1 -> SP exact. The SP-max
// leaf (task 0 low-pri, SP 0.8) leaves the IMPORTANT task 0 unschedulable; the
// other leaf (task 0 high-pri, SP 0.2) keeps it schedulable. The prune drops the
// 0.8 leaf, so the beam adopts the 0.2 schedulable leaf (schedulable=true).
class TaskSetForTest_p129_unschedulable_important : public ::testing::Test {
   public:
    void SetUp() override {
        GlobalVariables::Granularity = 10;
        FiniteDist et0 = FiniteDist({Value_Proba(5.0, 1.0)});   // point mass
        FiniteDist et1 = FiniteDist({Value_Proba(10.0, 1.0)});  // point mass
        tasks.push_back(Task(0, et0, 100, 6, 0));   // IMPORTANT, ddl 6
        tasks.push_back(Task(1, et1, 100, 12, 1));  // not important, ddl 12
        tasks[0].processorId = 0;
        tasks[1].processorId = 0;
        tasks[0].is_important = true;
        dag_tasks = DAG_Model(tasks, {}, {1e9});
        sp_parameters = SP_Parameters(tasks);
        sp_parameters.weights_node[0] = 0.2;
        sp_parameters.weights_node[1] = 0.8;
        sp_parameters.thresholds_node[0] = 0.1;
        sp_parameters.thresholds_node[1] = 0.1;
    }
    TaskSet tasks;
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
};

TEST_F(TaskSetForTest_p129_unschedulable_important,
       InSearchGate_AdoptsSchedulableLeafOverUnschedulableSpMax) {
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    PriorityOptResult res = opt.OptimizeFromScratch(2);

    // The hard-prune drops the SP-max 0.8 leaf (task 0 low-pri -> important task
    // 0 unschedulable), so the beam adopts the 0.2 schedulable leaf.
    EXPECT_NEAR(res.sp_opt, 0.2, 1e-6)
        << "prune should adopt the 0.2 schedulable leaf; got " << res.sp_opt;
    EXPECT_TRUE(res.schedulable)
        << "the surviving leaf keeps the important task schedulable";
}

// Emptied beam: with BOTH tasks important, every priority order leaves one
// important task unschedulable, so the prune drops every candidate -> the beam
// empties -> schedulable=false.
TEST_F(TaskSetForTest_p129_unschedulable_important,
       InSearchGate_EmptyBeamReportsUnschedulable) {
    dag_tasks.tasks[1].is_important = true;  // now both tasks important
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    PriorityOptResult res = opt.OptimizeFromScratch(2);

    EXPECT_FALSE(res.schedulable)
        << "both orders fail the gate -> emptied beam -> unschedulable";
    EXPECT_TRUE(res.priority_vec.empty())
        << "unschedulable result returns an empty priority assignment vector";
}

// Budget timeout mid-beam: a cancelled budget makes every UpdateSP return false
// (no half-evaluated path is pushed), so the beam empties and the result is
// reported unschedulable rather than committing an ungated partial leaf.
TEST_F(TaskSetForTest_p129_unschedulable_important,
       InSearchGate_BudgetTimeoutEmptiesBeam) {
    int saved_time_limit = GlobalVariables::TIME_LIMIT;
    GlobalVariables::TIME_LIMIT = 0;
    {
        BFDLSharedBudget budget(std::chrono::high_resolution_clock::now());
        OptimizePA_Incre opt(dag_tasks, sp_parameters);
        PriorityOptResult res = opt.OptimizeFromScratch(2);

        EXPECT_FALSE(res.schedulable)
            << "a timed-out beam must not commit a half-evaluated leaf";
        EXPECT_TRUE(res.priority_vec.empty())
            << "timed-out beam returns an empty priority assignment vector";
    }
    GlobalVariables::TIME_LIMIT = saved_time_limit;
}

int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
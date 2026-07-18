// #include <gtest/gtest.h>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Optimization/OptimizeSP_BF.h"
#include "sources/Optimization/OptimizeSP_Incre.h"
#include "sources/Optimization/OptimizeSP_TL_Incre.h"
#include "sources/Utils/Parameters.h"

#include <unistd.h>  // access/F_OK for the optional-data guard in the N=10 probe

#include <set>

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
    PriorityVec pa_vec1 = opt.OptimizeFromScratch(2);
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
    PriorityVec pa_vec1 = opt.OptimizeFromScratch(2);
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
    PriorityVec pa_vec1 = opt.OptimizeFromScratch(2);
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
// depending on bit-equal perf-pair dists (FiniteDist::operator!= is 10%-relative
// approx_equal, Probability.cpp:415-417).
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
    // OpenToAll full range is 4 (positions 0..3); carried at index 0 dropped -> 3.
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
              AnalyzePriorityChangeStatus(sp_parameters, 3, false));
    EXPECT_EQ(PriorityChangeStatus::Decrease,
              AnalyzePriorityChangeStatus(sp_parameters, 0, true));
    EXPECT_EQ(PriorityChangeStatus::Increase,
              AnalyzePriorityChangeStatus(sp_parameters, 0, false));
}

TEST_F(TaskSetForTest_robotics_v8, AssignAndUpdateSP) {
    dag_tasks.tasks[0].priority = 2;
    dag_tasks.tasks[1].priority = 1;  // SLAM has high priority
    double sp_ref = ObtainSP_DAG(dag_tasks, sp_parameters);

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
    PriorityVec pa_vec1 = opt.OptimizeFromScratch(2);
    EXPECT_EQ("TSP", dag_tasks.tasks[pa_vec1[0]].name);
    EXPECT_EQ("SLAM", dag_tasks.tasks[pa_vec1[1]].name);

    DAG_Model dag_tasks_update = ReadDAG_Tasks(
        GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v8.yaml", 5);
    pa_vec1 = opt.OptimizeIncre(dag_tasks_update);
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
    DAG_Model dag_base = ReadDAG_Tasks(
        GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v22.yaml");
    DAG_Model dag_update = ReadDAG_Tasks(
        GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v23.yaml");
    SP_Parameters sp = SP_Parameters(dag_base);

    // Sanity: the update must be a |diff|==1 case (else this isn't testing the
    // primitive's single-task contract).
    std::vector<DiffObj> diff = FindTaskWithDifferentEt(dag_base, dag_update);
    ASSERT_EQ(diff.size(), 1u);
    int task_id = diff[0].task_id;
    bool et_increased = diff[0].increase;

    // Two independent optimizers from the same scratch state — OptimizeFromScratch
    // is deterministic given K, so optA and optB hold identical opt_pa_/opt_sp_/
    // dag_tasks_ after this.
    OptimizePA_Incre optA(dag_base, sp);
    optA.OptimizeFromScratch(2);
    OptimizePA_Incre optB(dag_base, sp);
    optB.OptimizeFromScratch(2);
    AssertEqualVectorExact<int>(optA.opt_pa_, optB.opt_pa_, 1e-3, __LINE__);
    EXPECT_DOUBLE_EQ(optA.opt_sp_, optB.opt_sp_);

    // Path A: the full OptimizeIncre (baseline seed + one SingleTask call +
    // dag_tasks_ advance).
    PriorityVec pa_full = optA.OptimizeIncre(dag_update);

    // Path B: the primitive alone, with the SAME baseline seed OptimizeIncre
    // would have computed (the carried PA's SP under the new env). This is the
    // contract: OptimizeIncre_SingleTask TRUSTS opt_sp_ (caller-set).
    optB.opt_sp_ =
        EvaluateSPWithPriorityVec(dag_update, sp, optB.opt_pa_);
    PriorityVec pa_primitive =
        optB.OptimizeIncre_SingleTask(dag_update, task_id, et_increased);

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
    std::sort(indices.begin(), indices.end(), TaskSortingHeuristic{dag, sp_params});

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

    tasks[0].timePerformancePairs = {TimePerfPair(10, 0.5), TimePerfPair(20, 0.7), TimePerfPair(30, 0.9)};
    tasks[1].timePerformancePairs = {TimePerfPair(15, 0.5), TimePerfPair(25, 0.7), TimePerfPair(35, 0.9)};
    tasks[2].timePerformancePairs = {TimePerfPair(20, 0.5), TimePerfPair(30, 0.7), TimePerfPair(40, 0.9)};

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
    std::string p0 =
        dir + "/taskset_characteristics_interval_0.yaml";
    std::string p1 =
        dir + "/taskset_characteristics_interval_1.yaml";
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

int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
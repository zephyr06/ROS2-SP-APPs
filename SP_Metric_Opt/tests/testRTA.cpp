// #include <gtest/gtest.h>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Safety_Performance_Metric/Probability.h"
#include "sources/Safety_Performance_Metric/RTA.h"
#include "sources/Safety_Performance_Metric/RTA_Cache.h"  // PerCoreRTACache, ComputeRTA_FullAndCache, ClassifyReuse, RTAReuseClass (P1.9)
#include "sources/Safety_Performance_Metric/PrioritySwitchAnalysis.h"  // RestEqualAfterRemoving, AnalyzePrioritySwitch(PerCore), PrioritySwitchStatus/Analysis (P1.9 priority-analysis utilities)
#include "sources/Safety_Performance_Metric/SP_Metric.h"  // ObtainSP_Full_From_NodeRTAs (P1.12 2b SP-assembly differential)
#include "sources/Optimization/OptimizeSP_Base.h"  // EvaluateSPWithPriorityVec (the oracle the 2b swap replaced)
#include "sources/Optimization/OptimizeSP_TL_BF.h"  // UpdateExtDistBasedOnTimeLimit (TL-bake, mirrors :247 dag_tasks_cur)
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/readwrite.h"
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
    }

    // data members
    TaskSet tasks;
};
TEST_F(TaskSetForTest_2tasks, RTA) {
    GlobalVariables::Granularity = 10;
    std::vector<Value_Proba> dist_vec0 = {
        Value_Proba(1, 0.6), Value_Proba(2, 0.3), Value_Proba(3, 0.1)};
    FiniteDist rta0_expected(dist_vec0);

    std::vector<Value_Proba> dist_vec1 = {
        Value_Proba(5, 0.42),   Value_Proba(7, 0.234),  Value_Proba(8, 0.213),
        Value_Proba(9, 0.105),  Value_Proba(10, 0.025), Value_Proba(12, 0.0018),
        Value_Proba(13, 0.0012)};
    FiniteDist rta1_expected(dist_vec1);
    vector<FiniteDist> rtas_actual = ProbabilisticRTA_TaskSet_SingleCore(tasks);
    EXPECT_EQ(2, rtas_actual.size());
    EXPECT_EQ(rta0_expected, rtas_actual[0]);

    EXPECT_TRUE(rta1_expected == rtas_actual[1]);
    rta1_expected.print();
    rtas_actual[1].print();
}

class TaskSetForTest_2tasks_miss_ddl : public ::testing::Test {
   public:
    void SetUp() override {
        std::vector<Value_Proba> dist_vec1 = {Value_Proba(1, 0.6),
                                              Value_Proba(2, 0.4)};
        std::vector<Value_Proba> dist_vec2 = {Value_Proba(8, 0.4),
                                              Value_Proba(9, 0.6)};
        tasks.push_back(Task(0, dist_vec1, 4, 4, 0));
        tasks.push_back(Task(1, dist_vec2, 10, 10, 1));
    }

    // data members
    TaskSet tasks;
};
TEST_F(TaskSetForTest_2tasks_miss_ddl, RTA) {
    GlobalVariables::Granularity = 10;
    std::vector<Value_Proba> dist_vec0 = {Value_Proba(1, 0.6),
                                          Value_Proba(2, 0.4)};
    FiniteDist rta0_expected(dist_vec0);

    std::vector<Value_Proba> dist_vec1 = {Value_Proba(11, 1)};
    FiniteDist rta1_expected(dist_vec1);
    vector<FiniteDist> rtas_actual = ProbabilisticRTA_TaskSet_SingleCore(tasks);
    EXPECT_EQ(2, rtas_actual.size());
    EXPECT_EQ(rta0_expected, rtas_actual[0]);

    EXPECT_TRUE(rta1_expected == rtas_actual[1]);
    rta1_expected.print();
    rtas_actual[1].print();
}
class TaskSetForTest_2tasks_miss_partial_ddl : public ::testing::Test {
   public:
    void SetUp() override {
        std::vector<Value_Proba> dist_vec1 = {Value_Proba(1, 0.6),
                                              Value_Proba(2, 0.4)};
        std::vector<Value_Proba> dist_vec2 = {Value_Proba(1, 0.4),
                                              Value_Proba(9, 0.6)};
        tasks.push_back(Task(0, dist_vec1, 4, 4, 0));
        tasks.push_back(Task(1, dist_vec2, 10, 10, 1));
    }

    // data members
    TaskSet tasks;
};
TEST_F(TaskSetForTest_2tasks_miss_partial_ddl, RTA) {
    GlobalVariables::Granularity = 10;
    std::vector<Value_Proba> dist_vec0 = {Value_Proba(1, 0.6),
                                          Value_Proba(2, 0.4)};
    FiniteDist rta0_expected(dist_vec0);

    std::vector<Value_Proba> dist_vec1 = {
        Value_Proba(2, 0.24), Value_Proba(3, 0.16), Value_Proba(11, 0.6)};
    FiniteDist rta1_expected(dist_vec1);
    vector<FiniteDist> rtas_actual = ProbabilisticRTA_TaskSet_SingleCore(tasks);
    EXPECT_EQ(2, rtas_actual.size());
    EXPECT_EQ(rta0_expected, rtas_actual[0]);

    EXPECT_TRUE(rta1_expected == rtas_actual[1]);
    rta1_expected.print();
    rtas_actual[1].print();
}
TEST_F(TaskSetForTest_2tasks, RTA_change_priority) {
    GlobalVariables::Granularity = 10;
    std::vector<Value_Proba> dist_vec0 = {
        Value_Proba(1, 0.6), Value_Proba(2, 0.3), Value_Proba(3, 0.1)};
    FiniteDist rta0_expected(dist_vec0);

    std::vector<Value_Proba> dist_vec1 = {
        Value_Proba(5, 0.42),   Value_Proba(7, 0.234),  Value_Proba(8, 0.213),
        Value_Proba(9, 0.105),  Value_Proba(10, 0.025), Value_Proba(12, 0.0018),
        Value_Proba(13, 0.0012)};
    FiniteDist rta1_expected(dist_vec1);

    // swap task 0 and task 1
    Task temp = tasks[0];
    tasks[0] = tasks[1];
    tasks[1] = temp;

    vector<FiniteDist> rtas_actual = ProbabilisticRTA_TaskSet_SingleCore(tasks);
    EXPECT_EQ(2, rtas_actual.size());
    EXPECT_EQ(rta0_expected, rtas_actual[1]);
    EXPECT_TRUE(rta1_expected == rtas_actual[0]);
}

TEST_F(TaskSetForTest_2tasks, GetDDL_MissProbability) {
    GlobalVariables::Granularity = 10;
    std::vector<FiniteDist> rtas = ProbabilisticRTA_TaskSet_SingleCore(tasks);

    std::vector<Value_Proba> dist_vec0 = {
        Value_Proba(1, 0.6), Value_Proba(2, 0.3), Value_Proba(3, 0.1)};
    FiniteDist rta0_expected(dist_vec0);

    std::vector<Value_Proba> dist_vec1 = {
        Value_Proba(5, 0.42),   Value_Proba(7, 0.234),  Value_Proba(8, 0.213),
        Value_Proba(9, 0.105),  Value_Proba(10, 0.025), Value_Proba(12, 0.0018),
        Value_Proba(13, 0.0012)};
    FiniteDist rta1_expected(dist_vec1);
    vector<FiniteDist> rtas_actual = ProbabilisticRTA_TaskSet_SingleCore(tasks);
    EXPECT_NEAR(0.0012, GetDDL_MissProbability(rtas[1], 12), 1e-6);
}

TEST_F(TaskSetForTest_2tasks, GetDDL_MissProbability_v2) {
    GlobalVariables::Granularity = 10;
    FiniteDist dists({5, 6, 7, 8, 5, 6, 7, 8}, 10);

    EXPECT_NEAR(0.0, GetDDL_MissProbability(dists, 10), 1e-6);
    EXPECT_NEAR(1.0, GetDDL_MissProbability(dists, 4), 1e-6);
    EXPECT_NEAR(0.75, GetDDL_MissProbability(dists, 5), 1e-6);
    EXPECT_NEAR(0.5, GetDDL_MissProbability(dists, 6.5), 1e-6);
}
TEST_F(TaskSetForTest_2tasks, GetDDL_MissProbability_v3) {
    GlobalVariables::Granularity = 10;
    FiniteDist dists({5}, 10);

    EXPECT_NEAR(0.0, GetDDL_MissProbability(dists, 10), 1e-6);
    EXPECT_NEAR(1.0, GetDDL_MissProbability(dists, 4), 1e-6);
    EXPECT_NEAR(0.0, GetDDL_MissProbability(dists, 5), 1e-6);
}
TEST_F(TaskSetForTest_2tasks, GetDDL_MissProbability_v4) {
    GlobalVariables::Granularity = 10;
    FiniteDist dists({5, 10000, 10000}, 10);

    EXPECT_NEAR(0.66666666, GetDDL_MissProbability(dists, 10), 1e-3);
    EXPECT_NEAR(1.0, GetDDL_MissProbability(dists, 4), 1e-3);
    EXPECT_NEAR(0.66666666, GetDDL_MissProbability(dists, 5), 1e-3);
}
// 3-task single-core fixture for the HP-prefix checkpoint store (P1.9 step 2).
// priorities 0 < 1 < 2 → sorted HP-first order is exactly {t0, t1, t2}; gives
// hp_tasks_et_conv_vec non-trivial intermediate entries at indices 0,1,2.
class TaskSetForTest_3tasks_prefix : public ::testing::Test {
   public:
    void SetUp() override {
        GlobalVariables::Granularity = 10;
        std::vector<Value_Proba> d0 = {
            Value_Proba(1, 0.6), Value_Proba(2, 0.3), Value_Proba(3, 0.1)};
        std::vector<Value_Proba> d1 = {Value_Proba(4, 0.7),
                                       Value_Proba(5, 0.3)};
        std::vector<Value_Proba> d2 = {Value_Proba(2, 0.5),
                                       Value_Proba(6, 0.5)};
        // Task(id, exec, period, deadline, priority) — single-core RTA ignores
        // processorId (sorts by priority); 5-arg form matches the existing
        // fixtures above.
        tasks.push_back(Task(0, d0, 5, 5, 0));
        tasks.push_back(Task(1, d1, 12, 12, 1));
        tasks.push_back(Task(2, d2, 20, 20, 2));
    }
    TaskSet tasks;
};

// The 2-arg overload must reproduce the rolling hp_tasks_et_conv bit-for-bit:
// hp_tasks_et_conv_vec[i] == convolution of the higher-priority tasks' ET
// dists [0, i), with hp_tasks_et_conv_vec[0] == FiniteDist({Value_Proba(0,
// 1.0)}). This independently replays the rolling computation from
// RTA.cpp:73,81-82 and compares.
TEST_F(TaskSetForTest_3tasks_prefix, HpTasksEtConvVec_MatchesRollingValue) {
    std::vector<FiniteDist> rtas;
    std::vector<FiniteDist> hp_tasks_et_conv_vec;
    ProbabilisticRTA_TaskSet_SingleCore(tasks, hp_tasks_et_conv_vec);

    ASSERT_EQ(tasks.size(), hp_tasks_et_conv_vec.size());

    // Independent replay of the rolling hp_tasks_et_conv.
    FiniteDist rolling({Value_Proba(0, 1.0)});
    for (size_t i = 0; i < tasks.size(); i++) {
        EXPECT_TRUE(hp_tasks_et_conv_vec[i] == rolling)
            << "hp_tasks_et_conv_vec[" << i << "] != rolling value";
        rolling.CompressDistributionWithOnlySize(
            GlobalVariables::Granularity * 1);
        rolling.Convolve(tasks[i].execution_time_dist);
    }
}

// Behavior preservation: the 2-arg overload must return bit-identical rtas to
// the 1-arg version on the same input (the pinned-rtas tests above are the
// oracle; this adds an explicit same-input differential).
TEST_F(TaskSetForTest_3tasks_prefix, TwoArgOverload_SameRtasAsOneArg) {
    std::vector<FiniteDist> rtas_one_arg =
        ProbabilisticRTA_TaskSet_SingleCore(tasks);

    std::vector<FiniteDist> rtas_two_arg;
    std::vector<FiniteDist> hp_tasks_et_conv_vec_ignored;
    rtas_two_arg = ProbabilisticRTA_TaskSet_SingleCore(tasks, hp_tasks_et_conv_vec_ignored);

    ASSERT_EQ(rtas_one_arg.size(), rtas_two_arg.size());
    for (size_t i = 0; i < rtas_one_arg.size(); i++) {
        EXPECT_TRUE(rtas_one_arg[i] == rtas_two_arg[i])
            << "rtas[" << i << "] diverged between 1-arg and 2-arg overloads";
    }
}

// 4-task, 2-core fixture for the per-core RTA cache (P1.9 step 3c). Two tasks
// per core so ExtractTaskSetPerProcessor and the per-core cache are exercised
// non-trivially (v9 has only one task per core, which can't). Priorities are
// unique across the whole set; processorId partitions {0,1} vs {2,3}. The cache
// fn takes the FULL input tuple (dag, pa_vec, tl_vec), so the fixture holds all
// three. processorId is set on the tasks BEFORE DAG_Model construction (DAG_Model
// records task positions / categories at ctor time); DAG_Model(tasks, {}, {})
// builds an edge-free DAG (no cause-effect chains — fine for RTA-only tests).
// Task(id, exec, period, deadline, priority) — processorId is a default-0
// member set post-construction, matching how the existing fixtures set up tasks.
class TaskSetForTest_4tasks_2cores_cache : public ::testing::Test {
   public:
    void SetUp() override {
        GlobalVariables::Granularity = 10;
        std::vector<Value_Proba> d0 = {
            Value_Proba(1, 0.6), Value_Proba(2, 0.3), Value_Proba(3, 0.1)};
        std::vector<Value_Proba> d1 = {Value_Proba(4, 0.7),
                                       Value_Proba(5, 0.3)};
        std::vector<Value_Proba> d2 = {Value_Proba(2, 0.5),
                                       Value_Proba(6, 0.5)};
        std::vector<Value_Proba> d3 = {Value_Proba(3, 0.8),
                                       Value_Proba(7, 0.2)};
        tasks.push_back(Task(0, d0, 5, 5, 0));
        tasks.push_back(Task(1, d1, 12, 12, 1));
        tasks.push_back(Task(2, d2, 20, 20, 2));
        tasks.push_back(Task(3, d3, 30, 30, 3));
        // processorId is a default-0 member set post-construction (see the
        // Task ctor in RegularTasks.h). {0,1} on core 0; {2,3} on core 1.
        tasks[0].processorId = 0;
        tasks[1].processorId = 0;
        tasks[2].processorId = 1;
        tasks[3].processorId = 1;
        // Task 2 carries a perf pair (Hazard B coverage: a TL-flexible task
        // whose perf_coefficient != 1.0). Its ET avg ~4 → perf 0.5 at TL 4.
        tasks[2].timePerformancePairs = {TimePerfPair(2, 1.0),
                                         TimePerfPair(6, 0.5)};
        // priority_vec[i] = task id at priority index i (small index = high
        // priority). Here the ctor priorities already match id order, so the
        // HP-first per-core order is core0 {t0,t1}, core1 {t2,t3}.
        priority_vec = {0, 1, 2, 3};
        // -1 = no TL (keep the base Gaussian dist). Patches mutate individual
        // entries; full-passthrough TLs pin the differential vs the no-TL
        // oracle, and the TL-specific test toggles one entry.
        time_limits = {-1, -1, -1, -1};
        // One chain (t0 → t2) so DAG-level RTDA terms are non-trivial; the
        // cache's flat-by-id rta_ must still align with dag.tasks position.
        dag_tasks = DAG_Model(tasks, {{0, 2}}, {1e9});
    }
    TaskSet tasks;
    PriorityVec priority_vec;
    std::vector<double> time_limits;
    DAG_Model dag_tasks;
};

// Helper: apply pa_vec + tl_vec the SAME way the cache fn does, then call the
// old ProbabilisticRTA_TaskSet (the oracle). The cache fn's whole job is to
// reproduce this; bit-identical here means the cache path is a sound drop-in.
static std::vector<FiniteDist> OracleRtas(
    const DAG_Model& dag_tasks, const PriorityVec& priority_assignment,
    const std::vector<double>& time_limits) {
    TaskSet tasks_with_tl =
        ApplyTimeLimitsToTasksExecutionTime(dag_tasks.tasks, time_limits);
    TaskSet tasks_prioritized =
        UpdateTaskSetPriorities(tasks_with_tl, priority_assignment);
    return ProbabilisticRTA_TaskSet(tasks_prioritized);
}

// RTACache::Initialize must return bit-identical rtas to the oracle (apply TLs
// → apply pa_vec → ProbabilisticRTA_TaskSet) on the same input tuple. The
// acceptance gate: every cache path reproduces the full-recompute result.
TEST_F(TaskSetForTest_4tasks_2cores_cache, Initialize_SameRtasAs_Oracle) {
    std::vector<FiniteDist> rtas_oracle =
        OracleRtas(dag_tasks, priority_vec, time_limits);

    RTACache cache;
    const std::vector<FiniteDist>& rtas_cached =
        cache.Initialize(dag_tasks, priority_vec, time_limits);

    ASSERT_EQ(rtas_oracle.size(), rtas_cached.size());
    for (size_t i = 0; i < rtas_oracle.size(); i++) {
        EXPECT_TRUE(rtas_oracle[i] == rtas_cached[i])
            << "rtas[" << i << "] diverged between oracle and Initialize";
    }
    EXPECT_TRUE(cache.HasChampion());
}

// Initialize with a real TL applied (point-mass ET via GetUnitExecutionTimeDist)
// must still match the oracle bit-for-bit. Pins that the cache's TL-bake
// reproduces TL-driven RTA exactly.
TEST_F(TaskSetForTest_4tasks_2cores_cache, Initialize_WithTL_SameRtasAs_Oracle) {
    // Task 1 (core 0, lower priority on its core) gets TL 3 → only task 1's RTA
    // (the suffix) changes; task 0's HP set is unaffected.
    std::vector<double> tl = {-1, 3, -1, -1};

    std::vector<FiniteDist> rtas_oracle = OracleRtas(dag_tasks, priority_vec, tl);

    RTACache cache;
    const std::vector<FiniteDist>& rtas_cached =
        cache.Initialize(dag_tasks, priority_vec, tl);

    ASSERT_EQ(rtas_oracle.size(), rtas_cached.size());
    for (size_t i = 0; i < rtas_oracle.size(); i++) {
        EXPECT_TRUE(rtas_oracle[i] == rtas_cached[i])
            << "rtas[" << i << "] diverged under TL between oracle and cache";
    }
}

// Evaluate on a candidate identical to the champion → FullReuse: returns the
// champion rtas verbatim (zero RTA work), bit-identical to the oracle.
TEST_F(TaskSetForTest_4tasks_2cores_cache,
       Evaluate_IdentityCandidate_FullReuse) {
    RTACache cache;
    cache.Initialize(dag_tasks, priority_vec, time_limits);

    std::vector<FiniteDist> rtas_oracle =
        OracleRtas(dag_tasks, priority_vec, time_limits);
    const std::vector<FiniteDist>& rtas_eval =
        cache.Evaluate(dag_tasks, priority_vec, time_limits);

    ASSERT_EQ(rtas_oracle.size(), rtas_eval.size());
    for (size_t i = 0; i < rtas_oracle.size(); i++) {
        EXPECT_TRUE(rtas_oracle[i] == rtas_eval[i])
            << "rtas[" << i << "] diverged on identity Evaluate";
    }
    // Champion state untouched: a second identity Evaluate still matches.
    const std::vector<FiniteDist>& rtas_eval2 =
        cache.Evaluate(dag_tasks, priority_vec, time_limits);
    for (size_t i = 0; i < rtas_oracle.size(); i++) {
        EXPECT_TRUE(rtas_oracle[i] == rtas_eval2[i]);
    }
}

// P1.17 task 1c — pin that Evaluate fills EVERY candidate_rta_ slot from the
// reindex + recompute loops alone, with no reliance on the prior zero-init.
// Identity candidate is the strongest case: diff.changed_task_id == -1 so
// any_recompute == false and the recompute loop is skipped ENTIRELY (the early
// return at the |diff|==0 branch). Every slot must therefore be filled by the
// reindex loop (champion RTA reindexed by task id) — a regression that left any
// slot at the default-constructed FiniteDist would diverge from the oracle.
// This is the case that catches "drop the assign but a FullReuse slot goes
// unwritten".
TEST_F(TaskSetForTest_4tasks_2cores_cache,
       Evaluate_IdentityCandidate_EverySlotFilled_NoZeroInit) {
    RTACache cache;
    cache.Initialize(dag_tasks, priority_vec, time_limits);

    std::vector<FiniteDist> rtas_oracle =
        OracleRtas(dag_tasks, priority_vec, time_limits);
    const std::vector<FiniteDist>& rtas_eval =
        cache.Evaluate(dag_tasks, priority_vec, time_limits);

    ASSERT_EQ(rtas_oracle.size(), rtas_eval.size());
    // A default-constructed FiniteDist is empty; an oracle entry is a real
    // distribution. If a slot were left unwritten, this fails on emptiness OR
    // value (the identity-zero FiniteDist({Value_Proba(0,1.0)}) the prior
    // assign used would also fail against a non-degenerate oracle entry).
    for (size_t i = 0; i < rtas_oracle.size(); i++) {
        EXPECT_FALSE(rtas_eval[i].distribution.empty())
            << "rtas[" << i << "] was left unwritten (default-constructed)";
        EXPECT_TRUE(rtas_oracle[i] == rtas_eval[i])
            << "rtas[" << i << "] diverged on identity Evaluate (no zero-init)";
    }
}

// Evaluate on a candidate that differs by ONE task's TL (the Type-L serialized
// step) → ReuseHpTasksEt: patches the changed task's suffix via the stored
// HP-prefix, bit-identical to the oracle. Core 1 (untouched) reused verbatim.
TEST_F(TaskSetForTest_4tasks_2cores_cache, Evaluate_TLChange_OneTaskPatch) {
    RTACache cache;
    cache.Initialize(dag_tasks, priority_vec, time_limits);

    // Candidate: task 1's TL moves -1 → 3 (one task's ET changes on core 0).
    std::vector<double> tl_cand = {-1, 3, -1, -1};
    std::vector<FiniteDist> rtas_oracle =
        OracleRtas(dag_tasks, priority_vec, tl_cand);
    const std::vector<FiniteDist>& rtas_eval =
        cache.Evaluate(dag_tasks, priority_vec, tl_cand);

    ASSERT_EQ(rtas_oracle.size(), rtas_eval.size());
    for (size_t i = 0; i < rtas_oracle.size(); i++) {
        EXPECT_TRUE(rtas_oracle[i] == rtas_eval[i])
            << "rtas[" << i << "] diverged on TL-change Evaluate";
    }
}

// Evaluate on a candidate that differs by ONE task's priority position (the
// O(N²) priority-move case) → ReuseHpTasksEt: patches the moved task + its
// suffix via the stored HP-prefix, bit-identical to the oracle.
TEST_F(TaskSetForTest_4tasks_2cores_cache,
       Evaluate_PriorityMove_OneTaskPatch) {
    RTACache cache;
    cache.Initialize(dag_tasks, priority_vec, time_limits);

    // Candidate: swap tasks 0 and 1's priority positions on core 0
    // (priority_vec[0]=1, [1]=0 → core 0 order {t1, t0}).
    PriorityVec pa_cand = {1, 0, 2, 3};
    std::vector<FiniteDist> rtas_oracle =
        OracleRtas(dag_tasks, pa_cand, time_limits);
    const std::vector<FiniteDist>& rtas_eval =
        cache.Evaluate(dag_tasks, pa_cand, time_limits);

    ASSERT_EQ(rtas_oracle.size(), rtas_eval.size());
    for (size_t i = 0; i < rtas_oracle.size(); i++) {
        EXPECT_TRUE(rtas_oracle[i] == rtas_eval[i])
            << "rtas[" << i << "] diverged on priority-move Evaluate";
    }
}

// Evaluate on a candidate that differs by BOTH the moved task's ET AND its
// priority position (the combined ET+move case) → still one merged change →
// ReuseHpTasksEt, bit-identical to the oracle.
TEST_F(TaskSetForTest_4tasks_2cores_cache,
       Evaluate_TLAndPriorityMove_CombinedPatch) {
    RTACache cache;
    cache.Initialize(dag_tasks, priority_vec, time_limits);

    // Candidate: task 1's TL → 3 AND tasks 0/1 priority swap.
    PriorityVec pa_cand = {1, 0, 2, 3};
    std::vector<double> tl_cand = {-1, 3, -1, -1};
    std::vector<FiniteDist> rtas_oracle =
        OracleRtas(dag_tasks, pa_cand, tl_cand);
    const std::vector<FiniteDist>& rtas_eval =
        cache.Evaluate(dag_tasks, pa_cand, tl_cand);

    ASSERT_EQ(rtas_oracle.size(), rtas_eval.size());
    for (size_t i = 0; i < rtas_oracle.size(); i++) {
        EXPECT_TRUE(rtas_oracle[i] == rtas_eval[i])
            << "rtas[" << i << "] diverged on combined TL+move Evaluate";
    }
}

// P1.17 task 1a remainder — pin that champ_tasks_baked_ (the cached champion
// TL-bake IsSingleTaskChange reads instead of re-baking) is REFRESHED on
// AdoptChampion. The hazard this cache introduces: if champ_tasks_baked_ were
// left holding the FIRST champion's baked tasks after a second AdoptChampion
// with a different TL, FindTaskWithDifferentEt would compare the candidate
// against the stale bake → mis-classify the ET diff (wrong changed_task_id, or
// a false |diff|>1 throw). So: adopt a 2nd champion whose TL differs from the
// 1st, then Evaluate a candidate that differs from the 2nd by one task's ET.
// Bit-identical to the oracle AND the diff must land on the right task.
TEST_F(TaskSetForTest_4tasks_2cores_cache,
       Evaluate_TLChange_AfterAdoptChampion_StaleBakeGuard) {
    RTACache cache;
    // 1st champion: task 1 has no TL (-1).
    cache.Initialize(dag_tasks, priority_vec, time_limits);

    // 2nd champion: task 1's TL → 5. AdoptChampion must refresh champ_tasks_baked_
    // to THIS bake, not the 1st champion's.
    std::vector<double> tl_champ2 = {-1, 5, -1, -1};
    std::vector<FiniteDist> rtas_champ2 =
        OracleRtas(dag_tasks, priority_vec, tl_champ2);
    cache.AdoptChampion(dag_tasks, priority_vec, tl_champ2, rtas_champ2);

    // Candidate: task 1's TL → 3 (one ET change vs the 2nd champion, on core 0).
    std::vector<double> tl_cand = {-1, 3, -1, -1};
    std::vector<FiniteDist> rtas_oracle =
        OracleRtas(dag_tasks, priority_vec, tl_cand);
    const std::vector<FiniteDist>& rtas_eval =
        cache.Evaluate(dag_tasks, priority_vec, tl_cand);

    ASSERT_EQ(rtas_oracle.size(), rtas_eval.size());
    for (size_t i = 0; i < rtas_oracle.size(); i++) {
        EXPECT_TRUE(rtas_oracle[i] == rtas_eval[i])
            << "rtas[" << i << "] diverged after AdoptChampion (stale bake?)";
    }

    // The single change must be located on task 1 (the only ET-changed task vs
    // the 2nd champion). A stale champ_tasks_baked_ (still the 1st champion's
    // -1 bake) would see task 1 as changed AND mis-locate, or throw.
    TaskSetDifference d =
        cache.ComputeTaskSetDifference(dag_tasks, priority_vec, tl_cand);
    EXPECT_EQ(d.changed_task_id, 1);
    EXPECT_EQ(d.core, 0);
}

// P1.17 task 1b remainder — pin that the champion per-core order cached on
// Initialize/AdoptChampion is REFRESHED on AdoptChampion. The hazard this cache
// introduces: IsSingleTaskChange reads the cached champion per-core order to
// diff against the candidate; if it were left holding the FIRST champion's order
// after a second AdoptChampion whose PA differs, AnalyzePrioritySwitch would
// compare the candidate against the stale order → mis-locate the changed core /
// moved task, or a false |diff|>1 throw. So: adopt a 2nd champion whose PA swaps
// two tasks on core 0, then Evaluate a candidate that differs from the 2nd by one
// task's priority position on core 0. Bit-identical to the oracle AND the diff
// must land on the right task + core.
TEST_F(TaskSetForTest_4tasks_2cores_cache,
       Evaluate_PriorityMove_AfterAdoptChampion_StalePerCoreOrderGuard) {
    RTACache cache;
    // 1st champion: PA {0,1,2,3} → core0 order {t0,t1}, core1 order {t2,t3}.
    cache.Initialize(dag_tasks, priority_vec, time_limits);

    // 2nd champion: swap tasks 0/1 on core 0 → PA {1,0,2,3}, so the champion
    // core0 order is now {t1,t0}. AdoptChampion must refresh the cached champion
    // per-core order to THIS order, not the 1st champion's {t0,t1}.
    PriorityVec pa_champ2 = {1, 0, 2, 3};
    std::vector<FiniteDist> rtas_champ2 =
        OracleRtas(dag_tasks, pa_champ2, time_limits);
    cache.AdoptChampion(dag_tasks, pa_champ2, time_limits, rtas_champ2);

    // Candidate: PA {0,1,2,3} — vs the 2nd champion this is a single priority
    // swap of t0/t1 on core 0 (champion order {t1,t0} → candidate order {t0,t1}).
    // A stale champion per-core order ({t0,t1}, the 1st champion's) would see NO
    // order diff on core 0 → wrongly classify |diff|==0 (or a throw if the ET
    // cross-check disagrees) and Evaluate would return the 2nd champion's rta
    // verbatim (bit-non-identical to the candidate's true rta). Expectation: a
    // change IS detected on core 0 (the swap is symmetric, so the algorithm
    // reports the candidate-side task at the first mismatch, task 0 — the exact
    // id is not load-bearing, only that it is != -1 and on core 0).
    std::vector<FiniteDist> rtas_oracle =
        OracleRtas(dag_tasks, priority_vec, time_limits);
    const std::vector<FiniteDist>& rtas_eval =
        cache.Evaluate(dag_tasks, priority_vec, time_limits);

    ASSERT_EQ(rtas_oracle.size(), rtas_eval.size());
    for (size_t i = 0; i < rtas_oracle.size(); i++) {
        EXPECT_TRUE(rtas_oracle[i] == rtas_eval[i])
            << "rtas[" << i << "] diverged after AdoptChampion (stale per-core "
            << "order?)";
    }

    TaskSetDifference d =
        cache.ComputeTaskSetDifference(dag_tasks, priority_vec, time_limits);
    EXPECT_NE(d.changed_task_id, -1);
    EXPECT_EQ(d.core, 0);
}

// P1.25 — the P1.21 RTACache::Transaction (lazy copy-on-write) layer has been
// REMOVED. Its three former unit pins (Transaction_Commit_KeepsInTxAdopt /
// Transaction_NoCommit_RollsBackInTxAdopt /
// Transaction_NoAdopt_NoCommit_ChampionUntouched) tested the RAII guard's
// snapshot/commit/rollback machinery directly, so they cannot exist without the
// Transaction class. The load-bearing contract they protected — "a rejected
// sub-incremental walk step must NOT leave the cache champion desynced from
// res_opt_" — is now covered at the WALK level by the two P1.25 pins in
// tests/testIncreOpt_w_TL.cpp (SubIncrementalReject_RevertKeepsChampionOn
// CommittedTriple + SubIncrementalAccept_ChampionTracksCommittedTriple), which
// exercise the D1=(b) eager save/restore that replaces the transaction.

// P1.12 Phase 2 item 1a — the MISSING differential that localizes the :285
// divergence. The existing Evaluate_PriorityMove_OneTaskPatch above swaps two
// tasks on the SAME core (core0={t0,t1}); core0 → NoReuse (both recomputed),
// core1 FullReuse tasks (t2,t3) keep their priority-positions (2,3) → no
// scramble. That shape does NOT exercise the bug. THIS test moves a task on
// core1 to the front (priority-position 0), making core1 the changed core
// (NoReuse) and core0 the FullReuse core — but core0's tasks (t0,t1) now occupy
// priority-positions 1,2 (were 0,1) in the candidate. The champion rta_ is
// indexed by CHAMPION priority-position, so seeding candidate_rta_ = rta_ puts
// t0's champion RTA in slot 1 (candidate priority-position of t1) and t1's in
// slot 2 — a scramble. Expectation: cache.Evaluate MUST be bit-identical to the
// oracle per-task (priority-position indexed). RED on the bug; GREEN after the
// seeding reindex fix.
TEST_F(TaskSetForTest_4tasks_2cores_cache,
       Evaluate_PriorityMove_CrossCoreScramble_BitIdenticalToOracle) {
    RTACache cache;
    cache.Initialize(dag_tasks, priority_vec, time_limits);

    // Move task 2 (core1) to the front. Champion per-core order was
    // core0={t0,t1}, core1={t2,t3}; candidate is core0={t0,t1}, core1={t2,t3}
    // but task 2 now has the global highest priority. The changed core is core1
    // (task 2 moved); core0 is FullReuse — and core0's tasks shift priority-
    // positions (0,1 → 1,2), which is what trips the seeding scramble.
    PriorityVec pa_cand = {2, 0, 1, 3};
    std::vector<FiniteDist> rtas_oracle =
        OracleRtas(dag_tasks, pa_cand, time_limits);
    const std::vector<FiniteDist>& rtas_eval =
        cache.Evaluate(dag_tasks, pa_cand, time_limits);

    ASSERT_EQ(rtas_oracle.size(), rtas_eval.size());
    for (size_t i = 0; i < rtas_oracle.size(); i++) {
        EXPECT_TRUE(rtas_oracle[i] == rtas_eval[i])
            << "rtas[" << i << "] diverged on cross-core-scramble Evaluate "
            << "(champion priority-position indexing ≠ candidate's on FullReuse "
            << "core0 tasks t0/t1)";
    }
}

// P1.12 Phase 2 item 1a (SP-level pin) — the same scramble, asserted at the SP
// level via ObtainSP_Full_From_NodeRTAs (the consumer the :285 seam uses) vs
// EvaluateSPWithPriorityVec (the oracle). This is the direct primitive-level
// analogue of the OptimizeWithOptimizationSpace gate failure.
//
// Uses a CHAIN-FREE local DAG (not the fixture's chained `dag_tasks`, which has
// the chain t0→t2). The scramble PA `pa_cand = {2,0,1,3}` puts task 2 (the chain
// sink) at a HIGHER priority than task 0 (the chain source) → an infeasible
// cause-before-effect schedule → `GetFinishTime` aborts ("Schedule didn't find
// job!") on BOTH the cache arm AND the oracle arm (verified: the oracle crashes
// identically). The chain-free DAG removes that constraint so the scramble PA is
// feasible and the SP-level bit-identity can be pinned. The per-task RTA test
// above does NOT go through the schedule path (ProbabilisticRTA_TaskSet only),
// so it stays on the chained fixture.
TEST_F(TaskSetForTest_4tasks_2cores_cache,
       Evaluate_PriorityMove_CrossCoreScramble_SP_BitIdenticalToOracle) {
    // Chain-free DAG: same tasks/cores as the fixture, no cause-effect edges.
    DAG_Model dag_chainfree(tasks, {}, {});
    SP_Parameters sp(dag_chainfree);
    RTACache cache;
    cache.Initialize(dag_chainfree, priority_vec, time_limits);

    // Move task 2 (core1) to the front: changed core = core1 (NoReuse for t2,t3),
    // core0 (t0,t1) FullReuse and shifted to priority-positions 1,2 (were 0,1) —
    // the shape that trips the seeding scramble when candidate PA != champion PA.
    PriorityVec pa_cand = {2, 0, 1, 3};
    double sp_oracle = EvaluateSPWithPriorityVec(dag_chainfree, sp, pa_cand);
    const std::vector<FiniteDist>& rtas_eval =
        cache.Evaluate(dag_chainfree, pa_cand, time_limits);
    double sp_cache = ObtainSP_Full_From_NodeRTAs(
        dag_chainfree, sp, pa_cand, time_limits, rtas_eval);

    EXPECT_DOUBLE_EQ(sp_oracle, sp_cache)
        << "SP diverged on cross-core-scramble: cache Evaluate + "
        << "ObtainSP_Full_From_NodeRTAs != oracle EvaluateSPWithPriorityVec";
}

// Evaluate with no champion → Initialize (full compute), bit-identical to the
// oracle. The empty-cache fallback.
TEST_F(TaskSetForTest_4tasks_2cores_cache, Evaluate_NoChampion_FallsBackToInit) {
    RTACache cache;
    EXPECT_FALSE(cache.HasChampion());

    std::vector<FiniteDist> rtas_oracle =
        OracleRtas(dag_tasks, priority_vec, time_limits);
    const std::vector<FiniteDist>& rtas_eval =
        cache.Evaluate(dag_tasks, priority_vec, time_limits);

    ASSERT_EQ(rtas_oracle.size(), rtas_eval.size());
    for (size_t i = 0; i < rtas_oracle.size(); i++) {
        EXPECT_TRUE(rtas_oracle[i] == rtas_eval[i])
            << "rtas[" << i << "] diverged on no-champion Evaluate";
    }
    EXPECT_TRUE(cache.HasChampion());
}

// P1.12 Phase 1 step 3 — SP-ASSEMBLY differential (TL-walk). The Evaluate_* tests
// above pin that the cache's RTAS are bit-identical to the oracle; this pins that
// the SP assembled ON TOP of those rtas — `ObtainSP_Full_From_NodeRTAs(dag, sp,
// pa, tl, cache.Evaluate(...))` — is bit-identical to the oracle
// `EvaluateSPWithPriorityVec(UpdateExtDistBasedOnTimeLimit(dag, tl), sp, pa)` it
// replaced at the :247 baseline re-score (commit 5a172973). The end-to-end gate
// `testIncreOpt_w_TL::OptimizeWithOptimizationSpace` covers this only indirectly
// (through the full optimizer); these tests isolate the bit-identity at the
// exact seam, on the two branches the serialized TL walk serves.
//
// Helper: the oracle SP for a (dag, pa, tl) triple = bake TLs into the dag, then
// EvaluateSPWithPriorityVec. This is exactly what EvaluateTimeLimitConfig_
// SubIncremental did BEFORE the 2b swap (dag_tasks_cur = UpdateExtDistBasedOn-
// TimeLimit(dag_tasks_, time_limits); EvaluateSPWithPriorityVec(dag_tasks_cur,
// sp_parameters_, pa)). The cache arm does NOT pre-bake (the cache bakes TLs
// internally via ApplyTimeLimitsToTasksExecutionTime, and ObtainSP_Full_From_-
// NodeRTAs re-derives the prioritized TL-baked form identically — SP_Metric.h
// contract). Both must land on the same double.
static double OracleSP(const DAG_Model& dag_tasks,
                       const SP_Parameters& sp_parameters,
                       const PriorityVec& priority_assignment,
                       const std::vector<double>& time_limits) {
    DAG_Model dag_with_tl =
        UpdateExtDistBasedOnTimeLimit(dag_tasks, time_limits);
    return EvaluateSPWithPriorityVec(dag_with_tl, sp_parameters,
                                     priority_assignment);
}

// Type-L: the candidate differs from the champion by ONE task's TL. Task 2 has
// timePerformancePairs {(2,1.0),(6,0.5)} → a TL on it moves BOTH its ET (Hazard
// B surface: perf_coefficient != 1.0) and its RTA. This is the |diff|==1 patch
// branch — the cache patches task 2's RTA, ObtainSP_Full_From_NodeRTAs assembles
// the SP, and the result must equal the oracle's full recompute.
TEST_F(TaskSetForTest_4tasks_2cores_cache,
       SP_Assembly_TypeLChange_BitIdenticalToOracle) {
    SP_Parameters sp(dag_tasks);
    // Champion = no TLs anywhere. Candidate = task 2's TL moved to 6 (its other
    // perf pair) → one task's TL changed vs the champion.
    std::vector<double> tl_champion = {-1, -1, -1, -1};
    std::vector<double> tl_candidate = {-1, -1, 6, -1};

    RTACache cache;
    cache.Initialize(dag_tasks, priority_vec, tl_champion);

    double sp_oracle = OracleSP(dag_tasks, sp, priority_vec, tl_candidate);
    const std::vector<FiniteDist>& rtas =
        cache.Evaluate(dag_tasks, priority_vec, tl_candidate);
    double sp_cache = ObtainSP_Full_From_NodeRTAs(dag_tasks, sp, priority_vec,
                                                  tl_candidate, rtas);

    EXPECT_DOUBLE_EQ(sp_oracle, sp_cache)
        << "Type-L SP assembly diverged: oracle=" << sp_oracle
        << " cache=" << sp_cache;
}

// Type-E: the candidate == the champion (zero diff). The serialized walk hits
// this on every Type-E env entry (trial TL == committed TL, env absorbed on
// both diff sides) → |diff|==0 → Evaluate short-circuits to FullReuse. The SP
// must still be bit-identical (the FullReuse rtas are the champion's, and the
// assembly over them must match a fresh oracle recompute). Champion carries a
// real TL so the rtas are TL-baked, not the no-TL passthrough.
TEST_F(TaskSetForTest_4tasks_2cores_cache,
       SP_Assembly_TypeE_NoChange_BitIdenticalToOracle) {
    SP_Parameters sp(dag_tasks);
    std::vector<double> tl = {-1, -1, 6, -1};  // task 2 TL'd on both sides

    RTACache cache;
    cache.Initialize(dag_tasks, priority_vec, tl);

    double sp_oracle = OracleSP(dag_tasks, sp, priority_vec, tl);
    const std::vector<FiniteDist>& rtas =
        cache.Evaluate(dag_tasks, priority_vec, tl);
    double sp_cache = ObtainSP_Full_From_NodeRTAs(dag_tasks, sp, priority_vec,
                                                  tl, rtas);

    EXPECT_DOUBLE_EQ(sp_oracle, sp_cache)
        << "Type-E SP assembly diverged: oracle=" << sp_oracle
        << " cache=" << sp_cache;
}

// P1.12 2b BLOCKER reproduction — 3 tasks on ONE core. With 2 tasks the NoReuse
// lower-priority task has only ONE HP task, so the 2-arg path Compresses the
// running RTA exactly once (== the oracle's single Compress) → bit-identical,
// masking the bug. With 3 tasks the lowest-priority NoReuse task has TWO HP
// tasks → the 2-arg path Compresses the running RTA TWICE (once per HP task)
// while the oracle Compresses it ONCE → the lossy bucket-merge diverges once
// the convolved support crosses Granularity. This is the minimal fixture that
// reproduces the 2b divergence in isolation.
class TaskSetForTest_3tasks_1core_wideET : public ::testing::Test {
   public:
    void SetUp() override {
        GlobalVariables::Granularity = 10;
        // Wide Gaussians truncated to [20,80] at granularity 5. Three tasks'
        // ETs convolve to ≈[60,240] — well past Granularity=10, so the lossy
        // bucket-merge in CompressDistributionWithOnlySize fires.
        FiniteDist dist_wide0 =
            FiniteDist(GaussianDist(50, 8), 20, 80, 5);
        FiniteDist dist_wide1 =
            FiniteDist(GaussianDist(55, 8), 20, 80, 5);
        FiniteDist dist_wide2 =
            FiniteDist(GaussianDist(60, 8), 20, 80, 5);
        tasks.push_back(Task(0, dist_wide0, 200, 200, 0));
        tasks.push_back(Task(1, dist_wide1, 400, 400, 1));
        tasks.push_back(Task(2, dist_wide2, 800, 800, 2));
        // All three on core 0 so the lowest-priority NoReuse task's HP set holds
        // TWO wide-ET higher-priority tasks (the compress-count divergence).
        tasks[0].processorId = 0;
        tasks[1].processorId = 0;
        tasks[2].processorId = 0;
        priority_vec = {0, 1, 2};
        time_limits = {-1, -1, -1};
        dag_tasks = DAG_Model(tasks, {}, {1e9});
    }
    TaskSet tasks;
    PriorityVec priority_vec;
    std::vector<double> time_limits;
    DAG_Model dag_tasks;
};

// P1.12 2b BLOCKER: RTACache::Evaluate's NoReuse recompute path must be
// bit-identical to the oracle (ProbabilisticRTA_TaskSet) even when the convolved
// ET support crosses Granularity with ≥2 HP tasks. On HEAD this FAILS: Evaluate's
// NoReuse path (RTA_Cache.cpp:458) calls the 2-arg GetRTA_OneTask(task, hp_tasks),
// which Compresses+Convolves PER HP task on the running RTA; the oracle
// (RTA.cpp:88-113) Compresses the running RTA ONCE then Convolves against a
// precomputed rolling HP-ET convolution. CompressDistributionWithOnlySize is
// LOSSY, so the differing compress count (oracle=1, 2-arg=#HP-tasks) diverges
// once the support grows past Granularity. This test reproduces that divergence
// in isolation (vs the full testIncreOpt_w_TL integration that first surfaced it).
TEST_F(TaskSetForTest_3tasks_1core_wideET,
       Evaluate_NoReuseBitIdenticalToOracle_WhenSupportCrossesGranularity) {
    RTACache cache;
    cache.Initialize(dag_tasks, priority_vec, time_limits);

    // Candidate: task 2's TL moves -1 → 30 (point-mass ET via
    // GetUnitExecutionTimeDist). The whole core-0 set is NoReuse; task 2's HP
    // set is {task 0, task 1} with wide Gaussian ETs → the recompute convolves
    // the running RTA past Granularity with TWO HP iterations, where the
    // 2-arg compress-count divergence surfaces.
    std::vector<double> tl_cand = {-1, -1, 30};
    std::vector<FiniteDist> rtas_oracle =
        OracleRtas(dag_tasks, priority_vec, tl_cand);
    const std::vector<FiniteDist>& rtas_eval =
        cache.Evaluate(dag_tasks, priority_vec, tl_cand);

    ASSERT_EQ(rtas_oracle.size(), rtas_eval.size());
    for (size_t i = 0; i < rtas_oracle.size(); i++) {
        EXPECT_TRUE(rtas_oracle[i] == rtas_eval[i])
            << "rtas[" << i << "] diverged on wide-ET NoReuse Evaluate";
    }
}

// P1.12 2b BLOCKER — the priority-MOVE shape (the actual integration form).
// OptimizeIncre_SingleTask generates 1D priority-move candidates (one task's
// priority position changes vs the champion) and scores each via Evaluate +
// AdoptChampion on strict-improve. This mirrors that exact sequence on a single
// shared cache: Initialize the champion, then Evaluate a priority-move candidate
// (task 2 moved to the HIGHEST priority → {2,0,1}), adopt it, then Evaluate a
// SECOND move from the new champion. The multi-step AdoptChampion→Evaluate
// lifecycle (not the single isolated Evaluate above) is what the integration
// exercised when it diverged. HEAD should FAIL here if the blocker reproduces.
TEST_F(TaskSetForTest_3tasks_1core_wideET,
       Evaluate_PriorityMoveSequence_BitIdenticalToOracle) {
    RTACache cache;
    // Champion = identity PA, wide Gaussians, no TL.
    cache.Initialize(dag_tasks, priority_vec, time_limits);

    // Step 1: move task 2 to highest priority → {2, 0, 1}. Single priority
    // move vs the champion → NoReuse on core 0 (the whole core's HP sets
    // shift). Adopt the result so the next step patches vs the new champion.
    PriorityVec pa_move1 = {2, 0, 1};
    std::vector<FiniteDist> rtas_oracle_move1 =
        OracleRtas(dag_tasks, pa_move1, time_limits);
    const std::vector<FiniteDist>& rtas_eval_move1 =
        cache.Evaluate(dag_tasks, pa_move1, time_limits);
    for (size_t i = 0; i < rtas_oracle_move1.size(); i++) {
        EXPECT_TRUE(rtas_oracle_move1[i] == rtas_eval_move1[i])
            << "move1 rtas[" << i << "] diverged";
    }
    cache.AdoptChampion(dag_tasks, pa_move1, time_limits, rtas_eval_move1);

    // Step 2: from the new champion {2,0,1}, move task 0 to lowest priority →
    // {2, 1, 0}. Another single priority move → NoReuse on core 0.
    PriorityVec pa_move2 = {2, 1, 0};
    std::vector<FiniteDist> rtas_oracle_move2 =
        OracleRtas(dag_tasks, pa_move2, time_limits);
    const std::vector<FiniteDist>& rtas_eval_move2 =
        cache.Evaluate(dag_tasks, pa_move2, time_limits);
    for (size_t i = 0; i < rtas_oracle_move2.size(); i++) {
        EXPECT_TRUE(rtas_oracle_move2[i] == rtas_eval_move2[i])
            << "move2 rtas[" << i << "] diverged (post-AdoptChampion)";
    }
}

// P1.12 2b ROOT-CAUSE REGRESSION GUARD — pins that the two GetRTA_OneTask
// overloads are NOT equivalent on wide-ET multi-HP input, which is WHY
// RTACache::Evaluate's NoReuse path must use the 3-arg form (the oracle's form)
// and not the 2-arg form. The 2-arg form (RTA.cpp:32) Compresses+Convolves the
// running RTA PER HP task; the 3-arg form (RTA.cpp:46) Compresses ONCE then
// Convolves against a pre-built rolling HP-ET prefix. CompressDistributionWithOnlySize
// is LOSSY once support > Granularity, so the differing compress count yields a
// different FiniteDist. This test asserts the two forms DIFFER on a wide-ET 2-HP
// input — it is the load-bearing reproduction of the 2b mechanism. If a future
// refactor makes them equal again (e.g. Compress becomes lossless), this guard
// flips and the NoReuse path's choice of overload no longer matters; until then,
// Evaluate MUST call the 3-arg form to stay bit-identical to the oracle.
TEST(GetRTA_OneTaskDifferential, TwoArgDivergesFromThreeArgOnWideEt) {
    GlobalVariables::Granularity = 10;
    // Three wide Gaussians, each truncated to [20,80] at granularity 5 -> each ET
    // has 13 support points (> Granularity=10), so CompressDistributionWithOnlySize
    // is genuinely lossy on every Convolve.
    FiniteDist et_low = FiniteDist(GaussianDist(50, 8), 20, 80, 5);
    FiniteDist et_mid = FiniteDist(GaussianDist(55, 8), 20, 80, 5);
    FiniteDist et_high = FiniteDist(GaussianDist(60, 8), 20, 80, 5);
    // task_high is the task under analysis; {task_low, task_mid} are its 2 HP
    // tasks (priority order: low < mid < high). Wide ETs convolve well past
    // Granularity across the 2 HP iterations.
    Task task_low(0, et_low, 200, 200, 0);
    Task task_mid(1, et_mid, 400, 400, 1);
    Task task_high(2, et_high, 800, 800, 2);
    TaskSet hp_tasks = {task_low, task_mid};

    // 2-arg form (the DIVERGENT form Evaluate's NoReuse path must NOT use).
    FiniteDist rta_two_arg = GetRTA_OneTask(task_high, hp_tasks);

    // 3-arg form (the oracle's form): build the rolling HP-ET convolution exactly
    // as ProbabilisticRTA_TaskSet_SingleCore does (RTA.cpp:87,110-112), then call.
    FiniteDist hp_tasks_et_conv({Value_Proba(0, 1.0)});
    hp_tasks_et_conv.CompressDistributionWithOnlySize(
        GlobalVariables::Granularity * 1);
    hp_tasks_et_conv.Convolve(task_low.execution_time_dist);
    hp_tasks_et_conv.CompressDistributionWithOnlySize(
        GlobalVariables::Granularity * 1);
    hp_tasks_et_conv.Convolve(task_mid.execution_time_dist);
    FiniteDist rta_three_arg =
        GetRTA_OneTask(task_high, hp_tasks, hp_tasks_et_conv);

    EXPECT_FALSE(rta_two_arg == rta_three_arg)
        << "2-arg and 3-arg GetRTA_OneTask must diverge on wide-ET 2-HP input — "
           "if they are equal, the 2b root-cause premise no longer holds and "
           "Evaluate's NoReuse overload choice is no longer load-bearing.";
}

// AdoptChampion: after Evaluate produces a candidate RTA, AdoptChampion promotes
// it to champion (rebuilds HP-prefixes by re-rolling ET-convolution). A
// subsequent identity Evaluate on the adopted triple → FullReuse, bit-identical
// to the oracle. Pins that the rebuilt prefixes are valid for future patches.
TEST_F(TaskSetForTest_4tasks_2cores_cache,
       AdoptChampion_RebuiltPrefixesValidForNextPatch) {
    RTACache cache;
    cache.Initialize(dag_tasks, priority_vec, time_limits);

    // Walk: candidate = TL change on task 1, adopt it, then patch again from
    // the new champion.
    std::vector<double> tl_cand = {-1, 3, -1, -1};
    const std::vector<FiniteDist>& rtas_eval =
        cache.Evaluate(dag_tasks, priority_vec, tl_cand);
    cache.AdoptChampion(dag_tasks, priority_vec, tl_cand, rtas_eval);

    // From the adopted champion, patch task 1's TL again (3 → 5): one-task
    // change vs the new champion.
    std::vector<double> tl_cand2 = {-1, 5, -1, -1};
    std::vector<FiniteDist> rtas_oracle =
        OracleRtas(dag_tasks, priority_vec, tl_cand2);
    const std::vector<FiniteDist>& rtas_eval2 =
        cache.Evaluate(dag_tasks, priority_vec, tl_cand2);

    ASSERT_EQ(rtas_oracle.size(), rtas_eval2.size());
    for (size_t i = 0; i < rtas_oracle.size(); i++) {
        EXPECT_TRUE(rtas_oracle[i] == rtas_eval2[i])
            << "rtas[" << i
            << "] diverged on post-AdoptChampion patch (rebuilt prefix)";
    }
}

// ComputeTaskSetDifference: |diff|==0 → changed_task_id==-1; |diff|==1 (TL) →
// locators filled (changed task + core, old_pos==new_pos for ET-only); >1 →
// THROWS (violates the single-change invariant). IsSingleTaskChange is the
// non-throwing predicate. No `klass` field — the verdict is derived from the
// locators.
TEST_F(TaskSetForTest_4tasks_2cores_cache,
       ComputeTaskSetDifference_ClassesThreeCases) {
    RTACache cache;
    cache.Initialize(dag_tasks, priority_vec, time_limits);
    TaskSetDifference ignored;  // out-param for the predicate form

    // |diff|==0: same triple → changed_task_id == -1.
    TaskSetDifference d0 =
        cache.ComputeTaskSetDifference(dag_tasks, priority_vec, time_limits);
    EXPECT_EQ(d0.changed_task_id, -1);
    EXPECT_TRUE(cache.IsSingleTaskChange(dag_tasks, priority_vec, time_limits, ignored));

    // |diff|==1: one TL change on task 1 (core 0).
    std::vector<double> tl_cand = {-1, 3, -1, -1};
    TaskSetDifference d1 =
        cache.ComputeTaskSetDifference(dag_tasks, priority_vec, tl_cand);
    EXPECT_EQ(d1.changed_task_id, 1);
    EXPECT_EQ(d1.core, 0);
    EXPECT_EQ(d1.old_pos, d1.new_pos);  // ET-only move: position unchanged
    EXPECT_TRUE(cache.IsSingleTaskChange(dag_tasks, priority_vec, tl_cand, ignored));

    // |diff|>1: two TL changes → IsSingleTaskChange false, and
    // ComputeTaskSetDifference throws (invariant violation).
    std::vector<double> tl_two = {-1, 3, 4, -1};
    EXPECT_FALSE(cache.IsSingleTaskChange(dag_tasks, priority_vec, tl_two, ignored));
    EXPECT_THROW(
        cache.ComputeTaskSetDifference(dag_tasks, priority_vec, tl_two),
        std::runtime_error);
}

// ClassifyReusePerTask: |diff|==0 → all FullReuse; |diff|==1 (v1 cross-core
// reuse) → every task on the SAME core as the change is recompute (NoReuse),
// every task on a DIFFERENT core is FullReuse.
TEST_F(TaskSetForTest_4tasks_2cores_cache,
       ClassifyReusePerTask_TLChange_SuffixOnChangedCore) {
    RTACache cache;
    cache.Initialize(dag_tasks, priority_vec, time_limits);

    // |diff|==0 → all FullReuse.
    std::vector<RTAReusePerTask> r0 =
        cache.ClassifyReusePerTask(dag_tasks, priority_vec, time_limits);
    ASSERT_EQ(r0.size(), tasks.size());
    for (size_t i = 0; i < r0.size(); i++) {
        EXPECT_EQ(r0[i], RTAReusePerTask::FullReuse);
    }

    // |diff|==1: task 1 TL change (core 0). v1: every task on core 0 {t0,t1}
    // is recompute (the changed task's HP set shifts on its whole core); every
    // task on the untouched core 1 {t2,t3} is FullReuse.
    std::vector<double> tl_cand = {-1, 3, -1, -1};
    std::vector<RTAReusePerTask> r1 =
        cache.ClassifyReusePerTask(dag_tasks, priority_vec, tl_cand);
    ASSERT_EQ(r1.size(), tasks.size());
    EXPECT_EQ(r1[0], RTAReusePerTask::NoReuse);   // t0: same core as change
    EXPECT_EQ(r1[1], RTAReusePerTask::NoReuse);   // t1: changed task
    EXPECT_EQ(r1[2], RTAReusePerTask::FullReuse);  // t2: core 1 untouched
    EXPECT_EQ(r1[3], RTAReusePerTask::FullReuse);  // t3: core 1 untouched
}

// No champion → ClassifyReusePerTask returns all-NoReuse; ComputeTaskSetDifference
// returns {-1,...} (no verdict); IsSingleTaskChange returns false.
TEST_F(TaskSetForTest_4tasks_2cores_cache, NoChampion_AllNoReuse) {
    RTACache cache;
    EXPECT_FALSE(cache.HasChampion());

    std::vector<RTAReusePerTask> r =
        cache.ClassifyReusePerTask(dag_tasks, priority_vec, time_limits);
    ASSERT_EQ(r.size(), tasks.size());
    for (size_t i = 0; i < r.size(); i++) {
        EXPECT_EQ(r[i], RTAReusePerTask::NoReuse);
    }
    TaskSetDifference d =
        cache.ComputeTaskSetDifference(dag_tasks, priority_vec, time_limits);
    EXPECT_EQ(d.changed_task_id, -1);
    TaskSetDifference ignored;  // out-param for the predicate form
    EXPECT_FALSE(cache.IsSingleTaskChange(dag_tasks, priority_vec, time_limits, ignored));
}

// ============================================================================
// Direct unit tests for the P1.9 priority-analysis utilities
// (PrioritySwitchAnalysis.h). These are pure functions on hand-built
// vector<int> / vector<vector<int>> inputs — no DAG/Task setup — pinning the
// two-pointer-walk + remove-and-compare edge cases that the DAG-level RTACache
// tests above exercise only indirectly. The functions live in namespace
// SP_OPT_PA (in effect via `using namespace SP_OPT_PA;` above).
// ============================================================================

// RestEqualAfterRemoving: the two-pointer "remove one task from both vectors,
// compare the rest" test. Caller guarantees same-size orders + `task_id`
// appears exactly once in each; the cases below honor that (the size-mismatch
// case documents the violated-precondition return, not a legal call).
TEST(RestEqualAfterRemovingTest, IdenticalOrders_RemovingAnyTask_True) {
    std::vector<int> order = {0, 1, 2, 3};
    EXPECT_TRUE(RestEqualAfterRemoving(order, order, 0));
    EXPECT_TRUE(RestEqualAfterRemoving(order, order, 2));
    EXPECT_TRUE(RestEqualAfterRemoving(order, order, 3));
}

TEST(RestEqualAfterRemovingTest, OneTaskMovedToFront_True) {
    // candidate moved task 3 to the front; champion keeps id order.
    std::vector<int> cand = {3, 0, 1, 2};
    std::vector<int> champ = {0, 1, 2, 3};
    EXPECT_TRUE(RestEqualAfterRemoving(cand, champ, 3));
}

TEST(RestEqualAfterRemovingTest, OneTaskMovedToEnd_True) {
    // candidate moved task 0 to the end; exercises trailing-`task_id` drain on
    // the candidate side.
    std::vector<int> cand = {1, 2, 3, 0};
    std::vector<int> champ = {0, 1, 2, 3};
    EXPECT_TRUE(RestEqualAfterRemoving(cand, champ, 0));
}

TEST(RestEqualAfterRemovingTest, OneTaskMovedToMiddle_True) {
    std::vector<int> cand = {0, 2, 1, 3};
    std::vector<int> champ = {0, 1, 2, 3};
    EXPECT_TRUE(RestEqualAfterRemoving(cand, champ, 2));
}

TEST(RestEqualAfterRemovingTest, TaskIdAtFrontOfBoth_True) {
    // `task_id` is the first entry in both; the rest matches → true (drain on
    // both sides at the very first step).
    std::vector<int> cand = {7, 0, 1, 2};
    std::vector<int> champ = {7, 0, 1, 2};
    EXPECT_TRUE(RestEqualAfterRemoving(cand, champ, 7));
}

TEST(RestEqualAfterRemovingTest, SecondTaskAlsoMoved_False) {
    // Removing task 2 leaves candidate {0,1,3} vs champion {0,3,1} → differ.
    std::vector<int> cand = {0, 2, 1, 3};
    std::vector<int> champ = {0, 3, 2, 1};
    EXPECT_FALSE(RestEqualAfterRemoving(cand, champ, 2));
}

TEST(RestEqualAfterRemovingTest, TwoTaskSwap_False) {
    // A swap is 2 moves; removing one still leaves the other moved.
    std::vector<int> cand = {1, 0, 3, 2};
    std::vector<int> champ = {0, 1, 2, 3};
    EXPECT_FALSE(RestEqualAfterRemoving(cand, champ, 0));
    EXPECT_FALSE(RestEqualAfterRemoving(cand, champ, 1));
}

TEST(RestEqualAfterRemovingTest, SizeMismatchWithExtraNonTaskId_False) {
    // Precondition violated (different sizes = core migration; the caller
    // rejects this before calling). The walk must not misreport equality when
    // the length mismatch surfaces: removing `task_id` (5, absent from both)
    // leaves the 2-element candidate {0,1} vs the 3-element champion {0,1,2}
    // — the trailing drain cannot absorb a non-`task_id` entry, so the final
    // index check fails. (If the extra element WERE `task_id`, the rest would
    // genuinely match — the size guard is the caller's job, not this fn's.)
    std::vector<int> cand = {0, 1};
    std::vector<int> champ = {0, 1, 2};
    EXPECT_FALSE(RestEqualAfterRemoving(cand, champ, 5));
}

TEST(RestEqualAfterRemovingTest, SizeMismatchButExtraIsTaskId_True) {
    // Same-size precondition violated, BUT the only extra entry IS `task_id`:
    // removing it from the longer champion leaves {0,1} == candidate {0,1}, so
    // the rest genuinely matches → true. Documents that the size guard lives
    // in the caller (AnalyzePrioritySwitch's size check), NOT here — this fn
    // answers only "do the un-skipped sequences match?"
    std::vector<int> cand = {0, 1};
    std::vector<int> champ = {0, 1, 2};
    EXPECT_TRUE(RestEqualAfterRemoving(cand, champ, 2));
}

TEST(RestEqualAfterRemovingTest, EmptyOrders_True) {
    std::vector<int> empty;
    EXPECT_TRUE(RestEqualAfterRemoving(empty, empty, 0));
}

TEST(RestEqualAfterRemovingTest, SingleElement_RemovingIt_True) {
    std::vector<int> single = {5};
    EXPECT_TRUE(RestEqualAfterRemoving(single, single, 5));
}

// AnalyzePrioritySwitchPerCore: per-core single-move detection + locator fill.
// Returns AllIdentical / SingleChange (with moved_task_id/old_pos/new_pos) /
// NotSingle. Caller guarantees same-size vectors (a size mismatch is a core
// migration rejected before this fn — not constructed here, as the first-
// mismatch scan would read out of bounds).
TEST(AnalyzePrioritySwitchPerCoreTest, Identical_AllIdentical) {
    std::vector<int> order = {0, 1, 2, 3};
    PrioritySwitchAnalysis out;
    EXPECT_EQ(AnalyzePrioritySwitchPerCore(order, order, out),
              PrioritySwitchStatus::AllIdentical);
    // Locators untouched on AllIdentical.
    EXPECT_EQ(out.moved_task_id, -1);
}

TEST(AnalyzePrioritySwitchPerCoreTest, MoveToFront_SingleChange) {
    std::vector<int> cand = {3, 0, 1, 2};
    std::vector<int> champ = {0, 1, 2, 3};
    PrioritySwitchAnalysis out;
    EXPECT_EQ(AnalyzePrioritySwitchPerCore(cand, champ, out),
              PrioritySwitchStatus::SingleChange);
    EXPECT_EQ(out.moved_task_id, 3);
    EXPECT_EQ(out.old_pos, 3);  // champion's position of task 3
    EXPECT_EQ(out.new_pos, 0);  // candidate's position of task 3
}

TEST(AnalyzePrioritySwitchPerCoreTest, MoveToEnd_SingleChange) {
    std::vector<int> cand = {1, 2, 3, 0};
    std::vector<int> champ = {0, 1, 2, 3};
    PrioritySwitchAnalysis out;
    EXPECT_EQ(AnalyzePrioritySwitchPerCore(cand, champ, out),
              PrioritySwitchStatus::SingleChange);
    EXPECT_EQ(out.moved_task_id, 0);
    EXPECT_EQ(out.old_pos, 0);
    EXPECT_EQ(out.new_pos, 3);
}

TEST(AnalyzePrioritySwitchPerCoreTest, MoveToMiddle_SingleChange) {
    std::vector<int> cand = {0, 2, 1, 3};
    std::vector<int> champ = {0, 1, 2, 3};
    PrioritySwitchAnalysis out;
    EXPECT_EQ(AnalyzePrioritySwitchPerCore(cand, champ, out),
              PrioritySwitchStatus::SingleChange);
    EXPECT_EQ(out.moved_task_id, 2);
    EXPECT_EQ(out.old_pos, 2);
    EXPECT_EQ(out.new_pos, 1);
}

TEST(AnalyzePrioritySwitchPerCoreTest, TwoTaskSwap_NotSingle) {
    std::vector<int> cand = {1, 0, 3, 2};
    std::vector<int> champ = {0, 1, 2, 3};
    PrioritySwitchAnalysis out;
    EXPECT_EQ(AnalyzePrioritySwitchPerCore(cand, champ, out),
              PrioritySwitchStatus::NotSingle);
}

TEST(AnalyzePrioritySwitchPerCoreTest, TwoIndependentMoves_NotSingle) {
    // Tasks 1 and 3 both shifted; removing either leaves the other moved.
    std::vector<int> cand = {0, 3, 2, 1};
    std::vector<int> champ = {0, 1, 2, 3};
    PrioritySwitchAnalysis out;
    EXPECT_EQ(AnalyzePrioritySwitchPerCore(cand, champ, out),
              PrioritySwitchStatus::NotSingle);
}

TEST(AnalyzePrioritySwitchPerCoreTest, IdenticalSingleElement_AllIdentical) {
    std::vector<int> single = {5};
    PrioritySwitchAnalysis out;
    EXPECT_EQ(AnalyzePrioritySwitchPerCore(single, single, out),
              PrioritySwitchStatus::AllIdentical);
}

// AnalyzePrioritySwitch: whole-map size check + find-the-one-changed-core +
// delegate. Builds vector<vector<int>> inline (P1.20: flat per-core vector
// indexed by core; an absent core and an empty core both mean "zero tasks").
TEST(AnalyzePrioritySwitchTest, AllCoresIdentical_AllIdentical) {
    std::vector<std::vector<int>> per_core = {{0, 1}, {2, 3}};
    EXPECT_EQ(AnalyzePrioritySwitch(per_core, per_core).status,
              PrioritySwitchStatus::AllIdentical);
}

TEST(AnalyzePrioritySwitchTest, OneCoreSingleMove_SingleChange) {
    // One genuine single move on core 0: task 1 relocated to the front.
    std::vector<std::vector<int>> cand = {{1, 0, 2}, {3, 4}};
    std::vector<std::vector<int>> champ = {{0, 1, 2}, {3, 4}};
    PrioritySwitchAnalysis r = AnalyzePrioritySwitch(cand, champ);
    EXPECT_EQ(r.status, PrioritySwitchStatus::SingleChange);
    EXPECT_EQ(r.changed_core, 0);
    EXPECT_EQ(r.moved_task_id, 1);
}

TEST(AnalyzePrioritySwitchTest, OneCoreTwoMove_NotSingle) {
    std::vector<std::vector<int>> cand = {{1, 0, 3, 2}, {4, 5}};  // core 0: two swaps
    std::vector<std::vector<int>> champ = {{0, 1, 2, 3}, {4, 5}};
    EXPECT_EQ(AnalyzePrioritySwitch(cand, champ).status,
              PrioritySwitchStatus::NotSingle);
}

TEST(AnalyzePrioritySwitchTest, TwoCoresEachSingleMove_NotSingle) {
    // Each core has one genuine single move (task 2 → front on core 0; task 4
    // → front on core 1); two changed cores ⇒ NotSingle via the 2nd-core check.
    std::vector<std::vector<int>> cand = {{2, 0, 1}, {4, 3}};
    std::vector<std::vector<int>> champ = {{0, 1, 2}, {3, 4}};
    EXPECT_EQ(AnalyzePrioritySwitch(cand, champ).status,
              PrioritySwitchStatus::NotSingle);
}

TEST(AnalyzePrioritySwitchTest, SizeMismatchOnOneCore_NotSingle) {
    // Core 0 grew by one (core migration) ⇒ NotSingle via the size check.
    std::vector<std::vector<int>> cand = {{0, 1, 2}, {3, 4}};
    std::vector<std::vector<int>> champ = {{0, 1}, {3, 4}};
    EXPECT_EQ(AnalyzePrioritySwitch(cand, champ).status,
              PrioritySwitchStatus::NotSingle);
}

TEST(AnalyzePrioritySwitchTest, ChampionCoreEmptied_NotSingle) {
    // Champion has core 2 (non-empty) that the candidate lacks ⇒ migration.
    // cand covers cores {0,1} (core 0 non-empty, core 1 empty); champ covers
    // {0,1,2} (core 0 non-empty, core 1 empty, core 2 non-empty) ⇒ size
    // mismatch on core 2.
    std::vector<std::vector<int>> cand = {{0, 1}, {}};
    std::vector<std::vector<int>> champ = {{0, 1}, {}, {2, 3}};
    EXPECT_EQ(AnalyzePrioritySwitch(cand, champ).status,
              PrioritySwitchStatus::NotSingle);
}

class TaskSetv9 : public ::testing::Test {
   public:
    void SetUp() override {
        string file_path =
            GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v9.yaml";
        dag_tasks = ReadDAG_Tasks(file_path);
    }

    // data members
    DAG_Model dag_tasks;
};
TEST_F(TaskSetv9, RTA_w_processor_assignment) {
    GlobalVariables::Granularity = 10;
    std::vector<FiniteDist> rtas = ProbabilisticRTA_TaskSet(dag_tasks.tasks);
    // RT distribution from 200 to 200
    EXPECT_NEAR(0.0, GetDDL_MissProbability(rtas[0], 200), 1e-6);
    EXPECT_NEAR(1.0, GetDDL_MissProbability(rtas[0], 199), 1e-6);

    // RT distribution from 200 to 600
    EXPECT_NEAR(0.59, GetDDL_MissProbability(rtas[1], 400), 1e-2);
}
int main(int argc, char **argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
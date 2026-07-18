// #include <gtest/gtest.h>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Safety_Performance_Metric/Probability.h"
#include "sources/Safety_Performance_Metric/RTA.h"
#include "sources/Safety_Performance_Metric/RTA_Cache.h"  // PerCoreRTACache, ComputeRTA_FullAndCache, ClassifyReuse, RTAReuseClass (P1.9)
#include "sources/Safety_Performance_Metric/PrioritySwitchAnalysis.h"  // RestEqualAfterRemoving, AnalyzePrioritySwitch(PerCore), PrioritySwitchStatus/Analysis (P1.9 priority-analysis utilities)
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
// Task(id, exec, period, deadline, priority) — processorId is a default-(-1)
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
        // processorId is a default-(-1) member set post-construction (see the
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

    // |diff|==0: same triple → changed_task_id == -1.
    TaskSetDifference d0 =
        cache.ComputeTaskSetDifference(dag_tasks, priority_vec, time_limits);
    EXPECT_EQ(d0.changed_task_id, -1);
    EXPECT_TRUE(cache.IsSingleTaskChange(dag_tasks, priority_vec, time_limits));

    // |diff|==1: one TL change on task 1 (core 0).
    std::vector<double> tl_cand = {-1, 3, -1, -1};
    TaskSetDifference d1 =
        cache.ComputeTaskSetDifference(dag_tasks, priority_vec, tl_cand);
    EXPECT_EQ(d1.changed_task_id, 1);
    EXPECT_EQ(d1.core, 0);
    EXPECT_EQ(d1.old_pos, d1.new_pos);  // ET-only move: position unchanged
    EXPECT_TRUE(cache.IsSingleTaskChange(dag_tasks, priority_vec, tl_cand));

    // |diff|>1: two TL changes → IsSingleTaskChange false, and
    // ComputeTaskSetDifference throws (invariant violation).
    std::vector<double> tl_two = {-1, 3, 4, -1};
    EXPECT_FALSE(cache.IsSingleTaskChange(dag_tasks, priority_vec, tl_two));
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
    EXPECT_FALSE(cache.IsSingleTaskChange(dag_tasks, priority_vec, time_limits));
}

// ============================================================================
// Direct unit tests for the P1.9 priority-analysis utilities
// (PrioritySwitchAnalysis.h). These are pure functions on hand-built
// vector<int> / unordered_map<int, vector<int>> inputs — no DAG/Task setup —
// pinning the two-pointer-walk + remove-and-compare edge cases that the
// DAG-level RTACache tests above exercise only indirectly. The functions live
// in namespace SP_OPT_PA (in effect via `using namespace SP_OPT_PA;` above).
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
// delegate. Builds unordered_map<int, vector<int>> inline.
TEST(AnalyzePrioritySwitchTest, AllCoresIdentical_AllIdentical) {
    std::unordered_map<int, std::vector<int>> per_core = {
        {0, {0, 1}}, {1, {2, 3}}};
    EXPECT_EQ(AnalyzePrioritySwitch(per_core, per_core).status,
              PrioritySwitchStatus::AllIdentical);
}

TEST(AnalyzePrioritySwitchTest, OneCoreSingleMove_SingleChange) {
    // One genuine single move on core 0: task 1 relocated to the front.
    std::unordered_map<int, std::vector<int>> cand = {
        {0, {1, 0, 2}}, {1, {3, 4}}};
    std::unordered_map<int, std::vector<int>> champ = {
        {0, {0, 1, 2}}, {1, {3, 4}}};
    PrioritySwitchAnalysis r = AnalyzePrioritySwitch(cand, champ);
    EXPECT_EQ(r.status, PrioritySwitchStatus::SingleChange);
    EXPECT_EQ(r.changed_core, 0);
    EXPECT_EQ(r.moved_task_id, 1);
}

TEST(AnalyzePrioritySwitchTest, OneCoreTwoMove_NotSingle) {
    std::unordered_map<int, std::vector<int>> cand = {
        {0, {1, 0, 3, 2}}, {1, {4, 5}}};  // core 0: two swaps
    std::unordered_map<int, std::vector<int>> champ = {
        {0, {0, 1, 2, 3}}, {1, {4, 5}}};
    EXPECT_EQ(AnalyzePrioritySwitch(cand, champ).status,
              PrioritySwitchStatus::NotSingle);
}

TEST(AnalyzePrioritySwitchTest, TwoCoresEachSingleMove_NotSingle) {
    // Each core has one genuine single move (task 2 → front on core 0; task 4
    // → front on core 1); two changed cores ⇒ NotSingle via the 2nd-core check.
    std::unordered_map<int, std::vector<int>> cand = {
        {0, {2, 0, 1}}, {1, {4, 3}}};
    std::unordered_map<int, std::vector<int>> champ = {
        {0, {0, 1, 2}}, {1, {3, 4}}};
    EXPECT_EQ(AnalyzePrioritySwitch(cand, champ).status,
              PrioritySwitchStatus::NotSingle);
}

TEST(AnalyzePrioritySwitchTest, SizeMismatchOnOneCore_NotSingle) {
    // Core 0 grew by one (core migration) ⇒ NotSingle via the size check.
    std::unordered_map<int, std::vector<int>> cand = {
        {0, {0, 1, 2}}, {1, {3, 4}}};
    std::unordered_map<int, std::vector<int>> champ = {
        {0, {0, 1}}, {1, {3, 4}}};
    EXPECT_EQ(AnalyzePrioritySwitch(cand, champ).status,
              PrioritySwitchStatus::NotSingle);
}

TEST(AnalyzePrioritySwitchTest, ChampionCoreEmptied_NotSingle) {
    // Champion has core 2 (non-empty) that the candidate lacks ⇒ migration.
    std::unordered_map<int, std::vector<int>> cand = {{0, {0, 1}}};
    std::unordered_map<int, std::vector<int>> champ = {
        {0, {0, 1}}, {2, {2, 3}}};
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
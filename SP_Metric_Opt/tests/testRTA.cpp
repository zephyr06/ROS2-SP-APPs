// #include <gtest/gtest.h>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Safety_Performance_Metric/Probability.h"
#include "sources/Safety_Performance_Metric/RTA.h"
#include "sources/Safety_Performance_Metric/RTA_Cache.h"  // PerCoreRTACache, ComputeRTA_FullAndCache, ClassifyReuse, RTAReuseClass (P1.9)
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
        // priority_vec[i] = task id at priority index i (small index = high
        // priority). Here the ctor priorities already match id order, so the
        // HP-first per-core order is core0 {t0,t1}, core1 {t2,t3}.
        priority_vec = {0, 1, 2, 3};
        // -1 = no TL (keep the base Gaussian dist). Patches mutate individual
        // entries; full-passthrough TLs pin the differential vs the no-TL
        // oracle, and the TL-specific test toggles one entry.
        time_limits = {-1, -1, -1, -1};
        dag_tasks = DAG_Model(tasks, {}, {});
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

// ComputeRTA_FullAndCache must return bit-identical rtas to the oracle (the old
// path: apply TLs → apply pa_vec → ProbabilisticRTA_TaskSet) on the same input
// tuple. Decision-4 acceptance gate: the cache replaces the old path outright,
// so the old path is the oracle here (kept long enough to verify, then removed
// at step 3b).
TEST_F(TaskSetForTest_4tasks_2cores_cache,
       ComputeRTA_FullAndCache_SameRtasAs_Oracle) {
    std::vector<FiniteDist> rtas_oracle =
        OracleRtas(dag_tasks, priority_vec, time_limits);

    std::unordered_map<int, PerCoreRTACache> cache;
    std::vector<FiniteDist> rtas_cached =
        ComputeRTA_FullAndCache(dag_tasks, priority_vec, time_limits, cache);

    ASSERT_EQ(rtas_oracle.size(), rtas_cached.size());
    for (size_t i = 0; i < rtas_oracle.size(); i++) {
        EXPECT_TRUE(rtas_oracle[i] == rtas_cached[i])
            << "rtas[" << i << "] diverged between oracle and cached path";
    }
}

// The cache must be self-consistent: each core's entry has matching lengths
// (sorted_task_ids / rta / tl_vec / hp_tasks_et_conv_vec), its rta[i] equals the
// flat rtas[task's index] for the task at sorted position i, tl_vec[i] aligns
// with the time limit of sorted_task_ids[i], and every task lands in exactly one
// core. Also confirms both cores are present (2 tasks each).
TEST_F(TaskSetForTest_4tasks_2cores_cache,
       ComputeRTA_FullAndCache_CacheIsSelfConsistent) {
    std::unordered_map<int, PerCoreRTACache> cache;
    std::vector<FiniteDist> rtas =
        ComputeRTA_FullAndCache(dag_tasks, priority_vec, time_limits, cache);

    ASSERT_EQ(cache.size(), 2u);
    std::set<int> seen_task_ids;
    for (const auto& kv : cache) {
        const PerCoreRTACache& cache_entry = kv.second;
        ASSERT_EQ(cache_entry.SortedTaskIds().size(), cache_entry.Rta().size());
        ASSERT_EQ(cache_entry.SortedTaskIds().size(),
                  cache_entry.TlVec().size());
        ASSERT_EQ(cache_entry.SortedTaskIds().size(),
                  cache_entry.HpTasksEtConvVec().size());
        EXPECT_TRUE(cache_entry.HpTasksEtConvVec()[0] ==
                    FiniteDist({Value_Proba(0, 1.0)}))
            << "hp_tasks_et_conv_vec[0] must be the empty-HP-set identity";
        for (size_t i = 0; i < cache_entry.SortedTaskIds().size(); i++) {
            int task_id = cache_entry.SortedTaskIds()[i];
            seen_task_ids.insert(task_id);
            // tl_vec[i] must be the time limit of the task at sorted position i
            // (the ET-dist validity proxy the cache relies on).
            EXPECT_DOUBLE_EQ(time_limits[task_id], cache_entry.TlVec()[i])
                << "tl_vec[" << i << "] on core " << kv.first
                << " != time_limits of task " << task_id;
            // sorted HP-first (priority ascending), as SingleCore sorts.
            if (i > 0) {
                int prev = tasks[cache_entry.SortedTaskIds()[i - 1]].priority;
                int curr = tasks[cache_entry.SortedTaskIds()[i]].priority;
                EXPECT_LT(prev, curr) << "cache not sorted HP-first on core "
                                      << kv.first;
            }
            // cache_entry.Rta()[i] must equal the flat rtas for that task.
            int flat_index = -1;
            for (size_t j = 0; j < tasks.size(); j++) {
                if (tasks[j].id == task_id) {
                    flat_index = static_cast<int>(j);
                    break;
                }
            }
            ASSERT_NE(flat_index, -1);
            EXPECT_TRUE(cache_entry.Rta()[i] == rtas[flat_index])
                << "cache rta[" << i << "] on core " << kv.first
                << " != flat rtas[" << flat_index << "]";
        }
    }
    // every task landed in exactly one core.
    EXPECT_EQ(seen_task_ids.size(), tasks.size());
}

// TL coverage deferred from step 3a: with a real TL applied (a point-mass ET
// dist via GetUnitExecutionTimeDist), the cache fn must STILL match the oracle
// bit-for-bit. This pins that the cache's internal ApplyTimeLimitsToTasks
// ExecutionTime + per-core tl_vec storage reproduce TL-driven RTA exactly.
TEST_F(TaskSetForTest_4tasks_2cores_cache,
       ComputeRTA_FullAndCache_WithTL_SameRtasAs_Oracle) {
    // Give task 1 (core 0, lower priority on its core) a real TL of 3 — its ET
    // dist becomes a point mass at 3, changing its own RTA and task 0's... no:
    // task 1 is LOWER priority, so only task 1's own RTA (the suffix) changes.
    std::vector<double> tl = {-1, 3, -1, -1};

    std::vector<FiniteDist> rtas_oracle = OracleRtas(dag_tasks, priority_vec, tl);

    std::unordered_map<int, PerCoreRTACache> cache;
    std::vector<FiniteDist> rtas_cached =
        ComputeRTA_FullAndCache(dag_tasks, priority_vec, tl, cache);

    ASSERT_EQ(rtas_oracle.size(), rtas_cached.size());
    for (size_t i = 0; i < rtas_oracle.size(); i++) {
        EXPECT_TRUE(rtas_oracle[i] == rtas_cached[i])
            << "rtas[" << i << "] diverged under TL between oracle and cache";
    }
    // The stored tl_vec must reflect the applied TL on the affected core.
    ASSERT_EQ(cache.count(0), 1u);
    const PerCoreRTACache& core0 = cache.at(0);
    // core0 sorted HP-first = {t0, t1}; t0 has no TL (-1), t1 has TL 3.
    ASSERT_EQ(core0.SortedTaskIds().size(), 2u);
    ASSERT_EQ(core0.SortedTaskIds()[0], 0);
    ASSERT_EQ(core0.SortedTaskIds()[1], 1);
    EXPECT_DOUBLE_EQ(core0.TlVec()[0], -1.0);
    EXPECT_DOUBLE_EQ(core0.TlVec()[1], 3.0);
}

// ClassifyReuse v0 against the SAME state the cache was built with, with NO
// changed tasks: every task is RtaReuse (the "candidate == champion" case).
// Also exercises the class query helpers (PositionOfTask / ContainsTask) as a
// sanity check on the populated cache geometry.
TEST_F(TaskSetForTest_4tasks_2cores_cache,
       ClassifyReuse_IdentityCandidate_AllRtaReuse) {
    std::unordered_map<int, PerCoreRTACache> cache;
    ComputeRTA_FullAndCache(dag_tasks, priority_vec, time_limits, cache);

    std::vector<RTAReuseClass> reuse =
        ClassifyReuse(cache, dag_tasks, /*changed_task_ids=*/{});

    ASSERT_EQ(reuse.size(), tasks.size());
    for (size_t i = 0; i < reuse.size(); i++) {
        EXPECT_EQ(reuse[i], RTAReuseClass::RtaReuse)
            << "task " << i << " must be RtaReuse vs an unchanged champion";
    }

    // Sanity: core 0 owns {t0,t1}, core 1 owns {t2,t3}; t0 is at sorted
    // position 0 on core 0; t3 is NOT on core 0.
    const PerCoreRTACache& core0 = cache.at(0);
    EXPECT_EQ(core0.PositionOfTask(0), 0);
    EXPECT_EQ(core0.PositionOfTask(1), 1);
    EXPECT_FALSE(core0.ContainsTask(3));
    EXPECT_EQ(core0.Size(), 2);
}

// A TL change to one task on core 0 (task 1): v0 is conservative — BOTH tasks
// on the changed core (t0 AND t1) are Recompute (the unchanged prefix above the
// change is not yet exploited), while both tasks on the untouched core 1 are
// RtaReuse. This is the cross-core skip payoff; the within-core prefix reuse is
// a later refinement (RecomputeWithHpPrefix, not produced by v0).
TEST_F(TaskSetForTest_4tasks_2cores_cache,
       ClassifyReuse_TLChangeOnCore0_ChangedCoreRecomputes) {
    std::unordered_map<int, PerCoreRTACache> cache;
    ComputeRTA_FullAndCache(dag_tasks, priority_vec, time_limits, cache);

    // Task 1 sits on core 0; changing its TL marks core 0 changed.
    std::vector<RTAReuseClass> reuse =
        ClassifyReuse(cache, dag_tasks, /*changed_task_ids=*/{1});

    ASSERT_EQ(reuse.size(), tasks.size());
    // Core 0 = {t0, t1} → both Recompute (v0 conservative: whole core).
    EXPECT_EQ(reuse[0], RTAReuseClass::Recompute);
    EXPECT_EQ(reuse[1], RTAReuseClass::Recompute);
    // Core 1 = {t2, t3} → untouched → RtaReuse.
    EXPECT_EQ(reuse[2], RTAReuseClass::RtaReuse);
    EXPECT_EQ(reuse[3], RTAReuseClass::RtaReuse);
}

// A priority move on core 0 (changing task 0's priority position): same verdict
// shape as the TL change under v0 — both tasks on the changed core Recompute,
// both on the untouched core RtaReuse. Confirms v0 is uniform across change
// types (it keys on processorId only, not on change kind).
TEST_F(TaskSetForTest_4tasks_2cores_cache,
       ClassifyReuse_PriorityMoveOnCore0_ChangedCoreRecomputes) {
    std::unordered_map<int, PerCoreRTACache> cache;
    ComputeRTA_FullAndCache(dag_tasks, priority_vec, time_limits, cache);

    // Task 0's priority moves → core 0 changed.
    std::vector<RTAReuseClass> reuse =
        ClassifyReuse(cache, dag_tasks, /*changed_task_ids=*/{0});

    ASSERT_EQ(reuse.size(), tasks.size());
    EXPECT_EQ(reuse[0], RTAReuseClass::Recompute);
    EXPECT_EQ(reuse[1], RTAReuseClass::Recompute);
    EXPECT_EQ(reuse[2], RTAReuseClass::RtaReuse);
    EXPECT_EQ(reuse[3], RTAReuseClass::RtaReuse);
}

// ClassifyReuse on an empty cache: every task is Recompute (no champion to
// reuse). This replaces the old CacheConsistentWith_EmptyCache_IsFalse test —
// "empty cache" is captured by "no task is reusable" in the per-task vector.
TEST_F(TaskSetForTest_4tasks_2cores_cache,
       ClassifyReuse_EmptyCache_AllRecompute) {
    std::unordered_map<int, PerCoreRTACache> empty_cache;
    std::vector<RTAReuseClass> reuse =
        ClassifyReuse(empty_cache, dag_tasks, /*changed_task_ids=*/{});

    ASSERT_EQ(reuse.size(), tasks.size());
    for (size_t i = 0; i < reuse.size(); i++) {
        EXPECT_EQ(reuse[i], RTAReuseClass::Recompute)
            << "task " << i << " must Recompute with an empty cache";
    }
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
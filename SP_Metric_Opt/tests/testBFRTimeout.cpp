// #include <gtest/gtest.h>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Optimization/OptimizeSP_Base.h"
#include "sources/Optimization/OptimizeSP_TL_BF.h"
#include "sources/Safety_Performance_Metric/Probability.h"  // P1.14: GaussianDist, FiniteDist
#include "sources/Utils/Parameters.h"

using ::testing::AtLeast;  // #1
using ::testing::Return;
using namespace std;
using namespace SP_OPT_PA;
using namespace GlobalVariables;

// Helper to build a synthetic task with time-performance pairs.
static Task MakeTaskWithPerf(int id, double period, const std::string& name,
                             const std::vector<TimePerfPair>& pairs) {
    std::vector<Value_Proba> dist = {Value_Proba(1, 1.0)};
    Task t(id, FiniteDist(dist), period, period, 0, name);
    t.timePerformancePairs = pairs;
    return t;
}

// Create a DAG where many tasks have multiple time-limit options so the
// outer BF enumeration explodes combinatorially.
class TaskSetBFWithTimeout : public ::testing::Test {
   public:
    void SetUp() override {
        saved_time_limit_ = GlobalVariables::TIME_LIMIT;
    }

    void TearDown() override {
        GlobalVariables::TIME_LIMIT = saved_time_limit_;
    }

    DAG_Model BuildExplosiveDAG(size_t num_tasks, size_t num_options,
                                double period) {
        TaskSet tasks;
        for (size_t i = 0; i < num_tasks; ++i) {
            std::vector<TimePerfPair> pairs;
            for (size_t j = 0; j < num_options; ++j) {
                pairs.push_back(TimePerfPair(100.0 * (j + 1), j * 0.1));
            }
            tasks.push_back(MakeTaskWithPerf(
                static_cast<int>(i), period, "T" + std::to_string(i), pairs));
        }
        MAP_Prev empty_prev;
        DAG_Model dag(tasks, empty_prev, 0, 0);
        return dag;
    }

    int saved_time_limit_;
};

// With TIME_LIMIT set to 0 the fixed code bails out on the very first
// outer-loop entry (a few microseconds).  The buggy code still walks all
// K^N combinations because each inner OptimizePA_BF resets the timer;
// with 8 tasks / 3 options (6561 combos) the buggy run takes ~1 s on a
// modern CPU.  A 100 ms wall-clock ceiling is therefore safe and gives a
// >10x margin.
//
// NOTE (P1.14): this case passes on the BUGGY code too — TIME_LIMIT=0 makes
// the outer ifTimeout at OptimizeSP_TL_BF.cpp:38 trip instantly (0 >= 0), so
// the search never reaches a leaf and neither defect is exercised.  It guards
// only the trivial "zero budget aborts immediately" property.  The
// per-leaf-reset defect is exercised by RespectsGlobalTimeLimit_Aggregate
// below.
TEST_F(TaskSetBFWithTimeout, RespectsGlobalTimeLimit) {
    DAG_Model dag = BuildExplosiveDAG(8, 3, 1000.0);
    SP_Parameters sp(dag);

    GlobalVariables::TIME_LIMIT = 0;

    auto start = std::chrono::high_resolution_clock::now();
    ResourceOptResult res = EnumeratePA_with_TimeLimits(dag, sp);
    auto end = std::chrono::high_resolution_clock::now();

    double elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count();

    EXPECT_LT(elapsed_ms, 100.0)
        << "EnumeratePA_with_TimeLimits did not respect TIME_LIMIT ("
        << GlobalVariables::TIME_LIMIT << " s). Elapsed: " << elapsed_ms
        << " ms.";
}

// P1.14 — the real defect: a single EvaluateSPWithPriorityVec call whose RTA
// convolutions exceed TIME_LIMIT strands the BF search past the cap, because
// ifTimeout is checked only BETWEEN permutations (OptimizeSP_BF.cpp:8) and at
// outer-recursion entry (OptimizeSP_TL_BF.cpp:38) — never inside ObtainSP_DAG
// / the per-task RTA loop (RTA.cpp:87).  This taskset is built so that ONE
// SP-eval takes ~8 s (7 wide-Gaussian tasks at granularity 300 -> the HP-ET
// convolution support grows combinatorially along the priority chain), with a
// SINGLE TL-leaf (every task has only the default -1 option, so the outer
// enumeration has exactly one leaf and exactly one N! inner enumeration).
// With TIME_LIMIT=1 the buggy code cannot interrupt the in-flight eval and
// runs the full ~8 s; the fixed code installs a BFDLSharedBudget at
// EnumeratePA_with_TimeLimits entry and polls BFSharedBudgetCancelled()
// between RTA sub-computations, so the runaway eval is interrupted in place
// and the search stays bounded at ~1-2 s.
//
// This is the taskset_0 = 182 s/interval signature from the P25 A/B (there,
// the live sim's ET distributions make individual SP-evals take many seconds
// on many intervals; the cap was never honored because no check fired inside
// the eval).  A 4 s ceiling gives the fixed code a comfortable margin (1 s
// budget + one RTA-sub-computation overshoot + jitter) while still failing
// the buggy code (which runs ~8 s here).
TEST_F(TaskSetBFWithTimeout, RespectsGlobalTimeLimit_SingleEvalExceedsCap) {
    // 7 wide-Gaussian tasks; granularity 300 makes one SP-eval ~8 s.
    const int N = 7;
    const int granularity = 300;
    TaskSet tasks;
    for (int i = 0; i < N; ++i) {
        double period = 100.0 * (i + 1);
        GaussianDist g(50.0 + 5 * i, 15.0);
        FiniteDist dist(g, g.mu - 3 * g.sigma, g.mu + 3 * g.sigma, granularity);
        // No timePerformancePairs -> only the default -1 TL option -> the
        // outer TL enumeration has exactly ONE leaf, so the ONLY way the cap
        // can be honored is cooperative cancel INSIDE the one SP-eval.
        tasks.push_back(
            Task(i, dist, period, period * 0.9, 0, "T" + std::to_string(i)));
    }
    MAP_Prev empty_prev;
    DAG_Model dag(tasks, empty_prev, 0, 0);
    SP_Parameters sp(dag);

    GlobalVariables::TIME_LIMIT = 1;

    auto start = std::chrono::high_resolution_clock::now();
    ResourceOptResult res = EnumeratePA_with_TimeLimits(dag, sp);
    auto end = std::chrono::high_resolution_clock::now();

    double elapsed_s =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count() /
        1000.0;

    // Budget is 1 s; allow up to 4 s for one RTA-sub-computation overshoot +
    // jitter.  The buggy code (no cancel inside ObtainSP_DAG) runs ~8 s here.
    EXPECT_LT(elapsed_s, 4.0)
        << "EnumeratePA_with_TimeLimits did not respect TIME_LIMIT ("
        << GlobalVariables::TIME_LIMIT << " s) when a single SP-eval exceeded "
        << "the cap. The cooperative cancel inside ObtainSP_DAG/RTA did not "
        << "fire. Elapsed: " << elapsed_s << " s.";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}

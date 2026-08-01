// #include <gtest/gtest.h>

#include <chrono>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Optimization/OptimizeSP_TL_Incre.h"
#include "sources/Safety_Performance_Metric/Probability.h"  // GaussianDist, FiniteDist, Value_Proba
#include "sources/TaskModel/RegularTasks.h"                 // Task, DAG_Model, MAP_Prev, TaskSet
#include "sources/Utils/Parameters.h"

using ::testing::AtLeast;  // #1
using ::testing::Return;
using namespace std;
using namespace SP_OPT_PA;
using namespace GlobalVariables;

// Helper to build a synthetic task with time-performance pairs (mirrors
// testBFRTimeout.cpp's MakeTaskWithPerf so the two suites share a vocabulary).
static Task MakeTaskWithPerf(int id, double period, const std::string& name,
                             const std::vector<TimePerfPair>& pairs) {
    std::vector<Value_Proba> dist = {Value_Proba(1, 1.0)};
    Task t(id, FiniteDist(dist), period, period, 0, name);
    t.timePerformancePairs = pairs;
    return t;
}

// P1.14-mirror: the INCR optimizer (OptimizePA_Incre_with_TimeLimits) now
// installs a BFDLSharedBudget guard at Optimize_w_TL_ScratchOrIncre entry, so a
// runaway EvaluateSPWithPriorityVec / RTA convolution can be interrupted in
// place — exactly the BF fix (testBFRTimeout.cpp) one level up. These two cases
// transpose the BF proof to the INCR dispatcher.
class TaskSetINCRWithTimeout : public ::testing::Test {
   public:
    void SetUp() override {
        saved_time_limit_ = GlobalVariables::TIME_LIMIT;
        saved_reopt_period_ = GlobalVariables::ReoptimizationPeriod;
        saved_debug_mode_ = GlobalVariables::debugMode;
    }

    void TearDown() override {
        GlobalVariables::TIME_LIMIT = saved_time_limit_;
        GlobalVariables::ReoptimizationPeriod = saved_reopt_period_;
        GlobalVariables::debugMode = saved_debug_mode_;
    }

    // Explosive DAG: many tasks each with several TL options so the serialized
    // E+L queue / coordinate descent explores a combinatorial TL space. Used by
    // the zero-budget case (bails on the first inner poll).
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
    int saved_reopt_period_;
    int saved_debug_mode_;
};

// With TIME_LIMIT set to 0 the guarded dispatcher bails out on the very first
// inner BFSharedBudgetCancelled() poll. ReoptimizationPeriod is forced to 1 so
// interval 0 routes through ReOptimizePeriodic (the from-scratch descent — the
// most expensive path), maximizing the number of inner polls that fire.
//
// debugMode is turned OFF for this case: with debugMode=1 (the yaml default)
// OptimizeFromScratch's beam search emits a std::cout line per partial-path
// node, and that I/O dominates the wall-clock — it would mask whether the
// cancel actually fired. The mechanism under test is the budget poll, not the
// print path, so we measure it with the prints silenced. The unguarded code
// (no BFDLSharedBudget at the INCR dispatcher) would run the full 3^8 TL
// combinatorial search × the per-config beam — many seconds; a 500 ms ceiling
// is a >10x margin over the guarded bail and still catches a regression where
// the poll stops firing.
TEST_F(TaskSetINCRWithTimeout, RespectsGlobalTimeLimit) {
    DAG_Model dag = BuildExplosiveDAG(8, 3, 1000.0);
    SP_Parameters sp(dag);

    GlobalVariables::TIME_LIMIT = 0;
    GlobalVariables::ReoptimizationPeriod = 1;
    GlobalVariables::debugMode = 0;

    OptimizePA_Incre_with_TimeLimits opt(dag, sp);
    // The dispatcher now requires a pre-computed safe fallback (P0.6 §8: it
    // cannot build the worst-case DAG from dag_tasks_). Pre-call outside the
    // timed window so the budget-guard measurement below is unaffected. Under
    // TIME_LIMIT=0 the pre-call's own budget bails its seed eval; the gate is
    // vacuous here (no important tasks) → it stores the seed and returns fast.
    opt.ComputeSafeFallback(dag);

    auto start = std::chrono::high_resolution_clock::now();
    opt.Optimize_w_TL_ScratchOrIncre(
        dag, GlobalVariables::Layer_Node_During_Incremental_Optimization);
    auto end = std::chrono::high_resolution_clock::now();

    double elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count();

    EXPECT_LT(elapsed_ms, 500.0)
        << "INCR Optimize_w_TL_ScratchOrIncre did not respect TIME_LIMIT ("
        << GlobalVariables::TIME_LIMIT << " s). Elapsed: " << elapsed_ms
        << " ms.";
}

// P1.14-mirror — the real defect: a single EvaluateSPWithPriorityVec call
// whose RTA convolutions exceed TIME_LIMIT strands the INCR search past the cap,
// because without the guard BFSharedBudgetCancelled() is always false on the
// INCR path and no check fires inside ObtainSP_DAG / the per-task RTA loop.
// This taskset is the SAME wide-Gaussian construction as testBFRTimeout.cpp's
// RespectsGlobalTimeLimit_SingleEvalExceedsCap (7 tasks, granularity 300 -> one
// SP-eval ~8 s), with NO timePerformancePairs so the only TL option is the
// default -1 (the serialized queue walks a single config; the ONLY way the cap
// can be honored is cooperative cancel INSIDE the one SP-eval). With TIME_LIMIT=1
// the unguarded code runs the full ~8 s; the guarded code installs
// BFDLSharedBudget at Optimize_w_TL_ScratchOrIncre entry and polls
// BFSharedBudgetCancelled() between RTA sub-computations, so the runaway eval is
// interrupted in place and the call stays bounded at ~1-2 s.
//
// ReoptimizationPeriod=1 forces interval 0 through ReOptimizePeriodic -> the
// from-scratch descent, which scores the carried PA fresh (one expensive
// EvaluateSPWithPriorityVec) — the exact runaway-eval scenario.
TEST_F(TaskSetINCRWithTimeout, RespectsGlobalTimeLimit_SingleEvalExceedsCap) {
    // 7 wide-Gaussian tasks; granularity 300 makes one SP-eval ~8 s.
    const int N = 7;
    const int granularity = 300;
    TaskSet tasks;
    for (int i = 0; i < N; ++i) {
        double period = 100.0 * (i + 1);
        GaussianDist g(50.0 + 5 * i, 15.0);
        FiniteDist dist(g, g.mu - 3 * g.sigma, g.mu + 3 * g.sigma, granularity);
        // No timePerformancePairs -> only the default -1 TL option -> the ONLY
        // way the cap can be honored is cooperative cancel INSIDE the one SP-eval.
        tasks.push_back(
            Task(i, dist, period, period * 0.9, 0, "T" + std::to_string(i)));
    }
    MAP_Prev empty_prev;
    DAG_Model dag(tasks, empty_prev, 0, 0);
    SP_Parameters sp(dag);

    GlobalVariables::TIME_LIMIT = 1;
    GlobalVariables::ReoptimizationPeriod = 1;
    // Silence the per-node debug prints (see RespectsGlobalTimeLimit) so the
    // wall-clock measures the RTA convolutions + the cancel poll, not stdout
    // I/O. The cancel mechanism is independent of debugMode.
    GlobalVariables::debugMode = 0;

    OptimizePA_Incre_with_TimeLimits opt(dag, sp);
    // Pre-call the safe fallback (P0.6 §8 contract: the dispatcher requires it).
    // Outside the timed window; under TIME_LIMIT=1 the pre-call's seed eval bails
    // at the cap, the gate is vacuous (no important tasks) → stores the seed.
    opt.ComputeSafeFallback(dag);

    auto start = std::chrono::high_resolution_clock::now();
    opt.Optimize_w_TL_ScratchOrIncre(
        dag, GlobalVariables::Layer_Node_During_Incremental_Optimization);
    auto end = std::chrono::high_resolution_clock::now();

    double elapsed_s =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count() /
        1000.0;

    // Budget is 1 s; allow up to 4 s for one RTA-sub-computation overshoot +
    // jitter.  The unguarded code (no BFDLSharedBudget at the INCR dispatcher)
    // runs ~8 s here.
    EXPECT_LT(elapsed_s, 4.0)
        << "INCR Optimize_w_TL_ScratchOrIncre did not respect TIME_LIMIT ("
        << GlobalVariables::TIME_LIMIT << " s) when a single SP-eval exceeded "
        << "the cap. The cooperative cancel inside ObtainSP_DAG/RTA did not "
        << "fire (no BFDLSharedBudget installed at the INCR dispatcher). "
        << "Elapsed: " << elapsed_s << " s.";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}

// #include <gtest/gtest.h>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Optimization/OptimizeSP_Base.h"
#include "sources/Optimization/OptimizeSP_TL_BF.h"
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

int main(int argc, char** argv) {
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}

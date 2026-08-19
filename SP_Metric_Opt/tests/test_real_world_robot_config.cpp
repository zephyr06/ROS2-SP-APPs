// Real-world robot config evaluation, as a unit test.
//
// P0.11 (`debug_real_world_exp_config`): evaluate whether the BF and the
// incremental optimizers find the safety-correct priority ordering on the
// real-world 4-task / 2-core experiment config (TSP + SLAM compete on core 0).
//
// Hypothesis (the "performance" under test):
//   * SLAM execution time LOW  (mu=100)  -> optimizer assigns TSP HIGHER
//                                            priority than SLAM.
//   * SLAM execution time HIGH (mu=1200) -> optimizer assigns TSP LOWER
//                                            priority than SLAM.
//
// Realistic sp_thresholds only (0.1 and 0.01). The lenient 0.9 threshold is
// NOT a realistic real-world case and is deliberately excluded.
//
// Scenario yamls live in TaskData/test_real_world_robot_config/. They are
// self-contained copies (do NOT mutate all_time_records/task_characteristics.yaml).
//
// Priority convention (ResourceOptResult::id2priority): BIGGER integer = HIGHER
// priority.
//
// NOTE on SP comparability: SP_Func normalises each task between a threshold-
// dependent penalty floor and reward ceiling, so SP values are NOT comparable
// ACROSS thresholds. They ARE comparable within one scenario (same threshold),
// where BF (global optimum) SP >= INCR SP must hold. The P1.30 fix
// (FindTaskWithDifferentEt tight 1e-6 approx_equal) keeps INCR from inflating
// above BF; the INCR test below guards against that regression.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "sources/Optimization/OptimizeSP_TL_BF.h"
#include "sources/Optimization/OptimizeSP_TL_Incre.h"
#include "sources/Safety_Performance_Metric/SP_Metric.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/Parameters.h"

using namespace std;
using namespace SP_OPT_PA;
using namespace GlobalVariables;

namespace {

constexpr double kSpTolerance = 1e-2;  // INCR near-optimality vs BF (same scenario)

const string kScenarioDir =
    PROJECT_PATH + "TaskData/test_real_world_robot_config/";

// A scenario: which yaml, and whether the hypothesis expects TSP > SLAM.
struct Scenario {
    const char* file;
    bool tsp_higher_than_slam;  // true = low SLAM-ET; false = high SLAM-ET
};

const vector<Scenario>& LowScenarios() {
    static const vector<Scenario> v = {
        {"rw_slam_et_low_thr0p1.yaml", true},
        {"rw_slam_et_low_thr0p01.yaml", true},
    };
    return v;
}
const vector<Scenario>& HighScenarios() {
    static const vector<Scenario> v = {
        {"rw_slam_et_high_thr0p1.yaml", false},
        {"rw_slam_et_high_thr0p01.yaml", false},
    };
    return v;
}

int TaskIdByName(const DAG_Model& dag, const string& name) {
    for (const auto& t : dag.tasks)
        if (t.name == name) return t.id;
    ADD_FAILURE() << "task not found: " << name;
    return -1;
}

// Bigger id2priority value = higher priority.
int PriorityOf(const ResourceOptResult& res, int id) {
    return res.id2priority.at(id);
}

ResourceOptResult RunBF(const string& yaml_path) {
    DAG_Model dag = ReadDAG_Tasks(yaml_path);
    SP_Parameters sp = ReadSP_Parameters(yaml_path);
    return EnumeratePA_with_TimeLimits(dag, sp);
}

ResourceOptResult RunINCR(const string& yaml_path) {
    DAG_Model dag = ReadDAG_Tasks(yaml_path);
    SP_Parameters sp = ReadSP_Parameters(yaml_path);
    OptimizePA_Incre_with_TimeLimits opt(dag, sp);
    opt.ReOptimizePeriodic(dag,
                           Layer_Node_During_Incremental_Optimization);
    return opt.CollectResults();
}

// Assert the TSP-vs-SLAM ordering matches the hypothesis for one result.
void ExpectHypothesis(const string& label, const DAG_Model& dag,
                      const ResourceOptResult& res, bool tsp_higher) {
    int tsp = TaskIdByName(dag, "TSP");
    int slam = TaskIdByName(dag, "SLAM");
    int pri_tsp = PriorityOf(res, tsp);
    int pri_slam = PriorityOf(res, slam);
    if (tsp_higher)
        EXPECT_GT(pri_tsp, pri_slam)
            << label << ": expected TSP higher than SLAM (low SLAM-ET), got "
            << "TSP=" << pri_tsp << " SLAM=" << pri_slam;
    else
        EXPECT_LT(pri_tsp, pri_slam)
            << label << ": expected TSP lower than SLAM (high SLAM-ET), got "
            << "TSP=" << pri_tsp << " SLAM=" << pri_slam;
}

}  // namespace

// ---------------------------------------------------------------------------
// BF (brute-force global optimum): must satisfy the priority-swap hypothesis
// at every realistic threshold.
// ---------------------------------------------------------------------------
TEST(RealWorldRobotConfig, BF_LowSlamEt_AssignsTspHigherThanSlam) {
    for (const Scenario& s : LowScenarios()) {
        string path = kScenarioDir + s.file;
        DAG_Model dag = ReadDAG_Tasks(path);
        ResourceOptResult res = RunBF(path);
        ExpectHypothesis(string("BF/") + s.file, dag, res, s.tsp_higher_than_slam);
    }
}

TEST(RealWorldRobotConfig, BF_HighSlamEt_AssignsTspLowerThanSlam) {
    for (const Scenario& s : HighScenarios()) {
        string path = kScenarioDir + s.file;
        DAG_Model dag = ReadDAG_Tasks(path);
        ResourceOptResult res = RunBF(path);
        ExpectHypothesis(string("BF/") + s.file, dag, res, s.tsp_higher_than_slam);
    }
}

// ---------------------------------------------------------------------------
// INCR: at HIGH SLAM-ET it matches the hypothesis (TSP lower than SLAM).
// ---------------------------------------------------------------------------
TEST(RealWorldRobotConfig, INCR_HighSlamEt_AssignsTspLowerThanSlam) {
    for (const Scenario& s : HighScenarios()) {
        string path = kScenarioDir + s.file;
        DAG_Model dag = ReadDAG_Tasks(path);
        ResourceOptResult res = RunINCR(path);
        ExpectHypothesis(string("INCR/") + s.file, dag, res,
                         s.tsp_higher_than_slam);
    }
}

// ---------------------------------------------------------------------------
// INCR near-optimality: INCR SP must not exceed BF (global optimum) and must
// stay close to it. Guards against the P1.30 RTA-cache SP-inflation regression
// (FindTaskWithDifferentEt loose approx_equal). At LOW SLAM-ET the SP surface
// has a plateau (TSP>SLAM and SLAM>TSP are SP-equivalent), so INCR may pick the
// opposite PA from BF while still being SP-optimal -> we check SP, not PA.
// ---------------------------------------------------------------------------
TEST(RealWorldRobotConfig, INCR_NearOptimalSP_NeverExceedsBF) {
    for (const Scenario& s : LowScenarios()) {
        string path = kScenarioDir + s.file;
        ResourceOptResult bf = RunBF(path);
        ResourceOptResult incr = RunINCR(path);
        EXPECT_LE(incr.sp_opt, bf.sp_opt + kSpTolerance)
            << s.file << ": INCR SP " << incr.sp_opt
            << " exceeds BF SP " << bf.sp_opt << " (inflation regression?)";
        EXPECT_GE(incr.sp_opt, bf.sp_opt - kSpTolerance)
            << s.file << ": INCR SP " << incr.sp_opt
            << " far below BF SP " << bf.sp_opt;
    }
    for (const Scenario& s : HighScenarios()) {
        string path = kScenarioDir + s.file;
        ResourceOptResult bf = RunBF(path);
        ResourceOptResult incr = RunINCR(path);
        EXPECT_LE(incr.sp_opt, bf.sp_opt + kSpTolerance)
            << s.file << ": INCR SP " << incr.sp_opt
            << " exceeds BF SP " << bf.sp_opt << " (inflation regression?)";
        EXPECT_GE(incr.sp_opt, bf.sp_opt - kSpTolerance)
            << s.file << ": INCR SP " << incr.sp_opt
            << " far below BF SP " << bf.sp_opt;
    }
}

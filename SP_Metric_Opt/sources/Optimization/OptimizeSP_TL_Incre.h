#pragma once
#include "sources/Optimization/OptimizeSP_Incre.h"
#include "sources/Optimization/OptimizeSP_TL_BF.h"
#include "sources/Safety_Performance_Metric/SP_Metric.h"

namespace SP_OPT_PA {

// Relative-tolerance comparison for SP values.  Near-identical SPs produced by
// probabilistic RTA numerical noise are treated as equal so that the
// tie-breaker (tightest TL / smallest sum) can make a deterministic choice.
inline bool ApproxEqualSP(double a, double b, double rel_tol = 1e-9) {
    double diff = std::abs(a - b);
    if (diff <= rel_tol)
        return true;  // guard for values near zero
    double max_abs = std::max(std::abs(a), std::abs(b));
    return diff <= rel_tol * max_abs;
}

// this function must return values sorted from low to high
std::vector<std::vector<double>> RecordCloseTimeLimitOptions(
    const DAG_Model& dag_tasks, int radius);

struct HashKey4Vector {
    std::size_t operator()(const std::vector<double>& v) const {
        std::size_t seed = v.size();
        for (auto& i : v) {
            seed ^=
                std::hash<double>()(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

struct TaskSortingHeuristic {
    const DAG_Model& dag_tasks;
    const SP_Parameters& sp_parameters;

    bool operator()(size_t idx1, size_t idx2) const;
};

class OptimizePA_Incre_with_TimeLimits : public OptimizePA_Incre {
   public:
    OptimizePA_Incre_with_TimeLimits() {};
    OptimizePA_Incre_with_TimeLimits(const DAG_Model& dag_tasks,
                                     const SP_Parameters& sp_parameters)
        : OptimizePA_Incre(dag_tasks, sp_parameters) {
        time_limit_option_for_each_task_ = RecordTimeLimitOptions(dag_tasks_);
    }

    void TraverseTimeLimitOptions(int K, uint task_id,
                                  std::vector<double>& time_limits);

    // 1-arg overload: entry point for the INCR_SCRATCH ablation. Delegates to
    // the 3-arg ReOptimizePeriodic(dag, K,
    // ReoptimizationTimeLimitSearchRadius). INCR_SCRATCH constructs a FRESH
    // optimizer each interval (see SimulationOrchestrator INCR_SCRATCH branch),
    // so prev_optimizer_ is always uninitialized here → SeedIncumbentBaseline
    // takes its interval-0 branch (RM + min-TL) every call. This makes
    // INCR_SCRATCH an AMNESIAC wide-radius reopt: the compare-and-keep guard
    // measures the search against a synthetic RM baseline, NOT against the
    // previous interval's adopted solution. Contrast with
    // Optimize_w_TL_ScratchOrIncre (INCR with ReoptimizationPeriod=1), which
    // reuses a persistent optimizer so prev_optimizer_ carries the prior
    // interval's incumbent — its compare-and-keep is measured against that
    // running best. INCR(period=1) therefore weakly dominates INCR_SCRATCH in
    // SP (never worse, sometimes strictly better); INCR_SCRATCH is kept only as
    // the ablation that isolates the value of carrying the incumbent forward.
    PriorityVec ReOptimizePeriodic(int K);

    PriorityVec OptimizeIncre_w_TL(const DAG_Model& dag_tasks_update, int K);
    PriorityVec OptimizeIncre_w_TL(const DAG_Model& dag_tasks_update, int K,
                                   int radius);

    PriorityVec ReOptimizePeriodic(const DAG_Model& dag_tasks_update, int K,
                                   int radius);

    // Counter-driven dispatcher: every ReoptimizationPeriod-th call re-runs the
    // TL search with the wide radius (ReOptimizePeriodic, compare-and-keep);
    // otherwise runs the narrow-radius incremental search (OptimizeIncre_w_TL).
    // count == 0 routes to ReOptimizePeriodic, whose SeedIncumbentBaseline
    // interval-0 branch synthesizes an RM+min-TL incumbent — so the dispatcher
    // also serves as the interval-0 bootstrap for the INCR paths. The counter
    // advances by 1 after every call and never resets (modular arithmetic alone
    // decides reopt vs incremental).
    PriorityVec Optimize_w_TL_ScratchOrIncre(const DAG_Model& dag_tasks_update,
                                             int K);

    void ApplyWCETAblationIfRequired(DAG_Model& dag_tasks);

    void UpdateRecords(const OptimizePA_Incre& optimizer,
                       const std::vector<double>& time_limits);

    // Evaluates one TL vector. When `from_scratch` is true a fresh
    // OptimizePA_Incre is built and OptimizeFromScratch(K) is run, ignoring any
    // warm state. When false, the evaluator warm-starts from `prev_optimizer_`
    // (the incumbent) via OptimizeIncre, falling back to OptimizeFromScratch(K)
    // only when no incumbent exists yet.
    double EvaluateTimeLimitConfig_ScratchOrIncre(
        int K, const std::vector<double>& time_limits, bool from_scratch);

    std::vector<double> InitializeTimeLimitsFromETConfig();
    void InitializeTimeLimitsToSmallest(std::vector<double>& time_limits);
    // One time-limit per task, each at its smallest option (-1 if a task has no
    // time-performance pairs). Return-by-value variant;
    // InitializeTimeLimitsToSmallest delegates here.
    std::vector<double> SmallestTimeLimitVec() const;
    void PerformCoordinateDescentForTaskConfigOpt(
        int K, std::vector<double>& time_limits, bool from_scratch = false);

    // Fast path when GlobalVariables::disable_time_limit_opt is set: pin every
    // task's TL to its smallest option and evaluate that single config (no
    // coordinate-descent search). Returns the resulting priority assignment.
    PriorityVec OptimizeWithTimeLimitOptDisabled(
        int K, std::vector<double>& time_limits, bool from_scratch);

    // Compare-and-keep reoptimization helpers (see ReOptimizePeriodic 3-arg).
    // SeedIncumbentBaseline establishes the incumbent 4-tuple {dag, sp, pa, tl}
    // in state so UpdateRecords' compare guard acts as compare-and-keep.
    std::vector<double> ReconstructTimeLimitVecFromResOpt();
    PriorityVec RateMonotonicPriorityVec();
    void SeedStateFromIncumbent(const DAG_Model& dag_with_tl,
                                const PriorityVec& pa, double sp,
                                const std::vector<double>& tl);
    void SeedIncumbentBaseline();

    inline ResourceOptResult CollectResults() const { return res_opt_; }

    // data members
    ResourceOptResult res_opt_;
    std::vector<std::vector<double>> time_limit_option_for_each_task_;
    // Old implementation is based on timelimit2optimizer_: For each task id, it
    // maps time limit to the optimizer
    // We want to try a simpler approach
    OptimizePA_Incre prev_optimizer_;
    int eval_count_ = 0;
    int reoptimization_interval_count_ = 0;
};

inline PriorityVec PerformOptimizePA_Incre_w_TimeLimits(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    return opt.OptimizeFromScratch(
        GlobalVariables::Layer_Node_During_Incremental_Optimization);
}
}  // namespace SP_OPT_PA
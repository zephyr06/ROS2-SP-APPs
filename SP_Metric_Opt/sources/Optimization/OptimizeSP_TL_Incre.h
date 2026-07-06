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

// Trial-and-Error TL optimization helpers. Extracted from the coordinate
// descent so the lookup and the adopt predicate are unit-testable in isolation
// (they take no optimizer state). See OptimizeSingleTaskTimeLimit for the walk.

// Linear scan for `current_val` in `options`. Returns options.size() (the
// off-the-end sentinel, mirroring std::find) when the value is absent — the
// walk treats that sentinel as "baseline not in window, do not search".
size_t FindTimeLimitOptionIndex(const std::vector<double>& options,
                                double current_val);

// Per-step adopt predicate for the unidirectional TL walk. Returns true when
// `new_sp` strictly exceeds `current_best_sp` (not ApproxEqualSP), OR when the
// two are approx-equal (a tie) AND the walk is heading downward (step < 0) —
// the tie-break prefers the smaller TL, so a downward tie is an improvement
// while an upward tie is not. Mirrors the exhaustive descent's tie-break.
bool IsBetterTimeLimitOption(double new_sp, double current_best_sp, int step);

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
    // the 2-arg ReOptimizePeriodic(dag, K). INCR_SCRATCH constructs a FRESH
    // optimizer each interval (see SimulationOrchestrator INCR_SCRATCH branch),
    // so prev_optimizer_ is always uninitialized here → SeedIncumbentBaseline
    // takes its interval-0 branch (RM + min-TL) every call. This makes
    // INCR_SCRATCH an AMNESIAC reopt: the compare-and-keep guard measures the
    // search against a synthetic RM baseline, NOT against the previous
    // interval's adopted solution. Contrast with
    // Optimize_w_TL_ScratchOrIncre (INCR with ReoptimizationPeriod=1), which
    // reuses a persistent optimizer so prev_optimizer_ carries the prior
    // interval's incumbent — its compare-and-keep is measured against that
    // running best. INCR(period=1) therefore weakly dominates INCR_SCRATCH in
    // SP (never worse, sometimes strictly better); INCR_SCRATCH is kept only as
    // the ablation that isolates the value of carrying the incumbent forward.
    PriorityVec ReOptimizePeriodic(int K);

    PriorityVec OptimizeIncre_w_TL(const DAG_Model& dag_tasks_update, int K);

    PriorityVec ReOptimizePeriodic(const DAG_Model& dag_tasks_update, int K);

    // Counter-driven dispatcher: every ReoptimizationPeriod-th call re-runs the
    // TL search from scratch (ReOptimizePeriodic, compare-and-keep); otherwise
    // runs the incremental search (OptimizeIncre_w_TL, warm-started). Both
    // paths walk the SAME full per-task option set — the difference is whether
    // the PA search warm-starts from the incumbent (incremental) or re-searches
    // the full beam (reopt) — and the patience budget the walk tolerates
    // (incremental: patience=0 strict; reopt: patience=1, see
    // PerformCoordinateDescentForTaskConfigOpt). count == 0 routes to
    // ReOptimizePeriodic, whose SeedIncumbentBaseline interval-0 branch
    // synthesizes an RM+min-TL incumbent — so the dispatcher also serves as the
    // interval-0 bootstrap for the INCR paths. The counter advances by 1 after
    // every call and never resets (modular arithmetic alone decides reopt vs
    // incremental).
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
    //
    // Virtual so the unidirectional trial-and-error walk
    // (OptimizeSingleTaskTimeLimit) can be unit-tested with a deterministic
    // TL→SP stub instead of the real RTA-backed evaluator. The override is the
    // only call the walk makes per candidate, so the indirection cost is
    // negligible relative to one ObtainSP_DAG.
    virtual double EvaluateTimeLimitConfig_ScratchOrIncre(
        int K, const std::vector<double>& time_limits, bool from_scratch);

    std::vector<double> InitializeTimeLimitsFromETConfig();
    void InitializeTimeLimitsToSmallest(std::vector<double>& time_limits);
    // One time-limit per task, each at its smallest option (-1 if a task has no
    // time-performance pairs). Return-by-value variant;
    // InitializeTimeLimitsToSmallest delegates here.
    std::vector<double> SmallestTimeLimitVec() const;
    void PerformCoordinateDescentForTaskConfigOpt(
        int K, std::vector<double>& time_limits, bool from_scratch = false);

    // Unidirectional trial-and-error walk for ONE task's time limit. Steps
    // outward from `baseline_val` in direction `step` (+1 up, -1 down) through
    // the task's FULL recorded option set (time_limit_option_for_each_task_,
    // which holds every timePerformancePairs entry, not a radius-bounded
    // window), evaluating each candidate via
    // EvaluateTimeLimitConfig_ScratchOrIncre. Adopts a candidate when
    // IsBetterTimeLimitOption returns true; otherwise spends one unit of
    // `patience` (consecutive-non-improvement budget). Stops when patience runs
    // out or the option-set boundary is reached. Returns the best SP found and
    // leaves `time_limits[task_idx]` at the best option tried.
    //
    // `patience`: 0 = strict break on the first non-improving step (used by the
    // incremental path, whose warm-started PA search makes SP-vs-TL effectively
    // unimodal so a dip never hides a better option); 1 = tolerate one dip
    // before breaking (used by the from-scratch reopt path, where SP-vs-TL can
    // be non-unimodal at high utilization, so a single dip must not hide a
    // strictly better option further out). `current_sp` is the best SP so far
    // across the whole coordinate descent (carries across tasks and across the
    // backward/forward passes). `baseline_val` is the TL the walk steps from —
    // it MUST be a member of the option set, else the walk is a no-op.
    double OptimizeSingleTaskTimeLimit(size_t task_idx, int K,
                                       std::vector<double>& time_limits,
                                       double current_sp, double baseline_val,
                                       int step, bool from_scratch,
                                       int patience);

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
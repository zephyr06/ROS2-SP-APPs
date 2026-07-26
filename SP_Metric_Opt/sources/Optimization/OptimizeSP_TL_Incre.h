#pragma once
#include "sources/Optimization/OptimizeSP_Incre.h"
#include "sources/Optimization/OptimizeSP_TL_BF.h"
#include "sources/Safety_Performance_Metric/RTA_Cache.h"
#include "sources/Safety_Performance_Metric/SP_Metric.h"

namespace SP_OPT_PA {

// Near-identical SPs from RTA numerical noise compare equal so the tightest-TL
// tie-breaker is deterministic.
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

// TL-walk helpers — stateless so they are unit-testable in isolation.

// Linear scan; returns options.size() (std::find sentinel) when absent, which
// the walk treats as "baseline not in window, do not search".
size_t FindTimeLimitOptionIndex(const std::vector<double>& options,
                                double current_val);

// Adopt when strictly better, or on an approx-equal tie when walking downward
// (step<0): the tie-break prefers the smaller TL, so a downward tie improves.
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

// One entry in the serialized E+L queue. `kind` dispatches the handler:
// EnvChanged → sub-incremental re-search at the committed TL; TLFlexible → the
// TL walk whose steps call the sub-incremental.
//
// `et_increased` is the env-move direction, captured once at queue-build so the
// Type-E handler avoids recomputing the env diff per entry. Unused (false) for
// TLFlexible — the walk derives per-step direction from the trial-vs-committed
// TL sign.
struct SerializedTaskQueueEntry {
    int task_id;
    enum class Kind { EnvChanged, TLFlexible } kind;
    bool et_increased = false;
};

// Which descent `RunIntervalDescent` runs. The two modes share ONE descent body
// + ONE walk arm; the only deltas are (1) patience (inc=0, reopt=1) and (2) the
// baseline-seed preamble (inc: dedicated re-score; reopt: the one upfront
// from-scratch beam), which `SeedBaselineAndArmCache` branches on. The modes are
// coupled to their call sites (inc ← OptimizeIncre_w_TL, reopt ← ReOptimizePeriodic),
// so an enum (not loose bools) is the right shape — it cannot express an invalid
// combination.
enum class IntervalDescentMode { Incremental, Reopt };

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

    PriorityVec OptimizeIncre_w_TL(const DAG_Model& dag_tasks_update, int K);

    PriorityVec ReOptimizePeriodic(const DAG_Model& dag_tasks_update, int K);

    // Counter-driven dispatcher: every ReoptimizationPeriod-th call re-runs the
    // TL search from scratch (ReOptimizePeriodic, compare-and-keep), else the
    // warm-started incremental search (OptimizeIncre_w_TL). Both walk the same
    // full option set; they differ in warm-start (inc) vs full re-search (reopt)
    // and patience (0 vs 1). count==0 → ReOptimizePeriodic, which bootstraps the
    // interval-0 incumbent (RM+min-TL). Counter advances every call, never resets.
    PriorityVec Optimize_w_TL_ScratchOrIncre(const DAG_Model& dag_tasks_update,
                                             int K);

    void ApplyWCETAblationIfRequired(DAG_Model& dag_tasks);

    bool UpdateRecords(const OptimizePA_Incre& optimizer,
                       const std::vector<double>& time_limits);

    // Evaluates one TL vector. from_scratch: a fresh OptimizePA_Incre runs
    // OptimizeFromScratch (the bootstrap). Otherwise warm-starts from a throwaway
    // challenger rebuilt from res_opt_ (BuildChallengerFromIncumbent) + OptimizeIncre;
    // requires an incumbent.
    // Virtual so the reopt descent's baseline beam can be unit-tested with a
    // TL→SP stub; the indirection is one call per candidate (negligible vs ObtainSP_DAG).
    virtual double EvaluateTimeLimitConfig_ScratchOrIncre(
        int K, const std::vector<double>& time_limits, bool from_scratch);

    // Serialized-queue SP-eval: like the incremental branch above but calls the
    // |diff|==1 primitive OptimizeIncre_SingleTask.
    // `task_idx`: the one task whose ET differs from the champion (Type-E: env-
    // changed task, committed TL; Type-L: walked task, trial TL). `et_increased`:
    // caller-supplied ET direction vs the champion — drives half-range pruning in
    // the primitive (wrong direction prunes the wrong half), so the caller MUST
    // supply it. `K` is carried for signature symmetry only (no beam in a 1D
    // re-search).
    // Virtual so the serialized walk can be unit-tested with a TL→SP stub.
    virtual double EvaluateTimeLimitConfig_SubIncremental(
        int K, const std::vector<double>& time_limits, size_t task_idx,
        bool et_increased);

    // Assert the single-change invariant on the serialized path (debugMode-only).
    // The diff must be EMPTY (Type-E: env move absorbed into dag_tasks_ on both
    // sides → cancels) or flag exactly `task_idx` (Type-L: the walked task's TL
    // moved). |diff|>1 or a 1-flagging-different-task means the champion drifted
    // → throws. No-op when debugMode is off.
    void AssertSingleChangeInvariant(const DAG_Model& champion_dag,
                                     const DAG_Model& candidate_dag,
                                     size_t task_idx) const;

    std::vector<double> InitializeTimeLimitsFromETConfig();
    void InitializeTimeLimitsToSmallest(std::vector<double>& time_limits);
    // One TL per task, each at its smallest option (-1 if no perf pairs).
    std::vector<double> SmallestTimeLimitVec() const;
    // Coordinate-descent seed: both paths seed from the carried adopted TL in
    // res_opt_ (ReconstructTimeLimitVecFromResOpt) — the optimizer's own prior
    // output. The Gaussian-mean TL (InitializeTimeLimitsFromETConfig) is the
    // interval-0 fallback only (no incumbent).
    // `dag_tasks_prev_pre_tl` is the pre-absorb DAG captured before
    // ReOptimizePeriodic absorbed dag_tasks_update; it is the Type-E diff source
    // for BuildSerializedTaskQueue on the reopt descent (the incremental descent
    // captures its own prev_pre_tl in OptimizeIncre_w_TL).
    // Thin delegating wrapper around RunIntervalDescent(Reopt). Kept (not deleted)
    // so the reopt path has a named entry the 5.5 regression test comments refer
    // to; the `from_scratch` arg is gone (always true at the sole caller, and the
    // mode enum carries the same info).
    void PerformCoordinateDescentForTaskConfigOpt(
        int K, std::vector<double>& starting_time_limits,
        const DAG_Model& dag_tasks_prev_pre_tl);

    // The ONE interval-descent body shared by the incremental and reopt paths.
    // Patience is mode-selected (inc=IncrementalTimeLimitSearchPatience,
    // reopt=ReoptimizationTimeLimitSearchPatience); the baseline-seed preamble +
    // cache arming live in SeedBaselineAndArmCache(mode); the tail
    // (BuildSerializedTaskQueue + WalkSerializedTaskQueue + unconditional cache
    // disarm) is mode-independent. `dag_tasks_prev_pre_tl` is the pre-absorb DAG
    // (Type-E diff source for the serialized queue).
    void RunIntervalDescent(int K, std::vector<double>& starting_time_limits,
                            IntervalDescentMode mode,
                            const DAG_Model& dag_tasks_prev_pre_tl);

    // Reset + baseline seed + cache arm/adopt/re-sync. Returns the baseline SP.
    // Encapsulates the cache-arming ASYMMETRY so RunIntervalDescent has no
    // cache if/else: the incremental branch arms the cache FIRST (its baseline
    // is a |diff|==0 FullReuse — same PA+TL re-scored under the new DAG — safe to
    // route through the cache); the reopt branch runs the one upfront from-
    // scratch beam DISARMED (the beam is a memoryless >1 change that would make
    // RTACache::ComputeTaskSetDifference throw |diff|>1), then arms + AdoptChampion
    // + re-syncs starting_time_limits to the champion TL (the 5.5 crash fix —
    // without it the first walk step diffs champion-TL vs seed-TL >1 → throw).
    // `ResetIncumbentBaseline(mode==Reopt)` clears the cache, so the reopt beam
    // starts disarmed regardless of the arm ordering inside this helper.
    double SeedBaselineAndArmCache(int K,
                                   std::vector<double>& starting_time_limits,
                                   IntervalDescentMode mode);

    // Unidirectional trial-and-error walk for one task's TL. Steps outward from
    // `baseline_val` (a member of the option set, else no-op) in direction
    // `step`, evaluating each candidate via `eval`; adopts on
    // IsBetterTimeLimitOption else spends one unit of `patience` (a total
    // non-improvement budget, not reset on improvement), stopping at 0 or the
    // boundary.
    // patience: 0 = strict break on first non-improving step (incremental, SP-vs-
    // TL ~unimodal); 1 = tolerate one non-improving step (reopt, can be
    // non-unimodal).
    // The walk core is unit-tested directly with an injected TL→SP stub.
    double OptimizeSingleTaskTimeLimit_Impl(
        size_t task_idx, std::vector<double>& time_limits, double current_sp,
        double baseline_val, int step, int patience,
        std::function<double(const std::vector<double>&)> eval);

    // One task's sub-incremental TL walk: builds the sub-incremental eval lambda
    // (binding EvaluateTimeLimitConfig_SubIncremental) and runs the backward
    // (step=-1) then forward (step=+1) passes over OptimizeSingleTaskTimeLimit_Impl,
    // finally syncing the working TL vector to the adopted champion. Shared by the
    // incremental serialized queue (PerformSerializedTaskQueueOptimization Type-L
    // body) and the reopt descent (PerformCoordinateDescentForTaskConfigOpt) —
    // both ran this exact block inline before P2.11 Phase 1, differing only in the
    // task_idx source.
    double OptimizeOneTaskTimeLimit(
        int K, size_t task_idx, std::vector<double>& starting_time_limits,
        double current_config_sp, double baseline_val, int patience);

    // Fast path when disable_time_limit_opt is set: pin every TL to its smallest
    // option and evaluate that single config (no descent).
    PriorityVec OptimizeWithTimeLimitOptDisabled(
        int K, std::vector<double>& time_limits, bool from_scratch);

    // Type-L set: task IDs whose option set is NOT the {-1}-only sentinel (pure query).
    std::vector<int> CollectTLFlexibleTaskIds() const;

    // Merged E+L queue: Type-E (FindEnvTaskWithDifferentEt) + Type-L
    // (CollectTLFlexibleTaskIds), sorted together by task weight descending.
    // `prev_pre_tl` is the local pre-TL DAG captured before the absorb.
    // Disjoint by generator design (TL-flexible tasks have no env dependence);
    // a task in BOTH → CoutError (contract violation, not a silent pick).
    std::vector<SerializedTaskQueueEntry> BuildSerializedTaskQueue(
        const DAG_Model& dag_tasks_prev_pre_tl) const;

    // Walks the merged E+L serialized queue in place. Type-E → SubIncremental
    // re-search at the committed TL (no TL walk); Type-L → OptimizeOneTaskTimeLimit
    // TL walk. Each step adopts into res_opt_, so the next step's challenger sees
    // the new champion and the diff flags only the walked task (|diff|<=1).
    // Shared by PerformSerializedTaskQueueOptimization (incremental) and
    // PerformCoordinateDescentForTaskConfigOpt (reopt) — both ran this exact
    // loop inline before P2.11 Phase 1. Returns the final SP; syncs
    // starting_time_limits to the adopted champion.
    double WalkSerializedTaskQueue(
        const std::vector<SerializedTaskQueueEntry>& queue, int K,
        std::vector<double>& starting_time_limits, double current_config_sp,
        int patience);

    // Serialized loop driver — the incremental-path replacement for
    // PerformCoordinateDescentForTaskConfigOpt. Resets the baseline, re-scores the
    // champion under the new env (dedicated re-score, NOT ScratchOrIncre — must
    // not optimize before the queue's order is honored), builds the E+L queue, and
    // walks it serially. `dag_tasks_prev_pre_tl` is the pre-TL DAG captured before
    // the absorb (Type-E diff source).
    // Virtual so the path can be unit-tested with an observing stub.
    virtual void PerformSerializedTaskQueueOptimization(
        int K, std::vector<double>& starting_time_limits,
        const DAG_Model& dag_tasks_prev_pre_tl);

    // Compare-and-keep helpers (see ReOptimizePeriodic).
    std::vector<double> ReconstructTimeLimitVecFromResOpt();
    PriorityVec RateMonotonicPriorityVec();
    void SeedStateFromIncumbent(const DAG_Model& dag_with_tl,
                                const PriorityVec& pa, double sp,
                                const std::vector<double>& tl);
    // Reset the incumbent baseline before a new interval. from_scratch (reopt):
    // re-eval the carried {pa, tl} under the new DAG (or RM+min-TL at interval 0)
    // and commit it, so opt_sp_ holds the baseline for compare-and-keep.
    // !from_scratch (incremental): set opt_sp_=-1.0 so the first UpdateRecords
    // force-commits.
    void ResetIncumbentBaseline(bool from_scratch);

    // Incumbent-state helpers. res_opt_ is the single durable store;
    // CommitIncumbent is its only writer. BuildChallengerFromIncumbent rebuilds a
    // throwaway challenger from res_opt_ each candidate (not persistent); the
    // champion tracks the working TL so the diff flags only the walked task.
    void CommitIncumbent(const PriorityVec& pa, double sp,
                         const std::vector<double>& tl);
    OptimizePA_Incre BuildChallengerFromIncumbent();

    inline ResourceOptResult CollectResults() const { return res_opt_; }

    // data members
    ResourceOptResult res_opt_;
    // Single-champion RTA cache; mirrors res_opt_ (adopted at CommitIncumbent,
    // reset at ResetIncumbentBaseline). Routes the SubIncremental baseline
    // re-score through the cache.
    RTACache rta_cache_;
    // true only inside PerformSerializedTaskQueueOptimization (the sole path
    // where |diff|<=1 holds, so AdoptChampion stays consistent with res_opt_).
    // Cleared by ResetIncumbentBaseline so the reopt path (shares CommitIncumbent
    // but can commit a >1 change via memoryless OptimizeFromScratch) neither
    // throws in Evaluate nor regresses. CommitIncumbent does cache work iff true.
    bool rta_cache_active_ = false;
    // Transient per-call TL search window, recorded fresh each call. Not part of
    // the incumbent (the carried TL lives in res_opt_.id2time_limit).
    std::vector<std::vector<double>> time_limit_option_for_each_task_;
    // Incumbent gated by IfInitialized() (!opt_pa_.empty()); CommitIncumbent is
    // its only writer, so no separate bool is needed.
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
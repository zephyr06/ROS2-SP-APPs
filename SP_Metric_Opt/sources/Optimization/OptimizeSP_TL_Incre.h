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

// One entry in the serialized E+L queue (P1.10). `kind` dispatches the step
// handler: EnvChanged → one sub-incremental re-search with the committed TL;
// TLFlexible → the trial-and-error TL walk whose steps call the sub-incremental.
//
// `et_increased` is meaningful ONLY for Kind::EnvChanged entries: it is the
// env-move direction (FindEnvTaskWithDifferentEt's DiffObj.increase), captured
// once when the queue is built so the Type-E handler does NOT recompute the full
// env diff per entry just to recover one bool. For Kind::TLFlexible entries it is
// unused (the walk derives et_increased per step from the trial-vs-committed TL
// sign) and set to false.
struct SerializedTaskQueueEntry {
    int task_id;
    enum class Kind { EnvChanged, TLFlexible } kind;
    bool et_increased = false;
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
    // ReOptimizePeriodic, whose ResetIncumbentBaseline(true) interval-0 branch
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
    // warm state. When false, the evaluator warm-starts from a throwaway
    // challenger rebuilt from the incumbent (res_opt_) via
    // BuildChallengerFromIncumbent, then runs OptimizeIncre. Requires an
    // incumbent (IfInitialized()); the from-scratch path is the bootstrap.
    //
    // Virtual so the unidirectional trial-and-error walk
    // (OptimizeSingleTaskTimeLimit) can be unit-tested with a deterministic
    // TL→SP stub instead of the real RTA-backed evaluator. The override is the
    // only call the walk makes per candidate, so the indirection cost is
    // negligible relative to one ObtainSP_DAG.
    virtual double EvaluateTimeLimitConfig_ScratchOrIncre(
        int K, const std::vector<double>& time_limits, bool from_scratch);

    // The shared SP-eval entry for the serialized queue (P1.10). Mirrors the
    // incremental branch of EvaluateTimeLimitConfig_ScratchOrIncre but calls
    // OptimizeIncre_SingleTask (the |diff|==1 primitive) instead of OptimizeIncre.
    //
    // `task_idx`: the ONE task whose ET differs from the champion (Type-E: the
    // env-changed task, time_limits = committed TL; Type-L: the TL-walked task,
    // time_limits = trial TL). `et_increased`: caller-supplied direction of the
    // ET change vs the champion — drives AnalyzePriorityChangeStatus's half-range
    // pruning inside the primitive (a wrong direction prunes the wrong half and
    // can miss the optimum), so the caller MUST supply it. `K` is carried for
    // signature symmetry with ScratchOrIncre; the sub-incremental primitive
    // re-searches one task's 1D positions and does not use a beam width.
    //
    // Virtual so the serialized walk can be unit-tested with a TL→SP stub.
    virtual double EvaluateTimeLimitConfig_SubIncremental(
        int K, const std::vector<double>& time_limits, size_t task_idx,
        bool et_increased);

    // P1.10 Phase 3: assert the single-change invariant on the serialized path
    // (debugMode-only). `champion_dag` = BuildChallengerFromIncumbent's DAG;
    // `candidate_dag` = the trial DAG the sub-incremental primitive is about to
    // re-search. The diff must be EMPTY (Type-E: the env move is absorbed into
    // dag_tasks_ on both sides → cancels; candidate DAG == champion DAG, only the
    // re-searched PA varies) or flag exactly `task_idx` (Type-L: the walked task's
    // TL moved). |diff|>1 or a 1-flagging-different-task means the champion
    // drifted — the single-change premise P1.9's rev-2 cache relies on (single-task
    // RTA patch, or full reuse at |diff|==0) is broken → throws via CoutError.
    // No-op when debugMode is off (production stays free of the per-eval diff cost).
    void AssertSingleChangeInvariant(const DAG_Model& champion_dag,
                                     const DAG_Model& candidate_dag,
                                     size_t task_idx) const;

    std::vector<double> InitializeTimeLimitsFromETConfig();
    void InitializeTimeLimitsToSmallest(std::vector<double>& time_limits);
    // One time-limit per task, each at its smallest option (-1 if a task has no
    // time-performance pairs). Return-by-value variant;
    // InitializeTimeLimitsToSmallest delegates here.
    std::vector<double> SmallestTimeLimitVec() const;
    // `starting_time_limits`: the per-task TL vector the descent walks from.
    // Origin is path-dependent but now uniform: both the incremental and the
    // reopt paths seed from the carried adopted TL in res_opt_
    // (ReconstructTimeLimitVecFromResOpt) — the P1.4 permanent, unconditional
    // algorithmic seed (the optimizer's own prior output). The Gaussian-mean
    // TL (InitializeTimeLimitsFromETConfig) is used ONLY as the interval-0
    // fallback when no incumbent exists (IfInitialized() false on a fresh
    // optimizer).
    void PerformCoordinateDescentForTaskConfigOpt(
        int K, std::vector<double>& starting_time_limits,
        bool from_scratch = false);

    // Unidirectional trial-and-error walk for ONE task's time limit. Steps
    // outward from `baseline_val` in direction `step` (+1 up, -1 down) through
    // the task's FULL recorded option set, evaluating each candidate via `eval`
    // (a TL->SP function bound by the caller). Adopts a candidate when
    // IsBetterTimeLimitOption returns true; otherwise spends one unit of
    // `patience` (a total non-improvement budget, NOT reset on improvement).
    // Stops when patience hits 0 or the option-set boundary is reached.
    //
    // `patience`: 0 = strict break on the first non-improving step (incremental
    // path, warm-started PA search makes SP-vs-TL ~unimodal); 1 = tolerate one
    // non-improving step total before breaking (reopt path, where SP-vs-TL can
    // be non-unimodal at high utilization). `current_sp` is the best SP so far
    // across the whole descent. `baseline_val` is the TL the walk steps from —
    // it MUST be a member of the option set, else the walk is a no-op.
    //
    // `eval` receives the trial `time_limits` (with task_idx already set to the
    // candidate) and returns the resulting SP. The eval-injection overload is
    // the walk core: the legacy 7-arg wrapper below binds `eval` to
    // EvaluateTimeLimitConfig_ScratchOrIncre (the reopt/BF path); the
    // serialized Type-L step (P1.10) binds it to the sub-incremental eval,
    // which skips the redundant re-score the legacy eval pays each step.
    double OptimizeSingleTaskTimeLimit(
        size_t task_idx, int K, std::vector<double>& time_limits,
        double current_sp, double baseline_val, int step, bool from_scratch,
        int patience);

    // The walk core with an injected eval. `K` is captured into the eval
    // closure by the caller (the legacy path threads `from_scratch` instead).
    // Identical walk to the 7-arg overload above; factored out so the
    // serialized Type-L step can reuse the SAME patience-bounded outward walk
    // with a different (sub-incremental) per-candidate eval — behavior of the
    // walk itself is unchanged.
    double OptimizeSingleTaskTimeLimit_Impl(
        size_t task_idx, std::vector<double>& time_limits, double current_sp,
        double baseline_val, int step, int patience,
        std::function<double(const std::vector<double>&)> eval);

    // Fast path when GlobalVariables::disable_time_limit_opt is set: pin every
    // task's TL to its smallest option and evaluate that single config (no
    // coordinate-descent search). Returns the resulting priority assignment.
    PriorityVec OptimizeWithTimeLimitOptDisabled(
        int K, std::vector<double>& time_limits, bool from_scratch);

    // Type-L set (P1.10, helper C): task IDs with TL freedom, i.e. those whose
    // time_limit_option_for_each_task_[id] is NOT the {-1}-only sentinel. These
    // are the tasks the legacy descent walks (PerformCoordinateDescentForTaskConfigOpt
    // :266-267 skip is the inverse filter). Pure query over existing state.
    std::vector<int> CollectTLFlexibleTaskIds() const;

    // Merged + sorted E+L queue (P1.10, function D). Merges the Type-E set
    // (FindEnvTaskWithDifferentEt(dag_tasks_prev_pre_tl, dag_tasks_)) and the
    // Type-L set (CollectTLFlexibleTaskIds), sorts TOGETHER by task WEIGHT
    // DESCENDING (D3 — simple, uniform key; high-weight first). Each entry
    // carries its Kind so the loop dispatches to the right handler. `prev_pre_tl`
    // is the LOCAL pre-TL DAG captured by the caller before the absorb.
    //
    // DEDUP POLICY (#5): a task may NOT be both env-changed AND TL-flexible
    // (disjoint by generator design — TL-flexible tasks have no env dependence).
    // If a task appears in BOTH at runtime → CoutError (a contract violation,
    // not an optimization choice). NOT a silent winner-pick.
    std::vector<SerializedTaskQueueEntry> BuildSerializedTaskQueue(
        const DAG_Model& dag_tasks_prev_pre_tl) const;

    // Serialized loop driver (P1.10, function F) — the INCREMENTAL-path
    // replacement for PerformCoordinateDescentForTaskConfigOpt. Resets the
    // incumbent baseline, re-scores the champion under the new env (dedicated
    // re-score, NOT ScratchOrIncre — #6: must not optimize before the queue's
    // sorted order is honored), builds the E+L queue, and walks it serially
    // (EnvChanged → EvaluateTimeLimitConfig_SubIncremental; TLFlexible → the TL
    // walk whose steps call the sub-incremental). `dag_tasks_prev_pre_tl` is the
    // LOCAL pre-TL DAG captured before the :313 absorb (Type-E diff source).
    //
    // Virtual so the serialized incremental path can be unit-tested with a stub
    // that observes the entry (e.g. the starting TL vector, or that the
    // incremental branch ran at all) without altering the real RTA-driven walk —
    // mirroring the ScratchOrIncre override pattern.
    virtual void PerformSerializedTaskQueueOptimization(
        int K, std::vector<double>& starting_time_limits,
        const DAG_Model& dag_tasks_prev_pre_tl);

    // Compare-and-keep helpers (see ReOptimizePeriodic 3-arg).
    std::vector<double> ReconstructTimeLimitVecFromResOpt();
    PriorityVec RateMonotonicPriorityVec();
    void SeedStateFromIncumbent(const DAG_Model& dag_with_tl,
                                const PriorityVec& pa, double sp,
                                const std::vector<double>& tl);
    // Reset the incumbent baseline before a new interval's search.
    // from_scratch=true (reopt): re-eval the carried {pa, tl} under the new DAG
    // (or RM+min-TL at interval 0) and commit it, so opt_sp_ holds the baseline
    // and UpdateRecords' guard acts as compare-and-keep. from_scratch=false
    // (incremental): set opt_sp_=-1.0 so the first UpdateRecords force-commits
    // and res_opt_ is overwritten for the current interval.
    void ResetIncumbentBaseline(bool from_scratch);

    // Incumbent-state helpers (P0.5). res_opt_ is the single durable store;
    // CommitIncumbent is its only writer. BuildChallengerFromIncumbent rebuilds a
    // throwaway challenger from res_opt_ (the champion) each incremental candidate
    // — NOT a persistent challenger. The champion tracks the working TL, so the
    // diff flags only the task currently being walked → OptimizeIncre re-searches
    // just that task. Perfect for incremental opt; chosen over a persistent
    // challenger, which would drift to non-adopted candidates and flag extras.
    void CommitIncumbent(const PriorityVec& pa, double sp,
                         const std::vector<double>& tl);
    OptimizePA_Incre BuildChallengerFromIncumbent();

    inline ResourceOptResult CollectResults() const { return res_opt_; }

    // data members
    ResourceOptResult res_opt_;
    // Transient per-call: the per-task TL search window recorded fresh at the
    // top of each OptimizeIncre_w_TL / ReOptimizePeriodic call. NOT part of the
    // incumbent (the carried TL lives in res_opt_.id2time_limit).
    std::vector<std::vector<double>> time_limit_option_for_each_task_;
    // The incumbent is gated by IfInitialized() (base class: !opt_pa_.empty()).
    // CommitIncumbent is the single writer of opt_pa_, so it is the only thing
    // that establishes an incumbent — no separate bool is needed.
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
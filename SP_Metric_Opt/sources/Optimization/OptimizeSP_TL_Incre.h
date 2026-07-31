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

// P0.6 static-solution TL seed helper. Given a perf task's sorted-ascending
// timePerformancePairs (the TL grid) and its et_mean
// (= execution_time_dist.GetAvgValue()), return the index of the LARGEST grid
// option that is <= et_mean. This is the directional (at-or-below) variant of
// Find_Close_ExecutionTime (which is bidirectional and may pick above et_mean):
// the static solution must seed at-or-below the P0.8-certified WCET (= et_mean)
// so the seed's runtime ET (sim caps perf ET at min(et_mean, TL)) is <= the
// certified WCET → the seed is feasible-by-construction. If NO grid option is
// <= et_mean (et_mean below the smallest option), fall back to the smallest
// option (index 0) and let the caller flag it — the clamp is still feasible
// (min(et_mean, smallest) <= et_mean). Returns 0 on an empty grid (the caller
// treats empty pairs as a non-perf task and sets TL = -1, so the index is never
// read). Contract: timePerformancePairs is sorted ascending by time_limit
// (RecordCloseTimeLimitOptions guarantees this).
size_t FindLargestTimeLimitAtOrBelow(
    const std::vector<TimePerfPair>& time_perf_pairs, double et_mean);

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

    PriorityVec OptimizeIncre_w_TL(const DAG_Model& dag_tasks_update, int beam_search_width);

    PriorityVec ReOptimizePeriodic(const DAG_Model& dag_tasks_update, int beam_search_width);

    // Counter-driven dispatcher: every ReoptimizationPeriod-th call re-runs the
    // TL search from scratch (ReOptimizePeriodic, compare-and-keep), else the
    // warm-started incremental search (OptimizeIncre_w_TL). Both walk the same
    // full option set; they differ in warm-start (inc) vs full re-search (reopt)
    // and patience (0 vs 1). count==0 → ReOptimizePeriodic, which bootstraps the
    // interval-0 incumbent (RM+min-TL). Counter advances every call, never resets.
    PriorityVec Optimize_w_TL_ScratchOrIncre(const DAG_Model& dag_tasks_update,
                                             int beam_search_width);

    // P3.6 INCR_NO_REOPT arm — pure incremental, DM-fast bootstrap, no periodic
    // reopt. count==0 → BootstrapIncumbentFromDMFast (seed the DM+min-TL
    // incumbent, NO from-scratch descent); count>0 → OptimizeIncre_w_TL (warm-
    // started from the carried incumbent). Never calls ReOptimizePeriodic, so
    // the memoryless from-scratch search (the suspected P1.2 structural-corruption
    // vector) never runs — the incumbent evolves only through the 1-D incremental
    // walk. Contrast with INCR_Reopt_X (reopts every Xth interval). Counter
    // advances every call (uniform with Optimize_w_TL_ScratchOrIncre).
    PriorityVec OptimizePureIncremental(const DAG_Model& dag_tasks_update,
                                        int beam_search_width);

    // P3.6 interval-0 bootstrap: seed the incumbent from the DM-fast heuristic
    // (DM priorities via DeadlineMonotonicPriorityVec + smallest TL via
    // SmallestTimeLimitVec), commit it, and STOP — no descent. This is the
    // ResetIncumbentBaseline(true) interval-0 seed step in isolation, without
    // the RunIntervalDescent walk ReOptimizePeriodic runs afterwards.
    void BootstrapIncumbentFromDMFast(const DAG_Model& dag_tasks_update);

    void ApplyWCETAblationIfRequired(DAG_Model& dag_tasks);

    bool UpdateRecords(const OptimizePA_Incre& optimizer,
                       const std::vector<double>& time_limits);

    // The "would this candidate beat the incumbent?" predicate — the exact
    // condition `UpdateRecords` commits on (strictly-greater SP, OR an approx-
    // equal SP tie with a strictly-smaller total TL — the tie-break prefers the
    // tighter budget). Extracted so the P0.6 hard gate (step 4b) can ask "would
    // this commit?" BEFORE deciding to run the feasibility check: per the user's
    // rule the gate fires ONLY on would-beat ("checked whenever we make progress
    // from the champion; if challenger doesn't beat champion, we don't do the
    // check"). Behavior-identical to the prior inline logic in `UpdateRecords`.
    bool WouldBeatIncumbent(double challenger_sp,
                            const std::vector<double>& time_limits) const;

    // Evaluates one TL vector. from_scratch: a fresh OptimizePA_Incre runs
    // OptimizeFromScratch (the bootstrap). Otherwise warm-starts from a throwaway
    // challenger rebuilt from res_opt_ (BuildChallengerFromIncumbent) + OptimizeIncre;
    // requires an incumbent.
    // Virtual so the reopt descent's baseline beam can be unit-tested with a
    // TL→SP stub; the indirection is one call per candidate (negligible vs ObtainSP_DAG).
    virtual double CallOptimizerGivenTimeLimits(
        int beam_search_width, const std::vector<double>& time_limits, bool from_scratch);

    // Serialized-queue SP-eval: runs the incremental optimizer under one TL vector
    // honoring the |diff|<=1 single-change invariant (routes through the RTA cache).
    // `task_idx`: the one task whose ET differs from the champion (Type-E: env-
    // changed task, committed TL; Type-L: walked task, trial TL). `et_increased`:
    // caller-supplied ET direction vs the champion — drives half-range pruning in
    // the primitive (wrong direction prunes the wrong half), so the caller MUST
    // supply it. No beam: the primitive re-searches one task's 1D positions.
    // Virtual so the serialized walk can be unit-tested with a TL→SP stub.
    virtual double OptimizeIncreSingleTask(
        const std::vector<double>& time_limits, size_t task_idx,
        bool et_increased);

    // Assert the single-change invariant on the serialized path (debugMode-only).
    // The diff must be EMPTY (Type-E: env move absorbed into dag_tasks_ on both
    // sides → cancels) or flag exactly `task_idx` (Type-L: the walked task's TL
    // moved). |diff|>1 or a 1-flagging-different-task means the champion drifted
    // → throws. No-op when debugMode is off.
    void AssertSingleChangeInvariant(const DAG_Model& champion_dag,
                                     const DAG_Model& candidate_dag,
                                     size_t task_idx) const;

    // Absorb the interval's updated DAG into the optimizer state: overwrite
    // dag_tasks_, apply the WCET ablation if the arm requires it, and refresh the
    // per-task TL option set the descent walks. Shared by all three interval-entry
    // methods (OptimizeIncre_w_TL, ReOptimizePeriodic, BootstrapIncumbentFromDMFast)
    // so the absorb + WCET ablation + option-set refresh stay in sync. Callers that
    // need the pre-absorb DAG for the Type-E diff capture it BEFORE calling this.
    void AbsorbUpdatedDAG(const DAG_Model& dag_tasks_update);

    std::vector<double> InitializeTimeLimitsFromETConfig();
    // P0.6 static-solution TL seed: per-task, the largest TL grid option <=
    // et_mean (execution_time_dist.GetAvgValue()) for perf tasks; -1 for non-perf
    // tasks (no timePerformancePairs). The directional counterpart to
    // InitializeTimeLimitsFromETConfig (which uses the bidirectional
    // Find_Close_ExecutionTime and may pick above et_mean). Seeds at-or-below the
    // P0.8-certified WCET (= et_mean) so the seed is feasible-by-construction.
    std::vector<double> SeedTimeLimitsAtOrBelowEtMean() const;
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

    // The ONE interval-descent body shared by the incremental and reopt paths.
    // Patience is mode-selected (inc=IncrementalTimeLimitSearchPatience,
    // reopt=ReoptimizationTimeLimitSearchPatience); the baseline-seed preamble +
    // cache arming live in SeedBaselineAndArmCache(mode); the tail
    // (BuildSerializedTaskQueue + WalkSerializedTaskQueue + unconditional cache
    // disarm) is mode-independent. `dag_tasks_prev_pre_tl` is the pre-absorb DAG
    // (Type-E diff source for the serialized queue).
    void RunIntervalDescent(int beam_search_width, std::vector<double>& starting_time_limits,
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
    double SeedBaselineAndArmCache(int beam_search_width,
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
    double WalkOneTaskWithTimeLimitOptions(
        size_t task_idx, std::vector<double>& time_limits, double current_sp,
        double baseline_val, int step, int patience,
        std::function<double(const std::vector<double>&)> eval);

    // One task's sub-incremental TL walk: builds the sub-incremental eval lambda
    // (binding OptimizeIncreSingleTask) and runs the backward (step=-1) then
    // forward (step=+1) passes over WalkOneTaskWithTimeLimitOptions, finally
    // syncing the working TL vector to the adopted champion. Shared by the
    // incremental serialized queue (PerformSerializedTaskQueueOptimization Type-L
    // body) and the reopt descent (RunIntervalDescent(Reopt)) — both ran this
    // exact block inline before P2.11 Phase 1, differing only in the task_idx
    // source.
    double OptimizeOneTaskWithTimeLimit(
        size_t task_idx, std::vector<double>& starting_time_limits,
        double current_config_sp, double baseline_val, int patience);

    // Fast path when disable_time_limit_opt is set: pin every TL to its smallest
    // option and evaluate that single config (no descent).
    PriorityVec OptimizeWithTimeLimitOptDisabled(
        int beam_search_width, std::vector<double>& time_limits, bool from_scratch);

    // Type-L set: task IDs whose option set is NOT the {-1}-only sentinel (pure query).
    std::vector<int> CollectTLFlexibleTaskIds() const;

    // Merged E+L queue: Type-E (FindEnvTaskWithDifferentEt) + Type-L
    // (CollectTLFlexibleTaskIds), sorted together by task weight descending.
    // `prev_pre_tl` is the local pre-TL DAG captured before the absorb.
    // Disjoint by generator design (TL-flexible tasks have no env dependence);
    // a task in BOTH → CoutError (contract violation, not a silent pick).
    std::vector<SerializedTaskQueueEntry> BuildSerializedTaskQueue(
        const DAG_Model& dag_tasks_prev_pre_tl) const;

    // Walks the merged E+L serialized queue in place. Type-E → OptimizeIncreSingleTask
    // re-search at the committed TL (no TL walk); Type-L → OptimizeOneTaskWithTimeLimit
    // TL walk. Each step adopts into res_opt_, so the next step's challenger sees
    // the new champion and the diff flags only the walked task (|diff|<=1).
    // Shared by PerformSerializedTaskQueueOptimization (incremental) and
    // RunIntervalDescent(Reopt) — both ran this exact loop inline before P2.11
    // Phase 1. Returns the final SP; syncs starting_time_limits to the adopted
    // champion.
    double WalkSerializedTaskQueue(
        const std::vector<SerializedTaskQueueEntry>& queue,
        std::vector<double>& starting_time_limits, double current_config_sp,
        int patience);

    // Serialized loop driver — the incremental-path entry (the reopt path is
    // RunIntervalDescent(Reopt)). Resets the baseline, re-scores the champion
    // under the new env (dedicated re-score, NOT CallOptimizerGivenTimeLimits —
    // must not optimize before the queue's order is honored), builds the E+L
    // queue, and walks it serially. `dag_tasks_prev_pre_tl` is the pre-TL DAG
    // captured before the absorb (Type-E diff source).
    // Virtual so the path can be unit-tested with an observing stub.
    virtual void PerformSerializedTaskQueueOptimization(
        int beam_search_width, std::vector<double>& starting_time_limits,
        const DAG_Model& dag_tasks_prev_pre_tl);

    // Compare-and-keep helpers (see ReOptimizePeriodic).
    std::vector<double> ReconstructTimeLimitVecFromResOpt();
    // Deadline-Monotonic + important-first group-locked priority vector (P0.9).
    // Important tasks occupy the top slots (DM-ordered within the group — shorter
    // deadline = higher priority); non-important fill the lower slots (DM-ordered
    // within their group); every non-important task below every important task
    // (the priority lock). Matches the Python RTA's
    // ``_important_priority_order`` so P0.8's certification agrees with the
    // scheduler's actual seed PA. Ties (equal deadline within a group) broken by
    // avg ET ascending (deterministic). Replaces the former plain-RM
    // ``RateMonotonicPriorityVec`` (period sort) — constrained deadlines
    // (``deadline = period * U(0.5,1.0)``) make DM optimal.
    PriorityVec DeadlineMonotonicPriorityVec();
    void SeedStateFromIncumbent(const DAG_Model& dag_with_tl,
                                const PriorityVec& pa, double sp,
                                const std::vector<double>& tl);
    // Reset the incumbent baseline before a new interval. from_scratch (reopt):
    // re-eval the carried {pa, tl} under the new DAG (or DM+min-TL at interval 0)
    // and commit it, so opt_sp_ holds the baseline for compare-and-keep.
    // !from_scratch (incremental): set opt_sp_=-1.0 so the first UpdateRecords
    // force-commits.
    void ResetIncumbentBaseline(bool from_scratch);

    // Interval-0 DM-fast seed: DM priorities (DeadlineMonotonicPriorityVec) + every
    // task at its smallest TL option (SmallestTimeLimitVec), scored under the
    // current dag_tasks_ and committed as the incumbent. The shared bootstrap
    // used by ResetIncumbentBaseline's interval-0 else-branch (reopt's first
    // interval, reached via SeedBaselineAndArmCache) AND BootstrapIncumbentFromDMFast
    // (the INCR_NO_REOPT arm's interval-0 entry) — the two arms bootstrap
    // identically through this one primitive. Caller owns the cache state:
    // ResetIncumbentBaseline clears it first; BootstrapIncumbentFromDMFast runs
    // only at interval 0 on a fresh optimizer (nothing to clear).
    void SeedIncumbentFromDMFast();

    // P0.6 — the offline safe-fallback artifact. Seeds at the P0.8-certified point
    // (DM PA + TL <= et_mean), runs a gate-governed TL walk on a throwaway sibling, and
    // keeps the best-SP gate-feasible result. Sibling isolation → online byte-identical.
    ResourceOptResult ComputeSafeFallback();
    bool HasSafeFallback() const { return safe_fallback_.has_value(); }
    const ResourceOptResult& GetSafeFallback() const { return *safe_fallback_; }

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
    // P0.6 — the offline safe-fallback artifact. Separate from res_opt_ so the live
    // incumbent stays untouched (byte-identical online). Consumed by P0.7's fall-back.
    std::optional<ResourceOptResult> safe_fallback_;
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
    // P0.6 — hard per-candidate feasibility gate. When true, `UpdateRecords` adds an
    // extra acceptance test on a would-beat: the candidate commits only if
    // `ImportantTasksMeetThresholds` passes. Default false → prod byte-identical;
    // set true only inside `ComputeSafeFallback`.
    bool enforce_important_task_gate_ = false;
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
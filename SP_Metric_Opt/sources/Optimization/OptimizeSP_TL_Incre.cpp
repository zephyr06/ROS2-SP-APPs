
#include "sources/Optimization/OptimizeSP_TL_Incre.h"

#include <algorithm>
#include <numeric>

namespace SP_OPT_PA {

bool TaskSortingHeuristic::operator()(size_t idx1, size_t idx2) const {
    const auto& t1 = dag_tasks.tasks[idx1];
    const auto& t2 = dag_tasks.tasks[idx2];

    double w1 = 1.0;
    if (sp_parameters.weights_node.count(t1.id)) {
        w1 = sp_parameters.weights_node.at(t1.id);
    }
    double w2 = 1.0;
    if (sp_parameters.weights_node.count(t2.id)) {
        w2 = sp_parameters.weights_node.at(t2.id);
    }

    if (w1 != w2) {
        return w1 > w2;
    }

    double th1 = 0.5;
    if (sp_parameters.thresholds_node.count(t1.id)) {
        th1 = sp_parameters.thresholds_node.at(t1.id);
    }
    double th2 = 0.5;
    if (sp_parameters.thresholds_node.count(t2.id)) {
        th2 = sp_parameters.thresholds_node.at(t2.id);
    }

    if (th1 != th2) {
        return th1 < th2;
    }

    return t1.id < t2.id;
}

size_t Find_Close_ExecutionTime(
    const std::vector<TimePerfPair>& time_perf_pairs, double time_limit) {
    if (time_perf_pairs.size() == 0)
        return -1;
    size_t min_diff_index = 0;
    double min_diff = std::abs(time_perf_pairs[0].time_limit - time_limit);
    for (size_t i = 0; i < time_perf_pairs.size(); i++) {
        double diff = std::abs(time_perf_pairs[i].time_limit - time_limit);
        if (diff < min_diff) {
            min_diff = diff;
            min_diff_index = i;
        }
    }
    return min_diff_index;
}

std::vector<std::vector<double>> RecordCloseTimeLimitOptions(
    const DAG_Model& dag_tasks, int radius) {
    std::vector<std::vector<double>> time_limit_option_for_each_task;
    time_limit_option_for_each_task.reserve(dag_tasks.tasks.size());
    for (uint i = 0; i < dag_tasks.tasks.size(); i++) {
        time_limit_option_for_each_task.push_back({});
        size_t close_time_limit_index = Find_Close_ExecutionTime(
            dag_tasks.tasks[i].timePerformancePairs,
            dag_tasks.tasks[i].execution_time_dist.GetAvgValue());
        for (int j = max(0, static_cast<int>(close_time_limit_index) - radius);
             j <= static_cast<int>(close_time_limit_index) + radius &&
             j < static_cast<int>(
                     dag_tasks.tasks[i].timePerformancePairs.size());
             j++) {
            time_limit_option_for_each_task[i].push_back(
                dag_tasks.tasks[i].timePerformancePairs[j].time_limit);
        }
        if (dag_tasks.tasks[i].timePerformancePairs.size() == 0) {
            time_limit_option_for_each_task[i].push_back(-1);
        }
    }
    return time_limit_option_for_each_task;
}

size_t FindTimeLimitOptionIndex(const std::vector<double>& options,
                                double current_val) {
    auto it = std::find(options.begin(), options.end(), current_val);
    if (it == options.end()) {
        return options.size();
    }
    return static_cast<size_t>(std::distance(options.begin(), it));
}

bool IsBetterTimeLimitOption(double new_sp, double current_best_sp, int step) {
    if (new_sp > current_best_sp && !ApproxEqualSP(new_sp, current_best_sp)) {
        return true;
    }
    // Ties (approx-equal SP) prefer the smaller time limit. Walking downward
    // (step < 0) reaches a smaller TL next, so a downward tie is an
    // improvement; walking upward (step > 0) reaches a larger TL, so an upward
    // tie is rejected to keep the tighter TL already held.
    if (ApproxEqualSP(new_sp, current_best_sp) && step < 0) {
        return true;
    }
    return false;
}

void OptimizePA_Incre_with_TimeLimits::UpdateRecords(
    const OptimizePA_Incre& optimizer, const std::vector<double>& time_limits) {
    bool should_update = false;
    if (optimizer.opt_sp_ > opt_sp_ &&
        !ApproxEqualSP(optimizer.opt_sp_, opt_sp_)) {
        should_update = true;
    } else if (ApproxEqualSP(optimizer.opt_sp_, opt_sp_)) {
        double sum_new = 0;
        for (double val : time_limits) {
            if (val != -1.0)
                sum_new += val;
        }
        double sum_old = 0;
        for (auto const& [id, val] : res_opt_.id2time_limit) {
            if (val != -1.0)
                sum_old += val;
        }
        if (sum_new < sum_old) {
            should_update = true;
        }
    }

    if (should_update) {
        // CommitIncumbent owns res_opt_ + the opt_pa_/opt_sp_ mirrors. The
        // challenger (the `optimizer` arg) is a throwaway; only its adopted
        // {pa, sp, tl} survives across intervals.
        CommitIncumbent(optimizer.opt_pa_, optimizer.opt_sp_, time_limits);

        if (GlobalVariables::debugMode) {
            std::cout << "Time limit: \n";
            for (double time : time_limits) std::cout << time << " ";
            std::cout << "TraverseTimeLimitOptions: "
                      << "opt_sp_ = " << opt_sp_ << std::endl;
        }
    }
}

double OptimizePA_Incre_with_TimeLimits::EvaluateTimeLimitConfig_ScratchOrIncre(
    int K, const std::vector<double>& time_limits, bool from_scratch) {
    eval_count_++;
    DAG_Model dag_tasks_cur =
        UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits);

    double current_sp = -1.0;
    if (from_scratch) {
        // Reopt: ignore warm state, re-search the full beam for the current TL.
        OptimizePA_Incre optimizer(dag_tasks_cur, sp_parameters_);
        optimizer.OptimizeFromScratch(K);
        current_sp = optimizer.opt_sp_;
        UpdateRecords(optimizer, time_limits);
    } else if (IfInitialized()) {
        // Incremental: rebuild a throwaway challenger from res_opt_ (the champion)
        // each candidate, then OptimizeIncre. See BuildChallengerFromIncumbent —
        // this guarantees only the walked task's ET differs, the perfect case for
        // OptimizeIncre's diff-driven 1D re-search.
        OptimizePA_Incre optimizer = BuildChallengerFromIncumbent();
        optimizer.OptimizeIncre(dag_tasks_cur);
        current_sp = optimizer.opt_sp_;
        UpdateRecords(optimizer, time_limits);
    } else {
        // Contract violation: the incremental path needs an incumbent to
        // warm-start from, but none exists yet. Bootstrap with a from_scratch
        // call (e.g. ReOptimizePeriodic) first.
        CoutError(
            "EvaluateTimeLimitConfig_ScratchOrIncre: incremental path "
            "(from_scratch=false) requested but no incumbent is initialized. "
            "Bootstrap with a from_scratch call first "
            "(e.g. ReOptimizePeriodic).");
    }
    return current_sp;
}

std::vector<double>
OptimizePA_Incre_with_TimeLimits::InitializeTimeLimitsFromETConfig() {
    std::vector<double> time_limits(dag_tasks_.tasks.size());
    for (size_t i = 0; i < dag_tasks_.tasks.size(); i++) {
        if (dag_tasks_.tasks[i].timePerformancePairs.empty()) {
            time_limits[i] = -1.0;
        } else {
            size_t close_idx = Find_Close_ExecutionTime(
                dag_tasks_.tasks[i].timePerformancePairs,
                dag_tasks_.tasks[i].execution_time_dist.GetAvgValue());
            time_limits[i] =
                dag_tasks_.tasks[i].timePerformancePairs[close_idx].time_limit;
        }
    }
    return time_limits;
}

double OptimizePA_Incre_with_TimeLimits::OptimizeSingleTaskTimeLimit(
    size_t task_idx, int K, std::vector<double>& time_limits,
    double current_sp, double baseline_val, int step, bool from_scratch,
    int patience) {
    const std::vector<double>& opts = time_limit_option_for_each_task_[task_idx];
    // No TL freedom (the {-1}-only sentinel from RecordTimeLimitOptions):
    // nothing to walk. Return the carried SP unchanged.
    if (opts.size() == 1 && opts[0] == -1.0) {
        return current_sp;
    }

    size_t curr_opt_idx = FindTimeLimitOptionIndex(opts, baseline_val);
    // Stale baseline (not a member of the option set): no valid start index, so
    // the walk cannot run. Return the carried SP unchanged.
    if (curr_opt_idx == opts.size()) {
        return current_sp;
    }

    double best_sp = current_sp;
    double best_option_val = time_limits[task_idx];

    // Walk sequentially in direction `step`, stopping at the option-set boundary.
    // `patience` is a total non-improvement budget: each non-improving step spends
    // one unit (no reset on improvement); when it hits 0 the walk stops.
    for (int i = static_cast<int>(curr_opt_idx) + step;
         i >= 0 && i < static_cast<int>(opts.size()); i += step) {
        double val = opts[i];
        if (val == -1.0)  // defensive: a {-1} slot inside a real window
            continue;
        time_limits[task_idx] = val;
        double sp_val = EvaluateTimeLimitConfig_ScratchOrIncre(
            K, time_limits, from_scratch);

        if (IsBetterTimeLimitOption(sp_val, best_sp, step)) {
            best_sp = sp_val;
            best_option_val = val;
        } else if (patience == 0) {
            break;  // budget exhausted: stop the walk in this direction
        } else {
            --patience;  // spend one unit, keep stepping outward past the dip
        }
    }

    time_limits[task_idx] = best_option_val;
    return best_sp;
}

void OptimizePA_Incre_with_TimeLimits::PerformCoordinateDescentForTaskConfigOpt(
    int K, std::vector<double>& starting_time_limits, bool from_scratch) {
    std::vector<size_t> sorted_indices(dag_tasks_.tasks.size());
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(sorted_indices.begin(), sorted_indices.end(),
              TaskSortingHeuristic{dag_tasks_, sp_parameters_});

    // Patience: consecutive non-improving steps the walk tolerates. Incremental
    // (warm-started, SP-vs-TL ~unimodal) uses 0; reopt (full re-search, can be
    // non-unimodal) uses 1. Sourced from YAML globals; see parameters.yaml.
    int patience = from_scratch
                        ? GlobalVariables::ReoptimizationTimeLimitSearchPatience
                        : GlobalVariables::IncrementalTimeLimitSearchPatience;

    // Reset the incumbent baseline for this interval (reopt: re-eval+commit;
    // incremental: opt_sp_=-1.0 gate) so the baseline eval below measures and
    // commits against the correct current-interval baseline, not a stale prior.
    ResetIncumbentBaseline(from_scratch);
    double current_config_sp =
        EvaluateTimeLimitConfig_ScratchOrIncre(K, starting_time_limits,
                                               from_scratch);

    for (size_t idx : sorted_indices) {
        // Skip {-1}-only tasks (no timePerformancePairs → no TL freedom).
        const std::vector<double>& opts = time_limit_option_for_each_task_[idx];
        if (opts.size() == 1 && opts[0] == -1.0)
            continue;

        double baseline_val = starting_time_limits[idx];
        // 1. Backward pass: try decreasing the TL (tie-break toward smaller TL
        //    on SP ties — handled inside IsBetterTimeLimitOption via step<0).
        current_config_sp = OptimizeSingleTaskTimeLimit(
            idx, K, starting_time_limits, current_config_sp, baseline_val,
            /*step=*/-1, from_scratch, patience);
        // 2. Forward pass from the ORIGINAL starting TL (not the backward
        //    result), exploring the upward side from the same origin.
        current_config_sp = OptimizeSingleTaskTimeLimit(
            idx, K, starting_time_limits, current_config_sp, baseline_val,
            /*step=*/1, from_scratch, patience);
    }
}

PriorityVec OptimizePA_Incre_with_TimeLimits::Optimize_w_TL_ScratchOrIncre(
    const DAG_Model& dag_tasks_update, int K) {
    // Modular reopt: every ReoptimizationPeriod-th call (count % period == 0)
    // takes the from-scratch compare-and-keep path; otherwise the warm-started
    // incremental path. count == 0 routes to ReOptimizePeriodic, which
    // bootstraps the incumbent at interval 0 (the incremental path cannot run
    // without an incumbent). The counter advances every call and never resets.
    int period = GlobalVariables::ReoptimizationPeriod;
    bool trigger_reopt = (reoptimization_interval_count_ % period == 0);
    if (trigger_reopt) {
        ReOptimizePeriodic(dag_tasks_update, K);
    } else {
        OptimizeIncre_w_TL(dag_tasks_update, K);
    }
    reoptimization_interval_count_++;
    return opt_pa_;
}

PriorityVec OptimizePA_Incre_with_TimeLimits::OptimizeWithTimeLimitOptDisabled(
    int K, std::vector<double>& time_limits, bool from_scratch) {
    // This path bypasses the descent, so the interval reset the descent would
    // have done must run here too, else res_opt_ stays stale for this interval.
    ResetIncumbentBaseline(from_scratch);
    InitializeTimeLimitsToSmallest(time_limits);
    EvaluateTimeLimitConfig_ScratchOrIncre(K, time_limits, from_scratch);
    return opt_pa_;
}

PriorityVec OptimizePA_Incre_with_TimeLimits::OptimizeIncre_w_TL(
    const DAG_Model& dag_tasks_update, int K) {
    dag_tasks_ = dag_tasks_update;
    ApplyWCETAblationIfRequired(dag_tasks_);
    // Full per-task option set (every timePerformancePairs entry). The walk
    // steps over this set and stops on patience-bounded non-improvement — no
    // radius cap, so a tie-break or strictly-better option beyond the old
    // radius wall is reachable.
    time_limit_option_for_each_task_ = RecordTimeLimitOptions(dag_tasks_);
    // Start the descent from the CARRIED ADOPTED TL in res_opt_, NOT the
    // Gaussian-mean TL: both sides of FindTaskWithDifferentEt's diff must carry
    // the adopted TL, else unchanged perf-pair tasks are flagged as changed
    // (the P1.1 residual).
    std::vector<double> time_limits = ReconstructTimeLimitVecFromResOpt();
    if (GlobalVariables::disable_time_limit_opt) {
        return OptimizeWithTimeLimitOptDisabled(K, time_limits,
                                                /*from_scratch=*/false);
    }
    PerformCoordinateDescentForTaskConfigOpt(K, time_limits,
                                             /*from_scratch=*/false);
    return opt_pa_;
}

// Reconstruct the positional time-limit vector (one entry per task, in task
// order) from the recorded id→TL map. Tasks with no recorded TL get -1.
std::vector<double>
OptimizePA_Incre_with_TimeLimits::ReconstructTimeLimitVecFromResOpt() {
    std::vector<double> tl(dag_tasks_.tasks.size());
    for (size_t i = 0; i < dag_tasks_.tasks.size(); i++) {
        int id = dag_tasks_.tasks[i].id;
        tl[i] = res_opt_.id2time_limit.count(id) ? res_opt_.id2time_limit.at(id)
                                                 : -1.0;
    }
    return tl;
}

// Rate-monotonic priority vector: tasks sorted by period ascending, smallest
// period = highest priority (index 0). Ties broken by average execution time
// ascending (lower ET = higher priority) so the order is deterministic for
// tasks that share a period. Mirrors the orchestrator's RM mode.
PriorityVec OptimizePA_Incre_with_TimeLimits::RateMonotonicPriorityVec() {
    std::vector<int> sorted(dag_tasks_.tasks.size());
    std::iota(sorted.begin(), sorted.end(), 0);
    std::sort(sorted.begin(), sorted.end(), [&](int a, int b) {
        const Task& ta = dag_tasks_.tasks[a];
        const Task& tb = dag_tasks_.tasks[b];
        if (ta.period != tb.period)
            return ta.period < tb.period;
        return ta.execution_time_dist.GetAvgValue() <
               tb.execution_time_dist.GetAvgValue();
    });
    return sorted;
}

// Seed the incumbent 4-tuple {dag, sp, pa, tl} so opt_sp_ holds the baseline
// UpdateRecords' compare guard measures the search against.
void OptimizePA_Incre_with_TimeLimits::SeedStateFromIncumbent(
    const DAG_Model& dag_with_tl, const PriorityVec& pa, double sp,
    const std::vector<double>& tl) {
    // CommitIncumbent is the single writer for the durable incumbent store.
    // dag_with_tl itself is not stored — the challenger is rebuilt from
    // res_opt_.id2time_limit next interval.
    CommitIncumbent(pa, sp, tl);
}

// The ONE writer for the incumbent state: res_opt_ (durable) + the thin
// opt_pa_/opt_sp_ mirrors. Centralizing the writes here makes the
// sp_parameters_ / DAG desync class of bug structurally impossible.
void OptimizePA_Incre_with_TimeLimits::CommitIncumbent(
    const PriorityVec& pa, double sp, const std::vector<double>& tl) {
    opt_sp_ = sp;
    opt_pa_ = pa;
    res_opt_.SaveTimeLimits(dag_tasks_.tasks, tl);
    res_opt_.UpdatePriorityVec(opt_pa_);
    res_opt_.sp_opt = opt_sp_;
}

// Throwaway challenger rebuilt from res_opt_ (the champion) each candidate, not a
// persistent one. The champion TL tracks the working TL (UpdateRecords commits
// every adoption; the walk resets to the adopted best on no-improvement), so
// while one task is walked the diff flags ONLY that task → OptimizeIncre
// re-searches just its 1D priority variations. Perfect for incremental opt; a
// persistent challenger would drift to non-adopted candidates and flag extras.
OptimizePA_Incre OptimizePA_Incre_with_TimeLimits::BuildChallengerFromIncumbent() {
    std::vector<double> tl_prev = ReconstructTimeLimitVecFromResOpt();
    DAG_Model dag_with_tl_prev =
        UpdateExtDistBasedOnTimeLimit(dag_tasks_, tl_prev);
    OptimizePA_Incre challenger(dag_with_tl_prev, sp_parameters_);
    challenger.opt_pa_ = res_opt_.priority_vec;
    challenger.opt_sp_ = res_opt_.sp_opt;
    return challenger;
}

// Reset the incumbent baseline for the current interval, run BEFORE the
// descent's baseline eval. Branched on from_scratch:
//  - true  (reopt): the carried {pa, tl} is re-evaluated under the NEW DAG and
//    committed, so opt_sp_ holds the baseline the search compare-and-keeps
//    against (NOT -1.0). At interval 0 (no incumbent) a synthetic RM+min-TL
//    baseline is built and committed instead.
//  - false (incremental): only the opt_sp_=-1.0 gate is set. The baseline eval
//    then rebuilds the challenger from res_opt_, re-evals it, and
//    UpdateRecords force-commits (optimizer.opt_sp_ > -1.0 always) so res_opt_
//    is overwritten for the current interval. res_opt_ itself is NOT touched
//    here — the challenger is built from the carried prior, re-evaluated, then
//    committed (read prior → re-eval → commit).
void OptimizePA_Incre_with_TimeLimits::ResetIncumbentBaseline(
    bool from_scratch) {
    if (from_scratch) {
        if (IfInitialized()) {
            std::vector<double> tl_prev = ReconstructTimeLimitVecFromResOpt();
            PriorityVec pa_prev = opt_pa_;
            DAG_Model dag_new_with_tl_prev =
                UpdateExtDistBasedOnTimeLimit(dag_tasks_, tl_prev);
            double sp_prev_new = EvaluateSPWithPriorityVec(dag_new_with_tl_prev,
                                                           sp_parameters_,
                                                           pa_prev);
            SeedStateFromIncumbent(dag_new_with_tl_prev, pa_prev, sp_prev_new,
                                   tl_prev);
        } else {
            // Interval 0: RM priorities + every task at its smallest TL option.
            std::vector<double> tl_min = SmallestTimeLimitVec();
            PriorityVec pa_rm = RateMonotonicPriorityVec();
            DAG_Model dag_with_tl_min =
                UpdateExtDistBasedOnTimeLimit(dag_tasks_, tl_min);
            double sp_rm =
                EvaluateSPWithPriorityVec(dag_with_tl_min, sp_parameters_,
                                          pa_rm);
            SeedStateFromIncumbent(dag_with_tl_min, pa_rm, sp_rm, tl_min);
        }
    } else {
        opt_sp_ = -1.0;
    }
}

PriorityVec OptimizePA_Incre_with_TimeLimits::ReOptimizePeriodic(
    const DAG_Model& dag_tasks_update, int K) {
    // Compare-and-keep reopt: the incumbent in res_opt_ is re-evaluated under
    // the new DAG and seeded as the baseline, then a fresh from-scratch descent
    // runs. UpdateRecords' strictly-greater-SP guard (tie-break lower TL-sum)
    // preserves the incumbent when the search finds nothing better.

    dag_tasks_ = dag_tasks_update;
    ApplyWCETAblationIfRequired(dag_tasks_);
    // Full per-task option set — see OptimizeIncre_w_TL for why the walk is no
    // longer radius-capped.
    time_limit_option_for_each_task_ = RecordTimeLimitOptions(dag_tasks_);

    // The baseline reset (re-eval carried {pa, tl} under the new DAG, or
    // RM+min-TL at interval 0) now runs inside the descent via
    // ResetIncumbentBaseline(true), before the baseline eval.
    // Descent start (P1.4): seed from the carried adopted TL — the optimizer's
    // own prior result in res_opt_ — whenever an incumbent exists. This is the
    // permanent, unconditional reopt seed policy: the seed must be
    // algorithm-derived (the optimizer's own prior output), not read from the
    // YAML taskset characterization. The IfInitialized() gate auto-falls-back
    // to the Gaussian-mean TL (InitializeTimeLimitsFromETConfig) at interval 0
    // (no incumbent → ReconstructTimeLimitVecFromResOpt would be all -1 =
    // no-op); that fallback is irreducible for any seed policy — the very first
    // solve has no prior optimizer state to seed from.
    std::vector<double> time_limits =
        IfInitialized() ? ReconstructTimeLimitVecFromResOpt()
                        : InitializeTimeLimitsFromETConfig();
    if (GlobalVariables::disable_time_limit_opt) {
        OptimizeWithTimeLimitOptDisabled(K, time_limits, /*from_scratch=*/true);
    } else {
        PerformCoordinateDescentForTaskConfigOpt(K, time_limits,
                                                 /*from_scratch=*/true);
    }
    return opt_pa_;
}

void OptimizePA_Incre_with_TimeLimits::ApplyWCETAblationIfRequired(
    DAG_Model& dag_tasks) {
    if (GlobalVariables::use_wcet_execution_time) {
        for (auto& task : dag_tasks.tasks) {
            double max_et = task.execution_time_dist.max_time;
            task.execution_time_dist = GetUnitExecutionTimeDist(max_et);
            task.setExecGaussian(GaussianDist(max_et, 0.01));
            task.setExecutionTime(max_et);
        }
    }
}

std::vector<double> OptimizePA_Incre_with_TimeLimits::SmallestTimeLimitVec()
    const {
    std::vector<double> time_limits(dag_tasks_.tasks.size());
    for (size_t i = 0; i < dag_tasks_.tasks.size(); i++) {
        time_limits[i] =
            dag_tasks_.tasks[i].timePerformancePairs.empty()
                ? -1.0
                : dag_tasks_.tasks[i].timePerformancePairs[0].time_limit;
    }
    return time_limits;
}

void OptimizePA_Incre_with_TimeLimits::InitializeTimeLimitsToSmallest(
    std::vector<double>& time_limits) {
    std::vector<double> smallest = SmallestTimeLimitVec();
    for (size_t i = 0; i < time_limits.size(); i++) {
        time_limits[i] = smallest[i];
    }
}

}  // namespace SP_OPT_PA
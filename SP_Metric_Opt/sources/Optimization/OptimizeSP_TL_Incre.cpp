
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
        opt_sp_ = optimizer.opt_sp_;
        opt_pa_ = optimizer.opt_pa_;

        res_opt_.SaveTimeLimits(dag_tasks_.tasks, time_limits);
        res_opt_.UpdatePriorityVec(opt_pa_);
        res_opt_.sp_opt = opt_sp_;
        prev_optimizer_ = optimizer;

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
        // Reoptimization path: ignore any warm state and re-search from
        // scratch. This escapes PA drift by exploring the full beam for the
        // current TL.
        OptimizePA_Incre optimizer(dag_tasks_cur, sp_parameters_);
        optimizer.OptimizeFromScratch(K);
        current_sp = optimizer.opt_sp_;
        UpdateRecords(optimizer, time_limits);
    } else if (prev_optimizer_.IfInitialized()) {
        // Incremental path: warm-start from the incumbent and diff-search.
        OptimizePA_Incre optimizer = prev_optimizer_;
        optimizer.OptimizeIncre(dag_tasks_cur);
        current_sp = optimizer.opt_sp_;
        UpdateRecords(optimizer, time_limits);
    } else {
        // Contract violation: the incremental path (from_scratch=false) needs
        // an incumbent to warm-start from, but prev_optimizer_ is
        // uninitialized. Under the incremental-scheduler contract the
        // from-scratch bootstrap is scheduler-driven — from_scratch is called
        // at interval 0 (and, in future, periodically to escape drift) — so
        // OptimizeIncre_w_TL is never the bootstrap. Reaching here means a
        // caller invoked the incremental path before any from_scratch call
        // established an incumbent. Bootstrap with a from_scratch call (e.g.
        // ReOptimizePeriodic) first.
        CoutError(
            "EvaluateTimeLimitConfig_ScratchOrIncre: incremental path "
            "(from_scratch=false) requested but prev_optimizer_ is "
            "uninitialized. Bootstrap with a from_scratch call first "
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
    int consecutive_non_improving = 0;

    // Walk sequentially in direction `step`, stopping at the option-set boundary.
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
            consecutive_non_improving = 0;  // improvement resets the budget
        } else {
            ++consecutive_non_improving;
            if (consecutive_non_improving > patience) {
                break;  // budget exhausted: stop the walk in this direction
            }
            // Else: within budget, keep stepping outward to look past the dip.
        }
    }

    time_limits[task_idx] = best_option_val;
    return best_sp;
}

void OptimizePA_Incre_with_TimeLimits::PerformCoordinateDescentForTaskConfigOpt(
    int K, std::vector<double>& time_limits, bool from_scratch) {
    std::vector<size_t> sorted_indices(dag_tasks_.tasks.size());
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(sorted_indices.begin(), sorted_indices.end(),
              TaskSortingHeuristic{dag_tasks_, sp_parameters_});

    // Patience governs how many consecutive non-improving steps the
    // trial-and-error walk tolerates before stopping, sourced from the
    // YAML-loaded globals so it is tunable per-experiment without recompiling.
    // The incremental path warm-starts the PA search from the incumbent, so
    // SP-vs-TL is effectively unimodal and strict (patience=0) is safe and
    // cheapest. The reopt path re-searches priorities from scratch per
    // candidate, so SP-vs-TL can be non-unimodal at high utilization;
    // patience=1 tolerates a single dip so a strictly-better option further
    // out is not missed. See sources/parameters.yaml for tuning guidance.
    int patience = from_scratch
                        ? GlobalVariables::ReoptimizationTimeLimitSearchPatience
                        : GlobalVariables::IncrementalTimeLimitSearchPatience;

    // Establish the baseline SP of the starting configuration. This is the
    // best-yet the first task's backward pass measures against, and it doubles
    // as the compare-and-keep baseline already seeded by SeedIncumbentBaseline
    // (UpdateRecords adopts only strictly-better candidates). Re-evaluating the
    // starting config here is intentional: it gives the walk a concrete SP for
    // the EXACT starting TL vector, which SeedIncumbentBaseline may have
    // computed under a different {pa, tl} (the seeded incumbent's TL, not the
    // descent's InitializeTimeLimitsFromETConfig starting point).
    double current_config_sp =
        EvaluateTimeLimitConfig_ScratchOrIncre(K, time_limits, from_scratch);
    bool any_eval_ran = true;  // the baseline eval above counts

    for (size_t idx : sorted_indices) {
        // Skip a task whose only TL option is -1 (no timePerformancePairs →
        // RecordTimeLimitOptions recorded {-1}). There is no TL freedom
        // to search, and OptimizeIncre already does the PA search internally,
        // so re-evaluating here is a redundant incumbent re-eval. Skipping it
        // is the main cost win on the reused P25 tasksets (every task is
        // {-1}-only). opts.size()==1 && opts[0]==-1 is the exact no-pairs
        // predicate: RecordTimeLimitOptions only pushes -1 when a task
        // has zero pairs, so a task with real options is never skipped.
        const std::vector<double>& opts = time_limit_option_for_each_task_[idx];
        if (opts.size() == 1 && opts[0] == -1.0)
            continue;

        double baseline_val = time_limits[idx];
        // 1. Backward pass: try decreasing the time limit (tie-break toward
        //    smaller TL on SP ties — handled inside IsBetterTimeLimitOption
        //    via step<0).
        current_config_sp = OptimizeSingleTaskTimeLimit(
            idx, K, time_limits, current_config_sp, baseline_val,
            /*step=*/-1, from_scratch, patience);
        // 2. Forward pass: try increasing the time limit. baseline_val is the
        //    ORIGINAL starting TL (not the backward pass's result), so the
        //    forward pass explores the upward side from the same origin.
        current_config_sp = OptimizeSingleTaskTimeLimit(
            idx, K, time_limits, current_config_sp, baseline_val,
            /*step=*/1, from_scratch, patience);
        // OptimizeSingleTaskTimeLimit always runs >=1 eval when the task has
        // real options, so any_eval_ran stays true.
    }

    // Zero-work fallback. When EVERY task was {-1}-only the loop above ran zero
    // walk-evals (the baseline eval above is the only one), so UpdateRecords
    // fired exactly once (the baseline). That single fire is enough for
    // UpdateRecords to advance prev_optimizer_ via the baseline eval — but only
    // if the baseline eval actually ran OptimizeIncre (incremental path) or
    // OptimizeFromScratch (reopt path), which it did. So the all-{-1} case is
    // already covered by the baseline eval above; no extra fallback eval is
    // needed. The guard below is retained as a defensive no-op: it only fires
    // when the descent produced zero evals AND the DAG is non-empty, which
    // cannot happen now (the baseline eval always runs first), but is kept to
    // preserve the prior "no evals, no crash" contract for any future caller
    // that bypasses the baseline eval.
    if (!any_eval_ran && !dag_tasks_.tasks.empty()) {
        EvaluateTimeLimitConfig_ScratchOrIncre(K, time_limits, from_scratch);
    }
}

// 1-arg overload — INCR_SCRATCH entry point. See header for why this is an
// amnesiac reopt (fresh optimizer each interval → prev_optimizer_
// uninitialized → RM+min-TL baseline every call), distinct from the persistent
// optimizer used by Optimize_w_TL_ScratchOrIncre (INCR with period=1).
PriorityVec OptimizePA_Incre_with_TimeLimits::ReOptimizePeriodic(int K) {
    return ReOptimizePeriodic(dag_tasks_, K);
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
    InitializeTimeLimitsToSmallest(time_limits);
    EvaluateTimeLimitConfig_ScratchOrIncre(K, time_limits, from_scratch);
    return opt_pa_;
}

PriorityVec OptimizePA_Incre_with_TimeLimits::OptimizeIncre_w_TL(
    const DAG_Model& dag_tasks_update, int K) {
    opt_sp_ = -1.0;
    dag_tasks_ = dag_tasks_update;
    ApplyWCETAblationIfRequired(dag_tasks_);
    // Full per-task option set (every timePerformancePairs entry). The walk
    // steps over this set and stops on patience-bounded non-improvement — no
    // radius cap, so a tie-break or strictly-better option beyond the old
    // radius wall is reachable.
    time_limit_option_for_each_task_ = RecordTimeLimitOptions(dag_tasks_);
    std::vector<double> time_limits = InitializeTimeLimitsFromETConfig();
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

// Seed the full incumbent 4-tuple {dag, sp, pa, tl} into state. This is the
// baseline that UpdateRecords' compare guard measures the from-scratch search
// against, so opt_sp_ must hold it (NOT -1.0) when the search runs.
void OptimizePA_Incre_with_TimeLimits::SeedStateFromIncumbent(
    const DAG_Model& dag_with_tl, const PriorityVec& pa, double sp,
    const std::vector<double>& tl) {
    opt_sp_ = sp;
    opt_pa_ = pa;
    res_opt_.SaveTimeLimits(dag_tasks_.tasks, tl);
    res_opt_.UpdatePriorityVec(opt_pa_);
    res_opt_.sp_opt = opt_sp_;
    // prev_optimizer_ carries the TL-applied DAG so the next incremental
    // OptimizeIncre diffs against dag_with_tl, not the raw dag_tasks_.
    prev_optimizer_.UpdateDAG(dag_with_tl);
    prev_optimizer_.opt_pa_ = pa;
    prev_optimizer_.opt_sp_ = sp;
    // Carry sp_parameters_ too. IfInitialized() only checks !opt_pa_.empty(),
    // so without this the incremental branch
    // (EvaluateTimeLimitConfig_ScratchOrIncre) takes `optimizer =
    // prev_optimizer_` with an EMPTY sp_parameters_ → OptimizeIncre's SP-eval
    // throws _Map_base::at on thresholds_node. In production this is masked
    // because UpdateRecords (`prev_optimizer_ = optimizer`, a full copy)
    // usually fires between a reopt and the next incremental call; but a
    // no-improvement all-{-1} interval never fires UpdateRecords, and Fix B's
    // zero-work fallback would hit the same crash.
    prev_optimizer_.sp_parameters_ = sp_parameters_;
}

// Establish the incumbent baseline BEFORE the from-scratch search. An optimizer
// status is the 4-tuple {dag, sp, pa, tl}; prev_optimizer_ carries it across
// intervals. Two cases:
//  - Have incumbent: re-eval its {pa, tl} under the NEW DAG (its carried SP was
//    computed under an older DAG) → that re-evaluated tuple is the baseline.
//  - Interval 0 (no incumbent): synthesize one from RM priorities + min TL, and
//    evaluate it. This guarantees a valid baseline to compare against.
// The baseline is seeded into state via SeedStateFromIncumbent so that
// UpdateRecords' "adopt only if strictly greater SP (tie-break lower TL-sum)"
// guard, invoked during the search, IS the compare-and-keep — no separate
// restore step needed.
void OptimizePA_Incre_with_TimeLimits::SeedIncumbentBaseline() {
    if (prev_optimizer_.IfInitialized()) {
        std::vector<double> tl_prev = ReconstructTimeLimitVecFromResOpt();
        PriorityVec pa_prev = opt_pa_;
        DAG_Model dag_new_with_tl_prev =
            UpdateExtDistBasedOnTimeLimit(dag_tasks_, tl_prev);
        double sp_prev_new = EvaluateSPWithPriorityVec(dag_new_with_tl_prev,
                                                       sp_parameters_, pa_prev);
        SeedStateFromIncumbent(dag_new_with_tl_prev, pa_prev, sp_prev_new,
                               tl_prev);
    } else {
        // Interval 0: RM priorities + every task at its smallest TL option.
        std::vector<double> tl_min = SmallestTimeLimitVec();
        PriorityVec pa_rm = RateMonotonicPriorityVec();
        DAG_Model dag_with_tl_min =
            UpdateExtDistBasedOnTimeLimit(dag_tasks_, tl_min);
        double sp_rm =
            EvaluateSPWithPriorityVec(dag_with_tl_min, sp_parameters_, pa_rm);
        SeedStateFromIncumbent(dag_with_tl_min, pa_rm, sp_rm, tl_min);
    }
}

PriorityVec OptimizePA_Incre_with_TimeLimits::ReOptimizePeriodic(
    const DAG_Model& dag_tasks_update, int K) {
    // Compare-and-keep reoptimization. The incumbent is an optimizer status
    // {dag, sp, pa, tl} carried in prev_optimizer_. Seed the baseline (re-eval
    // under the new DAG, or RM+min-TL at interval 0) into state, then run a
    // fresh from-scratch coordinate descent over the FULL per-task option set.
    // UpdateRecords' compare guard (strictly-greater SP wins, tie-break lower
    // TL-sum) preserves the incumbent when the search finds nothing better — so
    // compare-and-keep is just the guard, with no separate restore step.

    dag_tasks_ = dag_tasks_update;
    ApplyWCETAblationIfRequired(dag_tasks_);
    // Full per-task option set — see OptimizeIncre_w_TL for why the walk is no
    // longer radius-capped.
    time_limit_option_for_each_task_ = RecordTimeLimitOptions(dag_tasks_);

    // Establish the incumbent baseline in state (opt_sp_ holds it, NOT -1.0).
    SeedIncumbentBaseline();

    // Fresh from-scratch descent. Each candidate the search evaluates is
    // compared against the seeded baseline inside UpdateRecords; the search
    // result is adopted only if it strictly improves SP (or ties with lower
    // TL-sum). Otherwise the baseline survives untouched.
    std::vector<double> time_limits = InitializeTimeLimitsFromETConfig();
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
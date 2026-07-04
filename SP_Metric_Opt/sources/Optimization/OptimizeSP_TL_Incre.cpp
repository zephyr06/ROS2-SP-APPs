
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
        // Reoptimization path: ignore any warm state and re-search from scratch.
        // This escapes PA drift by exploring the full beam for the current TL.
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
        // an incumbent to warm-start from, but prev_optimizer_ is uninitialized.
        // Under the incremental-scheduler contract the from-scratch bootstrap is
        // scheduler-driven — from_scratch is called at interval 0 (and, in
        // future, periodically to escape drift) — so OptimizeIncre_w_TL is never
        // the bootstrap. Reaching here means a caller invoked the incremental
        // path before any from_scratch call established an incumbent. Bootstrap
        // with a from_scratch call (e.g. ReOptimizePeriodic) first.
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

void OptimizePA_Incre_with_TimeLimits::PerformCoordinateDescentForTaskConfigOpt(
    int K, std::vector<double>& time_limits, bool from_scratch) {
    std::vector<size_t> sorted_indices(dag_tasks_.tasks.size());
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(sorted_indices.begin(), sorted_indices.end(),
              TaskSortingHeuristic{dag_tasks_, sp_parameters_});

    for (size_t idx : sorted_indices) {
        double best_sp = -2.0;
        double best_option_val = time_limits[idx];
        for (double val : time_limit_option_for_each_task_[idx]) {
            if (val == -1 && best_sp > -1)  // there are no options to evaluate
                continue;
            time_limits[idx] = val;
            double sp_val =
                EvaluateTimeLimitConfig_ScratchOrIncre(K, time_limits,
                                                       from_scratch);
            if (sp_val > best_sp && !ApproxEqualSP(sp_val, best_sp)) {
                best_sp = sp_val;
                best_option_val = val;
            } else if (ApproxEqualSP(sp_val, best_sp)) {
                if (val < best_option_val) {
                    best_option_val = val;
                }
            }
        }
        time_limits[idx] = best_option_val;
    }
}

PriorityVec OptimizePA_Incre_with_TimeLimits::ReOptimizePeriodic(int K) {
    return ReOptimizePeriodic(
        dag_tasks_, K, GlobalVariables::ReoptimizationTimeLimitsSearchRadius);
}

PriorityVec OptimizePA_Incre_with_TimeLimits::OptimizeIncre_w_TL(
    const DAG_Model& dag_tasks_update, int K) {
    return OptimizeIncre_w_TL(
        dag_tasks_update, K,
        GlobalVariables::ReoptimizationTimeLimitsSearchRadius);
}

PriorityVec OptimizePA_Incre_with_TimeLimits::OptimizeWithTimeLimitOptDisabled(
    int K, std::vector<double>& time_limits, bool from_scratch) {
    InitializeTimeLimitsToSmallest(time_limits);
    EvaluateTimeLimitConfig_ScratchOrIncre(K, time_limits, from_scratch);
    return opt_pa_;
}

PriorityVec OptimizePA_Incre_with_TimeLimits::OptimizeIncre_w_TL(
    const DAG_Model& dag_tasks_update, int K, int radius) {
    opt_sp_ = -1.0;
    dag_tasks_ = dag_tasks_update;
    ApplyWCETAblationIfRequired(dag_tasks_);
    time_limit_option_for_each_task_ =
        RecordCloseTimeLimitOptions(dag_tasks_, radius);
    std::vector<double> time_limits = InitializeTimeLimitsFromETConfig();
    if (GlobalVariables::disable_time_limit_opt) {
        return OptimizeWithTimeLimitOptDisabled(K, time_limits,
                                                /*from_scratch=*/false);
    }
    PerformCoordinateDescentForTaskConfigOpt(K, time_limits,
                                             /*from_scratch=*/false);
    return opt_pa_;
}

PriorityVec OptimizePA_Incre_with_TimeLimits::ReOptimizePeriodic(
    const DAG_Model& dag_tasks_update, int K, int radius) {
    opt_sp_ = -1.0;
    dag_tasks_ = dag_tasks_update;
    ApplyWCETAblationIfRequired(dag_tasks_);
    time_limit_option_for_each_task_ =
        RecordCloseTimeLimitOptions(dag_tasks_, radius);
    std::vector<double> time_limits = InitializeTimeLimitsFromETConfig();
    if (GlobalVariables::disable_time_limit_opt) {
        return OptimizeWithTimeLimitOptDisabled(K, time_limits,
                                                /*from_scratch=*/true);
    }
    PerformCoordinateDescentForTaskConfigOpt(K, time_limits,
                                             /*from_scratch=*/true);
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

void OptimizePA_Incre_with_TimeLimits::InitializeTimeLimitsToSmallest(
    std::vector<double>& time_limits) {
    for (size_t i = 0; i < time_limits.size(); i++) {
        if (dag_tasks_.tasks[i].timePerformancePairs.empty()) {
            time_limits[i] = -1.0;
        } else {
            time_limits[i] =
                dag_tasks_.tasks[i].timePerformancePairs[0].time_limit;
        }
    }
}

}  // namespace SP_OPT_PA
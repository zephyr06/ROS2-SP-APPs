
#include "sources/Optimization/OptimizeSP_TL_Incre.h"

namespace SP_OPT_PA {

size_t Find_Close_ExecutionTime(
    const std::vector<TimePerfPair>& time_perf_pairs, double time_limit) {
    if (time_perf_pairs.size() == 0) return -1;
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
    const DAG_Model& dag_tasks) {
    std::vector<std::vector<double>> time_limit_option_for_each_task;
    time_limit_option_for_each_task.reserve(dag_tasks.tasks.size());
    for (uint i = 0; i < dag_tasks.tasks.size(); i++) {
        time_limit_option_for_each_task.push_back({});
        size_t close_time_limit_index = Find_Close_ExecutionTime(
            dag_tasks.tasks[i].timePerformancePairs,
            dag_tasks.tasks[i].execution_time_dist.GetAvgValue());
        for (int j = max(0, static_cast<int>(close_time_limit_index) - 1);
             j <= close_time_limit_index + 1 &&
             j < dag_tasks.tasks[i].timePerformancePairs.size();
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

void OptimizePA_Incre_with_TimeLimits::TraverseTimeLimitFromScratch(
    int K, uint task_id, std::vector<double>& time_limits) {
    if (task_id == dag_tasks_.tasks.size()) {
        SP_Parameters sp_para_cur =
            AddWeightsFromTimeLimits(dag_tasks_, sp_parameters_, time_limits);
        DAG_Model dag_tasks_cur =
            UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits);
        OptimizePA_Incre optimizer(dag_tasks_cur, sp_para_cur);
        optimizer.OptimizeFromScratch(K);
        if (optimizer.opt_sp_ > opt_sp_) {
            opt_sp_ = optimizer.opt_sp_;
            opt_pa_ = optimizer.opt_pa_;

            res_opt_.SaveTimeLimits(dag_tasks_.tasks, time_limits);
            res_opt_.UpdatePriorityVec(opt_pa_);
            res_opt_.sp_opt = opt_sp_;

            if (GlobalVariables::debugMode) {
                std::cout << "Time limit: \n";
                for (double time : time_limits) std::cout << time << " ";
                std::cout << "TraverseTimeLimitFromScratch: "
                          << "opt_sp_ = " << opt_sp_ << std::endl;
            }
        }
        timelimit2optimizer_[task_id][time_limits[task_id]] = optimizer;
        return;
    } else {
        for (double time_limit : time_limit_option_for_each_task_[task_id]) {
                        time_limits.push_back(time_limit);
            TraverseTimeLimitFromScratch(K, task_id + 1, time_limits);
            time_limits.pop_back();
        }
    }
}

PriorityVec OptimizePA_Incre_with_TimeLimits::OptimizeFromScratch_w_TL(int K) {
    opt_sp_ = 0;
    std::vector<double> time_limits;
    time_limits.reserve(dag_tasks_.tasks.size());
    TraverseTimeLimitFromScratch(K, 0, time_limits);
    return opt_pa_;
}
// PriorityVec OptimizePA_Incre_with_TimeLimits::OptimizeIncre_w_TL(
//     const DAG_Model& dag_tasks_update);

}  // namespace SP_OPT_PA
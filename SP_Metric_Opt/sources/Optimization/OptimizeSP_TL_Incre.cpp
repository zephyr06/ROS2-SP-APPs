
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
std::vector<std::vector<double>> RecordTimeLimitOptions(
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

void OptimizePA_Incre_with_TimeLimits::Optimize(
    uint trav_task_index, std::vector<double>& time_limit_for_task) {
    if (trav_task_index == time_limit_option_for_each_task.size()) {
        SP_Parameters sp_para_cur = AddWeightsFromTimeLimits(
            dag_tasks, sp_parameters, time_limit_for_task);
        DAG_Model dag_tasks_cur =
            UpdateExtDistBasedOnTimeLimit(dag_tasks, time_limit_for_task);
        ResourceOptResult res_cur =
            OptimizePA_BruteForce(dag_tasks_cur, sp_para_cur);
        res_cur.SaveTimeLimits(dag_tasks.tasks, time_limit_for_task);
        if (res_cur.sp_opt > res_opt.sp_opt) {
            res_opt = res_cur;
        }
        return;
    } else {
        std::vector<double>& time_limit_options =
            time_limit_option_for_each_task[trav_task_index];
        for (double option : time_limit_options) {
            time_limit_for_task[trav_task_index] = option;
            Optimize(trav_task_index + 1, time_limit_for_task);
        }
    }
}

void OptimizePA_Incre_with_TimeLimits::Optimize() {
    std::vector<double> time_limit_for_task(N, -1);
    Optimize(0, time_limit_for_task);
}

PriorityVec OptimizePA_Incre_with_TimeLimits::OptimizeFromScratch(int K) {
    return {};
}

}  // namespace SP_OPT_PA

#include "sources/Optimization/OptimizeSP_TL_BF.h"

namespace SP_OPT_PA {
// -1 time limit means no time limit
SP_Parameters AddWeightsFromTimeLimits(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters,
    const std::vector<double> time_limit_for_task) {
    SP_Parameters sp_parameters_upd = sp_parameters;
    for (int i = 0; i < static_cast<int>(dag_tasks.tasks.size()); i++) {
        if (time_limit_for_task[i] != -1) {
            double update = GetPerfTerm(dag_tasks.tasks[i].timePerformancePairs,
                                        time_limit_for_task[i]);
            sp_parameters_upd.weights_node[i] *= update;
        }
    }
    return sp_parameters_upd;
}

DAG_Model UpdateExtDistBasedOnTimeLimit(const DAG_Model& dag_tasks,
                                        const std::vector<double>& time_limit) {
    DAG_Model dag_tasks_upd = dag_tasks;
    for (int i = 0; i < static_cast<int>(dag_tasks.tasks.size()); i++) {
        if (time_limit[i] != -1) {
            dag_tasks_upd.tasks[i].execution_time_dist =
                GetUnitExecutionTimeDist(time_limit[i]);
        }
    }
    return dag_tasks_upd;
}

void OptimizePA_with_TimeLimitsStatus::RecordTimeLimitOptions() {
    time_limit_option_for_each_task.reserve(dag_tasks.tasks.size());
    for (uint i = 0; i < dag_tasks.tasks.size(); i++) {
        time_limit_option_for_each_task.push_back({});
        for (uint j = 0; j < dag_tasks.tasks[i].timePerformancePairs.size();
             j++) {
            time_limit_option_for_each_task[i].push_back(
                dag_tasks.tasks[i].timePerformancePairs[j].time_limit);
        }
        if (dag_tasks.tasks[i].timePerformancePairs.size() == 0) {
            time_limit_option_for_each_task[i].push_back(-1);
        }
    }
}

void OptimizePA_with_TimeLimitsStatus::Optimize(
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

void OptimizePA_with_TimeLimitsStatus::Optimize() {
    std::vector<double> time_limit_for_task(N, -1);
    Optimize(0, time_limit_for_task);
}

ResourceOptResult BackTrackingPA_with_TimeLimits(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters) {
    OptimizePA_with_TimeLimitsStatus optimizer(dag_tasks, sp_parameters);
    optimizer.Optimize();
    return optimizer.res_opt;
}
}  // namespace SP_OPT_PA
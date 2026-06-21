#pragma once
#include "sources/Optimization/OptimizeSP_BF.h"
#include "sources/Safety_Performance_Metric/SP_Metric.h"

namespace SP_OPT_PA {
/**
 * Replace each task's execution_time_dist with a unit distribution at the given
 * time limit.  time_limit[i] == -1 means the task is left unchanged.
 */
DAG_Model UpdateExtDistBasedOnTimeLimit(const DAG_Model& dag_tasks,
                                        const std::vector<double>& time_limit);

std::vector<std::vector<double>> RecordTimeLimitOptions(
    const DAG_Model& dag_tasks);
class OptimizePA_with_TimeLimitsStatus {
   public:
    OptimizePA_with_TimeLimitsStatus(const DAG_Model& dag_tasks,
                                     const SP_Parameters& sp_parameters)
        : dag_tasks(dag_tasks),
          sp_parameters(sp_parameters),
          N(dag_tasks.tasks.size()) {
        res_opt.sp_opt = INT_MIN;
        time_limit_option_for_each_task = RecordTimeLimitOptions(dag_tasks);
    }

    void Optimize(uint trav_task_index,
                  std::vector<double>& time_limit_for_task);

    void Optimize();

    // data members
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
    int N;
    ResourceOptResult res_opt;
    std::vector<std::vector<double>> time_limit_option_for_each_task;
};

ResourceOptResult EnumeratePA_with_TimeLimits(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters);

}  // namespace SP_OPT_PA
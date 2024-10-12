#pragma once
#include "sources/Optimization/OptimizeSP_TL_BF.h"
#include "sources/Safety_Performance_Metric/SP_Metric.h"

namespace SP_OPT_PA {

std::vector<std::vector<double>> RecordTimeLimitOptions(
    const DAG_Model& dag_tasks);

class OptimizePA_Incre_with_TimeLimits {
   public:
    OptimizePA_Incre_with_TimeLimits(const DAG_Model& dag_tasks,
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
    PriorityVec OptimizeFromScratch(int K);

    // data members
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
    int N;
    ResourceOptResult res_opt;
    std::vector<std::vector<double>> time_limit_option_for_each_task;
};

inline PriorityVec PerformOptimizePA_Incre_w_TimeLimits(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    return opt.OptimizeFromScratch(
        GlobalVariables::Layer_Node_During_Incremental_Optimization);
}
}  // namespace SP_OPT_PA
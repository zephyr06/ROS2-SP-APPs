#pragma once
#include "sources/Optimization/OptimizeSP_Incre.h"
#include "sources/Optimization/OptimizeSP_TL_BF.h"
#include "sources/Safety_Performance_Metric/SP_Metric.h"

namespace SP_OPT_PA {

std::vector<std::vector<double>> RecordCloseTimeLimitOptions(
    const DAG_Model& dag_tasks);

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

class OptimizePA_Incre_with_TimeLimits : public OptimizePA_Incre {
   public:
   OptimizePA_Incre_with_TimeLimits(){};
    OptimizePA_Incre_with_TimeLimits(const DAG_Model& dag_tasks,
                                     const SP_Parameters& sp_parameters)
        : OptimizePA_Incre(dag_tasks, sp_parameters) {
        time_limit_option_for_each_task_ = RecordTimeLimitOptions(dag_tasks_);
    }

    void TraverseTimeLimitOptions(int K, uint task_id,
                                  std::vector<double>& time_limits);
    PriorityVec OptimizeFromScratch_w_TL(int K);

    PriorityVec OptimizeIncre_w_TL(const DAG_Model& dag_tasks_update, int K);

    void UpdateRecords(const OptimizePA_Incre& optimizer,
                       const std::vector<double>& time_limits);

    inline ResourceOptResult CollectResults() const { return res_opt_; }

    // data members
    ResourceOptResult res_opt_;
    std::vector<std::vector<double>> time_limit_option_for_each_task_;
    // For each task id, it maps time limit to the optimizer
    std::unordered_map<std::vector<double>, OptimizePA_Incre, HashKey4Vector>
        timelimit2optimizer_;
};

inline PriorityVec PerformOptimizePA_Incre_w_TimeLimits(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    return opt.OptimizeFromScratch(
        GlobalVariables::Layer_Node_During_Incremental_Optimization);
}
}  // namespace SP_OPT_PA
#pragma once

#include "sources/Optimization/OptimizeSP_Base.h"

namespace SP_OPT_PA {

class OptimizePA_BF : public OptimimizePA_Base {
   public:
    OptimizePA_BF(const DAG_Model& dag_tasks,
                  const SP_Parameters& sp_parameters)
        : OptimimizePA_Base(dag_tasks, sp_parameters) {}

    void IterateAllPAs(PriorityVec& priority_assignment,
                       std::unordered_set<int>& tasks_assigned_priority,
                       int start);

    PriorityVec Optimize();
};

inline ResourceOptResult OptimizePA_BruteForce(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters) {
    OptimizePA_BF opt(dag_tasks, sp_parameters);
    PriorityVec pa_vec = opt.Optimize();
    ResourceOptResult res;
    res.UpdatePriorityVec(pa_vec);
    res.sp_opt = opt.opt_sp_;
    return res;
}

}  // namespace SP_OPT_PA
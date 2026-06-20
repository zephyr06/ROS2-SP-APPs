#pragma once

#include "sources/Optimization/OptimizeSP_Base.h"

namespace SP_OPT_PA {

class OptimizePA_BF : public OptimimizePA_Base {
   public:
    OptimizePA_BF(const DAG_Model& dag_tasks,
                  const SP_Parameters& sp_parameters,
                  int tie_break_type = -1)
        : OptimimizePA_Base(dag_tasks, sp_parameters) {
        if (tie_break_type == -1) {
            tie_break_type_ = GlobalVariables::bf_tie_break_type;
        } else {
            tie_break_type_ = tie_break_type;
        }
    }

    void IterateAllPAs(PriorityVec& priority_assignment,
                       std::unordered_set<int>& tasks_assigned_priority,
                       int start);

    PriorityVec Optimize();

    int eval_priority_count = 0;

   private:
    bool BetterPriorityAssignment(const PriorityVec& pa1, const PriorityVec& pa2) const;
    int tie_break_type_;
};

inline ResourceOptResult OptimizePA_BruteForce(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters, int* out_priority_count = nullptr, int tie_break_type = -1) {
    OptimizePA_BF opt(dag_tasks, sp_parameters, tie_break_type);
    PriorityVec pa_vec = opt.Optimize();
    ResourceOptResult res;
    res.UpdatePriorityVec(pa_vec);
    res.sp_opt = opt.opt_sp_;
    if (out_priority_count) {
        *out_priority_count = opt.eval_priority_count;
    }
    return res;
}

inline ResourceOptResult OptimizePA_BruteForce(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters) {
    return OptimizePA_BruteForce(dag_tasks, sp_parameters, nullptr);
}

}  // namespace SP_OPT_PA
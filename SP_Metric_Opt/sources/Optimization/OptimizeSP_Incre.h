#pragma once

#include <queue>
#include <unordered_set>

#include "sources/Optimization/OptimizeSP_Base.h"

namespace SP_OPT_PA {
struct PriorityPartialPath {
    PriorityPartialPath() {}
    PriorityPartialPath(const DAG_Model& dag_tasks,
                        const SP_Parameters& sp_parameters);

    void AssignAndUpdateSP(int task_id);

    void UpdateSP(int task_id);
    inline void AssertValidIndex(size_t i) const {
        if (i >= pa_vec_lower_pri.size())
            CoutError("Empty path in GetTaskWeight");
    }
    inline int GetTaskWeight(size_t i) const {
        AssertValidIndex(i);
        return sp_parameters.weights_node.at(pa_vec_lower_pri[i]);
    }
    inline int GetTaskPeriod(size_t i) const {
        AssertValidIndex(i);
        return dag_tasks.tasks[pa_vec_lower_pri[i]].period;
    }
    // inline int GetLastTaskMinUtil(size_t i) const {
    //     AssertValidIndex(i);
    //     return dag_tasks.tasks[pa_vec_lower_pri[i]].utilization();
    // }
    inline int GetTaskMinEt(size_t i) const {
        AssertValidIndex(i);
        return dag_tasks.tasks[pa_vec_lower_pri[i]]
            .execution_time_dist.min_time;
    }

    // const DAG_Model& dag_tasks;
    // const SP_Parameters& sp_parameters;
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
    // double sp=0;
    double sp_lost = 0;
    PriorityVec pa_vec_lower_pri;
    std::unordered_set<int> tasks_to_assign;
};

struct CompPriorityPath {
    // return true if rhs is better than lhs
    bool operator()(const PriorityPartialPath& lhs,
                    const PriorityPartialPath& rhs) const;
};

std::vector<int> FindTaskWithDifferentEt(const DAG_Model& dag_tasks,
                                         const DAG_Model& dag_tasks_updated);

PriorityVec RemoveOneTask(const PriorityVec& pa_vec, int task_id);

std::vector<PriorityVec> FindPriorityVec1D_Variations(const PriorityVec& pa_vec,
                                                      int task_id);

class OptimizePA_Incre : public OptimimizePA_Base {
   public:
    OptimizePA_Incre() {}
    OptimizePA_Incre(const DAG_Model& dag_tasks,
                     const SP_Parameters& sp_parameters)
        : OptimimizePA_Base(dag_tasks, sp_parameters) {}

    // TODO: Current implementation doesn't consider end-to-end latency, need to
    // add later! One way to do it is by modifying the parameters of
    // sp_parameters
    /*
    The implementation for this function follows Audsley's algorithm with
    modifications for speed and optimization considerations:
    // 1. The algortihm iterativelys finds the task to assign the lowest
    priority to. However, since multiple tasks may qualify for the lowest
    priority,
    // the algorithm will consider all of them and save them as partial paths.
    // 2. The input argument K records the maximum number of partial paths under
    consideration in each iteration.
    // 3. This function updates both opt_pa_ and opt_sp_, and returns opt_pa_.
    */
    PriorityVec OptimizeFromScratch(int K);

    PriorityVec OptimizeIncre(const DAG_Model& dag_tasks_update);

    bool IfInitialized() const { return !dag_tasks_.tasks.empty(); }
};

inline PriorityVec PerformOptimizePA_Incre(const DAG_Model& dag_tasks,
                                           const SP_Parameters& sp_parameters) {
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    return opt.OptimizeFromScratch(
        GlobalVariables::Layer_Node_During_Incremental_Optimization);
}
}  // namespace SP_OPT_PA
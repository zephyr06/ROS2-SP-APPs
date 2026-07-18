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

struct DiffObj {
    int task_id;
    bool increase;
};
std::vector<DiffObj> FindTaskWithDifferentEt(
    const DAG_Model& dag_tasks, const DAG_Model& dag_tasks_updated);

PriorityVec RemoveOneTask(const PriorityVec& pa_vec, int task_id);

enum PriorityChangeStatus { Increase, Decrease, OpenToAll };

PriorityChangeStatus AnalyzePriorityChangeStatus(
    const SP_Parameters& sp_parameters, int task_id, bool et_increased);
// `exclude_opt_pa` (default true): skip emitting the variation that re-
// inserts task_id at its carried position (i == old_priority_index). That
// variation reconstructs `pa_vec` exactly, so scoring it re-computes the
// incumbent's SP — the redundant eval the sub-incremental (OptimizeIncre_
// SingleTask) avoids by scoring the carried PA once as the baseline. Callers
// that want the FULL candidate range (including the carried position) pass
// false (e.g. unit tests asserting the range contract).
std::vector<PriorityVec> FindPriorityVec1D_Variations(
    const PriorityVec& pa_vec, int task_id,
    PriorityChangeStatus priority_change, bool exclude_opt_pa = true);

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

    // Incremental re-search over ALL tasks whose ET changed since the last
    // dag_tasks_ (FindTaskWithDifferentEt). Seeds opt_sp_ to the carried PA's SP
    // under the new env (the former :243-244 baseline), then re-searches each
    // changed task. `baseline_sp` (default INT_MIN = "not provided") lets a
    // caller that already holds the carried PA's new-env SP skip the baseline
    // re-score; if provided it MUST equal
    // EvaluateSPWithPriorityVec(dag_tasks_update, sp_parameters_, opt_pa_) for
    // the exact dag_tasks_update + opt_pa_, else the strict-> adopt test would
    // compare against a wrong seed. Advances dag_tasks_ to dag_tasks_update
    // (orchestrator-owned under P0.5 — inert, the challenger is rebuilt each
    // step; kept for bit-identity).
    PriorityVec OptimizeIncre(const DAG_Model& dag_tasks_update,
                              double baseline_sp = INT_MIN);

    // The sub-incremental primitive: assumes EXACTLY ONE task's ET changed
    // (task_id). Trusts opt_sp_ as the current baseline (caller-set: OptimizeIncre
    // scores the carried PA at :243-244, or a TL handler seeds it via
    // BuildChallengerFromIncumbent / a fresh Type-E new-env baseline). Generates
    // the 1D priority variations for task_id (one half, per
    // AnalyzePriorityChangeStatus, with exclude_opt_pa=true), scores each, and
    // adopts on strict > — bit-identical to the former :274-292 loop body.
    // Mutates opt_pa_/opt_sp_ in place. Does NOT advance dag_tasks_
    // (orchestrator-owned).
    PriorityVec OptimizeIncre_SingleTask(const DAG_Model& dag_tasks_update,
                                         int task_id, bool et_increased);

    bool IfInitialized() const { return !opt_pa_.empty(); }
};

inline PriorityVec PerformOptimizePA_Incre(const DAG_Model& dag_tasks,
                                           const SP_Parameters& sp_parameters) {
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    return opt.OptimizeFromScratch(
        GlobalVariables::Layer_Node_During_Incremental_Optimization);
}
}  // namespace SP_OPT_PA
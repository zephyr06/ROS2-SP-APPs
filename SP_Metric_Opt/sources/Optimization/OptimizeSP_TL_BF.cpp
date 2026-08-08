
#include "sources/Optimization/OptimizeFallback.h"
#include "sources/Optimization/OptimizeSP_TL_BF.h"

namespace SP_OPT_PA {

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

std::vector<std::vector<double>> RecordTimeLimitOptions(
    const DAG_Model& dag_tasks) {
    std::vector<std::vector<double>> time_limit_option_for_each_task;
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
    return time_limit_option_for_each_task;
}

void OptimizePA_with_TimeLimitsStatus::Optimize(
    uint trav_task_index, std::vector<double>& time_limit_for_task) {
    // P1.14 — poll the SHARED BF budget (installed by
    // EnumeratePA_with_TimeLimits via BFDLSharedBudget) rather than this
    // object's own start_time_. The per-leaf OptimizePA_BF does the same.
    // BFSharedBudgetCancelled() is only true while a BFDLSharedBudget scope
    // is active, so this guard is inert outside a BF search.
    if (BFSharedBudgetCancelled())
        return;
    if (trav_task_index == time_limit_option_for_each_task.size()) {
        DAG_Model dag_tasks_cur =
            UpdateExtDistBasedOnTimeLimit(dag_tasks, time_limit_for_task);
        ResourceOptResult res_cur =
            OptimizePA_BruteForce(dag_tasks_cur, sp_parameters);
        res_cur.SaveTimeLimits(dag_tasks.tasks, time_limit_for_task);
        // P1.27 — gate each leaf in-search; else BF adopts the SP-max plan, the
        // post-hoc gate swaps it to RM-Fast, and BF < INCR. Vacuous w/o important.
        if (res_cur.sp_opt > res_opt.sp_opt &&
            ImportantTasksMeetThresholds(dag_tasks, sp_parameters,
                                         res_cur.priority_vec,
                                         time_limit_for_task)) {
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
    // P0.10 §2 — gate the final result on important-task schedulability; on FAIL
    // swap in the RM-Fast plan; on double-fail throw. Inside the optimizer so
    // every caller is gated. No-op when no task is important (gate vacuously
    // passes); only `is_important` task sets can trigger the swap.
    res_opt = AdoptRmFastFallbackIfUnschedulable(dag_tasks, sp_parameters, res_opt);
}

ResourceOptResult EnumeratePA_with_TimeLimits(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters) {
    // P1.14 — install ONE shared TIME_LIMIT budget for the entire BF search.
    // `start_time_` was captured at this optimizer's construction
    // (OptimizeSP_TL_BF.h:22), i.e. at entry to this function. The
    // BFDLSharedBudget guard publishes it so that:
    //   - the outer TL-combination recursion (Optimize above) polls it via
    //     BFSharedBudgetCancelled();
    //   - the inner per-leaf OptimizePA_BF (constructed fresh per leaf by
    //     OptimizePA_BruteForce) polls the SAME shared budget instead of its
    //     own per-leaf start_time_ (which previously reset every leaf and
    //     could not bound the aggregate);
    //   - ObtainSP_DAG / ObtainSP_TaskSet poll it between sub-computations so
    //     a single runaway EvaluateSPWithPriorityVec call (wide RTA
    //     convolutions) can be interrupted in place rather than stranding the
    //     search past the cap.
    // The guard restores the prior budget (nullptr) on destruction, so this
    // is re-entrant and inert for any non-BF caller of ObtainSP_DAG.
    OptimizePA_with_TimeLimitsStatus optimizer(dag_tasks, sp_parameters);
    BFDLSharedBudget shared_budget(optimizer.start_time_);
    optimizer.Optimize();
    return optimizer.res_opt;
}
}  // namespace SP_OPT_PA
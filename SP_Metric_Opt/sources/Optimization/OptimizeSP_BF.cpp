
#include "sources/Optimization/OptimizeSP_BF.h"
namespace SP_OPT_PA {

void OptimizePA_BF::IterateAllPAs(
    PriorityVec& priority_assignment,
    std::unordered_set<int>& tasks_assigned_priority, int start) {
    // P1.14 — poll the SHARED BF budget (installed by
    // EnumeratePA_with_TimeLimits) rather than this object's own per-leaf
    // start_time_ (which is captured fresh at every OptimizePA_BruteForce
    // construction and therefore could not bound the AGGREGATE across
    // leaves). BFSharedBudgetCancelled() returns false outside a BF search,
    // so the legacy standalone OptimizePA_BF::Optimize() path is unaffected.
    if (BFSharedBudgetCancelled())
        return;
    if (start == N) {
        // if(priority_assignment[0]!=1)
        //     return;
        // TaskSet tasks_eval =
        //     UpdateTaskSetPriorities(dag_tasks_.tasks, priority_assignment);
        // DAG_Model dag_tasks_eval = dag_tasks_;
        // dag_tasks_eval.tasks = tasks_eval;

        // double sp_eval = ObtainSP_DAG(dag_tasks_eval, sp_parameters_);
        double sp_eval = EvaluateSPWithPriorityVec(dag_tasks_, sp_parameters_,
                                                   priority_assignment)
                             .sp_value;
        PrintPA_IfDebugMode(priority_assignment, sp_eval);
        // P1.29 BF analogue — gate each candidate in-search: adopt only if it
        // both beats the incumbent SP AND keeps every important task
        // schedulable. Without this, BF commits the SP-max PA even when it
        // leaves an important task unschedulable, then the post-hoc backstop
        // swaps the whole plan to RM-Fast. dag_tasks_ already has TLs baked
        // (UpdateExtDistBasedOnTimeLimit), so the no-tl overload is correct.
        // Vacuous (never rejects) when no task is_important -> legacy identical.
        if (sp_eval > opt_sp_ &&
            ImportantTasksMeetThresholds(dag_tasks_, sp_parameters_,
                                         priority_assignment)) {
            opt_sp_ = sp_eval;
            opt_pa_ = priority_assignment;
        }
    } else {
        for (int i = 0; i < N; i++) {
            if (tasks_assigned_priority.count(i) == 0) {
                priority_assignment.push_back(i);
                tasks_assigned_priority.insert(i);
                IterateAllPAs(priority_assignment, tasks_assigned_priority,
                              start + 1);
                tasks_assigned_priority.erase(i);
                priority_assignment.pop_back();
            }
        }
    }
}

PriorityVec OptimizePA_BF::Optimize() {
    if(GlobalVariables::debugMode==1)
        BeginTimer("OptimizeBF_All");
    double initial_sp = ObtainSP_DAG(dag_tasks_, sp_parameters_).sp_value;
    PriorityVec pa = {};
    std::unordered_set<int> tasks_assigned_priority;
    opt_sp_ = initial_sp;
    opt_pa_ = GetPriorityAssignments(dag_tasks_.tasks);
    IterateAllPAs(pa, tasks_assigned_priority, 0);

    std::cout << "Initial SP is: " << initial_sp << "\n";
    std::cout << "Optimal SP is: " << opt_sp_ << "\n";
    if(GlobalVariables::debugMode==1)
        EndTimer("OptimizeBF_All");
    return opt_pa_;
}

// std::vector<int> TranslatePriorityVec(const PriorityVec& pa_vec) {
//     std::vector<int> res(pa_vec.size());
//     // int min_pa=10;
//     int max_pa = 10 * pa_vec.size();
//     for (int i = 0; i < pa_vec.size(); i++) {
//         res[pa_vec[i]] = max_pa - i * 10;
//     }
//     return res;
// }

}  // namespace SP_OPT_PA
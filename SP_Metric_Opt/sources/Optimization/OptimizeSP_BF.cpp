
#include "sources/Optimization/OptimizeSP_BF.h"
#include "sources/Utils/Parameters.h"

namespace SP_OPT_PA {

void OptimizePA_BF::IterateAllPAs(
    PriorityVec& priority_assignment,
    std::unordered_set<int>& tasks_assigned_priority, int start) {
    if (ifTimeout(start_time_))
        return;
    if (start == N) {
        // if(priority_assignment[0]!=1)
        //     return;
        // TaskSet tasks_eval =
        //     UpdateTaskSetPriorities(dag_tasks_.tasks, priority_assignment);
        // DAG_Model dag_tasks_eval = dag_tasks_;
        // dag_tasks_eval.tasks = tasks_eval;

#if defined(RYAN_HE_CHANGE_DEBUG)
        if (GlobalVariables::debugMode & DBG_PRT_MSK_OptimizeSP_BF) {
            std::cout << "####OptimizePA_BF::IterateAllPAs: to EvaluateSPWithPriorityVec for PA = ";
            for (int i = 0; i < priority_assignment.size(); i++) {
                std::cout << priority_assignment[i] << " ";
            }
            std::cout << std::endl;
        }
#endif
        // double sp_eval = ObtainSP_DAG(dag_tasks_eval, sp_parameters_);
        double sp_eval = EvaluateSPWithPriorityVec(dag_tasks_, sp_parameters_,
                                                   priority_assignment);
                                              
        PrintPA_IfDebugMode(priority_assignment, sp_eval);
        if (sp_eval > opt_sp_) {
#if defined(RYAN_HE_CHANGE_DEBUG)
            if (GlobalVariables::debugMode & DBG_PRT_MSK_OptimizeSP_BF) {
                std::cout << "####OptimizePA_BF::IterateAllPAs: to EvaluateSPWithPriorityVec DONE. New PA = ";
                for (int i = 0; i < priority_assignment.size(); i++) {
                    std::cout << priority_assignment[i] << " ";
                }
                std::cout << std::endl;
                std::cout << "####OptimizePA_BF::IterateAllPAs: new SP="<<sp_eval<<std::endl;
            }                 
#endif
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
#if defined(RYAN_HE_CHANGE_DEBUG)
    if (GlobalVariables::debugMode & DBG_PRT_MSK_OptimizeSP_BF)
        std::cout << "####OptimizePA_BF::Optimize: ObtainSP_DAG ... "<< std::endl;
#endif    
    double initial_sp = ObtainSP_DAG(dag_tasks_, sp_parameters_);
#if defined(RYAN_HE_CHANGE_DEBUG)
    if (GlobalVariables::debugMode & DBG_PRT_MSK_OptimizeSP_BF)
        std::cout << "####OptimizePA_BF::Optimize: initial_sp = " << initial_sp << std::endl;
#endif      
    PriorityVec pa = {};
    std::unordered_set<int> tasks_assigned_priority;
    opt_sp_ = initial_sp;
    opt_pa_ = GetPriorityAssignments(dag_tasks_.tasks);
    IterateAllPAs(pa, tasks_assigned_priority, 0);

#if defined(RYAN_HE_CHANGE_DEBUG)
    if (GlobalVariables::debugMode & DBG_PRT_MSK_OptimizeSP_BF)
        std::cout << "####OptimizePA_BF::Optimize: inital/optimal_sp = " << initial_sp << "/" << opt_sp_ << std::endl;
#else
    std::cout << "Initial SP is: " << initial_sp << "\n";
    std::cout << "Optimal SP is: " << opt_sp_ << "\n";
#endif

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
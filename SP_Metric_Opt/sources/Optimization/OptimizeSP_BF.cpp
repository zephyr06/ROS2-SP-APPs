
#include "sources/Optimization/OptimizeSP_BF.h"
#include "sources/Utils/Parameters.h"

namespace SP_OPT_PA {

void OptimizePA_BF::IterateAllPAs(
    PriorityVec& priority_assignment,
    std::unordered_set<int>& tasks_assigned_priority, int start) {
    if (ifTimeout(start_time_))
        return;
    if (start == N) {
        eval_priority_count++;
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
        double tolerance = 1e-9;
        if (sp_eval > opt_sp_ + tolerance || (tie_break_type_ == 1 &&
                                  std::abs(sp_eval - opt_sp_) <= tolerance && 
                                  BetterPriorityAssignment(priority_assignment, opt_pa_))) {
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

bool OptimizePA_BF::BetterPriorityAssignment(const PriorityVec& pa1, const PriorityVec& pa2) const {
    if (pa2.empty()) return true;
    int n = pa1.size();
    for (int i = n - 1; i >= 0; i--) {
        if (pa1[i] != pa2[i]) {
            double w1 = 1.0;
            if (sp_parameters_.weights_node.count(pa1[i])) {
                w1 = sp_parameters_.weights_node.at(pa1[i]);
            }
            double w2 = 1.0;
            if (sp_parameters_.weights_node.count(pa2[i])) {
                w2 = sp_parameters_.weights_node.at(pa2[i]);
            }
            if (w1 != w2) {
                return w1 < w2;
            } else {
                const Task& task1 = dag_tasks_.GetTask(pa1[i]);
                const Task& task2 = dag_tasks_.GetTask(pa2[i]);
                return task1.deadline > task2.deadline;
            }
        }
    }
    return false;
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
    eval_priority_count = 0;
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
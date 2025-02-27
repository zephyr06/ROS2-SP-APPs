
#include "sources/Optimization/OptimizeSP_TL_BF.h"

#include "sources/Utils/Parameters.h"

namespace SP_OPT_PA {

// -1 time limit means not having performance_records_time
// in paper, only TSP has performance_records_time
//
// looks like this is called for each tasks' performance_records_time
// time_limit is one of the entry's in performance_records_time
// update is one of the entry's in performance_records_perf
SP_Parameters AddWeightsFromTimeLimits(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters,
    const std::vector<double> time_limit_for_task) {
    SP_Parameters sp_parameters_upd = sp_parameters;
    for (int i = 0; i < static_cast<int>(dag_tasks.tasks.size()); i++) {
        if (time_limit_for_task[i] != -1) {
            double update = GetPerfTerm(dag_tasks.tasks[i].timePerformancePairs,
                                        time_limit_for_task[i]);
#ifdef RYAN_HE_CHANGE_DEBUG
            // if (1) {
            if (GlobalVariables::debugMode & DBG_PRT_MSK_OptimizeSP_TL_BF) {
                std::cout << "####AddWeightsFromTimeLimits: task[" << i;
                std::cout << "]'s time_limit_for_task="
                          << time_limit_for_task[i];
                std::cout << ", update=" << update << std::endl;
            }
#endif
            sp_parameters_upd.weights_node[i] *= update;
        }
    }
    return sp_parameters_upd;
}

// looks like this is called for each tasks' performance_records_time
// time_limit is one of the entry's in performance_records_time
DAG_Model UpdateExtDistBasedOnTimeLimit(const DAG_Model& dag_tasks,
                                        const std::vector<double>& time_limit) {
    DAG_Model dag_tasks_upd = dag_tasks;
    for (int i = 0; i < static_cast<int>(dag_tasks.tasks.size()); i++) {
        if (time_limit[i] != -1) {
            dag_tasks_upd.tasks[i].execution_time_dist =
                GetUnitExecutionTimeDist(time_limit[i]);
#ifdef RYAN_HE_CHANGE_DEBUG
            // if (1) {
            if (GlobalVariables::debugMode & DBG_PRT_MSK_OptimizeSP_TL_BF) {
                std::cout << "####UpdateExtDistBasedOnTimeLimit: task[" << i;
                std::cout << "]'s time_limit_for_task=" << time_limit[i];
                std::cout << ", update=" << std::endl;
                dag_tasks_upd.tasks[i].execution_time_dist.print();
            }
#endif
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
    if (ifTimeout(start_time_)) return;
    if (trav_task_index == time_limit_option_for_each_task.size()) {
#ifdef RYAN_HE_CHANGE_DEBUG
        if (GlobalVariables::debugMode & DBG_PRT_MSK_OptimizeSP_TL_BF)
            std::cout << "####OptimizePA_with_TimeLimitsStatus::Optimize: "
                         "EVALUATE PA ..."
                      << std::endl;
#endif
        SP_Parameters sp_para_cur = AddWeightsFromTimeLimits(
            dag_tasks, sp_parameters, time_limit_for_task);
        DAG_Model dag_tasks_cur =
            UpdateExtDistBasedOnTimeLimit(dag_tasks, time_limit_for_task);

#ifdef RYAN_HE_CHANGE_DEBUG
        if (GlobalVariables::debugMode & DBG_PRT_MSK_OptimizeSP_TL_BF)
            std::cout << "####OptimizePA_with_TimeLimitsStatus::Optimize: "
                         "OptimizePA_BruteForce ..."
                      << std::endl;
#endif
        ResourceOptResult res_cur =
            OptimizePA_BruteForce(dag_tasks_cur, sp_para_cur);
        res_cur.SaveTimeLimits(dag_tasks.tasks, time_limit_for_task);
        if (res_cur.sp_opt > res_opt.sp_opt) {
#ifdef RYAN_HE_CHANGE_DEBUG
            if (GlobalVariables::debugMode & DBG_PRT_MSK_OptimizeSP_TL_BF) {
                std::cout << "####OptimizePA_with_TimeLimitsStatus::Optimize: "
                             "new PA found, sp_opt="
                          << res_cur.sp_opt << std::endl;
                std::cout << "########PriorityVec: TASK[priorityVec[i]]'s "
                             "priority is i"
                          << std::endl;
                for (int i = 0; i < res_cur.priority_vec.size(); i++) {
                    std::cout << res_cur.priority_vec[i] << " ";
                }
                std::cout << std::endl;
            }
#endif
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
#ifdef RYAN_HE_CHANGE_DEBUG
    if (GlobalVariables::debugMode & DBG_PRT_MSK_OptimizeSP_TL_BF)
        std::cout << "####OptimizePA_with_TimeLimitsStatus::Optimize: " << N
                  << " tasks ..." << std::endl;
#endif

    Optimize(0, time_limit_for_task);
}

ResourceOptResult EnumeratePA_with_TimeLimits(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters) {
#ifdef RYAN_HE_CHANGE_DEBUG
    if (GlobalVariables::debugMode & DBG_PRT_MSK_OptimizeSP_TL_BF)
        std::cout << "####EnumeratePA_with_TimeLimits: call "
                     "OptimizePA_with_TimeLimitsStatus.optimize ..."
                  << std::endl;
#endif
    OptimizePA_with_TimeLimitsStatus optimizer(dag_tasks, sp_parameters);
    optimizer.Optimize();
#ifdef RYAN_HE_CHANGE_DEBUG
    if (GlobalVariables::debugMode & DBG_PRT_MSK_OptimizeSP_TL_BF) {
        // optimizer.res_opt is a PriorityVec which is int vector
        // print it
        std::cout << "####EnumeratePA_with_TimeLimits: "
                     "OptimizePA_with_TimeLimitsStatus.optimize DONE. "
                  << std::endl;
        std::cout << "########PriorityVec: TASK[priorityVec[i]]'s priority is i"
                  << std::endl;
        for (int i = 0; i < optimizer.res_opt.priority_vec.size(); i++) {
            std::cout << optimizer.res_opt.priority_vec[i] << " ";
        }
        std::cout << std::endl;
        std::cout << "########sp_opt:" << optimizer.res_opt.sp_opt << std::endl;
        std::cout << "########id2priority (not sure where it is used):"
                  << std::endl;
        for (int i = 0; i < optimizer.res_opt.id2priority.size(); i++) {
            std::cout << optimizer.res_opt.id2priority[i] << " ";
        }
        std::cout << std::endl;

        // this is important for TSP like performance_records_time
        std::cout << "########id2time_limit (task[i] select execution time "
                     "id2time_limit[i]):"
                  << std::endl;
        for (int i = 0; i < optimizer.res_opt.id2time_limit.size(); i++) {
            std::cout << optimizer.res_opt.id2time_limit[i] << " ";
        }
        std::cout << std::endl;
    }
#endif
    return optimizer.res_opt;
}
}  // namespace SP_OPT_PA
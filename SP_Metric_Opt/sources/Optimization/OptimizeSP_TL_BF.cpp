
#include "sources/Optimization/OptimizeSP_TL_BF.h"

#include "sources/Utils/Parameters.h"

#include <vector>
#include <algorithm>

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
    double util_regular_tasks = 0.0;
    for (int i=0;i<dag_tasks.tasks.size();i++) {
        double mu = dag_tasks.tasks[i].getExecGaussian().mu;
        double prd = dag_tasks.tasks[i].period;
    }

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
    #if 0
    printf("Ryan %s: \n",__func__);
    for (uint i = 0; i < time_limit_option_for_each_task.size(); i++) {
        for (int k=0;k<time_limit_option_for_each_task[i].size();k++) {
            printf("%.2f ",time_limit_option_for_each_task[i][k]);
        }
        printf("\n");
    }
    #endif

    return time_limit_option_for_each_task;
}


// RYAN_20250309
// Function to insert a new subsequence while keeping the sequence sorted
void insert_sorted(std::vector<std::pair<std::vector<double>, double>>& sequences, 
    const std::vector<double>& new_tasks, double utilization) {
    if (sequences.size()==0) {
        sequences.push_back({new_tasks, utilization});
        return;
    }

    // Use binary search to find the correct insertion point
    auto pos = lower_bound(sequences.begin(), sequences.end(), utilization,
        [](const std::pair<std::vector<double>, double>& a, double value) {
            return fabs(a.second - 1.0) < fabs(value - 1.0);
        });

    // Insert at the correct position
    sequences.insert(pos, {new_tasks, utilization});
}

// task_options is time_limit_option_for_each_task
void enumerate_options(std::vector<std::vector<double>> &all_sequences,
    std::vector<std::vector<double>>& task_options, 
    std::vector<double>& current_sequence, int task_idx) {
    if (task_idx == task_options.size()) {
        // Base case: all tasks have been assigned execution times
        all_sequences.push_back(current_sequence);
        return;
    }

    // Try each execution time for the current task
    for (double exec_time : task_options[task_idx]) {
        current_sequence.push_back(exec_time);
        enumerate_options(all_sequences, task_options, current_sequence, task_idx + 1);
        current_sequence.pop_back();  // Backtrack
    }
}

std::vector<std::vector<double>> enumerate_all_exeTOptions(
    const DAG_Model& dag_tasks) {

    double util_regular_tasks = 0.0;
    std::vector<double> prds;
    for (int i=0;i<dag_tasks.tasks.size();i++) {
        double prd = dag_tasks.tasks[i].period;
        prds.push_back(prd);
        if (dag_tasks.tasks[i].timePerformancePairs.size() == 0) {
            double mu = dag_tasks.tasks[i].getExecGaussian().mu;
            util_regular_tasks += mu/prd;
        }
    }
    //printf("Ryan %s: util_regular_tasks=%.4f\n",__func__,util_regular_tasks);

    std::vector<std::vector<double>> time_limit_option_for_each_task = RecordTimeLimitOptions(dag_tasks);

    std::vector<double> exeT_sequences;
    std::vector<std::vector<double>> exeT_all_sequences;
    enumerate_options(exeT_all_sequences,time_limit_option_for_each_task, exeT_sequences, 0);
    //printf("%d sequences\n",exeT_all_sequences.size());

    std::vector<std::pair<std::vector<double>, double>> sequences;
    for (int i=0;i<exeT_all_sequences.size();i++) {
        double u = util_regular_tasks;
        for (int k=0;k<exeT_all_sequences[i].size();k++) {
            if (exeT_all_sequences[i][k]>0) {
                u += exeT_all_sequences[i][k]/prds[k];
            } 
            //printf("%.2f ",exeT_all_sequences[i][k]);
        }
        // sequences.push_back({exeT_all_sequences[i],u});

        insert_sorted(sequences, exeT_all_sequences[i], u);

        //printf("\n%d: util=%f\n",i,u);
    }
 
    for (int i=0;i<sequences.size();i++) {
        //printf("%d util=%.2f\n",i,sequences[i].second);
        exeT_all_sequences[i] = sequences[i].first;
        //for (int k=0;k<exeT_all_sequences[i].size();k++) {
        //    printf("%.2f ",exeT_all_sequences[i][k]);
        //}
        //printf("\n");

    }

    return exeT_all_sequences;
}

// RYAN_20250309

void OptimizePA_with_TimeLimitsStatus::OptimizeDo(std::vector<double>& time_limit_for_task) {
    // call when (trav_task_index == time_limit_option_for_each_task.size()) 
    #ifdef RYAN_HE_CHANGE_DEBUG
    if (GlobalVariables::debugMode & DBG_PRT_MSK_OptimizeSP_TL_BF)
        std::cout << "####OptimizePA_with_TimeLimitsStatus::Optimize: "
                     "EVALUATE PA ..."
                  << std::endl;
    #endif
    #if 0
    printf("%s Ryan: OptimizePA_with_TimeLimitsStatus::Optimize...\n",__func__);
    for (int i=0;i<time_limit_for_task.size();i++) {
        printf("%.2f ",time_limit_for_task[i]);
    }
    printf("\n");
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
}

void OptimizePA_with_TimeLimitsStatus::Optimize(
    uint trav_task_index, std::vector<double>& time_limit_for_task) {
    if (ifTimeout(start_time_)) return;
    if (trav_task_index == time_limit_option_for_each_task.size()) {
        OptimizeDo(time_limit_for_task);
        return;
    } else {
        std::vector<double>& time_limit_options =
            time_limit_option_for_each_task[trav_task_index];

        for (double option : time_limit_options) {
            //printf("Ryan %s: trav_task_index=%d, option=%.2f\n",__func__,trav_task_index,option);
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

ResourceOptResult EnumeratePA_with_TimeLimits_sortOptionsFirst(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters) {

    #ifdef RYAN_HE_CHANGE_DEBUG
    if (GlobalVariables::debugMode & DBG_PRT_MSK_OptimizeSP_TL_BF)
        std::cout << "####EnumeratePA_with_TimeLimits: call "
                     "OptimizePA_with_TimeLimitsStatus.optimize ..."
                  << std::endl;
    #endif

    OptimizePA_with_TimeLimitsStatus optimizer(dag_tasks, sp_parameters);

    // find all sequences
    std::vector<std::vector<double>> options = enumerate_all_exeTOptions(dag_tasks);
    for (int i=0;i<options.size();i++) {
        printf("Ryan %s %d ...\n",__func__,i);
        for (int k=0;k<options[i].size();k++) {
            printf("%.2f ",options[i][k]);
        }
        printf("\n");
        optimizer.OptimizeDo(options[i]);
        if (ifTimeout(optimizer.start_time_)) break;
    }

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

ResourceOptResult EnumeratePA_with_TimeLimits(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters) {
#ifdef RYAN_HE_CHANGE_DEBUG
    if (GlobalVariables::debugMode & DBG_PRT_MSK_OptimizeSP_TL_BF)
        std::cout << "####EnumeratePA_with_TimeLimits: call "
                     "OptimizePA_with_TimeLimitsStatus.optimize ..."
                  << std::endl;
#endif
    OptimizePA_with_TimeLimitsStatus optimizer(dag_tasks, sp_parameters);
    //printf("Ryan %s: res_opt.sp_opt=%f\n",__func__,optimizer.res_opt.sp_opt);
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
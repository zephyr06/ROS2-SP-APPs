#include "sources/Optimization/OptimizeSP_TL_Incre.h"

namespace SP_OPT_PA {

// task_options is time_limit_option_for_each_task
void enumerate_options(std::vector<std::vector<double>>& all_sequences,
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
        enumerate_options(all_sequences, task_options, current_sequence,
                          task_idx + 1);
        current_sequence.pop_back();  // Backtrack
    }
}

std::vector<std::vector<double>> enumerate_all_exeTOptions(
    const DAG_Model& dag_tasks) {
    double util_regular_tasks = 0.0;
    std::vector<double> prds;
    for (int i = 0; i < dag_tasks.tasks.size(); i++) {
        double prd = dag_tasks.tasks[i].period;
        prds.push_back(prd);
        if (dag_tasks.tasks[i].timePerformancePairs.size() == 0) {
            double mu = dag_tasks.tasks[i].getExecGaussian().mu;
            util_regular_tasks += mu / prd;
        }
    }
    // printf("Ryan %s: util_regular_tasks=%.4f\n",__func__,util_regular_tasks);

    std::vector<std::vector<double>> time_limit_option_for_each_task =
        RecordTimeLimitOptions(dag_tasks);

    std::vector<double> exeT_sequences;
    std::vector<std::vector<double>> exeT_all_sequences;
    enumerate_options(exeT_all_sequences, time_limit_option_for_each_task,
                      exeT_sequences, 0);
    // printf("%d sequences\n",exeT_all_sequences.size());

    std::vector<std::pair<std::vector<double>, double>> sequences;
    for (int i = 0; i < exeT_all_sequences.size(); i++) {
        double u = util_regular_tasks;
        for (int k = 0; k < exeT_all_sequences[i].size(); k++) {
            if (exeT_all_sequences[i][k] > 0) {
                u += exeT_all_sequences[i][k] / prds[k];
            }
            // printf("%.2f ",exeT_all_sequences[i][k]);
        }
        // sequences.push_back({exeT_all_sequences[i],u});

        insert_sorted(sequences, exeT_all_sequences[i], u);

        // printf("\n%d: util=%f\n",i,u);
    }

    for (int i = 0; i < sequences.size(); i++) {
        // printf("%d util=%.2f\n",i,sequences[i].second);
        exeT_all_sequences[i] = sequences[i].first;
        // for (int k=0;k<exeT_all_sequences[i].size();k++) {
        //     printf("%.2f ",exeT_all_sequences[i][k]);
        // }
        // printf("\n");
    }

    return exeT_all_sequences;
}

}  // namespace SP_OPT_PA
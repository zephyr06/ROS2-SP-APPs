#pragma once
#include "sources/Optimization/OptimizeSP_Incre.h"
#include "sources/Optimization/OptimizeSP_TL_BF.h"
#include "sources/Safety_Performance_Metric/SP_Metric.h"

namespace SP_OPT_PA {

std::vector<std::vector<double>> RecordCloseTimeLimitOptions(
    const DAG_Model& dag_tasks);

struct HashKey4Vector {
    std::size_t operator()(const std::vector<double>& v) const {
        std::size_t seed = v.size();
        for (auto& i : v) {
            seed ^=
                std::hash<double>()(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

struct TaskSortingHeuristic {
    const DAG_Model& dag_tasks;
    const SP_Parameters& sp_parameters;

    bool operator()(size_t idx1, size_t idx2) const {
        const auto& t1 = dag_tasks.tasks[idx1];
        const auto& t2 = dag_tasks.tasks[idx2];
        
        double w1 = 1.0;
        if (sp_parameters.weights_node.count(t1.id)) {
            w1 = sp_parameters.weights_node.at(t1.id);
        }
        double w2 = 1.0;
        if (sp_parameters.weights_node.count(t2.id)) {
            w2 = sp_parameters.weights_node.at(t2.id);
        }

        if (w1 != w2) {
            return w1 > w2;
        }

        double th1 = 0.5;
        if (sp_parameters.thresholds_node.count(t1.id)) {
            th1 = sp_parameters.thresholds_node.at(t1.id);
        }
        double th2 = 0.5;
        if (sp_parameters.thresholds_node.count(t2.id)) {
            th2 = sp_parameters.thresholds_node.at(t2.id);
        }

        if (th1 != th2) {
            return th1 < th2;
        }

        return t1.id < t2.id;
    }
};

class OptimizePA_Incre_with_TimeLimits : public OptimizePA_Incre {
   public:
   OptimizePA_Incre_with_TimeLimits(){};
    OptimizePA_Incre_with_TimeLimits(const DAG_Model& dag_tasks,
                                     const SP_Parameters& sp_parameters)
        : OptimizePA_Incre(dag_tasks, sp_parameters) {
        time_limit_option_for_each_task_ = RecordTimeLimitOptions(dag_tasks_);
    }

    PriorityVec OptimizeFromScratch_w_TL(int K);

    PriorityVec OptimizeIncre_w_TL(const DAG_Model& dag_tasks_update, int K);

    void UpdateRecords(const OptimizePA_Incre& optimizer,
                       const std::vector<double>& time_limits);

    double EvaluateTimeLimitConfig(int K, const std::vector<double>& time_limits);

    inline ResourceOptResult CollectResults() const { return res_opt_; }

    // data members
    ResourceOptResult res_opt_;
    std::vector<std::vector<double>> time_limit_option_for_each_task_;
    // For each task id, it maps time limit to the optimizer
    std::unordered_map<std::vector<double>, OptimizePA_Incre, HashKey4Vector>
        timelimit2optimizer_;
};

inline PriorityVec PerformOptimizePA_Incre_w_TimeLimits(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    return opt.OptimizeFromScratch(
        GlobalVariables::Layer_Node_During_Incremental_Optimization);
}
}  // namespace SP_OPT_PA
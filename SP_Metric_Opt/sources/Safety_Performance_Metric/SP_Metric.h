#pragma once
#include "sources/Safety_Performance_Metric/ParametersSP.h"
#include "sources/Safety_Performance_Metric/Probability.h"
#include "sources/Safety_Performance_Metric/RTA.h"
#include "sources/Safety_Performance_Metric/RTDA_Prob.h"
#include "sources/TaskModel/RegularTasks.h"

namespace SP_OPT_PA {

std::vector<double> GetChainsDDL(const DAG_Model& dag_tasks);

// double ObtainSP(const std::vector<FiniteDist>& dists,
//                 const std::vector<double>& deadline,
//                 const std::vector<double>& ddl_miss_thresholds,
//                 const std::vector<double>& weights);

double ObtainSP(const FiniteDist& dist, double deadline,
                double ddl_miss_threshold, double weight);

inline double PenaltyFunc(double violate_probability, double threshold) {
    return -0.01 * exp(10 * abs(threshold - violate_probability));
}
inline double RewardFunc(double violate_probability, double threshold) {
    return log((threshold - violate_probability) + 1);
}
inline double SP_Func(double violate_probability, double threshold) {
    if (threshold >= violate_probability)
        return RewardFunc(violate_probability, threshold) - 0.5;
    else
        return PenaltyFunc(violate_probability, threshold) - 0.5;
}

inline double interpolate(double x, double x1, double y1, double x2,
                          double y2) {
    return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

// timePerformancePairs is required to be sorted by time
double GetPerfTerm(const std::vector<TimePerfPair>& timePerformancePairs,
                   double time_limit);

double ObtainSP_TaskSet(const TaskSet& tasks,
                        const SP_Parameters& sp_parameters);

double ObtainSP_DAG(const DAG_Model& dag_tasks,
                    const SP_Parameters& sp_parameters);

// assume the order of all the vectors are matched!!!
double ObtainSP_DAG_From_Dists(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters,
    const std::vector<FiniteDist>& node_rts_dists,
    const std::vector<FiniteDist>& path_latency_dists);
}  // namespace SP_OPT_PA
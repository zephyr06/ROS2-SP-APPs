#pragma once
#include "sources/Safety_Performance_Metric/ParametersSP.h"
#include "sources/Safety_Performance_Metric/Probability.h"
#include "sources/Safety_Performance_Metric/RTA.h"
#include "sources/Safety_Performance_Metric/RTDA_Prob.h"
#include "sources/TaskModel/RegularTasks.h"
#include "sources/Utils/Parameters.h"

namespace SP_OPT_PA {

std::vector<double> GetChainsDDL(const DAG_Model& dag_tasks);

inline double interpolate(double x, double x1, double y1, double x2,
                          double y2) {
    return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}
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

#if defined(RYAN_HE_CHANGE)
double SP_Func(double violate_probability, double threshold);
#else
inline double SP_Func(double violate_probability, double threshold) {
    double min_val, max_val, val;
    min_val = PenaltyFunc(1, threshold);
    max_val = RewardFunc(0, threshold);
    if (threshold >= violate_probability) {
        val = RewardFunc(violate_probability, threshold);
    } else {
        val = PenaltyFunc(violate_probability, threshold);
    }
    return interpolate(val, min_val, 0, max_val, 1);
}
#endif

// timePerformancePairs is required to be sorted by time
double GetPerfTerm(const std::vector<TimePerfPair>& timePerformancePairs,
                   double time_limit);

double GetTaskPerfTerm(
    double ext_time_single,
    const std::vector<TimePerfPair>& timePerformancePairs_Sorted);

double GetAvgTaskPerfTerm(std::string& ext_file_path,
                          std::vector<TimePerfPair> timePerformancePairs);

double ObtainSP_TaskSet(const TaskSet& tasks,
                        const SP_Parameters& sp_parameters);

double ObtainSP_DAG(const DAG_Model& dag_tasks,
                    const SP_Parameters& sp_parameters);

// assume the order of all the vectors are matched!!!
double ObtainSP_DAG_From_Dists(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters,
    const std::vector<FiniteDist>& node_rts_dists,
    const std::vector<FiniteDist>& path_latency_dists);

double ObtainSPFromRTAFiles(std::string& slam_path, std::string& rrt_path,
                            std::string& mpc_path, std::string& tsp_path,
                            std::string& tsp_ext_path, std::string& chain0_path,
                            std::string& file_path_ref
#if defined(RYAN_HE_CHANGE)
                            , int dbg = 0
#endif
                            );

#if defined(RYAN_HE_CHANGE)
double ObtainSPFromRTAFiles2(std::string& file_path_ref, std::string& data_dir, int dbg );
#endif

}  // namespace SP_OPT_PA
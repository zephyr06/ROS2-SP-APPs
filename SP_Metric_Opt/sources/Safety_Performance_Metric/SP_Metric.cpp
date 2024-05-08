
#include "sources/Safety_Performance_Metric/SP_Metric.h"

#include "sources/Utils/readwrite.h"
namespace SP_OPT_PA {
std::vector<double> GetChainsDDL(const DAG_Model& dag_tasks) {
    // std::vector<double> chains_ddl(dag_tasks.chains_.size(),
    //                                HyperPeriod(dag_tasks.tasks));
    return dag_tasks.chains_deadlines_;
}
double ObtainSP(const FiniteDist& dist, double deadline,
                double ddl_miss_threshold, double weight) {
    double ddl_miss_chance = GetDDL_MissProbability(dist, deadline);
    return SP_Func(ddl_miss_chance, ddl_miss_threshold) * weight;
}

// timePerformancePairs is required to be sorted by time
double GetPerfTerm(const std::vector<TimePerfPair>& timePerformancePairs,
                   double time_limit) {
    // Find the two time-performance pairs that surround the given time limit
    auto it = std::upper_bound(timePerformancePairs.begin(),
                               timePerformancePairs.end(), time_limit,
                               [](double time, const TimePerfPair& pair) {
                                   return time < pair.time_limit;
                               });

    if (it == timePerformancePairs.begin()) {
        // The given time limit is smaller than the smallest time in the pairs
        return 0.0;
    } else if (it == timePerformancePairs.end()) {
        // The given time limit is larger than the largest time in the pairs
        return timePerformancePairs.back().performance;
    } else {
        // The given time limit is between two time-performance pairs
        auto prevPair = std::prev(it);
        return interpolate(time_limit, prevPair->time_limit,
                           prevPair->performance, it->time_limit,
                           it->performance);
    }
}
// double ObtainSP(const std::vector<FiniteDist>& dists,
//                 const std::vector<double>& deadline,
//                 const std::unordered_map<int, double>& ddl_miss_thresholds,
//                 const std::unordered_map<int, double>& weights) {
//     int n = dists.size();
//     double sp_overall = 0;
//     for (int i = 0; i < n; i++) {
//         double ddl_miss_chance = GetDDL_MissProbability(dists[i],
//         deadline[i]); sp_overall +=
//             SP_Func(ddl_miss_chance, ddl_miss_thresholds[i]) * weights[i];
//     }
//     return sp_overall;
// }

double ObtainSP_TaskSet(const TaskSet& tasks,
                        const SP_Parameters& sp_parameters) {
    std::vector<FiniteDist> rtas = ProbabilisticRTA_TaskSet(tasks);
    // std::vector<double> deadlines = GetParameter<double>(tasks, "deadline");
    // return ObtainSP(rtas, deadlines, sp_parameters.thresholds_node,
    //                 sp_parameters.weights_node);
    double sp_overall = 0;
    for (int i = 0; i < tasks.size(); i++) {
        int task_id = tasks[i].id;
        double ddl_miss_chance =
            GetDDL_MissProbability(rtas[i], tasks[i].deadline);
        sp_overall += SP_Func(ddl_miss_chance,
                              sp_parameters.thresholds_node.at(task_id)) *
                      sp_parameters.weights_node.at(task_id);
    }
    return sp_overall;
}

double ObtainSP_DAG(const DAG_Model& dag_tasks,
                    const SP_Parameters& sp_parameters) {
    if (GlobalVariables::debugMode == 1) BeginTimer("ObtainSP_DAG");
    double sp_overall = ObtainSP_TaskSet(dag_tasks.tasks, sp_parameters);

    std::vector<FiniteDist> reaction_time_dists =
        GetRTDA_Dist_AllChains<ObjReactionTime>(dag_tasks);
    std::vector<double> chains_ddl = GetChainsDDL(dag_tasks);

    for (int i = 0; i < reaction_time_dists.size(); i++) {
        int chain_id = i;
        double ddl_miss_chance =
            GetDDL_MissProbability(reaction_time_dists[i], chains_ddl[i]);
        sp_overall += SP_Func(ddl_miss_chance,
                              sp_parameters.thresholds_path.at(chain_id)) *
                      sp_parameters.weights_path.at(chain_id);
    }

    if (GlobalVariables::debugMode == 1) EndTimer("ObtainSP_DAG");
    return sp_overall;
}

double ObtainSP_DAG_From_Dists(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters,
    const std::vector<FiniteDist>& node_rts_dists,
    const std::vector<FiniteDist>& path_latency_dists) {
    double sp_overall = 0;
    for (uint i = 0; i < dag_tasks.tasks.size(); i++) {
        int task_id = dag_tasks.tasks[i].id;
        sp_overall += ObtainSP(node_rts_dists[i], dag_tasks.tasks[i].deadline,
                               sp_parameters.thresholds_node.at(task_id),
                               sp_parameters.weights_node.at(task_id));
    }
    for (uint i = 0; i < dag_tasks.chains_.size(); i++) {
        sp_overall +=
            ObtainSP(path_latency_dists[i], dag_tasks.chains_deadlines_[i],
                     sp_parameters.thresholds_path.at(i),
                     sp_parameters.weights_path.at(i));
    }
    return sp_overall;
}

double GetTaskPerfTerm(
    double ext_time_single,
    const std::vector<TimePerfPair>& timePerformancePairs_Sorted) {
    auto itr = std::lower_bound(
        timePerformancePairs_Sorted.begin(), timePerformancePairs_Sorted.end(),
        ext_time_single, [](const TimePerfPair& pair, double ext_time_single) {
            return pair.time_limit < ext_time_single;
        });
    if (itr == timePerformancePairs_Sorted.end()) {
        return timePerformancePairs_Sorted.back().performance;
    }
    if (itr == timePerformancePairs_Sorted.begin()) {  // should never happen
        return timePerformancePairs_Sorted.begin()->performance;
    }
    if (ext_time_single == itr->time_limit) return itr->performance;
    auto itr_prev = itr - 1;
    if (itr_prev->time_limit <= ext_time_single &&
        ext_time_single < itr->time_limit) {
        return itr_prev->performance;
    } else {
        CoutError(
            "Input time performance pairs are not sorted based on time!\n");
    }
    return 0;
}

double GetAvgTaskPerfTerm(std::string& ext_file_path,
                          std::vector<TimePerfPair> timePerformancePairs) {
    if (timePerformancePairs.size() == 0) {
        CoutError("timePerformancePairs is empty!");
        return 1.0;
    }
    std::vector<double> ext_times = ReadTxtFile(ext_file_path);
    int n = ext_times.size();
    double avg_perf_coeff = 0;
    for (int i = 0; i < n; i++) {
        avg_perf_coeff += GetTaskPerfTerm(ext_times[i], timePerformancePairs);
    }
    return avg_perf_coeff / n;
}
}  // namespace SP_OPT_PA
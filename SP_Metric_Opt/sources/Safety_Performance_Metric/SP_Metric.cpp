
#include "sources/Safety_Performance_Metric/SP_Metric.h"

#include "sources/Optimization/OptimizeSP_Base.h"  // UpdateTaskSetPriorities
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
    // Find the first pair whose time limit is strictly greater than the query.
    auto it = std::upper_bound(timePerformancePairs.begin(),
                               timePerformancePairs.end(), time_limit,
                               [](double time, const TimePerfPair& pair) {
                                   return time < pair.time_limit;
                               });

    if (it == timePerformancePairs.begin()) {
        // The given time limit is smaller than the smallest time in the pairs
        return 0.0;
    } else if (it == timePerformancePairs.end()) {
        // The given time limit is >= the largest time in the pairs
        return timePerformancePairs.back().performance;
    } else {
        // Return the performance of the closest entry whose time limit is
        // <= the query (floor behavior).
        return std::prev(it)->performance;
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
    double sp_overall = 0;
    for (int i = 0; i < tasks.size(); i++) {
        // P1.14 — cooperative cancel: the per-task RTA above can be expensive
        // (wide ET distributions -> large convolution support); poll the BF
        // shared budget between tasks so a runaway eval is interruptible.
        // No-op outside a BF search (BFSharedBudgetCancelled() is false).
        if (BFSharedBudgetCancelled()) return sp_overall;
        int task_id = tasks[i].id;
        double ddl_miss_chance =
            GetDDL_MissProbability(rtas[i], tasks[i].deadline);
        double perf_coefficient = tasks[i].GetPerfCoefficient();
        sp_overall += SP_Func(ddl_miss_chance,
                              sp_parameters.thresholds_node.at(task_id)) *
                      sp_parameters.weights_node.at(task_id) *
                      perf_coefficient;
    }
    return sp_overall;
}

TaskSet ApplyTimeLimitsToTasksExecutionTime(
    const TaskSet& tasks, const std::vector<double>& time_limits) {
    TaskSet tasks_upd = tasks;
    for (int i = 0; i < static_cast<int>(tasks.size()); i++) {
        if (time_limits[i] != -1) {
            tasks_upd[i].execution_time_dist =
                GetUnitExecutionTimeDist(time_limits[i]);
        }
    }
    return tasks_upd;
}

double ObtainSP_TaskSet_And_TimeLimits(const TaskSet& tasks,
                                       const SP_Parameters& sp_parameters,
                                       const std::vector<double>& time_limits) {
    return ObtainSP_TaskSet(
        ApplyTimeLimitsToTasksExecutionTime(tasks, time_limits), sp_parameters);
}

double ObtainSP_DAG(const DAG_Model& dag_tasks,
                    const SP_Parameters& sp_parameters) {
    if (GlobalVariables::debugMode == 1) {
        BeginTimer("ObtainSP_DAG");
    }
    double sp_overall = ObtainSP_TaskSet(dag_tasks.tasks, sp_parameters);

    // P1.14 — cooperative cancel: if the per-task RTA above (or the budget
    // check inside ObtainSP_TaskSet) already saw the BF budget exhausted,
    // skip the expensive per-chain RTDA convolution and return immediately.
    // EvaluateSPWithPriorityVec discards the partial result via its own
    // post-call BFSharedBudgetCancelled() check.
    if (BFSharedBudgetCancelled()) {
        if (GlobalVariables::debugMode == 1)
            EndTimer("ObtainSP_DAG");
        return sp_overall;
    }

    std::vector<FiniteDist> reaction_time_dists =
        GetRTDA_Dist_AllChains<ObjReactionTime>(dag_tasks);
    std::vector<double> chains_ddl = GetChainsDDL(dag_tasks);

    for (int i = 0; i < reaction_time_dists.size(); i++) {
        // P1.14 — poll between chains as well: GetRTDA_Dist_AllChains is
        // computed eagerly above, but the per-chain ddl-miss probability
        // loop is a natural cancellation point and keeps the cancel
        // responsive if the chain list is long.
        if (BFSharedBudgetCancelled()) break;
        int chain_id = i;
        double ddl_miss_chance =
            GetDDL_MissProbability(reaction_time_dists[i], chains_ddl[i]);
        sp_overall += SP_Func(ddl_miss_chance,
                              sp_parameters.thresholds_path.at(chain_id)) *
                      sp_parameters.weights_path.at(chain_id);
    }

    if (GlobalVariables::debugMode == 1)
        EndTimer("ObtainSP_DAG");
    return sp_overall;
}

double ObtainSP_DAG(const DAG_Model& dag_tasks,
                    const SP_Parameters& sp_parameters,
                    const std::vector<double>& time_limits) {
    DAG_Model dag_tasks_upd = dag_tasks;
    // for (int i = 0; i < static_cast<int>(dag_tasks.tasks.size()); i++) {
    //     if (time_limits[i] != -1) {
    //         dag_tasks_upd.tasks[i].execution_time_dist =
    //             GetUnitExecutionTimeDist(time_limits[i]);
    //     }
    // }
    dag_tasks_upd.tasks =
        ApplyTimeLimitsToTasksExecutionTime(dag_tasks.tasks, time_limits);
    return ObtainSP_DAG(dag_tasks_upd, sp_parameters);
}

double ObtainSP_DAG_From_Dists(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters,
    const std::vector<FiniteDist>& node_rts_dists,
    const std::vector<FiniteDist>& path_latency_dists) {
    double sp_overall = 0;
    for (uint i = 0; i < dag_tasks.tasks.size(); i++) {
        int task_id = dag_tasks.tasks[i].id;
        double perf_coefficient = dag_tasks.tasks[i].GetPerfCoefficient();
        sp_overall +=
            ObtainSP(node_rts_dists[i], dag_tasks.tasks[i].deadline,
                     sp_parameters.thresholds_node.at(task_id),
                     sp_parameters.weights_node.at(task_id)) *
            perf_coefficient;
    }
    for (uint i = 0; i < dag_tasks.chains_.size(); i++) {
        sp_overall +=
            ObtainSP(path_latency_dists[i], dag_tasks.chains_deadlines_[i],
                     sp_parameters.thresholds_path.at(i),
                     sp_parameters.weights_path.at(i));
    }
    return sp_overall;
}

// P1.13 — cache-path drop-in for the oracle `EvaluateSPWithPriorityVec` body.
// Mirrors it EXACTLY (bake TL → apply pa → ObtainSP_DAG) except the per-node RTA
// is the caller-supplied `node_rtas` (an RTACache return) instead of a fresh
// `ProbabilisticRTA_TaskSet`. Chain terms recomputed via
// `GetRTDA_Dist_AllChains` (no cache win on chains — Q4; matches `ObtainSP_DAG`).
// `perf_coefficient` is handled uniformly inside `ObtainSP_DAG_From_Dists`
// (Hazard B fixed in place — no separate perf-coeff variant needed).
double ObtainSP_Full_From_NodeRTAs(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters,
    const std::vector<int>& priority_assignment,
    const std::vector<double>& tl,
    const std::vector<FiniteDist>& node_rtas) {
    TaskSet tasks_baked =
        ApplyTimeLimitsToTasksExecutionTime(dag_tasks.tasks, tl);
    TaskSet tasks_prioritized =
        UpdateTaskSetPriorities(tasks_baked, priority_assignment);
    DAG_Model dag_tasks_eval = dag_tasks;
    dag_tasks_eval.tasks = tasks_prioritized;

    std::vector<FiniteDist> reaction_time_dists =
        GetRTDA_Dist_AllChains<ObjReactionTime>(dag_tasks_eval);

    return ObtainSP_DAG_From_Dists(
        dag_tasks_eval, sp_parameters, node_rtas, reaction_time_dists);
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
    if (ext_time_single == itr->time_limit)
        return itr->performance;
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
    if (n == 0) {
        // CoutWarning("ext_file_path is empty!");
        return 0.0;
    }
    double avg_perf_coeff = 0;
    for (int i = 0; i < n; i++) {
        avg_perf_coeff += GetTaskPerfTerm(ext_times[i], timePerformancePairs);
    }
    return avg_perf_coeff / n;
}

double ObtainSPFromRTAFiles(std::string& slam_path, std::string& rrt_path,
                            std::string& mpc_path, std::string& tsp_path,
                            std::string& tsp_ext_path, std::string& chain0_path,
                            std::string& file_path_ref) {
    int granularity = GlobalVariables::Granularity;
    DAG_Model dag_tasks = ReadDAG_Tasks(file_path_ref);

    SP_Parameters sp_parameters = ReadSP_Parameters(file_path_ref);
    // Load TSP's MEASURED execution-time samples (tsp_ext_path) into TSP's
    // execution_time_dist BEFORE scoring. The node-RTA files (tsp_path etc.)
    // hold TSP's *response time*; the ET samples here are the input the SP
    // metric's perf coefficient must read. `ObtainSP_DAG_From_Dists` applies
    // `GetPerfCoefficient()` once (Hazard B fixed in place), and
    // `GetPerfCoefficient()` reads `execution_time_dist.GetAvgValue()` — so the
    // dist MUST carry the measured ET, not the analytic Gaussian from the yaml.
    // (The old code instead reduced these samples to a single average perf
    // coefficient and shoved it into TSP's weight slot — the wrong place, now
    // removed.)
    std::vector<double> tsp_ext_times = ReadTxtFile(tsp_ext_path);
    dag_tasks.tasks[0].execution_time_dist =
        FiniteDist(tsp_ext_times, granularity);

    std::vector<FiniteDist> node_rts_dists;
    node_rts_dists.push_back(FiniteDist(ReadTxtFile(tsp_path), granularity));
    node_rts_dists.push_back(FiniteDist(ReadTxtFile(mpc_path), granularity));
    node_rts_dists.push_back(FiniteDist(ReadTxtFile(rrt_path), granularity));
    node_rts_dists.push_back(FiniteDist(ReadTxtFile(slam_path), granularity));

    std::vector<FiniteDist> reaction_time_dists = {
        FiniteDist(ReadTxtFile(chain0_path), granularity)};

    double sp_value_overall = ObtainSP_DAG_From_Dists(
        dag_tasks, sp_parameters, node_rts_dists, reaction_time_dists);
    return sp_value_overall;
}
}  // namespace SP_OPT_PA
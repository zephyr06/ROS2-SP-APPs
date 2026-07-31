#pragma once
#include "sources/Safety_Performance_Metric/ParametersSP.h"
#include "sources/Safety_Performance_Metric/Probability.h"
#include "sources/Safety_Performance_Metric/RTA.h"
#include "sources/Safety_Performance_Metric/RTDA_Prob.h"
#include "sources/TaskModel/RegularTasks.h"

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

// Apply time limits to task execution_time_dist: for each i with
// time_limits[i] != -1, replace execution_time_dist with a unit distribution
// (point mass) at time_limits[i] via GetUnitExecutionTimeDist; -1 leaves the
// task's base dist untouched. Declared here so the per-core RTA cache
// (RTA_Cache.h) can apply TLs when building/patching a cache. Returns a new
// TaskSet (does not mutate the input).
TaskSet ApplyTimeLimitsToTasksExecutionTime(
    const TaskSet& tasks, const std::vector<double>& time_limits);

// Apply time limits to task execution_time_dist (unit distribution at time limit),
// then compute SP. time_limit[i] == -1 means no limit for that task.
double ObtainSP_TaskSet_And_TimeLimits(
    const TaskSet& tasks, const SP_Parameters& sp_parameters,
    const std::vector<double>& time_limits);

double ObtainSP_DAG(const DAG_Model& dag_tasks,
                    const SP_Parameters& sp_parameters);

// Apply time limits to task execution_time_dist (unit distribution at time limit),
// then compute DAG-level SP. time_limit[i] == -1 means no limit for that task.
double ObtainSP_DAG(const DAG_Model& dag_tasks,
                    const SP_Parameters& sp_parameters,
                    const std::vector<double>& time_limits);

// assume the order of all the vectors are matched!!!
// P1.13 (Hazard B): the node loop now multiplies `perf_coefficient`
// (`dag_tasks.tasks[i].GetPerfCoefficient()`), matching the oracle
// `ObtainSP_TaskSet` (SP_Metric.cpp:62,66). Previously this path called
// `ObtainSP`, which OMITS the coefficient — a divergence from the oracle fixed
// in place here so the performance factor is applied exactly once, in the SP
// function, uniformly across all SP-eval paths. `GetPerfCoefficient()` returns
// 1.0 for tasks without `timePerformancePairs`, so the change is a no-op for
// non-perf-eligible tasks.
double ObtainSP_DAG_From_Dists(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters,
    const std::vector<FiniteDist>& node_rts_dists,
    const std::vector<FiniteDist>& path_latency_dists);

// P1.13 — the cache-path drop-in for the oracle `EvaluateSPWithPriorityVec`
// body (OptimizeSP_Base.cpp:148-157). Mirrors it EXACTLY except the per-node RTA
// comes from the caller-supplied `node_rtas` (an RTACache::Evaluate/Initialize
// return) instead of being recomputed via `ProbabilisticRTA_TaskSet`. Steps:
// bake TLs → apply pa (sort) → score nodes with `ObtainSP_DAG_From_Dists`
// (perf_coefficient INCLUDED, Hazard B fixed in place) → add the chain/path SP
// terms.
//
// Path SP is RECOMPUTED per call via `GetRTDA_Dist_AllChains` (NOT served from
// the node RTA cache — chains use the path-latency dist, a different quantity;
// per Q4 there is no cache win on chains, and recomputing them preserves
// bit-identity with no regression). This matches `ObtainSP_DAG` (SP_Metric.cpp:
// 96-107), which also recomputes the chain dists on the prioritized DAG.
//
// CONTRACT (bit-identity): `node_rtas[i]` MUST be the RTA of the task at
// position i in `UpdateTaskSetPriorities(ApplyTimeLimitsToTasksExecutionTime(
// dag_tasks.tasks, tl), pa)` — i.e. the exact vector an RTACache fed the SAME
// (dag_tasks, pa, tl) returns (the cache indexes its flat rta_ over that same
// prioritized vector; see RTA_Cache.cpp:181-183,442-446). The caller re-derives
// nothing here — the bake+prioritize happens inside, identical to the cache's
// internal derivation, so `node_rtas[i]` pairs with `prioritized[i]`.
// `tl[i] == -1` means no TL for task i. Uses `std::vector<int>` (not
// `PriorityVec`) so SP_Metric.h stays a leaf — `PriorityVec` is typedef'd in
// OptimizeSP_Base.h, which includes THIS header (would form a cycle).
double ObtainSP_Full_From_NodeRTAs(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters,
    const std::vector<int>& priority_assignment,
    const std::vector<double>& tl,
    const std::vector<FiniteDist>& node_rtas);

// P0.6 step 4 — the hard per-candidate feasibility gate's pure predicate.
// Answers the user's hard guarantee: under the candidate {priority_assignment,
// tl}, does EVERY important task's probabilistic ddl_miss_chance stay at or
// below its SP threshold (`sp_parameters.thresholds_node[task_id]`)? Returns
// true iff for every important task i:
//     GetDDL_MissProbability(rta_dist_i, deadline_i) <= thresholds_node[i].
// Important = `Task::is_important` (top-50% by sp_weight, set at generation by
// P0.9). With no important tasks the universal quantifier is vacuously true
// (the constraint is free — the design's "seed region" degenerate case).
//
// CALLER PASSES THE ALREADY-COMPUTED PER-TASK RTAs (`node_rtas`). The gate's
// real caller is the TL walk's adoption site, which ALREADY materializes the
// node RTAs when it scores SP — the cache-path SP eval is
// `rta_cache_.Evaluate(dag, pa, tl)` → `ObtainSP_Full_From_NodeRTAs(dag, sp,
// pa, tl, node_rtas)` (`OptimizeSP_TL_Incre.cpp:238-243`). Re-deriving the
// RTAs here (the prior shape A, via `ProbabilisticRTA_TaskSet`) would
// duplicate exactly that work on every ADOPTED candidate → wasteful. So the
// gate mirrors `ObtainSP_Full_From_NodeRTAs`: accept the caller's RTAs.
//
// CONTRACT (identical to `ObtainSP_Full_From_NodeRTAs` above): `node_rtas[i]`
// MUST be the RTA of the task at position i in
// `UpdateTaskSetPriorities(ApplyTimeLimitsToTasksExecutionTime(dag_tasks.tasks,
// tl), priority_assignment)` — i.e. the exact vector an RTACache fed the SAME
// (dag_tasks, pa, tl) returns, OR a fresh `ProbabilisticRTA_TaskSet` of that
// prioritized set. The bake+prioritize happens INSIDE this predicate (same as
// `ObtainSP_Full_From_NodeRTAs`) so `node_rtas[i]` pairs with the prioritized
// task at index i. `tl[i] == -1` means no TL for task i. Uses
// `std::vector<int>` for `priority_assignment` (same cycle-avoidance reason as
// `ObtainSP_Full_From_NodeRTAs` above).
//
// This is the GATE PREDICATE; it does NOT alter SP or the optimizer state. The
// static-solution TL walk adopts an SP-better candidate TL only when this
// returns true (skip + continue otherwise). Pessimistic-bound soundness: the
// metric keeps non-perf Gaussian variance the sim drops and sets perf ET = TL
// = the sim's runtime cap, so the metric's ddl_miss_chance >= the sim's actual
// miss chance → a gate on the metric is a sound guarantee of the runtime
// condition (see `goal.md` "Why a hard gate").
bool ImportantTasksMeetThresholds(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters,
    const std::vector<int>& priority_assignment,
    const std::vector<double>& tl,
    const std::vector<FiniteDist>& node_rtas);

double ObtainSPFromRTAFiles(std::string& slam_path, std::string& rrt_path,
                            std::string& mpc_path, std::string& tsp_path,
                            std::string& tsp_ext_path, std::string& chain0_path,
                            std::string& file_path_ref);
}  // namespace SP_OPT_PA
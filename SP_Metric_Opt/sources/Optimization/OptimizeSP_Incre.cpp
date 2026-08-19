
#include "sources/Optimization/OptimizeSP_Incre.h"

#include "sources/Safety_Performance_Metric/RTA_Cache.h"
#include "sources/Safety_Performance_Metric/SP_Metric.h"

namespace SP_OPT_PA {
// Evaluate returns a const ref into the cache's candidate_rta_ buffer, which
// the next Evaluate overwrites. But the RTA is needed both to score SP AND (on
// adoption) to feed AdoptChampion, so binding the returned ref directly would
// dangle. This struct COPIES the RTA into a stable local. Mirrors the no-raw-
// pointer idiom.
struct NodeRtasHolder {
    std::vector<FiniteDist> rtas;
};

PriorityPartialPath::PriorityPartialPath(const DAG_Model& dag_tasks,
                                         const SP_Parameters& sp_parameters)
    : dag_tasks(dag_tasks), sp_parameters(sp_parameters) {
    tasks_to_assign.reserve(dag_tasks.tasks.size());
    for (int i = 0; i < static_cast<int>(dag_tasks.tasks.size()); i++) {
        tasks_to_assign.insert(i);
    }
}

// return true if rhs is 'better' than lhs
// one priority assignment is better than another priority assignment if
// - it has a larger sp value
// - if the sp values are the same, it has assigns the least-important task
// lowest priority
//     - Note that the low-priority is assigned to the last task added to
//     pa_vec_low_pri

bool CompPriorityPath::operator()(const PriorityPartialPath& lhs,
                                  const PriorityPartialPath& rhs) const {
    if (std::abs((lhs.sp_lost - rhs.sp_lost)) > 5e-2) {
        return lhs.sp_lost > rhs.sp_lost;  // small sp value first
    } else {
        for (int i = 0; i < lhs.pa_vec_lower_pri.size(); i++) {
            if (lhs.pa_vec_lower_pri[i] != rhs.pa_vec_lower_pri[i]) {
                if (lhs.GetTaskWeight(i) != rhs.GetTaskWeight(i)) {
                    return lhs.GetTaskWeight(i) >
                           rhs.GetTaskWeight(i);  // assign low priority
                                                  // to tasks with small
                                                  // weight
                } else {
                    return lhs.GetTaskMinEt(i) <
                           rhs.GetTaskMinEt(i);  // assign low priority
                                                 // to tasks with long ET
                }
            }
        }
        return true;  // should never happen, actually
    }
}

bool PriorityPartialPath::UpdateSP(int task_id) {
    // Poll the shared budget at each node so a from-scratch beam (interval 0 /
    // reopt) can't overrun TIME_LIMIT. On timeout return false: a half-evaluated
    // path whose important-task gate never ran must not be pushed (it would
    // commit an ungated leaf). Inert outside a BFDLSharedBudget scope.
    if (BFSharedBudgetCancelled())
        return false;
    TaskSet hp_tasks;
    hp_tasks.reserve(tasks_to_assign.size());
    for (int task_hp_id : tasks_to_assign) {
        if (dag_tasks.tasks[task_id].processorId ==
            dag_tasks.tasks[task_hp_id].processorId)
            hp_tasks.push_back(dag_tasks.tasks[task_hp_id]);
    }
    FiniteDist rta_curr = GetRTA_OneTask(dag_tasks.tasks[task_id], hp_tasks);
    double perf_coeff = dag_tasks.tasks[task_id].GetPerfCoefficient();
    double weight = 1.0 * sp_parameters.weights_node[task_id];
    double effective_weight = weight * perf_coeff;
    double sp_cur =
        ObtainSP({rta_curr}, {dag_tasks.tasks[task_id].deadline},
                 {sp_parameters.thresholds_node[task_id]}, {effective_weight});
    sp_lost += effective_weight - sp_cur;
    // Decided task's RTA is final (HP set = unassigned same-proc tasks). An
    // important task missing its ddl threshold -> this path is unschedulable.
    if (dag_tasks.tasks[task_id].is_important &&
        GetDDL_MissProbability(rta_curr, dag_tasks.tasks[task_id].deadline) >
            sp_parameters.thresholds_node[task_id])
        return false;
    return true;
}

bool PriorityPartialPath::AssignAndUpdateSP(int task_id) {
    if (tasks_to_assign.count(task_id)) {
        tasks_to_assign.erase(task_id);
        bool schedulable = UpdateSP(task_id);
        pa_vec_lower_pri.push_back(task_id);
        return schedulable;
    }
    CoutError("Task" + std::to_string(task_id) + " already assigned");
    return true;
}

PriorityOptResult OptimizePA_Incre::OptimizeFromScratch(int K) {
    PriorityVec priority_assignments = {};
    int lowest_priority = 0;
    int highest_priority = lowest_priority + N - 1;

    std::vector<PriorityPartialPath> partial_paths(
        1, PriorityPartialPath(dag_tasks_, sp_parameters_));
    partial_paths.reserve(K * N);

    for (int curr_priority = lowest_priority; curr_priority <= highest_priority;
         curr_priority++) {
        std::priority_queue<PriorityPartialPath,
                            std::vector<PriorityPartialPath>, CompPriorityPath>
            pq;
        for (int path_index = 0; path_index < partial_paths.size();
             path_index++) {
            PriorityPartialPath& path = partial_paths[path_index];
            for (int task_id : path.tasks_to_assign) {
                PriorityPartialPath new_path = path;
                if (new_path.AssignAndUpdateSP(task_id)) {
                    pq.push(new_path);
                    if (GlobalVariables::debugMode) {
                        std::cout << "Priority " << curr_priority << ":\n";
                        std::cout << "SP lost: " << new_path.sp_lost << " ";
                        std::cout << "partial paths:\n";
                        for (int j = 0; j < new_path.pa_vec_lower_pri.size();
                             j++) {
                            std::cout << new_path.pa_vec_lower_pri[j] << " ";
                        }
                        std::cout << "\n";
                    }
                }
            }
        }
        partial_paths.clear();
        while (partial_paths.size() < K && (!pq.empty())) {
            partial_paths.push_back(pq.top());
            pq.pop();
        }
        if (partial_paths.empty())
            break;  // every candidate pruned -> beam emptied

        if (GlobalVariables::debugMode) {
            std::cout << "Priority " << curr_priority << ":\n";
            std::cout << "partial paths:\n";
            for (int i = 0; i < partial_paths.size(); i++) {
                std::cout << "SP lost: " << partial_paths[i].sp_lost << " ";
                for (int j = 0; j < partial_paths[i].pa_vec_lower_pri.size();
                     j++) {
                    std::cout << partial_paths[i].pa_vec_lower_pri[j] << " ";
                }
                std::cout << "\n";
            }
        }
    }
    if (partial_paths.empty()) {
        // Every leaf pruned (or budget timed out) -> no schedulable plan.
        opt_pa_.clear();
        opt_sp_ = INT_MIN;
        return {PriorityVec{}, opt_sp_, /*schedulable=*/false};
    }
    PriorityVec res = partial_paths[0].pa_vec_lower_pri;
    std::reverse(res.begin(), res.end());
    opt_pa_ = res;
    double sum_sp_weights = 0;
    for (int i = 0; i < N; i++) {
        sum_sp_weights += sp_parameters_.weights_node[i] *
                          dag_tasks_.tasks[i].GetPerfCoefficient();
    }
    // old method is opt_sp_ = sum_sp_weights - partial_paths[0].sp_lost;
    // But that method doesn't exactly generate the same result as directly
    // calling evaluting SP
    opt_sp_ = EvaluateSPWithPriorityVec(dag_tasks_, sp_parameters_, opt_pa_)
                  .sp_value;
    return {res, opt_sp_, /*schedulable=*/true};
}

std::vector<int> FindTasksWithFlexibleTimeLimits(const DAG_Model& dag_tasks) {
    std::vector<int> res;
    for (int i = 0; i < dag_tasks.tasks.size(); i++) {
        if (!dag_tasks.tasks[i].timePerformancePairs.empty()) {
            res.push_back(i);
        }
    }
    return res;
}

std::vector<DiffObj> FindEnvTaskWithDifferentEt(
    const DAG_Model& dag_tasks, const DAG_Model& dag_tasks_updated) {
    // Type-E (env-changed) diff: FindTaskWithDifferentEt's result MINUS TL-
    // flexible tasks. See the header comment for why the TL-flexible set must
    // be filtered (FiniteDist::operator!= is 10%-relative approx_equal +
    // TL-flexible dists carry raw YAML mu/min/max → non-env inequality).
    std::vector<DiffObj> full =
        FindTaskWithDifferentEt(dag_tasks, dag_tasks_updated);
    std::vector<int> tl_flexible = FindTasksWithFlexibleTimeLimits(dag_tasks);
    std::unordered_set<int> tl_flexible_set(tl_flexible.begin(),
                                            tl_flexible.end());
    std::vector<DiffObj> seq;
    seq.reserve(full.size());
    for (const DiffObj& d : full) {
        if (tl_flexible_set.count(d.task_id) == 0) {
            seq.push_back(d);
        }
    }
    return seq;
}

std::vector<DiffObj> FindTaskWithDifferentEt(
    const DAG_Model& dag_tasks, const DAG_Model& dag_tasks_updated) {
    return FindTaskWithDifferentEt(dag_tasks.tasks, dag_tasks_updated.tasks);
}

std::vector<DiffObj> FindTaskWithDifferentEt(const TaskSet& tasks_base,
                                             const TaskSet& tasks_updated) {
    // P1.30: tight tolerance, NOT FiniteDist::operator!= (loose 1e-1 relative).
    // A TL-baked ET is a point mass at the TL; two close TLs (1100 vs 1200)
    // compare EQUAL under the loose operator== → change missed → stale cache RTA.
    // 1e-6 is effectively exact for TL magnitudes (1100/1200/1500...): it sits
    // ~10 orders above float epsilon (no drift false-positives) and ~3 orders
    // below the coarsest realistic TL-grid step (1100→1101 = 9.1e-4 ≫ 1e-6, so
    // no real TL change is ever merged). Global Value_Proba::operator== 1e-1
    // left untouched (may be load-bearing elsewhere); fix is LOCALIZED here.
    std::vector<DiffObj> seq;
    seq.reserve(tasks_base.size());
    for (int i = 0; i < tasks_base.size(); i++) {
        const FiniteDist& et_base = tasks_base[i].execution_time_dist;
        const FiniteDist& et_updated = tasks_updated[i].execution_time_dist;
        if (!et_base.approx_equal(et_updated, 1e-6)) {
            seq.push_back(DiffObj{
                i, et_base.GetAvgValue() < et_updated.GetAvgValue()});
        }
    }
    return seq;
}

PriorityVec RemoveOneTask(const PriorityVec& pa_vec, int task_id) {
    PriorityVec res = pa_vec;
    bool found = false;
    for (int i = 0; i < static_cast<int>(res.size()); i++) {
        if (res[i] == task_id) {
            res.erase(res.begin() + i);
            found = true;
            break;
        }
    }
    if (!found)
        CoutError("Task not found in RemoveOneTask");
    return res;
}
int GetProrityIndex(const PriorityVec& pa_vec, int task_id) {
    for (int i = 0; i < static_cast<int>(pa_vec.size()); i++) {
        if (pa_vec[i] == task_id) {
            return i;
        }
    }
    CoutError("Task not found in GetProrityIndex");
    return -1;
}

// TODO (efficiency improvements): add a new task under agents/active_Tasks,
// about FindPriorityVec1D_Variations in
// sources/Optimization/OptimizeSP_Incre.cpp. high-level idea is that, if the
// per-core priority assignment doesn't change after we obtain pa_vec_new, then
// there is no need to perform such optimization
std::vector<PriorityVec> FindPriorityVec1D_Variations(
    const PriorityVec& pa_vec, int task_id,
    PriorityChangeStatus priority_change, bool exclude_opt_pa) {
    int old_priority_index = GetProrityIndex(pa_vec, task_id);
    int lb, ub;
    switch (priority_change) {
        case Increase:
            lb = 0;
            ub = old_priority_index;
            break;
        case Decrease:
            lb = old_priority_index;
            ub = pa_vec.size() - 1;
            break;
        case OpenToAll:
            lb = 0;
            ub = pa_vec.size() - 1;
            break;
        default:
            CoutError("Invalid priority_change status");
            break;
    }

    std::vector<PriorityVec> res;
    res.reserve(pa_vec.size());
    PriorityVec pa_vec_ref = RemoveOneTask(pa_vec, task_id);
    for (int i = lb; i <= ub; i++) {
        // The carried position (i == old_priority_index) reconstructs pa_vec
        // exactly; scoring it duplicates the incumbent's baseline SP. Skip it
        // when the caller asked to exclude the carried PA (the default — the
        // sub-incremental scores the carried PA once as its baseline).
        if (exclude_opt_pa && i == old_priority_index)
            continue;
        PriorityVec pa_vec_new = pa_vec_ref;
        pa_vec_new.insert(pa_vec_new.begin() + i, task_id);
        res.push_back(pa_vec_new);
    }
    return res;
}

// TODO: re-evaluate this heuristic, i feel we can do better with
// trial-and-error walk
PriorityChangeStatus AnalyzePriorityChangeStatus(
    const SP_Parameters& sp_parameters, int task_id, bool et_increased) {
    if (sp_parameters.if_highest_weight_unique(task_id))
        return Increase;  // if the task has the highest weight, it should
                          // be assigned the higher priority

    if (et_increased) {
        return Decrease;  // Generally speaking, a task wigh higher ET
                          // should be assigned lwoer priority
    } else {
        return Increase;
    }
}

PriorityOptResult OptimizePA_Incre::OptimizeIncre_SingleTask(
    const DAG_Model& dag_tasks_update, int task_id, bool et_increased,
    RTACacheOpt rta_cache) {
    // Assumes EXACTLY ONE task's ET changed (task_id). Trusts opt_sp_ as the
    // current baseline (caller-set). Re-searches task_id over one half of the
    // priority positions (per AnalyzePriorityChangeStatus), adopting on strict
    // >. Bit-identical to the former :274-292 loop body. Does NOT advance
    // dag_tasks_ (orchestrator-owned).
    std::vector<PriorityVec> pa_vec_variations = FindPriorityVec1D_Variations(
        opt_pa_, task_id,
        AnalyzePriorityChangeStatus(sp_parameters_, task_id, et_increased));
    // Cache path. dag_tasks_update is TL-baked, so feed an all-(-1) tl:
    // ApplyTimeLimitsToTasksExecutionTime is a no-op, the cache sees exactly
    // the final ETs the oracle did → bit-identity. The champion is opt_pa_ on
    // the carried dag (established by OptimizeIncre's baseline Initialize, or a
    // prior adoption here). Each variation moves ONE task's priority position
    // vs opt_pa_ → |diff|<=1 → Evaluate patches the suffix. On a
    // strict-improvement adoption, AdoptChampion MUST advance the champion so
    // the NEXT variation's diff stays |diff|<=1 (Evaluate never advances the
    // champion itself).
    std::vector<double> no_tl(dag_tasks_update.tasks.size(), -1.0);
    for (const PriorityVec& priority_assignment : pa_vec_variations) {
        // Cooperative budget: stop re-searching PA variations once the per-
        // interval TIME_LIMIT expired. Retains the best-so-far opt_pa_/opt_sp_.
        // Inert within budget (incremental path finishes inside it) → prod
        // byte- identical.
        if (BFSharedBudgetCancelled()) {
            break;
        }
        double sp_eval;
        NodeRtasHolder rtas_holder;
        if (rta_cache) {
            // COPY Evaluate's return: the const ref points into the cache's
            // candidate_rta_ buffer, overwritten by the next Evaluate.
            // Stabilize in rtas_holder for both SP scoring and AdoptChampion.
            rtas_holder.rtas = rta_cache->get().Evaluate(
                dag_tasks_update, priority_assignment, no_tl);
            sp_eval = ObtainSP_Full_From_NodeRTAs(
                dag_tasks_update, sp_parameters_, priority_assignment, no_tl,
                rtas_holder.rtas);
        } else {
            sp_eval = EvaluateSPWithPriorityVec(
                          dag_tasks_update, sp_parameters_, priority_assignment)
                          .sp_value;
        }
        PrintPA_IfDebugMode(priority_assignment, sp_eval);
        if (sp_eval > opt_sp_) {
            opt_sp_ = sp_eval;
            opt_pa_ = priority_assignment;
            if (rta_cache) {
                rta_cache->get().AdoptChampion(dag_tasks_update,
                                               priority_assignment, no_tl,
                                               rtas_holder.rtas);
            }
        }
    }
    return {opt_pa_, opt_sp_, /*schedulable=*/true};
}

PriorityOptResult OptimizePA_Incre::OptimizeIncre(const DAG_Model& dag_tasks_update,
                                            double baseline_sp,
                                            RTACacheOpt rta_cache) {
    if (opt_pa_.size() == 0) {
        CoutError("OptimizeIncre called before OptimizeFromScratch");
    }
    // If no cache was provided, create a local one so the cache is ALWAYS
    // engaged for this interval: both the baseline re-score below and the
    // per-variation traversal in OptimizeIncre_SingleTask reuse RTA. The local
    // outlives the loop (same scope), bound via std::ref (no raw pointer). A
    // passed-in cache is used as-is (e.g. shared across intervals). Either way
    // rta_cache is engaged from here on.
    [[maybe_unused]] RTACache local_cache;
    if (!rta_cache) {
        rta_cache = std::ref(local_cache);
    }
    // reset optimal sp
    // baseline_sp (default INT_MIN = "not provided"): score the carried PA
    // under the new env. A caller that already holds that SP may pass it to
    // skip the re-score; if provided it MUST equal
    // EvaluateSPWithPriorityVec(dag_tasks_update, sp_parameters_, opt_pa_).
    //
    // Cache path: the baseline re-score must INITIALIZE the cache champion
    // (full RTA), NOT Evaluate — this is a fresh interval, the carried champion
    // (if any) is on the OLD dag_tasks_ and may differ by >1 task → Evaluate
    // would throw via ComputeTaskSetDifference. Initialize overwrites all prior
    // state and establishes opt_pa_ as the champion the downstream
    // OptimizeIncre_SingleTask 1D variations patch against. tl is all-(-1)
    // (dag_tasks_update is TL-baked). When baseline_sp is provided we trust it
    // (it MUST equal the cache-path score for the same triple) and still
    // Initialize the champion RTA.
    std::vector<double> no_tl(dag_tasks_update.tasks.size(), -1.0);
    if (baseline_sp == INT_MIN) {
        if (rta_cache) {
            const std::vector<FiniteDist>& baseline_rtas =
                rta_cache->get().Initialize(dag_tasks_update, opt_pa_, no_tl);
            opt_sp_ =
                ObtainSP_Full_From_NodeRTAs(dag_tasks_update, sp_parameters_,
                                            opt_pa_, no_tl, baseline_rtas);
        } else {
            opt_sp_ = EvaluateSPWithPriorityVec(dag_tasks_update,
                                                sp_parameters_, opt_pa_)
                          .sp_value;
        }
    } else {
        opt_sp_ = baseline_sp;
        if (rta_cache) {
            rta_cache->get().Initialize(dag_tasks_update, opt_pa_, no_tl);
        }
    }
    // std::cout << "Initial SP before incremental optimziation is: " << opt_sp_
    //           << "\n";
    std::vector<DiffObj> tasks_with_diff_et =
        FindTaskWithDifferentEt(dag_tasks_, dag_tasks_update);

    // Debug seam (debugMode==1, inert in production): emit the changed-task
    // count (ndiff) for this incremental call. A reusable invariant check —
    // the descent-start-TL should collapse perf-pair false positives (ndiff
    // drops to the genuine env-changed count); re-check if the
    // FindTaskWithDifferentEt baseline logic is touched.
    if (GlobalVariables::debugMode == 1) {
        static int probe_call_idx = 0;
        std::cerr << "[INCR-NDIFF-PROBE] call=" << probe_call_idx
                  << " ndiff=" << tasks_with_diff_et.size() << "\n";
        if (probe_call_idx < 2) {
            for (int i = 0; i < dag_tasks_.tasks.size(); i++) {
                bool flagged = (dag_tasks_.tasks[i].execution_time_dist !=
                                dag_tasks_update.tasks[i].execution_time_dist);
                std::cerr
                    << "[INCR-NDIFF-PROBE]   task " << i << " base_avg="
                    << dag_tasks_.tasks[i].execution_time_dist.GetAvgValue()
                    << " upd_avg="
                    << dag_tasks_update.tasks[i]
                           .execution_time_dist.GetAvgValue()
                    << (flagged ? " FLAGGED" : "") << "\n";
            }
        }
        probe_call_idx++;
    }

    for (DiffObj task_diff_obj : tasks_with_diff_et) {
        OptimizeIncre_SingleTask(dag_tasks_update, task_diff_obj.task_id,
                                 task_diff_obj.increase, rta_cache);
    }
    // std::cout << "Optimal SP after  incremental optimziation is: " << opt_sp_
    //           << "\n";
    // Advance this optimizer's dag_tasks_ to the current interval's DAG. This
    // optimizer is a THROWAWAY CHALLENGER (rebuilt from res_opt_ via
    // BuildChallengerFromIncumbent each interval), so this only affects future
    // calls WITHIN this descent — it dies with the local. The cross-interval
    // invariant is the adopted TL in res_opt_.id2time_limit (written by
    // CommitIncumbent); FindTaskWithDifferentEt above already captured the diff
    // against the OLD dag_tasks_, so this only matters for subsequent diffs
    // inside this call.
    dag_tasks_ = dag_tasks_update;
    // schedulable=true: the incremental walk's schedulability is gated by the
    // during-walk gate (UpdateRecords) + backstop, not by this field.
    return {opt_pa_, opt_sp_, /*schedulable=*/true};
}

}  // namespace SP_OPT_PA
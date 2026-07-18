
#include "sources/Optimization/OptimizeSP_Incre.h"

namespace SP_OPT_PA {

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

void PriorityPartialPath::UpdateSP(int task_id) {
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
}

void PriorityPartialPath::AssignAndUpdateSP(int task_id) {
    if (tasks_to_assign.count(task_id)) {
        tasks_to_assign.erase(task_id);
        UpdateSP(task_id);

        pa_vec_lower_pri.push_back(task_id);
    } else
        CoutError("Task" + std::to_string(task_id) + " already assigned");
}

PriorityVec OptimizePA_Incre::OptimizeFromScratch(int K) {
    PriorityVec priority_assignments = {};
    int lowest_priority = 0;
    int highest_priority = lowest_priority + N - 1;

    std::vector<PriorityPartialPath> partial_paths(
        1, PriorityPartialPath(dag_tasks_, sp_parameters_));
    partial_paths.reserve(K);

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
                new_path.AssignAndUpdateSP(task_id);
                pq.push(new_path);
                if (GlobalVariables::debugMode) {
                    std::cout << "Priority " << curr_priority << ":\n";
                    std::cout << "SP lost: " << new_path.sp_lost << " ";
                    std::cout << "partial paths:\n";
                    for (int j = 0; j < new_path.pa_vec_lower_pri.size(); j++) {
                        std::cout << new_path.pa_vec_lower_pri[j] << " ";
                    }
                    std::cout << "\n";
                }
            }
        }
        partial_paths.clear();
        while (partial_paths.size() < K && (!pq.empty())) {
            partial_paths.push_back(pq.top());
            pq.pop();
        }

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
    opt_sp_ = EvaluateSPWithPriorityVec(dag_tasks_, sp_parameters_, opt_pa_);
    return res;
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
    // Type-E (env-changed) diff: FindTaskWithDifferentEt's result MINUS
    // TL-flexible tasks. A TL-flexible task's execution_time_dist is built from
    // raw mu/min/max YAML fields (no adopted-TL override at read), and
    // FiniteDist::operator!= is a 10%-relative approx_equal, so its dist can
    // compare unequal across intervals for non-env (TL/grid) reasons; filtering
    // the TL-flexible set leaves a clean env-only signal. See the header comment.
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
    std::vector<DiffObj> seq;
    seq.reserve(dag_tasks.tasks.size());
    for (int i = 0; i < dag_tasks.tasks.size(); i++) {
        if (dag_tasks.tasks[i].execution_time_dist !=
            dag_tasks_updated.tasks[i].execution_time_dist) {
            if (dag_tasks.tasks[i].execution_time_dist.GetAvgValue() <
                dag_tasks_updated.tasks[i].execution_time_dist.GetAvgValue()) {
                seq.push_back(DiffObj{i, true});
            } else
                seq.push_back(DiffObj{i, false});
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
    if (et_increased) {
        if (sp_parameters.if_highest_weight_unique(task_id))
            return Increase;  // if the task has the highest weight, it should
                              // be assigned the higher priority
        else
            return Decrease;  // Generally speaking, a task wigh higher ET
                              // should be assigned lwoer priority
    } else {
        if (sp_parameters.if_highest_weight_unique(task_id))
            return Decrease;  // if the task has the highest weight, it should
                              // be assigned most of the resource; however, if
                              // it requires less resources, we can assign lower
                              // priority to it
        else
            return Increase;
    }
}

PriorityVec OptimizePA_Incre::OptimizeIncre_SingleTask(
    const DAG_Model& dag_tasks_update, int task_id, bool et_increased) {
    // Assumes EXACTLY ONE task's ET changed (task_id). Trusts opt_sp_ as the
    // current baseline (caller-set: OptimizeIncre scores the carried PA at
    // :243-244, or a TL handler seeds it). Re-searches task_id over one half of
    // the priority positions (per AnalyzePriorityChangeStatus), adopting on
    // strict >. Bit-identical to the former :274-292 loop body. Does NOT
    // advance dag_tasks_ (orchestrator-owned).
    std::vector<PriorityVec> pa_vec_variations = FindPriorityVec1D_Variations(
        opt_pa_, task_id,
        AnalyzePriorityChangeStatus(sp_parameters_, task_id, et_increased));
    for (const PriorityVec& priority_assignment : pa_vec_variations) {
        double sp_eval = EvaluateSPWithPriorityVec(
            dag_tasks_update, sp_parameters_, priority_assignment);
        PrintPA_IfDebugMode(priority_assignment, sp_eval);
        if (sp_eval > opt_sp_) {
            opt_sp_ = sp_eval;
            opt_pa_ = priority_assignment;
        }
    }
    return opt_pa_;
}

PriorityVec OptimizePA_Incre::OptimizeIncre(const DAG_Model& dag_tasks_update,
                                            double baseline_sp) {
    if (opt_pa_.size() == 0) {
        CoutError("OptimizeIncre called before OptimizeFromScratch");
    }
    // reset optimal sp
    // baseline_sp (default INT_MIN = "not provided"): score the carried PA
    // under the new env (the former :243-244 baseline). A caller that already
    // holds that SP may pass it to skip the re-score. If provided it MUST equal
    // EvaluateSPWithPriorityVec(dag_tasks_update, sp_parameters_, opt_pa_).
    opt_sp_ = (baseline_sp == INT_MIN)
                  ? EvaluateSPWithPriorityVec(dag_tasks_update, sp_parameters_,
                                              opt_pa_)
                  : baseline_sp;
    // std::cout << "Initial SP before incremental optimziation is: " << opt_sp_
    //           << "\n";
    std::vector<DiffObj> tasks_with_diff_et =
        FindTaskWithDifferentEt(dag_tasks_, dag_tasks_update);

    // Debug seam (debugMode==1, inert in production): emit the changed-task
    // count (ndiff) for this incremental call. Originally added for the P1.1
    // INCR_Reopt_10 probe (then named INCR_P10) that confirmed the descent-
    // start-TL fix collapses the perf-pair false positives (ndiff 5 -> 0).
    // Kept as a reusable invariant check — re-run the probe if the
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
                                 task_diff_obj.increase);
    }
    // std::cout << "Optimal SP after  incremental optimziation is: " << opt_sp_
    //           << "\n";
    // Advance this optimizer's dag_tasks_ to the current interval's DAG. Under
    // the P0.5 redesign this optimizer is a THROWAWAY CHALLENGER (rebuilt from
    // res_opt_ via BuildChallengerFromIncumbent each incremental interval), so
    // this assignment only affects future calls WITHIN this descent — whatever
    // it does to dag_tasks_ dies with the local. The cross-interval invariant
    // is the adopted TL in res_opt_.id2time_limit (written by CommitIncumbent
    // via UpdateRecords); the next interval's challenger is rebuilt from that,
    // not from a stored DAG. FindTaskWithDifferentEt above already captured the
    // diff against the OLD dag_tasks_, so this assignment only matters for
    // subsequent diffs inside this call.
    dag_tasks_ = dag_tasks_update;
    return opt_pa_;
}
}  // namespace SP_OPT_PA
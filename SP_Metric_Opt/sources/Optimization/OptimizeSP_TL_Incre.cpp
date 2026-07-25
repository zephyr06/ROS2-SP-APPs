
#include "sources/Optimization/OptimizeSP_TL_Incre.h"

#include <algorithm>
#include <numeric>

namespace SP_OPT_PA {

bool TaskSortingHeuristic::operator()(size_t idx1, size_t idx2) const {
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

size_t Find_Close_ExecutionTime(
    const std::vector<TimePerfPair>& time_perf_pairs, double time_limit) {
    if (time_perf_pairs.size() == 0)
        return -1;
    size_t min_diff_index = 0;
    double min_diff = std::abs(time_perf_pairs[0].time_limit - time_limit);
    for (size_t i = 0; i < time_perf_pairs.size(); i++) {
        double diff = std::abs(time_perf_pairs[i].time_limit - time_limit);
        if (diff < min_diff) {
            min_diff = diff;
            min_diff_index = i;
        }
    }
    return min_diff_index;
}

std::vector<std::vector<double>> RecordCloseTimeLimitOptions(
    const DAG_Model& dag_tasks, int radius) {
    std::vector<std::vector<double>> time_limit_option_for_each_task;
    time_limit_option_for_each_task.reserve(dag_tasks.tasks.size());
    for (uint i = 0; i < dag_tasks.tasks.size(); i++) {
        time_limit_option_for_each_task.push_back({});
        size_t close_time_limit_index = Find_Close_ExecutionTime(
            dag_tasks.tasks[i].timePerformancePairs,
            dag_tasks.tasks[i].execution_time_dist.GetAvgValue());
        for (int j = max(0, static_cast<int>(close_time_limit_index) - radius);
             j <= static_cast<int>(close_time_limit_index) + radius &&
             j < static_cast<int>(
                     dag_tasks.tasks[i].timePerformancePairs.size());
             j++) {
            time_limit_option_for_each_task[i].push_back(
                dag_tasks.tasks[i].timePerformancePairs[j].time_limit);
        }
        if (dag_tasks.tasks[i].timePerformancePairs.size() == 0) {
            time_limit_option_for_each_task[i].push_back(-1);
        }
    }
    return time_limit_option_for_each_task;
}

size_t FindTimeLimitOptionIndex(const std::vector<double>& options,
                                double current_val) {
    auto it = std::find(options.begin(), options.end(), current_val);
    if (it == options.end()) {
        return options.size();
    }
    return static_cast<size_t>(std::distance(options.begin(), it));
}

bool IsBetterTimeLimitOption(double new_sp, double current_best_sp, int step) {
    if (new_sp > current_best_sp && !ApproxEqualSP(new_sp, current_best_sp)) {
        return true;
    }
    // Ties (approx-equal SP) prefer the smaller TL. A downward step (step<0)
    // reaches a smaller TL next, so a downward tie is an improvement; an upward
    // tie is rejected to keep the tighter TL already held.
    if (ApproxEqualSP(new_sp, current_best_sp) && step < 0) {
        return true;
    }
    return false;
}

bool OptimizePA_Incre_with_TimeLimits::UpdateRecords(
    const OptimizePA_Incre& optimizer, const std::vector<double>& time_limits) {
    bool should_update = false;
    if (optimizer.opt_sp_ > opt_sp_ &&
        !ApproxEqualSP(optimizer.opt_sp_, opt_sp_)) {
        should_update = true;
    } else if (ApproxEqualSP(optimizer.opt_sp_, opt_sp_)) {
        double sum_new = 0;
        for (double val : time_limits) {
            if (val != -1.0)
                sum_new += val;
        }
        double sum_old = 0;
        for (auto const& [id, val] : res_opt_.id2time_limit) {
            if (val != -1.0)
                sum_old += val;
        }
        if (sum_new < sum_old) {
            should_update = true;
        }
    }

    if (should_update) {
        // CommitIncumbent owns res_opt_ + mirrors; the `optimizer` arg is a
        // throwaway, only its adopted {pa, sp, tl} survives.
        CommitIncumbent(optimizer.opt_pa_, optimizer.opt_sp_, time_limits);

        if (GlobalVariables::debugMode) {
            std::cout << "Time limit: \n";
            for (double time : time_limits) std::cout << time << " ";
            std::cout << "TraverseTimeLimitOptions: "
                      << "opt_sp_ = " << opt_sp_ << std::endl;
        }
    }
    return should_update;
}

double OptimizePA_Incre_with_TimeLimits::EvaluateTimeLimitConfig_ScratchOrIncre(
    int K, const std::vector<double>& time_limits, bool from_scratch) {
    eval_count_++;
    DAG_Model dag_tasks_cur =
        UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits);

    double current_sp = -1.0;
    if (from_scratch) {
        // Reopt: ignore warm state, re-search the full beam.
        OptimizePA_Incre optimizer(dag_tasks_cur, sp_parameters_);
        optimizer.OptimizeFromScratch(K);
        current_sp = optimizer.opt_sp_;
        UpdateRecords(optimizer, time_limits);
    } else if (IfInitialized()) {
        // Incremental: throwaway challenger from res_opt_, then OptimizeIncre.
        // Only the walked task's ET differs — the diff-driven 1D re-search case.
        OptimizePA_Incre optimizer = BuildChallengerFromIncumbent();
        optimizer.OptimizeIncre(dag_tasks_cur);
        current_sp = optimizer.opt_sp_;
        UpdateRecords(optimizer, time_limits);
    } else {
        // Contract violation: the incremental path needs an incumbent to
        // warm-start from, but none exists yet. Bootstrap with a from_scratch
        // call (e.g. ReOptimizePeriodic) first.
        CoutError(
            "EvaluateTimeLimitConfig_ScratchOrIncre: incremental path "
            "(from_scratch=false) requested but no incumbent is initialized. "
            "Bootstrap with a from_scratch call first "
            "(e.g. ReOptimizePeriodic).");
    }
    return current_sp;
}

double OptimizePA_Incre_with_TimeLimits::EvaluateTimeLimitConfig_SubIncremental(
    int K, const std::vector<double>& time_limits, size_t task_idx,
    bool et_increased) {
    eval_count_++;
    (void)K;  // unused: the primitive re-searches one task's 1D positions (no beam)

    // In-walk AdoptChampion calls advance the champion speculatively to trial
    // PAs the walk may not commit. On a REJECT the champion must be reverted,
    // else a later serialized eval diffs committed-PA vs drifted champion
    // (|diff|>1 → throw, or ≤1 → wrong RTA). Accept needs no revert:
    // CommitIncumbent re-adopts on every commit. Snapshot at entry, restore on reject.
    RTACache cache_backup = rta_cache_;

    // Per-eval DAG rebuild is required: the Type-L walk calls this once per
    // trial TL step with a different `time_limits`. The Type-E call (committed
    // TL) double-builds the champion DAG, but that's one call per Type-E entry
    // per interval — negligible, subsumed by the cache's |diff|==0 full-reuse path.
    DAG_Model dag_tasks_cur =
        UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits);

    // Contract: an incumbent must exist (the incremental path warm-starts from
    // res_opt_). Fail loudly rather than silently mis-seeding.
    if (!IfInitialized()) {
        CoutError(
            "EvaluateTimeLimitConfig_SubIncremental: no incumbent is "
            "initialized. The serialized path requires a prior from-scratch "
            "solve (ReOptimizePeriodic) to warm-start from.");
    }

    OptimizePA_Incre challenger = BuildChallengerFromIncumbent();

    // Debug-only invariant: the diff is |diff|<=1 (Type-L: 1, Type-E: 0 — the
    // env move was absorbed into dag_tasks_ before the champion was built, so it
    // cancels). |diff|>1 means the champion drifted, breaking the cache premise.
    AssertSingleChangeInvariant(challenger.dag_tasks_, dag_tasks_cur, task_idx);

    // Re-score the carried PA under dag_tasks_cur so the primitive's strict->
    // adopt test measures against the genuine new-TL baseline (else a TL step
    // that improves only via the carried PA is missed, stopping the walk early).
    // Routed through the RTA cache: Type-L → |diff|==1 (Evaluate patches one
    // task); Type-E → |diff|==0 (FullReuse). Bit-identical to the oracle.
    // Cancel contract: mirror the oracle's INT_MIN on cancel so UpdateRecords'
    // strict-> guard discards a cancelled eval. The cache path has no internal
    // cancel polls, so check both entry and post-Evaluate.
    if (BFSharedBudgetCancelled()) {
        challenger.opt_sp_ = INT_MIN;
    } else {
        const std::vector<FiniteDist>& baseline_rtas =
            rta_cache_.Evaluate(dag_tasks_cur, challenger.opt_pa_, time_limits);

        challenger.opt_sp_ = ObtainSP_Full_From_NodeRTAs(
            dag_tasks_cur, sp_parameters_, challenger.opt_pa_, time_limits,
            baseline_rtas);
    }
    challenger.OptimizeIncre_SingleTask(dag_tasks_cur,
                                        static_cast<int>(task_idx),
                                        et_increased, std::ref(rta_cache_));
    double current_sp = challenger.opt_sp_;
    bool updated = UpdateRecords(challenger, time_limits);
    if (!updated) {
        rta_cache_ = cache_backup;
    }
    return current_sp;
}

// Debug-only invariant check (see header). Throws on a champion drift.
void OptimizePA_Incre_with_TimeLimits::AssertSingleChangeInvariant(
    const DAG_Model& champion_dag, const DAG_Model& candidate_dag,
    size_t task_idx) const {
    if (!GlobalVariables::debugMode)
        return;
    std::vector<DiffObj> diff =
        FindTaskWithDifferentEt(champion_dag, candidate_dag);
    bool ok = true;
    if (diff.size() == 1) {
        ok = (diff[0].task_id == static_cast<int>(task_idx));
    } else if (diff.size() == 0) {
        ok = true;  // Type-E: env move cancels on both sides
    } else {
        ok = false;
    }
    if (!ok) {
        std::string flagged;
        for (size_t i = 0; i < diff.size(); i++) {
            flagged += std::to_string(diff[i].task_id);
            if (i + 1 < diff.size())
                flagged += ",";
        }
        CoutError(
            "Single-change invariant violated: serialized step for "
            "task " +
            std::to_string(task_idx) + " produced |diff|=" +
            std::to_string(diff.size()) + " (flagged tasks: [" + flagged +
            "]). Expected |diff|<=1 (Type-L: 1 on task_idx; Type-E: 0, env "
            "move "
            "cancels on both diff sides). A larger diff means the champion "
            "drifted — multiple ET changes in one eval — which breaks the "
            "sub-incremental's single-change premise and the RTA cache's "
            "single-task RTA patch.");
    }
}

std::vector<int> OptimizePA_Incre_with_TimeLimits::CollectTLFlexibleTaskIds()
    const {
    std::vector<int> res;
    res.reserve(time_limit_option_for_each_task_.size());
    for (size_t i = 0; i < time_limit_option_for_each_task_.size(); i++) {
        const std::vector<double>& opts = time_limit_option_for_each_task_[i];
        // {-1}-only sentinel = no TL freedom; anything else is walkable.
        if (!(opts.size() == 1 && opts[0] == -1.0)) {
            res.push_back(static_cast<int>(i));
        }
    }
    return res;
}

std::vector<SerializedTaskQueueEntry>
OptimizePA_Incre_with_TimeLimits::BuildSerializedTaskQueue(
    const DAG_Model& dag_tasks_prev_pre_tl) const {
    // Type-E: env-changed tasks (pre-TL DAG diff, TL-flexible filtered out).
    // Type-L: TL-flexible tasks (the perf-pair grid).
    std::vector<DiffObj> type_e =
        FindEnvTaskWithDifferentEt(dag_tasks_prev_pre_tl, dag_tasks_);
    std::vector<int> type_l = CollectTLFlexibleTaskIds();

    // E and L must be disjoint (TL-flexible tasks have no env dependence by
    // design); a task in both is a contract violation — hard-fail, no silent pick.
    std::unordered_set<int> type_l_set(type_l.begin(), type_l.end());
    for (const DiffObj& d : type_e) {
        if (type_l_set.count(d.task_id)) {
            CoutError(
                "BuildSerializedTaskQueue: task " + std::to_string(d.task_id) +
                " is both env-changed (Type-E) and TL-flexible (Type-L). The "
                "two sets must be disjoint (TL-flexible tasks have no env "
                "dependence by design) — this signals a generator/contract "
                "violation, not an optimization choice.");
        }
    }

    // Task weight (sort key), mirroring TaskSortingHeuristic; missing → 1.0.
    auto weight_of = [this](int task_idx) {
        int id = dag_tasks_.tasks.at(task_idx).id;
        return sp_parameters_.weights_node.count(id)
                   ? sp_parameters_.weights_node.at(id)
                   : 1.0;
    };

    std::vector<SerializedTaskQueueEntry> queue;
    queue.reserve(type_e.size() + type_l.size());
    for (const DiffObj& d : type_e) {
        // Carry the env-move direction so the handler avoids a per-entry recompute.
        queue.push_back({d.task_id, SerializedTaskQueueEntry::Kind::EnvChanged,
                         d.increase});
    }
    for (int tid : type_l) {
        // TLFlexible: et_increased unused (walk derives per-step direction).
        queue.push_back({tid, SerializedTaskQueueEntry::Kind::TLFlexible});
    }
    // Sort by weight descending. Stable so same-weight E/L keep insertion order.
    std::stable_sort(queue.begin(), queue.end(),
                     [&](const SerializedTaskQueueEntry& a,
                         const SerializedTaskQueueEntry& b) {
                         return weight_of(a.task_id) > weight_of(b.task_id);
                     });
    return queue;
}

void OptimizePA_Incre_with_TimeLimits::PerformSerializedTaskQueueOptimization(
    int K, std::vector<double>& starting_time_limits,
    const DAG_Model& dag_tasks_prev_pre_tl) {
    // Serialized loop replacing the legacy coordinate descent for the INCREMENTAL
    // path: a dedicated baseline re-score (no optimization) seeds opt_sp_, then
    // the queue walk does all optimization in weight order.

    int patience = GlobalVariables::IncrementalTimeLimitSearchPatience;

    // 1. Gate reset (incremental: opt_sp_=-1.0 so the first UpdateRecords
    //    force-commits), then re-arm the cache for this walk body. |diff|<=1
    //    holds for every CommitIncumbent while true, so AdoptChampion stays
    //    consistent with res_opt_ and Evaluate never throws.
    ResetIncumbentBaseline(/*from_scratch=*/false);
    rta_cache_active_ = true;

    // 2. Baseline = dedicated re-score (NO optimization). Must not route through
    //    ScratchOrIncre (which would OptimizeIncre before the queue's order is
    //    honored). Seeds opt_sp_ for the compare-and-keep guard.
    DAG_Model dag_baseline =
        UpdateExtDistBasedOnTimeLimit(dag_tasks_, starting_time_limits);
    double current_config_sp =
        EvaluateSPWithPriorityVec(dag_baseline, sp_parameters_, opt_pa_);
    CommitIncumbent(opt_pa_, current_config_sp, starting_time_limits);

    // 3. Build the merged + weight-sorted E+L queue.
    std::vector<SerializedTaskQueueEntry> queue =
        BuildSerializedTaskQueue(dag_tasks_prev_pre_tl);

    // 4. Walk serially: each step's UpdateRecords adopts into res_opt_, so the
    //    next step's challenger sees the new champion. The committed TL tracks
    //    the adopted best, so the diff flags only the walked task (|diff|==1).
    for (const SerializedTaskQueueEntry& entry : queue) {
        if (entry.kind == SerializedTaskQueueEntry::Kind::EnvChanged) {
            // Type-E: sub-incremental re-search at the committed TL (no TL walk).
            // et_increased = the env-move direction carried on the entry.
            current_config_sp = EvaluateTimeLimitConfig_SubIncremental(
                K, starting_time_limits, static_cast<size_t>(entry.task_id),
                entry.et_increased);
            starting_time_limits = ReconstructTimeLimitVecFromResOpt();
        } else {
            // Type-L: TL walk, each step calling the sub-incremental eval.
            // et_increased per step = sign of (trial TL − committed TL).
            double baseline_val = starting_time_limits[entry.task_id];
            size_t task_idx = static_cast<size_t>(entry.task_id);
            int K_cap = K;
            auto eval = [this, K_cap, task_idx,
                         baseline_val](const std::vector<double>& tl) {
                bool et_up = tl[task_idx] > baseline_val;
                return EvaluateTimeLimitConfig_SubIncremental(K_cap, tl,
                                                              task_idx, et_up);
            };
            // Backward pass (smaller TL), then forward pass from the same origin.
            current_config_sp = OptimizeSingleTaskTimeLimit_Impl(
                task_idx, starting_time_limits, current_config_sp, baseline_val,
                /*step=*/-1, patience, eval);
            current_config_sp = OptimizeSingleTaskTimeLimit_Impl(
                task_idx, starting_time_limits, current_config_sp, baseline_val,
                /*step=*/1, patience, eval);
            starting_time_limits = ReconstructTimeLimitVecFromResOpt();
        }
    }
    opt_pa_ = res_opt_.priority_vec;
    opt_sp_ = res_opt_.sp_opt;
}

std::vector<double>
OptimizePA_Incre_with_TimeLimits::InitializeTimeLimitsFromETConfig() {
    std::vector<double> time_limits(dag_tasks_.tasks.size());
    for (size_t i = 0; i < dag_tasks_.tasks.size(); i++) {
        if (dag_tasks_.tasks[i].timePerformancePairs.empty()) {
            time_limits[i] = -1.0;
        } else {
            size_t close_idx = Find_Close_ExecutionTime(
                dag_tasks_.tasks[i].timePerformancePairs,
                dag_tasks_.tasks[i].execution_time_dist.GetAvgValue());
            time_limits[i] =
                dag_tasks_.tasks[i].timePerformancePairs[close_idx].time_limit;
        }
    }
    return time_limits;
}

double OptimizePA_Incre_with_TimeLimits::OptimizeSingleTaskTimeLimit(
    size_t task_idx, int K, std::vector<double>& time_limits, double current_sp,
    double baseline_val, int step, bool from_scratch, int patience) {
    // Legacy wrapper: binds the eval to the reopt/BF path. The serialized Type-L
    // step calls OptimizeSingleTaskTimeLimit_Impl directly with a sub-incremental
    // eval (same walk, cheaper per-candidate score).
    int K_capture = K;
    bool from_scratch_capture = from_scratch;
    auto eval = [this, K_capture,
                 from_scratch_capture](const std::vector<double>& tl) {
        return EvaluateTimeLimitConfig_ScratchOrIncre(K_capture, tl,
                                                      from_scratch_capture);
    };
    return OptimizeSingleTaskTimeLimit_Impl(task_idx, time_limits, current_sp,
                                            baseline_val, step, patience, eval);
}

double OptimizePA_Incre_with_TimeLimits::OptimizeSingleTaskTimeLimit_Impl(
    size_t task_idx, std::vector<double>& time_limits, double current_sp,
    double baseline_val, int step, int patience,
    std::function<double(const std::vector<double>&)> eval) {
    const std::vector<double>& opts =
        time_limit_option_for_each_task_[task_idx];
    // No TL freedom ({-1}-only sentinel): nothing to walk.
    if (opts.size() == 1 && opts[0] == -1.0) {
        return current_sp;
    }

    size_t curr_opt_idx = FindTimeLimitOptionIndex(opts, baseline_val);
    // Stale baseline (not in the option set): walk cannot run.
    if (curr_opt_idx == opts.size()) {
        return current_sp;
    }

    double best_sp = current_sp;
    double best_option_val = time_limits[task_idx];

    // Walk in direction `step` to the boundary. `patience` is a total non-
    // improvement budget (no reset on improvement); at 0 the walk stops.
    for (int i = static_cast<int>(curr_opt_idx) + step;
         i >= 0 && i < static_cast<int>(opts.size()); i += step) {
        double val = opts[i];
        if (val == -1.0)  // defensive: a {-1} slot inside a real window
            continue;
        time_limits[task_idx] = val;
        double sp_val = eval(time_limits);

        if (IsBetterTimeLimitOption(sp_val, best_sp, step)) {
            best_sp = sp_val;
            best_option_val = val;
        } else if (patience == 0) {
            break;  // budget exhausted
        } else {
            --patience;  // spend one unit, keep stepping past the dip
        }
    }

    time_limits[task_idx] = best_option_val;
    return best_sp;
}

void OptimizePA_Incre_with_TimeLimits::PerformCoordinateDescentForTaskConfigOpt(
    int K, std::vector<double>& starting_time_limits, bool from_scratch) {
    std::vector<size_t> sorted_indices(dag_tasks_.tasks.size());
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(sorted_indices.begin(), sorted_indices.end(),
              TaskSortingHeuristic{dag_tasks_, sp_parameters_});

    // Patience: incremental (warm-started, ~unimodal) uses 0; reopt (full
    // re-search, can be non-unimodal) uses 1. From YAML globals.
    int patience = from_scratch
                       ? GlobalVariables::ReoptimizationTimeLimitSearchPatience
                       : GlobalVariables::IncrementalTimeLimitSearchPatience;

    // P2.9 lever A: when the flag is on AND this is the from-scratch/reopt
    // descent, route the per-task TL walk through the sub-incremental eval
    // (cache-routed, |diff|<=1 single-task re-search) instead of the legacy
    // full-beam eval. The incremental (warm-started) path already uses the
    // sub-incremental machinery directly (PerformSerializedTaskQueueOptimization),
    // so this only re-arms the reopt descent's walk — the common case where a
    // single-task TL change does not shift the global optimum PA by >1 task.
    // NOT bit-identical to the legacy from-scratch-per-candidate walk (reopt's
    // PA search can be non-unimodal at high util); gated, default OFF.
    // See parameters.yaml for the trade-off.
    bool use_subincremental_walk =
        from_scratch && GlobalVariables::ReoptimizationUseSubIncrementalWalk;

    // Reset the baseline for this interval so the eval below measures against
    // the correct current-interval baseline, not a stale prior.
    ResetIncumbentBaseline(from_scratch);
    double current_config_sp = EvaluateTimeLimitConfig_ScratchOrIncre(
        K, starting_time_limits, from_scratch);

    if (use_subincremental_walk) {
        // Re-arm the cache: the baseline beam above committed its champion via
        // CommitIncumbent, but with rta_cache_active_ false (ResetIncumbentBaseline
        // clears it), so the cache was NOT adopted. Adopt the committed triple
        // now (|diff|==0 → Evaluate's FullReuse path, zero-cost) and arm the gate
        // so every subsequent accepted walk step re-adopts via CommitIncumbent —
        // mirroring PerformSerializedTaskQueueOptimization's setup. This makes the
        // |diff|<=1 single-change invariant hold for the whole walk.
        rta_cache_active_ = true;
        std::vector<double> champion_tl = ReconstructTimeLimitVecFromResOpt();
        const std::vector<FiniteDist>& champ_rtas =
            rta_cache_.Evaluate(dag_tasks_, opt_pa_, champion_tl);
        rta_cache_.AdoptChampion(dag_tasks_, opt_pa_, champion_tl, champ_rtas);
    }

    for (size_t idx : sorted_indices) {
        // Skip {-1}-only tasks (no perf pairs → no TL freedom).
        const std::vector<double>& opts = time_limit_option_for_each_task_[idx];
        if (opts.size() == 1 && opts[0] == -1.0)
            continue;

        double baseline_val = starting_time_limits[idx];
        if (use_subincremental_walk) {
            // Sub-incremental walk: each trial TL calls the cache-routed
            // EvaluateTimeLimitConfig_SubIncremental (re-scores the carried PA +
            // 1D single-task re-search), reusing the incremental path's machinery.
            // et_increased per step = sign of (trial TL − committed TL), matching
            // PerformSerializedTaskQueueOptimization's Type-L body.
            size_t task_idx = idx;
            int K_cap = K;
            auto eval = [this, K_cap, task_idx,
                         baseline_val](const std::vector<double>& tl) {
                bool et_up = tl[task_idx] > baseline_val;
                return EvaluateTimeLimitConfig_SubIncremental(K_cap, tl, task_idx,
                                                              et_up);
            };
            // Backward pass (tie-break toward smaller TL on SP ties via step<0),
            // then a forward pass from the same origin.
            current_config_sp = OptimizeSingleTaskTimeLimit_Impl(
                task_idx, starting_time_limits, current_config_sp, baseline_val,
                /*step=*/-1, patience, eval);
            current_config_sp = OptimizeSingleTaskTimeLimit_Impl(
                task_idx, starting_time_limits, current_config_sp, baseline_val,
                /*step=*/1, patience, eval);
            // Keep the working TL vector tracking the committed best so the next
            // task's baseline reflects any adoption (mirrors the serialized loop).
            starting_time_limits = ReconstructTimeLimitVecFromResOpt();
        } else {
            // Legacy walk: full-beam eval per trial TL (OptimizeFromScratch).
            // Backward pass (tie-break toward smaller TL on SP ties via step<0),
            // then a forward pass from the original starting TL.
            current_config_sp = OptimizeSingleTaskTimeLimit(
                idx, K, starting_time_limits, current_config_sp, baseline_val,
                /*step=*/-1, from_scratch, patience);
            current_config_sp = OptimizeSingleTaskTimeLimit(
                idx, K, starting_time_limits, current_config_sp, baseline_val,
                /*step=*/1, from_scratch, patience);
        }
    }
    // Disarm the cache gate on the way out so the reopt path does not leave it
    // armed for the next interval's reset (ResetIncumbentBaseline clears it too,
    // but this keeps the gate scoped to this descent exactly).
    if (use_subincremental_walk) {
        rta_cache_active_ = false;
    }
}

PriorityVec OptimizePA_Incre_with_TimeLimits::Optimize_w_TL_ScratchOrIncre(
    const DAG_Model& dag_tasks_update, int K) {
    // One shared TIME_LIMIT budget for this interval's INCR call (covers both
    // branches + the disable_time_limit_opt bypass). Fresh per call — the
    // orchestrator reuses incr_optimizer_ across intervals, so construction-time
    // start_time_ would bound the whole simulation, not one interval.
    // BFSharedBudgetCancelled() is inert outside this scope. On cancel,
    // EvaluateSPWithPriorityVec returns INT_MIN; the walk's strict-> adopt guard
    // treats that as "not better" and keeps the incumbent (compare-and-keep), so
    // in-budget runs are byte-identical.
    BFDLSharedBudget shared_budget(std::chrono::high_resolution_clock::now());

    // Modular reopt: every ReoptimizationPeriod-th call (count % period == 0)
    // takes the from-scratch compare-and-keep path, else the warm-started
    // incremental path. count==0 → ReOptimizePeriodic bootstraps the interval-0
    // incumbent. Counter advances every call, never resets.
    int period = GlobalVariables::ReoptimizationPeriod;
    bool trigger_reopt = (reoptimization_interval_count_ % period == 0);
    if (trigger_reopt) {
        ReOptimizePeriodic(dag_tasks_update, K);
    } else {
        OptimizeIncre_w_TL(dag_tasks_update, K);
    }
    reoptimization_interval_count_++;
    return opt_pa_;
}

PriorityVec OptimizePA_Incre_with_TimeLimits::OptimizeWithTimeLimitOptDisabled(
    int K, std::vector<double>& time_limits, bool from_scratch) {
    // Bypasses the descent, so the interval reset it would do must run here.
    ResetIncumbentBaseline(from_scratch);
    InitializeTimeLimitsToSmallest(time_limits);
    EvaluateTimeLimitConfig_ScratchOrIncre(K, time_limits, from_scratch);
    return opt_pa_;
}

PriorityVec OptimizePA_Incre_with_TimeLimits::OptimizeIncre_w_TL(
    const DAG_Model& dag_tasks_update, int K) {
    // Capture the pre-TL env DAG before the absorb — this is the T-1 env for the
    // Type-E diff (FindEnvTaskWithDifferentEt) consumed by the serialized queue.
    DAG_Model dag_tasks_prev_pre_tl = dag_tasks_;
    dag_tasks_ = dag_tasks_update;
    ApplyWCETAblationIfRequired(dag_tasks_);
    // Full per-task option set; the walk stops on patience-bounded non-improvement
    // (no radius cap, so a better option beyond the old wall is reachable).
    time_limit_option_for_each_task_ = RecordTimeLimitOptions(dag_tasks_);
    // Seed from the carried adopted TL in res_opt_, NOT the Gaussian-mean TL:
    // both diff sides must carry the adopted TL, else unchanged perf-pair tasks
    // are flagged as changed.
    std::vector<double> time_limits = ReconstructTimeLimitVecFromResOpt();
    if (GlobalVariables::disable_time_limit_opt) {
        return OptimizeWithTimeLimitOptDisabled(K, time_limits,
                                                /*from_scratch=*/false);
    }
    // Serialized E+L queue — the incremental path's interval search (the legacy
    // PerformCoordinateDescentForTaskConfigOpt remains the from-scratch/reopt
    // descent; see ReOptimizePeriodic).
    PerformSerializedTaskQueueOptimization(K, time_limits,
                                           dag_tasks_prev_pre_tl);
    return opt_pa_;
}

// Reconstruct the positional time-limit vector (one entry per task, in task
// order) from the recorded id→TL map. Tasks with no recorded TL get -1.
std::vector<double>
OptimizePA_Incre_with_TimeLimits::ReconstructTimeLimitVecFromResOpt() {
    std::vector<double> tl(dag_tasks_.tasks.size());
    for (size_t i = 0; i < dag_tasks_.tasks.size(); i++) {
        int id = dag_tasks_.tasks[i].id;
        tl[i] = res_opt_.id2time_limit.count(id) ? res_opt_.id2time_limit.at(id)
                                                 : -1.0;
    }
    return tl;
}

// Rate-monotonic: tasks sorted by period ascending (smallest = highest
// priority), ties by avg ET ascending (deterministic). Mirrors the orchestrator's RM mode.
PriorityVec OptimizePA_Incre_with_TimeLimits::RateMonotonicPriorityVec() {
    std::vector<int> sorted(dag_tasks_.tasks.size());
    std::iota(sorted.begin(), sorted.end(), 0);
    std::sort(sorted.begin(), sorted.end(), [&](int a, int b) {
        const Task& ta = dag_tasks_.tasks[a];
        const Task& tb = dag_tasks_.tasks[b];
        if (ta.period != tb.period)
            return ta.period < tb.period;
        return ta.execution_time_dist.GetAvgValue() <
               tb.execution_time_dist.GetAvgValue();
    });
    return sorted;
}

// Seed the incumbent 4-tuple {dag, sp, pa, tl} as the compare-guard baseline.
// CommitIncumbent is the single writer; dag_with_tl is not stored — the
// challenger is rebuilt from res_opt_.id2time_limit next interval.
void OptimizePA_Incre_with_TimeLimits::SeedStateFromIncumbent(
    const DAG_Model& dag_with_tl, const PriorityVec& pa, double sp,
    const std::vector<double>& tl) {
    CommitIncumbent(pa, sp, tl);
}

// The one writer for the incumbent state: res_opt_ + the opt_pa_/opt_sp_
// mirrors. Centralizing writes here makes sp_parameters_/DAG desync impossible.
void OptimizePA_Incre_with_TimeLimits::CommitIncumbent(
    const PriorityVec& pa, double sp, const std::vector<double>& tl) {
    opt_sp_ = sp;
    opt_pa_ = pa;
    res_opt_.SaveTimeLimits(dag_tasks_.tasks, tl);
    res_opt_.UpdatePriorityVec(opt_pa_);
    res_opt_.sp_opt = opt_sp_;
    // Advance the cache champion to track res_opt_. Evaluate never advances the
    // champion (only AdoptChampion/Initialize do); a frozen champion would drift
    // to |diff|>1 and Evaluate would throw. Adopting the same triple just scored
    // hits the |diff|==0 FullReuse path (zero-cost). Gated by rta_cache_active_
    // so the reopt path (shares this writer, can commit >1) neither throws nor
    // regresses.
    if (rta_cache_active_) {
        const auto& rtas = rta_cache_.Evaluate(dag_tasks_, pa, tl);
        rta_cache_.AdoptChampion(dag_tasks_, pa, tl, rtas);
    }
}

// Throwaway challenger rebuilt from res_opt_ each candidate (not persistent).
// The champion TL tracks the working TL, so while one task is walked the diff
// flags only that task → OptimizeIncre re-searches its 1D variations.
OptimizePA_Incre
OptimizePA_Incre_with_TimeLimits::BuildChallengerFromIncumbent() {
    std::vector<double> tl_prev = ReconstructTimeLimitVecFromResOpt();
    DAG_Model dag_with_tl_prev =
        UpdateExtDistBasedOnTimeLimit(dag_tasks_, tl_prev);
    OptimizePA_Incre challenger(dag_with_tl_prev, sp_parameters_);
    challenger.opt_pa_ = res_opt_.priority_vec;
    challenger.opt_sp_ = res_opt_.sp_opt;
    return challenger;
}

// Reset the incumbent baseline before the descent's baseline eval.
// from_scratch (reopt): re-eval the carried {pa, tl} under the new DAG (or
// RM+min-TL at interval 0) and commit it, so opt_sp_ holds the compare-and-keep
// baseline (not -1.0). !from_scratch (incremental): set opt_sp_=-1.0 so the
// first UpdateRecords force-commits; res_opt_ itself is untouched.
void OptimizePA_Incre_with_TimeLimits::ResetIncumbentBaseline(
    bool from_scratch) {
    // A new interval starts cold: the cached champion triple is for the previous
    // interval's env+TL; carrying it forward would make the first serialized
    // eval diff >1 → Evaluate would throw. Default-construct (RTACache has no
    // Clear()) and clear the gate; PerformSerializedTaskQueueOptimization re-arms it.
    rta_cache_ = RTACache();
    rta_cache_active_ = false;
    if (from_scratch) {
        if (IfInitialized()) {
            std::vector<double> tl_prev = ReconstructTimeLimitVecFromResOpt();
            PriorityVec pa_prev = opt_pa_;
            DAG_Model dag_new_with_tl_prev =
                UpdateExtDistBasedOnTimeLimit(dag_tasks_, tl_prev);
            double sp_prev_new = EvaluateSPWithPriorityVec(
                dag_new_with_tl_prev, sp_parameters_, pa_prev);
            SeedStateFromIncumbent(dag_new_with_tl_prev, pa_prev, sp_prev_new,
                                   tl_prev);
        } else {
            // Interval 0: RM priorities + every task at its smallest TL option.
            std::vector<double> tl_min = SmallestTimeLimitVec();
            PriorityVec pa_rm = RateMonotonicPriorityVec();
            DAG_Model dag_with_tl_min =
                UpdateExtDistBasedOnTimeLimit(dag_tasks_, tl_min);
            double sp_rm = EvaluateSPWithPriorityVec(dag_with_tl_min,
                                                     sp_parameters_, pa_rm);
            SeedStateFromIncumbent(dag_with_tl_min, pa_rm, sp_rm, tl_min);
        }
    } else {
        opt_sp_ = -1.0;
    }
}

PriorityVec OptimizePA_Incre_with_TimeLimits::ReOptimizePeriodic(
    const DAG_Model& dag_tasks_update, int K) {
    // Compare-and-keep reopt: re-eval the incumbent under the new DAG, seed it
    // as baseline, then run a fresh from-scratch descent. UpdateRecords' strictly-
    // greater-SP guard (tie-break lower TL-sum) preserves the incumbent if the
    // search finds nothing better.

    dag_tasks_ = dag_tasks_update;
    ApplyWCETAblationIfRequired(dag_tasks_);
    // Full per-task option set (see OptimizeIncre_w_TL for the no-radius-cap walk).
    time_limit_option_for_each_task_ = RecordTimeLimitOptions(dag_tasks_);

    // The baseline reset (re-eval carried {pa, tl}, or RM+min-TL at interval 0)
    // runs inside the descent via ResetIncumbentBaseline(true). Seed from the
    // carried adopted TL when an incumbent exists; IfInitialized() auto-falls-
    // back to the Gaussian-mean TL at interval 0 (no prior state — irreducible).
    std::vector<double> time_limits = IfInitialized()
                                          ? ReconstructTimeLimitVecFromResOpt()
                                          : InitializeTimeLimitsFromETConfig();
    if (GlobalVariables::disable_time_limit_opt) {
        OptimizeWithTimeLimitOptDisabled(K, time_limits, /*from_scratch=*/true);
    } else {
        PerformCoordinateDescentForTaskConfigOpt(K, time_limits,
                                                 /*from_scratch=*/true);
    }
    return opt_pa_;
}

void OptimizePA_Incre_with_TimeLimits::ApplyWCETAblationIfRequired(
    DAG_Model& dag_tasks) {
    if (GlobalVariables::use_wcet_execution_time) {
        for (auto& task : dag_tasks.tasks) {
            double max_et = task.execution_time_dist.max_time;
            task.execution_time_dist = GetUnitExecutionTimeDist(max_et);
            task.setExecGaussian(GaussianDist(max_et, 0.01));
            task.setExecutionTime(max_et);
        }
    }
}

std::vector<double> OptimizePA_Incre_with_TimeLimits::SmallestTimeLimitVec()
    const {
    std::vector<double> time_limits(dag_tasks_.tasks.size());
    for (size_t i = 0; i < dag_tasks_.tasks.size(); i++) {
        time_limits[i] =
            dag_tasks_.tasks[i].timePerformancePairs.empty()
                ? -1.0
                : dag_tasks_.tasks[i].timePerformancePairs[0].time_limit;
    }
    return time_limits;
}

void OptimizePA_Incre_with_TimeLimits::InitializeTimeLimitsToSmallest(
    std::vector<double>& time_limits) {
    std::vector<double> smallest = SmallestTimeLimitVec();
    for (size_t i = 0; i < time_limits.size(); i++) {
        time_limits[i] = smallest[i];
    }
}

}  // namespace SP_OPT_PA
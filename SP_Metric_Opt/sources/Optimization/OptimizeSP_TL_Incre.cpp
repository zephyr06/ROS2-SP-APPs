
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
    // Ties (approx-equal SP) prefer the smaller time limit. Walking downward
    // (step < 0) reaches a smaller TL next, so a downward tie is an
    // improvement; walking upward (step > 0) reaches a larger TL, so an upward
    // tie is rejected to keep the tighter TL already held.
    if (ApproxEqualSP(new_sp, current_best_sp) && step < 0) {
        return true;
    }
    return false;
}

void OptimizePA_Incre_with_TimeLimits::UpdateRecords(
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
        // CommitIncumbent owns res_opt_ + the opt_pa_/opt_sp_ mirrors. The
        // challenger (the `optimizer` arg) is a throwaway; only its adopted
        // {pa, sp, tl} survives across intervals.
        CommitIncumbent(optimizer.opt_pa_, optimizer.opt_sp_, time_limits);

        if (GlobalVariables::debugMode) {
            std::cout << "Time limit: \n";
            for (double time : time_limits) std::cout << time << " ";
            std::cout << "TraverseTimeLimitOptions: "
                      << "opt_sp_ = " << opt_sp_ << std::endl;
        }
    }
}

double OptimizePA_Incre_with_TimeLimits::EvaluateTimeLimitConfig_ScratchOrIncre(
    int K, const std::vector<double>& time_limits, bool from_scratch) {
    eval_count_++;
    DAG_Model dag_tasks_cur =
        UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits);

    double current_sp = -1.0;
    if (from_scratch) {
        // Reopt: ignore warm state, re-search the full beam for the current TL.
        OptimizePA_Incre optimizer(dag_tasks_cur, sp_parameters_);
        optimizer.OptimizeFromScratch(K);
        current_sp = optimizer.opt_sp_;
        UpdateRecords(optimizer, time_limits);
    } else if (IfInitialized()) {
        // Incremental: rebuild a throwaway challenger from res_opt_ (the
        // champion) each candidate, then OptimizeIncre. See
        // BuildChallengerFromIncumbent — this guarantees only the walked task's
        // ET differs, the perfect case for OptimizeIncre's diff-driven 1D
        // re-search.
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
    (void)K;  // unused: the primitive re-searches one task's 1D positions (no
              // beam)

    // The per-eval DAG rebuild is required, not redundant: the cost-dominant
    // caller is the Type-L walk, which calls this once PER trial TL step with a
    // DIFFERENT `time_limits` each time, so each call's candidate DAG genuinely
    // differs and must be rebuilt. The Type-E call (committed TL) does double-
    // build the champion DAG (BuildChallengerFromIncumbent builds the same DAG
    // from the committed TL), but that is one call per Type-E entry per interval
    // — negligible, and the redundant build there is expected to be subsumed by
    // the P1.9 RTA cache's |diff|==0 full-reuse path (same DAG, different PA),
    // not by threading a pre-built DAG through this eval seam.
    DAG_Model dag_tasks_cur =
        UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits);

    // Contract: an incumbent must exist (the incremental path warm-starts from
    // res_opt_). The serialized driver calls ResetIncumbentBaseline(false)
    // first, which gates on IfInitialized() — if interval 0 ever reaches here
    // without bootstrap, fail loudly rather than silently mis-seeding.
    if (!IfInitialized()) {
        CoutError(
            "EvaluateTimeLimitConfig_SubIncremental: no incumbent is "
            "initialized. The serialized path requires a prior from-scratch "
            "solve (ReOptimizePeriodic) to warm-start from.");
    }

    OptimizePA_Incre challenger = BuildChallengerFromIncumbent();

    // P1.10 Phase 3 invariant (debugMode-only): every SP-eval on the serialized
    // path changes AT MOST one task's ET vs the champion.
    // BuildChallengerFromIncumbent builds the champion DAG from dag_tasks_ +
    // the committed TL; dag_tasks_cur carries dag_tasks_ + the eval's
    // time_limits. Two cases:
    //  - Type-L (trial TL != committed): the ONE walked task's ET differs →
    //  |diff|==1.
    //  - Type-E (committed TL, env-changed task): the env move was absorbed
    //  into
    //    dag_tasks_ at the :313 absorb BEFORE the champion was built, so it is
    //    on BOTH diff sides and cancels (goal.md premise-correction finding #1)
    //    → the candidate DAG == champion DAG → |diff|==0. The re-search is
    //    still meaningful: the carried PA may be stale under the new env, and
    //    opt_sp_ is re-scored fresh under dag_tasks_cur so the strict-> adopt
    //    test measures against the genuine new-env SP. |diff|==0 is the Type-E
    //    case (same DAG, only PA varies).
    // So the honest invariant is |diff|<=1 (Type-L: 1, Type-E: 0); |diff|>1
    // would mean the champion drifted (multiple ET changes in one eval),
    // breaking the sub-incremental's single-change premise. This is what P1.9's
    // rev-2 cache exploits: 0 → full RTA reuse (same DAG, different PA), 1 →
    // single-task patch. Gated on debugMode so it is cost-free in production; a
    // violation throws via CoutError so any future regression is surfaced
    // loudly (e.g. by the TDD test).
    AssertSingleChangeInvariant(challenger.dag_tasks_, dag_tasks_cur, task_idx);

    // OptimizeIncre_SingleTask TRUSTS opt_sp_ as the baseline (it does NOT
    // re-score the carried PA — that is the caller's job, mirroring
    // OptimizeIncre's :308-311 baseline). BuildChallengerFromIncumbent seeds
    // opt_sp_ from res_opt_.sp_opt, which is the champion's SP at the
    // CHAMPION's TL — but dag_tasks_cur carries the TRIAL TL, so that opt_sp_
    // is stale w.r.t. the candidate DAG. Re-score the carried PA under
    // dag_tasks_cur here so the primitive's strict-> adopt test measures
    // variations against the genuine new-TL baseline (else a TL step that
    // improves only via the carried PA — no priority change beats the stale
    // champion SP — would be missed, stopping the walk early). This is the one
    // re-score the sub-incremental path keeps (the redundant carried-POSITION
    // VARIATION is still dropped at the generator).
    challenger.opt_sp_ = EvaluateSPWithPriorityVec(
        dag_tasks_cur, sp_parameters_, challenger.opt_pa_);
    challenger.OptimizeIncre_SingleTask(
        dag_tasks_cur, static_cast<int>(task_idx), et_increased);
    double current_sp = challenger.opt_sp_;
    UpdateRecords(challenger, time_limits);
    return current_sp;
}

// P1.10 Phase 3: assert the single-change invariant on the serialized path.
// `champion_dag` = BuildChallengerFromIncumbent's DAG (dag_tasks_ + committed
// TL); `candidate_dag` = the trial DAG (dag_tasks_ + the eval's time_limits).
// The diff must be EMPTY or flag exactly `task_idx`:
//  - |diff|==0: Type-E (env move absorbed into dag_tasks_ on both sides →
//  cancels;
//    candidate DAG == champion DAG, only the re-searched PA varies → full RTA
//    reuse for P1.9).
//  - |diff|==1 flagging task_idx: Type-L (the walked task's TL moved).
//  - |diff|>1, or |diff|==1 flagging a DIFFERENT task: the champion drifted
//  (multiple
//    ET changes in one eval), breaking the sub-incremental's single-change
//    premise and P1.9's single-task RTA patch. Throws via CoutError.
// debugMode-only: production stays free of the per-eval diff cost.
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
        ok = true;  // Type-E: env move cancels (both sides carry dag_tasks_)
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
            "P1.10 single-change invariant violated: serialized step for "
            "task " +
            std::to_string(task_idx) + " produced |diff|=" +
            std::to_string(diff.size()) + " (flagged tasks: [" + flagged +
            "]). Expected |diff|<=1 (Type-L: 1 on task_idx; Type-E: 0, env "
            "move "
            "cancels on both diff sides). A larger diff means the champion "
            "drifted — multiple ET changes in one eval — which breaks the "
            "sub-incremental's single-change premise and P1.9's single-task "
            "RTA patch.");
    }
}

std::vector<int> OptimizePA_Incre_with_TimeLimits::CollectTLFlexibleTaskIds()
    const {
    std::vector<int> res;
    res.reserve(time_limit_option_for_each_task_.size());
    for (size_t i = 0; i < time_limit_option_for_each_task_.size(); i++) {
        const std::vector<double>& opts = time_limit_option_for_each_task_[i];
        // The {-1}-only sentinel (RecordTimeLimitOptions for a no-pairs task)
        // means no TL freedom. Anything else is a walkable option set.
        if (!(opts.size() == 1 && opts[0] == -1.0)) {
            res.push_back(static_cast<int>(i));
        }
    }
    return res;
}

std::vector<SerializedTaskQueueEntry>
OptimizePA_Incre_with_TimeLimits::BuildSerializedTaskQueue(
    const DAG_Model& dag_tasks_prev_pre_tl) const {
    // Type-E: env-changed tasks (pre-TL DAG diff, TL-flexible tasks filtered
    // out by FindEnvTaskWithDifferentEt). Each carries a direction
    // (DiffObj.increase).
    std::vector<DiffObj> type_e =
        FindEnvTaskWithDifferentEt(dag_tasks_prev_pre_tl, dag_tasks_);
    // Type-L: TL-flexible tasks (the perf-pair grid).
    std::vector<int> type_l = CollectTLFlexibleTaskIds();

    // #5 dedup: E and L must be disjoint. A task in both is a contract
    // violation (TL-flexible tasks have no env dependence by generator design)
    // — hard-fail, do NOT silently pick a winner.
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

    // Task weight (D3 sort key): sp_parameters_.weights_node keyed by task id,
    // mirroring TaskSortingHeuristic's accessor. Missing → 1.0 default.
    auto weight_of = [this](int task_idx) {
        int id = dag_tasks_.tasks.at(task_idx).id;
        return sp_parameters_.weights_node.count(id)
                   ? sp_parameters_.weights_node.at(id)
                   : 1.0;
    };

    std::vector<SerializedTaskQueueEntry> queue;
    queue.reserve(type_e.size() + type_l.size());
    for (const DiffObj& d : type_e) {
        // Carry the env-move direction (DiffObj.increase) on the entry so the
        // Type-E handler reads it directly instead of recomputing the full env
        // diff per entry to recover one bool (FindEnvTaskWithDifferentEt was
        // already computed once above to build this queue).
        queue.push_back({d.task_id, SerializedTaskQueueEntry::Kind::EnvChanged,
                         d.increase});
    }
    for (int tid : type_l) {
        // TLFlexible entries leave et_increased=false (unused — the walk derives
        // per-step direction from the trial-vs-committed TL sign).
        queue.push_back({tid, SerializedTaskQueueEntry::Kind::TLFlexible});
    }
    // Sort together by weight DESCENDING (D3). Stable so same-weight E/L keep
    // insertion order (E before L) — deterministic, no tie-break dependence.
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
    // The serialized loop replaces the legacy coordinate descent for the
    // INCREMENTAL path. It honors the proposal's "sort tasks, then optimize in
    // order" shape: a dedicated baseline re-score (NO optimization) seeds
    // opt_sp_, then the queue walk does all optimization in D3 weight order.

    // Patience for the Type-L outward walk — same source as the legacy descent
    // (incremental path = the warm-started, ~unimodal setting → 0 is safe).
    int patience = GlobalVariables::IncrementalTimeLimitSearchPatience;

    // 1. Gate reset (incremental: opt_sp_=-1.0 so the first UpdateRecords
    //    force-commits). Same role as the legacy :258.
    ResetIncumbentBaseline(/*from_scratch=*/false);
    // P1.12 increment 2a: re-arm the cache gate for THIS walk body. The reset
    // above cleared it (reopt path keeps the cache off; this path turns it on).
    // The single-change invariant (|diff|<=1 per serialized SP-eval) holds for
    // every CommitIncumbent reached while this is true, so AdoptChampion stays
    // consistent with res_opt_ and Evaluate never throws. The gate is cleared
    // again by the next interval's ResetIncumbentBaseline.
    rta_cache_active_ = true;

    // 2. Baseline = DEDICATED RE-SCORE (#6). The champion's carried {pa, tl} is
    //    re-scored under the NEW env DAG to seed opt_sp_. It must NOT optimize
    //    (no OptimizeIncre, no challenger rebuild) — the queue walk owns all
    //    optimization. Routing through EvaluateTimeLimitConfig_ScratchOrIncre
    //    would call OptimizeIncre → priority optimization on env-changed tasks
    //    BEFORE the queue's sorted order is honored, violating the shape.
    //    CommitIncumbent directly mirrors SeedStateFromIncumbent's write path.
    DAG_Model dag_baseline =
        UpdateExtDistBasedOnTimeLimit(dag_tasks_, starting_time_limits);
    double current_config_sp =
        EvaluateSPWithPriorityVec(dag_baseline, sp_parameters_, opt_pa_);
    CommitIncumbent(opt_pa_, current_config_sp, starting_time_limits);

    // 3. Build the merged + weight-sorted E+L queue (D).
    std::vector<SerializedTaskQueueEntry> queue =
        BuildSerializedTaskQueue(dag_tasks_prev_pre_tl);

    // 4. Walk the queue serially (D4 = running-adopted champion): each step's
    //    UpdateRecords adopts into res_opt_, the next step's challenger (built
    //    inside the sub-incremental eval) sees it. The committed TL tracks the
    //    adopted best via UpdateRecords, so the diff the next step sees flags
    //    only the one task currently being walked (|diff|==1 invariant).
    for (const SerializedTaskQueueEntry& entry : queue) {
        if (entry.kind == SerializedTaskQueueEntry::Kind::EnvChanged) {
            // Type-E: one sub-incremental re-search of env-changed task's 1D
            // priority with the COMMITTED TL (no TL walk). et_increased = the
            // env-move direction, carried on the entry (populated once in
            // BuildSerializedTaskQueue from FindEnvTaskWithDifferentEt's
            // DiffObj.increase) — no per-entry env-diff recompute.
            current_config_sp = EvaluateTimeLimitConfig_SubIncremental(
                K, starting_time_limits, static_cast<size_t>(entry.task_id),
                entry.et_increased);
            // SubIncremental commits the adopted {pa, tl} via UpdateRecords;
            // refresh the working TL vector from the (possibly) adopted
            // champion so the next step diffs against the true current
            // champion.
            starting_time_limits = ReconstructTimeLimitVecFromResOpt();
        } else {
            // Type-L: trial-and-error TL walk (the existing patience-bounded
            // outward walk), each TL step calling the sub-incremental eval
            // (skips the redundant re-score the legacy eval pays each step).
            // et_increased per step = sign of (trial TL − committed TL): a
            // larger TL yields a larger ET, so an upward step is
            // et_increased=true.
            double baseline_val = starting_time_limits[entry.task_id];
            size_t task_idx = static_cast<size_t>(entry.task_id);
            // Bind the sub-incremental eval: trial TL → SP, with this task's
            // et_increased set from the trial-vs-committed direction.
            int K_cap = K;
            auto eval = [this, K_cap, task_idx,
                         baseline_val](const std::vector<double>& tl) {
                bool et_up = tl[task_idx] > baseline_val;
                return EvaluateTimeLimitConfig_SubIncremental(K_cap, tl,
                                                              task_idx, et_up);
            };
            // Backward pass (smaller TL), then forward pass from the same
            // origin.
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
    // Legacy wrapper: bind the eval to the reopt/BF path
    // (EvaluateTimeLimitConfig_ScratchOrIncre). The serialized Type-L step
    // (P1.10) calls OptimizeSingleTaskTimeLimit_Impl directly with a
    // sub-incremental eval instead — same walk, cheaper per-candidate score.
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
    // No TL freedom (the {-1}-only sentinel from RecordTimeLimitOptions):
    // nothing to walk. Return the carried SP unchanged.
    if (opts.size() == 1 && opts[0] == -1.0) {
        return current_sp;
    }

    size_t curr_opt_idx = FindTimeLimitOptionIndex(opts, baseline_val);
    // Stale baseline (not a member of the option set): no valid start index, so
    // the walk cannot run. Return the carried SP unchanged.
    if (curr_opt_idx == opts.size()) {
        return current_sp;
    }

    double best_sp = current_sp;
    double best_option_val = time_limits[task_idx];

    // Walk sequentially in direction `step`, stopping at the option-set
    // boundary. `patience` is a total non-improvement budget: each
    // non-improving step spends one unit (no reset on improvement); when it
    // hits 0 the walk stops.
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
            break;  // budget exhausted: stop the walk in this direction
        } else {
            --patience;  // spend one unit, keep stepping outward past the dip
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

    // Patience: consecutive non-improving steps the walk tolerates. Incremental
    // (warm-started, SP-vs-TL ~unimodal) uses 0; reopt (full re-search, can be
    // non-unimodal) uses 1. Sourced from YAML globals; see parameters.yaml.
    int patience = from_scratch
                       ? GlobalVariables::ReoptimizationTimeLimitSearchPatience
                       : GlobalVariables::IncrementalTimeLimitSearchPatience;

    // Reset the incumbent baseline for this interval (reopt: re-eval+commit;
    // incremental: opt_sp_=-1.0 gate) so the baseline eval below measures and
    // commits against the correct current-interval baseline, not a stale prior.
    ResetIncumbentBaseline(from_scratch);
    double current_config_sp = EvaluateTimeLimitConfig_ScratchOrIncre(
        K, starting_time_limits, from_scratch);

    for (size_t idx : sorted_indices) {
        // Skip {-1}-only tasks (no timePerformancePairs → no TL freedom).
        const std::vector<double>& opts = time_limit_option_for_each_task_[idx];
        if (opts.size() == 1 && opts[0] == -1.0)
            continue;

        double baseline_val = starting_time_limits[idx];
        // 1. Backward pass: try decreasing the TL (tie-break toward smaller TL
        //    on SP ties — handled inside IsBetterTimeLimitOption via step<0).
        current_config_sp = OptimizeSingleTaskTimeLimit(
            idx, K, starting_time_limits, current_config_sp, baseline_val,
            /*step=*/-1, from_scratch, patience);
        // 2. Forward pass from the ORIGINAL starting TL (not the backward
        //    result), exploring the upward side from the same origin.
        current_config_sp = OptimizeSingleTaskTimeLimit(
            idx, K, starting_time_limits, current_config_sp, baseline_val,
            /*step=*/1, from_scratch, patience);
    }
}

PriorityVec OptimizePA_Incre_with_TimeLimits::Optimize_w_TL_ScratchOrIncre(
    const DAG_Model& dag_tasks_update, int K) {
    // P1.14-mirror — install ONE shared TIME_LIMIT budget for this interval's
    // INCR call (covers BOTH the incremental OptimizeIncre_w_TL and the
    // re-optimize ReOptimizePeriodic branches, plus the disable_time_limit_opt
    // bypass). The orchestrator constructs incr_optimizer_ once and reuses it
    // across intervals (SimulationOrchestrator.cpp:300), so capture a FRESH
    // TimerType here per call rather than the construction-time start_time_
    // (which would bound the whole simulation, not one interval).
    // BFSharedBudgetCancelled() is inert (returns false) outside this scope, so
    // non-INCR callers of the shared SP-eval functions are unaffected — exactly
    // the P1.14 BF shape (OptimizeSP_TL_BF.cpp:89 installs the same guard at
    // EnumeratePA_with_TimeLimits entry), one level up. On cancel,
    // EvaluateSPWithPriorityVec returns INT_MIN; the walk's strict-> adopt guard
    // (IsBetterTimeLimitOption / UpdateRecords) treats that as "not better" and
    // keeps the incumbent (compare-and-keep), so in-budget runs are
    // byte-identical. The existing polls inside ObtainSP_DAG / ObtainSP_TaskSet
    // / RTA.cpp / SP_Metric.cpp do the actual interruption.
    BFDLSharedBudget shared_budget(std::chrono::high_resolution_clock::now());

    // Modular reopt: every ReoptimizationPeriod-th call (count % period == 0)
    // takes the from-scratch compare-and-keep path; otherwise the warm-started
    // incremental path. count == 0 routes to ReOptimizePeriodic, which
    // bootstraps the incumbent at interval 0 (the incremental path cannot run
    // without an incumbent). The counter advances every call and never resets.
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
    // This path bypasses the descent, so the interval reset the descent would
    // have done must run here too, else res_opt_ stays stale for this interval.
    ResetIncumbentBaseline(from_scratch);
    InitializeTimeLimitsToSmallest(time_limits);
    EvaluateTimeLimitConfig_ScratchOrIncre(K, time_limits, from_scratch);
    return opt_pa_;
}

PriorityVec OptimizePA_Incre_with_TimeLimits::OptimizeIncre_w_TL(
    const DAG_Model& dag_tasks_update, int K) {
    // Capture the pre-TL env DAG BEFORE the absorb. dag_tasks_ currently holds
    // the PREVIOUS interval's pre-TL env DAG (TLs went into transient locals
    // last interval, never the member), so this is the T-1 env for the Type-E
    // diff (FindEnvTaskWithDifferentEt) consumed by the serialized queue below.
    DAG_Model dag_tasks_prev_pre_tl = dag_tasks_;
    dag_tasks_ = dag_tasks_update;
    ApplyWCETAblationIfRequired(dag_tasks_);
    // Full per-task option set (every timePerformancePairs entry). The walk
    // steps over this set and stops on patience-bounded non-improvement — no
    // radius cap, so a tie-break or strictly-better option beyond the old
    // radius wall is reachable.
    time_limit_option_for_each_task_ = RecordTimeLimitOptions(dag_tasks_);
    // Start the descent from the CARRIED ADOPTED TL in res_opt_, NOT the
    // Gaussian-mean TL: both sides of FindTaskWithDifferentEt's diff must carry
    // the adopted TL, else unchanged perf-pair tasks are flagged as changed
    // (the P1.1 residual).
    std::vector<double> time_limits = ReconstructTimeLimitVecFromResOpt();
    if (GlobalVariables::disable_time_limit_opt) {
        return OptimizeWithTimeLimitOptDisabled(K, time_limits,
                                                /*from_scratch=*/false);
    }
    // P1.10 serialized E+L queue — the incremental path's interval search. It
    // merges the env-changed (Type-E) and TL-flexible (Type-L) task sets, sorts
    // by weight descending, and walks one task at a time via the
    // sub-incremental primitive (|diff|<=1), exercising the Type-E re-search
    // the incremental path previously lacked. (The legacy
    // PerformCoordinateDescentForTaskConfigOpt remains the from-scratch/reopt
    // descent — see ReOptimizePeriodic.)
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

// Rate-monotonic priority vector: tasks sorted by period ascending, smallest
// period = highest priority (index 0). Ties broken by average execution time
// ascending (lower ET = higher priority) so the order is deterministic for
// tasks that share a period. Mirrors the orchestrator's RM mode.
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

// Seed the incumbent 4-tuple {dag, sp, pa, tl} so opt_sp_ holds the baseline
// UpdateRecords' compare guard measures the search against.
void OptimizePA_Incre_with_TimeLimits::SeedStateFromIncumbent(
    const DAG_Model& dag_with_tl, const PriorityVec& pa, double sp,
    const std::vector<double>& tl) {
    // CommitIncumbent is the single writer for the durable incumbent store.
    // dag_with_tl itself is not stored — the challenger is rebuilt from
    // res_opt_.id2time_limit next interval.
    CommitIncumbent(pa, sp, tl);
}

// The ONE writer for the incumbent state: res_opt_ (durable) + the thin
// opt_pa_/opt_sp_ mirrors. Centralizing the writes here makes the
// sp_parameters_ / DAG desync class of bug structurally impossible.
void OptimizePA_Incre_with_TimeLimits::CommitIncumbent(
    const PriorityVec& pa, double sp, const std::vector<double>& tl) {
    opt_sp_ = sp;
    opt_pa_ = pa;
    res_opt_.SaveTimeLimits(dag_tasks_.tasks, tl);
    res_opt_.UpdatePriorityVec(opt_pa_);
    res_opt_.sp_opt = opt_sp_;
    // P1.12 increment 2a: advance the cache-champion to track res_opt_. The
    // cache's Evaluate does NOT advance the champion (only AdoptChampion/
    // Initialize do); if the champion stays frozen at an earlier triple, a
    // later serialized eval drifts to |diff|>1 and Evaluate throws. AdoptChampion
    // is cheap (stores caller rtas + rebuilds per-core HP-prefixes, NO RTA) but
    // needs the rtas for THIS triple — fetch them via Evaluate against the SAME
    // (dag_tasks_, pa, tl) being committed: that triple == the candidate the
    // adopting eval just scored, so Evaluate short-circuits to FullReuse and
    // returns the cached candidate_rta_ (near-zero cost, no extra RTA, no SP
    // regression). Gated by rta_cache_active_ so the reopt path (shares this
    // writer, can commit a >1 change) neither throws nor regresses. NOT yet
    // read by the eval path — the oracle EvaluateSPWithPriorityVec is still
    // live; this only keeps the cache warm for the 2b read-side swap.
    if (rta_cache_active_) {
        const std::vector<FiniteDist>& rtas =
            rta_cache_.Evaluate(dag_tasks_, pa, tl);
        rta_cache_.AdoptChampion(dag_tasks_, pa, tl, rtas);
    }
}

// Throwaway challenger rebuilt from res_opt_ (the champion) each candidate, not
// a persistent one. The champion TL tracks the working TL (UpdateRecords
// commits every adoption; the walk resets to the adopted best on
// no-improvement), so while one task is walked the diff flags ONLY that task →
// OptimizeIncre re-searches just its 1D priority variations. Perfect for
// incremental opt; a persistent challenger would drift to non-adopted
// candidates and flag extras.
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

// Reset the incumbent baseline for the current interval, run BEFORE the
// descent's baseline eval. Branched on from_scratch:
//  - true  (reopt): the carried {pa, tl} is re-evaluated under the NEW DAG and
//    committed, so opt_sp_ holds the baseline the search compare-and-keeps
//    against (NOT -1.0). At interval 0 (no incumbent) a synthetic RM+min-TL
//    baseline is built and committed instead.
//  - false (incremental): only the opt_sp_=-1.0 gate is set. The baseline eval
//    then rebuilds the challenger from res_opt_, re-evals it, and
//    UpdateRecords force-commits (optimizer.opt_sp_ > -1.0 always) so res_opt_
//    is overwritten for the current interval. res_opt_ itself is NOT touched
//    here — the challenger is built from the carried prior, re-evaluated, then
//    committed (read prior → re-eval → commit).
void OptimizePA_Incre_with_TimeLimits::ResetIncumbentBaseline(
    bool from_scratch) {
    // P1.12 increment 2a: a new interval's walk starts from a cold cache. The
    // champion triple the cache holds is for the PREVIOUS interval's env+TL;
    // carrying it forward would make the first serialized eval diff >1 (the
    // env moved, dag_tasks_ re-seeded) → Evaluate would throw. Default-construct
    // (no RTA_Cache.h change — RTACache has no Clear()). Clear the gate too, so
    // the reopt path (which also calls ResetIncumbentBaseline(true)) keeps the
    // cache off; PerformSerializedTaskQueueOptimization re-arms it for the walk.
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
    // Compare-and-keep reopt: the incumbent in res_opt_ is re-evaluated under
    // the new DAG and seeded as the baseline, then a fresh from-scratch descent
    // runs. UpdateRecords' strictly-greater-SP guard (tie-break lower TL-sum)
    // preserves the incumbent when the search finds nothing better.

    dag_tasks_ = dag_tasks_update;
    ApplyWCETAblationIfRequired(dag_tasks_);
    // Full per-task option set — see OptimizeIncre_w_TL for why the walk is no
    // longer radius-capped.
    time_limit_option_for_each_task_ = RecordTimeLimitOptions(dag_tasks_);

    // The baseline reset (re-eval carried {pa, tl} under the new DAG, or
    // RM+min-TL at interval 0) now runs inside the descent via
    // ResetIncumbentBaseline(true), before the baseline eval.
    // Descent start (P1.4): seed from the carried adopted TL — the optimizer's
    // own prior result in res_opt_ — whenever an incumbent exists. This is the
    // permanent, unconditional reopt seed policy: the seed must be
    // algorithm-derived (the optimizer's own prior output), not read from the
    // YAML taskset characterization. The IfInitialized() gate auto-falls-back
    // to the Gaussian-mean TL (InitializeTimeLimitsFromETConfig) at interval 0
    // (no incumbent → ReconstructTimeLimitVecFromResOpt would be all -1 =
    // no-op); that fallback is irreducible for any seed policy — the very first
    // solve has no prior optimizer state to seed from.
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
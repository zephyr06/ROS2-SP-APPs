
#include "sources/Optimization/OptimizeSP_TL_Incre.h"

#include <algorithm>
#include <numeric>
#include <sstream>
#include <stdexcept>

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

// Directional (at-or-below et_mean) TL seed. See header for the contract.
// Linear scan of the ascending grid; keep the last index whose option is ≤
// et_mean. If none qualifies, fall back to index 0 (the smallest option).
size_t FindLargestTimeLimitAtOrBelow(
    const std::vector<TimePerfPair>& time_perf_pairs, double et_mean) {
    if (time_perf_pairs.empty())
        return 0;
    size_t best_idx = 0;  // fallback: the smallest option
    bool found_at_or_below = false;
    for (size_t i = 0; i < time_perf_pairs.size(); i++) {
        if (time_perf_pairs[i].time_limit <= et_mean) {
            best_idx = i;  // grid is ascending → later qualifying index is larger
            found_at_or_below = true;
        }
    }
    if (!found_at_or_below) {
        CoutWarning(
            "FindLargestTimeLimitAtOrBelow: no TL grid option is <= et_mean "
            "(et_mean below the smallest option); falling back to the smallest "
            "grid option. The seed remains feasible-by-construction (the sim "
            "caps perf runtime ET at min(et_mean, TL) <= et_mean).");
    }
    return best_idx;
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

bool DetectETJump(const DAG_Model& dag_old, const DAG_Model& dag_new,
                  double ratio_threshold) {
    // Per-task positional scan (mirrors FindTaskWithDifferentEt): avg ET is
    // execution_time_dist.GetAvgValue(), the truncated FiniteDist mean the RTA
    // uses. Trips on the FIRST task whose new/old ratio >= ratio_threshold. A
    // decreased ET (ratio < 1) never trips — only jumps UP short-circuit the walk.
    const TaskSet& old_tasks = dag_old.tasks;
    const TaskSet& new_tasks = dag_new.tasks;
    int count = static_cast<int>(std::min(old_tasks.size(), new_tasks.size()));
    for (int i = 0; i < count; i++) {
        double et_old = old_tasks[i].execution_time_dist.GetAvgValue();
        double et_new = new_tasks[i].execution_time_dist.GetAvgValue();
        if (et_old <= 0.0) {
            continue;  // degenerate dist (no defined mean) — cannot ratio
        }
        if (et_new / et_old >= ratio_threshold) {
            return true;
        }
    }
    return false;
}

std::string FormatIntervalFallbackLogCsv(
    const std::vector<IntervalFallbackOutcome>& log) {
    std::ostringstream out;
    out << "interval_idx,et_jump_short_circuited,during_walk_reject_count,"
           "backstop_verdict,backstop_culprit_task_id,backstop_culprit_miss_"
           "chance,backstop_culprit_threshold\n";
    for (const IntervalFallbackOutcome& e : log) {
        const char* verdict_str = "none";
        if (e.backstop_verdict ==
            IntervalFallbackOutcome::BackstopVerdict::kKeptWalk) {
            verdict_str = "kept_walk";
        } else if (e.backstop_verdict ==
                   IntervalFallbackOutcome::BackstopVerdict::kAdoptedFallback) {
            verdict_str = "adopted_fallback";
        }
        out << e.interval_idx << ","
            << (e.et_jump_short_circuited ? "true" : "false") << ","
            << e.during_walk_reject_count << "," << verdict_str << ",";
        if (e.backstop_verdict ==
            IntervalFallbackOutcome::BackstopVerdict::kAdoptedFallback) {
            out << e.backstop_culprit_task_id << ","
                << e.backstop_culprit_miss_chance << ","
                << e.backstop_culprit_threshold;
        } else {
            out << ",,";  // culprit fields blank unless the backstop adopted
        }
        out << "\n";
    }
    return out.str();
}

bool OptimizePA_Incre_with_TimeLimits::WouldBeatIncumbent(
    double challenger_sp, const std::vector<double>& time_limits) const {
    // Strictly-greater SP beats; an approx-equal SP tie beats only when the
    // challenger's total TL is strictly smaller (tie-break prefers the tighter
    // budget).
    if (challenger_sp > opt_sp_ && !ApproxEqualSP(challenger_sp, opt_sp_)) {
        return true;
    }
    if (!ApproxEqualSP(challenger_sp, opt_sp_)) {
        return false;  // strictly worse — not a beat
    }
    // approx-equal tie: beat iff the total TL is strictly smaller.
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
    return sum_new < sum_old;
}

bool OptimizePA_Incre_with_TimeLimits::UpdateRecords(
    const OptimizePA_Incre& optimizer, const std::vector<double>& time_limits) {
    bool should_update = WouldBeatIncumbent(optimizer.opt_sp_, time_limits);

    // Feasibility gate: on a would-beat, commit only if every important task's
    // ddl_miss_chance ≤ threshold. RTA re-runs the same rta_cache_.Evaluate
    // CommitIncumbent uses (candidate's final {pa, tl}); reject → no commit.
    // Armed by `enable_fallback_use_` — the single master switch for the fall-back
    // use (ON by default = shipped; the offline safe-fallback walk inside
    // `ComputeSafeFallback` forces it true on its throwaway sibling so the
    // certificate holds even in the measurement arm).
    bool gate_armed = enable_fallback_use_ && !BFSharedBudgetCancelled();
    if (should_update && gate_armed) {
        const std::vector<FiniteDist>& challenger_rtas =
            rta_cache_.Evaluate(dag_tasks_, optimizer.opt_pa_, time_limits);
        if (!ImportantTasksMeetThresholds(dag_tasks_, sp_parameters_,
                                          optimizer.opt_pa_, time_limits,
                                          challenger_rtas)) {
            // P0.7 step 4 — count the during-walk reject (trigger b-i). The live
            // interval's record is the log's back entry (pushed at dispatch entry).
            if (!interval_fallback_log_.empty()) {
                interval_fallback_log_.back().during_walk_reject_count++;
            }
            return false;  // gate-REJECT: SP-better but infeasible -> no commit
        }
    }

    if (should_update) {
        // CommitIncumbent owns res_opt_ + mirrors; the `optimizer` arg is a
        // throwaway, only its adopted {pa, sp, tl} survives.
        CommitIncumbent(optimizer.opt_pa_, optimizer.opt_sp_, time_limits);
    }
    return should_update;
}

double OptimizePA_Incre_with_TimeLimits::CallOptimizerGivenTimeLimits(
    int beam_search_width, const std::vector<double>& time_limits, bool from_scratch) {
    eval_count_++;
    DAG_Model dag_tasks_cur =
        UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits);

    double current_sp = -1.0;
    if (from_scratch) {
        // Reopt: ignore warm state, re-search the full beam.
        OptimizePA_Incre optimizer(dag_tasks_cur, sp_parameters_);
        optimizer.OptimizeFromScratch(beam_search_width);
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
            "CallOptimizerGivenTimeLimits: incremental path "
            "(from_scratch=false) requested but no incumbent is initialized. "
            "Bootstrap with a from_scratch call first "
            "(e.g. ReOptimizePeriodic).");
    }
    return current_sp;
}

double OptimizePA_Incre_with_TimeLimits::OptimizeIncreSingleTask(
    const std::vector<double>& time_limits, size_t task_idx,
    bool et_increased) {
    eval_count_++;

    // In-walk AdoptChampion calls advance the champion speculatively to trial PAs
    // the walk may not commit. On REJECT revert the champion, else a later
    // serialized eval diffs committed-PA vs drifted champion (|diff|>1 → throw).
    // Accept needs no revert (CommitIncumbent re-adopts each commit).
    RTACache cache_backup = rta_cache_;

    // Per-eval rebuild: the Type-L walk calls this once per trial TL step. The
    // Type-E call double-builds but is one per entry per interval — negligible,
    // subsumed by the cache's |diff|==0 full-reuse path.
    DAG_Model dag_tasks_cur =
        UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits);

    // Contract: an incumbent must exist (the incremental path warm-starts from
    // res_opt_). Fail loudly rather than silently mis-seeding.
    if (!IfInitialized()) {
        CoutError(
            "OptimizeIncreSingleTask: no incumbent is "
            "initialized. The serialized path requires a prior from-scratch "
            "solve (ReOptimizePeriodic) to warm-start from.");
    }

    OptimizePA_Incre challenger = BuildChallengerFromIncumbent();

    // Debug-only: diff must be |diff|≤1 (Type-L: 1; Type-E: 0 — the env move was
    // absorbed into dag_tasks_ before the champion was built, so it cancels).
    AssertSingleChangeInvariant(challenger.dag_tasks_, dag_tasks_cur, task_idx);

    // Re-score the carried PA under the new TL so the primitive's strict->adopt
    // test measures against the genuine new-TL baseline (else a TL step that
    // improves only via the carried PA is missed, stopping the walk early).
    // Routed through the RTA cache (Type-L → |diff|==1 patch; Type-E → |diff|==0
    // FullReuse). Mirror the oracle's INT_MIN on cancel so UpdateRecords' strict->
    // guard discards a cancelled eval. `baseline_rtas` points into the cache's
    // candidate_rta_ scratch (valid only until the next Evaluate/AdoptChampion) —
    // consumed HERE only, before PA descent; the gate in `UpdateRecords` fetches
    // the challenger's final RTA itself.
    const std::vector<FiniteDist>* baseline_rtas = nullptr;
    if (BFSharedBudgetCancelled()) {
        challenger.opt_sp_ = INT_MIN;
    } else {
        baseline_rtas =
            &rta_cache_.Evaluate(dag_tasks_cur, challenger.opt_pa_, time_limits);

        challenger.opt_sp_ = ObtainSP_Full_From_NodeRTAs(
            dag_tasks_cur, sp_parameters_, challenger.opt_pa_, time_limits,
            *baseline_rtas);
    }
    // Skip the O(N)-PA descent once the budget expired (it is the bulk of the
    // per-eval cost). opt_sp_ is then INT_MIN (entry cancel) or the just-scored
    // baseline (mid-baseline cancel); either way UpdateRecords' strict-> guard
    // discards or adopts safely. Inert within the 1s budget (the incremental
    // path finishes inside it) → prod bit-identical. PA descent runs whether or
    // not the gate is on — the gate is a pure accept/reject criterion at
    // `UpdateRecords` (reading the challenger's FINAL RTA there, not this
    // pre-descent `baseline_rtas` which PA descent overwrites).
    if (!BFSharedBudgetCancelled()) {
        challenger.OptimizeIncre_SingleTask(dag_tasks_cur,
                                            static_cast<int>(task_idx),
                                            et_increased, std::ref(rta_cache_));
    }
    double current_sp = challenger.opt_sp_;
    bool updated = UpdateRecords(challenger, time_limits);
    if (!updated) {
        rta_cache_ = cache_backup;
        // Gate-REJECT of an SP-better candidate: UpdateRecords returned false
        // WITHOUT committing, but current_sp still holds the rejected candidate's
        // better SP. Report the INCUMBENT SP so the walk's IsBetterTimeLimitOption
        // sees "no progress" and never tracks the rejected TL. Guarded by the same
        // master flag as the gate (`enable_fallback_use_`) → suppressed whenever
        // the gate is armed (online arm OR the offline safe-fallback walk).
        if (enable_fallback_use_ && !BFSharedBudgetCancelled() &&
            WouldBeatIncumbent(challenger.opt_sp_, time_limits)) {
            current_sp = res_opt_.sp_opt;
        }
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

double OptimizePA_Incre_with_TimeLimits::WalkSerializedTaskQueue(
    const std::vector<SerializedTaskQueueEntry>& queue,
    std::vector<double>& starting_time_limits, double current_config_sp,
    int patience) {
    // Shared per-entry dispatch. Each step's UpdateRecords adopts into res_opt_,
    // so the next step's challenger sees the new champion; the committed TL tracks
    // the adopted best, so the diff flags only the walked task (|diff|<=1).
    for (const SerializedTaskQueueEntry& entry : queue) {
        // Stop walking once the per-interval TIME_LIMIT expired (compare-and-keep
        // retains the best-so-far). Inert within budget → prod byte-identical.
        if (BFSharedBudgetCancelled()) {
            break;
        }
        if (entry.kind == SerializedTaskQueueEntry::Kind::EnvChanged) {
            // Type-E: sub-incremental re-search at the committed TL (no TL walk).
            // et_increased = the env-move direction carried on the entry.
            current_config_sp = OptimizeIncreSingleTask(
                starting_time_limits, static_cast<size_t>(entry.task_id),
                entry.et_increased);
            starting_time_limits = ReconstructTimeLimitVecFromResOpt();
        } else {
            // Type-L: TL walk, each step calling the sub-incremental eval.
            current_config_sp = OptimizeOneTaskWithTimeLimit(
                static_cast<size_t>(entry.task_id), starting_time_limits,
                current_config_sp, starting_time_limits[entry.task_id], patience);
        }
    }
    return current_config_sp;
}

double OptimizePA_Incre_with_TimeLimits::SeedBaselineAndArmCache(
    int beam_search_width, std::vector<double>& starting_time_limits, IntervalDescentMode mode) {
    // Reset + baseline seed + cache arm/adopt/re-sync. Gathered into one helper
    // so RunIntervalDescent has no mode-conditional cache code.
    if (mode == IntervalDescentMode::Incremental) {
        // Gate reset (opt_sp_=-1.0 so the first UpdateRecords force-commits), then
        // re-arm the cache. |diff|<=1 holds for every CommitIncumbent while true,
        // so AdoptChampion stays consistent with res_opt_ and Evaluate never throws.
        ResetIncumbentBaseline(/*from_scratch=*/false);
        rta_cache_active_ = true;

        // Baseline = dedicated re-score (NO optimization — must not route through
        // ScratchOrIncre, which would OptimizeIncre before the queue's order is
        // honored). Seeds opt_sp_ for the compare-and-keep guard. Arms FIRST: this
        // baseline is |diff|==0 FullReuse (same PA+TL re-scored under the new DAG)
        // — safe to route through the cache via CommitIncumbent.
        DAG_Model dag_baseline =
            UpdateExtDistBasedOnTimeLimit(dag_tasks_, starting_time_limits);
        double current_config_sp =
            EvaluateSPWithPriorityVec(dag_baseline, sp_parameters_, opt_pa_);
        CommitIncumbent(opt_pa_, current_config_sp, starting_time_limits);
        return current_config_sp;
    }

    // Reopt. ResetIncumbentBaseline(true) CLEARS the cache, so the one upfront
    // from-scratch beam below runs DISARMED — it is memoryless OptimizeFromScratch,
    // a >1 change that would make ComputeTaskSetDifference throw |diff|>1 →
    // SIGABRT if the cache were armed. Arming happens AFTER, for the walk only.
    ResetIncumbentBaseline(/*from_scratch=*/true);
    double current_config_sp = CallOptimizerGivenTimeLimits(
        beam_search_width, starting_time_limits, /*from_scratch=*/true);

    // Re-arm: the baseline beam committed its champion via CommitIncumbent with
    // rta_cache_active_ false (Reset cleared it), so the cache was NOT adopted.
    // Adopt the committed triple now (|diff|==0 → Evaluate's FullReuse path,
    // zero-cost) and arm so every accepted walk step re-adopts — making the
    // |diff|<=1 invariant hold for the whole walk.
    rta_cache_active_ = true;
    std::vector<double> champion_tl = ReconstructTimeLimitVecFromResOpt();
    const std::vector<FiniteDist>& champ_rtas =
        rta_cache_.Evaluate(dag_tasks_, opt_pa_, champion_tl);
    rta_cache_.AdoptChampion(dag_tasks_, opt_pa_, champion_tl, champ_rtas);

    // Re-sync the walk vector to the adopted champion TL. The from-scratch reopt
    // committed a TL that can diverge from the incoming starting_time_limits
    // (Gaussian seed at interval 0) by >1 task on a real taskset — without
    // re-sync the first walk step would diff champion-TL vs seed-TL >1 → throw.
    // champion_tl IS that committed TL (Evaluate/AdoptChampion only touch
    // rta_cache_, never res_opt_), so reuse it. The incremental branch needs no
    // re-sync: its baseline commit already sets champion-TL == walk-start TL.
    starting_time_limits = champion_tl;
    return current_config_sp;
}

void OptimizePA_Incre_with_TimeLimits::RunIntervalDescent(
    int beam_search_width, std::vector<double>& starting_time_limits, IntervalDescentMode mode,
    const DAG_Model& dag_tasks_prev_pre_tl) {
    // The ONE descent body shared by the incremental and reopt paths. Patience
    // is mode-selected; the baseline-seed + cache arming live in
    // SeedBaselineAndArmCache(mode); the tail (BuildSerializedTaskQueue +
    // WalkSerializedTaskQueue + cache disarm) is mode-independent.
    int patience = (mode == IntervalDescentMode::Reopt)
                       ? GlobalVariables::ReoptimizationTimeLimitSearchPatience
                       : GlobalVariables::IncrementalTimeLimitSearchPatience;

    double current_config_sp =
        SeedBaselineAndArmCache(beam_search_width, starting_time_limits, mode);

    // Build the merged + weight-sorted E+L queue, then walk serially: each step's
    // UpdateRecords adopts into res_opt_, so the next step's challenger sees the
    // new champion. The committed TL tracks the adopted best, so the diff flags
    // only the walked task (|diff|==1).
    std::vector<SerializedTaskQueueEntry> queue =
        BuildSerializedTaskQueue(dag_tasks_prev_pre_tl);
    WalkSerializedTaskQueue(queue, starting_time_limits, current_config_sp,
                            patience);

    // Unconditional disarm (bit-identical for SP: nothing reads rta_cache_active_
    // between this return and the next ResetIncumbentBaseline — only
    // CommitIncumbent reads it, called only inside the descent). Scopes the cache
    // gate to exactly this descent.
    rta_cache_active_ = false;
}

void OptimizePA_Incre_with_TimeLimits::PerformSerializedTaskQueueOptimization(
    int beam_search_width, std::vector<double>& starting_time_limits,
    const DAG_Model& dag_tasks_prev_pre_tl) {
    // Thin delegating wrapper. Kept virtual so the test stubs
    // (RecordingDispatcherOpt, StartTLStub) that override it to observe the entry
    // TL vector keep working — the parent delegates to RunIntervalDescent.
    RunIntervalDescent(beam_search_width, starting_time_limits, IntervalDescentMode::Incremental,
                       dag_tasks_prev_pre_tl);
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

// Directional et_mean-bounded seed vector (see header). Mirrors
// InitializeTimeLimitsFromETConfig but uses the at-or-below helper so no perf
// task seeds above the certified WCET.
std::vector<double>
OptimizePA_Incre_with_TimeLimits::SeedTimeLimitsAtOrBelowEtMean() const {
    std::vector<double> time_limits(dag_tasks_.tasks.size());
    for (size_t i = 0; i < dag_tasks_.tasks.size(); i++) {
        if (dag_tasks_.tasks[i].timePerformancePairs.empty()) {
            time_limits[i] = -1.0;
        } else {
            size_t seed_idx = FindLargestTimeLimitAtOrBelow(
                dag_tasks_.tasks[i].timePerformancePairs,
                dag_tasks_.tasks[i].execution_time_dist.GetAvgValue());
            time_limits[i] =
                dag_tasks_.tasks[i].timePerformancePairs[seed_idx].time_limit;
        }
    }
    return time_limits;
}

double OptimizePA_Incre_with_TimeLimits::WalkOneTaskWithTimeLimitOptions(
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
        // Stop stepping once the per-interval TIME_LIMIT expired (retains
        // best-so-far). Inert within budget → prod byte-identical.
        if (BFSharedBudgetCancelled()) {
            break;
        }
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

double OptimizePA_Incre_with_TimeLimits::OptimizeOneTaskWithTimeLimit(
    size_t task_idx, std::vector<double>& starting_time_limits,
    double current_config_sp, double baseline_val, int patience) {
    // Sub-incremental walk: each trial TL calls the cache-routed
    // OptimizeIncreSingleTask (re-scores the carried PA + 1D single-task
    // re-search), reusing the incremental path's machinery.
    // et_increased per step = sign of (trial TL − committed TL).
    auto eval = [this, task_idx,
                 baseline_val](const std::vector<double>& tl) {
        bool et_up = tl[task_idx] > baseline_val;
        return OptimizeIncreSingleTask(tl, task_idx, et_up);
    };
    // Backward pass (tie-break toward smaller TL on SP ties via step<0), then a
    // forward pass from the same origin.
    current_config_sp = WalkOneTaskWithTimeLimitOptions(
        task_idx, starting_time_limits, current_config_sp, baseline_val,
        /*step=*/-1, patience, eval);
    current_config_sp = WalkOneTaskWithTimeLimitOptions(
        task_idx, starting_time_limits, current_config_sp, baseline_val,
        /*step=*/1, patience, eval);
    // Keep the working TL vector tracking the committed best so the next task's
    // baseline reflects any adoption.
    starting_time_limits = ReconstructTimeLimitVecFromResOpt();
    return current_config_sp;
}

PriorityVec OptimizePA_Incre_with_TimeLimits::Optimize_w_TL_ScratchOrIncre(
    const DAG_Model& dag_tasks_update, int beam_search_width) {
    // The safe fallback MUST be pre-computed (offline, on the cross-interval
    // worst-case DAG) before any dispatch. This dispatcher sees only dag_tasks_
    // (one interval), NOT dag_tasks_vecs_, so it cannot build the worst-case DAG
    // soundly — lazy-computing on dag_tasks_ would certify against a single
    // interval → cross-interval unsoundness. Fail loud.
    if (!HasSafeFallback()) {
        throw std::runtime_error(
            "Optimize_w_TL_ScratchOrIncre: no safe fallback pre-computed. The "
            "caller must ComputeSafeFallback(worst_case_dag) before dispatching.");
    }

    // P0.7 step 4 — open this interval's fall-back outcome record (one entry per
    // dispatch call). interval_idx is the counter BEFORE this call advances it.
    interval_fallback_log_.push_back(
        IntervalFallbackOutcome{reoptimization_interval_count_});

    // P0.7 trigger (a): an ET-jump (any task's avg ET >= 1.5x the saved old dag)
    // short-circuits the walk — adopt the precomputed safe fallback directly.
    // Runs BEFORE AbsorbUpdatedDAG so dag_tasks_ is still the saved old dag. The
    // safe fallback was certified on the cross-interval worst-case DAG, which
    // stochastically dominates every interval → safe under the jumped ETs.
    // Gated by `enable_fallback_use_`: the shipped solution runs the fallback
    // fully enabled (default true); the measurement arm sets it false to produce
    // the "without fallback" baseline (trigger (a) then never short-circuits).
    if (enable_fallback_use_ && SkipOptOnETJump(dag_tasks_update)) {
        interval_fallback_log_.back().et_jump_short_circuited = true;
        AbsorbUpdatedDAG(dag_tasks_update);  // still absorb so next interval
        AdoptSafeFallbackAsIncumbent();       // compares against this jumped ET
        reoptimization_interval_count_++;
        return opt_pa_;
    }

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
        ReOptimizePeriodic(dag_tasks_update, beam_search_width);
    } else {
        OptimizeIncre_w_TL(dag_tasks_update, beam_search_width);
    }
    // P0.7 step 2c — post-walk schedulability backstop (D7 overturn). The walk
    // finished on its own; if its FINAL `res_opt_` fails the important-task gate,
    // adopt the safe fallback. No-op when `enable_fallback_use_` is false
    // (measurement arm) → flag-off byte-identical. Runs OUTSIDE the walk, so it
    // does not interrupt the incremental search (D6 dead: walk never halted).
    AdoptFallbackIfUnschedulable();
    reoptimization_interval_count_++;
    return opt_pa_;
}

void OptimizePA_Incre_with_TimeLimits::AbsorbUpdatedDAG(
    const DAG_Model& dag_tasks_update) {
    // Shared interval-entry absorb: overwrite the DAG, apply the WCET ablation if
    // the arm requires it, refresh the per-task TL option set. Called by
    // OptimizeIncre_w_TL, ReOptimizePeriodic, and BootstrapIncumbentFromDMFast so
    // the three stay in lockstep — the option set the descent walks must match the
    // DAG the incumbent was scored under, else unchanged perf-pair tasks mis-flag.
    // Callers needing the pre-absorb DAG (Type-E diff source) capture it first.
    dag_tasks_ = dag_tasks_update;
    ApplyWCETAblationIfRequired(dag_tasks_);
    time_limit_option_for_each_task_ = RecordTimeLimitOptions(dag_tasks_);
}

void OptimizePA_Incre_with_TimeLimits::BootstrapIncumbentFromDMFast(
    const DAG_Model& dag_tasks_update) {
    // Interval-0 seed-only bootstrap. Absorb the new DAG (keeps the incumbent +
    // option set consistent for the interval-1+ incremental walk), then run ONLY
    // the DM-fast seed step — the from-scratch descent that ReOptimizePeriodic
    // runs afterwards is SKIPPED (the whole point of the arm).
    // SeedIncumbentFromDMFast calls EvaluateSPWithPriorityVec directly (not
    // CallOptimizerGivenTimeLimits), so eval_count_ stays 0: a measurable
    // guarantee no descent ran. Interval 0 only, on a fresh optimizer → cache is
    // default-constructed and the gate false; no ResetIncumbentBaseline preamble.
    AbsorbUpdatedDAG(dag_tasks_update);
    SeedIncumbentFromDMFast();
}

PriorityVec OptimizePA_Incre_with_TimeLimits::OptimizePureIncremental(
    const DAG_Model& dag_tasks_update, int beam_search_width) {
    // INCR_NO_REOPT dispatcher. count==0 → seed-only DM-fast bootstrap (no
    // descent); count>0 → warm-started incremental walk. Never ReOptimizePeriodic.
    // P0.7 step 4 — open this interval's fall-back outcome record (one entry per
    // dispatch call). interval_idx is the counter BEFORE this call advances it.
    interval_fallback_log_.push_back(
        IntervalFallbackOutcome{reoptimization_interval_count_});
    // P0.7 trigger (a): an ET-jump short-circuits the walk (same contract as
    // Optimize_w_TL_ScratchOrIncre). Runs before AbsorbUpdatedDAG; on a trip the
    // safe fallback is adopted directly under the absorbed current dag. Gated by
    // `enable_fallback_use_` (default true = shipped; false = measurement arm).
    if (enable_fallback_use_ && HasSafeFallback() &&
        SkipOptOnETJump(dag_tasks_update)) {
        interval_fallback_log_.back().et_jump_short_circuited = true;
        AbsorbUpdatedDAG(dag_tasks_update);
        AdoptSafeFallbackAsIncumbent();
        reoptimization_interval_count_++;
        return opt_pa_;
    }
    // One shared TIME_LIMIT budget per interval (the orchestrator reuses
    // incr_optimizer_ across intervals, so a construction-time start_time_ would
    // bound the whole simulation, not one interval).
    BFDLSharedBudget shared_budget(std::chrono::high_resolution_clock::now());
    if (reoptimization_interval_count_ == 0) {
        BootstrapIncumbentFromDMFast(dag_tasks_update);
    } else {
        OptimizeIncre_w_TL(dag_tasks_update, beam_search_width);
    }
    // P0.7 step 2c — post-walk schedulability backstop (D7 overturn). Same
    // contract as Optimize_w_TL_ScratchOrIncre: if the walk's FINAL result fails
    // the gate, adopt the safe fallback. No-op in the measurement arm.
    AdoptFallbackIfUnschedulable();
    reoptimization_interval_count_++;
    return opt_pa_;
}

PriorityVec OptimizePA_Incre_with_TimeLimits::OptimizeWithTimeLimitOptDisabled(
    int beam_search_width, std::vector<double>& time_limits, bool from_scratch) {
    // Bypasses the descent, so the interval reset it would do must run here.
    ResetIncumbentBaseline(from_scratch);
    InitializeTimeLimitsToSmallest(time_limits);
    CallOptimizerGivenTimeLimits(beam_search_width, time_limits, from_scratch);
    return opt_pa_;
}

PriorityVec OptimizePA_Incre_with_TimeLimits::OptimizeIncre_w_TL(
    const DAG_Model& dag_tasks_update, int beam_search_width) {
    // Capture the pre-TL env DAG before the absorb — this is the T-1 env for the
    // Type-E diff (FindEnvTaskWithDifferentEt) consumed by the serialized queue.
    DAG_Model dag_tasks_prev_pre_tl = dag_tasks_;
    AbsorbUpdatedDAG(dag_tasks_update);
    // Seed from the carried adopted TL in res_opt_, NOT the Gaussian-mean TL:
    // both diff sides must carry the adopted TL, else unchanged perf-pair tasks
    // are flagged as changed.
    std::vector<double> time_limits = ReconstructTimeLimitVecFromResOpt();
    if (GlobalVariables::disable_time_limit_opt) {
        return OptimizeWithTimeLimitOptDisabled(beam_search_width, time_limits,
                                                /*from_scratch=*/false);
    }
    // Serialized E+L queue — the incremental path's interval search (the reopt
    // path is RunIntervalDescent(Reopt); see ReOptimizePeriodic).
    PerformSerializedTaskQueueOptimization(beam_search_width, time_limits,
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

// DM + important-first group lock. Important tasks occupy the top slots,
// DM-ordered within the group (shorter deadline = higher priority); non-important
// fill the lower slots, DM-ordered within their group; every non-important task
// below every important task (the lock — non-important tasks never interfere with
// the important group, making its RTA self-contained). Ties broken by avg ET
// ascending. The scheduler's seed PA; the Python RTA ranks the important group by
// the same deadline key, so certification matches the running scheduler. DM (not
// RM) because deadlines are constrained (deadline = period * U(0.5,1.0)).
PriorityVec OptimizePA_Incre_with_TimeLimits::DeadlineMonotonicPriorityVec() {
    std::vector<int> sorted(dag_tasks_.tasks.size());
    std::iota(sorted.begin(), sorted.end(), 0);
    std::sort(sorted.begin(), sorted.end(), [&](int a, int b) {
        const Task& ta = dag_tasks_.tasks[a];
        const Task& tb = dag_tasks_.tasks[b];
        // Group lock: important tasks always rank above non-important, regardless
        // of deadline. (Within a group, deadline decides.)
        if (ta.is_important != tb.is_important)
            return ta.is_important;
        if (ta.deadline != tb.deadline)
            return ta.deadline < tb.deadline;
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

// Interval-0 DM-fast seed: DM priorities + every task at its smallest TL option,
// scored under the current dag_tasks_ and committed as the incumbent baseline.
// Shared by ResetIncumbentBaseline's interval-0 else-branch (reopt's first
// interval, via SeedBaselineAndArmCache) and BootstrapIncumbentFromDMFast (the
// INCR_NO_REOPT arm's interval-0 entry) — both arms bootstrap identically.
void OptimizePA_Incre_with_TimeLimits::SeedIncumbentFromDMFast() {
    std::vector<double> tl_min = SmallestTimeLimitVec();
    PriorityVec pa_dm = DeadlineMonotonicPriorityVec();
    DAG_Model dag_with_tl_min =
        UpdateExtDistBasedOnTimeLimit(dag_tasks_, tl_min);
    double sp_dm = EvaluateSPWithPriorityVec(dag_with_tl_min, sp_parameters_,
                                             pa_dm);
    SeedStateFromIncumbent(dag_with_tl_min, pa_dm, sp_dm, tl_min);
}

// Offline safe-fallback artifact. Seeds at the perf-WCET point (DM PA + TL ≤
// et_mean), runs a gate-governed TL walk on a throwaway sibling, and keeps the
// best-SP gate-feasible result. See header for the isolation contract.
// `worst_case_dag`: the caller-built cross-interval worst-case DAG; the walk +
// gate run on IT (not dag_tasks_) so the certificate bounds every interval.
ResourceOptResult OptimizePA_Incre_with_TimeLimits::ComputeSafeFallback(
    const DAG_Model& worst_case_dag) {
    // The gate is probabilistic — force the real ET dist (not WCET) and TL-opt-on.
    bool prev_disable_tl_opt = GlobalVariables::disable_time_limit_opt;
    bool prev_use_wcet = GlobalVariables::use_wcet_execution_time;
    GlobalVariables::disable_time_limit_opt = false;
    GlobalVariables::use_wcet_execution_time = false;

    // Own budget around the whole compute (seed eval + walk): the dispatcher path
    // runs this BEFORE its own budget is installed, so without one a runaway seed
    // SP-eval could strand the compute past TIME_LIMIT.
    BFDLSharedBudget shared_budget(std::chrono::high_resolution_clock::now());

    // Throwaway sibling carries the walk on the worst-case DAG so the live incumbent
    // is never seeded from the fallback (an online trigger, not this compute). Fresh
    // SP_Parameters from worst_case_dag (thresholds/weights are constant across
    // intervals → equals sp_parameters_ value-wise).
    SP_Parameters sp_params_worst(worst_case_dag);
    OptimizePA_Incre_with_TimeLimits fallback_solver(worst_case_dag, sp_params_worst);
    // Force the master flag true on the throwaway sibling so the safe-fallback
    // walk ALWAYS gates (the safety certificate must hold even when the live
    // optimizer is in the measurement arm with its flag off). Trigger (a) and the
    // backstop are inert here anyway: count==0 → `SkipOptOnETJump` returns false,
    // and `HasSafeFallback()` is false on a fresh sibling → neither can fire.
    fallback_solver.enable_fallback_use_ = true;

    // Seed at the certified point: DM PA + et_mean-bounded TL (TL ≤ et_mean →
    // ddl_miss_chance = 0 → seed gate-feasible, so the gate can only REJECT). On the
    // worst-case DAG each dist is a point mass → GetAvgValue()==max_time → seed TL
    // ≤ max_time across ALL intervals → feasibility-by-construction cross-interval.
    std::vector<double> tl_seed = fallback_solver.SeedTimeLimitsAtOrBelowEtMean();
    PriorityVec pa_dm = fallback_solver.DeadlineMonotonicPriorityVec();
    DAG_Model dag_with_tl =
        UpdateExtDistBasedOnTimeLimit(fallback_solver.dag_tasks_, tl_seed);
    double sp_seed =
        EvaluateSPWithPriorityVec(dag_with_tl, fallback_solver.sp_parameters_, pa_dm);
    fallback_solver.SeedStateFromIncumbent(dag_with_tl, pa_dm, sp_seed, tl_seed);

    // Gate-governed incremental walk; PA descent runs normally under the gate.
    fallback_solver.OptimizeIncre_w_TL(
        fallback_solver.dag_tasks_,
        GlobalVariables::Layer_Node_During_Incremental_Optimization);
    // (The forced `enable_fallback_use_` on the throwaway sibling needs no
    // restore — `fallback_solver` is destroyed at end-of-function and nothing
    // reads its flag after the walk; CollectResults() returns res_opt_ only.)

    // Restore the forced global flags BEFORE the loud-fail check so a throw here
    // does not leak the forced TL-opt-on / real-ET state to the caller.
    GlobalVariables::disable_time_limit_opt = prev_disable_tl_opt;
    GlobalVariables::use_wcet_execution_time = prev_use_wcet;

    // Reconstruct the candidate {pa, tl} from the sibling's result.
    ResourceOptResult candidate = fallback_solver.CollectResults();
    std::vector<double> tl_result(worst_case_dag.tasks.size());
    for (size_t i = 0; i < worst_case_dag.tasks.size(); i++) {
        int id = worst_case_dag.tasks[i].id;
        tl_result[i] = candidate.id2time_limit.count(id) ? candidate.id2time_limit.at(id)
                                                          : -1.0;
    }
    // Loud-fail: re-run the gate on the FINAL stored result. The self-contained
    // overload derives RTAs fresh (the sibling's cache is armed only inside the
    // walk; reusing it post-walk risks a |diff|>1 throw). The walk only REJECTS →
    // a miss here means the seed itself was infeasible on the worst-case DAG → not
    // walk-fixable (raising TL worsens interference) → raise loud, do NOT store.
    if (!ImportantTasksMeetThresholds(worst_case_dag, sp_params_worst,
                                      candidate.priority_vec, tl_result)) {
        CoutWarning(
            "ComputeSafeFallback: the worst-case-DAG result VIOLATES the important-"
            "task gate (a task's ddl_miss_chance > threshold). No safe fallback "
            "exists for this task set — regenerate a new task set.");
        throw std::runtime_error(
            "ComputeSafeFallback: worst-case-DAG result fails the important-task gate");
    }

    safe_fallback_ = candidate;
    return *safe_fallback_;
}

bool OptimizePA_Incre_with_TimeLimits::SkipOptOnETJump(
    const DAG_Model& dag_tasks_update) const {
    // Interval 0 has no prior dag to compare against (dag_tasks_ holds the
    // construction DAG, which equals the interval-0 update) → never short-circuit.
    if (reoptimization_interval_count_ == 0) {
        return false;
    }
    return DetectETJump(dag_tasks_, dag_tasks_update);
}

void OptimizePA_Incre_with_TimeLimits::AdoptSafeFallbackAsIncumbent() {
    // Re-score the safe fallback's {PA,TL} under the current (absorbed) dag so the
    // interval's SP reflects THIS interval's ET, not the worst-case DAG's. The
    // {PA,TL} themselves are reused as-is (certified on the worst-case DAG, which
    // stochastically dominates every interval → safe under any interval's ET).
    const ResourceOptResult& fallback = *safe_fallback_;
    std::vector<double> tl_pos(dag_tasks_.tasks.size());
    for (size_t i = 0; i < dag_tasks_.tasks.size(); i++) {
        int id = dag_tasks_.tasks[i].id;
        tl_pos[i] = fallback.id2time_limit.count(id) ? fallback.id2time_limit.at(id)
                                                     : -1.0;
    }
    DAG_Model dag_with_tl = UpdateExtDistBasedOnTimeLimit(dag_tasks_, tl_pos);
    double sp = EvaluateSPWithPriorityVec(dag_with_tl, sp_parameters_,
                                          fallback.priority_vec);
    CommitIncumbent(fallback.priority_vec, sp, tl_pos);
}

bool OptimizePA_Incre_with_TimeLimits::AdoptFallbackIfUnschedulable() {
    // D7 overturn: after the walk finishes on its own, run the gate on the FINAL
    // `res_opt_`. If the walk's result FAILS the important-task gate → adopt the
    // safe fallback (re-scored under the current dag), then RE-VERIFY the adopted
    // fallback itself; else KEEP the walk's result even if `safe_fallback_` would
    // have higher global SP. Schedulability decides, not SP. Gated by
    // `enable_fallback_use_`: a no-op in the measurement arm (the "without
    // fallback" baseline keeps whatever the plain walk produced).
    if (!enable_fallback_use_) {
        return false;
    }
    // The fallback MUST be pre-computed (the dispatcher contract —
    // `Optimize_w_TL_ScratchOrIncre` throws upfront; `OptimizePureIncremental`
    // reaches here without that guard, so this is the backstop's loud-fail). Fall-
    // back enabled with no artifact to fall back TO is a caller bug → fail loud.
    if (!HasSafeFallback()) {
        CoutWarning(
            "AdoptFallbackIfUnschedulable: fall-back use is enabled but no safe "
            "fallback artifact was pre-computed. The caller must "
            "ComputeSafeFallback(worst_case_dag) before dispatching.");
        throw std::runtime_error(
            "AdoptFallbackIfUnschedulable: no safe fallback pre-computed (fall-"
            "back enabled but artifact missing)");
    }
    // The dispatcher must initialize `res_opt_` (the walk's result) before the
    // backstop — reaching here with no incumbent is a contract violation, and
    // silently keeping an empty {pa,tl} would ship an infeasible result.
    if (!IfInitialized()) {
        CoutWarning(
            "AdoptFallbackIfUnschedulable: reached with no initialized walk "
            "result to gate (the dispatcher must bootstrap before the backstop).");
        throw std::runtime_error(
            "AdoptFallbackIfUnschedulable: no incumbent to gate (dispatcher "
            "contract violation)");
    }
    std::vector<double> tl_pos = ReconstructTimeLimitVecFromResOpt();
    // Self-contained overload (derives RTAs fresh) — no shared cache is live
    // post-walk, mirroring `ComputeSafeFallback`'s loud-fail re-gate (cpp:962).
    if (ImportantTasksMeetThresholds(dag_tasks_, sp_parameters_,
                                     res_opt_.priority_vec, tl_pos)) {
        // P0.7 step 4 — walk result passed the gate → kept (trigger b-ii, no adopt).
        if (!interval_fallback_log_.empty()) {
            interval_fallback_log_.back().backstop_verdict =
                IntervalFallbackOutcome::BackstopVerdict::kKeptWalk;
        }
        return false;  // walk result is schedulable → keep it (NOT SP-compared).
    }
    // P0.7 step 4 — the walk result FAILED the gate; record the worst important-
    // task violator BEFORE the rescue overwrites res_opt_ (the culprit is under
    // the walk's {pa, tl}, not the fallback's).
    ImportantTaskMissInfo culprit;
    if (!interval_fallback_log_.empty()) {
        culprit = WorstCaseImportantTaskMissInfo(dag_tasks_, sp_parameters_,
                                             res_opt_.priority_vec, tl_pos);
    }
    // Rescue: adopt the safe fallback (re-scored under the current dag, committed
    // via CommitIncumbent so res_opt_ now holds the fallback's {pa,tl}).
    AdoptSafeFallbackAsIncumbent();
    if (!interval_fallback_log_.empty()) {
        IntervalFallbackOutcome& entry = interval_fallback_log_.back();
        entry.backstop_verdict =
            IntervalFallbackOutcome::BackstopVerdict::kAdoptedFallback;
        entry.backstop_culprit_task_id = culprit.task_id;
        entry.backstop_culprit_miss_chance = culprit.miss_chance;
        entry.backstop_culprit_threshold = culprit.threshold;
    }
    // Re-verify the adopted fallback on the current dag. It was certified on the
    // cross-interval worst-case DAG, which stochastically dominates every interval
    // → it MUST pass here. A second failure is a certificate violation (the worst-
    // case-DAG certificate no longer covers this interval → no safe solution
    // exists for this task set) → fail loud, do NOT silently ship an infeasible
    // result (mirror `ComputeSafeFallback`'s loud-fail).
    std::vector<double> tl_fb = ReconstructTimeLimitVecFromResOpt();
    if (!ImportantTasksMeetThresholds(dag_tasks_, sp_parameters_,
                                      res_opt_.priority_vec, tl_fb)) {
        CoutWarning(
            "AdoptFallbackIfUnschedulable: the safe fallback ITSELF fails the "
            "important-task gate on the current interval (a certificate "
            "violation — the worst-case-DAG fallback no longer dominates this "
            "interval). No safe solution exists for this task set — regenerate.");
        throw std::runtime_error(
            "AdoptFallbackIfUnschedulable: safe fallback fails the important-task "
            "gate (certificate violation)");
    }
    return true;
}

// Reset the incumbent baseline before the descent's baseline eval.
// from_scratch (reopt): re-eval the carried {pa, tl} under the new DAG (or
// DM+min-TL at interval 0) and commit it, so opt_sp_ holds the compare-and-keep
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
            // Interval 0: DM priorities + smallest TL (the shared DM-fast seed).
            SeedIncumbentFromDMFast();
        }
    } else {
        opt_sp_ = -1.0;
    }
}

PriorityVec OptimizePA_Incre_with_TimeLimits::ReOptimizePeriodic(
    const DAG_Model& dag_tasks_update, int beam_search_width) {
    // Compare-and-keep reopt: re-eval the incumbent under the new DAG, seed it
    // as baseline, then run a fresh from-scratch descent. UpdateRecords' strictly-
    // greater-SP guard (tie-break lower TL-sum) preserves the incumbent if the
    // search finds nothing better.

    // Capture the pre-absorb DAG before overwriting dag_tasks_ — it is the Type-E
    // diff source for BuildSerializedTaskQueue in RunIntervalDescent(Reopt)
    // (which walks the same serialized E+L queue the incremental path uses).
    // Cheap to capture (copy on write via the absorb).
    DAG_Model dag_tasks_prev_pre_tl = dag_tasks_;
    AbsorbUpdatedDAG(dag_tasks_update);

    // The baseline reset (re-eval carried {pa, tl}, or DM+min-TL at interval 0)
    // runs inside the descent via ResetIncumbentBaseline(true). Seed from the
    // carried adopted TL when an incumbent exists; IfInitialized() auto-falls-
    // back to the Gaussian-mean TL at interval 0 (no prior state — irreducible).
    std::vector<double> time_limits = IfInitialized()
                                          ? ReconstructTimeLimitVecFromResOpt()
                                          : InitializeTimeLimitsFromETConfig();
    if (GlobalVariables::disable_time_limit_opt) {
        OptimizeWithTimeLimitOptDisabled(beam_search_width, time_limits, /*from_scratch=*/true);
    } else {
        RunIntervalDescent(beam_search_width, time_limits, IntervalDescentMode::Reopt,
                           dag_tasks_prev_pre_tl);
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
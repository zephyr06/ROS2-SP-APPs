#include "sources/Safety_Performance_Metric/RTA_Cache.h"

#include <stdexcept>
#include <unordered_set>

#include "sources/Optimization/OptimizeSP_Base.h"  // UpdateTaskSetPriorities
#include "sources/Optimization/OptimizeSP_Incre.h"  // FindTaskWithDifferentEt, DiffObj
#include "sources/Safety_Performance_Metric/PrioritySwitchAnalysis.h"  // RestEqualAfterRemoving, FindCoreOfTask, AnalyzePrioritySwitch(PerCore), PrioritySwitchStatus/Analysis
#include "sources/Safety_Performance_Metric/SP_Metric.h"  // ApplyTimeLimitsToTasksExecutionTime
#include "sources/Utils/Parameters.h"  // GlobalVariables::Granularity

namespace SP_OPT_PA {

namespace {

// Identity HP-prefix (empty HP set) — matches RTA.cpp:86.
FiniteDist IdentityPrefix() {
    return FiniteDist(std::vector<Value_Proba>{Value_Proba(0, 1.0)});
}

// Roll `hp_tasks_et_conv` forward by one task, matching RTA.cpp:95-97:
// Compress(Granularity*1) then Convolve(task ET).
void RollPrefix(FiniteDist& hp_tasks_et_conv, const FiniteDist& et) {
    hp_tasks_et_conv.CompressDistributionWithOnlySize(
        GlobalVariables::Granularity * 1);
    hp_tasks_et_conv.Convolve(et);
}

}  // namespace

// Priority-analysis utilities (declared in PrioritySwitchAnalysis.h).

bool RestEqualAfterRemoving(const std::vector<int>& candidate_order,
                            const std::vector<int>& champion_order,
                            int task_id) {
    size_t i = 0, j = 0;
    while (i < candidate_order.size() && j < champion_order.size()) {
        if (candidate_order[i] == task_id) { i++; continue; }
        if (champion_order[j] == task_id) { j++; continue; }
        if (candidate_order[i] != champion_order[j]) return false;
        i++; j++;
    }
    // Drain any trailing `task_id` entries on either side; any other trailing
    // task means the orders differ in length after removal (unequal counts).
    while (i < candidate_order.size() && candidate_order[i] == task_id) i++;
    while (j < champion_order.size() && champion_order[j] == task_id) j++;
    return i == candidate_order.size() && j == champion_order.size();
}

int FindCoreOfTask(const std::unordered_map<int, std::vector<int>>& per_core,
                   int task_id) {
    for (const auto& [core, order] : per_core) {
        for (int tid : order)
            if (tid == task_id)
                return core;
    }
    return -1;
}

PrioritySwitchStatus AnalyzePrioritySwitchPerCore(
    const std::vector<int>& candidate_order,
    const std::vector<int>& champion_order, PrioritySwitchAnalysis& out) {
    if (candidate_order == champion_order)
        return PrioritySwitchStatus::AllIdentical;

    size_t i = 0;
    while (i < candidate_order.size() &&
           candidate_order[i] == champion_order[i])
        i++;
    if (i == candidate_order.size())
        return PrioritySwitchStatus::NotSingle;  // defensive: flagged diff but
                                                 // equal
    int candidate_task = candidate_order[i], champion_task = champion_order[i];
    int moved_task_id = -1;
    if (RestEqualAfterRemoving(candidate_order, champion_order,
                               candidate_task)) {
        moved_task_id = candidate_task;
    } else if (RestEqualAfterRemoving(candidate_order, champion_order,
                                      champion_task)) {
        moved_task_id = champion_task;
    } else {
        return PrioritySwitchStatus::NotSingle;  // neither removal matches ⇒ >1
                                                 // move
    }
    for (size_t j = 0; j < champion_order.size(); j++)
        if (champion_order[j] == moved_task_id) {
            out.old_pos = static_cast<int>(j);
            break;
        }
    for (size_t j = 0; j < candidate_order.size(); j++)
        if (candidate_order[j] == moved_task_id) {
            out.new_pos = static_cast<int>(j);
            break;
        }
    out.moved_task_id = moved_task_id;
    return PrioritySwitchStatus::SingleChange;
}

PrioritySwitchAnalysis AnalyzePrioritySwitch(
    const std::unordered_map<int, std::vector<int>>& candidate_per_core,
    const std::unordered_map<int, std::vector<int>>& champion_per_core) {
    static const std::vector<int> empty_vec;
    PrioritySwitchAnalysis result;

    // (1) Per-core size check.
    for (const auto& [core, candidate_order] : candidate_per_core) {
        auto champ_it = champion_per_core.find(core);
        const std::vector<int>& champion_order =
            champ_it != champion_per_core.end() ? champ_it->second : empty_vec;
        if (candidate_order.size() != champion_order.size())
            return result;
    }
    for (const auto& [core, champion_order] : champion_per_core) {
        if (candidate_per_core.find(core) != candidate_per_core.end())
            continue;
        if (!champion_order.empty())
            return result;  // champ core emptied/migrated
    }

    // (2) Find the one core (if any) whose order differs, then delegate the
    // per-core remove-and-compare to AnalyzePrioritySwitchPerCore.
    int changed_core = -1;
    for (const auto& [core, candidate_order] : candidate_per_core) {
        auto champ_it = champion_per_core.find(core);
        const std::vector<int>& champion_order =
            champ_it != champion_per_core.end() ? champ_it->second : empty_vec;
        const PrioritySwitchStatus per_core_status =
            AnalyzePrioritySwitchPerCore(candidate_order, champion_order,
                                         result);
        if (per_core_status != PrioritySwitchStatus::AllIdentical) {
            if (changed_core != -1)
                return result;  // 2nd changed core ⇒ >1
            if (per_core_status == PrioritySwitchStatus::NotSingle)
                return result;
            changed_core = core;
        }
    }

    // (3) No order diff ⇒ no priority change.
    if (changed_core == -1) {
        result.status = PrioritySwitchStatus::AllIdentical;
        return result;
    }

    result.status = PrioritySwitchStatus::SingleChange;
    result.changed_core = changed_core;
    return result;
}

// Per-core priority ORDER from an ALREADY-prioritized TaskSet: for each
// processorId, the task ids on that core in priority order (the order
// `prioritized` already carries). The shared partition body of
// PerCoreOrderFromPa + the champion-order cache build.
std::unordered_map<int, std::vector<int>> RTACache::PerCoreOrderOfPrioritized(
    const TaskSet& prioritized) const {
    std::unordered_map<int, std::vector<int>> order;
    for (const Task& t : prioritized) {
        order[t.processorId].push_back(t.id);
    }
    return order;
}

// Per-core priority ORDER from (dag, pa): for each processorId, the task ids
// on that core in ascending-priority order (lower pa position = higher
// priority). `pa` IS that ordering — UpdateTaskSetPriorities assigns
// priority=i to pa[i], and SortTasksByPriority sorts by priority ascending
// with distinct keys, yielding exactly pa[0], pa[1], ..., pa[N-1]. So walking
// `pa` and bucketing by processorId is identical to the old
// PerCoreOrderOfPrioritized(UpdateTaskSetPriorities(...)) — without the full
// TaskSet copy + the O(N log N) sort. Relies on the same id==index invariant
// every pa-as-index site relies on (UpdateTaskSetPriorities asserts it; every
// YAML fixture numbers tasks 0,1,2,... from index 0). processorId is untouched
// by the priority bake, so reading it from raw dag_tasks.tasks is sound.
std::unordered_map<int, std::vector<int>> RTACache::PerCoreOrderFromPa(
    const DAG_Model& dag_tasks, const PriorityVec& pa) const {
    std::unordered_map<int, std::vector<int>> order;
    for (int tid : pa) {
        order[dag_tasks.tasks[tid].processorId].push_back(tid);
    }
    return order;
}

// Full N-task RTA for (dag, pa, tl) + store the champion triple + flat rta_ +
// per-core HP-prefix checkpoints. Bit-identical to ProbabilisticRTA_TaskSet:
// bake TL → apply pa (sorts HP-first) → ProbabilisticRTA_TaskSet (partitions by
// processorId internally). The HP-prefix checkpoints are re-rolled separately
// (efficiency TODO: ProbabilisticRTA_TaskSet could emit them, avoiding a second
// per-core ET-convolution pass — deferred until the cache is wired hot).
const std::vector<FiniteDist>& RTACache::Initialize(
    const DAG_Model& dag_tasks, const PriorityVec& pa,
    const std::vector<double>& tl) {
    // Bake TL → apply pa (sorts HP-first) → RTA, writing each artifact directly
    // into its cache member: champ_tasks_baked_ is the canonical-order TL-bake
    // (what FindTaskWithDifferentEt reads by index), champ_prioritized_ is the
    // pa-sorted form (what Evaluate's reindex + RebuildPrefixes read), and
    // champ_per_core_ is the per-core order IsSingleTaskChange reads. All
    // helpers return-by-value + take const-ref, so they never mutate the member
    // we hand them.
    champ_tasks_baked_ =
        ApplyTimeLimitsToTasksExecutionTime(dag_tasks.tasks, tl);
    champ_prioritized_ = UpdateTaskSetPriorities(champ_tasks_baked_, pa);
    champ_per_core_ = PerCoreOrderOfPrioritized(champ_prioritized_);
    rta_ = ProbabilisticRTA_TaskSet(champ_prioritized_);

    RebuildPrefixes(champ_prioritized_);
    candidate_rta_ = rta_;
    return rta_;
}

// Cheap commit: store the champion triple + the caller-supplied rtas + rebuild
// hp_prefix_per_core_ by re-rolling the per-core ET-convolution. No RTA work
// (no ResolvePreemptions, no GetRTA_OneTask).
void RTACache::AdoptChampion(const DAG_Model& dag_tasks, const PriorityVec& pa,
                             const std::vector<double>& tl,
                             const std::vector<FiniteDist>& rtas) {
    rta_ = rtas;
    candidate_rta_ = rtas;

    // Same direct-to-member bake as Initialize; AdoptChampion takes caller-
    // supplied rtas so it skips the ProbabilisticRTA_TaskSet call.
    champ_tasks_baked_ =
        ApplyTimeLimitsToTasksExecutionTime(dag_tasks.tasks, tl);
    champ_prioritized_ = UpdateTaskSetPriorities(champ_tasks_baked_, pa);
    champ_per_core_ = PerCoreOrderOfPrioritized(champ_prioritized_);
    RebuildPrefixes(champ_prioritized_);
}

// Rebuild hp_prefix_per_core_[core][i] = HP-ET convolution of the priority-
// sorted tasks [0, i) on `core` (exactly what 3-arg GetRTA_OneTask consumes),
// by rolling the ET-convolution forward in priority order. Reproduces the
// prefixes ProbabilisticRTA_TaskSet_SingleCore emits (same bake, same sort).
void RTACache::RebuildPrefixes(const TaskSet& tasks_prioritized) {
    std::unordered_map<int, TaskSet> per_core =
        ExtractTaskSetPerProcessor(tasks_prioritized);
    hp_prefix_per_core_.clear();
    for (const auto& [core, core_tasks] : per_core) {
        // core_tasks is priority-sorted
        std::vector<FiniteDist>& prefixes = hp_prefix_per_core_[core];
        prefixes.assign(core_tasks.size(), IdentityPrefix());
        FiniteDist hp_tasks_et_conv = IdentityPrefix();
        for (size_t i = 0; i < core_tasks.size(); i++) {
            prefixes[i] = hp_tasks_et_conv;
            RollPrefix(hp_tasks_et_conv, core_tasks[i].execution_time_dist);
        }
    }
}

// The single shared single-change analyzer. See the header doc for the
// algorithm. Returns true + fills `out` iff |diff| <= 1; false otherwise.
// Never throws; leaves `out` untouched on false. No-champion → false.
bool RTACache::IsSingleTaskChange(const DAG_Model& dag_tasks,
                                  const PriorityVec& pa,
                                  const std::vector<double>& tl,
                                  TaskSetDifference& out) const {
    // Champion side reads champ_tasks_baked_ (cached canonical-order TL-bake);
    // only the candidate bake is per-call (its tl genuinely changes).
    // FindTaskWithDifferentEt walks .tasks[i] by index, so it needs canonical
    // (not pa-sorted) order on both sides. The TaskSet overload takes the two
    // baked TaskSets directly, so no throwaway DAG_Model (graph + per-processor
    // maps) is built just to overwrite .tasks.
    TaskSet cand_tasks_baked =
        ApplyTimeLimitsToTasksExecutionTime(dag_tasks.tasks, tl);
    std::vector<DiffObj> et_diff =
        FindTaskWithDifferentEt(champ_tasks_baked_, cand_tasks_baked);

    // (1) ET diff: >1 ET-changed task ⇒ not single.
    if (et_diff.size() > 1)
        return false;

    std::unordered_map<int, std::vector<int>> candidate_per_core =
        PerCoreOrderFromPa(dag_tasks, pa);

    // (2) Priority-order analysis on the two per-core maps. Returns the status
    // +, when single, the changed core + the moved task's locators for the
    // pure-priority-move case. The ET-known case re-runs the remove-and-compare
    // below with the known task id.
    PrioritySwitchAnalysis pa_switch =
        AnalyzePrioritySwitch(candidate_per_core, champ_per_core_);
    if (pa_switch.status == PrioritySwitchStatus::NotSingle)
        return false;

    int changed_core = pa_switch.status == PrioritySwitchStatus::SingleChange
                           ? pa_switch.changed_core
                           : -1;

    // (3) No ET diff + no per-core order diff ⇒ |diff|==0 (identity).
    if (et_diff.empty() && changed_core == -1) {
        out = TaskSetDifference{-1, -1, -1, -1};
        return true;
    }

    // (4) Cross-check: if both an ET diff and a priority move exist, they must
    // be on the SAME core (the moved task IS the ET-changed task) ⇒ one merged
    // change. Different cores ⇒ 2 changes ⇒ not single.
    int et_task_id = et_diff.empty() ? -1 : et_diff.front().task_id;
    int et_core =
        et_task_id != -1 ? FindCoreOfTask(candidate_per_core, et_task_id) : -1;
    if (et_task_id != -1 && changed_core != -1 && et_core != changed_core)
        return false;

    int moved_task_id, old_pos, new_pos;
    if (et_task_id != -1) {
        // ET-known branch: remove the ET-changed task from both orders on its
        // ACTUAL core and check the rest matches — else a SEPARATE task also
        // moved ⇒ not single. Its own priority move is absorbed by removing it
        // (combined ET+move = single change). changed_core == -1 here ⟺ ET-only
        // change ⟺ old_pos == new_pos.
        const std::vector<int>& candidate_order =
            candidate_per_core.at(et_core);
        const std::vector<int>& champion_order = champ_per_core_.at(et_core);
        moved_task_id = et_task_id;
        for (size_t i = 0; i < candidate_order.size(); i++)
            if (candidate_order[i] == et_task_id) {
                new_pos = static_cast<int>(i);
                break;
            }
        for (size_t i = 0; i < champion_order.size(); i++)
            if (champion_order[i] == et_task_id) {
                old_pos = static_cast<int>(i);
                break;
            }
        if (!RestEqualAfterRemoving(candidate_order, champion_order,
                                    et_task_id))
            return false;
        changed_core = et_core;
    } else {
        // Pure-priority-move branch (0 ET diff): moved task + locators were
        // discovered by AnalyzePrioritySwitch.
        moved_task_id = pa_switch.moved_task_id;
        old_pos = pa_switch.old_pos;
        new_pos = pa_switch.new_pos;
    }

    out = TaskSetDifference{moved_task_id, changed_core, old_pos, new_pos};
    return true;
}

// The difference between the candidate and the stored champion. Assumes the
// single-change invariant (|diff| <= 1); THROWS when >1 task differs.
// Returns a LOCATOR set (no verdict): changed_task_id==-1 ⟺ |diff|==0.
TaskSetDifference RTACache::ComputeTaskSetDifference(
    const DAG_Model& dag_tasks, const PriorityVec& pa,
    const std::vector<double>& tl) const {
    if (!HasChampion())
        return TaskSetDifference{-1, -1, -1, -1};

    TaskSetDifference diff;
    if (!IsSingleTaskChange(dag_tasks, pa, tl, diff)) {
        throw std::runtime_error(
            "RTACache::ComputeTaskSetDifference: candidate differs from "
            "champion by more than one task — violates the single-change "
            "invariant. Call IsSingleTaskChange to guard multi-change "
            "candidates.");
    }
    return diff;
}

// Per-task reuse view from the same diff: no champion / |diff|>1 → all NoReuse;
// |diff|==0 → all FullReuse; |diff|==1 → SAME core as the change is NoReuse,
// DIFFERENT core is FullReuse.
std::vector<RTAReusePerTask> RTACache::ClassifyReusePerTask(
    const DAG_Model& dag_tasks, const PriorityVec& pa,
    const std::vector<double>& tl) const {
    std::vector<RTAReusePerTask> result(dag_tasks.tasks.size(),
                                        RTAReusePerTask::NoReuse);
    if (!HasChampion())
        return result;

    TaskSetDifference diff = ComputeTaskSetDifference(dag_tasks, pa, tl);
    // |diff|==0 (changed_task_id == -1) → every task FullReuse.
    if (diff.changed_task_id == -1) {
        std::fill(result.begin(), result.end(), RTAReusePerTask::FullReuse);
        return result;
    }
    // |diff|==1: same-core (diff.core) → NoReuse, cross-core → FullReuse.
    std::fill(result.begin(), result.end(), RTAReusePerTask::FullReuse);
    std::unordered_map<int, std::vector<int>> cand_order =
        PerCoreOrderFromPa(dag_tasks, pa);
    const std::vector<int>& changed_core_tasks = cand_order.at(diff.core);
    for (int tid : changed_core_tasks) {
        result[tid] = RTAReusePerTask::NoReuse;
    }
    return result;
}

// Candidate RTA via the single-change invariant. Writes candidate_rta_
// (champion rta_ untouched), returns &candidate_rta_. Dispatch:
//   no champion            → Initialize (full compute).
//   changed_task_id == -1  → copy champion rta_ to candidate_rta_, return.
//   changed_task_id >= 0   → seed candidate_rta_ with champion rta_ (FullReuse
//                            tasks keep it), recompute the NoReuse tasks on
//                            diff.core via GetRTA_OneTask in candidate priority
//                            order.
const std::vector<FiniteDist>& RTACache::Evaluate(
    const DAG_Model& dag_tasks, const PriorityVec& pa,
    const std::vector<double>& tl) {
    if (!HasChampion()) {
        return Initialize(dag_tasks, pa, tl);
    }

    // Reindex the champion RTA by TASK ID, not by positional copy. rta_ is
    // indexed by CHAMPION priority-position, but candidate_rta_ is consumed as
    // CANDIDATE priority-position (ObtainSP_Full_From_NodeRTAs reads node_rtas[k]
    // as the candidate's tasks_prioritized[k]). When champion PA != candidate PA
    // a positional copy `candidate_rta_ = rta_` would put each FullReuse task's
    // champion RTA in the WRONG slot — a silent scramble. Map each task's
    // champion RTA into its candidate priority-position slot instead.
    TaskSet tasks_baked =
        ApplyTimeLimitsToTasksExecutionTime(dag_tasks.tasks, tl);
    TaskSet tasks_prioritized = UpdateTaskSetPriorities(tasks_baked, pa);
    std::unordered_map<int, int> task_id2index;
    for (size_t i = 0; i < tasks_prioritized.size(); i++) {
        task_id2index[tasks_prioritized[i].id] = static_cast<int>(i);
    }
    // Build the candidate per-core partition ONCE; it serves the recompute loop
    // below. (Baking only changes ET, never processorId or priority order, so
    // this == PerCoreOrderFromPa(dag, pa).)
    std::unordered_map<int, TaskSet> per_core =
        ExtractTaskSetPerProcessor(tasks_prioritized);

    // Reuse verdict from the shared classifier (same logic Evaluate used to
    // inline here): |diff|==0 → all FullReuse; |diff|==1 → tasks on diff.core
    // are NoReuse, every other core FullReuse. any_recompute iff at least one
    // NoReuse slot exists (i.e. |diff|==1 on a non-empty core).
    std::vector<RTAReusePerTask> verdict_per_task =
        ClassifyReusePerTask(dag_tasks, pa, tl);
    bool any_recompute = false;
    for (RTAReusePerTask v : verdict_per_task) {
        if (v == RTAReusePerTask::NoReuse) {
            any_recompute = true;
            break;
        }
    }

    // Size the buffer once; every slot is written before read (reindex fills
    // FullReuse slots, recompute overwrites the NoReuse slots). `resize` (not
    // `assign`) since nothing needs zero-init — no-op when already sized to N.
    candidate_rta_.resize(rta_.size());
    {
        // Champion priority-position → task id, mirroring how rta_ was built.
        // champ_prioritized_ is cached (invariant across one champion lifetime),
        // so this reindex reads it directly instead of re-baking the champion.
        for (size_t k = 0; k < champ_prioritized_.size() && k < rta_.size(); k++) {
            int tid = champ_prioritized_[k].id;
            auto it = task_id2index.find(tid);
            if (it != task_id2index.end()) {
                candidate_rta_[it->second] = rta_[k];
            }
        }
    }

    if (!any_recompute)
        return candidate_rta_;  // |diff|==0: pure reuse (reindexed by task id)

    // Recompute the NoReuse tasks. Walk each core in CANDIDATE priority order so
    // each recompute sees the correct candidate-ET HP set. FullReuse tasks are
    // skipped (their seeded value stays) but still folded into the rolling HP
    // prefix so a later NoReuse task's HP set is complete.
    for (const auto& [core, core_tasks] : per_core) {
        // Mirror ProbabilisticRTA_TaskSet_SingleCore: maintain a rolling
        // hp_tasks_et_conv (ET convolution of every higher-priority task on this
        // core so far, snapshotted BEFORE the current task is folded in) and call
        // the 3-arg GetRTA_OneTask with it. The 3-arg form Compresses the running
        // RTA ONCE then Convolves against the pre-built prefix — bit-identical to
        // the oracle. (The 2-arg form Compresses+Convolves PER HP task on the
        // running RTA; with >=2 HP tasks and convolved support past Granularity
        // the differing lossy compress count can diverge from the oracle.)
        TaskSet hp_tasks;
        FiniteDist hp_tasks_et_conv = IdentityPrefix();
        for (const Task& task_curr : core_tasks) {
            if (verdict_per_task[task_curr.id] == RTAReusePerTask::NoReuse) {
                candidate_rta_[task_id2index.at(task_curr.id)] =
                    GetRTA_OneTask(task_curr, hp_tasks, hp_tasks_et_conv);
            }
            hp_tasks.push_back(task_curr);
            RollPrefix(hp_tasks_et_conv, task_curr.execution_time_dist);
        }
    }
    return candidate_rta_;
}

}  // namespace SP_OPT_PA

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

// --- priority-analysis utilities (defined here, declared in
// PrioritySwitchAnalysis.h) ----------------------------------------------
// Bodies are the spec the header documents; see that header for the algorithm
// and the test suite (testRTA.cpp) for the pinned edge cases.

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

// Per-core priority ORDER from (dag, pa): for each processorId, the task ids
// on that core in ascending-priority order (lower pa position = higher
// priority; pa[i] = task id at priority index i). Mirrors
// ProbabilisticRTA_TaskSet_SingleCore's sort (RTA.cpp:72-74) applied AFTER
// UpdateTaskSetPriorities sets priority=i.
std::unordered_map<int, std::vector<int>> RTACache::PerCoreOrderFromPa(
    const DAG_Model& dag_tasks, const PriorityVec& pa) const {
    TaskSet prioritized = UpdateTaskSetPriorities(dag_tasks.tasks, pa);
    std::unordered_map<int, std::vector<int>> order;
    for (const Task& t : prioritized) {
        order[t.processorId].push_back(t.id);
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
    dag_champion_ = dag_tasks;
    pa_champion_ = pa;
    tl_champion_ = tl;

    TaskSet tasks_baked =
        ApplyTimeLimitsToTasksExecutionTime(dag_tasks.tasks, tl);
    TaskSet tasks_prioritized = UpdateTaskSetPriorities(tasks_baked, pa);
    rta_ = ProbabilisticRTA_TaskSet(tasks_prioritized);

    RebuildPrefixes(tasks_prioritized);
    candidate_rta_ = rta_;
    return rta_;
}

// Cheap commit: store the champion triple + the caller-supplied rtas (a vector
// Evaluate/Initialize returned for this triple) + rebuild hp_prefix_per_core_
// by re-rolling the per-core ET-convolution. No RTA work (no
// ResolvePreemptions, no GetRTA_OneTask).
void RTACache::AdoptChampion(const DAG_Model& dag_tasks, const PriorityVec& pa,
                             const std::vector<double>& tl,
                             const std::vector<FiniteDist>& rtas) {
    dag_champion_ = dag_tasks;
    pa_champion_ = pa;
    tl_champion_ = tl;
    rta_ = rtas;
    candidate_rta_ = rtas;

    TaskSet tasks_baked =
        ApplyTimeLimitsToTasksExecutionTime(dag_tasks.tasks, tl);
    TaskSet tasks_prioritized = UpdateTaskSetPriorities(tasks_baked, pa);
    RebuildPrefixes(tasks_prioritized);
}

// Rebuild hp_prefix_per_core_[core][i] = HP-ET convolution of the
// priority-sorted tasks [0, i) on `core` (i.e. exactly what 3-arg
// GetRTA_OneTask consumes), by rolling the ET-convolution forward in priority
// order. Reproduces exactly the prefixes ProbabilisticRTA_TaskSet_SingleCore
// emits (same bake, same sort).
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

// Boolean predicate: does the candidate differ from the stored champion by AT
// MOST one task (ET and/or per-core priority position)? Thin non-throwing
// wrapper over TryComputeSingleChange (the merged algorithm). See the header
// doc for the find-ET-diff → remove-one → compare-rest algorithm.
bool RTACache::IsSingleTaskChange(const DAG_Model& dag_tasks,
                                  const PriorityVec& pa,
                                  const std::vector<double>& tl) const {
    TaskSetDifference ignored;
    return TryComputeSingleChange(dag_tasks, pa, tl, ignored);
}

// The single shared single-change analyzer. See the header doc for the full
// algorithm. Returns true + fills `out` iff |diff| <= 1; false otherwise.
// Never throws; leaves `out` untouched on false. The no-champion case returns
// false (ComputeTaskSetDifference short-circuits it to {-1,...} before here).
//
// "Remove one task from both vectors, compare the rest": if removing a task
// from both the champion and candidate per-core order makes them equal, then
// that task's relocation (or ET change, if it's the known ET-diff task) is the
// SINGLE change; everything else is identical. If neither removal matches, a
// second task also moved → not single.
bool RTACache::TryComputeSingleChange(const DAG_Model& dag_tasks,
                                      const PriorityVec& pa,
                                      const std::vector<double>& tl,
                                      TaskSetDifference& out) const {
    // Dedup'd baked-DAG dance (was repeated in IsSingleTaskChange +
    // ComputeTaskSetDifference before the merge).
    TaskSet champ_baked =
        ApplyTimeLimitsToTasksExecutionTime(dag_champion_.tasks, tl_champion_);
    TaskSet cand_baked =
        ApplyTimeLimitsToTasksExecutionTime(dag_tasks.tasks, tl);
    DAG_Model champ_dag_baked = dag_champion_;
    champ_dag_baked.tasks = champ_baked;
    DAG_Model cand_dag_baked = dag_tasks;
    cand_dag_baked.tasks = cand_baked;
    std::vector<DiffObj> et_diff =
        FindTaskWithDifferentEt(champ_dag_baked, cand_dag_baked);

    // (1) ET diff: >1 ET-changed task ⇒ not single.
    if (et_diff.size() > 1)
        return false;

    std::unordered_map<int, std::vector<int>> candidate_per_core =
        PerCoreOrderFromPa(dag_tasks, pa);
    std::unordered_map<int, std::vector<int>> champion_per_core =
        PerCoreOrderFromPa(dag_champion_, pa_champion_);

    // (2) Priority-order analysis on the two per-core maps. Returns the status
    // (not single / all identical / single change) and, when single, the
    // changed core + the moved task's locators for the pure-priority-move case.
    // For the ET-known case we re-run the remove-and-compare below with the
    // known task id (it isn't a "discover the moved task" case).
    PrioritySwitchAnalysis pa_switch =
        AnalyzePrioritySwitch(candidate_per_core, champion_per_core);
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
    // be on the SAME core (the moved task is the ET-changed task) ⇒ one merged
    // change. Different cores ⇒ 2 changes ⇒ not single.
    int et_task_id = et_diff.empty() ? -1 : et_diff.front().task_id;
    int et_core =
        et_task_id != -1 ? FindCoreOfTask(candidate_per_core, et_task_id) : -1;
    if (et_task_id != -1 && changed_core != -1 && et_core != changed_core)
        return false;

    int moved_task_id, old_pos, new_pos;
    if (et_task_id != -1) {
        // ET-known branch: the changed task is et_task_id. Remove it from both
        // orders on its ACTUAL core (et_core) and check the rest matches — else
        // a SEPARATE task also moved ⇒ not single. The ET-diff task's own
        // priority move (if any) is absorbed by removing it (combined ET+move ⇒
        // single change). changed_core == -1 here ⟺ ET-only change (no priority
        // move) ⟺ old_pos == new_pos; set the locator core to where it sits.
        const std::vector<int>& candidate_order =
            candidate_per_core.at(et_core);
        const std::vector<int>& champion_order = champion_per_core.at(et_core);
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
        // Pure-priority-move branch (0 ET diff): the moved task + locators were
        // discovered by AnalyzePrioritySwitch.
        moved_task_id = pa_switch.moved_task_id;
        old_pos = pa_switch.old_pos;
        new_pos = pa_switch.new_pos;
    }

    out = TaskSetDifference{moved_task_id, changed_core, old_pos, new_pos};
    return true;
}

// The difference between the candidate and the stored champion. Assumes the
// P1.10 single-change invariant (|diff| <= 1); THROWS when >1 task differs.
// Delegates the analysis to TryComputeSingleChange (the merged
// IsSingleTaskChange algorithm). Returns a LOCATOR set (no klass):
// changed_task_id==-1 ⟺ |diff|==0.
TaskSetDifference RTACache::ComputeTaskSetDifference(
    const DAG_Model& dag_tasks, const PriorityVec& pa,
    const std::vector<double>& tl) const {
    if (!HasChampion())
        return TaskSetDifference{-1, -1, -1, -1};

    TaskSetDifference diff;
    if (!TryComputeSingleChange(dag_tasks, pa, tl, diff)) {
        throw std::runtime_error(
            "RTACache::ComputeTaskSetDifference: candidate differs from "
            "champion "
            "by more than one task — violates the P1.10 single-change "
            "invariant. "
            "Call IsSingleTaskChange to guard multi-change candidates.");
    }
    return diff;  // changed_task_id == -1 ⟺ |diff|==0
}

// Per-task reuse view from the same diff:
//   no champion / |diff|>1 → every task NoReuse
//   |diff|==0              → every task FullReuse
//   |diff|==1              → every task on the SAME core as the change is
//                            Recompute; every task on a DIFFERENT core is
//                            FullReuse. (v1: cross-core reuse; same-core
//                            suffix reuse is a later refinement.)
std::vector<RTAReusePerTask> RTACache::ClassifyReusePerTask(
    const DAG_Model& dag_tasks, const PriorityVec& pa,
    const std::vector<double>& tl) const {
    std::vector<RTAReusePerTask> result(dag_tasks.tasks.size(),
                                        RTAReusePerTask::NoReuse);
    if (!HasChampion())
        return result;

    TaskSetDifference diff = ComputeTaskSetDifference(dag_tasks, pa, tl);
    // Derive the per-task verdict from the locators (no klass field):
    // changed_task_id == -1 ⟺ |diff|==0 → every task FullReuse.
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
// (champion rta_ untouched), returns &candidate_rta_. Dispatch is derived from
// the locators ComputeTaskSetDifference returns (no klass field):
//   no champion            → Initialize (full compute).
//   changed_task_id == -1  → copy champion rta_ to candidate_rta_, return.
//   changed_task_id >= 0   → seed candidate_rta_ with champion rta_, then for
//                            each task on a DIFFERENT core than diff.core reuse
//                            verbatim; for each task on diff.core recompute via
//                            GetRTA_OneTask in candidate priority order.
const std::vector<FiniteDist>& RTACache::Evaluate(
    const DAG_Model& dag_tasks, const PriorityVec& pa,
    const std::vector<double>& tl) {
    if (!HasChampion()) {
        return Initialize(dag_tasks, pa, tl);
    }

    // Verdict-driven dispatch: ClassifyReusePerTask says per task whether to
    // reuse the champion RTA (FullReuse) or recompute (NoReuse). Seed every
    // task with the champion RTA, then overwrite only the NoReuse tasks. This
    // is the generalization point — a future same-core-suffix refinement only
    // needs ClassifyReusePerTask to emit ReuseHpTasksEt + a branch here, not a
    // rewrite of Evaluate.
    std::vector<RTAReusePerTask> verdict =
        ClassifyReusePerTask(dag_tasks, pa, tl);

    // P1.12 Phase 2 item 1b — reindex the champion RTA by TASK ID, not by a
    // positional copy. rta_ is indexed by CHAMPION priority-position (the oracle
    // ProbabilisticRTA_TaskSet writes rtas[task_id2index[...]] over the priority-
    // sorted tasks), but candidate_rta_ is consumed as CANDIDATE priority-
    // position (ObtainSP_Full_From_NodeRTAs reads node_rtas[k] as the candidate's
    // tasks_prioritized[k]). When champion PA != candidate PA the two priority-
    // position orderings differ, so `candidate_rta_ = rta_` (a positional copy)
    // would put each FullReuse task's champion RTA in the WRONG slot — a silent
    // scramble (the :285 divergence, pinned by
    // Evaluate_PriorityMove_CrossCoreScramble_BitIdenticalToOracle). Map each
    // task's champion RTA into its CANDIDATE priority-position slot instead.
    TaskSet tasks_baked =
        ApplyTimeLimitsToTasksExecutionTime(dag_tasks.tasks, tl);
    TaskSet tasks_prioritized = UpdateTaskSetPriorities(tasks_baked, pa);
    std::unordered_map<int, int> task_id2index;
    for (size_t i = 0; i < tasks_prioritized.size(); i++) {
        task_id2index[tasks_prioritized[i].id] = static_cast<int>(i);
    }
    candidate_rta_.assign(rta_.size(), FiniteDist({Value_Proba(0, 1.0)}));
    {
        // Champion priority-position -> task id, mirroring how rta_ was built
        // (Initialize/AdoptChampion bake champion TL + apply champion PA).
        TaskSet champ_baked =
            ApplyTimeLimitsToTasksExecutionTime(dag_champion_.tasks, tl_champion_);
        TaskSet champ_prioritized =
            UpdateTaskSetPriorities(champ_baked, pa_champion_);
        for (size_t k = 0; k < champ_prioritized.size() && k < rta_.size(); k++) {
            int tid = champ_prioritized[k].id;
            auto it = task_id2index.find(tid);
            if (it != task_id2index.end()) {
                candidate_rta_[it->second] = rta_[k];
            }
        }
    }

    bool any_recompute = false;
    for (RTAReusePerTask v : verdict) {
        if (v == RTAReusePerTask::NoReuse) {
            any_recompute = true;
            break;
        }
    }
    if (!any_recompute)
        return candidate_rta_;  // |diff|==0: pure reuse (reindexed by task id)

    // Recompute the NoReuse tasks. Walk each core in CANDIDATE priority order
    // so each recompute sees the correct candidate-ET HP set (the tasks above
    // it on the same core, accumulated as we walk). FullReuse tasks are skipped
    // (their seeded value stays) but still pushed to hp_tasks so a later
    // NoReuse task's HP set is complete.
    std::unordered_map<int, TaskSet> per_core =
        ExtractTaskSetPerProcessor(tasks_prioritized);

    for (const auto& [core, core_tasks] : per_core) {
        // core_tasks is in candidate priority order. Walk it exactly as
        // ProbabilisticRTA_TaskSet_SingleCore (RTA.cpp:88-113) does: maintain a
        // rolling hp_tasks_et_conv (= the ET convolution of every higher-priority
        // task on this core so far, snapshotted BEFORE the current task is folded
        // in) and call the 3-arg GetRTA_OneTask with it. The 3-arg form Compresses
        // the running RTA ONCE then Convolves against this pre-built prefix —
        // bit-identical to the oracle. (The 2-arg form Compresses+Convolves PER HP
        // task on the running RTA; with >=2 HP tasks and convolved support past
        // Granularity the differing lossy compress count can diverge from the
        // oracle. See Evaluate_NoReuseWideEtTaskWithTwoWideHpTasks_BitIdenticalToOracle.)
        TaskSet hp_tasks;
        FiniteDist hp_tasks_et_conv = IdentityPrefix();
        for (const Task& task_curr : core_tasks) {
            if (verdict[task_curr.id] == RTAReusePerTask::NoReuse) {
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

#pragma once

#include <functional>
#include <optional>
#include <queue>
#include <unordered_set>

#include "sources/Optimization/OptimizeSP_Base.h"

namespace SP_OPT_PA {

// P1.13 — forward-declare the FROZEN RTACache (sources/Safety_Performance_Metric/
// RTA_Cache.h) so OptimizeIncre / OptimizeIncre_SingleTask can take an opt-in
// cache ref WITHOUT pulling RTA_Cache.h into this header (no header cycle). The
// cache API is FROZEN (P1.11 Phase 0): Evaluate/AdoptChampion/Initialize take
// RAW dag + tl (NO sp_parameters) and bake TLs internally. Defined in RTA_Cache.cpp.
class RTACache;

// P1.13 — opt-in cache handle. NO raw pointers (user binding): the optimizer only
// BORROWS the cache for a call. `optional<reference_wrapper<RTACache>>` lets
// `std::nullopt` express "no cache → legacy oracle path" (behavior-preserving),
// which a bare `RTACache&` cannot. Access: `if (rta_cache)
// rta_cache->get().Evaluate(...)` (the `->get()` unwraps the reference_wrapper).
using RTACacheOpt = std::optional<std::reference_wrapper<RTACache>>;

struct PriorityPartialPath {
    PriorityPartialPath() {}
    PriorityPartialPath(const DAG_Model& dag_tasks,
                        const SP_Parameters& sp_parameters);

    void AssignAndUpdateSP(int task_id);

    void UpdateSP(int task_id);
    inline void AssertValidIndex(size_t i) const {
        if (i >= pa_vec_lower_pri.size())
            CoutError("Empty path in GetTaskWeight");
    }
    inline int GetTaskWeight(size_t i) const {
        AssertValidIndex(i);
        return sp_parameters.weights_node.at(pa_vec_lower_pri[i]);
    }
    inline int GetTaskPeriod(size_t i) const {
        AssertValidIndex(i);
        return dag_tasks.tasks[pa_vec_lower_pri[i]].period;
    }
    // inline int GetLastTaskMinUtil(size_t i) const {
    //     AssertValidIndex(i);
    //     return dag_tasks.tasks[pa_vec_lower_pri[i]].utilization();
    // }
    inline int GetTaskMinEt(size_t i) const {
        AssertValidIndex(i);
        return dag_tasks.tasks[pa_vec_lower_pri[i]]
            .execution_time_dist.min_time;
    }

    // const DAG_Model& dag_tasks;
    // const SP_Parameters& sp_parameters;
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
    // double sp=0;
    double sp_lost = 0;
    PriorityVec pa_vec_lower_pri;
    std::unordered_set<int> tasks_to_assign;
};

struct CompPriorityPath {
    // return true if rhs is better than lhs
    bool operator()(const PriorityPartialPath& lhs,
                    const PriorityPartialPath& rhs) const;
};

struct DiffObj {
    int task_id;
    bool increase;
};

// Task IDs with time-limit freedom: a task is TL-flexible iff it carries a
// non-empty `timePerformancePairs` (the perf-pair grid). Mirrors the
// `{-1}`-sentinel test in RecordTimeLimitOptions (OptimizeSP_TL_BF.cpp): tasks
// without pairs get the `{-1}`-only option set (no TL freedom); tasks with
// pairs are the TL-flexible set the serialized Type-L step walks.
std::vector<int> FindTasksWithFlexibleTimeLimits(const DAG_Model& dag_tasks);

// Reports tasks whose pre-TL execution_time_dist moved, MINUS TL-flexible
// tasks — the Type-E (env-changed) diff of the serialized search (P1.10 D2,
// amended 2026-07-17). FiniteDist::operator!= is a 10%-relative approx_equal
// (Probability.cpp:415-417), and a TL-flexible task's execution_time_dist is
// built from raw mu/min/max YAML with no read-time override to the adopted TL,
// so its dist can compare unequal across intervals for TL-induced (not env)
// reasons. Filtering TL-flexible tasks here is more robust than relying on the
// caller to equalize their ET. The UNFILTERED `FindTaskWithDifferentEt` below
// stays as-is for the TL-walk call site (`OptimizeIncre`'s :282), which MUST
// keep flagging the TL-walked (TL-flexible) task so its 1D priority is re-
// searched each TL step.
std::vector<DiffObj> FindEnvTaskWithDifferentEt(
    const DAG_Model& dag_tasks, const DAG_Model& dag_tasks_updated);

std::vector<DiffObj> FindTaskWithDifferentEt(
    const DAG_Model& dag_tasks, const DAG_Model& dag_tasks_updated);

PriorityVec RemoveOneTask(const PriorityVec& pa_vec, int task_id);

enum PriorityChangeStatus { Increase, Decrease, OpenToAll };

PriorityChangeStatus AnalyzePriorityChangeStatus(
    const SP_Parameters& sp_parameters, int task_id, bool et_increased);
// `exclude_opt_pa` (default true): skip the variation that re-inserts task_id
// at its carried position (i == old_priority_index), which reconstructs
// `pa_vec` exactly and re-computes the incumbent's SP — the redundant eval the
// sub-incremental (OptimizeIncre_SingleTask) avoids by scoring the carried PA
// once as its baseline. Callers that want the FULL candidate range (including
// the carried position) pass false (e.g. unit tests asserting the range
// contract).
std::vector<PriorityVec> FindPriorityVec1D_Variations(
    const PriorityVec& pa_vec, int task_id,
    PriorityChangeStatus priority_change, bool exclude_opt_pa = true);

class OptimizePA_Incre : public OptimimizePA_Base {
   public:
    OptimizePA_Incre() {}
    OptimizePA_Incre(const DAG_Model& dag_tasks,
                     const SP_Parameters& sp_parameters)
        : OptimimizePA_Base(dag_tasks, sp_parameters) {}

    // TODO: Current implementation doesn't consider end-to-end latency, need to
    // add later! One way to do it is by modifying the parameters of
    // sp_parameters
    /*
    The implementation for this function follows Audsley's algorithm with
    modifications for speed and optimization considerations:
    // 1. The algortihm iterativelys finds the task to assign the lowest
    priority to. However, since multiple tasks may qualify for the lowest
    priority,
    // the algorithm will consider all of them and save them as partial paths.
    // 2. The input argument K records the maximum number of partial paths under
    consideration in each iteration.
    // 3. This function updates both opt_pa_ and opt_sp_, and returns opt_pa_.
    */
    PriorityVec OptimizeFromScratch(int K);

    // Incremental re-search over ALL tasks whose ET changed since the last
    // dag_tasks_ (FindTaskWithDifferentEt). Seeds opt_sp_ to the carried PA's SP
    // under the new env, then re-searches each changed task. `baseline_sp`
    // (default INT_MIN = "not provided") lets a caller that already holds the
    // carried PA's new-env SP skip the baseline re-score; if provided it MUST
    // equal EvaluateSPWithPriorityVec(dag_tasks_update, sp_parameters_, opt_pa_),
    // else the strict-> adopt test compares against a wrong seed. Advances
    // dag_tasks_ to dag_tasks_update (orchestrator-owned under P0.5 — inert; the
    // challenger is rebuilt each step; kept for bit-identity).
    // P1.13 — `rta_cache` (default std::nullopt = create a local cache): the
    // cache is ALWAYS engaged for this interval. nullopt → OptimizeIncre builds a
    // local RTACache (same scope, outlives the loop) bound via std::ref (no raw
    // pointer); an engaged optional is used as-is (e.g. a shared cache across
    // intervals). The baseline re-score INITIALIZEs opt_pa_ as the cache
    // champion, and every downstream OptimizeIncre_SingleTask call reuses RTA
    // across its 1D priority variations (the engaged optional is forwarded into
    // the loop). Supersedes the earlier "nullopt → legacy oracle path" contract:
    // there is NO oracle arm inside OptimizeIncre anymore — the cache path is the
    // only path, bit-identical to the oracle by Hazard B correctness + the Q5
    // TL-baked-input invariant. No separate opt-in flag (Q6), unlike P1.12 2a's
    // rta_cache_active_ (which gated a SHARED CommitIncumbent reopt collision
    // that does NOT exist here). Champion lifecycle (Q3): adopt at this baseline
    // re-score + each OptimizeIncre_SingleTask adoption. `tl` plumbing (Q5 —
    // RESOLVED): the call-site dag_tasks_update arrives ALREADY TL-baked
    // (UpdateExtDistBasedOnTimeLimit at OptimizeSP_TL_Incre.cpp:146 ==
    // ApplyTimeLimitsToTasksExecutionTime — verified same loop/guard). So the
    // cache is fed the BAKED dag_tasks_update + an all-(-1) tl (cache bakes
    // nothing; sees exactly the final ETs the oracle did → bit-identity). No new
    // tl arg. Behavior-preserving by default.
    PriorityVec OptimizeIncre(const DAG_Model& dag_tasks_update,
                              double baseline_sp = INT_MIN,
                              RTACacheOpt rta_cache = std::nullopt);

    // The sub-incremental primitive: assumes EXACTLY ONE task's ET changed
    // (task_id). Trusts opt_sp_ as the current baseline (caller-set: OptimizeIncre
    // scores the carried PA, or a TL handler seeds it). Generates the 1D priority
    // variations for task_id (one half, per AnalyzePriorityChangeStatus, with
    // exclude_opt_pa=true), scores each, and adopts on strict > — bit-identical
    // to the former :274-292 loop body. Mutates opt_pa_/opt_sp_ in place. Does
    // NOT advance dag_tasks_ (orchestrator-owned).
    //
    // P1.13 — `rta_cache`: each FindPriorityVec1D_Variations candidate is scored
    // via rta_cache->get().Evaluate(baked_dag, priority_assignment, all_-1_tl) (a
    // ≤1-task RTA patch vs the champion whose PA == opt_pa_) +
    // ObtainSP_Full_From_NodeRTAs (Hazard B — multiplies perf_coefficient), INSTEAD
    // of EvaluateSPWithPriorityVec. The champion MUST advance at each strict-
    // improvement adoption (AdoptChampion with the candidate's RTA vector) so the
    // NEXT variation's diff stays |diff|<=1 (Evaluate does NOT advance the
    // champion itself; a stale champion drifts to |diff|>1 →
    // ComputeTaskSetDifference throws, called unguarded by ClassifyReusePerTask).
    // The engaged optional is forwarded here by OptimizeIncre (which owns the
    // cache — caller-supplied or a local it created at nullopt). Direct callers
    // may also pass one; if left nullopt, this primitive runs the legacy oracle
    // path (EvaluateSPWithPriorityVec per candidate) — but OptimizeIncre never
    // reaches that arm since it always binds a cache first. NO raw pointer:
    // borrows the cache via reference_wrapper for the call.
    PriorityVec OptimizeIncre_SingleTask(const DAG_Model& dag_tasks_update,
                                         int task_id, bool et_increased,
                                         RTACacheOpt rta_cache = std::nullopt);

    bool IfInitialized() const { return !opt_pa_.empty(); }
};

inline PriorityVec PerformOptimizePA_Incre(const DAG_Model& dag_tasks,
                                           const SP_Parameters& sp_parameters) {
    OptimizePA_Incre opt(dag_tasks, sp_parameters);
    return opt.OptimizeFromScratch(
        GlobalVariables::Layer_Node_During_Incremental_Optimization);
}
}  // namespace SP_OPT_PA
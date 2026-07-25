#pragma once

#include <functional>
#include <optional>
#include <queue>
#include <unordered_set>

#include "sources/Optimization/OptimizeSP_Base.h"

namespace SP_OPT_PA {

// Forward-declare RTACache so OptimizeIncre / OptimizeIncre_SingleTask can take a
// cache ref without pulling RTA_Cache.h into this header (no header cycle). The
// cache API is frozen: Evaluate/AdoptChampion/Initialize take raw dag + tl (no
// sp_parameters) and bake TLs internally.
class RTACache;

// Opt-in cache handle — the optimizer only BORROWS the cache. nullopt expresses
// "no cache → legacy oracle path" (behavior-preserving), which a bare RTACache&
// cannot. Access: `if (rta_cache) rta_cache->get().Evaluate(...)`.
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

    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
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

// Task IDs with time-limit freedom: a task is TL-flexible iff it has a
// non-empty `timePerformancePairs`. Mirrors the {-1}-sentinel test in
// RecordTimeLimitOptions — tasks without pairs get the {-1}-only option set.
std::vector<int> FindTasksWithFlexibleTimeLimits(const DAG_Model& dag_tasks);

// Tasks whose pre-TL execution_time_dist moved, MINUS TL-flexible tasks — the
// Type-E (env-changed) diff. TL-flexible tasks are filtered because
// FiniteDist::operator!= is a 10%-relative approx_equal and a TL-flexible
// task's dist is built from raw YAML mu/min/max (no TL override), so it can
// compare unequal across intervals for TL-induced (not env) reasons. The
// unfiltered `FindTaskWithDifferentEt` below stays for the TL-walk call site,
// which MUST keep flagging the walked task so its 1D priority is re-searched.
std::vector<DiffObj> FindEnvTaskWithDifferentEt(
    const DAG_Model& dag_tasks, const DAG_Model& dag_tasks_updated);

std::vector<DiffObj> FindTaskWithDifferentEt(
    const DAG_Model& dag_tasks, const DAG_Model& dag_tasks_updated);

// Same diff over two TaskSets directly (the DAG_Model overload just feeds
// dag.tasks here). Callers that already hold baked TaskSets (e.g.
// RTACache::IsSingleTaskChange, which has the champion bake cached) use this to
// skip constructing a throwaway DAG_Model.
std::vector<DiffObj> FindTaskWithDifferentEt(const TaskSet& tasks_base,
                                             const TaskSet& tasks_updated);

PriorityVec RemoveOneTask(const PriorityVec& pa_vec, int task_id);

enum PriorityChangeStatus { Increase, Decrease, OpenToAll };

PriorityChangeStatus AnalyzePriorityChangeStatus(
    const SP_Parameters& sp_parameters, int task_id, bool et_increased);
// `exclude_opt_pa` (default true): skip the variation that re-inserts task_id
// at its carried position, which reconstructs pa_vec exactly — the redundant
// eval the sub-incremental avoids by scoring the carried PA once as its
// baseline. Pass false for the full range (e.g. unit tests).
std::vector<PriorityVec> FindPriorityVec1D_Variations(
    const PriorityVec& pa_vec, int task_id,
    PriorityChangeStatus priority_change, bool exclude_opt_pa = true);

class OptimizePA_Incre : public OptimimizePA_Base {
   public:
    OptimizePA_Incre() {}
    OptimizePA_Incre(const DAG_Model& dag_tasks,
                     const SP_Parameters& sp_parameters)
        : OptimimizePA_Base(dag_tasks, sp_parameters) {}

    // TODO: consider end-to-end latency (e.g. via sp_parameters).
    // Audsley's algorithm with beam search: iteratively assigns the lowest
    // priority, keeping K partial paths. Updates opt_pa_/opt_sp_, returns opt_pa_.
    PriorityVec OptimizeFromScratch(int K);

    // Incremental re-search over every task whose ET changed since the last
    // dag_tasks_. Seeds opt_sp_ to the carried PA's SP under the new env, then
    // re-searches each changed task. Advances dag_tasks_ to dag_tasks_update.
    // `baseline_sp` (default INT_MIN): a caller holding the carried PA's new-env
    // SP may pass it to skip the re-score; if provided it MUST equal
    // EvaluateSPWithPriorityVec(dag_tasks_update, sp_parameters_, opt_pa_).
    // `rta_cache` (default nullopt = create a local cache): the cache is ALWAYS
    // engaged for this interval — nullopt builds a local RTACache (same scope,
    // bound via std::ref); an engaged optional is used as-is. The baseline re-
    // score INITIALIZEs opt_pa_ as champion, and every downstream
    // OptimizeIncre_SingleTask reuses RTA across its 1D variations. There is no
    // oracle arm here — the cache path is the only path, bit-identical to the
    // oracle. dag_tasks_update arrives ALREADY TL-baked, so the cache is fed the
    // baked DAG + an all-(-1) tl (bakes nothing; sees the final ETs the oracle
    // did → bit-identity).
    PriorityVec OptimizeIncre(const DAG_Model& dag_tasks_update,
                              double baseline_sp = INT_MIN,
                              RTACacheOpt rta_cache = std::nullopt);

    // Sub-incremental primitive: assumes EXACTLY ONE task's ET changed
    // (task_id). Trusts opt_sp_ as the baseline (caller-set). Generates the 1D
    // priority variations for task_id (one half, per AnalyzePriorityChangeStatus,
    // exclude_opt_pa=true), scores each, adopts on strict >. Mutates
    // opt_pa_/opt_sp_ in place; does NOT advance dag_tasks_.
    // `rta_cache`: each candidate is scored via
    // rta_cache->get().Evaluate(baked_dag, pa, all_-1_tl) (a ≤1-task RTA patch vs
    // the champion whose PA == opt_pa_) + ObtainSP_Full_From_NodeRTAs, instead of
    // EvaluateSPWithPriorityVec. The champion MUST advance at each strict-
    // improvement adoption (AdoptChampion) so the next variation's diff stays
    // |diff|<=1 — Evaluate never advances the champion itself; a stale champion
    // drifts to |diff|>1 and ComputeTaskSetDifference throws. OptimizeIncre
    // always binds a cache before reaching here, so the legacy oracle arm (nullopt)
    // is only for direct callers/tests.
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
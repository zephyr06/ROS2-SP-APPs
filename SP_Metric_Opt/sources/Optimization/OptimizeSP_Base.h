
#pragma once

#include <functional>
#include <optional>

#include "sources/Safety_Performance_Metric/SP_Metric.h"

namespace SP_OPT_PA {

bool ifTimeout(TimerType start_time);

// P1.14 — shared-budget cooperative timeout for the BF search.
//
// `TIME_LIMIT` is meant to bound ONE `EnumeratePA_with_TimeLimits` call per
// interval (a single shared 10 s budget, per design decision D1=(a)). The
// per-leaf `start_time_` captured in `OptimimizePA_Base` (`:60`) resets at
// every `OptimizePA_BruteForce` construction, so the inner `ifTimeout`
// (`OptimizeSP_BF.cpp:8`) cannot bound the AGGREGATE; and `ifTimeout` is
// otherwise checked only between permutations / at outer-recursion entry —
// never inside `ObtainSP_DAG`. A single `EvaluateSPWithPriorityVec` call
// whose RTA convolutions exceed `TIME_LIMIT` therefore strands the search
// past the cap (the taskset_0 = 182 s/interval signature in the P25 A/B).
//
// `BFDLSharedBudget` is the fix: a scope-guard that installs ONE shared
// `start_time` (captured at `EnumeratePA_with_TimeLimits` entry) for the
// duration of the whole BF search, plus a `cancelled` flag. The inner
// `OptimizePA_BF` reads this shared timer (instead of its per-leaf one) at
// every permutation node, and `ObtainSP_DAG` / `ObtainSP_TaskSet` poll
// `BFSharedBudgetCancelled()` between their per-task / per-chain
// sub-computations so a single runaway SP-eval can be interrupted in place.
// When interrupted mid-eval, `EvaluateSPWithPriorityVec` returns `INT_MIN`
// (a sentinel worse than any real SP) so the discarded permutation simply
// loses to the incumbent — the BF RESULT on in-budget runs is unchanged.
//
// Outside a `BFDLSharedBudget` scope, `BFSharedBudgetCancelled()` is always
// false, so non-BF callers of `ObtainSP_DAG` are unaffected.
class BFDLSharedBudget {
   public:
    // Installs `start` as the active shared BF budget for the lifetime of
    // this object. Re-entrant guard: the previous budget (if any) is saved
    // and restored on destruction so nested BF calls behave correctly.
    explicit BFDLSharedBudget(TimerType start);
    ~BFDLSharedBudget();

    BFDLSharedBudget(const BFDLSharedBudget&) = delete;
    BFDLSharedBudget& operator=(const BFDLSharedBudget&) = delete;

    // True iff the active shared budget has elapsed >= TIME_LIMIT seconds,
    // or no budget is active. Cheap: reads a single file-scope optional + one
    // elapsed-time compare. Safe to call from anywhere (returns false when
    // no BF search is in flight).
    friend bool BFSharedBudgetCancelled();

   private:
    TimerType start_;
    // P1.14 — non-owning observer of the previously-active scope (for
    // re-entrancy). `optional<reference_wrapper<>>` (not a raw pointer),
    // matching the codebase's RTACacheOpt idiom: the guard only BORROWS the
    // prior scope (which is itself a stack-local object), and `std::nullopt`
    // expresses "no prior scope" more directly than a null pointer.
    std::optional<std::reference_wrapper<BFDLSharedBudget>> prev_;
};

// True iff the currently-active BF shared budget (if any) has been exceeded.
// Always false outside a `BFDLSharedBudget` scope. Polled by the BF
// enumeration nodes AND by `ObtainSP_DAG`/`ObtainSP_TaskSet` between
// sub-computations so a single runaway SP-eval can be cooperatively
// cancelled.
bool BFSharedBudgetCancelled();

// task id sequence; small index have higher priority
typedef std::vector<int> PriorityVec;

inline std::string get_tsp_config_file_path() {
    return GlobalVariables::PROJECT_PATH +
           "applications/tsp_solver_osm/config/algorithm_config.yaml";
}

struct ResourceOptResult {
    void UpdatePriorityVec(const PriorityVec& pa) {
        priority_vec = pa;
        for (uint i = 0; i < pa.size(); i++) {
            int id = pa[i];
            id2priority[id] = pa.size() - i;
        }
    }
    // @time_limits: use -1 if one task does not have time limit
    void SaveTimeLimits(const TaskSet& tasks,
                        const std::vector<double>& time_limits) {
        for (int i = 0; i < static_cast<int>(tasks.size()); i++) {
            id2time_limit[tasks[i].id] = time_limits[i];
        }
    }
    std::unordered_map<int, int>
        id2priority;  // large priority values mean high priority
    std::unordered_map<int, double> id2time_limit;
    double sp_opt;
    PriorityVec priority_vec;  // for some old tests only
};

TaskSet UpdateTaskSetPriorities(const TaskSet& tasks,
                                const PriorityVec& priority_assignment);

PriorityVec GetPriorityAssignments(const TaskSet& tasks);

void PrintPriorityVec(const TaskSet& tasks,
                      const PriorityVec& priority_assignment);

void WritePriorityAssignments(std::string path, const TaskSet& tasks,
                              const PriorityVec& pa_vec, double time_taken);

class OptimimizePA_Base {
   public:
    OptimimizePA_Base() {}
    OptimimizePA_Base(const DAG_Model& dag_tasks,
                      const SP_Parameters& sp_parameters)
        : dag_tasks_(dag_tasks),
          sp_parameters_(sp_parameters),
          N(dag_tasks.tasks.size()),
          opt_sp_(INT_MIN),
          start_time_((std::chrono::high_resolution_clock::now())) {}

    void UpdateDAG(const DAG_Model& dag_tasks) { dag_tasks_ = dag_tasks; }
    
    // data members
    DAG_Model dag_tasks_;
    SP_Parameters sp_parameters_;
    int N;
    double opt_sp_;
    PriorityVec opt_pa_;
    TimerType start_time_;
};

TasksSP EvaluateSPWithPriorityVec(const DAG_Model& dag_tasks,
                                  const SP_Parameters& sp_parameters,
                                  const PriorityVec& priority_assignment);

void PrintPA_IfDebugMode(const PriorityVec& pa, double sp_eval);

YAML::Node PriorityAssignmentToYaml(const TaskSet& tasks,
                                    const PriorityVec& priority_assignment);

void WriteTimeLimitToYamlOSM(double time_limit_ms);

std::unordered_map<std::string, int> Task2priority_value(
    const TaskSet& tasks, const PriorityVec& priority_assignment);
}  // namespace SP_OPT_PA

#pragma once

#include "sources/Safety_Performance_Metric/SP_Metric.h"

namespace SP_OPT_PA {

bool ifTimeout(TimerType start_time);

// task id sequence; small index have higher priority
typedef std::vector<int> PriorityVec;

struct ResOptResult {
    void UpdatePriorityVec(const PriorityVec& pa) {
        for (uint i = 0; i < pa.size(); i++) {
            int id = pa[i];
            id2priority[id] = pa.size() - i;
        }
    }
    std::unordered_map<int, int>
        id2priority;  // large priority values mean high priority
    std::unordered_map<int, double> id2time_limit;
    double sp_opt;
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

    // data members
    DAG_Model dag_tasks_;
    SP_Parameters sp_parameters_;
    int N;
    double opt_sp_;
    PriorityVec opt_pa_;
    TimerType start_time_;
};

double EvaluateSPWithPriorityVec(const DAG_Model& dag_tasks,
                                 const SP_Parameters& sp_parameters,
                                 const PriorityVec& priority_assignment);

void PrintPA_IfDebugMode(const PriorityVec& pa, double sp_eval);

YAML::Node PriorityAssignmentToYaml(const TaskSet& tasks,
                                    const PriorityVec& priority_assignment);
}  // namespace SP_OPT_PA
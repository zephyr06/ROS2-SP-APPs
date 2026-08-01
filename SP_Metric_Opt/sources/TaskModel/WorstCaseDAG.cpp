#include "sources/TaskModel/DAG_Model.h"

#include <stdexcept>

namespace SP_OPT_PA {

// P0.6 §8: structural equality of two tasks (id/period/deadline/processorId/
// name + the timePerformancePairs grid). The ET dist is excluded — the builder
// fuses the max across intervals, so differing dists are the WHOLE POINT.
bool TaskStructureMatches(const Task &a, const Task &b) {
    if (a.id != b.id || a.period != b.period || a.deadline != b.deadline ||
        a.processorId != b.processorId || a.name != b.name) {
        return false;
    }
    if (a.timePerformancePairs.size() != b.timePerformancePairs.size()) {
        return false;
    }
    for (size_t k = 0; k < a.timePerformancePairs.size(); k++) {
        if (a.timePerformancePairs[k].time_limit != b.timePerformancePairs[k].time_limit ||
            a.timePerformancePairs[k].performance != b.timePerformancePairs[k].performance) {
            return false;
        }
    }
    return true;
}

// P0.6 §8: worst-case DAG across interval DAGs. Per task, the dist becomes a
// point mass at max(execution_time_max) across all intervals (stochastic
// dominance → the gate upper-bounds every interval). Structure is copied from
// interval 0; a structural mismatch across intervals is a loud failure.
DAG_Model BuildWorstCaseDagAcrossIntervals(const std::vector<DAG_Model> &interval_dags) {
    if (interval_dags.empty()) {
        throw std::runtime_error(
            "BuildWorstCaseDagAcrossIntervals: no interval DAGs provided");
    }

    // Interval 0 is the structural template (default copy ctor deep-copies
    // tasks / chains_ / chains_deadlines_).
    DAG_Model worst = interval_dags[0];
    const TaskSet &template_tasks = interval_dags[0].tasks;

    for (size_t j = 1; j < interval_dags.size(); j++) {
        const TaskSet &other = interval_dags[j].tasks;
        if (other.size() != template_tasks.size()) {
            throw std::runtime_error(
                "BuildWorstCaseDagAcrossIntervals: task count mismatch across "
                "interval DAGs (interval 0 has " +
                std::to_string(template_tasks.size()) + ", interval " +
                std::to_string(j) + " has " + std::to_string(other.size()) + ")");
        }
        for (size_t i = 0; i < template_tasks.size(); i++) {
            if (!TaskStructureMatches(template_tasks[i], other[i])) {
                throw std::runtime_error(
                    "BuildWorstCaseDagAcrossIntervals: structural mismatch on "
                    "task " +
                    std::to_string(i) + " between interval 0 and interval " +
                    std::to_string(j));
            }
        }
    }

    // Overwrite each task's dist with a point mass at the worst-case max_time.
    for (size_t i = 0; i < worst.tasks.size(); i++) {
        double max_wcet = interval_dags[0].tasks[i].execution_time_dist.max_time;
        for (size_t j = 1; j < interval_dags.size(); j++) {
            double candidate = interval_dags[j].tasks[i].execution_time_dist.max_time;
            if (candidate > max_wcet) max_wcet = candidate;
        }
        worst.tasks[i].execution_time_dist = GetUnitExecutionTimeDist(max_wcet);
    }
    return worst;
}

}  // namespace SP_OPT_PA

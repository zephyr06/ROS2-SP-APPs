#pragma once
// TODO: generalize to working with execution time distribution

#include "sources/TaskModel/RegularTasks.h"
namespace SP_OPT_PA {
FiniteDist GetRTA_OneTask(const Task& task_curr, const TaskSet& hp_tasks);
FiniteDist GetRTA_OneTask(const Task& task_curr, const TaskSet& hp_tasks, const FiniteDist& hp_tasks_et_conv);
std::vector<FiniteDist> ProbabilisticRTA_TaskSet_SingleCore(
    const TaskSet& tasks);

std::vector<FiniteDist> ProbabilisticRTA_TaskSet(const TaskSet& tasks);

double GetDDL_MissProbability(const FiniteDist& finite_dist, double ddl);
}  // namespace SP_OPT_PA
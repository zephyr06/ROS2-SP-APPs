#pragma once
// TODO: generalize to working with execution time distribution

#include <unordered_map>

#include "sources/TaskModel/RegularTasks.h"
namespace SP_OPT_PA {
FiniteDist GetRTA_OneTask(const Task& task_curr, const TaskSet& hp_tasks);
FiniteDist GetRTA_OneTask(const Task& task_curr, const TaskSet& hp_tasks, const FiniteDist& hp_tasks_et_conv);
std::vector<FiniteDist> ProbabilisticRTA_TaskSet_SingleCore(
    const TaskSet& tasks);
// Same RTA as the 1-arg overload, additionally emitting `hp_tasks_et_conv_vec`
// where hp_tasks_et_conv_vec[i] = convolution of the higher-priority tasks' ET
// dists [0, i) snapshotted at the top of iteration i (hp_tasks_et_conv_vec[0]
// == FiniteDist({Value_Proba(0, 1.0)})). This is exactly the value the 3-arg
// GetRTA_OneTask consumes. Pure refactor — the returned rtas are bit-identical
// to the 1-arg version.
std::vector<FiniteDist> ProbabilisticRTA_TaskSet_SingleCore(
    const TaskSet& tasks, std::vector<FiniteDist>& hp_tasks_et_conv_vec);

std::vector<FiniteDist> ProbabilisticRTA_TaskSet(const TaskSet& tasks);

// Partition `tasks` by `processorId` (one TaskSet per core). Declared here so
// the per-core RTA cache (RTA_Cache.h) can derive a candidate's per-core
// partition to diff against a cached `sorted_task_ids`.
// TODO: remove ProcessorTaskSet struct and methods (legacy).
std::unordered_map<int, TaskSet> ExtractTaskSetPerProcessor(const TaskSet& tasks);

double GetDDL_MissProbability(const FiniteDist& finite_dist, double ddl);
}  // namespace SP_OPT_PA
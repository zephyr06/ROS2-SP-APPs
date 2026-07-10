#include "sources/RTDA/ImplicitCommunication/SimulationOrchestrator.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>

#include "sources/RTDA/ImplicitCommunication/RunQueue.h"
#include "sources/Safety_Performance_Metric/SP_Metric.h"

namespace SP_OPT_PA {

namespace {
// True for the INCR_P<n> period-override modes (e.g. INCR_P1, INCR_P10,
// INCR_P60). These are INCR dispatches whose ReoptimizationPeriod is
// overridden at binary startup: RunOrchestrator.cpp parses the <n> suffix
// from the mode string and assigns GlobalVariables::ReoptimizationPeriod
// before the orchestrator is constructed. The orchestrator treats any
// INCR_P* exactly as INCR (same incr_optimizer_, same
// Optimize_w_TL_ScratchOrIncre dispatch); only the period differs. Keeping
// the INCR_P<n> string end-to-end (rather than normalizing to "INCR") means
// the output subdir is named INCR_P<n> consistently across ExportResults and
// RunOrchestrator's exec-time write, so the Python analysis finds both
// artifacts under taskset_dir/INCR_P<n>/INCR_P<n>/.
bool IsINCRPeriodVariant(const std::string& mode) {
    const std::string prefix = "INCR_P";
    if (mode.rfind(prefix, 0) != 0) return false;
    if (mode.size() == prefix.size()) return false;  // bare "INCR_P" — no digits
    for (size_t i = prefix.size(); i < mode.size(); i++) {
        if (mode[i] < '0' || mode[i] > '9') return false;
    }
    return true;
}
}  // namespace
BaseSimulationOrchestrator::BaseSimulationOrchestrator(
    const std::string& input_folder, const std::string& output_folder,
    LLint interval_duration_ms)
    : input_folder_(input_folder),
      output_folder_(output_folder),
      interval_duration_ms_(interval_duration_ms) {}

void BaseSimulationOrchestrator::LoadIntervalConfigs() {
    int interval_idx = 0;
    while (true) {
        // Require merged taskset files that contain ALL tasks (both p0 & p1)
        std::string file_path = input_folder_ + "/taskset_characteristics_interval_" +
                                std::to_string(interval_idx) + ".yaml";
        if (!std::filesystem::exists(file_path)) {
            if (interval_idx == 0) {
                std::cerr << "Error: No merged taskset files found in "
                          << input_folder_ << std::endl;
                std::cerr
                    << "Expected file example: taskset_characteristics_interval_0.yaml"
                    << std::endl;
                std::cerr
                    << "Merged files are required to ensure all processors' "
                       "tasks are loaded. Exiting."
                    << std::endl;
                std::exit(EXIT_FAILURE);
            }
            break;
        }

        DAG_Model dag_tasks = ReadDAG_Tasks(file_path);
        SP_Parameters sp_parameters = ReadSP_Parameters(file_path);
        TaskSetInfoDerived tasks_info(dag_tasks.tasks);

        dag_tasks_vecs_.push_back(dag_tasks);
        tasks_info_vecs_.push_back(tasks_info);
        sp_parameters_vecs_.push_back(sp_parameters);
        interval_idx++;
    }
}

std::vector<float> BaseSimulationOrchestrator::LoadJobExecutionTraces(
    int task_id, int path_idx, int inst_idx) {
    std::string file_path =
        input_folder_ + "/path_Et_task_" + std::to_string(task_id) + "_" +
        std::to_string(path_idx) + "_" + std::to_string(inst_idx) + ".txt";
    if (!std::filesystem::exists(file_path)) {
        return {};
    }

    std::ifstream infile(file_path);
    std::string line;
    std::vector<float> ets;
    while (std::getline(infile, line)) {
        std::stringstream ss(line);
        int x, y;
        float et;
        char comma;
        if (ss >> x >> comma >> y >> comma >> et) {
            ets.push_back(et);
        }
    }
    return ets;
}

void BaseSimulationOrchestrator::ExportResults(
    const std::string& scheduler_name) {
    // Determine effective export level & sampling
    int detail = GlobalVariables::EXPORT_DETAIL_LEVEL;
    int sample_sec = GlobalVariables::METRIC_SAMPLE_INTERVAL_SECONDS;
    if (detail < 0 || detail > 3) {
        detail = 3;  // clamp to valid range
    }

    std::string results_dir = output_folder_ + "/" + scheduler_name;
    std::filesystem::create_directories(results_dir);

    // Always write interval_sp_metrics.txt (Level-0 minimum), with sampling.
    // Format kept unchanged for backward compatibility: interval_index,sp_metric
    std::string metrics_path = results_dir + "/interval_sp_metrics.txt";
    std::ofstream metrics_file(metrics_path);
    for (size_t i = 0; i < interval_sp_metrics_.size(); i++) {
        LLint time_sec = i * interval_duration_ms_ / 1000;
        if (sample_sec > 0 && time_sec % sample_sec != 0) {
            continue;  // skip unsampled intervals
        }
        metrics_file << i << "," << interval_sp_metrics_[i] << "\n";
    }

    // Compute overall miss rate from job_history_
    LLint total_jobs = 0;
    LLint missed_jobs = 0;
    for (const auto& r : job_history_) {
        if (r.isOverrun) {
            missed_jobs++;
        }
        total_jobs++;
    }
    double overall_miss_rate =
        (total_jobs > 0)
            ? (static_cast<double>(missed_jobs) / static_cast<double>(total_jobs))
            : 0.0;

    // Always write miss_rate_summary.txt (Level-0 minimum)
    std::string miss_summary_path = results_dir + "/miss_rate_summary.txt";
    std::ofstream miss_summary_file(miss_summary_path);
    miss_summary_file << "total_jobs,missed_jobs,miss_rate\n";
    miss_summary_file << total_jobs << "," << missed_jobs << ","
                      << overall_miss_rate << "\n";

    // Group history by task id for everything that follows
    std::unordered_map<int, std::vector<JobRecord>> task_history;
    for (const auto& record : job_history_) {
        task_history[record.taskId].push_back(record);
    }

    // Level 1+: per-task miss rate summary
    if (detail >= 1) {
        std::string per_task_miss_path =
            results_dir + "/miss_rate_per_task.txt";
        std::ofstream per_task_miss_file(per_task_miss_path);
        per_task_miss_file << "task_id,total_jobs,missed_jobs,miss_rate,"
                              "avg_response_time,max_response_time\n";
        for (const auto& [task_id, records] : task_history) {
            LLint t_total = records.size();
            LLint t_missed = 0;
            double sum_rt = 0.0;
            double max_rt = 0.0;
            for (const auto& r : records) {
                double rt = static_cast<double>(r.finishTime - r.releaseTime);
                sum_rt += rt;
                if (rt > max_rt) max_rt = rt;
                if (r.isOverrun) t_missed++;
            }
            double avg_rt = (t_total > 0) ? (sum_rt / t_total) : 0.0;
            double mr = (t_total > 0)
                            ? (static_cast<double>(t_missed) / t_total)
                            : 0.0;
            per_task_miss_file << task_id << "," << t_total << ","
                               << t_missed << "," << mr << "," << avg_rt
                               << "," << max_rt << "\n";
        }
    }

    // Level 2+: per-task aggregate per interval (only sampled intervals)
    if (detail >= 2) {
        for (const auto& [task_id, records] : task_history) {
            std::string file_path = results_dir + "/task_aggregate_" +
                                    std::to_string(task_id) + ".txt";
            std::ofstream agg_file(file_path);
            agg_file << "interval_index,time_seconds,job_count,avg_response_"
                        "time,max_response_time,missed_jobs\n";

            for (size_t i = 0; i < interval_sp_metrics_.size(); i++) {
                LLint time_sec = i * interval_duration_ms_ / 1000;
                if (sample_sec > 0 && time_sec % sample_sec != 0) {
                    continue;
                }
                LLint int_start = i * interval_duration_ms_;
                LLint int_end = (i + 1) * interval_duration_ms_;
                int count = 0;
                double sum_rt = 0.0;
                double max_rt = 0.0;
                int int_missed = 0;
                for (const auto& r : records) {
                    if (r.releaseTime >= int_start && r.releaseTime < int_end) {
                        double rt = static_cast<double>(r.finishTime - r.releaseTime);
                        sum_rt += rt;
                        if (rt > max_rt) max_rt = rt;
                        if (r.isOverrun) int_missed++;
                        count++;
                    }
                }
                double avg_rt = (count > 0) ? (sum_rt / count) : 0.0;
                agg_file << i << "," << time_sec << "," << count << ","
                         << avg_rt << "," << max_rt << "," << int_missed
                         << "\n";
            }
        }
    }

    // Level 3: full per-job trace files (original behavior)
    if (detail >= 3) {
        for (const auto& [task_id, records] : task_history) {
            std::string file_path = results_dir + "/response_times_task_" +
                                    std::to_string(task_id) + ".txt";
            std::ofstream outfile(file_path);
            outfile << "jobId,release_time,start_time,finish_time,response_time,"
                       "execution_time,is_overrun\n";
            for (const auto& r : records) {
                outfile << r.jobId << "," << r.releaseTime << ","
                        << r.startTime << "," << r.finishTime << ","
                        << (r.finishTime - r.releaseTime) << ","
                        << r.executionTime << ","
                        << (r.isOverrun ? 1 : 0) << "\n";
            }
        }
    }
}

void BaseSimulationOrchestrator::PrintHyperperiodSchedule(
    LLint start_time, LLint end_time) const {
    std::cout << "--- Schedule within hyperperiod [" << start_time << ", "
              << end_time << "] ---\n";
    std::cout << "taskId,jobId,releaseTime,startTime,finishTime,executionTime,"
                 "responseTime,isOverrun\n";
    for (const auto& r : job_history_) {
        if (r.releaseTime >= start_time && r.releaseTime < end_time) {
            std::cout << r.taskId << "," << r.jobId << "," << r.releaseTime
                      << "," << r.startTime << "," << r.finishTime << ","
                      << r.executionTime << ","
                      << (r.finishTime - r.releaseTime) << ","
                      << (r.isOverrun ? 1 : 0) << "\n";
        }
    }
    std::cout << "--------------------------------------------\n";
}

FixedTaskPrioritySchedulingOrchestrator::
    FixedTaskPrioritySchedulingOrchestrator(const std::string& input_folder,
                                            const std::string& output_folder,
                                            const std::string& scheduler_mode,
                                            LLint interval_duration_ms)
    : BaseSimulationOrchestrator(input_folder, output_folder,
                                 interval_duration_ms),
      scheduler_mode_(scheduler_mode) {}

void FixedTaskPrioritySchedulingOrchestrator::RunSimulation() {
    LoadIntervalConfigs();
    if (dag_tasks_vecs_.empty()) {
        return;
    }

    if (scheduler_mode_ == "INCR" || IsINCRPeriodVariant(scheduler_mode_) ||
        scheduler_mode_ == "INCR_NO_TL" || scheduler_mode_ == "INCR_WCET" ||
        scheduler_mode_ == "INCR_SCRATCH") {
        incr_optimizer_ = OptimizePA_Incre_with_TimeLimits(
            dag_tasks_vecs_[0], sp_parameters_vecs_[0]);
    }

    for (size_t i = 0; i < dag_tasks_vecs_.size(); i++) {
        LLint start_time = i * interval_duration_ms_;
        LLint end_time = (i + 1) * interval_duration_ms_;
        SimulateInterval(i, start_time, end_time);
    }

    ExportResults(scheduler_mode_);
}

ResourceOptResult
FixedTaskPrioritySchedulingOrchestrator::DeterminePrioritiesAndBudgets(
    DAG_Model& dag_tasks, const SP_Parameters& sp_parameters) {
    ResourceOptResult res;
    if (scheduler_mode_ == "INCR" || IsINCRPeriodVariant(scheduler_mode_)) {
        incr_optimizer_.Optimize_w_TL_ScratchOrIncre(
            dag_tasks,
            GlobalVariables::Layer_Node_During_Incremental_Optimization);
        res = incr_optimizer_.CollectResults();
    } else if (scheduler_mode_ == "BF") {
        res = EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters);
    } else if (scheduler_mode_ == "INCR_SCRATCH") {
        // ABLATION: amnesiac wide-radius reopt. `scratch_opt` is constructed
        // fresh each interval and discarded, so has_incumbent_ is always false
        // → ResetIncumbentBaseline takes the interval-0 branch (RM + min-TL)
        // every call. The compare-and-keep guard therefore measures the wide
        // search against a synthetic RM baseline, NOT against the previous
        // interval's adopted solution. This isolates the value of carrying the
        // incumbent forward.
        // DO NOT confuse with INCR + ReoptimizationPeriod=1: that reuses the
        // persistent incr_optimizer_, so prev_optimizer_ carries the prior
        // interval's incumbent and compare-and-keep is measured against the
        // running best. INCR(period=1) weakly dominates INCR_SCRATCH in SP
        // (never worse, sometimes strictly better); INCR_SCRATCH is kept only
        // as the no-memory control.
        OptimizePA_Incre_with_TimeLimits scratch_opt(dag_tasks, sp_parameters);
        scratch_opt.ReOptimizePeriodic(
            GlobalVariables::Layer_Node_During_Incremental_Optimization);
        res = scratch_opt.CollectResults();
    } else if (scheduler_mode_ == "INCR_NO_TL") {
        bool prev = GlobalVariables::disable_time_limit_opt;
        GlobalVariables::disable_time_limit_opt = true;
        incr_optimizer_.Optimize_w_TL_ScratchOrIncre(
            dag_tasks,
            GlobalVariables::Layer_Node_During_Incremental_Optimization);
        res = incr_optimizer_.CollectResults();
        GlobalVariables::disable_time_limit_opt = prev;
    } else if (scheduler_mode_ == "INCR_WCET") {
        bool prev = GlobalVariables::use_wcet_execution_time;
        GlobalVariables::use_wcet_execution_time = true;
        incr_optimizer_.Optimize_w_TL_ScratchOrIncre(
            dag_tasks,
            GlobalVariables::Layer_Node_During_Incremental_Optimization);
        res = incr_optimizer_.CollectResults();
        GlobalVariables::use_wcet_execution_time = prev;
    } else if (scheduler_mode_ == "RM") {
        std::vector<int> sorted_indices(dag_tasks.tasks.size());
        std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
        std::sort(
            sorted_indices.begin(), sorted_indices.end(), [&](int a, int b) {
                return dag_tasks.tasks[a].period < dag_tasks.tasks[b].period;
            });

        for (size_t i = 0; i < sorted_indices.size(); i++) {
            res.priority_vec.push_back(sorted_indices[i]);
            res.id2time_limit[sorted_indices[i]] = -1.0;
        }
    } else if (scheduler_mode_ == "RM_FAST") {
        std::vector<int> sorted_indices(dag_tasks.tasks.size());
        std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
        std::sort(
            sorted_indices.begin(), sorted_indices.end(), [&](int a, int b) {
                return dag_tasks.tasks[a].period < dag_tasks.tasks[b].period;
            });

        for (size_t i = 0; i < sorted_indices.size(); i++) {
            res.priority_vec.push_back(sorted_indices[i]);
            if (dag_tasks.tasks[sorted_indices[i]]
                    .timePerformancePairs.empty()) {
                res.id2time_limit[sorted_indices[i]] = -1.0;
            } else {
                res.id2time_limit[sorted_indices[i]] =
                    dag_tasks.tasks[sorted_indices[i]]
                        .timePerformancePairs[0]
                        .time_limit;
            }
        }
    } else if (scheduler_mode_ == "RM_SLOW") {
        std::vector<int> sorted_indices(dag_tasks.tasks.size());
        std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
        std::sort(
            sorted_indices.begin(), sorted_indices.end(), [&](int a, int b) {
                return dag_tasks.tasks[a].period < dag_tasks.tasks[b].period;
            });

        for (size_t i = 0; i < sorted_indices.size(); i++) {
            res.priority_vec.push_back(sorted_indices[i]);
            if (dag_tasks.tasks[sorted_indices[i]]
                    .timePerformancePairs.empty()) {
                res.id2time_limit[sorted_indices[i]] = -1.0;
            } else {
                res.id2time_limit[sorted_indices[i]] =
                    dag_tasks.tasks[sorted_indices[i]]
                        .timePerformancePairs.back()
                        .time_limit;
            }
        }
    }
    return res;
}

void FixedTaskPrioritySchedulingOrchestrator::ApplyTaskConfigurations(
    DAG_Model& dag_tasks, const ResourceOptResult& res) {
    for (size_t i = 0; i < res.priority_vec.size(); i++) {
        int task_idx = res.priority_vec[i];
        dag_tasks.tasks[task_idx].priority = i;
    }

    for (auto& task : dag_tasks.tasks) {
        task.setExecutionTime(task.execution_time_dist.GetAvgValue());
    }
}

void FixedTaskPrioritySchedulingOrchestrator::RecordFinishedJobs(
    LLint time_now, RunQueue& run_queue, const ResourceOptResult& res,
    const DAG_Model& dag_tasks) {
    for (const auto& job : run_queue.schedule_) {
        if (job.second.finish == time_now) {
            JobRecord record;
            record.taskId = job.first.taskId;
            record.jobId = job.first.jobId;
            record.releaseTime =
                job.first.jobId * dag_tasks.tasks[record.taskId].period;
            record.startTime = job.second.start;
            record.finishTime = job.second.finish;
            record.executionTime = job.second.executionTime;

            double time_limit = -1.0;
            auto it_limit = res.id2time_limit.find(record.taskId);
            if (it_limit != res.id2time_limit.end()) {
                time_limit = it_limit->second;
            }
            record.isOverrun =
                (record.executionTime >= time_limit && time_limit > 0);

            auto it = std::find_if(job_history_.begin(), job_history_.end(),
                                   [&](const JobRecord& r) {
                                       return r.taskId == record.taskId &&
                                              r.jobId == record.jobId;
                                   });
            if (it == job_history_.end()) {
                job_history_.push_back(record);
            }
        }
    }
}

void FixedTaskPrioritySchedulingOrchestrator::ReleaseJobs(
    LLint time_now, LLint end_time, const DAG_Model& dag_tasks,
    const ResourceOptResult& res, RunQueue& run_queue,
    std::unordered_map<int, std::vector<float>>& traces,
    std::unordered_map<int, size_t>& trace_indices) {
    if (time_now >= end_time)
        return;

    for (size_t i = 0; i < dag_tasks.tasks.size(); i++) {
        const auto& task = dag_tasks.tasks[i];
        if (time_now % task.period == 0) {
            JobCEC job_curr(i, time_now / task.period);

            double execution_time;
            if (!traces[i].empty()) {
                size_t idx = trace_indices[i];
                execution_time = traces[i][idx];
                trace_indices[i] = (idx + 1) % traces[i].size();
            } else {
                execution_time = task.execution_time_dist.GetAvgValue();
            }

            double budget = -1.0;
            auto it_limit = res.id2time_limit.find(i);
            if (it_limit != res.id2time_limit.end()) {
                budget = it_limit->second;
            }

            if (budget > 0 && execution_time > budget) {
                execution_time = budget;
            }

            JobScheduleInfo job_info(job_curr, time_now + task.deadline,
                                     std::max(1, (int)(execution_time + 0.5)));

            if (run_queue.job_queue_.empty()) {
                run_queue.job_queue_.push_back(job_info);
            } else {
                double priority_curr = task.priority;
                auto itr = std::upper_bound(
                    run_queue.job_queue_.begin(), run_queue.job_queue_.end(),
                    priority_curr,
                    [&](double priority_curr, const JobScheduleInfo& element) {
                        return priority_curr <
                               dag_tasks.tasks[element.job.taskId].priority;
                    });
                run_queue.job_queue_.insert(itr, job_info);
            }
        }
    }
}

void FixedTaskPrioritySchedulingOrchestrator::SimulateInterval(int interval_idx,
                                                               LLint start_time,
                                                               LLint end_time) {
    DAG_Model& dag_tasks = dag_tasks_vecs_[interval_idx];
    const SP_Parameters& sp_parameters = sp_parameters_vecs_[interval_idx];

    ResourceOptResult res =
        DeterminePrioritiesAndBudgets(dag_tasks, sp_parameters);
    ApplyTaskConfigurations(dag_tasks, res);

    RunQueue run_queue(dag_tasks.tasks);
    std::unordered_map<int, std::vector<float>> traces;
    std::unordered_map<int, size_t> trace_indices;

    for (size_t i = 0; i < dag_tasks.tasks.size(); i++) {
        traces[i] = LoadJobExecutionTraces(i, 0, 0);
        trace_indices[i] = 0;
    }

    for (LLint time_now = start_time; time_now <= end_time; time_now++) {
        run_queue.RemoveFinishedJob(time_now);
        RecordFinishedJobs(time_now, run_queue, res, dag_tasks);
        ReleaseJobs(time_now, end_time, dag_tasks, res, run_queue, traces,
                    trace_indices);
        run_queue.RunJobHigestPriority(time_now);
    }

    std::vector<double> time_limits(dag_tasks.tasks.size(), -1);
    for (size_t i = 0; i < dag_tasks.tasks.size(); i++) {
        auto it = res.id2time_limit.find(dag_tasks.tasks[i].id);
        if (it != res.id2time_limit.end()) {
            time_limits[i] = it->second;
        }
    }
    interval_sp_metrics_.push_back(ObtainSP_TaskSet_And_TimeLimits(
        dag_tasks.tasks, sp_parameters, time_limits));
}

CFSSimulationOrchestrator::CFSSimulationOrchestrator(
    const std::string& input_folder, const std::string& output_folder,
    LLint interval_duration_ms)
    : BaseSimulationOrchestrator(input_folder, output_folder,
                                 interval_duration_ms) {}

void CFSSimulationOrchestrator::RunSimulation() {
    LoadIntervalConfigs();
    if (dag_tasks_vecs_.empty()) {
        return;
    }

    for (size_t i = 0; i < dag_tasks_vecs_.size(); i++) {
        LLint start_time = i * interval_duration_ms_;
        LLint end_time = (i + 1) * interval_duration_ms_;
        SimulateInterval(i, start_time, end_time);
    }

    ExportResults("CFS");
}

struct CFS_TaskCompareOrch {
    bool operator()(const std::pair<double, int>& a,
                    const std::pair<double, int>& b) const {
        if (a.first != b.first) {
            return a.first < b.first;
        }
        return a.second > b.second;
    }
};

void CFSSimulationOrchestrator::UpdateCFSVirtualTimes(
    int running_task_id, std::unordered_map<int, double>& accumulated_et,
    std::set<std::pair<double, int>>& run_queue_set) {
    if (running_task_id >= 0) {
        double old_et = accumulated_et[running_task_id];
        auto it = run_queue_set.find({old_et, running_task_id});
        if (it != run_queue_set.end()) {
            run_queue_set.erase(it);
        }
        accumulated_et[running_task_id] += 1.0;
        run_queue_set.insert(
            {accumulated_et[running_task_id], running_task_id});
    }
}

void CFSSimulationOrchestrator::RecordFinishedJobsCFS(
    LLint time_now, RunQueue& run_queue, const DAG_Model& dag_tasks) {
    for (const auto& job : run_queue.schedule_) {
        if (job.second.finish == time_now) {
            JobRecord record;
            record.taskId = job.first.taskId;
            record.jobId = job.first.jobId;
            record.releaseTime =
                job.first.jobId * dag_tasks.tasks[record.taskId].period;
            record.startTime = job.second.start;
            record.finishTime = job.second.finish;
            record.executionTime = job.second.executionTime;
            record.isOverrun = false;

            auto it = std::find_if(job_history_.begin(), job_history_.end(),
                                   [&](const JobRecord& r) {
                                       return r.taskId == record.taskId &&
                                              r.jobId == record.jobId;
                                   });
            if (it == job_history_.end()) {
                job_history_.push_back(record);
            }
        }
    }
}

void CFSSimulationOrchestrator::UpdateActiveCounts(
    RunQueue& run_queue, const DAG_Model& dag_tasks,
    std::unordered_map<int, int>& active_jobs_count,
    std::set<std::pair<double, int>>& run_queue_set,
    const std::unordered_map<int, double>& accumulated_et) {
    std::unordered_map<int, int> new_counts;
    for (const auto& job_info : run_queue.job_queue_) {
        new_counts[job_info.job.taskId]++;
    }
    for (const auto& task : dag_tasks.tasks) {
        int old_c = active_jobs_count[task.id];
        int new_c = new_counts[task.id];
        if (old_c > 0 && new_c == 0) {
            run_queue_set.erase({accumulated_et.at(task.id), task.id});
        }
        active_jobs_count[task.id] = new_c;
    }
}

void CFSSimulationOrchestrator::ReleaseJobsCFS(
    LLint time_now, LLint end_time, const DAG_Model& dag_tasks,
    RunQueue& run_queue, std::unordered_map<int, std::vector<float>>& traces,
    std::unordered_map<int, size_t>& trace_indices) {
    if (time_now >= end_time)
        return;

    for (size_t i = 0; i < dag_tasks.tasks.size(); i++) {
        const auto& task = dag_tasks.tasks[i];
        if (time_now % task.period == 0) {
            JobCEC job_curr(i, time_now / task.period);

            double execution_time;
            if (!traces[i].empty()) {
                size_t idx = trace_indices[i];
                execution_time = traces[i][idx];
                trace_indices[i] = (idx + 1) % traces[i].size();
            } else {
                execution_time = task.execution_time_dist.GetAvgValue();
            }

            JobScheduleInfo job_info(job_curr, time_now + task.deadline,
                                     std::max(1, (int)(execution_time + 0.5)));
            run_queue.job_queue_.push_back(job_info);
        }
    }
}

void CFSSimulationOrchestrator::ScheduleCFS(
    LLint time_now, RunQueue& run_queue,
    std::set<std::pair<double, int>>& run_queue_set, int& running_task_id) {
    running_task_id = -1;
    if (run_queue_set.empty())
        return;

    int min_taskId = run_queue_set.begin()->second;
    // Find the first (oldest) queued job for this task
    int min_job_index = -1;
    for (size_t i = 0; i < run_queue.job_queue_.size(); ++i) {
        if (run_queue.job_queue_[i].job.taskId == min_taskId) {
            min_job_index = static_cast<int>(i);
            break;
        }
    }
    if (min_job_index == -1)
        return;

    if (!run_queue.job_queue_[min_job_index].running) {
        run_queue.PreemptRunningJob(time_now);
        if (min_job_index > 0) {
            JobScheduleInfo selected_job = run_queue.job_queue_[min_job_index];
            run_queue.job_queue_.erase(run_queue.job_queue_.begin() +
                                       min_job_index);
            run_queue.job_queue_.insert(run_queue.job_queue_.begin(),
                                        selected_job);
        }
        if (!run_queue.RunJob(0, time_now)) {
            CoutError("Error in RunQueue SimulatedCFS_SingleCore RunJob!");
        }
    }
    running_task_id = min_taskId;
}

void CFSSimulationOrchestrator::SimulateInterval(int interval_idx,
                                                 LLint start_time,
                                                 LLint end_time) {
    DAG_Model& dag_tasks = dag_tasks_vecs_[interval_idx];
    const SP_Parameters& sp_parameters = sp_parameters_vecs_[interval_idx];

    for (auto& task : dag_tasks.tasks) {
        task.setExecutionTime(task.execution_time_dist.GetAvgValue());
    }

    RunQueue run_queue(dag_tasks.tasks);
    std::unordered_map<int, double> accumulated_et;
    std::unordered_map<int, int> active_jobs_count;
    std::set<std::pair<double, int>> run_queue_set;

    for (const auto& task : dag_tasks.tasks) {
        accumulated_et[task.id] = 0.0;
        active_jobs_count[task.id] = 0;
    }

    std::unordered_map<int, std::vector<float>> traces;
    std::unordered_map<int, size_t> trace_indices;
    for (size_t i = 0; i < dag_tasks.tasks.size(); i++) {
        traces[i] = LoadJobExecutionTraces(i, 0, 0);
        trace_indices[i] = 0;
    }

    int running_task_id = -1;

    for (LLint time_now = start_time; time_now <= end_time; time_now++) {
        UpdateCFSVirtualTimes(running_task_id, accumulated_et, run_queue_set);
        run_queue.RemoveFinishedJob(time_now);
        RecordFinishedJobsCFS(time_now, run_queue, dag_tasks);
        UpdateActiveCounts(run_queue, dag_tasks, active_jobs_count,
                           run_queue_set, accumulated_et);
        ReleaseJobsCFS(time_now, end_time, dag_tasks, run_queue, traces,
                       trace_indices);

        // Update active counts for newly released jobs and manage set
        int prev_counts[16];  // small fixed-size buffer for speed
        size_t n_tasks = dag_tasks.tasks.size();
        for (size_t i = 0; i < n_tasks; ++i) {
            prev_counts[i] = active_jobs_count[dag_tasks.tasks[i].id];
        }
        for (size_t i = 0; i < n_tasks; ++i) {
            int tid = dag_tasks.tasks[i].id;
            int new_c = 0;
            for (const auto& job_info : run_queue.job_queue_) {
                if (job_info.job.taskId == tid) {
                    ++new_c;
                }
            }
            if (prev_counts[i] == 0 && new_c > 0) {
                run_queue_set.insert({accumulated_et[tid], tid});
            }
            active_jobs_count[tid] = new_c;
        }

        ScheduleCFS(time_now, run_queue, run_queue_set, running_task_id);
    }

    // CFS does not use time limits; pass all -1
    std::vector<double> time_limits(dag_tasks.tasks.size(), -1);
    interval_sp_metrics_.push_back(ObtainSP_TaskSet_And_TimeLimits(
        dag_tasks.tasks, sp_parameters, time_limits));
}

}  // namespace SP_OPT_PA

#pragma once

#include <math.h>

#include <fstream>
#include <iostream>
#include <vector>

#include "sources/Safety_Performance_Metric/Probability.h"
#include "sources/Utils/DeclareDAG.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/colormod.h"
#include "sources/Utils/testMy.h"

namespace SP_OPT_PA {

inline bool CompareStringNoCase(const std::string& s1, const std::string s2) {
    return strcasecmp(s1.c_str(), s2.c_str()) == 0;
}
struct TimePerfPair {
    TimePerfPair(double t, double p) : time_limit(t), performance(p) {}
    double time_limit;  // units are ms
    double performance;
};

std::vector<double> str_seq2vector(const std::string& strs);

std::vector<TimePerfPair> AnalyzeTimePerfPair(const std::string& time_strs,
                                              const std::string& perf_strs);

class Task {
   public:
    // Task() {}
    // Default priority assignment is the index of the task in TaskSet
    Task(int id, const FiniteDist& exec, double period, double ddl,
         double priority, std::string name = "")
        : id(id),
          execution_time_dist(exec),
          period(period),
          deadline(ddl),
          priority(priority),
          name(name) {
        if (name == "")
            name = "Task_" + std::to_string(id);
        executionTime = -1;

#if defined(RYAN_HE_CHANGE)
        priorityType_ = "RM";  //
#endif
    }

#if defined(RYAN_HE_CHANGE)
    // RYAN_HE: allow set priority at runtime
    void set_priority(double p) { priority = p; }

    // modify public member priorityType_ to change how to calculate the value:
    // a larger return value means higher priority

    // RYAN_HE: this is old api. For RM and fixed execution time, it should
    // work.
    double get_priority() const;
    // double get_priority(int time_now) const;

    // RYAN_HE: this is new api. For CSP EDF, need this api
    double get_priority2(LLint time_now, LLint deadlineObj, LLint jobId) const;
#endif

    void print() {
        std::cout << "The period is: " << period << " The deadline is "
                  << deadline << std::endl;
    }

#if defined(RYAN_HE_CHANGE)
    // RYAN HE: return random execution time for a job following its
    // distribution
    double getExecutionTimeFromDist() {
        double v = execution_time_dist.getRandomValue();
#if defined(RYAN_HE_CHANGE_DEBUG)
        if (GlobalVariables::debugMode & DBG_PRT_MSK_TSK) {
            std::cout << "####getExecutionTimeFromDist: The execution time "
                         "distrubition is: "
                      << std::endl;
            execution_time_dist.print();
            std::cout << "####getExecutionTimeFromDist: The execution time is: "
                      << v << std::endl;
        }
#endif
        return v;
    }

    // RYAN HE: return min and max execution time (USED FOR RM_FAST/SLOW)
    double getExecutionTimeFromDistMin() const {
        double v = execution_time_dist.getMinValue();
        return v;
    }

    double getExecutionTimeFromDistMax() const {
        double v = execution_time_dist.getMaxValue();
        return v;
    }

    double getExecutionTimePerformanceSigma() const {
        return performance_records_sigma;
    }

    double getExecutionTimePerformanceMin() const {
        if (timePerformancePairs.size() > 0) {
            return timePerformancePairs[0].time_limit;
        } else {
            return -1.0;
        }
    }
    double getExecutionTimePerformanceMax() const {
        if (timePerformancePairs.size() > 0) {
            return timePerformancePairs[timePerformancePairs.size() - 1]
                .time_limit;
        } else {
            return -1.0;
        }
    }

    // RYAN HE: return the fixed execution time (int) since simulation time
    // granularity is ms
    int getExecutionTime() const {
        if (executionTime > 0)
            return executionTime;
        else
            CoutError("Execution time is not set!");
        return 0;
    }
    void setExecutionTime(int x) { executionTime = x; }
#else

    double getExecutionTime() const {
        if (executionTime > 0)
            return executionTime;
        else
            CoutError("Execution time is not set!");
        return 0;
    }
    void setExecutionTime(double x) { executionTime = x; }
#endif

    // TODO: test this scale function
    void Scale(double k) {
        period *= k;
        executionTime *= k;
        execution_time_dist.Scale(k);
        deadline *= k;
    }

    double utilization() const { return double(getExecutionTime()) / period; }

    inline void setExecGaussian(const GaussianDist& exec_gau) {
        exec_time_gauss = exec_gau;
    }

    inline GaussianDist getExecGaussian() const { return exec_time_gauss; }
    // Member list
    int id;
    FiniteDist execution_time_dist;
    int period;  // must be integer for the system simulation to work; if not
                 // integer, must be scaled to be so;
    double deadline;
    double priority;       // smaller value mean higher priority
    std::string name;      // optional
    int processorId = -1;  // -1 means not assigned to any processor, or all
                           // assigned to one single processor by default
    double total_running_time;
    std::vector<TimePerfPair> timePerformancePairs;

#if defined(RYAN_HE_CHANGE)
    // RYAN HE: add priority type to support assigned, RM, EDF.
    // Sen added one more type: FTP_Read_Priority_Value
    std::string priorityType_;
    double performance_records_sigma;
#endif

   private:
#if defined(RYAN_HE_CHANGE)
    // RYAN HE: change executionTime to int since simulation time granularity is
    // ms
    int executionTime;
#else
    double executionTime;
#endif
    GaussianDist exec_time_gauss;  // not used except IO part
};
typedef std::vector<SP_OPT_PA::Task> TaskSet;

inline double Utilization(const TaskSet& tasks) {
    double utilization = 0;
    for (uint i = 0; i < tasks.size(); i++)
        utilization += tasks[i].utilization();
    return utilization;
}

void ScaleToInteger(TaskSet& tasks);

inline void SortTasksByPriority(TaskSet& tasks) {
    std::sort(tasks.begin(), tasks.end(), [](const Task& t1, const Task& t2) {
        return t1.priority < t2.priority;
    });
}

long long int HyperPeriod(const TaskSet& tasks);

TaskSet ReadTaskSet(std::string path,
                    int granulairty = GlobalVariables::Granularity);

void AssignTaskSetPriorityById(TaskSet& tasks);

class TaskSetInfoDerived {
   public:
    TaskSetInfoDerived() {}

    TaskSetInfoDerived(const TaskSet& tasksInput) {
        tasks = tasksInput;
        N = tasks.size();
        hyper_period = HyperPeriod(tasks);
        variableDimension = 0;
        length = 0;
        for (int i = 0; i < N; i++) {
            LLint size = hyper_period / tasks[i].period;
            sizeOfVariables.push_back(size);
            variableDimension += size;
            length += sizeOfVariables[i];
        }
        RecordTaskPosition();
    }

    inline const Task& GetTask(uint task_id) const {
        return tasks[task_id2position_.at(task_id)];
    }
    inline const TaskSet& GetTaskSet() const { return tasks; }
#if defined(RYAN_HE_CHANGE)
    // RYAN HE: add it for CSP
    // CSP scheduler returns priority vectory, need to map index to id and then
    // get task so that we can assign task priority
    inline Task& GetTaskForPriority(uint task_id) {
        return tasks[task_id2position_.at(task_id)];
    }
#endif

    void RecordTaskPosition() {
        for (int i = 0; i < static_cast<int>(tasks.size()); i++) {
            task_id2position_[tasks[i].id] = i;
        }
    }
    inline int GetVariableSize(uint task_id) const {
        return sizeOfVariables[task_id2position_.at(task_id)];
    }

    // data members
    int N;
    LLint variableDimension;
    LLint hyper_period;
    LLint length;
    std::unordered_map<int, int> task_id2position_;

    TaskSet tasks;

   private:
    std::vector<LLint> sizeOfVariables;
};
int GetHyperPeriod(const TaskSetInfoDerived& tasks_info,
                   const std::vector<int>& chain);

template <typename T>
VectorDynamic GetParameterVD(const TaskSet& taskset,
                             std::string parameterType) {
    uint N = taskset.size();
    VectorDynamic parameterList;
    parameterList.resize(N, 1);
    parameterList.setZero();

    for (uint i = 0; i < N; i++) {
        if (parameterType == "period")
            parameterList(i, 0) = ((T)(taskset[i].period));
        else if (parameterType == "executionTime")
            parameterList(i, 0) = ((T)(taskset[i].getExecutionTime()));
        else if (parameterType == "deadline")
            parameterList(i, 0) = ((T)(taskset[i].deadline));
        else {
            std::cout << Color::red
                      << "parameterType in GetParameter is not recognized!\n"
                      << Color::def << std::endl;
            throw;
        }
    }
    return parameterList;
}

template <typename T>
std::vector<T> GetParameter(const TaskSet& taskset, std::string parameterType) {
    VectorDynamic resEigen = GetParameterVD<T>(taskset, parameterType);
    return Eigen2Vector<T>(resEigen);
}

void WriteTaskSet(std::string path, const TaskSet& tasks);
}  // namespace SP_OPT_PA
#include "RegularTasks.h"

#include <yaml-cpp/yaml.h>

#include <filesystem>


namespace SP_OPT_PA {

#if defined(RYAN_HE_CHANGE)

// RYAN_HE: moved get_priority from .h to .cpp
// just try to avoid keep recompiling eveything when anything changes in .h
// this function is pretty much unchanged
// NOTE: bigger priority means higher priority
double Task::get_priority() const {
  if (CompareStringNoCase(priorityType_, "RM")) {
    if (period > 0)
      return 1.0 / period +
             float(id) / pow(10, 6);  // pow(10, 6) because the minimum
                                      // possible period is 10^4; adding id
                                      // into priority breaks the tie when
                                      // different tasks have the same periods
    else
      CoutError("Period parameter less or equal to 0!");
  } else if (CompareStringNoCase(priorityType_, "orig")) {
    return id * -1;
  } else if (CompareStringNoCase(priorityType_, "assigned")) {
    return priority;
  }  
  else if (CompareStringNoCase(priorityType_, "DM")) {
	// d bigger means lower priority
    //std::cout << "####Task::priority: task_id="<<id<<", deadline="<<deadline<<", deadline="<<deadline<<std::endl;
	// the smaller deadline, the bigger priority should be  
    if (deadline > 0)
      return 1.0 / deadline +
             float(id) / pow(10, 6);  // pow(10, 6) because the minimum
                                      // possible period is 10^4; adding id
                                      // into priority breaks the tie when
                                      // different tasks have the same periods
    else
      return (-deadline) + 1e6 + float(id) / pow(10, 6);
  }  
  else {
    CoutError("Priority settings not recognized!");
  }
  return -1;
}

// RYAN_HE: for RunQueue2. 
// given time_now, deadlineJob and jobId, so that EDF priority can be computed
// jobId is used to break ties when EDF is same 
double Task::get_priority2(LLint time_now, LLint deadlineJob, LLint jobId) const {
  if (CompareStringNoCase(priorityType_, "RM")) {
    if (period > 0) {
      double f = pow(10,6);
      return 1.0 / period - double(jobId)/f + 
             double(id) / f;  // pow(10, 6) because the minimum
                                      // possible period is 10^4; adding id
                                      // into priority breaks the tie when
                                      // different tasks have the same periods
    } else {
      CoutError("Period parameter less or equal to 0!");
    }
  } else if (CompareStringNoCase(priorityType_, "orig")) {
    return id * -1;
  } else if (CompareStringNoCase(priorityType_, "assigned")) {
    return priority;
  } 
  else if (CompareStringNoCase(priorityType_, "DM")) {
	// d bigger means lower priority

	// the smaller deadline, the bigger priority should be  
	int d = deadlineJob-time_now;
	//std::cout << "####Task::priority_arg: task_id="<<id<<", deadlineObj="<<deadlineObj<<", relative deadline="<<d<<std::endl;
    if (d > 0) {
	  double f = pow(10,6); // for same id, jobid bigger, prio should be smaller
      return 1.0 / d - double(jobId)/f +
             double(id) / f;  // pow(10, 6) because the minimum
                                      // possible period is 10^4; adding id
                                      // into priority breaks the tie when
                                      // different tasks have the same periods
    } else {
      double f = pow(10,6);
      return (-d) - double(jobId)/f + double(id) / f;
	}
  }
  else {
    CoutError("Priority settings not recognized!");
  }
  return -1;	
}

/*
double Task::get_priority(int time_now) const {
  if (CompareStringNoCase(priorityType_, "RM")) {
    if (period > 0)
      return 1.0 / period +
             float(id) / pow(10, 6);  // pow(10, 6) because the minimum
                                      // possible period is 10^4; adding id
                                      // into priority breaks the tie when
                                      // different tasks have the same periods
    else
      CoutError("Period parameter less or equal to 0!");
  } else if (CompareStringNoCase(priorityType_, "orig")) {
    return id * -1;
  } else if (CompareStringNoCase(priorityType_, "assigned")) {
    return priority;
  } 
  else if (CompareStringNoCase(priorityType_, "DM")) {
	// d bigger means lower priority

	// the smaller deadline, the bigger priority should be  
	int d = deadline-time_now;
	std::cout << "####Task::priority_arg: task_id="<<id<<", deadline="<<deadline<<", relative deadline="<<d<<std::endl;
		
	// int d = deadline; // if deadline -- every cycle, just use this
    //std::cout << "####Task::priority_arg: task_id="<<id<<", deadline="<<deadline<<", deadline="<<deadline<<std::endl;	
    if (d > 0)
      return 1.0 / d +
             float(id) / pow(10, 6);  // pow(10, 6) because the minimum
                                      // possible period is 10^4; adding id
                                      // into priority breaks the tie when
                                      // different tasks have the same periods
    else
      return (-d) + 1e6 + float(id) / pow(10, 6);
  }
  else {
    CoutError("Priority settings not recognized!");
  }
  return -1;
}
*/
#endif

std::vector<double> str_seq2vector(const std::string &strs) {
    std::vector<double> res;
    res.reserve(strs.size());
    std::istringstream iss(strs);
    double number;
    // Read doubles from the stringstream and push them into the vector
    while (iss >> number) {
        res.push_back(number);
    }
    return res;
}
std::vector<TimePerfPair> AnalyzeTimePerfPair(const std::string &time_strs,
                                              const std::string &perf_strs) {
    std::vector<double> time_records = str_seq2vector(time_strs);
    std::vector<double> perf_records = str_seq2vector(perf_strs);
    std::vector<TimePerfPair> res;
    res.reserve(time_records.size());
    for (uint i = 0; i < time_records.size(); i++) {
        res.push_back(TimePerfPair(time_records[i], perf_records[i]));
    }
    return res;
}

// Recursive function to return gcd of a and b
long long gcd(long long int a, long long int b) {
    if (b == 0) return a;
    return gcd(b, a % b);
}

// Function to return LCM of two numbers
long long int lcm(long long int a, long long int b) {
    return (a / gcd(a, b)) * b;
}

long long int HyperPeriod(const TaskSet &tasks) {
    int N = tasks.size();
    if (N == 0) {
        std::cout << Color::red << "Empty task set in HyperPeriod()!\n";
        throw;
    } else {
        long long int hyper = tasks[0].period;
        for (int i = 1; i < N; i++) {
            hyper = lcm(hyper, tasks[i].period);
            if (hyper < 0 || hyper > LLONG_MAX) {
                std::cout << Color::red << "The hyper-period over flows!"
                          << Color::def << std::endl;
                throw;
            }
        }
        return hyper;
    }
}

TaskSet ReadTaskSet(std::string path, int granulairty) {
    if (!(std::filesystem::exists(path))) CoutError(path + " not exist!");
    YAML::Node config = YAML::LoadFile(path);
    YAML::Node tasksNode;
    if (config["tasks"]) {
        tasksNode = config["tasks"];
    } else {
        CoutError("Input file doesn't follow DAG format: " + path);
    }

    TaskSet tasks;
    tasks.reserve(tasksNode.size());
    int count = 0;
    for (size_t i = 0; i < tasksNode.size(); i++) {
        GaussianDist gauss(tasksNode[i]["execution_time_mu"].as<double>(),
                           tasksNode[i]["execution_time_sigma"].as<double>());
        FiniteDist finite_dist(
            gauss, tasksNode[i]["execution_time_min"].as<double>(),
            tasksNode[i]["execution_time_max"].as<double>(), granulairty);
        Task task(tasksNode[i]["id"].as<int>(), finite_dist,
                  tasksNode[i]["period"].as<double>(),
                  tasksNode[i]["deadline"].as<double>(), count++,
                  tasksNode[i]["name"].as<std::string>());
        if (tasksNode[i]["processorId"])
            task.processorId = tasksNode[i]["processorId"].as<int>();
        if (tasksNode[i]["total_running_time"])
            task.total_running_time =
                tasksNode[i]["total_running_time"].as<double>();
#if defined(RYAN_HE_CHANGE)
        if (tasksNode[i]["performance_records_sigma"]) {
            task.performance_records_sigma =
                tasksNode[i]["performance_records_sigma"].as<double>();
        } else {
            task.performance_records_sigma = 0.0;
        }
#endif                
        if (tasksNode[i]["performance_records_time"]) {
            task.timePerformancePairs = AnalyzeTimePerfPair(
                tasksNode[i]["performance_records_time"].as<std::string>(),
                tasksNode[i]["performance_records_perf"].as<std::string>());
            sort(task.timePerformancePairs.begin(),
                 task.timePerformancePairs.end(),
                 [](const TimePerfPair &a, const TimePerfPair &b) {
                     return a.time_limit < b.time_limit;
                 });
        }
        task.setExecGaussian(
            GaussianDist(tasksNode[i]["execution_time_mu"].as<double>(),
                         tasksNode[i]["execution_time_sigma"].as<double>()));
        tasks.push_back(task);
    }
    ScaleToInteger(tasks);
    return tasks;
}

void WriteTaskSet(std::string path, const TaskSet &tasks) {
    YAML::Node tasks_nodes;
    for (const Task &task : tasks) {
        YAML::Node task_node;
        task_node["id"] = std::to_string(task.id);
        task_node["execution_time_mu"] =
            std::to_string(task.getExecGaussian().mu);
        task_node["execution_time_sigma"] =
            std::to_string(task.getExecGaussian().sigma);
        task_node["execution_time_min"] =
            std::to_string(task.execution_time_dist.min_time);
        task_node["execution_time_max"] =
            std::to_string(task.execution_time_dist.max_time);
        task_node["period"] = std::to_string(task.period);
        task_node["deadline"] = std::to_string(task.deadline);
        task_node["name"] = task.name;
        tasks_nodes.push_back(task_node);
    }
    YAML::Node nodeRoot;
    nodeRoot["tasks"] = tasks_nodes;
    std::ofstream fout(path);
    fout << nodeRoot;
    fout.close();
}

void ScaleToInteger(TaskSet &tasks) {
    double min_period = tasks[0].period;
    for (uint i = 0; i < tasks.size(); i++) {
        min_period = min(tasks[i].period, min_period);
    }
    if (min_period < 1) {
        for (Task &task_curr : tasks) {
            task_curr.Scale(1 / min_period);
        }
    }
}

int GetHyperPeriod(const TaskSetInfoDerived &tasks_info,
                   const std::vector<int> &chain) {
    int hp = 1;
    for (int task_id : chain) {
        int period_curr = ceil(tasks_info.GetTask(task_id).period);
        hp = std::lcm(hp, period_curr);
    }
    return hp;
}

void AssignTaskSetPriorityById(TaskSet &tasks) {
    for (uint i = 0; i < tasks.size(); i++) tasks[i].priority = tasks[i].id;
}

}  // namespace SP_OPT_PA
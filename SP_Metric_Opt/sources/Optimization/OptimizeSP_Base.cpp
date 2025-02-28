
#include "sources/Optimization/OptimizeSP_Base.h"
#include "sources/UtilsForROS2/profile_and_record_time.h"

namespace SP_OPT_PA {

bool ifTimeout(TimerType start_time) {
    auto curr_time = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(curr_time - start_time)
            .count() >= GlobalVariables::TIME_LIMIT) {
        std::cout << "\nTime out when running OptimizeOrder. Maximum time is "
                  << GlobalVariables::TIME_LIMIT << " seconds.\n\n";
        return true;
    }
    return false;
}

PriorityVec GetPriorityAssignments(const TaskSet& tasks) {
    TaskSet tasks_copy = tasks;
    SortTasksByPriority(tasks_copy);
    PriorityVec pa_vec;
    pa_vec.reserve(tasks.size());
    for (uint i = 0; i < tasks_copy.size(); i++)
        pa_vec.push_back(tasks_copy[i].id);
    return pa_vec;
}

TaskSet UpdateTaskSetPriorities(const TaskSet& tasks,
                                const PriorityVec& priority_assignment) {
    TaskSet tasks_res = tasks;
    for (uint i = 0; i < priority_assignment.size(); i++) {
        if (tasks_res[priority_assignment[i]].id != priority_assignment[i])
            CoutError(
                "Input task set to UpdateTaskSetPriorities must be ordered by "
                "task id!");
        tasks_res[priority_assignment[i]].priority = i;
    }
    SortTasksByPriority(tasks_res);
    return tasks_res;
}

void PrintPriorityVec(const TaskSet& tasks,
                      const PriorityVec& priority_assignment) {
    std::cout << "Priority assignment (small values have higher priority):\n";
    for (uint i = 0; i < priority_assignment.size(); i++) {
        std::cout << tasks[priority_assignment[i]].name << ": " << i << "\n";
    }
}

bool check_task_set_applicable(const TaskSet& tasks) {
    int slam_index = -1, tsp_index = -1, rrt_index = -1, mpc_index = -1;
    for (uint i = 0; i < tasks.size(); i++) {
        if (tasks[i].name == "SLAM") slam_index = i;
        if (tasks[i].name == "TSP") tsp_index = i;
        if (tasks[i].name == "RRT") rrt_index = i;
        if (tasks[i].name == "MPC") mpc_index = i;
    }
    if (slam_index == -1 || tsp_index == -1 || rrt_index == -1 ||
        mpc_index == -1) {
        return false;
    }
    if (tasks[slam_index].processorId != tasks[tsp_index].processorId) {
        return false;
    }

    if (tasks[rrt_index].processorId != tasks[mpc_index].processorId) {
        return false;
    }
    return true;
}

std::unordered_map<std::string, int> Task2priority_value(
    const TaskSet& tasks, const PriorityVec& priority_assignment) {
    /*
    Logic for this function: Minimize priority assignment values' changes.
    This function only works for a specific task set where SLAM and TSP are
    assigned to the same core, and MPC and RRT are assigned to the same core.

    In this case, SLAM's priority is always 5, MPC's priority is always 2.
    If TSP has higher priority, it is 6, otherwise, 4;
    If RRT has higher priority, it is 3, otherwise, 1.
    */
    if (!check_task_set_applicable(tasks))
        CoutError("Task set is not applicable for Task2priority_value()!");
    std::unordered_map<std::string, int> task2priority;
    for (uint i = 0; i < priority_assignment.size(); i++) {
        task2priority[tasks[priority_assignment[i]].name] =
            priority_assignment.size() - i;
    }
    if (task2priority["SLAM"] < task2priority["TSP"]) {
        task2priority["SLAM"] = 5;
        task2priority["TSP"] = 6;
    } else {
        task2priority["SLAM"] = 5;
        task2priority["TSP"] = 4;
    }
    if (task2priority["RRT"] < task2priority["MPC"]) {
        task2priority["RRT"] = 1;
        task2priority["MPC"] = 2;
    } else {
        task2priority["MPC"] = 2;
        task2priority["RRT"] = 3;
    }

    return task2priority;
}

YAML::Node PriorityAssignmentToYaml(const TaskSet& tasks,
                                    const PriorityVec& priority_assignment) {
    auto task2priority = Task2priority_value(tasks, priority_assignment);
    YAML::Node dictionary;
    for (uint i = 0; i < priority_assignment.size(); i++) {
        dictionary[tasks[priority_assignment[i]].name] =
            task2priority[tasks[priority_assignment[i]].name];
    }
    return dictionary;
}

void WritePriorityAssignments(std::string path, const TaskSet& tasks,
                              const PriorityVec& pa_vec_input,
                              double time_taken) {
    auto dictionary = PriorityAssignmentToYaml(tasks, pa_vec_input);
    std::ofstream outputFile(path, std::ios::out);

    // Check if the file was opened successfully
    if (outputFile.is_open()) {
        outputFile
            << "# Priority assignments for each task (ordered by task id);\n"
            << "#For example, the first value is the priority for the first "
               "task with id = 0\n\n";
        // Iterate through the vector and write each element to the file
        // for (const int& element : pa_vec) {
        //     outputFile << element << "\n";
        // }
        outputFile << dictionary;

        outputFile << "\n# Run-time: "
                   << std::to_string(time_taken) + " seconds\n";
        // Close the file
        outputFile.close();

        std::cout << "Vector has been written to " << path << std::endl;
    } else {
        std::cerr << "Unable to open the file: " << path << std::endl;
    }
}

double EvaluateSPWithPriorityVec(const DAG_Model& dag_tasks,
                                 const SP_Parameters& sp_parameters,
                                 const PriorityVec& priority_assignment) {

    auto start_time = CurrentTimeInProfiler;
    TaskSet tasks_eval =
        UpdateTaskSetPriorities(dag_tasks.tasks, priority_assignment);
    DAG_Model dag_tasks_eval = dag_tasks;
    dag_tasks_eval.tasks = tasks_eval;
    auto res = ObtainSP_DAG(dag_tasks_eval, sp_parameters);

    auto finish_time = CurrentTimeInProfiler;
    if(GlobalVariables::debugMode == 1)
        CoutWarning("Running time to evaluate SP for one time: " + \
            std::to_string(getDuration(start_time, finish_time)));
    return res;
}

void PrintPA_IfDebugMode(const PriorityVec& priority_assignment,
                         double sp_eval) {
    if (GlobalVariables::debugMode == 1) {
        std::cout
            << "Try PA assignments (Task ID, high priority to low priority): ";
        for (int x : priority_assignment) std::cout << x << ", ";
        std::cout << sp_eval << "\n";
    }
}
void WriteTimeLimitToYamlOSM(double time_limit_ms) {
    if (time_limit_ms < 0) {  // not valid input
        CoutWarning("Not valid input for WriteTimeLimitToYamlOSM\n");
        return;
    }
    std::string path = get_tsp_config_file_path();

    YAML::Node config = YAML::LoadFile(path);
    config["general"]["max_time"] = time_limit_ms / 1000.0;
    std::ofstream fout(path);
    fout << config;
    fout.close();
}
}  // namespace SP_OPT_PA
#pragma once

#include <iostream>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <sched.h>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>
#include <sstream>
#include <array>
#include <memory>
#include <unordered_map>

#include <yaml-cpp/yaml.h>
#include "profiler.h"

class RealTimeManager
{
public:
    RealTimeManager() {}
    void setCPUAffinityAndPriority(std::string file_name = "")
    {
        // IMPORTANT: when setting cpu and priorities, the task names need to be ESACT the same as the node (executable) name defined in ROS2 CMake file
        std::unordered_map<std::string, std::string> executable_name_map = {
            {"TSP", "tsp_solver_listener"}, {"MPC", "listener_mpc"}, {"RRT", "rrt_listener"}, {"SLAM", "listener_slam"}, {"TALKER", "talker"}};

        std::vector<std::string> task_names;     // all task names
        std::vector<std::vector<int>> cpu_lists; // CPU cores you want to assign
        std::vector<int> scheduling_policies;    // desired scheduling policy
        std::vector<int> priorities;

        std::ifstream fin;
        if (file_name == "")
        {
            fin.open(getTimeRecordFolder() + "cpu_and_priority.yaml");
        }
        else
        {
            fin.open(file_name);
        }

        // Check if file is opened successfully
        if (!fin.is_open())
        {
            std::cerr << "Failed to open file." << std::endl;
            return;
        }

        // Load YAML data
        YAML::Node data = YAML::Load(fin);

        int max_priority = 0;         // the talker will have the largest priority
        bool has_fifo_policy = false; // if no FIFO policy, then the talker will use CFS, otherwise the talker will be FIFO with the largest priority
        for (std::size_t i = 0; i < data["tasks"].size(); i++)
        {
            // task name
            std::string name = data["tasks"][i]["name"].as<std::string>();
            task_names.push_back(executable_name_map[name]);

            // task cpu list
            cpu_lists.push_back({data["tasks"][i]["processorId"].as<int>()});
            for (int k = 0; k < cpu_lists.back().size(); k++) { // add +2 offset to save the core 0 for talkers, core 1 for ros2 launch
                cpu_lists[cpu_lists.size()-1][k] += 2;
            }

            // task policy
            std::string scheduling_policy = data["tasks"][i]["scheduling_policy"].as<std::string>();
            if (scheduling_policy == "SCHED_FIFO")
            {
                scheduling_policies.push_back(1); // 1 is for SCHED_FIFO
                has_fifo_policy = true;
            }
            else if (scheduling_policy == "SCHED_OTHER")
            {
                scheduling_policies.push_back(0); // 0 is for SCHED_OTHER, which is CPS
            }
            else
            {
                std::cerr << "Unrecognized scheduling policy: " << scheduling_policy << std::endl;
                std::cerr << "Supported policies are: SCHED_FIFO, SCHED_OTHER. \n";
                return;
            }

            // task priority
            int priority = data["tasks"][i]["priority"].as<int>();
            if (scheduling_policy == "SCHED_OTHER")
            {
                priorities.push_back(0); // SCHED_OTHER or CFS, only support priority level of 0
            }
            else
            {
                priorities.push_back(priority);
                if (max_priority < priority)
                {
                    max_priority = priority;
                }
            }
        }

        // add the talker task
        task_names.push_back("talker");
        cpu_lists.push_back({0});
        // add the ros2 launch task
        task_names.push_back("\"ros2 launch\"");
        cpu_lists.push_back({1});

        if (has_fifo_policy)
        {
            scheduling_policies.push_back(1);
            priorities.push_back(std::min(99, max_priority + 5));
            scheduling_policies.push_back(1);
            priorities.push_back(std::min(99, max_priority + 5));
        }
        else
        {
            scheduling_policies.push_back(0);
            priorities.push_back(0);
            scheduling_policies.push_back(0);
            priorities.push_back(0);
        }

        for (std::size_t i = 0; i < task_names.size(); i++)
        {
            std::cout << task_names[i] << ":   policy:" << scheduling_policies[i] << ",   priority:" << priorities[i] << "\n        cpu lists:";
            for (auto cpu : cpu_lists[i])
                std::cout << cpu << ", ";
            std::cout << std::endl;
        }

        setCPUAffinityAndPriority(task_names, cpu_lists, scheduling_policies, priorities);
    }

    void setCPUAffinityAndPriority(std::vector<std::string> task_names = {"talker"},    // all task names, if use this function, "talker" should be included as well
                                   std::vector<std::vector<int>> cpu_lists = {{0}},     // CPU cores you want to assign
                                   std::vector<int> scheduling_policies = {SCHED_FIFO}, // desired scheduling policy
                                   std::vector<int> priorities = {10})
    {
        if (task_names.size() != cpu_lists.size() || task_names.size() != scheduling_policies.size() || task_names.size() != priorities.size())
        {
            perror("setCPUAffinityAndPriority");
            std::cerr << "The input vectors don't have the same size! " << std::endl;
            return;
        }

        for (std::size_t i = 0; i < task_names.size(); i++)
            setCPUandScheduling(task_names[i], cpu_lists[i], scheduling_policies[i], priorities[i]);
    }

private:
    std::string exec(const char *cmd)
    {
        std::array<char, 128> buffer;
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
        if (!pipe)
        {
            throw std::runtime_error("popen() failed!");
        }
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
        {
            result += buffer.data();
        }
        return result;
    }

    std::vector<pid_t> getProcessPids(const std::string &task_name)
    {
        std::string cmd_pidof = "pgrep -f " + task_name;
        std::string pidList = exec(cmd_pidof.c_str());
        std::istringstream iss(pidList);
        std::vector<pid_t> pids;
        pid_t pid;
        while (iss >> pid)
        {
            pids.push_back(pid);
        }
        return pids;
    }

    std::vector<pid_t> getAllThreadIds(const std::vector<pid_t> &pids)
    {
        std::vector<pid_t> thread_ids;
        for (const auto &pid : pids)
        {
            std::string cmd = "ps -T -p " + std::to_string(pid) + " -o tid=";
            std::string output = exec(cmd.c_str());
            std::istringstream iss_tid(output);
            pid_t tid;
            while (iss_tid >> tid)
            {
                thread_ids.push_back(tid);
            }
        }
        return thread_ids;
    }

    void setCPUAffinity(const std::vector<pid_t> &thread_ids, const std::vector<int> &cpu_list)
    {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        for (int cpu : cpu_list)
        {
            CPU_SET(cpu, &cpuset);
        }
        for (pid_t tid : thread_ids)
        {
            if (sched_setaffinity(tid, sizeof(cpu_set_t), &cpuset) != 0)
            {
                perror("sched_setaffinity");
                std::cerr << "Error setting CPU affinity for thread " << tid << std::endl;
            }
        }
    }

    void setSchedulingPolicy(const std::vector<pid_t> &thread_ids, int policy, int priority)
    {
        struct sched_param param;
        param.sched_priority = priority;
        for (pid_t tid : thread_ids)
        {
            if (sched_setscheduler(tid, policy, &param) != 0)
            {
                perror("sched_setscheduler");
                std::cerr << "Error setting scheduling policy for thread " << tid << std::endl;
            }
        }
    }

    void setCPUandScheduling(std::string task_name = "talker",                     // task name
                             std::vector<int> cpu_list = {0, 1, 2, 3, 4, 5, 6, 7}, // CPU cores you want to assign
                             int scheduling_policy = SCHED_FIFO,                   // desired scheduling policy
                             int priority = 1)
    {
        try
        {
            std::vector<pid_t> pids = getProcessPids(task_name);
            if (!pids.empty())
            {
                std::cout << "PID(s) of task '" << task_name << "': ";
                for (pid_t pid : pids)
                {
                    std::cout << pid << " ";
                }
                std::cout << std::endl;

                std::vector<pid_t> thread_ids = getAllThreadIds(pids);
                if (!thread_ids.empty())
                {
                    std::cout << "Thread IDs: ";
                    for (pid_t tid : thread_ids)
                    {
                        std::cout << tid << " ";
                    }
                    std::cout << std::endl;

                    setCPUAffinity(thread_ids, cpu_list);
                    setSchedulingPolicy(thread_ids, scheduling_policy, priority);
                }
                else
                {
                    std::cout << "No thread IDs found." << std::endl;
                }
            }
            else
            {
                std::cout << "Task '" << task_name << "' not found or not running." << std::endl;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Exception caught: " << e.what() << std::endl;
            return;
        }
    }
};
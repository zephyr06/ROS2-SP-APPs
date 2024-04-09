#pragma once
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>

#include "string"

void VerifyFileExist(std::string filename)
{
    std::ifstream fin(filename);
    if (!fin.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
    }
}
void UpdatePriorityAssignments(std::string tasks_yaml,
                               std::string priority_yaml)
{
    VerifyFileExist(tasks_yaml);
    VerifyFileExist(priority_yaml);

    YAML::Node tasks = YAML::LoadFile(tasks_yaml);
    YAML::Node priorities = YAML::LoadFile(priority_yaml);

    for (YAML::const_iterator it = priorities.begin(); it != priorities.end();
         ++it)
    {
        std::string app_name = it->first.as<std::string>();
        int priority = it->second.as<int>();
        // cout << app_name << ", " << priority << "\n";
        for (YAML::Node::iterator it_task = tasks["tasks"].begin();
             it_task != tasks["tasks"].end(); ++it_task)
        {
            if (app_name == it_task->operator[]("name").as<std::string>())
            {
                it_task->operator[]("priority") = priority + 1;
                break;
            }
        }
    }

    std::ofstream fout(tasks_yaml);
    YAML::Emitter emitter;

    // Emit the YAML content to the emitter
    emitter << tasks;
    // Write the YAML content to the file
    fout << emitter.c_str();

    // Close the file
    fout.close();
}

void UpdateProcessorAssignmentsFromYamlFile(std::string local_yaml, std::string task_charac_yaml)
{
    VerifyFileExist(local_yaml);
    VerifyFileExist(task_charac_yaml);

    YAML::Node local_tasks = YAML::LoadFile(local_yaml);
    YAML::Node global_tasks = YAML::LoadFile(task_charac_yaml);

    for (YAML::Node::iterator it_g = global_tasks["tasks"].begin(); it_g != global_tasks["tasks"].end(); ++it_g)
    {
        std::string name_global = it_g->operator[]("name").as<std::string>();

        for (YAML::Node::iterator it_l = local_tasks["tasks"].begin(); it_l != local_tasks["tasks"].end(); ++it_l)
        {
            std::string name_local = it_l->operator[]("name").as<std::string>();
            if (name_global == name_local)
            {
                it_l->operator[]("processorId") = it_g->operator[]("processorId");
            }
        }
    }
    
    std::ofstream output_file(local_yaml);
    output_file << local_tasks;
    output_file.close();
}
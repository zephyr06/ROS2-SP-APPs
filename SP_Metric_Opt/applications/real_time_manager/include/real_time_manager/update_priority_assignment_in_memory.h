#include "applications/real_time_manager/include/real_time_manager/update_priority_assignments.h"

YAML::Node LoadYamlNode(const std::string& file_path, const YAML::Node& node) {
    if (!file_path.empty()) {
        CoutWarning("The provided tasks_yaml is empty.");
        VerifyFileExist(file_path);
        return YAML::LoadFile(file_path);
    }
    return node;
}

void SaveYamlNodeToFile(const std::string& file_path, const YAML::Node& node) {
    if (!file_path.empty()) {
        std::ofstream fout(file_path);
        YAML::Emitter emitter;
        emitter << node;
        fout << emitter.c_str();
        fout.close();
    }
    CoutWarning("The generated tasks_yaml is empty.");
}
void UpdatePriorityAssignments(const std::string& tasks_yaml,
                               const YAML::Node& tasks_node,
                               const std::string& priority_yaml,
                               const YAML::Node& priority_node,
                               bool write_to_disk = true) {
    // Load tasks and priorities
    YAML::Node tasks = LoadYamlNode(tasks_yaml, tasks_node);
    YAML::Node priorities = LoadYamlNode(priority_yaml, priority_node);

    // Update tasks with priorities
    for (YAML::const_iterator it = priorities.begin(); it != priorities.end();
         ++it) {
        std::string app_name = it->first.as<std::string>();
        int priority = it->second.as<int>();

        for (YAML::Node::iterator it_task = tasks["tasks"].begin();
             it_task != tasks["tasks"].end(); ++it_task) {
            if (app_name == (*it_task)["name"].as<std::string>()) {
                (*it_task)["priority"] = priority + 1;
                break;
            }
        }
    }

    // Save updated tasks if needed
    if (write_to_disk) {
        SaveYamlNodeToFile(tasks_yaml, tasks);
    }
}

void UpdateProcessorAssignmentsFromYamlFile(const std::string& local_yaml,
                                            const YAML::Node& local_node,
                                            const std::string& global_yaml,
                                            const YAML::Node& global_node,
                                            bool write_to_disk = true) {
    // Load local and global tasks
    YAML::Node local_tasks = LoadYamlNode(local_yaml, local_node);
    YAML::Node global_tasks = LoadYamlNode(global_yaml, global_node);

    // Update processor assignments
    for (YAML::Node::iterator it_g = global_tasks["tasks"].begin();
         it_g != global_tasks["tasks"].end(); ++it_g) {
        std::string name_global = (*it_g)["name"].as<std::string>();

        for (YAML::Node::iterator it_l = local_tasks["tasks"].begin();
             it_l != local_tasks["tasks"].end(); ++it_l) {
            std::string name_local = (*it_l)["name"].as<std::string>();
            if (name_global == name_local) {
                (*it_l)["processorId"] = (*it_g)["processorId"];
            }
        }
    }

    // Save updated local tasks if needed
    if (write_to_disk) {
        SaveYamlNodeToFile(local_yaml, local_tasks);
    }
}

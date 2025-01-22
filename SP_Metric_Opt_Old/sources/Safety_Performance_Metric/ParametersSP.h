#pragma once
#include <vector>

#include "sources/TaskModel/DAG_Model.h"

namespace SP_OPT_PA {

struct SP_Parameters {
    SP_Parameters() {}
    SP_Parameters(const TaskSet& tasks) {
        for (uint i = 0; i < tasks.size(); i++) {
            thresholds_node[tasks[i].id] = 0.5;
            weights_node[tasks[i].id] = 1;
        }
    }
    SP_Parameters(const DAG_Model& dag_tasks) : SP_Parameters(dag_tasks.tasks) {
        for (uint i = 0; i < dag_tasks.chains_.size(); i++) {
            thresholds_path[i] = 0.5;
            weights_path[i] = 1;
        }
    }
    void update_node_weight(int id, double multiplier) {
        weights_node[id] *= multiplier;
    }

    void reserve(size_t size) {
        thresholds_node.reserve(size);
        weights_node.reserve(size);
        thresholds_path.reserve(size);
        weights_path.reserve(size);
    }

    // return true if the task has uniquelly highest weight; false otherwise
    bool if_highest_weight_unique(int task_id) const {
        int weight_cur = weights_node.at(task_id);
        for (auto itr = weights_node.begin(); itr != weights_node.end();
             itr++) {
            if (itr->second >= weight_cur && itr->first != task_id) {
                return false;
            }
        }
        return true;
    }

    // data
    // std::vector<double> thresholds_node;
    // std::vector<double> weights_node;
    // std::vector<double> thresholds_path;
    // std::vector<double> weights_path;
    std::unordered_map<int, double> thresholds_node;
    std::unordered_map<int, double> weights_node;
    std::unordered_map<int, double> thresholds_path;
    std::unordered_map<int, double> weights_path;
};

SP_Parameters ReadSP_Parameters(const std::string& filename);
}  // namespace SP_OPT_PA
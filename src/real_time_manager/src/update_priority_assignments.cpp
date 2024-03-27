#include "real_time_manager.h"
#include "update_priority_assignment.h"

int main() {
    // TODO update directory

    std::filesystem::path current_file_path = std::filesystem::canonical(__FILE__);
    std::filesystem::path parent_file_directory = current_file_path.parent_path().parent_path();

    std::string local_config_yaml = parent_file_directory.string() + "/configs/local_cpu_and_priority.yaml";
    std::string priority_yaml = parent_file_directory.parent_path().parent_path().parent_path().string() + "/SP_Metric_Opt/TaskData/"
        "pa_res_test_robotics_v1.yaml";
    
    
    UpdatePriorityAssignments(local_config_yaml, priority_yaml);
}
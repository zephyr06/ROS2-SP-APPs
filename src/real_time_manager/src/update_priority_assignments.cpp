#include "update_priority_assignment.h"

int main() {
    // TODO update directory
    std::string tasks_yaml =
        "/home/zephyr/Programming/ROS2-SP-APPs/src/real_time_manager/configs/"
        "local_cpu_and_priority.yaml";
    std::string priority_yaml =
        "/home/zephyr/Programming/SP_Metric_Opt/TaskData/"
        "pa_res_test_robotics_v1.yaml";
    UpdatePriorityAssignments(tasks_yaml, priority_yaml);
}
#include "real_time_manager/real_time_manager.h"
#include "real_time_manager/update_priority_assignment.h"

int main()
{
  std::filesystem::path current_file_path = std::filesystem::canonical(__FILE__);
  std::filesystem::path package_directory = current_file_path.parent_path().parent_path();
  std::string local_config_yaml = package_directory.string() + "/configs/local_cpu_and_priority.yaml";
  std::string priority_yaml = package_directory.parent_path().parent_path().string() +
                              "/SP_Metric_Opt/TaskData/pa_result.yaml";
  std::string task_characteristics_yaml = getTimeRecordFolder() + "task_characteristics.yaml";

  // Perform schedule
  // Scheduler command to be executed
  std::string cmd = package_directory.parent_path().parent_path().string() + "/SP_Metric_Opt/release";
  cmd += "/tests/AnalyzePriorityAssignment --file_path ";
  cmd += task_characteristics_yaml;
  cmd += " --output_file_path ";
  cmd += priority_yaml;
  std::cout << "cmd is:" << cmd << std::endl;
  const char *scheduler_command = cmd.c_str();
  // Execute the command
  int result = system(scheduler_command);
  // Check if command execution was successful
  if (result != 0)
  {
    // Command execution failed
    std::cerr << "Scheduler command execution failed!\n";
    return -1;
  }

  // update priorities from the scheduler to local config yaml
  UpdatePriorityAssignments(local_config_yaml, priority_yaml);
}
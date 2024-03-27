#include <cstdlib>
#include "listener_base.h"
#include "real_time_manager.h"
#include "update_priority_assignment.h"
#include "execution_time_estimator.h"

class SchedulerApp : public AppBase
{
public:
    SchedulerApp() : AppBase("scheduler") {}
    void run() override
    {
        // no execution at the first instance
        if (cnt_++ == 0)
            return;

        std::filesystem::path current_file_path = std::filesystem::canonical(__FILE__);
        std::filesystem::path parent_file_directory = current_file_path.parent_path().parent_path();
        std::string local_config_yaml = parent_file_directory.string() + "/configs/local_cpu_and_priority.yaml";
        std::string priority_yaml = parent_file_directory.parent_path().parent_path().parent_path().string() +
                                    "/SP_Metric_Opt/TaskData/pa_res_test_robotics_v1.yaml";
        std::string task_characteristics_yaml = getTimeRecordFolder() + "task_characteristics.yaml";

        // Update execution time statitics
        et_estimator_.updateTaskExecutionTimeDistributions(50);

        // Perform schedule
        // Scheduler command to be executed
        std::string cmd = parent_file_directory.parent_path().parent_path().parent_path().string() + "/SP_Metric_Opt/release";
        cmd += "/tests/AnalyzePriorityAssignment --file_path ";
        cmd += task_characteristics_yaml;
        std::cout<<"cmd is:" << cmd << std::endl;
        const char *scheduler_command = cmd.c_str();
        // Execute the command
        int result = system(scheduler_command);
        // Check if command execution was successful
        if (result != 0)
        {
            // Command execution failed
            std::cerr << "Scheduler command execution failed!\n";
            return;
        }

        // update priorities from the scheduler to local config yaml
        UpdatePriorityAssignments(local_config_yaml, priority_yaml);

        // apply the new priority assignments
        rt_manager_.setCPUAffinityAndPriority(local_config_yaml);
    }

    ExecutionTimeEstimator et_estimator_;
    RealTimeManager rt_manager_;
    int cnt_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberAppBase<SchedulerApp>>());
    rclcpp::shutdown();
    return 0;
}
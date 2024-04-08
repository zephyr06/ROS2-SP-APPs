#include <cstdlib>
#include "listener_base.h"
#include "real_time_manager/real_time_manager.h"
#include "real_time_manager/update_priority_assignment.h"
#include "real_time_manager/execution_time_estimator.h"

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
        std::filesystem::path package_directory = current_file_path.parent_path().parent_path();
        std::string local_config_yaml = package_directory.string() + "/configs/local_cpu_and_priority.yaml";
        std::string priority_yaml = package_directory.parent_path().parent_path().parent_path().string() +
                                    "/SP_Metric_Opt/TaskData/pa_res_test_robotics_v1.yaml";
        std::string task_characteristics_yaml = getTimeRecordFolder() + "task_characteristics.yaml";


        // apply the new priority assignments

        std::string local_fix_priority_config_yaml = package_directory.string() + "/configs/local_fixed_cpu_and_priority.yaml";
        rt_manager_.setCPUAffinityAndPriority(local_fix_priority_config_yaml);
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
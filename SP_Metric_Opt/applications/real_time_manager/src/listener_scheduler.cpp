#include <cstdlib>

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/Parameters.h"
#include "sources/UtilsForROS2/Publisher.h"

#include "real_time_manager/execution_time_estimator.h"
#include "real_time_manager/real_time_manager.h"
#include "real_time_manager/update_priority_assignment.h"

class SchedulerApp : public AppBase {
   public:
    SchedulerApp() : AppBase("SCHEDULER") {}
    SchedulerApp(int argc, char *argv[]) : AppBase("SCHEDULER") {
        // supported scheduler are: CFS, RM, optimizerBF, optimizerIncremental
        scheduler_ = "optimizerBF";
        if (argc >= 2) {
            scheduler_ = argv[1];
        }
        if (scheduler_ != "CFS" && scheduler_ != "RM" &&
            scheduler_ != "optimizerBF" &&
            scheduler_ != "optimizerIncremental") {
            std::cerr << "Error: Unknown scheduler: " << scheduler_
                      << ". Supported scheduler are: CFS, RM, optimizerBF, "
                         "optimizerIncremental\n";
            std::cerr << "Exiting scheduler node.\n";
            std::exit(EXIT_FAILURE);
        }
        std::cout << "Using scheduler: " << scheduler_ << ".\n";
    }
    // function arguments msg_cnt not used for now
    void run(int ) override {
        // no execution at the first instance
        if (cnt_++ == 0) return;

        std::filesystem::path current_file_path =
            std::filesystem::canonical(__FILE__);
        std::filesystem::path package_directory =
            current_file_path.parent_path().parent_path(); // ROS2-SP-APPS/SP_Metric_Opt/real_time_manager
        std::string local_config_yaml =
            package_directory.string() + "/configs/local_cpu_and_priority.yaml";
        std::string sp_opt_folder_path = package_directory.parent_path().parent_path().string();
        if(sp_opt_folder_path.substr(sp_opt_folder_path.size()-13,13)!="SP_Metric_Opt")
            std::cerr<<"Path configuraiton is wrong, mostly because some files' location chagned!\n";

        std::string priority_yaml =
            sp_opt_folder_path + "/TaskData/pa_result.yaml";
        std::string task_characteristics_yaml =
            getTimeRecordFolder() + "task_characteristics.yaml";

        if (scheduler_ == "CFS") {
            std::string local_fixed_cpu_and_priority_yaml_CFS =
                package_directory.string() +
                "/configs/local_fixed_cpu_and_priority_CFS.yaml";
            UpdateProcessorAssignmentsFromYamlFile(
                local_fixed_cpu_and_priority_yaml_CFS,
                task_characteristics_yaml);
            rt_manager_.setCPUAffinityAndPriority(
                local_fixed_cpu_and_priority_yaml_CFS);
        } else if (scheduler_ == "RM") {
            std::string local_fixed_cpu_and_priority_yaml_RM =
                package_directory.string() +
                "/configs/local_fixed_cpu_and_priority_RM.yaml";
            UpdateProcessorAssignmentsFromYamlFile(
                local_fixed_cpu_and_priority_yaml_RM,
                task_characteristics_yaml);
            rt_manager_.setCPUAffinityAndPriority(
                local_fixed_cpu_and_priority_yaml_RM);
        } else if (scheduler_ == "optimizerBF" ||
                   scheduler_ ==
                       "optimizerIncremental") {  // todo: implement the
                                                  // "optimizerIncremental"

            // Update execution time statitics, use last 10 data records, replace the missed values with 2T seconds
            et_estimator_.updateTaskExecutionTimeDistributions(10, 2*3);
            std::cout<<"Updated execution time\n";

            // Perform schedule
            // Scheduler command to be executed
            std::string cmd =
                sp_opt_folder_path+"/release";
            cmd += "/tests/AnalyzePriorityAssignment --file_path ";
            cmd += task_characteristics_yaml;
            cmd += " --output_file_path ";
            cmd += priority_yaml;
            std::cout << "cmd is:" << cmd << std::endl;
            const char *scheduler_command = cmd.c_str();
            // Execute the command
            int result = system(scheduler_command);
            // Check if command execution was successful
            if (result != 0) {
                // Command execution failed
                std::cerr << "Scheduler command execution failed!\n";
                return;
            }

            // update priorities from the scheduler to local config yaml
            UpdatePriorityAssignments(local_config_yaml, priority_yaml);
            UpdateProcessorAssignmentsFromYamlFile(local_config_yaml,
                                                   task_characteristics_yaml);

            // apply the new priority assignments
            rt_manager_.setCPUAffinityAndPriority(local_config_yaml);
        }
    }

    ExecutionTimeEstimator et_estimator_;
    RealTimeManager rt_manager_;
    int cnt_ = 0;
    std::string scheduler_;
};

int main(int argc, char *argv[]) {
    
    SP_OPT_PA::DAG_Model dag_tasks = SP_OPT_PA::ReadDAG_Tasks(
        GlobalVariables::PROJECT_PATH +
        "../all_time_records/task_characteristics.yaml");
    if(argc<3)
        std::cerr<<"Wrong arg format: example usage ./scheduler_runner optimizerBF 30000 The last argument is period of the scheduler in miliseconds\n";
    int period = std::stoi(argv[2]);
    int count = dag_tasks.tasks[0].total_running_time / period;
    SchedulerApp app(argc, argv);
    PeriodicReleaser<SchedulerApp> releaser(period, count, app);
    releaser.release();
    return 0;
}
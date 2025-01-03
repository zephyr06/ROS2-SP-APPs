#include <cstdlib>

#include "real_time_manager/execution_time_estimator.h"
#include "real_time_manager/real_time_manager.h"
#include "real_time_manager/update_priority_assignment.h"
#include "real_time_manager/execution_time_estimator.h"
#include "sources/Optimization/OptimizeSP_Incre.h"
#include "sources/Optimization/OptimizeSP_TL_Incre.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/argparse.hpp"
#include "sources/Utils/profilier.h"
#include "sources/Utils/readwrite.h"
#include "sources/UtilsForROS2/Publisher.h"
#include "sources/UtilsForROS2/Recorder.h"

class SchedulerApp : public AppBase {
   public:
    SchedulerApp() : AppBase("SCHEDULER") {}

    SchedulerApp(int argc, char *argv[]) : AppBase("SCHEDULER") {
        recorder_ = Recorder("SCHEDULER_PUER_OPT");
        recorder_.write_initial_publish_time();
        recorder_.write_initial_receive_time();
        // supported scheduler are: CFS, RM, optimizerBF, optimizerIncremental
        // scheduler_ = "optimizerBF";
        if (argc >= 2) {
            scheduler_ = argv[1];
        } else {
            std::cout
                << "Must provide a valid and supported name of scheduler!\n";
        }
        if (scheduler_ != "CFS" && scheduler_ != "RM_Fast" &&
            scheduler_ != "RM_Slow" && scheduler_ != "RM" &&
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
    void run(int) override;

    ExecutionTimeEstimator et_estimator_;
    RealTimeManager rt_manager_;
    int cnt_ = 0;
    std::string scheduler_;
    // SP_OPT_PA::OptimizePA_Incre incremental_optimizer_;
    SP_OPT_PA::OptimizePA_Incre_with_TimeLimits incremental_optimizer_w_TL_;
    Recorder recorder_;
    ExecutionTimeProfiler exe_profiler_;

};

// function arguments msg_cnt not used for now
void SchedulerApp::run(int release_index) {
    // no execution at the first instance
    if (cnt_++ == 0) return;
    TimerType start_time = CurrentTimeInProfiler;
    recorder_.write_publish_time(getCurrentTimeStamp(), release_index);
    std::filesystem::path current_file_path =
        std::filesystem::canonical(__FILE__);
    std::filesystem::path package_directory =
        current_file_path.parent_path()
            .parent_path()
            .parent_path();  // ROS2-SP-APPS/SP_Metric_Opt/real_time_manager
    std::string local_config_yaml =
        package_directory.string() + "/configs/local_cpu_and_priority.yaml";
    std::string sp_opt_folder_path =
        package_directory.parent_path().parent_path().string();
    if (sp_opt_folder_path.substr(sp_opt_folder_path.size() - 13, 13) !=
        "SP_Metric_Opt")
        std::cerr << "Path configuraiton is wrong, mostly because some "
                     "files' location chagned!\n";

    std::string priority_yaml_output_path =
        sp_opt_folder_path + "/TaskData/pa_result.yaml";
    std::string task_characteristics_yaml =
        getTimeRecordFolder() + "task_characteristics.yaml";

    auto dag_tasks = SP_OPT_PA::ReadDAG_Tasks(task_characteristics_yaml);
    double fastest_time_limit =
        dag_tasks.GetTask(0).timePerformancePairs[0].time_limit;
    double slowest_time_limit =
        dag_tasks.GetTask(0).timePerformancePairs.back().time_limit;

    if (scheduler_ == "CFS") {
        std::string local_fixed_cpu_and_priority_yaml_CFS =
            package_directory.string() +
            "/configs/local_fixed_cpu_and_priority_CFS.yaml";
        SP_OPT_PA::WriteTimeLimitToYamlOSM(slowest_time_limit);
        UpdateProcessorAssignmentsFromYamlFile(
            local_fixed_cpu_and_priority_yaml_CFS, task_characteristics_yaml);
        rt_manager_.setCPUAffinityAndPriority(
            local_fixed_cpu_and_priority_yaml_CFS);
    } else if (scheduler_ == "RM_Fast" || scheduler_ == "RM_Slow" ||
               scheduler_ == "RM") {
        std::string local_fixed_cpu_and_priority_yaml_RM =
            package_directory.string() +
            "/configs/local_fixed_cpu_and_priority_RM.yaml";
        UpdateProcessorAssignmentsFromYamlFile(
            local_fixed_cpu_and_priority_yaml_RM, task_characteristics_yaml);
        rt_manager_.setCPUAffinityAndPriority(
            local_fixed_cpu_and_priority_yaml_RM);

        if (scheduler_ == "RM_Fast") {
            SP_OPT_PA::WriteTimeLimitToYamlOSM(
                fastest_time_limit);  // only write TSP's time limit
        } else if (scheduler_ == "RM_Slow") {
            SP_OPT_PA::WriteTimeLimitToYamlOSM(
                slowest_time_limit);  // only write TSP's time limit
        } else {
            ;  // do nothing
        }
    } else {
        // Update execution time statitics, use last 10 data records,
        // replace the missed values with 2T seconds
        et_estimator_.updateTaskExecutionTimeDistributions(10, 2 * 3);
        std::cout << "Updated execution time\n";
        double time_taken;

        using namespace SP_OPT_PA;
        DAG_Model dag_tasks = ReadDAG_Tasks(task_characteristics_yaml);
        SP_Parameters sp_parameters =
            ReadSP_Parameters(task_characteristics_yaml);

        ResourceOptResult opt_res_pa_and_tl;
        // auto start_time_for_optimization = CurrentTimeInProfiler;
        exe_profiler_.start();
        exe_profiler_.end();
        
        exe_profiler_.start();
        if (scheduler_ == "optimizerBF") {
            opt_res_pa_and_tl =
                EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters);
            TimerType finish_time = CurrentTimeInProfiler;
            time_taken = GetTimeTaken(start_time, finish_time);
        } else if (scheduler_ == "optimizerIncremental") {
            if (incremental_optimizer_w_TL_.IfInitialized()) {
                incremental_optimizer_w_TL_.OptimizeIncre_w_TL(
                    dag_tasks, GlobalVariables::
                                   Layer_Node_During_Incremental_Optimization);

                // incremental_optimizer_w_TL_.OptimizeFromScratch_w_TL(
                //    GlobalVariables::
                //         Layer_Node_During_Incremental_Optimization);
            } else {
                incremental_optimizer_w_TL_ =
                    OptimizePA_Incre_with_TimeLimits(dag_tasks, sp_parameters);

                incremental_optimizer_w_TL_.OptimizeFromScratch_w_TL(
                    GlobalVariables::
                        Layer_Node_During_Incremental_Optimization);
            }
            opt_res_pa_and_tl = incremental_optimizer_w_TL_.CollectResults();
            TimerType finish_time = CurrentTimeInProfiler;
            time_taken = GetTimeTaken(start_time, finish_time);
        } else {
            std::cerr << "Unknown scheduler: " << scheduler_ << "\n";
            return;
        }
        exe_profiler_.end();
        double time_for_optimization = exe_profiler_.get_exe_time();

        PriorityVec pa_opt = opt_res_pa_and_tl.priority_vec;
        WritePriorityAssignments(priority_yaml_output_path, dag_tasks.tasks,
                                 pa_opt, time_taken);
        WriteTimeLimitToYamlOSM(
            opt_res_pa_and_tl.id2time_limit[0]);  // only write TSP's time limit
        std::cout << "Time taken for scheduler to run one time: " << time_taken
                  << "\n";

        start_time = CurrentTimeInProfiler;
        // update priorities from the scheduler to local config yaml
        UpdatePriorityAssignments(local_config_yaml, priority_yaml_output_path);
        UpdateProcessorAssignmentsFromYamlFile(local_config_yaml,
                                               task_characteristics_yaml);

        auto finish_time = CurrentTimeInProfiler;
        std::cout << "Time to record priority assignments to disk: "
                  << GetTimeTaken(start_time, finish_time) << "\n";
        // apply the new priority assignments
        start_time = CurrentTimeInProfiler;
        rt_manager_.setCPUAffinityAndPriority(local_config_yaml);
        finish_time = CurrentTimeInProfiler;
        double time_for_OS_to_change_priority =
            GetTimeTaken(start_time, finish_time);
        std::cout << "Time to change priority assignments in OS: "
                  << time_for_OS_to_change_priority << "\n";
        recorder_.write_receive_time(getCurrentTimeStamp(), release_index);
        recorder_.write_execution_time(
            time_for_optimization + time_for_OS_to_change_priority,
            release_index);
    }
}

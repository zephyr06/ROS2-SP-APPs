#include "sources/applications/real_time_manager/include/real_time_manager/scheduler_wrapper.h"

// function arguments msg_cnt not used for now
void SchedulerApp::run(int) override {
    // no execution at the first instance
    if (cnt_++ == 0) return;
    TimerType start_time = CurrentTimeInProfiler;
    std::filesystem::path current_file_path =
        std::filesystem::canonical(__FILE__);
    std::filesystem::path package_directory =
        current_file_path.parent_path()
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

    if (scheduler_ == "CFS") {
        std::string local_fixed_cpu_and_priority_yaml_CFS =
            package_directory.string() +
            "/configs/local_fixed_cpu_and_priority_CFS.yaml";
        UpdateProcessorAssignmentsFromYamlFile(
            local_fixed_cpu_and_priority_yaml_CFS, task_characteristics_yaml);
        rt_manager_.setCPUAffinityAndPriority(
            local_fixed_cpu_and_priority_yaml_CFS);
    } else if (scheduler_ == "RM") {
        std::string local_fixed_cpu_and_priority_yaml_RM =
            package_directory.string() +
            "/configs/local_fixed_cpu_and_priority_RM.yaml";
        UpdateProcessorAssignmentsFromYamlFile(
            local_fixed_cpu_and_priority_yaml_RM, task_characteristics_yaml);
        rt_manager_.setCPUAffinityAndPriority(
            local_fixed_cpu_and_priority_yaml_RM);
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
        // apply the new priority assignments
        rt_manager_.setCPUAffinityAndPriority(local_config_yaml);
        auto finish_time = CurrentTimeInProfiler;
        std::cout << "Time to change priority assignments in OS: "
                  << GetTimeTaken(start_time, finish_time) << "\n";
    }
}

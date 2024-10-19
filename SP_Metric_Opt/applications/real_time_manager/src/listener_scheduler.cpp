#include "sources/applications/real_time_manager/include/real_time_manager/scheduler_wrapper.h"

int main(int argc, char *argv[]) {
    SP_OPT_PA::DAG_Model dag_tasks = SP_OPT_PA::ReadDAG_Tasks(
        GlobalVariables::PROJECT_PATH +
        "../all_time_records/task_characteristics.yaml");
    if (argc < 3)
        std::cerr << "Wrong arg format: example usage ./scheduler_runner "
                     "optimizerBF 30000 The last argument is period of the "
                     "scheduler in miliseconds\n";
    int period = std::stoi(argv[2]);
    int count = dag_tasks.tasks[0].total_running_time / period;
    SchedulerApp app(argc, argv);
    PeriodicReleaser<SchedulerApp> releaser(period, count, app);
    releaser.release();
    return 0;
}
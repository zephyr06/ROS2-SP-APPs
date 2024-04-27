
#include <cassert>

#include "MPCUsage.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/Parameters.h"
#include "sources/UtilsForROS2/Publisher.h"
class MPCApp : public AppBase {
   public:
    MPCApp() : AppBase("mpc") {}
    void run(int) override { mpc_main(); }
};

int main(int argc, char* argv[]) {
    SP_OPT_PA::DAG_Model dag_tasks = SP_OPT_PA::ReadDAG_Tasks(
        GlobalVariables::PROJECT_PATH +
        "../all_time_records/task_characteristics.yaml");

    int index = 1;

    assert(dag_tasks.tasks[index].name == "MPC");
    double period = dag_tasks.tasks[index].period;
    int count = dag_tasks.tasks[index].total_running_time / period;
    MPCApp app;
    PeriodicReleaser<MPCApp> releaser(period, count, app);
    releaser.release();
    return 0;
}
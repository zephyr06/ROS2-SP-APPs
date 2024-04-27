#include <cassert>

#include "TSPSolver.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/Parameters.h"
#include "sources/UtilsForROS2/Publisher.h"

class TSPApp : public AppBase {
   public:
    TSPApp() : AppBase("tsp") {}
    void run(int) override { callTSP(); }
};

int main(int argc, char* argv[]) {
    SP_OPT_PA::DAG_Model dag_tasks = SP_OPT_PA::ReadDAG_Tasks(
        GlobalVariables::PROJECT_PATH +
        "../all_time_records/task_characteristics.yaml");

    int index = 0;

    assert(dag_tasks.tasks[index].name == "TSP");
    double period = dag_tasks.tasks[index].period;
    int count = dag_tasks.tasks[index].total_running_time / period;
    TSPApp app;
    PeriodicReleaser<TSPApp> releaser(period, count, app);
    releaser.release();
    return 0;
}
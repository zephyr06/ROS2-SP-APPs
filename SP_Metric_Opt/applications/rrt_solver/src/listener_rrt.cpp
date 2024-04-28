
#include "rrt_solver/rrt_solver.h"
#include <cassert>

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/Parameters.h"
#include "sources/UtilsForROS2/Publisher.h"
class RRTApp : public AppBase {
 public:
  RRTApp() : AppBase("RRT") {}
  void run(int msg_cnt) override { rrtsolver_.solveWithoutUI(); }
  RRTSolver rrtsolver_;
};

int main(int argc, char* argv[]) {
    SP_OPT_PA::DAG_Model dag_tasks = SP_OPT_PA::ReadDAG_Tasks(
        GlobalVariables::PROJECT_PATH +
        "../all_time_records/task_characteristics.yaml");

    int index = 2;

    assert(dag_tasks.tasks[index].name == "RRT");
    double period = dag_tasks.tasks[index].period;
    int count = dag_tasks.tasks[index].total_running_time / period;
    RRTApp app;
    PeriodicReleaser<RRTApp> releaser(period, count, app);
    releaser.release();
    return 0;
}
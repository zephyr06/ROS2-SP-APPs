
#include "dynaslam/dynaslam_wrapper.h"

#include <cassert>

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/Parameters.h"
#include "sources/UtilsForROS2/Publisher.h"

class SLAMApp : public AppBase {
 public:
  SLAMApp() : AppBase("SLAM") { slam_wrapper_.init(); }
  void run(int msg_cnt) override { slam_wrapper_.next(msg_cnt); }
  DynaSLAMWrapperForROS2 slam_wrapper_;
};



int main(int argc, char* argv[]) {
    SP_OPT_PA::DAG_Model dag_tasks = SP_OPT_PA::ReadDAG_Tasks(
        GlobalVariables::PROJECT_PATH +
        "../all_time_records/task_characteristics.yaml");

    int index = 3;

    assert(dag_tasks.tasks[index].name == "SLAM");
    double period = dag_tasks.tasks[index].period;
    int count = dag_tasks.tasks[index].total_running_time / period;
    SLAMApp app;
    PeriodicReleaser<SLAMApp> releaser(period, count, app);
    releaser.release();
    return 0;
}
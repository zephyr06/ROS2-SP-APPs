#include <cassert>

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/Parameters.h"
#include "sources/UtilsForROS2/Publisher.h"
#include "tsp_osm_main_utils.h"

class TSPApp : public AppBase {
   public:
    TSPApp() : AppBase("TSP") { input_data_ = load_tsp_input(); }
    void run(int) override {
        std::cout << "Following cout related to TSP will be disabled\n";
        std::cout.setstate(std::ios_base::failbit);
        run_tsp(input_data_);
        // run_tsp_osm();
    }
    InputDataForTSP input_data_;
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
    exit(0);
    return 0;
}
#include <cstdlib>

#include "real_time_manager/execution_time_estimator.h"
#include "real_time_manager/real_time_manager.h"
#include "real_time_manager/update_priority_assignment.h"
#include "sources/Optimization/OptimizeSP_Incre.h"
#include "sources/Optimization/OptimizeSP_TL_Incre.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/argparse.hpp"
#include "sources/Utils/profilier.h"
#include "sources/Utils/readwrite.h"
#include "sources/UtilsForROS2/Publisher.h"

class SchedulerApp : public AppBase {
   public:
    SchedulerApp() : AppBase("SCHEDULER") {}
    SchedulerApp(int argc, char *argv[]) : AppBase("SCHEDULER") {
        // supported scheduler are: CFS, RM, optimizerBF, optimizerIncremental
        // scheduler_ = "optimizerBF";
        if (argc >= 2) {
            scheduler_ = argv[1];
        } else {
            std::cout
                << "Must provide a valid and supported name of scheduler!\n";
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
    void run(int) override;

    ExecutionTimeEstimator et_estimator_;
    RealTimeManager rt_manager_;
    int cnt_ = 0;
    std::string scheduler_;
    // SP_OPT_PA::OptimizePA_Incre incremental_optimizer_;
    SP_OPT_PA::OptimizePA_Incre_with_TimeLimits incremental_optimizer_w_TL_;
};

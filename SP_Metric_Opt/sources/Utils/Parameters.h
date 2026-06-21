#pragma once
#include <Eigen/Core>
#include <iostream>

#include "sources/Utils/testMy.h"
// All the global variables should be const

namespace GlobalVariables {

extern const std::string PROJECT_PATH;

// optimization settings
extern int debugMode;
extern int TIME_LIMIT;
extern int printRTA;

extern int Granularity;
extern int Layer_Node_During_Incremental_Optimization;
extern double Dist_compress_threshold;
extern bool disable_time_limit_opt;
extern bool use_wcet_execution_time;
}  // namespace GlobalVariables
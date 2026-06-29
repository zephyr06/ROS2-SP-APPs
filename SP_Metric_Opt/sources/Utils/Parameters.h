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
extern bool disable_time_limit_opt;
extern bool use_wcet_execution_time;
extern int TimeLimitSearchRadiusIncr;

// simulation export controls
extern int EXPORT_DETAIL_LEVEL;               // 0=sp_metrics_only, 1=+miss_rate_per_task, 2=+task_aggregate, 3=+full_job_traces
extern int METRIC_SAMPLE_INTERVAL_SECONDS;    // 0=un-sampled (all intervals), N>0=sample every N seconds
}
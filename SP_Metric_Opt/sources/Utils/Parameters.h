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
// Reopt TL-descent start: true = start from the carried adopted TL
// (ReconstructTimeLimitVecFromResOpt, the P0.5 symmetric fix); false (default)
// = Gaussian-mean TL (InitializeTimeLimitsFromETConfig, current behavior).
// Mode-string-driven only (INCR_P<n>_ADOPTED arms), never loaded from YAML.
extern bool ReoptStartFromAdoptedTL;
// P24 periodic reoptimization: every ReoptimizationPeriod-th interval, the TL
// search re-runs from scratch (compare-and-keep) instead of warm-starting from
// the incumbent, bounding priority drift over a long sim.
// ReoptimizationPeriod == 0 disables it (today's pure cache+scratch baseline).
extern int ReoptimizationPeriod;

// Trial-and-error walk patience: how many consecutive non-improving SP evals
// the outward TL walk tolerates before stopping in one direction. Incremental
// path uses IncrementalTimeLimitSearchPatience (warm-started PA search →
// effectively unimodal → 0 is safe); reopt path uses
// ReoptimizationTimeLimitSearchPatience (from-scratch PA search → can be
// non-unimodal at high util → 1 tolerates a single dip).
extern int IncrementalTimeLimitSearchPatience;
extern int ReoptimizationTimeLimitSearchPatience;

// simulation export controls
extern int EXPORT_DETAIL_LEVEL;               // 0=sp_metrics_only, 1=+miss_rate_per_task, 2=+task_aggregate, 3=+full_job_traces
extern int METRIC_SAMPLE_INTERVAL_SECONDS;    // 0=un-sampled (all intervals), N>0=sample every N seconds
}
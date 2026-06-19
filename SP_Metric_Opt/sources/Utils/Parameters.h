#pragma once
#include <Eigen/Core>
#include <iostream>

#include "sources/Utils/testMy.h"
// All the global variables should be const

namespace GlobalVariables {

#if defined(RYAN_HE_CHANGE_DEBUG)
// 1 is original,
// RYAN_HE: using bits to enable print for each module
#define DBG_PRT_MSK_MAIN 2
#define DBG_PRT_MSK_SIMULATION 4
#define DBG_PRT_MSK_RUNQUEUE 8
#define DBG_PRT_MSK_OptimizeSP_BF 16
#define DBG_PRT_MSK_OptimizeSP_Base 32
#define DBG_PRT_MSK_OptimizeSP_Incre 64
#define DBG_PRT_MSK_RTA 128
#define DBG_PRT_MSK_OptimizeSP_TL_BF 256
#define DBG_PRT_MSK_SP_Metric 512
#define DBG_PRT_MSK_TSK 1024
#define DBG_PRT_MSK_DBG_DDL_SP 0x8000 // 32768, debug for deadline miss and SP calcualation trace

#define DBG_PRT_MSK_ALL \
    (DBG_PRT_MSK_MAIN | DBG_PRT_MSK_OptimizeSP_TL_BF | DBG_PRT_MSK_SP_Metric | \
     DBG_PRT_MSK_OptimizeSP_BF | DBG_PRT_MSK_OptimizeSP_Base | \
     DBG_PRT_MSK_OptimizeSP_Incre | DBG_PRT_MSK_RTA | DBG_PRT_MSK_SIMULATION | \
     DBG_PRT_MSK_RUNQUEUE | DBG_PRT_MSK_TSK)

#endif

extern const std::string PROJECT_PATH;

// optimization settings
extern int debugMode;
extern int TIME_LIMIT;
extern int printRTA;

extern int Granularity;
extern int Layer_Node_During_Incremental_Optimization;
extern double Dist_compress_threshold;
}  // namespace GlobalVariables
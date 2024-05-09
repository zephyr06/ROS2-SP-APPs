#include "real_time_manager/execution_time_estimator.h"

int main(){
    ExecutionTimeEstimator et_estimator;
    
    et_estimator.updateTaskExecutionTimeDistributions(50,10);
}

#pragma once
#include <ctime>
#include <iostream>

class ExecutionTimeProfiler {
 public:
  ExecutionTimeProfiler() {}

  void start() {
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start_) == -1) {
      perror("clock gettime");
    }
  }

  void end() {
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end_) == -1) {
      perror("clock gettime");
    }
  }

  // return CPU time excluding preemption
  double get_exe_time() {
    return (end_.tv_sec - start_.tv_sec) +
           (end_.tv_nsec - start_.tv_nsec) / double(1e9);
  }

  // data members

  struct timespec start_, end_;
};
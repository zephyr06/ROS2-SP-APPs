#pragma once

#include <unistd.h>  // only effective with Linux, "windows.h" works with Windows

#include <chrono>
#include <ctime>
#include <iostream>
#include <ratio>
#include <unordered_map>
#include <vector>

// #include "sources/Utils/Parameters.h"
#include "sources/Utils/colormod.h"
#include "sources/Utils/testMy.h"

#define PROFILE_CODE

#define CurrentTimeInProfiler std::chrono::high_resolution_clock::now()

typedef std::chrono::time_point<std::chrono::high_resolution_clock> TimerType;
// #define BeginTimerAppInProfiler BeginTimer(__FUNCTION__);
// #define EndTimerAppInProfiler EndTimer(__FUNCTION__);
extern std::mutex mtx_profiler;
struct ProfilerData {
    TimerType begin;
    TimerType end;
    double accum;
    int call_time;
    ProfilerData()
        : begin(std::chrono::high_resolution_clock::now()),
          end(std::chrono::high_resolution_clock::now()),
          accum(0),
          call_time(0) {}
    inline void UpdateAccum() {
        auto duration =
            std::chrono::duration_cast<std::chrono::microseconds>(end - begin);
        accum += double(duration.count()) / 1e6;
        call_time++;
    }
};
extern std::unordered_map<std::string, ProfilerData> profilerMap;

void BeginTimer(std::string funcName);

void EndTimer(std::string funcName, bool print = false);

struct TimerDataProfiler {
    std::string name;
    double accum;
    int call_time;
};
bool compareProfiler(TimerDataProfiler a, TimerDataProfiler b);
void PrintTimer();

class TimerFunc {
   public:
    TimerFunc() : begin(std::chrono::high_resolution_clock::now()) {}
    ~TimerFunc() {
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - begin);
        std::cout << "Accumulated time during the execution is: "
                  << double(duration.count()) / 1e6 << " seconds!\n";
    }

   private:
    typedef std::chrono::time_point<std::chrono::high_resolution_clock>
        TimerType;
    TimerType begin;
};

inline double GetTimeTaken(TimerType start, TimerType stop) {
    return double(std::chrono::duration_cast<std::chrono::microseconds>(stop -
                                                                        start)
                      .count()) /
           1e6;
}

inline void print_time_in_us() {
    // Get current time point
    auto now = std::chrono::system_clock::now();

    // Convert to time_t
    std::time_t time_t_now = std::chrono::system_clock::to_time_t(now);

    // Convert to local time
    std::tm* local_tm = std::localtime(&time_t_now);

    // Print in desired format
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", local_tm);

    // Get microseconds
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
                            now.time_since_epoch())
                            .count() %
                        1000000;

    std::cout << buffer << "." << std::setw(6) << std::setfill('0')
              << microseconds << std::endl;
}
#pragma once

#include <unistd.h>  // only effective with Linux, "windows.h" works with Windows

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <ratio>
#include <string>
#define CurrentTimeInProfiler std::chrono::high_resolution_clock::now()

typedef std::chrono::time_point<std::chrono::high_resolution_clock> TimerType;

// return in microseconds
inline double getDuration(TimerType start_time, TimerType end_time) {
    return std::chrono::duration_cast<std::chrono::microseconds>(end_time -
                                                                 start_time)
               .count() /
           1e6;
}
inline void write_current_time_to_file(const std::string& filename,
                                       double current_time,
                                       const std::string& message = "") {
    std::ofstream file;
    file.open(filename, std::ios::app);
    // double time_now =
    // std::chrono::duration_cast<std::chrono::microseconds>(time_to_write -
    // start_time).count()/1e6;

    file << message << "::" << std::setprecision(20) << current_time
         << std::endl;
    file.close();
}

inline std::string getTimeRecordFolder() {
    std::filesystem::path current_file_path =
        std::filesystem::canonical(__FILE__);
    std::filesystem::path current_file_directory =
        current_file_path
            .parent_path()   // SP_Metric_Opt/sources/UtilsForROS2
            .parent_path()   // SP_Metric_Opt/sources
            .parent_path()   // SP_Metric_Opt
            .parent_path();  // ROS2-SP-APPS
    std::string time_record_file_path =
        current_file_directory.string() + "/all_time_records/";
    std::cout << time_record_file_path << "\n";
    return time_record_file_path;
}

double getCurrentTimeStamp() {
    using namespace std::chrono;
    return duration_cast<microseconds>(system_clock::now().time_since_epoch())
               .count() /
           1000000.0;
}

inline std::string getTopicName(std::string& app_name) {
    return "topic_" + app_name;
}
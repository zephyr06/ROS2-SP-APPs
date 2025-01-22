#pragma once
#include "sources/UtilsForROS2/execution_time_profiler.h"
#include "sources/UtilsForROS2/profile_and_record_time.h"

struct Recorder {
    Recorder(){};
    Recorder(std::string app_name)
        : app_name_(app_name),
          publisher_file_path_(getTimeRecordFolder() + app_name_ +
                               "_publisher.txt"),
          subscriber_file_path_(getTimeRecordFolder() + app_name_ +
                                "_subscriber.txt"),
          execution_time_file_path_(getTimeRecordFolder() + app_name_ +
                                    "_execution_time.txt") {}

    inline void write_initial_publish_time() {
        publisher_start_time_ = getCurrentTimeStamp();
        write_current_time_to_file(publisher_file_path_, publisher_start_time_,
                                   "Start time of publisher: ");
    }
    inline void write_initial_receive_time() {
        subscriber_start_time_ = getCurrentTimeStamp();
        write_current_time_to_file(subscriber_file_path_,
                                   subscriber_start_time_,
                                   "Start time of subscriber: ");
    }

    inline void write_execution_time(double ext_time, int index) {
        write_current_time_to_file(execution_time_file_path_, ext_time,
                                   "Execution time: " + app_name_ +
                                       " message:: " + std::to_string(index));
    }
    inline void write_publish_time(double time, int index) {
        write_current_time_to_file(publisher_file_path_,
                                   time - publisher_start_time_,
                                   "Publishing time: " + app_name_ +
                                       " message:: " + std::to_string(index));
    }
    inline void write_receive_time(double time, int index) {
        write_current_time_to_file(subscriber_file_path_,
                                   time - subscriber_start_time_,
                                   "Receiving time: " + app_name_ +
                                       " message:: " + std::to_string(index));
    }

    std::string app_name_;
    std::string publisher_file_path_;
    std::string subscriber_file_path_;
    std::string execution_time_file_path_;
    double publisher_start_time_;
    double subscriber_start_time_;
};
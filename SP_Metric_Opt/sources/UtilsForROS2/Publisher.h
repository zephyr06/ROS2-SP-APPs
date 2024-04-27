#pragma once

#include <unistd.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time.hpp>
#include <iomanip>
#include <iostream>

#include "sources/Utils/profilier.h"
#include "sources/UtilsForROS2/AppBase.h"
#include "sources/UtilsForROS2/execution_time_profiler.h"
#include "sources/UtilsForROS2/profile_and_record_time.h"

struct Recorder {
    Recorder(std::string app_name)
        : app_name_(app_name),
          publisher_file_path_(getTimeRecordFolder() + app_name_ +
                               "_publisher.txt"),
          listener_file_path_(getTimeRecordFolder() + app_name_ +
                              "_subscriber.txt"),
          execution_time_file_path_(getTimeRecordFolder() + app_name_ +
                                    "_execution_time.txt") {}
    inline void write_execution_time(double ext_time, int index) {
        write_current_time_to_file(execution_time_file_path_, ext_time,
                                   "Execution time: " + app_name_ +
                                       " message:: " + std::to_string(index));
    }
    inline void write_publish_time(double time, int index) {
        write_current_time_to_file(publisher_file_path_, time,
                                   "Publishing time: " + app_name_ +
                                       " message:: " + std::to_string(index));
    }
    inline void write_receive_time(double time, int index) {
        write_current_time_to_file(listener_file_path_, time,
                                   "Receiving time: " + app_name_ +
                                       " message:: " + std::to_string(index));
    }
    std::string app_name_;
    std::string publisher_file_path_;
    std::string listener_file_path_;
    std::string execution_time_file_path_;
};

template <typename AppBase>
class PeriodicReleaser {
   public:
    PeriodicReleaser(int period_ms, int release_total, const AppBase& app)
        : app_(app),
          period_ms_(period_ms),
          release_total_(release_total),
          release_index_(0),
          recorder_(app_.app_name_) {}

    void run_app_and_record_time() {
        recorder_.write_publish_time(getCurrentTimeStamp(), release_index_);
        exe_profiler_.start();
        app_.run(0);
        exe_profiler_.end();
        recorder_.write_receive_time(getCurrentTimeStamp(), release_index_);
        recorder_.write_execution_time(exe_profiler_.get_exe_time(),
                                       release_index_);
    }

    void caller(const boost::system::error_code&,
                boost::asio::deadline_timer& t) {
        if (release_total_ == release_index_)
            return;
        run_app_and_record_time();
        release_index_++;
        t.expires_at(t.expires_at() +
                     boost::posix_time::milliseconds(period_ms_));
        // if (++count < 100)
        t.async_wait(boost::bind(&PeriodicReleaser<AppBase>::caller, this,
                                 boost::asio::placeholders::error,
                                 boost::ref(t)));
    }

    void release() {
        boost::asio::io_service io;
        boost::asio::deadline_timer t(
            io, boost::posix_time::milliseconds(period_ms_));
        t.async_wait(boost::bind(&PeriodicReleaser<AppBase>::caller, this,
                                 boost::asio::placeholders::error,
                                 boost::ref(t)));
        io.run();
    }

    AppBase app_;
    int period_ms_;
    int release_total_;
    int release_index_;
    Recorder recorder_;
    ExecutionTimeProfiler exe_profiler_;
};

void busySpinForSeconds(int ms) {
    auto startTime = std::chrono::high_resolution_clock::now();
    auto endTime = startTime + std::chrono::milliseconds(ms);

    while (std::chrono::high_resolution_clock::now() < endTime) {
        // Busy spin
    }
}

class AppTest : public AppBase {
   public:
    AppTest() : AppBase("AppTest") {}
    void run(int) override { std::cout << "Run one time!\n"; }
};
#pragma once
#include <memory>

#include "execution_time_profiler.h"
#include "profiler.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
class AppBase {
 public:
  AppBase(std::string app_name) : app_name_(app_name) {}
  virtual void run() { ; }
  // data member
  std::string app_name_;
};

template <typename AppBase>
class SubscriberAppBase : public rclcpp::Node {
 public:
  SubscriberAppBase()
      : Node("minimal_subscriber"),
        start_time_(CurrentTimeInProfiler),
        target_profile_data_file_path_(getTimeRecordFolder() + app_.app_name_ +
                                       "_subscriber.txt"),
        execution_time_profile_data_file_path_(
            getTimeRecordFolder() + app_.app_name_ + "_execution_time.txt") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        getTopicName(app_.app_name_), 1,
        std::bind(&SubscriberAppBase::topic_callback, this, _1));

    write_current_time_to_file(target_profile_data_file_path_,
                               getCurrentTimeStamp(),
                               "Start time of subscriber: ");
  }

 private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    // callTSP();
    exe_profiler_.start();
    app_.run();
    exe_profiler_.end();
    double current_time = getDuration(start_time_, CurrentTimeInProfiler);
    double app_exe_time = exe_profiler_.get_exe_time();  // actual CPU time
    std::string receive_message_subscriber =
        "Receiving " + app_.app_name_ + " message:" + msg->data;
    write_current_time_to_file(target_profile_data_file_path_, current_time,
                               receive_message_subscriber);
    std::string receive_message_execution =
        "Execution time " + app_.app_name_ + " message:" + msg->data;
    write_current_time_to_file(execution_time_profile_data_file_path_,
                               app_exe_time, receive_message_execution);
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  // data related to profiler
  AppBase app_;
  TimerType start_time_;
  std::string target_profile_data_file_path_;
  ExecutionTimeProfiler exe_profiler_;
  std::string execution_time_profile_data_file_path_;
};

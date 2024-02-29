#pragma once
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "profiler.h"

using std::placeholders::_1;
class AppBase
{
public:
  AppBase(std::string app_name):app_name_(app_name){}
  virtual void run(){;}
  // data member
  std::string app_name_;
};


template<typename AppBase>
class SubscriberAppBase : public rclcpp::Node
{
public:
  SubscriberAppBase()
  : Node("minimal_subscriber"),  start_time_(CurrentTimeInProfiler), target_profile_data_file_path_(getTimeRecordFolder()+app_.app_name_+"_subscriber.txt")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      getTopicName(app_.app_name_), 1, std::bind(&SubscriberAppBase::topic_callback, this, _1));
    write_current_time_to_file(target_profile_data_file_path_, getCurrentTimeStamp(), "Start time of subscriber: ");
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) 
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    // callTSP();
    app_.run();
    double current_time=getDuration(start_time_, CurrentTimeInProfiler);
    std::string receive_message ="Receiving "+app_.app_name_+" message:"+msg->data;
    write_current_time_to_file(target_profile_data_file_path_, current_time, receive_message);
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  // data related to profiler
  AppBase app_;
  TimerType start_time_;
  std::string target_profile_data_file_path_;
};

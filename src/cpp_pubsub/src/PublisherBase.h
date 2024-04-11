#pragma once

#include <chrono>
// #include <functional>
#include <memory>
#include <string>

#include <filesystem>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "profiler.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(std::string& app_name, std::chrono::milliseconds period)
  : app_name_(app_name), 
    period_(period),
    Node("minimal_publisher"), count_(0), 
   start_time_(CurrentTimeInProfiler), 
   target_profile_data_file_path_(getTimeRecordFolder()+"publisher"+"_"+app_name_+".txt")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>(getTopicName(app_name_), 1);
    timer_ = this->create_wall_timer(
      period_, std::bind(&MinimalPublisher::timer_callback, this));
    write_current_time_to_file(target_profile_data_file_path_, getCurrentTimeStamp(), "Start time of publisher: ");
  }


private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = app_name_ + " " +std::to_string(count_++);
    // RCLCPP_INFO(this->get_logger(), "Publishing TSP: '%s'", message.data.c_str(start_time_, CurrentTimeInProfiler));
    
    double current_time=getDuration(start_time_, CurrentTimeInProfiler);
    write_current_time_to_file(target_profile_data_file_path_, current_time, "Publishing " + app_name_+" "+message.data );
    // RCLCPP_INFO(this->get_logger(), "Publishing " + app_name_ + ": %f", current_time);
    publisher_->publish(message);
  }

  // data members
  std::string app_name_;
  // rclcpp::Time period_;
  std::chrono::milliseconds period_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;

  // data related to profiler
  TimerType start_time_;
  std::string target_profile_data_file_path_;
};

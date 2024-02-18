// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "TSPSolver.h"
#include "profiler.h"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber"),  start_time_(CurrentTimeInProfiler), target_profile_data_file_path_(getTimeRecordFolder()+"tsp_subscriber.txt")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic_tsp", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    callTSP();
    double current_time=getDuration(start_time_, CurrentTimeInProfiler);
    std::string receive_message ="Receiving TSP message:"+msg.data;
    write_current_time_to_file(target_profile_data_file_path_, current_time, receive_message);
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;


  // data related to profiler
  TimerType start_time_;
  std::string target_profile_data_file_path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

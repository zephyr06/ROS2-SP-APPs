#pragma once
#include "listener_base.h"
#include "TSPSolver.h"
class TSPApp : public AppBase
{
public:
  TSPApp():AppBase("tsp"){}
  void run() override
  {
    callTSP();
  }

};
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberAppBase<TSPApp>>());
  rclcpp::shutdown();
  return 0;
}
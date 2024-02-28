#pragma once
#include "listener_base.h"
#include "MPCUsage.h"
class TSPApp : public AppBase
{
public:
  TSPApp():AppBase("mpc"){}
  void run() override
  {
    mpc_main();
  }

};
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberAppBase<TSPApp>>());
  rclcpp::shutdown();
  return 0;
}
#pragma once
#include "listener_base.h"
#include "rrt_demo_withoutGUI.cpp"
class RRTApp : public AppBase
{
public:
    RRTApp():AppBase("rrt"){}
    void run() override
    {
        rrtsolver_.solveWithoutUI();
    }
    RRTSolver rrtsolver_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberAppBase<RRTApp>>());
  rclcpp::shutdown();
  return 0;
}
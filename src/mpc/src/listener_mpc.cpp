
#include "MPCUsage.h"
#include "listener_base.h"
class MPCApp : public AppBase {
 public:
  MPCApp() : AppBase("mpc") {}
  void run() override { mpc_main(); }
};
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberAppBase<MPCApp>>());
  rclcpp::shutdown();
  return 0;
}
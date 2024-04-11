#include "TSPSolver.h"
#include "listener_base.h"
class TSPApp : public AppBase {
 public:
  TSPApp() : AppBase("tsp") {}
  void run(int msg_cnt) override { callTSP(); }
};
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberAppBase<TSPApp>>());
  rclcpp::shutdown();
  return 0;
}
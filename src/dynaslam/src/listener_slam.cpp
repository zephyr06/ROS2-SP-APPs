
#include "dynaslam/dynaslam_wrapper.h"
#include "listener_base.h"
class SLAMApp : public AppBase {
 public:
  SLAMApp() : AppBase("slam") { slam_wrapper_.init(); }
  void run(int msg_cnt) override { slam_wrapper_.next(msg_cnt); }
  DynaSLAMWrapperForROS2 slam_wrapper_;
};
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberAppBase<SLAMApp>>());
  rclcpp::shutdown();
  return 0;
}
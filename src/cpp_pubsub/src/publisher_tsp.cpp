#include "PublisherBase.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::string app_name="tsp";
  rclcpp::spin(std::make_shared<MinimalPublisher>(app_name, std::chrono::milliseconds(500)));
  rclcpp::shutdown();
  return 0;
}

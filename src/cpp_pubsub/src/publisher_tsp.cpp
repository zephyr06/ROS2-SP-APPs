#include "PublisherBase.h"

int main(int argc, char * argv[])
{
  if(argc !=3){
    std::cerr<<"Usage: "<<" application_name period (miliseconds)\n";
  }
  std::string app_name=argv[1];
  std::chrono::milliseconds period(std::atoi(argv[2]));
  std::cout<<"Publish a topic whose name and periods are: "+app_name+", "+std::to_string(std::atoi(argv[2]))+"ms\n";
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>(app_name, period ));
  rclcpp::shutdown();
  return 0;
}

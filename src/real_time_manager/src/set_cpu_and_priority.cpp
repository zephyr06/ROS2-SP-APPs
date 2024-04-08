
#include "real_time_manager/real_time_manager.h"

int main()
{

    RealTimeManager rt_manager;

    // default will read yaml file inside the "getTimeRecordFolder() + "cpu_and_priority.yaml"""
    // e.g. the next comment out line
    // rt_manager.setCPUAffinityAndPriority(); 
    
    std::filesystem::path current_file_path = std::filesystem::canonical(__FILE__);
    std::filesystem::path package_directory = current_file_path.parent_path().parent_path();
    std::string local_config_yaml = package_directory.string() + "/configs/local_cpu_and_priority.yaml";

    // std::cout<< local_config_yaml;
    rt_manager.setCPUAffinityAndPriority(local_config_yaml); // or we can provide a local file for manual test
}

#include "real_time_manager.h"

int main()
{

    RealTimeManager rt_manager;

    // default will read yaml file inside the "getTimeRecordFolder() + "cpu_and_priority.yaml"""
    // e.g. the next comment out line
    // rt_manager.setCPUAffinityAndPriority(); 
    
    std::filesystem::path current_file_path = std::filesystem::canonical(__FILE__);
    std::filesystem::path current_file_directory = current_file_path.parent_path();
    std::string local_file = current_file_directory.string() + "/configs/local_cpu_and_priority.yaml";
    rt_manager.setCPUAffinityAndPriority(local_file); // or we can provide a local file for manual test
}
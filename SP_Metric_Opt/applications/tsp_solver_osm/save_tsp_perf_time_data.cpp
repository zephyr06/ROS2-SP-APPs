#include <fstream>
#include <iostream>
#include <iomanip>
#include <ctime>

#include "tsp_osm_main_utils.h"
std::string current_time2str(){

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    return oss.str();
}
int main() {
    // Define the range of time limits
    double min_time_limit = 0.05;   // Minimum time limit
    double max_time_limit = 2.5;      // Maximum time limit
    double time_limit_step = 0.05;  // Step size for time limits

    // Open the output file
    std::string tsp_osm_project_path =
        GlobalVariables::PROJECT_PATH + "applications/tsp_solver_osm/";
    std::ofstream outfile(tsp_osm_project_path + "performance_data_" + current_time2str()+".txt");
    if (!outfile) {
        std::cerr << "Failed to open the output file!" << std::endl;
        return 1;
    }
    InputDataForTSP input_data = load_tsp_input();
    // Iterate over the time limits and save the performance data
    for (double time_limit = min_time_limit; time_limit <= max_time_limit;
         time_limit += time_limit_step) {
        // Run the TSP solver with the current time limit
        double performance = run_tsp(input_data, time_limit);

        // Save the time limit and performance into the output file
        outfile << time_limit << ", " << performance << std::endl;
    }

    // Close the output file
    outfile.close();

    return 0;
}
#include "sources/RTDA/ImplicitCommunication/SimulationOrchestrator.h"
#include <iostream>
#include <fstream>
#include <numeric>

using namespace SP_OPT_PA;

int main(int argc, char** argv) {
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0] << " <input_folder> <output_folder> <mode> <duration_ms>\n";
        return 1;
    }

    std::string input_folder = argv[1];
    std::string output_folder = argv[2];
    std::string mode = argv[3];
    LLint duration = std::stoll(argv[4]);

    std::cout << "Running Orchestrator: Input=" << input_folder
              << ", Output=" << output_folder
              << ", Mode=" << mode
              << ", Duration=" << duration << " ms\n";

    FixedTaskPrioritySchedulingOrchestrator orchestrator(input_folder, output_folder, mode, duration);
    orchestrator.RunSimulation();

    const auto& metrics = orchestrator.GetIntervalSPMetrics();
    if (!metrics.empty()) {
        double sum = std::accumulate(metrics.begin(), metrics.end(), 0.0);
        double avg = sum / metrics.size();
        std::cout << "Average SP Metric: " << avg << "\n";

        // Store it in a file as requested by the user
        std::string summary_path = output_folder + "/" + mode + "/sp_metrics_summary.txt";
        std::ofstream summary_file(summary_path);
        if (summary_file.is_open()) {
            summary_file << avg << "\n";
            summary_file.close();
            std::cout << "Saved average SP to: " << summary_path << "\n";
        } else {
            std::cerr << "Failed to write summary to: " << summary_path << "\n";
        }
    } else {
        std::cout << "No intervals simulated, no SP metrics calculated.\n";
    }

    return 0;
}

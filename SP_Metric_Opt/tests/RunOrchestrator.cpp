#include "sources/RTDA/ImplicitCommunication/SimulationOrchestrator.h"
#include <chrono>
#include <iostream>
#include <fstream>
#include <numeric>

using namespace SP_OPT_PA;

int main(int argc, char** argv) {
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0]
                  << " <input_folder> <output_folder> <mode> <duration_ms>"
                  << " [export_level] [sample_interval_sec]\n";
        std::cerr << "Modes: RM, BF, INCR, INCR_NO_TL, INCR_WCET, INCR_SCRATCH, "
                  << "RM_FAST, RM_SLOW\n";
        std::cerr << "  export_level: 0=sp only, 1=+task miss rate, "
                  << "2=+task aggregate, 3=full traces (def="
                  << GlobalVariables::EXPORT_DETAIL_LEVEL << ")\n";
        std::cerr << "  sample_interval_sec: output interval only every N seconds "
                  << "(0=all, def="
                  << GlobalVariables::METRIC_SAMPLE_INTERVAL_SECONDS << ")\n";
        return 1;
    }

    std::string input_folder = argv[1];
    std::string output_folder = argv[2];
    std::string mode = argv[3];
    LLint duration = std::stoll(argv[4]);

    if (argc >= 6) {
        GlobalVariables::EXPORT_DETAIL_LEVEL = std::stoi(argv[5]);
    }
    if (argc >= 7) {
        GlobalVariables::METRIC_SAMPLE_INTERVAL_SECONDS = std::stoi(argv[6]);
    }

    std::cout << "Running Orchestrator: Input=" << input_folder
              << ", Output=" << output_folder
              << ", Mode=" << mode
              << ", Duration=" << duration << " ms"
              << ", ExportLevel=" << GlobalVariables::EXPORT_DETAIL_LEVEL
              << ", SampleInterval="
              << GlobalVariables::METRIC_SAMPLE_INTERVAL_SECONDS << "s\n";

    auto start_time = std::chrono::high_resolution_clock::now();

    if (mode == "CFS") {
        CFSSimulationOrchestrator orchestrator(input_folder, output_folder, duration);
        orchestrator.RunSimulation();

        auto end_time = std::chrono::high_resolution_clock::now();
        double exec_seconds = std::chrono::duration<double>(end_time - start_time).count();
        std::cout << "ExecutionTime_s: " << exec_seconds << "\n";

        std::string exec_time_path = output_folder + "/" + mode + "/scheduler_execution_time.txt";
        std::ofstream exec_time_file(exec_time_path);
        if (exec_time_file.is_open()) {
            exec_time_file << exec_seconds << "\n";
            exec_time_file.close();
        }

        const auto& metrics = orchestrator.GetIntervalSPMetrics();
        if (!metrics.empty()) {
            double sum = std::accumulate(metrics.begin(), metrics.end(), 0.0);
            double avg = sum / metrics.size();
            std::cout << "Average SP Metric: " << avg << "\n";

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

    FixedTaskPrioritySchedulingOrchestrator orchestrator(input_folder, output_folder, mode, duration);
    orchestrator.RunSimulation();

    auto end_time = std::chrono::high_resolution_clock::now();
    double exec_seconds = std::chrono::duration<double>(end_time - start_time).count();
    std::cout << "ExecutionTime_s: " << exec_seconds << "\n";

    std::string exec_time_path = output_folder + "/" + mode + "/scheduler_execution_time.txt";
    std::ofstream exec_time_file(exec_time_path);
    if (exec_time_file.is_open()) {
        exec_time_file << exec_seconds << "\n";
        exec_time_file.close();
    }

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

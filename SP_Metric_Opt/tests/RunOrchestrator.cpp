#include "sources/RTDA/ImplicitCommunication/SimulationOrchestrator.h"
#include <cctype>
#include <chrono>
#include <iostream>
#include <fstream>
#include <numeric>
#include <string>

using namespace SP_OPT_PA;

// Parse an INCR_P<n> mode string and override ReoptimizationPeriod with n.
// Period is otherwise loaded from parameters.yaml; the orchestrator dispatches
// INCR_P<n> exactly as INCR (IsINCRPeriodVariant). Encoding the period in the
// mode string lets the A/B config sweep through the existing scheduler-name
// plumbing with no YAML mutation (which would race compare_optimizers.py's
// parallel workers).
//
// P1.4 history: an INCR_P<n>_ADOPTED suffix used to additionally set the
// ReoptStartFromAdoptedTL flag so the reopt descent seeded from the carried
// adopted TL (the algorithmic seed) instead of the Gaussian-mean TL. P1.4 made
// that seed the permanent, unconditional policy and REMOVED the flag (and the
// _ADOPTED arms). Any trailing suffix after the digits (e.g. a stale
// _ADOPTED arm in a config) is now a HARD ERROR rather than a silent
// fall-through — the P1.3 regression was exactly a stale config silently
// dispatching to an empty result.
// Returns true if mode is an INCR_P<n> variant (regardless of whether the
// override succeeded); false otherwise.
static bool MaybeOverrideReoptPeriod(const std::string& mode) {
    const std::string prefix = "INCR_P";
    if (mode.rfind(prefix, 0) != 0 || mode.size() <= prefix.size()) {
        return false;  // not an INCR_P* variant (or bare "INCR_P")
    }
    size_t i = prefix.size();
    while (i < mode.size() && std::isdigit(static_cast<unsigned char>(mode[i]))) {
        i++;
    }
    if (i == prefix.size()) {
        return true;  // INCR_P with no digits — reject silently
    }
    const std::string digits = mode.substr(prefix.size(), i - prefix.size());
    int period = 0;
    try {
        period = std::stoi(digits);
    } catch (const std::exception& e) {
        std::cerr << "Error: invalid INCR_P<n> suffix '" << digits
                  << "' in mode '" << mode << "': " << e.what() << "\n";
        return true;
    }
    if (period < 1) {
        std::cerr << "Error: INCR_P<n> period must be >= 1, got " << period
                  << " (mode=" << mode << ")\n";
        return true;
    }
    if (i != mode.size()) {
        // P1.4: any trailing suffix (e.g. the removed _ADOPTED) is a hard
        // error, not a silent alias. Fail loudly so a stale config can't
        // dispatch to an empty ResourceOptResult (the P1.3 trap).
        std::cerr << "Error: unrecognized INCR_P<n> suffix '"
                  << mode.substr(i) << "' in mode '" << mode
                  << "'. P1.4 removed the _ADOPTED arms (the adopted-TL seed "
                  << "is now the unconditional default). Use plain INCR_P"
                  << period << ".\n";
        return true;
    }
    GlobalVariables::ReoptimizationPeriod = period;
    return true;
}

int main(int argc, char** argv) {
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0]
                  << " <input_folder> <output_folder> <mode> <duration_ms>"
                  << " [export_level] [sample_interval_sec]\n";
        std::cerr << "Modes: RM, BF, INCR, INCR_NO_TL, INCR_WCET, INCR_SCRATCH, "
                  << "RM_FAST, RM_SLOW\n";
        std::cerr << "  INCR_P<n>: INCR with ReoptimizationPeriod overridden to n "
                  << "(e.g. INCR_P1, INCR_P10, INCR_P30, INCR_P60)\n";
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

    // INCR_P<n> period override. Done after the optional CLI overrides so the
    // period is set exactly once and deterministically from the mode string.
    MaybeOverrideReoptPeriod(mode);

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

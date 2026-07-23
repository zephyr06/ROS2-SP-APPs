#include "sources/RTDA/ImplicitCommunication/SimulationOrchestrator.h"
#include <cctype>
#include <chrono>
#include <iostream>
#include <fstream>
#include <numeric>
#include <string>

using namespace SP_OPT_PA;

// Parse an INCR_Reopt_X mode string and override ReoptimizationPeriod with X.
// Period is otherwise loaded from parameters.yaml; the orchestrator dispatches
// INCR_Reopt_X exactly as INCR (IsINCRPeriodVariant). Encoding the period in
// the mode string lets the A/B config sweep through the existing scheduler-name
// plumbing with no YAML mutation (which would race compare_optimizers.py's
// parallel workers). X is the reoptimization period: every X-th interval runs
// a from-scratch descent (ReOptimizePeriodic), the rest run warm-started
// incremental (OptimizeIncre_w_TL). X=1 = reopt every interval (the max-reopt
// extreme); larger X = more incremental, less reopt. The name surfaces what the
// period counts, unlike the retired INCR_P<n> form where P1 read as
// "incremental, period 1" but was the max-reopt arm.
//
// P2.4: the old INCR_P<n> name is RETIRED. A stale INCR_P<n> config is a HARD
// ERROR (mirror the P1.4 _ADOPTED pattern), NOT a silent alias — the P1.3
// regression was exactly a stale config silently dispatching to an empty
// result. The hard error prints to stderr and the mode falls through dispatch
// to the RM baseline (degenerate), so the stale name is loud, not silent.
//
// P1.4 history: an INCR_Reopt_X_ADOPTED suffix used to additionally set the
// ReoptStartFromAdoptedTL flag so the reopt descent seeded from the carried
// adopted TL (the algorithmic seed) instead of the Gaussian-mean TL. P1.4 made
// that seed the permanent, unconditional policy and REMOVED the flag (and the
// _ADOPTED arms). Any trailing suffix after the digits (e.g. a stale _ADOPTED
// arm in a config) is now a HARD ERROR rather than a silent fall-through.
// Returns true if mode is an INCR_Reopt_X variant OR a retired INCR_P<n> form
// (either way "claimed" so it falls through to the RM baseline, not silently
// aliased); false otherwise.
static bool MaybeOverrideReoptPeriod(const std::string& mode) {
    const std::string prefix = "INCR_Reopt_";
    if (mode.rfind(prefix, 0) == 0 && mode.size() > prefix.size()) {
        size_t i = prefix.size();
        while (i < mode.size() && std::isdigit(static_cast<unsigned char>(mode[i]))) {
            i++;
        }
        if (i == prefix.size()) {
            return true;  // INCR_Reopt_ with no digits — reject silently
        }
        const std::string digits = mode.substr(prefix.size(), i - prefix.size());
        int period = 0;
        try {
            period = std::stoi(digits);
        } catch (const std::exception& e) {
            std::cerr << "Error: invalid INCR_Reopt_X suffix '" << digits
                      << "' in mode '" << mode << "': " << e.what() << "\n";
            return true;
        }
        if (period < 1) {
            std::cerr << "Error: INCR_Reopt_X period must be >= 1, got " << period
                      << " (mode=" << mode << ")\n";
            return true;
        }
        if (i != mode.size()) {
            // P1.4: any trailing suffix (e.g. the removed _ADOPTED) is a hard
            // error, not a silent alias. Fail loudly so a stale config can't
            // dispatch to an empty ResourceOptResult (the P1.3 trap).
            std::cerr << "Error: unrecognized INCR_Reopt_X suffix '"
                      << mode.substr(i) << "' in mode '" << mode
                      << "'. P1.4 removed the _ADOPTED arms (the adopted-TL seed "
                      << "is now the unconditional default). Use plain INCR_Reopt_"
                      << period << ".\n";
            return true;
        }
        GlobalVariables::ReoptimizationPeriod = period;
        return true;
    }

    // P2.4: the retired INCR_P<n> name is a HARD ERROR, not a silent alias.
    const std::string old_prefix = "INCR_P";
    if (mode.rfind(old_prefix, 0) == 0 && mode.size() > old_prefix.size()) {
        size_t i = old_prefix.size();
        while (i < mode.size() && std::isdigit(static_cast<unsigned char>(mode[i]))) {
            i++;
        }
        if (i > old_prefix.size()) {
            // Stale INCR_P<n> (possibly with a trailing suffix) — tell the user
            // the new name. Covers INCR_P1, INCR_P10, and the doubly-stale
            // INCR_P<n>_ADOPTED (both the P2.4 rename and P1.4's suffix
            // removal). The period is NOT overridden, so the run falls through
            // dispatch to the RM baseline (degenerate) — loud, not silent.
            std::cerr << "Error: mode '" << mode
                      << "' uses the RETIRED INCR_P<n> name (P2.4 renamed it)."
                      << " The P<n> knob ran the wrong way for a reader (P1 ="
                      << " reopt every interval, the max-reopt extreme, NOT"
                      << " incremental). Use INCR_Reopt_<n> instead (same period,"
                      << " same dispatch). This is a HARD ERROR, not a silent"
                      << " alias — the run will fall through to the RM baseline.\n";
            return true;
        }
        // INCR_P followed by a non-digit (e.g. the future INCR_PURE) is not a
        // retired period-variant rename; leave it for its own dispatch branch.
    }

    return false;
}

int main(int argc, char** argv) {
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0]
                  << " <input_folder> <output_folder> <mode> <duration_ms>"
                  << " [export_level] [sample_interval_sec]\n";
        std::cerr << "Modes: RM, BF, INCR, INCR_NO_TL, INCR_WCET, "
                  << "RM_FAST, RM_SLOW\n";
        std::cerr << "  INCR_Reopt_X: INCR with ReoptimizationPeriod overridden to X "
                  << "(X = reopt period; X=1 reopts every interval, larger X = more"
                  << " incremental. e.g. INCR_Reopt_1, INCR_Reopt_5, INCR_Reopt_10,"
                  << " INCR_Reopt_30, INCR_Reopt_60). Bare INCR uses the YAML period"
                  << " (default 10). The retired INCR_P<n> name is a HARD ERROR.\n";
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

    // INCR_Reopt_X period override. Done after the optional CLI overrides so the
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
        // exec_seconds is the TOTAL RunSimulation() wall-time (taskset I/O +
        // scheduler + RTDA rollout + SP-metric + export) — printed for reference
        // only, unparsed. scheduler_execution_time.txt gets the scheduler-only
        // sum (DeterminePrioritiesAndBudgets; 0.0 for CFS, which has none).
        std::cout << "TotalProcessTime_s: " << exec_seconds << "\n";
        std::cout << "SchedulerExecutionTime_s: " << orchestrator.GetSchedulerExecutionTime() << "\n";

        std::string exec_time_path = output_folder + "/" + mode + "/scheduler_execution_time.txt";
        std::ofstream exec_time_file(exec_time_path);
        if (exec_time_file.is_open()) {
            exec_time_file << orchestrator.GetSchedulerExecutionTime() << "\n";
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
    // exec_seconds is the TOTAL RunSimulation() wall-time (taskset I/O +
    // scheduler + RTDA rollout + SP-metric + export) — printed for reference
    // only, unparsed. scheduler_execution_time.txt gets the scheduler-only
    // sum (DeterminePrioritiesAndBudgets, where the RTA cache + Transaction
    // live; excludes RTDA rollout / SP-metric / I/O).
    std::cout << "TotalProcessTime_s: " << exec_seconds << "\n";
    std::cout << "SchedulerExecutionTime_s: " << orchestrator.GetSchedulerExecutionTime() << "\n";

    std::string exec_time_path = output_folder + "/" + mode + "/scheduler_execution_time.txt";
    std::ofstream exec_time_file(exec_time_path);
    if (exec_time_file.is_open()) {
        exec_time_file << orchestrator.GetSchedulerExecutionTime() << "\n";
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

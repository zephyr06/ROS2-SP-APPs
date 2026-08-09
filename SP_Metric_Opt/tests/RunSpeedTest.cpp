#include "sources/RTDA/ImplicitCommunication/SimulationOrchestrator.h"
#include <cctype>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

using namespace SP_OPT_PA;

// Parse an INCR_Reopt_X mode string and override GlobalVariables::ReoptimizationPeriod with X.
static bool MaybeOverrideReoptPeriod(const std::string& mode) {
    const std::string prefix = "INCR_Reopt_";
    if (mode.rfind(prefix, 0) == 0 && mode.size() > prefix.size()) {
        size_t i = prefix.size();
        while (i < mode.size() && std::isdigit(static_cast<unsigned char>(mode[i]))) {
            i++;
        }
        if (i == prefix.size()) {
            return false;
        }
        const std::string digits = mode.substr(prefix.size(), i - prefix.size());
        int period = 0;
        try {
            period = std::stoi(digits);
        } catch (const std::exception& e) {
            std::cerr << "Error: invalid INCR_Reopt_X suffix '" << digits
                      << "' in mode '" << mode << "': " << e.what() << "\n";
            return false;
        }
        if (period < 1) {
            std::cerr << "Error: INCR_Reopt_X period must be >= 1, got " << period
                      << " (mode=" << mode << ")\n";
            return false;
        }
        GlobalVariables::ReoptimizationPeriod = period;
        return true;
    }
    return false;
}

struct BenchmarkResult {
    std::string mode;
    double total_wall_time_s;
    double total_sched_time_s;
    size_t num_intervals;
    double avg_et_s;
    double threshold_s;
    double avg_sp;
    bool passed;
};

int main(int argc, char** argv) {
    std::string taskset_dir = GlobalVariables::PROJECT_PATH + "tests/speed_test/taskset_N10";
    double threshold_reopt1 = 0.1;
    double threshold_reopt10 = 0.1;

    if (argc >= 2) {
        taskset_dir = argv[1];
    }
    if (argc >= 3) {
        try {
            threshold_reopt1 = std::stod(argv[2]);
            threshold_reopt10 = threshold_reopt1;
        } catch (const std::exception& e) {
            std::cerr << "Error parsing threshold argument: " << argv[2] << "\n";
            return 1;
        }
    }
    if (argc >= 4) {
        try {
            threshold_reopt10 = std::stod(argv[3]);
        } catch (const std::exception& e) {
            std::cerr << "Error parsing threshold_10 argument: " << argv[3] << "\n";
            return 1;
        }
    }

    if (!std::filesystem::exists(taskset_dir)) {
        std::cerr << "Error: Taskset directory does not exist: " << taskset_dir << "\n";
        return 1;
    }

    std::filesystem::path output_base = std::filesystem::temp_directory_path() / "speed_test_benchmark_output";
    std::filesystem::create_directories(output_base);

    std::cout << "==================================================\n";
    std::cout << "Running Release Speed Test Benchmark\n";
    std::cout << "Taskset Directory : " << taskset_dir << "\n";
    std::cout << "Threshold (INCR_Reopt_1)  : " << threshold_reopt1 << " s / interval\n";
    std::cout << "Threshold (INCR_Reopt_10) : " << threshold_reopt10 << " s / interval\n";
    std::cout << "==================================================\n\n";

    std::vector<std::pair<std::string, double>> schedulers = {
        {"INCR_Reopt_1", threshold_reopt1},
        {"INCR_Reopt_10", threshold_reopt10}
    };

    std::vector<BenchmarkResult> results;
    bool all_passed = true;
    LLint duration_ms = 10000; // 10s/interval (10x fewer ticks than 100000); max task period 1000ms -> >=10 releases

    for (const auto& [mode, threshold] : schedulers) {
        MaybeOverrideReoptPeriod(mode);

        std::string mode_out_dir = (output_base / mode).string();
        std::filesystem::create_directories(mode_out_dir);

        auto wall_start = std::chrono::high_resolution_clock::now();
        FixedTaskPrioritySchedulingOrchestrator orchestrator(taskset_dir, mode_out_dir, mode, duration_ms);
        orchestrator.RunSimulation();
        auto wall_end = std::chrono::high_resolution_clock::now();

        double wall_time_s = std::chrono::duration<double>(wall_end - wall_start).count();
        double sched_time_s = orchestrator.GetSchedulerExecutionTime();
        const std::vector<double>& sp_metrics = orchestrator.GetIntervalSPMetrics();
        size_t num_intervals = sp_metrics.size();
        double avg_et = (num_intervals > 0) ? (sched_time_s / num_intervals) : 0.0;
        bool passed = (avg_et <= threshold);

        // Mean of per-interval SP metrics (analytic, in [0,1]). The first
        // interval uses the seed/un-optimized priority vector; subsequent
        // intervals reflect the optimizer's chosen priority + time-limit.
        double sp_sum = 0.0;
        for (double sp : sp_metrics) {
            sp_sum += sp;
        }
        double avg_sp = (num_intervals > 0) ? (sp_sum / static_cast<double>(num_intervals)) : 0.0;

        if (!passed) {
            all_passed = false;
        }

        results.push_back({mode, wall_time_s, sched_time_s, num_intervals, avg_et, threshold, avg_sp, passed});
    }

    std::cout << "\n==================================================\n";
    std::cout << "SPEED TEST BENCHMARK RESULTS SUMMARY\n";
    std::cout << "==================================================\n";

    for (const auto& res : results) {
        std::cout << "Scheduler Mode : " << res.mode << "\n";
        std::cout << "  Total Process Wall Time : " << std::fixed << std::setprecision(6) << res.total_wall_time_s << " s\n";
        std::cout << "  Total Scheduler ET      : " << std::fixed << std::setprecision(6) << res.total_sched_time_s << " s\n";
        std::cout << "  Simulated Intervals     : " << res.num_intervals << "\n";
        std::cout << "  Avg ET per Interval     : " << std::fixed << std::setprecision(6) << res.avg_et_s << " s\n";
        std::cout << "  Average SP Metric       : " << std::fixed << std::setprecision(6) << res.avg_sp << "\n";
        std::cout << "  Target Threshold        : " << std::fixed << std::setprecision(6) << res.threshold_s << " s\n";
        std::cout << "  Status                  : " << (res.passed ? "PASS" : "FAIL") << "\n";
        std::cout << "--------------------------------------------------\n";
    }

    std::cout << "OVERALL BENCHMARK VERDICT : " << (all_passed ? "PASS" : "FAIL") << "\n";
    std::cout << "==================================================\n";

    return all_passed ? 0 : 1;
}

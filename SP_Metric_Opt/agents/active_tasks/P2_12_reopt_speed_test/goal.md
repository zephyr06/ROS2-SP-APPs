# P2.12 — C++ Executable Speed Test Benchmark for Reoptimization

## The Goal

Implement a C++ speed benchmark executable (`RunSpeedTest`) under `tests/` that reads the taskset at `tests/speed_test/taskset_N8` (or a specified input directory), executes the `INCR_Reopt_1` and `INCR_Reopt_10` schedulers, records their scheduler execution times, and compares the average execution time (ET) per activation against manually specified thresholds (e.g., 0.1s). The benchmark reports explicit PASS/FAIL status for each scheduler and overall.

## Why it matters

To track and catch performance regressions caused by recent commits. Speed benchmarks must run in Release mode rather than Debug mode to measure true runtime performance without being skewed by debug assertions or unoptimized library overhead.

## Key Requirements

1. Add active task files in `agents/active_tasks/P2_12_reopt_speed_test/`.
2. Implement `tests/RunSpeedTest.cpp` capable of running `INCR_Reopt_1` and `INCR_Reopt_10` using `FixedTaskPrioritySchedulingOrchestrator`.
3. Configure `tests/CMakeLists.txt` to register `RunSpeedTest` executable target.
4. Support CLI threshold parameters and input directory overrides with sensible defaults.
5. Print detailed comparison output (Total Sched ET, Avg Sched ET vs Threshold, PASS/FAIL).
6. Build and run in Release mode to verify performance against thresholds.

# P2.21 — Fix Application API Incompatibilities and Build Setup

**Priority:** P2 (hygiene / build correctness / application compatibility)  
**Status:** IN PROGRESS (2026-08-19)  
**Depends on:** Main optimization API redesigns (P0.5, P0.7, P1.10, P1.28)

## Goal

Resolve API incompatibilities and compilation issues in `SP_Metric_Opt/applications/` caused by earlier refactorings in the main optimization module (`SP_OPT_PA::OptimizePA_Incre_with_TimeLimits`). Enable clean compilation of standalone applications (`real_time_manager`, `tsp_solver_osm`) while conditionally guarding hardware/robot-specific applications (`mpc`, `dynaslam`, `rrt_solver`).

## Issues Identified & Fixed

1. **Obsolete API in `scheduler_wrapper.h`:**
   - `OptimizePA_Incre_with_TimeLimits` previously replaced `OptimizeFromScratch_w_TL` with `ReOptimizePeriodic`.
   - `SchedulerApp::run()` in `applications/real_time_manager/include/real_time_manager/scheduler_wrapper.h` was calling `incremental_optimizer_w_TL_.OptimizeFromScratch_w_TL(...)`, causing build failure in `listener_scheduler`.
   - Fixed by calling `incremental_optimizer_w_TL_.ReOptimizePeriodic(...)`.

2. **Incorrect include path in `update_priority_assignment_in_memory.h`:**
   - Included non-existent header `"applications/real_time_manager/include/real_time_manager/update_priority_assignments.h"`.
   - Fixed to `#include "real_time_manager/update_priority_assignment.h"` and added missing `#include "sources/Utils/testMy.h"`.

3. **Compiler warning in `tsp_osm_main_utils.h`:**
   - `run_tsp` function had a missing return path at the end of a non-void function.
   - Added `return -1;` fallback to eliminate `-Wreturn-type` warning.

4. **CMake configuration for applications (`applications/CMakeLists.txt` & root `CMakeLists.txt`):**
   - Root `CMakeLists.txt` had `# add_subdirectory(applications)` commented out because `dynaslam`, `mpc`, and `rrt_solver` require dependencies not present on standard development workstations (e.g. `OsqpEigen`, `SFML`, `Pangolin/ORB_SLAM2/YOLO`).
   - Added conditional `find_package` and existence checks in `applications/CMakeLists.txt` so `mpc`, `dynaslam`, and `rrt_solver` only configure if their dependencies exist.
   - Enabled `add_subdirectory(applications)` in root `CMakeLists.txt`.

## Verification

- `cd build && cmake .. && make -j5`: builds 100% cleanly including `et_statistics`, `set_cpu_and_priority`, `update_priority_assignments`, `listener_scheduler`, `tsp_solver_executable_osm`, `tsp_solver_listener`, `save_tsp_perf_time_data`.
- `cd build && ctest --output-on-failure`: 17/17 tests PASS.
- `cd release && cmake .. && make -j5 && ./tests/RunSpeedTest`: PASS (0.026–0.027 s/interval vs 0.10 s threshold).

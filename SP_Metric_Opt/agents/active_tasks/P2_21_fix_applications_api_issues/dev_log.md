# Dev Log: P2.21 — Fix Application API Incompatibilities and Build Setup

## 2026-08-19

- **Investigation:**
  - Audited `SP_Metric_Opt/applications/` to check compatibility with recent optimizer changes.
  - Root `CMakeLists.txt` had commented out `add_subdirectory(applications)`.
  - Attempting to compile `listener_scheduler` failed due to missing member `OptimizeFromScratch_w_TL` on `OptimizePA_Incre_with_TimeLimits`. In earlier refactorings, from-scratch optimization with time limits was consolidated into `ReOptimizePeriodic`.
  - `update_priority_assignment_in_memory.h` referenced misspelled header `update_priority_assignments.h` and was missing `testMy.h` for `CoutWarning`.
  - `tsp_osm_main_utils.h` produced a `-Wreturn-type` warning due to missing return after a `switch` statement in `run_tsp`.
  - `mpc`, `rrt_solver`, and `dynaslam` have external dependencies (`OsqpEigen`, `SFML`, `Pangolin`/`ORB_SLAM2`) that are not present on generic workstations and should only be conditionally built when available.

- **Changes Made:**
  - `applications/real_time_manager/include/real_time_manager/scheduler_wrapper.h`: Changed `incremental_optimizer_w_TL_.OptimizeFromScratch_w_TL(...)` to `incremental_optimizer_w_TL_.ReOptimizePeriodic(...)`.
  - `applications/real_time_manager/include/real_time_manager/update_priority_assignment_in_memory.h`: Corrected include path to `real_time_manager/update_priority_assignment.h` and included `sources/Utils/testMy.h`.
  - `applications/tsp_solver_osm/tsp_osm_main_utils.h`: Added `return -1;` to end of `run_tsp` function.
  - `applications/CMakeLists.txt`: Added `find_package` / conditional guards for `OsqpEigen`, `SFML`, and `Pangolin`/`ORB_SLAM2`.
  - `CMakeLists.txt`: Enabled `add_subdirectory(applications)`.

- **Verification:**
  - `cd build && cmake .. && make -j5`: Built `real_time_manager` targets (`et_statistics`, `set_cpu_and_priority`, `update_priority_assignments`, `listener_scheduler`) and `tsp_solver_osm` targets (`tsp_solver_executable_osm`, `tsp_solver_listener`, `save_tsp_perf_time_data`) with 0 errors and 0 warnings.
  - `cd build && ctest --output-on-failure`: 17/17 tests PASS (100%).
  - `cd release && cmake .. && make -j5 && ./tests/RunSpeedTest`: Speed test benchmark PASS (avg ET 0.026–0.027s/interval vs 0.10s threshold).

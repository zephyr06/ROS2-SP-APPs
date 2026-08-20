# Tasks: P2.21 — Fix Application API Incompatibilities and Build Setup

- [x] **Step 1: Audit application source code for API mismatches**
  - Inspect `applications/real_time_manager/`, `applications/tsp_solver_osm/`, `applications/mpc/`, `applications/rrt_solver/`, `applications/dynaslam/`.
  - Identified `OptimizeFromScratch_w_TL` removal in `OptimizePA_Incre_with_TimeLimits` impacting `scheduler_wrapper.h`.
  - Identified misspelled include in `update_priority_assignment_in_memory.h`.
  - Identified missing non-void return in `tsp_osm_main_utils.h`.

- [x] **Step 2: Fix API and source code issues in applications**
  - Updated `scheduler_wrapper.h` to call `ReOptimizePeriodic` on uninitialized incremental optimizer instances.
  - Fixed `#include` and missing helper imports in `update_priority_assignment_in_memory.h`.
  - Added fallback `return -1;` in `run_tsp` in `tsp_osm_main_utils.h`.

- [x] **Step 3: Update CMake configuration**
  - Updated `applications/CMakeLists.txt` to conditionally check for `OsqpEigen`, `SFML`, and `ORB_SLAM2`/`Pangolin`.
  - Enabled `add_subdirectory(applications)` in root `CMakeLists.txt`.

- [x] **Step 4: Verify build and test suite**
  - Ran `cd build && make -j5` (all targets built successfully).
  - Ran `cd build && ctest --output-on-failure` (17/17 tests pass).
  - Ran `cd release && make -j5 && ./tests/RunSpeedTest` (PASS).

- [x] **Step 5: Stage changes for user review**
  - Run `git add` on modified files.

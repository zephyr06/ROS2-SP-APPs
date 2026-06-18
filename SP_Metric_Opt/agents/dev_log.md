# Development Log: Sanity Check on Simulation Experiment Implementation

This log contains the analysis of the C++ and Python codebase compared against the simulation experiment design described in `paper_sections/section14_simu_exp.tex`.

## 1. Mathematical Model vs. Code Implementation (GMM & Polar Coordinates)

### Paper Description
- The paper describes simulating the robot's physical location $(x, y)$ and the task's execution time $C_i$ using a joint 3D Gaussian distribution (GMM) covering a 95% confidence interval:
  - $\mu_x^E = W^E/2, \sigma_x = W^E/4$
  - $\mu_y^E = H^E/2, \sigma_y = H^E/4$
  - Variance of task execution time: $\sigma_{c,i} = k \mu_{c,i}^E$ where $k \in [0.5, 0.6]$.
  - Covariance ratios $\rho_{x,c,i}, \rho_{y,c,i}$ randomly generated in $[-1, 1]$.
- Footnote 2 states: *"In implementation, we actually used polar coordinates instead of Cartesian coordinates, but the math basically remains the same. Please check our released the code for more details."*

### Code Status & Verification
- **Dataset Generation Script Found:** The dataset generation logic is implemented in [gen_taskset.py](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Gen_Taskset/gen_taskset.py).
- **Polar Coordinates Conversion:** The script takes Cartesian coordinates $x, y$ as input inputs, converts them internally to polar coordinates $r$ (radius) and $a$ (angle) to sample the conditional distribution of execution time under the joint 3D Gaussian Mixture Model, and writes out Cartesian trace files (`x, y, et`). This fully matches and aligns with the paper's description of polar coordinate-based sampling.
- **Cartesian Coordinates:** As per user input, using Cartesian coordinates is fine and will be kept for simplicity during debugging.

---

## 2. C++ Simulation Logic (`sources/RTDA/ImplicitCommunication/ScheduleSimulation.cpp`)

- The scheduling simulation functions (like `SimulateCSPSched` and `SimulateCSPSched_vecs` under the `#if defined(RYAN_HE_CHANGE)` guards) correctly implement the simulation loop over multiple time intervals (each associated with a `taskset_characteristics_[x].yaml` configuration file representing 10s steps).
- Supported scheduling policies are correctly implemented and dispatched:
  - Brute Force (`BR`)
  - Incremental (`INCR`)
  - RM Fast (`RM_FAST`)
  - RM Slow (`RM_SLOW`)
  - CFS (`CFS`)
- Priority recalculations occur at the correct `reevaluate_prio_interval_ms` boundaries (default 10,000ms / 10s).

---

## 3. Completed Tasks Archive
The following tasks have been successfully implemented and verified:
- [x] Fix Google Test dependency header pollution by replacing global include paths with target-based includes in `CMakeLists.txt`.
- [x] Resolve the ambiguous `IsXDigit` and `GTEST_FLAG_SET` compile errors in `gtest-all.cc`.
- [x] Adjust `testIncreOpt_w_TL` debug timing threshold check (increase or conditionalize for Debug builds).
- [x] Fix incremental optimizer `opt_sp_` reset bug in `OptimizeIncre_w_TL`.
- [x] Verify that `Gen_Taskset/gen_taskset.py` performs GMM polar/Cartesian dataset generation.
- [x] Uncomment `testOptimizeIncrePA` in `tests/CMakeLists.txt` and rename it to `testOptimizeIncrePA_run` to prevent name conflict.
- [x] Design and add C++ Google Test cases: `UnfeasibleTaskSet`, `DeterministicTaskSet`, and `PartitionedCoreOptimization` (all passing).


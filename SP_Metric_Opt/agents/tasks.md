High-level Task list:
- [x] Fix opt_sp_ initialization bug, instead of initializing it as 0, we'll use -1.
- [x] change configuration optimization's iteration logic. current implemntation iterates through all the possibile combination of execution time limit configurations of all tasks, which has expoentnial run-time complexity. We'll switch it to linear iteration by checking configuration of each task one by one. implement task sorting before iterating on tasks. related commit: 3b158b51c0dc88df38f0b6670173d781731ec6bc
- [x] add complete fairness scheduler simulation code, and related tests. check code in "simulation_exp" branch for reference, but notice that code in that branch has bugs and possible logic issues. so be cautious about what to take, and follow TDD.
- [x] add python simulated training data generator to generate simulated task sets, add core functionality and related tests (check simulation_exp branch):
    - [x] add python simulated environment that evaluates different schedulers' performance on the simulated dataset
    - [x] add pipeline script to generate python simulated task set from given configuration files end-to-end
- add ablation study baseline (check simulation_exp branch):
    * INCR_NO_TL
    * INCR_WCET
    * INCR_SATIC_ET
- add end-to-end task set execution schedule code in c++. the code should read from each individual task set at different time stamps, then run different scheduler method, and monitor overall SP metric (check simulation_exp branch for basic ideas of related code)
- add any missed baselines for simulation experiments such as CFS
- add more types of reulst visualization figure plots(check simulation_exp branch):
* x axis: number of tasks per task set
* y axis:
    - figure 1: sp metric (differnt schedulers)
    - figure 2 : scheduler decision execution time
    - figure 3: average sp metric (only INCR scheduler, consider different optimizer invocation intervals)
    - figure 4: deadline miss rate of important tasks
- add end-to-end script to run simulation evaluation of all baseline methods for given task set config.
- other functional changes implemented in simulation_exp branch but not yet in this branch.


----------------------------- Future implementation plans----------------
# Implementation Plan - Add Ablation Study Baselines

This plan details:
1. Porting the complete C++ scheduler simulation and priority optimization codebase from the `simulation_exp` branch, which implements the ablation study baselines:
   - `INCR_NO_TL` (Task Budget/Time Limit Optimization disabled).
   - `INCR_WCET` (Constant Worst-Case Execution Time modeling).
   - `INCR_NO_SORT` (Heuristic sorting disabled in CD priority assignment).
   - Custom priority reevaluation intervals (`_int<X>s` scheduler suffix).
2. Compiling the C++ simulation binaries.
3. Running verification tests.

## User Review Required

> [!IMPORTANT]
> - We will checkout all C++ source code (`sources/`) and C++ test drivers (`tests/`) from the `simulation_exp` branch.
> - This brings in the full multi-core `CSPSimulation_2` source and the updated `AnalyzeSP_Metric.cpp` logic.
> - We will keep our own Python taskset generator logic (`Gen_Taskset/`) and `run_sim_experiments.py` which are already updated for 1 GMM instance and relocated output directories.
> - We will rebuild the C++ executables in `release` and `build` directories using CMake.

## Proposed Changes

### Porting C++ Source Code & Tests

- We will run `git checkout simulation_exp -- <files>` for:
  - All C++ files under `sources/`
  - All C++ source and CMakeLists files under `tests/` (except python test scripts)

### Build Settings

#### [MODIFY] [CMakeLists.txt](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/tests/CMakeLists.txt)
- Register `CSPSimulation_2` executable.

## Verification Plan

### Automated Tests
- Run `make` in `release` and `build` directories to ensure C++ compilation finishes successfully.
- Run `PYTHONPATH=. pytest Gen_Taskset/tests tests/python` to ensure python tests pass.
- Run the compiled C++ tests under `build/tests/` to verify priority optimization and scheduling correctness.

### Manual Verification
- Run simulation experiments with one of the baseline schedulers (e.g., `INCR_NO_TL` or `INCR_WCET`) to ensure they output expected response times and log files:
  ```bash
  python3 run_sim_experiments.py --num_tasks 4 --schedulers INCR INCR_NO_TL INCR_WCET CFS
  ```
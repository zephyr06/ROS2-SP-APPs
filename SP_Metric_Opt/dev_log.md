# Developer Log - ROS2 Safety-Performance Priority Optimization

## 📅 Development Journal & Completed Tasks

### Refactoring & Core Setup
- **Monolithic Script Modularization**: Split the legacy `gen_taskset.py` into a cohesive Python package under `Gen_Taskset/lib/`.
- **Core Partitioning Implementation**: Replaced hardcoded processor assignment with a greedy bin-packing core load balancing algorithm supporting multi-core execution profiles.
- **Specification Reconciliation**: 
  - Verified and updated small/big periods select sets (in ms periods instead of legacy Hz bounds).
  - Aligned variance configurations ($k \in [0.5, 0.6]$ standard deviation ratio).
  - Added randomized relative deadlines (SP deadline thresholds) generated from 50% to 100% of periods.
  - Aligned discrete threshold choices `[0.2, 0.4, 0.6, 0.8, 1.0]`.

### C++ Priority Optimization Test Suite
- Implemented consistency tests between brute-force (`OptimizePA_BF`) and incremental (`OptimizePA_Incre`) schedulers under `tests/testOptimizeIncrePA.cpp`.
- Implemented deterministic limits, multi-core partitioning independent RTA calculations, and single-core/multi-core overload boundary checks under `tests/testRTA.cpp`.
- Verified that all unit and integration tests compile and pass in both Debug and Release builds.

### Pipeline Automation & Speedup Optimizations
- Implemented `run_sim_experiments.py` orchestrating trace generation, multi-core scheduler simulation runs, and stats boxplots.
- **Concurreny (8x speedup)**: Utilized Python's `concurrent.futures.ThreadPoolExecutor` to run scheduler simulation instances in parallel across all 8 CPU cores.
- **Timeout Warn Silence (30x speedup)**: Silenced verbose brute-force timeout outputs to console under non-debug mode in `OptimizeSP_Base.cpp`. Set `TIME_LIMIT: 2`s (or `1`s for $\ge 8$ tasks) in `parameters.yaml` dynamically during simulation runs using a clean `atexit` restoration handler.

---

## 📈 Active Development Status & Live Run Logs

### 6-Task Experiment Batch
- **Configuration**: `taskset_cfg_paper_6.json` (2 big tasks, 4 small tasks).
- **Execution Setup**: 10 task sets, 8 instances per task set, 1,000,000 ms simulation time.
- **Background Task ID**: `task-1682`
- **Output Directory**: `TaskData/experiment_6_tasks/`

### 8-Task Experiment Batch
- **Configuration**: `taskset_cfg_paper_8.json` (2 big tasks, 6 small tasks).
- **Execution Setup**: 5 task sets, 2 instances per task set (customized to balance core load), 1,000,000 ms simulation time.
- **Background Task ID**: `task-1729`
- **Output Directory**: `TaskData/experiment_8_tasks/`

---

## 🚀 Recommended Next Steps
Once the background simulation tasks finish, the final steps are:
1. **Analyze Box Plots**: Compare `comparison_plots.png` generated in both `TaskData/experiment_6_tasks` and `TaskData/experiment_8_tasks`.
2. **Review Miss Rates & SP Metrics**: Inspect `comparison_summary.csv` containing mean and standard deviation of miss rates and SP values for CFS, RM_Fast, RM_Slow, BR, and INCR.
3. **LaTeX Paper Updates**: Update final figures and empirical result summaries in `section14_simu_exp.tex` and `section15_simu_exp_analysis.tex` to present the newly obtained 6-task and 8-task scheduler performance results.

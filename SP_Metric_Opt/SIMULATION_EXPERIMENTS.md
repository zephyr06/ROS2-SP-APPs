# Simulation Experiments

This document provides a detailed evaluation of the simulation experiments implemented in this repository, including concrete code pointers and snippets, followed by step-by-step instructions on how to generate tasksets, run the simulation, and visualize the SP-Metric results.

> [!NOTE]
> The historical discrepancies in the legacy monolithic generator script (`gen_taskset.py`) described below have been **fully resolved** in the newly refactored, modular package under [Gen_Taskset/lib/](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Gen_Taskset/lib/).

---

## Codebase Evaluation & Concrete Proof

Below is the evaluation of the discrepancies between the paper's description (mainly `section14_simu_exp.tex`) and the legacy implementation in the repository, along with how they are resolved in the new package.

### 1. CPU Core Assignment Discrepancy
* **Claim**: The paper outlines scheduling 10 tasks on 2 CPU cores independently. However, the legacy generator script `gen_taskset.py` hardcoded `processorId: 0` for all tasks, placing the entire workload on a single CPU core. Although the simulator itself (`SimulateCSPSched` and `SimulatedCSP_SingleCore`) supports multi-core simulation by partitioning tasks by `processorId`, the generated datasets forced single-core execution. Consequently, tasksets generated with high target utilization (e.g., `MEAN_CPU_UTIL: 1.6`, intended for 2 cores) heavily overloaded the single core.
* **Resolution**: In [Gen_Taskset/lib/taskset_generator.py](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Gen_Taskset/lib/taskset_generator.py), we implemented a **greedy bin-packing core partition algorithm** that dynamically distributes tasks across the available cores (`N_CORES`) based on total utilization load, ensuring balanced multi-core utilization.
* **Legacy File Pointer**: `Gen_Taskset/gen_taskset.py` (deleted)
* **Legacy Code Proof**:
  ```python
  t['processorId'] = 0  # <--- HARDCODED TO CPU 0
  ```

### 2. Task Period Sets Discrepancy
* **Claim**: The paper mentions that task periods are randomly selected from the set `[20, 33, 50, 100, 200, 500, 1000] ms`. The legacy generator instead selected HZ values translating to `[20, 33.3, 50, 100, 1000, 2000, 4000] ms`.
* **Resolution**: We standardized all configuration schemas to use millisecond periods (`ms`) directly under `SMALL_PERIODS_MS` and `BIG_PERIODS_MS` (converting Hz to ms transparently inside [Gen_Taskset/lib/generation_config_parser.py](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Gen_Taskset/lib/generation_config_parser.py)). In the newly created paper configs ([Gen_Taskset/task_sets_config/taskset_cfg_paper_4.json](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Gen_Taskset/task_sets_config/taskset_cfg_paper_4.json) etc.), the exact period set from the paper is explicitly configured in milliseconds.
* **Legacy File Pointer**: `Gen_Taskset/gen_taskset.py` (deleted)
* **Legacy Config Pointer**: [taskset_cfg_10_1.json](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Gen_Taskset/task_sets_config/taskset_cfg_10_1.json)

### 3. SP Weights Discrepancy
* **Claim**: The paper outlines randomly generating SP weights from 50% to 100% relative to the task periods. In the codebase, weights are assigned statically (regular tasks get `1.0`, soft/time-variant tasks get `2.0`) and then normalized such that the sum is exactly `5.0`.
* **Resolution**: The modular package maintains compatibility with the existing normalize-to-sum constraint (e.g. `SP_WEIGHTS_SUM: 5.0`), but configuration variables now support customization of this target sum.
* **Legacy File Pointer**: `Gen_Taskset/gen_taskset.py` (deleted)

### 4. SP Thresholds Discrepancy
* **Claim**: The paper specifies a discrete threshold set `[0.2, 0.4, 0.6, 0.8, 1.0]`, but the legacy codebase generated thresholds as continuous uniform values in a specified range.
* **Resolution**: In [Gen_Taskset/lib/taskset_generator.py](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Gen_Taskset/lib/taskset_generator.py), we implemented support for both discrete threshold choices (`SP_THRESHOLDS_SET`) and range-based thresholds (`SP_THRESHOLD_RANGE`). In our paper-based configurations, thresholds are selected randomly from the exact discrete set `[0.2, 0.4, 0.6, 0.8, 1.0]`.
* **Legacy File Pointer**: `Gen_Taskset/gen_taskset.py` (deleted)

---

## Instructions for Running Simulation Experiments

Follow these steps to run the simulation experiments and visualize/calculate the SP-Metric results:

### Step 1: Compile the Release Binaries
Compile the C++ simulation and analysis tools in Release mode:
```bash
mkdir -p release
cd release
cmake -DCMAKE_BUILD_TYPE=RELEASE ..
make -j4
cd ..
```
This produces `CSPSimulation_2` (for running simulations with multiple intervals and execution time traces) and `AnalyzeSP_Metric` (for calculating the SP-Metric).

### Step 2: Generate Simulated Taskset and Trajectories
Generate the taskset configurations and spatial execution time traces using GMM. For example, to generate a 10-task set with 8 simulation instances per trajectory under `TaskData/taskset_cfg_10_1_gen_1`:
```bash
python3 Gen_Taskset/executable/run_generator.py Gen_Taskset/task_sets_config/taskset_cfg_10_1.json --dir_path TaskData/taskset_cfg_10_1_gen_1 --add_perf_records --n_sec 1000 --n_path_per_task 1 --n_inst_per_path 8
```
This generates the interval task parameters `taskset_characteristics_[interval_idx].yaml` and execution time trace files `path_Et_task_[task_id]_[path_idx]_[instance_idx].txt`.

### Step 3: Run the Scheduler Simulation
Run the simulation target with the generated tasks for the desired scheduler (e.g., `INCR`, `BR`, `RM_FAST`, `RM_SLOW`, `CFS`).
```bash
cd release
./tests/CSPSimulation_2 --input_folder ../TaskData/taskset_cfg_10_1_gen_1 --output_folder ../TaskData/taskset_cfg_10_1_gen_1 --simt 1000000 --scheduler INCR --output_job_not_executed 1 --verbose 1
cd ..
```
This simulates scheduling over the specified simulation time (`--simt` in ms, e.g., `1000000` ms for `1000` seconds of trace data) and writes the scheduling results (`sim_res_[scheduler]_[instance].txt`) and logs to the folder.

### Step 4: Calculate SP-Metric and Visualize Results
Run the Python script to calculate the SP-Metric per 10-second interval (which invokes the compiled C++ `AnalyzeSP_Metric` executable) and generate summary box plots:
```bash
python Visualize_SP_Metric/ryan_draw_sim_rst.py --sim_rst_dir TaskData/taskset_cfg_10_1_gen_1 --method INCR
```
This generates:
* `sp_value_INCR_[instance].txt` containing the calculated SP-Metric values per interval.
* A box plot visualization saved to `TaskData/taskset_cfg_10_1_gen_1/sim_res_p0_INCR/box_plot_of_all_data_INCR.pdf`.
* A CSV data summary saved to `TaskData/taskset_cfg_10_1_gen_1/sim_res_p0_INCR/sp_data.csv`.

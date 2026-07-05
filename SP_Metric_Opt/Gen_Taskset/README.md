# Gen_Taskset: Modular Taskset and GMM Trace Generator

This package provides a modular tool to generate real-time multi-core tasksets with execution times modeled as dynamic Gaussian Mixture Models (GMM) varying over a generated spatial path.

---

## 1. Taskset Configuration (.json)

Scenario configurations are specified in JSON files (e.g., inside `Gen_Taskset/task_sets_config/`). Below are the primary configuration variables:

* **Task Period Partitioning**:
  * `PERIODS_MS` (list of int): The period pool every task draws from, in milliseconds (e.g. `[1000, 500, 200, 100, 50, 33, 20]`). Replaces the former Hz-based `SMALL_PERIOD_HZ` / `BIG_PERIOD_HZ` lists.
  * `N_TASKS` (int): Total number of tasks to generate (every task draws one period from `PERIODS_MS`). Replaces the former `N_SMALL_PERIOD_TASKS` / `N_BIG_PERIOD_TASKS` split.
* **Execution Time ($E_t$) and Variance**:
  * `Et_OVER_PERIOD_RANGE` (list `[min, max]`): Bounds for initial mean execution time ratio ($E_t / P$).
  * `SIGMA_OVER_Et_RANGE` (list `[min, max]`): Ratio of standard deviation ($\sigma$) over mean execution time.
  * `FINAL_Et_OVER_PERIOD_RANGE` (list `[min, max]`): Absolute physical bounds for $E_t / P$.
* **Correlations & GMM Parameters**:
  * `RO_1_Et_RANGE` (list `[min, max]`): Correlation coefficient between $X$-dimension and execution time.
  * `RO_2_Et_RANGE` (list `[min, max]`): Correlation coefficient between $Y$-dimension and execution time.
  * `N_GMM_COMPONENTS_PER_TASK` (int): Number of GMM components per task.
* **Target System Constraints**:
  * `CPU_UTIL_RANDOM_RANGE` (list `[low, high]`): Per-core utilization range; each task set samples one value uniformly (replaces the former fixed `MEAN_CPU_UTIL` scalar, removed in P13).
  * `N_CORES` (int): Number of target CPU processor cores.
  * `SP_WEIGHTS_SUM` (float): Sum of safety priority weights for SP-Metric calculation.

---

## 2. Running the Pipeline Generator

### A. Core Generator Executable
To run the generator directly for a specific configuration to generate task parameters and execution traces:

```bash
python3 Gen_Taskset/executable/run_generator.py <config_json_path> \
    --n_sec <duration_seconds> \
    --n_path_per_task <number_of_paths> \
    --n_inst_per_path <number_of_gmm_instances> \
    --dir_path <destination_folder> \
    --add_perf_records
```

**Key Arguments**:
* `config_json_path` (positional): Path to the scenario JSON configuration.
* `--n_sec` (int): Duration of generated trace paths in seconds (default: `1000`).
* `--n_path_per_task` (int): Number of spatial paths to generate (default: `1`).
* `--n_inst_per_path` (int): Number of random GMM trace instances to generate per path (default: `1`).
* `--dir_path` (str): Destination folder for output files.
* `--add_perf_records` (flag): Append performance scaling curves (used for soft tasks).

### B. Batch Simulation Runner
To generate tasksets and simulate different schedulers concurrently (INCR, CFS, BR, RM, etc.):

```bash
python3 run_sim_experiments.py --num_tasks <4|6|8>
```

When `--num_tasks` is specified, the script uses the corresponding paper config file and outputs the generated files under `simulation_experiments/experiment_<N>_tasks/`.

---

## 3. Output Directory Structure

Upon completion, the target directory will contain:

* `generator_config.json`: Copy of the configuration parameters.
* `taskset_param.yaml`: Core task parameters including periods, deadlines, core affinity, and mixture models.
* `taskset_characteristics.yaml`: Global task specifications (mu, sigma, min, max) for interval 0.
* `taskset_characteristics_{interval}.yaml`: Calculated characteristics specifically for time window `{interval}`.
* `taskset_characteristics_i{interval}_p{processor}.yaml`: Core-local subset characteristics for core `{processor}` during `{interval}`.
* `path_0.png`: Spatial trajectory plotting.
* `cpu_util.png`: Plotted expected CPU utilization over time.
* `path_Et_task_{task}_{path}_{inst}.txt`: Raw execution time traces for each task step-by-step.

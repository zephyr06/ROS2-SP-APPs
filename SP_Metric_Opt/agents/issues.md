# Unresolved Issues in Gen_Taskset

This file documents the issues identified in the taskset generation codebase that remain unresolved.

---

## 1. Generation Pipeline Crash on Long-Period Tasks
* **Location**: [orchestrator.py: L168-L179](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Gen_Taskset/lib/orchestrator.py#L168-L179)
* **Description**: To compute interval statistics, the trace generator requires at least 2 job samples of every task in each interval:
  ```python
  if len(interval_ets) >= 2:
      # compute mean, std...
  else:
      ok = 0
      break
  ```
* **Impact**: If a user configures a task with a long period (e.g., $\ge 5000\text{ ms}$) and runs the default interval check with a $10\text{ s}$ update interval, the number of executions per interval will drop below 2. This sets `ok = 0`, causing the generator to abort early with an error.

---

## 2. Dynamic Task Deadlines across Intervals (Inconsistency)
* **Location**: [yaml_exporter.py: L76](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Gen_Taskset/lib/yaml_exporter.py#L76)
* **Description**: Task deadlines are not stored in the base `taskset_param.yaml`. Instead, they are generated on-the-fly via `random.uniform(0.5, 1.0)` inside `convert_taskset_parameters_to_cpp_yaml`.
* **Impact**: The same task will have different, randomized deadlines in `taskset_characteristics_0.yaml`, `taskset_characteristics_1.yaml`, etc., which violates the real-time assumption of static task sets and makes optimization unstable.

---

## 3. Ignored `SP_THRESHOLD_RANGE` due to Default Fallback Values
* **Location**: [taskset_generator.py: L208-L218](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Gen_Taskset/lib/taskset_generator.py#L208-L218)
* **Description**: In `generate_taskset_parameters`, `sp_thresholds_set` is resolved using `cfgs.get("SP_THRESHOLDS_SET", [0.2, 0.4, 0.6, 0.8, 1.0])`.
* **Impact**: Since it defaults to a non-empty list, the condition `if sp_thresholds_set:` is always `True`. Therefore, the `else` branch (which selects from `SP_THRESHOLD_RANGE` uniformly) is never reached, even if the user explicitly configured `SP_THRESHOLD_RANGE` and omitted `SP_THRESHOLDS_SET`.

---

## 4. Actual CPU Utilization Deficit at High Workloads
* **Location**: [gmm_model.py: L112-L122](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Gen_Taskset/lib/gmm_model.py#L112-L122)
* **Description**: Capping during sampling restricts task execution times to a maximum of `0.9 * period`. If scaling sets the mean execution time `Et_mean` of a task close to or above this limit, the actual execution times will be heavily capped.
* **Impact**: The actual simulated average CPU utilization will be lower than the target `MEAN_CPU_UTIL` specified in the configuration.

---

## 5. Specification Test Failures due to Default UUniFast Enablement
* **Location**: [taskset_generator.py: L248](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Gen_Taskset/lib/taskset_generator.py#L248) and [test_specifications.py: L219](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Gen_Taskset/tests/test_specifications.py#L219)
* **Description**: By default, `use_uunifast = cfgs.get("USE_UUNIFAST", True)` is active. This forces legacy configurations (which do not specify `USE_UUNIFAST`) to generate task sets using UUniFast. 
* **Impact**: UUniFast can distribute utilizations that fall outside the legacy boundary check `[0.05 * period, 0.9 * period]` checked in `test_specifications.py`. Since legacy configs do not cap UUniFast to these bounds, testing them yields `AssertionError` (e.g. `assert 112.232 < (100 + 0.0001)` or `assert 23.749 < (20 + 0.0001)`).

---

## 6. Lack of Configuration Separation for Different Task Types
* **Location**: [generation_config_parser.py](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Gen_Taskset/lib/generation_config_parser.py)
* **Description**: The configuration JSON schema treats parameters globally for all tasks (e.g., a single global `SIGMA_OVER_Et_RANGE`, `RO_1_Et_RANGE`, `RO_2_Et_RANGE` range). There is no mechanism in the schema to define distinct parameter ranges or structures specifically for environment-dependent vs. environment-independent tasks, or big-period vs. small-period tasks.
* **Impact**: Restricts flexibility to generate task sets with mixed behaviors where different task types follow different stochastic or spatial patterns.

---

## 7. UUniFast Fallback Scaling Bug (Exceeds Utilization Cap / Un-schedulable Tasks)
* **Location**: [taskset_generator.py: L51-L59](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Gen_Taskset/lib/taskset_generator.py#L51-L59)
* **Description**: If the random generation fails to meet the cap after 100 tries, the fallback logic caps the elements at `max_util_cap` and scales them up by `target_util / current_sum` to preserve the target total utilization.
* **Bug**: The scaling factor is not constrained. Scaling up can push individual task utilizations above `max_util_cap` (and even above `1.0`), violating the core schedulability constraint of real-time systems and causing execution times to exceed task periods (`assert et < period` fails in `test_specifications.py`).

---

## 8. Coupling Mismatch between `env_dependent` and Performance Records (`perf_sel`)
* **Location**: [yaml_exporter.py: L28-L49](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Gen_Taskset/lib/yaml_exporter.py#L28-L49)
* **Description**: Performance records selection (`perf_sel`) is chosen randomly among tasks with `period >= MIN_PERIOD_WITH_PERFORMANCE_RECORDS`. It completely ignores the `env_dependent` attribute of the tasks.
* **Impact**: A task can be allocated performance records (`sp_weight = 2.0`) despite being completely static (not env-dependent), and an environment-dependent task can be left without performance records. Performance records should be coupled to environment-dependent tasks.

---

## 9. Exporter Unaware of Randomized `N_ENV_DEPENDENT_TASKS`
* **Location**: [yaml_exporter.py: L28](file:///home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/Gen_Taskset/lib/yaml_exporter.py#L28)
* **Description**: The exporter resolves the performance record count using `g_sel_n_perf_record_task = cfgs.get("N_ENV_DEPENDENT_TASKS", 2)`.
* **Impact**: When `N_ENV_DEPENDENT_TASKS` is omitted from the configuration, `taskset_generator.py` randomizes it internally (`random.randint(1, n_tasks)`). However, since this randomized value is not saved back into the `cfgs` dictionary, the exporter falls back to the default of `2` rather than aligning with the actual number of env-dependent tasks generated.
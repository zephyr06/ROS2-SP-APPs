# Simulation Experiment & Code Maintenance Plan

This document outlines the three major tasks to complete and verify the safety-performance scheduling simulation experiments.

---

## Task 1: Comprehensive C++ Test Suite for Core Optimization & Scheduling (High Priority)
* **Goal**: Maximize test coverage and verify the mathematical and logical correctness of all optimization, scheduling, and RTA routines.
* **Component Coverage**:
  * **Priority Optimizers**: Verify `OptimizePA_Incre`, `OptimizePA_BF`, `OptimizePA_Incre_with_TimeLimits`, and `OptimizePA_TL_BF`. Check that priority order permutations are mathematically correct and minimize SP loss.
  * **Response Time Analysis (RTA)**: Verify `GetRTA_OneTask`, `ProbabilisticRTA_TaskSet_SingleCore`, and `ProbabilisticRTA_TaskSet`. Cover edge cases like zero execution time variance, multi-core processor partitions, and utilization overloads.
  * **Simulation Routines**: Test scheduler simulators (`SimulatedCSP_SingleCore_CSP`, `SimulatedCSP_SingleCore_RM`, `SimulatedCSP_SingleCore_CFS`) under deterministic task releases.
  * **Google Test Integration**: Add both fine-grained unit tests and system-level integration tests in `tests/`.
* **Sub-Tasks Checklist**:
  - `[ ]` Design and add unit tests for `GetRTA_OneTask` with deterministic, normal, and uniform distributions.
  - `[ ]` Add unit tests for priority assignment algorithms under varying core allocations and weights.
  - `[ ]` Add integration tests verifying scheduler execution output trace formats.

---

## Task 2: Automated Simulation Experiment and Analysis Pipeline
* **Goal**: Establish a reproducible, modular workflow to generate datasets, execute multiple scheduler simulations, calculate metric values, and aggregate comparison results automatically.
* **Workflow Architecture**:
  1. **Taskset Generator (`run_generator.py`)**: Generates configurations and spatial traces for tasks.
  2. **Batch Simulation Runner (`run_experiments.py`)**: Iterates through each generated task set, runs `CSPSimulation_2` for all target schedulers (`INCR`, `BR`, `RM_FAST`, `RM_SLOW`, `CFS`), and saves intermediate results (e.g. response time traces, scheduler logs) systematically to preserve debug context.
  3. **SP-Metric Evaluator & Plotter**: Invokes `AnalyzeSP_Metric` on the output folders, compiles a unified comparison table `comparison_summary.csv`, and draws side-by-side comparison charts `comparison_plots.png` comparing SP values and deadline miss rates across all schedulers.
* **Sub-Tasks Checklist**:
  - `[x]` Design and implement `Visualize_SP_Metric/run_sim_experiments.py` orchestrator script.
  - `[x]` Implement robust structured directory saving for debug trace outputs.
  - `[x]` Write side-by-side plotting utilities.
  - `[x]` Verify the entire simulation pipeline end-to-end on 4-task and 6-task configurations.

---

## Task 3: Paper Alignment & Documentation Improvement
* **Goal**: Align simulation configurations and behaviors with paper claims, and suggest edits to improve paper writing quality.
* **Constraints**:
  * Keep C++ simulation and Python generation configurations strictly aligned with `paper_sections/full_paper_sections/section14_simu_exp.tex`.
  * Highlight and suggest corrections for low-quality text, typos, or unclear formulations in the LaTeX source files.
  * **CRITICAL REQUIREMENT**: Do not edit any `.tex` files directly without obtaining explicit user approval.
* **Sub-Tasks Checklist**:
  - `[ ]` Review LaTeX documents for typos and clarity.
  - `[ ]` Prepare a list of recommended edits for paper improvements.
  - `[ ]` Validate config keys and execution parameter ranges against final paper values.

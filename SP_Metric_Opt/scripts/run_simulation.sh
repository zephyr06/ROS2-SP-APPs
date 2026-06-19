#!/bin/bash

# ==============================================================================
# Simulation Experiment Configuration
# ==============================================================================
# Global parameter: Number of tasks per taskset (Options: 4, 6, 8)
NUM_TASKS=${NUM_TASKS:-6}

# Number of random tasksets to generate and simulate
NUM_TASKSETS=${NUM_TASKSETS:-5}

# Number of trace instances per path to simulate
NUM_INSTANCES=${NUM_INSTANCES:-8}

# Simulation execution time in ms (1,000,000 ms = 1000 seconds)
SIM_TIME=${SIM_TIME:-1000000}

# Schedulers to run and compare
SCHEDULERS=${SCHEDULERS:-"INCR INCR_SWAP BR RM_FAST RM_SLOW CFS"}

# Build directory (release / build)
BIN_DIR=${BIN_DIR:-"release"}

# Verbosity Level (0: Silent, 1: Normal progress logs, 2: Debug/Full C++ simulator output)
VERBOSE=${VERBOSE:-1}
# ==============================================================================

# Get the directory of this script and resolve the project root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( dirname "$SCRIPT_DIR" )"

echo "================================================================================"
echo "Starting End-to-End Simulation Experiments"
echo "Project Root: $PROJECT_ROOT"
echo "Number of Tasks: $NUM_TASKS"
echo "Number of Tasksets: $NUM_TASKSETS"
echo "Number of Instances: $NUM_INSTANCES"
echo "Simulation Time (ms): $SIM_TIME"
echo "Verbosity Level: $VERBOSE"
echo "================================================================================"

# Execute the simulation experiments python runner
python3 "$PROJECT_ROOT/run_sim_experiments.py" \
  --num_tasks "$NUM_TASKS" \
  --n_tasksets "$NUM_TASKSETS" \
  --n_inst "$NUM_INSTANCES" \
  --simt "$SIM_TIME" \
  --schedulers $SCHEDULERS \
  --bin_dir "$BIN_DIR" \
  --verbose "$VERBOSE"

echo "================================================================================"
echo "Simulation Run Completed!"
echo "Results and final visualizations saved under: $PROJECT_ROOT/TaskData/experiment_${NUM_TASKS}_tasks/"
echo "Summary statistics file: $PROJECT_ROOT/TaskData/experiment_${NUM_TASKS}_tasks/comparison_summary.csv"
echo "Comparison plot: $PROJECT_ROOT/TaskData/experiment_${NUM_TASKS}_tasks/comparison_plots.png"
echo "================================================================================"

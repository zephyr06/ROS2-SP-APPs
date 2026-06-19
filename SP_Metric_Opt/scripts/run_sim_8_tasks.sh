#!/bin/bash

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Run the simulation with 8 tasks
NUM_TASKS=8 "$SCRIPT_DIR/run_simulation.sh"

#!/bin/bash

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Run the simulation with 6 tasks
NUM_TASKS=6 "$SCRIPT_DIR/run_simulation.sh"

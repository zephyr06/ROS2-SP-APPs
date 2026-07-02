#!/bin/bash
# Run the trigger-interval sweep experiment and generate Figure 2.
#
# Usage:
#   ./run_interval_sweep.sh
#   MODE=prod ./run_interval_sweep.sh
#   NUM_TASKS=6 NUM_TASKSETS=10 ./run_interval_sweep.sh
#
# Environment variables override defaults just like run_simulation.sh.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

MODE="${MODE:-test}"
OUTPUT_PARENT="${OUTPUT_PARENT:-simulation_experiments/optimizer_comparison}"
PYTHON="${PYTHON:-python3}"

print_header "Interval Sweep" "Mode:         ${MODE}"

cd "${PROJECT_ROOT}"
"${PYTHON}" -m simulation_experiments.interval_sweep \
    --mode "${MODE}" \
    --output_parent "${OUTPUT_PARENT}"

print_footer

#!/bin/bash
# Generate all paper figures from existing experiment data.
#
# This script does NOT run simulations. It only produces figures using data
# already present in simulation_experiments/optimizer_comparison/. Run
# run_end_to_end.sh (or run_simulation.sh + run_interval_sweep.sh) first to
# produce that data, including the interval-sweep results that feed Figure 2.
#
# Usage:
#   ./run_paper_figures.sh
#   MODE=prod ./run_paper_figures.sh
#
# Figures produced:
#   Fig 1A-1F: Cross-task-count and distribution figures
#   Fig 3:     Important-task miss rate
# (Figure 2 is rendered by run_interval_sweep.sh, which produces the sweep
# data it plots -- it is not a figures-only step.)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

MODE="${MODE:-test}"
OUTPUT_PARENT="${OUTPUT_PARENT:-simulation_experiments/optimizer_comparison}"
PYTHON="${PYTHON:-python3}"

print_header "Paper Figures Generation" \
    "Mode:         ${MODE}" \
    "Output:       ${OUTPUT_PARENT}/figures"

cd "${PROJECT_ROOT}"

# --- Cross-task-count aggregation (Fig 1A-1F, Fig 3) ---
# aggregate_across_tasks reads existing comparison_summary.csv files and emits
# the cross-task scaling figures plus the SP boxplot and miss-rate figure.
echo ">>> Generating Figures 1A-1F and Figure 3 ..."
"${PYTHON}" -m simulation_experiments.aggregate_across_tasks \
    --mode "${MODE}"

print_footer "Output directory: ${PROJECT_ROOT}/${OUTPUT_PARENT}/figures"

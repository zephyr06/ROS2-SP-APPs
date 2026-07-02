#!/bin/bash
# Run the full simulation-experiment pipeline end-to-end from one config file.
#
# This is the single entry point: it loads configs/experiment_config.json and
# runs all three stages (simulate -> sweep -> aggregate) in order, so you do
# not have to run steps 1/2/3 by hand. Every experiment parameter comes from
# the JSON config; the only knobs here select mode / stages / verbosity.
#
# Usage:
#   ./run_end_to_end.sh                      # test mode, all stages
#   MODE=prod ./run_end_to_end.sh            # paper-grade, all stages
#   STEPS="simulate aggregate" ./run_end_to_end.sh   # skip the sweep
#   DRY_RUN=1 ./run_end_to_end.sh            # print commands, run nothing
#   BIN_DIR=build ./run_end_to_end.sh        # use a non-release binary
#
# Environment variables (all optional):
#   MODE         - test | prod (default: test)
#   STEPS        - space-separated subset of: simulate sweep aggregate
#                  (default: all three, in fixed order)
#   BIN_DIR      - directory holding the C++ binaries (default: release)
#   VERBOSE      - 0 | 1 | 2 forwarded to all stages (default: 1)
#   DRY_RUN      - set to "1" to print commands without executing
#   PYTHON       - python interpreter (default: python3)
#   CONFIG_JSON  - path to an experiment config JSON (default: the shipped
#                  configs/experiment_config.json). Point this at an alternate
#                  config to scope the run -- e.g. the simulation-only config
#                  replaces the former run_simulation.sh:
#                      CONFIG_JSON=simulation_experiments/configs/simulation_only_config.json \
#                          STEPS=simulate MODE=prod ./run_end_to_end.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

MODE="${MODE:-test}"
STEPS="${STEPS:-}"
BIN_DIR="${BIN_DIR:-release}"
VERBOSE="${VERBOSE:-1}"
DRY_RUN="${DRY_RUN:-0}"
PYTHON="${PYTHON:-python3}"
CONFIG_JSON="${CONFIG_JSON:-}"

SIM_BIN="${PROJECT_ROOT}/${BIN_DIR}/tests/RunOrchestrator"

# --- Header ---
print_header "End-to-End Pipeline" \
    "Mode:         ${MODE}" \
    "Steps:        ${STEPS:-(all: simulate sweep aggregate)}" \
    "Binary:       ${SIM_BIN}" \
    "Verbose:      ${VERBOSE}" \
    "$( [[ "${DRY_RUN}" == "1" ]] && echo "Dry run:      YES (commands printed, nothing executed)" )"

# --- Validation (skip for dry-run, since nothing runs) ---
if [[ "${DRY_RUN}" != "1" ]]; then
    require_binary "${SIM_BIN}"
fi

register_signal_trap

# --- Build Python command ---
CMD=(
    "${PYTHON}" -m simulation_experiments.run_end_to_end_experiments
    --mode "${MODE}"
    --bin_dir "${BIN_DIR}"
    --verbose "${VERBOSE}"
)

if [[ -n "${STEPS}" ]]; then
    CMD+=(--steps ${STEPS})
fi
if [[ -n "${CONFIG_JSON}" ]]; then
    CMD+=(--config_json "${CONFIG_JSON}")
fi
if [[ "${DRY_RUN}" == "1" ]]; then
    CMD+=(--dry_run)
fi

# --- Run ---
cd "${PROJECT_ROOT}"
"${CMD[@]}"

print_footer "Figures: ${PROJECT_ROOT}/simulation_experiments/optimizer_comparison/runs"

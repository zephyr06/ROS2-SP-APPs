#!/bin/bash
# Run the full simulation-experiment pipeline end-to-end from one config file.
#
# This is the single entry point: it loads the configured paper_simulation_config.json
# and runs all three stages in fixed order -- simulate -> sweep -> aggregate --
# so you do not have to run steps 1/2/3 by hand. Every experiment parameter
# comes from the JSON config; the only knobs here select mode / verbosity.
#
# Stages always run together (in order); there is no stage-selection knob:
#   1. simulate  - per-task-count simulations (writes tasks{N}_dur{D}_.../)
#   2. sweep     - trigger-interval sweep (Fig 2)
#   3. aggregate - cross-task figures (Figs 1A-1F, 3) under runs/<run_id>/figures/
#
# Usage:
#   ./run_simulation_and_plot_figures.sh                      # test mode, all stages
#   MODE=prod ./run_simulation_and_plot_figures.sh            # paper-grade, all stages
#   DRY_RUN=1 ./run_simulation_and_plot_figures.sh            # print commands, run nothing
#   BIN_DIR=build ./run_simulation_and_plot_figures.sh        # use a non-release binary
#
# Environment variables (all optional):
#   MODE         - test | prod (default: test)
#   BIN_DIR      - directory holding the C++ binaries (default: release)
#   VERBOSE      - 0 | 1 | 2 forwarded to all stages (default: 1)
#   DRY_RUN      - set to "1" to print commands without executing
#   PYTHON       - python interpreter (default: python3)
#   CONFIG_JSON  - path to an experiment config JSON (default: the shipped
#                  configs/paper_simulation_config.json). Point this at an alternate
#                  config to scope the run -- e.g.:
#                      CONFIG_JSON=simulation_experiments/configs/incr_et_profiling.json \
#                          ./run_simulation_and_plot_figures.sh
#   RERUN_MODE   - reuse (default) | clear_all | clear_results
#                  'clear_all' wipes <run_root>/sim/ (generated tasksets +
#                  per-scheduler results + sweep variants) before the stages
#                  run, so everything regenerates from scratch. 'reuse' keeps
#                  existing artifacts and lets each stage's own reuse/resume
#                  guards decide. 'clear_results' keeps the generated tasksets
#                  and wipes ONLY the per-scheduler result subdirs, so a fresh
#                  simulate re-runs the binary against the reused tasksets
#                  (A/B two binaries on identical tasksets without regeneration).
#
# parameters.yaml TIME_LIMIT overwrite:
#   The C++ binary reads TIME_LIMIT from sources/parameters.yaml at STATIC INIT
#   (Parameters.cpp: YAML::LoadFile runs before main()), so the file must carry
#   the configured value before the binary process starts. This script delegates
#   the edit to scripts/lib/patch_time_limit.py: it reads time_limit_seconds from the
#   active config/mode (default 1s), backs up sources/parameters.yaml, patches
#   the TIME_LIMIT line, and this script restores the original file on exit
#   (normal / error / signal). Skipped on DRY_RUN. time_limit_seconds bounds ONE
#   optimizer call (the per-activation budget a single
#   EnumeratePA_with_TimeLimits / OptimizeIncre call runs against), NOT the task
#   time-limits themselves.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

MODE="${MODE:-test}"
BIN_DIR="${BIN_DIR:-release}"
VERBOSE="${VERBOSE:-1}"
DRY_RUN="${DRY_RUN:-0}"
PYTHON="${PYTHON:-python3}"
CONFIG_JSON="${CONFIG_JSON:-}"
RERUN_MODE="${RERUN_MODE:-reuse}"

SIM_BIN="${PROJECT_ROOT}/${BIN_DIR}/tests/RunOrchestrator"

# --- Header ---
print_header "End-to-End Pipeline" \
    "Mode:         ${MODE}" \
    "Stages:       simulate -> sweep -> aggregate (fixed order)" \
    "Binary:       ${SIM_BIN}" \
    "Verbose:      ${VERBOSE}" \
    "Rerun mode:   ${RERUN_MODE}" \
    "$( [[ "${DRY_RUN}" == "1" ]] && echo "Dry run:      YES (commands printed, nothing executed)" )"

# --- Build the release binaries first (skip for dry-run) ---
# Rebuilds the C++ binaries in ${BIN_DIR} so the pipeline runs against the
# current source. Dry-run skips both the build and the binary check below.
if [[ "${DRY_RUN}" != "1" ]]; then
    print_header "Building ${BIN_DIR}/" "make -j6 in ${PROJECT_ROOT}/${BIN_DIR}"
    cd "${PROJECT_ROOT}/${BIN_DIR}" && make -j6 || {
        echo "ERROR: build failed in ${PROJECT_ROOT}/${BIN_DIR}" >&2
        exit 1
    }
    cd "${PROJECT_ROOT}"
fi

# --- Validation (skip for dry-run, since nothing runs) ---
if [[ "${DRY_RUN}" != "1" ]]; then
    require_binary "${SIM_BIN}"
fi

register_signal_trap

# --- Overwrite sources/parameters.yaml TIME_LIMIT from the config ---
# Delegates the YAML edit to scripts/lib/patch_time_limit.py (readable Python, not
# an inline awk/JSON one-liner). The helper backs up the original, patches only
# the first 'TIME_LIMIT:' line (preserving the trailing comment), and prints the
# backup path as its final stdout line -- captured here so the EXIT trap can
# restore the original on every exit path (normal / error / signal). The C++
# binary reads TIME_LIMIT at static init (before main()), so the patch must land
# before the pipeline spawns the binary. No-op on DRY_RUN (nothing runs, nothing
# is patched).
PARAMS_YAML="${PROJECT_ROOT}/sources/parameters.yaml"
_PARAMS_BACKUP=""

restore_params_yaml() {
    if [[ -n "${_PARAMS_BACKUP}" ]]; then
        # Pass the captured backup path explicitly (positional: PARAMS_YAML BACKUP)
        # so restore is not dependent on the sibling-search fallback.
        "${PYTHON}" "${SCRIPT_DIR}/lib/patch_time_limit.py" restore \
            "${PARAMS_YAML}" "${_PARAMS_BACKUP}" >/dev/null 2>&1 || true
        _PARAMS_BACKUP=""
    fi
}
trap restore_params_yaml EXIT

if [[ "${DRY_RUN}" != "1" ]]; then
    if [[ ! -f "${PARAMS_YAML}" ]]; then
        echo "ERROR: ${PARAMS_YAML} not found -- cannot set TIME_LIMIT." >&2
        exit 2
    fi
    # Resolve the config path the pipeline will use: the --config_json default
    # in run_end_to_end_experiments.py is the shipped paper_simulation_config,
    # so fall back to it when CONFIG_JSON is unset.
    _RESOLVED_CONFIG="${CONFIG_JSON:-${PROJECT_ROOT}/simulation_experiments/configs/paper_simulation_config.json}"
    if [[ ! -f "${_RESOLVED_CONFIG}" ]]; then
        echo "ERROR: config not found: ${_RESOLVED_CONFIG}" >&2
        exit 2
    fi
    # `patch` reads time_limit_seconds from <MODE>_mode, backs up parameters.yaml,
    # patches the TIME_LIMIT line, and prints the backup path as its FINAL line.
    # Capture every line (status echo + backup path) but keep only the last for
    # the restore trap; fail loudly (exit 2) if the helper rejects the contract.
    _PATCH_OUT="$("${PYTHON}" "${SCRIPT_DIR}/lib/patch_time_limit.py" patch \
        "${_RESOLVED_CONFIG}" "${MODE}" "${PARAMS_YAML}")" || {
        echo "${_PATCH_OUT}" >&2
        echo "ERROR: patch_time_limit.py patch failed." >&2
        exit 2
    }
    _PARAMS_BACKUP="$(printf '%s\n' "${_PATCH_OUT}" | tail -n 1)"
fi

# --- Build Python command ---
CMD=(
    "${PYTHON}" -m simulation_experiments.run_end_to_end_experiments
    --mode "${MODE}"
    --bin_dir "${BIN_DIR}"
    --verbose "${VERBOSE}"
)

if [[ -n "${CONFIG_JSON}" ]]; then
    CMD+=(--config_json "${CONFIG_JSON}")
fi
if [[ "${RERUN_MODE}" != "reuse" ]]; then
    CMD+=(--rerun_mode "${RERUN_MODE}")
fi
if [[ "${DRY_RUN}" == "1" ]]; then
    CMD+=(--dry_run)
fi

# --- Run ---
cd "${PROJECT_ROOT}"
"${CMD[@]}"

print_footer "Figures: ${PROJECT_ROOT}/simulation_experiments/optimizer_comparison/runs"

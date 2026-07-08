#!/bin/bash
# Project evaluation suite: run the pipeline, then evaluate the north-star gates.
#
# This is the single entry point for the "did this change help?" dashboard. It
# runs run_end_to_end.sh with the evaluation-suite config (which produces the
# comparison_summary.csv files the suite reads), then invokes
# simulation_experiments.evaluation_suite.py to evaluate the five north-star
# gates (Q1-Q3 SP-quality, E1 overhead, E2 INCR<=SCRATCH ET) into a single
# PASS/FAIL verdict. Exit code is 0 only if every gate PASSES, so CI / a shell
# gate can act on it.
#
# Usage:
#   ./scripts/run_evaluation_suite.sh            # test mode (fast smoke, ~1 min)
#   MODE=prod ./scripts/run_evaluation_suite.sh  # full gate run (~15-20 min)
#   SKIP_PIPELINE=1 ./scripts/run_evaluation_suite.sh   # only re-evaluate an
#                                                       # already-completed run
#   DRY_RUN=1 ./scripts/run_evaluation_suite.sh  # print commands, run nothing
#
# Environment variables (all optional):
#   MODE            - test | prod (default: test)
#   SKIP_PIPELINE   - set to "1" to skip run_end_to_end.sh and only re-run the
#                     evaluator against the run root from the config (use after a
#                     successful pipeline run, to re-render the verdict)
#   CONFIG_JSON     - override the eval config (default: the shipped
#                     configs/evaluation_suite_config.json)
#   OVERHEAD_N      - override the E1 overhead probe N (default: from config)
#   DRY_RUN         - set to "1" to print commands without executing
#   PYTHON          - python interpreter (default: python3)
#   BIN_DIR, VERBOSE- forwarded to run_end_to_end.sh when the pipeline runs

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

MODE="${MODE:-test}"
SKIP_PIPELINE="${SKIP_PIPELINE:-0}"
DRY_RUN="${DRY_RUN:-0}"
PYTHON="${PYTHON:-python3}"
CONFIG_JSON="${CONFIG_JSON:-${PROJECT_ROOT}/simulation_experiments/configs/evaluation_suite_config.json}"

# --- Header ---
if [[ "${SKIP_PIPELINE}" == "1" ]]; then
    stage_line="Stages:       evaluate only (pipeline skipped)"
else
    stage_line="Stages:       run_end_to_end.sh -> evaluate north-star gates"
fi
print_header "Project Evaluation Suite" \
    "Mode:         ${MODE}" \
    "${stage_line}" \
    "Config:       ${CONFIG_JSON}" \
    "$( [[ "${DRY_RUN}" == "1" ]] && echo "Dry run:      YES (commands printed, nothing executed)" )"

# --- Validate config exists ---
if [[ ! -f "${CONFIG_JSON}" ]]; then
    echo "ERROR: eval config not found: ${CONFIG_JSON}" >&2
    exit 2
fi

# Locate the e2e wrapper this script delegates to (for the pipeline stage).
E2E_WRAPPER="${SCRIPT_DIR}/run_end_to_end.sh"

cd "${PROJECT_ROOT}"

# --- Stage 1: run the simulation pipeline (unless skipped / dry-run) ---
if [[ "${SKIP_PIPELINE}" != "1" ]]; then
    if [[ "${DRY_RUN}" == "1" ]]; then
        echo "[dry-run] MODE=${MODE} CONFIG_JSON=${CONFIG_JSON} BIN_DIR=${BIN_DIR:-release} ${E2E_WRAPPER}"
    else
        export MODE
        export CONFIG_JSON
        BIN_DIR="${BIN_DIR:-release}" "${E2E_WRAPPER}" || {
            echo "ERROR: pipeline stage failed; not running the evaluator." >&2
            exit 1
        }
    fi
fi

# --- Stage 2: evaluate the north-star gates ---
EVAL_CMD=(
    "${PYTHON}" -m simulation_experiments.evaluation_suite
    --config_json "${CONFIG_JSON}"
    --mode "${MODE}"
)
if [[ -n "${OVERHEAD_N:-}" ]]; then
    EVAL_CMD+=(--overhead_n "${OVERHEAD_N}")
fi

if [[ "${DRY_RUN}" == "1" ]]; then
    echo "[dry-run] ${EVAL_CMD[*]}"
    print_footer "Dry run complete -- nothing executed."
    exit 0
fi

"${EVAL_CMD[@]}"
rc=$?

print_footer "Verdict: $([[ ${rc} -eq 0 ]] && echo 'PASS' || echo 'FAIL') (exit ${rc})"
exit ${rc}

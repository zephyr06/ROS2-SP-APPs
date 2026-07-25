#!/bin/bash
# The single entry point: run the full simulation pipeline from one config JSON,
# then evaluate the north-star gates into a PASS/FAIL verdict.
#
# This is the ONLY .sh entry point (since the P2.8 one-.sh consolidation folded
# the former run_simulation_and_plot_figures.sh pipeline body in here and deleted
# the redundant figures-only run_paper_figures.sh). Experiments differ by which
# config JSON you point CONFIG_JSON at -- you do NOT add new .sh files for new
# experiments.
#
# Stages (in fixed order):
#   1. build    - compile the C++ binaries in BIN_DIR (skipped when SKIP_PIPELINE=1)
#   2. patch    - overwrite sources/parameters.yaml TIME_LIMIT from the config's
#                 time_limit_seconds (backed up + restored on every exit path)
#   3. simulate - per-task-count simulations (writes tasks{N}_dur{D}_.../)
#   4. sweep    - trigger-interval sweep (Fig 2)
#   5. aggregate- cross-task figures (Figs 1A-1F, 3) under runs/<run_id>/figures/
#   6. evaluate - the five north-star gates (Q1-Q3 SP-quality, E1 overhead,
#                 E3 INCR_Reopt_X period-monotonicity) into one PASS/FAIL verdict
# Stages 3-5 are run_end_to_end_experiments' fixed simulate->sweep->aggregate;
# stage 6 is evaluation_suite.py. Exit code is 0 only if every gate PASSES, so
# CI / a shell gate can act on it.
#
# Both the pipeline (stages 1-5) and the evaluator (stage 6) read the SAME config
# JSON (configs/paper_simulation_config.json by default -- the single config
# since P2.8 folded the eval_* keys in and deleted the dedicated gate-eval
# config). The pipeline ignores the eval_* keys; evaluation_suite.py reads them.
#
# Usage:
#   ./scripts/run_simulation_plot_eval_ns.sh            # test mode: pipeline + eval (~1 min)
#   MODE=prod ./scripts/run_simulation_plot_eval_ns.sh  # full paper-grade + gate run
#   SKIP_EVAL=1 ./scripts/run_simulation_plot_eval_ns.sh   # pipeline only (all figures,
#                                                          # no north-star verdict)
#   SKIP_PIPELINE=1 ./scripts/run_simulation_plot_eval_ns.sh  # eval only: re-evaluate an
#                                                             # already-completed run (no build)
#   DRY_RUN=1 ./scripts/run_simulation_plot_eval_ns.sh  # print commands, run nothing
#
# Environment variables (all optional):
#   MODE         - test | prod (default: test)
#   BIN_DIR      - directory holding the C++ binaries (default: release)
#   VERBOSE      - 0 | 1 | 2 forwarded to the pipeline (default: 1)
#   DRY_RUN      - set to "1" to print commands without executing
#   PYTHON       - python interpreter (default: python3)
#   CONFIG_JSON  - path to an experiment config JSON (default: the shipped
#                  configs/paper_simulation_config.json). Point this at an alternate
#                  config to scope the run -- e.g.:
#                      CONFIG_JSON=simulation_experiments/configs/incr_et_profiling.json \
#                          ./scripts/run_simulation_plot_eval_ns.sh
#   RERUN_MODE   - reuse (default) | clear_all | clear_results
#                  'clear_all' wipes <run_root>/sim/ (generated tasksets +
#                  per-scheduler results + sweep variants) before the stages
#                  run, so everything regenerates from scratch. 'reuse' keeps
#                  existing artifacts and lets each stage's own reuse/resume
#                  guards decide. 'clear_results' keeps the generated tasksets
#                  and wipes ONLY the per-scheduler result subdirs, so a fresh
#                  simulate re-runs the binary against the reused tasksets
#                  (A/B two binaries on identical tasksets without regeneration).
#   SKIP_PIPELINE- set to "1" to skip stages 1-5 (build + patch + simulate/sweep/
#                  aggregate) and ONLY re-run the evaluator against the run root
#                  from the config (use after a successful pipeline run, to
#                  re-render the verdict). Needs no build.
#   SKIP_EVAL    - set to "1" to skip stage 6 (the north-star evaluation) and
#                  stop after the pipeline produces figures. The figures-only
#                  path (replaces the deleted run_paper_figures.sh). Mutually
#                  exclusive with SKIP_PIPELINE (both set is an error -- you'd
#                  run nothing).
#   OVERHEAD_N   - override the E1 overhead probe N (default: from config)
#
# parameters.yaml TIME_LIMIT overwrite:
#   The C++ binary reads TIME_LIMIT from sources/parameters.yaml at STATIC INIT
#   (Parameters.cpp: YAML::LoadFile runs before main()), so the file must carry
#   the configured value before the binary process starts. This script delegates
#   the edit to scripts/lib/patch_time_limit.py: it reads time_limit_seconds from the
#   active config/mode (default 1s), backs up sources/parameters.yaml, patches
#   the TIME_LIMIT line, and this script restores the original file on exit
#   (normal / error / signal). Skipped on DRY_RUN and when SKIP_PIPELINE=1 (no
#   pipeline runs, so the binary never starts). time_limit_seconds bounds ONE
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
CONFIG_JSON="${CONFIG_JSON:-${PROJECT_ROOT}/simulation_experiments/configs/paper_simulation_config.json}"
RERUN_MODE="${RERUN_MODE:-reuse}"
SKIP_PIPELINE="${SKIP_PIPELINE:-0}"
SKIP_EVAL="${SKIP_EVAL:-0}"
OVERHEAD_N="${OVERHEAD_N:-}"

SIM_BIN="${PROJECT_ROOT}/${BIN_DIR}/tests/RunOrchestrator"

# SKIP_PIPELINE + SKIP_EVAL both set means run nothing -- fail loudly rather than
# silently exit 0 (a degenerate config must not masquerade as a green run).
if [[ "${SKIP_PIPELINE}" == "1" && "${SKIP_EVAL}" == "1" ]]; then
    echo "ERROR: SKIP_PIPELINE=1 and SKIP_EVAL=1 are mutually exclusive (you'd run nothing)." >&2
    exit 2
fi

# --- Header ---
if [[ "${SKIP_PIPELINE}" == "1" ]]; then
    stage_line="Stages:       evaluate only (pipeline skipped)"
elif [[ "${SKIP_EVAL}" == "1" ]]; then
    stage_line="Stages:       build -> patch -> simulate -> sweep -> aggregate (eval skipped)"
else
    stage_line="Stages:       build -> patch -> simulate -> sweep -> aggregate -> evaluate"
fi
print_header "Simulation Pipeline + North-Star Evaluation" \
    "Mode:         ${MODE}" \
    "${stage_line}" \
    "Binary:       ${SIM_BIN}" \
    "Config:       ${CONFIG_JSON}" \
    "Verbose:      ${VERBOSE}" \
    "Rerun mode:   ${RERUN_MODE}" \
    "$( [[ "${DRY_RUN}" == "1" ]] && echo "Dry run:      YES (commands printed, nothing executed)" )"

# --- Validate config exists ---
if [[ ! -f "${CONFIG_JSON}" ]]; then
    echo "ERROR: config not found: ${CONFIG_JSON}" >&2
    exit 2
fi

# Build the C++ binaries in ${BIN_DIR} so the pipeline runs against the current
# source. Dry-run skips both the build and the binary check below. SKIP_PIPELINE=1
# skips the whole pipeline (incl. build) -- the evaluator reads already-written
# CSVs and needs no binary.
if [[ "${SKIP_PIPELINE}" != "1" && "${DRY_RUN}" != "1" ]]; then
    print_header "Building ${BIN_DIR}/" "make -j6 in ${PROJECT_ROOT}/${BIN_DIR}"
    cd "${PROJECT_ROOT}/${BIN_DIR}" && make -j6 || {
        echo "ERROR: build failed in ${PROJECT_ROOT}/${BIN_DIR}" >&2
        exit 1
    }
    cd "${PROJECT_ROOT}"
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
# before the pipeline spawns the binary. No-op on DRY_RUN and SKIP_PIPELINE=1
# (nothing runs, nothing is patched).
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

if [[ "${SKIP_PIPELINE}" != "1" && "${DRY_RUN}" != "1" ]]; then
    if [[ ! -f "${PARAMS_YAML}" ]]; then
        echo "ERROR: ${PARAMS_YAML} not found -- cannot set TIME_LIMIT." >&2
        exit 2
    fi
    # `patch` reads time_limit_seconds from <MODE>_mode, backs up parameters.yaml,
    # patches the TIME_LIMIT line, and prints the backup path as its FINAL line.
    # Capture every line (status echo + backup path) but keep only the last for
    # the restore trap; fail loudly (exit 2) if the helper rejects the contract.
    _PATCH_OUT="$("${PYTHON}" "${SCRIPT_DIR}/lib/patch_time_limit.py" patch \
        "${CONFIG_JSON}" "${MODE}" "${PARAMS_YAML}")" || {
        echo "${_PATCH_OUT}" >&2
        echo "ERROR: patch_time_limit.py patch failed." >&2
        exit 2
    }
    _PARAMS_BACKUP="$(printf '%s\n' "${_PATCH_OUT}" | tail -n 1)"
fi

# --- Stages 1-5: run the simulation pipeline (unless skipped / dry-run) ---
# build -> patch (done above) -> simulate -> sweep -> aggregate, all driven by
# run_end_to_end_experiments in fixed stage order. DRY_RUN lets it print its
# build+run commands; SKIP_PIPELINE=1 skips the whole pipeline (eval-only).
if [[ "${SKIP_PIPELINE}" != "1" ]]; then
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

    cd "${PROJECT_ROOT}"
    if [[ "${DRY_RUN}" == "1" ]]; then
        "${CMD[@]}"
    else
        "${CMD[@]}" || {
            echo "ERROR: pipeline stage failed; not running the evaluator." >&2
            exit 1
        }
    fi
fi

# --- Stage 6: evaluate the north-star gates (unless skipped / dry-run) ---
if [[ "${SKIP_EVAL}" != "1" ]]; then
    EVAL_CMD=(
        "${PYTHON}" -m simulation_experiments.evaluation_suite
        --config_json "${CONFIG_JSON}"
        --mode "${MODE}"
    )
    if [[ -n "${OVERHEAD_N}" ]]; then
        EVAL_CMD+=(--overhead_n "${OVERHEAD_N}")
    fi

    if [[ "${DRY_RUN}" == "1" ]]; then
        echo "[dry-run] ${EVAL_CMD[*]}"
    else
        cd "${PROJECT_ROOT}"
        "${EVAL_CMD[@]}"
        rc=$?
        print_footer "Verdict: $([[ ${rc} -eq 0 ]] && echo 'PASS' || echo 'FAIL') (exit ${rc})"
        exit ${rc}
    fi
fi

print_footer "Figures: ${PROJECT_ROOT}/simulation_experiments/optimizer_comparison/runs"

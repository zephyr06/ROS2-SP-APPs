#!/bin/bash
# A/B driver for the PA heuristic exploration. Usage: bash _ab_run.sh <phase>
#   phase=dry-baseline | baseline | variant | dry-variant
set -e
cd "$(dirname "$0")"
export CONFIG_JSON=simulation_experiments/configs/pa_heuristic_ab.json
export MODE=test
export SKIP_EVAL=1
case "$1" in
  dry-baseline)
    RERUN_MODE=clear_all DRY_RUN=1 ./scripts/run_simulation_plot_eval_ns.sh ;;
  baseline)
    RERUN_MODE=clear_all ./scripts/run_simulation_plot_eval_ns.sh ;;
  dry-variant)
    RERUN_MODE=clear_results DRY_RUN=1 ./scripts/run_simulation_plot_eval_ns.sh ;;
  variant)
    RERUN_MODE=clear_results ./scripts/run_simulation_plot_eval_ns.sh ;;
  *)
    echo "unknown phase: $1"; exit 2 ;;
esac

# shellcheck shell=bash
# common.sh -- shared helpers for the SP-Metric experiment shell scripts.
#
# Source this from any script in scripts/ with:
#     SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
#     # shellcheck source=lib/common.sh
#     source "${SCRIPT_DIR}/lib/common.sh"
#
# It sets up `set -euo pipefail`, resolves SCRIPT_DIR / PROJECT_ROOT, and
# exposes header/footer printing, a binary guard, and a Ctrl-C / TERM trap.

set -euo pipefail

# Resolve the project layout once, relative to the sourcing script.
# Requires SCRIPT_DIR to already be set by the caller.
if [[ -z "${SCRIPT_DIR:-}" ]]; then
    echo "common.sh: caller must set SCRIPT_DIR before sourcing." >&2
    exit 1
fi
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly SCRIPT_DIR PROJECT_ROOT

# Print a standard banner with the job title and key=value lines.
# Usage: print_header "Title" "Key1: value1" "Key2: value2"
print_header() {
    local title="$1"; shift
    local start_time
    start_time="$(date '+%Y-%m-%d %H:%M:%S')"
    echo "========================================================================"
    echo "  SP-Metric Optimization: ${title}"
    echo "========================================================================"
    echo "  Start time:   ${start_time}"
    while (( $# )); do
        echo "  $1"
        shift
    done
    echo "========================================================================"
}

# Print the closing banner. Usage: print_footer ["extra line ..."]
print_footer() {
    local end_time
    end_time="$(date '+%Y-%m-%d %H:%M:%S')"
    echo ""
    echo "========================================================================"
    echo "  Finished at: ${end_time}"
    if (( $# )); then
        echo "  $1"
    fi
    echo "========================================================================"
}

# Abort unless the RunOrchestrator binary exists and is executable.
# Usage: require_binary "${SIM_BIN}"
require_binary() {
    local sim_bin="$1"
    if [[ ! -x "${sim_bin}" ]]; then
        echo "Error: RunOrchestrator binary not found or not executable: ${sim_bin}" >&2
        echo "Compile in release mode first (e.g., 'make release'), or set BIN_DIR." >&2
        exit 1
    fi
}

# Install a Ctrl-C / TERM trap that kills the whole process group.
# Should be called once per script after any setup it must not clobber.
register_signal_trap() {
    trap '
        echo ""
        echo "Caught signal. Terminating child processes..."
        kill 0 2>/dev/null || true
        exit 130
    ' INT TERM
}

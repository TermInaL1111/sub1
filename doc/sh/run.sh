#!/usr/bin/env bash
# =============================================================================
# run.sh — Decoupled Evaluation Platform: verify & evaluate
#
# Usage:
#   ./run.sh test              Run all unit tests (17 tests, ~1s)
#   ./run.sh mock              Quick mock-sim pipeline test (3 episodes, ~1s)
#   ./run.sh eval              Real Habitat eval (5 episodes, ~5-10 min)
#   ./run.sh eval EPISODES     Real Habitat eval with N episodes
#   ./run.sh benchmark         Benchmark matrix (2 policies × 5 episodes)
#   ./run.sh collect PATH      Collect training traces to PATH.jsonl
#   ./run.sh all               test → mock → eval (full verification)
#
# Docker commands are used when running on the host (not inside container).
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DECOUP_ROOT="$SCRIPT_DIR"
ROS_X_ROOT="$(dirname "$DECOUP_ROOT")"

# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
_inside_docker() { [ -f /.dockerenv ] 2>/dev/null; }
_red()   { echo -e "\033[31m$*\033[0m"; }
_green() { echo -e "\033[32m$*\033[0m"; }
_cyan()  { echo -e "\033[36m$*\033[0m"; }

_docker_cmd() {
    # If already inside Docker, run directly. Otherwise use docker compose.
    if _inside_docker; then
        export PYTHONPATH="$DECOUP_ROOT:$PYTHONPATH"
        cd "$DECOUP_ROOT"
        bash -lc "$*"
    else
        cd "$ROS_X_ROOT/.." 2>/dev/null || cd "$HOME/ApexNav"
        sudo docker compose run --rm shell bash -lc "
            export PYTHONPATH=/workspace/ApexNav/ros-x-habitat/decoup:\$PYTHONPATH
            cd /workspace/ApexNav/ros-x-habitat/decoup
            $*
        "
    fi
}

_sep() { echo ""; echo "$(printf '=%.0s' {1..60})"; echo "  $*"; echo "$(printf '=%.0s' {1..60})"; }

# ---------------------------------------------------------------------------
# commands
# ---------------------------------------------------------------------------
cmd_test() {
    _sep "Unit Tests (17 tests)"
    _docker_cmd "python -m unittest sim_bridge.test.test_decoupled_e2e -v"
    _green "✓ All tests passed"
}

cmd_mock() {
    local episodes="${1:-3}"
    _sep "Mock Pipeline ($episodes episodes)"
    _docker_cmd "python -m sim_bridge.run_eval --mock --policy random_explorer --episodes $episodes"
    _green "✓ Mock pipeline OK"
}

cmd_eval() {
    local episodes="${1:-5}"
    local policy="${2:-random_explorer}"
    local dataset="${3:-hm3dv2}"
    _sep "Habitat Eval: $policy / $dataset ($episodes episodes)"
    _docker_cmd "python -m sim_bridge.run_eval --policy $policy --dataset $dataset --episodes $episodes"
}

cmd_benchmark() {
    local episodes="${1:-5}"
    local dataset="${2:-hm3dv2}"
    _sep "Benchmark Matrix ($episodes episodes × 2 policies)"
    _docker_cmd "python -m sim_bridge.run_eval --benchmark --policies random_explorer apexnav --dataset $dataset --episodes $episodes --output /tmp/benchmark_results.json"
    _green "✓ Benchmark complete → /tmp/benchmark_results.json"
}

cmd_collect() {
    local output="${1:-/tmp/training_trace.jsonl}"
    local episodes="${2:-5}"
    _sep "Data Collection → $output ($episodes episodes)"
    _docker_cmd "python -m sim_bridge.run_eval --mock --policy random_explorer --episodes $episodes --collect $output"
    _green "✓ Trace saved to $output"
}

cmd_all() {
    _cyan "Full Verification Pipeline"
    cmd_test
    cmd_mock 3
    cmd_eval 2 random_explorer hm3dv2
    _green "✓ All checks passed"
}

cmd_help() {
    echo "Usage: ./run.sh <command> [args]"
    echo ""
    echo "Commands:"
    echo "  test                 Run all 17 unit tests"
    echo "  mock [N]             Mock pipeline (N episodes, default 3)"
    echo "  eval [N] [policy] [dataset]   Habitat eval"
    echo "  benchmark [N] [dataset]       Policy × simulator matrix"
    echo "  collect PATH [N]     Collect JSONL training traces"
    echo "  all                  test → mock → eval (full check)"
    echo ""
    echo "Examples:"
    echo "  ./run.sh test"
    echo "  ./run.sh mock 5"
    echo "  ./run.sh eval 10 random_explorer mp3d"
    echo "  ./run.sh benchmark 10 hm3dv2"
    echo "  ./run.sh collect /tmp/trace.jsonl 20"
    echo "  ./run.sh all"
}

# ---------------------------------------------------------------------------
# dispatch
# ---------------------------------------------------------------------------
case "${1:-help}" in
    test)       cmd_test ;;
    mock)       cmd_mock "${2:-3}" ;;
    eval)       cmd_eval "${2:-5}" "${3:-random_explorer}" "${4:-hm3dv2}" ;;
    benchmark)  cmd_benchmark "${2:-5}" "${3:-hm3dv2}" ;;
    collect)    cmd_collect "${2:-/tmp/trace.jsonl}" "${3:-5}" ;;
    all)        cmd_all ;;
    help|--help|-h)  cmd_help ;;
    *)          _red "Unknown command: $1"; cmd_help; exit 1 ;;
esac

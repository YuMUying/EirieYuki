#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WCR_WORKSPACE:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
PROJECT_ROOT="$(cd "${WORKSPACE}/../../.." && pwd)"
MODE="${1:?usage: run_mode.sh MODE [OUTPUT_DIR]}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${2:-$PROJECT_ROOT/results/chassis_simulation/trajectory_tests/${STAMP}_mode${MODE}}"
LAUNCH_PID=""

cleanup() {
  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
    for _ in {1..40}; do
      kill -0 "$LAUNCH_PID" 2>/dev/null || break
      sleep 0.25
    done
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
      kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
    fi
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  LAUNCH_PID=""
}
trap cleanup EXIT INT TERM

source "$WORKSPACE/install/setup.bash"
mkdir -p "$OUTPUT_DIR"

setsid ros2 launch wcr_launcher launcher.launch.py \
  rviz:=false gz_args:="-r -s -v2" \
  >"$OUTPUT_DIR/simulation.log" 2>&1 &
LAUNCH_PID=$!

set +e
python3 "$SCRIPT_DIR/run_planner_scenario.py" \
  --mode "$MODE" --output "$OUTPUT_DIR" \
  >"$OUTPUT_DIR/recorder.log" 2>&1
RESULT=$?
set -e

cleanup
if [[ $RESULT -ne 0 ]]; then
  echo "Mode $MODE failed. See $OUTPUT_DIR/recorder.log and simulation.log." >&2
  exit "$RESULT"
fi

echo "Trajectory test logs: $OUTPUT_DIR"

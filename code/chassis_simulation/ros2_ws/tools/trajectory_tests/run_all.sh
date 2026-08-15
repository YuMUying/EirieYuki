#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WCR_WORKSPACE:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
PROJECT_ROOT="$(cd "${WORKSPACE}/../../.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_ROOT="${1:-$PROJECT_ROOT/results/chassis_simulation/trajectory_tests/$STAMP}"
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
set -u
mkdir -p "$OUTPUT_ROOT"

run_mode() {
  local mode="$1"
  local mode_dir="$OUTPUT_ROOT/mode${mode}"
  mkdir -p "$mode_dir"

  setsid ros2 launch wcr_launcher launcher.launch.py \
    rviz:=false gz_args:="-r -s -v2" \
    >"$mode_dir/simulation.log" 2>&1 &
  LAUNCH_PID=$!

  set +e
  python3 "$SCRIPT_DIR/run_planner_scenario.py" \
    --mode "$mode" --output "$mode_dir" \
    >"$mode_dir/recorder.log" 2>&1
  local result=$?
  set -e

  cleanup
  sleep 3
  if [[ $result -ne 0 ]]; then
    echo "Mode $mode failed. See $mode_dir/recorder.log and simulation.log." >&2
    return "$result"
  fi
}

run_mode 1
run_mode 2

python3 - "$OUTPUT_ROOT" <<'PY'
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
combined = {}
for mode in (1, 2):
    with (root / f"mode{mode}" / "summary.json").open(encoding="utf-8") as stream:
        combined[f"mode{mode}"] = json.load(stream)
with (root / "combined_summary.json").open("w", encoding="utf-8") as stream:
    json.dump(combined, stream, indent=2)
PY

echo "Trajectory test logs: $OUTPUT_ROOT"

#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "Source /opt/ros/<distro>/setup.bash before running this script." >&2
  exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VENV_PATH="${1:-${SCRIPT_DIR}/.venv-ros}"

python3 -m venv --system-site-packages "${VENV_PATH}"
source "${VENV_PATH}/bin/activate"
python -m pip install --upgrade pip setuptools wheel
python -m pip install -e "${SCRIPT_DIR}"
python - <<'PY'
import cv2
import numpy
import onnxruntime
import rclpy

required = {"CPUExecutionProvider"}
available = set(onnxruntime.get_available_providers())
if not required.issubset(available):
    raise RuntimeError(f"Missing ONNX providers: {required - available}")
print(f"ROS runtime ready: cv2={cv2.__version__} numpy={numpy.__version__} "
      f"onnxruntime={onnxruntime.__version__} providers={sorted(available)}")
PY

echo "Activate before colcon build and runtime: source ${VENV_PATH}/bin/activate"

#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${PROJECT_DIR}/.venv"

if ! python3 -m venv --help >/dev/null 2>&1; then
  printf 'python3-venv is missing. Install it with: sudo apt install python3.12-venv\n' >&2
  exit 1
fi

python3 -m venv "${VENV_DIR}"
"${VENV_DIR}/bin/python" -m pip install --upgrade pip setuptools wheel
"${VENV_DIR}/bin/pip" install --index-url https://download.pytorch.org/whl/cpu "torch>=2.2,<3.0"
"${VENV_DIR}/bin/pip" install -r "${PROJECT_DIR}/requirements-runtime.txt"

printf '\nEnvironment ready. Activate it with:\n  source "%s/bin/activate"\n' "${VENV_DIR}"

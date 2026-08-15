#!/usr/bin/env bash
set -euo pipefail

CODE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${CODE_DIR}/../../.." && pwd)"
DATASET="${PROJECT_ROOT}/datasets/weld_vision/WES-Combined-Dataset"
OUTPUT="${PROJECT_ROOT}/results/weld_vision/training/unet_cpu_mvp"

if [[ ! -f "${DATASET}/dataset.json" ]]; then
  printf 'Dataset is missing or incomplete: %s\n' "${DATASET}" >&2
  exit 1
fi

cd "${CODE_DIR}"
exec .venv/bin/python -u train.py \
  --data "${DATASET}" \
  --output "${OUTPUT}" \
  --epochs 30 \
  --image-size 192 \
  --batch-size 8 \
  --base-channels 8 \
  --workers 2 \
  --original-only

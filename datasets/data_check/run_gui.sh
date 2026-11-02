#!/usr/bin/env bash
# Launch GUI multi-G1 mode-2 stability check for datasets/walking.
#
# Usage:
#   bash datasets/data_check/run_gui.sh [num_envs] [start] [end] [work_dir]
# Examples:
#   bash datasets/data_check/run_gui.sh 8
#   bash datasets/data_check/run_gui.sh 8 0 15
#   bash datasets/data_check/run_gui.sh 4 20 27 datasets/data_check/runs/batch2
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

export OMNI_KIT_ACCEPT_EULA="${OMNI_KIT_ACCEPT_EULA:-YES}"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"

NUM_ENVS="${1:-8}"
START="${2:-}"
END="${3:-}"
WORK_DIR="${4:-datasets/data_check/runs/walking_gui}"

RANGE_ARGS=()
if [[ -n "${START}" ]]; then
  RANGE_ARGS+=(--start "${START}")
fi
if [[ -n "${END}" ]]; then
  RANGE_ARGS+=(--end "${END}")
fi

echo "[run_gui] root=$ROOT num_envs=$NUM_ENVS start=${START:-all} end=${END:-all} work=$WORK_DIR"
python -m datasets.data_check pipeline \
  --dataset datasets/walking \
  --work-dir "$WORK_DIR" \
  --num-envs "$NUM_ENVS" \
  --env-spacing 3.0 \
  "${RANGE_ARGS[@]}"

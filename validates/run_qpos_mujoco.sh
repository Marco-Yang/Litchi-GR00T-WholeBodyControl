#!/usr/bin/env bash
# 2/5 — Dataset observation.state (qpos) kinematic MuJoCo replay (no SONIC).
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/_paths.sh"

EP="${EP:-0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
RAW="${VALIDATE_RUNS}/ep$(printf '%03d' "${EP}")_qpos_${STAMP}.mp4"
OUT="${VALIDATE_VIDEOS}/ep$(printf '%03d' "${EP}")_qpos_mujoco_h264.mp4"
LABEL="${LABEL:-qpos MuJoCo}"

echo "[qpos] DS=${DS} EP=${EP} MJCF=${MJCF}"
# Use VLA venv (needs both mujoco + pyarrow); sim venv has no pyarrow/pip.
"${PY}" -u "${VALIDATE_LIB}/replay_dataset_qpos_kinematic.py" \
  --dataset "${DS}" --episode "${EP}" --out "${RAW}" --mjcf "${MJCF}" --ffmpeg "${FFMPEG_BIN}"
burn_label_h264 "${RAW}" "${LABEL}" "${OUT}"
echo "[qpos] → ${OUT}"

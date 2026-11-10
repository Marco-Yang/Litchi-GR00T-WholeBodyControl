#!/usr/bin/env bash
# 1/5 — Dataset teleop.smpl_joints → skeleton mp4 (no MuJoCo / SONIC).
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/_paths.sh"

EP="${EP:-0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
RAW="${VALIDATE_RUNS}/ep$(printf '%03d' "${EP}")_smpl_${STAMP}.mp4"
OUT="${VALIDATE_VIDEOS}/ep$(printf '%03d' "${EP}")_smpl_h264.mp4"
LABEL="${LABEL:-SMPL skeleton}"

echo "[smpl] DS=${DS} EP=${EP}"
"${PY}" -u "${VALIDATE_LIB}/replay_dataset_smpl_skeleton.py" \
  --dataset "${DS}" --episode "${EP}" --out "${RAW}" --ffmpeg "${FFMPEG_BIN}"
burn_label_h264 "${RAW}" "${LABEL}" "${OUT}"
echo "[smpl] → ${OUT}"

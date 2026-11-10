#!/usr/bin/env bash
# 3/5 — Disk action.motion_token → SONIC v4 decoder → MuJoCo.
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/_paths.sh"

EP="${EP:-0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
export WORK_DIR="${VALIDATE_RUNS}/token_decode_ep$(printf '%03d' "${EP}")_${STAMP}"
export OUT_MP4="${VALIDATE_RUNS}/ep$(printf '%03d' "${EP}")_token_decode_${STAMP}.mp4"
OUT="${VALIDATE_VIDEOS}/ep$(printf '%03d' "${EP}")_token_decode_mujoco_h264.mp4"
LABEL="${LABEL:-token decode MuJoCo}"

echo "[token] DS=${DS} EP=${EP}"
bash "${VALIDATE_LIB}/run_mujoco_token_decode.sh"
[[ -f "${OUT_MP4}" ]] || { echo "ERROR: missing ${OUT_MP4}" >&2; exit 1; }
burn_label_h264 "${OUT_MP4}" "${LABEL}" "${OUT}"
echo "[token] → ${OUT}"

#!/usr/bin/env bash
# 4/5 — VLA open-loop (GT dataset proprio) → token → SONIC → MuJoCo (+ RTC chunk inpaint).
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/_paths.sh"

EP="${EP:-0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
export GT_PROPRIO_ONLY=1
export CLOSED_LOOP_SEQUENTIAL="${CLOSED_LOOP_SEQUENTIAL:-0}"
export RTC_ENABLE="${RTC_ENABLE:-1}"
export RTC_OVERLAP_STEPS="${RTC_OVERLAP_STEPS:-28}"
export RTC_FROZEN_STEPS="${RTC_FROZEN_STEPS:-5}"
export RTC_RAMP_RATE="${RTC_RAMP_RATE:-3.0}"
export CKPT_TAG="${CKPT_TAG:-origin}"
# Always fresh paths (do not inherit OUT_MP4/WORK_DIR from a prior OL/CL in the same shell).
export WORK_DIR="${VALIDATE_RUNS}/vla_ol_ep$(printf '%03d' "${EP}")_${STAMP}"
export OUT_MP4="${VALIDATE_RUNS}/ep$(printf '%03d' "${EP}")_vla_openloop_${STAMP}.mp4"
OUT="${VALIDATE_VIDEOS}/ep$(printf '%03d' "${EP}")_vla_openloop_h264.mp4"
LABEL="${LABEL:-VLA open-loop}"

echo "[vla-ol] CKPT=${CKPT} DS=${DS} EP=${EP}"
bash "${VALIDATE_LIB}/run_mujoco_vla.sh"
[[ -f "${OUT_MP4}" ]] || { echo "ERROR: missing ${OUT_MP4}" >&2; exit 1; }
burn_label_h264 "${OUT_MP4}" "${LABEL}" "${OUT}"
echo "[vla-ol] → ${OUT}"

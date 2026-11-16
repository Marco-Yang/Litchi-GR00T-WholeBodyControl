#!/usr/bin/env bash
# Compare: GT token->SONIC v1.1 (left) | CL RTC o20/f4 | CL RTC o28/f4
# Dual-cam strip below MuJoCo; single bottom label (no double burn).
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/_paths.sh"

export CKPT="${CKPT:-${WBC_ROOT}/../Gr00T/checkpoints/skill2/checkpoint-49400}"
export DS="${DS:-${WBC_ROOT}/datasets/skill2_pick_toy/2026-11-12-16-53-36}"
export EP="${EP:-0}"
export MODEL_TAG="${MODEL_TAG:-$(basename "${CKPT}" | tr -cd 'A-Za-z0-9._-')}"
VALIDATE_VIDEOS="${VALIDATE_ROOT}/videos/${MODEL_TAG}"
VALIDATE_RUNS="${VALIDATE_ROOT}/runs/${MODEL_TAG}"
mkdir -p "${VALIDATE_VIDEOS}" "${VALIDATE_RUNS}"

# VLA needs transformers/torch — never inherit a polluted PY=sim from the shell.
if [[ -x "${WBC_ROOT}/.venv_inference/bin/python" ]]; then
  export PY="${VLA_PY:-${WBC_ROOT}/.venv_inference/bin/python}"
elif [[ -x "${HOME}/miniconda3/envs/gr00t/bin/python" ]]; then
  export PY="${VLA_PY:-${HOME}/miniconda3/envs/gr00t/bin/python}"
fi
"${PY}" -c "import transformers" >/dev/null 2>&1 || {
  echo "ERROR: PY=${PY} missing transformers (set VLA_PY=...)" >&2
  exit 1
}

export RTC_ENABLE=1
export RTC_RAMP_RATE=3.0
export RTC_FROZEN_STEPS=4
export ACTION_HORIZON=40
export GT_PANEL_LAYOUT=bottom
export LABEL_BOTTOM_MARGIN="${LABEL_BOTTOM_MARGIN:-160}"
export NET_IFACE=lo
export VLA_DEVICE="${VLA_DEVICE:-cuda:0}"
export DEPLOY_CUDA_VISIBLE_DEVICES="${DEPLOY_CUDA_VISIBLE_DEVICES:-0}"
export POLICY_PACK="${POLICY_PACK:-policy/sonic_v1_1}"

EP_TAG="$(printf '%03d' "${EP}")"
GT_OUT="${VALIDATE_VIDEOS}/ep${EP_TAG}_token_decode_mujoco_h264.mp4"
CL20_OUT="${VALIDATE_VIDEOS}/ep${EP_TAG}_vla_closedloop_RTCo20f4_h264.mp4"
CL28_OUT="${VALIDATE_VIDEOS}/ep${EP_TAG}_vla_closedloop_RTCo28f4_h264.mp4"
COMPOSE_OUT="${VALIDATE_VIDEOS}/ep${EP_TAG}_1x3_GTtoken_RTCo20f4_o28f4_h264.mp4"
SKIP_EXISTING="${SKIP_EXISTING:-1}"

echo "[compare] CKPT=${CKPT}"
echo "[compare] DS=${DS} EP=${EP} POLICY_PACK=${POLICY_PACK}"
echo "[compare] layout: GT token-decode | RTC o20/f4 | RTC o28/f4"

# --- 0) GT motion_token -> SONIC v1.1 decode (leftmost) ---
if [[ "${SKIP_EXISTING}" == "1" && -f "${GT_OUT}" && "${FORCE_GT:-0}" != "1" ]]; then
  echo "[compare] skip existing ${GT_OUT}"
else
  export LABEL="GT token SONIC v1.1"
  bash "${VALIDATE_ROOT}/run_token_decode_mujoco.sh"
  echo "[compare] wrote ${GT_OUT}"
fi

run_cl() {
  local overlap="$1"
  local exec_h="$2"
  local out="$3"
  local label="$4"
  if [[ "${SKIP_EXISTING}" == "1" && -f "${out}" ]]; then
    echo "[compare] skip existing ${out}"
    return 0
  fi
  export RTC_OVERLAP_STEPS="${overlap}"
  export EXEC_HORIZON="${exec_h}"
  export LABEL="${label}"
  bash "${VALIDATE_ROOT}/run_vla_closedloop.sh"
  local src="${VALIDATE_VIDEOS}/ep${EP_TAG}_vla_closedloop_h264.mp4"
  cp -f "${src}" "${out}"
  echo "[compare] wrote ${out}"
}

run_cl 20 20 "${CL20_OUT}" "RTC o20/f4/3.0"
run_cl 28 12 "${CL28_OUT}" "RTC o28/f4/3.0"

COMPOSE_PY="${SIM_PY}"
if ! "${COMPOSE_PY}" -c "import cv2" >/dev/null 2>&1; then
  COMPOSE_PY="${PY}"
fi
# Labels already on panels; --no-relabel avoids double burn.
# Order: GT leftmost, then o20, then o28.
"${COMPOSE_PY}" -u "${VALIDATE_ROOT}/compose_videos.py" \
  --model-tag "${MODEL_TAG}" \
  --panel "${GT_OUT}:GT token SONIC v1.1" \
  --panel "${CL20_OUT}:RTC o20/f4/3.0" \
  --panel "${CL28_OUT}:RTC o28/f4/3.0" \
  --layout hstack \
  --panel-w 640 \
  --panel-h 640 \
  --out-fps 30 \
  --align max \
  --no-relabel \
  --ffmpeg "${FFMPEG_BIN}" \
  --ffprobe "$(dirname "${FFMPEG_BIN}")/ffprobe" \
  --out "${COMPOSE_OUT}"

echo "[compare] DONE ${COMPOSE_OUT}"

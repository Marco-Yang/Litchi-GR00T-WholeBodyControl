#!/usr/bin/env bash
# One-shot: check Revo2 XML → render L/R hand close-up → H.264 deliverable.
# Intended for deploy-host agents. See AGENT.md.
set -euo pipefail

KIT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WBC_ROOT="$(cd "${KIT_DIR}/../.." && pwd)"
OUT_DIR="${OUT_DIR:-${KIT_DIR}/out}"
mkdir -p "${OUT_DIR}"

SCENE_XML="${SCENE_XML:-${KIT_DIR}/assets/scene_43dof_revo2.xml}"
# Prefer vizclean if requested: SCENE_VARIANT=vizclean ./run_all.sh
if [[ "${SCENE_VARIANT:-}" == "vizclean" ]]; then
  SCENE_XML="${KIT_DIR}/assets/scene_43dof_revo2_vizclean.xml"
fi

# Python with mujoco (deploy host usually has .venv_sim under WBC)
if [[ -n "${SIM_PY:-}" && -x "${SIM_PY}" ]]; then
  :
elif [[ -x "${WBC_ROOT}/.venv_sim/bin/python" ]]; then
  SIM_PY="${WBC_ROOT}/.venv_sim/bin/python"
elif [[ -x "${WBC_ROOT}/.venv_inference/bin/python" ]]; then
  SIM_PY="${WBC_ROOT}/.venv_inference/bin/python"
elif command -v python3 >/dev/null 2>&1; then
  SIM_PY="$(command -v python3)"
else
  echo "ERROR: no python found; set SIM_PY=/path/to/python with mujoco+imageio+opencv" >&2
  exit 1
fi

# Project ffmpeg (H.264). Fall back to PATH.
FFMPEG_CANDIDATES=(
  "${FFMPEG_BIN:-}"
  "${WBC_ROOT}/../VLA/.tools/bin/ffmpeg"
  "/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/.tools/bin/ffmpeg"
  "$(command -v ffmpeg || true)"
)
FFMPEG_BIN=""
for c in "${FFMPEG_CANDIDATES[@]}"; do
  [[ -n "${c}" && -x "${c}" ]] || continue
  FFMPEG_BIN="${c}"
  break
done
if [[ -z "${FFMPEG_BIN}" ]]; then
  echo "ERROR: ffmpeg not found; set FFMPEG_BIN=/path/to/ffmpeg with libx264" >&2
  exit 1
fi

# Headless GL
export MUJOCO_GL="${MUJOCO_GL:-egl}"

RAW_MP4="${OUT_DIR}/revo2_hand_closeup_raw.mp4"
H264_MP4="${OUT_DIR}/revo2_hand_closeup_h264.mp4"
MID_PNG="${OUT_DIR}/revo2_hand_closeup_mid.png"
LOG="${OUT_DIR}/run_all.log"

{
  echo "=== hand_closeup_kit run_all ==="
  echo "KIT_DIR=${KIT_DIR}"
  echo "WBC_ROOT=${WBC_ROOT}"
  echo "SIM_PY=${SIM_PY}"
  echo "FFMPEG_BIN=${FFMPEG_BIN}"
  echo "MUJOCO_GL=${MUJOCO_GL}"
  echo "SCENE_XML=${SCENE_XML}"
  echo "OUT_DIR=${OUT_DIR}"
  "${SIM_PY}" -c 'import mujoco, imageio, cv2, numpy; print("imports_ok", mujoco.__version__)'
} | tee "${LOG}"

echo "[1/3] check_xml" | tee -a "${LOG}"
"${SIM_PY}" "${KIT_DIR}/check_xml.py" --scene-xml "${SCENE_XML}" 2>&1 | tee -a "${LOG}"

echo "[2/3] render close-up" | tee -a "${LOG}"
"${SIM_PY}" "${KIT_DIR}/render_mjcf_hand_closeup.py" \
  --scene-xml "${SCENE_XML}" \
  --out-mp4 "${RAW_MP4}" \
  --mid-png "${MID_PNG}" \
  --fps 30 --seconds 5 \
  2>&1 | tee -a "${LOG}"

echo "[3/3] re-encode H.264" | tee -a "${LOG}"
"${FFMPEG_BIN}" -y -i "${RAW_MP4}" \
  -c:v libx264 -pix_fmt yuv420p -movflags +faststart \
  "${H264_MP4}" 2>&1 | tee -a "${LOG}"
rm -f "${RAW_MP4}"

ls -lh "${H264_MP4}" "${MID_PNG}" | tee -a "${LOG}"
echo "DONE deliverable=${H264_MP4}" | tee -a "${LOG}"

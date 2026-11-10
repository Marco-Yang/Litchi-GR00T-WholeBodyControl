# Shared paths for validates/ launchers.
#   source "$(cd "$(dirname "$0")" && pwd)/_paths.sh"
#
# Deliverable videos:
#   validates/videos/<CKPT basename>/...

VALIDATE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WBC_ROOT="${WBC_ROOT:-$(cd "${VALIDATE_ROOT}/.." && pwd)}"
VALIDATE_LIB="${VALIDATE_LIB:-${VALIDATE_ROOT}/lib}"
LITCHI_ROOT="${LITCHI_ROOT:-/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/Litchi-Isaac-GR00T}"
DEPLOY="${DEPLOY:-${WBC_ROOT}/gear_sonic_deploy}"
SIM_PY="${SIM_PY:-${WBC_ROOT}/.venv_sim/bin/python}"
PY="${PY:-${LITCHI_ROOT}/.venv/bin/python}"
TOOLS="${TOOLS:-/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/.tools}"
GROOT_VENV_SP="${GROOT_VENV_SP:-${LITCHI_ROOT}/.venv/lib/python3.12/site-packages}"
FFMPEG_BIN="${FFMPEG_BIN:-${TOOLS}/bin/ffmpeg}"
MJCF="${MJCF:-${DEPLOY}/g1/g1_29dof_with_revo2.xml}"

# Default dataset / origin VLA ckpt (override with DS= CKPT=).
DS="${DS:-/home/zhenyu/workspace/data/demo/dataset/origin/walk_towards_the_green_table/2026-11-05-13-35-11}"
CKPT="${CKPT:-/home/zhenyu/workspace/data/demo/outputs/walk_towards_the_green_table_skill1_all_rtc_2cam_origin/walk_towards_green_table_skill1_all_rtc_2cam_origin}"
EP="${EP:-0}"

# videos/<weight_name>/  — weight folder name = basename(CKPT) unless MODEL_TAG= set.
MODEL_TAG="${MODEL_TAG:-$(basename "${CKPT}" | tr -cd 'A-Za-z0-9._-')}"
VALIDATE_VIDEOS="${VALIDATE_VIDEOS:-${VALIDATE_ROOT}/videos/${MODEL_TAG}}"
VALIDATE_RUNS="${VALIDATE_RUNS:-${VALIDATE_ROOT}/runs/${MODEL_TAG}}"

mkdir -p "${VALIDATE_VIDEOS}" "${VALIDATE_RUNS}"

burn_label_h264() {
  # burn_label_h264 <in.mp4> <label> [out_h264.mp4]
  local src="${1:?}"
  local label="${2:?}"
  local dst="${3:-${VALIDATE_VIDEOS}/$(basename "${src%.mp4}")_h264.mp4}"
  "${PY}" -u "${VALIDATE_LIB}/burn_bottom_label.py" \
    --in "${src}" --label "${label}" --out "${dst}" --ffmpeg "${FFMPEG_BIN}"
  echo "[validates] labeled h264 → ${dst}"
}

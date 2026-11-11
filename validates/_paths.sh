# Shared paths for validates/ launchers.
#   source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_paths.sh"
#
# Deliverable videos:
#   validates/videos/<CKPT basename>/...

VALIDATE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WBC_ROOT="${WBC_ROOT:-$(cd "${VALIDATE_ROOT}/.." && pwd)}"
VALIDATE_LIB="${VALIDATE_LIB:-${VALIDATE_ROOT}/lib}"
DEPLOY="${DEPLOY:-${WBC_ROOT}/gear_sonic_deploy}"
SIM_PY="${SIM_PY:-${WBC_ROOT}/.venv_sim/bin/python}"

# Prefer conda gr00t (torch+pyarrow); else .venv_inference; else zhenyu Litchi-Isaac.
if [[ -z "${PY:-}" ]]; then
  if [[ -x "${HOME}/miniconda3/envs/gr00t/bin/python" ]]; then
    PY="${HOME}/miniconda3/envs/gr00t/bin/python"
  elif [[ -x "${WBC_ROOT}/.venv_inference/bin/python" ]]; then
    PY="${WBC_ROOT}/.venv_inference/bin/python"
  elif [[ -x "/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/Litchi-Isaac-GR00T/.venv/bin/python" ]]; then
    PY="/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/Litchi-Isaac-GR00T/.venv/bin/python"
  else
    PY="$(command -v python3)"
  fi
fi

# Isaac-GR00T / Litchi-Isaac root (parent of the gr00t package; needs gr00t/data).
if [[ -z "${LITCHI_ROOT:-}" ]]; then
  if [[ -d "${WBC_ROOT}/.deps/Isaac-GR00T/gr00t/data" ]]; then
    LITCHI_ROOT="$(cd "${WBC_ROOT}/.deps/Isaac-GR00T" && pwd)"
  else
    _LITCHI_FROM_PY="$("${PY}" - <<'PY' 2>/dev/null || true
import gr00t, pathlib
print(pathlib.Path(gr00t.__file__).resolve().parents[1])
PY
)"
    if [[ -n "${_LITCHI_FROM_PY}" && -d "${_LITCHI_FROM_PY}/gr00t/data" ]]; then
      LITCHI_ROOT="${_LITCHI_FROM_PY}"
    elif [[ -d "${WBC_ROOT}/../Gr00T/gr00t/data" ]]; then
      LITCHI_ROOT="$(cd "${WBC_ROOT}/../Gr00T" && pwd)"
    elif [[ -d "/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/Litchi-Isaac-GR00T/gr00t/data" ]]; then
      LITCHI_ROOT="/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/Litchi-Isaac-GR00T"
    else
      LITCHI_ROOT="$(cd "$(dirname "${PY}")/../.." && pwd)"
    fi
  fi
fi
# site-packages next to PY (conda or venv)
_PY_PREFIX="$(cd "$(dirname "${PY}")/.." && pwd)"
if [[ -d "${_PY_PREFIX}/lib/python3.12/site-packages" ]]; then
  GROOT_VENV_SP="${GROOT_VENV_SP:-${_PY_PREFIX}/lib/python3.12/site-packages}"
elif [[ -d "${_PY_PREFIX}/lib/python3.10/site-packages" ]]; then
  GROOT_VENV_SP="${GROOT_VENV_SP:-${_PY_PREFIX}/lib/python3.10/site-packages}"
else
  GROOT_VENV_SP="${GROOT_VENV_SP:-${_PY_PREFIX}/lib/python3.12/site-packages}"
fi

# ffmpeg: PATH, tools tree, or conda env
if [[ -z "${FFMPEG_BIN:-}" ]]; then
  if command -v ffmpeg >/dev/null 2>&1; then
    FFMPEG_BIN="$(command -v ffmpeg)"
  elif [[ -x "${HOME}/miniconda3/envs/gr00t/bin/ffmpeg" ]]; then
    FFMPEG_BIN="${HOME}/miniconda3/envs/gr00t/bin/ffmpeg"
  elif [[ -x "/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/.tools/bin/ffmpeg" ]]; then
    FFMPEG_BIN="/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/.tools/bin/ffmpeg"
  else
    FFMPEG_BIN="ffmpeg"
  fi
fi
TOOLS="${TOOLS:-$(dirname "${FFMPEG_BIN}")/..}"
MJCF="${MJCF:-${DEPLOY}/g1/g1_29dof_with_revo2.xml}"

# Local ORT / ZMQ for deploy LD_LIBRARY_PATH overrides in lib/*.sh
ORT_LIB="${ORT_LIB:-${HOME}/.local/onnxruntime/lib}"
export ORT_LIB

# Default dataset / origin VLA ckpt (override with DS= CKPT=).
# Local mirrors of the zhenyu demo paths when present.
_DS_LOCAL="${WBC_ROOT}/datasets/walking/2026-11-05-13-35-11"
_CKPT_CANDIDATES=(
  "${CKPT:-}"
  "${VLA_CKPT:-}"
  "${WBC_ROOT}/../Gr00T/checkpoints/walk_towards_green_table_skill1_all_rtc_2cam_origin"
  "${WBC_ROOT}/../Gr00T/checkpoints/finetuned"
  "${HOME}/checkpoints/walk_towards_green_table_skill1_all_rtc_2cam_origin/walk_towards_green_table_skill1_all_rtc_2cam_origin"
  "/home/zhenyu/workspace/data/demo/outputs/walk_towards_the_green_table_skill1_all_rtc_2cam_origin/walk_towards_green_table_skill1_all_rtc_2cam_origin"
)
DS="${DS:-}"
if [[ -z "${DS}" ]]; then
  if [[ -d "${_DS_LOCAL}" ]]; then
    DS="${_DS_LOCAL}"
  else
    DS="/home/zhenyu/workspace/data/demo/dataset/origin/walk_towards_the_green_table/2026-11-05-13-35-11"
  fi
fi
if [[ -z "${CKPT:-}" ]]; then
  for c in "${_CKPT_CANDIDATES[@]}"; do
    [[ -n "${c}" && -d "${c}" && ( -f "${c}/config.json" || -f "${c}/model.safetensors.index.json" ) ]] || continue
    CKPT="${c}"
    break
  done
fi
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

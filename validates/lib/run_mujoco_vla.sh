#!/usr/bin/env bash
# Open-loop MuJoCo: mode2-RTC finetuned GR00T → SONIC decoder
#
# Control:
#   frames [0, GT_BOOTSTRAP_FRAMES): dataset GT action.motion_token → decoder
#   then: dual-cam (ego+head) + proprio → VLA → token → decoder → qpos
#
# Timing (same as validate_20_settle1_tail2 / GT replay):
#   deploy ready → settle PRE_GO_SETTLE_S (default 1s) → RECORD+GO together
#   after episode wall-time → keep recording RECORD_TAIL_SLEEP_S (default 2s)
#
# NO Phi_0_wpy.
set -euo pipefail
VALIDATE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "${VALIDATE_ROOT}/_paths.sh"
# >>> edit these for other ckpt / episode <<<
CKPT="${CKPT:-/home/zhenyu/workspace/data/demo/outputs/walk_towards_the_green_table_skill1_all_rtc_2cam_origin/walk_towards_green_table_skill1_all_rtc_2cam_origin}"
DS="${DS:-/home/zhenyu/workspace/data/demo/dataset/origin/walk_towards_the_green_table/2026-11-05-13-35-11}"
EP="${EP:-0}"
GT_BOOTSTRAP_FRAMES="${GT_BOOTSTRAP_FRAMES:-10}"
# 1 = open-loop policy proprio (dataset state); 0 = closed-loop (sim LowState)
GT_PROPRIO_ONLY="${GT_PROPRIO_ONLY:-1}"
PRE_GO_SETTLE_S="${PRE_GO_SETTLE_S:-1}"
RECORD_TAIL_SLEEP_S="${RECORD_TAIL_SLEEP_S:-2}"
CONTROL_FPS="${CONTROL_FPS:-50}"
EXEC_HORIZON="${EXEC_HORIZON:-12}"
ACTION_HORIZON="${ACTION_HORIZON:-40}"
DEPLOY_CUDA="${DEPLOY_CUDA_VISIBLE_DEVICES:-1}"
VLA_DEVICE="${VLA_DEVICE:-cuda:0}"
# DDS NIC for MuJoCo validates MUST stay on loopback.
# Never default to enp11s0 / robot LAN — deploy LowCmd on that NIC reaches the real G1.
# Override only for intentional bring-up: NET_IFACE=enp11s0 (real) or a dedicated sim NIC.
NET_IFACE="${NET_IFACE:-lo}"
if [[ "${NET_IFACE}" == "enp11s0" || "${NET_IFACE}" == "eth0" ]]; then
  echo "WARNING: NET_IFACE=${NET_IFACE} is a robot/control NIC — real G1 may receive LowCmd" >&2
fi
CKPT_TAG="${CKPT_TAG:-$(basename "${CKPT}" | tr -cd 'A-Za-z0-9._-')}"
if [[ "${GT_PROPRIO_ONLY}" == "1" ]]; then
  PROPRIO_TAG="openloop"
else
  PROPRIO_TAG="closedloop"
fi

# Episode wall-time from parquet length (control fps), unless MOTION_SECONDS set.
if [[ -z "${MOTION_SECONDS:-}" ]]; then
  MOTION_SECONDS="$("${PY}" - <<PY
import pyarrow.parquet as pq
from pathlib import Path
p = Path("${DS}") / "data/chunk-000" / f"episode_{int('${EP}'):06d}.parquet"
n = pq.read_metadata(p).num_rows
print(f"{n / float('${CONTROL_FPS}'):.3f}")
PY
)"
fi

if [[ "${SIM_PY}" == *Phi_0* ]] || [[ "${DEPLOY}" == *Phi_0* ]] || [[ "${WBC_ROOT}" == *Phi_0* ]] || [[ "${CKPT}" == *Phi_0* ]]; then
  echo "ERROR: Phi_0_wpy path leaked into WBC stack" >&2
  exit 1
fi
if [[ ! -x "${SIM_PY}" ]]; then
  echo "ERROR: missing WBC sim python ${SIM_PY}" >&2
  exit 1
fi
if [[ ! -x "${DEPLOY}/target/release/g1_deploy_onnx_ref" ]]; then
  echo "ERROR: missing WBC deploy binary under ${DEPLOY}" >&2
  exit 1
fi
if [[ ! -d "${CKPT}" ]]; then
  echo "ERROR: missing checkpoint ${CKPT}" >&2
  exit 1
fi

WORK_DIR="${WORK_DIR:-${VALIDATE_RUNS}/vla_mode2_rtc_ep$(printf '%03d' "${EP}")_gtboot${GT_BOOTSTRAP_FRAMES}_${PROPRIO_TAG}_${CKPT_TAG}_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "${WORK_DIR}/logs"
LOG_DIR="${WORK_DIR}/logs"
OUT_MP4="${OUT_MP4:-${VALIDATE_VIDEOS}/ep$(printf '%03d' "${EP}")_vla_mode2_rtc_${CKPT_TAG}_gtboot${GT_BOOTSTRAP_FRAMES}_${PROPRIO_TAG}_settle1_tail2.mp4}"
ROOT_CSV="${OUT_MP4%.mp4}.root_track.csv"
export ROOT_TRACK_CSV="${ROOT_CSV}"

RECORD_START="${WORK_DIR}/.record_start"
RECORD_STOP="${WORK_DIR}/.record_stop"
ARM_FLAG="${WORK_DIR}/.arm"
STREAM_FLAG="${WORK_DIR}/.stream"
READY_FLAG="${WORK_DIR}/.ready"
GO_FLAG="${WORK_DIR}/.go"
DEPLOY_FIFO="${WORK_DIR}/.deploy_fifo"
rm -f "${RECORD_START}" "${RECORD_STOP}" "${ARM_FLAG}" "${STREAM_FLAG}" \
  "${READY_FLAG}" "${GO_FLAG}" "${DEPLOY_FIFO}"

# Local ORT / ZMQ (override with ORT= / ZMQ_LIB=). Fall back to zhenyu demo tree if present.
ORT="${ORT:-}"
if [[ -z "${ORT}" ]]; then
  if [[ -d "${ORT_LIB:-${HOME}/.local/onnxruntime/lib}" ]]; then
    ORT="${ORT_LIB:-${HOME}/.local/onnxruntime/lib}"
  elif [[ -d "/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/deps/onnxruntime-linux-x64-1.16.3/lib" ]]; then
    ORT="/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/deps/onnxruntime-linux-x64-1.16.3/lib"
  fi
fi
ZMQ_LIB="${ZMQ_LIB:-}"
if [[ -z "${ZMQ_LIB}" ]]; then
  if [[ -d "${GROOT_VENV_SP}/pyzmq.libs" ]]; then
    ZMQ_LIB="${GROOT_VENV_SP}/pyzmq.libs"
  elif [[ -d "/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/deps/libzmq" ]]; then
    ZMQ_LIB="/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/deps/libzmq"
  else
    ZMQ_LIB="/lib/x86_64-linux-gnu"
  fi
fi
DDS="${DEPLOY}/thirdparty/unitree_sdk2/thirdparty/lib/x86_64"
# Deploy binary links system TensorRT (e.g. 10.13.x). Do NOT prepend pip/conda
# tensorrt_libs (10.15.x) — that breaks deserialize / version checks.
# ROS2 libs are required by g1_deploy_onnx_ref (librcl_*), keep them early.
ROS_LIB="${ROS_LIB:-/opt/ros/jazzy/lib}"
ROS_LIB64="${ROS_LIB64:-/opt/ros/jazzy/lib/x86_64-linux-gnu}"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  # Keep current LD; setup.bash prepends ROS paths.
  source /opt/ros/jazzy/setup.bash
  set -u
fi
export LD_LIBRARY_PATH="\
${ORT}:\
${ROS_LIB}:\
${ROS_LIB64}:\
/usr/local/cuda/lib64:\
${GROOT_VENV_SP}/nvidia/cuda_runtime/lib:\
${GROOT_VENV_SP}/nvidia/cublas/lib:\
${GROOT_VENV_SP}/nvidia/cudnn/lib:\
${GROOT_VENV_SP}/nvidia/cuda_nvrtc/lib:\
${ZMQ_LIB}:\
${DDS}:\
${TOOLS}/lib:\
${LD_LIBRARY_PATH:-}"
export PATH="$(dirname "${FFMPEG_BIN}"):${TOOLS}/bin:${PATH}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export PYTHONUNBUFFERED=1
export PYTHONPATH="${LITCHI_ROOT}:${WBC_ROOT}:${PYTHONPATH:-}"
export HF_HOME="${HF_HOME:-${HOME}/.cache/huggingface}"
export HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"
export TRANSFORMERS_OFFLINE="${TRANSFORMERS_OFFLINE:-1}"
export NO_ALBUMENTATIONS_UPDATE=1
unset PHI0_MUJOCO_KEEP_POSE_ON_LOWCMD PHI0_MUJOCO_HOLD_RSI_UNTIL_FLAG \
  PHI0_MUJOCO_DISABLE_FALL_RESET PHI0_CL_RELEASE_RSI_HOLD_FLAG || true

PIDS=()
cleanup() {
  set +e
  touch "${RECORD_STOP}" 2>/dev/null || true
  for pid in "${PIDS[@]:-}"; do kill "${pid}" 2>/dev/null || true; done
  sleep 1
  for pid in "${PIDS[@]:-}"; do kill -9 "${pid}" 2>/dev/null || true; done
  exec 3>&- 2>/dev/null || true
}
trap cleanup EXIT

log() { echo "[wbc_vla_mode2 $(date +%H:%M:%S)] $*"; }
wait_log() {
  local file="$1" pat="$2" timeout_s="$3" label="$4" i
  for ((i=1; i<=timeout_s; i++)); do
    if grep -qE "${pat}" "${file}" 2>/dev/null; then
      log "OK ${label} (${i}s)"; return 0
    fi
    sleep 1
  done
  log "TIMEOUT ${label}"; tail -60 "${file}" || true; return 1
}

log "CKPT=${CKPT}"
log "DS=${DS} EP=${EP} MOTION_SECONDS=${MOTION_SECONDS}"
log "GT_BOOTSTRAP_FRAMES=${GT_BOOTSTRAP_FRAMES} PRE_GO_SETTLE_S=${PRE_GO_SETTLE_S} RECORD_TAIL_SLEEP_S=${RECORD_TAIL_SLEEP_S}"
log "NET_IFACE=${NET_IFACE} WORK_DIR=${WORK_DIR}"

GT_RSI_NPZ="${WORK_DIR}/gt_rsi_ep$(printf '%03d' "${EP}")_f0.npz"
log "build GT RSI npz → ${GT_RSI_NPZ}"
"${PY}" - <<PY
import numpy as np, pyarrow.parquet as pq
from pathlib import Path
root = Path("${DS}")
ep = f"{int('${EP}'):06d}"
table = pq.read_table(root / "data/chunk-000" / f"episode_{ep}.parquet")
st = np.asarray(table.column("observation.state")[0].as_py(), dtype=np.float64)
body = np.concatenate([st[0:15], st[15:22], st[29:36]])  # body29, skip hands
if "observation.init_base_quat" in table.column_names:
    quat = np.asarray(table.column("observation.init_base_quat")[0].as_py(), dtype=np.float64)
else:
    quat = np.asarray(table.column("observation.root_orientation")[0].as_py(), dtype=np.float64)
root_xyz = np.array([0.0, 0.0, 0.793], dtype=np.float64)
np.savez("${GT_RSI_NPZ}",
         reference_root_xyz=root_xyz.reshape(1, 3),
         reference_root_quat_wxyz=quat.reshape(1, 4),
         observation_qpos=body.reshape(1, -1))
print(f"wrote ${GT_RSI_NPZ} L_elbow={body[18]:.3f} R_elbow={body[25]:.3f}")
PY

log "start WBC sim (ground walk, no elastic band, GT RSI)..."
(
  cd "${WBC_ROOT}"
  # Prefer validates viz-clean scene (unified Revo2 hand meshes, no dual hand collision draw).
  REVO2_SCENE="${REVO2_SCENE:-${VALIDATE_ROOT}/assets/scene_43dof_revo2_vizclean.xml}"
  if [[ ! -f "${REVO2_SCENE}" ]]; then
    REVO2_SCENE="${DEPLOY}/g1/scene_43dof_revo2.xml"
  fi
  "${SIM_PY}" -u "${VALIDATE_LIB}/run_sim_record.py" \
    --interface "${NET_IFACE}" \
    --no-enable-onscreen \
    --enable-offscreen \
    --revo2-hand \
    --revo2-scene "${REVO2_SCENE}" \
    --disable-elastic-band \
    --disable-fall-reset \
    --gt-rsi-npz "${GT_RSI_NPZ}" \
    --standing-root-z 0.793 \
    --record-mp4 "${OUT_MP4}" \
    --record-start-flag "${RECORD_START}" \
    --record-stop-flag "${RECORD_STOP}" \
    --record-fps "${RECORD_FPS:-50}" \
    --g1-debug-host 127.0.0.1 \
    --g1-debug-port 5557
) > "${LOG_DIR}/sim.log" 2>&1 &
PIDS+=($!)
wait_log "${LOG_DIR}/sim.log" "sim loop starting" 120 "sim" || exit 1
sleep 3
if ! kill -0 "${PIDS[0]}" 2>/dev/null; then
  log "ERROR: sim died"; tail -50 "${LOG_DIR}/sim.log"; exit 1
fi
if grep -q "error in simulator" "${LOG_DIR}/sim.log"; then
  log "ERROR: sim error"; tail -50 "${LOG_DIR}/sim.log"; exit 1
fi
if ! grep -qE '\[revo2_hand\]|scene_43dof_revo2' "${LOG_DIR}/sim.log"; then
  log "ERROR: expected revo2 / scene_43dof_revo2 in sim.log"
  tail -40 "${LOG_DIR}/sim.log"; exit 1
fi
if ! grep -q "GT RSI" "${LOG_DIR}/sim.log"; then
  log "ERROR: GT RSI not applied"
  tail -40 "${LOG_DIR}/sim.log"; exit 1
fi
if grep -q "ELASTIC_BAND=True" "${LOG_DIR}/sim.log"; then
  log "ERROR: elastic band still enabled — need ground walking"
  exit 1
fi
sleep 2

# Load VLA before deploy so LowState keeps flowing while the heavy model loads.
# Cameras: always dataset dual-cam.
# Proprio: GT_PROPRIO_ONLY=1 → dataset state (open-loop); 0 → sim LowState (closed-loop).
# RTC defaults match origin train: H=40, s=12 → overlap=28, frozen=5, ramp=3.0.
# CLOSED_LOOP_SEQUENTIAL=1 disables RTC (hard cut) for A/B debug.
VLA_PROPRIO_ARGS=()
CLOSED_LOOP_SEQUENTIAL="${CLOSED_LOOP_SEQUENTIAL:-0}"
RTC_ENABLE="${RTC_ENABLE:-1}"
RTC_OVERLAP_STEPS="${RTC_OVERLAP_STEPS:-28}"
RTC_FROZEN_STEPS="${RTC_FROZEN_STEPS:-5}"
RTC_RAMP_RATE="${RTC_RAMP_RATE:-3.0}"
if [[ "${GT_PROPRIO_ONLY}" == "1" ]]; then
  VLA_PROPRIO_ARGS+=(--gt-proprio-only)
  log "start VLA replay OPEN-LOOP (gt-proprio-only, bootstrap=${GT_BOOTSTRAP_FRAMES}, rtc=${RTC_ENABLE})..."
else
  if [[ "${CLOSED_LOOP_SEQUENTIAL}" == "1" ]]; then
    VLA_PROPRIO_ARGS+=(--closed-loop-sequential)
    log "start VLA replay CLOSED-LOOP sequential hard-cut (sim proprio, bootstrap=${GT_BOOTSTRAP_FRAMES})..."
  else
    log "start VLA replay CLOSED-LOOP RTC (sim proprio, overlap=${RTC_OVERLAP_STEPS} frozen=${RTC_FROZEN_STEPS} ramp=${RTC_RAMP_RATE})..."
  fi
fi
if [[ "${RTC_ENABLE}" == "1" ]]; then
  VLA_PROPRIO_ARGS+=(--rtc)
else
  VLA_PROPRIO_ARGS+=(--no-rtc)
fi
VLA_PROPRIO_ARGS+=(
  --rtc-overlap-steps "${RTC_OVERLAP_STEPS}"
  --rtc-frozen-steps "${RTC_FROZEN_STEPS}"
  --rtc-ramp-rate "${RTC_RAMP_RATE}"
)
(
  cd "${LITCHI_ROOT}"
  PYTHONPATH="${LITCHI_ROOT}:${WBC_ROOT}:${PYTHONPATH:-}" \
  "${PY}" -u "${VALIDATE_LIB}/replay_vla_offline_zmq.py" \
    --model-path "${CKPT}" \
    --dataset "${DS}" \
    --episode "${EP}" \
    --device "${VLA_DEVICE}" \
    --control-fps "${CONTROL_FPS}" \
    --action-horizon "${ACTION_HORIZON}" \
    --execution-horizon "${EXEC_HORIZON}" \
    --motion-seconds "${MOTION_SECONDS}" \
    --arm-flag "${ARM_FLAG}" \
    --stream-flag "${STREAM_FLAG}" \
    --go-flag "${GO_FLAG}" \
    --ready-flag "${READY_FLAG}" \
    --gt-bootstrap-frames "${GT_BOOTSTRAP_FRAMES}" \
    "${VLA_PROPRIO_ARGS[@]}"
) > "${LOG_DIR}/vla.log" 2>&1 &
PIDS+=($!)
wait_log "${LOG_DIR}/vla.log" "bound tcp://" 600 "vla bind" || exit 1
grep -q "dual_cam=True" "${LOG_DIR}/vla.log" || log "WARN: dual_cam not confirmed in vla.log"
grep -q "GT bootstrap token0" "${LOG_DIR}/vla.log" || log "WARN: GT bootstrap token0 not confirmed"

POLICY_PACK="${POLICY_PACK:-policy/sonic_v1_1}"
DECODER_ONNX="${DECODER_ONNX:-${POLICY_PACK}/model_decoder.onnx}"
ENCODER_ONNX="${ENCODER_ONNX:-${POLICY_PACK}/model_encoder.onnx}"
OBS_CONFIG="${OBS_CONFIG:-${POLICY_PACK}/observation_config.yaml}"
for f in "${DEPLOY}/${DECODER_ONNX}" "${DEPLOY}/${ENCODER_ONNX}" "${DEPLOY}/${OBS_CONFIG}"; do
  [[ -e "${f}" ]] || { log "ERROR: missing ${f}"; exit 1; }
done
log "start g1_deploy_onnx_ref on ${NET_IFACE} (pack=${POLICY_PACK})..."
BIN="${DEPLOY}/target/release/g1_deploy_onnx_ref"
mkfifo "${DEPLOY_FIFO}"
(
  cd "${DEPLOY}"
  CUDA_VISIBLE_DEVICES="${DEPLOY_CUDA}" \
  "${BIN}" "${NET_IFACE}" "${DECODER_ONNX}" reference/example/ \
    --obs-config "${OBS_CONFIG}" \
    --encoder-file "${ENCODER_ONNX}" \
    --planner-file planner/target_vel/V2/planner_sonic.onnx \
    --input-type zmq_manager --output-type all \
    --zmq-host 127.0.0.1 --zmq-port 5556 --disable-crc-check < "${DEPLOY_FIFO}"
) > "${LOG_DIR}/deploy.log" 2>&1 &
PIDS+=($!)
exec 3>"${DEPLOY_FIFO}"
wait_log "${LOG_DIR}/deploy.log" "Init Done" 300 "deploy Init Done" || exit 1

touch "${STREAM_FLAG}"
sleep 0.2
touch "${ARM_FLAG}"
wait_log "${LOG_DIR}/deploy.log" "transitioning to CONTROL state" 90 "CONTROL" || true
wait_log "${LOG_DIR}/vla.log" "sent start" 30 "vla start" || true

log "enable ZMQ streaming (ENTER) + I (no trailing newline)..."
for _ in $(seq 1 12); do
  grep -q "ZMQ STREAMING MODE: ENABLED" "${LOG_DIR}/deploy.log" 2>/dev/null && break
  printf '\n' >&3
  sleep 2
done
grep -q "ZMQ STREAMING MODE: ENABLED" "${LOG_DIR}/deploy.log" || {
  log "ERROR: streaming not enabled"; tail -30 "${LOG_DIR}/deploy.log"; exit 1
}
printf 'I' >&3
sleep 0.5
local_en=$(grep -n "ZMQ STREAMING MODE: ENABLED" "${LOG_DIR}/deploy.log" | tail -1 | cut -d: -f1)
local_dis=$(grep -n "ZMQ STREAMING MODE: DISABLED" "${LOG_DIR}/deploy.log" | tail -1 | cut -d: -f1 || true)
if [[ -n "${local_dis}" && -n "${local_en}" && "${local_dis}" -gt "${local_en}" ]]; then
  log "WARN: streaming toggled OFF — re-enable"
  printf '\n' >&3
  sleep 1
fi
wait_log "${LOG_DIR}/deploy.log" "Streaming data mean delay" 30 "deploy streaming loop" || true
local_en=$(grep -n "ZMQ STREAMING MODE: ENABLED" "${LOG_DIR}/deploy.log" | tail -1 | cut -d: -f1)
local_dis=$(grep -n "ZMQ STREAMING MODE: DISABLED" "${LOG_DIR}/deploy.log" | tail -1 | cut -d: -f1 || true)
if [[ -n "${local_dis}" && -n "${local_en}" && "${local_dis}" -gt "${local_en}" ]]; then
  log "ERROR: streaming disabled before GO"; exit 1
fi

if [[ "${GT_PROPRIO_ONLY}" == "1" ]]; then
  # Open-loop VLA precomputes the token plan while holding GT[0] — wait before record.
  wait_log "${LOG_DIR}/vla.log" "precompute done" 600 "vla open-loop plan" || exit 1
  log "SONIC deploy + VLA plan ready — settle ${PRE_GO_SETTLE_S}s then RECORD+GO together..."
else
  # Closed-loop: model is loaded and holding GT[0]; infer online after GO.
  wait_log "${LOG_DIR}/vla.log" "wait go" 600 "vla closed-loop ready" || exit 1
  log "SONIC deploy + VLA closed-loop ready — settle ${PRE_GO_SETTLE_S}s then RECORD+GO together..."
fi
sleep "${PRE_GO_SETTLE_S}"
touch "${RECORD_START}"
touch "${GO_FLAG}"
wait_log "${LOG_DIR}/sim.log" "sim_record\\] started" 30 "record" || {
  log "ERROR: recorder did not start"; tail -50 "${LOG_DIR}/sim.log"; exit 1
}
if [[ "${GT_PROPRIO_ONLY}" == "1" ]]; then
  log "GO — GT bootstrap ${GT_BOOTSTRAP_FRAMES} frames, then open-loop VLA dual-cam tokens 1:1"
else
  log "GO — GT bootstrap ${GT_BOOTSTRAP_FRAMES} frames, then closed-loop VLA (sim proprio)"
fi

# Wait for motion window, then keep recording for RECORD_TAIL_SLEEP_S.
sleep "${MOTION_SECONDS}"
log "episode wall done — record tail ${RECORD_TAIL_SLEEP_S}s..."
sleep "${RECORD_TAIL_SLEEP_S}"
touch "${RECORD_STOP}"
wait_log "${LOG_DIR}/sim.log" "sim_record\\] saved" 30 "record save" || true
wait_log "${LOG_DIR}/vla.log" "\\[vla\\] done" 60 "vla done" || true
sleep 1

log "OUT_MP4=${OUT_MP4}"
ls -la "${OUT_MP4}" || true
if [[ -f "${ROOT_CSV}" ]]; then
  log "root_track=${ROOT_CSV}"
  "${SIM_PY}" - <<PY
import csv, math
from pathlib import Path
p=Path("${ROOT_CSV}")
rows=list(csv.DictReader(p.open()))
xs=[float(r["sim_x"]) for r in rows]; ys=[float(r["sim_y"]) for r in rows]
dx=xs[-1]-xs[0]; dy=ys[-1]-ys[0]
span=math.hypot(max(xs)-min(xs), max(ys)-min(ys))
print(f"root_start=({xs[0]:.3f},{ys[0]:.3f}) end=({xs[-1]:.3f},{ys[-1]:.3f}) delta=({dx:.3f},{dy:.3f}) xy_span={span:.3f} n={len(xs)}")
print("OK: non-trivial root xy motion" if span >= 0.05 else "WARN: root xy span < 5cm")
PY
else
  log "WARN: missing root_track csv ${ROOT_CSV}"
fi
rg -n 'GT bootstrap|switch to model|dual_cam|infer frame=' "${LOG_DIR}/vla.log" | head -40 || true
rg -o 'tokens\[0\]=[-+0-9.]+' "${LOG_DIR}/deploy.log" | sort | uniq -c | sort -rn | head -12 || true
grep -E 'Lost LowState|Temporary motion completed|INTERFACE=|ENV_TYPE=' "${LOG_DIR}/deploy.log" "${LOG_DIR}/sim.log" || true

log "WORK_DIR=${WORK_DIR}"

#!/usr/bin/env bash
# Litchi-GR00T-WholeBodyControl + GT motion-token replay (NO Phi_0_wpy runtime dependency)
#
# Timing (same as run_ep0_vla_mode2_rtc_replay.sh / smpl_v3):
#   deploy ready → settle PRE_GO_SETTLE_S (default 1s) → RECORD+GO together
#   after episode wall-time → keep recording RECORD_TAIL_SLEEP_S (default 2s)
set -euo pipefail
VALIDATE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "${VALIDATE_ROOT}/_paths.sh"
DS="${DS:-/home/zhenyu/workspace/data/demo/dataset/origin/walk_towards_the_green_table/2026-11-05-13-35-11}"
EP="${EP:-0}"
CONTROL_FPS="${CONTROL_FPS:-50}"
PRE_GO_SETTLE_S="${PRE_GO_SETTLE_S:-1}"
RECORD_TAIL_SLEEP_S="${RECORD_TAIL_SLEEP_S:-2}"
# Match smpl_v3 / qpos kinematic clock (override with RECORD_FPS=30 to match VLA mp4).
RECORD_FPS="${RECORD_FPS:-50}"
DEPLOY_CUDA="${DEPLOY_CUDA_VISIBLE_DEVICES:-1}"
# DDS: lo drops LowState here; eth1 + forced ENV_TYPE=sim (in run_sim_record).
NET_IFACE="${NET_IFACE:-eth1}"

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

if [[ ! -x "${SIM_PY}" ]]; then
  echo "ERROR: missing WBC sim python ${SIM_PY}" >&2
  exit 1
fi
if [[ ! -x "${DEPLOY}/target/release/g1_deploy_onnx_ref" ]]; then
  echo "ERROR: missing WBC deploy binary under ${DEPLOY}" >&2
  exit 1
fi

WORK_DIR="${WORK_DIR:-${VALIDATE_RUNS}/gt_replay_ep$(printf '%03d' "${EP}")_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "${WORK_DIR}/logs"
LOG_DIR="${WORK_DIR}/logs"
OUT_MP4="${OUT_MP4:-${VALIDATE_VIDEOS}/ep$(printf '%03d' "${EP}")_gt_token_replay.mp4}"
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

ORT="/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/deps/onnxruntime-linux-x64-1.16.3/lib"
DDS="${DEPLOY}/thirdparty/unitree_sdk2/thirdparty/lib/x86_64"
export LD_LIBRARY_PATH="\
${ORT}:\
${GROOT_VENV_SP}/nvidia/cuda_runtime/lib:\
${GROOT_VENV_SP}/tensorrt_libs:\
${GROOT_VENV_SP}/nvidia/cublas/lib:\
${GROOT_VENV_SP}/nvidia/cudnn/lib:\
${GROOT_VENV_SP}/nvidia/cuda_nvrtc/lib:\
${GROOT_VENV_SP}/pyzmq.libs:\
/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/deps/libzmq:\
${DDS}:\
${TOOLS}/lib:\
${LD_LIBRARY_PATH:-}"
export PATH="${TOOLS}/bin:${PATH}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export PYTHONUNBUFFERED=1
export PYTHONPATH="${WBC_ROOT}:${PYTHONPATH:-}"
# Explicitly NO Phi_0 env hacks that pin RSI pose forever.
unset PHI0_MUJOCO_KEEP_POSE_ON_LOWCMD PHI0_MUJOCO_HOLD_RSI_UNTIL_FLAG \
  PHI0_MUJOCO_DISABLE_FALL_RESET PHI0_CL_RELEASE_RSI_HOLD_FLAG || true
export GT_ZMQ_RECORD_STALL_S="${GT_ZMQ_RECORD_STALL_S:-120}"

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

log() { echo "[wbc_gt $(date +%H:%M:%S)] $*"; }
wait_log() {
  local file="$1" pat="$2" timeout_s="$3" label="$4" i
  for ((i=1; i<=timeout_s; i++)); do
    if grep -qE "${pat}" "${file}" 2>/dev/null; then
      log "OK ${label} (${i}s)"; return 0
    fi
    sleep 1
  done
  log "TIMEOUT ${label}"; tail -40 "${file}" || true; return 1
}

# 0) GT RSI from observation.state frame0 (body29) — ground start, no hang
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

# 1) Sim
log "start WBC sim (revo2, ground, GT RSI, NET_IFACE=${NET_IFACE})..."
(
  cd "${WBC_ROOT}"
  "${SIM_PY}" -u "${VALIDATE_LIB}/run_sim_record.py" \
    --interface "${NET_IFACE}" \
    --no-enable-onscreen \
    --enable-offscreen \
    --revo2-hand \
    --disable-elastic-band \
    --disable-fall-reset \
    --gt-rsi-npz "${GT_RSI_NPZ}" \
    --standing-root-z 0.793 \
    --record-mp4 "${OUT_MP4}" \
    --record-start-flag "${RECORD_START}" \
    --record-stop-flag "${RECORD_STOP}" \
    --record-fps "${RECORD_FPS}" \
    --g1-debug-host 127.0.0.1 \
    --g1-debug-port 5557
) > "${LOG_DIR}/sim.log" 2>&1 &
PIDS+=($!)
wait_log "${LOG_DIR}/sim.log" "sim loop starting" 120 "sim" || exit 1
grep -qE 'revo2_hand|scene_43dof_revo2' "${LOG_DIR}/sim.log" || {
  log "ERROR: expected revo2 scene"; tail -40 "${LOG_DIR}/sim.log"; exit 1
}
grep -qE 'ELASTIC_BAND=False' "${LOG_DIR}/sim.log" || log "WARN: expected ELASTIC_BAND=False"
log "MOTION_SECONDS=${MOTION_SECONDS} PRE_GO_SETTLE_S=${PRE_GO_SETTLE_S} RECORD_TAIL_SLEEP_S=${RECORD_TAIL_SLEEP_S} RECORD_FPS=${RECORD_FPS}"
sleep 6

# 2) Deploy from WBC tree
# Default sonic_v1_1 — must match数采 / rewrite encoder（release ≠ v1.1）。
POLICY_PACK="${POLICY_PACK:-policy/sonic_v1_1}"
DECODER_ONNX="${DECODER_ONNX:-${POLICY_PACK}/model_decoder.onnx}"
ENCODER_ONNX="${ENCODER_ONNX:-${POLICY_PACK}/model_encoder.onnx}"
OBS_CONFIG="${OBS_CONFIG:-${POLICY_PACK}/observation_config.yaml}"
for f in "${DEPLOY}/${DECODER_ONNX}" "${DEPLOY}/${ENCODER_ONNX}" "${DEPLOY}/${OBS_CONFIG}"; do
  if [[ ! -e "${f}" ]]; then
    log "ERROR: missing ${f} (set POLICY_PACK / DECODER_ONNX / ENCODER_ONNX / OBS_CONFIG)"
    exit 1
  fi
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
if grep -qE 'ChannelFactory|DDS.*fail|bind.*failed' "${LOG_DIR}/deploy.log"; then
  log "ERROR: DDS issue on ${NET_IFACE}"; tail -40 "${LOG_DIR}/deploy.log"; exit 1
fi

# 3) GT token replay (optional TOKENS_NPZ overrides parquet motion_token → still ZMQ v4)
log "start GT motion replay..."
REPLAY_EXTRA=()
if [[ -n "${TOKENS_NPZ:-}" ]]; then
  log "TOKENS_NPZ=${TOKENS_NPZ} start=${TOKEN_START:-0} stride=${TOKEN_STRIDE:-1} n=${TOKEN_N:-0}"
  REPLAY_EXTRA+=(--tokens-npz "${TOKENS_NPZ}")
  REPLAY_EXTRA+=(--token-start "${TOKEN_START:-0}")
  REPLAY_EXTRA+=(--token-stride "${TOKEN_STRIDE:-1}")
  REPLAY_EXTRA+=(--token-n "${TOKEN_N:-0}")
fi
(
  PYTHONPATH="${WBC_ROOT}:${PYTHONPATH:-}" \
  "${PY}" -u "${VALIDATE_LIB}/replay_gt_motion_zmq.py" \
    --dataset "${DS}" \
    --episode "${EP}" \
    --fps "${CONTROL_FPS}" \
    --arm-flag "${ARM_FLAG}" \
    --stream-flag "${STREAM_FLAG}" \
    --go-flag "${GO_FLAG}" \
    --ready-flag "${READY_FLAG}" \
    "${REPLAY_EXTRA[@]}"
) > "${LOG_DIR}/replay.log" 2>&1 &
PIDS+=($!)
wait_log "${LOG_DIR}/replay.log" "bound tcp://" 60 "replay bind" || exit 1

touch "${STREAM_FLAG}"
sleep 0.2
touch "${ARM_FLAG}"
wait_log "${LOG_DIR}/deploy.log" "transitioning to CONTROL state" 90 "CONTROL" || true
wait_log "${LOG_DIR}/replay.log" "sent start" 30 "replay start" || true

log "enable ZMQ streaming (ENTER) + I (no trailing newline — newline toggles stream OFF)..."
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
wait_log "${LOG_DIR}/sim.log" "deploy lowcmd active" 15 "lowcmd" || true
local_en=$(grep -n "ZMQ STREAMING MODE: ENABLED" "${LOG_DIR}/deploy.log" | tail -1 | cut -d: -f1)
local_dis=$(grep -n "ZMQ STREAMING MODE: DISABLED" "${LOG_DIR}/deploy.log" | tail -1 | cut -d: -f1 || true)
if [[ -n "${local_dis}" && -n "${local_en}" && "${local_dis}" -gt "${local_en}" ]]; then
  log "ERROR: streaming disabled before GO"; exit 1
fi

# Match VLA / smpl_v3: settle, then start record and token playback together.
log "SONIC deploy ready — settle ${PRE_GO_SETTLE_S}s then RECORD+GO together..."
sleep "${PRE_GO_SETTLE_S}"
touch "${RECORD_START}"
touch "${GO_FLAG}"
wait_log "${LOG_DIR}/sim.log" "sim_record\\] started" 30 "record" || {
  log "ERROR: recorder did not start"; tail -50 "${LOG_DIR}/sim.log"; exit 1
}
log "GO — GT motion_token → SONIC decode (record+playback started together)"

# Episode wall-time (parquet length / CONTROL_FPS), then keep recording tail.
sleep "${MOTION_SECONDS}"
log "episode wall done — record tail ${RECORD_TAIL_SLEEP_S}s..."
sleep "${RECORD_TAIL_SLEEP_S}"
touch "${RECORD_STOP}"
wait_log "${LOG_DIR}/sim.log" "sim_record\\] saved" 30 "record save" || true
wait_log "${LOG_DIR}/replay.log" "end of episode" 30 "replay end" || true
sleep 1

# Motion check: require non-trivial root xy motion from HUD/logs if available
log "OUT_MP4=${OUT_MP4}"
ls -la "${OUT_MP4}" || true
if [[ -f "${ROOT_CSV}" ]]; then
  "${PY}" - <<PY
import csv, math
from pathlib import Path
p=Path("${ROOT_CSV}")
rows=list(csv.DictReader(p.open()))
xs=[float(r.get("sim_x", r.get("x", 0))) for r in rows]
ys=[float(r.get("sim_y", r.get("y", 0))) for r in rows]
zs=[float(r.get("sim_z", r.get("z", 0))) for r in rows]
span=math.hypot(max(xs)-min(xs), max(ys)-min(ys)) if xs else 0.0
print(f"[root_track] n={len(rows)} xy_span={span:.3f} z=[{min(zs):.3f},{max(zs):.3f}]" if zs else "empty")
PY
fi
# grab a mid frame for elbow visual check
if [[ -f "${OUT_MP4}" ]]; then
  "${PY}" - <<PY
import cv2
from pathlib import Path
mp4=Path("${OUT_MP4}")
cap=cv2.VideoCapture(str(mp4))
n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, min(n//3, max(n-1,0))))
ok, frame=cap.read()
if ok:
  out=mp4.with_name("frame_mid.png")
  cv2.imwrite(str(out), frame)
  print(f"[frame] {out} n={n}")
PY
fi
# Token diversity check
if ! grep -qE 'tokens\[0\]=\+?0\.0625|tokens\[0\]=0\.0625|t0=\+0\.0625|t0=\+0\.125' "${LOG_DIR}/deploy.log" "${LOG_DIR}/replay.log" 2>/dev/null; then
  log "WARN: did not see non-stand token0 in logs (check deploy verbosity)"
fi
rg -n 'tokens\[0\]=' "${LOG_DIR}/deploy.log" | head -5 || true
rg -n 'tokens\[0\]=' "${LOG_DIR}/deploy.log" | rg -v '\-0\.0625' | head -10 || true
rg -n 'gt_replay\] fi=' "${LOG_DIR}/replay.log" | head -10 || true

log "WORK_DIR=${WORK_DIR}"

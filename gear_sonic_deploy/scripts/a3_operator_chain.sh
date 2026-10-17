#!/usr/bin/env bash
# ============================================================================
# Operator-side chain of the A3 / A3 Ultra Pico teleop: everything between
# pico_manager_thread_server.py and the deploy binary, selected with --encoder.
#
#   scripts/a3_operator_chain.sh --encoder a3-fast|g1|smpl [--pico-host HOST] [--pico-port 5556] [--real] [--no-bridge]
#
#   a3-fast / g1 : a3_teleop_retarget_node.py --mode a3-ref  (Pico SMPL -> A3 joint references, ZMQ :5560)
#                  + a3_ta_bridge (ZMQ :5560 -> ROS2 /ta/whole_body_command).  Needs the ROS2 environment of the
#                  robot's network (sim: deploy.sh exports the private domain; real: ROS_DOMAIN_ID, A3_TRANSPORT ...).
#   smpl         : a3_teleop_retarget_node.py --mode smpl-pose  (re-publishes the Pico pose stream with A3 wrist
#                  angles on ZMQ :5561; the deploy's smpl_zmq source subscribes to it).  No ROS needed.
#
# deploy.sh starts this next to the deploy binary for `--input-type zmq_manager` (sim / one machine).  On the real
# robot run it on the operator PC and start deploy.sh on the robot with `--no-chain`.
# --no-bridge : a3-fast / g1 only: start the retarget node but not a3_ta_bridge.  The deploy then reads the `a3_ref` ZMQ stream
#          (tcp://<this PC>:5560) itself (`--teleop-input zmq`): no ROS 2 at all on this side, nothing published on the robot
#          network.  deploy.sh passes it together with --teleop-input zmq.
# --real : the robot is a real one on the network (a3-fast / g1): refuse to start unless ROS_DOMAIN_ID is set (not 0) and
#          ROS_LOCALHOST_ONLY is not 1, because /ta/whole_body_command must reach the robot's AimRT.  Nothing else may publish that
#          topic meanwhile (the vendor's own TA / teleop stack).  deploy.sh passes it for `real`.
# Environment: A3_RETARGET_PYTHON (python with mujoco + mink + GMR + pyzmq, default: conda env gmr).
#   A3_FOOT_ROT_WEIGHT=<w> (default 8; 0 = GMR's stock table) / A3_ANKLE_KNEE_ROT_WEIGHT=<w> (unset = stock): IK rotation weights of the foot / ankle-pitch links.
#     The stock table leaves the foot orientation unconstrained, so the ankle roll of a plainly standing person drifts to its joint limit within ~3 s (2026-10-17).
#   A3_LOWER_BODY=stand|track (default track): stand fixes the legs of the a3_ref stream to the policy-stand reference (a3-fast / g1).
#   A3_FOOT_LIFT_SCALE=<s> (default 1): multiply each foot's height above the lower foot before IK.  Unset leaves the targets unchanged.
#   a3-fast / g1 additionally need the ROS2 environment of the robot's network, because a3_ta_bridge publishes
#   /ta/whole_body_command over ROS2: set ROS_DOMAIN_ID (and ROS_LOCALHOST_ONLY=0 across machines) like the robot, and
#   either source ROS2 or point A3_ROS_PREFIX at a prefix whose lib/ has the ROS2 libraries (default: conda env
#   a3_deploy_verify, used only when the bridge cannot resolve its libraries otherwise).  Against the MuJoCo sim started by
#   deploy.sh that is: ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 (deploy.sh exports both when it runs this script itself).
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GEAR_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd -- "${GEAR_ROOT}/.." && pwd)"

ENCODER="a3-fast"
PICO_HOST="localhost"
PICO_PORT=5556
REAL=0
NO_BRIDGE=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --encoder)   ENCODER="$2"; shift 2 ;;
    --pico-host) PICO_HOST="$2"; shift 2 ;;
    --pico-port) PICO_PORT="$2"; shift 2 ;;
    --real)      REAL=1; shift ;;
    --no-bridge) NO_BRIDGE=1; shift ;;
    -h|--help)   sed -n 2,22p "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "a3_operator_chain.sh: unknown argument $1" >&2; exit 2 ;;
  esac
done
case "${ENCODER}" in a3-fast|a3_fast|g1|smpl) ;; *) echo "--encoder must be a3-fast, g1 or smpl" >&2; exit 2 ;; esac
FOOT_ROT_WEIGHT="${A3_FOOT_ROT_WEIGHT-8}"   # unset -> 8; set to empty or 0 to keep GMR's stock table (an empty value passes no option)

if [[ "${REAL}" == "1" && "${ENCODER}" != "smpl" && "${NO_BRIDGE}" != "1" ]]; then
  if [[ -z "${ROS_DOMAIN_ID:-}" || "${ROS_DOMAIN_ID}" == "0" ]]; then
    echo "[a3-chain] --real: set ROS_DOMAIN_ID to the robot's domain (unset / 0 is shared with every ROS 2 participant)" >&2; exit 66
  fi
  if [[ "${ROS_LOCALHOST_ONLY:-0}" == "1" ]]; then
    echo "[a3-chain] --real: ROS_LOCALHOST_ONLY=1 keeps /ta/whole_body_command on this PC; export ROS_LOCALHOST_ONLY=0" >&2; exit 66
  fi
  echo "[a3-chain] real robot: ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0} RMW=${RMW_IMPLEMENTATION:-default}; nothing else may publish /ta/whole_body_command"
  if [[ "${RMW_IMPLEMENTATION:-}" != "rmw_cyclonedds_cpp" ]]; then
    echo "[a3-chain] WARNING: RMW is not rmw_cyclonedds_cpp.  The conda Humble Fast DDS 2.6.4 allocates tens of GB when another host in the domain" >&2
    echo "[a3-chain]   announces malformed ros_discovery_info (plan 9.7 / 18.6) and can freeze this PC.  Recommended: RMW_IMPLEMENTATION=rmw_cyclonedds_cpp," >&2
    echo "[a3-chain]   CYCLONEDDS_URI pinned to the robot NIC, and a memory cap (systemd-run --user --scope -p MemoryMax=2G -p MemorySwapMax=0)." >&2
  elif [[ -z "${CYCLONEDDS_URI:-}" ]]; then
    echo "[a3-chain] WARNING: CYCLONEDDS_URI is not set: Cyclone DDS will announce on every NIC (Wi-Fi, Tailscale, virtual); pin it to the robot NIC" >&2
  fi
fi

case "$(uname -m)" in
  x86_64)        ARCH="x86_64" ;;
  aarch64|arm64) ARCH="rockchip" ;;
  *) echo "unsupported machine: $(uname -m)" >&2; exit 2 ;;
esac
PKG_DIR="${GEAR_ROOT}/dist/a3_deploy_${ARCH}"

CONDA_ROOT="${CONDA_ROOT:-$HOME/miniconda3}"
RETARGET_PY="${A3_RETARGET_PYTHON:-${CONDA_ROOT}/envs/gmr/bin/python}"
[[ -x "${RETARGET_PY}" ]] || { echo "[a3-chain] retarget python not found: ${RETARGET_PY} (set A3_RETARGET_PYTHON; needs mujoco+mink+GMR)" >&2; exit 66; }

CHILDREN=()
cleanup() { for p in "${CHILDREN[@]+"${CHILDREN[@]}"}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM

NODE="${REPO_ROOT}/gear_sonic/scripts/a3_teleop_retarget_node.py"
if [[ "${ENCODER}" == "smpl" ]]; then
  echo "[a3-chain] encoder=smpl: Pico tcp://${PICO_HOST}:${PICO_PORT} 'pose'  ->  A3 wrist angles  ->  tcp://*:5561 'pose'"
  PYTHONPATH="${REPO_ROOT}:${PYTHONPATH:-}" "${RETARGET_PY}" "${NODE}" --mode smpl-pose \
      --pico-host "${PICO_HOST}" --pico-port "${PICO_PORT}" --out-port 5561 </dev/null &
  CHILDREN+=("$!")
else
  echo "[a3-chain] encoder=${ENCODER}: Pico tcp://${PICO_HOST}:${PICO_PORT} 'pose'  ->  A3 joint references :5560  ->  a3_ta_bridge  ->  /ta/whole_body_command"
  # A3_LOWER_BODY=stand: legs fixed to the policy-stand reference (default track = legs follow the operator)
  PYTHONPATH="${REPO_ROOT}:${PYTHONPATH:-}" "${RETARGET_PY}" "${NODE}" --mode a3-ref \
      --pico-host "${PICO_HOST}" --pico-port "${PICO_PORT}" --out-port 5560 --lower-body "${A3_LOWER_BODY:-track}" \
      ${FOOT_ROT_WEIGHT:+--foot-rot-weight "${FOOT_ROT_WEIGHT}"} ${A3_ANKLE_KNEE_ROT_WEIGHT:+--ankle-knee-rot-weight "${A3_ANKLE_KNEE_ROT_WEIGHT}"} \
      ${A3_FOOT_LIFT_SCALE:+--foot-lift-scale "${A3_FOOT_LIFT_SCALE}"} </dev/null &
  CHILDREN+=("$!")
  sleep 1
  if [[ "${NO_BRIDGE}" == "1" ]]; then
    echo "[a3-chain] --no-bridge: the deploy reads tcp://<this PC>:5560 'a3_ref' itself (--teleop-input zmq); no ROS 2 on this side"
    wait -n "${CHILDREN[@]}" || true
    echo "[a3-chain] a chain process exited; stopping the rest"
    exit 0
  fi
  [[ -x "${PKG_DIR}/a3_ta_bridge" ]] || { echo "[a3-chain] ${PKG_DIR}/a3_ta_bridge missing (build the package: deploy.sh --robot a3_ultra --build-only)" >&2; exit 66; }
  BRIDGE_LD="${PKG_DIR}:${LD_LIBRARY_PATH:-}"
  if LD_LIBRARY_PATH="${BRIDGE_LD}" ldd "${PKG_DIR}/a3_ta_bridge" 2>/dev/null | grep -q "not found"; then
    ROS_PREFIX="${A3_ROS_PREFIX:-${CONDA_ROOT}/envs/a3_deploy_verify}"
    if [[ -d "${ROS_PREFIX}/lib" ]]; then
      BRIDGE_LD="${ROS_PREFIX}/lib:${BRIDGE_LD}"
    else
      echo "[a3-chain] a3_ta_bridge cannot resolve its ROS2 libraries: source your ROS2 environment or set A3_ROS_PREFIX" >&2
    fi
  fi
  ( export LD_LIBRARY_PATH="${BRIDGE_LD}"
    exec "${PKG_DIR}/a3_ta_bridge" --zmq tcp://127.0.0.1:5560 ) </dev/null &
  CHILDREN+=("$!")
fi
wait -n "${CHILDREN[@]}" || true
echo "[a3-chain] a chain process exited; stopping the rest"

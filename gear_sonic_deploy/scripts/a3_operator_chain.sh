#!/usr/bin/env bash
# ============================================================================
# Operator-side chain of the A3 / A3 Ultra Pico teleop: everything between
# pico_manager_thread_server.py and the deploy binary, selected with --encoder.
#
#   scripts/a3_operator_chain.sh --encoder a3-fast|g1|smpl [--pico-host HOST] [--pico-port 5556]
#
#   a3-fast / g1 : a3_teleop_retarget_node.py --mode a3-ref  (Pico SMPL -> A3 joint references, ZMQ :5560)
#                  + a3_ta_bridge (ZMQ :5560 -> ROS2 /ta/whole_body_command).  Needs the ROS2 environment of the
#                  robot's network (sim: deploy.sh exports the private domain; real: ROS_DOMAIN_ID, A3_TRANSPORT ...).
#   smpl         : a3_teleop_retarget_node.py --mode smpl-pose  (re-publishes the Pico pose stream with A3 wrist
#                  angles on ZMQ :5561; the deploy's smpl_zmq source subscribes to it).  No ROS needed.
#
# deploy.sh starts this next to the deploy binary for `--input-type zmq_manager` (sim / one machine).  On the real
# robot run it on the operator PC and start deploy.sh on the robot with `--no-chain`.
# Environment: A3_RETARGET_PYTHON (python with mujoco + mink + GMR + pyzmq, default: conda env gmr).
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
while [[ $# -gt 0 ]]; do
  case "$1" in
    --encoder)   ENCODER="$2"; shift 2 ;;
    --pico-host) PICO_HOST="$2"; shift 2 ;;
    --pico-port) PICO_PORT="$2"; shift 2 ;;
    -h|--help)   sed -n 2,22p "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "a3_operator_chain.sh: unknown argument $1" >&2; exit 2 ;;
  esac
done
case "${ENCODER}" in a3-fast|a3_fast|g1|smpl) ;; *) echo "--encoder must be a3-fast, g1 or smpl" >&2; exit 2 ;; esac

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
  PYTHONPATH="${REPO_ROOT}:${PYTHONPATH:-}" "${RETARGET_PY}" "${NODE}" --mode a3-ref \
      --pico-host "${PICO_HOST}" --pico-port "${PICO_PORT}" --out-port 5560 </dev/null &
  CHILDREN+=("$!")
  sleep 1
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

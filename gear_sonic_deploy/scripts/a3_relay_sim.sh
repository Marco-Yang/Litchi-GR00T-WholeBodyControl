#!/usr/bin/env bash
# ============================================================================
# a3_zmq_relay against the MuJoCo A3 simulator (the simulator is a ROS 2 peer; on the real robot the relay uses iceoryx instead).
#
#   scripts/a3_relay_sim.sh [relay options]        e.g.  --bind 127.0.0.1 [--listen-only] [--stale-ms 60 --hold-ms 150 --ramp-ms 300]
#
# Same private ROS domain as deploy.sh sim (A3_SIM_ROS_DOMAIN_ID, default 87, ROS_LOCALHOST_ONLY=1).  Start the simulator first
# (python gear_sonic/scripts/run_sim_loop.py --robot a3_ultra), then this, then
#   ./deploy.sh --robot a3_ultra --input-type zmq_manager sim --encoder a3-fast --teleop-input zmq --robot-link zmq --relay-host 127.0.0.1
# ============================================================================
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GEAR_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
case "$(uname -m)" in x86_64) ARCH=x86_64 ;; aarch64|arm64) ARCH=rockchip ;; *) echo "unsupported machine" >&2; exit 2 ;; esac
PKG_DIR="${GEAR_ROOT}/dist/a3_deploy_${ARCH}"
[[ -x "${PKG_DIR}/run_a3_relay.sh" && -x "${PKG_DIR}/a3_zmq_relay" ]] || {
  echo "[a3-relay] ${PKG_DIR} has no a3_zmq_relay / run_a3_relay.sh: rebuild the package (scripts/build_a3_deploy_pkg.sh --arch ${ARCH})" >&2; exit 66; }
export A3_TRANSPORT=ros2
export ROS_DOMAIN_ID="${A3_SIM_ROS_DOMAIN_ID:-87}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
A3_ROS_PREFIX="${A3_ROS_PREFIX:-${CONDA_PREFIX:-$HOME/miniconda3}/envs/a3_deploy_verify}"
[[ -d "${A3_ROS_PREFIX}/lib" ]] && export LD_LIBRARY_PATH="${A3_ROS_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
echo "[a3-relay] simulator relay: transport=ros2 ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=1"
exec "${PKG_DIR}/run_a3_relay.sh" "$@"

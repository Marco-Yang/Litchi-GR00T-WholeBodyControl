#!/usr/bin/env bash
# Dual RealSense RGB-only camera server for G1 teleop data collection.
# Run ON the robot Orin (192.168.123.164), not on the workstation.
#
# Mapping (verified 2026-10-30):
#   ego_view  <- new neck D435I  serial 242622070951  (forward workspace view)
#   head      <- stock onboard   serial 254522070308  (downward / body view)
#
# Usage (from repo root on the robot):
#   bash gear_sonic_deploy/scripts/g1_dual_realsense_rgb_server.sh
#   bash gear_sonic_deploy/scripts/g1_dual_realsense_rgb_server.sh --port 5555
#
# Before start, frees head RGB (/dev/video10) from Unitree videohub and kills
# any previous publisher on the camera port. Set G1_CAM_SKIP_CLEANUP=1 to skip.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

EGO_SN="${G1_EGO_REALSENSE_SN:-242622070951}"
HEAD_SN="${G1_HEAD_REALSENSE_SN:-254522070308}"
PORT="${G1_CAMERA_PORT:-5555}"

# Allow --port N from passthrough args (used for cleanup + server).
prev=""
for arg in "$@"; do
  if [[ "$prev" == "--port" ]]; then
    PORT="$arg"
  fi
  prev="$arg"
done

cleanup_previous_camera_holders() {
  if [[ "${G1_CAM_SKIP_CLEANUP:-0}" == "1" ]]; then
    echo "[g1-cam] cleanup skipped (G1_CAM_SKIP_CLEANUP=1)"
    return 0
  fi

  echo "[g1-cam] cleanup: stop prior publishers / videohub, free :$PORT and /dev/video10"

  # Previous publishers only (do NOT pkill this .sh — would kill ourselves).
  pkill -9 -f 'g1_dual_realsense_rgb_zmq_server.py' 2>/dev/null || true
  pkill -9 -f 'gear_sonic.camera.composed_camera' 2>/dev/null || true

  if ! command -v sudo >/dev/null 2>&1; then
    echo "[g1-cam] WARN: sudo missing; cannot stop videohub_pc4_chest (root)" >&2
    return 0
  fi

  # Unitree stock service respawns videohub_pc4_chest on /dev/video10 (head RGB).
  # Stopping master_service then killing leftovers avoids Device busy on head.
  sudo systemctl stop master_service 2>/dev/null || true
  sleep 0.3
  sudo pkill -9 -f 'videohub_pc4_chest' 2>/dev/null || true
  sudo pkill -9 -f 'videohub_pc4' 2>/dev/null || true
  sudo fuser -k /dev/video10 2>/dev/null || true
  sudo fuser -k "${PORT}/tcp" 2>/dev/null || true
  sleep 0.5

  if pgrep -af 'videohub_pc4_chest' >/dev/null 2>&1; then
    echo "[g1-cam] WARN: videohub_pc4_chest still alive after cleanup:" >&2
    pgrep -af 'videohub_pc4_chest' >&2 || true
  else
    echo "[g1-cam] cleanup: videohub gone, port ${PORT} cleared"
  fi
}

if [[ -x "$REPO_ROOT/.venv_camera/bin/python" ]]; then
  PY="$REPO_ROOT/.venv_camera/bin/python"
elif command -v python3 >/dev/null; then
  PY=python3
  export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
else
  echo "No python found" >&2
  exit 1
fi

cleanup_previous_camera_holders

echo "[g1-cam] repo=$REPO_ROOT"
echo "[g1-cam] ego_view SN=$EGO_SN  head SN=$HEAD_SN  port=$PORT  python=$PY"
# Prefer the Python 3.8-safe standalone publisher on Orin (no gear_sonic typing deps).
STANDALONE="$REPO_ROOT/gear_sonic_deploy/scripts/g1_dual_realsense_rgb_zmq_server.py"
if [[ -f "$STANDALONE" ]]; then
  export LD_PRELOAD="${LD_PRELOAD:-/lib/aarch64-linux-gnu/libgomp.so.1}"
  EXTRA=()
  if [[ "${G1_SHOW_WRIST_STATUS:-0}" == "1" ]]; then
    EXTRA+=(--show-wrist-status)
  fi
  if [[ -n "${G1_WRIST_STATUS_FILE:-}" ]]; then
    EXTRA+=(--wrist-status-file "$G1_WRIST_STATUS_FILE")
  fi
  exec "$PY" "$STANDALONE" \
    --ego-sn "$EGO_SN" \
    --head-sn "$HEAD_SN" \
    --fps 30 \
    --port "$PORT" \
    "${EXTRA[@]}" \
    "$@"
fi

# Fallback: full composed_camera module (needs Python >=3.10 / .venv_camera).
exec "$PY" -m gear_sonic.camera.composed_camera \
  --ego-view-camera realsense --ego-view-device-id "$EGO_SN" \
  --head-camera realsense --head-device-id "$HEAD_SN" \
  --fps 30 \
  --port "$PORT" \
  "$@"

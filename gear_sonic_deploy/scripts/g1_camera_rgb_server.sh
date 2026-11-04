#!/usr/bin/env bash
# Unified G1 camera ZMQ launcher (run ON Orin).
#
# Default: ego_view + head RealSense → :5555
# With wrist: also left_wrist + right_wrist Abham → :5558
#
# Usage (repo root on robot):
#   bash gear_sonic_deploy/scripts/g1_camera_rgb_server.sh
#   bash gear_sonic_deploy/scripts/g1_camera_rgb_server.sh wrist
#   bash gear_sonic_deploy/scripts/g1_camera_rgb_server.sh --wrist

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

ENABLE_WRIST=0
PASSTHROUGH=()
for arg in "$@"; do
  case "$arg" in
    wrist|--wrist)
      ENABLE_WRIST=1
      ;;
    *)
      PASSTHROUGH+=("$arg")
      ;;
  esac
done

WRIST_PORT="${G1_WRIST_CAMERA_PORT:-5558}"
WRIST_LOG="${G1_WRIST_LOG:-/tmp/g1_wrist_abham.log}"
RS_PORT="${G1_CAMERA_PORT:-5555}"

# 1) Clean RealSense / videohub FIRST (before opening Abham).
#    Never fuser-kill Abham video nodes (often /dev/video10+ after reboot).
echo "[g1-cam] pre-clean RealSense holders / videohub (port ${RS_PORT})"
pkill -9 -f 'g1_dual_realsense_rgb_zmq_server.py' 2>/dev/null || true
pkill -9 -f 'gear_sonic.camera.composed_camera' 2>/dev/null || true
if command -v sudo >/dev/null 2>&1; then
  sudo systemctl stop master_service 2>/dev/null || true
  sleep 0.2
  sudo pkill -9 -f 'videohub_pc4_chest' 2>/dev/null || true
  sudo pkill -9 -f 'videohub_pc4' 2>/dev/null || true
  sudo fuser -k "${RS_PORT}/tcp" 2>/dev/null || true
fi
sleep 0.3

echo "[g1-cam] stop previous wrist publisher (if any)"
pkill -9 -f 'g1_wrist_abham_rgb_zmq_server.py' 2>/dev/null || true
fuser -k "${WRIST_PORT}/tcp" 2>/dev/null || true
sleep 0.2

if [[ "$ENABLE_WRIST" -eq 1 ]]; then
  echo "[g1-cam] mode=ego+head+wrist  (:5555 + :${WRIST_PORT})"
  export G1_SHOW_WRIST_STATUS=1
  export G1_WRIST_STATUS_FILE="${G1_WRIST_STATUS_FILE:-/tmp/g1_wrist_status.json}"
  rm -f "$G1_WRIST_STATUS_FILE" 2>/dev/null || true
  : > "$WRIST_LOG"
  if [[ -x "$REPO_ROOT/.venv_camera/bin/python" ]]; then
    WRIST_PY="$REPO_ROOT/.venv_camera/bin/python"
  else
    WRIST_PY=python3
  fi
  PYTHONUNBUFFERED=1 setsid nohup "$WRIST_PY" -u \
    "$REPO_ROOT/gear_sonic_deploy/scripts/g1_wrist_abham_rgb_zmq_server.py" \
    --port "$WRIST_PORT" \
    --fps 30 \
    --width 640 \
    --height 480 \
    >>"$WRIST_LOG" 2>&1 </dev/null &
  WRIST_PID=$!
  echo "$WRIST_PID" > /tmp/g1_wrist_abham.pid
  echo "[g1-cam] wrist pid=$WRIST_PID  py=$WRIST_PY  log=$WRIST_LOG status=$G1_WRIST_STATUS_FILE"

  ok=0
  for i in $(seq 1 40); do
    if ss -ltn 2>/dev/null | grep -q ":${WRIST_PORT} "; then
      ok=1
      break
    fi
    if ! kill -0 "$WRIST_PID" 2>/dev/null \
        && ! pgrep -f 'g1_wrist_abham_rgb_zmq_server.py' >/dev/null 2>&1; then
      echo "[g1-cam] ERROR: wrist publisher exited before binding :${WRIST_PORT}" >&2
      tail -80 "$WRIST_LOG" >&2 || true
      exit 1
    fi
    sleep 0.25
  done
  if [[ "$ok" -ne 1 ]]; then
    echo "[g1-cam] ERROR: wrist did not listen on :${WRIST_PORT} in time" >&2
    tail -80 "$WRIST_LOG" >&2 || true
    exit 1
  fi
  echo "[g1-cam] wrist OK  LISTEN :${WRIST_PORT}"
else
  echo "[g1-cam] mode=ego+head only  (:5555)"
fi

# 2) Dual RealSense in foreground; skip second cleanup so wrist stays alive.
export G1_CAM_SKIP_CLEANUP=1
exec bash "$REPO_ROOT/gear_sonic_deploy/scripts/g1_dual_realsense_rgb_server.sh" \
  "${PASSTHROUGH[@]+"${PASSTHROUGH[@]}"}"

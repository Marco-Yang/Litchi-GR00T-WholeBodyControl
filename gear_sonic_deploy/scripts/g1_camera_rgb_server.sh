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
  nohup bash "$REPO_ROOT/gear_sonic_deploy/scripts/g1_wrist_abham_rgb_server.sh" \
    --port "$WRIST_PORT" \
    >> "$WRIST_LOG" 2>&1 &
  WRIST_BASH_PID=$!
  echo "[g1-cam] wrist launcher pid=$WRIST_BASH_PID  log=$WRIST_LOG status=$G1_WRIST_STATUS_FILE"

  # Wait until PUB is actually listening (device enum can lag / flake).
  ok=0
  for i in $(seq 1 40); do
    if ss -ltn 2>/dev/null | grep -q ":${WRIST_PORT} "; then
      ok=1
      break
    fi
    # launcher exited early
    if ! kill -0 "$WRIST_BASH_PID" 2>/dev/null \
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

# Dual RealSense takes the foreground terminal (Ctrl+C stops it).
exec bash "$REPO_ROOT/gear_sonic_deploy/scripts/g1_dual_realsense_rgb_server.sh" \
  "${PASSTHROUGH[@]+"${PASSTHROUGH[@]}"}"

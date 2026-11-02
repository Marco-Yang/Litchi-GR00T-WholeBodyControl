#!/usr/bin/env bash
# Dual Abham (UVC) wrist camera ZMQ server for G1 Orin.
# Run ON the robot. Default PUB port 5558 (5555=RealSense, 5556=Pico pose).
#
#   bash gear_sonic_deploy/scripts/g1_wrist_abham_rgb_server.sh
#   bash gear_sonic_deploy/scripts/g1_wrist_abham_rgb_server.sh --port 5558
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

PORT="${G1_WRIST_CAMERA_PORT:-5558}"
prev=""
for arg in "$@"; do
  if [[ "$prev" == "--port" ]]; then
    PORT="$arg"
  fi
  prev="$arg"
done

if [[ -x "$REPO_ROOT/.venv_camera/bin/python" ]]; then
  PY="$REPO_ROOT/.venv_camera/bin/python"
else
  PY=python3
fi

echo "[g1-wrist] checking Abham UVC devices..."
if ! ls /dev/v4l/by-id/*Abham*index0 /dev/v4l/by-id/*HHWei*index0 /dev/v4l/by-id/*HHW*index0 2>/dev/null | head -5; then
  echo "[g1-wrist] ERROR: no Abham by-id nodes. USB-485/wrist hub likely unplugged." >&2
  echo "  Expect HHW002 + HHW001 under /dev/v4l/by-id/ and ttyACM0/1 for BrainCo." >&2
  v4l2-ctl --list-devices 2>&1 | head -40 >&2 || true
  lsusb | grep -E '1c45|1a86' >&2 || true
  exit 1
fi

pkill -9 -f 'g1_wrist_abham_rgb_zmq_server.py' 2>/dev/null || true
fuser -k "${PORT}/tcp" 2>/dev/null || true
sleep 0.3

echo "[g1-wrist] repo=$REPO_ROOT port=$PORT python=$PY"
exec "$PY" "$REPO_ROOT/gear_sonic_deploy/scripts/g1_wrist_abham_rgb_zmq_server.py" \
  --port "$PORT" \
  --fps 30 \
  --width 640 \
  --height 480 \
  "$@"

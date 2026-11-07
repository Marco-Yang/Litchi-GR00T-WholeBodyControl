#!/usr/bin/env bash
# Host terminal launcher (NOT Cursor agent sandbox).
# Needs /dev/nvidia* for SONIC TensorRT.
set -uo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO"

if [[ ! -e /dev/nvidia0 ]]; then
  echo "ERROR: /dev/nvidia0 missing. Run this in a normal host terminal, not the agent sandbox."
  nvidia-smi -L || true
  exit 1
fi
nvidia-smi -L

# Clear leftovers from prior agent attempts
fuser -k 5555/tcp 2>/dev/null || true
fuser -k 5556/tcp 2>/dev/null || true
pkill -9 -f 'run_sim_loop.py' 2>/dev/null || true
pkill -9 -f 'g1_deploy_onnx_ref' 2>/dev/null || true
pkill -9 -f 'replay_dataset_mode2_zmq' 2>/dev/null || true
pkill -9 -f 'record_sim_camera_mp4' 2>/dev/null || true
sleep 2

export MUJOCO_GL=egl
export PYTHONUNBUFFERED=1
exec bash gear_sonic/scripts/run_mujoco_mode2_dataset_replay.sh \
  datasets/walking/2026-11-05-15-57-52 \
  5 \
  datasets/data_check/runs/mujoco_mode2_walking_155752 \
  42

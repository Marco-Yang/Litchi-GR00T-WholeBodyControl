#!/usr/bin/env bash
# Host-side launcher (no Cursor sandbox). Requires /dev/nvidia*.
set -uo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO"
exec bash gear_sonic/scripts/run_mujoco_mode2_dataset_replay.sh \
  datasets/walking/2026-11-05-15-57-52 \
  5 \
  datasets/data_check/runs/mujoco_mode2_walking_155752 \
  42

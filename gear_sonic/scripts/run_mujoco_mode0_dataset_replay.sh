#!/usr/bin/env bash
# Run SONIC v1.1 encode-mode-0 dynamics replay in MuJoCo
# (observation.state qpos + Revo2 hand qpos via ZMQ protocol v1 → deploy → sim).
# New script — does not modify run_mujoco_mode2_*.sh / replay_dataset_mode2_zmq.py.
# Outputs ONLY overview mp4s under outdir.
#
#   bash gear_sonic/scripts/run_mujoco_mode0_dataset_replay.sh \
#     [dataset] [num_episodes] [outdir] [seed]
#
# Or fix the episode list:
#   EPISODES_OVERRIDE="1 7 17 40 47" bash gear_sonic/scripts/run_mujoco_mode0_dataset_replay.sh ...
#
set -uo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export OMNI_KIT_ACCEPT_EULA="${OMNI_KIT_ACCEPT_EULA:-YES}"
export G1_SIM_CAMERA="${G1_SIM_CAMERA:-overview}"

DATASET="${1:-datasets/walking/2026-11-05-15-57-52}"
N="${2:-5}"
OUT="${3:-datasets/data_check/runs/mujoco_mode0_$(date +%Y%m%d_%H%M%S)}"
SEED="${4:-42}"

[[ "$DATASET" = /* ]] || DATASET="$REPO/$DATASET"
[[ "$OUT" = /* ]] || OUT="$REPO/$OUT"

PY_SIM="${REPO}/.venv_sim/bin/python"
PY_DC="${REPO}/.venv_data_collection/bin/python"
[[ -x "$PY_SIM" ]] || PY_SIM="${REPO}/.venv_teleop/bin/python"
[[ -x "$PY_DC" ]] || PY_DC="$PY_SIM"

mkdir -p "$OUT"
WORK="$(mktemp -d /tmp/mujoco_mode0_XXXXXX)"
KEEP_WORK=0
cleanup_work() {
  if [[ "$KEEP_WORK" == "1" ]]; then
    echo "keeping work dir for debug: $WORK"
  else
    rm -rf "$WORK" 2>/dev/null || true
  fi
}
trap cleanup_work EXIT

echo "repo=$REPO dataset=$DATASET n=$N seed=$SEED out=$OUT camera=$G1_SIM_CAMERA work=$WORK encode=mode0/g1"

if [[ -n "${EPISODES_OVERRIDE:-}" ]]; then
  read -r -a EPISODES <<< "${EPISODES_OVERRIDE//,/ }"
else
  mapfile -t EPISODES < <("$PY_DC" - <<PY
import json, random
from pathlib import Path
root = Path("$DATASET")
info = json.loads((root/"meta/info.json").read_text())
n = int(info["total_episodes"])
rng = random.Random(int("$SEED"))
ids = sorted(rng.sample(range(n), k=min(int("$N"), n)))
print("\n".join(str(i) for i in ids))
PY
)
fi
echo "episodes: ${EPISODES[*]}"

descend() { local c; for c in $(pgrep -P "$1" 2>/dev/null); do echo "$c"; descend "$c"; done; }

kill_tree() {
  local root="$1"
  [[ -n "$root" ]] || return 0
  local p
  for p in $(descend "$root") "$root"; do
    kill -INT "$p" 2>/dev/null || true
  done
  sleep 1
  for p in $(descend "$root") "$root"; do
    kill -0 "$p" 2>/dev/null && kill -KILL "$p" 2>/dev/null || true
  done
}

clear_ports() {
  fuser -k 5555/tcp 2>/dev/null || true
  fuser -k 5556/tcp 2>/dev/null || true
  pkill -9 -f 'run_sim_loop.py' 2>/dev/null || true
  pkill -9 -f 'g1_deploy_onnx_ref' 2>/dev/null || true
  pkill -9 -f 'replay_dataset_mode0_zmq' 2>/dev/null || true
  pkill -9 -f 'record_sim_camera_mp4' 2>/dev/null || true
  sleep 2
}

find "$OUT" -mindepth 1 -maxdepth 1 -type d -name 'episode_*' -exec rm -rf {} + 2>/dev/null || true
rm -f "$OUT"/run_meta.txt "$OUT"/host_run.log 2>/dev/null || true

for EP in "${EPISODES[@]}"; do
  EP_TAG="$(printf '%06d' "$EP")"
  CASE="$WORK/ep_$EP_TAG"
  VIDEO_OUT="$OUT/episode_${EP_TAG}.mp4"
  rm -rf "$CASE"
  mkdir -p "$CASE"
  echo "======== episode $EP (mode0/qpos) -> $VIDEO_OUT ========"

  if ss -ltn 2>/dev/null | grep -qE ':(5555|5556)\b'; then
    echo "PORTS BUSY — clearing"
    clear_ports
    if ss -ltn 2>/dev/null | grep -qE ':(5555|5556)\b'; then
      echo "PORTS STILL BUSY"; exit 1
    fi
  fi

  rm -f "$CASE/stop_sim"
  (
    cd "$REPO"
    export MUJOCO_GL G1_SIM_CAMERA
    # Hang until deploy PLANNER is up; 40s leaves margin for Init + y.
    # Do NOT inherit ambient short values (e.g. 8/25).
    export G1_SIM_BAND_RELEASE_S=40
    export G1_SIM_STOP_FILE="$CASE/stop_sim"
    nohup "$PY_SIM" -u gear_sonic/scripts/run_sim_loop.py \
      --robot g1 \
      --no-enable-onscreen \
      --enable-offscreen \
      --enable-image-publish \
      --camera-port 5555 \
      > "$CASE/simloop.log" 2>&1 < /dev/null &
    echo $! > "$CASE/simloop.pid"
  )
  cam_ok=0
  for _ in $(seq 1 60); do
    if ss -ltn 2>/dev/null | grep -qE ':5555\b'; then cam_ok=1; break; fi
    sleep 1
  done
  if [[ "$cam_ok" != "1" ]]; then
    echo "sim camera port 5555 not up"; tail -30 "$CASE/simloop.log" || true
    touch "$CASE/stop_sim" 2>/dev/null || true
    clear_ports
    continue
  fi
  # Give Unitree DDS LowState a few seconds on lo before deploy attaches.
  sleep 5
  echo "sim camera ready $(date +%T)"

  start_deploy() {
    (
      set +u
      cd "$REPO/gear_sonic_deploy" || exit 1
      # shellcheck disable=SC1091
      source scripts/setup_env.sh >"$CASE/setup_env.log" 2>&1 || true
      # stdin 'y' answers deploy.sh "Proceed with deployment?" only.
      # Policy start (operator_state.start) comes later from ZMQ command fifo.
      nohup bash -c "set +u; printf 'y\\n' | ./deploy.sh \
        --robot g1 \
        --cp policy/sonic_v1_1/model \
        --obs-config policy/sonic_v1_1/observation_config.yaml \
        --input-type zmq_manager \
        --motor-kp-scale 4,10=1.5 \
        --motor-kd-scale 4,10=1.5 \
        sim" > "$CASE/deploy.log" 2>&1 < /dev/null &
      echo $! > "$CASE/deploy.pid"
    )
  }

  deploy_ready=0
  for attempt in 1 2; do
    : > "$CASE/deploy.log"
    start_deploy
    sleep 2
    ok=0
    for _ in $(seq 1 120); do
      if grep -q "Lost LowState data connection\|Safety check failed" "$CASE/deploy.log" 2>/dev/null; then
        ok=-1; break
      fi
      if grep -q "Init Done" "$CASE/deploy.log" 2>/dev/null; then ok=1; break; fi
      sleep 1
    done
    if [[ "$ok" == "1" ]]; then
      sleep 3
      if grep -q "Lost LowState data connection\|Safety check failed" "$CASE/deploy.log" 2>/dev/null; then
        ok=-1
      fi
    fi
    if [[ "$ok" == "1" ]]; then
      deploy_ready=1
      break
    fi
    echo "deploy attempt $attempt failed (ok=$ok); restarting deploy"
    kill_tree "$(cat "$CASE/deploy.pid" 2>/dev/null || true)"
    pkill -9 -f 'g1_deploy_onnx_ref' 2>/dev/null || true
    sleep 3
  done
  if [[ "$deploy_ready" != "1" ]]; then
    echo "deploy Init Done failed after retries"; tail -30 "$CASE/deploy.log" || true
    clear_ports; continue
  fi
  echo "deploy ready $(date +%T)"

  FIFO="$CASE/cmd.fifo"
  rm -f "$FIFO" "$CASE/go"
  mkfifo "$FIFO"
  (
    cd "$REPO"
    nohup "$PY_DC" gear_sonic/scripts/replay_dataset_mode0_zmq.py \
      --dataset "$DATASET" \
      --episode "$EP" \
      --command-fifo "$FIFO" \
      --wait-for "$CASE/go" \
      --stand-seconds 1.5 \
      > "$CASE/replay.log" 2>&1 < /dev/null &
    echo $! > "$CASE/replay.pid"
  )
  stream_ok=0
  for _ in $(seq 1 60); do
    if grep -q 'command fifo:' "$CASE/replay.log" 2>/dev/null; then stream_ok=1; break; fi
    if grep -q 'ModuleNotFoundError\|Traceback' "$CASE/replay.log" 2>/dev/null; then break; fi
    if [[ -f "$CASE/replay.pid" ]] && ! kill -0 "$(cat "$CASE/replay.pid")" 2>/dev/null; then break; fi
    sleep 0.5
  done
  if [[ "$stream_ok" != "1" ]]; then
    echo "streamer failed"; tail -30 "$CASE/replay.log" || true
    touch "$CASE/stop_sim" 2>/dev/null || true
    clear_ports
    continue
  fi

  rm -f "$CASE/stop_cam"
  (
    cd "$REPO"
    nohup "$PY_SIM" gear_sonic/scripts/record_sim_camera_mp4.py \
      --host 127.0.0.1 --port 5555 \
      --out "$CASE/replay.mp4" \
      --camera overview \
      --fps 30 \
      --stop-file "$CASE/stop_cam" \
      > "$CASE/cam.log" 2>&1 < /dev/null &
    echo $! > "$CASE/cam.pid"
  )

  timeout 5 bash -c "echo y > \"$FIFO\"" || echo "fifo y write failed"
  echo "planner start $(date +%T)"
  # Confirm control actually started (start_control_ applied).
  ctrl_ok=0
  for _ in $(seq 1 30); do
    if grep -qE 'Lost LowState data connection|Safety check failed, cannot start|Program exiting normally' "$CASE/deploy.log" 2>/dev/null; then
      ctrl_ok=-1; break
    fi
    if grep -qE 'operator_state\.start=true|motion name is planner_motion|Planner enabled' "$CASE/deploy.log" 2>/dev/null; then
      ctrl_ok=1; break
    fi
    sleep 1
  done
  if [[ "$ctrl_ok" != "1" ]]; then
    echo "PLANNER control did not start (ctrl_ok=$ctrl_ok)"
    KEEP_WORK=1
    tail -40 "$CASE/deploy.log" || true
    touch "$CASE/stop_cam" "$CASE/stop_sim" 2>/dev/null || true
    clear_ports
    continue
  fi
  echo "planner control ready $(date +%T)"

  band_ok=0
  for _ in $(seq 1 60); do
    if grep -qE 'Lost LowState data connection|Safety check failed, cannot start|Program exiting normally' "$CASE/deploy.log" 2>/dev/null; then
      echo "deploy died during PLANNER stand"
      KEEP_WORK=1
      tail -30 "$CASE/deploy.log" || true
      touch "$CASE/stop_cam" "$CASE/stop_sim" 2>/dev/null || true
      clear_ports
      band_ok=-1
      break
    fi
    if [[ -f "$CASE/replay.pid" ]] && ! kill -0 "$(cat "$CASE/replay.pid")" 2>/dev/null; then
      echo "streamer died during PLANNER stand"
      KEEP_WORK=1
      tail -40 "$CASE/replay.log" || true
      touch "$CASE/stop_cam" "$CASE/stop_sim" 2>/dev/null || true
      clear_ports
      band_ok=-1
      break
    fi
    if grep -q 'elastic band released' "$CASE/simloop.log" 2>/dev/null; then
      band_ok=1
      break
    fi
    sleep 1
  done
  if [[ "$band_ok" == "-1" ]]; then
    continue
  fi
  if [[ "$band_ok" != "1" ]]; then
    echo "band release timeout (continuing anyway)"
  else
    echo "band released $(date +%T); settle 3s"
  fi
  sleep 3
  if grep -qE 'Lost LowState data connection|Safety check failed, cannot start|Program exiting normally' "$CASE/deploy.log" 2>/dev/null; then
    echo "deploy unhealthy before POSE"; KEEP_WORK=1; clear_ports; continue
  fi
  # Switch to streamed pose follow, then unblock pose publishing.
  timeout 5 bash -c "echo x > \"$FIFO\"" || echo "fifo x write failed"
  sleep 1
  touch "$CASE/go"
  echo "follow started $(date +%T)"

  i=0
  while ! grep -q "^done$" "$CASE/replay.log" 2>/dev/null && [[ $i -lt 120 ]]; do
    if [[ -f "$CASE/replay.pid" ]] && ! kill -0 "$(cat "$CASE/replay.pid")" 2>/dev/null; then
      echo "streamer exited"; break
    fi
    sleep 2
    i=$((i + 1))
  done
  sleep 3
  touch "$CASE/stop_cam" "$CASE/stop_sim"
  sleep 3

  DEPLOY_BIN="$REPO/gear_sonic_deploy/target/release/g1_deploy_onnx_ref"
  [[ -x "$DEPLOY_BIN" ]] || DEPLOY_BIN="$(find "$REPO/gear_sonic_deploy" -name 'g1_deploy_onnx_ref' -type f 2>/dev/null | head -1)"
  kill_tree "$(cat "$CASE/deploy.pid" 2>/dev/null || true)"
  kill_tree "$(cat "$CASE/replay.pid" 2>/dev/null || true)"
  kill_tree "$(cat "$CASE/cam.pid" 2>/dev/null || true)"
  kill_tree "$(cat "$CASE/simloop.pid" 2>/dev/null || true)"
  [[ -n "${DEPLOY_BIN:-}" ]] && fuser -k "$DEPLOY_BIN" 2>/dev/null || true
  clear_ports

  if [[ -f "$CASE/replay.mp4" && -s "$CASE/replay.mp4" ]]; then
    mv -f "$CASE/replay.mp4" "$VIDEO_OUT"
    echo "ok $VIDEO_OUT ($(du -h "$VIDEO_OUT" | awk '{print $1}'))"
    rm -rf "$CASE"
  else
    echo "missing video for episode $EP (see $CASE)"
    KEEP_WORK=1
    tail -30 "$CASE/cam.log" 2>/dev/null || true
    tail -30 "$CASE/replay.log" 2>/dev/null || true
  fi
done

echo "ALL_DONE out=$OUT"
ls -lh "$OUT"/episode_*.mp4 2>/dev/null || true

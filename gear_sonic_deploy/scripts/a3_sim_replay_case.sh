#!/usr/bin/env bash
# Unattended MuJoCo run of a RECORDED Pico session (a3_pose_recorder.py), headless: PD stand (y) -> follow (x) -> the whole recording replayed with
# frame-index timing.  Leaves <outdir>/case_<name>/{sim.csv,dump.bin,simloop.log,deploy.log,replay.log,status}; score it with a3_score_sim_replay.py.
#
#   [A3_SIM_GAINS_JSON=gains.json] [A3_SIM_FRICTION_SCALE=0.2] [A3_FOOT_LIFT_SCALE=1.3] gear_sonic_deploy/scripts/a3_sim_replay_case.sh <name> <recording.pkl> [outdir]
#
# Takes ~3.6 min for a 140 s recording.  Needs the ports 5556 / 5560 free (it refuses to start otherwise).  It kills only the processes it started, by PID
# (the deploy binary is found among the descendants of its own deploy.sh), never by name.  Do not run it while a session of yours uses the sim / deploy.
set +u
NAME="${1:?name}"; REC="$(readlink -f "${2:?recording.pkl}")"
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="$(mkdir -p "${3:-$REPO/sim_replay_runs}" && cd "${3:-$REPO/sim_replay_runs}" && pwd)"
SP="$OUT/case_$NAME"; rm -rf "$SP"; mkdir -p "$SP"; cd "$REPO"
descend() { local c; for c in $(pgrep -P "$1" 2>/dev/null); do echo "$c"; descend "$c"; done; }
CFG="gear_sonic_deploy/dist/a3_deploy_x86_64/config/a3_runtime_config.case_$NAME.yaml"
python3 - "$SP" "$CFG" <<'PY'
import sys,re
sp,cfg=sys.argv[1:3]
s=open("gear_sonic_deploy/dist/a3_deploy_x86_64/config/a3_runtime_config.yaml").read()
s=re.sub(r"(?m)^  dump_ticks: .*$","  dump_ticks: 60000",s); s=re.sub(r"(?m)^  dump_path: .*$",f"  dump_path: {sp}/dump.bin",s)
s=s.replace("evaluation_recording:\n  enabled: true","evaluation_recording:\n  enabled: false",1)
open(cfg,"w").write(s)
PY
if ss -ltn 2>/dev/null | grep -qE ":(5556|5560)\b"; then echo "PORTS BUSY (5556/5560): another sim / deploy / replay is running" | tee "$SP/status"; rm -f "$CFG"; exit 1; fi
( A3_SIM_BAND_RELEASE_S=30 A3_SIM_LOG_CSV="$SP/sim.csv" nohup .venv_teleop/bin/python gear_sonic/scripts/run_sim_loop.py --robot a3_ultra --no-enable-onscreen > "$SP/simloop.log" 2>&1 < /dev/null & echo $! > "$SP/simloop.pid" )
sleep 10
( cd gear_sonic_deploy && source scripts/setup_env.sh >/dev/null 2>&1; env A3_RUNTIME_CFG="$REPO/$CFG" nohup ./deploy.sh --robot a3_ultra --input-type zmq_manager sim --encoder a3-fast > "$SP/deploy.log" 2>&1 < /dev/null & echo $! > "$SP/deploy.pid" )
sleep 14
( nohup .venv_teleop/bin/python gear_sonic_deploy/scripts/a3_pose_replay_raw.py "$REC" --no-commands --command-fifo "$SP/fifo" --wait-for "$SP/go" > "$SP/replay.log" 2>&1 < /dev/null & echo $! > "$SP/replay.pid" )
sleep 5
echo y > "$SP/fifo"; sleep 25            # PD stand; the sim releases the band at its t = 30 s
echo x > "$SP/fifo"; sleep 6             # follow (the deploy log is block-buffered: do not test it here; the dump tells afterwards)
touch "$SP/go"; echo "replay started $(date +%T)" > "$SP/status"
i=0; while ! grep -q "done" "$SP/replay.log" && [ $i -lt 400 ]; do sleep 2; i=$((i+1)); done   # up to 800 s
sleep 6
# no final "y": the (non-looping) replay has exited and a write to a fifo without a reader blocks forever
DP=""; for p in $(descend "$(cat "$SP/deploy.pid")"); do ps -o args= -p "$p" 2>/dev/null | grep -q a3_deploy_onnx_ref && DP="$p"; done
[ -n "$DP" ] && kill -INT "$DP"; sleep 4
[ -n "$DP" ] && kill -0 "$DP" 2>/dev/null && kill "$DP"
for f in replay simloop; do kill "$(cat "$SP/$f.pid")" 2>/dev/null; done
sleep 3
rm -f "$CFG"
echo "finished $(date +%T) deploy_pid=${DP:-none}" >> "$SP/status"; touch "$SP/DONE"
echo "done: $SP  (score: PYTHONPATH=. python gear_sonic_deploy/scripts/a3_score_sim_replay.py --csv $SP/sim.csv --dump $SP/dump.bin)"

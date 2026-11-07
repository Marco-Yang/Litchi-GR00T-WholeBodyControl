#!/usr/bin/env bash
# Prefer Sonic follow (pico_manager + deploy) over exporter/viewer on 32-thread CPUs.
#
# Default split (9950X3D / 0-31):
#   pico_manager     → 0-11    nice -10
#   deploy           → 12-23   nice -10
#   exporter         → 24-30   nice +10
#   camera_viewer    → 31      nice +10   (single CPU, isolated)
#
# Usage:
#   bash gear_sonic_deploy/scripts/pin_data_collection_cpus.sh
#   VIEW_CPU=30 bash gear_sonic_deploy/scripts/pin_data_collection_cpus.sh
set -euo pipefail

NPROC="$(nproc)"
LAST=$((NPROC - 1))

PICO_CPUS="${PICO_CPUS:-0-11}"
DEPLOY_CPUS="${DEPLOY_CPUS:-12-23}"
VIEW_CPU="${VIEW_CPU:-${LAST}}"          # one core for viewer
# exporter: remaining high cores excluding VIEW_CPU
if [[ -z "${EXP_CPUS:-}" ]]; then
  if [[ "$LAST" -ge 1 ]]; then
    EXP_CPUS="24-$((LAST - 1))"
    # if LAST < 25, fall back to a small set not overlapping sonic
    if [[ $((LAST - 1)) -lt 24 ]]; then
      EXP_CPUS="$LAST"
    fi
  else
    EXP_CPUS="0"
  fi
fi

pin() {
  local name="$1" pattern="$2" cpus="$3" nice="$4"
  local pids pid cmd
  pids="$(pgrep -f "$pattern" || true)"
  if [[ -z "$pids" ]]; then
    echo "[pin] SKIP $name (not running)"
    return 0
  fi
  for pid in $pids; do
    [[ -r "/proc/$pid/cmdline" ]] || continue
    cmd="$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true)"
    case "$cmd" in
      *'just run'*|*'sh -cu'*|*bash*pin_data_collection*|*'pgrep '*) continue ;;
    esac
    echo "[pin] $name pid=$pid nice=$nice cpus=$cpus"
    echo "      $cmd"
    taskset -cp "$cpus" "$pid" >/dev/null
    renice -n "$nice" -p "$pid" >/dev/null 2>&1 || true
  done
}

echo "[pin] nproc=$NPROC  pico=$PICO_CPUS  deploy=$DEPLOY_CPUS  exporter=$EXP_CPUS  viewer=$VIEW_CPU"
pin pico_manager 'pico_manager_thread_server.py' "$PICO_CPUS" -10
pin deploy 'g1_deploy_onnx_ref' "$DEPLOY_CPUS" -10
pin exporter 'run_data_exporter.py' "$EXP_CPUS" 10
pin camera_viewer 'run_camera_viewer.py' "$VIEW_CPU" 10

echo "[pin] done:"
ps -eo pid,psr,pcpu,nice,cmd | grep -E 'pico_manager_thread_server|run_data_exporter|g1_deploy_onnx_ref|run_camera_viewer' | grep -v grep || true
echo "[pin] affinity:"
for pat in 'pico_manager_thread_server.py' 'g1_deploy_onnx_ref' 'run_data_exporter.py' 'run_camera_viewer.py'; do
  for pid in $(pgrep -f "$pat" || true); do
    cmd="$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true)"
    case "$cmd" in *'just run'*|*'sh -cu'*|*bash*) continue ;; esac
    echo -n "  pid=$pid "; taskset -cp "$pid" 2>/dev/null || true
  done
done

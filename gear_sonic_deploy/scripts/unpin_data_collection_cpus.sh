#!/usr/bin/env bash
# Undo CPU affinity / nice from pin_data_collection_cpus.sh.
# Restores all listed processes to the full CPU set and nice=0.
#
# Usage:
#   bash gear_sonic_deploy/scripts/unpin_data_collection_cpus.sh
set -euo pipefail

NPROC="$(nproc)"
ALL_CPUS="0-$((NPROC - 1))"

unpin() {
  local name="$1" pattern="$2"
  local pids pid cmd
  pids="$(pgrep -f "$pattern" || true)"
  if [[ -z "$pids" ]]; then
    echo "[unpin] SKIP $name (not running)"
    return 0
  fi
  for pid in $pids; do
    [[ -r "/proc/$pid/cmdline" ]] || continue
    cmd="$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true)"
    case "$cmd" in
      *'just run'*|*'sh -cu'*|*bash*unpin_data_collection*|*'pgrep '*) continue ;;
    esac
    echo "[unpin] $name pid=$pid -> cpus=$ALL_CPUS nice=0"
    taskset -cp "$ALL_CPUS" "$pid" >/dev/null
    renice -n 0 -p "$pid" >/dev/null 2>&1 || true
  done
}

echo "[unpin] restore affinity to $ALL_CPUS"
unpin pico_manager 'pico_manager_thread_server.py'
unpin deploy 'g1_deploy_onnx_ref'
unpin exporter 'run_data_exporter.py'
unpin camera_viewer 'run_camera_viewer.py'

echo "[unpin] done:"
ps -eo pid,psr,pcpu,nice,cmd | grep -E 'pico_manager_thread_server|run_data_exporter|g1_deploy_onnx_ref|run_camera_viewer' | grep -v grep || true
echo "[unpin] affinity:"
for pat in 'pico_manager_thread_server.py' 'g1_deploy_onnx_ref' 'run_data_exporter.py' 'run_camera_viewer.py'; do
  for pid in $(pgrep -f "$pat" || true); do
    cmd="$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true)"
    case "$cmd" in *'just run'*|*'sh -cu'*|*bash*) continue ;; esac
    echo -n "  pid=$pid "; taskset -cp "$pid" 2>/dev/null || true
  done
done

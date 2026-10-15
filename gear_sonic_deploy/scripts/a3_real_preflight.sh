#!/usr/bin/env bash
# ============================================================================
# A3 / A3 Ultra real-robot preflight.  READ-ONLY: it publishes nothing, starts nothing, changes nothing.
#
#   robot side (run on the MDU, inside the copied package; also staged into the package root):
#       ./a3_real_preflight.sh [--operator-host IP] [--pico-port 5556]
#   operator side (the PC with the headset; from the repo):
#       gear_sonic_deploy/scripts/a3_real_preflight.sh --operator [--robot-host IP]
#
# Prints PASS / WARN / FAIL per check; the exit code is 1 when anything FAILED.  It mirrors the vendor bring-up
# (docs/a3_training2sim2deploy.md section 6.1: the stock motion_control must not run next to SONIC, aarch64 + RKNN package,
# A3_ROBOT_ENV, transport) and adds what the Pico teleop needs (ROS 2 domain shared with the operator PC, the Pico command channel).
# Passing it does not make the robot safe: a gantry / sling, a physical emergency stop and a supervisor are still required.
# ============================================================================
set -uo pipefail

MODE="robot"
OPERATOR_HOST=""
ROBOT_HOST=""
PICO_PORT=5556
while [[ $# -gt 0 ]]; do
  case "$1" in
    --operator)      MODE="operator"; shift ;;
    --operator-host) OPERATOR_HOST="$2"; shift 2 ;;
    --robot-host)    ROBOT_HOST="$2"; shift 2 ;;
    --pico-port)     PICO_PORT="$2"; shift 2 ;;
    -h|--help)       sed -n 2,16p "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "a3_real_preflight.sh: unknown argument $1" >&2; exit 2 ;;
  esac
done

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -x "${HERE}/a3_deploy_onnx_ref" ]]; then
  PKG="${HERE}"                                   # inside the package
else
  case "$(uname -m)" in aarch64|arm64) ARCH=rockchip ;; *) ARCH=x86_64 ;; esac
  PKG="$(cd -- "${HERE}/.." && pwd)/dist/a3_deploy_${ARCH}"   # repo checkout
fi

FAILS=0; WARNS=0
pass() { printf '  [PASS] %s\n' "$*"; }
warn() { printf '  [WARN] %s\n' "$*"; WARNS=$((WARNS+1)); }
fail() { printf '  [FAIL] %s\n' "$*"; FAILS=$((FAILS+1)); }
section() { printf '\n== %s\n' "$*"; }
cfg_val() { # cfg_val FILE KEY -> first "KEY: value" (indentation ignored, trailing comment stripped)
  grep -E "^[[:space:]]*$2:" "$1" 2>/dev/null | head -1 | sed -E "s/^[[:space:]]*$2:[[:space:]]*//; s/[[:space:]]*#.*$//"
}
tcp_open() { (exec 3<>"/dev/tcp/$1/$2") 2>/dev/null; }
ping_ok() { ping -c1 -W2 "$1" >/dev/null 2>&1; }

echo "A3 real-robot preflight (${MODE} side), package: ${PKG}"

if [[ "${MODE}" == "robot" ]]; then
  section "package"
  BIN="${PKG}/a3_deploy_onnx_ref"; CFG="${PKG}/config/a3_runtime_config.yaml"
  if [[ -x "${BIN}" ]]; then pass "a3_deploy_onnx_ref present"; else fail "missing ${BIN} (build / copy the package)"; fi
  if command -v file >/dev/null 2>&1 && [[ -x "${BIN}" ]]; then
    case "$(uname -m)" in
      aarch64|arm64) file "${BIN}" | grep -q 'ARM aarch64' && pass "binary is aarch64" || fail "this is an aarch64 machine but the binary is not aarch64 (an x86 package was copied?)" ;;
      *) warn "this is $(uname -m): the robot runs the aarch64 (Rockchip / Thor) package, not this one" ;;
    esac
  fi
  if [[ -f "${CFG}" ]]; then
    pass "runtime config ${CFG##*/}"
    case "$(uname -m)" in
      aarch64|arm64) grep -Eq '^[[:space:]]*backend:[[:space:]]*rknn[[:space:]]*$' "${CFG}" && pass "onnx.backend: rknn" || warn "onnx.backend is not rknn in ${CFG##*/} (Rockchip packages are expected to use RKNN; Thor uses its own runtime)" ;;
    esac
    [[ "$(cfg_val "${CFG}" dry_run)" == "false" ]] && pass "backend.dry_run: false" || warn "backend.dry_run is not false: no commands would be published"
    [[ "$(cfg_val "${CFG}" hand_bridge_enabled)" == "false" ]] && pass "hand_bridge_enabled: false (fingers are not driven)" || warn "hand_bridge_enabled is not false: the hand bridge publishes finger commands"
    echo "  [info] watchdog max_frame_age_ms=$(cfg_val "${CFG}" max_frame_age_ms), stale_behavior=$(cfg_val "${CFG}" stale_behavior), pd_stand_ticks=$(cfg_val "${CFG}" pd_stand_ticks), teleop.default_source=$(cfg_val "${CFG}" default_source)"
  else
    fail "missing ${CFG}"
  fi
  for m in model_step_200000_g1 model_step_200000_a3_fast; do
    if ls "${PKG}/models/${m}".onnx "${PKG}/models/${m}".rknn >/dev/null 2>&1; then pass "models/${m} (.onnx and .rknn)"
    elif ls "${PKG}/models/${m}".* >/dev/null 2>&1; then warn "models/${m}: only $(cd "${PKG}/models" && ls "${m}".* | tr '\n' ' ')"
    else fail "models/${m}.* missing"; fi
  done

  section "vendor environment"
  if [[ -n "${A3_ROBOT_ENV:-}" && -r "${A3_ROBOT_ENV}" ]]; then pass "A3_ROBOT_ENV=${A3_ROBOT_ENV}"
  else warn "A3_ROBOT_ENV is unset or unreadable (Rockchip packages source it by default; export A3_ROBOT_ENV=/agibot/software/v0/entry/env/env.sh)"; fi
  transport="${A3_TRANSPORT:-iceoryx}"
  case "${transport}" in
    iceoryx|ros2) [[ -f "${PKG}/config/a3_aimrt_config.${transport}.yaml" ]] && pass "A3_TRANSPORT=${transport}" || fail "no config/a3_aimrt_config.${transport}.yaml" ;;
    *) fail "A3_TRANSPORT='${transport}' (expected iceoryx or ros2)" ;;
  esac
  if pgrep -x motion_control >/dev/null 2>&1; then
    fail "the stock motion_control is running: it and SONIC must never publish body-drive commands together (vendor section 6.1 step 1-2: Motion group -> [ \"agent\" ] only)"
  else pass "motion_control is not running"; fi
  if pgrep -x a3_deploy_onnx_ref >/dev/null 2>&1; then fail "another a3_deploy_onnx_ref is already running"; else pass "no other a3_deploy_onnx_ref"; fi
  if pgrep -f run_sim_loop.py >/dev/null 2>&1; then fail "run_sim_loop.py is running: the simulator and the real robot must not share a ROS domain"; fi

  section "network to the operator PC (a3-fast teleop: /ta/whole_body_command over ROS 2 + Pico commands over ZMQ)"
  if [[ -n "${ROS_DOMAIN_ID:-}" && "${ROS_DOMAIN_ID}" != "0" ]]; then pass "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} (must be the same on the operator PC)"
  else warn "ROS_DOMAIN_ID is ${ROS_DOMAIN_ID:-unset}: use the site's dedicated domain, identical on the robot and the operator PC (domain 0 is shared with every ROS 2 participant)"; fi
  [[ "${ROS_LOCALHOST_ONLY:-0}" == "1" ]] && fail "ROS_LOCALHOST_ONLY=1: the operator PC's /ta/whole_body_command can never arrive" || pass "ROS_LOCALHOST_ONLY is not 1"
  if [[ -n "${OPERATOR_HOST}" ]]; then
    ping_ok "${OPERATOR_HOST}" && pass "operator PC ${OPERATOR_HOST} answers ping" || warn "operator PC ${OPERATOR_HOST} does not answer ping (ICMP may be filtered)"
    tcp_open "${OPERATOR_HOST}" "${PICO_PORT}" && pass "Pico command channel tcp://${OPERATOR_HOST}:${PICO_PORT} is open" \
      || warn "tcp://${OPERATOR_HOST}:${PICO_PORT} is closed: start pico_manager_thread_server.py on the operator PC (without it A+B+X+Y never arrives; use the keyboard S / T / P)"
  else
    warn "no --operator-host given: pass the operator PC's IP and start the deploy with --zmq-host <that IP>, otherwise the deploy listens for Pico commands on this machine only"
  fi
  section "disk"
  avail="$(df -Pm "${PKG}" 2>/dev/null | awk 'NR==2{print $4}')"
  [[ -n "${avail}" && "${avail}" -gt 1024 ]] && pass "${avail} MB free for eval_records / logs" || warn "less than 1 GB free next to the package (eval recording writes there)"
else
  section "operator PC"
  pgrep -x RoboticsServiceProcess >/dev/null 2>&1 && pass "XRoboToolkit PC Service is running" || fail "RoboticsServiceProcess is not running (bash /opt/apps/roboticsservice/runService.sh)"
  CONDA_ROOT="${CONDA_ROOT:-$HOME/miniconda3}"
  RETARGET_PY="${A3_RETARGET_PYTHON:-${CONDA_ROOT}/envs/gmr/bin/python}"
  if [[ -x "${RETARGET_PY}" ]] && "${RETARGET_PY}" -c "import mujoco, mink, zmq, general_motion_retargeting" >/dev/null 2>&1; then pass "retarget python (mujoco, mink, zmq, GMR) ${RETARGET_PY}"
  else fail "retarget python lacks mujoco / mink / zmq / GMR: ${RETARGET_PY} (set A3_RETARGET_PYTHON)"; fi
  if [[ -x "${PKG}/a3_ta_bridge" ]]; then
    if LD_LIBRARY_PATH="${PKG}:${A3_ROS_PREFIX:-${CONDA_ROOT}/envs/a3_deploy_verify}/lib:${LD_LIBRARY_PATH:-}" ldd "${PKG}/a3_ta_bridge" 2>/dev/null | grep -q "not found"; then
      fail "a3_ta_bridge cannot resolve its libraries (source ROS 2 or set A3_ROS_PREFIX)"
    else pass "a3_ta_bridge resolves its libraries"; fi
  else fail "missing ${PKG}/a3_ta_bridge"; fi
  if [[ -n "${ROS_DOMAIN_ID:-}" && "${ROS_DOMAIN_ID}" != "0" ]]; then pass "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} (must equal the robot's)"
  else fail "ROS_DOMAIN_ID is ${ROS_DOMAIN_ID:-unset}/0: set the robot's domain"; fi
  [[ "${ROS_LOCALHOST_ONLY:-0}" == "1" ]] && fail "ROS_LOCALHOST_ONLY=1: the bridge would never reach the robot (export ROS_LOCALHOST_ONLY=0)" || pass "ROS_LOCALHOST_ONLY is not 1"
  for p in "${PICO_PORT}" 5560; do
    ss -ltn 2>/dev/null | grep -Eq "[:.]${p}[[:space:]]" && warn "port ${p} is already in use (a previous run?)" || pass "port ${p} is free"
  done
  if [[ -n "${ROBOT_HOST}" ]]; then ping_ok "${ROBOT_HOST}" && pass "robot ${ROBOT_HOST} answers ping" || warn "robot ${ROBOT_HOST} does not answer ping"; fi
  pgrep -f run_sim_loop.py >/dev/null 2>&1 && fail "run_sim_loop.py is running: stop the simulator"
fi

echo
printf 'Result: %d FAIL, %d WARN.\n' "${FAILS}" "${WARNS}"
cat <<'EOF'
Not checked by any script: a safety gantry / sling is attached, a physical emergency stop is in reach, a supervisor other than the
operator is present, the robot has been validated in MuJoCo, and the first runs are short and small-amplitude.
EOF
[[ "${FAILS}" -eq 0 ]]

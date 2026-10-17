#!/usr/bin/env bash
# ============================================================================
# A3 / A3 Ultra real-robot preflight.  READ-ONLY: it publishes nothing, starts nothing, changes nothing.
#
#   robot side (run on the MDU, inside the copied package; also staged into the package root):
#       ./a3_real_preflight.sh [--operator-host IP] [--pico-port 5556]
#   robot side, route C (only the ZMQ relay is on the MDU; inside the relay bundle a3_relay_<arch>/):
#       ./a3_real_preflight.sh --relay [--bind-ip 10.42.10.12] [--listen-only]
#   operator side, route C (no ROS 2 between this PC and the robot; --relay-host also checks that the relay answers on its ZMQ state port):
#       gear_sonic_deploy/scripts/a3_real_preflight.sh --operator --relay-host 10.42.10.12
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
BIND_IP=""
LISTEN_ONLY=0
RELAY_HOST=""
RELAY_STATE_PORT=5570
OPERATOR_HOST=""
ROBOT_HOST=""
PICO_PORT=5556
while [[ $# -gt 0 ]]; do
  case "$1" in
    --operator)      MODE="operator"; shift ;;
    --relay)         MODE="relay"; shift ;;
    --relay-host)    RELAY_HOST="$2"; shift 2 ;;
    --relay-state-port) RELAY_STATE_PORT="$2"; shift 2 ;;
    --bind-ip)       BIND_IP="$2"; shift 2 ;;
    --listen-only)   LISTEN_ONLY=1; shift ;;
    --operator-host) OPERATOR_HOST="$2"; shift 2 ;;
    --robot-host)    ROBOT_HOST="$2"; shift 2 ;;
    --pico-port)     PICO_PORT="$2"; shift 2 ;;
    -h|--help)       sed -n 2,20p "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "a3_real_preflight.sh: unknown argument $1" >&2; exit 2 ;;
  esac
done

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -x "${HERE}/a3_deploy_onnx_ref" || -x "${HERE}/a3_zmq_relay" ]]; then
  PKG="${HERE}"                                   # inside the package
else
  case "$(uname -m)" in aarch64|arm64) ARCH=rockchip ;; *) ARCH=x86_64 ;; esac
  PKG="$(cd -- "${HERE}/.." && pwd)/dist/a3_deploy_${ARCH}"   # repo checkout
fi

# NOTE: pgrep -x compares the kernel process name, which is cut to 15 characters ("a3_deploy_onnx_" for a3_deploy_onnx_ref), so it never matches
# the long binary names: those checks match the command line (-f) instead.
FAILS=0; WARNS=0
pass() { printf '  [PASS] %s\n' "$*"; }
warn() { printf '  [WARN] %s\n' "$*"; WARNS=$((WARNS+1)); }
fail() { printf '  [FAIL] %s\n' "$*"; FAILS=$((FAILS+1)); }
section() { printf '\n== %s\n' "$*"; }
cfg_val() { # cfg_val FILE KEY -> first "KEY: value" (indentation ignored, trailing comment stripped)
  grep -E "^[[:space:]]*$2:" "$1" 2>/dev/null | head -1 | sed -E "s/^[[:space:]]*$2:[[:space:]]*//; s/[[:space:]]*#.*$//"
}
# the vendor's motion control: its kernel process name may be mc, motion_control or (AimRT renames its processes, e.g. "aimrt_main") neither,
# so also match the command line when the executable is called motion_control or mc
mc_running() {
  pgrep -x mc >/dev/null 2>&1 || pgrep -x motion_control >/dev/null 2>&1 || pgrep -f '(^|/)(motion_control|mc)( |$)' >/dev/null 2>&1
}
tcp_open() { (exec 3<>"/dev/tcp/$1/$2") 2>/dev/null; }
ping_ok() { ping -c1 -W2 "$1" >/dev/null 2>&1; }

echo "A3 real-robot preflight (${MODE} side), package: ${PKG}"

if [[ "${MODE}" == "relay" ]]; then
  # Route C, robot side: only a3_zmq_relay runs here (iceoryx to body-drive on one side, ZMQ to the operator PC on the other).  Read-only.
  section "relay package"
  RBIN="${PKG}/a3_zmq_relay"; ICFG="${PKG}/config/a3_aimrt_config.iceoryx.yaml"
  if [[ -x "${RBIN}" ]]; then pass "a3_zmq_relay present"; else fail "missing ${RBIN} (copy the relay bundle a3_relay_<arch>/)"; fi
  if [[ -x "${PKG}/run_a3_relay.sh" && -x "${PKG}/run_a3.sh" ]]; then pass "run_a3_relay.sh / run_a3.sh present"; else fail "run_a3_relay.sh / run_a3.sh missing or not executable"; fi
  if [[ -f "${ICFG}" ]]; then pass "config/a3_aimrt_config.iceoryx.yaml"; else fail "missing ${ICFG}"; fi
  if command -v file >/dev/null 2>&1 && [[ -x "${RBIN}" ]]; then
    case "$(uname -m)" in
      aarch64|arm64) if file "${RBIN}" | grep -q 'ARM aarch64'; then pass "relay binary is aarch64"; else fail "this is an aarch64 machine but the relay binary is not aarch64"; fi ;;
      *) warn "this is $(uname -m): the MDU runs the aarch64 relay, not this one" ;;
    esac
  fi
  if [[ -f "${ICFG}" ]] && grep -q "libaimrt_net_plugin.so" "${ICFG}"; then
    echo "  [info] the stock AimRT config also loads net_plugin (HTTP on 127.0.0.1:50901) and ros2_plugin (a ROS 2 node in the vendor's domain): the same as the vendor-validated deploy"
  fi
  section "shared libraries (ldd follows the binaries' RUNPATH: \$ORIGIN, /opt/ros/jazzy/lib)"
  if command -v ldd >/dev/null 2>&1 && [[ -x "${RBIN}" ]]; then
    for lib in a3_zmq_relay libaimrt_iceoryx_plugin.so libaimrt_ros2_plugin.so libaimrt_net_plugin.so; do
      [[ -e "${PKG}/${lib}" ]] || continue
      if [[ "$(uname -m)" == "x86_64" ]]; then ros_lib="${A3_ROS_PREFIX:-${HOME}/miniconda3/envs/a3_deploy_verify}/lib"; else ros_lib="/opt/ros/jazzy/lib:/opt/ros/humble/lib"; fi
      ldd_out="$(LD_LIBRARY_PATH="${PKG}:${ros_lib}:${LD_LIBRARY_PATH:-}" ldd "${PKG}/${lib}" 2>&1 || true)"
      if grep -q "not a dynamic executable" <<<"${ldd_out}"; then
        echo "  [info] ${lib}: ldd cannot inspect it on this machine ($(uname -m)): run this preflight on the MDU"
      elif grep -q "not found" <<<"${ldd_out}"; then
        fail "${lib}: missing shared libraries: $(grep 'not found' <<<"${ldd_out}" | awk '{print $1}' | tr '\n' ' ')"
        if grep 'libzmq' <<<"${ldd_out}" | grep -q 'not found'; then
          echo "         libzmq.so.5: the bundle does not carry it (plan 18.20). Install the system libzmq5 (needs root / the vendor)."
        fi
      else
        pass "${lib}: all shared libraries resolve"
      fi
    done
  fi
  section "vendor environment and robot side"
  if [[ -n "${A3_ROBOT_ENV:-}" && -r "${A3_ROBOT_ENV}" ]]; then pass "A3_ROBOT_ENV=${A3_ROBOT_ENV}"
  else warn "A3_ROBOT_ENV is unset or unreadable (run_a3.sh needs it: export A3_ROBOT_ENV=/agibot/software/v0/entry/env/env.sh)"; fi
  if mc_running; then
    if [[ "${LISTEN_ONLY}" == "1" ]]; then
      warn "the stock motion_control (mc) is running: fine for --listen-only (the relay creates no command publisher), but stop it before the relay may forward commands"
    else
      fail "the stock motion_control (mc) is running: it and the relay must never publish body-drive commands together (stop it first; plan 18.18)"
    fi
  else pass "motion_control (mc) is not running"; fi
  if pgrep -f "roudi" >/dev/null 2>&1; then pass "iceoryx RouDi is running"; else warn "no RouDi process found: the relay's iceoryx side has nothing to attach to (the vendor starts agibot_roudi)"; fi
  if pgrep -f '(^|/)a3_zmq_relay( |$)' >/dev/null 2>&1; then fail "another a3_zmq_relay is already running"; else pass "no other a3_zmq_relay"; fi
  if pgrep -f '(^|/)a3_deploy_onnx_ref( |$)' >/dev/null 2>&1; then fail "a full a3_deploy_onnx_ref is running on this machine: relay and deploy would both write body-drive"; else pass "no a3_deploy_onnx_ref on this machine"; fi
  for p in 5570 5571 50901; do
    if ss -ltn 2>/dev/null | grep -Eq "[:.]${p}[[:space:]]"; then fail "port ${p} is already in use (relay state 5570 / command 5571 / AimRT http 50901)"; else pass "port ${p} is free"; fi
  done
  if [[ -n "${BIND_IP}" ]]; then
    if command -v ip >/dev/null 2>&1 && ip -o addr 2>/dev/null | grep -qw "${BIND_IP}"; then pass "${BIND_IP} is an address of this machine (use it for --bind)"
    else fail "${BIND_IP} is not an address of this machine: the relay could not bind there"; fi
  else warn "no --bind-ip given: pass the robot-network address you will use for --bind (10.42.10.12)"; fi
  echo "  [info] load: $(cut -d' ' -f1-3 /proc/loadavg 2>/dev/null)  cores: $(nproc 2>/dev/null)  free disk here: $(df -h "${PKG}" 2>/dev/null | awk 'NR==2{print $4}')"
  echo
  printf 'Result: %d FAIL, %d WARN.\n' "${FAILS}" "${WARNS}"
  cat <<'NOTE'
Not checked: that body-drive is actually publishing, the vendor's behaviour when commands stop (hal_ethercat timeout, unanswered), the operator
PC side, a gantry / sling, a physical emergency stop, a supervisor.  Start with --listen-only and check the link from the PC before any command.
NOTE
  [[ "${FAILS}" -eq 0 ]]
  exit $?
fi

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
  # shared libraries: ldd honours the binaries' RUNPATH ($ORIGIN:/opt/ros/jazzy/lib:...), so this needs no vendor env.  The aarch64 package does NOT
  # bundle libzmq.so.5 (a3_deploy_onnx_ref and a3_zmq_relay both need it; plan 18.20): on the MDU it must come from the system.
  if command -v ldd >/dev/null 2>&1; then
    for b in a3_deploy_onnx_ref a3_zmq_relay; do
      [[ -x "${PKG}/${b}" ]] || continue
      # on the x86_64 dev PC the ROS 2 libraries come from the conda env (deploy_a3.sh does the same); on the MDU from /opt/ros via RUNPATH
      if [[ "$(uname -m)" == "x86_64" ]]; then ros_lib="${A3_ROS_PREFIX:-${HOME}/miniconda3/envs/a3_deploy_verify}/lib"; else ros_lib="/opt/ros/jazzy/lib:/opt/ros/humble/lib"; fi
      ldd_out="$(LD_LIBRARY_PATH="${PKG}:${ros_lib}:${LD_LIBRARY_PATH:-}" ldd "${PKG}/${b}" 2>&1 || true)"
      if grep -q "not a dynamic executable" <<<"${ldd_out}"; then
        echo "  [info] ${b}: ldd cannot inspect it on this machine ($(uname -m) vs the package's architecture): run this preflight on the MDU"
      elif grep -q "not found" <<<"${ldd_out}"; then
        fail "${b}: missing shared libraries: $(grep 'not found' <<<"${ldd_out}" | awk '{print $1}' | tr '\n' ' ')"
        grep -q "libzmq" <<<"${ldd_out}" && grep 'libzmq' <<<"${ldd_out}" | grep -q 'not found' && \
          echo "         libzmq.so.5: the package does not bundle it. Install the system libzmq5 (needs root / the vendor), or copy libzmq.so.5 AND its dependencies (libsodium, libpgm, libnorm, libbsd, libgssapi_krb5) from the sysroot into ${PKG}"
      else
        pass "${b}: all shared libraries resolve"
      fi
    done
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
  if mc_running; then
    fail "the stock motion_control is running: it and SONIC must never publish body-drive commands together (vendor section 6.1 step 1-2: Motion group -> [ \"agent\" ] only)"
  else pass "motion_control is not running"; fi
  if pgrep -f '(^|/)a3_deploy_onnx_ref( |$)' >/dev/null 2>&1; then fail "another a3_deploy_onnx_ref is already running"; else pass "no other a3_deploy_onnx_ref"; fi
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
  pgrep -f '(^|/)RoboticsServiceProcess( |$)' >/dev/null 2>&1 && pass "XRoboToolkit PC Service is running" || fail "RoboticsServiceProcess is not running (bash /opt/apps/roboticsservice/runService.sh)"
  CONDA_ROOT="${CONDA_ROOT:-$HOME/miniconda3}"
  RETARGET_PY="${A3_RETARGET_PYTHON:-${CONDA_ROOT}/envs/gmr/bin/python}"
  if [[ -x "${RETARGET_PY}" ]] && "${RETARGET_PY}" -c "import mujoco, mink, zmq, general_motion_retargeting" >/dev/null 2>&1; then pass "retarget python (mujoco, mink, zmq, GMR) ${RETARGET_PY}"
  else fail "retarget python lacks mujoco / mink / zmq / GMR: ${RETARGET_PY} (set A3_RETARGET_PYTHON)"; fi
  if [[ -n "${RELAY_HOST}" ]]; then
    echo "  [info] route C (--relay-host ${RELAY_HOST}): the deploy reaches the robot over ZMQ, so the ROS 2 / a3_ta_bridge checks are skipped (use --teleop-input zmq)"
  elif [[ -x "${PKG}/a3_ta_bridge" ]]; then
    if LD_LIBRARY_PATH="${PKG}:${A3_ROS_PREFIX:-${CONDA_ROOT}/envs/a3_deploy_verify}/lib:${LD_LIBRARY_PATH:-}" ldd "${PKG}/a3_ta_bridge" 2>/dev/null | grep -q "not found"; then
      fail "a3_ta_bridge cannot resolve its libraries (source ROS 2 or set A3_ROS_PREFIX)"
    else pass "a3_ta_bridge resolves its libraries"; fi
  else fail "missing ${PKG}/a3_ta_bridge"; fi
  if [[ -z "${RELAY_HOST}" ]]; then
    if [[ -n "${ROS_DOMAIN_ID:-}" && "${ROS_DOMAIN_ID}" != "0" ]]; then pass "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} (must equal the robot's)"
    else fail "ROS_DOMAIN_ID is ${ROS_DOMAIN_ID:-unset}/0: set the robot's domain"; fi
    [[ "${ROS_LOCALHOST_ONLY:-0}" == "1" ]] && fail "ROS_LOCALHOST_ONLY=1: the bridge would never reach the robot (export ROS_LOCALHOST_ONLY=0)" || pass "ROS_LOCALHOST_ONLY is not 1"
  else
    ping_ok "${RELAY_HOST}" && pass "relay host ${RELAY_HOST} answers ping" || warn "relay host ${RELAY_HOST} does not answer ping"
    if tcp_open "${RELAY_HOST}" "${RELAY_STATE_PORT}"; then pass "something accepts connections on ${RELAY_HOST}:${RELAY_STATE_PORT} (the relay's state port; confirm with a3_relay_tool.py listen)"
    else warn "nothing accepts connections on ${RELAY_HOST}:${RELAY_STATE_PORT}: the relay is not running yet (start it on the MDU first)"; fi
    PYZMQ="${A3_RETARGET_PYTHON:-${CONDA_ROOT}/envs/gmr/bin/python}"
    "${PYZMQ}" -c "import zmq" >/dev/null 2>&1 && pass "pyzmq is available for a3_relay_tool.py (${PYZMQ})" || warn "no pyzmq in ${PYZMQ}: a3_relay_tool.py needs it (conda base / system python3 do not have it)"
  fi
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

#!/usr/bin/env bash
# ============================================================================
# A3 / A3 Ultra deploy launcher  (called by deploy.sh --robot a3|a3_ultra)
#
#   scripts/deploy_a3.sh [--robot a3|a3_ultra] [sim|real] [--encoder a3-fast|smpl|g1] [--smpl-model PATH|stub]
#                        [--input-type zmq_manager] [--zmq-host HOST] [--no-chain]
#                        [--teleop-input ta|zmq] [--remote [--ros-iface-ip IP]] [--robot-link zmq --relay-host HOST [--relay-state-port N] [--relay-command-port N]]
#                        [--build-only] [--no-build] [--yes] [-- <a3_deploy_onnx_ref args>]
#
# 1. Makes sure the 035 ONNX/RKNN weights are present (download_from_hf.py --robot a3).
# 2. Makes sure the deploy package dist/a3_deploy_<arch>/ exists (build_a3_deploy_pkg.sh).
# 3. Runs the package launcher run_a3.sh  (AimRT: A3_TRANSPORT=iceoryx|ros2).
#
# --encoder  which encoder drives TELEOP (default a3-fast).  The deploy starts with that source selected and LOCKED
#        (keys 1/2/3 cannot switch it), a source that is not available is a startup error, and the matching
#        operator-side chain is started (scripts/a3_operator_chain.sh):
#          a3-fast : Pico SMPL -> GMR retargeting -> A3 joint references -> a3_ta_bridge -> /ta/whole_body_command ->
#                    the 035 a3_fast ONNX (low latency, ~220 ms).  Weights: downloaded 035.
#          g1      : same chain, the 035 g1 ONNX (slower joint-space encoder, ~940 ms).
#          smpl    : the Pico SMPL stream (+ A3 wrist angles) goes to the deploy's SMPL source and an SMPL-encoder
#                    ONNX (obs_dict[1,1770] -> action[1,29]) runs.  No such checkpoint is published for A3 / A3 Ultra:
#                    pass --smpl-model <your exported model_step_N_smpl.onnx>, or --smpl-model stub for a PLUMBING
#                    TEST policy that stands and ignores the operator (see a3_smpl_onnx_tool.py).
# --teleop-input ta|zmq : how the a3-fast / g1 reference reaches the deploy.  ta (default): the retarget node -> a3_ta_bridge -> ROS 2
#        /ta/whole_body_command.  zmq: the deploy reads the retarget node's `a3_ref` ZMQ stream itself (tcp://<--zmq-host>:5560): no
#        bridge, no ROS 2 hop, nothing published on the robot network (so no vendor subscriber of /ta/whole_body_command sees it).
# --remote : (real only) run the deploy on THIS machine against the robot over ROS 2, Litchi's "ADU-style cross-board transport"
#        (config/a3_aimrt_config.ros2.yaml: body-drive state / command topics over ROS 2).  It needs the robot's body-drive to be on
#        ROS 2: a read-only gate (a3_ros_graph_probe.py --sonic-gate) checks the 6 state + 4 command topics, their types and QoS and
#        REFUSES to start otherwise (on 2026-10-15 this robot did not expose them, plan 18.8).  ROS_DOMAIN_ID (not 0) and
#        ROS_LOCALHOST_ONLY=0 must be set; the RMW defaults to Cyclone DDS (--ros-iface-ip pins it to the robot NIC); every ROS 2
#        process runs under a memory cap (A3_MEMORY_MAX, default 16G; A3_MEMCAP=off disables it).
# --robot-link zmq : run the deploy on THIS machine and reach the robot through a3_zmq_relay (runs on the MDU next to body-drive, iceoryx
#        on its side, ZMQ on ours).  No ROS 2 / DDS on this machine, no vendor config change.  The relay owns the failsafe: if this machine or the
#        cable goes quiet it holds the last command ~0.2 s and then ramps to damping.  Needs --relay-host (the robot-network address the relay
#        is bound to, e.g. 10.42.10.12) and --teleop-input zmq (there is no AimRT /ta subscription in this mode).  Start the relay first
#        (docs/notes/a3/pico_real_teleop.md 4.6).  Default --robot-link aimrt = this process sits next to body-drive (the MDU package).
# --input-type zmq_manager : the A3 counterpart of G1's Pico flow: also start the operator chain next to the deploy
#        binary (sim / one machine).  --zmq-host = where pico_manager_thread_server.py runs.  --no-chain skips it
#        (real robot: run a3_operator_chain.sh on the operator PC); then --zmq-host = the PC the SMPL stream comes from.
# sim  : talk to the MuJoCo A3 simulator (run_sim_loop.py --robot a3|a3_ultra) over ROS2 /body_drive/*.
#        Forces A3_TRANSPORT=ros2, a private ROS domain (A3_SIM_ROS_DOMAIN_ID, default 87) and
#        ROS_LOCALHOST_ONLY=1, and puts the ROS2 conda env libs (A3_ROS_PREFIX) on LD_LIBRARY_PATH.
# real : talk to the robot; use your site's A3_TRANSPORT / ROS_DOMAIN_ID / A3_ROBOT_ENV (default).  The deploy starts IDLE and sends
#        nothing until the operator presses S (PD stand) / Pico A+B+X+Y; asks for a confirmation first (--yes / A3_ASSUME_YES=1 skips it).
#        The robot itself runs the aarch64 package on the MDU (see docs/notes/a3/pico_real_teleop.md); this launcher is for a machine
#        that has the repo and the matching package.  Run scripts/a3_real_preflight.sh first.
#
# Environment:
#   A3_TRANSPORT         iceoryx (default on x86_64) | ros2
#   A3_ROBOT_ENV         vendor env script to source (required on rockchip)
#   A3_SOURCE_ROBOT_ENV  0 to skip sourcing the vendor env
#   GS_A3_NO_BUILD=1     never build, fail if the package is missing
#   A3_SMPL_MODEL        default SMPL-encoder ONNX for --encoder smpl
#   A3_TOOL_PYTHON       python with PyYAML + onnx for the config / ONNX helpers (default: first that has them)
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GEAR_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd -- "${GEAR_ROOT}/.." && pwd)"

ROBOT="a3"
BUILD_ONLY=0
NO_BUILD="${GS_A3_NO_BUILD:-0}"
PASS=()
TARGET=""
INPUT_TYPE=""
ZMQ_HOST="localhost"
ENCODER="a3-fast"
TELEOP_INPUT="ta"
SMPL_MODEL="${A3_SMPL_MODEL:-}"
NO_CHAIN=0
REMOTE=0
ROS_IFACE_IP="${A3_ROS_IFACE_IP:-}"
ROBOT_LINK="aimrt"
RELAY_HOST=""
RELAY_STATE_PORT=""
RELAY_COMMAND_PORT=""
ASSUME_YES="${A3_ASSUME_YES:-0}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot)      ROBOT="$2"; shift 2 ;;
    --build-only) BUILD_ONLY=1; shift ;;
    --no-build)   NO_BUILD=1; shift ;;
    --)           shift; PASS+=("$@"); break ;;
    -h|--help)    sed -n 2,40p "${BASH_SOURCE[0]}"; exit 0 ;;
    # G1-style positional interface selectors are meaningless for A3 (AimRT transport).
    sim|real)     TARGET="$1"; shift ;;
    --input-type) INPUT_TYPE="$2"; shift 2 ;;
    --input-type=*) INPUT_TYPE="${1#--input-type=}"; shift ;;
    --output-type) shift 2 ;;   # G1 option; A3 always publishes the /body_drive/* commands
    --zmq-host)   ZMQ_HOST="$2"; shift 2 ;;
    --encoder)    ENCODER="$2"; shift 2 ;;
    --teleop-input) TELEOP_INPUT="$2"; shift 2 ;;
    --teleop-input=*) TELEOP_INPUT="${1#--teleop-input=}"; shift ;;
    --encoder=*)  ENCODER="${1#--encoder=}"; shift ;;
    --smpl-model) SMPL_MODEL="$2"; shift 2 ;;
    --smpl-model=*) SMPL_MODEL="${1#--smpl-model=}"; shift ;;
    --no-chain)   NO_CHAIN=1; shift ;;
    --yes)        ASSUME_YES=1; shift ;;
    --remote)     REMOTE=1; shift ;;
    --ros-iface-ip) ROS_IFACE_IP="$2"; shift 2 ;;
    --robot-link) ROBOT_LINK="$2"; shift 2 ;;
    --robot-link=*) ROBOT_LINK="${1#--robot-link=}"; shift ;;
    --relay-host) RELAY_HOST="$2"; shift 2 ;;
    --relay-host=*) RELAY_HOST="${1#--relay-host=}"; shift ;;
    --relay-state-port) RELAY_STATE_PORT="$2"; shift 2 ;;
    --relay-command-port) RELAY_COMMAND_PORT="$2"; shift 2 ;;
    *)            PASS+=("$1"); shift ;;
  esac
done

case "${ROBOT}" in
  a3|a3_ultra) ;;
  *) echo "deploy_a3.sh: unsupported --robot '${ROBOT}' (expected a3 or a3_ultra)" >&2; exit 2 ;;
esac
if [[ "${ROBOT}" == "a3_ultra" ]]; then
  echo "[a3] --robot a3_ultra: running the downloaded 035 weights (HF sonic-for-a3/sonic; the release states support for A3 and A3 Ultra)."
  echo "[a3]   policy output: the 29 body joints, exactly as for A3 (same Kp/Kd, action scale, default pose = Litchi a3_024);"
  echo "[a3]   the two A3 Ultra head joints are held at their default pose q=0 (Kp 40 / Kd 2), the fingers are not driven."
  echo "[a3]   'train_agent_trl.py --robot a3_ultra' trains with the very same low-level parameters (robots/a3_ultra.py)."
  echo "[a3] NOTE: a policy trained from scratch with --robot a3_ultra (63-DOF T3D0 contract) has no deploy export;"
  echo "[a3]       pass a custom config with: A3_RUNTIME_CFG=<path> deploy.sh ... (a second --runtime-cfg is ignored)"
fi

case "$(uname -m)" in
  x86_64)        ARCH="x86_64" ;;
  aarch64|arm64) ARCH="rockchip" ;;
  *) echo "unsupported machine: $(uname -m)" >&2; exit 2 ;;
esac
PKG_DIR="${GEAR_ROOT}/dist/a3_deploy_${ARCH}"

# --- 1. weights --------------------------------------------------------------
MODEL_DIR="${GEAR_ROOT}/assets/a3_runtime/models/035_step200000"
if ! ls "${MODEL_DIR}"/*.onnx >/dev/null 2>&1; then
  echo "[a3] 035 ONNX weights missing -> downloading (python download_from_hf.py --robot a3)"
  PY="${PYTHON:-python3}"
  "${PY}" "${REPO_ROOT}/download_from_hf.py" --robot a3 --component onnx rknn
fi

# --- 2. package --------------------------------------------------------------
if [[ ! -x "${PKG_DIR}/run_a3.sh" ]]; then
  if [[ "${NO_BUILD}" == "1" ]]; then
    echo "[a3] package ${PKG_DIR} missing and builds are disabled (GS_A3_NO_BUILD / --no-build)" >&2
    exit 66
  fi
  echo "[a3] building deploy package for ${ARCH} ..."
  "${SCRIPT_DIR}/build_a3_deploy_pkg.sh" --arch "${ARCH}"
fi
if [[ "${BUILD_ONLY}" == "1" ]]; then
  echo "[a3] package ready: ${PKG_DIR}"
  exit 0
fi

# --- 2b. encoder -------------------------------------------------------------
case "${ROBOT_LINK}" in aimrt|zmq) ;; *) echo "[a3] --robot-link must be aimrt or zmq (got '${ROBOT_LINK}')" >&2; exit 2 ;; esac
if [[ "${ROBOT_LINK}" == "zmq" ]]; then
  [[ -n "${RELAY_HOST}" ]] || { echo "[a3] --robot-link zmq needs --relay-host HOST (where a3_zmq_relay is bound, e.g. 10.42.10.12)" >&2; exit 2; }
  [[ "${REMOTE}" != "1" ]] || { echo "[a3] --robot-link zmq and --remote (ROS 2) are two different ways to reach the robot; pick one" >&2; exit 2; }
fi
if [[ "${REMOTE}" == "1" && "${TARGET}" != "real" ]]; then
  echo "[a3] --remote is for the real robot ('real'); the simulator is already a remote ROS 2 peer" >&2; exit 2
fi
case "${ENCODER}" in
  a3-fast|a3_fast|a3fast) ENCODER="a3-fast"; SOURCE="a3_fast" ;;
  g1)                     SOURCE="g1" ;;
  smpl)                   SOURCE="smpl" ;;
  *) echo "[a3] --encoder must be a3-fast, smpl or g1 (got '${ENCODER}')" >&2; exit 2 ;;
esac

# first python from the list that can import the given modules
pick_python() {
  local mods="$1"; shift
  local c
  for c in "${A3_TOOL_PYTHON:-}" "$@" python3; do
    [[ -n "${c}" ]] || continue
    if command -v "${c}" >/dev/null 2>&1 && "${c}" -c "import ${mods}" >/dev/null 2>&1; then echo "${c}"; return 0; fi
  done
  return 1
}
CONDA_ROOT="${CONDA_ROOT:-$HOME/miniconda3}"

if [[ "${SOURCE}" == "smpl" ]]; then
  STUB_DIR="${GEAR_ROOT}/assets/a3_runtime/models/a3_smpl_stub"
  if [[ -z "${SMPL_MODEL}" ]]; then
    SMPL_MODEL="${GEAR_ROOT}/assets/a3_runtime/models/a3_ultra_smpl/model_smpl.onnx"
  elif [[ "${SMPL_MODEL}" == "stub" ]]; then
    SMPL_MODEL="${STUB_DIR}/model_smpl_stub.onnx"
    ONNX_PY="$(pick_python onnx "${CONDA_ROOT}/envs/isaaclab/bin/python")" || {
      echo "[a3] need a python with the onnx package to write the stub (set A3_TOOL_PYTHON)" >&2; exit 66; }
    FAST_ONNX="$(ls "${PKG_DIR}"/models/*a3_fast.onnx 2>/dev/null | head -1 || true)"
    [[ -n "${FAST_ONNX}" ]] || { echo "[a3] ${PKG_DIR}/models has no *a3_fast.onnx to build the stub from" >&2; exit 66; }
    "${ONNX_PY}" "${SCRIPT_DIR}/a3_smpl_onnx_tool.py" stub --base-model "${FAST_ONNX}" --out "${SMPL_MODEL}"
    echo "[a3] ##################################################################################"
    echo "[a3] # --smpl-model stub: PLUMBING TEST ONLY. The stub ignores the SMPL tokenizer: the"
    echo "[a3] # robot STANDS (a3_fast policy on the default stand tokenizer) and does NOT follow"
    echo "[a3] # the operator. It exercises the SMPL data path, source locking and the safety logic."
    echo "[a3] ##################################################################################"
  fi
  if [[ ! -f "${SMPL_MODEL}" ]]; then
    echo "[a3] --encoder smpl needs an SMPL-encoder policy and '${SMPL_MODEL}' does not exist." >&2
    echo "[a3]   No SMPL-encoder checkpoint is published for A3 / A3 Ultra (the 035 release has only the g1 and a3_fast" >&2
    echo "[a3]   encoders). Train one and export it (docs/workflow/a3_integration_plan.md section 15):" >&2
    echo "[a3]     python gear_sonic/eval_agent_trl.py +checkpoint=<model.pt> +export_onnx_only=true   -> model_step_<N>_smpl.onnx" >&2
    echo "[a3]   then pass --smpl-model <that file>. To test the data path only: --smpl-model stub." >&2
    exit 66
  fi
  if [[ "${SMPL_MODEL}" == *.rknn ]]; then
    echo "[a3] ${SMPL_MODEL}: RKNN model, interface check skipped (the deploy checks the input size on load)"
  elif ONNX_PY="$(pick_python onnx "${CONDA_ROOT}/envs/isaaclab/bin/python")"; then
    "${ONNX_PY}" "${SCRIPT_DIR}/a3_smpl_onnx_tool.py" check "${SMPL_MODEL}" || exit 65
  else
    echo "[a3] no python with onnx found: SMPL interface check skipped (the deploy checks the input size on load)"
  fi
  CFG_PY="$(pick_python yaml "${CONDA_ROOT}/envs/gmr/bin/python" "${CONDA_ROOT}/envs/isaaclab/bin/python")" || {
    echo "[a3] need a python with PyYAML to generate the runtime config (set A3_TOOL_PYTHON)" >&2; exit 66; }
  if [[ "${NO_CHAIN}" == "1" ]]; then SMPL_HOST="${ZMQ_HOST}"; else SMPL_HOST="localhost"; fi
  GEN_CFG="${PKG_DIR}/config/a3_runtime_config.generated.yaml"
  "${CFG_PY}" "${SCRIPT_DIR}/a3_make_runtime_cfg.py" --base "${PKG_DIR}/config/a3_runtime_config.yaml" --out "${GEN_CFG}" \
      --encoder smpl --smpl-model "$(cd "$(dirname "${SMPL_MODEL}")" && pwd)/$(basename "${SMPL_MODEL}")" \
      --smpl-zmq-host "${SMPL_HOST}" --smpl-zmq-port 5561
  export A3_RUNTIME_CFG="${GEN_CFG}"
fi
case "${TELEOP_INPUT}" in ta|zmq) ;; *) echo "[a3] --teleop-input must be ta or zmq (got '${TELEOP_INPUT}')" >&2; exit 2 ;; esac
if [[ "${ROBOT_LINK}" == "zmq" && "${SOURCE}" != "smpl" && "${TELEOP_INPUT}" != "zmq" ]]; then
  echo "[a3] --robot-link zmq has no AimRT in the deploy process, so the /ta/whole_body_command subscription does not exist:" >&2
  echo "[a3]   add --teleop-input zmq (the deploy reads the retarget node's a3_ref stream itself)" >&2
  exit 2
fi
if [[ "${ROBOT_LINK}" == "zmq" ]]; then
  PASS+=(--robot-link zmq --relay-host "${RELAY_HOST}")
  [[ -z "${RELAY_STATE_PORT}" ]] || PASS+=(--relay-state-port "${RELAY_STATE_PORT}")
  [[ -z "${RELAY_COMMAND_PORT}" ]] || PASS+=(--relay-command-port "${RELAY_COMMAND_PORT}")
fi
PASS+=(--teleop-source "${SOURCE}")
if [[ "${TELEOP_INPUT}" == "zmq" && "${SOURCE}" != "smpl" ]]; then
  # the deploy itself subscribes to the retarget node's a3_ref stream (port 5560 on the operator PC)
  if [[ "${NO_CHAIN}" == "1" ]]; then A3_REF_HOST="${ZMQ_HOST}"; else A3_REF_HOST="localhost"; fi
  PASS+=(--teleop-input zmq --a3-ref-host "${A3_REF_HOST}" --a3-ref-port 5560)
fi
# The automatic PD stand at startup is for the simulator without the Pico flow (scripted runs).  With the Pico flow (zmq_manager) and on
# the real robot the deploy starts IDLE and sends nothing until the operator hardens the robot: A+B+X+Y (or S) - the G1 operator flow
# (vendor procedure: docs/a3_training2sim2deploy.md section 6.1).
if [[ "${TARGET}" == "sim" && "${INPUT_TYPE}" != "zmq_manager" ]]; then PASS+=(--auto-pd-stand); fi
echo "[a3] encoder=${ENCODER} (TELEOP source '${SOURCE}' preselected and locked in the deploy)"

# --- 3. run ------------------------------------------------------------------
if [[ "${TARGET}" == "sim" && "${ROBOT_LINK}" == "zmq" ]]; then
  # The simulator is reached through the relay (scripts/a3_relay_sim.sh runs it on the simulator's ROS 2 domain); this process
  # only needs the shared libraries of the package, no ROS domain.
  A3_ROS_PREFIX="${A3_ROS_PREFIX:-${CONDA_PREFIX:-$HOME/miniconda3}/envs/a3_deploy_verify}"
  [[ -d "${A3_ROS_PREFIX}/lib" ]] && export LD_LIBRARY_PATH="${A3_ROS_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
  echo "[a3] sim target through the ZMQ relay at ${RELAY_HOST}: start the simulator and scripts/a3_relay_sim.sh first"
elif [[ "${TARGET}" == "sim" ]]; then
  # The MuJoCo A3 simulator (a3_sim.py) publishes the robot-side topics on ROS2 only.
  export A3_TRANSPORT=ros2
  export ROS_DOMAIN_ID="${A3_SIM_ROS_DOMAIN_ID:-87}"
  export ROS_LOCALHOST_ONLY=1
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  A3_ROS_PREFIX="${A3_ROS_PREFIX:-${CONDA_PREFIX:-$HOME/miniconda3}/envs/a3_deploy_verify}"
  [[ -d "${A3_ROS_PREFIX}/lib" ]] || A3_ROS_PREFIX="${CONDA_PREFIX:-}"
  if [[ -n "${A3_ROS_PREFIX}" && -d "${A3_ROS_PREFIX}/lib" ]]; then
    export LD_LIBRARY_PATH="${A3_ROS_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
  fi
  echo "[a3] sim target: transport=ros2 ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=1 ros_prefix=${A3_ROS_PREFIX:-<none>}"
  echo "[a3] start the simulator first:  python gear_sonic/scripts/run_sim_loop.py --robot ${ROBOT}"
fi

RUN_PREFIX=()
if [[ "${REMOTE}" == "1" ]]; then
  # Litchi's ADU-style transport: this machine is "another board" of the robot's ROS 2 network.
  export A3_TRANSPORT=ros2
  if [[ -z "${ROS_DOMAIN_ID:-}" || "${ROS_DOMAIN_ID}" == "0" ]]; then
    echo "[a3] --remote: set ROS_DOMAIN_ID to the robot's domain (unset / 0 is shared with every ROS 2 participant)" >&2; exit 66
  fi
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
  if [[ "${ROS_LOCALHOST_ONLY}" == "1" ]]; then
    echo "[a3] --remote: ROS_LOCALHOST_ONLY=1 keeps ROS 2 on this PC; export ROS_LOCALHOST_ONLY=0" >&2; exit 66
  fi
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  if [[ "${RMW_IMPLEMENTATION}" == "rmw_cyclonedds_cpp" && -z "${CYCLONEDDS_URI:-}" && -n "${ROS_IFACE_IP}" ]]; then
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface address=\"${ROS_IFACE_IP}\" multicast=\"true\"/></Interfaces></General></Domain></CycloneDDS>"
  fi
  if [[ "${RMW_IMPLEMENTATION}" != "rmw_cyclonedds_cpp" ]]; then
    echo "[a3] WARNING: RMW is ${RMW_IMPLEMENTATION}; the conda Humble Fast DDS 2.6.4 can allocate tens of GB next to foreign participants (plan 9.7 / 18.7)" >&2
  elif [[ -z "${CYCLONEDDS_URI:-}" ]]; then
    echo "[a3] WARNING: Cyclone DDS is not pinned to a NIC (--ros-iface-ip / CYCLONEDDS_URI): it announces on every interface" >&2
  fi
  A3_ROS_PREFIX="${A3_ROS_PREFIX:-${CONDA_PREFIX:-$HOME/miniconda3}/envs/a3_deploy_verify}"
  if [[ -d "${A3_ROS_PREFIX}/lib" ]]; then export LD_LIBRARY_PATH="${A3_ROS_PREFIX}/lib:${LD_LIBRARY_PATH:-}"; fi
  if [[ "${A3_MEMCAP:-on}" != "off" ]] && command -v systemd-run >/dev/null 2>&1; then
    RUN_PREFIX=(systemd-run --user --scope -q -p "MemoryMax=${A3_MEMORY_MAX:-16G}" -p MemorySwapMax=0)
  else
    echo "[a3] WARNING: no memory cap for the ROS 2 processes (systemd-run missing or A3_MEMCAP=off)" >&2
  fi
  echo "[a3] remote deploy: transport=ros2 ROS_DOMAIN_ID=${ROS_DOMAIN_ID} RMW=${RMW_IMPLEMENTATION} memory cap=${A3_MEMORY_MAX:-16G}"
fi

if [[ "${TARGET}" == "real" ]]; then
  echo "[a3] ============================== REAL ROBOT =============================="
  echo "[a3]  transport=${A3_TRANSPORT:-iceoryx}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}  ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset}  robot=${ROBOT}"
  echo "[a3]  Before you go on: gantry / sling attached, physical emergency stop in reach, a supervisor next to the robot,"
  echo "[a3]  the stock motion_control NOT running, MuJoCo validation done, scripts/a3_real_preflight.sh clean."
  if [[ "${ROBOT_LINK}" == "zmq" ]]; then
    echo "[a3]  ZMQ RELAY ${RELAY_HOST}: this machine runs the deploy; a3_zmq_relay on the MDU must already be running (start it first, without --listen-only)."
    echo "[a3]  If this machine or the cable goes quiet the relay holds the last command ~0.2 s, then ramps to damping (latched); restart the deploy to take over again."
    echo "[a3]  The deploy is not real-time: Wi-Fi is not acceptable for this link, use the wired robot network."
  elif [[ "${REMOTE}" == "1" ]]; then
    echo "[a3]  REMOTE: this machine runs the deploy and commands the robot over ROS 2; if this machine or the cable fails the commands stop."
  elif [[ "${ARCH}" == "x86_64" ]]; then
    echo "[a3]  NOTE: this is an x86_64 package. The robot's own computer runs the aarch64 (Rockchip / Thor) package."
  fi
  if [[ "${ASSUME_YES}" != "1" ]]; then
    if [[ ! -t 0 ]]; then echo "[a3] not a terminal: pass --yes (or A3_ASSUME_YES=1) to confirm a real-robot start" >&2; exit 2; fi
    read -r -p "[a3] Proceed with the REAL-ROBOT deployment? [y/N] " _ans
    case "${_ans}" in y|Y|yes|YES) ;; *) echo "[a3] aborted"; exit 1 ;; esac
  fi
fi

if [[ "${REMOTE}" == "1" ]]; then
  # Gate (read-only): does the robot's ROS 2 graph offer what the ros2 deploy config needs?  Refuse to start if not.
  echo "[a3] gate: checking the robot's ROS 2 graph for the 6 body-drive state and 4 command topics (read-only, ~20 s) ..."
  PROBE_RC=0
  if [[ -n "${A3_GRAPH_PROBE_CMD:-}" ]]; then
    PROBE_OUT="$(bash -c "${A3_GRAPH_PROBE_CMD}" 2>/dev/null)" || PROBE_RC=$?
  else
    PROBE_OUT="$("${RUN_PREFIX[@]+"${RUN_PREFIX[@]}"}" timeout 90 "${A3_ROS_PREFIX}/bin/python" "${SCRIPT_DIR}/a3_ros_graph_probe.py" 15 --sonic-gate 2>/dev/null)" || PROBE_RC=$?
  fi
  printf '%s\n' "${PROBE_OUT}" | tail -14
  if [[ "${PROBE_RC}" != "0" ]]; then
    echo "[a3] gate FAILED (exit ${PROBE_RC}): the robot does not offer the body-drive topics SONIC needs on ROS 2, so the deploy is NOT started." >&2
    echo "[a3]   Nothing was published.  Options: have body-drive's ROS 2 backend enabled on the MDU, or deploy the aarch64 package on the MDU" >&2
    echo "[a3]   (docs/notes/a3/pico_real_teleop.md, plan 18.8)." >&2
    exit 66
  fi
fi

CHILDREN=()
cleanup() { for p in "${CHILDREN[@]+"${CHILDREN[@]}"}"; do kill "$p" 2>/dev/null || true; done; }
if [[ -n "${INPUT_TYPE}" && "${INPUT_TYPE}" != "zmq_manager" ]]; then
  echo "[a3] --input-type '${INPUT_TYPE}' is not supported for A3 (only zmq_manager = Pico teleop)" >&2
  exit 2
fi
if [[ "${INPUT_TYPE}" == "zmq_manager" ]]; then
  if [[ "${NO_CHAIN}" == "1" ]]; then
    echo "[a3] --no-chain: not starting the operator chain here; run scripts/a3_operator_chain.sh --encoder ${ENCODER} on the operator PC"
  else
    trap cleanup EXIT INT TERM
    echo "[a3] Pico flow: operator chain for --encoder ${ENCODER}; Pico stream from tcp://${ZMQ_HOST}:5556 topic 'pose'"
    CHAIN_ARGS=(--encoder "${ENCODER}" --pico-host "${ZMQ_HOST}")
    if [[ "${TELEOP_INPUT}" == "zmq" ]]; then CHAIN_ARGS+=(--no-bridge); fi
    if [[ "${TARGET}" == "real" ]]; then CHAIN_ARGS+=(--real); fi
    "${SCRIPT_DIR}/a3_operator_chain.sh" "${CHAIN_ARGS[@]}" </dev/null &
    CHILDREN+=("$!")
    sleep 2
    if ! kill -0 "${CHILDREN[0]}" 2>/dev/null; then
      echo "[a3] the operator chain exited right after starting (its message is above): fix that, or run" >&2
      echo "[a3] scripts/a3_operator_chain.sh --encoder ${ENCODER} on another machine and pass --no-chain here" >&2
      exit 66
    fi
  fi
  # Same operator sequence as G1 MuJoCo teleop: wait Init Done, A+B+X+Y, key 9, A+X.
  # --teleop-source makes a3_deploy_onnx_ref auto-enter PD stand and listen for Pico ZMQ command.
  PASS+=(--zmq-host "${ZMQ_HOST}" --zmq-port 5556)
  echo "[a3] Pico flow (same as G1), the deploy starts IDLE and sends nothing:"
  echo "[a3]   A+B+X+Y  robot hardens and holds the default standing pose (PD stand, Litchi gains; 'Init Done' when the 3 s ramp is finished)"
  if [[ "${TARGET}" == "sim" ]]; then
    echo "[a3]   MuJoCo window key 9 releases the elastic band"
  else
    echo "[a3]   (real robot: robot supported, then lower the sling)"
  fi
  echo "[a3]   A+X      follow the operator;   A+X again: policy stand (a3_fast + default stand, like G1 PLANNER);   A+B+X+Y again: disable (kp = kd = 0, limp)"
  echo "[a3]   Pico commands come from tcp://${ZMQ_HOST}:5556; keyboard S (stand) / T (follow) / P (disable) work as backup"
  echo "[a3]   link loss: robot state stale, relay failsafe or a silent teleop stream drop the robot to passive / the standing pose (like G1)"
  "${RUN_PREFIX[@]+"${RUN_PREFIX[@]}"}" "${PKG_DIR}/run_a3.sh" "${PASS[@]+"${PASS[@]}"}"
  exit $?
fi
exec "${RUN_PREFIX[@]+"${RUN_PREFIX[@]}"}" "${PKG_DIR}/run_a3.sh" "${PASS[@]+"${PASS[@]}"}"

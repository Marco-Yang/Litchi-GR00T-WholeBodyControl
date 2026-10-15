#!/usr/bin/env bash
# ============================================================================
# One-time setup of the ROS2 Python env used by the A3 / A3 Ultra MuJoCo simulator
# (gear_sonic/scripts/run_sim_loop.py --robot a3|a3_ultra).
#
# The A3 deploy binary talks to the robot over ROS2 /body_drive/* topics with AgiBot's
# `joint_msgs`; the simulator has to speak the same messages, so it needs rclpy + the
# joint_msgs Python bindings + mujoco in one interpreter.
#
#   scripts/setup_a3_sim_env.sh [--env a3_deploy_verify] [--build-dir <cmake build dir>]
#
# Prerequisites:
#   * conda env with ROS2 Humble (rclpy, sensor_msgs) -- the same env used to build the deploy
#     package (see docs/workflow/a3_integration_plan.md, Phase 3). Default name: a3_deploy_verify.
#   * a CMake build dir of the deploy package that contains joint_msgs_build/ (e.g. build_verify_full).
# What it does (only touches the given conda env):
#   1. pip install mujoco scipy tyro
#   2. builds the joint_msgs Python typesupport targets (if not built yet)
#   3. copies the joint_msgs Python package + libjoint_msgs__*.so into the env and sets their RUNPATH
# ============================================================================
set -euo pipefail
ENV_NAME="a3_deploy_verify"
BUILD_DIR=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --env) ENV_NAME="$2"; shift 2 ;;
    --build-dir) BUILD_DIR="$2"; shift 2 ;;
    -h|--help) sed -n 2,22p "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown arg $1" >&2; exit 2 ;;
  esac
done
GEAR_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -z "${BUILD_DIR}" ]]; then
  for d in build_verify_full build/a3_pkg_x86_64 build; do
    if [[ -d "${GEAR_ROOT}/${d}/src/g1/g1_deploy_onnx_ref/joint_msgs_build" ]]; then BUILD_DIR="${GEAR_ROOT}/${d}"; break; fi
  done
fi
[[ -n "${BUILD_DIR}" ]] || { echo "no build dir with joint_msgs_build found; pass --build-dir" >&2; exit 66; }
JM="${BUILD_DIR}/src/g1/g1_deploy_onnx_ref/joint_msgs_build"

# conda's ROS activation hooks reference unset variables, so relax -u around them.
set +u
# shellcheck disable=SC1091
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate "${ENV_NAME}"
set -u
python -c "import rclpy, sensor_msgs.msg" || { echo "env ${ENV_NAME} has no ROS2 python (rclpy)" >&2; exit 66; }

pip install mujoco scipy tyro

if ! ls "${JM}"/rosidl_generator_py/joint_msgs/joint_msgs_s__rosidl_typesupport_c*.so >/dev/null 2>&1; then
  echo "[a3-sim-env] building joint_msgs python typesupport in ${BUILD_DIR}"
  CPATH="${CONDA_PREFIX}/include" LIBRARY_PATH="${CONDA_PREFIX}/lib" cmake --build "${BUILD_DIR}" -j"$(nproc)" \
    --target joint_msgs__rosidl_generator_py joint_msgs__rosidl_typesupport_c__pyext \
             joint_msgs__rosidl_typesupport_fastrtps_c__pyext joint_msgs__rosidl_typesupport_introspection_c__pyext
fi
SP="$(python -c 'import site; print(site.getsitepackages()[0])')"
rm -rf "${SP}/joint_msgs"
cp -r "${JM}/rosidl_generator_py/joint_msgs" "${SP}/joint_msgs"
cp "${JM}"/libjoint_msgs__*.so "${CONDA_PREFIX}/lib/"
# The extensions are linked against libs in the package dir and the env lib dir; give them a RUNPATH so
# they import without an activated env / LD_LIBRARY_PATH (run_sim_loop re-execs into this env bare).
command -v patchelf >/dev/null || pip install patchelf
for f in "${SP}"/joint_msgs/*.so; do patchelf --set-rpath '$ORIGIN:$ORIGIN/../../..' "$f"; done
python -c "import joint_msgs.msg, rclpy, mujoco; print('[a3-sim-env] OK: rclpy + joint_msgs + mujoco', mujoco.__version__)"

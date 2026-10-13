#!/usr/bin/env bash
# ============================================================================
# A3 / A3 Ultra deploy launcher  (called by deploy.sh --robot a3|a3_ultra)
#
#   scripts/deploy_a3.sh [--robot a3|a3_ultra] [--build-only] [--no-build] [-- <a3_deploy_onnx_ref args>]
#
# 1. Makes sure the 035 ONNX/RKNN weights are present (download_from_hf.py --robot a3).
# 2. Makes sure the deploy package dist/a3_deploy_<arch>/ exists (build_a3_deploy_pkg.sh).
# 3. exec's the package launcher run_a3.sh  (AimRT: A3_TRANSPORT=iceoryx|ros2).
#
# Environment:
#   A3_TRANSPORT         iceoryx (default on x86_64) | ros2
#   A3_ROBOT_ENV         vendor env script to source (required on rockchip)
#   A3_SOURCE_ROBOT_ENV  0 to skip sourcing the vendor env
#   GS_A3_NO_BUILD=1     never build, fail if the package is missing
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GEAR_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd -- "${GEAR_ROOT}/.." && pwd)"

ROBOT="a3"
BUILD_ONLY=0
NO_BUILD="${GS_A3_NO_BUILD:-0}"
PASS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot)      ROBOT="$2"; shift 2 ;;
    --build-only) BUILD_ONLY=1; shift ;;
    --no-build)   NO_BUILD=1; shift ;;
    --)           shift; PASS+=("$@"); break ;;
    -h|--help)    sed -n 2,20p "${BASH_SOURCE[0]}"; exit 0 ;;
    # G1-style positional interface selectors are meaningless for A3 (AimRT transport).
    sim|real)     echo "[a3] note: '$1' ignored (A3 uses A3_TRANSPORT=iceoryx|ros2)"; shift ;;
    *)            PASS+=("$1"); shift ;;
  esac
done

case "${ROBOT}" in
  a3|a3_ultra) ;;
  *) echo "deploy_a3.sh: unsupported --robot '${ROBOT}' (expected a3 or a3_ultra)" >&2; exit 2 ;;
esac
if [[ "${ROBOT}" == "a3_ultra" ]]; then
  echo "[a3] --robot a3_ultra: A3 Ultra shares the A3 hardware platform; deploying the 035 (A3) contract."
  echo "[a3] NOTE: the from-scratch 63-DOF A3 Ultra (T3D0) training contract has no deploy export yet;"
  echo "[a3]       pass a custom config with: -- --runtime-cfg=<path>"
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

# --- 3. run ------------------------------------------------------------------
exec "${PKG_DIR}/run_a3.sh" "${PASS[@]+"${PASS[@]}"}"

#!/usr/bin/env bash
# Source this (do not bash-execute) to export VLA_CKPT / LITCHI_ROOT / RTC_* for
# origin vs hand-split-body real-robot deploy terminals.
#
#   source gear_sonic/scripts/source_deploy_branch.sh hand-split-body
#   source gear_sonic/scripts/source_deploy_branch.sh origin
#
# Optional overrides before sourcing:
#   VLA_CKPT=/path/to/checkpoint-49400
#   LITCHI_ROOT=/path/to/Litchi-Isaac-GR00T

_BRANCH="${1:-}"
if [[ -z "${_BRANCH}" ]]; then
  echo "Usage: source gear_sonic/scripts/source_deploy_branch.sh <origin|hand-split-body>" >&2
  return 2 2>/dev/null || exit 2
fi

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_REPO_ROOT="$(cd "${_SCRIPT_DIR}/../.." && pwd)"
_PY="${_REPO_ROOT}/.venv_inference/bin/python"
if [[ ! -x "${_PY}" ]]; then
  _PY="$(command -v python3)"
fi

_EXPORTS="$(
  REPO_ROOT="${_REPO_ROOT}" \
  VLA_CKPT="${VLA_CKPT:-}" \
  LITCHI_ROOT="${LITCHI_ROOT:-}" \
  "${_PY}" - "${_BRANCH}" <<'PY'
import os
import sys
from pathlib import Path

repo = Path(os.environ["REPO_ROOT"]).resolve()
sys.path.insert(0, str(repo))
from gear_sonic.utils.inference.deploy_branch import (
    format_branch_banner,
    resolve_deploy_branch,
    shell_exports,
)

branch = sys.argv[1]
ckpt = os.environ.get("VLA_CKPT") or None
litchi = os.environ.get("LITCHI_ROOT") or None
profile = resolve_deploy_branch(
    branch,
    vla_ckpt=ckpt,
    litchi_root=litchi,
    require_ckpt=False,
)
print(shell_exports(profile))
print(format_branch_banner(profile), file=sys.stderr)
if profile.split_hand_body:
    print(
        "[deploy-branch] PolicyServer MUST use Litchi-Isaac-GR00T "
        "(split_hand_body_action_head).",
        file=sys.stderr,
    )
PY
)" || {
  echo "[deploy-branch] failed to resolve branch=${_BRANCH}" >&2
  return 1 2>/dev/null || exit 1
}

eval "${_EXPORTS}"
export REPO="${REPO:-${_REPO_ROOT}}"
export DEPLOY="${DEPLOY:-${REPO}/gear_sonic_deploy}"
echo "[deploy-branch] exported DEPLOY_BRANCH=${DEPLOY_BRANCH}"
echo "[deploy-branch] VLA_CKPT=${VLA_CKPT}"
echo "[deploy-branch] LITCHI_ROOT=${LITCHI_ROOT}"
echo "[deploy-branch] RTC overlap=${RTC_OVERLAP_STEPS} frozen=${RTC_FROZEN_STEPS} ramp=${RTC_RAMP_RATE} exec_h=${RTC_EXECUTION_HORIZON} denoise=${NUM_INFERENCE_TIMESTEPS} client_rtc=${RTC_CLIENT_ENABLED} rate=${INFERENCE_RATE}"

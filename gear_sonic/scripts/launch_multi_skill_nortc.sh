#!/usr/bin/env bash
# Boot shared-VLM multi-skill PolicyServer (skill1/2/3 nortc).
# One Cosmos backbone + all action_heads resident; keyboard switch is a head swap.
set -euo pipefail

REPO="${REPO:-/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl}"
LITCHI_ROOT="${LITCHI_ROOT:-$REPO/.deps/Isaac-GR00T}"
SKILLS_CONFIG="${SKILLS_CONFIG:-$REPO/gear_sonic/config/multi_skill_nortc_skills.json}"
HF_HOME="${HF_HOME:-${HOME}/.cache/huggingface}"
GROOT_BACKBONE_PATH="${GROOT_BACKBONE_PATH:-$HF_HOME/hub/models--nvidia--Cosmos-Reason2-2B/snapshots/9ce19a195e423419c349abfc86fd07178b230561}"

test -f "$LITCHI_ROOT/gr00t/eval/run_gr00t_server.py" || {
  echo "MISSING Isaac-GR00T at $LITCHI_ROOT"
  exit 1
}
test -d "$GROOT_BACKBONE_PATH" || {
  echo "MISSING Cosmos local snapshot: $GROOT_BACKBONE_PATH"
  exit 1
}
test -f "$SKILLS_CONFIG" || {
  echo "MISSING skills config: $SKILLS_CONFIG"
  exit 1
}

export REPO LITCHI_ROOT HF_HOME GROOT_BACKBONE_PATH
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 GROOT_HF_LOCAL_FIRST=1
export NO_ALBUMENTATIONS_UPDATE=1
unset PYTHONHOME PYTHONSTARTUP || true
export PYTHONPATH="$LITCHI_ROOT:$REPO"

cd "$REPO"
# shellcheck disable=SC1091
source "$REPO/.venv_inference/bin/activate"

PY="${REPO}/.venv_inference/bin/python"
"$PY" -c "import numpy, torch; print('launcher preflight OK', numpy.__version__)"

exec "$PY" gear_sonic/scripts/run_shared_vlm_skill_server.py \
  --skills-config "$SKILLS_CONFIG" \
  --litchi-root "$LITCHI_ROOT" \
  --device cuda:0 \
  "$@"

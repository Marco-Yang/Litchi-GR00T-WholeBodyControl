#!/usr/bin/env bash
# install_inference.sh
# Sets up the .venv_inference venv for running VLA inference with
# Isaac-GR00T PolicyClient against a remote or local policy server.
#
# Default (deploy-host client): installs gear_sonic + PolicyClient deps without
# pulling the multi-GiB pip tensorrt-cu12-libs wheel. Heavy CUDA/TRT/torch stacks
# are reused from an existing conda env when available (see CONDA_GR00T_PREFIX).
#
# Full offline-capable server install: set FULL_GR00T_DEPS=1
#
# Usage:  bash install_scripts/install_inference.sh   (run from repo root)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Optional overrides
CONDA_GR00T_PREFIX="${CONDA_GR00T_PREFIX:-$HOME/miniconda3/envs/gr00t}"
GR00T_SRC="${GR00T_SRC:-}"
FULL_GR00T_DEPS="${FULL_GR00T_DEPS:-0}"
KEEP_VENV="${KEEP_VENV:-0}"

# ── 0. System dependencies ────────────────────────────────────────────────────
ARCH="$(uname -m)"
echo "[OK] Architecture: $ARCH"

# ── 1. Ensure uv is installed and available ──────────────────────────────────
if ! command -v uv &>/dev/null; then
    echo "[INFO] uv not found – installing via official installer …"
    curl -LsSf https://astral.sh/uv/install.sh | sh

    if [ -f "$HOME/.local/bin/env" ]; then
        # shellcheck disable=SC1091
        source "$HOME/.local/bin/env"
    elif [ -f "$HOME/.cargo/env" ]; then
        # shellcheck disable=SC1091
        source "$HOME/.cargo/env"
    else
        export PATH="$HOME/.local/bin:$PATH"
    fi

    if ! command -v uv &>/dev/null; then
        echo "[ERROR] uv installation succeeded but binary not found on PATH."
        echo "        Please add ~/.local/bin (or ~/.cargo/bin) to your PATH and re-run."
        exit 1
    fi
fi
echo "[OK] uv $(uv --version)"

# ── 2. Install a uv-managed Python 3.12 (Isaac-GR00T / gr00t requires >=3.12,<3.13) ─
echo "[INFO] Installing uv-managed Python 3.12 (includes development headers) …"
uv python install 3.12
MANAGED_PY="$(uv python find --no-project 3.12)"
echo "[OK] Using Python: $MANAGED_PY"

# ── 3. Resolve Isaac-GR00T source ────────────────────────────────────────────
resolve_gr00t_src() {
    if [ -n "$GR00T_SRC" ] && [ -f "$GR00T_SRC/pyproject.toml" ]; then
        echo "$GR00T_SRC"
        return 0
    fi
    # uv git checkout cache (if a prior resolve already cloned Isaac-GR00T)
    local cached
    cached="$(find "${UV_CACHE_DIR:-$HOME/.cache/uv}" -type f -path '*/Isaac-GR00T/*/pyproject.toml' 2>/dev/null | head -1 || true)"
    if [ -z "$cached" ]; then
        cached="$(find /tmp/cursor-sandbox-cache -type f -path '*/checkouts/*/pyproject.toml' 2>/dev/null \
            | while read -r f; do grep -q 'name = "gr00t"' "$f" 2>/dev/null && echo "$f"; done | head -1 || true)"
    fi
    if [ -n "$cached" ]; then
        dirname "$cached"
        return 0
    fi
    return 1
}

# ── 4. Create / reuse venv ───────────────────────────────────────────────────
cd "$REPO_ROOT"
if [ "$KEEP_VENV" != "1" ]; then
    echo "[INFO] Removing old .venv_inference (if present) …"
    rm -rf .venv_inference
fi
if [ ! -x .venv_inference/bin/python ]; then
    echo "[INFO] Creating .venv_inference with uv-managed Python 3.12 …"
    uv venv .venv_inference --python "$MANAGED_PY" --prompt gear_sonic_inference
fi
# shellcheck disable=SC1091
source .venv_inference/bin/activate
PY="$REPO_ROOT/.venv_inference/bin/python"
export GIT_SUBMODULE_STRATEGY="${GIT_SUBMODULE_STRATEGY:-none}"

link_conda_stack() {
    local conda_sp="$CONDA_GR00T_PREFIX/lib/python3.12/site-packages"
    local vsp="$REPO_ROOT/.venv_inference/lib/python3.12/site-packages"
    if [ ! -d "$conda_sp" ]; then
        echo "[WARN] CONDA_GR00T_PREFIX site-packages not found: $conda_sp"
        return 1
    fi
    echo "[INFO] Linking heavy packages from $CONDA_GR00T_PREFIX …"
    python3 - <<PY
from pathlib import Path
conda = Path("$conda_sp")
vsp = Path("$vsp")
mods = [
    "torch", "torchgen", "functorch", "torchvision", "torchvision.libs", "torchcodec",
    "triton", "nvidia", "sympy", "mpmath", "networkx", "fsspec", "filelock",
    "jinja2", "markupsafe",
    "transformers", "huggingface_hub", "safetensors", "tokenizers", "regex", "tqdm",
    "accelerate", "diffusers", "einops", "peft", "omegaconf", "antlr4",
    "requests", "charset_normalizer", "idna", "urllib3", "certifi", "hf_xet",
    "click", "annotated_types", "pydantic", "pydantic_core",
    "tensorrt", "tensorrt_bindings", "tensorrt_libs",
    "numpy", "numpy.libs", "scipy", "scipy.libs", "cv2", "yaml", "PIL",
    "msgpack", "msgpack.libs", "tyro", "docstring_parser", "typeguard", "shtab",
    "rich", "markdown_it", "mdurl", "pygments", "eval_type_backport",
    "zmq", "pyzmq.libs",
]
linked = 0
for name in mods:
    src = conda / name
    dest = vsp / name
    if src.exists() and not dest.exists():
        dest.symlink_to(src)
        linked += 1
# dist-info + single-file modules
for p in conda.iterdir():
    n = p.name
    low = n.lower()
    keep = False
    if n.endswith(".dist-info") or n.endswith(".libs"):
        base = n.split("-")[0].lower().replace("_", "-")
        keys = [m.lower().replace("_", "-") for m in mods] + [
            "pyzmq", "msgpack-numpy", "opencv-python-headless", "pyyaml",
            "tensorrt-cu12", "tensorrt-cu12-libs", "tensorrt-cu12-bindings",
            "torch", "torchvision", "huggingface-hub", "pillow",
        ]
        keep = any(base == k or base.startswith(k) for k in keys)
    if n in ("msgpack_numpy.py", "typing_extensions.py"):
        keep = True
    if keep and not (vsp / n).exists():
        (vsp / n).symlink_to(p)
        linked += 1
print(f"[OK] linked {linked} entries from conda gr00t")
PY
}

install_client_lite() {
    echo "[INFO] Client-lite install (PolicyClient path; skip pip tensorrt mega-wheels) …"
    local gr00t_src
    if ! gr00t_src="$(resolve_gr00t_src)"; then
        echo "[INFO] Cloning Isaac-GR00T (shallow, no submodules) …"
        gr00t_src="$REPO_ROOT/external_dependencies/Isaac-GR00T"
        mkdir -p "$(dirname "$gr00t_src")"
        if [ ! -f "$gr00t_src/pyproject.toml" ]; then
            git clone --depth 1 --recurse-submodules=no \
                https://github.com/NVIDIA/Isaac-GR00T.git "$gr00t_src"
        fi
    fi
    echo "[OK] GR00T_SRC=$gr00t_src"

    # Light wheels first (may use index; Mihomo fake-ip can stall — prefer offline wheels if present)
    if [ -d /tmp/g1_wheels ] && ls /tmp/g1_wheels/*.whl >/dev/null 2>&1; then
        uv pip install --python "$PY" --no-index --find-links /tmp/g1_wheels \
            pyzmq msgpack msgpack-numpy numpy tyro opencv-python scipy 2>/dev/null \
            || true
        uv pip install --python "$PY" --no-index --find-links /tmp/g1_wheels --no-deps \
            cmeel cmeel-boost cmeel-zlib cmeel-assimp cmeel-octomap cmeel-qhull \
            cmeel-console-bridge cmeel-urdfdom cmeel-tinyxml2 cmeel-urdfdom-headers \
            coal libcoal eigenpy hpp-fcl libpinocchio pin 2>/dev/null \
            || true
    fi
    uv pip install --python "$PY" pip setuptools wheel >/dev/null
    link_conda_stack || true

    # Ensure core client packages even if conda link skipped some
    "$PY" -m pip install --quiet pyzmq msgpack msgpack-numpy 'numpy==1.26.4' tyro 2>/dev/null || true

    "$PY" -m pip install --no-deps -e "$gr00t_src"
    "$PY" -m pip install --no-deps -e "$REPO_ROOT/gear_sonic"
}

if [ "$FULL_GR00T_DEPS" = "1" ]; then
    echo "[INFO] FULL_GR00T_DEPS=1 → uv pip install -e gear_sonic[inference]"
    echo "[WARN] This may download tensorrt-cu12-libs (~4GiB). Prefer Mihomo DIRECT / real DNS."
    uv pip install -e "gear_sonic[inference]"
else
    install_client_lite
fi

echo "[INFO] Verifying preflight import …"
"$PY" -c "import zmq; print('inference OK', zmq.__version__)"
"$PY" -c "from gr00t.policy.server_client import PolicyClient; print('PolicyClient OK')" 2>/dev/null \
    || echo "[WARN] PolicyClient import needs torch/transformers (link conda or FULL_GR00T_DEPS=1)"

echo ""
echo "══════════════════════════════════════════════════════════════"
echo "  Setup complete!  Activate the venv with:"
echo ""
echo "    source .venv_inference/bin/activate"
echo ""
echo "  You should see (gear_sonic_inference) in your prompt."
echo ""
echo "  Then run VLA inference with:"
echo "    python gear_sonic/scripts/run_vla_inference.py --help"
echo "══════════════════════════════════════════════════════════════"

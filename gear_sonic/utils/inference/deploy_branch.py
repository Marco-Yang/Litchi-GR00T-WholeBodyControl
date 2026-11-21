"""Deploy-branch profiles for origin vs hand-split-body VLA real-robot runs.

C++ SONIC + ZMQ action protocol are identical for both branches. The difference
is (1) checkpoint weights / action-head topology and (2) train-aligned RTC
defaults. PolicyServer must load via Litchi-Isaac-GR00T (supports
``split_hand_body_action_head``); stock Isaac without split cannot load the
hand-split-body checkpoint.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping


_REPO_ROOT = Path(__file__).resolve().parents[3]
_WORKSPACE = Path("/home/zhenyu/workspace")
_DEFAULT_LITCHI = Path(
    "/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/Litchi-Isaac-GR00T"
)


@dataclass(frozen=True)
class DeployBranchProfile:
    """Resolved deploy-branch settings."""

    name: str
    split_hand_body: bool
    action_horizon: int
    rtc_overlap_steps: int
    rtc_frozen_steps: int
    rtc_ramp_rate: float
    vla_ckpt: Path
    litchi_root: Path
    description: str
    rtc_execution_horizon: int = 15
    num_inference_timesteps: int = 4
    rtc_client_enabled: bool = False

    @property
    def inference_rate(self) -> float:
        """Client re-query rate (Hz) when RTC overlap is 0: publish_hz / exec_h."""
        exec_h = max(1, int(self.rtc_execution_horizon))
        return 50.0 / float(exec_h)


# Deploy RTC for the active origin ckpt (skill25). Client uses NO overlap
# inpainting (o=0/f=0); re-query every rtc_execution_horizon steps via --rate.
_BRANCH_SPECS: Mapping[str, dict] = {
    "origin": {
        "aliases": (
            "origin",
            "joint",
            "skill25",
            "skill2.5",
            "promptfix",
            "b64",
            "b64-e4",
        ),
        "split_hand_body": False,
        "action_horizon": 40,
        # skill2_pick_toy_new_rtc_s15_d8_o6_f6_true_e4/checkpoint-10952
        # train-aligned: o=6 f=6 ramp=3.0 exec_h=15 max_delay=8 denoise=4.
        "rtc_overlap_steps": 6,
        "rtc_frozen_steps": 6,
        "rtc_ramp_rate": 3.0,
        "rtc_execution_horizon": 15,
        "num_inference_timesteps": 4,
        "rtc_client_enabled": True,
        "description": "joint action head (skill2_pick_toy_new_rtc_s15_d8_o6_f6_true_e4)",
        "ckpt_candidates": (
            Path(
                "/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Gr00T"
                "/checkpoints/skill2_pick_toy_new_rtc_s15_d8_o6_f6_true_e4"
                "/checkpoint-10952"
            ),
            _REPO_ROOT.parent
            / "Gr00T/checkpoints/skill2_pick_toy_new_rtc_s15_d8_o6_f6_true_e4"
            / "checkpoint-10952",
        ),
    },
    "hand-split-body": {
        "aliases": (
            "hand-split-body",
            "hand_body_split",
            "hand-body-split",
            "split",
            "hand_body",
        ),
        "split_hand_body": True,
        "action_horizon": 40,
        "rtc_overlap_steps": 25,
        "rtc_frozen_steps": 6,
        "rtc_ramp_rate": 3.0,
        "rtc_execution_horizon": 15,
        "num_inference_timesteps": 4,
        "rtc_client_enabled": True,
        "description": "Phi0-style split hand/body action head",
        "ckpt_candidates": (
            _WORKSPACE
            / "data/demo/outputs/skill2_pick_toy_hand_body_split"
            / "skill2_pick_toy_hand_body_split/checkpoint-49400",
            _REPO_ROOT.parent
            / "Gr00T/checkpoints/skill2_hand_body_split/checkpoint-49400",
            Path(
                "/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Gr00T"
                "/checkpoints/skill2_hand_body_split/checkpoint-49400"
            ),
        ),
    },
}


def _normalize_branch(name: str) -> str:
    key = str(name).strip().lower().replace("_", "-")
    # keep hand_body_split → hand-body-split then map via aliases
    for canonical, spec in _BRANCH_SPECS.items():
        aliases = {a.lower().replace("_", "-") for a in spec["aliases"]}
        aliases.add(canonical.lower().replace("_", "-"))
        if key in aliases or key.replace("-", "") in {
            a.replace("-", "") for a in aliases
        }:
            return canonical
    known = ", ".join(sorted(_BRANCH_SPECS))
    raise ValueError(
        f"Unknown deploy branch {name!r}. Expected one of: {known} "
        f"(aliases: hand_body_split, joint, …)"
    )


def _first_existing(paths: tuple[Path, ...]) -> Path | None:
    for p in paths:
        if p.is_dir() and (
            (p / "config.json").is_file()
            or (p / "model.safetensors.index.json").is_file()
        ):
            return p.resolve()
    return None


def _tree_has_split_support(root: Path) -> bool:
    """True if this Isaac/Litchi tree can construct split hand/body decoders."""
    marker = root / "gr00t/model/gr00t_n1d7/gr00t_n1d7.py"
    if not marker.is_file():
        return False
    try:
        text = marker.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return False
    return "split_hand_body_action_head" in text and "action_decoder_body" in text


def _resolve_litchi_root(
    override: str | Path | None = None,
    *,
    require_split: bool = False,
) -> Path:
    if override:
        root = Path(override).expanduser().resolve()
        if (root / "gr00t/eval/run_gr00t_server.py").is_file():
            if require_split and not _tree_has_split_support(root):
                raise FileNotFoundError(
                    f"LITCHI_ROOT={root} lacks split_hand_body_action_head support"
                )
            return root
        raise FileNotFoundError(f"LITCHI_ROOT invalid: {root}")

    env_root = os.environ.get("LITCHI_ROOT")
    # For hand-split-body, prefer known Litchi-Isaac (has split) before possibly
    # stale $REPO/.deps/Isaac-GR00T copies used for origin-only deploy.
    if require_split:
        candidates = (
            Path(env_root) if env_root else None,
            _DEFAULT_LITCHI,
            _REPO_ROOT / ".deps/Isaac-GR00T",
            _REPO_ROOT.parent / "Gr00T",
            Path(
                "/home/user/YZY/HumanoidWBM/RL_controller/Deploy/"
                "Litchi-GR00T-WholeBodyControl/.deps/Isaac-GR00T"
            ),
        )
    else:
        candidates = (
            Path(env_root) if env_root else None,
            _REPO_ROOT / ".deps/Isaac-GR00T",
            _DEFAULT_LITCHI,
            _REPO_ROOT.parent / "Gr00T",
            Path(
                "/home/user/YZY/HumanoidWBM/RL_controller/Deploy/"
                "Litchi-GR00T-WholeBodyControl/.deps/Isaac-GR00T"
            ),
        )
    for c in candidates:
        if c is None:
            continue
        root = Path(c).expanduser()
        if not (root / "gr00t/eval/run_gr00t_server.py").is_file():
            continue
        if require_split and not _tree_has_split_support(root):
            continue
        return root.resolve()
    # Prefer Litchi-Isaac even if missing at resolve-time (docs / other hosts).
    return _DEFAULT_LITCHI


def resolve_deploy_branch(
    name: str,
    *,
    vla_ckpt: str | Path | None = None,
    litchi_root: str | Path | None = None,
    require_ckpt: bool = False,
) -> DeployBranchProfile:
    """Resolve ``origin`` / ``hand-split-body`` to ckpt + train-aligned RTC."""
    canonical = _normalize_branch(name)
    spec = _BRANCH_SPECS[canonical]
    split = bool(spec["split_hand_body"])

    if vla_ckpt:
        ckpt = Path(vla_ckpt).expanduser().resolve()
    else:
        found = _first_existing(tuple(Path(p) for p in spec["ckpt_candidates"]))
        if found is None:
            ckpt = Path(spec["ckpt_candidates"][0])
            if require_ckpt:
                raise FileNotFoundError(
                    f"No checkpoint found for deploy branch {canonical!r}. "
                    f"Tried: {list(spec['ckpt_candidates'])}"
                )
        else:
            ckpt = found

    if require_ckpt and not (
        (ckpt / "config.json").is_file()
        or (ckpt / "model.safetensors.index.json").is_file()
    ):
        raise FileNotFoundError(f"VLA checkpoint missing/incomplete: {ckpt}")

    return DeployBranchProfile(
        name=canonical,
        split_hand_body=split,
        action_horizon=int(spec["action_horizon"]),
        rtc_overlap_steps=int(spec["rtc_overlap_steps"]),
        rtc_frozen_steps=int(spec["rtc_frozen_steps"]),
        rtc_ramp_rate=float(spec["rtc_ramp_rate"]),
        vla_ckpt=ckpt,
        litchi_root=_resolve_litchi_root(litchi_root, require_split=split),
        description=str(spec["description"]),
        rtc_execution_horizon=int(spec.get("rtc_execution_horizon", 15)),
        num_inference_timesteps=int(spec.get("num_inference_timesteps", 4)),
        rtc_client_enabled=bool(spec.get("rtc_client_enabled", True)),
    )


# Legacy CLI defaults when neither --deploy-branch nor explicit RTC is set.
_LEGACY_ACTION_HORIZON = 40
_LEGACY_RTC_OVERLAP = 16
_LEGACY_RTC_FROZEN = 8
_LEGACY_RTC_RAMP = 5.0


def finalize_inference_rtc(config, profile: DeployBranchProfile | None = None):
    """Resolve Optional RTC fields: explicit CLI > branch profile > legacy.

    ``run_vla_inference`` / ``launch_inference`` should declare
    ``action_horizon`` / ``rtc_*`` as ``None`` by default so that an explicit
    ``--rtc-overlap-steps 16`` is distinguishable from "unset".
    """
    if profile is None and getattr(config, "deploy_branch", None):
        profile = resolve_deploy_branch(config.deploy_branch, require_ckpt=False)

    def _pick(current, branch_val, legacy):
        if current is not None:
            return current
        if profile is not None:
            return branch_val
        return legacy

    config.action_horizon = int(
        _pick(
            getattr(config, "action_horizon", None),
            None if profile is None else profile.action_horizon,
            _LEGACY_ACTION_HORIZON,
        )
    )
    config.rtc_overlap_steps = int(
        _pick(
            getattr(config, "rtc_overlap_steps", None),
            None if profile is None else profile.rtc_overlap_steps,
            _LEGACY_RTC_OVERLAP,
        )
    )
    config.rtc_frozen_steps = int(
        _pick(
            getattr(config, "rtc_frozen_steps", None),
            None if profile is None else profile.rtc_frozen_steps,
            _LEGACY_RTC_FROZEN,
        )
    )
    config.rtc_ramp_rate = float(
        _pick(
            getattr(config, "rtc_ramp_rate", None),
            None if profile is None else profile.rtc_ramp_rate,
            _LEGACY_RTC_RAMP,
        )
    )
    return profile


def apply_branch_to_inference_config(config, profile: DeployBranchProfile) -> None:
    """Backward-compatible alias: finalize RTC with an already-resolved profile."""
    finalize_inference_rtc(config, profile=profile)


def format_branch_banner(profile: DeployBranchProfile) -> str:
    return (
        f"[deploy-branch] {profile.name}: {profile.description}\n"
        f"  split_hand_body={profile.split_hand_body}\n"
        f"  VLA_CKPT={profile.vla_ckpt}\n"
        f"  LITCHI_ROOT={profile.litchi_root}\n"
        f"  RTC H={profile.action_horizon} "
        f"overlap={profile.rtc_overlap_steps} "
        f"frozen={profile.rtc_frozen_steps} "
        f"ramp={profile.rtc_ramp_rate} "
        f"exec_h={profile.rtc_execution_horizon} "
        f"denoise_steps={profile.num_inference_timesteps} "
        f"client_rtc={'on' if profile.rtc_client_enabled else 'off'} "
        f"rate≈{profile.inference_rate:.4f}"
    )


def shell_exports(profile: DeployBranchProfile) -> str:
    """Bash exports for multi-terminal real deploy (C2/C4)."""
    lines = [
        f'export DEPLOY_BRANCH="{profile.name}"',
        f'export SPLIT_HAND_BODY="{1 if profile.split_hand_body else 0}"',
        f'export VLA_CKPT="{profile.vla_ckpt}"',
        f'export LITCHI_ROOT="{profile.litchi_root}"',
        f"export ACTION_HORIZON={profile.action_horizon}",
        f"export RTC_OVERLAP_STEPS={profile.rtc_overlap_steps}",
        f"export RTC_FROZEN_STEPS={profile.rtc_frozen_steps}",
        f"export RTC_RAMP_RATE={profile.rtc_ramp_rate}",
        f"export RTC_EXECUTION_HORIZON={profile.rtc_execution_horizon}",
        f"export NUM_INFERENCE_TIMESTEPS={profile.num_inference_timesteps}",
        f"export RTC_CLIENT_ENABLED={'1' if profile.rtc_client_enabled else '0'}",
        f"export INFERENCE_RATE={profile.inference_rate}",
    ]
    return "\n".join(lines)

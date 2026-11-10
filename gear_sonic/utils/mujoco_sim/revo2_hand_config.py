"""BrainCo Revo2 (6+6) MuJoCo hand overrides for gear_sonic sim.

Matches ``g1_29dof_with_revo2.xml`` / ``scene_43dof_revo2.xml`` joint names
(not the older ``left_hand_revo2_*`` aliases).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping, MutableMapping

DEFAULT_REVO2_SCENE = "gear_sonic/data/robot_model/model_data/g1/scene_43dof_revo2.xml"

# Command packet order: proximal, metacarpal, index, middle, ring, pinky.
LEFT_HAND_JOINT_NAMES = [
    "left_thumb_proximal_joint",
    "left_thumb_metacarpal_joint",
    "left_index_proximal_joint",
    "left_middle_proximal_joint",
    "left_ring_proximal_joint",
    "left_pinky_proximal_joint",
]
RIGHT_HAND_JOINT_NAMES = [
    "right_thumb_proximal_joint",
    "right_thumb_metacarpal_joint",
    "right_index_proximal_joint",
    "right_middle_proximal_joint",
    "right_ring_proximal_joint",
    "right_pinky_proximal_joint",
]

# Normalized [0,1] → rad (Revo2 deploy envelope).
HAND_COMMAND_POSITION_SCALES = [
    1.0472,
    1.5184,
    1.4661,
    1.4661,
    1.4661,
    1.4661,
]


def resolve_revo2_scene(revo2_scene: str = "", *, gear_sonic_root: str | Path | None = None) -> Path:
    """Return absolute scene path; default is in-repo ``scene_43dof_revo2.xml``."""
    raw = (revo2_scene or "").strip() or DEFAULT_REVO2_SCENE
    p = Path(raw).expanduser()
    if p.is_file():
        return p.resolve()
    if gear_sonic_root is not None:
        cand = Path(gear_sonic_root) / raw
        if cand.is_file():
            return cand.resolve()
    # Relative to package root (…/GR00T-WholeBodyControl)
    here = Path(__file__).resolve()
    repo = here.parents[3]  # gear_sonic/utils/mujoco_sim -> repo
    cand = repo / raw
    if cand.is_file():
        return cand.resolve()
    raise FileNotFoundError(f"Revo2 MuJoCo scene not found: {raw}")


def apply_revo2_hand_wbc_config(
    wbc_config: MutableMapping[str, Any],
    *,
    revo2_scene: str = "",
    gear_sonic_root: str | Path | None = None,
) -> Mapping[str, Any]:
    """Patch WBC yaml dict for G1 body + BrainCo Revo2 hands."""
    scene = resolve_revo2_scene(revo2_scene, gear_sonic_root=gear_sonic_root)
    print(f"[revo2_hand] ROBOT_SCENE={scene} (g1_debug left/right_hand_q → VLA proprio41)")
    effort = list(wbc_config["motor_effort_limit_list"])
    # Dex3 yaml interleaves 7-DoF hands after each arm; strip to body29.
    body_effort = effort[:22] + effort[29:36]
    if len(body_effort) != 29:
        raise ValueError(f"unexpected G1 body effort layout: {len(body_effort)}")
    # scene actuators: body29 + 22 finger motors (6+6 active + distal mimics; tip welded).
    finger_effort = [2.0] * 22
    wbc_config.update(
        {
            "ROBOT_SCENE": str(scene),
            "NUM_HAND_MOTORS": 6,
            "NUM_HAND_JOINTS": 6,
            "motor_effort_limit_list": body_effort + finger_effort,
            "HAND_COMMAND_POSITION_SCALES": list(HAND_COMMAND_POSITION_SCALES),
            "HAND_COMMAND_POSITION_OFFSETS": [0.0] * 6,
            "LEFT_HAND_JOINT_NAMES": list(LEFT_HAND_JOINT_NAMES),
            "RIGHT_HAND_JOINT_NAMES": list(RIGHT_HAND_JOINT_NAMES),
            "HAND_SIM_KP": 12.0,
            "HAND_SIM_KD": 0.35,
        }
    )
    return wbc_config

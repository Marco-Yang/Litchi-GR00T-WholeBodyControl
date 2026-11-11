#!/usr/bin/env python3
"""Load kit scene XML and verify Revo2 hand joints (no Unitree rubber-hand bodies)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main() -> int:
    import mujoco

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--scene-xml",
        type=Path,
        default=Path(__file__).resolve().parent / "assets" / "scene_43dof_revo2.xml",
    )
    args = ap.parse_args()
    scene = args.scene_xml.resolve()
    if not scene.is_file():
        print(f"ERROR: scene not found: {scene}", file=sys.stderr)
        return 2

    print(f"loading: {scene}")
    try:
        model = mujoco.MjModel.from_xml_path(str(scene))
    except Exception as exc:
        print(f"ERROR: from_xml_path failed: {exc}", file=sys.stderr)
        print(
            "HINT: meshdir must resolve to gear_sonic/.../g1/meshes; "
            "run this kit from inside Litchi-GR00T-WholeBodyControl tree.",
            file=sys.stderr,
        )
        return 3

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    hands: list[str] = []
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) or ""
        if any(k in name for k in ("thumb", "index", "middle", "ring", "pinky")):
            hands.append(name)

    rubber = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        for i in range(model.nbody)
        if "rubber_hand" in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "")
    ]

    print(f"nq={model.nq} nv={model.nv} nbody={model.nbody} ngeom={model.ngeom}")
    print(f"hand_joints={len(hands)}")
    for n in hands:
        print(f"  {n}")
    print(f"rubber_hand_bodies={rubber}")

    ok = len(hands) >= 20 and not rubber
    if ok:
        print("CHECK_OK Revo2 scene loads; finger joints present; no rubber_hand body")
        return 0
    print("CHECK_FAIL unexpected hand layout", file=sys.stderr)
    return 4


if __name__ == "__main__":
    raise SystemExit(main())

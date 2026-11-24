#!/usr/bin/env python3
"""Shared-VLM multi-skill PolicyServer (1× backbone + N× action heads).

Loads every skill's action_head into GPU once, shares the frozen Cosmos
backbone, and switches skills by pointer swap (millisecond-scale).

Ports:
  * ``policy_port`` (default 5550) — Isaac-GR00T PolicyServer (get_action/reset)
  * ``supervisor_port`` (default 5590) — control REP: switch / status
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import sys
import threading
from typing import Any

import tyro
import zmq


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


@dataclass
class SharedVlmServerConfig:
    skills_config: str = str(
        _repo_root() / "gear_sonic/config/multi_skill_nortc_skills.json"
    )
    litchi_root: str = str(_repo_root() / ".deps/Isaac-GR00T")
    device: str = "cuda:0"
    embodiment_tag: str = "UNITREE_G1_SONIC"
    host: str = "0.0.0.0"
    policy_port: int | None = None
    supervisor_port: int | None = None
    strict: bool = True
    verify_backbone_identical: bool = True
    # Boundary-judge vision tokens (training-identical VisionEncoder), PUB for observe.
    publish_boundary_vision: bool = True
    boundary_vision_port: int = 5591
    camera_host: str = "192.168.123.164"
    camera_port: int = 5555
    boundary_vision_hz: float = 5.0
    boundary_vision_ckpt: str | None = None


def _prepare_env(litchi_root: Path) -> None:
    """Ensure Isaac-GR00T + gear_sonic imports resolve; prefer offline HF."""
    repo = _repo_root()
    # gear_sonic lives under REPO; gr00t under LITCHI_ROOT.
    for path in (str(litchi_root), str(repo)):
        if path not in sys.path:
            sys.path.insert(0, path)
    os.environ.setdefault("HF_HOME", str(Path.home() / ".cache/huggingface"))
    snap = (
        Path(os.environ["HF_HOME"])
        / "hub/models--nvidia--Cosmos-Reason2-2B"
        / "snapshots/9ce19a195e423419c349abfc86fd07178b230561"
    )
    os.environ.setdefault("GROOT_BACKBONE_PATH", str(snap))
    os.environ.setdefault("HF_HUB_OFFLINE", "1")
    os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")
    os.environ.setdefault("GROOT_HF_LOCAL_FIRST", "1")
    os.environ.setdefault("NO_ALBUMENTATIONS_UPDATE", "1")


def _handle_control(raw: str, policy: Any) -> dict:
    raw = (raw or "").strip()
    if not raw:
        return {"ok": False, "error": "empty request"}
    try:
        if raw[0] == "{":
            msg = json.loads(raw)
        elif raw.startswith("skill:"):
            msg = {"cmd": "switch", "skill_id": raw.split(":", 1)[1].strip()}
        elif raw in policy.registry.skills:
            msg = {"cmd": "switch", "skill_id": raw}
        elif raw == "status":
            msg = {"cmd": "status"}
        else:
            return {"ok": False, "error": f"unrecognized request: {raw!r}"}
    except json.JSONDecodeError as exc:
        return {"ok": False, "error": f"bad json: {exc}"}

    cmd = str(msg.get("cmd") or "switch").strip().lower()
    if cmd == "status":
        return policy.status()
    if cmd in ("switch", "load", "set"):
        skill_id = str(msg.get("skill_id") or msg.get("id") or "").strip()
        if not skill_id:
            return {"ok": False, "error": "missing skill_id"}
        try:
            return policy.switch_skill(skill_id)
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": str(exc), "skill_id": skill_id}
    return {"ok": False, "error": f"unknown cmd={cmd!r}"}


def _serve_control(policy: Any, port: int) -> None:
    ctx = zmq.Context()
    sock = ctx.socket(zmq.REP)
    sock.bind(f"tcp://*:{port}")
    print(f"[shared-vlm] control REP on tcp://*:{port}", flush=True)
    try:
        while True:
            raw = sock.recv_string()
            print(f"[shared-vlm] control request: {raw!r}", flush=True)
            try:
                reply = _handle_control(raw, policy)
            except Exception as exc:  # noqa: BLE001
                reply = {"ok": False, "error": str(exc)}
            sock.send_string(json.dumps(reply, ensure_ascii=False))
    finally:
        sock.close(0)
        ctx.term()


def main(config: SharedVlmServerConfig) -> None:
    litchi_root = Path(config.litchi_root).expanduser().resolve()
    if not (litchi_root / "gr00t/eval/run_gr00t_server.py").is_file():
        raise FileNotFoundError(f"Missing Isaac-GR00T at {litchi_root}")
    _prepare_env(litchi_root)

    from gr00t.policy.server_client import PolicyServer

    from gear_sonic.utils.inference.shared_vlm_multi_skill_policy import (
        SharedVlmMultiSkillPolicy,
    )
    from gear_sonic.utils.inference.skill_registry import load_skill_registry

    registry = load_skill_registry(config.skills_config)
    policy_port = int(config.policy_port or registry.policy_port)
    supervisor_port = int(config.supervisor_port or registry.supervisor_port)
    # Keep registry ports aligned with CLI overrides for client replies.
    registry.policy_port = policy_port
    registry.supervisor_port = supervisor_port

    print(
        f"[shared-vlm] skills={[s.as_dict() for s in registry.skills.values()]}",
        flush=True,
    )
    policy = SharedVlmMultiSkillPolicy(
        registry,
        embodiment_tag=config.embodiment_tag,
        device=config.device,
        strict=config.strict,
        verify_backbone_identical=config.verify_backbone_identical,
    )

    vision_pub = None
    if config.publish_boundary_vision:
        from gear_sonic.utils.inference.boundary_vision_publisher import (
            BoundaryVisionPublisher,
            BoundaryVisionPublisherConfig,
        )

        # Default: skill1 ckpt (same visual weights as all nortc skills).
        default_skill = registry.get(registry.default_skill_id)
        vision_ckpt = config.boundary_vision_ckpt or str(default_skill.checkpoint)
        vision_pub = BoundaryVisionPublisher(
            BoundaryVisionPublisherConfig(
                camera_host=config.camera_host,
                camera_port=int(config.camera_port),
                pub_bind=f"tcp://*:{int(config.boundary_vision_port)}",
                vision_checkpoint=vision_ckpt,
                device=config.device,
                hz=float(config.boundary_vision_hz),
            )
        )
        vision_pub.start()

    control_thread = threading.Thread(
        target=_serve_control,
        args=(policy, supervisor_port),
        daemon=True,
        name="shared-vlm-control",
    )
    control_thread.start()

    print(
        f"[shared-vlm] PolicyServer host={config.host} port={policy_port} "
        f"device={config.device}",
        flush=True,
    )
    try:
        PolicyServer.start_server(policy, port=policy_port, host=config.host)
    finally:
        if vision_pub is not None:
            vision_pub.close()


if __name__ == "__main__":
    main(tyro.cli(SharedVlmServerConfig))

"""Shared-backbone multi-skill Gr00t policy (1× VLM + N× action heads)."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import torch
from safetensors import safe_open
from transformers import AutoConfig

from gr00t.data.embodiment_tags import EmbodimentTag
from gr00t.model.gr00t_n1d7.gr00t_n1d7 import Gr00tN1d7ActionHead
from gr00t.policy.gr00t_policy import Gr00tPolicy

from gear_sonic.utils.inference.skill_registry import SkillRegistry, SkillSpec


def _load_statistics_json(ckpt: Path) -> dict[str, Any]:
    """Load per-checkpoint ``statistics.json`` used for state/action normalize."""
    path = ckpt / "statistics.json"
    if not path.is_file():
        raise FileNotFoundError(f"Missing statistics.json under {ckpt}")
    return json.loads(path.read_text(encoding="utf-8"))


def _load_action_head_state_dict(ckpt: Path) -> dict[str, torch.Tensor]:
    """Read only ``action_head.*`` tensors from a full N1.7 checkpoint."""
    prefix = "action_head."
    sd: dict[str, torch.Tensor] = {}
    files = sorted(ckpt.glob("*.safetensors"))
    if not files:
        raise FileNotFoundError(f"No *.safetensors under {ckpt}")
    for path in files:
        with safe_open(path, framework="pt") as handle:
            for key in handle.keys():
                if key.startswith(prefix):
                    sd[key[len(prefix) :]] = handle.get_tensor(key)
    if not sd:
        raise RuntimeError(f"No action_head.* tensors in {ckpt}")
    return sd


def _backbone_keys_equal(ckpt_a: Path, ckpt_b: Path) -> bool:
    """Exact equality check for ``backbone.*`` tensors (CPU, streaming)."""
    prefix = "backbone."

    def _collect(ckpt: Path) -> dict[str, torch.Tensor]:
        out: dict[str, torch.Tensor] = {}
        for path in sorted(ckpt.glob("*.safetensors")):
            with safe_open(path, framework="pt") as handle:
                for key in handle.keys():
                    if key.startswith(prefix):
                        out[key] = handle.get_tensor(key)
        return out

    a = _collect(ckpt_a)
    b = _collect(ckpt_b)
    if a.keys() != b.keys():
        return False
    for key in a:
        if not torch.equal(a[key], b[key]):
            return False
    return True


def load_action_head_from_checkpoint(
    ckpt: Path,
    *,
    device: str | torch.device,
    dtype: torch.dtype = torch.bfloat16,
) -> Gr00tN1d7ActionHead:
    """Construct ``Gr00tN1d7ActionHead`` and load weights from ``ckpt`` only."""
    import gr00t.model  # noqa: F401 — register AutoConfig

    config = AutoConfig.from_pretrained(str(ckpt))
    head = Gr00tN1d7ActionHead(config)
    state = _load_action_head_state_dict(ckpt)
    missing, unexpected = head.load_state_dict(state, strict=True)
    if missing or unexpected:
        raise RuntimeError(
            f"action_head load mismatch for {ckpt}: missing={missing} unexpected={unexpected}"
        )
    head.to(device=device, dtype=dtype)
    head.eval()
    return head


class SharedVlmMultiSkillPolicy(Gr00tPolicy):
    """One shared Cosmos backbone; hot-swap among resident action heads."""

    def __init__(
        self,
        registry: SkillRegistry,
        *,
        embodiment_tag: EmbodimentTag | str,
        device: str = "cuda:0",
        strict: bool = True,
        verify_backbone_identical: bool = True,
    ):
        default = registry.get(registry.default_skill_id)
        print(
            f"[shared-vlm] loading base skill={default.id} ({default.label}) "
            f"from {default.checkpoint}",
            flush=True,
        )
        super().__init__(
            embodiment_tag=embodiment_tag,
            model_path=str(default.checkpoint),
            device=device,
            strict=strict,
        )
        self.registry = registry
        self.device_str = str(device)
        self.active_skill_id = default.id
        self.action_heads: dict[str, Gr00tN1d7ActionHead] = {
            default.id: self.model.action_head
        }
        # Per-skill action/state normalization (must switch with the head).
        self.skill_statistics: dict[str, dict[str, Any]] = {
            default.id: _load_statistics_json(default.checkpoint)
        }

        for skill_id in registry.ordered_ids():
            if skill_id == default.id:
                continue
            skill = registry.get(skill_id)
            if verify_backbone_identical:
                print(
                    f"[shared-vlm] verifying backbone identical: "
                    f"{default.id} vs {skill_id} ...",
                    flush=True,
                )
                if not _backbone_keys_equal(default.checkpoint, skill.checkpoint):
                    raise RuntimeError(
                        f"Backbone weights differ between skill {default.id} "
                        f"({default.checkpoint}) and skill {skill_id} "
                        f"({skill.checkpoint}); shared-VLM mode is invalid."
                    )
                print(f"[shared-vlm] backbone OK for skill={skill_id}", flush=True)
            print(
                f"[shared-vlm] loading action_head skill={skill_id} ({skill.label}) "
                f"from {skill.checkpoint}",
                flush=True,
            )
            head = load_action_head_from_checkpoint(
                skill.checkpoint,
                device=device,
                dtype=torch.bfloat16,
            )
            self.action_heads[skill_id] = head
            self.skill_statistics[skill_id] = _load_statistics_json(skill.checkpoint)

        # Point model at default head (already is, but be explicit).
        self.model.action_head = self.action_heads[self.active_skill_id]
        self._apply_skill_statistics(self.active_skill_id)
        print(
            f"[shared-vlm] ready skills={list(self.action_heads)} "
            f"active={self.active_skill_id} "
            f"(1×backbone + {len(self.action_heads)}×action_head; "
            f"per-skill statistics.json loaded)",
            flush=True,
        )

    def active_skill(self) -> SkillSpec:
        return self.registry.get(self.active_skill_id)

    def _apply_skill_statistics(self, skill_id: str) -> None:
        """Swap processor normalize/denorm stats to the active skill's checkpoint."""
        stats = self.skill_statistics[skill_id]
        # override=True is required: otherwise unitree_g1_sonic stats from skill1
        # are kept forever and skill2/3 denorm silently uses the wrong mean/std.
        self.processor.set_statistics(stats, override=True)

    def switch_skill(self, skill_id: str) -> dict[str, Any]:
        skill = self.registry.get(skill_id)
        if skill.id not in self.action_heads:
            raise KeyError(f"No action_head loaded for skill_id={skill.id!r}")
        if skill.id != self.active_skill_id:
            self.model.action_head = self.action_heads[skill.id]
            self._apply_skill_statistics(skill.id)
            self.active_skill_id = skill.id
            self.reset()
            print(
                f"[shared-vlm] SWITCH active→{skill.id} ({skill.label}) "
                f"(action_head + statistics.json)",
                flush=True,
            )
        return {
            "ok": True,
            "skill_id": skill.id,
            "name": skill.name,
            "label": skill.label,
            "checkpoint": str(skill.checkpoint),
            "prompt": skill.prompt,
            "policy_host": self.registry.policy_host,
            "policy_port": self.registry.policy_port,
            "ready": True,
            "promoted": True,
            "warm": True,
            "mode": "shared-vlm",
            "loaded_skills": list(self.action_heads),
            "statistics_swapped": True,
        }

    def status(self) -> dict[str, Any]:
        skill = self.active_skill()
        return {
            "ok": True,
            "mode": "shared-vlm",
            "active_skill_id": skill.id,
            "active_port": self.registry.policy_port,
            "prompt": skill.prompt,
            "loaded_skills": list(self.action_heads),
            "policy_port": self.registry.policy_port,
        }

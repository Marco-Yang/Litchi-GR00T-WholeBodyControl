"""Multi-skill VLA registry for keyboard hot-switching (nortc deploy)."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class SkillSpec:
    id: str
    name: str
    label: str
    checkpoint: Path
    prompt: str

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "label": self.label,
            "checkpoint": str(self.checkpoint),
            "prompt": self.prompt,
        }


@dataclass
class SkillRegistry:
    skills: dict[str, SkillSpec]
    default_skill_id: str
    policy_host: str = "127.0.0.1"
    policy_port: int = 5550
    standby_port: int = 5551
    supervisor_host: str = "127.0.0.1"
    supervisor_port: int = 5590

    def get(self, skill_id: str) -> SkillSpec:
        key = str(skill_id).strip()
        if key not in self.skills:
            known = ", ".join(sorted(self.skills))
            raise KeyError(f"Unknown skill_id={key!r}; known: {known}")
        return self.skills[key]

    def ordered_ids(self) -> list[str]:
        return sorted(self.skills.keys(), key=lambda x: (len(x), x))

    def next_id(self, skill_id: str) -> str:
        """Cyclic next skill in ordered id list (1→2→3→1)."""
        ids = self.ordered_ids()
        if skill_id not in ids:
            raise KeyError(skill_id)
        return ids[(ids.index(skill_id) + 1) % len(ids)]


def load_skill_registry(path: str | Path) -> SkillRegistry:
    cfg_path = Path(path).expanduser().resolve()
    raw = json.loads(cfg_path.read_text(encoding="utf-8"))
    skills: dict[str, SkillSpec] = {}
    for item in raw.get("skills", []):
        sid = str(item["id"]).strip()
        ckpt = Path(item["checkpoint"]).expanduser().resolve()
        if not (ckpt / "config.json").is_file():
            raise FileNotFoundError(
                f"Skill {sid}: missing config.json under checkpoint {ckpt} "
                "(use checkpoint-XXXX dir, not global_step*)"
            )
        skills[sid] = SkillSpec(
            id=sid,
            name=str(item.get("name") or f"skill{sid}"),
            label=str(item.get("label") or item.get("name") or sid),
            checkpoint=ckpt,
            prompt=str(item["prompt"]).strip(),
        )
    if not skills:
        raise ValueError(f"No skills defined in {cfg_path}")
    default_id = str(raw.get("default_skill_id") or next(iter(skills))).strip()
    if default_id not in skills:
        raise KeyError(f"default_skill_id={default_id!r} not in skills")
    return SkillRegistry(
        skills=skills,
        default_skill_id=default_id,
        policy_host=str(raw.get("policy_host") or "127.0.0.1"),
        policy_port=int(raw.get("policy_port") or 5550),
        standby_port=int(raw.get("standby_port") or 5551),
        supervisor_host=str(raw.get("supervisor_host") or "127.0.0.1"),
        supervisor_port=int(raw.get("supervisor_port") or 5590),
    )

"""Training-identical VisionEncoder for the skill-boundary judge.

Mirrors ``Litchi-Agent-for-HumanoidVLA/skill_agent/vision_encoder.py``:
frozen VLA ``backbone.model.model.visual.*`` into Qwen3-VL vision tower,
resize to ``image_wh``, spatial-merge, then adaptive pool to ``grid``.
"""

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn.functional as F

VISUAL_PREFIX = "backbone.model.model.visual."
BASE_VISUAL_PREFIX = "model.visual."
DEFAULT_CAMERAS = ("ego_view", "head")
BOUNDARY_VISION_TOPIC = b"boundary_vision"


def resize_rgb(img: np.ndarray, wh: tuple[int, int]) -> np.ndarray:
    if img.shape[1] == wh[0] and img.shape[0] == wh[1]:
        return img
    return cv2.resize(img, wh, interpolation=cv2.INTER_AREA)


def _load_visual_state(vla_checkpoint: Path | None, base: str) -> dict[str, torch.Tensor]:
    from safetensors import safe_open

    if vla_checkpoint is not None:
        index = json.loads((vla_checkpoint / "model.safetensors.index.json").read_text())["weight_map"]
        files: dict[str, list[str]] = {}
        for k, f in index.items():
            if k.startswith(VISUAL_PREFIX):
                files.setdefault(f, []).append(k)
        out: dict[str, torch.Tensor] = {}
        for f, keys in files.items():
            with safe_open(str(vla_checkpoint / f), "pt") as fh:
                out.update({k[len(VISUAL_PREFIX) :]: fh.get_tensor(k) for k in keys})
        return out
    from huggingface_hub import snapshot_download

    snap = Path(snapshot_download(base))
    out = {}
    for f in sorted(snap.glob("*.safetensors")):
        with safe_open(str(f), "pt") as fh:
            out.update(
                {
                    k[len(BASE_VISUAL_PREFIX) :]: fh.get_tensor(k)
                    for k in fh.keys()
                    if k.startswith(BASE_VISUAL_PREFIX)
                }
            )
    return out


class VisionEncoder:
    """RGB ``[H, W, 3]`` images -> ``[B, rows * cols, D]`` grid-pooled merged vision tokens."""

    def __init__(
        self,
        base: str = "Qwen/Qwen3-VL-2B-Instruct",
        vla_checkpoint: str | Path | None = None,
        image_wh: tuple[int, int] = (384, 288),
        grid: tuple[int, int] = (3, 4),
        device: str = "cuda",
        dtype: torch.dtype = torch.bfloat16,
    ):
        from transformers import AutoConfig, AutoImageProcessor
        from transformers.models.qwen3_vl.modeling_qwen3_vl import Qwen3VLVisionModel

        cfg = AutoConfig.from_pretrained(base).vision_config
        cfg._attn_implementation = "sdpa"
        model = Qwen3VLVisionModel(cfg)
        missing, unexpected = model.load_state_dict(
            _load_visual_state(Path(vla_checkpoint) if vla_checkpoint else None, base),
            strict=False,
        )
        if missing or unexpected:
            raise RuntimeError(
                f"vision weights mismatch: missing={missing[:5]} unexpected={unexpected[:5]}"
            )
        self.model = model.to(device=device, dtype=dtype).eval().requires_grad_(False)
        self.processor = AutoImageProcessor.from_pretrained(base)
        self.device, self.dtype = device, dtype
        self.image_wh, self.grid = tuple(image_wh), tuple(grid)
        unit = cfg.patch_size * cfg.spatial_merge_size
        if image_wh[0] % unit or image_wh[1] % unit:
            raise ValueError(f"image size {image_wh} must be a multiple of {unit}")
        self.merged_hw = (image_wh[1] // unit, image_wh[0] // unit)
        self.dim = cfg.out_hidden_size
        self.n_tokens = grid[0] * grid[1]
        self.source = str(vla_checkpoint) if vla_checkpoint else base

    @torch.inference_mode()
    def encode(self, images: list[np.ndarray]) -> torch.Tensor:
        imgs = [resize_rgb(im, self.image_wh) for im in images]
        enc = self.processor(images=imgs, return_tensors="pt", do_resize=False)
        # transformers>=4.57 returns (merged_hidden_states, deepstack_feature_lists).
        out = self.model(
            enc["pixel_values"].to(self.device, self.dtype),
            grid_thw=enc["image_grid_thw"].to(self.device),
        )
        if isinstance(out, tuple):
            pooled = out[0]
        else:
            pooled = getattr(out, "pooler_output", out)
        mh, mw = self.merged_hw
        x = pooled.view(len(imgs), mh, mw, -1).permute(0, 3, 1, 2).float()
        x = F.adaptive_avg_pool2d(x, self.grid)
        return x.flatten(2).transpose(1, 2).contiguous()

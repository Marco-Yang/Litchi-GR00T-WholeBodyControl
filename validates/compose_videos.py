#!/usr/bin/env python3
"""Stitch validate videos into a grid with true realtime speed alignment (ffmpeg).

Rules (default):
  - All panels start at t=0 together
  - Playback speed = original realtime (setpts + fps, no time-stretch)
  - Timeline = longest panel; shorter panels hold last frame (tpad clone)

Example:
  python compose_videos.py --model-tag ORIGIN --layout 2x2 \\
    --panel ep000_smpl_h264.mp4:"SMPL skeleton" \\
    --panel ep000_qpos_mujoco_h264.mp4:"qpos MuJoCo" \\
    --panel ep000_vla_openloop_h264.mp4:"VLA open-loop" \\
    --panel ep000_vla_closedloop_h264.mp4:"VLA closed-loop RTC" \\
    --out ep000_2x2_smpl_qpos_ol_cl_h264.mp4
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_VIDEOS_ROOT = _HERE / "videos"


def _default_ffmpeg() -> Path:
    for cand in (
        Path(os.environ["FFMPEG_BIN"]) if os.environ.get("FFMPEG_BIN") else None,
        Path.home() / "miniconda3/envs/gr00t/bin/ffmpeg",
        Path("/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/.tools/bin/ffmpeg"),
    ):
        if cand is not None and cand.is_file():
            return cand
    which = shutil.which("ffmpeg")
    return Path(which) if which else Path("ffmpeg")


_DEFAULT_FFMPEG = _default_ffmpeg()
_DEFAULT_FFPROBE = _DEFAULT_FFMPEG.with_name("ffprobe")

sys.path.insert(0, str(_HERE / "lib"))
from burn_bottom_label import burn as burn_label  # noqa: E402


def _videos_dir(model_tag: str | None) -> Path:
    return _VIDEOS_ROOT / model_tag if model_tag else _VIDEOS_ROOT


def _resolve(p: str | Path, model_tag: str | None = None) -> Path:
    path = Path(p)
    if path.is_file():
        return path
    vdir = _videos_dir(model_tag)
    for cand in (vdir / path.name, vdir / path, _VIDEOS_ROOT / path.name):
        if cand.is_file():
            return cand
    return path


def _parse_panel(spec: str, model_tag: str | None = None) -> tuple[Path, str]:
    if ":" in spec:
        path_s, label = spec.split(":", 1)
    else:
        path_s, label = spec, Path(spec).stem
    return _resolve(path_s, model_tag), label


def _probe_duration(ffprobe: Path, path: Path) -> float:
    out = subprocess.check_output(
        [
            str(ffprobe),
            "-v",
            "error",
            "-show_entries",
            "format=duration",
            "-of",
            "csv=p=0",
            str(path),
        ],
        text=True,
    ).strip()
    return float(out)


def _layout_cols(layout: str, n: int) -> int:
    if layout == "hstack":
        return n
    if layout == "vstack":
        return 1
    if "x" in layout:
        return int(layout.split("x", 1)[0])
    return n


def compose(
    panels: list[tuple[Path, str]],
    out: Path,
    *,
    layout: str,
    panel_w: int,
    panel_h: int,
    ffmpeg: Path,
    ffprobe: Path,
    relabel: bool = True,
    model_tag: str | None = None,
    out_fps: float = 30.0,
    align: str = "max",
) -> Path:
    if not ffmpeg.is_file():
        raise SystemExit(f"missing ffmpeg: {ffmpeg}")
    if not ffprobe.is_file():
        raise SystemExit(f"missing ffprobe: {ffprobe}")

    # Optional bottom labels → temp files (keep source fps/duration).
    tmp_dir: tempfile.TemporaryDirectory[str] | None = None
    inputs: list[Path] = []
    if relabel:
        tmp_dir = tempfile.TemporaryDirectory(prefix="validate_compose_")
        tdir = Path(tmp_dir.name)
        for i, (path, label) in enumerate(panels):
            labeled = tdir / f"panel_{i:02d}.mp4"
            burn_label(path, label, labeled, ffmpeg)
            inputs.append(labeled)
            print(f"[compose] labeled {label!r} ← {path}", flush=True)
    else:
        inputs = [p for p, _ in panels]
        for path, label in panels:
            print(f"[compose] use {label!r} ← {path}", flush=True)

    durs = [_probe_duration(ffprobe, p) for p in inputs]
    if align == "min":
        timeline = min(durs)
        pads = [0.0] * len(durs)
        # trim via -t below; no pad
    else:
        timeline = max(durs)
        pads = [max(0.0, timeline - d) for d in durs]
    print(
        f"[compose] durs={[round(d, 3) for d in durs]} timeline={timeline:.3f}s "
        f"align={align} out_fps={out_fps}",
        flush=True,
    )

    cols = _layout_cols(layout, len(inputs))
    rows = (len(inputs) + cols - 1) // cols

    # Per-panel: realtime speed (setpts), CFR out_fps, scale, pad/hold last frame.
    filters = []
    for i, pad in enumerate(pads):
        chain = (
            f"[{i}:v]setpts=PTS-STARTPTS,"
            f"fps={out_fps}:round=near,"
            f"scale={panel_w}:{panel_h}:flags=bicubic,setsar=1,format=yuv420p"
        )
        if align == "max" and pad > 1e-6:
            chain += f",tpad=stop_mode=clone:stop_duration={pad:.6f}"
        chain += f"[v{i}]"
        filters.append(chain)

    # Build row stacks then vstack.
    row_labels = []
    for r in range(rows):
        idxs = list(range(r * cols, min((r + 1) * cols, len(inputs))))
        # pad missing cells with last panel freeze (black via color source would need extra input;
        # require full rectangle layouts in practice)
        if len(idxs) == 1:
            filters.append(f"[v{idxs[0]}]copy[row{r}]")
        else:
            ins = "".join(f"[v{j}]" for j in idxs)
            filters.append(f"{ins}hstack=inputs={len(idxs)}[row{r}]")
        row_labels.append(f"[row{r}]")
    if rows == 1:
        filters.append(f"{row_labels[0]}copy[vout]")
    else:
        filters.append(f"{''.join(row_labels)}vstack=inputs={rows}[vout]")

    out = Path(out)
    if not out.is_absolute():
        out = _videos_dir(model_tag) / out
    out.parent.mkdir(parents=True, exist_ok=True)

    cmd = [str(ffmpeg), "-y"]
    for p in inputs:
        cmd += ["-i", str(p)]
    cmd += [
        "-filter_complex",
        ";".join(filters),
        "-map",
        "[vout]",
        "-t",
        f"{timeline:.6f}",
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(out),
    ]
    print("[compose] ffmpeg realtime CFR stitch...", flush=True)
    subprocess.check_call(cmd)
    if tmp_dir is not None:
        tmp_dir.cleanup()
    print(f"[compose] {out} timeline={timeline:.3f}s layout={layout}", flush=True)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--panel", action="append", required=True, help="path[:label]")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--model-tag", default=None)
    ap.add_argument("--layout", default="hstack", help="hstack | vstack | CxR e.g. 2x2")
    ap.add_argument("--panel-w", type=int, default=640)
    ap.add_argument("--panel-h", type=int, default=480)
    ap.add_argument("--out-fps", type=float, default=30.0, help="CFR display rate; does not change speed")
    ap.add_argument(
        "--align",
        choices=("min", "max"),
        default="max",
        help="max: pad short to longest (hold last). min: trim all to shortest.",
    )
    ap.add_argument("--ffmpeg", type=Path, default=_DEFAULT_FFMPEG)
    ap.add_argument("--ffprobe", type=Path, default=_DEFAULT_FFPROBE)
    ap.add_argument("--no-relabel", action="store_true")
    args = ap.parse_args()

    panels = [_parse_panel(s, args.model_tag) for s in args.panel]
    for p, _ in panels:
        if not p.is_file():
            raise SystemExit(f"missing panel video: {p}")
    compose(
        panels,
        args.out,
        layout=args.layout,
        panel_w=args.panel_w,
        panel_h=args.panel_h,
        ffmpeg=args.ffmpeg,
        ffprobe=args.ffprobe,
        relabel=not args.no_relabel,
        model_tag=args.model_tag,
        out_fps=args.out_fps,
        align=args.align,
    )


if __name__ == "__main__":
    main()

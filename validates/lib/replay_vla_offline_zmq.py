#!/usr/bin/env python3
"""Offline GR00T → SONIC ZMQ v4 replay (WBC + Litchi only; no Phi_0_wpy).

Uses GT ego (+ head when present) frames for the whole episode (dataset dual-cam).
Control stream: hold GT motion_token[0] (+ GT hands) until GO, optionally stream
GT tokens for --gt-bootstrap-frames, then switch to VLA inference outputs.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path

import cv2
import msgpack
import numpy as np
import pyarrow.parquet as pq
import zmq

# Prefer env overrides; else resolve from this file / common local layouts.
_VALIDATE_LIB = Path(__file__).resolve().parent
_WBC_DEFAULT = _VALIDATE_LIB.parents[1]  # .../Litchi-GR00T-WholeBodyControl
_LITCHI_CANDIDATES = [
    Path(__import__("os").environ[k])
    for k in ("LITCHI_ROOT", "ISAAC_GR00T_ROOT", "GR00T_ROOT")
    if __import__("os").environ.get(k)
] + [
    Path("/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/Litchi-Isaac-GR00T"),
    _WBC_DEFAULT.parent / "Gr00T",
]
WBC = Path(__import__("os").environ.get("WBC_ROOT", str(_WBC_DEFAULT))).resolve()


def _is_isaac_gr00t_root(p: Path) -> bool:
    # Require data/ so a stub Deploy/Gr00T/gr00t/policy tree is not selected.
    return (p / "gr00t" / "data").is_dir() and (p / "gr00t" / "policy").is_dir()


LITCHI = next(
    (p.resolve() for p in _LITCHI_CANDIDATES if _is_isaac_gr00t_root(p)),
    None,
)
# If already importable (editable / venv), keep that root on path; else insert LITCHI.
try:
    import gr00t as _gr00t_probe  # noqa: F401
except ImportError:
    if LITCHI is not None and str(LITCHI) not in sys.path:
        sys.path.insert(0, str(LITCHI))
if str(WBC) not in sys.path:
    sys.path.insert(0, str(WBC))
if LITCHI is None:
    try:
        import gr00t as _g
        from pathlib import Path as _P

        LITCHI = _P(_g.__file__).resolve().parents[1]
    except Exception:
        LITCHI = _WBC_DEFAULT

from gear_sonic.utils.teleop.zmq.zmq_planner_sender import (  # noqa: E402
    build_command_message,
    pack_pose_message,
)

# RTC helpers mirrored from gear_sonic.utils.inference.vla_utils (avoid importing
# that module here: it pulls RobotModel/pinocchio, which breaks the VLA venv).


def calculate_latency_compensated_index(
    inference_delay: float, control_freq: float, action_horizon: int
) -> int:
    raw_index = np.round(inference_delay * control_freq)
    return int(np.clip(raw_index, 0, action_horizon - 1))


def rtc_execution_index(chunk_index: int, action_horizon: int, overlap_steps: int) -> int:
    origin = action_horizon - overlap_steps
    return int(np.clip(chunk_index - origin, 0, action_horizon - 1))


def should_trigger_rtc_inference(
    cached_chunk_exists: bool,
    inference_thread_running: bool,
    action_chunk_index: int,
    action_horizon: int,
    overlap_steps: int,
) -> bool:
    if not cached_chunk_exists:
        return True
    if inference_thread_running:
        return False
    return action_chunk_index >= action_horizon - overlap_steps
from gr00t.data.embodiment_tags import EmbodimentTag  # noqa: E402
from gr00t.data.utils import parse_observation_gr00t  # noqa: E402
from gr00t.policy.gr00t_policy import Gr00tPolicy  # noqa: E402


def pack_latent_v4(
    motion_token: np.ndarray,
    frame_index: int,
    left_hand: np.ndarray | None,
    right_hand: np.ndarray | None,
    body_quat_w: np.ndarray | None = None,
) -> bytes:
    mt = np.asarray(motion_token, dtype=np.float32).reshape(1, -1)
    pose = {
        "token_state": mt,
        "frame_index": np.asarray([int(frame_index)], dtype=np.int64),
    }
    if left_hand is not None:
        pose["left_hand_joints"] = np.asarray(left_hand, dtype=np.float32).reshape(1, 7)
    if right_hand is not None:
        pose["right_hand_joints"] = np.asarray(right_hand, dtype=np.float32).reshape(1, 7)
    # Align deploy heading/anchor with v3 (deploy zeros cmd_z on streamed token motion).
    if body_quat_w is not None:
        pose["body_quat_w"] = np.asarray(body_quat_w, dtype=np.float32).reshape(1, 4)
    return pack_pose_message(pose, topic="pose", version=4)


def projected_gravity_wxyz(quat_wxyz: np.ndarray) -> np.ndarray:
    q = np.asarray(quat_wxyz, dtype=np.float64).reshape(4)
    w, x, y, z = q
    g = np.array(
        [
            2 * (x * z - w * y),
            2 * (y * z + w * x),
            1 - 2 * (x * x + y * y),
        ],
        dtype=np.float32,
    )
    return np.array([-g[0], -g[1], -g[2]], dtype=np.float32)


class G1DebugSub:
    def __init__(self, host: str, port: int):
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt_string(zmq.SUBSCRIBE, "g1_debug")
        self._sock.setsockopt(zmq.CONFLATE, 1)
        self._sock.setsockopt(zmq.RCVTIMEO, 0)
        self._sock.connect(f"tcp://{host}:{port}")
        self._msg = None
        self._topic = "g1_debug"

    def get(self):
        try:
            raw = self._sock.recv(zmq.NOBLOCK)
        except zmq.Again:
            return self._msg
        payload = raw[len(self._topic) :]
        msg = msgpack.unpackb(payload, raw=False)
        out = {}
        for k, v in msg.items():
            out[k] = np.asarray(v) if isinstance(v, (list, tuple)) else v
        self._msg = out
        return out


def body29_to_state_groups(body29: np.ndarray) -> dict[str, np.ndarray]:
    b = np.asarray(body29, dtype=np.float32).reshape(29)
    return {
        "left_leg": b[0:6],
        "right_leg": b[6:12],
        "waist": b[12:15],
        "left_arm": b[15:22],
        "right_arm": b[22:29],
    }


def hand7(msg: dict, key: str) -> np.ndarray:
    v = msg.get(key)
    if v is None:
        return np.zeros(7, dtype=np.float32)
    a = np.asarray(v, dtype=np.float32).reshape(-1)
    out = np.zeros(7, dtype=np.float32)
    out[: min(7, a.size)] = a[:7]
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model-path", required=True)
    ap.add_argument("--dataset", type=Path, required=True)
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--prompt", default="")
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--zmq-host", default="127.0.0.1")
    ap.add_argument("--zmq-port", type=int, default=5556)
    ap.add_argument("--state-host", default="127.0.0.1")
    ap.add_argument("--state-port", type=int, default=5557)
    ap.add_argument("--control-fps", type=float, default=50.0)
    ap.add_argument("--action-horizon", type=int, default=40)
    ap.add_argument("--execution-horizon", type=int, default=12)
    ap.add_argument("--motion-seconds", type=float, default=12.0)
    ap.add_argument("--arm-flag", type=Path, required=True)
    ap.add_argument("--stream-flag", type=Path, required=True)
    ap.add_argument("--go-flag", type=Path, required=True)
    ap.add_argument("--ready-flag", type=Path, required=True)
    ap.add_argument("--seed-proprio-frames", type=int, default=5)
    ap.add_argument(
        "--gt-bootstrap-frames",
        type=int,
        default=50,
        help="After GO, stream this many GT motion tokens before VLA outputs.",
    )
    ap.add_argument("--gt-proprio-only", action="store_true",
                    help="Always use dataset proprio (fully offline open-loop).")
    ap.add_argument(
        "--dump-tokens-npz",
        type=Path,
        default=None,
        help="After open-loop precompute, save plan tokens (+ GT) to this npz and continue "
        "(or exit if --dump-only).",
    )
    ap.add_argument(
        "--dump-only",
        action="store_true",
        help="With --gt-proprio-only: precompute + dump tokens, then exit (no ZMQ GO stream).",
    )
    ap.add_argument(
        "--closed-loop-sequential",
        action="store_true",
        help=(
            "In closed-loop mode, stream each inferred chunk 1:1 from index 0 "
            "(hard cut; no RTC inpainting / delay index). Debug only — default is RTC."
        ),
    )
    ap.add_argument(
        "--rtc",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Closed-loop / open-loop precompute: pass training RTC options into "
            "get_action so consecutive chunks inpaint the unexecuted tail "
            "(matches origin RTC train). Disable with --no-rtc."
        ),
    )
    ap.add_argument(
        "--rtc-overlap-steps",
        type=int,
        default=28,
        help="Deploy RTC overlap (origin train: 28 = H-s).",
    )
    ap.add_argument(
        "--rtc-frozen-steps",
        type=int,
        default=5,
        help="Deploy RTC frozen prefix (origin train: 5).",
    )
    ap.add_argument(
        "--rtc-ramp-rate",
        type=float,
        default=3.0,
        help="Deploy RTC ramp rate (origin train: 3.0).",
    )
    args = ap.parse_args()

    if "Phi_0" in str(args.model_path):
        raise SystemExit("ERROR: Phi_0 path not allowed for model-path")

    ep_pad = f"{int(args.episode):06d}"
    root = Path(args.dataset)
    parquet = root / "data/chunk-000" / f"episode_{ep_pad}.parquet"
    ego_mp4 = root / "videos/chunk-000/observation.images.ego_view" / f"episode_{ep_pad}.mp4"
    head_mp4 = root / "videos/chunk-000/observation.images.head" / f"episode_{ep_pad}.mp4"

    table = pq.read_table(parquet)
    state43 = np.stack(table.column("observation.state").to_pylist()).astype(np.float32)
    grav = np.stack(table.column("observation.projected_gravity").to_pylist()).astype(np.float32)
    gt_tokens = np.stack(table.column("action.motion_token").to_pylist()).astype(np.float32)
    if "teleop.left_hand_joints" in table.column_names:
        gt_lh = np.stack(table.column("teleop.left_hand_joints").to_pylist()).astype(np.float32)
        gt_rh = np.stack(table.column("teleop.right_hand_joints").to_pylist()).astype(np.float32)
    else:
        # fall back to state hand DOFs
        gt_lh = state43[:, 22:29].copy()
        gt_rh = state43[:, 36:43].copy()
    quat_col = next(
        (
            c
            for c in (
                "teleop.body_quat_w",
                "observation.init_base_quat",
                "observation.root_orientation",
            )
            if c in table.column_names
        ),
        None,
    )
    if quat_col is not None:
        gt_bq = np.stack(table.column(quat_col).to_pylist()).astype(np.float32)
        if gt_bq.ndim == 1:
            gt_bq = np.repeat(gt_bq.reshape(1, -1), gt_tokens.shape[0], axis=0)
        elif gt_bq.ndim > 2:
            gt_bq = gt_bq.reshape(gt_bq.shape[0], -1)[:, :4]
    else:
        gt_bq = np.tile(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32), (gt_tokens.shape[0], 1))
    print(
        f"[vla] GT bootstrap token0[:4]={gt_tokens[0, :4]} "
        f"gt_bootstrap_frames={int(args.gt_bootstrap_frames)} quat={quat_col or 'identity'}",
        flush=True,
    )
    tasks = {}
    with open(root / "meta/tasks.jsonl") as f:
        for line in f:
            row = json.loads(line)
            tasks[int(row["task_index"])] = row["task"]
    task_idx = int(np.asarray(table.column("task_index").to_pylist()).reshape(-1)[0])
    prompt = args.prompt or tasks.get(task_idx, "走到绿色桌子前")

    def _load_rgb_mp4(path: Path) -> list[np.ndarray]:
        cap = cv2.VideoCapture(str(path))
        if not cap.isOpened():
            raise RuntimeError(f"cannot open {path}")
        frames: list[np.ndarray] = []
        while True:
            ok, bgr = cap.read()
            if not ok:
                break
            frames.append(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        cap.release()
        return frames

    ego_frames = _load_rgb_mp4(ego_mp4)
    head_frames: list[np.ndarray] | None = None
    if head_mp4.is_file():
        head_frames = _load_rgb_mp4(head_mp4)
    n = len(ego_frames)
    if head_frames is not None and len(head_frames) != n:
        print(
            f"[vla] WARN head_frames={len(head_frames)} != ego_frames={n}; "
            "indexing with clamp",
            flush=True,
        )
    print(
        f"[vla] prompt={prompt!r} ego_frames={n} "
        f"head_frames={0 if head_frames is None else len(head_frames)} "
        f"state={state43.shape}",
        flush=True,
    )
    print(f"[vla] loading policy {args.model_path} on {args.device}", flush=True)

    policy = Gr00tPolicy(
        embodiment_tag=EmbodimentTag.UNITREE_G1_SONIC,
        model_path=args.model_path,
        device=args.device,
        strict=True,
    )
    modality = policy.get_modality_config()
    video_keys = list(getattr(modality.get("video", None), "modality_keys", []) or [])
    need_head = "head" in video_keys
    if need_head and head_frames is None:
        raise RuntimeError(
            f"policy modality requires video.head but missing {head_mp4}"
        )
    print(f"[vla] video modality_keys={video_keys} dual_cam={need_head}", flush=True)
    try:
        ah = policy.model.action_head
        if hasattr(ah, "num_inference_timesteps"):
            ah.num_inference_timesteps = 4
    except Exception as e:  # noqa: BLE001
        print(f"[vla] warn denoising: {e}", flush=True)

    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.bind(f"tcp://{args.zmq_host}:{args.zmq_port}")
    print(f"[vla] bound tcp://{args.zmq_host}:{args.zmq_port}", flush=True)
    args.ready_flag.parent.mkdir(parents=True, exist_ok=True)
    args.ready_flag.touch()
    time.sleep(0.5)

    if args.dump_only:
        if not args.gt_proprio_only:
            raise SystemExit("--dump-only requires --gt-proprio-only")
        if args.dump_tokens_npz is None:
            raise SystemExit("--dump-only requires --dump-tokens-npz")
        print("[vla] dump-only: skip arm/stream waits", flush=True)
    else:
        print(f"[vla] wait arm {args.arm_flag}", flush=True)
        while not args.arm_flag.exists():
            time.sleep(0.05)
        pub.send(build_command_message(start=True, stop=False, planner=True))
        print("[vla] sent start (planner)", flush=True)
        time.sleep(0.3)
        while not args.stream_flag.exists():
            time.sleep(0.05)
        pub.send(build_command_message(start=True, stop=False, planner=False))
        print("[vla] sent start (streamed)", flush=True)

    zero7 = np.zeros(7, dtype=np.float32)
    period = 1.0 / float(args.control_fps)
    horizon = int(args.action_horizon)
    exec_h = int(args.execution_horizon)
    max_steps = int(float(args.motion_seconds) * float(args.control_fps))
    bootstrap_n = max(0, int(args.gt_bootstrap_frames))
    use_rtc = bool(args.rtc) and not bool(args.closed_loop_sequential)
    rtc_overlap = int(args.rtc_overlap_steps)
    rtc_frozen = int(args.rtc_frozen_steps)
    rtc_ramp = float(args.rtc_ramp_rate)
    if use_rtc:
        if not (0 <= rtc_frozen < rtc_overlap <= horizon and rtc_ramp > 0):
            raise SystemExit(
                "RTC requires 0 <= rtc_frozen_steps < rtc_overlap_steps <= action_horizon "
                f"and rtc_ramp_rate > 0; got frozen={rtc_frozen} overlap={rtc_overlap} "
                f"horizon={horizon} ramp={rtc_ramp}"
            )
        print(
            f"[vla] RTC ON H={horizon} overlap={rtc_overlap} frozen={rtc_frozen} "
            f"ramp={rtc_ramp} exec_h={exec_h}",
            flush=True,
        )
    else:
        print(
            f"[vla] RTC OFF "
            f"(rtc={bool(args.rtc)} sequential={bool(args.closed_loop_sequential)})",
            flush=True,
        )

    def rtc_options_dict() -> dict:
        return {
            "action_horizon": horizon,
            "rtc_overlap_steps": rtc_overlap,
            "rtc_frozen_steps": rtc_frozen,
            "rtc_ramp_rate": rtc_ramp,
        }

    def build_obs(fi: int, msg: dict | None):
        ego = ego_frames[min(fi, n - 1)]
        use_gt = args.gt_proprio_only or msg is None or fi < int(args.seed_proprio_frames)
        if use_gt:
            st = state43[min(fi, state43.shape[0] - 1)]
            g = grav[min(fi, grav.shape[0] - 1)]
            lh = st[22:29]
            rh = st[36:43]
            groups = {
                "left_leg": st[0:6],
                "right_leg": st[6:12],
                "waist": st[12:15],
                "left_arm": st[15:22],
                "right_arm": st[29:36],
            }
        else:
            assert msg is not None
            body = np.asarray(msg.get("body_q", np.zeros(29)), dtype=np.float32).reshape(-1)
            if body.size < 29:
                body = np.pad(body, (0, 29 - body.size))
            groups = body29_to_state_groups(body[:29])
            lh = hand7(msg, "left_hand_q")
            rh = hand7(msg, "right_hand_q")
            quat = np.asarray(msg.get("base_quat", [1, 0, 0, 0]), dtype=np.float32).reshape(4)
            g = projected_gravity_wxyz(quat)

        flat = {
            "video.ego_view": np.asarray(ego, dtype=np.uint8)[None, ...],
            "state.left_leg": groups["left_leg"][None, :],
            "state.right_leg": groups["right_leg"][None, :],
            "state.waist": groups["waist"][None, :],
            "state.left_arm": groups["left_arm"][None, :],
            "state.left_hand": np.asarray(lh, dtype=np.float32)[None, :],
            "state.right_arm": groups["right_arm"][None, :],
            "state.right_hand": np.asarray(rh, dtype=np.float32)[None, :],
            "state.projected_gravity": np.asarray(g, dtype=np.float32)[None, :],
            "annotation.human.task_description": prompt,
        }
        if need_head:
            assert head_frames is not None
            head = head_frames[min(fi, len(head_frames) - 1)]
            flat["video.head"] = np.asarray(head, dtype=np.uint8)[None, ...]
        return parse_observation_gr00t(flat, modality)

    def infer_at(
        fi: int,
        msg: dict | None = None,
        options: dict | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
        obs = build_obs(fi, msg)
        t_inf0 = time.monotonic()
        action, info = policy.get_action(obs, options)
        mt = np.asarray(action.get("motion_token", action.get("action.motion_token")), dtype=np.float32)
        lh_a = np.asarray(
            action.get("left_hand_joints", action.get("action.left_hand_joints", zero7)),
            dtype=np.float32,
        )
        rh_a = np.asarray(
            action.get("right_hand_joints", action.get("action.right_hand_joints", zero7)),
            dtype=np.float32,
        )
        if mt.ndim == 3:
            mt = mt[0]
        if lh_a.ndim == 3:
            lh_a = lh_a[0]
        if rh_a.ndim == 3:
            rh_a = rh_a[0]
        rtc_flag = bool((info or {}).get("rtc", False))
        print(
            f"[vla] infer frame={fi} mt={mt.shape} "
            f"t0={float(mt[0, 0]) if mt.ndim == 2 else float(mt.reshape(-1)[0]):+.4f} "
            f"delay={time.monotonic() - t_inf0:.2f}s rtc={rtc_flag}",
            flush=True,
        )
        return mt, lh_a, rh_a, info or {}

    # -------------------------------------------------------------------------
    # Open-loop (--gt-proprio-only): precompute BEFORE GO (while holding GT[0]),
    # then on GO stream GT bootstrap + planned tokens 1:1 with no wall-clock RTC
    # delay indexing (that caused sudden arm-bend via stale/jumped chunks).
    # Hands: dataset GT hands (body motion comes from motion_token).
    # -------------------------------------------------------------------------
    if args.gt_proprio_only:
        stop_hold = threading.Event()

        def _hold_gt0() -> None:
            k = 0
            while not stop_hold.is_set():
                pub.send(pack_latent_v4(gt_tokens[0], k, gt_lh[0], gt_rh[0], gt_bq[0]))
                k += 1
                time.sleep(period)

        hold_th = threading.Thread(target=_hold_gt0, daemon=True)
        hold_th.start()
        print(
            f"[vla] open-loop precompute BEFORE GO for steps "
            f"[{bootstrap_n}, {max_steps}) exec_h={exec_h} (holding GT[0])",
            flush=True,
        )

        plan_mt = np.zeros((max_steps, 64), dtype=np.float32)
        plan_lh = np.zeros((max_steps, 7), dtype=np.float32)
        plan_rh = np.zeros((max_steps, 7), dtype=np.float32)
        for i in range(max_steps):
            gi = min(i, gt_tokens.shape[0] - 1)
            plan_mt[i] = gt_tokens[gi]
            plan_lh[i] = gt_lh[gi]
            plan_rh[i] = gt_rh[gi]

        prev_chunk: np.ndarray | None = None
        for fi in range(bootstrap_n, max_steps, exec_h):
            opts = rtc_options_dict() if (use_rtc and prev_chunk is not None) else None
            mt, _lh_a, _rh_a, _info = infer_at(min(fi, n - 1), None, options=opts)
            # With RTC, new chunk index 0 continues previous[exec_h] after inpaint.
            take = min(exec_h, mt.shape[0], max_steps - fi)
            plan_mt[fi : fi + take] = mt[:take]
            prev_chunk = mt
            for j in range(take):
                gj = min(fi + j, gt_lh.shape[0] - 1)
                plan_lh[fi + j] = gt_lh[gj]
                plan_rh[fi + j] = gt_rh[gj]
            if fi == bootstrap_n:
                d0 = float(np.linalg.norm(plan_mt[fi] - gt_tokens[min(fi, gt_tokens.shape[0] - 1)]))
                print(
                    f"[vla] switch discontinuity L2(token_vla0, token_gt)={d0:.4f} "
                    f"vla0[:4]={plan_mt[fi, :4].tolist()} "
                    f"gt[:4]={gt_tokens[min(fi, gt_tokens.shape[0] - 1), :4].tolist()}",
                    flush=True,
                )

        if args.dump_tokens_npz is not None:
            out_npz = Path(args.dump_tokens_npz)
            out_npz.parent.mkdir(parents=True, exist_ok=True)
            gt_aligned = np.stack(
                [gt_tokens[min(i, gt_tokens.shape[0] - 1)] for i in range(max_steps)],
                axis=0,
            ).astype(np.float32)
            per = np.linalg.norm(plan_mt - gt_aligned, axis=1)
            np.savez_compressed(
                out_npz,
                tokens=plan_mt,
                gt_tokens=gt_aligned,
                bootstrap_n=np.int32(bootstrap_n),
                exec_h=np.int32(exec_h),
                action_horizon=np.int32(horizon),
                l2_per_step=per.astype(np.float32),
                l2_mean=np.float32(per.mean()),
                l2_post_boot=np.float32(per[bootstrap_n:].mean() if bootstrap_n < max_steps else 0.0),
            )
            print(
                f"[vla] dumped tokens → {out_npz} shape={plan_mt.shape} "
                f"L2_mean={float(per.mean()):.4f} L2_post_boot={float(per[bootstrap_n:].mean()):.4f}",
                flush=True,
            )
            if args.dump_only:
                stop_hold.set()
                hold_th.join(timeout=2.0)
                print("[vla] dump-only done", flush=True)
                return

        print(
            f"[vla] precompute done — wait go {args.go_flag} "
            f"(still holding GT token0 t0={float(gt_tokens[0, 0]):+.4f})",
            flush=True,
        )
        while not args.go_flag.exists():
            time.sleep(0.05)
        stop_hold.set()
        hold_th.join(timeout=2.0)
        print(
            f"[vla] go — GT bootstrap {bootstrap_n} frames, then open-loop VLA 1:1",
            flush=True,
        )

        t_next = time.monotonic()
        for step in range(max_steps):
            if step < bootstrap_n:
                gi = min(step, gt_tokens.shape[0] - 1)
                pub.send(pack_latent_v4(gt_tokens[gi], step, gt_lh[gi], gt_rh[gi], gt_bq[min(gi, gt_bq.shape[0]-1)]))
                if step == 0 or (step + 1) == bootstrap_n:
                    print(
                        f"[vla] GT bootstrap step={step}/{bootstrap_n} "
                        f"t0={float(gt_tokens[gi, 0]):+.4f}",
                        flush=True,
                    )
            else:
                pub.send(pack_latent_v4(plan_mt[step], step, plan_lh[step], plan_rh[step], gt_bq[min(step, gt_bq.shape[0]-1)]))
                if step == bootstrap_n or step % 25 == 0:
                    print(
                        f"[vla] stream step={step} t0={float(plan_mt[step, 0]):+.4f}",
                        flush=True,
                    )
            t_next += period
            sleep_s = t_next - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                t_next = time.monotonic()
        print("[vla] done", flush=True)
        return

    # -------------------------------------------------------------------------
    # Closed-loop / realtime path (sim proprio): deploy-matched RTC scheduling.
    # -------------------------------------------------------------------------
    # Hold GT frame-0 until GO — matches dataset standing pose.
    fi_hold = 0
    print(
        f"[vla] wait go {args.go_flag} (holding GT token0 t0={float(gt_tokens[0, 0]):+.4f})",
        flush=True,
    )
    while not args.go_flag.exists():
        pub.send(pack_latent_v4(gt_tokens[0], fi_hold, gt_lh[0], gt_rh[0], gt_bq[0]))
        fi_hold += 1
        time.sleep(period)
    print(
        f"[vla] go — GT bootstrap {bootstrap_n} frames, then VLA (closed-loop"
        f"{', RTC' if use_rtc else ', hard-cut'})",
        flush=True,
    )

    state_sub = G1DebugSub(args.state_host, args.state_port)
    chunk_lock = threading.Lock()
    cached: dict = {
        "motion": None,
        "lh": None,
        "rh": None,
        "t_infer_start": 0.0,
        "rtc_applied": False,
        "new_chunk": False,
        "pending_options": None,
    }
    busy = threading.Event()
    stop = threading.Event()
    frame_i = {"i": 0}
    chunk_idx = {"i": 0}

    def infer_once() -> None:
        fi = frame_i["i"]
        with chunk_lock:
            opts = cached["pending_options"]
        t_start = time.monotonic()
        mt, lh_a, rh_a, info = infer_at(fi, state_sub.get(), options=opts)
        with chunk_lock:
            cached["motion"] = mt
            cached["lh"] = lh_a
            cached["rh"] = rh_a
            cached["t_infer_start"] = t_start
            cached["rtc_applied"] = bool(info.get("rtc", False))
            cached["new_chunk"] = True

    def worker() -> None:
        while not stop.is_set():
            if busy.is_set():
                try:
                    infer_once()
                except Exception as e:  # noqa: BLE001
                    print(f"[vla] infer error: {e}", flush=True)
                finally:
                    busy.clear()
            else:
                time.sleep(0.01)

    th = threading.Thread(target=worker, daemon=True)
    th.start()

    t_next = time.monotonic()
    last_infer = -1e9
    vla_started = False

    for step in range(max_steps):
        frame_i["i"] = min(step, n - 1)

        if step < bootstrap_n:
            gi = min(step, gt_tokens.shape[0] - 1)
            pub.send(
                pack_latent_v4(
                    gt_tokens[gi],
                    step,
                    gt_lh[gi],
                    gt_rh[gi],
                    gt_bq[min(gi, gt_bq.shape[0] - 1)],
                )
            )
            if step == 0 or (step + 1) == bootstrap_n or step % 25 == 0:
                print(
                    f"[vla] GT bootstrap step={step}/{bootstrap_n} "
                    f"t0={float(gt_tokens[gi, 0]):+.4f}",
                    flush=True,
                )
        else:
            if not vla_started:
                vla_started = True
                print(f"[vla] switch to model inference at step={step}", flush=True)

            # Apply newly finished chunk (same order as run_vla_inference.py).
            with chunk_lock:
                if cached["new_chunk"]:
                    delay = time.monotonic() - float(cached["t_infer_start"])
                    if use_rtc and cached["rtc_applied"]:
                        chunk_idx["i"] = rtc_execution_index(
                            chunk_idx["i"], horizon, rtc_overlap
                        )
                        print(
                            f"[vla] new chunk RTC start={chunk_idx['i']} "
                            f"latency={delay:.3f}s",
                            flush=True,
                        )
                    elif args.closed_loop_sequential:
                        chunk_idx["i"] = 0
                        print(
                            f"[vla] new chunk sequential start=0 latency={delay:.3f}s",
                            flush=True,
                        )
                    else:
                        chunk_idx["i"] = calculate_latency_compensated_index(
                            delay, float(args.control_fps), horizon
                        )
                        print(
                            f"[vla] new chunk delay-index start={chunk_idx['i']} "
                            f"latency={delay:.3f}s",
                            flush=True,
                        )
                    cached["new_chunk"] = False

            worker_busy = busy.is_set()
            with chunk_lock:
                has_chunk = cached["motion"] is not None
                cur_for_trigger = int(chunk_idx["i"])

            if use_rtc:
                should_start = should_trigger_rtc_inference(
                    cached_chunk_exists=has_chunk,
                    inference_thread_running=worker_busy,
                    action_chunk_index=cur_for_trigger,
                    action_horizon=horizon,
                    overlap_steps=rtc_overlap,
                )
            else:
                should_start = (not has_chunk or (time.monotonic() - last_infer) >= (
                    exec_h / float(args.control_fps)
                )) and not worker_busy

            if should_start and not worker_busy:
                with chunk_lock:
                    if use_rtc and has_chunk:
                        cached["pending_options"] = rtc_options_dict()
                    else:
                        cached["pending_options"] = None
                busy.set()
                last_infer = time.monotonic()

            with chunk_lock:
                mt = cached["motion"]
                lh = cached["lh"]
                rh = cached["rh"]

            if mt is None:
                gi = min(step, gt_tokens.shape[0] - 1)
                pub.send(
                    pack_latent_v4(
                        gt_tokens[gi],
                        step,
                        gt_lh[gi],
                        gt_rh[gi],
                        gt_bq[min(gi, gt_bq.shape[0] - 1)],
                    )
                )
            else:
                cur = min(int(chunk_idx["i"]), mt.shape[0] - 1)
                lh_row = lh[cur] if lh is not None and lh.ndim == 2 else zero7
                rh_row = rh[cur] if rh is not None and rh.ndim == 2 else zero7
                if np.asarray(lh_row).reshape(-1).shape[0] != 7:
                    lh_row = zero7
                if np.asarray(rh_row).reshape(-1).shape[0] != 7:
                    rh_row = zero7
                pub.send(
                    pack_latent_v4(
                        mt[cur],
                        step,
                        lh_row,
                        rh_row,
                        gt_bq[min(step, gt_bq.shape[0] - 1)],
                    )
                )
                chunk_idx["i"] = min(int(chunk_idx["i"]) + 1, horizon - 1)

        t_next += period
        sleep_s = t_next - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            t_next = time.monotonic()

    stop.set()
    print("[vla] done", flush=True)


if __name__ == "__main__":
    main()

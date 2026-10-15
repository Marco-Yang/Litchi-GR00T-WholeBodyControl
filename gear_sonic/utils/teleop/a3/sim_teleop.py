"""Live teleop driver for ``sim2sim_a3_mujoco.py`` (MuJoCo closed-loop, real-time).

Mirrors the deploy's TELEOP mode (``a3_deploy/main.cpp`` + ``a3_teleop_reference.cpp``):

* joint-space frames (``a3_ref`` ZMQ stream from ``a3_teleop_retarget_node.py``, or any
  producer of the ``/ta/whole_body_command`` content) are stamped on arrival and pushed
  into a delayed-window buffer;
* each 50 Hz policy tick the buffer builds the 640-float tokenizer prefix of the
  ``g1`` / ``a3_fast`` encoder (window = now - delay .. +10 slots);
* before data is available the policy is fed the default standing prefix and its
  command is blended toward the nominal stand (``ApplyStandFallbackCommandFilter``);
* a stale stream holds the last command; the reference yaw is aligned to the robot at
  the start of each stream segment (``teleop.yaw_offset_compensation``).

The simulator is paced to wall clock, because the buffer works on wall-clock arrival
stamps exactly like on the robot.
"""

from __future__ import annotations

import json
from pathlib import Path
import threading
import time

import numpy as np

from gear_sonic.utils.teleop.a3.frames import (
    MJ29_JOINT_NAMES,
    unpack_frame,
)
from gear_sonic.utils.teleop.a3.reference_buffer import (
    NUM_FUTURE_FRAMES,
    A3TeleopReferenceBuffer,
    A3TeleopReferenceOptions,
    TokenizerStatus,
    build_default_stand_tokenizer_slice,
)

# teleop block of a3_runtime_config.yaml
DEFAULT_DELAY_MS = {"g1": 940.0, "a3_fast": 220.0}
FALLBACK_POLICY_BLEND = 0.25
FALLBACK_MAX_DELTA_RAD = 0.08
FELL_PELVIS_Z = 0.45


class TeleopZmqSource:
    """Background SUB socket that stamps frames on arrival and feeds the buffer."""

    def __init__(self, url: str, buffer: A3TeleopReferenceBuffer, topic: bytes = b"a3_ref") -> None:
        import zmq

        self._zmq = zmq
        self._buffer = buffer
        self._topic = topic
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.SUBSCRIBE, topic)
        self._sock.setsockopt(zmq.RCVHWM, 64)
        self._sock.connect(url)
        self._stop = threading.Event()
        self.received = 0
        self.rejected = 0
        self._thread = threading.Thread(target=self._run, name="a3-teleop-zmq", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        poller = self._zmq.Poller()
        poller.register(self._sock, self._zmq.POLLIN)
        while not self._stop.is_set():
            if not poller.poll(50):
                continue
            try:
                frame = unpack_frame(self._sock.recv(), self._topic)
            except ValueError:
                self.rejected += 1
                continue
            frame.stamp_ns = time.time_ns()  # same as the AimRT callback: re-stamp on arrival
            self._buffer.push_frame(frame)
            self.received += 1

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._sock.close(0)
        self._ctx.term()


def _tokenizer_to_encoder_input(slice640: np.ndarray) -> np.ndarray:
    """640-float prefix -> (10, 64) encoder input, same layout as build_encoder_input()."""
    cmd = slice640[:580].reshape(NUM_FUTURE_FRAMES, 58)
    ori = slice640[580:].reshape(NUM_FUTURE_FRAMES, 6)
    return np.concatenate([cmd, ori], axis=-1).astype(np.float32)


def _make_runner_class(s2s):
    class TeleopSimRunner(s2s.LoopSimRunner):
        def __init__(self, *args, buffer, source, duration_s, stale_ms, report_path, use_viewer, **kwargs):
            super().__init__(*args, **kwargs)
            self.buffer = buffer
            self.source = source
            self.duration_s = float(duration_s)
            self.stale_ns = int(stale_ms * 1e6)
            self.report_path = report_path
            self.use_viewer = bool(use_viewer)
            mapping = self.runtime.mapping
            # canonical (C++/TA) mj29 order -> this model's mj29 order, by name
            name_to_canon = {n: i for i, n in enumerate(MJ29_JOINT_NAMES)}
            self._model_from_canon = np.array([name_to_canon[n] for n in mapping.mj29_joint_names], dtype=np.int64)
            self._default_q_il = mapping.default_q_il.copy()

        # -- fallback filter (ApplyStandFallbackCommandFilter) -------------------
        def _fallback_filter(self, target_il: np.ndarray) -> np.ndarray:
            delta = (target_il - self._default_q_il) * FALLBACK_POLICY_BLEND
            return self._default_q_il + np.clip(delta, -FALLBACK_MAX_DELTA_RAD, FALLBACK_MAX_DELTA_RAD)

        def _robot_q_canon(self) -> np.ndarray:
            q_model = s2s.get_loop_joint_q_mj29(self.data, self.runtime, self.solver)
            q = np.empty(29)
            q[self._model_from_canon] = q_model
            return q

        def _run_policy_steps(self) -> None:  # noqa: C901 - one linear control loop
            stand_slice = build_default_stand_tokenizer_slice()
            hold_slice = None
            yaw_offset = 0.0
            yaw_valid = False
            stream_was_stale = False
            running_started = False
            t_wall = time.perf_counter()
            last_log = t_wall
            sim_dt = s2s.POLICY_DT
            stats = {
                "steps": 0, "running_steps": 0, "hold_steps": 0, "fallback_steps": 0,
                "fell": False, "fell_at_s": None, "min_pelvis_z": 1e9, "max_tilt_deg": 0.0,
            }
            track_err: list[float] = []
            anchor_id = self.runtime.mapping.anchor_body_id
            print("[teleop] waiting for a3_ref frames (policy runs on the standing fallback until then)", flush=True)
            if self.use_viewer:
                self._sync_viewer_data(None)
                self._launch_viewer()

            while True:
                if self.duration_s > 0.0 and self.data.time >= self.duration_s:
                    break
                now_ns = time.time_ns()
                anchor_quat = s2s.body_quat_wxyz(self.data, anchor_id)
                latest_ns = self.buffer.latest_stamp_ns()
                has_latest = latest_ns > 0
                stale = has_latest and (now_ns - latest_ns) > self.stale_ns

                if stale:
                    stream_was_stale = True
                elif has_latest and (not yaw_valid or stream_was_stale):
                    # start of a (new) stream segment: align reference yaw to the robot and
                    # restart the window from the freshest frame, like the deploy does.
                    yaw_offset = self.buffer.compute_latest_yaw_offset_rad(anchor_quat)
                    self.buffer.reset_to_latest_frame()
                    yaw_valid = True
                    stream_was_stale = False
                    running_started = False
                    self.history = s2s.prime_history(
                        s2s.build_loop_current_obs_terms(
                            self.model, self.data, self.runtime, self.solver, self.last_action_il
                        )
                    )
                    print(f"[teleop] stream segment start: yaw_offset={np.degrees(yaw_offset):.1f} deg", flush=True)

                slice640 = None
                status = TokenizerStatus.NO_DATA
                if yaw_valid and not stale:
                    slice640, status = self.buffer.build_tokenizer_slice(now_ns, anchor_quat, yaw_offset)

                use_filter = None
                if slice640 is not None:
                    hold_slice = slice640
                    if not running_started:
                        running_started = True
                    stats["running_steps"] += 1
                elif hold_slice is not None and running_started:
                    slice640 = hold_slice  # stale / buffering: hold last command
                    stats["hold_steps"] += 1
                else:
                    slice640 = stand_slice
                    use_filter = self._fallback_filter
                    stats["fallback_steps"] += 1

                result = self._step_policy_with_encoder_input(
                    _tokenizer_to_encoder_input(slice640), target_il_filter=use_filter
                )
                del result
                stats["steps"] += 1

                # --- diagnostics ------------------------------------------------
                pelvis_z = float(self.data.xpos[anchor_id][2])
                stats["min_pelvis_z"] = min(stats["min_pelvis_z"], pelvis_z)
                gravity_b = s2s.quat_rotate_inverse_wxyz(anchor_quat, np.array([0.0, 0.0, -1.0]))
                tilt = float(np.degrees(np.arccos(np.clip(-gravity_b[2], -1.0, 1.0))))
                stats["max_tilt_deg"] = max(stats["max_tilt_deg"], tilt)
                if status == TokenizerStatus.RUNNING:
                    ref = self.buffer.sample_delayed_frame(now_ns)
                    if ref is not None:
                        track_err.append(float(np.sqrt(np.mean((self._robot_q_canon() - ref.q_mj29) ** 2))))
                if pelvis_z < FELL_PELVIS_Z and not stats["fell"]:
                    stats["fell"] = True
                    stats["fell_at_s"] = float(self.data.time)
                    print(f"[teleop] FELL at sim t={self.data.time:.2f}s (pelvis z={pelvis_z:.2f})", flush=True)
                    break

                if self.use_viewer and not self._sync_viewer(None):
                    break
                t_wall += sim_dt
                sleep = t_wall - time.perf_counter()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    t_wall = time.perf_counter()
                if time.perf_counter() - last_log >= 2.0:
                    last_log = time.perf_counter()
                    err_txt = f"{np.mean(track_err[-100:]):.3f}" if track_err else "n/a"
                    print(
                        f"[teleop] t={self.data.time:6.1f}s status={status.name:<9} frames={self.buffer.frame_count():3d} "
                        f"rx={self.source.received} pelvis_z={pelvis_z:.2f} tilt={tilt:4.1f}deg track_rmse={err_txt}",
                        flush=True,
                    )

            report = {
                **stats,
                "sim_seconds": float(self.data.time),
                "frames_received": int(self.source.received),
                "tracking_rmse_mean_rad": float(np.mean(track_err)) if track_err else None,
                "tracking_rmse_p95_rad": float(np.percentile(track_err, 95)) if track_err else None,
                "tracking_rmse_max_rad": float(np.max(track_err)) if track_err else None,
                "encoder": self.config.encoder,
                "delay_ms": self.buffer.delay_ns / 1e6,
            }
            print("[teleop] summary: " + json.dumps(report), flush=True)
            if self.report_path is not None:
                Path(self.report_path).parent.mkdir(parents=True, exist_ok=True)
                Path(self.report_path).write_text(json.dumps(report, indent=2))

    return TeleopSimRunner


def run_teleop_sim(s2s, model, data, runtime, motion_paths, initial_reference, solver, policy, controls,
                   device, config, args) -> None:
    encoder_key = "a3_fast" if config.encoder == "a3_fast" else "g1"
    delay_ms = args.teleop_delay_ms if args.teleop_delay_ms is not None else DEFAULT_DELAY_MS[encoder_key]
    options = A3TeleopReferenceOptions(
        delay_ns=int(delay_ms * 1e6),
        policy_hz=1.0 / s2s.POLICY_DT,
        future_frame_skip=int(config.future_frame_skip),
        max_frames=512,
    )
    buffer = A3TeleopReferenceBuffer(options)
    source = TeleopZmqSource(args.teleop_zmq, buffer)
    print(
        f"[teleop] encoder={config.encoder} skip={options.future_frame_skip} delay={delay_ms:.0f}ms "
        f"source={args.teleop_zmq}",
        flush=True,
    )
    runner_cls = _make_runner_class(s2s)
    runner = runner_cls(
        model, data, runtime, motion_paths, None, initial_reference, solver, policy, controls, device, config,
        None,
        buffer=buffer, source=source, duration_s=args.teleop_seconds, stale_ms=args.teleop_stale_ms,
        report_path=args.teleop_report, use_viewer=args.teleop_viewer,
    )
    source.start()
    try:
        runner._reset_to_current_reference()
        runner._run_policy_steps()
    finally:
        runner._close()
        source.close()

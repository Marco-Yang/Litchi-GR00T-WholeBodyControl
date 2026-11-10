"""
Sonic VLA data exporter for G1 -- NO ROS 2 DEPENDENCY.

All data sources use ZMQ:
  1. Robot state  -> ZMQ SUB on ``g1_debug`` topic (port 5557, from C++ zmq_output_handler)
  2. SMPL pose    -> ZMQ SUB on ``pose`` topic     (port 5556, from pico_manager_thread_server)
  3. Camera       -> ZMQ/TCP via ComposedCameraClientSensor

Robot config (``script_config`` in info.json) is read from the ``robot_config``
ZMQ topic re-published every ~2 s by the C++ process.  If the config is not
received within the timeout the exporter exits with an error.

Virtual environment setup (run from repo root):
    bash install_scripts/install_data_collection.sh
    source .venv_data_collection/bin/activate

Usage (from repo root):
    python gear_sonic/scripts/run_data_exporter.py --task-prompt "pick up the cup"
    python gear_sonic/scripts/run_data_exporter.py --task-prompt "walk forward" --dataset-name my_session
"""

from collections import deque
from dataclasses import dataclass
from datetime import datetime
import json
import os
import time
from typing import Optional

import msgpack
import numpy as np
from scipy.spatial.transform import Rotation as R
import tyro
import zmq

from gear_sonic.data.exporter import Gr00tDataExporter
from gear_sonic.data.features_sonic_vla import (
    get_features_sonic_vla,
    get_g1_robot_model,
    get_head_camera_features,
    get_head_camera_modality_config,
    get_modality_config_sonic_vla,
    get_wrist_camera_features,
    get_wrist_camera_modality_config,
)
from gear_sonic.camera.composed_camera import ComposedCameraClientSensor
from gear_sonic.utils.data_collection.episode_state import EpisodeState
from gear_sonic.utils.data_collection.keyboard_subscriber import ZMQKeyboardSubscriber
from gear_sonic.utils.data_collection.revo2_teleop_hands import Revo2TeleopHands
from gear_sonic.utils.data_collection.telemetry import Telemetry
from gear_sonic.utils.data_collection.text_to_speech import TextToSpeech
from gear_sonic.utils.data_collection.transforms import compute_projected_gravity, quat_to_rot6d
from gear_sonic.utils.data_collection.zmq_state_subscriber import (
    ZMQStateSubscriber,
    poll_robot_config_zmq,
)

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------


@dataclass
class SonicDataExporterConfig:
    """CLI config for the ROS-free Sonic data exporter."""

    # Dataset
    dataset_name: str | None = None
    """Dataset name (auto-generated if creating new)."""

    task_prompt: str = "demo"
    """Language task prompt."""

    root_output_dir: str = "outputs"
    """Root output directory."""

    data_collection_frequency: int = 50
    """Data collection frequency (Hz)."""


    # Camera
    camera_host: str = "localhost"
    """Camera server host."""

    camera_port: int = 5555
    """Camera server port."""

    # ZMQ: Sonic / SMPL pose (from pico_manager_thread_server)
    sonic_zmq_host: str = "localhost"
    """ZMQ host for Sonic SMPL pose messages."""

    sonic_zmq_port: int = 5556
    """ZMQ port for Sonic SMPL pose messages."""

    # ZMQ: Robot state (from C++ zmq_output_handler, g1_debug topic)
    state_zmq_host: str = "localhost"
    """ZMQ host for robot state (g1_debug topic from C++ deploy)."""

    state_zmq_port: int = 5557
    """ZMQ port for robot state (same socket as robot_config topic)."""

    # Robot config
    robot_config_timeout: float = 0
    """Seconds to wait for the ZMQ robot_config message at startup (0 = wait forever)."""

    record_wrist_cameras: Optional[bool] = None
    """Wrist video: ``None``/omit = auto-detect :5558; ``True`` force on; ``False`` force off
    (``--no-record-wrist-cameras``)."""

    wrist_camera_port: int = 5558
    """ZMQ port for left_wrist + right_wrist (Orin Abham server)."""

    record_head_camera: Optional[bool] = None
    """Head video: ``None``/omit = auto-detect ``head`` on :5555; ``True`` force on;
    ``False`` force off (``--no-record-head-camera``)."""

    camera_probe_timeout_s: float = 2.0
    """Seconds to wait for a camera ZMQ frame when auto-detecting streams."""

    text_to_speech: bool = True
    """Use text-to-speech voice feedback."""

    # Hand embodiment: dex3 (default 7D) or revo2 (6D from recorded grasp JSON)
    hand_type: str = "dex3"
    """Hand action labels: ``dex3`` (7D) or ``revo2`` (6D Grasp traj + Pico trigger)."""

    revo2_pose: str = (
        "../Manus/config/revo2_grasp_pose.json"
    )
    """Path to Revo2 grasp JSON (relative to Litchi repo root or absolute)."""

    revo2_orin_host: str = "192.168.123.164"
    """Orin host for Revo2 UDP Modbus bridge during recording."""

    revo2_udp_port: int = 19500
    """UDP port of Orin revo2_modbus_rx."""

    revo2_send: bool = True
    """If True with hand_type=revo2, also stream cmds to real hands via UDP."""

    revo2_release_blend_s: float = 0.35
    """Seconds to blend back to open after trigger release."""

    # Supervised motion_token rewrite (Direction-1 dual-track).
    # Default OFF = Origin / live route: disk keeps deploy ``token_state`` (matches
    # successful VLA→decoder closed-loop training). Opt in to open-loop mode-2
    # rewrite with ``--rewrite-motion-token-supervised``.
    rewrite_motion_token_supervised: bool = False
    """If True, before parquet save rewrite ``action.motion_token`` with open-loop
    SONIC v1.1 mode-2 encode from buffered ``teleop.smpl_*`` (same rules as §8
    refill); live ``token_state`` is backed up under ``meta/motion_token_live/``.
    Default False: disk keeps raw live tokens (Origin route). Enable with
    ``--rewrite-motion-token-supervised``."""

    supervised_token_device: str = "cpu"
    """ORT device for episode-end supervised encode: ``cpu`` / ``cuda`` / ``auto``."""


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def probe_zmq_image_keys(host: str, port: int, timeout_s: float = 2.0) -> set[str]:
    """Return image keys from one msgpack camera PUB frame, or empty if unavailable.

    Does not JPEG-decode; only inspects ``images`` dict keys (Orin dual/wrist servers).
    """
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVTIMEO, 300)
    sock.setsockopt(zmq.RCVHWM, 2)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    sock.connect(f"tcp://{host}:{int(port)}")
    # PUB/SUB slow-joiner
    time.sleep(0.35)
    keys: set[str] = set()
    deadline = time.monotonic() + max(0.2, float(timeout_s))
    while time.monotonic() < deadline:
        try:
            packed = sock.recv()
        except zmq.Again:
            continue
        try:
            data = msgpack.unpackb(packed, raw=False)
        except Exception:
            continue
        if not isinstance(data, dict):
            continue
        images = data.get("images")
        if isinstance(images, dict) and images:
            keys.update(str(k) for k in images.keys())
            break
    sock.close()
    ctx.term()
    return keys


def resolve_camera_recording_flags(
    config: "SonicDataExporterConfig",
) -> tuple[bool, bool]:
    """Resolve head/wrist recording with optional auto-detect against live ZMQ streams.

    Returns ``(record_wrist, record_head)``.
    """
    timeout = float(config.camera_probe_timeout_s)
    auto_head = config.record_head_camera is None
    auto_wrist = config.record_wrist_cameras is None

    ego_keys: set[str] = set()
    wrist_keys: set[str] = set()
    if auto_head or auto_wrist:
        print(
            f"[Camera] probing streams on {config.camera_host}:"
            f"{config.camera_port} / :{config.wrist_camera_port} "
            f"(timeout={timeout:.1f}s) ..."
        )
    if auto_head:
        ego_keys = probe_zmq_image_keys(config.camera_host, config.camera_port, timeout)
        print(f"[Camera] :{config.camera_port} keys={sorted(ego_keys) or '—'}")
        if "ego_view" not in ego_keys:
            print(
                f"[Camera] WARNING: no ego_view on :{config.camera_port} — "
                "start Orin dual RealSense before recording"
            )
    if auto_wrist:
        wrist_keys = probe_zmq_image_keys(
            config.camera_host, config.wrist_camera_port, timeout
        )
        print(f"[Camera] :{config.wrist_camera_port} keys={sorted(wrist_keys) or '—'}")

    if auto_wrist:
        record_wrist = {"left_wrist", "right_wrist"}.issubset(wrist_keys)
    else:
        record_wrist = bool(config.record_wrist_cameras)

    if auto_head:
        record_head = "head" in ego_keys
    else:
        record_head = bool(config.record_head_camera)

    mode = []
    mode.append("ego_view")
    if record_head:
        mode.append("head")
    if record_wrist:
        mode.append("left_wrist+right_wrist")
    print(f"[Camera] recording → {', '.join(mode)}")
    return record_wrist, record_head


class TimeDeltaException(Exception):
    def __init__(self, failure_count: int, reset_timeout_sec: float):
        self.failure_count = failure_count
        self.reset_timeout_sec = reset_timeout_sec
        self.message = f"{self.failure_count} failures in {self.reset_timeout_sec} seconds"
        super().__init__(self.message)


def unpack_pose_message(packed_data: bytes, topic: str = "pose") -> dict:
    """Unpack a single-frame packed message from pico_manager_thread_server.

    Wire format: [topic_prefix][1280-byte JSON header][concatenated binary fields]
    """
    HEADER_SIZE = 1280

    topic_bytes = topic.encode("utf-8")
    if not packed_data.startswith(topic_bytes):
        raise ValueError(f"Message does not start with expected topic '{topic}'")

    offset = len(topic_bytes)
    if len(packed_data) < offset + HEADER_SIZE:
        raise ValueError(f"Packed data too small: {len(packed_data)} < {offset + HEADER_SIZE}")

    header_bytes = packed_data[offset : offset + HEADER_SIZE]
    null_idx = header_bytes.find(b"\x00")
    if null_idx > 0:
        header_bytes = header_bytes[:null_idx]

    header = json.loads(header_bytes.decode("utf-8"))
    fields = header.get("fields", [])

    result = {"version": header.get("v", 0), "endian": header.get("endian", "le")}
    current_offset = offset + HEADER_SIZE
    dtype_map = {
        "f32": np.float32,
        "f64": np.float64,
        "i32": np.int32,
        "i64": np.int64,
        "bool": bool,
    }

    for field in fields:
        dtype = dtype_map.get(field["dtype"], np.float32)
        shape = tuple(field["shape"])
        n_bytes = int(np.prod(shape)) * np.dtype(dtype).itemsize
        result[field["name"]] = (
            np.frombuffer(packed_data[current_offset : current_offset + n_bytes], dtype=dtype)
            .reshape(shape)
            .copy()
        )
        current_offset += n_bytes

    return result


class TimingThresholdMonitor:
    def __init__(self, max_failures=3, reset_timeout_sec=5, time_delta=0.2, raise_exception=False):
        self.max_failures = max_failures
        self.reset_timeout_sec = reset_timeout_sec
        self.failure_count = 0
        self.last_failure_time = 0
        self.time_delta = time_delta
        self.raise_exception = raise_exception

    def reset(self):
        self.failure_count = 0
        self.last_failure_time = 0

    def log_time_delta(self, time_delta_sec: float):
        time_delta = abs(time_delta_sec)
        if time_delta > self.time_delta:
            self.failure_count += 1
            self.last_failure_time = time.monotonic()

        if self.is_threshold_exceeded():
            print(
                f"Time delta exception: {self.failure_count} failures in "
                f"{self.reset_timeout_sec} seconds, time delta: {time_delta}"
            )
            if self.raise_exception:
                raise TimeDeltaException(self.failure_count, self.reset_timeout_sec)

    def is_threshold_exceeded(self):
        if self.failure_count >= self.max_failures:
            return True
        if time.monotonic() - self.last_failure_time > self.reset_timeout_sec:
            self.reset()
        return False


# ---------------------------------------------------------------------------
# Data Collector
# ---------------------------------------------------------------------------


class GrootDataCollector:
    """Collects data from G1 robot in Sonic CPP + SMPL mode -- no ROS 2.

    Data sources (all ZMQ):
      - ``g1_debug`` topic        -> proprio (body_q, hand_q, actions, base_quat, ...)
      - ``pose`` topic            -> SMPL pose (smpl_joints, body_quat_w, hand_joints, ...)
      - ``planner`` topic         -> planner commands (vr_position, vr_orientation, ...)
      - ``manager_state`` topic   -> current stream mode + toggle flags
      - Camera client             -> ego-view images
    """

    def __init__(
        self,
        camera_host: str,
        camera_port: int,
        data_exporter: Gr00tDataExporter,
        robot_model,
        text_to_speech=None,
        frequency: int = 20,
        sonic_data_zmq_host: str = "localhost",
        sonic_data_zmq_port: int = 5556,
        state_zmq_host: str = "localhost",
        state_zmq_port: int = 5557,
        revo2_hands: Revo2TeleopHands | None = None,
        record_wrist_cameras: bool = False,
        wrist_camera_port: int = 5558,
        rewrite_motion_token_supervised: bool = False,
        supervised_token_device: str = "cpu",
    ):
        self.text_to_speech = text_to_speech
        self.frequency = frequency
        self.loop_period = 1.0 / frequency
        self.data_exporter = data_exporter
        self.robot_model = robot_model
        self.revo2_hands = revo2_hands
        self._latest_revo2_cmds = None
        self.record_wrist_cameras = bool(record_wrist_cameras)
        self.rewrite_motion_token_supervised = bool(rewrite_motion_token_supervised)
        self.supervised_token_device = str(supervised_token_device or "cpu")

        self._episode_state = EpisodeState()
        self._keyboard_listener = ZMQKeyboardSubscriber()

        self._image_subscriber = ComposedCameraClientSensor(server_ip=camera_host, port=camera_port)
        self._wrist_image_subscriber = None
        if self.record_wrist_cameras:
            self._wrist_image_subscriber = ComposedCameraClientSensor(
                server_ip=camera_host, port=wrist_camera_port
            )
            print(
                f"[Camera] wrist subscribe {camera_host}:{wrist_camera_port} "
                f"(left_wrist + right_wrist)"
            )

        self.obs_act_buffer = deque(maxlen=100)
        self.latest_image_msg = None
        self.latest_proprio_msg = None
        self.latest_sonic_msg = None
        self.latest_planner_msg = None

        self.current_stream_mode = 0

        self._manager_toggle_dc = False
        self._manager_toggle_da = False
        self._toggle_dc_seen = False
        self._toggle_da_seen = False

        self._state_subscriber = ZMQStateSubscriber(
            host=state_zmq_host,
            port=state_zmq_port,
        )

        self._sonic_zmq_ctx = None
        self._sonic_zmq_socket = None
        try:
            self._sonic_zmq_ctx = zmq.Context()
            self._sonic_zmq_socket = self._sonic_zmq_ctx.socket(zmq.SUB)
            self._sonic_zmq_socket.connect(f"tcp://{sonic_data_zmq_host}:{sonic_data_zmq_port}")
            self._sonic_zmq_socket.setsockopt(zmq.RCVTIMEO, 100)
            self._sonic_zmq_socket.setsockopt(zmq.CONFLATE, 0)
            # High HWM: pose floods must not drop one-shot manager_state stop toggles.
            self._sonic_zmq_socket.setsockopt(zmq.RCVHWM, 1000)
            self._sonic_zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "pose")
            self._sonic_zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "planner")
            self._sonic_zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "manager_state")
            time.sleep(0.5)
            print(f"[Sonic] Connected to ZMQ at {sonic_data_zmq_host}:{sonic_data_zmq_port}")
            print("[Sonic] Subscribed to: pose, planner, manager_state")
        except Exception as e:
            print(f"[Sonic] Warning: Failed to initialize ZMQ subscriber: {e}")
            self._sonic_zmq_socket = None

        self.telemetry = Telemetry(window_size=100)
        self.sonic_timing_monitor = TimingThresholdMonitor(
            max_failures=3, reset_timeout_sec=5, time_delta=0.1
        )

        self._last_latency_log_time = 0.0
        self._initial_yaw = None

        print(f"Recording to {self.data_exporter.meta.root}")

    @property
    def current_episode_index(self):
        return self.data_exporter.episode_buffer["episode_index"]

    def _print_and_say(self, message: str, say: bool = True, blocking: bool = False):
        if self.text_to_speech is not None:
            self.text_to_speech.print_and_say(message, say, blocking=blocking)
        else:
            print(message)

    def _poll_state_zmq(self):
        """Poll the ``g1_debug`` ZMQ topic for robot state (non-blocking)."""
        msg = self._state_subscriber.get_msg(clear=True)
        if msg is None:
            return

        if msg.get("ros_timestamp", 0.0) == 0.0:
            msg["ros_timestamp"] = time.time()

        self.latest_proprio_msg = msg

    def _poll_camera_messages(self) -> None:
        """Merge ego/head (:5555) and optional wrist (:5558) image streams."""
        images: dict = {}
        timestamps: dict = {}

        ego_msg = self._image_subscriber.read()
        if ego_msg is not None:
            if ego_msg.get("images"):
                images.update(ego_msg["images"])
            if ego_msg.get("timestamps"):
                timestamps.update(ego_msg["timestamps"])

        if self._wrist_image_subscriber is not None:
            wrist_msg = self._wrist_image_subscriber.read()
            if wrist_msg is not None:
                if wrist_msg.get("images"):
                    images.update(wrist_msg["images"])
                if wrist_msg.get("timestamps"):
                    timestamps.update(wrist_msg["timestamps"])

        if not images:
            return

        if self.latest_image_msg is None:
            self.latest_image_msg = {"images": {}, "timestamps": {}}
        # Keep last-good frames per key so a lagging wrist port doesn't drop ego/head.
        self.latest_image_msg["images"].update(images)
        self.latest_image_msg.setdefault("timestamps", {}).update(timestamps)

    def _check_recording_commands(self):
        """Check keyboard + ZMQ toggle flags for recording commands."""
        key = self._keyboard_listener.read_msg()

        if self._manager_toggle_da:
            key = "x"
            self._manager_toggle_da = False
        elif self._manager_toggle_dc:
            key = "c"
            self._manager_toggle_dc = False

        if key == "c":
            self._episode_state.change_state()
            if self._episode_state.get_state() == self._episode_state.RECORDING:
                self._initial_yaw = None
                self._print_and_say(
                    f"Started recording {self.current_episode_index}", blocking=False
                )
            elif self._episode_state.get_state() == self._episode_state.NEED_TO_SAVE:
                self._print_and_say("Stopping recording, preparing to save", blocking=False)
            elif self._episode_state.get_state() == self._episode_state.IDLE:
                self._print_and_say("Saved episode and back to idle state", blocking=False)
        elif key == "x":
            if self._episode_state.get_state() == self._episode_state.RECORDING:
                self.data_exporter.save_episode_as_discarded()
                self._episode_state.reset_state()
                self._initial_yaw = None
                self._print_and_say("Discarded episode", blocking=False)

    def _poll_sonic_zmq_messages(self):
        """Poll ZMQ for pose, planner, and manager_state messages (non-blocking)."""
        if self._sonic_zmq_socket is None:
            return

        # Drain aggressively so a burst after a slow video put cannot bury stop toggles.
        max_polls = 500
        for _ in range(max_polls):
            try:
                raw = self._sonic_zmq_socket.recv(zmq.NOBLOCK)
            except zmq.Again:
                break

            if raw.startswith(b"manager_state"):
                self._handle_manager_state(raw)
            elif raw.startswith(b"planner"):
                self._handle_planner_message(raw)
            elif raw.startswith(b"pose"):
                self._handle_pose_message(raw)

    def _handle_manager_state(self, raw: bytes) -> None:
        try:
            data = unpack_pose_message(raw, topic="manager_state")
        except Exception:
            return

        if "stream_mode" in data:
            self.current_stream_mode = int(data["stream_mode"].flat[0])

        # Rising-edge latch: pico may sticky-repeat toggles so a stalled exporter
        # still sees stop; only the first True after a False should fire once.
        dc = self._extract_bool(data, "toggle_data_collection")
        da = self._extract_bool(data, "toggle_data_abort")
        if dc and not self._toggle_dc_seen:
            self._manager_toggle_dc = True
        if da and not self._toggle_da_seen:
            self._manager_toggle_da = True
        self._toggle_dc_seen = dc
        self._toggle_da_seen = da

    def _handle_planner_message(self, raw: bytes) -> None:
        try:
            data = unpack_pose_message(raw, topic="planner")
        except Exception:
            return

        planner_mode = int(data["mode"].flat[0]) if "mode" in data else 0
        planner_movement = (
            data["movement"].flatten().astype(np.float32)
            if "movement" in data and data["movement"].size == 3
            else np.zeros(3, dtype=np.float32)
        )
        planner_facing = (
            data["facing"].flatten().astype(np.float32)
            if "facing" in data and data["facing"].size == 3
            else np.array([1.0, 0.0, 0.0], dtype=np.float32)
        )
        planner_speed = float(data["speed"].flat[0]) if "speed" in data else -1.0
        planner_height = float(data["height"].flat[0]) if "height" in data else -1.0

        vr_3pt_position = None
        if "vr_position" in data and data["vr_position"].size == 9:
            vr_3pt_position = data["vr_position"].flatten().astype(np.float32)
        vr_3pt_orientation = None
        if "vr_orientation" in data and data["vr_orientation"].size == 12:
            vr_3pt_orientation = data["vr_orientation"].flatten().astype(np.float32)

        self.latest_planner_msg = {
            "planner_mode": planner_mode,
            "planner_movement": planner_movement,
            "planner_facing": planner_facing,
            "planner_speed": planner_speed,
            "planner_height": planner_height,
            "vr_3pt_position": vr_3pt_position,
            "vr_3pt_orientation": vr_3pt_orientation,
            "left_hand_joints": self._extract_hand_joints(data, "left_hand_joints"),
            "right_hand_joints": self._extract_hand_joints(data, "right_hand_joints"),
            "receive_timestamp": time.time(),
        }

    def _handle_pose_message(self, raw: bytes) -> None:
        G1_L_WRIST_ROLL_IDX = 23
        G1_L_WRIST_PITCH_IDX = 25
        G1_L_WRIST_YAW_IDX = 27
        G1_R_WRIST_ROLL_IDX = 24
        G1_R_WRIST_PITCH_IDX = 26
        G1_R_WRIST_YAW_IDX = 28

        try:
            pose_data = unpack_pose_message(raw, topic="pose")
        except Exception as e:
            print(f"[Sonic] Error unpacking pose message: {e}")
            return

        try:
            if "smpl_joints" not in pose_data or len(pose_data["smpl_joints"].shape) != 3:
                return

            left_wrist_joints = None
            right_wrist_joints = None
            if "joint_pos" in pose_data and len(pose_data["joint_pos"].shape) == 2:
                joint_pos = pose_data["joint_pos"][0]
                left_wrist_joints = np.array(
                    [
                        joint_pos[G1_L_WRIST_ROLL_IDX],
                        joint_pos[G1_L_WRIST_PITCH_IDX],
                        joint_pos[G1_L_WRIST_YAW_IDX],
                    ],
                    dtype=np.float32,
                )
                right_wrist_joints = np.array(
                    [
                        joint_pos[G1_R_WRIST_ROLL_IDX],
                        joint_pos[G1_R_WRIST_PITCH_IDX],
                        joint_pos[G1_R_WRIST_YAW_IDX],
                    ],
                    dtype=np.float32,
                )

            frame_index = None
            if "frame_index" in pose_data:
                frame_index = np.array([pose_data["frame_index"].flat[0]], dtype=np.int64)

            smpl_pose = np.zeros(63, dtype=np.float32)
            if "smpl_pose" in pose_data:
                raw_pose = pose_data["smpl_pose"]
                if raw_pose.ndim == 3:
                    smpl_pose = raw_pose[0].flatten().astype(np.float32)
                elif raw_pose.ndim == 2:
                    smpl_pose = raw_pose.flatten().astype(np.float32)
                elif raw_pose.ndim == 1 and raw_pose.size == 63:
                    smpl_pose = raw_pose.astype(np.float32)

            left_hand_joints = self._extract_hand_joints(pose_data, "left_hand_joints")
            right_hand_joints = self._extract_hand_joints(pose_data, "right_hand_joints")

            def _trig(key: str) -> float:
                if key not in pose_data or pose_data[key] is None:
                    return 0.0
                return float(np.asarray(pose_data[key]).flat[0])

            vr_3pt_position = None
            if "vr_position" in pose_data and pose_data["vr_position"].size == 9:
                vr_3pt_position = pose_data["vr_position"].flatten().astype(np.float32)
            vr_3pt_orientation = None
            if "vr_orientation" in pose_data and pose_data["vr_orientation"].size == 12:
                vr_3pt_orientation = pose_data["vr_orientation"].flatten().astype(np.float32)

            self.latest_sonic_msg = {
                "smpl_joints": pose_data["smpl_joints"][0],
                "smpl_pose": smpl_pose,
                "body_quat_w": (
                    pose_data["body_quat_w"][0] if "body_quat_w" in pose_data else None
                ),
                "left_hand_joints": left_hand_joints,
                "right_hand_joints": right_hand_joints,
                "left_trigger": _trig("left_trigger"),
                "right_trigger": _trig("right_trigger"),
                "left_wrist_joints": left_wrist_joints,
                "right_wrist_joints": right_wrist_joints,
                "vr_3pt_position": vr_3pt_position,
                "vr_3pt_orientation": vr_3pt_orientation,
                "frame_index": frame_index,
                "receive_timestamp": time.time(),
            }
        except Exception as e:
            if not hasattr(self, "_sonic_error_count"):
                self._sonic_error_count = 0
            self._sonic_error_count += 1
            if self._sonic_error_count == 1 or self._sonic_error_count % 100 == 0:
                print(f"[Sonic] Error processing pose message: {e}")

    @staticmethod
    def _extract_hand_joints(pose_data: dict, key: str) -> np.ndarray:
        arr = pose_data.get(key)
        if arr is not None:
            if arr.ndim > 1:
                arr = arr[0]
            return arr.astype(np.float32)
        return np.zeros(7, dtype=np.float32)

    def _resolve_dex3_hand_joints(
        self,
        hand_msg: dict | None,
        pico_key: str,
        proprio_action_key: str,
        proprio_state_key: str,
    ) -> np.ndarray:
        """Return 7D Dex3 targets for ``teleop.*_hand_joints``.

        Order: Pico Trigger/Grip command → deploy last hand action → measured ``hand_q``.
        """
        zeros = np.zeros(7, dtype=np.float32)
        if hand_msg is not None and hand_msg.get(pico_key) is not None:
            pico = np.asarray(hand_msg[pico_key], dtype=np.float32).reshape(-1)
            if pico.size >= 7 and float(np.max(np.abs(pico[:7]))) > 1e-6:
                return pico[:7].astype(np.float32)

        proprio = self.latest_proprio_msg or {}
        for key in (proprio_action_key, proprio_state_key):
            val = proprio.get(key)
            if val is None:
                continue
            arr = np.asarray(val, dtype=np.float32).reshape(-1)
            if arr.size >= 7 and float(np.max(np.abs(arr[:7]))) > 1e-6:
                return arr[:7].astype(np.float32)
        return zeros

    @staticmethod
    def _extract_bool(pose_data: dict, key: str) -> bool:
        val = pose_data.get(key)
        if val is None:
            return False
        if isinstance(val, np.ndarray):
            return bool(val.flat[0])
        return bool(val)

    def _log_latency_periodic(
        self,
        sonic_latency_ms: float | None = None,
    ):
        current_time = time.time()
        if current_time - self._last_latency_log_time >= 1.0:
            self._last_latency_log_time = current_time
            parts = []
            if sonic_latency_ms is not None:
                parts.append(f"Sonic Pose: {sonic_latency_ms:.1f}ms")
            if parts:
                print(f"[Latency] {', '.join(parts)}")

    def _add_images_to_frame_data(self, frame_data: dict) -> None:
        if self.latest_image_msg is None:
            return
        images = self.latest_image_msg["images"]
        for feature_name, feature_info in self.data_exporter.features.items():
            if feature_info.get("dtype") in ["image", "video"]:
                image_key = feature_name.split(".")[-1]
                if image_key not in images:
                    raise ValueError(
                        f"Required image '{image_key}' for feature '{feature_name}' "
                        f"not found in image message. Available: {list(images.keys())}"
                    )
                frame_data[feature_name] = images[image_key]

    def _finalize_frame(self, t_start: float) -> bool:
        t_end = time.monotonic()
        if t_end - t_start > (1 / self.frequency):
            print(f"DataExporter Missed: {t_end - t_start} sec")

        if self._episode_state.get_state() == self._episode_state.NEED_TO_SAVE:
            buffer_size = self.data_exporter.episode_buffer.get("size", 0)
            if buffer_size > 0:
                self._rewrite_supervised_motion_tokens()
                self.data_exporter.save_episode()
                self.sonic_timing_monitor.reset()
                self._initial_yaw = None
                self._print_and_say("Finished saving episode")
            else:
                self._print_and_say("Skipping save: no frames collected", say=False)
            self._episode_state.change_state()
        return True

    def _add_data_frame(self):
        t_start = time.monotonic()

        if self.latest_proprio_msg is None or self.latest_image_msg is None:
            self._print_and_say(
                f"Waiting for message. "
                f"Avail msg: proprio {self.latest_proprio_msg is not None} | "
                f"image {self.latest_image_msg is not None}",
                say=False,
            )
            return False

        if self._episode_state.get_state() != self._episode_state.RECORDING:
            return self._finalize_frame(t_start)

        return self._add_data_frame_sonic(t_start)

    def _add_data_frame_sonic(self, t_start: float) -> bool:
        """Build one data frame in Sonic CPP + SMPL mode."""
        assert self.latest_proprio_msg is not None
        proprio = self.latest_proprio_msg

        left_hand_q = proprio["left_hand_q"]
        right_hand_q = proprio["right_hand_q"]
        left_hand_act = proprio["last_left_hand_action"]
        right_hand_act = proprio["last_right_hand_action"]
        # No Dex3 DDS with Revo2: fill 7D hand slots from commanded Revo2 (norm 0..1).
        if self.revo2_hands is not None:
            lh_s, rh_s = self.revo2_hands.state_hand_vectors(self._latest_revo2_cmds)
            left_hand_q = lh_s
            right_hand_q = rh_s
            left_hand_act = lh_s.astype(np.float32)
            right_hand_act = rh_s.astype(np.float32)

        whole_q = self.robot_model.get_configuration_from_actuated_joints(
            body_actuated_joint_values=proprio["body_q"],
            left_hand_actuated_joint_values=left_hand_q,
            right_hand_actuated_joint_values=right_hand_q,
        )
        whole_action_wbc = self.robot_model.get_configuration_from_actuated_joints(
            body_actuated_joint_values=proprio["last_action"],
            left_hand_actuated_joint_values=left_hand_act,
            right_hand_actuated_joint_values=right_hand_act,
        )

        self.robot_model.cache_forward_kinematics(whole_q)
        eef_parts = []
        for side in ["left", "right"]:
            placement = self.robot_model.frame_placement(
                self.robot_model.supplemental_info.hand_frame_names[side]
            )
            pos = placement.translation[:3]
            quat = R.from_matrix(placement.rotation).as_quat(scalar_first=True)
            eef_parts.append(np.concatenate([pos, quat]))
        observation_eef_state = np.concatenate(eef_parts)

        frame_data: dict = {
            "observation.state": whole_q,
            "observation.eef_state": observation_eef_state,
            "action.wbc": whole_action_wbc,
        }

        self._add_cpp_state_features(frame_data, proprio)

        sonic_latency_ms = self._add_sonic_pose_features(frame_data)

        self._add_images_to_frame_data(frame_data)

        self._log_latency_periodic(sonic_latency_ms)

        def _interrupt() -> bool:
            # Keep consuming ZMQ while video queues are full so Pico stop is not lost.
            self._poll_sonic_zmq_messages()
            return bool(self._manager_toggle_dc or self._manager_toggle_da)

        accepted = self.data_exporter.add_frame(frame_data, interrupt_check=_interrupt)
        if not accepted:
            # Apply pending stop/abort on the same iteration.
            self._check_recording_commands()
        return self._finalize_frame(t_start)

    def _add_cpp_state_features(self, frame_data: dict, proprio: dict) -> None:
        if "base_quat" in proprio:
            base_quat = np.asarray(proprio["base_quat"], dtype=np.float64)
            frame_data["observation.root_orientation"] = base_quat
            frame_data["observation.projected_gravity"] = compute_projected_gravity(
                base_quat
            ).astype(np.float64)

            if "init_ref_data_root_rot_array" in proprio:
                frame_data["observation.cpp_rotation_offset"] = np.asarray(
                    proprio["init_ref_data_root_rot_array"], dtype=np.float64
                )
            else:
                frame_data["observation.cpp_rotation_offset"] = np.array(
                    [1.0, 0.0, 0.0, 0.0], dtype=np.float64
                )
        else:
            frame_data["observation.root_orientation"] = np.array(
                [1.0, 0.0, 0.0, 0.0], dtype=np.float64
            )
            frame_data["observation.projected_gravity"] = np.array(
                [0.0, 0.0, -1.0], dtype=np.float64
            )
            frame_data["observation.cpp_rotation_offset"] = np.array(
                [1.0, 0.0, 0.0, 0.0], dtype=np.float64
            )

        if "init_base_quat" in proprio:
            frame_data["observation.init_base_quat"] = np.asarray(
                proprio["init_base_quat"], dtype=np.float64
            )
        else:
            frame_data["observation.init_base_quat"] = np.array(
                [1.0, 0.0, 0.0, 0.0], dtype=np.float64
            )

        if "delta_heading" in proprio:
            dh = proprio["delta_heading"]
            if isinstance(dh, np.ndarray):
                dh = dh.item() if dh.size == 1 else dh[0]
            frame_data["teleop.delta_heading"] = np.array([float(dh)], dtype=np.float64)
        else:
            frame_data["teleop.delta_heading"] = np.zeros(1, dtype=np.float64)

        # Live deploy latent (closed-loop heading). Disk supervision is rewritten
        # to open-loop mode-2 at episode save — see ``_rewrite_supervised_motion_tokens``.
        if "token_state" in proprio:
            frame_data["action.motion_token"] = np.asarray(proprio["token_state"], dtype=np.float64)
        else:
            frame_data["action.motion_token"] = np.zeros(64, dtype=np.float64)

    def _rewrite_supervised_motion_tokens(self) -> None:
        """Replace buffered live tokens with §8-identical open-loop mode-2 tokens."""
        if not self.rewrite_motion_token_supervised:
            return
        buf = self.data_exporter.episode_buffer
        if buf is None or int(buf.get("size", 0)) <= 0:
            return
        try:
            from gear_sonic.utils.data_collection.supervised_mode2_tokens import (
                append_rewrite_record,
                rewrite_episode_buffer_motion_tokens,
            )
        except Exception as exc:
            self._print_and_say(
                f"[motion_token] supervised rewrite unavailable ({exc}); keeping live tokens",
                say=False,
            )
            return

        root = self.data_exporter.meta.root
        live_dir = root / "meta" / "motion_token_live"
        try:
            meta = rewrite_episode_buffer_motion_tokens(
                buf,
                device=self.supervised_token_device,
                live_backup_dir=live_dir,
                episode_index=int(buf.get("episode_index", -1)),
            )
        except Exception as exc:
            self._print_and_say(
                f"[motion_token] supervised rewrite failed ({exc}); keeping live tokens",
                say=False,
            )
            return

        try:
            append_rewrite_record(root, meta)
        except Exception as exc:
            print(f"[motion_token] warning: could not append rewrite record: {exc}")

        if meta.get("rewrote"):
            l2 = meta.get("l2_vs_live_mean")
            l2_s = f"{l2:.3f}" if isinstance(l2, (int, float)) else "?"
            self._print_and_say(
                f"[motion_token] ep{meta.get('episode_index')} rewritten "
                f"mode2 open-loop (L2 vs live mean={l2_s})",
                say=False,
            )
        elif meta.get("skipped"):
            self._print_and_say(
                f"[motion_token] rewrite skipped: {meta.get('reason')}",
                say=False,
            )

    def _add_sonic_pose_features(self, frame_data: dict) -> float | None:
        """Add teleop features based on current stream mode."""
        sonic_latency_ms = None

        frame_data["teleop.stream_mode"] = np.array([self.current_stream_mode], dtype=np.int32)

        smpl_msg = self.latest_sonic_msg
        use_smpl = False
        if self.current_stream_mode in (1, 4) and smpl_msg is not None:
            receive_ts = smpl_msg.get("receive_timestamp")
            if receive_ts is not None:
                age_sec = time.time() - receive_ts
                sonic_latency_ms = age_sec * 1000
                self.sonic_timing_monitor.log_time_delta(age_sec)
                if sonic_latency_ms <= 100.0:
                    use_smpl = True
                elif (self.sonic_timing_monitor.failure_count + 1) % 10 == 0:
                    self._print_and_say(
                        f"Sonic pose stale ({sonic_latency_ms:.1f}ms old), using zeros",
                        say=False,
                    )
            else:
                use_smpl = True

        planner_msg = self.latest_planner_msg
        use_planner = False
        if self.current_stream_mode == 5 and planner_msg is not None:
            receive_ts = planner_msg.get("receive_timestamp")
            if receive_ts is not None:
                age_sec = time.time() - receive_ts
                planner_latency_ms = age_sec * 1000
                if sonic_latency_ms is None:
                    sonic_latency_ms = planner_latency_ms
                if planner_latency_ms <= 200.0:
                    use_planner = True
            else:
                use_planner = True

        # SMPL features
        if use_smpl and smpl_msg.get("smpl_joints") is not None:
            joints = np.asarray(smpl_msg["smpl_joints"], dtype=np.float32)
            if joints.ndim == 2:
                joints = joints.flatten()
            frame_data["teleop.smpl_joints"] = np.ascontiguousarray(joints, dtype=np.float32)
        else:
            frame_data["teleop.smpl_joints"] = np.zeros(72, dtype=np.float32)

        if use_smpl and smpl_msg.get("smpl_pose") is not None:
            pose = np.asarray(smpl_msg["smpl_pose"], dtype=np.float32)
            if pose.ndim > 1:
                pose = pose.flatten()
            frame_data["teleop.smpl_pose"] = np.ascontiguousarray(pose, dtype=np.float32)
        else:
            frame_data["teleop.smpl_pose"] = np.zeros(63, dtype=np.float32)

        if use_smpl and smpl_msg.get("body_quat_w") is not None:
            body_quat_w = smpl_msg["body_quat_w"].astype(np.float32)
            frame_data["teleop.body_quat_w"] = body_quat_w
            frame_data["teleop.target_body_orientation"] = self._compute_target_body_orientation(
                body_quat_w, frame_data
            )
        else:
            frame_data["teleop.body_quat_w"] = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
            frame_data["teleop.target_body_orientation"] = quat_to_rot6d(
                np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
            )

        frame_data["teleop.left_wrist_joints"] = (
            smpl_msg["left_wrist_joints"].astype(np.float32)
            if use_smpl and smpl_msg.get("left_wrist_joints") is not None
            else np.zeros(3, dtype=np.float32)
        )
        frame_data["teleop.right_wrist_joints"] = (
            smpl_msg["right_wrist_joints"].astype(np.float32)
            if use_smpl and smpl_msg.get("right_wrist_joints") is not None
            else np.zeros(3, dtype=np.float32)
        )

        frame_data["teleop.smpl_frame_index"] = (
            smpl_msg["frame_index"].astype(np.int64)
            if use_smpl and smpl_msg is not None and smpl_msg.get("frame_index") is not None
            else np.array([0], dtype=np.int64)
        )

        hand_msg = (
            smpl_msg if self.current_stream_mode in (1, 4) and smpl_msg is not None
            else planner_msg if planner_msg is not None
            else smpl_msg
        )
        # Dex3 7D action labels for GR00T UNITREE_G1_SONIC.
        # Hand action labels:
        #   dex3  → Pico Dex3 7D (fallback deploy / measured)
        #   revo2 → recorded grasp traj 6D driven by Pico triggers
        if self.revo2_hands is not None:
            # Pad 6D→7D so schema matches stock UNITREE_G1_SONIC (Dex3-sized).
            left_h, right_h = self.revo2_hands.teleop_vectors(
                self._latest_revo2_cmds, pad_to_dex3=True
            )
            frame_data["teleop.left_hand_joints"] = left_h
            frame_data["teleop.right_hand_joints"] = right_h
        else:
            # Prefer Pico Trigger/Grip targets; if those are missing/zero, fall back to
            # the command (then measured state) published on g1_debug so recording
            # chords like Left Grip+A still yield non-empty teleop.*_hand_joints when
            # the hand was actually driven.
            frame_data["teleop.left_hand_joints"] = self._resolve_dex3_hand_joints(
                hand_msg, "left_hand_joints", "last_left_hand_action", "left_hand_q"
            )
            frame_data["teleop.right_hand_joints"] = self._resolve_dex3_hand_joints(
                hand_msg, "right_hand_joints", "last_right_hand_action", "right_hand_q"
            )

        # Planner command fields
        frame_data["teleop.planner_mode"] = np.array(
            [planner_msg["planner_mode"]] if use_planner else [0],
            dtype=np.int32,
        )
        frame_data["teleop.planner_movement"] = (
            planner_msg["planner_movement"].copy()
            if use_planner and planner_msg.get("planner_movement") is not None
            else np.zeros(3, dtype=np.float32)
        )
        frame_data["teleop.planner_facing"] = (
            planner_msg["planner_facing"].copy()
            if use_planner and planner_msg.get("planner_facing") is not None
            else np.array([1.0, 0.0, 0.0], dtype=np.float32)
        )
        frame_data["teleop.planner_speed"] = np.array(
            [planner_msg["planner_speed"]] if use_planner else [-1.0],
            dtype=np.float32,
        )
        frame_data["teleop.planner_height"] = np.array(
            [planner_msg["planner_height"]] if use_planner else [-1.0],
            dtype=np.float32,
        )

        # VR 3-point pose
        frame_data["teleop.vr_3pt_position"] = (
            planner_msg["vr_3pt_position"].astype(np.float32)
            if use_planner and planner_msg.get("vr_3pt_position") is not None
            else np.zeros(9, dtype=np.float32)
        )
        if use_planner and planner_msg.get("vr_3pt_orientation") is not None:
            frame_data["teleop.vr_3pt_orientation"] = quat_to_rot6d(
                planner_msg["vr_3pt_orientation"].astype(np.float32)
            )
        else:
            frame_data["teleop.vr_3pt_orientation"] = np.zeros(18, dtype=np.float32)

        return sonic_latency_ms

    def _compute_target_body_orientation(
        self, body_quat_w: np.ndarray, frame_data: dict
    ) -> np.ndarray:
        """Compute yaw-normalised target body orientation as rot6d (6-dim)."""
        delta_heading = float(frame_data.get("teleop.delta_heading", [0.0])[0])

        body_rot = R.from_quat(body_quat_w, scalar_first=True)
        target_rot = R.from_euler("z", delta_heading, degrees=False) * body_rot

        euler = target_rot.as_euler("ZYX", degrees=False)
        current_yaw = euler[0]

        if self._initial_yaw is None:
            self._initial_yaw = current_yaw

        normalised_euler = np.array([current_yaw - self._initial_yaw, euler[1], euler[2]])
        target_quat = (
            R.from_euler("ZYX", normalised_euler, degrees=False)
            .as_quat(scalar_first=True)
            .astype(np.float32)
        )
        return quat_to_rot6d(target_quat)

    def save_and_cleanup(self):
        try:
            self._print_and_say("saving episode done", blocking=False)
            buffer_size = self.data_exporter.episode_buffer.get("size", 0)
            if buffer_size > 0:
                self._rewrite_supervised_motion_tokens()
                self.data_exporter.save_episode()
            self._print_and_say(
                f"Recording complete: {self.data_exporter.meta.root}", say=False, blocking=True
            )
        except Exception as e:
            self._print_and_say(f"Error saving episode: {e}", blocking=True)

        try:
            self._state_subscriber.close()
        except Exception:
            pass
        for sock in [self._sonic_zmq_socket]:
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
        for ctx in [self._sonic_zmq_ctx]:
            if ctx is not None:
                try:
                    ctx.term()
                except Exception:
                    pass
        if self.revo2_hands is not None:
            try:
                self.revo2_hands.close()
            except Exception:
                pass

        self._print_and_say("Shutting down data exporter...", say=False)

    def run(self):
        try:
            while True:
                t_start = time.monotonic()
                with self.telemetry.timer("total_loop"):
                    with self.telemetry.timer("poll_state"):
                        self._poll_state_zmq()

                    with self.telemetry.timer("poll_sonic"):
                        self._poll_sonic_zmq_messages()

                    # Apply Pico start/stop BEFORE add_frame so a slow encode cannot
                    # bury the edge-triggered toggle for a full queue.put stall.
                    with self.telemetry.timer("check_recording_commands"):
                        self._check_recording_commands()

                    if self.revo2_hands is not None:
                        self._latest_revo2_cmds = self.revo2_hands.step(
                            self.latest_sonic_msg
                        )

                    with self.telemetry.timer("poll_image"):
                        self._poll_camera_messages()

                    with self.telemetry.timer("add_frame"):
                        self._add_data_frame()

                    end_time = time.monotonic()

                elapsed = time.monotonic() - t_start
                sleep_time = self.loop_period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

                if (end_time - t_start) > self.loop_period:
                    self.telemetry.log_timing_info(
                        context="Data Exporter Loop Missed", threshold=0.001
                    )

        except KeyboardInterrupt:
            print("Data exporter terminated by user")
            buffer_size = self.data_exporter.episode_buffer.get("size", 0)
            if buffer_size > 0:
                self.data_exporter.save_episode_as_discarded()

        finally:
            self.save_and_cleanup()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(config: SonicDataExporterConfig):
    g1_rm = get_g1_robot_model()

    hand_type = (config.hand_type or "dex3").strip().lower()
    if hand_type not in ("dex3", "revo2"):
        raise ValueError(f"hand_type must be dex3|revo2, got {config.hand_type!r}")

    # Always 7D teleop hand slots: stock GR00T UNITREE_G1_SONIC expects Dex3 7+7.
    # Revo2 writes 6D cmds in [0..1000] into the first 6 dims and pads dim7=0.
    hand_dim = 7
    dataset_features = get_features_sonic_vla(g1_rm, hand_dim=hand_dim)
    modality_config = get_modality_config_sonic_vla(g1_rm, hand_dim=hand_dim)

    record_wrist, record_head = resolve_camera_recording_flags(config)

    if record_wrist:
        print("[Camera] Wrist cameras enabled — adding to dataset schema")
        dataset_features.update(get_wrist_camera_features())
        wrist_modality = get_wrist_camera_modality_config()
        for key, value in wrist_modality.items():
            if key in modality_config:
                modality_config[key].update(value)
            else:
                modality_config[key] = value

    if record_head:
        print("[Camera] Head camera enabled — adding to dataset schema")
        dataset_features.update(get_head_camera_features())
        head_modality = get_head_camera_modality_config()
        for key, value in head_modality.items():
            if key in modality_config:
                modality_config[key].update(value)
            else:
                modality_config[key] = value

    revo2_hands = None
    revo2_pose_resolved = None
    if hand_type == "revo2":
        pose_path = config.revo2_pose
        if not os.path.isabs(pose_path):
            # resolve relative to Litchi repo root (cwd when following docs)
            pose_path = os.path.abspath(pose_path)
        revo2_pose_resolved = pose_path
        revo2_hands = Revo2TeleopHands(
            pose_path=pose_path,
            hz=float(config.data_collection_frequency),
            release_blend_s=config.revo2_release_blend_s,
            orin_host=config.revo2_orin_host if config.revo2_send else None,
            udp_port=config.revo2_udp_port,
            send=config.revo2_send,
        )
        print(
            f"[Hand] Revo2 mode: cmds=6D(0..1000) → teleop.*_hand_joints padded to 7D "
            f"(UNITREE_G1_SONIC) from {pose_path}"
        )
    else:
        print("[Hand] Dex3 mode: teleop.*_hand_joints = 7D")

    text_to_speech = TextToSpeech() if config.text_to_speech else None

    robot_config = poll_robot_config_zmq(
        config.state_zmq_host, config.state_zmq_port, config.robot_config_timeout
    )

    data_exporter = Gr00tDataExporter.create(
        save_root=f"{config.root_output_dir}/{config.dataset_name}",
        fps=config.data_collection_frequency,
        features=dataset_features,
        modality_config=modality_config,
        task=config.task_prompt,
        script_config={
            **robot_config,
            "record_wrist_cameras": record_wrist,
            "record_head_camera": record_head,
            "hand_type": hand_type,
            "revo2_pose": revo2_pose_resolved,
        },
    )

    data_collector = GrootDataCollector(
        frequency=config.data_collection_frequency,
        data_exporter=data_exporter,
        robot_model=g1_rm,
        camera_host=config.camera_host,
        camera_port=config.camera_port,
        text_to_speech=text_to_speech,
        sonic_data_zmq_host=config.sonic_zmq_host,
        sonic_data_zmq_port=config.sonic_zmq_port,
        state_zmq_host=config.state_zmq_host,
        state_zmq_port=config.state_zmq_port,
        revo2_hands=revo2_hands,
        record_wrist_cameras=record_wrist,
        wrist_camera_port=config.wrist_camera_port,
        rewrite_motion_token_supervised=config.rewrite_motion_token_supervised,
        supervised_token_device=config.supervised_token_device,
    )
    if config.rewrite_motion_token_supervised:
        # Fail fast: HF ``datasets`` name clash / missing ORT / missing encoder must not
        # silently leave live tokens on disk for a whole session.
        try:
            import onnxruntime  # noqa: F401
            from pathlib import Path as _Path

            from gear_sonic.utils.data_collection import supervised_mode2_tokens as _sm2

            _enc = (
                _Path(__file__).resolve().parents[2]
                / "gear_sonic_deploy"
                / "policy"
                / "sonic_v1_1"
                / "model_encoder.onnx"
            )
            if not _enc.is_file():
                raise FileNotFoundError(f"missing encoder onnx: {_enc}")
            # Resolve refill helpers now (HF ``datasets`` name clash fails here, not mid-episode).
            _sm2._import_refill()
            print(
                "[motion_token] episode-end rewrite ENABLED "
                f"(device={config.supervised_token_device}): disk tokens = open-loop mode2; "
                "live deploy tokens backed up under meta/motion_token_live/; "
                f"encoder={_enc}"
            )
        except Exception as exc:
            raise RuntimeError(
                "[motion_token] supervised rewrite preflight failed — refusing to record "
                "with rewrite enabled (would silently keep live tokens). "
                "Fix ORT/encoder/import, or pass --no-rewrite-motion-token-supervised. "
                f"Cause: {exc}"
            ) from exc
    else:
        print("[motion_token] episode-end rewrite DISABLED — disk keeps live token_state")
    data_collector.run()


if __name__ == "__main__":
    config = tyro.cli(SonicDataExporterConfig)

    if config.dataset_name is None:
        config.dataset_name = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")

    main(config)

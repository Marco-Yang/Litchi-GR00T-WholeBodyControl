"""
ROS-free camera viewer with optional recording.

Connects to one or more ZMQ camera servers and displays whatever streams arrive
(OpenCV). Supports recording to MP4.

Virtual environment setup (run from repo root):
    bash install_scripts/install_data_collection.sh
    source .venv_data_collection/bin/activate

Usage (shows whatever streams are live — 1..4 of ego/head/wrists):
    python gear_sonic/scripts/run_camera_viewer.py \\
      --camera-host 192.168.123.164

Controls (OpenCV window must be focused):
    R - Start/stop recording
    Q - Quit
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time
from typing import Optional

import cv2
import numpy as np
import tyro

from gear_sonic.camera.composed_camera import ComposedCameraClientSensor


# Preferred tile order when present; unknown names append alphabetically.
_STREAM_ORDER = ("ego_view", "head", "left_wrist", "right_wrist")

_DEFAULT_EGO_HEAD_PORT = 5555
_DEFAULT_WRIST_PORT = 5558


@dataclass
class CameraViewerConfig:
    """CLI config for the ROS-free camera viewer."""

    camera_host: str = "localhost"
    """Camera server hostname."""

    camera_port: int = _DEFAULT_EGO_HEAD_PORT
    """ZMQ port for ego_view + head (default 5555)."""

    wrist_port: int = _DEFAULT_WRIST_PORT
    """ZMQ port for wrist Abham server (default 5558). Always subscribed; missing streams are skipped."""

    camera_ports: Optional[tuple[int, ...]] = None
    """Override: explicit ZMQ PUB ports only (default: camera_port + wrist_port)."""

    fps: int = 30
    """Target display refresh rate (Hz)."""

    output_path: Optional[str] = None
    """Output directory for recordings. Auto-creates 'camera_recordings/' if not set."""

    codec: str = "mp4v"
    """Video codec for recording (e.g., 'mp4v', 'XVID')."""

    max_display_width: int = 480
    """Max width per camera tile in the display window."""

    wait_secs: float = 10.0
    """How long to wait for the first frame from any port."""


def _resolve_ports(config: CameraViewerConfig) -> list[int]:
    if config.camera_ports:
        return list(config.camera_ports)
    ports = [config.camera_port]
    if config.wrist_port not in ports:
        ports.append(config.wrist_port)
    return ports


def _ordered_names(names: set[str]) -> list[str]:
    ordered = [n for n in _STREAM_ORDER if n in names]
    rest = sorted(names - set(ordered))
    return ordered + rest


def _merge_latest(clients: list[ComposedCameraClientSensor]) -> dict:
    """Merge images/timestamps from all ports; later ports override same key."""
    images: dict = {}
    timestamps: dict = {}
    for client in clients:
        msg = client.read(blocking=False)
        if not msg:
            continue
        if msg.get("images"):
            images.update(msg["images"])
        if msg.get("timestamps"):
            timestamps.update(msg["timestamps"])
    if not images:
        return {}
    return {"images": images, "timestamps": timestamps}


def _layout_tiles(tiles: list[np.ndarray]) -> np.ndarray:
    """1–2: row; 3–4: 2x2; more: wrap at 2 columns."""
    if not tiles:
        return np.zeros((240, 320, 3), dtype=np.uint8)

    max_h = max(t.shape[0] for t in tiles)
    max_w = max(t.shape[1] for t in tiles)

    def pad(t: np.ndarray) -> np.ndarray:
        h, w = t.shape[:2]
        out = np.zeros((max_h, max_w, 3), dtype=np.uint8)
        out[:h, :w] = t
        return out

    padded = [pad(t) for t in tiles]
    n = len(padded)
    if n <= 2:
        return np.hstack(padded)

    cols = 2
    rows = []
    for i in range(0, n, cols):
        row = padded[i : i + cols]
        while len(row) < cols:
            row.append(np.zeros((max_h, max_w, 3), dtype=np.uint8))
        rows.append(np.hstack(row))
    return np.vstack(rows)


def main(config: CameraViewerConfig):
    ports = _resolve_ports(config)
    clients = [
        ComposedCameraClientSensor(server_ip=config.camera_host, port=p) for p in ports
    ]
    print(f"Subscribing {config.camera_host}:{ports}")

    print("Waiting for first camera frame from any port...")
    sample = {}
    deadline = time.time() + config.wait_secs
    while time.time() < deadline:
        sample = _merge_latest(clients)
        if sample.get("images"):
            break
        time.sleep(0.05)

    if not sample.get("images"):
        print(
            f"ERROR: No camera frames after {config.wait_secs:.0f}s. "
            f"Check servers on {config.camera_host}:{ports}."
        )
        for c in clients:
            c.close()
        return

    camera_names = _ordered_names(set(sample["images"].keys()))
    print(f"Detected {len(camera_names)} camera stream(s): {camera_names}")

    output_dir = Path(config.output_path) if config.output_path else Path("camera_recordings")

    is_recording = False
    video_writers: dict[str, cv2.VideoWriter] = {}
    frame_count = 0
    recording_start_time = 0.0
    recording_dir = Path(".")
    loop_period = 1.0 / config.fps

    window_name = "SONIC Camera Viewer"
    print(f"Target FPS: {config.fps}")
    print(f"Recordings will be saved to: {output_dir}")
    print("Controls: R = start/stop recording, Q = quit")

    try:
        while True:
            t_start = time.monotonic()

            image_data = _merge_latest(clients)
            if not image_data.get("images"):
                elapsed = time.monotonic() - t_start
                remaining = loop_period - elapsed
                if remaining > 0:
                    time.sleep(remaining)
                continue

            # Hot-plug / late ports: show whatever is present this frame.
            camera_names = _ordered_names(set(image_data["images"].keys()))

            tiles = []
            for name in camera_names:
                img = image_data["images"].get(name)
                if img is None:
                    continue

                if img.ndim == 3 and img.shape[2] == 3:
                    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                else:
                    img_bgr = img

                if is_recording and name in video_writers:
                    video_writers[name].write(img_bgr)

                h, w = img_bgr.shape[:2]
                if w > config.max_display_width:
                    scale = config.max_display_width / w
                    img_bgr = cv2.resize(
                        img_bgr, (config.max_display_width, int(h * scale))
                    )

                label = f"[REC] {name}" if is_recording else name
                cv2.putText(
                    img_bgr,
                    label,
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
                tiles.append(img_bgr)

            if tiles:
                canvas = _layout_tiles(tiles)

                if is_recording:
                    frame_count += 1
                    elapsed_rec = time.time() - recording_start_time
                    status = f"REC {frame_count}f / {elapsed_rec:.1f}s"
                    cv2.putText(
                        canvas,
                        status,
                        (10, canvas.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 255),
                        2,
                    )

                cv2.imshow(window_name, canvas)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                print("Quit requested.")
                break
            elif key == ord("r"):
                if not is_recording:
                    recording_dir = output_dir / f"rec_{time.strftime('%Y%m%d_%H%M%S')}"
                    recording_dir.mkdir(parents=True, exist_ok=True)

                    fourcc = cv2.VideoWriter_fourcc(*config.codec)
                    video_writers = {}
                    for name in camera_names:
                        img = image_data["images"].get(name)
                        if img is not None:
                            h, w = img.shape[:2]
                            path = recording_dir / f"{name}.mp4"
                            video_writers[name] = cv2.VideoWriter(
                                str(path), fourcc, config.fps, (w, h)
                            )

                    is_recording = True
                    recording_start_time = time.time()
                    frame_count = 0
                    print(f"Recording started: {recording_dir}")
                else:
                    is_recording = False
                    for writer in video_writers.values():
                        writer.release()
                    video_writers = {}
                    duration = time.time() - recording_start_time
                    print(
                        f"Recording stopped - {duration:.1f}s, {frame_count} frames "
                        f"-> {recording_dir}"
                    )

            elapsed = time.monotonic() - t_start
            remaining = loop_period - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if video_writers:
            for writer in video_writers.values():
                writer.release()
            if is_recording:
                duration = time.time() - recording_start_time
                print(f"Final recording: {duration:.1f}s, {frame_count} frames")

        for c in clients:
            c.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    config = tyro.cli(CameraViewerConfig)
    main(config)

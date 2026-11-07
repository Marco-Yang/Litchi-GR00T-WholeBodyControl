import os
import queue
import sys
import tempfile
import threading
import time

import av
import numpy as np

# Process-level codec warm cache: (width, height, fps, codec) -> True
_CODEC_WARMED: set[tuple] = set()
_STOP = object()
# First-frame stderr redirect is process-global — must not race across writer threads.
_STDERR_LOCK = threading.Lock()


def ensure_codec_warmed(width: int, height: int, fps: float, codec: str = "h264") -> None:
    """Throwaway 2-frame encode so the first real episode does not pay cold-start cost."""
    key = (int(width), int(height), float(fps), codec)
    if key in _CODEC_WARMED:
        return
    fd, path = tempfile.mkstemp(suffix=".mp4")
    os.close(fd)
    try:
        writer = VideoWriter(
            path,
            width,
            height,
            fps,
            codec=codec,
            buffer_size=8,
            warm_codec=False,
        )
        blank = np.zeros((height, width, 3), dtype=np.uint8)
        writer.add_frame(blank)
        writer.add_frame(blank)
        writer.stop()
    finally:
        try:
            os.remove(path)
        except OSError:
            pass
    _CODEC_WARMED.add(key)


class VideoWriter:
    def __init__(
        self,
        output_path: str,
        width: int,
        height: int,
        fps: float,
        codec: str = "h264",
        buffer_size: int = 200,
        warm_codec: bool = True,
    ):
        self.output_path = output_path
        self._first_frame = True
        self._closed = False
        self._done = threading.Event()
        self._reject_frames = False

        output_dir = os.path.dirname(output_path)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)

        if warm_codec:
            ensure_codec_warmed(width, height, fps, codec)

        self.queue: queue.Queue = queue.Queue(maxsize=buffer_size)
        self.container = av.open(output_path, mode="w")
        # PyAV wants an int/Rational rate; collection fps is typically 30/50.
        self.stream = self.container.add_stream(codec, rate=int(round(fps)))
        self.stream.width = width
        self.stream.height = height
        self._thread = threading.Thread(
            target=self._writer_worker, name=f"VideoWriter:{os.path.basename(output_path)}", daemon=True
        )
        self._thread.start()

    def _assert_dimensions(self, frame: np.ndarray) -> None:
        assert (
            frame.shape[1] == self.stream.width and frame.shape[0] == self.stream.height
        ), (
            f"Incorrect frame dimensions. Input dimensions: {frame.shape[1]}x{frame.shape[0]}. "
            f"Expected dimensions: {self.stream.width}x{self.stream.height}"
        )

    def add_frame(self, frame: np.ndarray, interrupt_check=None) -> bool:
        """Enqueue a frame without dropping.

        Uses short timed puts so the caller can keep polling Pico stop/abort
        (via ``interrupt_check``) while the encoder catches up.

        Returns False if stopped/cancelled before the frame was queued.
        """
        if self._reject_frames or self._closed:
            return False
        self._assert_dimensions(frame)
        while True:
            if self._reject_frames or self._closed:
                return False
            if interrupt_check is not None and interrupt_check():
                return False
            try:
                self.queue.put(frame, timeout=0.05)
                return True
            except queue.Full:
                continue

    def _encode(self, frame: np.ndarray) -> None:
        video_frame = av.VideoFrame.from_ndarray(frame, format="rgb24")
        if self._first_frame:
            with _STDERR_LOCK:
                stderr_fd = sys.stderr.fileno()
                old_stderr = os.dup(stderr_fd)
                devnull = os.open(os.devnull, os.O_WRONLY)
                os.dup2(devnull, stderr_fd)
                try:
                    packets = self.stream.encode(video_frame)
                    for packet in packets:
                        self.container.mux(packet)
                finally:
                    os.dup2(old_stderr, stderr_fd)
                    os.close(old_stderr)
                    os.close(devnull)
                    self._first_frame = False
        else:
            packets = self.stream.encode(video_frame)
            for packet in packets:
                self.container.mux(packet)

    def _writer_worker(self) -> None:
        while True:
            frame = self.queue.get()
            if frame is _STOP:
                try:
                    self._flush_stream()
                    if not self._closed:
                        self.container.close()
                        self._closed = True
                finally:
                    self._done.set()
                return
            try:
                self._assert_dimensions(frame)
                self._encode(frame)
            except Exception as exc:
                print(f"[VideoWriter] encode error for {self.output_path}: {exc}")
                try:
                    if not self._closed:
                        self.container.close()
                        self._closed = True
                finally:
                    self._done.set()
                return

    def _flush_stream(self) -> None:
        packets = self.stream.encode()
        for packet in packets:
            self.container.mux(packet)

    def stop(self) -> str:
        """Drain queued frames, flush encoder, close container."""
        if self._closed and self._done.is_set():
            return self.output_path
        self._reject_frames = True
        print(f"Waiting for video writer queue to drain ({self.queue.qsize()} frames)...")
        # Wait until there is room / queue has drained enough to accept sentinel.
        # Worker keeps consuming frames, so this unblocks even if queue was full.
        deadline = time.time() + 120.0
        while time.time() < deadline:
            try:
                self.queue.put(_STOP, timeout=0.2)
                break
            except queue.Full:
                if not self._thread.is_alive():
                    print(f"WARNING: video writer thread died for {self.output_path}")
                    break
        else:
            print(f"WARNING: could not enqueue stop sentinel for {self.output_path}")

        if not self._done.wait(timeout=120.0):
            print(f"WARNING: video writer drain timed out for {self.output_path}")
            if not self._closed:
                try:
                    self.container.close()
                except Exception:
                    pass
                self._closed = True
                self._done.set()
        self._thread.join(timeout=5.0)
        print("Video writer finished.")
        return self.output_path

    def cancel(self) -> None:
        """Immediately stops writing and deletes the output file."""
        self._reject_frames = True
        if not self._closed:
            try:
                self.container.close()
            except Exception:
                pass
            self._closed = True
            self._done.set()
        if os.path.exists(self.output_path):
            os.remove(self.output_path)

    def __del__(self) -> None:
        if not self._closed:
            try:
                self.container.close()
            except Exception:
                pass

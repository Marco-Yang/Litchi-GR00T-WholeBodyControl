"""VLA action → Orin Revo2 Modbus UDP bridge.

Training labels (``hand_type=revo2``) store 6D cmds in ``[0, 1000]`` padded to
Dex3-sized 7D. At inference, take the first 6 dims of each predicted hand vector
and publish the same JSON UDP packet that ``revo2_modbus_rx`` expects.

Also exposes last cmds as 7D ``[0, 1]`` state stand-ins so observation matches
how Revo2 episodes were recorded (no Dex3 DDS).
"""

from __future__ import annotations

import os
import sys
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

_MANUS_SIM = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "..",
        "..",
        "..",
        "Manus",
        "sim",
    )
)


def _ensure_manus_sim_path() -> str:
    if not os.path.isdir(_MANUS_SIM):
        raise FileNotFoundError(f"Manus sim dir missing: {_MANUS_SIM}")
    if _MANUS_SIM not in sys.path:
        sys.path.insert(0, _MANUS_SIM)
    return _MANUS_SIM


def joints_to_revo2_cmd(joints: Sequence[float] | np.ndarray) -> np.ndarray:
    """Map model ``*_hand_joints`` (6D or 7D padded) → Revo2 6D in ``[0, 1000]``.

    If the vector looks normalized (``max|v| <= 1.5``), scale by 1000 first —
    matches ``state_hand_vectors`` / accidental denorm gaps.
    """
    v = np.asarray(joints, dtype=np.float32).reshape(-1)
    if v.size < 6:
        v = np.pad(v, (0, 6 - v.size))
    v = v[:6].astype(np.float32, copy=False)
    if float(np.nanmax(np.abs(v))) <= 1.5:
        v = v * 1000.0
    return np.clip(v, 0.0, 1000.0).astype(np.float32)


class Revo2ActionBridge:
    """Publish predicted hand joints to Orin ``revo2_modbus_rx`` (:19500)."""

    DIM = 6

    def __init__(self, host: str, port: int = 19500):
        _ensure_manus_sim_path()
        from revo2_cmd_protocol import CmdUdpPublisher

        self.host = host
        self.port = int(port)
        self._pub = CmdUdpPublisher(host=host, port=self.port)
        self._last: Dict[str, np.ndarray] = {
            "left": np.zeros(self.DIM, dtype=np.float32),
            "right": np.zeros(self.DIM, dtype=np.float32),
        }
        self.open_all()
        print(f"[revo2-bridge] UDP → {host}:{self.port}")

    def send(
        self,
        left_hand_joints: Optional[Sequence[float] | np.ndarray],
        right_hand_joints: Optional[Sequence[float] | np.ndarray],
    ) -> Dict[str, np.ndarray]:
        cmds: Dict[str, np.ndarray] = {}
        if left_hand_joints is not None:
            cmds["left"] = joints_to_revo2_cmd(left_hand_joints)
            self._last["left"] = cmds["left"]
        if right_hand_joints is not None:
            cmds["right"] = joints_to_revo2_cmd(right_hand_joints)
            self._last["right"] = cmds["right"]
        if cmds:
            self._pub.send(cmds)
        return cmds

    def open_all(self) -> None:
        zeros = np.zeros(self.DIM, dtype=np.float32)
        self._last = {"left": zeros.copy(), "right": zeros.copy()}
        self._pub.send({"left": zeros, "right": zeros})

    def state_hand_vectors(self) -> Tuple[np.ndarray, np.ndarray]:
        """7D stand-ins for ``observation.state`` hand slices (cmd/1000, pad 0)."""

        def _one(side: str) -> np.ndarray:
            v6 = self._last[side].astype(np.float32) / 1000.0
            return np.concatenate([v6, np.zeros(1, dtype=np.float32)])

        return _one("left"), _one("right")

    def close(self) -> None:
        try:
            self.open_all()
        except Exception:
            pass
        try:
            self._pub.close()
        except Exception:
            pass

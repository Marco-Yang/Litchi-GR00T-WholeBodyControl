"""Revo2 hand teleop for Sonic VLA data collection.

Maps Pico triggers → recorded grasp trajectory (Manus JSON) → 6D cmds,
optionally UDP-publishes to Orin Modbus bridge.

Semantics (opposite of Dex3):
  released → open / record START
  pressed  → play traj → close / record STOP, hold
  release  → short blend back to open
"""

from __future__ import annotations

import os
import sys
from typing import Dict, Optional, Tuple

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


class Revo2TeleopHands:
    """Per-frame Revo2 command source driven by analog triggers."""

    JOINT_NAMES = ("Thumb", "ThumbAux", "Index", "Middle", "Ring", "Pinky")
    DIM = 6

    def __init__(
        self,
        pose_path: str,
        hz: float = 50.0,
        release_blend_s: float = 0.35,
        trigger_press: float = 0.45,
        trigger_release: float = 0.25,
        orin_host: Optional[str] = None,
        udp_port: int = 19500,
        send: bool = True,
    ):
        _ensure_manus_sim_path()
        from grasp_trajectory import TriggerTrajectoryPlayer, load_grasp_pose
        from revo2_cmd_protocol import CmdUdpPublisher

        if not os.path.isfile(pose_path):
            raise FileNotFoundError(
                f"Revo2 pose JSON missing: {pose_path}\n"
                "Record first via Manus scripts/run_real_revo2_record.sh"
            )

        self.pose_path = os.path.abspath(pose_path)
        self.hz = float(hz)
        self.trigger_press = float(trigger_press)
        self.trigger_release = float(trigger_release)
        pose = load_grasp_pose(self.pose_path)
        self.players: Dict[str, TriggerTrajectoryPlayer] = {}
        for side in ("left", "right"):
            if side in (pose.get("close") or {}) or side in (pose.get("trajectory") or {}):
                self.players[side] = TriggerTrajectoryPlayer(
                    pose, side, hz=self.hz, release_blend_s=release_blend_s
                )
        if not self.players:
            raise RuntimeError(f"no left/right trajectory in {self.pose_path}")

        self._latched = {"left": False, "right": False}
        self._last_cmd: Dict[str, np.ndarray] = {
            s: self.players[s].current().astype(np.float32) for s in self.players
        }
        self._pub = None
        if send and orin_host:
            self._pub = CmdUdpPublisher(host=orin_host, port=udp_port)
            self._pub.send({s: self._last_cmd[s] for s in self.players})
            print(
                f"[revo2] UDP → {orin_host}:{udp_port} | pose={self.pose_path}"
            )
        else:
            print(f"[revo2] record-only (no UDP) | pose={self.pose_path}")

    def _pressed(self, side: str, raw: float) -> bool:
        if self._latched[side]:
            if raw <= self.trigger_release:
                self._latched[side] = False
        else:
            if raw >= self.trigger_press:
                self._latched[side] = True
        return self._latched[side]

    @staticmethod
    def _raw_trigger(sonic_msg: Optional[dict], side: str) -> float:
        if not sonic_msg:
            return 0.0
        key = f"{side}_trigger"
        if key in sonic_msg and sonic_msg[key] is not None:
            return float(np.asarray(sonic_msg[key]).flat[0])
        return 0.0

    def step(self, sonic_msg: Optional[dict]) -> Dict[str, np.ndarray]:
        """Advance players from Pico triggers; return {left|right: float32[6]} in 0..1000."""
        cmds: Dict[str, np.ndarray] = {}
        for side, player in self.players.items():
            raw = self._raw_trigger(sonic_msg, side)
            trig = self._pressed(side, raw)
            cmd = player.step(trig).astype(np.float32)
            self._last_cmd[side] = cmd
            cmds[side] = cmd
        if self._pub is not None:
            self._pub.send(cmds)
        return cmds

    def teleop_vectors(
        self,
        cmds: Optional[Dict[str, np.ndarray]] = None,
        *,
        pad_to_dex3: bool = True,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Return hand vectors for ``teleop.*_hand_joints``.

        Raw Revo2 is 6D in ``[0, 1000]``. For stock GR00T ``UNITREE_G1_SONIC``
        (expects Dex3 7D), pad a trailing 0 → shape (7,).
        """
        src = cmds if cmds is not None else self._last_cmd

        def _one(side: str) -> np.ndarray:
            v = np.asarray(src.get(side, np.zeros(6)), dtype=np.float32).reshape(-1)[:6]
            if v.size < 6:
                v = np.pad(v, (0, 6 - v.size))
            if pad_to_dex3:
                v = np.concatenate([v, np.zeros(1, dtype=np.float32)])
            return v.astype(np.float32)

        return _one("left"), _one("right")

    def state_hand_vectors(
        self, cmds: Optional[Dict[str, np.ndarray]] = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """7D stand-ins for ``observation.state`` hand slices (Revo2/1000, pad 0)."""
        left, right = self.teleop_vectors(cmds, pad_to_dex3=True)
        return (left / 1000.0).astype(np.float64), (right / 1000.0).astype(np.float64)

    def close(self) -> None:
        if self._pub is not None:
            try:
                open_cmds = {
                    s: np.zeros(6, dtype=np.float32) for s in self.players
                }
                self._pub.send(open_cmds)
            except Exception:
                pass
            try:
                self._pub.close()
            except Exception:
                pass
            self._pub = None

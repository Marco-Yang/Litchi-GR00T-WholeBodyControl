"""3D episode-id labels floating above each G1 in the Isaac viewport.

Shows the LeRobot episode index (e.g. ``12`` from ``episode_000012``) above the
robot root so a multi-env dynamics check is easy to map back to the dataset.

The overlay SceneView has no manipulators so mouse orbit/zoom still goes to the
viewport camera (requires free-camera mode — see ``simple_isaac_viewer``).
"""

from __future__ import annotations

import re
from typing import Any

_EPISODE_RE = re.compile(r"episode[_\-]?(\d+)", re.IGNORECASE)


def episode_id_from_key(motion_key: Any) -> str:
    """Extract a short episode id from a motion lib key / filename."""
    name = str(motion_key)
    base = name.rsplit("/", 1)[-1]
    base = base.rsplit(".", 1)[0]
    m = _EPISODE_RE.search(base)
    if m:
        return str(int(m.group(1)))
    nums = re.findall(r"\d+", base)
    if nums:
        return str(int(nums[-1]))
    return base[:8] or "?"


def _has_robot(scene: Any) -> bool:
    if scene is None:
        return False
    try:
        keys = scene.keys() if hasattr(scene, "keys") else []
        if "robot" in keys:
            return True
    except Exception:  # noqa: BLE001
        pass
    try:
        _ = scene["robot"]
        return True
    except Exception:  # noqa: BLE001
        return False


def _unwrap_env(env: Any) -> Any:
    cur = env
    for _ in range(8):
        scene = getattr(cur, "scene", None)
        if _has_robot(scene):
            return cur
        nxt = getattr(cur, "unwrapped", None)
        if nxt is not None and nxt is not cur:
            cur = nxt
            continue
        nxt = getattr(cur, "env", None)
        if nxt is not None and nxt is not cur:
            cur = nxt
            continue
        break
    return getattr(env, "unwrapped", env)


def _motion_keys_for_envs(env: Any, num_envs: int) -> list[str]:
    """Best-effort per-env motion key list (length == num_envs)."""
    keys: list[Any] = []
    candidates = [env]
    for name in ("_motion_lib", "motion_lib", "motion_command"):
        obj = getattr(env, name, None)
        if obj is not None:
            candidates.append(obj)
            lib = getattr(obj, "motion_lib", None)
            if lib is not None:
                candidates.append(lib)
    # Also probe wrapped isaac env / command manager.
    isaac = _unwrap_env(env)
    if isaac is not env:
        candidates.append(isaac)
        try:
            cmd = isaac.command_manager.get_term("motion")
            candidates.append(cmd)
            if getattr(cmd, "motion_lib", None) is not None:
                candidates.append(cmd.motion_lib)
        except Exception:  # noqa: BLE001
            pass

    for obj in candidates:
        for attr in ("curr_motion_keys", "_curr_motion_keys"):
            val = getattr(obj, attr, None)
            if val is not None:
                keys = list(val)
                break
        if keys:
            break
        data_keys = getattr(obj, "_motion_data_keys", None)
        motion_ids = getattr(obj, "motion_ids", None)
        if data_keys is not None and motion_ids is not None:
            try:
                ids = motion_ids.detach().cpu().tolist()
                keys = [data_keys[int(i)] for i in ids]
                break
            except Exception:  # noqa: BLE001
                keys = []

    if len(keys) < num_envs:
        keys = list(keys) + [f"env_{i}" for i in range(len(keys), num_envs)]
    return [str(k) for k in keys[:num_envs]]


class EpisodeHeadLabels:
    """Viewport scene labels that track each env's robot root + Z offset."""

    def __init__(self, env: Any, *, z_offset: float = 1.35, font_size: int = 28):
        self._raw_env = env
        self.env = _unwrap_env(env)
        self.z_offset = float(z_offset)
        self.font_size = int(font_size)
        self._enabled = False
        self._labels: list[Any] = []
        self._transforms: list[Any] = []
        self._scene_view = None
        self._frame = None
        self._warned = False
        self._hud = None

        try:
            import omni.ui as ui
            from omni.ui import color as cl
            from omni.ui import scene as sc
            from omni.kit.viewport.utility import get_active_viewport, get_active_viewport_window
        except Exception as exc:  # noqa: BLE001
            print(f"[episode_labels] UI unavailable ({exc}); labels disabled")
            return

        self._ui = ui
        self._sc = sc
        self._cl = cl

        num_envs = int(getattr(self.env, "num_envs", 0) or getattr(env, "num_envs", 0) or 0)
        if num_envs <= 0:
            print("[episode_labels] num_envs=0; labels disabled")
            return

        keys = _motion_keys_for_envs(self._raw_env, num_envs)
        ids = [episode_id_from_key(k) for k in keys]
        print(f"[episode_labels] env→episode: {list(enumerate(ids))}")

        # Compact HUD (does not cover the 3D viewport center).
        try:
            self._hud = ui.Window(
                "Episode IDs",
                width=160,
                height=min(36 + 18 * num_envs, 420),
                flags=ui.WINDOW_FLAGS_NO_RESIZE | ui.WINDOW_FLAGS_NO_SCROLLBAR,
            )
            self._hud.position_x = 12
            self._hud.position_y = 40
            with self._hud.frame:
                with ui.VStack(spacing=2):
                    ui.Label("ep # above head", height=18, style={"color": 0xFF9EE6FF})
                    self._hud_labels: list[Any] = []
                    for i, eid in enumerate(ids):
                        lb = ui.Label(f"env{i}: {eid}", height=16, style={"color": 0xFF66FFE6})
                        self._hud_labels.append(lb)
        except Exception as exc:  # noqa: BLE001
            print(f"[episode_labels] HUD skipped: {exc}")
            self._hud_labels = []

        try:
            vp_window = get_active_viewport_window()
            if vp_window is None:
                print("[episode_labels] no active viewport window")
                return
            self._frame = vp_window.get_frame("data_check_episode_labels")
            with self._frame:
                # No manipulators / gestures → mouse events pass to viewport camera.
                self._scene_view = sc.SceneView(
                    aspect_ratio_policy=sc.AspectRatioPolicy.PRESERVE_ASPECT_FIT,
                    height=ui.Fraction(1),
                )
                with self._scene_view.scene:
                    for eid in ids:
                        xf = sc.Transform()
                        self._transforms.append(xf)
                        with xf:
                            label = sc.Label(
                                eid,
                                alignment=ui.Alignment.CENTER,
                                color=cl("#FFE66D"),
                                size=self.font_size,
                            )
                            self._labels.append(label)

            self._sync_camera(get_active_viewport())
            self._enabled = True
        except Exception as exc:  # noqa: BLE001
            print(f"[episode_labels] failed to create overlay: {exc}")
            self._enabled = False

    def _sync_camera(self, vp: Any = None) -> None:
        if self._scene_view is None:
            return
        try:
            from omni.kit.viewport.utility import get_active_viewport

            if vp is None:
                vp = get_active_viewport()
            if vp is not None and hasattr(vp, "viewport_api"):
                vp = vp.viewport_api
            if vp is None:
                return
            model = self._scene_view.model
            if model is None:
                return

            def _flat(mat: Any) -> list[float] | None:
                if mat is None:
                    return None
                try:
                    return [float(x) for row in mat for x in row]
                except Exception:  # noqa: BLE001
                    try:
                        return [float(mat[i][j]) for i in range(4) for j in range(4)]
                    except Exception:  # noqa: BLE001
                        return None

            view = _flat(getattr(vp, "view", None))
            proj = _flat(getattr(vp, "projection", None))
            if view is not None:
                model.set_floats("view", view)
            if proj is not None:
                model.set_floats("projection", proj)
        except Exception:  # noqa: BLE001
            pass

    def refresh_ids(self) -> None:
        if not self._labels and not getattr(self, "_hud_labels", None):
            return
        keys = _motion_keys_for_envs(self._raw_env, max(len(self._labels), 1))
        for label, key in zip(self._labels, keys, strict=False):
            try:
                label.text = episode_id_from_key(key)
            except Exception:  # noqa: BLE001
                pass
        for i, (hl, key) in enumerate(zip(getattr(self, "_hud_labels", []), keys, strict=False)):
            try:
                hl.text = f"env{i}: {episode_id_from_key(key)}"
            except Exception:  # noqa: BLE001
                pass

    def update(self) -> None:
        if not self._enabled and not getattr(self, "_hud_labels", None):
            return
        try:
            robot = self.env.scene["robot"]
            pos = robot.data.root_pos_w.detach()
            n = min(len(self._transforms), int(pos.shape[0])) if self._transforms else 0
            self.refresh_ids()
            self._sync_camera()
            sc = self._sc
            for i in range(n):
                x = float(pos[i, 0].item())
                y = float(pos[i, 1].item())
                z = float(pos[i, 2].item()) + self.z_offset
                self._transforms[i].transform = sc.Matrix44.get_translation_matrix(x, y, z)
        except Exception as exc:  # noqa: BLE001
            if not self._warned:
                print(f"[episode_labels] update failed: {exc}")
                self._warned = True

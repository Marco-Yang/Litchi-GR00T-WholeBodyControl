"""MuJoCo-like Isaac viewport: free camera + minimal chrome.

Isaac Lab's default eval path locks the viewport camera to ``asset_root``
(robot) every frame, which silently undoes orbit/zoom. This helper switches to
a static world camera, hides the IsaacLab side panel, and fills the viewport.
"""

from __future__ import annotations

from typing import Any, Sequence


def _isaac_env(env: Any) -> Any:
    """Unwrap ManagerEnvWrapper / gym layers down to ManagerBased*Env."""
    cur = env
    for _ in range(8):
        if hasattr(cur, "viewport_camera_controller") and hasattr(cur, "scene"):
            return cur
        if hasattr(cur, "scene") and hasattr(cur, "sim"):
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
    return cur


def enable_free_camera(
    env: Any,
    *,
    eye: Sequence[float] | None = None,
    lookat: Sequence[float] | None = None,
) -> bool:
    """Stop per-frame robot tracking so the user can orbit / pan / zoom."""
    isaac = _isaac_env(env)
    cam = getattr(isaac, "viewport_camera_controller", None)
    if cam is None:
        print("[simple_viewer] no viewport_camera_controller; skip free camera")
        return False

    try:
        # ``world`` origin disables the post-update tracking callback body.
        cam.update_view_to_world()
        if eye is not None or lookat is not None:
            cam.update_view_location(
                eye=list(eye) if eye is not None else None,
                lookat=list(lookat) if lookat is not None else None,
            )
        # Keep ManagerEnvWrapper's F-key toggle state consistent if present.
        wrapper = env
        if hasattr(wrapper, "viewer_focused"):
            wrapper.viewer_focused = False
        print(
            "[simple_viewer] free camera ON (world). "
            "Orbit/zoom/pan should work — Isaac default: Alt+LMB orbit, "
            "scroll zoom, Alt+MMB pan (or Magellan/gamepad if remapped)."
        )
        return True
    except Exception as exc:  # noqa: BLE001
        print(f"[simple_viewer] free camera failed: {exc}")
        return False


def simplify_ui(env: Any | None = None) -> None:
    """Hide IsaacLab control dock and expand the viewport (MuJoCo-like)."""
    try:
        import carb
        import omni.ui as ui
    except Exception as exc:  # noqa: BLE001
        print(f"[simple_viewer] UI modules unavailable: {exc}")
        return

    settings = carb.settings.get_settings()
    # Fill viewport into the main window like a simple viewer.
    for path, value in (
        ("/app/viewport/defaults/fillViewport", True),
        ("/app/viewport/grid/enabled", True),
        ("/app/viewport/camShowSpeedOnStart", False),
        # Quiet some editor chrome; harmless if keys are absent.
        ("/app/window/showStatusBar", False),
        ("/persistent/app/viewport/displayOptions", 0),
    ):
        try:
            settings.set(path, value)
        except Exception:  # noqa: BLE001
            pass

    # Hide the IsaacLab task panel (RIGHT_TOP dock) if present.
    try:
        isaac = _isaac_env(env) if env is not None else None
        window_mgr = getattr(isaac, "window", None) if isaac is not None else None
        ui_window = getattr(window_mgr, "ui_window", None) if window_mgr is not None else None
        if ui_window is not None:
            ui_window.visible = False
            print("[simple_viewer] hid IsaacLab side panel")
    except Exception as exc:  # noqa: BLE001
        print(f"[simple_viewer] could not hide IsaacLab panel: {exc}")

    # Hide common editor docks if they exist (best-effort; names vary by Kit).
    hide_titles = (
        "Stage",
        "Property",
        "Layer",
        "Render Settings",
        "Console",
        "Content",
        "IsaacLab",
    )
    try:
        for title in hide_titles:
            try:
                ui.Workspace.show_window(title, False)
            except Exception:  # noqa: BLE001
                pass
    except Exception:  # noqa: BLE001
        pass

    print("[simple_viewer] viewport chrome simplified")


def setup_simple_viewer(
    env: Any,
    *,
    eye: Sequence[float] | None = None,
    lookat: Sequence[float] | None = None,
) -> None:
    """Apply free-camera + minimal UI for dynamics inspection."""
    simplify_ui(env)
    enable_free_camera(env, eye=eye, lookat=lookat)

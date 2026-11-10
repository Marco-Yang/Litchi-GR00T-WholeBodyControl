"""Force NVIDIA GPU OpenGL/EGL for MuJoCo (avoid Mesa swrast CPU burn).

Import this module before any mujoco / OpenGL import.

On Intel+NVIDIA hybrid displays, plain `__GLX_VENDOR_LIBRARY_NAME=nvidia`
fails to create a window; PRIME offload is required (see GR00T troubleshooting
§12 / issue #25). Override with MUJOCO_FORCE_CPU_GL=1 for software GL.
"""

from __future__ import annotations

import os


def ensure_gpu_gl() -> None:
    if os.environ.get("MUJOCO_FORCE_CPU_GL", "").strip().lower() in (
        "1",
        "true",
        "yes",
        "on",
    ):
        return

    # Hybrid iGPU display + NVIDIA dGPU (local workstation default).
    os.environ.setdefault("__NV_PRIME_RENDER_OFFLOAD", "1")
    os.environ.setdefault("__GLX_VENDOR_LIBRARY_NAME", "nvidia")
    os.environ.setdefault("__VK_LAYER_NV_optimus", "NVIDIA_only")
    # Headless MUJOCO_GL=egl path.
    nvidia_egl = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
    if os.path.isfile(nvidia_egl):
        os.environ.setdefault("__EGL_VENDOR_LIBRARY_FILENAMES", nvidia_egl)


ensure_gpu_gl()

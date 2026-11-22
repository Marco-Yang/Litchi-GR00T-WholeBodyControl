#!/usr/bin/env python3
"""Dual-buffer PolicyServer supervisor for near-instant skill switching.

Always keeps **two** Isaac-GR00T PolicyServers resident (active + standby):

* Boot: load skill1 on ``policy_port`` (active), skill2 on ``standby_port``.
* Switch to standby skill: promote instantly (client reconnects to that port),
  then background-kill the old active and warm-start the cyclic next skill.
* Switch to a cold skill: block-load onto the free slot, then promote.

VRAM: one N1.7 server ≈6.8GB on 5090; two ≈14GB — fits in 32GB.

Control protocol (ZMQ REP, default ``tcp://*:5590``):
  request: ``{"cmd":"switch","skill_id":"2"}`` / ``skill:2`` / ``status``
  reply:   includes ``policy_port`` the client must use after the switch.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from typing import Any

import tyro
import zmq

from gear_sonic.utils.inference.skill_registry import SkillRegistry, load_skill_registry


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


@dataclass
class SupervisorConfig:
    skills_config: str = str(
        _repo_root() / "gear_sonic/config/multi_skill_nortc_skills.json"
    )
    litchi_root: str = str(_repo_root() / ".deps/Isaac-GR00T")
    python_bin: str = str(_repo_root() / ".venv_inference/bin/python")
    device: str = "cuda:0"
    embodiment_tag: str = "UNITREE_G1_SONIC"
    policy_port: int | None = None
    standby_port: int | None = None
    supervisor_port: int | None = None
    initial_skill_id: str | None = None
    ready_timeout_s: float = 300.0
    kill_timeout_s: float = 20.0


@dataclass
class _Slot:
    """One PolicyServer process occupying a TCP port."""

    port: int
    skill_id: str | None = None
    proc: subprocess.Popen | None = None
    log_f: Any = None

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None


class DualBufferSupervisor:
    def __init__(self, config: SupervisorConfig, registry: SkillRegistry):
        self.config = config
        self.registry = registry
        self.port_a = int(config.policy_port or registry.policy_port)
        self.port_b = int(config.standby_port or registry.standby_port)
        if self.port_a == self.port_b:
            raise ValueError("policy_port and standby_port must differ")
        self.supervisor_port = int(config.supervisor_port or registry.supervisor_port)
        self.litchi_root = Path(config.litchi_root).expanduser().resolve()
        self.python_bin = Path(config.python_bin).expanduser()
        if not self.python_bin.is_absolute():
            self.python_bin = (Path.cwd() / self.python_bin).resolve()
        server_py = self.litchi_root / "gr00t/eval/run_gr00t_server.py"
        if not server_py.is_file():
            raise FileNotFoundError(f"Missing PolicyServer script: {server_py}")
        if not self.python_bin.exists():
            raise FileNotFoundError(f"Missing python: {self.python_bin}")
        self.server_py = server_py
        print(f"[supervisor] python_bin={self.python_bin}", flush=True)

        self.active = _Slot(port=self.port_a)
        self.standby = _Slot(port=self.port_b)
        self._bg_lock = threading.Lock()
        self._bg_thread: threading.Thread | None = None
        self._preflight_done = False

    # ------------------------------------------------------------------ env
    def _child_env(self) -> dict[str, str]:
        env = os.environ.copy()
        for key in (
            "PYTHONHOME",
            "PYTHONPATH",
            "PYTHONSTARTUP",
            "PYTHONNOUSERSITE",
            "CONDA_PREFIX",
            "CONDA_DEFAULT_ENV",
            "CONDA_PYTHON_EXE",
            "CONDA_PROMPT_MODIFIER",
            "_CE_M",
            "_CE_CONDA",
        ):
            env.pop(key, None)
        venv_dir = self.python_bin.parent.parent
        venv_bin = self.python_bin.parent
        env["VIRTUAL_ENV"] = str(venv_dir)
        env["PATH"] = f"{venv_bin}:{env.get('PATH', '')}"
        env.setdefault("HF_HOME", str(Path.home() / ".cache/huggingface"))
        snap = (
            Path(env["HF_HOME"])
            / "hub/models--nvidia--Cosmos-Reason2-2B"
            / "snapshots/9ce19a195e423419c349abfc86fd07178b230561"
        )
        env.setdefault("GROOT_BACKBONE_PATH", str(snap))
        env["HF_HUB_OFFLINE"] = env.get("HF_HUB_OFFLINE", "1")
        env["TRANSFORMERS_OFFLINE"] = env.get("TRANSFORMERS_OFFLINE", "1")
        env["GROOT_HF_LOCAL_FIRST"] = "1"
        env["NO_ALBUMENTATIONS_UPDATE"] = "1"
        env["PYTHONPATH"] = str(self.litchi_root)
        return env

    def _preflight_python(self) -> None:
        if self._preflight_done:
            return
        check = subprocess.run(
            [
                str(self.python_bin),
                "-c",
                "import numpy, torch; print('preflight OK', numpy.__version__, torch.__version__)",
            ],
            env=self._child_env(),
            capture_output=True,
            text=True,
            check=False,
        )
        if check.returncode != 0:
            raise RuntimeError(
                "PolicyServer python preflight failed (numpy/torch import). "
                f"python={self.python_bin}\nstdout:\n{check.stdout}\nstderr:\n{check.stderr}"
            )
        print(f"[supervisor] {check.stdout.strip()}", flush=True)
        self._preflight_done = True

    # ----------------------------------------------------------- process ops
    def _stop_slot(self, slot: _Slot, *, wait: bool = True) -> None:
        proc = slot.proc
        slot.proc = None
        skill = slot.skill_id
        slot.skill_id = None
        log_f = slot.log_f
        slot.log_f = None
        if proc is None:
            if log_f is not None:
                try:
                    log_f.close()
                except Exception:
                    pass
            return
        if proc.poll() is None:
            print(
                f"[supervisor] stopping skill={skill} port={slot.port} pid={proc.pid}",
                flush=True,
            )
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            if wait:
                deadline = time.time() + float(self.config.kill_timeout_s)
                while time.time() < deadline and proc.poll() is None:
                    time.sleep(0.2)
                if proc.poll() is None:
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    try:
                        proc.wait(timeout=5)
                    except Exception:
                        pass
        if log_f is not None:
            try:
                log_f.close()
            except Exception:
                pass

    def _wait_ready(self, slot: _Slot) -> bool:
        from gr00t.policy.server_client import PolicyClient

        client = PolicyClient(host="127.0.0.1", port=slot.port, timeout_ms=2000)
        deadline = time.time() + float(self.config.ready_timeout_s)
        try:
            while time.time() < deadline:
                if slot.proc is not None and slot.proc.poll() is not None:
                    print(
                        f"[supervisor] PolicyServer exited early "
                        f"skill={slot.skill_id} code={slot.proc.returncode} "
                        f"(see /tmp/gr00t_policy_server_skill{slot.skill_id}.log)",
                        flush=True,
                    )
                    return False
                if client.ping():
                    return True
                time.sleep(1.0)
            return False
        finally:
            client.close()

    def _spawn(self, slot: _Slot, skill_id: str) -> bool:
        """Start skill on ``slot`` (slot must be empty). Returns ready bool."""
        self._preflight_python()
        skill = self.registry.get(skill_id)
        self._stop_slot(slot, wait=True)
        cmd = [
            str(self.python_bin),
            str(self.server_py),
            "--model-path",
            str(skill.checkpoint),
            "--embodiment-tag",
            self.config.embodiment_tag,
            "--device",
            self.config.device,
            "--port",
            str(slot.port),
        ]
        log_path = Path("/tmp") / f"gr00t_policy_server_skill{skill.id}_p{slot.port}.log"
        log_f = open(log_path, "w", encoding="utf-8")
        print(
            f"[supervisor] spawning skill={skill.id} ({skill.label}) "
            f"port={slot.port} log={log_path}",
            flush=True,
        )
        proc = subprocess.Popen(
            cmd,
            cwd=str(self.litchi_root),
            env=self._child_env(),
            start_new_session=True,
            stdout=log_f,
            stderr=subprocess.STDOUT,
        )
        slot.proc = proc
        slot.log_f = log_f
        slot.skill_id = skill.id
        ready = self._wait_ready(slot)
        if not ready:
            self._stop_slot(slot, wait=True)
            return False
        print(
            f"[supervisor] READY skill={skill.id} ({skill.label}) port={slot.port}",
            flush=True,
        )
        return True

    def _reply_for(self, skill_id: str, *, promoted: bool, warm: bool) -> dict:
        skill = self.registry.get(skill_id)
        prefetch_id = self.registry.next_id(skill_id)
        return {
            "ok": True,
            "skill_id": skill.id,
            "name": skill.name,
            "label": skill.label,
            "checkpoint": str(skill.checkpoint),
            "prompt": skill.prompt,
            "policy_host": "127.0.0.1",
            "policy_port": self.active.port,
            "standby_skill_id": self.standby.skill_id,
            "standby_port": self.standby.port,
            "prefetch_skill_id": prefetch_id,
            "ready": True,
            "promoted": promoted,
            "warm": warm,
        }

    # ---------------------------------------------------------- dual-buffer
    def boot(self) -> None:
        initial = self.config.initial_skill_id or self.registry.default_skill_id
        next_id = self.registry.next_id(initial)
        print(
            f"[supervisor] dual-buffer boot active={initial}@{self.active.port} "
            f"standby={next_id}@{self.standby.port}",
            flush=True,
        )
        if not self._spawn(self.active, initial):
            raise RuntimeError(f"Failed to boot active skill={initial}")
        if not self._spawn(self.standby, next_id):
            raise RuntimeError(f"Failed to boot standby skill={next_id}")
        print(
            f"[supervisor] both ready — active={self.active.skill_id}:{self.active.port} "
            f"standby={self.standby.skill_id}:{self.standby.port}",
            flush=True,
        )

    def switch(self, skill_id: str) -> dict:
        skill_id = str(skill_id).strip()
        skill = self.registry.get(skill_id)  # validate

        # Already active.
        if self.active.skill_id == skill_id and self.active.running():
            return self._reply_for(skill_id, promoted=False, warm=True)

        # Wait for any in-flight warm-start to finish before mutating slots.
        if self._bg_thread is not None and self._bg_thread.is_alive():
            print("[supervisor] waiting for background warm-start to finish...", flush=True)
            self._bg_thread.join(timeout=float(self.config.ready_timeout_s) + 30)

        # Warm promote: requested skill is on standby.
        if self.standby.skill_id == skill_id and self.standby.running():
            old_active = self.active
            print(
                f"[supervisor] WARM promote skill={skill_id} "
                f"port {self.standby.port} (was active {old_active.skill_id}@"
                f"{old_active.port})",
                flush=True,
            )
            # Swap roles.
            self.active, self.standby = self.standby, self.active
            # Kill former active (now in standby slot) in background, then
            # warm-start cyclic next onto that freed port.
            former = self.standby

            def _retire_and_prefetch() -> None:
                with self._bg_lock:
                    self._stop_slot(former, wait=True)
                    next_id = self.registry.next_id(skill_id)
                    print(
                        f"[supervisor] background warm-start next skill={next_id} "
                        f"on port={former.port}",
                        flush=True,
                    )
                    ok = self._spawn(former, next_id)
                    # former is self.standby after swap
                    if not ok:
                        print(
                            f"[supervisor] WARNING: failed to warm-start skill={next_id}",
                            flush=True,
                        )

            self._bg_thread = threading.Thread(
                target=_retire_and_prefetch, daemon=True, name="retire-prefetch"
            )
            self._bg_thread.start()
            return self._reply_for(skill_id, promoted=True, warm=True)

        # Cold path: load requested skill onto standby (blocking), then promote.
        print(
            f"[supervisor] COLD load skill={skill_id} onto standby port={self.standby.port}",
            flush=True,
        )
        if not self._spawn(self.standby, skill_id):
            return {
                "ok": False,
                "skill_id": skill_id,
                "ready": False,
                "error": f"cold load of skill={skill_id} failed",
            }
        old_active = self.active
        self.active, self.standby = self.standby, self.active

        def _retire_and_prefetch_cold() -> None:
            with self._bg_lock:
                self._stop_slot(old_active, wait=True)
                # After swap, standby slot object is old_active
                next_id = self.registry.next_id(skill_id)
                ok = self._spawn(old_active, next_id)
                if not ok:
                    print(
                        f"[supervisor] WARNING: failed to warm-start skill={next_id}",
                        flush=True,
                    )

        self._bg_thread = threading.Thread(
            target=_retire_and_prefetch_cold, daemon=True, name="cold-retire-prefetch"
        )
        self._bg_thread.start()
        return self._reply_for(skill_id, promoted=True, warm=False)

    def status(self) -> dict:
        return {
            "ok": True,
            "mode": "dual-buffer",
            "active_skill_id": self.active.skill_id,
            "active_port": self.active.port,
            "active_running": self.active.running(),
            "standby_skill_id": self.standby.skill_id,
            "standby_port": self.standby.port,
            "standby_running": self.standby.running(),
            "prompt": (
                None
                if self.active.skill_id is None
                else self.registry.get(self.active.skill_id).prompt
            ),
        }

    def handle_request(self, raw: str) -> dict:
        raw = (raw or "").strip()
        if not raw:
            return {"ok": False, "error": "empty request"}
        try:
            if raw[0] == "{":
                msg = json.loads(raw)
            elif raw.startswith("skill:"):
                msg = {"cmd": "switch", "skill_id": raw.split(":", 1)[1].strip()}
            elif raw in self.registry.skills:
                msg = {"cmd": "switch", "skill_id": raw}
            elif raw == "status":
                msg = {"cmd": "status"}
            else:
                return {"ok": False, "error": f"unrecognized request: {raw!r}"}
        except json.JSONDecodeError as exc:
            return {"ok": False, "error": f"bad json: {exc}"}

        cmd = str(msg.get("cmd") or "switch").strip().lower()
        if cmd == "status":
            return self.status()
        if cmd in ("switch", "load", "set"):
            skill_id = str(msg.get("skill_id") or msg.get("id") or "").strip()
            if not skill_id:
                return {"ok": False, "error": "missing skill_id"}
            return self.switch(skill_id)
        return {"ok": False, "error": f"unknown cmd={cmd!r}"}

    def serve_forever(self) -> None:
        print(
            f"[supervisor] skills={[s.as_dict() for s in self.registry.skills.values()]}",
            flush=True,
        )
        self.boot()
        ctx = zmq.Context()
        sock = ctx.socket(zmq.REP)
        sock.bind(f"tcp://*:{self.supervisor_port}")
        print(
            f"[supervisor] control REP on tcp://*:{self.supervisor_port} "
            f"(active {self.active.port} / standby {self.standby.port})",
            flush=True,
        )
        try:
            while True:
                raw = sock.recv_string()
                print(f"[supervisor] request: {raw!r}", flush=True)
                try:
                    reply = self.handle_request(raw)
                except Exception as exc:  # noqa: BLE001
                    reply = {"ok": False, "error": str(exc)}
                sock.send_string(json.dumps(reply, ensure_ascii=False))
        except KeyboardInterrupt:
            print("\n[supervisor] shutting down", flush=True)
        finally:
            if self._bg_thread is not None and self._bg_thread.is_alive():
                self._bg_thread.join(timeout=5)
            self._stop_slot(self.active, wait=True)
            self._stop_slot(self.standby, wait=True)
            sock.close(0)
            ctx.term()


def main(config: SupervisorConfig) -> None:
    registry = load_skill_registry(config.skills_config)
    sys.path.insert(0, str(Path(config.litchi_root).expanduser().resolve()))
    DualBufferSupervisor(config, registry).serve_forever()


if __name__ == "__main__":
    main(tyro.cli(SupervisorConfig))

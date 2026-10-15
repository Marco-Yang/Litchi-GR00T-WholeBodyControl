"""A3 ZMQ relay route ("deploy on the operator PC, relay on the MDU"): launcher arguments and guards, and the Python side of the wire
format (scripts/a3_relay_tool.py) against the golden frames the C++ unit test (test_a3_relay_wire.cpp) also reads."""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import shutil
import struct
import subprocess

import pytest

REPO = Path(__file__).resolve().parents[1]
DEPLOY = REPO / "gear_sonic_deploy"
GOLDEN = DEPLOY / "src/g1/g1_deploy_onnx_ref/unit_tests/golden"


def _tool():
    spec = importlib.util.spec_from_file_location("a3_relay_tool", DEPLOY / "scripts/a3_relay_tool.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ---- wire format -------------------------------------------------------------------------------------------------------------
def test_python_frames_are_byte_identical_to_the_golden_files_the_cpp_test_reads():
    tool = _tool()
    assert tool.golden_command() == (GOLDEN / "a3_relay_golden_cmd.bin").read_bytes()
    assert tool.golden_state() == (GOLDEN / "a3_relay_golden_state.bin").read_bytes()
    assert len(tool.golden_command()) == tool.CMD_BYTES == 1272
    assert len(tool.golden_state()) == tool.STATE_BYTES == 976


def test_command_and_state_round_trip():
    tool = _tool()
    c = tool.unpack_command(tool.golden_command())
    assert c["session"] == 0x1122334455667788 and c["seq"] == 4242 and c["kp"][3] == 4.0 and c["q_des"][30] == pytest.approx(0.30)
    s = tool.unpack_state(tool.golden_state())
    assert tool.MODES[s["mode"]] == "HOLD" and tool.REJECTS[s["last_reject"]] == "latched" and s["flags"] == 0b111
    assert s["q"][5] == pytest.approx(0.10) and s["pelvis"][0] == 0.5 and s["torso"][9] == 9.5


def test_frames_with_the_wrong_size_or_magic_are_refused():
    tool = _tool()
    with pytest.raises(ValueError):
        tool.unpack_command(tool.golden_command()[:-1])
    with pytest.raises(ValueError):
        tool.unpack_state(tool.golden_command())  # a command frame is not a state frame
    bad = bytearray(tool.golden_state())
    bad[0:4] = b"XXXX"
    with pytest.raises(ValueError):
        tool.unpack_state(bytes(bad))
    bad = bytearray(tool.golden_command())
    struct.pack_into("<H", bad, 6, 29)  # dof
    with pytest.raises(ValueError):
        tool.unpack_command(bytes(bad))


# ---- launcher ----------------------------------------------------------------------------------------------------------------
pytestmark_bash = pytest.mark.skipif(shutil.which("bash") is None, reason="needs bash")


def _fake_tree(tmp_path: Path) -> Path:
    root = tmp_path / "gear_sonic_deploy"
    (root / "scripts").mkdir(parents=True)
    for name in ("deploy_a3.sh", "a3_operator_chain.sh", "a3_real_preflight.sh"):
        shutil.copy(DEPLOY / "scripts" / name, root / "scripts" / name)
    arch = "rockchip" if os.uname().machine in ("aarch64", "arm64") else "x86_64"
    pkg = root / "dist" / f"a3_deploy_{arch}"
    pkg.mkdir(parents=True)
    run = pkg / "run_a3.sh"
    run.write_text('#!/usr/bin/env bash\necho "ARGS: $* TRANSPORT=${A3_TRANSPORT:-} DOMAIN=${ROS_DOMAIN_ID:-}" > "${A3_TEST_OUT}"\n')
    run.chmod(0o755)
    models = root / "assets/a3_runtime/models/035_step200000"
    models.mkdir(parents=True)
    (models / "m.onnx").write_bytes(b"")
    return root


def _launch(root: Path, args: list[str], tmp_path: Path):
    out = tmp_path / "args.txt"
    env = {**os.environ, "A3_TEST_OUT": str(out)}
    env.pop("A3_ASSUME_YES", None)
    for var in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY"):
        env.pop(var, None)
    done = subprocess.run(["bash", str(root / "scripts/deploy_a3.sh"), *args], stdin=subprocess.DEVNULL, capture_output=True, text=True,
                          env=env, timeout=60)
    return done, (out.read_text().strip() if out.exists() else "")


@pytestmark_bash
def test_robot_link_zmq_passes_the_relay_endpoint_and_needs_no_ros_domain(tmp_path):
    root = _fake_tree(tmp_path)
    done, args = _launch(root, ["--robot", "a3_ultra", "sim", "--encoder", "a3-fast", "--teleop-input", "zmq", "--robot-link", "zmq",
                                "--relay-host", "127.0.0.1", "--relay-state-port", "6570", "--relay-command-port", "6571"], tmp_path)
    assert done.returncode == 0, done.stderr
    assert "--robot-link zmq --relay-host 127.0.0.1 --relay-state-port 6570 --relay-command-port 6571" in args
    assert "--teleop-input zmq" in args and "TRANSPORT= DOMAIN=" in args  # no ROS 2 transport / domain forced for this link
    # the default link stays AimRT: no relay arguments
    done, args = _launch(root, ["--robot", "a3_ultra", "sim", "--encoder", "a3-fast"], tmp_path)
    assert done.returncode == 0 and "--robot-link" not in args and "--relay-host" not in args and "DOMAIN=87" in args


@pytestmark_bash
def test_robot_link_zmq_is_refused_without_a_relay_host_with_ta_teleop_or_with_remote(tmp_path):
    root = _fake_tree(tmp_path)
    done, args = _launch(root, ["--robot", "a3_ultra", "sim", "--teleop-input", "zmq", "--robot-link", "zmq"], tmp_path)
    assert done.returncode == 2 and args == "" and "--relay-host" in done.stderr
    done, args = _launch(root, ["--robot", "a3_ultra", "sim", "--robot-link", "zmq", "--relay-host", "127.0.0.1"], tmp_path)  # --teleop-input ta
    assert done.returncode == 2 and args == "" and "--teleop-input zmq" in done.stderr
    done, args = _launch(root, ["--robot", "a3_ultra", "real", "--teleop-input", "zmq", "--robot-link", "zmq", "--relay-host", "10.1.1.1",
                                "--remote", "--yes"], tmp_path)
    assert done.returncode == 2 and args == "" and "pick one" in done.stderr
    done, args = _launch(root, ["--robot", "a3_ultra", "sim", "--robot-link", "udp"], tmp_path)
    assert done.returncode == 2 and "--robot-link must be aimrt or zmq" in done.stderr


@pytestmark_bash
def test_real_robot_start_over_the_relay_still_needs_a_confirmation_and_says_who_owns_the_failsafe(tmp_path):
    root = _fake_tree(tmp_path)
    base = ["--robot", "a3_ultra", "real", "--teleop-input", "zmq", "--robot-link", "zmq", "--relay-host", "10.42.10.12"]
    done, args = _launch(root, base, tmp_path)  # stdin is not a terminal, no --yes
    assert done.returncode == 2 and args == "" and "ZMQ RELAY 10.42.10.12" in done.stdout and "--yes" in done.stderr
    assert "damping" in done.stdout and "Wi-Fi is not acceptable" in done.stdout
    done, args = _launch(root, [*base, "--yes"], tmp_path)
    assert done.returncode == 0 and "--auto-pd-stand" not in args and "--robot-link zmq --relay-host 10.42.10.12" in args

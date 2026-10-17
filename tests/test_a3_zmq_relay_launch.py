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
    run.write_text('#!/usr/bin/env bash\necho "ARGS: $* TRANSPORT=${A3_TRANSPORT:-} DOMAIN=${ROS_DOMAIN_ID:-} LDLP=${LD_LIBRARY_PATH:-}" > "${A3_TEST_OUT}"\n')
    run.chmod(0o755)
    models = root / "assets/a3_runtime/models/035_step200000"
    models.mkdir(parents=True)
    (models / "m.onnx").write_bytes(b"")
    return root


def _launch(root: Path, args: list[str], tmp_path: Path, env_extra: dict | None = None):
    out = tmp_path / "args.txt"
    env = {**os.environ, "A3_TEST_OUT": str(out), **(env_extra or {})}
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


# ---- --telecom: one switch for the three routes ------------------------------------------------------------------------------
@pytestmark_bash
def test_telecom_zmq_is_the_same_route_as_robot_link_zmq(tmp_path):
    root = _fake_tree(tmp_path)
    legacy, a_legacy = _launch(root, ["--robot", "a3_ultra", "sim", "--teleop-input", "zmq", "--robot-link", "zmq", "--relay-host", "127.0.0.1"], tmp_path)
    new, a_new = _launch(root, ["--robot", "a3_ultra", "sim", "--teleop-input", "zmq", "--telecom", "zmq", "--relay-host", "127.0.0.1"], tmp_path)
    assert legacy.returncode == 0 and new.returncode == 0, new.stderr
    assert a_new == a_legacy and "--robot-link zmq --relay-host 127.0.0.1" in a_new and "TRANSPORT= DOMAIN=" in a_new
    assert "telecom=zmq" in new.stdout
    new, a_new = _launch(root, ["--robot", "a3_ultra", "sim", "--teleop-input", "zmq", "--telecom=zmq", "--relay-host", "127.0.0.1",
                                "--relay-state-port", "6570", "--relay-command-port", "6571"], tmp_path)
    assert new.returncode == 0 and "--relay-state-port 6570 --relay-command-port 6571" in a_new


@pytestmark_bash
def test_telecom_ros2_is_the_simulators_transport_and_the_default_for_sim(tmp_path):
    root = _fake_tree(tmp_path)
    done, args = _launch(root, ["--robot", "a3_ultra", "sim", "--telecom", "ros2"], tmp_path)
    default, a_default = _launch(root, ["--robot", "a3_ultra", "sim"], tmp_path)
    assert done.returncode == 0 and default.returncode == 0
    assert args == a_default and "TRANSPORT=ros2" in args and "DOMAIN=87" in args and "--robot-link" not in args
    assert "telecom=ros2" in done.stdout and "telecom=ros2" in default.stdout


@pytestmark_bash
def test_telecom_iceoryx_is_the_on_robot_route_and_cannot_reach_the_simulator(tmp_path):
    root = _fake_tree(tmp_path)
    done, args = _launch(root, ["--robot", "a3_ultra", "sim", "--telecom", "iceoryx"], tmp_path)
    assert done.returncode == 2 and args == "" and "speaks ROS 2" in done.stderr
    done, args = _launch(root, ["--robot", "a3_ultra", "real", "--telecom", "iceoryx", "--yes"], tmp_path)
    assert done.returncode == 0 and "TRANSPORT=iceoryx" in args and "--robot-link" not in args and "telecom=iceoryx" in done.stdout
    done, args = _launch(root, ["--robot", "a3_ultra", "real", "--yes"], tmp_path)  # no flag on 'real': the site's A3_TRANSPORT decides, as before
    assert done.returncode == 0 and "telecom=site default" in done.stdout


@pytestmark_bash
def test_telecom_is_validated_and_must_agree_with_the_older_flags(tmp_path):
    root = _fake_tree(tmp_path)
    done, args = _launch(root, ["--robot", "a3_ultra", "sim", "--telecom", "udp"], tmp_path)
    assert done.returncode == 2 and args == "" and "--telecom must be iceoryx, ros2 or zmq" in done.stderr
    done, args = _launch(root, ["--robot", "a3_ultra", "sim", "--teleop-input", "zmq", "--telecom", "zmq"], tmp_path)  # no relay host
    assert done.returncode == 2 and args == "" and "--relay-host" in done.stderr
    done, args = _launch(root, ["--robot", "a3_ultra", "sim", "--telecom", "zmq", "--relay-host", "127.0.0.1"], tmp_path)  # --teleop-input ta
    assert done.returncode == 2 and args == "" and "--teleop-input zmq" in done.stderr
    done, args = _launch(root, ["--robot", "a3_ultra", "real", "--telecom", "ros2", "--robot-link", "zmq", "--relay-host", "10.1.1.1", "--yes"], tmp_path)
    assert done.returncode == 2 and args == "" and "contradicts --robot-link zmq" in done.stderr
    done, args = _launch(root, ["--robot", "a3_ultra", "real", "--telecom", "zmq", "--remote", "--relay-host", "10.1.1.1", "--yes"], tmp_path)
    assert done.returncode == 2 and args == "" and "contradicts --remote" in done.stderr
    done, args = _launch(root, ["--robot", "a3_ultra", "--telecom", "ros2"], tmp_path)  # neither sim nor real
    assert done.returncode == 2 and "needs a target" in done.stderr


def test_snapshot_reports_every_slot_at_full_precision_from_a_live_stream(capsys):
    zmq = pytest.importorskip("zmq")
    import threading
    import time

    tool = _tool()
    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    port = pub.bind_to_random_port("tcp://127.0.0.1")
    stop = threading.Event()

    def publish():
        seq = 0
        while not stop.is_set():
            q = [0.02 * i + 1e-4 * (seq % 3) for i in range(tool.DOF)]  # a tiny wobble: std must be > 0 and not rounded away
            pub.send(tool.pack_state(0xAB, seq, 0, 0, 0b11, 0, 0, 0, 0, 1_500_000, 1_000_000, 0, q, [0.0] * tool.DOF, [0.5] * tool.DOF,
                                     [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81]))
            seq += 1
            time.sleep(0.005)

    t = threading.Thread(target=publish, daemon=True)
    t.start()
    try:
        time.sleep(0.2)
        rc = tool.main(["snapshot", "--host", "127.0.0.1", "--state-port", str(port), "--seconds", "0.6"])
    finally:
        stop.set()
        t.join(timeout=2)
        pub.close(0)
        ctx.term()
    out = capsys.readouterr().out
    assert rc == 0 and "seq gaps" in out and "waist_yaw" in out and "R_ankle_r" in out and "pelvis" in out and "|accel| 9.810" in out
    assert out.count("\n") >= tool.DOF + 4
    assert "mode(s)=['DISARMED']" in out and "valid=" in out


def test_listen_output_is_visible_live_through_a_pipe(tmp_path):
    """`listen | tee file`: stdout is a pipe, which Python block-buffers by default; the tool must flush line by line (an empty tee file and a
    silent terminal made a real 90 s recording look hung and got it interrupted)."""
    zmq = pytest.importorskip("zmq")
    import sys
    import threading
    import time

    tool = _tool()
    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    port = pub.bind_to_random_port("tcp://127.0.0.1")
    stop = threading.Event()

    def publish():
        seq = 0
        while not stop.is_set():
            q = [0.01 * i for i in range(tool.DOF)]
            pub.send(tool.pack_state(0xAB, seq, 0, 0, 0b11, 0, 0, 0, 0, 1_500_000, 1_000_000, 0, q, [0.0] * tool.DOF, [0.0] * tool.DOF,
                                     [1.0] + [0.0] * 8 + [9.81], [1.0] + [0.0] * 8 + [9.81]))
            seq += 1
            time.sleep(0.005)

    t = threading.Thread(target=publish, daemon=True)
    t.start()
    proc = subprocess.Popen([sys.executable, str(DEPLOY / "scripts/a3_relay_tool.py"), "listen", "--host", "127.0.0.1", "--state-port", str(port),
                             "--seconds", "20"], stdout=subprocess.PIPE, text=True)
    try:
        import select

        ready, _, _ = select.select([proc.stdout], [], [], 6.0)
        assert ready, "no output within 6 s although the tool runs for 20 s: stdout is block-buffered"
        assert "mode=DISARMED" in proc.stdout.readline()
        assert proc.poll() is None  # still running when the first line arrived
    finally:
        proc.kill()
        proc.wait()
        stop.set()
        t.join(timeout=2)
        pub.close(0)
        ctx.term()


@pytestmark_bash
def test_zmq_link_gives_the_binary_its_ros_message_libraries_on_the_real_robot_too(tmp_path):
    """The x86 deploy binary is linked against ROS 2 typesupport libraries even over ZMQ; ``--telecom zmq real`` once started it without them
    ("libsensor_msgs__rosidl_typesupport_cpp.so: cannot open shared object file") because only the sim branch set the library path."""
    root = _fake_tree(tmp_path)
    prefix = tmp_path / "rosenv"
    (prefix / "lib").mkdir(parents=True)
    for target in ("real", "sim"):
        done, args = _launch(root, ["--robot", "a3_ultra", target, "--teleop-input", "zmq", "--telecom", "zmq", "--relay-host", "10.42.10.12", "--yes"],
                             tmp_path, {"A3_ROS_PREFIX": str(prefix)})
        assert done.returncode == 0, done.stderr
        assert f"LDLP={prefix}/lib" in args, (target, args)
        assert "TRANSPORT= DOMAIN=" in args  # still no ROS transport / domain forced
    # an absent prefix must not break the launch (the binary then reports the missing library itself)
    done, args = _launch(root, ["--robot", "a3_ultra", "real", "--teleop-input", "zmq", "--telecom", "zmq", "--relay-host", "10.42.10.12", "--yes"],
                         tmp_path, {"A3_ROS_PREFIX": str(tmp_path / "nowhere")})
    assert done.returncode == 0 and "LDLP=" in args and "nowhere" not in args


def test_record_saves_every_state_frame_and_keeps_the_data_on_ctrl_c(tmp_path):
    """`record` (full-rate, read-only) is how a real teleop run is analysed afterwards; Ctrl-C must keep what was recorded."""
    zmq = pytest.importorskip("zmq")
    np = pytest.importorskip("numpy")
    import signal
    import sys
    import threading
    import time

    tool = _tool()
    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    port = pub.bind_to_random_port("tcp://127.0.0.1")
    stop = threading.Event()

    def publish():
        seq = 0
        while not stop.is_set():
            q = [0.001 * seq + 0.01 * i for i in range(tool.DOF)]
            pub.send(tool.pack_state(0xAB, seq, 1, 1, 0b111, 0x1234, seq, 0, seq // 10, 1_500_000, 1_000_000, 0, q, [0.0] * tool.DOF, [0.5] * tool.DOF,
                                     [1.0] + [0.0] * 8 + [9.81], [1.0] + [0.0] * 8 + [9.81]))
            seq += 1
            time.sleep(0.005)

    t = threading.Thread(target=publish, daemon=True)
    t.start()
    out = tmp_path / "run.npz"
    proc = subprocess.Popen([sys.executable, str(DEPLOY / "scripts/a3_relay_tool.py"), "record", "--host", "127.0.0.1", "--state-port", str(port),
                             "--seconds", "60", "--out", str(out)], stdout=subprocess.PIPE, text=True)
    try:
        time.sleep(2.0)
        proc.send_signal(signal.SIGINT)
        text, _ = proc.communicate(timeout=20)
    finally:
        if proc.poll() is None:
            proc.kill()
        stop.set()
        t.join(timeout=2)
        pub.close(0)
        ctx.term()
    assert "saved" in text and out.exists(), text
    d = np.load(out)
    n = len(d["seq"])
    assert 100 < n < 600, n  # ~2 s at 200 Hz
    assert d["q"].shape == (n, tool.DOF) and d["tau"].shape == (n, tool.DOF) and d["pelvis"].shape == (n, 10)
    assert np.all(np.diff(d["t_wall"]) >= 0) and np.all(d["mode"] == 1) and np.all(d["owner"] == 0x1234)
    assert d["tau"][0, 0] == 0.5 and list(d["slot_names"][:3]) == ["waist_yaw", "waist_roll", "waist_pitch"]

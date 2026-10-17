"""Real-robot launch logic of the A3 / A3 Ultra deploy: what the launcher passes to the binary, and the guards around it.

The vendor procedure (docs/a3_training2sim2deploy.md section 6.1) starts the deploy in IDLE and sends nothing until the operator
presses S.  The MuJoCo flow keeps the G1-like automatic PD stand ("Init Done"); it is opt-in (`--auto-pd-stand`), passed only for
`sim`.  Also pinned: the real-robot confirmation, the operator-chain guard for ROS settings, the preflight, and the ta-bridge QoS
(a best_effort publisher cannot reach the reliable `/ta/whole_body_command` subscription of config/a3_aimrt_config.iceoryx.yaml).
"""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import re
import shutil
import subprocess

import pytest

REPO = Path(__file__).resolve().parents[1]
DEPLOY = REPO / "gear_sonic_deploy"
SRC = DEPLOY / "src/g1/g1_deploy_onnx_ref"
pytestmark = pytest.mark.skipif(shutil.which("bash") is None, reason="needs bash")


def _fake_tree(tmp_path: Path) -> Path:
    """A minimal gear_sonic_deploy tree: the real launcher scripts + a fake package whose run_a3.sh records its arguments."""
    root = tmp_path / "gear_sonic_deploy"
    (root / "scripts").mkdir(parents=True)
    for name in ("deploy_a3.sh", "a3_operator_chain.sh", "a3_real_preflight.sh"):
        shutil.copy(DEPLOY / "scripts" / name, root / "scripts" / name)
    arch = "rockchip" if os.uname().machine in ("aarch64", "arm64") else "x86_64"
    pkg = root / "dist" / f"a3_deploy_{arch}"
    pkg.mkdir(parents=True)
    run = pkg / "run_a3.sh"
    run.write_text('#!/usr/bin/env bash\necho "ARGS: $* TRANSPORT=${A3_TRANSPORT:-} RMW=${RMW_IMPLEMENTATION:-}" > "${A3_TEST_OUT}"\n')
    run.chmod(0o755)
    models = root / "assets/a3_runtime/models/035_step200000"
    models.mkdir(parents=True)
    (models / "m.onnx").write_bytes(b"")
    return root


def _run_launcher(root: Path, args: list[str], tmp_path: Path, env_extra: dict | None = None):
    out = tmp_path / "args.txt"
    env = {**os.environ, "A3_TEST_OUT": str(out), "PATH": os.environ["PATH"]}
    env.pop("A3_ASSUME_YES", None)
    env.update(env_extra or {})
    done = subprocess.run(["bash", str(root / "scripts/deploy_a3.sh"), *args], stdin=subprocess.DEVNULL, capture_output=True,
                          text=True, env=env, timeout=60)
    return done, (out.read_text().strip() if out.exists() else "")


def test_sim_keeps_the_automatic_pd_stand_real_does_not(tmp_path):
    root = _fake_tree(tmp_path)
    done, args = _run_launcher(root, ["--robot", "a3_ultra", "sim", "--encoder", "a3-fast"], tmp_path)
    assert done.returncode == 0, done.stderr
    assert "--teleop-source a3_fast" in args and "--auto-pd-stand" in args
    done, args = _run_launcher(root, ["--robot", "a3_ultra", "real", "--encoder", "a3-fast", "--yes"], tmp_path)
    assert done.returncode == 0, done.stderr
    assert "--teleop-source a3_fast" in args and "--auto-pd-stand" not in args  # the real deploy starts IDLE


def test_pico_flow_in_the_simulator_starts_idle_like_the_real_robot(tmp_path):
    """With the Pico flow (zmq_manager) the robot is hardened by A+B+X+Y, not automatically at startup (G1 operator flow)."""
    root = _fake_tree(tmp_path)
    done, args = _run_launcher(
        root, ["--robot", "a3_ultra", "--input-type", "zmq_manager", "--no-chain", "sim", "--encoder", "a3-fast"], tmp_path)
    assert done.returncode == 0, done.stderr
    assert "--auto-pd-stand" not in args and "--zmq-port 5556" in args
    assert "A+B+X+Y  robot hardens" in done.stdout and "key 9 releases the elastic band" in done.stdout
    # without the Pico flow the scripted simulator run keeps its automatic PD stand
    done, args = _run_launcher(root, ["--robot", "a3_ultra", "sim", "--encoder", "a3-fast"], tmp_path)
    assert "--auto-pd-stand" in args


def test_real_start_needs_a_confirmation(tmp_path):
    root = _fake_tree(tmp_path)
    done, args = _run_launcher(root, ["--robot", "a3_ultra", "real"], tmp_path)  # stdin is not a terminal, no --yes
    assert done.returncode == 2 and args == ""
    assert "REAL ROBOT" in done.stdout and "--yes" in done.stderr
    done, args = _run_launcher(root, ["--robot", "a3_ultra", "real"], tmp_path, {"A3_ASSUME_YES": "1"})
    assert done.returncode == 0 and "--teleop-source a3_fast" in args


def test_real_pico_commands_come_from_the_operator_pc(tmp_path):
    root = _fake_tree(tmp_path)
    done, args = _run_launcher(
        root, ["--robot", "a3_ultra", "--input-type", "zmq_manager", "--no-chain", "--zmq-host", "10.1.2.3", "real", "--yes"], tmp_path)
    assert done.returncode == 0, done.stderr
    assert "--zmq-host 10.1.2.3 --zmq-port 5556" in args and "--auto-pd-stand" not in args
    assert "starts IDLE and sends nothing" in done.stdout
    # the G1 operator flow is printed: A+B+X+Y hardens to the default standing pose, A+X follows / returns, A+B+X+Y disables
    assert "robot hardens and holds the default standing pose" in done.stdout and "kp = kd = 0, limp" in done.stdout


def test_operator_chain_refuses_a_real_robot_without_a_reachable_ros_setup(tmp_path):
    root = _fake_tree(tmp_path)
    chain = str(root / "scripts/a3_operator_chain.sh")
    clean = {"PATH": os.environ["PATH"], "HOME": os.environ.get("HOME", "/tmp"), "A3_RETARGET_PYTHON": "/nonexistent/python"}

    def run(encoder, **ros):
        return subprocess.run(["bash", chain, "--encoder", encoder, "--real"], capture_output=True, text=True, timeout=60, env={**clean, **ros})

    done = run("a3-fast", ROS_DOMAIN_ID="87", ROS_LOCALHOST_ONLY="1")
    assert done.returncode == 66 and "ROS_LOCALHOST_ONLY=1" in done.stderr and "real robot:" not in done.stdout
    done = run("a3-fast", ROS_DOMAIN_ID="0", ROS_LOCALHOST_ONLY="0")
    assert done.returncode == 66 and "ROS_DOMAIN_ID" in done.stderr
    done = run("a3-fast")  # nothing set
    assert done.returncode == 66 and "ROS_DOMAIN_ID" in done.stderr
    done = run("a3-fast", ROS_DOMAIN_ID="42", ROS_LOCALHOST_ONLY="0")
    assert "real robot: ROS_DOMAIN_ID=42" in done.stdout and "retarget python not found" in done.stderr  # past the guard
    assert "WARNING: RMW is not rmw_cyclonedds_cpp" in done.stderr  # Fast DDS 2.6.4 can blow up on foreign discovery data
    done = run("a3-fast", ROS_DOMAIN_ID="42", ROS_LOCALHOST_ONLY="0", RMW_IMPLEMENTATION="rmw_cyclonedds_cpp")
    assert "CYCLONEDDS_URI is not set" in done.stderr and "RMW is not" not in done.stderr
    done = run("a3-fast", ROS_DOMAIN_ID="42", ROS_LOCALHOST_ONLY="0", RMW_IMPLEMENTATION="rmw_cyclonedds_cpp", CYCLONEDDS_URI="<CycloneDDS/>")
    assert "WARNING" not in done.stderr
    done = run("smpl")  # the SMPL path needs no ROS at all
    assert "ROS_DOMAIN_ID" not in done.stderr


def test_preflight_fails_without_a_package_and_is_read_only(tmp_path):
    root = tmp_path / "gear_sonic_deploy"
    (root / "scripts").mkdir(parents=True)
    shutil.copy(DEPLOY / "scripts/a3_real_preflight.sh", root / "scripts/a3_real_preflight.sh")
    before = sorted(p.name for p in tmp_path.rglob("*"))
    done = subprocess.run(["bash", str(root / "scripts/a3_real_preflight.sh")], capture_output=True, text=True, timeout=60)
    assert done.returncode == 1 and "[FAIL] missing" in done.stdout and "gantry" in done.stdout
    assert sorted(p.name for p in tmp_path.rglob("*")) == before
    helped = subprocess.run(["bash", str(root / "scripts/a3_real_preflight.sh"), "--help"], capture_output=True, text=True, timeout=60)
    assert helped.returncode == 0 and "READ-ONLY" in helped.stdout


def test_preflight_is_staged_into_the_package_by_the_build_script():
    build = (DEPLOY / "scripts/build_a3_deploy_pkg.sh").read_text()
    assert re.search(r'install -m 0755 "\$\{SCRIPT_DIR\}/a3_real_preflight.sh" "\$\{PKG_DIR\}/a3_real_preflight.sh"', build)


def test_binary_gates_the_automatic_pd_stand_behind_its_own_flag():
    main = (SRC / "src/a3_deploy/main.cpp").read_text()
    assert '"--auto-pd-stand"' in main.split("bool ValidateKnownFlags")[1][:700]  # accepted by the flag validation
    block = main.split("Pico bring-up when deploy.sh passed --teleop-source", 1)[1][:1500]
    assert block.index('HasFlag(argc, argv, "--auto-pd-stand")') < block.index("RequestDeployMode(manual_control, DeployMode::kPdStand")
    assert "startup IDLE, no commands sent" in block


def test_ta_bridge_publishes_reliable_by_default():
    """ROS 2: best_effort publisher -> reliable subscriber never matches.  The iceoryx (real robot) AimRT config subscribes reliable."""
    src = (SRC / "tools/a3_ta_bridge/a3_ta_bridge.cpp").read_text()
    assert 'std::string qos = "reliable";' in src and "qos.reliable()" in src
    iceoryx = (DEPLOY / "dist/a3_deploy_x86_64/config/a3_aimrt_config.iceoryx.yaml")
    if iceoryx.exists():
        assert re.search(r"reliability:\s*reliable", iceoryx.read_text())


def test_teleop_input_zmq_reads_a3_ref_directly_and_starts_no_bridge(tmp_path):
    root = _fake_tree(tmp_path)
    done, args = _run_launcher(root, ["--robot", "a3_ultra", "sim", "--encoder", "a3-fast"], tmp_path)
    assert "--teleop-input" not in args  # default: the /ta/whole_body_command subscription
    done, args = _run_launcher(root, ["--robot", "a3_ultra", "sim", "--encoder", "a3-fast", "--teleop-input", "zmq"], tmp_path)
    assert done.returncode == 0, done.stderr
    assert "--teleop-input zmq --a3-ref-host localhost --a3-ref-port 5560" in args
    done, args = _run_launcher(root, ["--robot", "a3_ultra", "--input-type", "zmq_manager", "--no-chain", "--zmq-host", "10.42.10.99",
                                      "real", "--yes", "--teleop-input=zmq"], tmp_path)
    assert done.returncode == 0, done.stderr
    assert "--a3-ref-host 10.42.10.99 --a3-ref-port 5560" in args and "--zmq-host 10.42.10.99" in args
    done, _ = _run_launcher(root, ["--robot", "a3_ultra", "sim", "--teleop-input", "udp"], tmp_path)
    assert done.returncode == 2 and "--teleop-input must be ta or zmq" in done.stderr


def test_operator_chain_without_a_bridge_needs_no_ros_setup(tmp_path):
    root = _fake_tree(tmp_path)
    chain = str(root / "scripts/a3_operator_chain.sh")
    env = {"PATH": os.environ["PATH"], "HOME": os.environ.get("HOME", "/tmp"), "A3_RETARGET_PYTHON": "/nonexistent/python"}
    done = subprocess.run(["bash", chain, "--encoder", "a3-fast", "--real", "--no-bridge"], capture_output=True, text=True, timeout=60, env=env)
    assert "ROS_DOMAIN_ID" not in done.stderr and "retarget python not found" in done.stderr  # not refused for missing ROS settings


def test_remote_mode_is_real_only_and_needs_a_ros_domain(tmp_path):
    root = _fake_tree(tmp_path)
    base = {"A3_MEMCAP": "off", "A3_GRAPH_PROBE_CMD": "true"}
    done, _ = _run_launcher(root, ["--robot", "a3_ultra", "sim", "--remote"], tmp_path, base)
    assert done.returncode == 2 and "--remote is for the real robot" in done.stderr
    clean = {k: v for k, v in os.environ.items() if k not in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY")}
    out = tmp_path / "args.txt"

    def run(extra_env):
        env = {**clean, "A3_TEST_OUT": str(out), **base, **extra_env}
        return subprocess.run(["bash", str(root / "scripts/deploy_a3.sh"), "--robot", "a3_ultra", "real", "--remote", "--yes"],
                              stdin=subprocess.DEVNULL, capture_output=True, text=True, env=env, timeout=60)

    assert run({}).returncode == 66  # no ROS_DOMAIN_ID
    assert run({"ROS_DOMAIN_ID": "0"}).returncode == 66
    done = run({"ROS_DOMAIN_ID": "232", "ROS_LOCALHOST_ONLY": "1"})
    assert done.returncode == 66 and "ROS_LOCALHOST_ONLY=1" in done.stderr
    assert not out.exists()


def test_remote_mode_refuses_to_start_when_the_robot_does_not_offer_the_topics(tmp_path):
    root = _fake_tree(tmp_path)
    out = tmp_path / "args.txt"
    env = {**os.environ, "A3_TEST_OUT": str(out), "ROS_DOMAIN_ID": "232", "ROS_LOCALHOST_ONLY": "0", "A3_MEMCAP": "off",
           "A3_GRAPH_PROBE_CMD": "echo 'SONIC ros2 gate: FAIL (10 problems)'; exit 3"}
    cmd = ["bash", str(root / "scripts/deploy_a3.sh"), "--robot", "a3_ultra", "real", "--remote", "--yes"]
    done = subprocess.run(cmd, stdin=subprocess.DEVNULL, capture_output=True, text=True, env=env, timeout=60)
    assert done.returncode == 66 and "gate FAILED" in done.stderr and "NOT started" in done.stderr
    assert "SONIC ros2 gate: FAIL" in done.stdout and not out.exists()  # the deploy binary never ran
    env["A3_GRAPH_PROBE_CMD"] = "echo 'SONIC ros2 gate: PASS'"
    done = subprocess.run(cmd, stdin=subprocess.DEVNULL, capture_output=True, text=True, env=env, timeout=60)
    assert done.returncode == 0, done.stderr
    args = out.read_text()
    assert "--teleop-source a3_fast" in args and "--auto-pd-stand" not in args  # still IDLE on the real robot
    assert "TRANSPORT=ros2" in args and "RMW=rmw_cyclonedds_cpp" in args
    assert "REMOTE" in done.stdout and "WARNING: Cyclone DDS is not pinned" in done.stderr


def _probe_module():
    spec = importlib.util.spec_from_file_location("a3_ros_graph_probe", DEPLOY / "scripts/a3_ros_graph_probe.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _offering():
    mod = _probe_module()
    types = {t: [ty] for t, (ty, _) in mod.SONIC_TOPICS.items()}
    pubs = {t: ["best_effort"] for t, (_, role) in mod.SONIC_TOPICS.items() if role == "pub"}
    subs = {t: ["best_effort"] for t, (_, role) in mod.SONIC_TOPICS.items() if role == "sub"}
    return mod, types, pubs, subs


def test_sonic_gate_passes_what_the_mujoco_robot_offers_and_fails_the_real_snapshot():
    mod, types, pubs, subs = _offering()
    assert mod.sonic_gate(types, pubs, subs) == []
    # 2026-10-15 snapshot of the real robot (plan 18.8): no /body_drive state or command topic on ROS 2 except one leg command mirror
    real = mod.sonic_gate({"/body_drive/leg_joint_command_ros2": ["joint_msgs/msg/JointCommand"]}, {}, {})
    assert len(real) == 10 and all("not on the ROS 2 network" in p for p in real)


def test_sonic_gate_checks_types_publishers_and_qos():
    mod, types, pubs, subs = _offering()
    subs["/body_drive/leg_joint_command"] = ["best_effort", "reliable"]  # one reliable robot subscriber is unreachable for best_effort
    problems = mod.sonic_gate(types, pubs, subs)
    assert len(problems) == 1 and "/body_drive/leg_joint_command" in problems[0] and "reliable" in problems[0]
    mod, types, pubs, subs = _offering()
    pubs["/body_drive/arm_joint_state"] = ["reliable"]  # a reliable robot publisher is fine for our best_effort subscriber
    types["/body_drive/torso_imu/data"] = ["sensor_msgs/msg/JointState"]
    del subs["/body_drive/neck_joint_command"]
    del pubs["/body_drive/pelvis_imu/data"]
    problems = mod.sonic_gate(types, pubs, subs)
    assert len(problems) == 3
    assert any("torso_imu" in p and "instead of" in p for p in problems)
    assert any("neck_joint_command" in p and "nobody subscribes" in p for p in problems)
    assert any("pelvis_imu" in p and "nobody publishes" in p for p in problems)


def test_telecom_ros2_on_the_real_robot_is_the_remote_route_with_its_gate_and_ros_checks(tmp_path):
    """``--telecom ros2 real`` == ``--remote``: same ROS domain checks, same read-only gate, same Cyclone / memory-cap setup."""
    root = _fake_tree(tmp_path)
    out = tmp_path / "args.txt"
    clean = {k: v for k, v in os.environ.items() if k not in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY")}
    base = {**clean, "A3_TEST_OUT": str(out), "A3_MEMCAP": "off", "A3_GRAPH_PROBE_CMD": "echo 'SONIC ros2 gate: PASS'"}
    cmd = ["bash", str(root / "scripts/deploy_a3.sh"), "--robot", "a3_ultra", "real", "--telecom", "ros2", "--yes"]

    def run(env_extra, command=cmd):
        return subprocess.run(command, stdin=subprocess.DEVNULL, capture_output=True, text=True, env={**base, **env_extra}, timeout=60)

    assert run({}).returncode == 66 and not out.exists()                       # no ROS_DOMAIN_ID
    assert run({"ROS_DOMAIN_ID": "232", "ROS_LOCALHOST_ONLY": "1"}).returncode == 66
    failing = run({"ROS_DOMAIN_ID": "232", "ROS_LOCALHOST_ONLY": "0", "A3_GRAPH_PROBE_CMD": "echo 'SONIC ros2 gate: FAIL'; exit 3"})
    assert failing.returncode == 66 and "gate FAILED" in failing.stderr and "--telecom zmq" in failing.stderr and not out.exists()
    done = run({"ROS_DOMAIN_ID": "232", "ROS_LOCALHOST_ONLY": "0"})
    assert done.returncode == 0, done.stderr
    args = out.read_text()
    assert "TRANSPORT=ros2" in args and "RMW=rmw_cyclonedds_cpp" in args and "--auto-pd-stand" not in args and "--robot-link" not in args
    assert "telecom=ros2" in done.stdout and "REMOTE" in done.stdout
    out.unlink()
    legacy = [*cmd[:-3], "--remote", "--yes"]                                  # the older spelling reaches the same route
    done = run({"ROS_DOMAIN_ID": "232", "ROS_LOCALHOST_ONLY": "0"}, legacy)
    assert done.returncode == 0 and out.read_text() == args and "telecom=ros2" in done.stdout


def test_preflight_reports_missing_shared_libraries_of_the_robot_binaries(tmp_path):
    """The aarch64 package does not bundle libzmq.so.5 (plan 18.20): the preflight must say so before the first start, and stay read-only."""
    pkg = tmp_path / "pkg"
    (pkg / "config").mkdir(parents=True)
    for name in ("a3_deploy_onnx_ref", "a3_zmq_relay"):
        f = pkg / name
        f.write_text("#!/bin/sh\n")
        f.chmod(0o755)
    shutil.copy(DEPLOY / "scripts/a3_real_preflight.sh", pkg / "a3_real_preflight.sh")
    fake = tmp_path / "bin"
    fake.mkdir()
    ldd = fake / "ldd"

    def run(ldd_body):
        ldd.write_text("#!/bin/sh\n" + ldd_body)
        ldd.chmod(0o755)
        snapshot = sorted(p.name for p in tmp_path.rglob("*"))
        env = {**os.environ, "PATH": f"{fake}:{os.environ['PATH']}"}
        done = subprocess.run(["bash", str(pkg / "a3_real_preflight.sh")], capture_output=True, text=True, env=env, timeout=60)
        assert sorted(p.name for p in tmp_path.rglob("*")) == snapshot  # read-only: nothing created
        return done

    done = run('echo "\tlibzmq.so.5 => not found"; echo "\tlibc.so.6 => /lib/libc.so.6 (0x1)"\n')
    assert "[FAIL] a3_deploy_onnx_ref: missing shared libraries: libzmq.so.5" in done.stdout
    assert "[FAIL] a3_zmq_relay: missing shared libraries: libzmq.so.5" in done.stdout and "does not bundle it" in done.stdout
    assert done.returncode == 1
    ok = run('echo "\tlibc.so.6 => /lib/libc.so.6 (0x1)"\n')
    assert "[PASS] a3_zmq_relay: all shared libraries resolve" in ok.stdout and "missing shared libraries" not in ok.stdout
    foreign = run('echo "\tnot a dynamic executable"; exit 1\n')
    assert "ldd cannot inspect it" in foreign.stdout and "missing shared libraries" not in foreign.stdout


# ---- route C, robot side: preflight --relay + the relay bundle --------------------------------------------------------------
def _relay_bundle(tmp_path: Path) -> Path:
    pkg = tmp_path / "a3_relay_test"
    (pkg / "config").mkdir(parents=True)
    for name in ("a3_zmq_relay", "run_a3.sh", "run_a3_relay.sh", "libaimrt_iceoryx_plugin.so"):
        f = pkg / name
        f.write_text("#!/bin/sh\n")
        f.chmod(0o755)
    (pkg / "config/a3_aimrt_config.iceoryx.yaml").write_text("plugins:\n  - path: libaimrt_net_plugin.so\n")
    shutil.copy(DEPLOY / "scripts/a3_real_preflight.sh", pkg / "a3_real_preflight.sh")
    return pkg


def _preflight_relay(pkg: Path, tmp_path: Path, args: list[str], ldd_body: str = 'echo "\\tlibc.so.6 => /lib/libc.so.6 (0x1)"\n'):
    fake = tmp_path / "bin"
    fake.mkdir(exist_ok=True)
    (fake / "ldd").write_text("#!/bin/sh\n" + ldd_body)
    (fake / "ldd").chmod(0o755)
    env = {**os.environ, "PATH": f"{fake}:{os.environ['PATH']}"}
    return subprocess.run(["bash", str(pkg / "a3_real_preflight.sh"), "--relay", *args], capture_output=True, text=True, env=env, timeout=60)


def test_preflight_relay_mode_checks_the_relay_bundle_not_the_full_deploy(tmp_path):
    pkg = _relay_bundle(tmp_path)
    before = sorted(p.name for p in tmp_path.rglob("*"))
    done = _preflight_relay(pkg, tmp_path, ["--bind-ip", "127.0.0.1"])
    out = done.stdout
    assert "(relay side)" in out and "[PASS] a3_zmq_relay present" in out and "[PASS] config/a3_aimrt_config.iceoryx.yaml" in out
    assert "a3_deploy_onnx_ref present" not in out and "models/" not in out  # none of the full-deploy package checks
    assert "[PASS] a3_zmq_relay: all shared libraries resolve" in out and "[PASS] 127.0.0.1 is an address of this machine" in out
    assert "net_plugin" in out  # the stock config's side effects are announced
    missing = _preflight_relay(pkg, tmp_path, ["--bind-ip", "127.0.0.1"], 'echo "\\tlibzmq.so.5 => not found"\n')
    assert "[FAIL] a3_zmq_relay: missing shared libraries: libzmq.so.5" in missing.stdout and "does not carry it" in missing.stdout and missing.returncode == 1
    wrong_ip = _preflight_relay(pkg, tmp_path, ["--bind-ip", "203.0.113.9"])
    assert "[FAIL] 203.0.113.9 is not an address of this machine" in wrong_ip.stdout
    assert sorted(p.name for p in tmp_path.rglob("*")) == sorted(before + ["ldd", "bin"]) or "bin" in before  # read-only (only the fake ldd was added)


def test_preflight_sees_processes_whose_names_the_kernel_truncates(tmp_path):
    """pgrep -x cannot match names over 15 characters (a3_deploy_onnx_ref, RoboticsServiceProcess): the preflight matches the command line."""
    pkg = _relay_bundle(tmp_path)
    sleeper = shutil.which("sleep")
    host_has_deploy = subprocess.run(["pgrep", "-f", "(^|/)a3_deploy_onnx_ref( |$)"], capture_output=True).returncode == 0  # e.g. a live session
    host_has_mc = subprocess.run(["pgrep", "-f", "(^|/)(motion_control|mc)( |$)"], capture_output=True).returncode == 0
    procs = []
    try:
        for name in ("a3_deploy_onnx_ref", "mc"):
            exe = tmp_path / "run" / name
            exe.parent.mkdir(exist_ok=True)
            shutil.copy(sleeper, exe)
            procs.append(subprocess.Popen([str(exe), "30"]))
        import time

        time.sleep(0.3)
        done = _preflight_relay(pkg, tmp_path, [])
        assert "[FAIL] a full a3_deploy_onnx_ref is running on this machine" in done.stdout
        assert "[FAIL] the stock motion_control (mc) is running" in done.stdout
        listen = _preflight_relay(pkg, tmp_path, ["--listen-only"])
        assert "[WARN] the stock motion_control (mc) is running: fine for --listen-only" in listen.stdout
    finally:
        for p in procs:
            p.kill()
            p.wait()
    quiet = _preflight_relay(pkg, tmp_path, [])  # the fake processes are gone: the checks go back to PASS unless the host has real ones
    if not host_has_deploy:
        assert "[PASS] no a3_deploy_onnx_ref on this machine" in quiet.stdout
    if not host_has_mc:
        assert "[PASS] motion_control (mc) is not running" in quiet.stdout


def test_relay_bundle_contains_exactly_what_the_relay_needs(tmp_path):
    spec = importlib.util.spec_from_file_location("a3_make_relay_bundle", DEPLOY / "scripts/a3_make_relay_bundle.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    pkg = DEPLOY / "dist/a3_deploy_x86_64"
    if not (pkg / "a3_zmq_relay").exists() or shutil.which("readelf") is None:
        pytest.skip("needs the built x86 package and readelf")
    out = tmp_path / "bundle.tar.gz"
    members, external = mod.build(pkg, out, "a3_relay_x86_64")
    names = {p.name for p in members}
    assert {"a3_zmq_relay", "run_a3_relay.sh", "a3_real_preflight.sh", "libaimrt_iceoryx_plugin.so", "libaimrt_ros2_plugin.so"} <= names
    assert not any(n.startswith(("libonnxruntime", "librknnrt", "a3_deploy_onnx_ref", "model_")) for n in names)  # no policy runtime, no models
    assert "libzmq.so.5" in external and "libc.so.6" in external  # system libraries are reported, not bundled
    import tarfile

    with tarfile.open(out) as tf:
        listing = tf.getnames()
    assert "a3_relay_x86_64/MANIFEST.sha256" in listing and "a3_relay_x86_64/a3_zmq_relay" in listing
    assert (tmp_path / "bundle.tar.gz.sha256").read_text().split()[0] == mod.sha256(out)


def test_operator_preflight_for_route_c_skips_ros_and_checks_that_the_relay_answers(tmp_path):
    import socket

    root = tmp_path / "gear_sonic_deploy"
    (root / "scripts").mkdir(parents=True)
    shutil.copy(DEPLOY / "scripts/a3_real_preflight.sh", root / "scripts/a3_real_preflight.sh")
    env = {k: v for k, v in os.environ.items() if k not in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY")}

    def run(*args):
        return subprocess.run(["bash", str(root / "scripts/a3_real_preflight.sh"), "--operator", *args], capture_output=True, text=True, env=env, timeout=60)

    ros = run()  # the ROS 2 route still demands the robot's domain
    assert "[FAIL] ROS_DOMAIN_ID is unset/0" in ros.stdout
    with socket.socket() as srv:
        srv.bind(("127.0.0.1", 0))
        srv.listen(1)
        port = srv.getsockname()[1]
        up = run("--relay-host", "127.0.0.1", "--relay-state-port", str(port))
    assert "ROS_DOMAIN_ID" not in up.stdout and "route C" in up.stdout
    assert f"[PASS] something accepts connections on 127.0.0.1:{port}" in up.stdout
    down = run("--relay-host", "127.0.0.1", "--relay-state-port", str(port))  # the socket is closed now
    assert f"[WARN] nothing accepts connections on 127.0.0.1:{port}: the relay is not running yet" in down.stdout

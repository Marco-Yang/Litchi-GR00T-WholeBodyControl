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

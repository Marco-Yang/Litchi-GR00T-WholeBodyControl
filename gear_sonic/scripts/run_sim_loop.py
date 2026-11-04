"""Entry point for running a MuJoCo simulation loop for the selected robot (--robot).

Parses a YAML-based WBC config via tyro CLI and launches the simulator:
  * g1            -> Unitree-SDK2 MuJoCo sim (base_sim.py), optional offscreen image publishing
  * a3 / a3_ultra -> A3 body-drive ROS2 MuJoCo sim (a3_sim.py); the C++ a3_deploy_onnx_ref talks to
                     it exactly as it talks to the real robot (same as the G1 sim/real symmetry)
"""

import os
from pathlib import Path
import sys
from typing import Dict

import tyro

from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig

ArgsConfig = SimLoopConfig

# A3 / A3 Ultra are simulated through the A3 body-drive ROS2 contract (see a3_sim.py).
A3_SIM_ROBOTS = ("a3", "a3_ultra")
A3_SIM_ENV = "a3_deploy_verify"  # conda env that has rclpy + joint_msgs (setup_a3_sim_env.sh)


class SimWrapper:
    def __init__(self, robot_model, env_name: str, config: Dict[str, any], **kwargs):
        from gear_sonic.utils.mujoco_sim.simulator_factory import SimulatorFactory

        self.robot_model = robot_model
        self.config = config

        # ChannelFactoryInitialize is done once inside BaseSimulator.
        # Do not call init_channel here: a second Init on the same process fails with
        # "create domain error" / "channel factory init error" (Unitree SDK singleton).
        self.sim = SimulatorFactory.create_simulator(
            config=self.config,
            env_name=env_name,
            **kwargs,
        )


def _ros_python_candidates():
    """Interpreters that may provide rclpy + joint_msgs, in order of preference.

    The dedicated conda env comes before CONDA_PREFIX. Activating .venv_teleop on top of
    conda base leaves CONDA_PREFIX pointing at base, which has neither the ROS messages
    nor tyro.
    """
    seen = set()

    def emit(path):
        if not path:
            return
        p = Path(path)
        if not p.exists():
            return
        key = os.path.realpath(p)
        if key in seen:
            return
        seen.add(key)
        yield str(p)

    if os.environ.get("A3_SIM_PYTHON"):
        yield from emit(os.environ["A3_SIM_PYTHON"])
    root = Path(os.environ.get("CONDA_ROOT", Path.home() / "miniconda3"))
    yield from emit(root / "envs" / A3_SIM_ENV / "bin" / "python")
    prefix = os.environ.get("CONDA_PREFIX")
    if prefix:
        yield from emit(Path(prefix) / "bin" / "python")


def _candidate_has_a3_stack(py: str) -> bool:
    import subprocess

    try:
        result = subprocess.run(
            [py, "-c", "import tyro, rclpy, joint_msgs.msg, sensor_msgs.msg"],
            capture_output=True,
            timeout=30,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0


def _have_a3_ros_stack() -> bool:
    try:
        import rclpy  # noqa: F401
        import joint_msgs.msg  # noqa: F401
        import sensor_msgs.msg  # noqa: F401

        return True
    except Exception:  # noqa: BLE001
        return False


def _reexec_in_ros_env() -> None:
    """The G1 venv has no ROS2; hand the same command line to the interpreter that does."""
    for py in _ros_python_candidates():
        if os.path.realpath(py) == os.path.realpath(sys.executable):
            continue
        if not _candidate_has_a3_stack(py):
            continue
        env = dict(os.environ)
        env["A3_SIM_REEXEC"] = "1"
        # gear_sonic is installed editable in .venv_teleop, not in the ROS conda env.
        repo_root = str(Path(__file__).resolve().parents[2])
        env["PYTHONPATH"] = repo_root + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
        print(f"[a3-sim] rclpy/joint_msgs not importable here; re-running under {py}")
        os.execve(py, [py, *sys.argv], env)
    raise SystemExit(
        "A3 sim needs rclpy and joint_msgs (ROS2 Humble). Run gear_sonic_deploy/scripts/setup_a3_sim_env.sh "
        "once, or set A3_SIM_PYTHON to an interpreter that has them."
    )


def _setup_ros_env(wbc_config: dict) -> None:
    # Dedicated domain + localhost only: the default domain 0 on a LAN once made Fast-DDS allocate tens of GB.
    os.environ.setdefault("ROS_DOMAIN_ID", str(wbc_config.get("ROS_DOMAIN_ID", 87)))
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    pm = os.environ.get("A3_JOINT_MSGS_PYTHONPATH")
    if pm:
        sys.path.insert(0, pm)


def run_a3_sim(config: ArgsConfig) -> None:
    sys.stdout.reconfigure(line_buffering=True)  # logs are read live when redirected to a file
    wbc_config = config.load_wbc_yaml()
    wbc_config["ENV_NAME"] = config.env_name
    _setup_ros_env(wbc_config)
    if not _have_a3_ros_stack():
        if os.environ.get("A3_SIM_REEXEC"):
            raise SystemExit("A3 sim: rclpy/joint_msgs still not importable after re-exec (see setup_a3_sim_env.sh)")
        _reexec_in_ros_env()

    from gear_sonic.utils.mujoco_sim.a3_sim import A3Ros2Bridge, A3Simulator

    print(
        f"[a3-sim] robot={config.robot} scene={wbc_config['ROBOT_SCENE']} dt={wbc_config['SIMULATE_DT']} "
        f"ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']} ROS_LOCALHOST_ONLY={os.environ['ROS_LOCALHOST_ONLY']}"
    )
    sim = A3Simulator(
        wbc_config,
        onscreen=wbc_config.get("ENABLE_ONSCREEN", True),
        bridge=A3Ros2Bridge(),
    )
    sim.start()


def main(config: ArgsConfig):
    if config.robot in A3_SIM_ROBOTS:
        return run_a3_sim(config)

    from gear_sonic.data.robot_model.instantiation import instantiate_robot_model
    from gear_sonic.utils.mujoco_sim.simulator_factory import SimulatorFactory

    wbc_config = config.load_wbc_yaml()
    # NOTE: we will override the interface to local if it is not specified
    wbc_config["ENV_NAME"] = config.env_name

    if config.enable_image_publish:
        assert (
            config.enable_offscreen
        ), "enable_offscreen must be True when enable_image_publish is True"

    robot_model = instantiate_robot_model(config.robot)

    sim_wrapper = SimWrapper(
        robot_model=robot_model,
        env_name=config.env_name,
        config=wbc_config,
        onscreen=wbc_config.get("ENABLE_ONSCREEN", True),
        offscreen=wbc_config.get("ENABLE_OFFSCREEN", False),
        enable_image_publish=config.enable_image_publish,
    )
    # Start simulator as independent process
    SimulatorFactory.start_simulator(
        sim_wrapper.sim,
        as_thread=False,
        enable_image_publish=config.enable_image_publish,
        mp_start_method=config.mp_start_method,
        camera_port=config.camera_port,
    )


if __name__ == "__main__":
    config = tyro.cli(ArgsConfig)
    main(config)

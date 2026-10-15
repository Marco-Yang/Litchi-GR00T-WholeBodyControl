"""MuJoCo simulator that stands in for the A3 / A3 Ultra body-drive stack.

This is the A3 counterpart of ``base_sim.py`` + ``unitree_sdk2py_bridge.py``.  The G1 simulator
speaks the Unitree SDK2 DDS contract so that the *same* C++ ``g1_deploy_onnx_ref`` binary runs
against MuJoCo and the real robot.  Here the contract is the A3 AimRT/ROS2 one the deploy binary
(``a3_deploy_onnx_ref``) already uses against the real robot (see
``gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/robot_io/a3_aimrt_backend.cpp``):

    publishes   /body_drive/{leg,waist,arm,neck}_joint_state   joint_msgs/JointState
                /body_drive/{pelvis,torso}_imu/data            sensor_msgs/Imu
    subscribes  /body_drive/{leg,waist,arm,neck}_joint_command joint_msgs/JointCommand

Each command joint carries (position, velocity, effort, stiffness, damping); the drive applies
``tau = effort + kp (q_des - q) + kd (dq_des - dq)`` which we reproduce at the simulation rate,
exactly like ``DefaultEnv.compute_body_torques`` does for G1.

Needs a ROS2 Python env (rclpy) with ``joint_msgs`` Python bindings -- see
``gear_sonic_deploy/scripts/setup_a3_sim_env.sh``.  Use a dedicated ROS_DOMAIN_ID and
ROS_LOCALHOST_ONLY=1 on a dev machine (docs/workflow/a3_integration_plan.md, 9.7).
"""

from __future__ import annotations

from pathlib import Path
import threading
import time
from typing import Dict, List, Optional

import mujoco
import mujoco.viewer
import numpy as np

GEAR_SONIC_ROOT = Path(__file__).resolve().parents[3]

# A3 native joint order of every body-drive topic (mirrors robot_io/layouts.cpp, a3_layout_extra.cpp).
LEG_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
]  # fmt: skip
WAIST_JOINTS = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
ARM_JOINTS = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]  # fmt: skip
NECK_JOINTS = ["head_yaw_joint", "head_pitch_joint"]

GROUPS = {
    "leg": LEG_JOINTS,
    "waist": WAIST_JOINTS,
    "arm": ARM_JOINTS,
    "neck": NECK_JOINTS,
}


class ElasticBand:
    """Virtual spring band used to hold the robot up before the policy takes over.

    Same behaviour as ``unitree_sdk2py_bridge.ElasticBand`` (key 9 toggles, 7/8 shorten/lengthen),
    re-implemented here so this module has no unitree_sdk2py dependency.
    """

    def __init__(self, point=(0.0, 0.0, 1.2)):
        self.kp_pos, self.kd_pos, self.kp_ang, self.kd_ang = 10000.0, 1000.0, 1000.0, 10.0
        self.point = np.array(point, dtype=float)
        self.length = 0.0
        self.enable = True

    def advance(self, pose: np.ndarray) -> np.ndarray:
        from scipy.spatial.transform import Rotation

        pos, quat, lin_vel, ang_vel = pose[0:3], pose[3:7], pose[7:10], pose[10:13]
        f = self.kp_pos * (self.point - pos + np.array([0, 0, self.length])) + self.kd_pos * (
            0 - lin_vel
        )
        rotvec = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_rotvec()
        return np.concatenate([f, -self.kp_ang * rotvec - self.kd_ang * ang_vel])

    def key_callback(self, key):
        import glfw

        if key == glfw.KEY_7:
            self.length -= 0.1
        elif key == glfw.KEY_8:
            self.length += 0.1
        elif key == glfw.KEY_9:
            self.enable = not self.enable
            print(f"ElasticBand enable: {self.enable}")


class A3Ros2Bridge:
    """rclpy node: publish robot state / IMU, receive joint commands (thread-safe)."""

    def __init__(self, node_name: str = "a3_mujoco_sim"):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from joint_msgs.msg import JointCommand, JointState
        from sensor_msgs.msg import Imu

        self._JointState, self._Imu = JointState, Imu
        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self.node = rclpy.create_node(node_name)
        # Same QoS as the deploy side's AimRT ros2 backend (best_effort, keep_last 10).
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self._state_pub = {
            g: self.node.create_publisher(JointState, f"/body_drive/{g}_joint_state", qos)
            for g in GROUPS
        }
        self._imu_pub = {
            n: self.node.create_publisher(Imu, f"/body_drive/{n}_imu/data", qos)
            for n in ("pelvis", "torso")
        }
        self._lock = threading.Lock()
        # name -> (q, dq, tau_ff, kp, kd), updated from command callbacks
        self.cmd: Dict[str, tuple] = {}
        self.cmd_rx = {g: 0 for g in GROUPS}
        self.last_cmd_time = {g: 0.0 for g in GROUPS}
        for g in GROUPS:
            self.node.create_subscription(
                JointCommand,
                f"/body_drive/{g}_joint_command",
                lambda msg, g=g: self._on_cmd(g, msg),
                qos,
            )
        self._seq = {g: 0 for g in GROUPS}
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception:  # noqa: BLE001  (ExternalShutdownException on SIGINT/SIGTERM)
            pass

    def ok(self) -> bool:
        return self._rclpy.ok()

    def _on_cmd(self, group: str, msg) -> None:
        now = time.monotonic()
        with self._lock:
            for j in msg.joints:
                self.cmd[j.name] = (j.position, j.velocity, j.effort, j.stiffness, j.damping)
            self.cmd_rx[group] += 1
            self.last_cmd_time[group] = now

    def get_cmd(self, name: str) -> Optional[tuple]:
        with self._lock:
            return self.cmd.get(name)

    def snapshot_cmd(self) -> Dict[str, tuple]:
        with self._lock:
            return dict(self.cmd)

    def publish_joint_state(self, group: str, names: List[str], q, dq, tau) -> None:
        from joint_msgs.msg import State

        msg = self._JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self._seq[group] += 1
        joints = []
        for i, n in enumerate(names):
            s = State()
            s.name, s.sequence = n, self._seq[group] & 0xFFFFFFFF
            s.position, s.velocity, s.effort = float(q[i]), float(dq[i]), float(tau[i])
            joints.append(s)
        msg.joints = joints
        self._state_pub[group].publish(msg)

    def publish_imu(self, which: str, quat_wxyz, gyro_b, acc_b) -> None:
        msg = self._Imu()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = f"{which}_imu"
        msg.orientation.w, msg.orientation.x = float(quat_wxyz[0]), float(quat_wxyz[1])
        msg.orientation.y, msg.orientation.z = float(quat_wxyz[2]), float(quat_wxyz[3])
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = map(float, gyro_b)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = map(
            float, acc_b
        )
        self._imu_pub[which].publish(msg)

    def close(self) -> None:
        try:
            self._executor.shutdown()
            self.node.destroy_node()
            if self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception as e:  # noqa: BLE001
            print(f"[a3-sim] bridge close warning: {e}")


class A3Simulator:
    """MuJoCo loop: viewer, elastic band, PD from the latest drive command, state/IMU publishing."""

    def __init__(self, config: Dict, onscreen: bool = True, bridge: Optional[A3Ros2Bridge] = None):
        self.config = config
        self.dt = float(config.get("SIMULATE_DT", config.get("SIMULATE_DT_DEFAULT", 0.002)))
        self.state_decim = max(1, int(round(1.0 / config.get("STATE_PUBLISH_HZ", 250) / self.dt)))
        self.viewer_dt = float(config.get("VIEWER_DT", 0.02))
        self.model = mujoco.MjModel.from_xml_path(str(GEAR_SONIC_ROOT / config["ROBOT_SCENE"]))
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = self.dt
        if self.model.jnt_type[0] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError("A3 sim expects a floating base (free joint) as the first joint")
        self.root_body = config.get("ROOT_BODY", "pelvis_link")
        self.root_id = self.model.body(self.root_body).id
        self.imu_site = {
            "pelvis": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "imu_in_pelvis"),
            "torso": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "imu_in_torso"),
        }
        if min(self.imu_site.values()) < 0:
            raise ValueError("MJCF needs sites imu_in_pelvis / imu_in_torso")

        # topic joint name -> MuJoCo joint name (A3 Ultra calls its head pitch joint differently)
        alias = config.get("JOINT_ALIAS", {}) or {}
        self.mj_name = {n: alias.get(n, n) for g in GROUPS.values() for n in g}
        self.qadr, self.vadr, self.actid = {}, {}, {}
        for n, mj in self.mj_name.items():
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, mj)
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, mj)
            if jid < 0 or aid < 0:
                raise ValueError(f"joint/actuator '{mj}' (topic name '{n}') not in MJCF")
            self.qadr[n], self.vadr[n] = int(self.model.jnt_qposadr[jid]), int(self.model.jnt_dofadr[jid])
            self.actid[n] = aid
        # Rotor inertia: the plain A3 T2D5 MJCF has armature 0 although the policy was trained with the contract
        # values; fill them in (only where the MJCF has none, so A3 Ultra's own values are never overwritten).
        for n, val in (config.get("JOINT_ARMATURE_IF_MISSING") or {}).items():
            if n in self.vadr and self.model.dof_armature[self.vadr[n]] == 0.0:
                self.model.dof_armature[self.vadr[n]] = float(val)
        # Everything else that is actuated (fingers: position actuators) just holds its keyframe value.
        driven = set(self.actid.values())
        self.hold_act = [i for i in range(self.model.nu) if i not in driven]
        # Joints without an actuator-force limit in the MJCF (the A3 T2D5 head joints) are not clipped; clipping them to
        # the [0, 0] placeholder range would leave the head limp instead of held at its default pose.
        self.effort_limit = {}
        for n in self.actid:
            jid = self.model.joint(self.mj_name[n]).id
            self.effort_limit[n] = (
                float(self.model.jnt_actfrcrange[jid][1]) if self.model.jnt_actfrclimited[jid] else float("inf")
            )

        self.bridge = bridge
        # Headless/unattended runs cannot press key 9: A3_SIM_BAND_RELEASE_S releases the band after N sim seconds.
        import os

        rel = os.environ.get("A3_SIM_BAND_RELEASE_S")
        self.band_release_s = float(rel) if rel else None
        self.verbose = bool(os.environ.get("A3_SIM_VERBOSE"))
        # A3_SIM_LOG_CSV=<path>: 50 Hz state log (time, pelvis pose, 29 body joints, 2 head joints) for offline metrics.
        self._log = None
        log_path = os.environ.get("A3_SIM_LOG_CSV")
        if log_path:
            self._log = open(log_path, "w")
            cols = ["t", "px", "py", "pz", "qw", "qx", "qy", "qz"]
            cols += [f"q:{n}" for n in LEG_JOINTS + WAIST_JOINTS + ARM_JOINTS + NECK_JOINTS]
            cols += [f"cmd:{n}" for n in LEG_JOINTS + WAIST_JOINTS + ARM_JOINTS + NECK_JOINTS]
            self._log.write(",".join(cols) + "\n")
        self.band = ElasticBand() if config.get("ENABLE_ELASTIC_BAND", True) else None
        self.viewer = None
        self.onscreen = onscreen
        self._running = True
        self.reset()
        if onscreen:
            kw = dict(show_left_ui=False, show_right_ui=False)
            if self.band is not None:
                kw["key_callback"] = self.band.key_callback
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data, **kw)
            self.viewer.cam.azimuth, self.viewer.cam.elevation = 120, -30
            self.viewer.cam.distance = float(config.get("VIEWER_DISTANCE", 3.0))
            self.viewer.cam.lookat = np.array([0, 0, 0.8])
            self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self.viewer.cam.trackbodyid = self.root_id

    # ------------------------------------------------------------------ state
    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        if self.model.nkey > 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        self.hold_ctrl = {i: float(self.data.ctrl[i]) for i in self.hold_act}
        # Start (and hang the band) at standing height: the keyframe pelvis z of some MJCFs is well above
        # the ground, and the free fall on release would knock the robot over.
        if "BAND_HEIGHT" in self.config:
            self.data.qpos[2] = float(self.config["BAND_HEIGHT"])
        mujoco.mj_forward(self.model, self.data)
        if self.band is not None:
            hang_z = max(0.9, float(self.data.qpos[2]))
            self.band.point = np.array([*self.data.qpos[:2], hang_z])
            self.band.enable = True
            print(
                f"[a3-sim] elastic band ON (pelvis_z={hang_z:.2f} m); "
                "MuJoCo window: press 9 to release onto the ground, 7/8 shorten/lengthen"
            )

    def _imu(self, which: str):
        sid = self.imu_site[which]
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, self.data.site_xmat[sid])
        # angular velocity and specific force expressed in the sensor frame
        vel6 = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_SITE, sid, vel6, 1)
        acc6 = np.zeros(6)
        mujoco.mj_objectAcceleration(self.model, self.data, mujoco.mjtObj.mjOBJ_SITE, sid, acc6, 1)
        # cacc already contains gravity (cacc[world] = -g), i.e. this is the proper acceleration an
        # IMU measures (+9.81 up when standing still).
        return quat, vel6[:3], acc6[3:]

    def publish_state(self) -> None:
        if self.bridge is None:
            return
        for g, names in GROUPS.items():
            q = [self.data.qpos[self.qadr[n]] for n in names]
            dq = [self.data.qvel[self.vadr[n]] for n in names]
            tau = [self.data.actuator_force[self.actid[n]] for n in names]
            self.bridge.publish_joint_state(g, names, q, dq, tau)
        mujoco.mj_rnePostConstraint(self.model, self.data)  # fills cacc for mj_objectAcceleration
        for which in ("pelvis", "torso"):
            self.bridge.publish_imu(which, *self._imu(which))

    # ---------------------------------------------------------------- physics
    def _apply_pd(self) -> None:
        cmds = self.bridge.snapshot_cmd() if self.bridge is not None else {}
        for n, aid in self.actid.items():
            c = cmds.get(n)
            if c is None:
                self.data.ctrl[aid] = 0.0
                continue
            qd, dqd, tau_ff, kp, kd = c
            tau = tau_ff + kp * (qd - self.data.qpos[self.qadr[n]]) + kd * (
                dqd - self.data.qvel[self.vadr[n]]
            )
            lim = self.effort_limit[n]
            self.data.ctrl[aid] = float(np.clip(tau, -lim, lim))
        for i, v in self.hold_ctrl.items():
            self.data.ctrl[i] = v

    def step(self) -> None:
        if (
            self.band is not None
            and self.band.enable
            and self.band_release_s is not None
            and self.data.time >= self.band_release_s
        ):
            self.band.enable = False
            print(f"[a3-sim] elastic band released at t={self.data.time:.1f}s (A3_SIM_BAND_RELEASE_S)")
        if self.band is not None:
            if self.band.enable:
                pose = np.zeros(13)
                pose[0:3] = self.data.xpos[self.root_id]
                pose[3:7] = self.data.xquat[self.root_id]
                v = np.zeros(6)
                mujoco.mj_objectVelocity(
                    self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.root_id, v, 0
                )
                pose[7:10], pose[10:13] = v[3:6], v[0:3]
                self.data.xfrc_applied[self.root_id] = self.band.advance(pose)
            else:
                self.data.xfrc_applied[self.root_id] = 0.0
        self._apply_pd()
        mujoco.mj_step(self.model, self.data)
        if self.data.qpos[2] < float(self.config.get("FALL_RESET_Z", 0.25)):
            print(f"Warning: robot fell (pelvis z={self.data.qpos[2]:.2f} m), resetting (band re-enabled, key 9 to release)")
            self.reset()
            if self.band is not None:
                self.band.enable = True

    def _log_row(self) -> None:
        d = self.data
        names = LEG_JOINTS + WAIST_JOINTS + ARM_JOINTS + NECK_JOINTS
        cmds = self.bridge.snapshot_cmd() if self.bridge is not None else {}
        row = [d.time, *d.qpos[:3], *d.qpos[3:7]]
        row += [d.qpos[self.qadr[n]] for n in names]
        row += [cmds[n][0] if n in cmds else float("nan") for n in names]
        self._log.write(",".join(f"{v:.6g}" for v in row) + "\n")

    def start(self) -> None:
        n, t0, last_report = 0, time.monotonic(), time.monotonic()
        view_every = max(1, int(round(self.viewer_dt / self.dt)))
        try:
            while (
                self._running
                and (self.bridge is None or self.bridge.ok())
                and (self.viewer is None or self.viewer.is_running())
            ):
                t_step = time.monotonic()
                self.step()
                if n % self.state_decim == 0:
                    self.publish_state()
                if self._log is not None and n % max(1, int(round(0.02 / self.dt))) == 0:
                    self._log_row()
                if self.viewer is not None and n % view_every == 0:
                    self.viewer.sync()
                n += 1
                sleep = self.dt - (time.monotonic() - t_step)
                if sleep > 0:
                    time.sleep(sleep)
                if self.verbose and n % int(round(1.0 / self.dt)) == 0:
                    R = self.data.xmat[self.root_id].reshape(3, 3)
                    tilt = float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
                    print(f"[a3-sim] t={self.data.time:6.1f}s pelvis_z={self.data.qpos[2]:.3f} tilt={tilt:5.1f}deg band={self.band.enable if self.band else None} cmd_rx={sum(self.bridge.cmd_rx.values()) if self.bridge else 0} "
                          f"R_shoulder(p,r)=({self.data.qpos[self.qadr['right_shoulder_pitch_joint']]:+.2f},{self.data.qpos[self.qadr['right_shoulder_roll_joint']]:+.2f}) R_elbow={self.data.qpos[self.qadr['right_elbow_joint']]:+.2f} "
                          f"xy=({self.data.qpos[0]:+.2f},{self.data.qpos[1]:+.2f}) head=({self.data.qpos[self.qadr['head_yaw_joint']]:+.3f},{self.data.qpos[self.qadr['head_pitch_joint']]:+.3f})")
                if time.monotonic() - last_report > 10.0:
                    rtf = n * self.dt / (time.monotonic() - t0)
                    rx = dict(self.bridge.cmd_rx) if self.bridge else {}
                    print(f"[a3-sim] t={self.data.time:6.1f}s rtf={rtf:.2f} cmd_rx={rx} band={self.band.enable if self.band else None}")
                    last_report = time.monotonic()
        except KeyboardInterrupt:
            print("A3 simulator interrupted by user.")
        finally:
            self.close()

    def close(self) -> None:
        self._running = False
        if getattr(self, "_log", None) is not None:
            self._log.close()
            self._log = None
        try:
            if self.viewer is not None:
                self.viewer.close()
        except Exception as e:  # noqa: BLE001
            print(f"Warning during close: {e}")
        if self.bridge is not None:
            self.bridge.close()

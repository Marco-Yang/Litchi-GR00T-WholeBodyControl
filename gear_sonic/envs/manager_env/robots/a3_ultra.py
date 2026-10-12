"""A3 Ultra (T3D0) robot configuration.

Kp/Kd, joint effort/velocity limits come from AgiBot's
docs/robot_parameters/A3_Ultra/A3_关节控制参数KpKd.docx (31 body joints: legs,
waist, arms, head). The hand (32 finger joints per both hands) has no vendor
Kp/Kd data and is intentionally left out of `actuators` below, so it is
unactuated by the training articulation — mirroring AgiBot's own
a3_ultra_t3d0.xml MJCF, whose <actuator> section likewise only drives these
same 31 body joints and leaves the fingers passive.

The default standing pose (init_state.joint_pos) and spawn height come from
the <keyframe> in gear_sonic/data/assets/robot_description/mjcf/a3_ultra_t3d0.xml
(a vendor-provided default qpos), not an estimate.

IsaacLab<->MuJoCo DOF/body index mappings were computed (not hand-guessed) by:
  1. BFS traversal of the URDF kinematic tree via Pinocchio, which was verified
     to exactly reproduce H2_ISAACLAB_JOINTS from gear_sonic/envs/manager_env/robots/h2.py
     before being trusted for a3_ultra.
  2. Joint/body declaration order read directly from a3_ultra_t3d0.xml via mujoco.
  3. A round-trip check (isaaclab -> mujoco -> isaaclab) over every joint and
     body name, which passed for all 63 DOF / 64 bodies.

Armature values come from docs/robot_parameters/A3_Ultra/armature.png:
  J_output = J_motor * N^2  (reflected rotor inertia at joint output)
  Parallel joints (ankle, waist_roll/pitch) use the equivalent serial-joint
  inertia from docs/robot_parameters/A3_Ultra/instructions.md.
  Per instructions.md note 2, a 3% domain-rand spread is recommended.
"""

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
import isaaclab.sim as sim_utils

ASSET_DIR = "gear_sonic/data/assets"

# "All bodies in IsaacLab traversal order (including root 'pelvis')" -- see
# gear_sonic/envs/manager_env/robots/h2.py / docs/source/user_guide/new_embodiments.md.
A3ULTRA_ISAACLAB_JOINTS = [
    "pelvis_link",
    "left_hip_pitch_Link",
    "right_hip_pitch_Link",
    "waist_yaw_Link",
    "left_hip_roll_Link",
    "right_hip_roll_Link",
    "waist_roll_Link",
    "left_hip_yaw_Link",
    "right_hip_yaw_Link",
    "torso_Link",
    "left_knee_Link",
    "right_knee_Link",
    "head_yaw_Link",
    "left_shoulder_pitch_Link",
    "right_shoulder_pitch_Link",
    "left_ankle_pitch_Link",
    "right_ankle_pitch_Link",
    "ultra_head_pitch_Link",
    "left_shoulder_roll_Link",
    "right_shoulder_roll_Link",
    "left_ankle_roll_Link",
    "right_ankle_roll_Link",
    "left_shoulder_yaw_Link",
    "right_shoulder_yaw_Link",
    "left_elbow_Link",
    "right_elbow_Link",
    "left_wrist_roll_Link",
    "right_wrist_roll_Link",
    "left_wrist_pitch_Link",
    "right_wrist_pitch_Link",
    "left_wrist_yaw_Link",
    "right_wrist_yaw_Link",
    "L_index_abad_link",
    "L_middle_pip_link",
    "L_pinky_abad_link",
    "L_ring_abad_link",
    "L_thumb_roll_link",
    "R_index_abad_link",
    "R_middle_pip_link",
    "R_pinky_abad_link",
    "R_ring_abad_link",
    "R_thumb_roll_link",
    "L_index_pip_link",
    "L_middle_dip_link",
    "L_pinky_pip_link",
    "L_ring_pip_link",
    "L_thumb_abad_link",
    "R_index_pip_link",
    "R_middle_dip_link",
    "R_pinky_pip_link",
    "R_ring_pip_link",
    "R_thumb_abad_link",
    "L_index_dip_link",
    "L_pinky_dip_link",
    "L_ring_dip_link",
    "L_thumb_mcp_link",
    "R_index_dip_link",
    "R_pinky_dip_link",
    "R_ring_dip_link",
    "R_thumb_mcp_link",
    "L_thumb_pip_link",
    "R_thumb_pip_link",
    "L_thumb_dip_link",
    "R_thumb_dip_link",
]

A3ULTRA_ISAACLAB_TO_MUJOCO_DOF = [
    51, 57, 0, 52, 58, 1, 53, 59, 2, 54, 60, 3, 5, 28, 55, 61, 4, 6, 29, 56, 62, 7, 30, 8, 31, 9,
    32, 10, 33, 11, 34, 17, 20, 25, 22, 12, 40, 43, 48, 45, 35, 18, 21, 26, 23, 13, 41, 44, 49, 46,
    36, 19, 27, 24, 14, 42, 50, 47, 37, 15, 38, 16, 39,
]

A3ULTRA_MUJOCO_TO_ISAACLAB_DOF = [
    2, 5, 8, 11, 16, 12, 17, 21, 23, 25, 27, 29, 35, 45, 54, 59, 61, 31, 41, 51, 32, 42, 34, 44,
    53, 33, 43, 52, 13, 18, 22, 24, 26, 28, 30, 40, 50, 58, 60, 62, 36, 46, 55, 37, 47, 39, 49, 57,
    38, 48, 56, 0, 3, 6, 9, 14, 19, 1, 4, 7, 10, 15, 20,
]

A3ULTRA_ISAACLAB_TO_MUJOCO_BODY = [
    0, 52, 58, 1, 53, 59, 2, 54, 60, 3, 55, 61, 4, 6, 29, 56, 62, 5, 7, 30, 57, 63, 8, 31, 9, 32,
    10, 33, 11, 34, 12, 35, 18, 21, 26, 23, 13, 41, 44, 49, 46, 36, 19, 22, 27, 24, 14, 42, 45, 50,
    47, 37, 20, 28, 25, 15, 43, 51, 48, 38, 16, 39, 17, 40,
]

A3ULTRA_MUJOCO_TO_ISAACLAB_BODY = [
    0, 3, 6, 9, 12, 17, 13, 18, 22, 24, 26, 28, 30, 36, 46, 55, 60, 62, 32, 42, 52, 33, 43, 35, 45,
    54, 34, 44, 53, 14, 19, 23, 25, 27, 29, 31, 41, 51, 59, 61, 63, 37, 47, 56, 38, 48, 40, 50, 58,
    39, 49, 57, 1, 4, 7, 10, 15, 20, 2, 5, 8, 11, 16, 21,
]

A3ULTRA_ISAACLAB_TO_MUJOCO_MAPPING = {
    "isaaclab_joints": A3ULTRA_ISAACLAB_JOINTS,
    "isaaclab_to_mujoco_dof": A3ULTRA_ISAACLAB_TO_MUJOCO_DOF,
    "mujoco_to_isaaclab_dof": A3ULTRA_MUJOCO_TO_ISAACLAB_DOF,
    "isaaclab_to_mujoco_body": A3ULTRA_ISAACLAB_TO_MUJOCO_BODY,
    "mujoco_to_isaaclab_body": A3ULTRA_MUJOCO_TO_ISAACLAB_BODY,
}

A3_ULTRA_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        fix_base=False,
        replace_cylinders_with_capsules=True,
        asset_path=f"{ASSET_DIR}/robot_description/urdf/a3_ultra/a3_ultra_t3d0.urdf",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
    ),
    # z=1.2 and all joint defaults below come from the <keyframe> in
    # gear_sonic/data/assets/robot_description/mjcf/a3_ultra_t3d0.xml (vendor default pose).
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.2),
        joint_pos={
            "left_hip_pitch_joint": -0.202414,
            "right_hip_pitch_joint": -0.202414,
            "left_hip_roll_joint": 0.010857,
            "right_hip_roll_joint": -0.010857,
            "left_hip_yaw_joint": -0.062196,
            "right_hip_yaw_joint": 0.062196,
            "left_knee_joint": 0.236535,
            "right_knee_joint": 0.236535,
            "left_ankle_pitch_joint": -0.113223,
            "right_ankle_pitch_joint": -0.113223,
            "left_ankle_roll_joint": -0.013529,
            "right_ankle_roll_joint": 0.013529,
            "waist_yaw_joint": 0.003413,
            "waist_roll_joint": -0.007723,
            "waist_pitch_joint": -0.056826,
            "left_shoulder_pitch_joint": 0.290207,
            "right_shoulder_pitch_joint": 0.290207,
            "left_shoulder_roll_joint": 0.105228,
            "right_shoulder_roll_joint": -0.105228,
            "left_shoulder_yaw_joint": -0.013017,
            "right_shoulder_yaw_joint": 0.013017,
            "left_elbow_joint": 0.832663,
            "right_elbow_joint": 0.832663,
            "left_wrist_roll_joint": -0.720453,
            "right_wrist_roll_joint": 0.720453,
            "left_wrist_pitch_joint": 0.010952,
            "right_wrist_pitch_joint": 0.010952,
            "left_wrist_yaw_joint": 0.009157,
            "right_wrist_yaw_joint": -0.009157,
            "head_yaw_joint": -0.013721,
            "ultra_head_pitch_joint": 0.014967,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        # PFP-93-65: J_motor=138.5069e-6, N=21.906 → J_output=0.066466 kg·m²
        # Kp=200, Kd=7, effort=220 Nm, vel=12.04 rad/s
        "hip": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_pitch_joint", ".*_hip_roll_joint", ".*_hip_yaw_joint",
            ],
            effort_limit_sim=220.0,
            velocity_limit_sim=12.04,
            stiffness=200.0,
            damping=7.0,
            armature=0.066466,
        ),
        # PFP-110-75: J_motor=300.8507171e-6, N=20 → J_output=0.120340 kg·m²
        # Kp=360, Kd=13, effort=320 Nm, vel=14.66 rad/s
        "knee": ImplicitActuatorCfg(
            joint_names_expr=[".*_knee_joint"],
            effort_limit_sim=320.0,
            velocity_limit_sim=14.66,
            stiffness=360.0,
            damping=13.0,
            armature=0.120340,
        ),
        # PFP-78-58 parallel: J_base=0.012083, mult=5.333 → 0.064441 kg·m²
        # Kp=80, Kd=3, effort=118.2 Nm, vel=10.80 rad/s
        "ankle_pitch": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch_joint"],
            effort_limit_sim=118.2,
            velocity_limit_sim=10.80,
            stiffness=80.0,
            damping=3.0,
            armature=0.064441,
        ),
        # PFP-78-58 parallel: J_base=0.012083, mult=1.66562 → 0.020126 kg·m²
        # Kp=80, Kd=3, effort=54.75 Nm, vel=19.37 rad/s
        "ankle_roll": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_roll_joint"],
            effort_limit_sim=54.75,
            velocity_limit_sim=19.37,
            stiffness=80.0,
            damping=3.0,
            armature=0.020126,
        ),
        # PFP-93-65: same motor as hip → J_output=0.066466 kg·m²
        # Kp=200, Kd=7, effort=220 Nm, vel=12.04 rad/s
        "waist_yaw": ImplicitActuatorCfg(
            joint_names_expr=["waist_yaw_joint"],
            effort_limit_sim=220.0,
            velocity_limit_sim=12.04,
            stiffness=200.0,
            damping=7.0,
            armature=0.066466,
        ),
        # PFP-78-58 parallel: J_base=0.012083, mult=7.3 → 0.088209 kg·m²
        # Kp=80, Kd=3, effort=118 Nm (waist_pitch equivalent), vel=9.25 rad/s
        "waist_pitch": ImplicitActuatorCfg(
            joint_names_expr=["waist_pitch_joint"],
            effort_limit_sim=118.0,
            velocity_limit_sim=9.25,
            stiffness=80.0,
            damping=3.0,
            armature=0.088209,
        ),
        # PFP-78-58 parallel: J_base=0.012083, mult=1.21 → 0.014621 kg·m²
        # Kp=80, Kd=3, effort=46 Nm (waist_roll equivalent), vel=22.70 rad/s
        "waist_roll": ImplicitActuatorCfg(
            joint_names_expr=["waist_roll_joint"],
            effort_limit_sim=46.0,
            velocity_limit_sim=22.70,
            stiffness=80.0,
            damping=3.0,
            armature=0.014621,
        ),
        # PFP-78-58 serial: J_motor=30.118e-6, N=20.03 → J_output=0.012083 kg·m²
        # Kp=90, Kd=4, effort=60 Nm, vel=13.61 rad/s
        "shoulder": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_pitch_joint", ".*_shoulder_roll_joint"],
            effort_limit_sim=60.0,
            velocity_limit_sim=13.61,
            stiffness=90.0,
            damping=4.0,
            armature=0.012083,
        ),
        # PFP-59-60: J_motor=10.1374e-6, N=22.136 → J_output=0.004967 kg·m²
        # Kp=40, Kd=3, effort=24 Nm, vel=15.71 rad/s
        "shoulder_yaw_elbow": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_yaw_joint", ".*_elbow_joint"],
            effort_limit_sim=24.0,
            velocity_limit_sim=15.71,
            stiffness=40.0,
            damping=3.0,
            armature=0.004967,
        ),
        # PFP-41-48: J_motor=1.359e-6, N=24.415 → J_output=0.000810 kg·m²
        # Kp=40, Kd=3, effort=24 Nm, vel=15.71 rad/s
        "wrist_roll": ImplicitActuatorCfg(
            joint_names_expr=[".*_wrist_roll_joint"],
            effort_limit_sim=24.0,
            velocity_limit_sim=15.71,
            stiffness=40.0,
            damping=3.0,
            armature=0.000810,
        ),
        # PFP-41-48: same motor → J_output=0.000810 kg·m²
        # Kp=15, Kd=1, effort=6 Nm, vel=12.78 rad/s
        "wrist_pitch_yaw": ImplicitActuatorCfg(
            joint_names_expr=[".*_wrist_pitch_joint", ".*_wrist_yaw_joint"],
            effort_limit_sim=6.0,
            velocity_limit_sim=12.78,
            stiffness=15.0,
            damping=1.0,
            armature=0.000810,
        ),
        # PFP-41-48: same motor → J_output=0.000810 kg·m²
        # Kp=40, Kd=2, effort=6 Nm, vel=12.78 rad/s
        "head": ImplicitActuatorCfg(
            joint_names_expr=["head_yaw_joint", "ultra_head_pitch_joint"],
            effort_limit_sim=6.0,
            velocity_limit_sim=12.78,
            stiffness=40.0,
            damping=2.0,
            armature=0.000810,
        ),
        # NOTE: no "hands" actuator group. The 32 finger joints (16 per hand) have no
        # vendor Kp/Kd and are left unactuated here -- same as AgiBot's own
        # a3_ultra_t3d0.xml MJCF, whose <actuator> section only drives these 31 body
        # joints. They must also be excluded from manager_env.actions.joint_pos.joint_names
        # in the experiment config (see sonic_a3_ultra.yaml), otherwise IsaacLab's default
        # joint_names=[".*"] action term would include them in the RL action space.
    },
)

A3_ULTRA_ACTION_SCALE = {}
for a in A3_ULTRA_CFG.actuators.values():
    e = a.effort_limit_sim
    s = a.stiffness
    names = a.joint_names_expr
    if not isinstance(e, dict):
        e = dict.fromkeys(names, e)
    if not isinstance(s, dict):
        s = dict.fromkeys(names, s)
    for n in names:
        if n in e and n in s and s[n]:
            A3_ULTRA_ACTION_SCALE[n] = 0.25 * e[n] / s[n]

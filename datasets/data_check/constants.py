"""Shared constants for walking dataset mode-2 Isaac stability checks."""

from __future__ import annotations

# observation.state layout (RobotModel / LeRobot modality.json)
LEFT_LEG = slice(0, 6)
RIGHT_LEG = slice(6, 12)
WAIST = slice(12, 15)
LEFT_ARM = slice(15, 22)
LEFT_HAND = slice(22, 29)
RIGHT_ARM = slice(29, 36)
RIGHT_HAND = slice(36, 43)

# 29-D body DOF without Dex3 hands (MuJoCo / deploy actuated body order used in state)
BODY_DOF_SLICES = (LEFT_LEG, RIGHT_LEG, WAIST, LEFT_ARM, RIGHT_ARM)

# Balance-critical indices inside observation.state (legs + waist + ankles emphasized)
BALANCE_STATE_INDICES = list(range(0, 15))  # legs + waist
ANKLE_STATE_INDICES = [4, 5, 10, 11]  # ankle pitch/roll L/R in 6-D leg groups
KNEE_STATE_INDICES = [3, 9]

# IsaacLab wrist DOF indices (deploy ZMQ joint_pos / smpl encoder obs joints_idx)
# Order: L_roll, R_roll, L_pitch, R_pitch, L_yaw, R_yaw
ISAACLAB_WRIST_DOF_INDICES = (23, 24, 25, 26, 27, 28)
LEFT_WRIST_IN_BODY29 = [19, 20, 21]  # MuJoCo DFS body29
RIGHT_WRIST_IN_BODY29 = [26, 27, 28]

# From gear_sonic/envs/manager_env/robots/g1.py — isaaclab_dof[i] → slot in mujoco order
# mujoco_ordered = isaaclab_ordered[G1_ISAACLAB_TO_MUJOCO_DOF]
G1_ISAACLAB_TO_MUJOCO_DOF = (
    0,
    3,
    6,
    9,
    13,
    17,
    1,
    4,
    7,
    10,
    14,
    18,
    2,
    5,
    8,
    11,
    15,
    19,
    21,
    23,
    25,
    27,
    12,
    16,
    20,
    22,
    24,
    26,
    28,
)

G1_STANDING_ROOT_Z = 0.78

STATE_JOINT_NAMES_43 = (
    [f"left_leg_{i}" for i in range(6)]
    + [f"right_leg_{i}" for i in range(6)]
    + [f"waist_{i}" for i in range(3)]
    + [f"left_arm_{i}" for i in range(7)]
    + [f"left_hand_{i}" for i in range(7)]
    + [f"right_arm_{i}" for i in range(7)]
    + [f"right_hand_{i}" for i in range(7)]
)

DEFAULT_TASK_PROMPT = "从初始位置出发走到绿色桌子前面"
DEFAULT_FPS = 50.0
SMPL_NUM_JOINTS = 24
SMPL_POSE_DIM = 63  # 21 * 3 axis-angle
HAND_DIM = 7
BODY_DOF_DIM = 29
STATE_DIM = 43

# G1 hinge axes in MuJoCo actuator / LeRobot body29 order
# (from gear_sonic/data/assets/robot_description/mjcf/g1_29dof_rev_1_0.xml).
# pose_aa body joints must be dof_angle * axis — putting everything on +X breaks FK.
G1_BODY29_DOF_AXES = (
    (0, 1, 0),  # left_hip_pitch
    (1, 0, 0),  # left_hip_roll
    (0, 0, 1),  # left_hip_yaw
    (0, 1, 0),  # left_knee
    (0, 1, 0),  # left_ankle_pitch
    (1, 0, 0),  # left_ankle_roll
    (0, 1, 0),  # right_hip_pitch
    (1, 0, 0),  # right_hip_roll
    (0, 0, 1),  # right_hip_yaw
    (0, 1, 0),  # right_knee
    (0, 1, 0),  # right_ankle_pitch
    (1, 0, 0),  # right_ankle_roll
    (0, 0, 1),  # waist_yaw
    (1, 0, 0),  # waist_roll
    (0, 1, 0),  # waist_pitch
    (0, 1, 0),  # left_shoulder_pitch
    (1, 0, 0),  # left_shoulder_roll
    (0, 0, 1),  # left_shoulder_yaw
    (0, 1, 0),  # left_elbow
    (1, 0, 0),  # left_wrist_roll
    (0, 1, 0),  # left_wrist_pitch
    (0, 0, 1),  # left_wrist_yaw
    (0, 1, 0),  # right_shoulder_pitch
    (1, 0, 0),  # right_shoulder_roll
    (0, 0, 1),  # right_shoulder_yaw
    (0, 1, 0),  # right_elbow
    (1, 0, 0),  # right_wrist_roll
    (0, 1, 0),  # right_wrist_pitch
    (0, 0, 1),  # right_wrist_yaw
)

# Jump thresholds (rad / m) at 50 Hz teleop.
# Policy (sim eyeball): local single-joint glitches are OK to keep; do NOT loosen
# whole-body stumble / fall proxies (gravity_tilt). Isaac fall terms stay separate.
THRESHOLDS = {
    # --- local / single-joint (loose; teleop+Pico noise) ---
    "state_dq_rad": 1.00,  # any-joint |Δq|
    "state_ddq_rad": 1.80,  # second-diff spike
    "smpl_joint_m": 0.50,  # single SMPL joint jump (m)
    "smpl_pose_rad": 1.20,  # SMPL aa jump
    "hand_dq_rad": 1.20,
    "balance_dq_rad": 0.90,  # single ankle/waist/knee spike (not whole-body fall)
    # --- stumble / fall proxy on recorded robot (strict; do not loosen) ---
    "gravity_tilt": 0.35,  # |Δ projected_gravity|; large tilt / stumble
}

# Isaac im_eval quality screen (per-episode, from metrics_eval.json
# ``eval/all_metrics_dict``). Tuned for cleaner training sets: prefer cutting
# high accel/vel (jitter / jump proxies) and the tracking tail, not just falls.
# Anchored on walking_new teleop mode-2 (newer sessions mpjpe_l≈160–190 /
# joint_mse_l≈0.04; older ≈230–246 / 0.07). Any hit → drop from LeRobot.
ISAAC_QUALITY_THRESHOLDS = {
    "drop_if_terminated": True,
    "min_progress": 0.99,
    "max_mpjpe_l": 250.0,  # mm local MPJPE (was 320)
    "max_mpjpe_pa": 96.0,  # mm PA (was 130)
    "max_joint_mse_l": 0.078,  # rad^2 (was 0.12)
    "max_accel_dist": 1.75,  # jump / jitter proxy (was 3.5)
    "max_vel_dist": 11.0,  # jump / jitter proxy (was 18)
}

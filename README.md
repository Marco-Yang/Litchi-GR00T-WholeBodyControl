<div align="center">

  <img src="media/groot_wbc.png" width="800" alt="GEAR SONIC Header">

  <!-- --- -->
  
  
</div>

<div align="center">

[![License](https://img.shields.io/badge/License-Apache%202.0-76B900.svg)](LICENSE)
[![IsaacLab](https://img.shields.io/badge/IsaacLab-2.3.2-orange.svg)](https://github.com/isaac-sim/IsaacLab/releases/tag/v2.3.2)
[![Documentation](https://img.shields.io/badge/docs-GitHub%20Pages-76B900.svg)](https://nvlabs.github.io/GR00T-WholeBodyControl/)
[![Demo](https://img.shields.io/badge/Live%20Demo-GEAR--SONIC-blue.svg)](https://nvlabs.github.io/GEAR-SONIC/demo.html)

</div>

---




# GR00T-WholeBodyControl

This is the codebase for the **GR00T Whole-Body Control (WBC)** projects. It hosts model checkpoints and scripts for training, evaluating, and deploying advanced whole-body controllers for humanoid robots. We currently support:

- **Decoupled WBC**: the decoupled controller (RL for lower body, and IK for upper body) used in NVIDIA GR00T [N1.5](https://research.nvidia.com/labs/gear/gr00t-n1_5/) and [N1.6](https://research.nvidia.com/labs/gear/gr00t-n1_6/) models;
- **GEAR-SONIC Series**: our latest iteration of generalist humanoid whole-body controllers (see our [whitepaper](https://nvlabs.github.io/GEAR-SONIC/));
- **MotionBricks**: a real-time latent generative model for interactive motion control in animation and robotics (see the [project page](https://nvlabs.github.io/motionbricks/)).

## News

- **[2026-08-31]** **SONIC deployment update** — added per-motor Kp/Kd scaling to reduce stumbling.
- **[2026-07-23]** **SONIC v1.1 checkpoint** — released a robot-heading-normalized SONIC controller trained with wrist-pose augmentation for whole-body teleoperation and SONIC-backed VLA execution. See the [Model Card](#model-card) and [Download Models](https://nvlabs.github.io/GR00T-WholeBodyControl/getting_started/download_models.html#sonic-v11-checkpoint).
- **[06/16]** **Isaac Teleop Setup (CloudXR / DeviceIO, in-process)** — added bring-up docs for the in-process CloudXR path via `isaacteleop[cloudxr]`, with no separate publisher container. See [Isaac Teleop Setup](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/isaac_teleop_publisher_setup.html).
- **[2026-06-16]** **Low-latency teleoperation checkpoint** — released a SONIC checkpoint with 4-frame SMPL reference lookahead for more responsive whole-body teleoperation. See the [Model Card](#model-card), [Download Models](https://nvlabs.github.io/GR00T-WholeBodyControl/getting_started/download_models.html#low-latency-teleoperation-checkpoint), and [VLA Inference](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/vla_inference.html#low-latency-teleoperation-checkpoint).
- **[2026-05-07]** 🤖 **End-to-end VLA workflow on G1** — collect teleop data, fine-tune Isaac-GR00T N1.7, and deploy with SONIC whole-body control. See [Data Collection](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/data_collection.html), [VLA Workflow](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/vla_workflow.html), and [VLA Inference](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/vla_inference.html).
- **[2026-04-27]** 🧩 **MotionBricks preview** — interactive G1 demo, pretrained checkpoints (VQVAE · pose · root), synthetic training code, and motion-representation docs. See [`motionbricks/`](motionbricks/) and the [project page](https://nvlabs.github.io/motionbricks/).
- **[2026-04-14]** 🌐 **[Live web demo](https://nvlabs.github.io/GEAR-SONIC/demo.html)** — try SONIC interactively in your browser. Features [Kimodo](https://github.com/nv-tlabs/kimodo) text-to-motion generation.
- **[2026-04-10]** 🚀 Released **SONIC training code and checkpoint** on [HuggingFace](https://huggingface.co/nvidia/GEAR-SONIC). Train from scratch or finetune. **Additional embodiment support** and **VLA data collection pipeline**. See [Training Guide](https://nvlabs.github.io/GR00T-WholeBodyControl/user_guide/training.html).
- **[2026-03-24]** 🔧 C++ inference stack update: motor error monitoring, TTS alerts, ZMQ protocol v4, idle-mode readaptation. **ZMQ header size changed to 1280 bytes.**
- **[2026-03-16]** 📦 [BONES-SEED](https://huggingface.co/datasets/bones-studio/seed) open-sourced — 142K+ human motions (~288 hours) with G1 MuJoCo trajectories.
- **[2026-02-19]** 🎉 Released GEAR-SONIC: pretrained checkpoints, C++ inference, VR teleoperation, and documentation.
- **[2025-11-12]** 🏁 Initial release with Decoupled WBC for GR00T N1.5 and N1.6.

## Table of Contents

- [News](#news)
- [GEAR-SONIC](#gear-sonic)
- [Model Card](#model-card)
- [VR Whole-Body Teleoperation](#vr-whole-body-teleoperation)
- [Kinematic Planner](#kinematic-planner)
- [SONIC Training](#sonic-training)
- [AgiBot A3 / A3 Ultra (`--robot`)](#agibot-a3--a3-ultra---robot)
- [TODOs](#todos)
- [What's Included](#whats-included)
  - [Setup](#setup)
- [Documentation](#documentation)
- [Citation](#citation)
- [License](#license)
- [Support](#support)
- [MotionBricks](#motionbricks)
- [Decoupled WBC](#decoupled-wbc)


## GEAR-SONIC 

<p style="font-size: 1.2em;">
    <a href="https://nvlabs.github.io/GEAR-SONIC/"><strong>Website</strong></a> | 
    <a href="https://huggingface.co/nvidia/GEAR-SONIC"><strong>Model</strong></a> | 
    <a href="https://arxiv.org/abs/2511.07820"><strong>Paper</strong></a> | 
    <a href="https://nvlabs.github.io/GR00T-WholeBodyControl/"><strong>Docs</strong></a>
  </p>

<div align="center">
  <img src="docs/source/_static/sonic-preview-gif-480P.gif" width="800" >
  
</div>

SONIC is a humanoid behavior foundation model that gives robots a core set of motor skills learned from large-scale human motion data. Rather than building separate controllers for predefined motions, SONIC uses motion tracking as a scalable training task, enabling a single unified policy to produce natural, whole-body movement and support a wide range of behaviors — from walking and crawling to teleoperation and multi-modal control. It is designed to generalize beyond the motions it has seen during training and to serve as a foundation for higher-level planning and interaction.

In this repo, we release SONIC's training code, deployment framework, model checkpoints, and teleoperation stack for data collection.

## Model Card

SONIC provides three released Unitree G1 checkpoints. Choose the model based on its reference representation and intended deployment.

### Available Models

| Model | Hugging Face location | SMPL reference input | Intended use and comments |
|---|---|---|---|
| **Default SONIC (original release)** | Top-level `model_encoder.onnx`, `model_decoder.onnx`, and `observation_config.yaml`; training checkpoint at `sonic_release/last.pt` | 10 future frames at 20 ms spacing, approximately 200 ms of reference lookahead | Default general-purpose SONIC controller for motion tracking, planning, teleoperation, and compatibility with existing deployments. G1 and teleoperation future-reference observations use `step5`. |
| **Low-latency teleoperation** | [`low_latency/`](https://huggingface.co/nvidia/GEAR-SONIC/tree/main/low_latency) | 4 future frames at 20 ms spacing, approximately 80 ms of reference lookahead | Intended for more responsive whole-body teleoperation and VLA execution. G1 and teleoperation future-reference observations use `step1`. Use its encoder, decoder, and observation config together. |
| **SONIC v1.1** | [`sonic_v1_1/`](https://huggingface.co/nvidia/GEAR-SONIC/tree/main/sonic_v1_1) | 10 future frames at 20 ms spacing, approximately 200 ms of reference lookahead | Uses robot-heading-normalized target orientation and was trained with wrist-pose augmentation. Intended for heading-stable whole-body teleoperation and SONIC-backed VLA policies that use this controller. G1 and teleoperation future-reference observations use `step5`; this is not the low-latency model. |

All three models use the SONIC universal-token controller, produce 64-dimensional latent motion tokens, run the controller at 50 Hz, and support SMPL pose, G1 motion reference, and teleoperation inputs. Deployment uses C++ and TensorRT; the PyTorch checkpoints support Isaac Lab evaluation and continued training.

The lookahead values describe the reference horizon presented to the controller. They are **not** measurements of total end-to-end teleoperation latency, which also includes sensing, networking, preprocessing, and inference. Model weights are covered by the [NVIDIA Open Model License](LICENSE).

### Released Files

| Model | Deployment files | PyTorch and configuration files |
|---|---|---|
| Default SONIC | `model_encoder.onnx`, `model_decoder.onnx`, `observation_config.yaml` | `sonic_release/last.pt`, `sonic_release/config.yaml` |
| Low-latency teleoperation | `low_latency/model_encoder.onnx`, `low_latency/model_decoder.onnx`, `low_latency/observation_config.yaml` | `low_latency/last.pt`, `low_latency/config.yaml`, `low_latency/model_config.yaml` |
| SONIC v1.1 | `sonic_v1_1/model_encoder.onnx`, `sonic_v1_1/model_decoder.onnx`, `sonic_v1_1/observation_config.yaml` | `sonic_v1_1/last.pt`, `sonic_v1_1/config.yaml`, `sonic_v1_1/model_config.yaml` |

The root [`config.json`](https://huggingface.co/nvidia/GEAR-SONIC/blob/main/config.json)
is the canonical release manifest for these variants and shared artifacts. The
official downloader fetches and validates it before downloading model files.

### Usage

Download the default model and planner:

```bash
python download_from_hf.py
```

Download the low-latency teleoperation model and planner:

```bash
python download_from_hf.py --low-latency
```

Download SONIC v1.1 and the planner:

```bash
python download_from_hf.py --sonic-v1-1
```

Run the default C++ deployment stack:

```bash
cd gear_sonic_deploy
./deploy.sh --input-type zmq_manager real
```

Run the low-latency C++ deployment stack:

```bash
cd gear_sonic_deploy
./deploy.sh \
    --cp policy/low_latency/model \
    --obs-config policy/low_latency/observation_config.yaml \
    --input-type zmq_manager \
    real
```

Run the SONIC v1.1 C++ deployment stack:

```bash
cd gear_sonic_deploy
./deploy.sh \
    --cp policy/sonic_v1_1/model \
    --obs-config policy/sonic_v1_1/observation_config.yaml \
    --input-type zmq_manager \
    --motor-kp-scale 4,10=1.5 \
    --motor-kd-scale 4,10=1.5 \
    real
```

This is the tested SONIC v1.1 deployment tuning. Hardware indices `4` and
`10` are the left and right ankle-pitch motors; increasing their control gains
improves whole-body stability and, in practice, wrist tracking. The flags are
opt-in and do not alter other checkpoints.

Run the default Python VLA launcher, which orchestrates the C++ controller and Python inference client:

```bash
python gear_sonic/scripts/launch_inference.py \
    --camera-host 192.168.123.164 \
    --prompt "pick up the cup"
```

For the low-latency model, add the matching deployment files:

```bash
python gear_sonic/scripts/launch_inference.py \
    --deploy-checkpoint policy/low_latency/model \
    --deploy-obs-config policy/low_latency/observation_config.yaml \
    --camera-host 192.168.123.164 \
    --prompt "pick up the cup"
```

For SONIC v1.1, use the matching model/config and gain tuning:

```bash
python gear_sonic/scripts/launch_inference.py \
    --deploy-checkpoint policy/sonic_v1_1/model \
    --deploy-obs-config policy/sonic_v1_1/observation_config.yaml \
    --deploy-motor-kp-scale 4,10=1.5 \
    --deploy-motor-kd-scale 4,10=1.5 \
    --camera-host 192.168.123.164 \
    --prompt "pick up the cup"
```

See [Downloading Model Checkpoints](docs/source/getting_started/download_models.md#sonic-v11-checkpoint) for Python checkpoint evaluation and additional deployment options. Test in simulation before using the checkpoint on a physical robot.


## VR Whole-Body Teleoperation

SONIC supports real-time whole-body teleoperation via PICO VR headset, enabling natural human-to-robot motion transfer for data collection and interactive control.

<div align="center">
  <img src="docs/source/_static/sonic_v1_1_demo.gif" width="800" alt="SONIC v1.1 whole-body teleoperation demo">
</div>

<p align="center"><em><strong>SONIC v1.1:</strong> Whole-body teleoperation mode with expressive wrist motion and dynamic movement.</em></p>

<div align="center">
  <img src="docs/source/_static/sonic_low_latency_demo.gif" width="640" alt="SONIC Low Latency whole-body teleoperation and ground pickup">
</div>

<p align="center"><em><strong>SONIC Low Latency:</strong> Whole-body teleoperation mode with a successful ground pickup.</em></p>

This repo can also drive the headset over Isaac Teleop / CloudXR by launching `gear_sonic/scripts/pico_manager_thread_server.py --input-source isaac-teleop`. The streamer hosts the CloudXR runtime in-process via `isaacteleop[cloudxr]` — no separate publisher container required. That path is currently documented and supported only for **G1 with a Thor backpack**. The Isaac Teleop bring-up steps are documented in [`docs/source/tutorials/isaac_teleop_publisher_setup.md`](docs/source/tutorials/isaac_teleop_publisher_setup.md).

<div align="center">
<table>
<tr>
<td align="center"><b>Walking</b></td>
<td align="center"><b>Running</b></td>
</tr>
<tr>
<td align="center"><img src="media/teleop_walking.gif" width="400"></td>
<td align="center"><img src="media/teleop_running.gif" width="400"></td>
</tr>
<tr>
<td align="center"><b>Sideways Movement</b></td>
<td align="center"><b>Kneeling</b></td>
</tr>
<tr>
<td align="center"><img src="media/teleop_sideways.gif" width="400"></td>
<td align="center"><img src="media/teleop_kneeling.gif" width="400"></td>
</tr>
<tr>
<td align="center"><b>Getting Up</b></td>
<td align="center"><b>Jumping</b></td>
</tr>
<tr>
<td align="center"><img src="media/teleop_getup.gif" width="400"></td>
<td align="center"><img src="media/teleop_jumping.gif" width="400"></td>
</tr>
<tr>
<td align="center"><b>Bimanual Manipulation</b></td>
<td align="center"><b>Object Hand-off</b></td>
</tr>
<tr>
<td align="center"><img src="media/teleop_bimanual.gif" width="400"></td>
<td align="center"><img src="media/teleop_switch_hands.gif" width="400"></td>
</tr>
</table>
</div>

## Kinematic Planner

SONIC includes a kinematic planner for real-time locomotion generation — choose a movement style, steer with keyboard/gamepad, and adjust speed and height on the fly.

<div align="center">
<table>
<tr>
<td align="center" colspan="2"><b>In-the-Wild Navigation</b></td>
</tr>
<tr>
<td align="center" colspan="2"><img src="media/planner/planner_in_the_wild_navigation.gif" width="800"></td>
</tr>
<tr>
<td align="center"><b>Run</b></td>
<td align="center"><b>Happy</b></td>
</tr>
<tr>
<td align="center"><img src="media/planner/planner_run.gif" width="400"></td>
<td align="center"><img src="media/planner/planner_happy.gif" width="400"></td>
</tr>
<tr>
<td align="center"><b>Stealth</b></td>
<td align="center"><b>Injured</b></td>
</tr>
<tr>
<td align="center"><img src="media/planner/planner_stealth.gif" width="400"></td>
<td align="center"><img src="media/planner/planner_injured.gif" width="400"></td>
</tr>
<tr>
<td align="center"><b>Kneeling</b></td>
<td align="center"><b>Hand Crawling</b></td>
</tr>
<tr>
<td align="center"><img src="media/planner/planner_kneeling.gif" width="400"></td>
<td align="center"><img src="media/planner/planner_hand_crawling.gif" width="400"></td>
</tr>
<tr>
<td align="center"><b>Elbow Crawling</b></td>
<td align="center"><b>Boxing</b></td>
</tr>
<tr>
<td align="center"><img src="media/planner/planner_elbow_crawling.gif" width="400"></td>
<td align="center"><img src="media/planner/planner_boxing.gif" width="400"></td>
</tr>
</table>
</div>

## SONIC Training

SONIC can be trained from scratch on the [Bones-SEED](https://huggingface.co/datasets/bones-studio/seed)
motion capture dataset (142K+ motions, ~288 hours, Unitree G1 retargeted), or finetuned
from the released checkpoint on [Hugging Face](https://huggingface.co/nvidia/GEAR-SONIC).

### Quick start

```bash
# Install training dependencies (Isaac Lab must be installed separately — see docs)
pip install -e "gear_sonic/[training]"

# Download checkpoint + SMPL data from Hugging Face
pip install huggingface_hub
python download_from_hf.py --training

# Download Bones-SEED G1 CSVs from huggingface.co/datasets/bones-studio/seed, then convert and filter
python gear_sonic/data_process/convert_soma_csv_to_motion_lib.py \
    --input /path/to/bones_seed/g1/csv/ \
    --output data/motion_lib_bones_seed/robot --fps 30 --fps_source 120 --individual --num_workers 16
python gear_sonic/data_process/filter_and_copy_bones_data.py \
    --source data/motion_lib_bones_seed/robot --dest data/motion_lib_bones_seed/robot_filtered

# Finetune from released checkpoint (64+ GPUs recommended)
accelerate launch --num_processes=8 gear_sonic/train_agent_trl.py \
    +exp=manager/universal_token/all_modes/sonic_release \
    +checkpoint=sonic_release/last.pt \
    num_envs=4096 headless=True \
    ++manager_env.commands.motion.motion_lib_cfg.motion_file=data/motion_lib_bones_seed/robot_filtered \
    ++manager_env.commands.motion.motion_lib_cfg.smpl_motion_file=data/smpl_filtered
```

Adaptive sampling is enabled by default and attributes failures to the motion
cursor tracked before environment reset.

For the full guide including multi-node training, evaluation, ONNX export, and SOMA encoder setup:
📖 [Installation (Training)](https://nvlabs.github.io/GR00T-WholeBodyControl/getting_started/installation_training.html) |
[Training Guide](https://nvlabs.github.io/GR00T-WholeBodyControl/user_guide/training.html)

## AgiBot A3 / A3 Ultra (`--robot`)

This repository also carries the AgiBot A3 adaptation of SONIC from
[Litchi_sonic_for_a3](https://agibottech.github.io/sonic_for_a3/) (035 / step-200k
checkpoint, MuJoCo sim2sim, AimRT/ROS 2 C++ deploy with ONNX Runtime and RKNN backends).
Training and deployment select the robot with one flag:

| `--robot` | Training experiment | Deploy (`gear_sonic_deploy/deploy.sh`) | Notes |
|---|---|---|---|
| `g1` (default) | `sonic_v1_1` | Original G1 path (TensorRT + `unitree_sdk2`) | Unchanged |
| `a3` | `sonic_a3_035` | `deploy_a3.sh --robot a3` (ONNX Runtime CPU / RKNN) | AgiBot A3 (T2D5), 29 DOF, 035 contract |
| `a3_ultra` | `sonic_a3_ultra` | `deploy_a3.sh --robot a3_ultra` | AgiBot A3 Ultra (T3D0). **Deploys the downloaded 035 weights** (officially stated to support A3 and A3 Ultra); head joints fixed at their default pose, fingers not driven. **Trains the A3 contract**: `sonic_a3_ultra` inherits `sonic_a3_035` (A3 URDF, a3_024 parameters, domain randomization tuned for A3 *and* A3 Ultra, 29 policy joints) on the Bones-SEED library retargeted onto the A3 itself (`batch_retarget_bvh_soma_a3ultra.py --robot a3`, `a3_29` format). The Ultra's own 63-DOF contract is `+exp=manager/universal_token/all_modes/sonic_a3_ultra_t3d0` (not deployable) |
| `h2` | `sonic_h2` | not available (no C++ backend yet) | |

For `a3_ultra`, the joint-level control parameters (Kp/Kd, effort/velocity limits, armature, friction,
default pose, action scale) are the Litchi A3 `a3_024` contract everywhere -- training
(`robots/a3_ultra.py`, `A3_ULTRA_GAIN_SOURCE = "litchi_a3_024"`), the C++ deploy profile `a3_024`, and the
MuJoCo simulator (which receives them from the deploy binary). AgiBot's A3 Ultra Kp/Kd document is
kept in `a3_ultra.py` but is not used (`"vendor_doc"`). `tests/test_a3_ultra_contract.py` keeps the three in sync.

### Quick start

```bash
# 1. Weights of the official 035 step-200k checkpoint (size + SHA256 verified, `--list` previews)
python download_from_hf.py --robot a3                       # default: ONNX + RKNN deploy models (~254 MB)
python download_from_hf.py --robot a3 --component pt        # PyTorch checkpoint (~402 MB)
python download_from_hf.py --robot a3 --component all       # also the RK3588 cross-compile sysroot

# 2. MuJoCo sim2sim with the downloaded PT (regression gate, not a safety proof)
python gear_sonic/scripts/sim2sim_a3_mujoco.py \
    --checkpoint checkpoints/035_step200000/model_step_200000.pt \
    --motion a3_data/agibot_a3/001_walk_front_slow.csv \
    --encoder-mode a3_fast --batch-once

# 3. Deploy: downloads ONNX if missing, builds dist/a3_deploy_<arch>/, then launches
gear_sonic_deploy/deploy.sh --robot a3 -- --probe          # receive + inference, no commands sent
gear_sonic_deploy/deploy.sh --robot a3_ultra --build-only  # build the package only

# 4. Training: the robot flag expands to +exp=manager/universal_token/all_modes/<exp>
accelerate launch --num_processes=1 gear_sonic/train_agent_trl.py --robot a3 \
    checkpoint=checkpoints/035_step200000/model_step_200000.pt +resume=false \
    headless=true num_envs=4096 \
    "++manager_env.commands.motion.motion_lib_cfg.motion_file=[/abs/path/to/a3_motionlib]"
# once: retarget Bones-SEED onto the A3 with the A3 parameters (a3_29 library, 129k clips, ~2 h with 16 workers)
python gear_sonic/data_process/batch_retarget_bvh_soma_a3ultra.py --robot a3 --tar data/bones_seed/soma_proportional.tar.gz \
    --output data/motion_lib_a3_29d/robot_filtered_v15_clean --only_from data/motion_lib_a3ultra/robot_filtered_v15_clean --workers 16 \
    --keep_heading   # keep the capture heading (the frame of the SMPL data); without it every clip faces +X at frame 0
# a library made without --keep_heading is turned back in place of a re-run (SMPL encoder only; plan section 17.8):
python gear_sonic/data_process/restore_a3_heading.py --library <lib> --output <lib>_heading
python gear_sonic/data_process/compare_smpl_vs_a3.py --stats --robot <lib>_heading   # heading offset to data/smpl_filtered (~0)
python gear_sonic/train_agent_trl.py --robot a3_ultra ...   # same flag for the other robots; reads that library (035 contract, DR over A3 and A3 Ultra)
python gear_sonic/train_agent_trl.py +exp=manager/universal_token/all_modes/sonic_a3_ultra_smpl ...   # + our SMPL encoder; needs data/smpl_filtered (plan section 17)
```

#### A3 Ultra with the downloaded weights (MuJoCo, same flow as G1)

```bash
python download_from_hf.py --robot a3_ultra                  # the same 035 weights; there is no A3U-specific checkpoint
gear_sonic_deploy/scripts/setup_a3_sim_env.sh                # one-time: ROS 2 python env for the simulator
python gear_sonic/scripts/run_sim_loop.py --robot a3_ultra   # terminal 1: MuJoCo with the A3 Ultra (T3D0) model
gear_sonic_deploy/deploy.sh --robot a3_ultra sim             # terminal 2: unchanged C++ deploy binary (keys S, M, R / T, 3)
```

See `docs/notes/a3/mujoco_pico_sim_teleop.md` for the full A3 / A3 Ultra MuJoCo + Pico tutorial (three terminals; `docs/notes/g1/mujoco_pico_sim_teleop.md` section 8 is the short version) and
`docs/workflow/a3_integration_plan.md` section 14 for what was and was not verified on A3 Ultra.

#### Pico teleop encoder: `--encoder a3-fast|smpl`

```bash
gear_sonic_deploy/deploy.sh --robot a3_ultra --input-type zmq_manager sim --encoder a3-fast   # default: downloaded 035 a3_fast policy
gear_sonic_deploy/deploy.sh --robot a3_ultra --input-type zmq_manager sim --encoder smpl --smpl-model <model_step_N_smpl.onnx>
python gear_sonic/scripts/pico_manager_thread_server.py --robot a3_ultra --manager --num_frames_to_send 10   # Pico side
```

`--encoder a3-fast` (Pico SMPL -> GMR retargeting -> A3 joint references -> `a3_ta_bridge` -> `/ta/whole_body_command`)
is fully usable with the downloaded weights. `--encoder smpl` (Pico SMPL stream + A3 wrist angles -> SMPL-encoder policy)
needs an SMPL-encoder ONNX (`obs_dict[1,1770] -> action[1,29]`) that **does not exist for A3 / A3 Ultra yet**: the 035
release only has the `g1` and `a3_fast` encoders. Without a model the flag exits with instructions;
`--smpl-model stub` runs a plumbing-test policy that stands and ignores the operator. The chosen source is locked in the
deploy binary (`--teleop-source`, keys `1/2/3` are rejected). Real robot: run `scripts/a3_operator_chain.sh --encoder ...`
on the operator PC and `deploy.sh ... real --no-chain` on the robot. Details, verification and caveats:
`docs/workflow/a3_integration_plan.md` section 15.

Verified with the official weights: ONNX matches the PT policy to better than 4e-6 per step,
and the PT and both ONNX models complete a 1652-step MuJoCo walk without falling. Always
validate in MuJoCo and use a safety gantry before touching a real robot.

### Data

| Source | What it is | Where |
|---|---|---|
| `a3_data/agibot_a3/*.csv` | 20 flat A3 sample motions shipped with the release | in git |
| Bones-SEED G1 CSV (`g1.tar.gz`) | Gated dataset, accept the license on Hugging Face first | `data/bones_seed/` (not in git) |
| `data_process/convert_a3_csv_to_motion_lib.py` | A3 flat CSV to motion-lib PKL | |
| `data_process/batch_retarget_bvh_soma_a3ultra.py` | SOMA BVH to A3 Ultra motion-lib | |
| `data_process/restore_a3_heading.py`, `compare_smpl_vs_a3.py`, `make_compare_videos.py` | turn a retarget library back to the capture heading; SMPL-vs-robot checks and videos | |

The internal motion dataset used for the 035 checkpoint is not released. The SOMA
retargeter itself lives in [NVIDIA/soma-retargeter](https://github.com/NVIDIA/soma-retargeter).

### Environment notes (C++ deploy and training)

- **ROS 2 Humble through conda (robostack).** Use `empy==3.3.4` (empy 4.x breaks `rosidl`),
  and install conda `libacl`. Export `CPATH=$CONDA_PREFIX/include` and
  `LIBRARY_PATH=$CONDA_PREFIX/lib` for the build, and add `$CONDA_PREFIX/lib` to
  `LD_LIBRARY_PATH` to run (otherwise `librclcpp.so` is not found).
- **Python for the build**: `build_a3_deploy_pkg.sh` uses `$GS_PYTHON3_EXECUTABLE`, otherwise
  `$CONDA_PREFIX/bin/python3`, so it does not pick up the system Python 3.12.
- **iceoryx RouDi**: the deploy package is built against iceoryx 2.95.4 and needs a RouDi of the
  same version. conda's `iox-roudi` (2.0.2) is incompatible and the deploy waits forever with
  "RouDi not found". Build the matching one with
  `cmake --build <build_dir> --target iox-roudi`.
- **TensorRT-free build**: A3 packages use `-DENABLE_TRT_INFERENCE=OFF -DENABLE_A3_AIMRT_BACKEND=ON`;
  the G1 binary needs `ENABLE_G1_BACKEND` (default on when TensorRT is enabled).
- **Paths with spaces**: `accelerate launch` splits Hydra overrides at spaces. Check the repo out
  to a path without spaces, or use a symlink for training.
- **Isaac Sim**: run in a clean environment (do not activate other conda envs first);
  `app.close()` can hang, so scripts exit with `os._exit(0)`.
- **transformers 5.x** has no `TrainingArguments.logging_dir`; the trainer handles this.

### Git policy for A3 files

Large assets are not committed: weights (`*.pt`, `*.onnx`, `*.rknn`), motion libraries, downloaded
datasets (`data/`), A3 Ultra meshes and USD. Small files needed to reproduce the pipeline are
whitelisted in `.gitignore`: A3 CMake sources, sample CSVs, `gear_sonic/data/a3_motor_model/`,
A3 MJCF/URDF and A3 STL meshes (Git LFS). Fetch weights with `download_from_hf.py`.

Full integration notes: [`docs/workflow/a3_integration_plan.md`](docs/workflow/a3_integration_plan.md)
and the [A3 training contract](docs/a3_035_training_contract.md).

## TODOs

- [x] Release pretrained SONIC policy checkpoints
- [x] Open source C++ inference stack
- [x] Setup documentation
- [x] Open source teleoperation stack and demonstration scripts
- [x] Release training scripts and recipes for motion imitation and fine-tuning
- [x] Open source large-scale data collection workflows and fine-tuning VLA scripts. 
- [x] Publish additional preprocessed large-scale human motion datasets



## What's Included

This release includes:

- **`gear_sonic_deploy`**: C++ inference stack for deploying SONIC policies on real hardware
- **`gear_sonic`**: Full SONIC training stack — PPO training, data processing pipeline, and configuration system for training on Bones-SEED and custom motion datasets
- **`motionbricks`**: Preview release of the MotionBricks real-time latent generative stack — interactive G1 demo, pretrained checkpoints, synthetic training code, and motion-representation docs

### Setup

> **Git LFS required.** This repo contains large binary assets (meshes, ONNX
> models). Without Git LFS, you will get small pointer files instead of actual
> data, causing silent failures. Install Git LFS first if you don't have it:
> `sudo apt install git-lfs && git lfs install`
>
> MotionBricks pretrained checkpoints are skipped by default to avoid an extra
> ~2.2 GiB download during normal monorepo setup. MotionBricks GIFs and meshes
> still download normally. Fetch the checkpoints explicitly if you plan to run
> the MotionBricks demo.

```bash
git clone https://github.com/NVlabs/GR00T-WholeBodyControl.git
cd GR00T-WholeBodyControl
git lfs pull

# Optional: fetch MotionBricks pretrained checkpoints.
git lfs pull --include="motionbricks/out/**" --exclude=""

# Verify your environment
python check_environment.py
```

### Which environment do I need?

| I want to... | Environment | How to install |
|---|---|---|
| **Train / finetune SONIC** | Isaac Lab's Python env | [Install Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html), then `pip install -e "gear_sonic/[training]"` |
| **Run MuJoCo simulation** | `.venv_sim` (auto-created) | `bash install_scripts/install_mujoco_sim.sh` |
| **VR teleoperation** | `.venv_teleop` (auto-created) | `bash install_scripts/install_pico.sh` |
| **Collect data** | `.venv_data_collection` (auto-created) | `bash install_scripts/install_data_collection.sh` |
| **Deploy on real robot** | C++ build | See [deployment docs](https://nvlabs.github.io/GR00T-WholeBodyControl/getting_started/installation_deploy.html) |

Each use case has its own lightweight environment. The install scripts use `uv`
and create isolated venvs automatically — you don't need to manage them manually.
Training is the only one that requires Isaac Lab (installed separately).

## Documentation

📚 **[Full Documentation](https://nvlabs.github.io/GR00T-WholeBodyControl/)**

### Getting Started
- [Installation Guide](https://nvlabs.github.io/GR00T-WholeBodyControl/getting_started/installation_deploy.html)
- [Quick Start](https://nvlabs.github.io/GR00T-WholeBodyControl/getting_started/quickstart.html)
- [VR Teleoperation Setup](https://nvlabs.github.io/GR00T-WholeBodyControl/getting_started/vr_teleop_setup.html)

### Tutorials
- [Keyboard Control](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/keyboard.html)
- [Gamepad Control](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/gamepad.html)
- [ZMQ Communication](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/zmq.html)
- [ZMQ Manager / PICO VR](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/vr_wholebody_teleop.html)

### Training
- [Installation (Training)](https://nvlabs.github.io/GR00T-WholeBodyControl/getting_started/installation_training.html)
- [Training Guide](https://nvlabs.github.io/GR00T-WholeBodyControl/user_guide/training.html)
- [Training Data](https://nvlabs.github.io/GR00T-WholeBodyControl/user_guide/training_data.html)

### Best Practices
- [Teleoperation](https://nvlabs.github.io/GR00T-WholeBodyControl/user_guide/teleoperation.html)






---

## Citation

If you use GEAR-SONIC in your research, please cite:

```bibtex
@article{luo2025sonic,
    title={SONIC: Supersizing Motion Tracking for Natural Humanoid Whole-Body Control},
    author={Luo, Zhengyi and Yuan, Ye and Wang, Tingwu and Li, Chenran and Chen, Sirui and Casta\~neda, Fernando and Cao, Zi-Ang and Li, Jiefeng and Minor, David and Ben, Qingwei and Da, Xingye and Ding, Runyu and Hogg, Cyrus and Song, Lina and Lim, Edy and Jeong, Eugene and He, Tairan and Xue, Haoru and Xiao, Wenli and Wang, Zi and Yuen, Simon and Kautz, Jan and Chang, Yan and Iqbal, Umar and Fan, Linxi and Zhu, Yuke},
    journal={arXiv preprint arXiv:2511.07820},
    year={2025}
}
```

---

## License

This project uses dual licensing:

- **Source Code**: Licensed under Apache License 2.0 - applies to all code, scripts, and software components in this repository
- **Model Weights**: Licensed under NVIDIA Open Model License - applies to all trained model checkpoints and weights

See [LICENSE](LICENSE) for the complete dual-license text.

Please review both licenses before using this project. The NVIDIA Open Model License permits commercial use with attribution and requires compliance with NVIDIA's Trustworthy AI terms.

All required legal documents, including the Apache 2.0 license, 3rd-party attributions, and DCO language, are consolidated in the /legal folder of this repository.

---

## Support

For questions and issues, please contact the GEAR WBC team at [gear-wbc@nvidia.com](mailto:gear-wbc@nvidia.com) to provide feedback! 

## MotionBricks

<p style="font-size: 1.2em;">
  <a href="https://nvlabs.github.io/motionbricks/"><strong>Project page</strong></a> |
  <a href="motionbricks/README.md"><strong>Subproject README</strong></a>
</p>

<div align="center">
  <img src="motionbricks/assets/gifs/teaser_animation.gif" width="400">
  <img src="motionbricks/assets/gifs/teaser_robotics.gif" width="400">
</div>

MotionBricks is a real-time generative framework that transforms interactive motion control for animation and robotics. It combines a large-scale latent backbone with intuitive "smart primitives" to deliver high-quality, zero-shot motion synthesis at 15,000 FPS — complementing the tracking-based GEAR-SONIC controllers in this repo.

This preview release ships an interactive G1 demo (keyboard-driven, MuJoCo viewer), pretrained checkpoints (VQVAE · pose · root), a synthetic training pipeline, and motion-representation docs. Its pretrained checkpoints are opt-in for monorepo clones; run `git lfs pull --include="motionbricks/out/**" --exclude=""` from the repo root before using the demo. A full release — fully embedded in the GEAR-SONIC pipeline — is targeted for approximately one month out. See [`motionbricks/README.md`](motionbricks/README.md) for setup, demo, and training instructions.

## Decoupled WBC

For the Decoupled WBC used in GR00T N1.5 and N1.6 models, please refer to the [Decoupled WBC documentation](docs/source/references/decoupled_wbc.md).


## Acknowledgments
We would like to acknowledge the following projects from which parts of the code in this repo are derived from:
- [Beyond Mimic](https://github.com/HybridRobotics/whole_body_tracking)
- [Isaac Lab](https://github.com/isaac-sim/IsaacLab)

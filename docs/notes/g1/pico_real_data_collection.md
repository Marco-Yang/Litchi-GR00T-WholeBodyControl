# G1 真机遥操数采（SONIC v1.1 + 双 RealSense RGB → GR00T VLA）

本文是本机实操 runbook：**G1 + SONIC v1.1** 真机 PICO 全身遥操数采，输出可直接给 **Isaac-GR00T N1.7 / `UNITREE_G1_SONIC`** 微调的 LeRobot 数据集。

策略固定用 `gear_sonic_deploy/policy/sonic_v1_1/`（`--cp policy/sonic_v1_1/model` + 对应 `observation_config.yaml`），不要用默认 `policy/release`。

遥操按键见 [pico_real_teleop.md](pico_real_teleop.md)。上游：[`data_collection.md`](../../source/tutorials/data_collection.md)、[`vla_workflow.md`](../../source/tutorials/vla_workflow.md)。

---

## 0. 与 GR00T G1 VLA 的对应关系

下游训练仓库：[NVIDIA/Isaac-GR00T](https://github.com/NVIDIA/Isaac-GR00T)，embodiment：`UNITREE_G1_SONIC`。

| GR00T 模态 | 本数采落盘字段 | 说明 |
|---|---|---|
| video `ego_view` | `videos/.../observation.images.ego_view/*.mp4` | **必须**；颈部 D435I |
| video `head` | `.../observation.images.head/*.mp4` | 可选；默认 GR00T 配置**不读** |
| video `left/right_wrist` | `.../observation.images.left_wrist` / `right_wrist` | 可选；腕部 Abham UVC（需 `--record-wrist-cameras` + Orin `:5558`） |
| state 腿/腰/臂/手 | `observation.state` 切片（含 Dex3 左右各 7D） | Dex3 **实测状态** |
| state `projected_gravity` | `observation.projected_gravity` | |
| action `motion_token` | `action.motion_token` (64D) | SONIC latent；录制时由 deploy 的 `token_state` 写入 |
| action `left/right_hand_joints` | `teleop.left_hand_joints` / `teleop.right_hand_joints`（各 **7D = Dex3**） | **夹爪动作标签**；仅 Pico **Trigger**（默认闭合，扣下张开）；exporter 可回退到 deploy 手指令/状态 |
| language | `meta/tasks.jsonl` + `task_index` | `--task-prompt` |

**Dex3 会录两路：**

1. **状态**：`observation.state` 的 `left_hand` / `right_hand`（7+7）← 机器人反馈  
2. **动作标签**：`teleop.*_hand_joints`（7+7）← Pico **仅 Trigger** 开合指令（GR00T 训手用这个）；exporter 可回退用 deploy 手指令/实测

数采脚本**没有漏字段**。**左 Grip+A / 左 Grip+B** 只负责开始/丢弃录制，**不会**闭合 Dex3；夹爪默认闭合，扣 **Trigger** 张开。

---

## 1. 本机已核实（2026-10-30）

相机在 **机器人 Orin `192.168.123.164`**（`unitree` / 密码 `123`），不是工作站 USB。

| 角色 | 标识 | 位置 / 类型 | 备注 |
|---|---|---|---|
| **`ego_view`（主视角，VLA 必用）** | SN `242622070951` | 颈部 D435I | ZMQ `:5555` |
| **`head`（第二路，可选）** | SN `254522070308` | 机载 D435I | 同 `:5555` |
| **`left_wrist` / `right_wrist`（可选）** | Abham `HHW002` / `HHW001` | 腕部 UVC（`1c45:6200`） | 与强脑 USB-485 同插入端；ZMQ `:5558` |

| 压测 | 结果 |
|---|---|
| 双 RealSense **仅 RGB** 640×480@30 | 约 30 fps，可用 |
| 双 RealSense RGB+Depth@30 | **不可用** |
| 双腕部 Abham MJPG 640×480 | 约 30 fps（需 Hub/`ttyACM*` 在线） |
| 数采策略 | RealSense **只开 RGB**；腕部按需 |

---

## 2. 架构

```text
  工作站                                              G1 Orin 192.168.123.164
┌────────────────┐ ┌────────────────┐               ┌──────────────────────────┐
│ deploy v1.1    │ │ pico_manager   │               │ D435I ego+head → :5555   │
│ ZMQ :5557      │ │ ZMQ :5556      │               │ Abham wrists  → :5558    │
│ state+token    │ │ SMPL+Dex3 cmd  │               │ （腕部可选）               │
└───────┬────────┘ └───────┬────────┘               └────────────┬─────────────┘
        └─────────┬────────┘─────────────────────────────────────┘
                  ▼
         run_data_exporter.py  →  outputs/<dataset>/  (LeRobot + modality.json)
```

| 源 | 端口 | 写入数据集的关键内容 |
|---|---|---|
| C++ 部署 | `5557` | `observation.state`（含 Dex3 状态）、`action.motion_token`、重力/姿态 |
| Pico | `5556` | SMPL、`teleop.*_hand_joints`（Dex3 目标）、录制按键 |
| Camera 头胸（Orin） | `5555` | `ego_view` + `head` JPEG（双 RealSense） |
| Camera 腕部（Orin，可选） | `5558` | `left_wrist` + `right_wrist` JPEG（Abham UVC） |

> **端口不要混：** Pico pose = `5556`；头胸相机 = `5555`；腕部相机 = `5558`。

---

## 3. 一次性准备

### 3.0 上真机前网络自检（必做）

两张网、两个地址，不要混：

| 地址 | 网卡 / 位置 | 用途 |
|---|---|---|
| **192.168.50.112**（以现场 `wlp8s0` 为准） | 工作站 Wi‑Fi | 头显 XRoboToolkit **PC Service** 填这个 |
| **192.168.123.100** | 工作站有线 `enp11s0`（NM 连接名 `G1-Control`） | DDS / `deploy.sh ... real` 本机侧 |
| **192.168.123.164** | G1 Orin | SSH / 相机 ZMQ `:5555`（头胸）+ 可选腕部 `:5558` |
| **192.168.123.161** | G1 机体系 | Unitree 本体控制 |

```bash
# 1) 头显填的 Wi‑Fi IP（当前实测 wlp8s0）
ip -4 -br addr show wlp8s0

# 2) 本机必须有 192.168.123.x（网线接 G1 控制网）；没有则 real / 相机都会失败
ip -4 -br addr | grep 192.168.123 || echo 'MISSING G1 control iface — 插网线后再继续'

# 3) Orin 可达
ping -c 2 -W 2 192.168.123.164
```

`A+X`（POSE）依赖 SMPL FK 资产（本机已生成；若换机请确认存在）：

```text
gear_sonic/data/human/human_joints_info.pkl
```

### 3.1 工作站

```bash
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl
# 若尚未装数采环境：
# bash install_scripts/install_data_collection.sh   # → .venv_data_collection
```

清仿真 / 部署残留（**sim 与 real 不能同时跑**）：

```bash
pkill -f run_sim_loop.py || true
pkill -f g1_deploy_onnx_ref || true
pkill -f pico_manager_thread_server.py || true
ss -ltn | grep -E '5556|5557' || echo ports_free
```

PC Service（头显填 **当前** `wlp8s0` IP，现为 `192.168.50.112`）：

```bash
pgrep -af RoboticsServiceProcess || bash /opt/apps/roboticsservice/runService.sh
```

### 3.2 机器人：同步并启动相机 server

Orin 为 Python 3.8，用 **一条** 统一启动脚本。**先确认 3.0 里 Orin ping 通。**

工作站同步脚本：

```bash
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl
rsync -av --relative \
  gear_sonic/camera \
  gear_sonic/__init__.py \
  gear_sonic_deploy/scripts/g1_camera_rgb_server.sh \
  gear_sonic_deploy/scripts/g1_dual_realsense_rgb_server.sh \
  gear_sonic_deploy/scripts/g1_dual_realsense_rgb_zmq_server.py \
  gear_sonic_deploy/scripts/g1_wrist_abham_rgb_server.sh \
  gear_sonic_deploy/scripts/g1_wrist_abham_rgb_zmq_server.py \
  unitree@192.168.123.164:/home/unitree/GR00T-WholeBodyControl/
```

Orin 上启动（SSH `unitree@192.168.123.164`，密码 `123`）— **只这一条**：

```bash
ssh unitree@192.168.123.164
cd /home/unitree/GR00T-WholeBodyControl

# 默认：只启 ego_view + head → :5555
bash gear_sonic_deploy/scripts/g1_camera_rgb_server.sh

# 四路：外加参数 wrist（再启 left/right_wrist → :5558）
# 需强脑插入端在线（ttyACM0/1 + Abham HHW001/HHW002）
bash gear_sonic_deploy/scripts/g1_camera_rgb_server.sh wrist
```

### 3.3 工作站：相机监控（可选，与数采无关）

一条命令：订 `:5555` + `:5558`，**有几路显示几路**（最多 2×2）：

```bash
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl
source .venv_data_collection/bin/activate
# 若 imshow 报 GUI not implemented：先卸 opencv-python-headless
python gear_sonic/scripts/run_camera_viewer.py \
  --camera-host 192.168.123.164
```

---

## 4. 真机数采启动顺序

安全：吊带；物理急停有人盯；**已关掉** `run_sim_loop.py`；PC Service **WORKING + Send**。

### 4.1 机器人 — Camera

按 **§3.2** 一条命令：

```bash
# Orin
bash gear_sonic_deploy/scripts/g1_camera_rgb_server.sh          # ego+head
# 或
bash gear_sonic_deploy/scripts/g1_camera_rgb_server.sh wrist    # 四路
```

工作站确认画面（§3.3，有什么显示什么）：

```bash
source .venv_data_collection/bin/activate
python gear_sonic/scripts/run_camera_viewer.py --camera-host 192.168.123.164
```

### 4.2 工作站终端 1 — 部署（SONIC v1.1）

```bash
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl/gear_sonic_deploy
source scripts/setup_env.sh
./deploy.sh \
  --robot g1 \
  --cp policy/sonic_v1_1/model \
  --obs-config policy/sonic_v1_1/observation_config.yaml \
  --input-type zmq_manager \
  --motor-kp-scale 4,10=1.5 \
  --motor-kd-scale 4,10=1.5 \
  real
# 提示 Proceed 时输入 y；等到 Init Done
# 确认左右 Dex3 都张开，且 DDS 有 rt/dex3/*/state
```

`real` 自动选本机 `192.168.123.x` 网卡。若脚本报找不到该网段，先查网线；仍不行把最后的 `real` 换成机器人实际 IP（例如 `192.168.123.164` 所在侧的本机地址对应接口，或直接传接口名）。

### 4.3 工作站终端 2 — Pico

数采可关可视化减负；首次上真机建议先开可视化确认脚部跟踪：

```bash
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl
source .venv_teleop/bin/activate
# 首次：加 --vis_vr3pt --vis_smpl；确认跟踪 OK 后再去掉减负
python gear_sonic/scripts/pico_manager_thread_server.py --robot g1 --manager
```

头显 PC Service 填 **192.168.50.112**（变了就用 `ip -4 -br addr show wlp8s0` 重查）。缺 `human_joints_info.pkl` 时按 **A+X** 会直接崩。

### 4.4 工作站终端 3 — Data exporter

```bash
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl
source .venv_data_collection/bin/activate
python gear_sonic/scripts/run_data_exporter.py \
  --task-prompt "把桌上的杯子拿起来放到盘子里" \
  --camera-host 192.168.123.164 \
  --camera-port 5555 \
  --record-head-camera \
  --root-output-dir outputs
```

### 4.5 操作（含 Dex3）

1. 标定姿态 → **A+B+X+Y** 启动（PLANNER）→ 身体对齐后再 **A+X** 进 **POSE**  
2. **左 Grip + A**：开始/结束 episode  
3. **左 Grip + B**：丢弃当前段  
4. **开合 Dex3**：**只扣 Trigger**：默认闭合，扣下（>0.5）张开；写入 `teleop.*_hand_joints`。  
   - **左 Grip + A / 左 Grip + B** 只做开始录制 / 丢弃，**不**驱动夹爪。  
   - exporter 若收到的 Pico 手指令接近全 0，会回退写入 deploy 的 `last_*_hand_action`，再不行用实测 `*_hand_q`。  
5. 抓取类任务请在 episode 内明确做「伸手 → 松开 Trigger 抓取 → 移动 → 扣 Trigger 放开」。  
6. 急停：物理急停；部署终端按 `O`（焦点必须在该终端）

官方建议同一任务至少 **50–100** 条成功 episode 再微调。

---

## 5. 落盘与训前处理

```text
outputs/<dataset>/
├── data/chunk-000/episode_*.parquet
├── videos/chunk-000/observation.images.ego_view/episode_*.mp4
├── videos/chunk-000/observation.images.head/episode_*.mp4   # 可选
└── meta/
    ├── info.json
    ├── modality.json      # GR00T 需要
    ├── episodes.jsonl
    ├── tasks.jsonl
    └── episodes_stats.jsonl
```

### 5.1 校验是否够给 GR00T 训

```bash
source .venv_data_collection/bin/activate
python gear_sonic/scripts/validate_g1_sonic_vla_dataset.py \
  --dataset-path outputs/<dataset>
```

期望：`motion_token` 大量非零；若要训夹爪，`teleop.*_hand_joints` 也要有明显非零比例。

### 5.2 清洗（微调前必做）

去掉 discarded episode 与 stale SMPL 帧。`--output-path` **只填一个地址**（任务根目录，如 `datasets/walking`）；其下按与 `outputs/` 相同的 session 文件夹名分别落盘，不会合成一个大包：

```bash
# 单 / 多 session 都行：只改 --output-path 这一处任务根目录
python gear_sonic/scripts/process_dataset.py \
  --dataset-path \
    outputs/2026-11-01-15-43-46 \
    outputs/2026-11-01-16-12-03 \
  --output-path datasets/walking
# -> datasets/walking/2026-11-01-15-43-46
# -> datasets/walking/2026-11-01-16-12-03
```

落盘结构（与 `outputs/` 对齐）：

```text
datasets/walking/                 # <-- 外参只给这个地址
├── 2026-11-01-15-43-46/          # 独立 LeRobot 数据集
│   ├── data/
│   ├── videos/
│   └── meta/
├── 2026-11-01-16-12-03/
└── ...
```

若仍需合并成单一数据集，另加 `--merge` 并把 `--output-path` 指到合并后的目录。

### 5.3 微调（在 Isaac-GR00T 仓库）

```bash
uv run python gr00t/experiment/launch_finetune.py \
  --base-model-path nvidia/GR00T-N1.7-3B \
  --dataset-path /path/to/datasets/walking/<session> \
  --embodiment-tag UNITREE_G1_SONIC \
  --modality-config-path gr00t/configs/data/embodiment_configs.py \
  ...
```

---

## 6. 排错

| 现象 | 处理 |
|---|---|
| `teleop.*_hand_joints` 全 0 | 松开 **Trigger** 闭合 / 扣下张开（Grip 不驱动手）；确认 Dex3 DDS；重启 pico_manager；跑校验脚本 |
| `observation.state` 手有值但 teleop 手为 0 | 更新后的 exporter 会回退填 hand action/state；仍全 0 则手根本没被驱动 |
| `motion_token` 全 0 | deploy 未跑 v1.1 / 策略未启动；需有 `token_state` |
| 红物体变蓝 | 工作站用已修的 `ImageUtils.decode_image`；重录 |
| `cv2.imshow` GUI not implemented | 卸掉 `opencv-python-headless`；若重装 opencv 把 numpy 升到 2.x，再执行 `uv pip install 'numpy>=1.26,<2' --python .venv_data_collection/bin/python`（pinocchio 需 numpy 1.x） |
| pinocchio / NumPy 2.x ImportError 或 segfault | 同上，把 `.venv_data_collection` 的 numpy 钉回 `<2` |
| `5555` 连不上 | Orin 上起 `g1_dual_realsense_rgb_server.sh`；确认本机有 `192.168.123.x` 且 `ping 192.168.123.164` |
| 监控无腕部画面 | Orin 用 `g1_camera_rgb_server.sh wrist`；查 `ls /dev/v4l/by-id/*Abham* /dev/ttyACM*`；强脑插入端易掉线需重插 |
| 腕部占了 `5556` | 与 Pico 冲突；腕部必须用 `5558` |
| `Could not find 192.168.123.x` / real 起不来 | 工作站未接 G1 控制网；插网线后再 `ip -4 -br addr` |
| 头显 `connect_error` | PC Service 未起，或 Wi‑Fi IP 变了（现 `wlp8s0`=`192.168.50.112`） |
| 按 A+X 进程中止 | 缺 `gear_sonic/data/human/human_joints_info.pkl` |
| 左手夹爪不动 | 查 `rt/dex3/left/state`；必要时整机重启 |

---

## 7. 相关文档

- [pico_real_teleop.md](pico_real_teleop.md)  
- [mujoco_pico_sim_teleop.md](mujoco_pico_sim_teleop.md)  
- [../../source/tutorials/data_collection.md](../../source/tutorials/data_collection.md)  
- [../../source/tutorials/vla_workflow.md](../../source/tutorials/vla_workflow.md)  

---

## 8. 数据回放筛选（在数据电脑上）

在装有 Isaac Lab 的**数据电脑**上做：关节/SMPL 跳变预筛 + Isaac mode-2 回放。仓库根目录：`Litchi-GR00T-WholeBodyControl`。

```bash
conda activate sikan
cd /path/to/Litchi-GR00T-WholeBodyControl
export OMNI_KIT_ACCEPT_EULA=YES
export CUDA_VISIBLE_DEVICES=0
```

清洗后每个 session 是独立数据集，回放时把 `--dataset` 指到具体 session 目录：

### 8.1 GUI 抽查（推荐先看一小段）

```bash
python -m datasets.data_check pipeline \
  --dataset datasets/walking/2026-11-01-15-43-46 \
  --work-dir datasets/data_check/runs/2026-11-01-15-43-46_0_15 \
  --start 0 --end 15 \
  --num-envs 0 \
  --env-spacing 6 \
  --python "$(which python)"
```

### 8.2 全量 headless

```bash
python -m datasets.data_check pipeline \
  --dataset datasets/walking/2026-11-01-15-43-46 \
  --work-dir datasets/data_check/runs/2026-11-01-15-43-46_all \
  --num-envs 21 \
  --env-spacing 6 \
  --headless \
  --exit-after-metrics \
  --python "$(which python)"
```

### 8.3 看结果

- 跳变不通过：`<work-dir>/joint_stability.json`（字段 `unstable_episode_indices`）
- 汇总：`<work-dir>/failed_validation.txt`（若已生成）
- Isaac 指标：`<work-dir>/isaac_eval/metrics/metrics_eval.json`

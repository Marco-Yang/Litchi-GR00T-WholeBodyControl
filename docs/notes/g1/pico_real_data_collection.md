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
| state 腿/腰/臂/手 | `observation.state` 切片（含 Dex3 左右各 7D） | Dex3 **实测状态** |
| state `projected_gravity` | `observation.projected_gravity` | |
| action `motion_token` | `action.motion_token` (64D) | SONIC latent；录制时由 deploy 的 `token_state` 写入 |
| action `left/right_hand_joints` | `teleop.left_hand_joints` / `teleop.right_hand_joints`（各 **7D = Dex3**） | **夹爪动作标签**；Pico **Trigger 或 Grip**；exporter 可回退到 deploy 手指令/状态 |
| language | `meta/tasks.jsonl` + `task_index` | `--task-prompt` |

**Dex3 会录两路：**

1. **状态**：`observation.state` 的 `left_hand` / `right_hand`（7+7）← 机器人反馈  
2. **动作标签**：`teleop.*_hand_joints`（7+7）← Pico **Trigger/Grip** 开合指令（GR00T 训手用这个）；exporter 可回退用 deploy 手指令/实测

数采脚本**没有漏字段**。旧数据若 teleop 手全 0，多半是当时只靠 Trigger、且没扣；新逻辑下按住 **左 Grip+A** 录制时左手也会闭合并写入标签。

---

## 1. 本机已核实（2026-10-30）

相机在 **机器人 Orin `192.168.123.164`**（`unitree` / 密码 `123`），不是工作站 USB。

| 角色 | 序列号 | 位置 | USB |
|---|---|---|---|
| **`ego_view`（主视角，VLA 必用）** | `242622070951` | **新加颈部** D435I | USB2 480M（`1-2.2`） |
| **`head`（第二路，可选）** | `254522070308` | **原装机载** D435I | USB2 480M（`1-2.3`） |

| 压测 | 结果 |
|---|---|
| 双路 **仅 RGB** 640×480@30 | 约 30 fps，可用 |
| 双路 RGB+Depth@30 | **不可用** |
| 数采策略 | **只开 RGB** |

---

## 2. 架构

```text
  工作站                                              G1 Orin 192.168.123.164
┌────────────────┐ ┌────────────────┐               ┌─────────────────────────┐
│ deploy v1.1    │ │ pico_manager   │               │ D435I neck = ego_view   │
│ ZMQ :5557      │ │ ZMQ :5556      │               │ D435I stock = head      │
│ state+token    │ │ SMPL+Dex3 cmd  │               │ RGB-only @30 → :5555    │
└───────┬────────┘ └───────┬────────┘               └───────────┬─────────────┘
        └─────────┬────────┘────────────────────────────────────┘
                  ▼
         run_data_exporter.py  →  outputs/<dataset>/  (LeRobot + modality.json)
```

| 源 | 端口 | 写入数据集的关键内容 |
|---|---|---|
| C++ 部署 | `5557` | `observation.state`（含 Dex3 状态）、`action.motion_token`、重力/姿态 |
| Pico | `5556` | SMPL、`teleop.*_hand_joints`（Dex3 目标）、录制按键 |
| Camera（Orin） | `5555` | `ego_view` + `head` JPEG |

---

## 3. 一次性准备

### 3.1 工作站

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
bash install_scripts/install_data_collection.sh   # → .venv_data_collection
```

清端口残留：

```bash
pkill -f g1_deploy_onnx_ref || true
ss -ltn | grep -E '5556|5557' || echo ports_free
```

### 3.2 机器人：同步并起双 RGB server

Orin 为 Python 3.8，用 standalone 发布器（脚本已自动选择）：

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
rsync -av --relative \
  gear_sonic/camera \
  gear_sonic/__init__.py \
  gear_sonic_deploy/scripts/g1_dual_realsense_rgb_server.sh \
  gear_sonic_deploy/scripts/g1_dual_realsense_rgb_zmq_server.py \
  unitree@192.168.123.164:/home/unitree/GR00T-WholeBodyControl/
```

```bash
ssh unitree@192.168.123.164
cd /home/unitree/GR00T-WholeBodyControl
bash gear_sonic_deploy/scripts/g1_dual_realsense_rgb_server.sh
# 应 LISTEN :5555
```

可选预览（与数采无关）：

```bash
source .venv_data_collection/bin/activate
# 若 imshow 报 GUI not implemented：先卸 opencv-python-headless
python gear_sonic/scripts/run_camera_viewer.py \
  --camera-host 192.168.123.164 --camera-port 5555
```

---

## 4. 真机数采启动顺序

安全：吊带；物理急停有人盯；关掉 `run_sim_loop.py`。

### 4.1 机器人 — Camera

保持 `g1_dual_realsense_rgb_server.sh` 运行。

### 4.2 工作站终端 1 — 部署（SONIC v1.1）

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
source scripts/setup_env.sh
./deploy.sh \
  --robot g1 \
  --cp policy/sonic_v1_1/model \
  --obs-config policy/sonic_v1_1/observation_config.yaml \
  --input-type zmq_manager \
  real
# Init Done；确认左右 Dex3 都张开，且 DDS 有 rt/dex3/*/state
```

### 4.3 工作站终端 2 — Pico

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --robot g1 --manager
```

PC Service WORKING + Send。Wi‑Fi IP 以 `ip -4 -br addr show wlp8s0` 为准。

### 4.4 工作站终端 3 — Data exporter

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
source .venv_data_collection/bin/activate
python gear_sonic/scripts/run_data_exporter.py \
  --task-prompt "把桌上的杯子拿起来放到盘子里" \
  --camera-host 192.168.123.164 \
  --camera-port 5555 \
  --record-head-camera \
  --root-output-dir outputs
```

### 4.5 操作（含 Dex3）

1. 标定 → **A+B+X+Y** 启动 → **A+X** 进 **POSE**  
2. **左 Grip + A**：开始/结束 episode  
3. **左 Grip + B**：丢弃当前段  
4. **开合 Dex3**：扣 **Trigger 或 Grip**（任一 >0.5 即闭合该侧手），都会写入 `teleop.*_hand_joints`。  
   - **左 Grip + A / 左 Grip + B** 仍是开始录制 / 丢弃；按住左 Grip 时左手也会闭合，因此录制组合键本身就会带上左手 Dex3 标签。  
   - exporter 若收到的 Pico 手指令接近全 0，会回退写入 deploy 的 `last_*_hand_action`，再不行用实测 `*_hand_q`。  
5. 抓取类任务请在 episode 内明确做「伸手 → Grip/Trigger 闭合 → 移动 → 松开」。  
6. 急停：物理急停；部署终端按 `O`

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

```bash
python gear_sonic/scripts/process_dataset.py \
  --dataset-path outputs/<dataset> \
  --output-path outputs/<dataset>_cleaned
```

去掉 discarded episode 与 stale SMPL 帧。多 session 可同一命令 merge。

### 5.3 微调（在 Isaac-GR00T 仓库）

```bash
uv run python gr00t/experiment/launch_finetune.py \
  --base-model-path nvidia/GR00T-N1.7-3B \
  --dataset-path /path/to/<dataset>_cleaned \
  --embodiment-tag UNITREE_G1_SONIC \
  --modality-config-path gr00t/configs/data/embodiment_configs.py \
  ...
```

---

## 6. 排错

| 现象 | 处理 |
|---|---|
| `teleop.*_hand_joints` 全 0 | 扣 **Trigger 或 Grip**；确认 Dex3 DDS；重启 pico_manager（需含 Grip→手指改动）；跑校验脚本 |
| `observation.state` 手有值但 teleop 手为 0 | 更新后的 exporter 会回退填 hand action/state；仍全 0 则手根本没被驱动 |
| `motion_token` 全 0 | deploy 未跑 v1.1 / 策略未启动；需有 `token_state` |
| 红物体变蓝 | 工作站用已修的 `ImageUtils.decode_image`；重录 |
| `cv2.imshow` GUI not implemented | 卸掉 `opencv-python-headless`；若重装 opencv 把 numpy 升到 2.x，再执行 `uv pip install 'numpy>=1.26,<2' --python .venv_data_collection/bin/python`（pinocchio 需 numpy 1.x） |
| pinocchio / NumPy 2.x ImportError 或 segfault | 同上，把 `.venv_data_collection` 的 numpy 钉回 `<2` |
| `5555` 连不上 | Orin 上起 `g1_dual_realsense_rgb_server.sh` |
| 左手夹爪不动 | 查 `rt/dex3/left/state`；必要时整机重启 |

---

## 7. 相关文档

- [pico_real_teleop.md](pico_real_teleop.md)  
- [mujoco_pico_sim_teleop.md](mujoco_pico_sim_teleop.md)  
- [../../source/tutorials/data_collection.md](../../source/tutorials/data_collection.md)  
- [../../source/tutorials/vla_workflow.md](../../source/tutorials/vla_workflow.md)  

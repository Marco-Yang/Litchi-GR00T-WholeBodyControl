# G1 真机遥操数采（SONIC v1.1 + 双 RealSense RGB）

本文是本机实操 runbook：**G1 + SONIC v1.1** 真机 PICO 全身遥操，并把 **机器人状态 / SMPL / 双路头部 RGB** 记到工作站，输出 LeRobot 数据集。

策略固定用 `gear_sonic_deploy/policy/sonic_v1_1/`（`--cp policy/sonic_v1_1/model` + 对应 `observation_config.yaml`），不要用默认 `policy/release`。

遥操按键见 [pico_real_teleop.md](pico_real_teleop.md)。上游英文教程：[data_collection.md](../../source/tutorials/data_collection.md)。

---

## 0. 本机已核实（2026-10-30）

相机都在 **机器人 Orin `192.168.123.164`** 上（`unitree` / 密码 `123`），不是工作站 USB。

| 角色 | 序列号 | 位置 | USB |
|---|---|---|---|
| **`ego_view`（主视角）** | `242622070951` | **新加颈部** D435I，前视工作区 | USB2 480M（`1-2.2`） |
| **`head`（第二路）** | `254522070308` | **原装机载** D435I，偏下视 | USB2 480M（`1-2.3`） |

| 压测 | 结果 |
|---|---|
| 双路 **仅 RGB** 640×480@30 | 约 30.2 / 30.0 fps，短测可用 |
| 双路 RGB+Depth@30 | **不可用**（严重掉帧） |
| 数采策略 | **只开 RGB**（`enable_depth=False`） |

测试帧：`_assets/g1_neck_cam_242622070951.png`（新颈部）、`_assets/g1_head_cam_test.png`（原装）。

---

## 1. 架构

```text
  工作站                                              G1 Orin 192.168.123.164
┌────────────────┐ ┌────────────────┐               ┌─────────────────────────┐
│ deploy v1.1    │ │ pico_manager   │               │ D435I neck 2426… = ego  │
│ ZMQ :5557      │ │ ZMQ :5556      │               │ D435I stock 2545… = head│
└───────┬────────┘ └───────┬────────┘               │ RGB-only @30 → :5555    │
        └─────────┬────────┘────────────────────────┴───────────┬─────────────┘
                  ▼                                             │
         run_data_exporter.py  ←── camera-host 192.168.123.164 ─┘
                  │
                  ▼
         outputs/<dataset>/  (parquet + ego_view.mp4 + head.mp4)
```

| 源 | 端口 | 内容 |
|---|---|---|
| C++ 部署 | `5557` | 关节 / IMU / robot_config |
| Pico | `5556` | SMPL pose + command |
| Camera server（机器人） | `5555` | `ego_view` + `head` JPEG |

---

## 2. 一次性准备

### 2.1 工作站

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
bash install_scripts/install_data_collection.sh   # → .venv_data_collection
```

清端口残留：

```bash
pkill -f g1_deploy_onnx_ref || true
ss -ltn | grep -E '5556|5557' || echo ports_free
```

### 2.2 机器人：同步仓库并起双 RGB server

Orin 系统 Python 是 **3.8**，不能直接跑 `gear_sonic.camera.composed_camera`（typing 语法不兼容）。仓库脚本会优先走 **standalone** 发布器：

- `gear_sonic_deploy/scripts/g1_dual_realsense_rgb_zmq_server.py`（RGB only，无 depth）
- wrapper：`g1_dual_realsense_rgb_server.sh`（自动 `LD_PRELOAD=libgomp.so.1`，避免 OpenCV `libgomp` 冲突）

工作站同步到 Orin（必须在**仓库根目录**执行，不要在 `gear_sonic_deploy/` 里跑）：

```bash
# 在工作站执行（ssh unitree@192.168.123.164，密码 123）
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
rsync -av --relative \
  gear_sonic/camera \
  gear_sonic/__init__.py \
  gear_sonic_deploy/scripts/g1_dual_realsense_rgb_server.sh \
  gear_sonic_deploy/scripts/g1_dual_realsense_rgb_zmq_server.py \
  unitree@192.168.123.164:/home/unitree/GR00T-WholeBodyControl/
```

机器人上确认两台相机：

```bash
ssh unitree@192.168.123.164
rs-enumerate-devices -s
# 应同时看到 242622070951 与 254522070308
```

启动（机器人终端保持运行；已核实 `0.0.0.0:5555` LISTEN）：

```bash
cd /home/unitree/GR00T-WholeBodyControl
bash gear_sonic_deploy/scripts/g1_dual_realsense_rgb_server.sh
# 日志应类似：
# [g1-cam] ego_view SN=242622070951  head SN=254522070308  port=5555  python=python3
# ... g1_dual_realsense_rgb_zmq_server.py ...
```

工作站预览：

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
source .venv_data_collection/bin/activate
# 若 imshow 报 “The function is not implemented”（装了 opencv-python-headless）：
#   uv pip uninstall opencv-python-headless --python .venv_data_collection/bin/python
python gear_sonic/scripts/run_camera_viewer.py \
  --camera-host 192.168.123.164 --camera-port 5555
```

应看到两路：`ego_view`（颈部前视）+ `head`（原装下视）。预览只是可选检查，数采不依赖它。---

## 3. 真机数采启动顺序

安全：吊带；物理急停有人盯；关掉 `run_sim_loop.py`。

### 3.1 机器人 — Camera（已在第 2.2 节）

保持 `g1_dual_realsense_rgb_server.sh` 运行，`5555` 在听。

### 3.2 工作站终端 1 — 部署（SONIC v1.1）

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
source scripts/setup_env.sh
./deploy.sh \
  --robot g1 \
  --cp policy/sonic_v1_1/model \
  --obs-config policy/sonic_v1_1/observation_config.yaml \
  --input-type zmq_manager \
  real
# Init Done；确认左右 Dex3 都张开
```

启动前确认文件存在：`policy/sonic_v1_1/model_encoder.onnx`、`model_decoder.onnx`、`observation_config.yaml`。

### 3.3 工作站终端 2 — Pico

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --robot g1 --manager
```

PC Service 需已开；头显 WORKING + Send。当前 Wi‑Fi 地址以 `ip -4 -br addr show wlp8s0` 为准（现为 `192.168.50.61`）。

### 3.4 工作站终端 3 — Data exporter（双 RGB）

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

或 tmux 一键（相机服务仍须在机器人上先开；同样指定 v1.1）：

```bash
python gear_sonic/scripts/launch_data_collection.py \
  --camera-host 192.168.123.164 \
  --record-head-camera \
  --deploy-checkpoint policy/sonic_v1_1/model \
  --deploy-obs-config policy/sonic_v1_1/observation_config.yaml \
  --task-prompt "把桌上的杯子拿起来放到盘子里"
```

### 3.5 操作

1. 标定姿态 → **A+B+X+Y** 启动 → **A+X** 进 POSE  
2. **左 Grip + A**：开始/结束 episode  
3. **左 Grip + B**：丢弃当前段  
4. 急停：物理急停；软件在部署终端按 `O`

---

## 4. 落盘

```text
outputs/<dataset>/
├── data/train-*.parquet
├── videos/observation.images.ego_view/episode_*.mp4
├── videos/observation.images.head/episode_*.mp4
└── meta/...
```

清洗：

```bash
source .venv_data_collection/bin/activate
python gear_sonic/scripts/process_dataset.py \
  --dataset-path outputs/<dataset> \
  --output-path outputs/<dataset>_cleaned
```

---

## 5. 排错

| 现象 | 处理 |
|---|---|
| 红物体变蓝 / 蓝变红 | RGB/BGR 通道反了。工作站需用已修复的 `ImageUtils.decode_image`（返回 RGB）；Orin 上 `g1_dual_realsense_rgb_zmq_server.py` 保持 RGB→BGR 再 JPEG。旧 episode 需重录 |
| `cv2.imshow` / GUI not implemented | `.venv_data_collection` 里卸掉 `opencv-python-headless`，保留 `opencv-python` |
| 只有一台 RealSense | 检查新相机线；应出现两个 SN |
| 双路深度掉帧 | 不要开深度；本脚本默认 RGB only |
| `5555` 连不上 | 机器人 server 未起；`ss -ltn \| grep 5555` |
| Orin `TypeError: 'type' object is not subscriptable` | 在跑 `composed_camera`；改用 `g1_dual_realsense_rgb_server.sh`（standalone） |
| Orin `libgomp: cannot allocate memory in static TLS block` | 确认脚本已 `LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1` |
| deploy `Address already in use` | `pkill -f g1_deploy_onnx_ref` |
| exporter 缺 `head` | 必须加 `--record-head-camera`，且 Orin 双路 server 在跑 |
| 左手夹爪不动 | 查 `rt/dex3/left/state` 是否有数据；必要时整机重启 |

---

## 6. 相关文档

- [pico_real_teleop.md](pico_real_teleop.md)  
- [mujoco_pico_sim_teleop.md](mujoco_pico_sim_teleop.md)  
- [../../source/tutorials/data_collection.md](../../source/tutorials/data_collection.md)  

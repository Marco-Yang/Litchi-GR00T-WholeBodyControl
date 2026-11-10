# G1 真机遥操数采（SONIC v1.1 + 双 RealSense RGB → GR00T VLA）

本文是本机实操 runbook：**G1 + SONIC v1.1** 真机 PICO 全身遥操数采，输出可直接给 **Isaac-GR00T N1.7 / `UNITREE_G1_SONIC`** 微调的 LeRobot 数据集。

策略固定用 `gear_sonic_deploy/policy/sonic_v1_1/`（`--cp policy/sonic_v1_1/model` + 对应 `observation_config.yaml`），不要用默认 `policy/release`。

遥操按键见 [pico_real_teleop.md](pico_real_teleop.md)。上游：[`data_collection.md`](../../source/tutorials/data_collection.md)、[`vla_workflow.md`](../../source/tutorials/vla_workflow.md)。

> **手部路径：** 下文 §0–§8 仍以 **Dex3** 数采为主。已换装 **强脑 Revo2** 时，Pico 扳机控手请直接看 **[§9 Pico 控强脑 Revo2](#9-pico-控强脑-revo2替代-dex3)**（逻辑与 Dex3 **相反**）。完整 Manus/Revo2 速查：[`../../../../Manus/命令速查.md`](../../../../Manus/命令速查.md)。

---

## 0. 与 GR00T G1 VLA 的对应关系

下游训练仓库：[NVIDIA/Isaac-GR00T](https://github.com/NVIDIA/Isaac-GR00T)，embodiment：`UNITREE_G1_SONIC`。

| GR00T 模态 | 本数采落盘字段 | 说明 |
|---|---|---|
| video `ego_view` | `videos/.../observation.images.ego_view/*.mp4` | **必须**；颈部 D435I |
| video `head` | `.../observation.images.head/*.mp4` | 与 Litchi-Isaac-GR00T `unitree_g1_sonic` 一致：**ego_view + head** 双相机训练 |
| video `left/right_wrist` | `.../observation.images.left_wrist` / `right_wrist` | 可选；腕部 Abham UVC（需 `--record-wrist-cameras` + Orin `:5558`） |
| state 腿/腰/臂/手 | `observation.state` 切片（含 Dex3 左右各 7D） | Dex3 **实测状态** |
| state `projected_gravity` | `observation.projected_gravity` | |
| action `motion_token` | `action.motion_token` (64D) | SONIC v1.1 **64D latent 监督**。**默认 = Origin / live**：盘上直接保留 deploy 闭环 `token_state`（与 `dataset/origin/...` 同流形）。可选开环 rewrite：加 `--rewrite-motion-token-supervised`，落盘前用 `teleop.smpl_*` + 腕解析 + **开环 heading** 经 `model_encoder.onnx` 重写（与 §8 同规则），live 备份到 `meta/motion_token_live/`。详见 [§4.5.1](#451-actionmotion_token-落盘两种路线) |
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

安全：吊带；物理急停有人盯；**已关掉** `run_sim_loop.py`。

**Revo2 数采需要启动（按序）：**

| 顺序 | 位置 | 进程 |
|------|------|------|
| 1 | Orin | Camera（§4.1） |
| 2 | Orin | **`revo2_modbus_rx`**（§4.2） |
| 3 | 工作站终端 1 | deploy SONIC v1.1（§4.3） |
| 4 | 工作站终端 2 | pico_manager（§4.4） |
| 5 | 工作站终端 3 | data exporter `--hand-type revo2`（§4.5） |
| 6 | 工作站 | CPU 绑核（§4.6；三端起来后执行） |

### 4.1 机器人 — Camera（ego + head + **左右腕**）

按 **§3.2**，Revo2 数采默认开四路：`ego_view`/`head` → `:5555`，`left_wrist`/`right_wrist`（Abham）→ `:5558`：

```bash
# Orin（需腕部 Abham HHW002/HHW001 在线）
bash gear_sonic_deploy/scripts/g1_camera_rgb_server.sh wrist
```

工作站确认四路画面（有什么显示什么）：

```bash
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl
source .venv_data_collection/bin/activate
python gear_sonic/scripts/run_camera_viewer.py --camera-host 192.168.123.164
# viewer 会订 :5555 + :5558；应看到 ego_view / head / left_wrist / right_wrist
```

### 4.2 机器人 — Revo2 Modbus 接收端（`revo2_modbus_rx`）

手部 UDP→串口桥，**Revo2 数采必开**（Dex3 旧流程可跳过）。在**工作站**执行（脚本会 SSH 到 Orin）：

```bash
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Manus
bash scripts/start_orin_revo2_rx.sh

# 确认日志
ssh unitree@192.168.123.164 'tail -f /tmp/revo2_modbus_rx.log'
```

串口须为 `/dev/ttyCH343USB*`（左 `USB2@0x7E`，右 `USB1@0x7F`）。详见 §9.4 / [`../../../../Manus/命令速查.md`](../../../../Manus/命令速查.md)。

> 数采时由 **data exporter** 经 UDP `:19500` 下发手指令；**不要**再并行 `run_real_revo2_pico.sh`。

### 4.3 工作站终端 1 — 部署（SONIC v1.1）

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
```

`real` 自动选本机 `192.168.123.x` 网卡。若脚本报找不到该网段，先查网线；仍不行把最后的 `real` 换成机器人实际 IP。

### 4.4 工作站终端 2 — Pico

数采可关可视化减负；首次上真机建议先开可视化确认脚部跟踪：

```bash
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl
source .venv_teleop/bin/activate
# 首次：加 --vis_vr3pt --vis_smpl；确认跟踪 OK 后再去掉减负
python gear_sonic/scripts/pico_manager_thread_server.py --robot g1 --manager
```

缺 `gear_sonic/data/human/human_joints_info.pkl` 时按 **A+X** 会直接崩。

### 4.5 工作站终端 3 — Data exporter

**Revo2（当前，需 §4.2 已起）：** Pico 扳机驱动录制 grasp 轨迹并 UDP 下发真手；落盘时 6D **pad 成 7D**（末维 0），匹配 GR00T `UNITREE_G1_SONIC`（action = 64 + 7 + 7）。

**相机落盘自动探测（默认）：** 启动时探测 Orin `:5555` / `:5558`。有 `head` → 录 head；`:5558` 上同时有 `left_wrist`+`right_wrist` → 录腕部；否则只录已有流。`ego_view` 始终在 schema 中。

| Orin | 探测结果（无需额外 exporter 相机参数） |
|---|---|
| `g1_camera_rgb_server.sh` | `ego_view` + `head` |
| `g1_camera_rgb_server.sh wrist` | 上表 + `left_wrist` + `right_wrist` |

强制开关（一般不用）：`--record-wrist-cameras` / `--no-record-wrist-cameras`，`--record-head-camera` / `--no-record-head-camera`。

**推荐命令（Revo2；相机自动；默认 Origin / live token）：**

```bash
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl
source .venv_data_collection/bin/activate
python gear_sonic/scripts/run_data_exporter.py \
  --task-prompt "走到绿色桌子前" \
  --camera-host 192.168.123.164 \
  --camera-port 5555 \
  --root-output-dir outputs \
  --hand-type revo2 \
  --revo2-pose ../Manus/config/revo2_grasp_pose_02.json \
  --revo2-orin-host 192.168.123.164 \
  --revo2-send
```

启动日志应出现：

- `[Camera] recording → ego_view, head`（或带腕部的四路）
- **`[motion_token] episode-end rewrite DISABLED — disk keeps live token_state`**（默认 live；必看）

若要开环 mode-2 监督，显式加 `--rewrite-motion-token-supervised`，日志变为 `rewrite ENABLED ... disk tokens = open-loop mode2`。

**Dex3（旧）手：** 不需要 §4.2；去掉 `--hand-type revo2` 及相关参数即可；相机同样自动探测。默认同样是 live。

#### 4.5.1 `action.motion_token` 落盘：三条相关路径

帧循环里 exporter **始终**先抄 deploy 的 `token_state` 进 buffer。落盘/清洗时还有三条互不混淆的路径：

| # | 路径 | 何时发生 | 参数 / 入口 | 盘上 `action.motion_token` | 启动/日志关键字 |
|---|---|---|---|---|---|
| **1** | **Origin / live（默认）** | 数采落盘 | 默认；或显式 `--no-rewrite-motion-token-supervised` | 闭环控机 live `token_state` | `rewrite DISABLED — disk keeps live token_state` |
| **2** | **开环 supervised rewrite** | 数采 episode 落盘前 | `--rewrite-motion-token-supervised` | 开环 mode-2 重写；live → `meta/motion_token_live/` | `rewrite ENABLED ... disk tokens = open-loop mode2` |
| **3** | **事后 `--token-refill`** | 清洗 `pipeline` / `refill-tokens` | `--token-refill`（默认关） | 用开环 mode-2 **覆盖已落盘** token | `[pipeline] refilled motion_token ...`；默认则是 `keeping on-disk` |

要点：

1. **默认已是 live**（路径 1）。`launch_data_collection.py` 不传该 flag 时也会走 live，无需再手动加 `--no-rewrite-...`。
2. **tyro 布尔开关**：关 rewrite = 默认 / `--no-rewrite-motion-token-supervised`；开 = `--rewrite-motion-token-supervised`。不要写 `=False`。
3. 控机不受影响：C++ deploy 始终用 live `token_state`；flag / refill 只改**盘上监督列**。
4. **路径 3 不要拿来「修正」live-origin 盘**：训 VLA 请保留 live；只有明确要开环 mode-2 监督时才 `--token-refill` 或数采时走路径 2。
5. 采完抽查（路径 1）：**不应**出现 `meta/motion_token_live/` backup，也不应有 `meta/motion_token_supervised_rewrite.jsonl` 的 `rewrote: true`；启动日志必须是 `DISABLED`。

### 4.6 工作站 — CPU 绑核 / 解绑（跟随优先）

**deploy / pico_manager / exporter（及可选 camera_viewer）都起来之后**再绑，优先保证 Sonic 跟随核，避免 exporter/viewer 抢核。

默认分配（32 逻辑核，如 9950X3D）：

| 进程 | CPU | nice |
|------|-----|------|
| pico_manager | 0–11 | -10 |
| deploy (`g1_deploy_onnx_ref`) | 12–23 | -10 |
| exporter | 24–30 | +10 |
| camera_viewer（若在跑） | **31**（单核） | +10 |

**绑核：**

```bash
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl
bash gear_sonic_deploy/scripts/pin_data_collection_cpus.sh
```

**解绑（恢复全核 0–31、nice=0）：**

```bash
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl
bash gear_sonic_deploy/scripts/unpin_data_collection_cpus.sh
```

说明：

- `renice -10` 若无效，可加 `sudo` 再跑 pin 脚本。
- pico_manager 虽绑 12 核，Python/GIL 下有效负载仍接近单核；deploy 才是多核策略侧。
- 进程重启后需重新执行 pin。viewer 单核可改：`VIEW_CPU=30 bash gear_sonic_deploy/scripts/pin_data_collection_cpus.sh`。

### 4.7 操作

1. 标定姿态 → **A+B+X+Y** 启动（PLANNER）→ 身体对齐后再 **A+X** 进 **POSE**  
2. **左 Grip + A**：开始/结束 episode  
   - **开录后先静止约 0.5–1s 再动手**，避开视频编码器起步尖峰，减少前期帧变稀。  
3. **左 Grip + B**：丢弃当前段  
4. **开合手（Revo2）**：**松扳机 = 开掌（录制 START）**；**扣扳机 = 沿 grasp JSON 闭合并保持**；再松 = 短时回开。左右扳机各控左右手。  
   - （Dex3 旧逻辑相反：默认闭合，扣扳机张开。）  
   - **左 Grip + A / B** 只做录制起停/丢弃，**不**驱动手。  
5. 抓取类任务在 episode 内明确做「伸手 → 扣扳机抓取 → 移动 → 松扳机放开」等。  
6. 急停：物理急停；部署终端按 `O`（焦点必须在该终端）

官方建议同一任务至少 **50–100** 条成功 episode 再微调。

---

## 5. 落盘与训前处理

```text
outputs/<dataset>/
├── data/chunk-000/episode_*.parquet
├── videos/chunk-000/observation.images.ego_view/episode_*.mp4
├── videos/chunk-000/observation.images.head/episode_*.mp4
├── videos/chunk-000/observation.images.left_wrist/episode_*.mp4   # Abham 左腕
├── videos/chunk-000/observation.images.right_wrist/episode_*.mp4  # Abham 右腕
└── meta/
    ├── info.json
    ├── modality.json      # GR00T 需要；含 video.left_wrist / right_wrist
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
| 想采 live 却变成开环 token | exporter 带了 `--rewrite-motion-token-supervised`；去掉后重启，确认日志 `rewrite DISABLED` |
| live 路线却有 `meta/motion_token_live/` | 说明该 session 实际跑过 rewrite；不要当 origin 用 |
| live 盘被 pipeline 冲成开环 | 清洗时误加了 `--token-refill`；live-origin 训 VLA 不要 refill |
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
- [§9 Pico 控强脑 Revo2](#9-pico-控强脑-revo2替代-dex3)  
- [`../../../../Manus/命令速查.md`](../../../../Manus/命令速查.md)

---

## 8. 数据回放筛选（在数据电脑上）

在装有 Isaac Lab 的**数据电脑**上做：关节/SMPL 跳变预筛 → Isaac mode-2 回放。仓库根目录：`Litchi-GR00T-WholeBodyControl`。

```bash
conda activate sikan
cd /path/to/Litchi-GR00T-WholeBodyControl
export OMNI_KIT_ACCEPT_EULA=YES
export CUDA_VISIBLE_DEVICES=0
```

清洗后每个 session 是独立数据集，回放时把 `--dataset` 指到具体 session 目录。

**`action.motion_token`：清洗默认不再清除/重写。**  
`pipeline` **默认保留盘上 token**，日志会打印 `keeping on-disk action.motion_token`。盘上内容取决于录制时的 exporter 路线（见 [§4.5.1](#451-actionmotion_token-落盘三条相关路径)）：

- **Origin / live（数采默认）**：盘上是 live `token_state`，训 VLA 请**直接用**，**不要** `--token-refill`。
- **开环 rewrite 采的数据**（录制时加了 `--rewrite-motion-token-supervised`）：盘上已是开环 mode-2，一般也**不必**再 refill。

仅当你明确要把某盘改成开环 mode-2 监督时，才加 `--token-refill`（或单独跑 `refill-tokens`）。旧 token 备份在 `<work-dir>/motion_token_backup/`；摘要：`meta/motion_token_mode2_refill.json`。

| 参数 | 默认 | 含义 |
|---|---|---|
| （无） | — | **默认不重写 token** |
| `--token-refill` | off | **可选**：用开环 mode2 覆盖盘上 `action.motion_token`（旧数据） |
| `--token-device` | `auto` | 仅配合 `--token-refill`：`cuda` / `cpu` / `auto` |
| `--token-workers` | `4` | 仅配合 `--token-refill`：并行 episode 数 |
| `--skip-token-refill` | — | 已废弃（默认就是跳过）；旧脚本带着也不影响 |

旧盘单独重填 token（不跑 Isaac）：

```bash
python -m datasets.data_check refill-tokens \
  --dataset datasets/walking \
  --device cuda --workers 8
```

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
- （仅在使用了 `--token-refill` 时）Token 重填：`<dataset>/meta/motion_token_mode2_refill.json`

---

## 9. Pico 控强脑 Revo2（替代 Dex3）

G1 已换 **Revo2** 后，手部不再走 Dex3 DDS / `pico_manager` 的 7D 夹爪映射；改为：

1. **Manus 手套** 录一条开→合轨迹 → JSON  
2. **Pico 左右扳机** 驱动该轨迹回放 → UDP → Orin Modbus → 真手  

仓库根（本节命令均在此目录）：

```text
/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Manus
```

更细的跟踪 / 串口 / 排错见 [`../../../../Manus/命令速查.md`](../../../../Manus/命令速查.md)。

### 9.1 扳机逻辑（与 Dex3 相反）

| 状态 | Revo2 | Dex3（旧，§4.5） |
|------|--------|------------------|
| **默认 / 扳机松开** | 录制 **START** = 五指全开 `[0]*6` | 默认 **闭合** |
| **扳机按下** | 沿录制轨迹播到 **STOP**（闭合）并保持 | 扣下 **张开** |
| **扳机再松开** | **约 0.35s** 平滑回 START（非整段倒放） | 松开再闭合 |

左右扳机各自控制左右手。回开过慢可加 `--release-blend-s 0.2`。

### 9.2 硬件 / 网络前提

| 项 | 要求 |
|---|---|
| 工作站 ↔ G1 | 有线 `192.168.123.x`，`ping 192.168.123.164` |
| Orin 串口 | **`/dev/ttyCH343USB*`**（ch343）；左 `USB2@0x7E`，右 `USB1@0x7F` |
| Manus（仅录制时） | Win **MANUS Core** `192.168.50.36`；本机 `ping` 通 |
| Pico（仅回放时） | PC Service 运行；头显 **WORKING** + Controller + **Send**；Wi‑Fi IP 填 `wlp8s0`（现约 `192.168.50.112`） |

```bash
# 网络
ip -4 -br addr | grep 192.168.123 || echo 'MISSING G1 control iface'
ping -c 2 -W 2 192.168.123.164

# PC Service（Pico 回放前）
pgrep -af RoboticsServiceProcess || bash /opt/apps/roboticsservice/runService.sh
```

### 9.3 拓扑

```text
【录制】
  Win MANUS Core 192.168.50.36
          │ SDK Remote
          ▼
  本机  real_revo2_record.py   R=开始 / S=写 JSON
          │ UDP :19500
          ▼
  Orin  revo2_modbus_rx.py → ttyCH343USB* → Revo2 L/R

【Pico 回放】
  PICO Trigger (XRoboToolkit)
          │
          ▼
  本机  real_revo2_trigger.py --source pico  +  pose JSON
          │ UDP :19500
          ▼
  Orin  同上 → Revo2
```

### 9.4 一次性 / 每次：Orin 接收端

```bash
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Manus

# 部署并启动 Orin Modbus UDP 接收（会停冲突的 brainco_hand.service）
bash scripts/start_orin_revo2_rx.sh

# 日志
ssh unitree@192.168.123.164 'tail -f /tmp/revo2_modbus_rx.log'
```

后续命令若加 `--no-orin-start`，表示假设接收端已在跑。

### 9.5 录制抓取轨迹（Manus）

空闲时手保持 **Revo2 全开**。按 **R** 后从全开 smoothstep 接到当前手套，再跟踪；**S** 结束写盘。

```bash
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Manus
# Win 上 MANUS Core 已开
bash scripts/run_real_revo2_record.sh --host 192.168.50.36 --no-orin-start

# 指定输出文件（推荐按条编号，避免覆盖）
bash scripts/run_real_revo2_record.sh --host 192.168.50.36 --no-orin-start \
  --out config/revo2_grasp_pose_02.json
```

| 键 | 作用 |
|----|------|
| **R** | 开始录制（轨迹从全开起 → blend → 跟 Manus） |
| **S** | 停止并写入 JSON |
| **q** | 退出（手回全开） |

默认不写 `--out` 时覆盖 `config/revo2_grasp_pose.json`。

可选：先只做连续跟踪确认映射：

```bash
bash scripts/run_real_revo2_track.sh --host 192.168.50.36 --no-orin-start
```

### 9.6 Pico 扳机回放（真机）

头显已 **WORKING + Controller + Send**，Orin 接收端已起：

```bash
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Manus

# 推荐：真机 + 真实 PICO 扳机
bash scripts/run_real_revo2_pico.sh --no-orin-start \
  --pose config/revo2_grasp_pose_02.json

# 等价写法
bash scripts/run_sim_revo2_trigger.sh --pico --no-orin-start \
  --pose config/revo2_grasp_pose_02.json

# 回开更快（可选）
bash scripts/run_real_revo2_pico.sh --no-orin-start \
  --pose config/revo2_grasp_pose_02.json \
  --release-blend-s 0.2
```

| 输入 | 作用 |
|------|------|
| PICO **左 / 右扳机** | 左 / 右手：按下→播到 close；松开→短时回 open |
| 终端 **`q`** | 退出（手回全开） |

终端日志里扣扳机后应看到 `raw L/R` 升高、`L`/`R` 标志亮起；默认停在开掌全 0 是正常的。

### 9.7 无 Pico 时的验录 / 键盘回放

```bash
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Manus

# MuJoCo 仿真（键盘当扳机）
bash scripts/run_sim_revo2_trigger.sh --pose config/revo2_grasp_pose_02.json

# 真机 + 键盘扳机
bash scripts/run_sim_revo2_trigger.sh --real --no-orin-start \
  --pose config/revo2_grasp_pose_02.json

# 启动后立刻自动播一遍（不必扣扳机）
bash scripts/run_sim_revo2_trigger.sh --real --no-orin-start --autoplay \
  --pose config/revo2_grasp_pose_02.json
```

| 键 | 作用 |
|----|------|
| `1` / `2` / `b` | 左 / 右 / 双手扳机（切换） |
| `p` | 双手自动播一遍 open→close |
| `空格` | 清扳机并回开 |
| `q` | 退出 |

### 9.8 推荐现场顺序

**A. 只验手（不数采）**

1. 网线 + `ping 192.168.123.164`  
2. Manus 目录：`bash scripts/start_orin_revo2_rx.sh`  
3. （可选）录轨迹：`run_real_revo2_record.sh ... --out config/revo2_grasp_pose_XX.json`  
4. Pico 回放：`run_real_revo2_pico.sh --no-orin-start --pose config/revo2_grasp_pose_XX.json`

**B. 全身数采 + Revo2 手（推荐）**

1. Orin：`g1_camera_rgb_server.sh`（无腕部则**不要** `wrist`）  
2. Manus：`start_orin_revo2_rx.sh`  
3. 工作站：PC Service + `deploy.sh ... real`（§4.2）+ `pico_manager_thread_server.py`（§4.3）  
4. Data exporter 用 **§4.4 Revo2** 命令（`--hand-type revo2 --revo2-pose ...`）  
5. 操作同 §4.5，但手部：**松扳机=开掌，扣扳机=沿录制轨迹闭合**（与 Dex3 相反）  

落盘（对齐 GR00T N1.7 / `UNITREE_G1_SONIC`）：

| 字段 | 维度 | Revo2 含义 |
|------|------|------------|
| `action.motion_token` | 64 | 默认：live `token_state`；可选 `--rewrite-motion-token-supervised` 写开环 mode-2（见 §4.5.1） |
| `teleop.left/right_hand_joints` | **7**（前 6 = Revo2 0..1000，第 7 = 0） | 扳机轨迹指令 |
| `observation.state` 手部切片 | 各 7 | 用 Revo2/1000 填入（无 Dex3 实测） |
| `observation.images.ego_view` | 必选 | Orin `:5555` |
| `meta/modality.json` | `left/right_hand_joints.end=7` | 与 Dex3 数采同 schema |

`script_config.hand_type=revo2`。勿与旧 Dex3 数据混训同一 hand 统计（量纲不同）。

### 9.9 排错（Revo2）

| 现象 | 处理 |
|---|---|
| 回放一直全 0 | 未扣扳机；或键盘源未按 `p`/`b`。Pico 源看 `raw L/R` 是否 >0.45 |
| 扣扳机手不动 | Orin rx 是否在跑；`tail /tmp/revo2_modbus_rx.log`；串口是否为 `ttyCH343USB*` |
| 松开回开很慢 | 旧逻辑整段倒放；更新后默认 0.35s，可加 `--release-blend-s 0.2` 并**重启**脚本 |
| Pico `raw` 一直 0 | PC Service / 头显未 WORKING 或未 Send；Wi‑Fi IP 是否填对 |
| 与 Dex3 习惯反了 | Revo2：**松=开，扣=合**；Dex3：**松=合，扣=开** |

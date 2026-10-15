# MuJoCo 仿真 + PICO 全身遥操操作指南

本文说明如何在本机用 **PICO VR** 对 **MuJoCo 中的 Unitree G1** 做 SONIC 全身遥操。  
环境需已按仓库 README 配置完成（`.venv_teleop`、C++ `gear_sonic_deploy`、模型文件、XRoboToolkit PC Service）。

---

## 架构概览

```
PICO 头显/手柄/脚部 Tracker
        │  (XRoboToolkit，同 Wi‑Fi)
        ▼
本机 PC Service + pico_manager_thread_server.py
        │  (ZMQ)
        ▼
gear_sonic_deploy（C++ 策略，--input-type zmq_manager sim）
        │  (DDS / sim bridge)
        ▼
run_sim_loop.py（MuJoCo 仿真窗口）
```

需要同时开 **3 个终端**。

### 用 `--robot` 指定机器人

三个终端的命令都带 `--robot <名字>`（默认就是 `g1`，显式写出来是为了和训练、其它机器人保持同一套用法）。**三个终端必须用同一个机器人。**

| 终端 | 命令中的写法 | 说明 |
|---|---|---|
| 1 仿真 | `run_sim_loop.py --robot g1` | `g1` 走 Unitree-SDK2 仿真；`a3` / `a3_ultra` 走 A3 的 ROS2 仿真（第 8 节）；`h2` 暂无仿真配置，会报错 |
| 2 部署 | `./deploy.sh --robot g1 ... sim` | `--robot` 写在 `sim`/`real` 之前；`a3` / `a3_ultra` 转给 `scripts/deploy_a3.sh`；`h2` 暂无 C++ 后端 |
| 3 Pico | `pico_manager_thread_server.py --robot g1 --manager` | 决定用哪个机器人的 IK/FK 模型（`g1`/`a3`/`a3_ultra`/`h2` 都已注册） |

训练同理：`python gear_sonic/train_agent_trl.py --robot g1 ...`。

---

## 0. 前置检查

仓库根目录（下文记为 `$REPO`）：

```text
/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl
```

新开终端建议先：

```bash
source ~/.bashrc
# 确认已设置：
#   export TensorRT_ROOT=$HOME/TensorRT
#   export PATH=/usr/local/cuda-13.2/bin:$HOME/.local/bin:$PATH
```

确认文件存在：

| 项 | 路径 |
|---|---|
| 遥操环境 | `$REPO/.venv_teleop` |
| 部署可执行文件 | `$REPO/gear_sonic_deploy/target/release/g1_deploy_onnx_ref` |
| 默认策略 | `$REPO/gear_sonic_deploy/policy/release/model_*.onnx` |
| Planner | `$REPO/gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx` |

硬件：

- PICO 4 / 4 Pro + 双手柄
- 2× 脚踝 Motion Tracker（指示灯朝上）
- 穿**紧身裤**，避免遮挡脚部跟踪
- 电脑与 PICO 在**同一低延迟 Wi‑Fi**（尽量私有路由器）

---

## 1. 连接 PICO 与电脑

先启动 PC Service，并在头显里连上，状态变成 **WORKING**。完成这一节之后，再开第 2 节的三个终端。

### 1.1 电脑端：启动 PC Service

PC Service 装在 `/opt/apps/roboticsservice/`，进程名是 `RoboticsServiceProcess`。三个终端启动之前先确认它在运行：

```bash
pgrep -af RoboticsServiceProcess
```

没有输出就启动（会弹出 PC Service 窗口）：

```bash
bash /opt/apps/roboticsservice/runService.sh
```

应用菜单里的名称是 **XRoboToolkit-PC-Service**。遥操用上面的 `runService.sh` 即可，不需要 `run3D.sh`。

头显要填的是这台电脑的局域网地址，不是 `hostname -I` 里后面的虚拟网卡地址：

| 地址 | 用途 |
|---|---|
| **192.168.110.79** | 无线网卡 `wlp8s0`。头显 PC Service 一栏填这个 |
| 100.95.251.103 | Tailscale，不要填 |
| 198.18.0.1 | Meta 虚拟网卡，不要填 |

地址变了再查一次，取 `wlp8s0` 那一行：

```bash
hostname -I
ip -4 -br addr show wlp8s0
```

### 1.2 头显端

1. PICO 与电脑连同一 Wi‑Fi。
2. 打开 **XRoboToolkit** App。
3. 在 **PC Service** 填入 **192.168.110.79**；状态显示 **WORKING** 即连接成功。若已填过，点 **Reconnect**。
4. 勾选：
   - Tracking：`Head`、`Controller`
   - Data/Control：点 **Send**
   - Pico Motion Tracker：**Full body**
5. 开发者选项中关闭 **Safeguard**。
6. 配对并标定两个脚部 Tracker（站直标定 → 低头看脚直到识别）。

> 终端 3 若无可视化窗口或报连不上设备，优先排查本节。

---

## 2. 三个终端启动

开始前确认第 1 节已经完成：PC Service 进程在运行，头显状态是 **WORKING**，并且已经点了 **Send**。PC Service 没起来时，头显填 `192.168.110.79` 会报 `connect_error TCP connection failed`。

### 终端 1 — MuJoCo 仿真

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
source .venv_teleop/bin/activate
python gear_sonic/scripts/run_sim_loop.py --robot g1
```

应弹出 MuJoCo 窗口；机器人初始可能悬空，属正常。

### 终端 2 — C++ 策略（等终端 1 就绪后再开）

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
source scripts/setup_env.sh
./deploy.sh --robot g1 --input-type zmq_manager sim
```

等到日志出现 **`Init Done`** 再继续。

使用 **SONIC v1.1** 时：

```bash
./deploy.sh \
  --robot g1 \
  --cp policy/sonic_v1_1/model \
  --obs-config policy/sonic_v1_1/observation_config.yaml \
  --input-type zmq_manager \
  sim
```

### 终端 3 — PICO 数据流

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --robot g1 --manager --vis_vr3pt --vis_smpl
```

应再弹出可视化窗口（G1 参考姿态 / SMPL）。首次建议带 `--vis_vr3pt --vis_smpl`，确认跟踪正常后再去掉可视化以减负。

---

## 3. 第一次遥操流程

1. **标定姿态**：站直、双脚并拢、看向前方；上臂贴身，前臂向前弯约 90°（肘成 L），掌心向内。
2. 手柄同时按 **A + B + X + Y**  
   → 策略启动 + 全身标定（`CALIB_FULL`），进入 **PLANNER** 模式。
3. 在 MuJoCo 窗口点击一下，按 **`9`** 可将机器人放到地面（按文档/现场习惯操作）。
4. 将自己身体摆成与仿真机器人接近后，按 **A + X**  
   → 进入 **POSE**（全身镜像遥操）。
5. 自然走路、伸手即可；过度犹豫、刻意放慢反而更容易失稳。
6. 再按 **A + X** → 回到 PLANNER。
7. 再按 **A + B + X + Y** → 停止策略（OFF）。

### 急停

| 方式 | 操作 |
|---|---|
| PICO 手柄 | **A + B + X + Y** |
| 终端 2 键盘 | **`O`** |

### 模式切换安全

切到 **POSE** 或 **VR_3PT** 前，务必先让身体与机器人当前姿态大致对齐。  
错位时切换会导致仿真里猛然跟姿；真机上会非常危险。

---

## 4. 常用手柄说明（摘要）

| 操作 | 按键 |
|---|---|
| 启动 / 急停 | A+B+X+Y |
| PLANNER ↔ POSE | A+X |
| 进入/退出 VR_3PT（手腕重标定） | 左摇杆按下 |
| 移动（Planner 相关模式） | 左摇杆 |
| 转向 | 右摇杆水平 |
| 切换步态模式 | A+B / X+Y |

完整说明见仓库文档：`docs/source/tutorials/vr_wholebody_teleop.md`。

---

## 5. 常见问题

| 现象 | 处理 |
|---|---|
| 终端 3 无窗口 / 无数据 | 查 PC Service 是否运行、头显 Status 是否 WORKING、是否点了 Send、是否同网段 |
| 脚部乱、易摔 | Tracker 未配对、电池不足、衣物遮挡；重新标定 |
| 动作一顿一顿 | Wi‑Fi 延迟；终端 2 若出现 `High ZMQ latency`，换近距离私有路由 |
| 延迟参考 | 好：&lt;10 ms；可接受：10–30 ms；差：&gt;30 ms |
| 找不到 TensorRT | `source ~/.bashrc`，确认 `echo $TensorRT_ROOT` 为 `$HOME/TensorRT` |
| 路径含空格 | 命令中的 `cd` 路径请始终加引号（本仓库路径含空格） |

---

## 6. 建议练习顺序

1. 只开仿真 + 部署，用键盘熟悉急停（`O`）。
2. 接上 PICO，先在可视化窗口确认 avatar / 脚部跟踪正确。
3. 标定 → PLANNER → 对齐 → POSE，只做站立与缓慢手臂动作。
4. 再练自然行走与转向。
5. **仿真熟练并会急停后**，再按 `docs/notes/pico_real_teleop.md` 上真机。真机时必须先关掉 `run_sim_loop.py`（sim 与 real 不能同时跑）。

真机启动示例（勿与终端 1 同时开；完整步骤见 `docs/notes/pico_real_teleop.md`）：

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
source scripts/setup_env.sh
./deploy.sh --robot g1 --input-type zmq_manager real
```

---

## 7. 相关官方文档

- VR 硬件与软件安装：`docs/source/getting_started/vr_teleop_setup.md`
- 全身遥操教程：`docs/source/tutorials/vr_wholebody_teleop.md`
- 遥操最佳实践：`docs/source/user_guide/teleoperation.md`
- Quick Start（含 sim2sim）：`docs/source/getting_started/quickstart.md`

---

## 8. 同一套三终端流程用于 A3 / A3 Ultra（`--robot a3_ultra`）

> A3 / A3 Ultra 的**完整教程**（架构、启动日志、第一次遥操、没有头显时的联调、常见问题、验证范围）见 [../a3/mujoco_pico_sim_teleop.md](../a3/mujoco_pico_sim_teleop.md)；本节保留作速查。

三个终端的角色和 G1 完全一致，只是机器人侧接口从 Unitree DDS 换成 A3 的 ROS2 `/body_drive/*`（同一个 C++ 部署二进制既连仿真也连真机）。
权重用 035（`a3` 与 `a3_ultra` 共用，同一硬件平台）；仿真里 `a3_ultra` 用 A3 Ultra 自己的模型（T3D0 MJCF）。

**一次性准备**（只需做一次）：

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
./scripts/setup_a3_sim_env.sh        # 给 conda 环境 a3_deploy_verify 装 mujoco + joint_msgs Python 绑定
```

并确保 035 权重与部署包存在（`deploy.sh --robot a3_ultra` 首次会自动下载/构建）。

### 权重：直接用下载的 035（不训练自己的 a3_ultra 模型）

- 唯一可下载的权重是 Hugging Face `sonic-for-a3/sonic` 的 `035_step200000`（PT / ONNX / RKNN；整个 HF 与 GitHub 上没有 A3 Ultra 专属权重）。官方 README 与发布元数据（`config.json` 里 `"robots": ["A3", "A3 Ultra"]`、`"policy_joints": 29`）都明确说它同时支持 A3 与 A3 Ultra（训练用 A3 的 URDF，域随机化针对 A3 / A3U 两者调过）。下载命令：
  ```bash
  python download_from_hf.py --robot a3_ultra --component onnx      # 部署/仿真用（g1 与 a3_fast 两个 ONNX）
  python download_from_hf.py --robot a3_ultra --component rknn      # RK3588 板子用
  python download_from_hf.py --robot a3_ultra --component pt        # PyTorch 权重（继续训练/导出用）
  ```
- 策略只驱动 **29 个身体关节**。A3 Ultra 多出来的两个头关节**固定在默认姿态**（q=0，Kp=40 / Kd=2，与 A3 完全一致），手指不驱动；其余 29 个关节的输出、Kp/Kd、动作缩放、默认站姿都和 A3 一致。Pico 链路里的重定向节点默认也不发送头部角度（`a3_teleop_retarget_node.py --track-head` 才会发）。
- 兼容性的依据和本机验证见 `docs/workflow/a3_integration_plan.md` 第 14 节；**真机未验证**。
> 仓库路径含空格会让部分脚本拆分参数，建议用无空格软链接路径，例如 `ln -s "$REPO" /tmp/gwbc`，下面都在 `/tmp/gwbc` 下执行。

### 终端 1 — MuJoCo 仿真

```bash
cd /tmp/gwbc && source .venv_teleop/bin/activate
python gear_sonic/scripts/run_sim_loop.py --robot a3_ultra
```

会自动切到带 ROS2 的 conda 环境运行（日志里有 `re-running under .../a3_deploy_verify/...`）。机器人一开始被弹性绳**吊离地面**（pelvis ≈ 1.35 m），和 G1 一样；窗口里按 **`9`** 放到地面，`7/8` 调绳长。
默认 `ROS_DOMAIN_ID=87` + `ROS_LOCALHOST_ONLY=1`，**不要**在开发机上用默认域 0（见 plan 9.7）。

### 终端 2 — C++ 策略 + Pico 重定向链

```bash
cd /tmp/gwbc/gear_sonic_deploy
source scripts/setup_env.sh
./deploy.sh --robot a3_ultra --input-type zmq_manager sim --encoder a3-fast      # 默认，下载的 035 a3_fast 权重
```

等到日志出现 **`Init Done`** 再继续（部署会自动进入 PD 站立，和 G1 的 Init 坡道对应）。

`--encoder` 选 TELEOP 用哪个编码器，启动时就选好并**锁定**（部署程序的 `--teleop-source`）：
- `a3-fast`（默认）：`--input-type zmq_manager` 会在部署程序旁边自动起 `a3_teleop_retarget_node.py`（Pico SMPL → GMR → A3 关节参考）和 `a3_ta_bridge`（→ ROS2 `/ta/whole_body_command`）。**完整可用。**
- `smpl`：需要 SMPL-encoder ONNX；**A3 / A3 Ultra 目前没有**（035 只有 `g1` 与 `a3_fast`）。
- `g1`：同 `a3-fast` 链路但延迟约 940 ms，只作对照。

日志里若出现 `xrobotoolkit_sdk not found, skip for now`，这是重定向环境的提示，**不是报错**，可以忽略。

键盘仍可用作备份（平时按手柄即可）：

| 键 | 作用 |
|---|---|
| `S` / `T` / `P` | PD 站立 / 遥操 / 被动（急停软件兜底） |
| `1` / `2` / `3` | 选源（经 `deploy.sh` 启动时已锁定，按别的源会 `rejected`） |
| `Q` | 退出 |

### 终端 3 — PICO 数据流

```bash
cd /tmp/gwbc && source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --robot a3_ultra --manager --num_frames_to_send 10
```

### 第一次遥操（与第 3 节 G1 相同）

1. 终端 2 等到 **`Init Done`**（机器人仍吊着、PD 站立）。
2. 手柄 **A + B + X + Y** → 部署进入 teleop（日志有 `[pico-cmd] ... start -> teleop`）。
3. MuJoCo 窗口点一下，按 **`9`** 放到地面。
4. 身体与机器人姿态大致对齐后，按 **A + X** → **POSE**（全身镜像；A3 没有 planner 策略，POSE 才会持续推姿态流）。
5. 再按 **A + B + X + Y** → 停止（passive）。

`--vis_vr3pt` 的 G1 网格叠加对 A3 不可用，会自动关闭。

### 没有头显时的联调（用 Pico 模拟器代替终端 3）

```bash
PYTHONPATH=. /home/user/miniconda3/envs/gmr/bin/python gear_sonic/scripts/pico_pose_zmq_replay.py --clip synthetic:wave --stand-seconds 6 --loop
```

### 真机

```bash
# 操作端 PC（连头显的那台）：Pico 服务 + 操作端链路（a3-fast 经 ROS2 把 /ta/whole_body_command 发给机器人，要与机器人同一 ROS_DOMAIN_ID）
python gear_sonic/scripts/pico_manager_thread_server.py --robot a3_ultra --manager --num_frames_to_send 10
gear_sonic_deploy/scripts/a3_operator_chain.sh --encoder a3-fast --real
# 机器人上（--no-chain = 不在本机起操作端链路；--zmq-host 必须是操作端 PC 的 IP，否则收不到手柄的 A+B+X+Y）。真机以 IDLE 启动，按 S 才进 PD 站立
./deploy.sh --robot a3_ultra --input-type zmq_manager --no-chain --zmq-host <操作端IP> real --encoder a3-fast   # 不要与终端 1 同时开
```

**真机的完整流程（两台机器、aarch64 包、`sm_config`、preflight / probe、急停与失效行为、未验证项）见 [../a3/pico_real_teleop.md](../a3/pico_real_teleop.md)。**

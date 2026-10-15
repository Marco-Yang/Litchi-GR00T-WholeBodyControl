# 真机 + PICO 全身遥操操作指南

本文说明如何在本机用 **PICO VR** 对**真机**做 SONIC 全身遥操。仿真流程见同目录的 `mujoco_pico_sim_teleop.md`。先在仿真里练熟急停和模式切换，再按本文上真机。

环境需已按仓库 README 配置完成（`.venv_teleop`、C++ `gear_sonic_deploy`、模型文件、XRoboToolkit PC Service）。

**安全**：旁边要有人能按物理急停。软件急停（G1 的 `O` / 手柄 A+B+X+Y，A3 的 `P`）不能代替物理急停。`run_sim_loop.py` 必须先关掉，仿真和真机不能同时跑。

---

## 架构概览

```
PICO 头显/手柄/脚部 Tracker
        │  (XRoboToolkit，同 Wi‑Fi，填电脑局域网 IP)
        ▼
本机 PC Service + pico_manager_thread_server.py
        │  (ZMQ)
        ▼
gear_sonic_deploy（C++ 策略，--input-type zmq_manager real）
        │
        ▼
真机（G1：Unitree DDS；A3 / A3 Ultra：AimRT）
```

真机**不要**开 MuJoCo。G1 是 **2 个终端**；A3 / A3 Ultra 也是 2 个终端，部署程序会在旁边再拉起重定向和 ROS2 桥。

### 用 `--robot` 指定机器人

两个终端必须用同一个机器人。

| 终端 | 命令中的写法 | 说明 |
|---|---|---|
| 1 部署 | `./deploy.sh --robot g1 ... real` | `--robot` 写在 `real` 之前。`a3` / `a3_ultra` 转给 `scripts/deploy_a3.sh`。`h2` 暂无 C++ 后端 |
| 2 Pico | `pico_manager_thread_server.py --robot g1 --manager` | 决定用哪个机器人的 IK/FK 模型 |

头显填的 **192.168.110.79** 是这台电脑的 Wi‑Fi 地址，用来收 PICO 数据。它不是机器人的地址。G1 真机走另一张网卡上的 `192.168.123.x`，由 `real` 自动选择。

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

确认没有残留的仿真进程：

```bash
pgrep -af run_sim_loop.py
```

有输出就先停掉。仿真和真机同时开会抢同一路接口。

确认文件存在：

| 项 | 路径 |
|---|---|
| 遥操环境 | `$REPO/.venv_teleop` |
| 部署可执行文件 | `$REPO/gear_sonic_deploy/target/release/g1_deploy_onnx_ref` |
| 默认策略 | `$REPO/gear_sonic_deploy/policy/release/model_*.onnx` |
| Planner | `$REPO/gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx` |

硬件：

- 机器人已上电，急停可按，周围留出空地；首次上电用吊带或龙门架
- PICO 4 / 4 Pro + 双手柄
- 2× 脚踝 Motion Tracker（指示灯朝上）
- 穿**紧身裤**，避免遮挡脚部跟踪
- 电脑与 PICO 在**同一低延迟 Wi‑Fi**
- 电脑与机器人另有一条控制网络（G1 为 `192.168.123.x`）

---

## 1. 连接 PICO 与电脑

先启动 PC Service，并在头显里连上，状态变成 **WORKING**。完成这一节之后，再开第 2 节的终端。

### 1.1 电脑端：启动 PC Service

PC Service 装在 `/opt/apps/roboticsservice/`，进程名是 `RoboticsServiceProcess`。部署终端启动之前先确认它在运行：

```bash
pgrep -af RoboticsServiceProcess
```

没有输出就启动（会弹出 PC Service 窗口）：

```bash
bash /opt/apps/roboticsservice/runService.sh
```

应用菜单里的名称是 **XRoboToolkit-PC-Service**。遥操用上面的 `runService.sh` 即可，不需要 `run3D.sh`。

头显要填的是这台电脑的局域网地址，不是 `hostname -I` 里后面的虚拟网卡地址，也不是机器人的 `192.168.123.x`：

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
6. 配对并标定两个脚部 Tracker（站直标定 → 低头看脚直到识别）。标定后把头显推到额头上、镜头朝前，让摄像头继续看见脚踝。XRoboToolkit 里 Full body 的 Tracker 数量应为 2。

> 终端 2 若一直刷 `waiting for body data...`，先确认 Tracker 数量是 2，再重启这条 Python 命令。PC Service 同一时间只给最新的一个程序推跟踪数据。

---

## 2. 两个终端启动（G1）

开始前确认第 1 节已经完成：PC Service 进程在运行，头显状态是 **WORKING**，并且已经点了 **Send**。PC Service 没起来时，头显填 `192.168.110.79` 会报 `connect_error TCP connection failed`。

不要启动 `run_sim_loop.py`。

### 终端 1 — C++ 策略

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
source scripts/setup_env.sh
./deploy.sh --robot g1 --input-type zmq_manager real
```

`real` 会找带 `192.168.123.` 的网卡。找不到时脚本会改用其它非回环网卡并打印警告；这时把最后的 `real` 换成机器人实际 IP，例如 `192.168.123.161`。

等到日志出现 **`Init Done`** 再继续。启动前终端会问 `Proceed with deployment? [Y/n]`，输入 `y`。

使用 **SONIC v1.1** 时：

```bash
./deploy.sh \
  --robot g1 \
  --cp policy/sonic_v1_1/model \
  --obs-config policy/sonic_v1_1/observation_config.yaml \
  --input-type zmq_manager \
  real
```

### 终端 2 — PICO 数据流

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --robot g1 --manager --vis_vr3pt --vis_smpl
```

应弹出可视化窗口（G1 参考姿态 / SMPL）。确认脚部跟踪正确后，可以去掉 `--vis_vr3pt --vis_smpl` 以减负。窗口停在 `waiting for body data...` 时，回到第 1.2 节核对 Full body 和 Tracker 数量。

---

## 3. 第一次遥操流程

机器人先吊着或旁边有人扶稳。操作者和安全员不要是同一个人。

1. **标定姿态**：站直、双脚并拢、看向前方；上臂贴身，前臂向前弯约 90°（肘成 L），掌心向内。保持 1–2 秒不动。
2. 手柄同时按 **A + B + X + Y**  
   → 策略启动 + 全身标定（`CALIB_FULL`），进入 **PLANNER** 模式。
3. 先只在 PLANNER 里用摇杆做很小的移动，确认机器人响应方向正确。
4. 将自己身体摆成与机器人当前姿态接近后，再按 **A + X**  
   → 进入 **POSE**（全身镜像遥操）。
5. 先做站立和缓慢抬臂。确认稳定后再走路。
6. 再按 **A + X** → 回到 PLANNER。
7. 再按 **A + B + X + Y** → 停止策略（OFF）。

### 急停

机器人已经在乱动时，先按机器人上的**物理急停**。软件急停是下面两条，不能代替物理急停。

**G1：在部署那个终端里按 `O`（不要按回车）。** 这是程序里的急停。它会停掉策略，并在退出前发一帧阻尼指令：前馈力矩 0、kp=0、kd=8。键盘焦点必须在这个终端；在别的窗口按 `O` 没有用。

部署窗口不在眼前、来不及点回去时，在任意一个终端执行：

```bash
pkill -INT -x g1_deploy_onnx_ref || pkill -TERM -x g1_deploy_onnx_ref
```

这条命令只是让部署进程停掉，**不会**走 `O` 那条阻尼指令。进程停了以后如果机器人还在用力或继续倒，立刻按物理急停。不要用 `kill -9`，那会跳过进程自己的退出清理。

### 模式切换安全

切到 **POSE** 或 **VR_3PT** 前，务必先让身体与机器人当前姿态大致对齐。错位时切换，真机会猛然跟姿。

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
| 头显 `connect_error TCP connection failed` | PC Service 没启动。先跑第 1.1 节，再填 `192.168.110.79` |
| 终端 2 一直 `waiting for body data...` | Head/Controller 通了也不够。Full body 的 Tracker 数量要是 2，并且摄像头能看见脚踝。改完后重启终端 2 |
| `Could not find 192.168.123.x interface` | 电脑没接到 G1 的控制网。接上网线或把 `real` 换成机器人 IP |
| 终端 1 和仿真抢接口 | 停掉 `run_sim_loop.py` 后再开 `real` |
| 动作一顿一顿 | PICO 的 Wi‑Fi 延迟。终端 1 若出现 `High ZMQ latency`，换近距离私有路由 |
| 找不到 TensorRT | `source ~/.bashrc`，确认 `echo $TensorRT_ROOT` 为 `$HOME/TensorRT` |
| 路径含空格 | 命令中的 `cd` 路径请始终加引号 |

---

## 6. A3 / A3 Ultra 真机（`--robot a3_ultra`）

本机现有的真机是 **A3 Ultra**。权重用下载的 035（`a3` 与 `a3_ultra` 共用）。策略只驱动 29 个身体关节；多出来的头关节保持默认姿态，手指不驱动。不要和仿真同时开。

**A3 真机有单独的教程：[../a3/pico_real_teleop.md](../a3/pico_real_teleop.md)**（两台机器、aarch64 包、`sm_config`、preflight / probe、急停与失效行为），以及**部署进程放在本机、通过 ZMQ 中继连机器人**的路线（第 4.6 节）。下面是摘要。

PICO 的连接、IP 和 PC Service 与第 1 节相同。操作顺序和 G1 一致：部署程序以 **IDLE** 启动、不发指令 → 机器人悬空时按 **A+B+X+Y**（或键盘 `S`）让机器人**硬起来**并保持默认站立姿态（PD 站立，打印 `Init Done`）→ 放下吊带站稳 → **A+X**（开始跟随，遥操）→ 再按 **A+X** 回到默认站立姿态 → **A+B+X+Y** 失能（passive）。A3 没有 planner 策略，站立姿态由 PD 站立承担。链路 / 状态流失联时部署程序自己失能，姿态流静默 1 秒回到站立姿态。

### 终端 1 — C++ 策略

路径含空格时用无空格软链接，例如 `ln -s "$REPO" /tmp/gwbc`。

```bash
cd /tmp/gwbc/gear_sonic_deploy
./deploy.sh --robot a3_ultra --input-type zmq_manager real --encoder a3-fast
```

`--encoder a3-fast`（默认）：这条命令会在部署程序旁边启动 `a3_teleop_retarget_node.py`（PICO 的 SMPL → A3 关节）和 `a3_ta_bridge`（发到 `/ta/whole_body_command`），并监听 Pico 的 ZMQ `command`（A+B+X+Y / A+X）；**不会**自动进入 PD 站立（A+B+X+Y 或按 `S`）。启动前会打印 REAL ROBOT 横幅并要求确认。`--encoder smpl --smpl-model <onnx>` 需要 SMPL-encoder 模型，**A3 / A3 Ultra 目前没有**。`real` 使用现场的 `A3_TRANSPORT`、`ROS_DOMAIN_ID`、`A3_ROBOT_ENV`，**不要**沿用仿真的 `ROS_DOMAIN_ID=87` 和 `ROS_LOCALHOST_ONLY=1`。

机器人上通常没有 GMR / mujoco 环境，这时把操作端链路放到连头显的 PC 上、机器人只跑部署：

```bash
# 操作端 PC（与机器人同一 ROS_DOMAIN_ID 的网络里）
gear_sonic_deploy/scripts/a3_operator_chain.sh --encoder a3-fast --real
# 机器人上
./deploy.sh --robot a3_ultra --input-type zmq_manager --no-chain --zmq-host <操作端IP> real --encoder a3-fast
```

首次在吊带下进行。按 `S` 等到 **`Init Done`**，放下吊带站稳后：A+B+X+Y 启动遥操，再 A+X。键盘备份：

| 键 | 作用 |
|---|---|
| `S` / `T` / `P` | PD 站立 / 遥操 / 被动（软件兜底，不能替代物理急停） |
| `1` / `2` / `3` | 源已由 `--encoder` 锁定，按别的源会 `rejected` |
| `Q` | 退出 |

A3 部署窗口不在眼前时，在任意终端执行：

```bash
pkill -INT -x a3_deploy_onnx_ref || pkill -TERM -x a3_deploy_onnx_ref
```

`SIGINT` 会让 A3 部署程序走自己的退出流程并停止发指令。它不等于按 `P` 进入被动。机器人仍在乱动时，按物理急停。

### 终端 2 — PICO 数据流

```bash
cd /tmp/gwbc && source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --robot a3_ultra --manager --num_frames_to_send 10
```

`--num_frames_to_send 10`：SMPL 编码器要 10 帧窗口（`a3-fast` 用 5 或 10 都行）。`--vis_vr3pt` 的 G1 网格叠加对 A3 不可用，会自动关闭。

上电后如果策略一直不启动，先在机器人侧看 `/body_drive/neck_joint_state` 里的头关节名字是不是 `head_yaw_joint` / `head_pitch_joint`。缺这一路状态时，部署程序不会开始输出指令。更完整的上机前检查在 `docs/a3_training2sim2deploy.md` 第 6 节和 `docs/workflow/a3_integration_plan.md` 第 14.5 节。

---

## 7. 相关文档

- 仿真操作：`docs/notes/mujoco_pico_sim_teleop.md`
- VR 硬件与软件安装：`docs/source/getting_started/vr_teleop_setup.md`
- 全身遥操教程：`docs/source/tutorials/vr_wholebody_teleop.md`
- A3 真机上电顺序：`docs/a3_training2sim2deploy.md` 第 6 节

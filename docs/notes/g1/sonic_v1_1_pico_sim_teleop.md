# SONIC v1.1 + PICO 全身遥操——MuJoCo 仿真完整指南

> **目标**：用 PICO VR 头显 + 脚踝 Tracker，在 MuJoCo 仿真环境中对 Unitree G1 做全身遥操。  
> 本文使用 **SONIC v1.1 checkpoint**（机器人朝向归一化、腕部增强训练版本）。  
> 请先在仿真里练熟急停和模式切换，再考虑真机。

---

## 目录

1. [系统架构](#1-系统架构)
2. [硬件准备](#2-硬件准备)
3. [环境确认](#3-环境确认)
4. [第一步：连接 PICO](#4-第一步连接-pico)
5. [第二步：启动三个终端](#5-第二步启动三个终端)
6. [第三步：执行遥操](#6-第三步执行遥操)
7. [急停方法](#7-急停方法)
8. [常见问题排查](#8-常见问题排查)
9. [真机部署（仿真熟练后）](#9-真机部署仿真熟练后)

---

## 1. 系统架构

```
PICO 头显 + 手柄 + 脚踝 Tracker
          │
          │ 追踪数据（XRoboToolkit，同 Wi-Fi 局域网）
          ▼
本机 PC Service（RoboticsService）
          │
          │ ZMQ 消息
          ▼
终端 3：pico_manager_thread_server.py
          │
          │ ZMQ 消息（SMPL 全身姿态 / 规划指令）
          ▼
终端 2：gear_sonic_deploy / g1_deploy_onnx_ref（C++ SONIC v1.1 策略）
          │
          │ DDS / 仿真桥接
          ▼
终端 1：run_sim_loop.py（MuJoCo 仿真窗口，G1 机器人）
```

三个终端必须**同时运行**，缺任何一个遥操都不会工作。

---

## 2. 硬件准备

| 设备 | 要求 |
|---|---|
| PICO 头显 | PICO 4 或 PICO 4 Pro |
| 手柄 | 左右各一，随头显配备 |
| 脚踝 Tracker | 2 个 PICO Motion Tracker，绑左右脚踝 |
| 着装 | **紧身裤或运动紧身裤**（宽松裤子会遮挡 Tracker 视觉追踪，导致脚部数据失真） |
| 网络 | 电脑与 PICO **同一局域网**，推荐私有路由器 5 GHz，延迟目标 < 10 ms |

---

## 3. 环境确认

### 3.1 本机 IP（填进 PICO 的就是这个）

本机 `hostname -I` 的实际输出（以当前机器为准，会变）：

```text
192.168.110.104 198.18.0.1
```

| 地址 | 用途 |
|---|---|
| **192.168.110.104**（`wlp8s0`） | 局域网地址。PICO 的 XRoboToolkit 里填这个 |
| 198.18.0.1 | 代理/虚拟网卡地址，不要填进 PICO |

需要重新确认时再跑：

```bash
hostname -I
```

### 3.2 本机环境（已确认）

在本机执行 `source ~/.bashrc` 后：

| 检查项 | 本机实际值 |
|---|---|
| `$TensorRT_ROOT` | `/usr` |
| `nvcc` | `/usr/local/cuda-12.9/bin/nvcc` |

需要重新确认时再跑：

```bash
source ~/.bashrc
echo $TensorRT_ROOT    # /usr
which nvcc             # /usr/local/cuda-12.9/bin/nvcc
```

如果输出为空，手动补上：

```bash
export TensorRT_ROOT=/usr
export PATH=/usr/local/cuda-12.9/bin:$HOME/.local/bin:$PATH
```

### 3.3 确认模型文件存在

```bash
REPO="/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl"

ls -lh \
  "$REPO/gear_sonic_deploy/target/release/g1_deploy_onnx_ref" \
  "$REPO/gear_sonic_deploy/policy/sonic_v1_1/model_encoder.onnx" \
  "$REPO/gear_sonic_deploy/policy/sonic_v1_1/model_decoder.onnx" \
  "$REPO/gear_sonic_deploy/policy/sonic_v1_1/observation_config.yaml" \
  "$REPO/gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx"
```

期望结果：

```
5.3M  g1_deploy_onnx_ref        ← C++ 可执行文件
48M   model_encoder.onnx
143M  model_decoder.onnx
2.5K  observation_config.yaml
739M  planner_sonic.onnx
```

文件不存在则返回上一级 README 重新下载模型。

### 3.4 确认 Python 环境

```bash
REPO="/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl"
"$REPO/.venv_teleop/bin/python" -c \
  "import xrobotoolkit_sdk, mujoco, pinocchio, zmq; print('OK mujoco', mujoco.__version__)"
# 期望输出：OK mujoco 3.15.0
```

---

## 4. 第一步：连接 PICO

> 必须在启动终端之前完成，否则终端 3 拿不到数据。

### 4.1 启动 PC Service（电脑端）

PC Service 已安装在 `/opt/apps/roboticsservice/`，进程名是 `RoboticsServiceProcess`。确认它在运行：

```bash
pgrep -af RoboticsServiceProcess
```

没有输出就启动服务（会弹出 PC Service 窗口）：

```bash
bash /opt/apps/roboticsservice/runService.sh
```

应用菜单里的名称是 **XRoboToolkit-PC-Service**，对应脚本是 `/opt/apps/roboticsservice/run3D.sh`。遥操只需要上面的 `runService.sh`。

### 4.2 配置头显端（戴上 PICO 操作）

**a. 确认 Wi-Fi**  
PICO 与电脑必须在同一网络。在 PICO 控制中心底部确认已连 Wi-Fi。

**b. 打开 XRoboToolkit App**  
在 PICO 应用库（Unknown 分类）找到 XRoboToolkit 并打开。

**c. 填入电脑 IP**  
在 `PC Service` 一栏点 `Enter`，输入本机局域网地址：

```text
192.168.110.104
```

不要填 `198.18.0.1`。确认后状态栏显示 **WORKING** 表示连接成功。若已填写，点 **Reconnect**。

**d. 勾选追踪项目**（必须全选，否则脚部数据不发送）：

- Tracking 栏：`Head` ✓、`Controller` ✓
- Data/Control 栏：点 **Send** 按钮
- Pico Motion Tracker 栏：选 **Full body**

**e. 关闭 Safeguard**  
Settings → Developer → Safeguard → **关闭**

### 4.3 绑定脚踝 Tracker 并标定

**绑定方式**：Tracker 绑在脚踝外侧，**指示灯朝上**，不要松动。

**配对 Tracker**：
1. 进入 PICO Wi-Fi 设置界面，点击头显图标上方的小圆圈（Motion Tracker 图标）。
2. 点击每个 Tracker 旁的 `i` 图标，先取消所有配对（Unpair All）。
3. 点右上角 `Pair`，同时长按两个 Tracker 顶部按钮约 **6 秒**，直到指示灯红蓝交替闪烁。

**标定**：
1. 戴上头显，两个 Tracker 已绑好。
2. 点击 **Calibrate**，按提示完成两段动作：
   - Sequence 1：**站直、手柄垂放两侧、静止不动**约 3 秒。
   - Sequence 2：**低头盯着两个脚踝 Tracker**，直到头显摄像头识别到它们。
3. 标定成功后，把头显推到额头上（保持朝前），让摄像头持续看见脚踝。

---

## 5. 第二步：启动三个终端

> **顺序很重要**：先终端 1，等 MuJoCo 窗口出现，再开终端 2，等 `Init done`，最后开终端 3。

---

### 终端 1 — MuJoCo 仿真

```bash
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl
source .venv_teleop/bin/activate
python gear_sonic/scripts/run_sim_loop.py --robot g1
```

**预期结果**：弹出 MuJoCo 窗口，G1 机器人出现在场景中（可能初始悬空，属正常）。

---

### 终端 2 — C++ 策略（SONIC v1.1）

等终端 1 的 MuJoCo 窗口稳定后，打开新终端：

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
  sim
```

参数说明：

| 参数 | 含义 |
|---|---|
| `--robot g1` | 指定 G1（与终端 1/3 一致） |
| `--cp policy/sonic_v1_1/model` | 指定 SONIC v1.1 模型前缀（会自动拼 `_encoder.onnx` / `_decoder.onnx`） |
| `--obs-config policy/sonic_v1_1/observation_config.yaml` | 使用 v1.1 的观测配置（不能混用 release） |
| `--input-type zmq_manager` | 接收 PICO 遥操数据（通过 ZMQ） |
| `--motor-kp-scale 4,10=1.5` | 左脚踝 pitch（索引 4）和右脚踝 pitch（索引 10）Kp 增益 ×1.5 |
| `--motor-kd-scale 4,10=1.5` | 同上，Kd 增益 ×1.5 |
| `sim` | 仿真模式（自动禁用 CRC 校验，走 loopback 接口） |

脚本会询问是否确认部署，输入 `Y` 回车。

**预期结果**：日志滚动，最终出现 **`Init Done`**；路径须含 `policy/sonic_v1_1/`。

---

### 终端 3 — PICO 数据流

等终端 2 出现 `Init Done` 后，打开新终端：

```bash
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl
source .venv_teleop/bin/activate

python gear_sonic/scripts/pico_manager_thread_server.py \
  --robot g1 \
  --manager \
  --vis_vr3pt \
  --vis_smpl
```

**预期结果**：

- 再弹出两个可视化窗口：
  - `VR3PT` 窗口：G1 骨架，根据你的头和手运动。
  - `SMPL` 窗口：全身人体模型，跟随你整体动作。
- 终端输出每 5 秒打印一次延迟统计。

如果没有弹窗但也没有报错，说明收到数据了但窗口在后台，用 Alt+Tab 找一下。  
如果报错 `Connection refused` 或数据全为 0，返回第 4 节检查 XRoboToolkit 连接。

---

## 6. 第三步：执行遥操

### 6.1 标定姿态（每次启动必须做）

**在按任何手柄按键之前**，先摆好标定姿态：

```
站直，目视正前方
双脚并拢（脚尖朝前）
上臂自然下垂，紧贴身体两侧
前臂水平向前，与上臂成 90°（手肘成 L 型）
手心朝向身体中线（相对而立）
```

对照终端 3 的 VR3PT 可视化窗口，确认窗口里的 G1 骨架与你的姿态一致后，再继续。

### 6.2 完整操作流程

```
步骤 1：摆好标定姿态
          │
          ▼
步骤 2：手柄同时按 A + B + X + Y
         ↓ 策略启动，执行全身一次性标定（CALIB_FULL）
         → 进入 PLANNER 模式（机器人站立待命）

步骤 3：在 MuJoCo 窗口点一下，按键盘 9
         → 机器人落到地面（如果还悬空的话）

步骤 4：对齐姿态
         → 看仿真里机器人现在的站姿，让自己身体大致匹配
          （不需要完全一样，但手臂位置尽量对齐）

步骤 5：手柄按 A + X
         ↓ 进入 POSE 模式（全身镜像遥操正式开始）
         → 你的动作实时映射到仿真机器人

步骤 6：自然行走、转身、伸手
         → 不要刻意放慢动作，自然最好

步骤 7：结束时，按 A + X 退回 PLANNER
         然后按 A + B + X + Y 停止策略（进入 OFF）
```

### 6.3 Planner 模式下的手柄控制

当处于 **PLANNER** 模式（没有进入 POSE），可以用手柄遥控机器人行走：

| 手柄输入 | 功能 |
|---|---|
| 左摇杆前后 | 前进 / 后退 |
| 左摇杆左右 | 横向移动 |
| 右摇杆左右 | 原地转向 |
| A + B | 切换到下一个步态 |
| X + Y | 切换到上一个步态 |

步态 ID 对照：

| ID | 步态 |
|---|---|
| 0 | 站立（Idle，默认） |
| 1 | 慢走 |
| 2 | 正常走 |
| 3 | 跑步 |
| 4 | 蹲姿 |
| 5 | 双膝跪地 |

### 6.4 切换模式时的安全要点

> ⚠️ **这是最重要的安全规则**，请认真阅读：

**切入 POSE 模式之前**，必须先让自己的姿态接近机器人当前姿态，再按 A+X。  

- 错误示范：机器人站直，你蹲着，直接按 A+X → 机器人瞬间跟着蹲，仿真里猛动，真机上会摔倒。
- 正确做法：看仿真窗口 → 站直对齐 → 再按 A+X。

**VR_3PT 模式**（左摇杆按下进入，手部单独追踪）同样需要先对齐手臂再进入。  
进入 VR_3PT 时会触发一次手腕重标定（CALIB），手臂姿态不对，手部控制会乱。

---

## 7. 急停方法

**发现任何异常，立刻急停：**

| 方法 | 操作 | 说明 |
|---|---|---|
| PICO 手柄（首选） | 同时按 **A + B + X + Y** | 任何模式下均有效 |
| 键盘 | 在终端 2 里按 **`O`** | 键盘必须聚焦在终端 2 |

急停后策略进入 OFF 状态，机器人停止响应。  
下次重新开始，从标定姿态 → A+B+X+Y 重新启动。

---

## 8. 常见问题排查

### 问题 1：终端 3 报错或没有可视化窗口

**检查清单**：

```bash
# 1. 确认 PC Service 在运行；没有输出就先启动
pgrep -af RoboticsServiceProcess
bash /opt/apps/roboticsservice/runService.sh

# 2. 确认防火墙没有拦截
sudo ufw status    # 若 active，考虑暂时关闭或放行相关端口
```

- 头显端：XRoboToolkit 里状态是否 **WORKING**？
- 头显端：是否点了 **Send**？
- 头显端：Pico Motion Tracker 是否选了 **Full body**？
- 网络：头显和电脑是否在同一个网段？

### 问题 2：脚部 Tracker 数据乱跳或机器人容易摔倒

- 重新标定 Tracker（头显里 Calibrate）
- 确认 Tracker 没有松动，指示灯朝上
- 换上紧身裤，确保 Tracker 可见
- 头显戴在额头时，保持朝前，摄像头需要能看见脚踝

### 问题 3：动作卡顿，有明显延迟或一顿一顿

终端 2 如果输出 `High ZMQ latency: XXms`，说明网络延迟过高。

- 换 5 GHz 私有路由器
- 减少 Wi-Fi 干扰（关闭其他 Wi-Fi 设备）
- 检查路由器与电脑、头显的距离

延迟参考：< 10 ms 良好，10~30 ms 可接受，> 30 ms 会出现明显抖动。

### 问题 4：终端 2 报 `TensorRT_ROOT not set` 或找不到 libnvinfer

```bash
source ~/.bashrc
# 或手动（deb 安装的 TensorRT 10.13 在系统路径）：
export TensorRT_ROOT=/usr
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/local/cuda-12.9/lib64:$LD_LIBRARY_PATH
```

然后重新运行终端 2 的命令。

### 问题 5：部署脚本找不到模型文件

确认路径中有空格时加了引号。  
例如，直接在 `gear_sonic_deploy/` 目录下运行 `./deploy.sh`，不要 `cd` 到其他目录再用相对路径。

### 问题 6：标定后机器人姿态偏移

- 标定时姿态没有保持 1~2 秒不动
- 再按 A+B+X+Y 重新标定，这次保持标定姿态更长时间
- 遥操一段时间后 PICO 追踪漂移也会导致偏移，随时可以重新标定（A+B+X+Y 停止 → 标定 → A+B+X+Y 重新启动）

---

## 9. 真机部署（仿真熟练后）

> **前置条件**：
> - 已在仿真里成功完成遥操，熟练掌握急停
> - G1 机器人已开机，通过以太网或 Wi-Fi 连接到同一台电脑
> - 安全员就位，随时准备在终端 2 按 `O`
> - **必须先关闭 `run_sim_loop.py`**（sim 与 real 不能同时运行）

终端 1（不需要再开 MuJoCo）

**终端 2 — 真机策略**

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
```

> `real` 会自动检测 192.168.123.x 网段的机器人接口。  
> 若自动检测失败，直接传机器人 IP：将最后的 `real` 替换为 `192.168.123.161`（具体以机器人实际 IP 为准）。

**终端 3 — PICO 数据流（与仿真相同，可去掉可视化节省资源）**

```bash
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl
source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --robot g1 --manager
```

遥操操作流程与仿真完全一致，急停方法也相同。

---

## 附录：关键文件位置

```
仓库根目录：
/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl/
│
├── .venv_teleop/                          # 遥操 Python 环境
│
├── gear_sonic/scripts/
│   ├── run_sim_loop.py                    # 终端 1：MuJoCo 仿真
│   └── pico_manager_thread_server.py      # 终端 3：PICO 数据流
│
├── gear_sonic_deploy/
│   ├── deploy.sh                          # 终端 2：部署入口脚本
│   ├── scripts/setup_env.sh               # 环境变量（TensorRT、CUDA、ONNX）
│   ├── policy/sonic_v1_1/
│   │   ├── model_encoder.onnx             # SONIC v1.1 编码器
│   │   ├── model_decoder.onnx             # SONIC v1.1 解码器
│   │   └── observation_config.yaml        # 观测配置（必须与模型配套）
│   ├── planner/target_vel/V2/
│   │   └── planner_sonic.onnx             # 运动规划器
│   └── target/release/
│       └── g1_deploy_onnx_ref             # 编译好的 C++ 可执行文件
│
└── docs/notes/
    ├── mujoco_pico_sim_teleop.md          # 通用仿真遥操指南
    └── sonic_v1_1_pico_sim_teleop.md      # 本文
```

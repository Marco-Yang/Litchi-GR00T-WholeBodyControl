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

### 1.1 电脑端

1. 打开 **XRoboToolkit / RoboticsService**（PC Service）。
2. 查看本机 IP：

```bash
hostname -I
```

### 1.2 头显端

1. PICO 与电脑连同一 Wi‑Fi。
2. 打开 **XRoboToolkit** App。
3. 在 **PC Service** 填入电脑 IP；状态显示 **WORKING** 即连接成功。
4. 勾选：
   - Tracking：`Head`、`Controller`
   - Data/Control：点 **Send**
   - Pico Motion Tracker：**Full body**
5. 开发者选项中关闭 **Safeguard**。
6. 配对并标定两个脚部 Tracker（站直标定 → 低头看脚直到识别）。

> 终端 3 若无可视化窗口或报连不上设备，优先排查本节。

---

## 2. 三个终端启动

### 终端 1 — MuJoCo 仿真

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
source .venv_teleop/bin/activate
python gear_sonic/scripts/run_sim_loop.py
```

应弹出 MuJoCo 窗口；机器人初始可能悬空，属正常。

### 终端 2 — C++ 策略（等终端 1 就绪后再开）

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
source scripts/setup_env.sh
./deploy.sh --input-type zmq_manager sim
```

等到日志出现 **`Init done`** 再继续。

使用 **SONIC v1.1** 时：

```bash
./deploy.sh \
  --cp policy/sonic_v1_1/model \
  --obs-config policy/sonic_v1_1/observation_config.yaml \
  --input-type zmq_manager \
  sim
```

### 终端 3 — PICO 数据流

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --manager --vis_vr3pt --vis_smpl
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
5. **仿真熟练并会急停后**，再考虑真机；真机时必须先关掉 `run_sim_loop.py`（sim 与 real 不能同时跑）。

真机启动示例（勿与终端 1 同时开）：

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
source scripts/setup_env.sh
./deploy.sh --input-type zmq_manager real
```

---

## 7. 相关官方文档

- VR 硬件与软件安装：`docs/source/getting_started/vr_teleop_setup.md`
- 全身遥操教程：`docs/source/tutorials/vr_wholebody_teleop.md`
- 遥操最佳实践：`docs/source/user_guide/teleoperation.md`
- Quick Start（含 sim2sim）：`docs/source/getting_started/quickstart.md`

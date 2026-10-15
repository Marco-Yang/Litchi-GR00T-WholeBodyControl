# A3 / A3 Ultra：MuJoCo 仿真 + PICO 全身遥操操作指南

本文说明如何在本机用 **PICO VR** 对 **MuJoCo 中的 AgiBot A3 / A3 Ultra** 做 SONIC 全身遥操。流程和 G1 的 [mujoco_pico_sim_teleop.md](../g1/mujoco_pico_sim_teleop.md) 一一对应（三个终端、同一套手柄操作），只是机器人侧、权重和中间多了一步“人体 → A3 关节”的重定向。
命令以 `--robot a3_ultra` 为例；`--robot a3` 的用法完全相同（仿真换成 A3 的 T2D5 模型）。

> 范围：**只在 MuJoCo 里**。真机见本文第 8 节，那里列了没验证过的部分。

---

## 架构概览

```
PICO 头显/手柄/脚部 Tracker
        │  (XRoboToolkit，同 Wi-Fi)
        ▼
本机 PC Service + pico_manager_thread_server.py          ← 终端 3
        │  ZMQ :5556   topic "pose"（人体 SMPL 姿态）、topic "command"（手柄按键）
        ▼
deploy.sh --robot a3_ultra  ……（一条命令起下面四样）      ← 终端 2
   ├─ a3_teleop_retarget_node.py   人体 SMPL → GMR 逆运动学 → A3 的 29 个关节参考   （ZMQ :5560 "a3_ref"）
   ├─ a3_ta_bridge                 ZMQ → ROS2 /ta/whole_body_command
   └─ a3_deploy_onnx_ref           C++ 策略（035 的 a3_fast 编码器，ONNX），输出 29 个关节的目标
        │  ROS2  /body_drive/*   （ROS_DOMAIN_ID=87，ROS_LOCALHOST_ONLY=1）
        ▼
run_sim_loop.py --robot a3_ultra（MuJoCo 窗口）           ← 终端 1
```

需要同时开 **3 个终端**。**三个终端必须用同一个机器人。**

### 和 G1 流程的区别

| | G1 | A3 / A3 Ultra |
|---|---|---|
| 策略输入 | Pico 的人体 SMPL 直接进 SMPL 编码器 | 035 **没有** SMPL 编码器：先用 GMR 把人体姿态重定向成 A3 的 29 个关节，再交给 `a3_fast` 编码器（`--encoder a3-fast`，默认） |
| 机器人接口 | Unitree DDS | ROS2 `/body_drive/*`（同一个 C++ 二进制连仿真也连真机） |
| 策略权重 | G1 的 SONIC | 下载的 035（`a3` 与 `a3_ultra` 共用）；只驱动 29 个身体关节，A3 Ultra 的两个头关节固定在默认姿态，手指不驱动 |
| 模式 | PLANNER / POSE / VR_3PT | **没有 planner**：`A+B+X+Y` 进入遥操后，要按 **A+X 进 POSE** 才会持续推姿态流 |
| 额外环境 | `.venv_teleop` | 还要 conda 环境 `a3_deploy_verify`（mujoco + ROS2 Python 绑定，一次性准备，见第 0 节）和 `gmr`（重定向节点，`deploy.sh` 自己会调用） |
| `--vis_vr3pt` | G1 网格叠加 | 对 A3 不可用，会自动关闭 |

---

## 0. 前置检查

仓库根目录（下文记为 `$REPO`）：

```text
/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl
```

路径里有空格：下文所有 `cd` 都加了引号，这三条命令这样写已经验证可用。其它自己写的脚本如果把路径拆开了，用无空格软链接：`ln -s "$REPO" /tmp/gwbc`，然后在 `/tmp/gwbc` 下执行。

**一次性准备**（只做一次）：

```bash
cd "$REPO/gear_sonic_deploy"
./scripts/setup_a3_sim_env.sh        # 给 conda 环境 a3_deploy_verify 装 mujoco + joint_msgs
```

确认文件存在：

| 项 | 路径 |
|---|---|
| 遥操环境 | `$REPO/.venv_teleop` |
| 仿真 conda 环境 | `~/miniconda3/envs/a3_deploy_verify`（`run_sim_loop.py` 会自动切过去，日志里有 `re-running under .../a3_deploy_verify/...`） |
| 重定向 conda 环境 | `~/miniconda3/envs/gmr`（GMR + mink + mujoco） |
| 部署包 | `$REPO/gear_sonic_deploy/dist/a3_deploy_x86_64/`（`a3_deploy_onnx_ref`、`a3_ta_bridge`） |
| 策略 | `…/dist/a3_deploy_x86_64/models/model_step_200000_a3_fast.onnx`、`…_g1.onnx` |

策略和部署包缺了：`deploy.sh --robot a3_ultra` 首次运行会自动下载 / 构建；也可以手动下载：

```bash
python download_from_hf.py --robot a3_ultra --component onnx      # 部署/仿真用
```

硬件（与 G1 相同）：PICO 4 / 4 Pro + 双手柄；2× 脚踝 Motion Tracker；穿紧身裤；电脑与 PICO 在同一低延迟 Wi-Fi。

---

## 1. 连接 PICO 与电脑

和 G1 完全一样，详见 [G1 文档第 1 节](../g1/mujoco_pico_sim_teleop.md)。要点：

1. 电脑端启动 PC Service：`pgrep -af RoboticsServiceProcess`，没有输出就 `bash /opt/apps/roboticsservice/runService.sh`。
2. 头显里打开 **XRoboToolkit**，**PC Service** 填电脑的**局域网地址**（`ip -4 -br addr show <无线网卡>`，不要填 Tailscale / 虚拟网卡的地址），状态 **WORKING**。
3. 勾选 Tracking：`Head`、`Controller`；Data/Control：点 **Send**；Pico Motion Tracker：**Full body**。
4. 开发者选项里关闭 **Safeguard**；配对并标定两个脚部 Tracker。

> 终端 3 一直打印 `waiting for body data...`，就是这一节没做完（头显没在发数据），见第 6 节。

---

## 2. 三个终端启动

按顺序：终端 1 → 终端 2（等 `Init Done`）→ 终端 3。

### 终端 1 — MuJoCo 仿真

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
source .venv_teleop/bin/activate
python gear_sonic/scripts/run_sim_loop.py --robot a3_ultra
```

应弹出 MuJoCo 窗口。机器人一开始被**弹性绳吊离地面**（骨盆约 1.35 m），和 G1 一样。窗口里：**`9`** 放到地面，`7` / `8` 缩短 / 加长绳子。

终端里会看到：

```text
[a3-sim] robot=a3_ultra scene=…/a3_ultra_t3d0.xml dt=0.002 ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1
[a3-sim] elastic band ON (pelvis_z=1.35 m); MuJoCo window: press 9 to release onto the ground, 7/8 shorten/lengthen
[a3-sim] t=   8.3s rtf=0.83 cmd_rx={'leg': 0, 'waist': 0, 'arm': 0, 'neck': 0} band=True
```

`cmd_rx` 是仿真收到的指令计数，终端 2 起来之后它会开始增长；全是 0 说明部署程序没连上（见第 6 节）。`rtf` 是仿真相对实时的速度，这台机器上约 0.83。

### 终端 2 — C++ 策略 + 重定向链（等终端 1 就绪后再开）

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
source scripts/setup_env.sh
./deploy.sh --robot a3_ultra --input-type zmq_manager sim --encoder a3-fast
```

`--robot` 写在 `sim` / `real` 之前。`--input-type zmq_manager` 会在部署程序旁边自动起重定向节点和 `a3_ta_bridge`，不用另开终端。

**等到日志出现 `Init Done`** 再继续（此时部署已自动进入 PD 站立，机器人还吊在绳上），随后会打印：

```text
[a3] encoder=a3-fast (TELEOP source 'a3_fast' preselected and locked in the deploy)
[a3-chain] encoder=a3-fast: Pico tcp://localhost:5556 'pose'  ->  A3 joint references :5560  ->  a3_ta_bridge  ->  /ta/whole_body_command
…
[a3_backend] topic readiness after startup gate:
  waist        ready=yes samples=3        ← 6 行都应是 ready=yes；ready=no 说明没连上仿真
…
Init Done
[pico-cmd] PD stand ready — press A+B+X+Y to start teleop, then A+X for POSE; MuJoCo window key 9 releases the elastic band
```

两条日志可以忽略：`setup_env.sh` 里的 `ROS2 not found in common locations`（那是 G1 部署环境的提示）和 `xrobotoolkit_sdk not found, skip for now`（重定向环境的提示）。

`--encoder` 在启动时就选好并**锁定**：

| `--encoder` | 说明 |
|---|---|
| `a3-fast`（默认） | 下载的 035 `a3_fast` 编码器，延迟约 220 ms，**完整可用** |
| `g1` | 同一条链路但延迟约 940 ms，只作对照 |
| `smpl` | 需要 A3 的 SMPL 编码器 ONNX；**目前没有**（035 只有 `g1` 与 `a3_fast`），只有站立桩用来验证数据通路 |

### 终端 3 — PICO 数据流

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --robot a3_ultra --manager --num_frames_to_send 10
```

- `--robot a3_ultra`：决定用哪个机器人的 IK / FK 模型，要和另外两个终端一致。
- `--manager`：同时发 `pose`（姿态流）和 `command`（手柄按键）两个 topic，部署程序靠后者响应 A+B+X+Y / A+X。
- `--num_frames_to_send 10`：每条消息带 **10 帧**窗口（默认 5）。A3 链路统一用 10；`--encoder smpl` 要求 10 帧窗口，用默认的 5 会不够。
- 头显数据到了之前它会一直打印 `waiting for body data...`，这是正常的等待状态。

---

## 3. 第一次遥操流程

1. 终端 2 等到 **`Init Done`**，机器人在 MuJoCo 里**吊着**做 PD 站立。
2. **标定姿态**：站直、双脚并拢、看向前方；上臂贴身，前臂向前弯约 90°（肘成 L），掌心向内。
3. 手柄同时按 **A + B + X + Y** → 部署进入 teleop（终端 2 有 `[mode] pd_stand -> teleop`；手柄触发时前面还会有一行 `[pico-cmd] … start -> teleop`）。
4. 在 MuJoCo 窗口点一下，按 **`9`** 把机器人放到地面。
5. 身体摆成与仿真机器人接近后，按 **A + X** → **POSE**（全身镜像）。A3 没有 planner，**POSE 才会持续推姿态流**。
6. 自然走路、伸手；过度犹豫反而更容易失稳。
7. 再按 **A + B + X + Y** → 停止（passive）。

终端 2 进入 teleop 后会连续打印这几类行，用来判断链路是否健康：

```text
[mode] pd_stand -> teleop
[teleop] first whole_body_command received: stamp_ns=…
[a3-retarget] sent=2017 solve_ms mean=2.17 p95=2.54 max=3.58     ← 重定向每帧约 2 ms
[a3_ta_bridge] forwarded=2100 rejected=0 last_age_ms=2.5 …      ← rejected 应一直是 0
[frames] n=7004 mode=teleop … sync=ok … dt=20.02ms …            ← 策略 50 Hz，sync=ok
```

### 急停

| 方式 | 操作 |
|---|---|
| PICO 手柄 | **A + B + X + Y** |
| 终端 2 键盘 | **`P`**（passive），退出 `Q` |

### 模式切换安全

进 **POSE** 之前务必先让身体与机器人当前姿态大致对齐，错位时切换会让仿真里的机器人猛然跟姿；真机上会非常危险。

---

## 4. 终端 2 的键盘备份

平时按手柄即可。部署程序需要**真终端**（TTY），管道输入无效。

| 键 | 作用 |
|---|---|
| `S` / `T` / `P` | PD 站立 / 遥操 / 被动 |
| `M` | 动作回放模式（20 条内置参考动作，`[` `]` 切换，`R` 重播） |
| `1` / `2` / `3` | 选源。经 `deploy.sh` 启动时已锁定，按别的源会 `rejected` |
| `Q` | 退出 |

---

## 5. 没有头显时的联调

用 Pico 模拟器代替终端 3（和它占用同一个 ZMQ 端口，**两个不能同时开**）：

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
PYTHONPATH=. /home/user/miniconda3/envs/gmr/bin/python gear_sonic/scripts/pico_pose_zmq_replay.py \
    --clip synthetic:wave --stand-seconds 6 --loop
```

它不发手柄按键，所以在终端 2 里按 **`T`** 代替 A+B+X+Y；放绳子在 MuJoCo 窗口按 `9`。机器人应站稳并重复右臂挥手。

无人值守时（脚本里跑，按不了 `9`），仿真加环境变量让弹性绳在 N 秒仿真时间后自动松开：

```bash
A3_SIM_BAND_RELEASE_S=75 python gear_sonic/scripts/run_sim_loop.py --robot a3_ultra
```

---

## 6. 常见问题

| 现象 | 处理 |
|---|---|
| 终端 3 一直 `waiting for body data...` | 头显没在发数据：查 PC Service 是否在运行、头显状态是否 WORKING、是否点了 **Send**、是否同网段（第 1 节） |
| 终端 2 里 `ready=no samples=0`，仿真里 `cmd_rx` 全是 0 | 部署程序没连上仿真：**先开终端 1，再开终端 2**；先确认终端 1 的进程还在（它退出之后，终端 2 会一直 `ready=no`，要重开终端 1）；两边的 `ROS_DOMAIN_ID` 都应是 87 |
| 机器人一开始悬空 | 正常，是弹性绳；窗口里按 `9` |
| 机器人摔倒 | 仿真会提示 `robot fell … resetting (band re-enabled, key 9 to release)` 并把绳子重新挂上；先回到 PD 站立再来 |
| 动作一顿一顿 | Wi-Fi 延迟；终端 2 若出现 `High ZMQ latency`，换近距离私有路由 |
| 重定向后手臂乱 / 跟不上 | 先看终端 2 的 `rejected` 是否一直是 0、`solve_ms` 是否约 2 ms；再确认标定姿态做对了 |
| 端口被占用（5556 / 5560） | 上一轮的进程没退干净：`ss -ltnp \| grep -E ':(5556\|5560)'`，按 PID 单独结束，不要 `pkill -f` |
| `--vis_vr3pt` 没有效果 | 它是 G1 的网格叠加，对 A3 不可用，已自动关闭 |
| 路径含空格报错 | 本文三条命令都已验证；自己的脚本用 `/tmp/gwbc` 软链接 |
| 仿真比实时慢（`rtf` < 1） | 机器负载高（MuJoCo 窗口 + 策略 + 重定向都在跑），不影响流程，时间只是变慢 |

---

## 7. 这份文档验证过什么

2026-10-15 在本机按上面的三条命令（路径带空格，原样）跑过一遍：

- 终端 1 → 终端 2 → 终端 3 依次启动；终端 2 打印 `ready=yes`（6 个话题）和 `Init Done`，仿真 `cmd_rx` 从 0 持续增长。
- 终端 3 用真实命令能起来并连上本机 PC Service，因为当时没有头显在发数据，停在 `waiting for body data...`。
- 之后把终端 3 换成 Pico 模拟器（`synthetic:wave`），终端 2 按 `T`：`mode=teleop`，重定向每帧平均约 2.2 ms（p95 约 2.5–2.8 ms），`a3_ta_bridge` 转发 2600+ 帧、`rejected=0`，策略 50 Hz 周期稳定在约 20 ms、`sync=ok`，仿真里机器人没有摔倒。

**没有验证**：

- **真实头显**的全链路（标定姿态、A+B+X+Y、A+X 的实机手感）。这部分请你戴上头显按第 3 节走一遍；有问题记到 `docs/workflow/sonic_a3_ultra_worklog.md`。
- **真机**。
- `--encoder smpl`（没有 A3 的 SMPL 编码器权重）。

---

## 8. 真机

仿真熟练并会急停之后才上真机，真机时必须先关掉 `run_sim_loop.py`（sim 与 real 不能同时跑）。

```bash
# 操作端 PC（连头显的那台）：Pico 服务 + 操作端链路（a3-fast 经 ROS2 把 /ta/whole_body_command 发给机器人，要与机器人同一 ROS_DOMAIN_ID）
python gear_sonic/scripts/pico_manager_thread_server.py --robot a3_ultra --manager --num_frames_to_send 10
gear_sonic_deploy/scripts/a3_operator_chain.sh --encoder a3-fast
# 机器人上：--no-chain = 不在本机起操作端链路
./deploy.sh --robot a3_ultra --input-type zmq_manager --no-chain real --encoder a3-fast
```

真机侧用你站点自己的 `A3_TRANSPORT` / `ROS_DOMAIN_ID` / `A3_ROBOT_ENV`（见 `docs/a3_training2sim2deploy.md` 第 6 节）。**上真机前务必读 `docs/workflow/a3_integration_plan.md` 12.7 的未验证项。**

---

## 9. 相关文档

- G1 版本（连接头显、手柄说明更完整）：[../g1/mujoco_pico_sim_teleop.md](../g1/mujoco_pico_sim_teleop.md)
- A3 与 A3 Ultra 的参数对照：[a3_vs_a3_ultra_params.md](a3_vs_a3_ultra_params.md)
- 链路设计、编码器选择、各项验证数据：`docs/workflow/a3_integration_plan.md` 第 9、12、14、15 节
- 工作日志：`docs/workflow/sonic_a3_ultra_worklog.md`
- VR 硬件与软件安装 / 全身遥操教程：`docs/source/getting_started/vr_teleop_setup.md`、`docs/source/tutorials/vr_wholebody_teleop.md`

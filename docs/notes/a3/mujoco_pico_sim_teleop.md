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

另有一种**经 ZMQ 中继**的变体（部署进程不直连机器人，真机上部署在本机、中继在 MDU 上的路线在仿真里的等价物）：多开一个中继终端，部署程序加 `--teleop-input zmq --robot-link zmq --relay-host 127.0.0.1`，共 **4 个终端**，见第 5b 节。其余步骤和按键完全一样。

### 和 G1 流程的区别

| | G1 | A3 / A3 Ultra |
|---|---|---|
| 策略输入 | Pico 的人体 SMPL 直接进 SMPL 编码器 | 035 **没有** SMPL 编码器：先用 GMR 把人体姿态重定向成 A3 的 29 个关节，再交给 `a3_fast` 编码器（`--encoder a3-fast`，默认） |
| 机器人接口 | Unitree DDS | ROS2 `/body_drive/*`（同一个 C++ 二进制连仿真也连真机） |
| 策略权重 | G1 的 SONIC | 下载的 035（`a3` 与 `a3_ultra` 共用）；只驱动 29 个身体关节，A3 Ultra 的两个头关节固定在默认姿态，手指不驱动 |
| 模式 | PLANNER / POSE / VR_3PT | **没有 planner 流**，但按键对齐 G1：`A+B+X+Y` 第一次 = 开环 PD 站立（硬起来）→ `A+X` = 跟随 → 再按 `A+X` = **policy stand**（`a3_fast` 继续跑，参考收到默认站姿，对应 G1 的 POSE→PLANNER）→ 再 `A+X` 又跟随 → `A+B+X+Y` = passive。没有 VR_3PT |
| 额外环境 | `.venv_teleop` | 还要 conda 环境 `a3_deploy_verify`（mujoco + ROS2 Python 绑定，一次性准备，见第 0 节）和 `gmr`（重定向节点，`deploy.sh` 自己会调用） |
| `--vis_vr3pt` / `--vis_smpl` | 调试窗口：三点（头 + 双手）追踪、SMPL 关节小球，外加叠一个 G1 网格 | 只是可视化，不影响控制。G1 网格对 A3 不可用，脚本发现 `--robot` 不是 g1 会自动关掉（打印 `G1 mesh overlay disabled`）；其余部分理论上还能显示，**但我没在 A3 上实际打开验证过**，所以默认命令里没带 |

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

按顺序：终端 1 → 终端 2 → 终端 3。部署程序启动后是 **IDLE，什么指令都不发**；按手柄 A+B+X+Y 才让机器人硬起来。

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

等 6 路话题都 `ready=yes`（说明连上了仿真）。此时仿真里的 `cmd_rx` 仍是 0：**部署程序是 IDLE，没发任何指令**（和真机一样，不会自动站立）。按 A+B+X+Y（第 3 节）之后，终端 2 才会打印 `Init Done`（3 秒的 PD 站立过渡走完）。启动日志：

```text
[a3] encoder=a3-fast (TELEOP source 'a3_fast' preselected and locked in the deploy)
[a3-chain] encoder=a3-fast: Pico tcp://localhost:5556 'pose'  ->  A3 joint references :5560  ->  a3_ta_bridge  ->  /ta/whole_body_command
…
[a3_backend] topic readiness after startup gate:
  waist        ready=yes samples=3        ← 6 行都应是 ready=yes；ready=no 说明没连上仿真
…
[pico-cmd] listening tcp://localhost:5556 topic 'command' (A+B+X+Y: harden to PD stand / disable, A+X: follow <-> standing pose; same flow as G1)
✓ startup IDLE, no commands sent: press S (PD stand) with the robot supported, or Pico A+B+X+Y
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
python gear_sonic/scripts/pico_manager_thread_server.py --robot a3_ultra --manager --num_frames_to_send 10 --vis_vr3pt --vis_smpl
```

- `--robot a3_ultra`：决定用哪个机器人的 IK / FK 模型，要和另外两个终端一致。
- `--manager`：同时发 `pose`（姿态流）和 `command`（手柄按键）两个 topic，部署程序靠后者响应 A+B+X+Y / A+X。
- `--num_frames_to_send 10`：每条消息带 **10 帧**窗口（默认 5）。A3 链路统一用 10；`--encoder smpl` 要求 10 帧窗口，用默认的 5 会不够。
- 头显数据到了之前它会一直打印 `waiting for body data...`，这是正常的等待状态。
- **可选** `--vis_vr3pt --vis_smpl`（G1 教程里带的两个参数）：只弹调试窗口看三点追踪和 SMPL 关节，需要显示器和 PyVista（本机 `.venv_teleop` 里有 0.49），不影响控制。我没放进默认命令：窗口里的 G1 网格对 A3 没有意义（会被自动关掉），而且这个窗口在 A3 上没验证过。追踪不对劲、想确认脚部 Tracker 时再加。

---

## 3. 第一次遥操流程

和 G1 的手柄流程一致，部署程序一开始是 **IDLE**：

1. **标定姿态**：站直、双脚并拢、看向前方；上臂贴身，前臂向前弯约 90°（肘成 L），掌心向内。
2. 手柄同时按 **A + B + X + Y** → 机器人**硬起来**，3 秒内从当前姿态过渡到**默认站立姿态**（PD 站立，Litchi 真机的 PD 站立增益）。终端 2：`[mode] idle -> pd_stand`、`[pico-cmd] A+B+X+Y -> PD stand …`，走完后打印 `Init Done`。此时机器人还吊在绳上。
3. 在 MuJoCo 窗口点一下，按 **`9`** 把机器人放到地面，看它是否稳稳站住。
4. 身体摆成与仿真机器人接近后，按 **A + X** → **遥操跟随**（`[mode] pd_stand -> teleop`；A3 没有 planner，跟随才会持续推姿态流）。
5. 自然走路、伸手；过度犹豫反而更容易失稳。
6. 再按 **A + X** → **policy stand**（模式仍是 `teleop`，终端 2：`[pico-cmd] A+X -> policy stand`、`policy_stand=1`）。`a3_fast` 不停，参考切到默认站姿（和 G1 回 PLANNER 一样靠策略把身体拉正）。可以再按 A+X 重新跟随。键盘 **`S`** 仍是开环 PD 站立（会关策略），和手柄这条不是同一条路。
7. 按 **A + B + X + Y** → **失能**（passive，kp = kd = 0，机器人放松）。真机和 Litchi 一样用零增益，不是阻尼指令。

**Pico 管理脚本在你按第二次 A+B+X+Y（停止）之后会自己退出**（和 G1 一样）；想再启动，重新运行终端 3 的命令。

**失联时的行为**（和 G1“安全检查失败就停止、不自动恢复”一致）：

| 情况 | 结果 | 怎么恢复 |
|---|---|---|
| 机器人状态流中断 / 过期（超过 `max_frame_age_ms` = 50 ms） | 终端 2 `[link] robot state lost or stale … -> passive`，机器人失能 | 链路好了以后再按 A+B+X+Y；Pico 脚本这时还以为自己在运行，**第一次按键等于它的“停止”（对已失能的机器人无效），再按一次才是“启动”** |
| Pico / 重定向的姿态流断了超过 1 秒（`teleop.stale_fallback_ms`） | 终端 2 `[teleop] reference lost … policy stand`，`a3_fast` 继续用默认站姿参考（不再切开环 PD 站立） | 姿态流恢复后按 A+X 重新跟随（G1 的 planner 过期回 IDLE 站立） |
| 经中继（第 2b 节）时中继守卫锁存了失联保护 | `[link] the relay is damping … -> passive` | 再按 A+B+X+Y（本机会换一个新会话，中继在姿态接近时重新接管） |

终端 2 进入 teleop 后会连续打印这几类行，用来判断链路是否健康：

```text
[mode] pd_stand -> teleop
[teleop] first whole_body_command received: stamp_ns=…
[a3-retarget] sent=2017 solve_ms mean=2.17 p95=2.54 max=3.58     ← 重定向每帧约 2 ms
[a3_ta_bridge] forwarded=2100 rejected=0 last_age_ms=2.5 …      ← rejected 应一直是 0
[frames] n=7004 mode=teleop … sync=ok … dt=20.02ms …            ← 策略 50 Hz，sync=ok
```

**增益（Kp / Kd）和 Litchi 真机部署一致**：

| 状态 | 增益 |
|---|---|
| 默认站立姿态（仅 A+B+X+Y 第一次硬起来，或键盘 `S`） | Litchi 的 PD 站立增益（`a3_pd_stand_kps/kds`），3 秒从当前姿态线性过渡（开环，不看 IMU） |
| 跟随（A+X） | Litchi 真机的策略增益档 `policy_control.profile: a3_024`（`a3_024_kps/kds`，只有 4 个踝关节是 60 / 1.2），头 kp 40 / kd 2 |
| policy stand（遥操中再按 A+X） | 仍用策略增益；参考 = 默认站姿 tokenizer，经 `a3_fast`（对应 G1 PLANNER） |
| 失能（A+B+X+Y 再按、键盘 `P`、链路故障） | kp = kd = tau_ff = 0（Litchi 的 passive） |
| 经中继、中继锁存失联保护后的阻尼 | kp → 0，kd → `a3_024` 策略 kd 表（`a3_zmq_relay --gain-profile`） |

我们的 A3 参数文件和 Litchi 的逐字节相同。第一次硬起来的“默认站立姿态”是开环 PD 站立；遥操中再按 A+X 的“站着等”是 policy stand（策略 + 默认站姿参考）。

**要留意的单帧尖峰**：在 MuJoCo 里机器人吊在绳上时开跟随，部署程序发出的 `q_des` 会出现**单帧尖峰**（例如左膝一帧到 2.5 rad 再回到 0.24 rad，髋俯仰一帧 −1.78 rad，右肩进跟随时一帧 1.66 rad 的步长）。直连和经中继发的指令完全一样，中继的上限（7 / 6 rad）不会拦它们；真机吊着开跟随时要留意，先在仿真里看熟。

**肩关节大范围转动（和 G1 不一样）**：A3 这条链路多了一步 GMR 逆解（人体 SMPL → 29 关节参考），`a3_fast` 再去跟踪这些关节角。G1 是 Pico SMPL 直接进策略编码器，没有这一步逆解。A3 Ultra 肩滚转向内只有约 5°（`a3_ultra_t3d0.xml`：左 −0.087…2.618 rad，右镜像）；两臂绕肩转一圈时，肩会顶在限位上，够不着的角度进肘和腕，反向把肩转回来后肘/腕仍粘在顶限时的那套解上，左右会不对称。看起来像“不跟随”，其实是逆解贴限后热启动粘住了。G1 转肩时少跟一点、始终在限位内，是策略近似，不是靠事后限幅。已经卡住时再按一次 **A+X** 开新的 IK 段；日常遥操不要把肩拧过内侧那 5°。更完整的对照见 `docs/workflow/a3_integration_plan.md` 18.15。

### 急停

| 方式 | 操作 |
|---|---|
| PICO 手柄 | **A + B + X + Y**（机器人已硬起来时 = 失能 passive） |
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

加 `--commands` 它还会替手柄发按键（和真的 Pico 管理脚本同一套 `command` 消息）：**在运行模拟器的这个终端里按 `y` = A+B+X+Y，按 `x` = A+X**，然后和第 3 节一样走；放绳子在 MuJoCo 窗口按 `9`。机器人应站稳并重复右臂挥手。

```bash
PYTHONPATH=. /home/user/miniconda3/envs/gmr/bin/python gear_sonic/scripts/pico_pose_zmq_replay.py \
    --clip synthetic:wave --stand-seconds 6 --loop --commands
```

（不用 `--commands` 时它不发按键，可以在终端 2 里按 `S` / `T` / `P` 代替 A+B+X+Y / A+X / 失能。）

正常遥操**一定要在 MuJoCo 窗口里按 `9` 松开弹性绳**（和 G1 一样）；`7` / `8` 调绳长。只有脚本 / 无窗口的自动化测试按不了键，仿真才有一个测试专用的环境变量 `A3_SIM_BAND_RELEASE_S=<秒>`（到点自动松绳），平时不要用。

---

## 5b. 经 ZMQ 中继的变体（部署进程不直连机器人）

真机上部署进程放在本机、机器人上跑 `a3_zmq_relay` 的路线（`pico_real_teleop.md` 第 4.6 节）在仿真里是这样的：多一个**中继终端**，部署程序加两个参数。中继接仿真（ROS 2，私有域 87），部署程序只和中继说 ZMQ。**四个终端，按顺序：**

```bash
# 终端 1 — MuJoCo 仿真（不变）
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
source .venv_teleop/bin/activate
python gear_sonic/scripts/run_sim_loop.py --robot a3_ultra

# 终端 2 — 中继（新）
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
scripts/a3_relay_sim.sh --bind 127.0.0.1

# 终端 3 — 部署程序（比第 2 节多 --teleop-input zmq --robot-link zmq --relay-host 127.0.0.1）
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
source scripts/setup_env.sh
./deploy.sh --robot a3_ultra --input-type zmq_manager sim --encoder a3-fast \
    --teleop-input zmq --robot-link zmq --relay-host 127.0.0.1

# 终端 4 — Pico 数据流（真头显）或模拟器（二选一，不能同时开）
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --robot a3_ultra --manager --num_frames_to_send 10
#   或：PYTHONPATH=. /home/user/miniconda3/envs/gmr/bin/python gear_sonic/scripts/pico_pose_zmq_replay.py \
#         --clip synthetic:wave --stand-seconds 6 --loop --commands        # 终端里按 y = A+B+X+Y，x = A+X
```

`--teleop-input zmq` 是必须的：这条路线的部署进程里没有 AimRT，`/ta` 订阅不存在，重定向节点的 `a3_ref` 流由部署程序直接读（`--no-bridge`，`deploy.sh` 自己处理）。然后按第 3 节走（A+B+X+Y → 按 9 → A+X → A+X → A+B+X+Y）。

**要观察什么**：

| 终端 | 看什么 |
|---|---|
| 中继（终端 2） | 每秒一行状态：`DISARMED`（还没收到合格指令，**什么都不发布**）→ `ACTIVE` → `acc=` 增长、`rej=0`；退出时打印整次最大的偏差 / 步长（哪个关节、前后值） |
| 部署（终端 3） | `[a3_zmq] relay mode …`、`[a3_zmq] robot enabled: new session …`（每次重新使能都换一个新会话）、`Init Done`；链路故障时 `[link] … -> passive` |
| 仿真（终端 1） | 在你按 A+B+X+Y 之前 `cmd_rx` 一直是 0 |

**可以试的故障**（都在仿真里，吊在绳上做更安全）：
- 在另一个终端 `kill -STOP $(pgrep -x a3_deploy_onnx_ref)`，约 2 秒后 `kill -CONT $(pgrep -x a3_deploy_onnx_ref)`（挂起部署程序）：中继先 `HOLD`（保持最后一条指令约 0.2 秒）再 `DAMP`（kp 在 0.3 秒内降到 0，kd 用 Litchi 真机的 `a3_024` 策略 kd 表），部署程序打印 `[link] … -> passive`；再按 A+B+X+Y 重新硬起来（新会话，中继在姿态接近时重新接管）。
- 关掉终端 4（Pico 数据流）：约 1 秒后部署程序进入 policy stand（`[teleop] reference lost … policy stand`），策略继续站着。
- Ctrl-C 中继：它先阻尼 2 秒再退出；第二次 Ctrl-C 立即退出。

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
| 加了 `--vis_vr3pt` 看不到机器人网格 | 网格是 G1 的，对 A3 没有意义，已自动关闭；三点追踪轴和 SMPL 关节（`--vis_smpl`）才是在 A3 上还有用的部分 |
| 路径含空格报错 | 本文三条命令都已验证；自己的脚本用 `/tmp/gwbc` 软链接 |
| 仿真比实时慢（`rtf` < 1） | 机器负载高（MuJoCo 窗口 + 策略 + 重定向都在跑），不影响流程，时间只是变慢 |

---

## 7. 这份文档验证过什么

**2026-10-16，新流程（G1 式按键）+ 经中继的变体**，MuJoCo 无窗口、用 Pico 模拟器（`--commands`）替手柄发和真管理脚本同一套的 `command` 消息：

- A+B+X+Y 之前仿真 `cmd_rx` = 0（部署程序是 IDLE，什么都不发）；A+B+X+Y → 中继 `DISARMED → ACTIVE`、`Init Done`；松绳后 PD 站立骨盆高度 1.073 m、倾角 0.4°。
- A+X 跟随（挥手，倾角 ≤ 2.0°）→ A+X 回站立（1.073 m）→ A+X 再跟随。
- 挂起部署程序 1.5 s：中继 `HOLD → DAMP`，部署程序 `[link] … -> passive`；再按两次 A+B+X+Y（Pico 脚本的“停止”和“启动”）→ 新会话，中继 `DAMP → ACTIVE`，PD 站立恢复。
- 杀掉姿态流进程：约 1 s 后 `[teleop] reference lost … back to PD stand`，机器人继续站着；键盘 `P` → passive，机器人倒下重置。
- C++ 单元测试 31 个、Python 测试 21 个全过。详细记录见 `docs/workflow/a3_integration_plan.md` 第 18.13、18.14 节。

**2026-10-15，旧流程（直连）：**

2026-10-15 在本机按上面的三条命令（路径带空格，原样）跑过一遍：

- 终端 1 → 终端 2 → 终端 3 依次启动；终端 2 打印 `ready=yes`（6 个话题）；A+B+X+Y 之前仿真 `cmd_rx` 一直是 0，之后打印 `Init Done`、`cmd_rx` 持续增长。
- 终端 3 用真实命令能起来并连上本机 PC Service，因为当时没有头显在发数据，停在 `waiting for body data...`。
- 之后把终端 3 换成 Pico 模拟器（`synthetic:wave`，旧版流程按 `T`）：`mode=teleop`，重定向每帧平均约 2.2 ms（p95 约 2.5–2.8 ms），`a3_ta_bridge` 转发 2600+ 帧、`rejected=0`，策略 50 Hz 周期稳定在约 20 ms、`sync=ok`，仿真里机器人没有摔倒。

**没有验证**：

- **真实头显**的全链路（标定姿态、A+B+X+Y、A+X 的实机手感）。上面的新流程只用 Pico 模拟器发过同样的 `command` 消息；请你戴上头显按第 3 节走一遍，有问题记到 `docs/workflow/sonic_a3_ultra_worklog.md`。
- 中继变体下，经中继的长时间稳定性（只跑了几分钟，5 次中继跑里有 1 次松绳后摔倒，1 次直连基线没摔，样本不足以下结论）。
- **真机**。
- `--encoder smpl`（没有 A3 的 SMPL 编码器权重）。

---

## 8. 真机

仿真熟练并会急停之后才上真机，真机时必须先关掉 `run_sim_loop.py`（sim 与 real 不能同时跑）。**真机有自己的教程：[pico_real_teleop.md](pico_real_teleop.md)**，里面是两台机器（操作端 PC + 机器人 MDU 上的 aarch64 包）的启动方式、`sm_config` / preflight / probe、IDLE 启动后按 `S` 的第一次遥操流程、急停与失效行为。和仿真的几个关键区别：

- 真机和仿真的 Pico 流程一样：部署程序以 **IDLE** 启动、不发指令，A+B+X+Y 让机器人硬起来。只有不带 `--input-type zmq_manager` 的脚本化仿真运行才会自动 PD 站立（`--auto-pd-stand`）。
- 机器人侧要用 `--zmq-host <操作端PC的IP>`，否则收不到手柄的 A+B+X+Y。
- 仿真里松绳是在 MuJoCo 窗口按 `9`；真机是缓慢放下吊带。
- **部署进程放在本机、机器人上只跑中继**的路线见 [pico_real_teleop.md](pico_real_teleop.md) 第 4.6 节；MDU 上要用的 aarch64 包需要 Docker 交叉编译，sysroot 和 aarch64 ONNX Runtime 已下载，Docker 还没装（0.2 节有命令）。厂商还没同意在机器人上运行会发布关节指令的程序（见 `real_mdu_body_drive.md`），在那之前中继只用于仿真。

---

## 9. 相关文档

- G1 版本（连接头显、手柄说明更完整）：[../g1/mujoco_pico_sim_teleop.md](../g1/mujoco_pico_sim_teleop.md)
- A3 与 A3 Ultra 的参数对照：[a3_vs_a3_ultra_params.md](a3_vs_a3_ultra_params.md)
- 链路设计、编码器选择、各项验证数据：`docs/workflow/a3_integration_plan.md` 第 9、12、14、15 节
- 工作日志：`docs/workflow/sonic_a3_ultra_worklog.md`
- VR 硬件与软件安装 / 全身遥操教程：`docs/source/getting_started/vr_teleop_setup.md`、`docs/source/tutorials/vr_wholebody_teleop.md`

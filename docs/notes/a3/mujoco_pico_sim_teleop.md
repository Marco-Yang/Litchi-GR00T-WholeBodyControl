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

# 终端 3 — 部署程序（比第 2 节多 --teleop-input zmq --telecom zmq --relay-host 127.0.0.1；旧写法 --robot-link zmq 仍可用）
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
source scripts/setup_env.sh
./deploy.sh --robot a3_ultra --input-type zmq_manager sim --encoder a3-fast \
    --teleop-input zmq --telecom zmq --relay-host 127.0.0.1

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

## 5c. 腿 / 脚的跟踪：`track` 模式下录一段真人数据，离线调参（2026-10-17）

> **录制 / 回放的完整流程现在单独写在 [mujoco_pico_record_replay.md](mujoco_pico_record_replay.md)（以那份为准，包含更正过的回放步骤）；本节保留当时的背景和结果。**

**背景。** `--lower-body track`（默认）让腿跟随操作者。实时重定向（GMR 的 `smplx_to_a3_ultra` 表）没有约束脚的朝向：一个**完全静止**的站立者，踝 roll 在约 3 s 内就漂到关节极限（±0.349 rad）；离线对比 8 / 24 段行走片段，踝关节与离线参考几乎不相关（相关系数 −0.25…+0.29），踝 roll 在 10–13% 的帧里贴着极限。给脚加一个旋转权重（`A3_FOOT_ROT_WEIGHT`，**现在默认 8**，`0` = 原表）后：24 段片段上踝 RMSE 0.207 → 0.124 rad，平均相关 0.65 → 0.88，踝 roll 贴极限 9.9% → 6.5%；静止站立不再漂移（长时间 −0.03 / +0.05 rad）。权重 ≥ 20 会让部分片段发散（腿 RMSE 0.43+），所以取 8（5–12 等效）。MuJoCo 里同一段 SMPL 行走片段、同样的流程，原表 vs 权重 8：踝 roll 参考贴极限 26% → 4% 的帧，髋 yaw 相关 0.69 / 0.71 → 0.85 / 0.85，膝 0.51 / 0.62 → 0.65 / 0.70，脚在任务空间里“参考 → 实际执行”的残差 13.5 / 10.5 → 10.9 / 8.7 cm；两者都走了约 30 m 不摔。**没有改善的：** 策略执行的抬脚高度只有参考的 50–75%（参考 24 cm，执行 12–19 cm），延迟约 40 ms——这是策略的限制，不是重定向。

**为什么要录你自己的数据。** 上面都是动捕 SMPL；你的 Pico（头显 + 脚踝追踪器）的数据和它不一样。录下原始的 `pose` / `command` 流以后，同一段动作可以离线用不同设置反复打分，也可以不用你再次示范就重放进仿真闭环。

### 终端（5 个，顺序固定）

先在仓库根目录生成这次运行专用的 dump 配置（只改 `dump_ticks` / `dump_path`，包里的配置不动；每次运行用不同的名字，**不要重复执行旧的命令**）：

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
ls pico_leg1.pkl dump_sim_leg1.bin sim_leg1.csv 2>&1 | head -3        # 应当都是“没有那个文件”；有的话把 leg1 换成 leg2
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_make_dump_cfg.py --dump "$PWD/dump_sim_leg1.bin"
```

```bash
# 终端 1 — MuJoCo（窗口里 9 放到地面）
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
source .venv_teleop/bin/activate
A3_SIM_LOG_CSV="$PWD/sim_leg1.csv" python gear_sonic/scripts/run_sim_loop.py --robot a3_ultra

# 终端 2 — 策略 + 重定向链（等终端 1 就绪；默认 track，脚旋转权重 8）
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
source scripts/setup_env.sh
A3_RUNTIME_CFG="$PWD/dist/a3_deploy_x86_64/config/a3_runtime_config.dump.yaml" ./deploy.sh --robot a3_ultra --input-type zmq_manager sim --encoder a3-fast

# 终端 3 — 录制器（只订阅，不影响别的；必须在终端 4 之前开）
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_pose_recorder.py --out pico_leg1.pkl --seconds 900

# 终端 4 — Pico 数据流
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --robot a3_ultra --manager --num_frames_to_send 10
```
终端 3 每 2 s 打印一行 `pose windows … (~50/s)`，约 50/s 才说明头显和追踪器在发。

### 严格示范（你的动作；总共约 4 分钟）

**站姿：** 双脚平行、与髋同宽（脚间约 15 cm）、脚尖朝前、膝盖微松；上臂贴身，前臂向前，肘约 60–75°，手掌向内。**全程视线向前，头不要转，上半身不要主动摆。** 追踪器按平时的流程校准好。

0. 终端 1 的窗口按 **9**（放到地面），等机器人站稳。手柄 **A+B+X+Y**（PD 站立）→ 等终端 2 打印 `Init Done`，再站稳 5 s。
1. 手柄 **A+X**（进跟随）。**一动不动站 15 s**。
2. **重心转移（脚不离地）：** 把全部重量压到左脚，保持 2 s，再压到右脚，保持 2 s；共 5 个来回（20 s）。
3. **抬左脚：** 左脚垂直向上抬约 15 cm（脚底与地面平行，脚尖不要下垂），停 1 s，放下；每次共 4 s，做 5 次；然后站定 5 s。
4. **抬右脚：** 同上，右脚 5 次；站定 5 s。
5. **原地踏步：** 左右脚交替，每步 1 s，脚抬高约 15 cm，共 20 步（20 s）；站定 5 s。
6. **前后迈步：** 左脚向前迈约 30 cm，停 1 s，收回；做 3 次；右脚同样 3 次；每次 5 s；站定 5 s。
7. **原地走：** 自然步速，原地走 20 步；站定 10 s。
8. 手柄 **A+B+X+Y**（失能）。

**每一步开始前先喊出编号**（例如对着麦克风 / 看着表默念），并**把你实际做的和上面不一样的地方记下来**（比如“第 6 步右脚只迈了一半”）。机器人在仿真里摔倒不要紧：窗口会自己复位（按 9 重新放下），继续做下一步，但**把摔倒发生在哪一步记下来**。

### 收尾和分析（按顺序）

```bash
# 先停录制器（Ctrl-C 会保存），再停终端 4、2、1
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
ls -l --time-style=full-iso pico_leg1.pkl dump_sim_leg1.bin sim_leg1.csv       # 三个文件的时间应当都是刚才

# 1) 录下的真人数据：原表 vs 脚权重 8（同一段动作；约 1 分钟一次，要 gmr 环境）
PYTHONPATH=. ~/miniconda3/envs/gmr/bin/python gear_sonic_deploy/scripts/a3_eval_pose_recording.py pico_leg1.pkl --foot-rot-weight 0 | tee eval_leg1_stock.txt
PYTHONPATH=. ~/miniconda3/envs/gmr/bin/python gear_sonic_deploy/scripts/a3_eval_pose_recording.py pico_leg1.pkl --foot-rot-weight 8 | tee eval_leg1_w8.txt

# 2) 仿真闭环（这次实际执行的）：参考腿 vs 实测腿，和任务空间里脚的执行误差
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_analyze_teleop_run.py dump_sim_leg1.bin | tee analysis_sim_leg1.txt
```
把 `eval_leg1_stock.txt`、`eval_leg1_w8.txt`、`analysis_sim_leg1.txt` 和你记下的“哪里和示范不一样 / 哪一步摔倒”发给我。`eval_*` 里看这几项：**STILL** 行（静止时踝 roll 应接近 0，不贴 ±0.349）、**ALL frames** 行（贴极限的帧占比）、**foot while moving** 行（人脚 → 机器人脚的残差 cm 和 R²，R² 越接近 1 越好）。

### 之后不需要你再示范就能做的事

录好的 `pico_leg1.pkl` 可以重放进仿真，换任意设置做闭环对比（`a3_pose_replay_raw.py` 代替终端 4，**两者不能同时开**）：

```bash
# 终端 4 换成：
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_pose_replay_raw.py pico_leg1.pkl            # 默认连 command 一起重放（A+B+X+Y / A+X 在原来的时刻）
# 终端 2 换设置：A3_FOOT_ROT_WEIGHT=0（原表）/ 8 / 12 …  ./deploy.sh …
```

### 第一次录制（`pico_leg1`）的结果和三个现象（2026-10-18）

- **文件：** 录制器当时没有过滤话题，`manager_status`（管理脚本每秒发几万条、每条 1.3 KB 的状态流）把 139 s 的会话录成了 7.8 GB；真正有用的只有 6953 个 `pose` 窗口（82 MB）。已改成只保存 `pose` / `command` / `planner`（`--all` 才保存全部），并把这次的录制瘦身成 `pico_leg1_slim.pkl`（同样的数据，82 MB）。7.8 GB 的 `pico_leg1.pkl` 确认 slim 文件在以后可以自己删。**第二次运行复用了 `leg1` 这个名字，把第一次运行的 `dump_sim_leg1.bin` 和 `sim_leg1.csv`（机器人执行数据）覆盖了**，Pico 录制没有被覆盖，所以执行数据可以用重放补回（下面）。每次运行换一个名字（`leg2`、`leg3` …）。
- **录制里没有 A+B+X+Y / A+X 的 `command` 消息**（只有开头一条空消息和结尾两条停止消息）：第一次是用部署终端的键盘 `S` / `T` 进的跟随。重放时用 `a3_pose_replay_raw.py` 的 `--command-fifo` 发 `y` / `x`。
- **真实 Pico + 脚踝追踪器数据上的打分（离线，无机器人）：** 静止时踝 roll：原表 +0.252 / −0.113 rad，**40% 的静止帧贴着极限**，所有帧里 44.5% 贴极限；**脚权重 8：0% 贴极限**（静止均值 −0.043 / +0.192）。人脚 → 机器人脚（参考，FK）：残差 3.9 / 3.7 cm，R² 0.91 / 0.92，延迟 0 ms（原表和权重 8 一样：脚的**位置**一直跟得很好，坏的是脚的**朝向**）。髋 yaw 范围 −0.36…+0.33 rad（正常）。
- **现象 1，左脚抬一会机器人就收回：** 参考没有收回。你每次抬左脚，人脚保持的时间和参考保持的时间几乎相同（第一次长抬 7.36 s vs 6.84 s；1.80 vs 1.74；1.34 vs 1.28；1.40 vs 1.38 …），峰值高度参考略低（33 → 30 cm）。所以收脚发生在**策略执行**这一层（策略站不住单脚就把脚放下），不是重定向。要靠重放得到执行数据才能确认和量化。
- **现象 2，向正前方迈左脚 40 cm 却向左前方：** 脚权重对步的方向**没有影响**（权重 0 / 2 / 5 / 8 / 12 的结果一样）。录制里 40 cm 的大步，左脚：人的方向 +11° / +6°（人脚本身已经偏左 7.8 / 4.7 cm），参考 +13° / **+21°**（第二步侧向 17.3 cm，人只有 4.7 cm）；右脚：人 +25…+28°（跨向左边 18–23 cm），参考只有 +6…+8°（侧向被压缩，横向增益约 0.5 / 0.73）。这些大步里参考的脚还比人脚高 12–15 cm（人 2.6–5.7 cm，参考 15.8–17.9 cm）：重定向把骨盆固定在恒定高度，长步幅时人的骨盆会下降，机器人的 IK 只好把脚抬高并向外摆。**试过降低骨盆位置权重（100 → 30 / 10 / 3）：与离线参考的腿误差从 0.145 反而升到 0.164 / 0.173 / 0.182，所以不是修法，已撤销。**
- **现象 3，向前走时脚底打滑：** 仿真的摩擦系数不低（脚的碰撞体 1.5，地面默认 1，接触取较大者）。打分表里的 p90（这次重放 `rb0` 是 24.7 cm）把“脚一直没离地”的十几秒粘成一次站立：短于 1 秒的单脚支撑，滑动中位数 1.0 cm、p90 2.3 cm；长于 4 秒的那些占了总滑动路程的 77%。真正看得见的是 36 个 1 秒窗口，脚离地不到 2 cm、水平净位移 ≥ 8 cm。其中 8 次是另一只人脚抬着（支撑脚不该动）：支撑脚平均滑 15.5 cm，参考脚在同一秒也移了 15.2 cm，参考离地只有 0.8 cm，方向主要向后。另外 21 次两只人脚的高度差不到 4 cm（录制 90–120 s 的走路，人脚自己几乎没有离地）：脚在地上挪 8–51 cm，参考脚挪得一样多甚至更多，参考离地平均 4.6 cm。所以长尾是参考把步子贴在地面上发出去，策略跟着滑，不是摩擦没托住一只本来该钉住的脚。

### 仿真里“静止时躯干一直左右轻微晃”（2026-10-18，用户报告）

用用户自己那次回放的仿真日志（`sim_leg1r.csv`，664 s）逐 10 s 窗口测的骨盆 / 躯干（FK）左右倾角：

| 阶段 | 骨盆 roll 标准差 | 躯干 roll 标准差 / 峰峰 | 腰 roll 关节标准差 | 0.5–2.5 Hz 占方差 |
|---|---|---|---|---|
| PD 站立（无策略，t 40–400 s） | 0.000–0.003° | 0.000–0.003° / 0.01° | 0.000° | — |
| 策略站立，参考恒定（t 450–660 s，手臂几乎不动 <0.03 rad） | **0.06–0.08°** | **0.53–0.60° / 约 2°** | **0.52–0.57°** | **0.92–0.99** |

- **PD 站立纹丝不动，策略一接管就开始晃，而且是单一频率（约 1.1 Hz）、幅度稳定、持续 200 s 以上。** 晃的是**腰 roll 关节本身**（骨盆几乎不动），所以看起来是“躯干左右摇”。
- 此时参考是恒定的（腰 roll 参考标准差 0.000°），策略输出的腰 roll 指令却在 ±5°（标准差 2.9°，峰峰 11°）、1.04–1.16 Hz 摆动，腰关节只跟出 ±1°（腰 roll Kp 50 / Kd 2 / 动作尺度 0.23，摩擦损耗 0.69 N·m）。→ **策略自己的闭环极限环**，不是参考噪声、不是重定向、不是脚。
- 真机上同一件事也在：腰 pitch 指令标准差 2.3° 在 1.05 Hz，实测只有 0.39°；原始动作的主频也是 1.05 Hz（`real_run1`）。仿真和真机一致，说明不是仿真的问题。
- **可以做但没做的（都需要改部署、重编译）：** ① 对腰 roll（和 pitch）指令做低通（约 0.5 Hz 截止，会给操作者的躯干倾斜带来约 0.3 s 的滞后）；② 腰 roll 的 Kd 加倍（偏离训练的增益）；③ 训练侧加动作变化率惩罚（根治，在训练你的 SMPL 编码器时一起做）。幅度只有约 ±1°，也可以接受不处理。

**是不是训练 / 部署的 Kp、Kd 不一致？（2026-10-18）** 不是：

- 部署策略路径的增益 = 训练增益。`a3_policy_parameters.hpp` 的 `a3_024_kps/kds`（腰 85/50/50 Kp、3/2/2 Kd，踝 60/1.2，其余同 `a3.py`）= Litchi 的 `a3.py`；发布的 035 checkpoint 的 `config.yaml` 写 `robot: type: a3_024`。`docs/robot_parameters/A3_Ultra` 的厂家表是另一张表（髋 200/7、膝 360/13、踝 80/3、腰 roll/pitch 80/3、腰 yaw 200/7、肩 90/4…，比训练表高 1.3–2.5 倍），部署没有用它。
- PD 站立（`y`）用的是**另一张**更硬的表（`a3_pd_stand_kps/kds`，腰 400/500/500、Kd 4，腿 1500/400/300/2000/500/500），不是策略增益。PD 站立纹丝不动、策略接管才晃，所以问题出在策略阶段，不是这张表。
- 动作尺度 `0.25 × effort / Kp` 是策略合同的一部分：把增益换成另一张表而不重训，等于同一个动作输出换了 1.3–2.5 倍的力矩幅度。

仿真里把“换表”做成实验（`A3_SIM_GAINS_JSON`，只替换策略阶段的增益，动作尺度不变；静止、策略站立、51 s 窗口；单次运行，重复一次的差别约 10%）：

| 情形 | 躯干 roll 标准差 / 峰峰 | 躯干 pitch 标准差 | 腰 roll 指令标准差 |
|---|---|---|---|
| 基线（训练增益） | 0.245° / 1.22° | 0.456° | 0.89° |
| 对照：覆盖成同样的训练增益（证明开关本身是空操作） | 0.270° / 1.15° | 0.485° | 0.96° |
| 厂家表，全部关节 | **0.120° / 0.45°** | 0.317° | 0.42° |
| 厂家表，只换腰 | 0.172° / 0.66° | 0.526°（没变） | 0.45° |
| 关节摩擦损耗 ×0.2（`A3_SIM_FRICTION_SCALE`） | 0.062° / 0.31° | 0.095° | 0.33° |
| 关节摩擦损耗 ×0 | 0.071° / 0.25° | 0.019° | — |

- 换成厂家表后站立是稳的，躯干 roll 幅度约减半；摩擦调小效果更大。两条线指向同一个机制：幅度大致由“关节摩擦 / Kp”决定的粘滑极限环（**这是和数据吻合的解释，没有单独证实**）。
- **不建议因此改真机增益：** ① 只测了静止站立，走路 / 抬脚没测；② 增益高 1.3–2.5 倍而策略和动作尺度不变，真机上没有验证过；③ 改它要改 `a3_024_kps/kds` 并重编译，PD 站立的表不动。要验证应先在仿真里用同样的开关放一遍重放的走路动作，看跟踪和摔倒。
- 仿真关节摩擦 `frictionloss`（N·m，腰 roll 0.69、踝 pitch 1.4、膝 2.43）和训练里 PhysX 的摩擦系数（无量纲，随机化 ×[1,3]）单位不同，不能直接比大小；所以“仿真摩擦偏大”只是怀疑，没有量。
- **一个我自己犯的错：** 第一次做这个实验时，覆盖把 PD 站立阶段的增益也替换了（腰 400/500/500 → 厂家 200/80/80，或训练的 85/50/50），机器人在松开吊带后几秒内摔倒，我据此以为“厂家表会让机器人站不住”。用“覆盖成训练增益”做对照时同样摔倒，才发现是 PD 站立被改弱了。现在的开关只替换 Kp 等于该关节策略 Kp 的指令（JSON 第三个数字），对照实验与基线一致，才有上面的表。

仿真里的实验开关（默认全关，只在 `a3_sim.py` 里生效）：

| 环境变量 | 作用 |
|---|---|
| `A3_SIM_FRICTION_SCALE=0.2` | 所有受驱动关节的 `frictionloss` 乘该系数 |
| `A3_SIM_DAMPING_SCALE=…` | 同上，对 `damping` |
| `A3_SIM_GAINS_JSON=文件` | `{"waist_roll_joint": [kp, kd, 策略kp]}`：替换部署发来的 Kp / Kd；第三个数字存在时只替换 Kp 等于它的指令（即只改策略阶段，不碰 PD 站立） |

### 换增益对“走路”有没有用？整段重放 `pico_leg1_slim.pkl`（2026-10-18）

同一段录制（136 s 跟随，按帧编号重放，见 `mujoco_pico_record_replay.md` 5b），只改仿真里策略阶段的增益（`A3_SIM_GAINS_JSON`，动作尺度不变）。7 次运行都没有摔倒（最低骨盆高度 0.98–1.02 m）：

| 设置 | 脚位置残差 L / R | 躯干 roll 标准差 | 基座角速度峰值 | 打滑 p90 / 打滑 >3 cm 的占比 |
|---|---|---|---|---|
| 训练增益（`rb0`、`rb1`、`rb2`，三次） | 6.7/6.8、6.5/6.7、6.5/6.7 cm | 3.55°、3.56°、3.54° | 2.08、1.95、2.02 rad/s | 18.6、12.9、25.6 cm / 33、28、32% |
| 腿增益 ×0.8（Kp、Kd 同乘） | 7.3 / 6.9 | **2.97°** | 1.76 | 34.2 cm / 45% |
| 腿增益 ×1.25 | 6.6 / 6.9 | 4.22° | 2.91 | 12.1 cm / 33% |
| 厂家表，只换腿 | 7.7 / 7.0 | 4.42° | 3.62 | 20.4 cm / 45% |
| 厂家表，全部关节 | 7.9 / 7.5 | 4.94° | 3.29 | 12.2 cm / 31% |

- **没有任何一组增益比训练增益跟得更好。** 腿更硬 → 躯干 roll 摆动和基座角速度峰值单调变大（噪声只有 0.01°，所以这个趋势是真的），脚的残差没变好；更软 → roll 摆动小一点（2.97°），但脚残差略差。打滑 p90 在三次相同设置的重复里就有 12.9–25.6 cm 的散布，所以 ×0.8 的 34.2 cm 看不出和基线的区别；打滑 >3 cm 的占比（基线 28–33%，×0.8 和厂家只换腿 45%）有一点偏多的迹象，证据弱。→ **训练增益已经是这个策略的合适点，不建议改增益。** 静止站立时厂家表能把摆动减半（上一节），走路时反而更差，两件事不能互相代替。
- **现象 1（抬着的脚被收回）不是增益问题，而是策略的行为：** 整段里有 20 次持续 ≥ 1 s 的抬脚；其中两次长时间保持（右脚 35.2 s 起 7.6 s，左脚 18.0 s 起 7.7 s）。右脚那次，参考抬高 21 cm 然后慢慢降到 10–14 cm，**策略在参考抬高还有约 14 cm 时（39.5 s）自己把脚放下**，之后 3 s 里参考一直在 10–12 cm 它也不再抬（髋 / 膝的指令直接跳到接近站立值）。左脚那次保持到参考降到约 10–14 cm 才落下。七种设置下这次保持的执行高度都是 9.6–11.8 cm（参考 15.1 cm，43–51% 的时间达到参考的一半以上），落脚时刻和高度几乎一样，说明它与增益无关。推测：训练数据里“低抬脚”是几百毫秒的迈步，持续几秒的 10–14 cm 抬脚在分布之外，策略把它当作“该落地了”（**只是推测，没有对照实验**）。
- **我试过的一个解释被证伪了：** “参考的重心不在支撑脚上方，所以策略放脚”。我在“骨盆直立”的假设下算了参考的重心到支撑脚的距离：失败的右脚抬起时是 11–15 cm，但成功保持了 6 s 的左脚抬起时反而是 19–25 cm。身体侧倾的整体姿态下“骨盆直立”的假设不成立，这个量没有信息。
- **抬脚高度 ×1.3 试过了，不采用。** 重定向新增 `A3_FOOT_LIFT_SCALE`（默认 1，只放大离地那只脚高出支撑脚的部分）。同一段录制重放（`lift13`，没摔倒，骨盆最低 1.00 m）：脚残差 6.9 / 6.9 cm（基线三次 6.5–6.8），躯干 roll 标准差仍是 3.58°。右脚那次 7.6 s 保持，参考峰值 21.7 → 28.8 cm，执行峰值 22 → 33 cm，脚确实抬得更高；但放下的时刻只晚了约 1 s。基线在参考降到 13.8 cm 时执行已经是 4.7 cm；×1.3 在参考 18.4 cm 时执行还有 15.9 cm，参考降到 16.0 cm 时执行落到 6.3 cm。放下点大约是**这一次抬脚峰值的 60%**，不是一条固定的 14 cm 线，所以把整段抬脚放大并不能让后半段留在阈值上面。默认仍是 1。
- **`leg2`（2026-10-17，另一次真人录制，不是上面那段）：** 摆臂时腿不动、肘相关 0.98；向前约 3.4 m 再走回，航向和路线一致，短支撑脚滑动中位数约 1 cm；五次抬脚（参考 22–36 cm）执行到 20–31 cm 并保持 1.3–1.9 s；下蹲两次，骨盆降到 0.74 m / 0.72 m 后站回。没有改默认值。

### 用重放把这三个现象在仿真里复现（不需要你再做动作）

先关掉旧的终端（你上一次的部署终端还开着，占着 5560 端口），然后：

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
ls dump_sim_leg1r.bin sim_leg1r.csv leg1r_cmd leg1r_go 2>&1 | head -4          # 都应当是“没有那个文件”
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_make_dump_cfg.py --dump "$PWD/dump_sim_leg1r.bin"

# 终端 1 — MuJoCo
source .venv_teleop/bin/activate
A3_SIM_LOG_CSV="$PWD/sim_leg1r.csv" python gear_sonic/scripts/run_sim_loop.py --robot a3_ultra

# 终端 2 — 部署（等终端 1 就绪）
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
source scripts/setup_env.sh
A3_RUNTIME_CFG="$PWD/dist/a3_deploy_x86_64/config/a3_runtime_config.dump.yaml" ./deploy.sh --robot a3_ultra --input-type zmq_manager sim --encoder a3-fast

# 终端 3 — 重放（先等待，不发姿态）
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_pose_replay_raw.py pico_leg1_slim.pkl --no-commands --command-fifo "$PWD/leg1r_cmd" --wait-for "$PWD/leg1r_go"
```
然后在**终端 4**（或任何一个终端）按顺序：
```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
echo y > leg1r_cmd        # PD 站立；等终端 2 打印 Init Done，立刻在终端 1 的窗口按 9，再等 5 秒
echo x > leg1r_cmd        # 进跟随（此时还没有姿态流，部署用站立参考）
touch leg1r_go            # 开始重放你的 140 s 动作；之后什么都不用做
```
140 s 后依次 Ctrl-C 终端 3、2、1。分析：
```bash
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_analyze_teleop_run.py dump_sim_leg1r.bin | tee analysis_sim_leg1r.txt
```
把 `analysis_sim_leg1r.txt` 发给我，`dump_sim_leg1r.bin` 和 `sim_leg1r.csv` 我会直接读（对到每一次抬脚、每一步）。

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
- **部署进程放在本机、机器人上只跑中继**的路线见 [pico_real_teleop.md](pico_real_teleop.md) 第 4.6 节；MDU 上要用的 aarch64 包已构建在 `gear_sonic_deploy/dist/a3_deploy_rockchip/`（见 0.2 节），没有在 MDU 上运行过。厂商还没同意在机器人上运行会发布关节指令的程序（见 `real_mdu_body_drive.md`），在那之前中继只用于仿真。

---

## 9. 相关文档

- G1 版本（连接头显、手柄说明更完整）：[../g1/mujoco_pico_sim_teleop.md](../g1/mujoco_pico_sim_teleop.md)
- A3 与 A3 Ultra 的参数对照：[a3_vs_a3_ultra_params.md](a3_vs_a3_ultra_params.md)
- 链路设计、编码器选择、各项验证数据：`docs/workflow/a3_integration_plan.md` 第 9、12、14、15 节
- 工作日志：`docs/workflow/sonic_a3_ultra_worklog.md`
- VR 硬件与软件安装 / 全身遥操教程：`docs/source/getting_started/vr_teleop_setup.md`、`docs/source/tutorials/vr_wholebody_teleop.md`

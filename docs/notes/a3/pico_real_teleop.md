# A3 / A3 Ultra 真机 + PICO 全身遥操操作指南

本文说明如何用 **PICO VR** 对**真机 A3 / A3 Ultra** 做 SONIC 全身遥操。结构仿照 G1 的 [pico_real_teleop.md](../g1/pico_real_teleop.md)；真机上的启动方式、检查顺序和急停说明按 Litchi 的真机部署流程（`docs/a3_training2sim2deploy.md` 第 6 节）。仿真流程见同目录的 [mujoco_pico_sim_teleop.md](mujoco_pico_sim_teleop.md)，**先在 MuJoCo 里练熟再上真机**。

> **本文还没有在真机上跑过。** MuJoCo 里的整条链路验证过（见仿真文档第 7 节）；本文的真机部分来自代码审查、Litchi 的真机流程和脚本 / 假目录测试。第一次上真机请当作调试，每一步都有人盯着，出了问题把现象记进 `docs/workflow/sonic_a3_ultra_worklog.md`。

**安全**（Litchi README 原文意思）：必须用**安全龙门架 / 吊带**和防护措施；035 不保证每个动作都稳定，可能失稳、摔倒；真机之前先在 MuJoCo 里验证；旁边要有人能按**物理急停**，操作者和安全员不要是同一个人。软件急停（键盘 `P`、手柄 A+B+X+Y）**不能代替物理急停**。

---

## 架构概览

和 G1 不同，A3 真机是**两台机器**：

```
操作端 PC（连头显的那台）                                         机器人（MDU，aarch64）
─────────────────────────────────────                          ──────────────────────────────
PICO ──XRoboToolkit──▶ PC Service                                       ┌────────────────────────┐
                         │                                              │ a3_deploy_onnx_ref     │
        pico_manager_thread_server.py                                   │  (run_a3.sh，035 的     │
          ZMQ :5556  "pose"（人体 SMPL）  ──┐                            │   a3_fast，RKNN)        │
                      "command"（A+B+X+Y）──┼───────────────────────────▶│  订阅 Pico command      │
                                           │  (TCP，操作端 PC → 机器人)   │                        │
        a3_operator_chain.sh --real        │                            │                        │
          a3_teleop_retarget_node.py ◀─────┘                            │                        │
            人体 SMPL → GMR → A3 的 29 个关节参考                         │                        │
          a3_ta_bridge  ─ROS 2（reliable，同 ROS_DOMAIN_ID）──────────────▶ /ta/whole_body_command  │
                                                                        │       ▼                │
                                                                        │  /body_drive/*（iceoryx）│
                                                                        └────────────────────────┘
```

两条网络通路都要通：**ROS 2**（`/ta/whole_body_command`，姿态参考）和 **TCP 5556**（Pico 的 `command`，A+B+X+Y 启停）。任何一条断了，另一条不会替它兜底：断 ROS 2 → 机器人停在最后一帧参考；断 5556 → 手柄停不下来，只能用机器人侧键盘 `P` 或物理急停（见第 6 节）。

### 和 G1 真机流程的区别

| | G1 | A3 / A3 Ultra |
|---|---|---|
| 机器 | 一台电脑直连机器人（`192.168.123.x`） | 操作端 PC + 机器人 MDU 上的**预编译 aarch64 包**（`run_a3.sh`），MDU 上没有仓库和 `deploy.sh` |
| 启动后 | `Init Done`：自动做初始化坡道 | **IDLE，不发任何指令**；操作者按 **`S`** 才进 PD 站立（Litchi 流程）。仿真里为了方便是自动的 |
| 停止 | A+B+X+Y → 回 planner / 键盘 `O` 阻尼 | A+B+X+Y 或键盘 `P` → **passive：kp = kd = 0，机器人完全放松**，没有吊带会直接瘫倒 |
| 看门狗 | G1 自己的 | 状态超过 50 ms 没更新 → safe halt：同样是 kp = kd = 0 |
| 模式 | PLANNER / POSE / VR_3PT | 没有 planner；A+B+X+Y 进遥操后要 **A+X 进 POSE** 才会推姿态流 |
| 与厂商软件 | — | **厂商自带的 `motion_control` 不能和 SONIC 同时发指令**（第 2 节） |

### 适用范围（Litchi 的限制，原样适用）

- 只有 `g1` 和 `a3_fast` 两个编码器；**没有 SMPL / 遥操编码器**，所以才有我们这条“人体 → GMR → A3 关节”的重定向链。`--encoder smpl` 没有真实权重，真机不要用。
- 适合双脚着地或单脚抬起的动作；**不支持**跪、躺、爬起、手撑地；不支持快跑等高动态动作。
- 手腕跟踪还不好（训练数据分布所限）。
- `a3_fast` 延迟约 220 ms（未来参考 180 ms）；遥操手感会有滞后。
- A3 Ultra 与 A3 的差异（035 按 A3 训练、域随机化覆盖了 Ultra）：头部 2 个关节固定在默认姿态（策略不驱动），手指不驱动；Ultra 髋 roll 限位 85°（A3 为 92°）、膝 145°（A3 143°），部署程序**没有**对策略输出再做关节限位裁剪，只做 ±20 的原始动作裁剪。

---

## 0. 前置检查

### 0.1 目标平台与网络（已确认，2026-10-15）

这台 A3 Ultra 有三块板，算力平台不是二选一（来自接网线后对机器人内部网的检查：只做了 ping，**没有登录、没有下发任何指令**；地址和角色取自智元 A3 Ultra 地址表，Thor 那块另有网卡 MAC `48:b0:2d:63:4d:4d` 属于 NVIDIA 佐证）：

| 地址 | 角色 | 平台 | 与本文的关系 |
|---|---|---|---|
| `10.42.10.10` | **HDU**，头部域；网上的 PTP 主时钟 | RK3588S2 | 文档里的 SSH / rsync 跳板机，不部署 |
| `10.42.10.11` | **ADU**，具身智能 | NVIDIA **Thor** | Litchi 的 Thor 部署教程还是 TODO，**这次不往它上面部署** |
| `10.42.10.12` | **MDU**，运动控制 | **RK3588** | **SONIC 的 aarch64 / RKNN 包部署在这里**，下面所有“机器人上”的步骤都指它 |
| `10.42.10.150` | 同一张网上的另一台设备，网卡厂商 DJI Baiwang | — | 不是三块工控机之一，不要碰 |

所以走本仓库支持的路径：`build_a3_deploy_pkg.sh --arch rockchip`（RKNN），用 `run_a3.sh` 启动。

操作端 PC 的网络（本机现状）：网线接的是机器人内部网 `eth_hdu`（`10.42.10.0/24`），**不是** G1 用的 `192.168.123.0/24`。网卡 `enp11s0` 同时带 `10.42.10.99/24`（新加的）和原来的 `192.168.123.99/24`，没有设网关；默认路由仍走 Wi-Fi（头显和 PC Service 用的是 Wi-Fi 那张网）。因此：

- 机器人上的 `--zmq-host` 填 **`10.42.10.99`**（操作端 PC 在机器人内部网里的地址）；
- preflight：MDU 上 `--operator-host 10.42.10.99`，操作端 `--robot-host 10.42.10.12`；
- ROS 2 的 Fast DDS 发现要走 `enp11s0`（多网卡 + Tailscale + 虚拟网卡的机器上，发现失败时给 Fast DDS 加网卡白名单 `<interfaceWhiteList>`，只留 `10.42.10.99`；这条没有验证过）；
- 机器人的 `ROS_DOMAIN_ID`：只读探测显示三块板都在**域 232**（plan 18.8）；官方值仍要登录 MDU 读厂商环境（`A3_ROBOT_ENV`，例如 `/agibot/software/v0/entry/env/env.sh`）才能确认，不要猜。机器人侧 ROS 2 是 **Jazzy**（交叉编译用的 sysroot 里是 `opt/ros/jazzy`），操作端 `a3_ta_bridge` 是 Humble，二者经 Fast DDS 互通没有验证过。

### 0.2 在开发机上构建 aarch64 包并传到 MDU

**什么要在 MDU 上跑，什么不用**：这个 aarch64 构建是给**跑在 MDU 上的东西**用的。路线 A（完整 SONIC 放 MDU）要整包（策略 + ONNX Runtime + RKNN）；**ZMQ 中继路线（4.6）只有 `a3_zmq_relay` 跑在 MDU 上**，SONIC（策略、编码器、遥操）仍然在本机，不用把它下到 MDU。两条路线都要这套交叉编译环境（中继和完整部署共用同一个 CMake 工程、同一个 sysroot），区别只是包里带不带推理库；`build_a3_deploy_pkg.sh --arch rockchip` 会把 `a3_zmq_relay`、`run_a3_relay.sh` 一起打进包。

构建依赖（见 `build_a3_deploy_pkg.sh --help`），**本机现状（2026-10-16）**：

| 依赖 | 状态 |
|---|---|
| Docker | **没装**，也没有 aarch64 交叉编译器；装它们需要 root（`sudo` 要密码），只能由你来装，见下面 |
| Rockchip sysroot（677 MB，`python download_from_hf.py --component sysroot`，再 `sha256sum -c`） | **已下载并校验通过**：`gear_sonic_deploy/thirdparty/rockchip_sysroot/rockchip-1.0-aarch64-sysroot.tar.gz` |
| aarch64 ONNX Runtime 1.19.2 压缩包（官方 release 的 `onnxruntime-linux-aarch64-1.19.2.tgz`，带 `lib/cmake/onnxruntime/onnxruntimeConfig.cmake`，符合要求；本仓库不分发） | **已下载**：`gear_sonic_deploy/thirdparty/onnxruntime/dl/onnxruntime-linux-aarch64-1.19.2.tgz`（5 MB，git 忽略） |
| RKNN 模型（`python download_from_hf.py --robot a3_ultra --component rknn`） | 本机 x86 包的 `models/` 里已有 035 的 `.rknn` |

**装 Docker（一次性，需要你输入 sudo 密码）**：

```bash
sudo apt-get update && sudo apt-get install -y docker.io      # Ubuntu 的 Docker 包；也可以用 Docker 官方的 docker-ce
sudo usermod -aG docker "$USER" && newgrp docker               # 之后不用 sudo 就能 docker run
docker run --rm hello-world                                    # 确认能用（拉镜像要走代理时，给 Docker 守护进程配代理）
```

构建脚本会用 `docker/Dockerfile.a3-rockchip-builder`（Debian bookworm + `g++-aarch64-linux-gnu` + 上面的 sysroot）构建，本机的 `http_proxy` / `all_proxy` 会传进去（指向 127.0.0.1 时自动用 `--network host`）。**这一步没有跑过**（Docker 没装）。

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
gear_sonic_deploy/scripts/build_a3_deploy_pkg.sh --arch rockchip --jobs 12 \
    --onnxruntime-aarch64-tarball "$PWD/gear_sonic_deploy/thirdparty/onnxruntime/dl/onnxruntime-linux-aarch64-1.19.2.tgz"
# 传到 MDU（具体跳板机 / 路径按你们现场；开发机只构建和传输，不在上面发指令）
rsync -a gear_sonic_deploy/dist/a3_deploy_rockchip/ <用户>@10.42.10.12:/agibot/<sonic-package-root>/   # 经不经 HDU（10.42.10.10）跳板按现场
```

> 注意：这个构建脚本开头会清空 `dist/a3_deploy_<arch>/`，构建失败时那个目录里原来的包也没了。

### 0.3 操作端 PC

- 遥操环境 `.venv_teleop`、conda 环境 `gmr`（GMR + mujoco + mink）、conda 环境 `a3_deploy_verify`（`a3_ta_bridge` 需要的 ROS 2 库，或系统装了 ROS 2 也行）。
- 与机器人**同一个 `ROS_DOMAIN_ID`**，并且 `ROS_LOCALHOST_ONLY=0`；两边 RMW 相同（默认 Fast DDS）。**不要用默认域 0**：本机验证时发现 conda 的 ROS 2 在域 0 上内存会暴涨到几十 GB（plan 9.7）。
- 硬件同 G1：PICO 4 / 4 Pro + 双手柄、2× 脚踝 Motion Tracker、紧身裤、PC 与头显同一低延迟 Wi-Fi。

### 0.4 操作端 PC 上的 ROS 2：RMW、网卡、内存上限（重要）

操作端的 `a3_ta_bridge`（以及如果以后在本机跑部署程序，那个进程）会以 ROS 2 参与者的身份加入机器人的域。本机 conda 的 Humble 用的 Fast DDS 2.6.4 有个已知的坑（plan 9.7 / 18.7）：在这台 PC 的 Wi-Fi 局域网（域 0）里加入域时，它会一次分配几十 GB 内存，把整机卡死（2026-10-14 发生过，10-15 在 2 GB 内存上限下复现：几秒内被杀）。实验显示触发者是**其它主机上的参与者**（空的域 99 正常；域 0 里忽略其它主机的参与者也正常）；域 0 里能看到的唯一另一台 DDS 主机是 `192.168.110.14`（主机名 `ubuntu`，用户 `descfly`，有 RTPS 2.3 的新版参与者），但**没有单独隔离证明就是它，也没有确定它的数据是畸形还是 Fast DDS 2.6.4 解析不了的更新格式**。机器人三块板的 53 个参与者和本机 Fast DDS 一样是 RTPS 2.2，所以机器人域不一定会触发，但**没有在机器人域上验证过**。因此在操作端 PC 上统一这样启动：

```bash
export ROS_DOMAIN_ID=232              # 被动探测看到机器人三块板都在域 232；以厂商 env.sh 为准
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp        # conda 环境里已有（librmw_cyclonedds_cpp.so），不用安装
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp11s0" multicast="true"/></Interfaces></General></Domain></CycloneDDS>'
systemd-run --user --scope -p MemoryMax=2G -p MemorySwapMax=0 \
    gear_sonic_deploy/scripts/a3_operator_chain.sh --encoder a3-fast --real
```

- **钉网卡**：不钉的话 DDS 会向本机所有网卡（Wi-Fi、Tailscale、Meta 虚拟网卡、机器人网）广播参与者；实测我们的参与者出现在 6 个接口上。
- **内存上限**：`systemd-run --user --scope -p MemoryMax=2G -p MemorySwapMax=0` 超限时只杀这个进程，不会拖死整机（实测）。
- **已在机器人域上验证过加入本身是安全的**：2026-10-15 用 `gear_sonic_deploy/scripts/a3_ros_graph_probe.py`（只读，不发布、不订阅）以 Cyclone、钉 `10.42.10.99`、16 GB 上限加入域 232，**峰值内存 41 MB，没有暴涨；能与机器人的 Fast DDS 互相发现**，看到 339 个话题（plan 18.8）。Cyclone 对 Wi-Fi 域 0 的情况只有“12 秒没被 2 GB 上限杀掉”的间接证据。`a3_operator_chain.sh --real` 在 RMW 不是 Cyclone 或没钉网卡时会打印警告。
- 想让操作端 PC **完全不碰 DDS**：给部署程序加一个 ZMQ `a3_ref` 输入（和已有的 SMPL ZMQ 源同一机制）；这需要改 C++ 并重新交叉编译。加入机器人域已验证安全，所以现在不急着做。

### 0.5 遥操参考怎么送到部署程序：`/ta`（默认）还是 ZMQ（`--teleop-input zmq`，推荐）

默认走 `retarget 节点 → a3_ta_bridge → ROS 2 /ta/whole_body_command → 部署程序`。新增的 **`--teleop-input zmq`** 让部署程序**直接订阅重定向节点的 `a3_ref` ZMQ 流**（`tcp://<操作端PC>:5560`，`--a3-ref-host` / `--a3-ref-port`），不要 `a3_ta_bridge`、不要 ROS 2、**机器人网络上不发布任何话题**。仿真里验证过（无窗口：`mode=teleop`、没有 `a3_ta_bridge` 进程、右臂跟着挥、倾角 ≤2°、0 次摔倒）。它同时绕开下面这个问题，真机推荐用它：

**`/ta/whole_body_command` 上已有 2 个厂商订阅者（走默认 `/ta` 时，上真机前必须查清）**

只读探测（plan 18.8）看到：这个话题**现在没有发布者**，但**已经有 2 个订阅者，QoS 都是 best_effort**（我们的 reliable 发布者与它们兼容）；`/ta/realtime_status`、`/ta/service/status` 也只有订阅者——厂商自己的 TA 服务没在跑，有模块在等它。AimRT 的参与者不带 ROS 节点信息，所以**不知道这 2 个订阅者是谁**。我们的桥一开始发布，它们也会收到我们的帧。登录 MDU / ADU 看进程、并在 `sm_config` 关掉 `motion_control` 之后，确认它们不会按这些帧动作，再启动桥。

**已查清**（plan 18.11，用户本人登录 MDU 的只读检查）：其中一个订阅者就是厂商的 `motion_control`（`mc`，`sub_ta_whole_body_command`）——`mc` 在跑时，往这个话题发的帧会被它当遥操指令消费；所以真机用 `--teleop-input zmq`，**不要往机器人网发布它**。

另外：2026-10-15 的只读探测只在 `/body_drive/` 前缀下看到 `/body_drive/leg_joint_command_ros2`。MDU 上厂商配置（plan 18.12）显示：5 个指令话题和腿 / 脖子 / 手的状态**在 t3d0 里根本没有 ROS 2 出口**，只有手臂、腰、两个 IMU 有 `/ros2/body_drive/…` 镜像。所以 `--remote` 目前走不通，要厂商改配置，见 4.5。

---

## 1. 连接 PICO 与电脑

和 G1 一样，详见 [G1 真机文档第 1 节](../g1/pico_real_teleop.md)：启动 PC Service（`bash /opt/apps/roboticsservice/runService.sh`）→ 头显里 XRoboToolkit 填电脑的**局域网地址**、状态 **WORKING** → 勾选 `Head` / `Controller` / **Send** / **Full body**，关闭 Safeguard，配对并标定两个脚部 Tracker。

头显填的是**操作端 PC** 的地址，不是机器人的地址。

---

## 2. 机器人侧准备（在 MDU 上做，需要授权人员）

> 以下与 Litchi 的 `docs/a3_training2sim2deploy.md` 第 6.1 节一致，命令是它的节选；以那份文档和厂商资料为准。**绝不允许 SONIC 和系统自带的 `motion_control` 同时发布 body-drive 指令。**

1. 备份并手动修改 `sm_config.yaml`，让 `"Motion"` 功能组只启动 `agent`（把原来含 `"mc"` 的那个列表改成 `[ "agent" ]`，其它内容原样保留）：

   ```bash
   cd /agibot/software/v0/config/sm
   BACKUP="sm_config.yaml.before_sonic_$(date +%Y%m%d_%H%M%S)"
   sudo cp -a sm_config.yaml "$BACKUP"
   sudoedit sm_config.yaml
   sudo systemctl restart agibot_pm
   ```

   等约 3 分钟，确认 `systemctl is-active --quiet agibot_pm` 成功、`ps aux | grep '[m]otion_control'` **没有输出**；否则恢复备份、重启服务，不要启动 SONIC。

2. 进入包目录，只读检查（Litchi 步骤 3）：

   ```bash
   cd /agibot/<sonic-package-root>
   file ./a3_deploy_onnx_ref | grep -F 'ARM aarch64'
   grep -Eq '^[[:space:]]*backend:[[:space:]]*rknn[[:space:]]*$' config/a3_runtime_config.yaml
   test -f models/model_step_200000_g1.rknn && test -f models/model_step_200000_a3_fast.rknn
   ```

3. 环境变量（按现场；`run_a3.sh` 在 Rockchip 包里默认会 source 厂商环境，所以 `A3_ROBOT_ENV` 必须设）：

   ```bash
   export A3_ROBOT_ENV=/agibot/software/v0/entry/env/env.sh
   export A3_TRANSPORT=iceoryx            # 机器人本机的 body-drive 走 iceoryx；/ta/whole_body_command 同时订阅 iceoryx 和 ROS 2
   export ROS_DOMAIN_ID=<现场域>          # 与操作端 PC 相同
   export ROS_LOCALHOST_ONLY=0
   ```

---

## 3. 上机检查：preflight → probe

### 3.1 preflight（只读，两边都跑）

```bash
# 机器人 MDU，包目录里（构建脚本已把它放进包）
./a3_real_preflight.sh --operator-host <操作端PC的IP>
# 操作端 PC
gear_sonic_deploy/scripts/a3_real_preflight.sh --operator --robot-host <机器人IP>
```

它检查：aarch64 包 + RKNN 后端 + 模型文件、`A3_ROBOT_ENV` / `A3_TRANSPORT`、**`motion_control` 没在运行**、没有重复的部署进程、`ROS_DOMAIN_ID` / `ROS_LOCALHOST_ONLY`、操作端的 5556 通不通、操作端的 PC Service / 重定向 Python / `a3_ta_bridge` 库 / 端口占用。`FAIL` 必须处理，`WARN` 逐条看。它**不能**检查龙门架、急停和安全员。

### 3.2 probe（不发指令）

```bash
A3_TRANSPORT=iceoryx A3_PROBE_SOURCE=g1 A3_LATENCY_LOG=verbose taskset -c 4-5 ./run_a3_probe.sh
```

`--probe` 加载 035 模型、接收状态、量推理延迟，**强制关闭指令发布**。确认 6 路 body-drive 状态（waist / leg / arm / neck / 两个 IMU）都 `ready=yes`，没有持续的 stale 或 safe halt，模型后端正确。排查传输问题可用 `taskset -c 4-5 ./run_a3.sh --dry-run`（不加载模型）。

---

## 4. 启动（三处，顺序不限，推荐先开机器人侧）

**不要加 `--auto-start`，也不要加 `--auto-pd-stand`**（后者只给仿真用）：部署程序以 **IDLE** 启动，不发任何指令。

### 机器人（MDU）— C++ 策略

```bash
cd /agibot/<sonic-package-root>
taskset -c 4-5 ./run_a3.sh --teleop-source a3_fast --zmq-host <操作端PC的IP> --zmq-port 5556 \
    --teleop-input zmq --a3-ref-host <操作端PC的IP> --a3-ref-port 5560      # 推荐：参考流走 ZMQ，不经 ROS 2 /ta
```
去掉最后一行就是默认的 `/ta` 路径（要在操作端起 `a3_ta_bridge`，见 0.5）。
- `--teleop-source a3_fast`：锁定遥操编码器（按 `1/2/3` 切别的源会被拒绝），同时让部署程序监听 Pico 的 `command` topic（A+B+X+Y）。
- `--zmq-host`：**Pico 服务器所在的机器**（本机的操作端 PC 在机器人内部网里是 `10.42.10.99`）。不写就默认 `localhost`，机器人本机没有任何 Pico 服务，A+B+X+Y 永远收不到（只剩键盘）。
- 部署程序需要**真终端**（TTY）才能响应键盘：用 `ssh -t`，或在 MDU 的本地终端里跑。

启动后应看到：

```text
✓ startup IDLE, no commands sent: press S (PD stand) with the robot supported, or Pico A+B+X+Y
[pico-cmd] listening tcp://<操作端PC的IP>:5556 topic 'command' (A+B+X+Y start->teleop, stop->passive; same as G1)
✓ entering policy loop; Ctrl-C to exit
```

在开发机上（仓库 + 匹配的包）也可以用 `deploy.sh`，等价：

```bash
./deploy.sh --robot a3_ultra --input-type zmq_manager --no-chain --zmq-host <操作端PC的IP> real --encoder a3-fast
```

它会先打印 **REAL ROBOT** 横幅并要求确认（`[y/N]`；脚本里用 `--yes` 或 `A3_ASSUME_YES=1`）。

### 操作端 PC — PICO 数据流（终端 A）

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --robot a3_ultra --manager --num_frames_to_send 10
```

### 操作端 PC — 重定向链（终端 B）

走 ZMQ（推荐，不需要任何 ROS 2 设置）：

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
gear_sonic_deploy/scripts/a3_operator_chain.sh --encoder a3-fast --real --no-bridge
```

走默认的 `/ta`（要 ROS 2，见 0.4）：

```bash
export ROS_DOMAIN_ID=<现场域> ROS_LOCALHOST_ONLY=0
gear_sonic_deploy/scripts/a3_operator_chain.sh --encoder a3-fast --real
```

`--real` 会在 `ROS_DOMAIN_ID` 没设 / 为 0、或 `ROS_LOCALHOST_ONLY=1` 时**拒绝启动**。它拉起 `a3_teleop_retarget_node.py`（人体 → A3 关节，每帧约 2 ms）和 `a3_ta_bridge`（发布 `/ta/whole_body_command`，**reliable** QoS——真机的 AimRT 配置订阅的是 reliable，best_effort 的发布者收不到）。**这个 topic 同一时间只能有一个发布者**：厂商自己的遥操 / TA 软件不能同时开。

---

## 4.5 路线 B：不部署到 MDU，在本机跑 SONIC（Litchi 的 “ADU-style” 跨板 ROS 2）

> **按目前的机器人配置这条路走不通，要厂商改配置（plan 18.12）；不想等厂商，用 4.6 的 ZMQ 中继。**

Litchi 的 `a3_aimrt_config.ros2.yaml` 就是为“部署进程不在 MDU 上、body-drive 走 ROS 2 跨板”设计的，本机用 `deploy.sh … --remote` 按这个逻辑跑：

```bash
export ROS_DOMAIN_ID=232 ROS_LOCALHOST_ONLY=0        # 以厂商 env.sh 为准
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
./deploy.sh --robot a3_ultra --input-type zmq_manager real --remote --ros-iface-ip 10.42.10.99 \
    --encoder a3-fast --teleop-input zmq
```

`--remote` 的行为：`A3_TRANSPORT=ros2`；RMW 默认 Cyclone，`--ros-iface-ip` 把它钉在机器人网卡；所有 ROS 2 进程放进内存上限（`A3_MEMORY_MAX`，默认 16G）；先打印 REAL ROBOT 横幅要求确认；然后**先跑只读的门**（`a3_ros_graph_probe.py --sonic-gate`）：机器人必须发布 6 个状态话题（`/body_drive/{waist,leg,arm,neck}_joint_state`、`/body_drive/{pelvis,torso}_imu/data`）并订阅 4 个指令话题（`/body_drive/*_joint_command`），类型对、QoS 可达（我们的指令发布者是 best_effort，机器人侧任何 reliable 订阅者都收不到），**不满足就拒绝启动，什么都不发布**；通过后仍然是 IDLE 启动，按 `S` 才动。

**目前会被门挡住**：MDU 上 t3d0 的 `hal_ethercat`（plan 18.12）已经加载 ros2 后端，但 `/body_drive/*_joint_command` 的 `enable_backends` 只有 `[iceoryx]`；状态只有手臂、腰、两个 IMU 在 `/ros2/body_drive/…` 有镜像，**腿、脖子、手的状态没有 ROS 2 出口**。要走这条路必须由厂商改 `hal_ethercat`（和手的 `hal_elink`）的配置并重启 EtherCAT 主站——动力级操作，我们不做，先问厂商（问题清单见 plan 18.12）。厂商若用 `/ros2/` 前缀，部署程序还要加一个可配置的话题前缀。

**为什么不是“把机器人链路改成 ZMQ”**：ZMQ 需要对端有一个 ZMQ 端点，而机器人上没有任何进程说 ZMQ；要用就得在 MDU 上放一个中继（订阅 iceoryx 的 body-drive、转 ZMQ），这个中继要交叉编译到 aarch64、要有自己的失联保护——既要登录 MDU 又要 Docker 交叉编译，比让厂商配置打开 ROS 2 后端更重，所以链路仍按 Litchi 的 ROS 2。ZMQ 用在**遥操参考**这一段（0.5）。

---

## 4.6 路线 C：ZMQ 中继（部署进程在本机，不走 ROS 2，不改厂商配置）

4.5 的 ROS 2 路线需要厂商改 EtherCAT 主站的配置（plan 18.12），所以另一条路是：**在 MDU 上放一个很小的中继进程 `a3_zmq_relay`**，它一侧说机器人自己的传输（iceoryx，和完整部署用的是同一套 `A3AimrtBackend`、同一个 `a3_aimrt_config.iceoryx.yaml`），另一侧说 ZMQ；部署进程留在本机，用 `--robot-link zmq` 通过中继读状态、发指令。本机不需要 ROS 2 / DDS，也不会加入机器人的 ROS 域。

```
本机：a3_deploy_onnx_ref --robot-link zmq               MDU：a3_zmq_relay                     body-drive
  策略 / 编码器 / 遥操（a3_ref ZMQ）  ◀─ZMQ PUB/SUB──  状态 A3ST（200 Hz，31 关节 + 2 个 IMU）  ◀─iceoryx─  hal_ethercat
                                     ─ZMQ PUB/SUB──▶  指令 A3CM（~50 Hz）→ 校验 → 失联保护 ─iceoryx─▶
```

**中继做什么**（`include/a3_relay/a3_relay_guard.hpp`，守卫有 19 个单元测试，线格式 6 个）：

| 情况 | 中继的行为 |
|---|---|
| 启动后、本机还没发过合格指令 | 什么都不发布（`DISARMED`）。`--listen-only` 连指令发布者都不创建 |
| 第一帧指令 | 必须离**实测姿态**很近（躯干 / 四肢 ≤ 0.35 rad，颈部 ≤ 1.0 rad）才“上膛”（`ACTIVE`）；部署的 PD 站立从实测姿态起步，正常满足 |
| 每一帧指令 | 校验：全是有限数；kp / kd 在上限内（上限 = 2.5 × 策略和 PD 站立用的最大增益）；|tau_ff| ≤ 1，|dq_des| ≤ 5；q_des 离实测 ≤ 7 rad、相邻两帧之差 ≤ 6 rad（**荒谬值上限，不是动作限幅**：MuJoCo 里机器人吊在绳上开遥操，部署自己就会一帧给出 1.6–2.7 rad 的跳变、q_des 离实测最大 4.8 rad，直连也一样；真机吊着开遥操时要留意这一点，中继退出时会打印每个关节的最大值）；序号递增；只认第一个会话（session）。不合格的帧**整帧丢弃**，也不刷新“最后一次合格帧”的时间，所以一直发坏帧等于断线 |
| 60 ms 没有合格帧 | `HOLD`：按 50 Hz 重发最后一条指令（速度置 0），短时保持姿态 |
| 再过 150 ms 还没有 | `DAMP`（**锁存**）：用 300 ms 把 kp、tau_ff 降到 0、kd 升到阻尼值，目标位置不动。阻尼用的 kd 是 **Litchi 真机配置 `policy_control.profile: a3_024` 的策略 kd 表**（`--gain-profile` 可改成 `a3`），不是自己另选的数 |
| `DAMP` 之后 | 旧会话的指令一律拒绝（`latched`）；操作员再按 A+B+X+Y（或 `S`）时本机**换一个新会话**，并且第一帧（PD 站立从实测姿态起步，正常满足）离实测姿态足够近，中继才重新接管。本机在这期间会收到 `[link] … -> passive` |
| 收到 SIGINT / SIGTERM | 先 `DAMP` 2 s 再退出（第二次信号立即退出）；从未发过指令则直接退出 |
| 本机发来 kp = kd = 0（部署自己的 passive / safe halt） | 照转，不替部署做主；之后如果又断线，中继会把它阻尼住而不是一直瘫软 |

状态一侧：中继每 5 ms 取一次机器人侧的最新状态（`latest_cache`，每路 10 ms 新鲜度）；某一路过期时，状态帧照发但标成无效，本机据此拒绝使用。本机把帧到达时间减去中继量到的年龄，换成**本机时钟**的时间戳，机器人的时钟不出机器人。

**在 MDU 上**（需要 aarch64 包，包里已有 `a3_zmq_relay` 和 `run_a3_relay.sh`；构建见 0.2，需要 Docker）：

```bash
# 先只听，不发任何指令：确认状态流通、两边网络通
./run_a3_relay.sh --bind 10.42.10.12 --allow-peer 10.42.10.99 --listen-only
# 本机另开终端
python3 gear_sonic_deploy/scripts/a3_relay_tool.py listen --host 10.42.10.12 --seconds 10
# 确认后去掉 --listen-only 再启动
./run_a3_relay.sh --bind 10.42.10.12 --allow-peer 10.42.10.99
```

`--bind` 必须写具体的机器人网地址（拒绝 `0.0.0.0`）；`--allow-peer` 只收这个地址发来的指令帧（ZMQ 没有鉴权，这是第二道门；第一道是 `--bind`，第三道是上面的校验）。其它选项见 `a3_zmq_relay --help`（`--stale-ms`、`--hold-ms`、`--ramp-ms`、各种限幅、`--rt-priority`）。中继退出时会打印本次最大的 q_des 偏差和帧间步长，用它们收紧 `--max-q-err` / `--max-q-step`。

**在本机**：

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
./deploy.sh --robot a3_ultra --input-type zmq_manager real --encoder a3-fast --teleop-input zmq \
    --robot-link zmq --relay-host 10.42.10.12
```

`--robot-link zmq` 要求 `--teleop-input zmq`（部署进程里没有 AimRT，`/ta` 订阅不存在）和 `--relay-host`；`sync_mode` 必须是 `latest_cache`、`phase_align_to_sync` 必须是 `false`（默认配置就是这样）。启动后是 IDLE，A+B+X+Y（或 `S`）才站立；横幅会提醒“中继必须已经在跑”。本机终端里会打印 `[a3_zmq] relay mode …` 的变化。

**没有机器人时的联调**：`scripts/a3_relay_sim.sh`（中继接 MuJoCo 仿真，ROS 域 87）+ `deploy.sh … sim … --robot-link zmq --relay-host 127.0.0.1`。`a3_relay_tool.py fake-pc` 是一个不带部署的假 PC：按实测姿态保持几秒，然后静默或发垃圾，用来看中继的保护。**它会真的通过中继给仿真 / 机器人发指令**，只在仿真或机器人吊着时用。

**限制与没有验证的**：
- **手（O10）不转发**（`hand_bridge_enabled=false`，和默认运行配置一致）；`--probe` / `--dry-run` 不发指令。
- 部署进程不是实时进程：本机到机器人的这一段必须走有线网，Wi-Fi 不行。10 ms 的新鲜度只用在中继本地；本机侧用 `max_frame_age_ms`（50 ms）。
- 中继退出之后 `hal_ethercat` 在没有指令时的行为**厂商还没回答**（plan 18.12 的问题 3）：不要把退出中继当停机手段，带电时保持中继在跑。
- 阻尼值（`a3_024` 的策略 kd 表）和各个限幅是**初值**，没有在真机上标定；第一次一定在吊着、急停在手的状态下试，并故意断线一次看 `HOLD → DAMP`。
- aarch64 包没有构建（Docker 未装），中继没有在 MDU 上跑过；iceoryx 那一跳是现有代码（`A3AimrtBackend`），中继本身只在 x86 + MuJoCo 上验证过。
- 中继和带 AimRT 的完整部署（`run_a3.sh` 的默认）**不能在同一台机器上同时运行**：两者的 AimRT 配置都监听 `127.0.0.1:50901`，后启动的会失败。
- **厂商还没同意在机器人上运行会发布关节指令的程序**（2026-10-16 的回复：部分指令可带 Kp/Kd，写进 3.3；见 `real_mdu_body_drive.md`）。在他们说明之前，中继也只用于仿真；`--listen-only` 不创建指令发布者。

---

## 5. 第一次遥操流程

机器人**吊着**（龙门架 / 吊带受力），安全员在旁边，物理急停在手。流程和 G1 一致：

1. 终端 B 起来后、没有人按手柄时，机器人侧什么都不会动。先在部署终端里确认 `startup IDLE, no commands sent`。
2. 机器人悬空，手柄同时按 **A + B + X + Y**（或部署终端键盘 **`S`**，不要回车）→ 机器人**硬起来**：`[mode] idle -> pd_stand`，从**当前姿态**用 3 秒（150 个 50 Hz 周期）线性过渡到**默认站立姿态**，用 Litchi 真机的 PD 站立增益。完成后打印 `Init Done`。
3. **缓慢放下吊带**，让机器人自己站在地上。**站不稳 = 停**（A+B+X+Y 或 `P`），这说明硬件或标定有问题（比如关节零位不对），不要继续做任何动作。
4. 站稳后，**标定姿态**：站直、双脚并拢、看向前方；上臂贴身，前臂向前弯约 90°（肘成 L），掌心向内，保持 1–2 秒。
5. 把自己的身体摆成和机器人当前姿态接近，再按 **A + X** → `[pico-cmd] A+X -> teleop`，机器人**开始跟随**。先只做缓慢的抬臂、小幅度动作。`[a3_ta_bridge] forwarded=… rejected=0`（走 ZMQ 输入时没有这一行）。
6. 再按 **A + X** → 回到默认站立姿态（`[mode] teleop -> pd_stand`，3 秒过渡），可以再按 A+X 重新跟随。
7. 结束：按 **A + B + X + Y** → **失能**（passive，kp = kd = 0，和 Litchi 一样用零增益）。**这一步机器人会瘫软**，所以吊带要重新受力 / 有人扶稳。Pico 管理脚本在这次“停止”之后会自己退出，下次要重新运行。

**失联时**（和 G1 一致：安全检查失败 → 停止，不自动恢复）：机器人状态流中断 / 过期 → `[link] robot state lost or stale … -> passive`，机器人失能；姿态流静默超过 1 秒（`teleop.stale_fallback_ms`）→ 自动回到默认站立姿态；经 ZMQ 中继时中继的失联保护锁存 → `[link] the relay is damping … -> passive`。恢复都要**重新按 A+B+X+Y**（Pico 脚本这时还以为自己在运行：第一次按键是它的“停止”，对已失能的机器人无效，再按一次才是“启动”）。

### 模式切换安全

进 POSE 之前一定先让身体与机器人当前姿态对齐；错位时切换，真机会猛然跟姿。重新进入 POSE 时（Pico 退出再进）部署端会按 G1 同一套规则重新开始会话，初始朝向会重新对齐，所以要面向与机器人一致的方向再按 A + X。

---

## 6. 急停与失效行为

按“从最可靠到最不可靠”：

| 方式 | 作用 | 说明 |
|---|---|---|
| **物理急停** | 切断动力 | 最可靠，必须在手边 |
| 部署终端键盘 **`P`** | passive：kp = kd = 0 | 要求终端是 TTY（ssh -t / 本地终端），键盘焦点在那个终端。**机器人会瘫软** |
| 手柄 **A + B + X + Y**（机器人已硬起来时） | 同上（passive，kp = kd = 0） | 走网络：Pico 服务器发**一条**停止消息就退出；这条消息丢了或网络断了就停不下来 |
| `pkill -INT -x a3_deploy_onnx_ref` | 部署进程退出，不再发指令 | 不等于 `P`；退出之后机器人的行为取决于厂商底层，不要把它当急停 |

自动的失效行为：

- **ZMQ 中继路线（4.6）**：本机进程退出 / 网线断 / 卡住，中继 60 ms 后重发最后一条指令，再 150 ms 后用 300 ms 过渡到纯阻尼（kp → 0）并锁存；之后要操作员再按 A+B+X+Y（本机换新会话）才能重新接管。这一段在机器人侧，本机死了也照样生效。
- 同一路线下，本机的 `P` / A+B+X+Y 仍然是 kp = kd = 0（瘫软），照常转发给机器人。

- 状态流超过 `max_frame_age_ms = 50` 没更新 → **safe halt**（kp = kd = 0，机器人放松）。
- 姿态参考流中断（操作者走开、网络断了）→ 先**保持最后一帧参考**（`stale_warn_ms = 250` 起警告），静默超过 `teleop.stale_fallback_ms = 1000` 就**自动回到默认站立姿态**（PD 站立，和 G1 的 planner 过期回 IDLE 站立一致）。要再跟随就按 A+X。
- 机器人状态流过期（看门狗 safe-halt）、或中继锁存了失联保护 → 链路监督线程把机器人**失能**（passive），不会在链路恢复后自己恢复；要再按 A+B+X+Y。
- 策略输出非有限值被置 0；原始动作裁剪到 ±20，但**没有关节限位裁剪**。

---

## 7. 常见问题

| 现象 | 处理 |
|---|---|
| 部署起来后机器人没反应 | 正常：IDLE 不发指令。按 `S` |
| 按了 A+B+X+Y 没反应，`[pico-cmd]` 一行都没有 | 机器人侧 `--zmq-host` 没写成操作端 PC 的 IP；或操作端 5556 不通（防火墙 / `pico_manager_thread_server.py` 没开）。可以先 `./a3_real_preflight.sh --operator-host <IP>` |
| 进了遥操但机器人不跟动作，一直 `no_data: using stand reference tokenizer` | `/ta/whole_body_command` 没到：两边 `ROS_DOMAIN_ID` / `ROS_LOCALHOST_ONLY` 不一致，或 QoS 不匹配（旧版 `a3_ta_bridge` 是 best_effort，真机配置是 reliable），或终端 B 没起、没人按 A+X。bridge 的日志里应有 `qos=reliable` 和持续增长的 `forwarded` |
| 部署程序不开始输出指令，卡在等状态 | 缺一路 body-drive 状态；先看 `a3_backend topic readiness` 里哪一路 `ready=no`。A3 Ultra 的颈部状态里头俯仰关节叫 `ultra_head_pitch_joint`，部署程序已把它当 `head_pitch_joint` 的别名 |
| 机器人头部不稳 / 不动 | **未验证**：颈部**指令**用的是 `head_yaw_joint` / `head_pitch_joint` 的名字，状态侧才有 Ultra 别名；Ultra 的颈部控制器是否接受这个名字没有在真机上确认 |
| 动作一顿一顿 | Wi-Fi 延迟（头显到 PC）；部署终端若有 `High ZMQ latency`，换近距离私有路由 |
| 部署终端 `safe halt` 计数增加 | 状态流时间戳抖动超过 50 ms；先 `taskset` / 实时优先级、再查网络 |
| preflight 说 `motion_control` 在运行 | 回到第 2 节第 1 步；不要继续 |

---

## 8. 这份文档验证过什么

**验证过（MuJoCo / 本机）**：整条 Pico → 重定向 → `a3_ta_bridge` → 部署 → 仿真链路；`--teleop-source` 锁定编码器；Pico 的 `command` 起停（`start` → teleop，`stop` → passive）；**IDLE 启动不发指令、按 `S` 才开始发**（仿真 `cmd_rx` 在 IDLE 时一直是 0）；`a3_ta_bridge` 的 QoS 兼容性（ROS 2：best_effort 发布者 → reliable 订阅者收到 0 条，reliable 发布者两种订阅者都收得到）；`deploy_a3.sh` 对 `sim` / `real` 的参数（`tests/test_a3_real_deploy_scripts.py`）；preflight 脚本在假包目录上的行为。

**ZMQ 中继路线（4.6）在 MuJoCo 上验证过**：中继 + 部署（`--robot-link zmq`）+ 仿真整链路站立稳定（PD 站立、松绳后骨盆高度和倾角与直连一致）；断线保护：短暂卡顿（120 ms）`HOLD → ACTIVE` 自愈，进程被杀 `HOLD → DAMP`，新会话在绳上重新接管 `DAMP → ACTIVE`，站立时卡 1.5 s 进入 `DAMP` 后机器人倒下；指令线上的 kp 在约 0.4 s 内从 2000 降到 0；遥操（wave、walk 片段）全程 7198 帧放行、0 次拒绝，松绳后自由站立；`--allow-peer`、乱码帧、`--listen-only` 的拒绝；线格式与 Python 工具逐字节一致；NaN / inf 过滤不受 `-ffast-math` 影响（单元测试抓到过这个问题）。样本不足以说明中继是否改变摔倒率（5 次中继跑里有 1 次松绳后摔倒，1 次直连基线没摔）。

**没有验证**：真机的任何部分——aarch64 / RKNN 包的构建与运行、`sm_config` 步骤、iceoryx 传输下 `/ta/whole_body_command` 从 PC 经 ROS 2 到达机器人（含 PC 的 Humble 与机器人侧 ROS 2 发行版的互通）、PD 站立在真机上的表现、颈部指令名、A3 Ultra 的关节限位与质量差异在真机上的影响、网络断开时的实际行为。

---

## 9. 相关文档

- 仿真教程：[mujoco_pico_sim_teleop.md](mujoco_pico_sim_teleop.md)
- G1 真机（Pico、手柄说明更完整）：[../g1/pico_real_teleop.md](../g1/pico_real_teleop.md)
- Litchi 的 A3 训练 → 仿真 → 部署与上机流程：`docs/a3_training2sim2deploy.md` 第 5、6 节；035 训练合约：`docs/a3_035_training_contract.md`
- 链路设计、验证数据、真机审查记录：`docs/workflow/a3_integration_plan.md` 第 12、14、15 节和第 18 节；工作日志 `docs/workflow/sonic_a3_ultra_worklog.md` #51

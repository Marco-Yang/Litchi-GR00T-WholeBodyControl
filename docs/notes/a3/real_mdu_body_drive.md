# A3 Ultra 真机：MDU、body-drive 与现在卡在哪里

本文说明这台远征 A3 Ultra（T3D0）真机目前的网络和算力平台、为什么开发机还不能给关节发低层 PD 指令、相关名词各自是什么，以及厂商回答之前和回答之后分别怎么做。测量来自 2026-10-15 的只读检查：本机网卡探测、用户本人登录 `agi@10.42.10.12` 后贴回的输出。原始记录在 `docs/workflow/a3_integration_plan.md` 第 18.8–18.12 节和 `docs/workflow/sonic_a3_ultra_worklog.md` 第 52 条。

**厂商已经回了一句（2026-10-16）：有一部分指令可以带 Kp/Kd，写进版本 3.3。** 这是软件版本，不是本文的某一节。当前公开接口文档是 v3.2，机器人上还没有这份更新。其余问题没回。在 3.3 的话题名、消息字段和升级方式写清楚之前，不要改机器人上的配置，也不要启动会发布关节指令的程序。

**厂商第二次回复（2026-10-16，用户转述）：开放 `body_drive` 的前置条件是关闭他们的运控（运控指部署 policy，就是开源项目 `Litchi_sonic_for_a3` 的部署）；带 PD 入参的接口下周更新，但建立在运控之上。** 我们的解读：运控开着时不能写 `/body_drive`（本文 6.3 本来就是这么要求的）；这个 PD 入参接口是给运行中的运控传 PD 参数的入口，不是绕开运控写关节的通道，它能不能送「外部目标 + Kp/Kd」还不知道；怎么关、怎么恢复、开放是 iceoryx 还是 ROS 2、要不要重启主站、超时后的行为，仍没回答。依据和还要问的四条见 `docs/workflow/a3_integration_plan.md` 第 18.17 节。机器人上仍然什么都没动。

**厂商第三次回复（2026-10-16，售后）和官方 `joint_demo`：**运控的模型**不可以替换**；`body_drive` 有两种访问方式——iceoryx 直接读写，或改为 ros2；官方关闭 / 重启运控的命令是 `curl -X POST http://127.0.0.1:50080/json/stop_app -d '{"app_name": "mc"}'`（重启把 `stop_app` 换成 `start_app`）；带 Kp/Kd 的手臂入口是 `/motion/control/pnc/arm_joint_command`（`joint_msgs::msg::JointCommand`，Kp 放 `stiffness`、Kd 放 `damping`，只有手臂，是给运行中的运控的）。随附的 `docs/robot_parameters/A3_Ultra/joint_demo.zip`（readme + 腿的收发例程 + 预编译 `joint_msgs`）说明**客户自己**在 MDU 的 `hal_ethercat` 配置里给话题添加 ros2 通信形式，再关 `mc`、用 `JointCommand`（best_effort、50 Hz）收发；它的 `joint_msgs` 与我们的逐字节相同。两处还对不上：关 `mc` 的地址售后写 `127.0.0.1:50080`、readme 写 `10.42.10.10:50080`；`hal_ethercat` 指令超时后的行为和“改完配置怎么生效”没有回答。机器人上仍然什么都没动。详见 `docs/workflow/a3_integration_plan.md` 第 18.18 节。

真机操作步骤（吊带、急停、启动命令）见 [pico_real_teleop.md](pico_real_teleop.md)。仿真见 [mujoco_pico_sim_teleop.md](mujoco_pico_sim_teleop.md)。

---

## 1. 现在的结论

这台机器人里面有三台电脑。运动控制在 **MDU**（`10.42.10.12`，RK3588）。具身算力在 **ADU**（`10.42.10.11`，NVIDIA Thor）。头部在 **HDU**（`10.42.10.10`，RK3588S2）。三台都已 ping 通。

SONIC 若要在开发机上跑、再去驱动这台机器人的电机，现在缺的仍是 MDU 上那组 `/body_drive/*`：它们只在 MDU 自己的内存里传递（iceoryx），开发机看不见、也写不进去。厂商 2026-10-16 的回复是：有一部分指令可以带 Kp/Kd，会更新到**版本 3.3**（我们看过的公开接口是 [v3.2](https://open.agibot.com/docs/aimdk/a3-ultra/v3_2/dev_guide/07-00-detailed_interface)）。话题名、覆盖哪些关节、这台机器要不要升级，回复里都没有。

要走通，仍要等 3.3 的接口说明，或厂商改 EtherCAT 主站配置。重启主站是动力级操作，不由我们在机器人上试。

---

## 2. 已经确认的事实

### 2.1 本机网线

网线接在本机 `enp11s0`（NetworkManager 配置名 `Wired connection 2`）。这条线进的是机器人内部网 `eth_hdu`，网段 **`10.42.10.0/24`**。G1 用过的 `192.168.123.0/24` 上没有任何设备应答。

本机在这张网卡上有两个地址，都没有网关，`ipv4.never-default=yes`，无线网仍然是上网的默认出口：

| 地址 | 用途 |
|---|---|
| `10.42.10.99/24` | 2026-10-15 为这台 A3 Ultra 加上的。之后和机器人说话用它 |
| `192.168.123.99/24` | 以前留给 G1 的，还留着 |

### 2.2 机器人上的三台电脑

智元文档把这三台的关系写成：HDU 经 Wi-Fi 出外网，并和 MDU、ADU 一起用 `eth_hdu`（`10.42.10.x`）组成内部网。地址表见[详细接口文档](https://open.agibot.com/docs/aimdk/a3-ultra/v3_2/dev_guide/07-00-detailed_interface)，分工见[A3 Ultra 概述](https://open.agibot.com/docs/en/aimdk/a3-ultra/v3_2/dev_guide/01-a3_overview)。2026-10-15 21:16 从本机 ping，四台都是 2/2 通。

| 地址 | 角色 | 平台 | 这次看到的证据 |
|---|---|---|---|
| `10.42.10.10` | HDU，头部域 | RK3588S2 | 地址表。抓包里它是 PTP 主时钟。MAC `6e:59:13:b5:5c:77`（本地生成的地址，不能用来认厂商）。TTL 64，约 0.3–0.6 ms |
| `10.42.10.11` | ADU，具身智能 | NVIDIA Thor | 地址表，且 MAC `48:b0:2d:63:4d:4d` 属于 NVIDIA。TTL 64，约 0.5–1.4 ms |
| `10.42.10.12` | MDU，运动控制 | RK3588 | 登录后的板名是 `Rockchip RK3588 AGIBOT A3 MDU V4 Board`。MAC `b6:f8:f9:86:27:bb`。TTL 64，约 0.3–0.6 ms |
| `10.42.10.150` | 同一网上的另一台设备 | 网卡厂商 DJI Baiwang（MAC `ec:72:f7:0f:1f:22`） | 不是三块工控机。TTL 255 |

抓包里还有一台 MAC `02:aa:bb:cc:dd:00` 在询问 `192.168.7.1`（源地址 `192.168.7.150`），像 USB 虚拟网卡。本机不在那个网段，没有跟。

### 2.3 MDU 登录后看到的系统

用户用 `agi@10.42.10.12` 登录，只读，没有改文件。

- Debian 12，内核 6.1.118 PREEMPT_RT，aarch64，8 核（4×Cortex-A76 + 4×Cortex-A55），内存 16 GB（当时用了约 8.4 GB，没有 swap）。
- 负载平均 **10.4**。8 个核上平均有 10.4 个任务在跑或在等 CPU，比核心数多，CPU 已经排上队。这不是内存占用，也不是故障码。短时间超过 8 在开机或同时跑厂商控制器时常见；若一直停在 10 以上，后加的实时任务会和现有进程抢核。
- 网口：`eth_mdu 10.42.10.12/24`、EtherCAT 口 `ecat` / `ecat2`、另一网口 `taixin_mdu 10.42.2.50/24`。
- `/opt/ros/jazzy` 存在。RKNN 运行时 `librknnrt` 2.3.2，与 Litchi 使用的版本一致。机型配置是 `a3_ultra_t3d0`。MDU 上没有现成的 SONIC 包。
- 官方 ROS 环境：`ROS_DOMAIN_ID=232`、`ROS_LOCALHOST_ONLY=0`。MDU 的 DDS 只从 `10.42.10.12` 这个网口收发。本机用 Cyclone、钉在 `10.42.10.99`、内存上限 16 GB 做只读发现时，峰值约 41 MB，能看见机器人上的话题。
- `agibot_pm`、iceoryx 的 `agibot_roudi`、EtherCAT 主站 `aimrt_main_hal`、`hal_elink` 都在跑。厂商运动程序 `motion_control`（进程名 `mc`）也在跑。
- `sm_config.yaml` 里 `"Motion": [ "agent", "mc" ]`。文件属主是 `agi`，登录用户能写。根文件系统是 overlay，改完能不能掉电保留还不知道。

---

## 3. 当前面临的问题

开发机已经能 ping 通 MDU，也能在 ROS 2 域 232 里看见一部分话题。缺的是 SONIC 真正用来驱动电机的那一组频道。

### 3.1 主问题

SONIC 的部署程序要做两件事：读关节和 IMU 的当前值，再按 50 Hz 写出每个关节的目标位置、Kp、Kd。它使用的话题名是 `/body_drive/*`。在这台 t3d0 上，这些话题只走 iceoryx，只有 MDU 本机进程够得着。

因此「部署程序放在这台 x86 开发机上」这条路，按现在的出厂配置走不通。Litchi 的 `a3_aimrt_config.ros2.yaml` 假设 body-drive 已经出现在 ROS 2 上（部署进程在一块板、电机在另一块板）。这台机器不满足这个假设，Litchi 文档里也没有这条路在真机上跑通过的记录。

### 3.2 话题现在分别在哪

正在运行的配置是 `hal_ethercat_a3_ultra_t3d0_cfg.yaml`（约 26 KB）。`channel:` 段里：

| 话题 | 谁在用 | 搬运方式 |
|---|---|---|
| `/body_drive/{arm,leg,hand,neck,waist}_joint_command` | `hal_ethercat` 订阅；手的那路在 `hal_elink` | 只有 iceoryx |
| `/body_drive/{arm,leg,hand,neck,waist}_joint_state`、`/body_drive/{pelvis,torso}_imu/data` | `hal_ethercat` 发布；手的状态由 `hal_elink` 发布 | 只有 iceoryx |
| `/ros2/body_drive/{arm_joint_state,waist_joint_state,pelvis_imu/data,torso_imu/data}` | 一份 ROS 2 镜像 | 只有 ros2。只有这 4 个 |
| 腿、脖子、手的状态 | — | 没有 ROS 2 出口 |

腿是 12 个关节，是 SONIC 最需要的状态。没有它，策略不会启动。

兄弟机型（`p1d0`、`t2d5`）的配置里，手臂和腰的 `enable_ros2_state_topic` 写的是 `false`。t3d0 这次没把 module 段打全，所以不知道那 4 个 `/ros2/body_drive/*` 镜像是不是真的在发。IMU 那路被 UWB 融合配置引用过（`/ros2/body_drive/torso_imu/data`），较像是开着的，但是在哪块板上发还没单独确认。

ros2 后端自己的 QoS 是 `best_effort / volatile / keep_last 10`，和我们 `a3_aimrt_config.ros2.yaml` 一致。厂商一旦给指令话题加上这个后端，我们的发布者能被对上。以前担心的「对方订阅是 reliable、我们发 best_effort 对不上」在这条配置上不会出现。

### 3.3 网上已有的指令入口不能代替 body-drive

`motion_control` 在 ROS 2 和 HTTP 上订阅的是：

- `/motion/control/{leg,arm,waist,neck,hand}_joint_command`
- `/motion/control/pnc/arm_joint_command`
- `/motion/control/locomotion_velocity`
- `/motion/control/move_waist`
- `/ta/whole_body_command`
- `/wbc/infer/reference_window`

这些是交给厂商控制器的参考：速度、腰的动作、全身参考轨迹。厂商注释里，`/motion/control/leg_joint_command` 曾经是 91 个数的扁平数组，后来被 `/ta/whole_body_command` 取代。只读检查当时没有看到它们带 SONIC 用的那组 Kp、Kd。域 232 里还有一个 `/body_drive/leg_joint_command_ros2`（类型 `joint_msgs/msg/JointCommand`，有发布者、没有订阅者），像是 `mc` 发出去的一份副本，不能当成电机入口。

厂商 2026-10-16 回复原文：「是有部分可传 kpkd 的指令，3.3 会更新上去。」这里的 **3.3 是版本号**，对应现在公开的 v3.2 的下一版，不是本文第 3.3 节。「部分」表示不是每一条指令都能带 Kp/Kd；「会更新上去」表示带 Kp/Kd 的字段或话题要到版本 3.3 才写进接口。当前这台机器和 v3.2 文档都还不是那一版。哪些话题、消息里字段叫什么、3.3 是只更新文档还是要升级 MDU 上的软件，都还要他们补一句。

`mc` 正在运行，并且订阅 `/ta/whole_body_command`。开发机若往这个话题发布，厂商控制器会把它当成遥操参考消费。所以遥操用的关节参考走 ZMQ（`--teleop-input zmq`），不在机器人网上发布 `/ta`。

### 3.4 就算打开了 ROS 2，还有四件没有答案的事

1. 改的是 EtherCAT 主站。要不要重启 `aimrt_main_hal`、改完掉电还在不在，厂商还没说。配置目录属主是 `agi`、权限 755，登录就能改，没有防误改。根文件系统是 overlay，`findmnt` 那次没有输出，掉电保持仍然未知。
2. iceoryx 只有 MDU 本机进程能写关节指令。改成 ROS 2 之后，域 232 里的 HDU、ADU、开发机、任何接到 `10.42.10.0/24` 的电脑都能写。看不到鉴权。`mc` 若不关，它和 SONIC 会同时往同一组关节写。
3. 指令中断后，`hal_ethercat` 是保持最后一帧、进阻尼，还是直接掉力矩，不知道。这决定失联时机器人会停在一个姿势、变软，还是瘫倒。
4. 关掉 `mc` 的官方步骤还没被厂商确认。MDU 上现况是 `Motion: ["agent", "mc"]`。Litchi 6.1 的做法是改成只留 `["agent"]` 再重启 `agibot_pm`。`start_motion_control.sh` 只看了开头，不能当作官方步骤已经核对完。

另外，配置注释里的消息类型写成 `hal_msgs::msg::JointState / JointCommand`，我们的部署程序用的是 `joint_msgs`。域里那个 `leg_joint_command_ros2` 确实是 `joint_msgs/msg/JointCommand`，注释可能是旧的。类型不一致时，ROS 2 会直接拒绝对接。等话题真的打开后，用只读探测核对类型，再启动部署。

### 3.5 负载

MDU 在还没加上 SONIC、也还没多发一路 ROS 2 状态时，负载已经是 10.4/8。腿状态若按控制频率再抄一份到网上，CPU 和 `10.42.10.12` 这个网口都会再占一截。这个预算只有厂商清楚。他们若认为不能接受，开发机经 ROS 2 驱动关节这条路就不该开。

---

## 4. 名词和原理

下面按数据真正走过的顺序讲。先有三台电脑，再有碰电机的那个程序，然后才是话题、AimRT、共享内存和 ROS 2。

### 4.1 MDU、HDU、ADU

A3 Ultra 不是一台电脑连着所有电机。身体里有三台工控机，用内部网连在一起。

**MDU**（Motion Domain Unit，运动域）是连着电机的那一台，地址 `10.42.10.12`，芯片 RK3588。EtherCAT 线插在它的 `ecat` 口上。关节最后怎么动，一定发生在这台机器上。SONIC 的 aarch64 / RKNN 包如果要放进机器人，也是放在这里。

**HDU**（Head Domain Unit，头部域）地址 `10.42.10.10`，芯片 RK3588S2。头部交互、一部分传感器和对外的 Wi-Fi 在这边。它也是这次抓包里的 PTP 主时钟，给另外两台对时。

**ADU**（AGI Domain Unit，具身智能域）地址 `10.42.10.11`，芯片 NVIDIA Thor。视觉和其它大算力算法放在这里。Litchi 的 Thor 部署教程还是 TODO。035 策略的部署目标是 MDU，不是 Thor。

开发机能 ping 通 `10.42.10.12`，只说明网线、IP 和操作系统的网络栈是通的。ping 走的是 ICMP，不经过关节话题，也不会让电机动。

### 4.2 EtherCAT 和 `hal_ethercat`

电机驱动器挂在一条工业总线上，协议是 EtherCAT。MDU 上的程序 `hal_ethercat`（进程名 `aimrt_main_hal`，配置 `hal_ethercat_a3_ultra_t3d0_cfg.yaml`）是这条总线的主站：它周期地读编码器、写力矩或位置环参数。

别的程序不直接写 EtherCAT 帧，只跟 `hal_ethercat` 交换消息。手部走另一个进程 `hal_elink`，话题名仍然是 `/body_drive/hand_joint_*`。急停状态由 `hal_elink` 发布到 `/hal_state/emergency`（iceoryx 和 ROS 2 都有），`hal_ethercat` 订阅它。部署程序以后可以订这个话题做急停感知，现在还没做。

重启 `aimrt_main_hal` 等于重启电机主站。机器人必须悬空，物理急停必须在手边，并且要厂商认可。这就是「动力级」的意思：改的不是一个普通日志开关。

### 4.3 `/body_drive/*` 在干什么

程序之间用约定好名字的频道收发消息，这些频道叫话题。`/body_drive/` 是身体驱动这一组的前缀。

往下发的是指令，一条里面是该部位每个关节的目标位置、Kp、Kd 等：

- `/body_drive/leg_joint_command`
- `/body_drive/arm_joint_command`
- `/body_drive/waist_joint_command`
- `/body_drive/neck_joint_command`
- `/body_drive/hand_joint_command`

`hal_ethercat`（手是 `hal_elink`）订阅它们，转成 EtherCAT 帧发给驱动器。这就是低层 PD：刚度 Kp 和阻尼 Kd 随指令一起下发，电机按 `tau = Kp*(q_des-q) + Kd*(dq_des-dq) + tau_ff` 这一类规律在本地闭环。SONIC 的 035 合约依赖它自己那组 Kp/Kd，不能换成厂商运动程序内部的另一套增益。

往上发的是状态和 IMU：

- `/body_drive/{leg,arm,waist,neck,hand}_joint_state`：当前角度、速度
- `/body_drive/pelvis_imu/data`、`/body_drive/torso_imu/data`

SONIC 用这些读数组成本体感知，再决定下一拍指令。缺腿或脖子的状态时，部署程序会一直等同步，策略不启动。脖子还有一个命名问题：部署程序认 `head_yaw_joint` / `head_pitch_joint`，Ultra 的 URDF 里俯仰关节叫 `ultra_head_pitch_joint`。状态侧代码把这个名字当成别名；指令侧仍发出 `head_pitch_joint`。真机控制器是否接受这个指令名，还没验证。

```
厂商 motion_control（现在就在 MDU 上）     SONIC（只有也在 MDU 上时才能接到）
        │ 写「腿：目标角度、Kp、Kd」                 │ 读「腿现在的角度」
        ▼                                          ▲
   /body_drive/leg_joint_command            /body_drive/leg_joint_state
        │                                          │
        └──────────────►  hal_ethercat  ───────────┘
                              │
                         EtherCAT
                              │
                         腿、臂、腰、头的电机
```

### 4.4 低层 PD 和厂商参考指令不是同一层

两层都叫「关节指令」，去向不同。

| | 低层 PD：`/body_drive/*_joint_command` | 厂商参考：`/motion/control/*`、`/ta/whole_body_command` |
|---|---|---|
| 谁消费 | EtherCAT 主站，接着交给电机驱动器 | 厂商的 `motion_control` |
| 内容 | 目标位置、Kp、Kd，按关节 | 给厂商控制器跟踪的参考（速度、动作、全身轨迹） |
| SONIC 能不能用 | 这是部署程序设计的机器人接口 | 不能拿来代替。`mc` 收到后会按它自己的控制律再去写 `/body_drive` |

`mc` 开着时，它已经在写 `/body_drive`。SONIC 再写同一组话题，两个发布者会抢同一批电机。所以真机步骤里要求先让 `Motion` 功能组不再启动 `mc`。

### 4.5 AimRT 是什么

AimRT 是智元的机器人运行时。`hal_ethercat`、`hal_elink`、`motion_control` 都是跑在它里面的模块。模块不互相调用函数，而是对 AimRT 说：我发布这个话题，或我订阅那个话题。AimRT 负责把消息交到订阅者。

AimRT 不规定消息必须走内存还是走网线。每种搬运方式是一个后端，写在配置的 `enable_backends` 里：

- `iceoryx`：留在本机内存。
- `ros2`：再抄一份，用 ROS 2 从网口送出。
- 两个都写：本机一份，网上同时一份。

这台机器的 AimRT 配置里，ros2 插件已经加载，后端也定义好了。缺的不是新程序，而是那 5 个指令话题和腿、脖子、手的状态没有把 `ros2` 写进 `enable_backends`。腿和脖子的状态在 t3d0 里甚至没有现成的 `/ros2/` 镜像条目，所以不完全是「勾一个已有开关」；指令话题则是在已有条目上加后端。手的指令在 `hal_elink` 的配置里，两份都要改。

```
motion_control ──投递──► AimRT ──按话题名分拣──► hal_ethercat
                          │
                          ├─ iceoryx：放进 MDU 自己的内存（现在的 /body_drive/*）
                          └─ ros2：从网口送到别的电脑（现在只有少数 /ros2/body_drive/*）
```

### 4.6 共享内存是什么

一台电脑上每个进程平时有自己的内存。进程 A 写入的字节，进程 B 看不到。这是操作系统的隔离：一个程序写错，不会直接改掉另一个程序的数据。

关节循环大约每一两毫秒要交换一整包状态和指令。若每次都复制进内核、再复制进对方进程，会多占时间和 CPU。共享内存的做法是：操作系统在物理内存里划出一块，同时映射进两个进程的地址空间。发布者写入一格，订阅者从同一格读出。数据没有离开这台机器的内存，也没有经过网卡。

这块内存只属于 MDU。HDU、Thor、开发机各有各的内存。网线不会把对方的内存自动映射过来，它只传送程序主动发出的网络报文。

### 4.7 iceoryx 是什么

iceoryx 是管理上面那些共享内存格子的库。它在内存里做发布/订阅队列，规定谁写哪一格、读者读哪一格，避免一边还在写、另一边已经拿去用。MDU 上的服务 `agibot_roudi` 就是它的守护进程。AimRT 的 iceoryx 后端调用这套机制。

所以「`/body_drive/*` 只走 iceoryx，同一块 MDU 上的进程才够得着」是字面意思：公告板钉在 MDU 的内存里。`motion_control` 和 `hal_ethercat` 都是这块板上的进程，能读能写。开发机上的 SONIC 不在这块内存里。网线通、IP 能 ping 通，仍然看不到 `/body_drive/leg_joint_state`，也写不进 `/body_drive/leg_joint_command`。

### 4.8 ROS 2、域和 QoS

ROS 2 在这里是另一种搬运方式。AimRT 把消息复制成 DDS 报文，从 `10.42.10.12` 发出去。对方收到后再放进自己的内存。开发机因此能看见话题，代价是多一次拷贝，MDU 的 CPU 和网口也多干一份活。

**域**（`ROS_DOMAIN_ID`）把网络上的 ROS 2 分成互不发现的组。这台机器人的官方域是 **232**。开发机上没设置域的进程会进默认域 0；实验室 Wi-Fi 的域 0 上还有别的机器，本机 Fast DDS 曾经在域 0 上内存暴涨。和机器人说话时使用 232，并把网卡钉在 `10.42.10.99`，不要用仿真那套 `ROS_DOMAIN_ID=87` 加 `ROS_LOCALHOST_ONLY=1`。

**QoS** 是收发双方要谈拢的投递约定。这里用的是：

- `best_effort`：来不及就丢，不重传。控制周期更在意新的一拍，而不是补旧的一拍。
- `volatile`：不保留历史。晚加入的订阅者拿不到加入之前的最后一条。
- `keep_last 10`：队列里只留最近 10 条。

两边约定不一致时，发布者和订阅者在发现阶段就对不上，消息一条都到不了，而且不会有「发送失败」的业务报错。

### 4.9 `/ros2/` 前缀

厂商已经有一小部分状态使用另一个话题名发到 ROS 2，例如 `/ros2/body_drive/arm_joint_state`，而不是在原名 `/body_drive/arm_joint_state` 上多开一个后端。我们的部署程序把话题名写死成 `/body_drive/*`。

- 厂商若在原名上加 ros2 后端：开发机不用改代码。
- 厂商若继续用 `/ros2/` 前缀：要给部署程序加可配置的话题前缀。等他们答复再改，不提前做。

### 4.10 负载 10.4/8

Linux 的负载按「有多少任务在使用 CPU，或正在等 CPU」计算，不按百分比。8 是 RK3588 的核心数。小于 8 表示还有核空着；等于 8 表示刚好满；大于 8 表示有任务在排队。10.4/8 大约是核心数的 1.3 倍。MDU 内存是 16 GB，这个 8 不是内存容量。

Litchi 真机流程用 `taskset` 把策略绑到指定核，就是为了在这种争抢里留出实时核。多开一路 ROS 2 发布之前，要厂商判断还剩不剩这个余量。

---

## 5. 两条部署路线

遥操参考和机器人链路是两段，不要混在一个「通信」里。

**遥操参考**已经可以不经过机器人网络：Pico 的人体姿态在开发机上重定向成 A3 的 29 个关节，经 ZMQ 送给部署程序（`--teleop-input zmq`）。这段在 MuJoCo 里验证过。它解决不了电机那一段。

**机器人链路**是部署程序和 `/body_drive/*` 之间的连接。现在有三条，都还没在这台真机上跑过。路线 C（ZMQ 中继）是 2026-10-16 加的，不依赖厂商改配置。

**一个开关选路线（2026-10-16）**：`deploy.sh … --telecom iceoryx|ros2|zmq` 对应下面的 A / B / C（`deploy_a3.sh` 里解析，旧的 `--remote` = `ros2`、`--robot-link zmq` = `zmq` 仍可用）；B 和 C 用同一个部署二进制，只是机器人后端不同，所以可以都保留、到时候再选。选哪条的比较见 `docs/workflow/a3_integration_plan.md` 18.18，命令见 [pico_real_teleop.md](pico_real_teleop.md) 4.4。

### 路线 A：部署程序放在 MDU 上，继续用 iceoryx

部署程序和 `hal_ethercat` 在同一块内存里，不需要厂商打开 ROS 2，也不需要改 `hal_ethercat`。需要的是一块 aarch64 包（Docker、Rockchip sysroot、aarch64 ONNX Runtime、RKNN 模型），以及按厂商认可的步骤关掉 `mc`。MDU 上没有仓库和 `deploy.sh`，只有包里的 `run_a3.sh`。

代价是：包还没在本机构建过；MDU 负载已经 10.4/8，策略要和现有厂商进程抢 8 个核。当前选择是先不走这条。

### 路线 B：部署程序放在开发机上，经 ROS 2 接 body-drive

这是现在想走的路。`deploy.sh --remote` 会把传输设为 ros2、用 Cyclone 钉在 `10.42.10.99`、加上内存上限，并先做一次门检查：机器人必须已经在发布 SONIC 要的状态话题、订阅指令话题。不满足就退出，不发布任何东西。以 18.8 的真机快照，这道门是失败的，因为那 10 个话题都不在 ROS 2 上。

要让门通过，厂商得改 `hal_ethercat_a3_ultra_t3d0_cfg.yaml` 和手的 `hal_elink` 配置，并重启主站。我们不改这两份文件。

### 路线 C：部署程序放在开发机上，经 ZMQ 中继接 body-drive

在 MDU 上放一个很小的进程 `a3_zmq_relay`：一侧用机器人自己的传输（iceoryx，和完整部署同一套 `A3AimrtBackend`），另一侧说 ZMQ；开发机上的部署程序用 `--robot-link zmq --relay-host 10.42.10.12` 通过它读状态、发指令。开发机不需要 ROS 2 / DDS，厂商不用改 `hal_ethercat` 配置，也不用重启主站。

中继把「失联以后怎么办」放在机器人这一侧：本机进程退出、网线断或卡住时，中继在 60 ms 后重发最后一条指令，再 150 ms 后用 300 ms 过渡到纯阻尼并锁存，之后要重启部署进程才能重新接管（这是用户选定的「短时保持姿态再阻尼」）。中继还校验每一帧指令（有限数、增益上限、离实测姿态的距离、帧间步长、序号、单一会话），第一帧必须离实测姿态很近才放行。这回答了 3.4 里第 3 件事里「我们这一侧能做什么」的部分；`hal_ethercat` 自己在指令中断后的行为仍然要厂商回答。

代价：机器人上多一个我们写的进程（要登录 MDU、拷 aarch64 包；包和 Docker 已有，见 pico_real_teleop.md 0.2，还没在 MDU 上跑过）；MDU 负载已经 10.4/8，中继不做推理，开销远小于完整部署，但没有实测过；它没有在真机上跑过，只在 MuJoCo 里验证。厂商 3.3 版若提供一条能带 Kp/Kd 的官方 ROS 2 指令入口，并且 SONIC 能用，它可以代替这个中继。

用法和限制见 [pico_real_teleop.md](pico_real_teleop.md) 第 4.6 节，设计和验证记录见 `docs/workflow/a3_integration_plan.md` 第 18.13 节。**中继也是会发布关节指令的程序，同样受 6.1 的「厂商答复前不启动」限制；`--listen-only` 不创建指令发布者，只转发状态，用来先确认链路。**

无论哪条路线，`motion_control` 都不能和 SONIC 同时写 `/body_drive`。

---

## 6. 怎么做

### 6.1 现在

1. 把第 7 节的原文发给厂商。回答到来之前，不改 `/agibot` 下的配置，不重启 `aimrt_main_hal`、`hal_elink`、`agibot_pm`，不运行 `deploy.sh real` 或任何会发布关节指令的程序。
2. 开发机保持 `10.42.10.99/24` 在 `enp11s0` 上，无网关。用它访问机器人，不要用 Wi-Fi 地址 `192.168.110.79`，也不要用仿真域 87。
3. 若再做一次域 232 的只读探测，只用修好的探测工具：列全话题、Cyclone 钉 `10.42.10.99`、内存上限 16 GB、不发布、不订阅。目的只是确认哪些 `/ros2/body_drive/*` 真的在发、消息类型和 QoS 是什么。代码侧在厂商答复前不改话题名。

### 6.2 厂商答复之后

按他们的答案分支，仍然先悬空、物理急停在手，再动主站。

| 他们的回答 | 我们接着做的事 |
|---|---|
| 可以在原名 `/body_drive/*` 上加 ros2 后端，QoS 为 best_effort / volatile / keep_last 10 | 等他们改完并重启主站。开发机只读核对类型是 `joint_msgs`、状态和指令话题都在。然后才考虑 `deploy.sh --remote`。先过门，门失败就不启动 |
| 只能用 `/ros2/` 前缀 | 先改部署程序的话题前缀（小改动），再按上一行核对。前缀没落地之前不要启动 |
| 版本 3.3 提供一部分可带 Kp/Kd 的指令（2026-10-16 已如此回复，话题名未给） | 等 3.3 的接口说明或升级包。用他们写明的话题和字段。不在 v3.2 的机器上猜话题，也不自行打开 `enable_backends` |
| 必须重启 `aimrt_main_hal`，或配置掉电不保留 | 重启只在悬空和厂商步骤下做。若掉电不保留，把「如何做成持久配置」问清楚再改，避免重启后主站回到旧配置、而开发机仍按新话题在发 |
| 域 232 里谁都能发，或失联行为是保持最后一帧 / 直接掉使能 | 把这个行为写进操作步骤。物理急停仍然是第一手段。`mc` 未确认关掉之前不发指令 |
| 厂商 2026-10-16 第三次回复（售后）+ `joint_demo`：官方关运控 / 重启命令（HTTP `stop_app` / `start_app`，`app_name: mc`）；客户自己给 `hal_ethercat` 配置加 ros2 通信形式；运控模型不可替换；`/motion/control/pnc/arm_joint_command` 只管手臂 | 路线 B 有了官方做法，但要你或智元的人在悬空 + 物理急停下改动力级配置；改完怎么生效、超时行为、`50080` 在哪台机器上仍未答复，这三件有答案之前不动。见 plan 18.18 |
| 厂商 2026-10-16 第二次回复：开放 `body_drive` 的前提是关运控；PD 入参接口下周更新、建立在运控之上 | 前半是「必须先关运控」的确认；官方关法 / 恢复步骤仍没有。等下周的接口说明，不预先猜话题、不预先改 `sm_config`。见 plan 18.17 |
| 关 `mc` 的步骤与 Litchi 一致，或他们给出了别的步骤 | 只按厂商确认的步骤做。做完用 `ps` 确认没有 `motion_control` / `mc`，再允许 SONIC 写 `/body_drive` |
| 多一路 ROS 2 状态超出 MDU 预算 | 停止路线 B。若仍要上真机，回到路线 A：在 MDU 上跑 aarch64 包，走 iceoryx，并重新评估 10.4/8 的负载 |

路线 B 真要启动时，遥操参考继续用 `--teleop-input zmq`，避免 `/ta/whole_body_command` 被还活着的 `mc` 消费。部署进程以 IDLE 启动，不自动进 PD 站立；悬空时才按 `S`。软件急停是键盘 `P`（Kp、Kd、前馈力矩都为 0，机器人会放松）。它不能代替物理急停。

### 6.3 明确不做

- 不在机器人上手工改 `hal_ethercat_a3_ultra_t3d0_cfg.yaml` 或 `hal_elink` 来「试一下 ROS 2」。
- 不把 `/motion/control/*` 或 `/ta/whole_body_command` 当成 SONIC 的电机接口。
- 不在 `mc` 还在运行时发布 `/body_drive/*_joint_command`。
- 不因为 ping 通了就运行 `deploy.sh real`。

---

## 7. 发给厂商的提问

下面这段可以原样转发。

---

我们有一台远征 A3 Ultra（T3D0）。开发机已接入机器人内部网，MDU 地址是 `10.42.10.12`，ROS 域是 `232`。我们希望在开发机上通过 ROS 2 向关节发送低层 PD 指令（目标位置、Kp、Kd），并订阅关节状态来做自研控制器。

只读看过正在运行的配置 `hal_ethercat_a3_ultra_t3d0_cfg.yaml`：`/body_drive/{arm,leg,waist,neck,hand}_joint_command` 以及对应的关节状态、骨盆和躯干 IMU，目前都只有 iceoryx 后端。ROS 2 上已有的镜像只有 `/ros2/body_drive/` 下的手臂状态、腰状态和两个 IMU。`motion_control` 在 ROS 2 上提供的 `/motion/control/*_joint_command` 和 `/ta/whole_body_command` 是给厂商控制器的参考指令，不是低层 PD 透传。

请逐条确认：

1. 能否在 t3d0 的 `hal_ethercat` 上，给 `/body_drive/{arm,leg,waist,neck}_joint_command` 加上 ROS 2 后端？手部 `/body_drive/hand_joint_command` 在 `hal_elink` 里，能否同样加上？QoS 是否为 `best_effort / volatile / keep_last 10`？能否把 `leg_joint_state`、`neck_joint_state`、`hand_joint_state` 也发到 ROS 2？话题是用现在的同名，还是像现有镜像一样加 `/ros2/` 前缀？t3d0 上 `enable_ros2_state_topic` 现在的值是什么，已有的手臂和腰镜像是否真的在发布？

2. 上述修改是否必须重启 `aimrt_main_hal`？配置写入后掉电、重启是否仍然保留？除了改 `hal_ethercat` 的 `enable_backends`，有没有官方支持的方式，让外部控制器经 ROS 2 下发低层 PD（目标位置、Kp、Kd），而不是只发 `/motion/control` 或 `/ta/whole_body_command` 这种参考指令？

3. ROS 2 指令入口打开后，谁可以发 `/body_drive/*_joint_command`？有没有鉴权、白名单或域隔离？`motion_control` 是否必须先关掉，否则会和外部发布者同时写同一组关节？外部指令中断或超时后，`hal_ethercat` 是保持最后一帧、进入阻尼，还是直接掉使能？超时时间是多少？

4. 官方关闭 `motion_control`、只保留代理的做法，是不是把 `/agibot/software/v0/config/sm/sm_config.yaml` 里 Motion 功能组改成 `["agent"]`？改完如何生效，是否需要重启 `agibot_pm` 或状态机？请给出推荐步骤。

5. 当前 MDU 负载大约是 10.4/8。若再按控制频率把腿、脖子、手的状态从 ROS 2 发布出去，CPU 和网口开销是否在实时预算内？有没有推荐的发布频率或必须避开的核？

---

五问和上面几节的对应关系：

| 问 | 我们为什么要这个答案 |
|---|---|
| 1 | 确认缺的 5 路指令和腿、脖子、手的状态能不能出现在 ROS 2 上，以及话题名用不用改代码 |
| 2 | 确认这是不是一次主站重启、改动会不会断电丢失，以及有没有不必改主站配置的官方入口。2026-10-16 只回答了最后一点的一半：有一部分指令能带 Kp/Kd，写在版本 3.3；话题名和要不要升级这台机器还没说 |
| 3 | 确认打开之后谁都能不能发指令，以及网断了电机是保持、阻尼还是瘫软 |
| 4 | 确认怎样让厂商控制器停止和 SONIC 抢同一组 `/body_drive` 指令 |
| 5 | 确认 MDU 在 10.4/8 的负载上还能不能再抄一份高频状态到网上。不能的话就停路线 B |

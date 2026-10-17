# A3 / A3 Ultra 真机 + PICO 全身遥操操作指南

本文说明如何用 **PICO VR** 对**真机 A3 / A3 Ultra** 做 SONIC 全身遥操。结构仿照 G1 的 [pico_real_teleop.md](../g1/pico_real_teleop.md)；真机上的启动方式、检查顺序和急停说明按 Litchi 的真机部署流程（`docs/a3_training2sim2deploy.md` 第 6 节）。仿真流程见同目录的 [mujoco_pico_sim_teleop.md](mujoco_pico_sim_teleop.md)，**先在 MuJoCo 里练熟再上真机**。

> **真机验证状态（2026-10-17）：** 路线 C（ZMQ 中继）在**悬空**下验证过：PD 站立、`p`、重新接管、Ctrl-C 断线（HOLD → DAMP）、中继崩溃（机器人保持最后一条指令）、命令流动时急停（立刻软掉）；**策略（a3_fast）和真 Pico 头显进真机部署还没有跑过**，MuJoCo 里整条链路验证过（见仿真文档第 7 节）。第一次让策略在真机上跑请当作调试，每一步都有人盯着，出了问题把现象记进 `docs/workflow/sonic_a3_ultra_worklog.md`。**直接照下面的“快速流程”做。**

**安全**（Litchi README 原文意思）：必须用**安全龙门架 / 吊带**和防护措施；035 不保证每个动作都稳定，可能失稳、摔倒；真机之前先在 MuJoCo 里验证；旁边要有人能按**物理急停**，操作者和安全员不要是同一个人。软件急停（键盘 `P`、手柄 A+B+X+Y）**不能代替物理急停**。

---

## 快速流程：路线 C（PC 经 ZMQ 中继）的真机 Pico 遥操（2026-10-17，当前照这一节做）

**范围。** 机器人在 MDU 上只跑 `a3_zmq_relay`，SONIC 部署程序和 Pico 链路都在本机 PC。MDU 地址 `10.42.10.12`，PC `10.42.10.99`，用户 `agi`。其余小节是背景和细节，这一节是可以直接照着做的顺序。

**一条红线：悬空时不要进遥操（A+X）。** 遥操 = 策略在跑，策略需要脚着地；悬空时它会乱挥（单帧步长 1.6–2.7 rad，指令离实测 4–4.8 rad，plan 18.13），中继的限幅（7 / 6 rad）只拦荒谬值，不拦这个。悬空时只做 PD 站立，**脚承重之后才按 A+X**。

### 开始之前（缺一不做）

- 机器人在龙门架上，**吊绳全程不撤**，吊架能**可控慢放**；整个过程吊绳要能接住机器人（最后一档是“刚好松弛、下沉不到约 3 cm 就接得住”）。
- **遥控器（L1+L2+R1+R2 同时按 ≥ 0.5 s）和 AimMaster 悬浮球的急停各有专人**，不是操作者；另有第二个人；机器人摆动范围（约 1 m）内没有人和杂物；电池够（两块都看一下）。
- Pico 头显已连上，XRoboToolkit PC Service 在跑（`pgrep -f RoboticsServiceProcess`）。
- 已知并接受的限制：① 断线（本机死 / 网线断）后约 0.5 s 内从站立变阻尼瘫软，靠吊绳接住；② **中继进程一旦崩溃 / 被杀，机器人卡在最后一条指令上不会松**（`hal_ethercat` 保持最后一条指令，实测至少 1 分钟不超时），放开它只有急停、重启中继加部署按 `p`、断电；③ 急停 = 四肢下电，之后要**整机重启**，`mc` 会自己回来、MDU 上的中继进程没了。

### 终端分工

| 名字 | 在哪 | 做什么 |
|---|---|---|
| M1 | MDU（`ssh agi@10.42.10.12`） | 中继（一直开着，**不要在里面按键**） |
| M2 | MDU，第二个 ssh | `stop_app` / `start_app` / 检查 |
| P1 | 本机 | 部署（有 TTY，键盘 `s` / `p` 在这个终端里；**不用的时候不要往里按键**） |
| P2 | 本机 | Pico manager |
| P3 | 本机 | 只读：预检 / 快照 / 录状态流 |

### 步骤

**1. 停掉厂商的运控 `mc`（M2；整机每次开机后 `mc` 都会自己起来，所以每次都要做）。** 机器人必须已悬空、绳子承重、急停手就位。
```bash
ssh agi@10.42.10.12
pgrep -a -x mc | cut -c1-100                      # 应当有一行（进程名是 mc，命令行是 ./motion_control）
curl -sS -i -H 'content-type:application/json' -X POST 'http://127.0.0.1:50080/json/stop_app' -d '{"app_name": "mc"}'
sleep 3; pgrep -a -x mc || echo "mc stopped"      # 必须看到 mc stopped
```
停 `mc` 之后机器人没有智元的站立 / 平衡保护，只靠吊架；`hal_ethercat` 在状态流上不受影响（2a 实测）。

**2. 起中继（M1）。**
```bash
ssh agi@10.42.10.12
cd ~/a3_relay_rockchip && export A3_ROBOT_ENV=/agibot/software/v0/entry/env/env.sh
./a3_real_preflight.sh --relay --bind-ip 10.42.10.12          # 必须 0 FAIL（“mc 没在跑”是 PASS，没有别的中继）
./run_a3_relay.sh --bind 10.42.10.12 --allow-peer 10.42.10.99
```
预期状态行 `DISARMED owner=0 acc=0 … cmd_rx=0/s published=0`：在本机发来第一条合法指令之前它什么都不发布。

**3. 本机预检和只读确认（P3）。**
```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
gear_sonic_deploy/scripts/a3_real_preflight.sh --operator --relay-host 10.42.10.12     # 0 FAIL、0 WARN
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_relay_tool.py snapshot --host 10.42.10.12 --seconds 3 | tee snapshot_start.txt
.venv_sim/bin/python -I gear_sonic_deploy/scripts/a3_pd_stand_preview.py snapshot_start.txt --collision      # 看行程：> 0.5 rad 的关节先用手摆近默认站姿
```

**4. 起 Pico manager（P2）和部署（P1）。**
```bash
# P2
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --robot a3_ultra --manager --num_frames_to_send 10

# P1（--input-type zmq_manager 会自己起重定向链；横幅处输入 y）
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
./deploy.sh --robot a3_ultra --input-type zmq_manager real --encoder a3-fast --teleop-input zmq --telecom zmq --relay-host 10.42.10.12
```
等 P1 出现 `✓ startup IDLE, no commands sent` 和 `✓ entering policy loop`。此时机器人侧什么都不会动。

**可选的零风险预演（5 分钟，想先看 Pico 数据流时做）：** 第 2 步的中继换成 `./run_a3_relay.sh --bind 10.42.10.12 --allow-peer 10.42.10.99 --listen-only`（`mc` 不用停，机器人不可能收到指令），再开 `gear_sonic_deploy/scripts/a3_ref_monitor.py --seconds 60` 看 `a3_ref` 的速率 / 范围 / 跳变，按 A+B+X+Y 和 A+X 都不会有任何东西到机器人；做完 Ctrl-C 中继，按第 2 步重新起正式的中继。

**5. 悬空 PD 站立（只验证站得住）。** 向两个急停手喊一声，然后 Pico 手柄同时按 **A+B+X+Y**（或在 P1 按一次 `s`，不要回车）。预期 P1：`[mode] idle -> pd_stand` → `robot enabled: new session …` → `relay mode ACTIVE` → 3 s 后 `Init Done`；M1：`mode DISARMED -> ACTIVE`，`cmd_rx=50/s`，`rej=0`。机器人用 3 s 从当前姿态走到默认站姿。保持 ≥ 10 s。**不要按 A+X。**

**6. 吊绳逐步放下（脚承重）。** 每次放下 **1–2 cm**，停 10 s，同时在 P3 取一次快照看腿的 tau：
```bash
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_relay_tool.py snapshot --host 10.42.10.12 --seconds 3 | tee snapshot_load_N.txt
```
脚着地之后继续放，直到吊绳刚好松弛但下沉不到约 3 cm 就接得住。每一档看：站得住、关节不抖、髋 / 膝 / 踝的 tau 随承重平滑增长（`q_err_max` 不上升；如果在一半承重时 tau 已经接近电机额定值就停）、M1 的 `rej` 不涨。**站不稳 = 停**：A+B+X+Y 或 P1 按 `p`（机器人立刻瘫软，吊绳必须还拉着），把吊绳拉回承重，查零位 / 增益，不要继续。

**7. 标定姿态 → 进遥操。** 操作者站直、双脚并拢、看向前方；上臂贴身，前臂向前弯约 90°（肘成 L），掌心向内，保持 1–2 s，**身体姿态和机器人默认站姿对齐，面朝机器人朝向的方向**。按 **A+X** → P1：`[pico-cmd] A+X -> teleop`，机器人开始跟随（**这是策略第一次在真机上跑**）。先站着不动 15 s，看稳不稳；然后只做缓慢的抬臂、小幅度动作。

**8. 遥操中的开关。** 再按 **A+X** → 策略站立（仍在遥操模式里，a3_fast 继续跑，参考换成默认站姿）；再按 A+X → 重新跟随。

**9. 结束。** 先 A+X 回策略站立；然后 **A+B+X+Y** → passive（kp = kd = 0，**瘫软**，吊绳必须承重 / 有人扶稳）；P1、P2 里 Ctrl-C；M1 里 Ctrl-C（中继先阻尼 2 s 再退出）；最后恢复厂商运控（M2）：
```bash
curl -sS -i -H 'content-type:application/json' -X POST 'http://127.0.0.1:50080/json/start_app' -d '{"app_name": "mc"}'
sleep 5; pgrep -a -x mc
```
Pico 管理脚本在“停止”之后会自己退出，下次要重新运行。

### 出了问题怎么办（从最快到最慢）

| 现象 | 做什么 | 结果 |
|---|---|---|
| 有人 / 物体可能被打到、大幅摆动、异响、焦味 | **立刻急停**（遥控器 L1+L2+R1+R2 ≥ 0.5 s，或 AimMaster） | 四肢下电，吊绳接住；**之后要整机重启** |
| 站不稳 / 抖 / 策略行为不对但没有危险 | Pico A+B+X+Y 或 P1 按 `p` | passive，瘫软，吊绳承重 |
| P1 打印 `[link] … -> passive` | 看原因：状态流断（急停 / MDU 问题）、中继 DAMP（本机卡住或断线）、listen-only | 恢复要重新 A+B+X+Y（中继 DAMP 锁存：新会话在测得位姿 0.35 rad 内才被接受） |
| 本机程序死了 / 网线断 | 什么都不用做 | 中继 60 ms 后 HOLD，约 0.5 s 内变阻尼，吊绳接住 |
| **MDU 上的中继进程死了 / 被杀（M1 出现 `Killed` 或消失）** | **机器人会一直僵硬地保持最后一条指令**：叫急停手，或重启中继 + 重启部署 + `p` | 没有任何自动放开 |
| 按了急停之后 | 停 P1、P2、M1，整机正常重启（不要强制关机）；重启后 `mc` 自己起来，重新从第 1 步开始 | 我们放在 `~` 下的文件会保留 |

**杀进程、停进程的命令一律先 `ps -eo pid,ppid,stat,comm,args | grep <名字>` 看清目标，按 PID 杀，并验证它真的没了**（中继的内核进程名是 `aimrt_main`，不是 `a3_zmq_relay`，`pkill -x a3_zmq_relay` 什么都不会杀）。

### 2026-10-17 第一次遥操的结果和一个已知问题：跟随模式下的腿部参考（下半身差、A+X 切回时抖）

**现象。** 第一次真机遥操（脚着地、吊绳松）：上半身跟踪好；下半身跟踪差，尤其是脚，整体前后晃；每次 A+X 回策略站立时抖动着恢复；MuJoCo 里没有。部署日志本身是健康的（50 Hz 循环 `infer_dt=20.00 ms`，推理 1.3 ms，重定向 2.2 ms，端到端 `total` 2.5–9 ms，`rx` 0.35 ms，中继 `refused` 35 帧 / 12 min，都是单帧）。

**我在不接机器人的情况下查到的（`a3_ref_monitor.py`，纯 PC）。** 用合成的“站着的人”走一遍真实的重定向链，`a3_ref` 里的腿部参考是：左 / 右踝 roll **+0.348 / −0.347 rad，正好在关节极限（±0.349）上**；而部署“策略站立”用的站姿参考（`teleop_motions/..._stand_Skeleton0.csv`）里是 **+0.007 / −0.054**。也就是说跟随模式的腿参考和策略站立的腿参考相差约 **20°（踝 roll）**，髋 pitch 也差约 0.15 rad。原因：实时 GMR 的表（`general_motion_retargeting/ik_configs/smplx_to_a3_ultra.json`）里脚的**旋转权重是 0**（`left_ankle_roll_Link: ['left_foot', 100, 0, …]`，两张表都是），脚的朝向没有约束，IK 为了凑脚的位置把踝 roll 推到极限。**A+X 回策略站立时，腿的参考一下子跳 20°，策略要把脚拉回去，这和“切回默认姿态时抖”吻合；跟随时脚的参考就是歪的，和“下半身差，尤其是脚”吻合。** MuJoCo 里同一份参考也存在，但策略在理想的踝 / 地面上扛得住；真机扛不住。**再次更正（2026-10-17 晚，真机带数据记录的第一次运行，`real_run1`）：下面这条“被仿真削弱”的结论本身也站不住，真机数据支持“跟随时的腿参考是原因”。** 两次更正的来龙去脉：03:00 我只看到“参考的踝 roll 贴极限”；15:30 我在 MuJoCo 里看测得的踝 roll 和基座晃动都很小，就说“不足以解释”；但那次我**没有看策略输出的髋 roll 指令**，而且合成站立的人是纹丝不动的。真机数据（用同一套方法，参考取自 dump 的观测，不是日志时间）表明：

| 阶段（真机，脚着地、绳松） | 基座俯仰晃动 | 基座角速度 | 髋 roll 稳态力矩 | 最差的腿指令误差 |
|---|---|---|---|---|
| PD 站立（开环，无策略） | 0.00–0.02° 标准差，几乎不动 | 0.002 rad/s | 7–9 N·m（≈4%） | — |
| 策略站立（策略 + 默认站姿参考，无操作者） | 标准差 0.01°，峰峰 0.1° | 0.004 rad/s | 1–9 N·m | 0.21 rad（踝 pitch） |
| **跟随（策略 + 操作者）** | **标准差 0.35–0.56°，峰峰 2.0–2.8°** | **0.013–0.020 rad/s** | **左 +61 / 右 −53 N·m（上限的 22–28%，峰值 83 N·m）** | **左髋 roll +0.49…+0.52 rad** |
| 仿真跟随（合成站立的人） | 标准差 0.16–0.17°，峰峰 0.7° | 0.008 rad/s | （仿真无力矩记录） | 左髋 roll 指令 +0.24 |

- **同一个策略、同一个机器人，策略站立和 PD 站立都纹丝不动；只有跟随时晃**（前后约 2–3° 峰峰，≈ 你看到的 5 cm）。所以不是机械、吊绳、PD 增益、IMU、延迟的问题（它们在站立时都在场，也没有问题）。
- **机理（数据支持，不是纯推测）：** 跟随时策略收到的腿参考是歪的——左 / 右踝 roll 参考 **+0.348 / −0.343 rad（贴关节极限）**，实测踝 roll ≈ 0（策略不跟）；作为对这个不可行参考的反应，策略输出的**髋 roll 指令是 ±0.47–0.48 rad，而实测髋 roll ≈ −0.04**（脚踩着地，腿张不开），于是髋 roll 关节长期顶着 **±50–60 N·m 的稳态力矩**（站立时 1–9 N·m）。这是一个顶着摩擦力的、不可行的指令，晃动和“轻微抖动”就是它在真机上的表现；仿真里同一个指令只有一半幅度（+0.24），所以仿真晃得小。
- **切换时的“疯狂抖动”也对得上：** 切到策略站立（或切回跟随）时，腿参考一下子变化，髋 roll 指令在几个 tick 内从 ±0.47 回到约 0（腿指令单步最大 0.27–0.32 rad/tick），预载的 50–60 N·m 突然释放：**腿关节速度峰值 3.4–4.4 rad/s，基座角速度峰值 2.2–2.8 rad/s**（仿真：0.8–1.0 和 0.5–0.6 rad/s，差 4–5 倍）。
- 跟随时的**手臂**参考也和我的合成测试不同：你按“前臂向前约 90°”站着，参考的肘角却只有 0.29–0.36 rad（合成人是 1.57）——参考里的肘比你实际的小很多，**这一点没有查原因**（头显的身体姿态来源？校准？），上半身“跟踪得好”指的是机器人跟着参考，不是参考跟着你。

**下一次真机跑（一个开关的 A/B）：** 用 `A3_LOWER_BODY=stand` 把腿固定为策略站立的参考，其余完全不变（做法见下面“下一次真机跑”，只在部署命令前加这一个环境变量）。**预测（可以被证伪）：** 髋 roll 指令回到 ≈ 0.06 rad，稳态力矩 < 10 N·m，基座晃动和策略站立一样（峰峰 < 0.3°），切换时没有 3–4 rad/s 的腿速度。预测不成立（腿固定后仍晃），说明还有别的原因，再看 dump。代价：机器人不复现操作者的腿，只做上半身遥操。长期的做法有两个：修 GMR 表里脚的旋转权重 / 偏移（让实时重定向给出可行的腿参考），或等 SMPL 编码器。

**`A3_LOWER_BODY=stand` 的 A/B 结果（2026-10-17 20:00，`real_run2`，预测基本成立）：**

| | run1（腿跟随操作者） | run2（腿固定为站立参考） |
|---|---|---|
| 髋 roll 指令（左 / 右）| +0.47 / −0.48 rad（实测 −0.04）| −0.09 / +0.10 rad（实测 +0.02） |
| 髋 roll 稳态力矩（左 / 右）| 61 / 53 N·m（上限的 22–28%）| 10–14 N·m（≈5%） |
| 跟随、手臂不动时基座俯仰（标准差 / 峰峰）| 0.35–0.56° / 2.0–2.8° | 0.29° / 1.6° |
| 手臂大幅摆动时基座俯仰（峰峰）| （没做）| 3.0–4.8° |
| 切换时基座角速度峰值 | 2.2–2.8 rad/s | 1.1–3.4 rad/s（持续约 100 ms，0.4 s 内回到 0.05 以下）|

- **成立的：** 髋 roll 的 50–60 N·m 稳态顶撑没了（−80%），力矩现在只有几 N·m 到十几 N·m；手臂跟踪很好（见下）；你的主观感受（更稳、不再突然抖）和数据一致。
- **没有消失的，也没有预测对的：** 切换时仍有一个约 100 ms 的小冲击（基座角速度峰值 1.1–3.4 rad/s，仿真 0.5–0.6）。原因不是腿，是**手臂参考在一个 tick 内的阶跃**：切到策略站立时参考的手臂一帧内变化 0.5–1.24 rad（肘从操作者的 0.1 跳到站立参考的 1.03），策略输出的手臂指令单步 0.3–0.7 rad、腿指令单步 0.4–0.75 rad，身体被手臂的突然摆动带一下。**最大的一次（3.43 rad/s）正是操作者手臂伸直垂着时切的**（跳 1.24 rad）；最小的一次（1.10）阶跃只有 0.58。所以**切换前把手臂摆到站立参考的姿势**（见下面的示范）能把冲击再压低。
- **手臂跟踪：** 手臂 100° 的摆动，机器人关节与参考的相关系数 0.99，幅度比 0.9–1.05，滞后 20–60 ms，均方根误差 2–4°；操作者手臂静止时，机器人手臂的抖动幅度就是参考的抖动幅度（肘 ±8° 是**参考本身**在晃：操作者的前臂 / 头显的手部数据，不是机器人自己晃）。
- **“躯干静止时微微慢慢晃”：** 跟随时操作者的腰参考本身在动（“静止”的 10 s 里俯仰峰峰 4.4°，肘参考峰峰 51°），机器人基座俯仰与操作者腰俯仰参考的相关系数 −0.80（肘 −0.75）：这是策略在**平衡操作者身体重心的移动**（躯干前倾，骨盆往后让），0.2 Hz 左右，幅度 1.3–1.6°，不是失稳。要让机器人躯干更静，是操作者的躯干要更静。
- **策略站立段没有 run1 那么静**（基座俯仰标准差 0.14 / 0.03 / 0.16° vs run1 的 0.01°，角速度 0.010–0.015 vs 0.004 rad/s）：绝对值仍然很小，**原因没有查**（run2 里站立段紧接在跟随之后，脚的位置 / 重心可能和 run1 不同）。
- **踝 pitch 的 12–17% 稳态力矩是正常的重力补偿**（踝 Kp 60 × 0.3 rad ≈ 20 N·m），分析脚本现在按“指令误差 × Kp 占力矩上限的比例”标 LARGE（> 25%），不再按角度标，所以 run2 没有任何 LARGE。
- **腿固定后：** 机器人不复现操作者的腿，只做上半身遥操。这对现在的目标（上半身遥操）够用；要做蹲、迈步，需要修 GMR 表里脚的旋转权重 / 偏移（让实时重定向给出可行的腿参考）后再开 `track`。

**严格示范（下一次真机跑，腿固定为站立，约 4 分钟；目的：把“参考本身的噪声”和“机器人的响应”分开，并验证切换前对齐手臂能把冲击压低）。** 操作者全程双脚不动，脚尖朝前，视线向前。**标定姿势** = 上臂贴身，前臂向前，肘弯约 60–75°（站立参考的肘是左 59° / 右 74°，手掌向内），保持 2 s。流程（每一步先看手机计时，别抢）：

1. 摆标定姿势 2 s → **A+X（进跟随）** → **保持标定姿势一动不动 20 s**（躯干不要前后晃，手别抖；这 20 s 给出参考噪声的基线）。
2. **慢摆 5 次：** 两只手同时，从标定姿势抬到胸口高度（肘约 90°，前臂水平）再放回，**每次 4 s（上 2 s 下 2 s）**；摆完回标定姿势静止 5 s。
3. **快摆 5 次：** 同样的动作，**每次 2 s（上 1 s 下 1 s）**；摆完回标定姿势静止 5 s。
4. **单手：** 只动左手 3 次（4 s 一次），静止 5 s；只动右手 3 次，静止 5 s。
5. **对齐切换（新）：** 回标定姿势，**保持 2 s，再按 A+X（进策略站立）**；站立 10 s；回标定姿势、保持 2 s，**再按 A+X（回跟随）**；跟随静止 10 s。**这样做 2 次。**
6. **不对齐切换（对照，1 次）：** 双臂自然垂在身侧（肘伸直），**直接按 A+X**（进策略站立）；站立 10 s；A+B+X+Y 失能（吊绳承重），收尾同前面的步骤。
7. 你记下来的只要：第 5 步的两次和第 6 步那次，你**主观**上哪一次最冲（1 句话）。

跑完分析命令同前（`real_run3.npz` / `dump_real_run3.bin`；**跑完先改名，别重复执行旧的 mv**——run2 的第一次分析就是因为 `mv` 命令里的名字是旧的，覆盖了 run1 的 dump）。判据：对齐切换的基座角速度峰值应明显低于不对齐的那次（run2 里的对照：阶跃 0.58 rad → 1.10 rad/s；阶跃 1.24 rad → 3.43 rad/s）；如果对齐了还是 > 2 rad/s，说明冲击不只来自手臂阶跃，再看别的。

**第三次真机跑（`real_run3`，腿固定，严格示范）和两个遗留问题（2026-10-17 晚）：**

- **切换时的冲击仍在，而且和“操作者对齐”基本无关：** 6 次切换基座角速度峰值 3.72 / 2.55 / 3.46 / 2.88 / 3.27 / **0.75** rad/s。只有最后一次小，是因为那一次参考的手臂和站立参考只差 0.45 rad；其余 5 次**参考**里的手臂和站立参考差 0.87–1.25 rad（右肘差约 1 rad）。也就是说你摆的“标定姿势”（肘 60–75°）在**参考里**没有变成站立参考的肘（左 59° / 右 74°）：参考的右肘只有约 0.3 rad。操作者看不到参考，所以“对齐”做不到。冲击峰值随参考阶跃大致单调（0.45→0.75，0.87→2.55，0.95→3.72），但持续只有约 100 ms。
- **问题 1：跟随时人静止，机器人躯干仍有约 1 Hz、峰峰约 1° 的前后晃**（基座俯仰标准差 0.2–0.3°，约合头部 1–2 cm；比 run1 的 2–3° 小很多）。**已排除：** ① 腿参考（已固定）；② 硬件 / 吊绳 / 延迟（策略站立同样的机器人纹丝不动；而且无头 MuJoCo 里**重放这段真实参考**，仿真照样出现 0.93 Hz、占方差 77% 的俯仰晃动）；③ 参考的噪声（仿真里把参考换成**常数**后晃动没有消失，1 s 低通滤波也没有降低：0.22° → 0.17°，所以滤波修不了）；④ 手臂静态姿势（run2+run3 的 70 个手臂近乎静止的 5 s 窗口，肘参考与晃动的相关系数 +0.11 / +0.18，肘弯 > 0.8 的 4 个窗口也没有更安静）；⑤ 手碰到大腿（测得姿势下手 / 腕离身体最近 ≥ 70 mm，简化碰撞模型）。**结论：这是策略在“跟随”模式下自己的闭环行为（一个小的极限环），部署端的滤波 / 固定都去不掉。** 实用办法：不动的时候用 **A+X 的策略站立**（同一台机器人纹丝不动），要动再 A+X 回跟随；长期靠重训 / 微调策略。（常数参考的那一块在仿真里反而晃得更大：0.78° / 峰峰 8°，但块的顺序和时间是混杂的，不能据此下结论。）
- **问题 2：下半身跟踪 / 走路。** `A3_LOWER_BODY=stand` 把腿固定了，**按设计就不会走路**。要走路需要 `track`，而 `track` 现在有两个已知障碍：(a) 站着不动时参考的踝 roll 就贴关节极限（髋 roll 顶 50–60 N·m）；(b) 参考里腿的质量本身有问题。**软件管线本身能走：** 我把 SMPL 行走片段（`sample_data/smpl_filtered/walk_forward_amateur_001__A001.pkl`）经过同一条实时重定向链放进 MuJoCo，机器人在约 28 s 里向前走了 6.5 m，随后转身往回走，约 60 s 内没有摔倒。但腿参考和实测的跟踪并不好：髋 pitch 相关 0.94，膝 0.5–0.6，**踝 pitch 相关 ≈ 0 / 负，踝 roll 均方根误差 0.3 rad，髋 yaw 参考 ±1.0–1.25 rad 而实测只有 ±0.5–0.6**（脚部参考不可靠，策略大部分是靠髋 / 膝在走）。**决定性但我不知道的一点：你戴的 Pico 有没有脚踝追踪器（Motion Tracker）？** 只有头显 + 手柄时腿是估出来的，真人的腿数据不可能准，再怎么修重定向也走不好。

**真机 vs 仿真：已经排除的和还没排除的（2026-10-17）**

- 已排除：**IMU 的安装朝向**。用 7 份真机快照（腰 yaw −15° … +25°，腰 pitch −7° … −18°），把骨盆 IMU 的重力方向经 MJCF 的腰运动学（骨盆 / 躯干两个 IMU site，姿态单位四元数）换算到躯干 IMU 系，与实测躯干 IMU 重力方向比较：误差 0.2–0.4°；假设“骨盆 IMU 绕 z 转了 180°”误差 2–53°，“躯干 IMU 转了 180°”误差 1–15°。所以两个 IMU 的重力 / 角速度坐标系都与仿真用的 MJCF 一致；骨盆和躯干 IMU 的 **yaw 读数**差 180° 只是各自的航向参考不同（部署用航向补偿，真机日志里 `fallback_yaw_offset_deg` ≈ ±178°，仿真里 ≈ 0.1°，这是机器人朝向的差别，不是故障）。
- 已排除：**部署的循环时序**（真机 20.00 ms、推理 1.3 ms、端到端 2.5–9 ms）。**腿参考贴极限**不再算排除：见上面的再次更正，它现在是首要嫌疑。
- 还没排除（上面的再次更正之后，次要）：① **被控对象不同**（仿真里同一个不可行指令只引起一半幅度的指令和较小的晃动，所以真机放大了它）：仿真是关节空间的理想 PD、球形脚碰撞、无并联踝连杆、无回差、无足底形变（Litchi 的 README 也说它们的被动脚模型“与真实足底形变仍有很大差别”，sim-to-real 改进还在 TODO 里）；真机的踝是并联结构，策略踝增益只有 kp 60 / kd 1.2，可能低阻尼振荡；② **吊绳的影响**（绳子“有活动空间”，前后晃有可能是人 + 绳子的摆动叠加）；③ **操作者真实的腿 / 脚部 Tracker 数据**与合成站立不同（需要头显检查）；④ A+X 切换时**手臂**参考突变（操作者手臂姿态 → 默认姿态，肘约 0.8 rad）带动全身；仿真合成站立的手臂本来就接近默认，所以看不出来。
- 区分它们要靠数据：下一次真机跑的时候 dump 已经打开；仿真基线已经有了（腿 q_des 抖动均方根 0.08–0.30 rad/s，踝 roll 角速度均方根 0.001–0.004 rad/s，基座俯仰晃动标准差 0.04–0.18°，切换瞬态：腿指令单步 ≤0.083 rad、基座角速度峰值 ≤0.62 rad/s），真机数据用同一个脚本分析，直接比。

### 下一次真机跑：带数据记录的短测试（2026-10-17；第一次 = `real_run1` 已做，结果见上面的“再次更正”；第二次 = `real_run2`，加了 `A3_LOWER_BODY=stand`，结果见上面的 A/B 表；第三次 = 上面的“严格示范”）

**目的：** 把“真机上哪里不一样”从猜变成看数字。三个对照放在同一次运行里：PD 站立（开环保持姿态，没有策略）→ 策略站立（策略 + 默认站姿参考，没有操作者）→ 跟随（策略 + 操作者）。判据：
- **PD 站立就晃** → 不是策略，是机械 / 吊绳 / 零位 / PD 增益。
- **PD 站立稳，策略站立晃** → 策略 + 真机被控对象的问题（和参考、操作者无关）。
- **策略站立稳，只有跟随晃** → 参考 / 操作者的问题（这时再试 `A3_LOWER_BODY=stand`）。
- **只有切换瞬间抖** → 看是手臂还是腿在抖（见下面第 9 步要你记的），再看 dump 里的瞬态。

**两份数据：** ① 部署的每 tick dump（已打开：`dist/a3_deploy_x86_64/config/a3_runtime_config.yaml` 里 `logging.dump_ticks: 60000`，写到仓库根的 `dump_teleop_real.bin`；只在策略运行时写，即策略站立和跟随，**不含 PD 站立**；**每次运行都会覆盖上一次，跑完先改名**）；② 中继状态流的 200 Hz 录制（`a3_relay_tool.py record`：全部 31 个槽位的 q / dq / tau、两个 IMU、中继模式，含 PD 站立阶段）。

**前提：** 和“快速流程”一样的全部条件（吊绳不撤、两个急停手、第二个人、周围清空、电池）。**吊绳要给到“脚承重但绳子刚好松弛”**，不要比上次更松。

**终端：** M2 停 `mc`、M1 中继、P1 部署、P2 Pico manager、P3 录制。命令见“快速流程”第 1–4 步，**多出来的只有 P3 和第 9 步**：

1. M2 停 `mc`；M1 起中继（快速流程步骤 1、2，预检 0 FAIL）。
2. **P3 先开录制**（中继起来之后；录 420 s，Ctrl-C 也会保存）：
```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_relay_tool.py record --host 10.42.10.12 --seconds 420 --out real_run1.npz
```
3. P2 起 Pico manager（快速流程步骤 4）。
4. **P1 起部署，然后马上开始后面的步骤**（dump 的上限是 60000 个策略 tick = 20 分钟，足够；但别让部署空等很久）。**下一次（A/B）在命令最前面加 `A3_LOWER_BODY=stand`**；`real_run1` 没有加：
```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
A3_LOWER_BODY=stand ./deploy.sh --robot a3_ultra --input-type zmq_manager real --encoder a3-fast --teleop-input zmq --telecom zmq --relay-host 10.42.10.12
```
5. **PD 站立（对照 ①）：** 手柄 A+B+X+Y（或 P1 按一次 `s`）。吊绳**分档放下**：每次 1–2 cm，停 10 s，直到脚完全承重、绳子刚好松弛。**在 PD 站立下站满 30 s 不动**，看有没有前后晃（有的话马上告诉我，这就是“PD 站立也晃”）。
6. **策略站立（对照 ②，没有操作者）：** 操作者摆成标定姿态，站直、双脚并拢、面朝机器人朝向，A+X 进跟随后**立刻再按 A+X**（进入策略站立）。这一步里策略吃的是默认站姿参考，和操作者无关。**站满 30 s 不动**。
7. **跟随（对照 ③）：** 再按 A+X（跟随），操作者**站着不动 30 s**，不要做手臂动作。
8. 再按 A+X（策略站立）10 s，再按 A+X（跟随）10 s，然后 A+B+X+Y 失能（吊绳承重）。
9. **你要记下来的（用手机备忘录就行）：** 每次切换时**哪里在抖**（手臂 / 腿 / 整个身体）、抖多久、是衰减还是越来越大、前后晃的大概幅度（厘米）；任何时候出现越晃越大 → 立刻 A+B+X+Y。
10. 收尾：P1、P2 里 Ctrl-C（**用 Ctrl-C 退出部署，dump 才会正常关闭**），P3 的录制 Ctrl-C，M1 中继 Ctrl-C，M2 `start_app mc`（快速流程步骤 9）。然后**立刻改名**：
```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
mv dump_teleop_real.bin dump_real_run1.bin
ls -l dump_real_run1.bin real_run1.npz
```
11. 自己先看一眼（只读，不接机器人，会打印每段的踝 / 膝均值、前后晃的幅度和频率、基座角速度、指令抖动、跟踪误差、每次切换后 3 s 的瞬态，加上 `--record` 还有腿部力矩占关节上限的比例）：
```bash
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_analyze_teleop_run.py dump_real_run1.bin --record real_run1.npz | tee analysis_real_run1.txt
```
把 `analysis_real_run1.txt`、P1 的完整输出（带 `[mode]` / `[pico-cmd]` 的行）、第 9 步的笔记发给我。仿真基线用同一个脚本分析得到，我会并排比。

**注意：** 分段用的是策略收到的**参考**（从 dump 的观测里解出来）：参考等于部署的“策略站立”参考（`teleop_motions/..._stand_Skeleton0.csv` 的均值，29 个关节都在 0.08 rad 以内）的 tick 算策略站立，其余算跟随（`--stand-tol` 可调）。（第一版用肘角指令分段，真机上标反了，已改。）注意：加了 `A3_LOWER_BODY=stand` 之后跟随时腿参考等于站立参考，分段靠手臂 / 腰的差别区分；操作者手臂也站得和站立参考一样时会被标成策略站立，需要时看 `ref` 的时间线。PD 站立阶段不在 dump 里，要看 `real_run1.npz`（里面有 mode 和 tau）。

**可以直接试的开关（腿固定为策略站立的参考，手臂 / 腰仍然跟随操作者）：**
```bash
A3_LOWER_BODY=stand ./deploy.sh --robot a3_ultra --input-type zmq_manager real --encoder a3-fast --teleop-input zmq --telecom zmq --relay-host 10.42.10.12
```
默认是 `track`（腿跟随操作者，行为和以前一样）。`stand` 时跟随模式和策略站立用**同一个**腿参考，A+X 切换没有腿的跳变；代价是机器人不再复现操作者的腿（下蹲、迈步）。试这个开关就是在真机上做 A/B：如果下半身变稳、切换不抖，说明原因是腿参考；如果还是晃，说明是真机动力学 / 延迟 / 支撑的问题，要看 dump。

**不接机器人先看真人头显的腿参考（5 分钟，零风险）：** 只开 Pico manager、重定向链和监视，不开部署、不开中继：
```bash
# P2
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl" && source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --robot a3_ultra --manager --num_frames_to_send 10
# P1（只起链，不起部署）
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl" && gear_sonic_deploy/scripts/a3_operator_chain.sh --encoder a3-fast --no-bridge
# P3
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_ref_monitor.py --seconds 45 | tee ref_monitor_real_headset.txt
```
戴头显站直不动，按 A+B+X+Y、再 A+X 进 POSE，**站着不动 30 s**，再慢慢抬一次左脚、一次右脚（各约 2 s）。看输出里 `per group` 的两行腿：站着不动时 `std` 应当约 0.01 rad 以内，`mean − default` 应当接近 0（踝 roll 不应该贴着 ±0.35）。

**每 tick 的 dump（已打开）：** `dist/a3_deploy_x86_64/config/a3_runtime_config.yaml` 的 `logging.dump_ticks: 60000`（备份 `a3_runtime_config.yaml.bak_before_dump`；测完设回 0）：obs / 原始动作 / q_des / 实测关节位置 / 速度 / 重力方向 / 角速度，路径 `dump_teleop_real.bin`（仓库根）。计数从部署启动算起，所以测试前再启动部署。

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
| Docker | **已装**（Docker 29.1.3，`/var/run/docker.sock` 建于 2026-10-16 02:40，不是我装的）。我这个会话的用户不在 `docker` 组里，连不上守护进程（`permission denied`），所以我不能自己再构建；下面“装 Docker”的命令只在换机器时用 |
| **aarch64 包** | **已构建**：`gear_sonic_deploy/dist/a3_deploy_rockchip/`（2026-10-16 02:45–02:47，不是我构建的；含 aarch64 的 `a3_zmq_relay`、`a3_deploy_onnx_ref`、`run_a3_relay.sh`、ONNX Runtime 1.19.2、`librknnrt.so`、035 的 g1 / a3_fast 的 `.onnx` 和 `.rknn`）。核对过 ELF 是 ARM aarch64、需要 GLIBC ≤ 2.34 / GLIBCXX ≤ 3.4.30（MDU 是 Debian 12，glibc 2.36，应满足，**没有在 MDU 上跑过**）。**包里没带 `libzmq.so.5`**，而 `a3_zmq_relay` 和 `a3_deploy_onnx_ref` 都要它（Litchi 自己的部署二进制也一样依赖它、也不带，所以多半是 MDU 系统自带的；**没有核实**）：上 MDU 前先在包目录里 `ldd ./a3_zmq_relay \| grep 'not found'`（只读），有输出就要补库 |
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

> **智元 2026-10-16 给出了关闭运控的官方方式**（plan 18.18）：`curl -i -H 'content-type:application/json' -X POST 'http://127.0.0.1:50080/json/stop_app' -d '{"app_name": "mc"}'`，恢复用 `start_app`——不用改 `sm_config.yaml`、不用重启 `agibot_pm`。**尚未在真机上验证**，而且智元的 readme 里写的地址是 `10.42.10.10:50080`（HDU），与这里的 `127.0.0.1` 不一致；先问清楚是哪台机器，再用。下面 Litchi 的 `sm_config` 做法保留作对照。无论哪种，停掉 `mc` 之后机器人没有智元的站立 / 平衡，只能吊起来做。

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

## 4.4 选路线：`--telecom iceoryx|ros2|zmq`

三条机器人链路用同一个开关选，其余参数（`--encoder`、`--teleop-input` 等）不变。`deploy.sh --robot a3_ultra … real|sim` 会把它交给 `scripts/deploy_a3.sh`：

| `--telecom` | 路线 | 部署进程在哪 | 机器人侧要什么 | 失联以后 | 旧写法 |
|---|---|---|---|---|---|
| `iceoryx` | A | MDU 上（aarch64 包） | 关 `mc`；没有别的 | 部署进程自己的看门狗 | 真机不写参数（`A3_TRANSPORT` 决定） |
| `ros2` | B | 本机 PC | 关 `mc`，并且 MDU 的 `hal_ethercat` / `hal_elink` 配置里给 `/body_drive/*` 加了 ros2 通信形式（智元的 `joint_demo`，plan 18.18） | **只有 `hal_ethercat` 自己的超时行为（智元没回答）** | `--remote`（真机）；仿真默认就是它 |
| `zmq` | C | 本机 PC | 关 `mc`，MDU 上跑 `a3_zmq_relay` | **中继兜底：60 ms 后重发，再 150 ms 保持，再 300 ms 渐变到阻尼并锁存** | `--robot-link zmq` |

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
# B：PC 经 ROS 2（要 ROS_DOMAIN_ID、ROS_LOCALHOST_ONLY=0；先过只读的门）
./deploy.sh --robot a3_ultra --input-type zmq_manager real --telecom ros2 --ros-iface-ip 10.42.10.99 --encoder a3-fast --teleop-input zmq
# C：PC 经 ZMQ 中继（MDU 上先启动 a3_zmq_relay）
./deploy.sh --robot a3_ultra --input-type zmq_manager real --telecom zmq --relay-host 10.42.10.12 --encoder a3-fast --teleop-input zmq
# A：在 MDU 上、用 aarch64 包里的 run_a3.sh（不写 --telecom，或 --telecom iceoryx）
```

规则：`sim` 的默认是 `ros2`（MuJoCo 仿真是 ROS 2 对端），`--telecom iceoryx` 连不上仿真会直接报错；`--telecom zmq` 必须同时给 `--relay-host` 和 `--teleop-input zmq`；`--remote` / `--robot-link zmq` 仍然可用，和 `--telecom` 矛盾时报错（`--telecom ros2 --robot-link zmq`）。横幅里会打印当前的 `telecom=…`。**三条路线都没有在真机上跑过**：B 和 C 在 MuJoCo 里验证过（B 就是仿真那条 ROS 2，C 见 4.6），A 和 C 要用的 aarch64 包已构建（0.2 表），但没有在 MDU 上运行过。

---

## 4.5 路线 B：不部署到 MDU，在本机跑 SONIC（Litchi 的 “ADU-style” 跨板 ROS 2）

> **按机器人现在的配置这条路走不通：门会拦住。智元 2026-10-16 的 `joint_demo` 说明由客户自己在 MDU 的 `hal_ethercat` 配置里给话题加 ros2 通信形式、再关 `mc`（plan 18.18），这是动力级配置，要在悬空 + 物理急停下由你或智元的人改；改完怎么生效、指令超时后的行为智元还没回答。不想等，用 4.6 的 ZMQ 中继。**

Litchi 的 `a3_aimrt_config.ros2.yaml` 就是为“部署进程不在 MDU 上、body-drive 走 ROS 2 跨板”设计的，本机用 `deploy.sh … --telecom ros2`（旧写法 `--remote`）按这个逻辑跑：

```bash
export ROS_DOMAIN_ID=232 ROS_LOCALHOST_ONLY=0        # 以厂商 env.sh 为准
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
./deploy.sh --robot a3_ultra --input-type zmq_manager real --telecom ros2 --ros-iface-ip 10.42.10.99 \
    --encoder a3-fast --teleop-input zmq
```

`--telecom ros2`（真机）的行为：`A3_TRANSPORT=ros2`；RMW 默认 Cyclone，`--ros-iface-ip` 把它钉在机器人网卡；所有 ROS 2 进程放进内存上限（`A3_MEMORY_MAX`，默认 16G）；先打印 REAL ROBOT 横幅要求确认；然后**先跑只读的门**（`a3_ros_graph_probe.py --sonic-gate`）：机器人必须发布 6 个状态话题（`/body_drive/{waist,leg,arm,neck}_joint_state`、`/body_drive/{pelvis,torso}_imu/data`）并订阅 4 个指令话题（`/body_drive/*_joint_command`），类型对、QoS 可达（我们的指令发布者是 best_effort，机器人侧任何 reliable 订阅者都收不到），**不满足就拒绝启动，什么都不发布**；通过后仍然是 IDLE 启动，按 `S` 才动。

**目前会被门挡住**：MDU 上 t3d0 的 `hal_ethercat`（plan 18.12）已经加载 ros2 后端，但 `/body_drive/*_joint_command` 的 `enable_backends` 只有 `[iceoryx]`；状态只有手臂、腰、两个 IMU 在 `/ros2/body_drive/…` 有镜像，**腿、脖子、手的状态没有 ROS 2 出口**。智元的做法（`joint_demo` readme）是：在 MDU 上对应机型的 `hal_ethercat_*_cfg.yaml` 里给这些话题添加 ros2 通信形式（手在 `hal_elink` 的配置里），话题名不变（demo 用的就是 `/body_drive/leg_joint_command`，不需要 `/ros2/` 前缀），然后关 `mc`（`stop_app`）。demo 只演示了腿；我们要的是全部 5 个指令话题和 6 个状态话题，门会逐个检查。**这是 EtherCAT 主站的配置，我不做**。

**和 4.6 的区别**：B 不需要在 MDU 上放我们的进程（不用 aarch64 包），但要改机器人配置，并且 PC 或网线出问题时，机器人怎么办完全取决于 `hal_ethercat`（没有答案）；C 不改机器人配置，失联保护由我们的中继负责，但要在 MDU 上放一个 aarch64 进程。选哪条见 4.4。ZMQ 同时也用在**遥操参考**这一段（0.5）。

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

**上线步骤（路线 C，按顺序；每一步通过再做下一步）**

*第 0 步：在本机（PC），不碰机器人。*
- 要拷的东西是 `gear_sonic_deploy/dist/a3_relay_rockchip.tar.gz`（6.5 MB，只有中继 + AimRT 插件 + 配置 + 预检，没有模型和策略；`.sha256` 在旁边）。生成方法：`python3 gear_sonic_deploy/scripts/a3_make_relay_bundle.py --pkg gear_sonic_deploy/dist/a3_deploy_rockchip`（用 `--check` 只看文件清单和“必须由 MDU 提供的库”）。
- 操作端预检：`gear_sonic_deploy/scripts/a3_real_preflight.sh --operator --relay-host 10.42.10.12`（C 不查 ROS 2；会提示“中继还没起”，这是对的）。

*第 1 步：MDU 上只听，`mc` 不用停，机器人不会动。* 中继在 `--listen-only` 下不创建指令发布者，只读状态。
```bash
# 在本机，把包拷到 agi 的家目录（不要放进 /agibot 下面；密码由你输入）
scp gear_sonic_deploy/dist/a3_relay_rockchip.tar.gz gear_sonic_deploy/dist/a3_relay_rockchip.tar.gz.sha256 agi@10.42.10.12:~/

# 在 MDU 上：先单独执行 ssh、输完密码、看到提示符之后，再粘贴下面的命令（和 ssh 一起粘贴会被密码提示吞掉）。
# 登录后默认目录是 /agibot（厂商目录）：先 cd ~，不要在 /agibot 下解压或创建任何东西
cd ~ && sha256sum -c a3_relay_rockchip.tar.gz.sha256 && tar xzf a3_relay_rockchip.tar.gz && cd a3_relay_rockchip
export A3_ROBOT_ENV=/agibot/software/v0/entry/env/env.sh
./a3_real_preflight.sh --relay --bind-ip 10.42.10.12 --listen-only    # 只读；有 FAIL 先处理，别往下走
./run_a3_relay.sh --bind 10.42.10.12 --allow-peer 10.42.10.99 --listen-only
```
预检里最可能出 FAIL 的是 **`libzmq.so.5` 缺失**（包里没带，要装系统的 libzmq5，需要 root / 厂商）和**端口 5570 / 5571 / 50901 被占**。中继每 2 秒打印一行状态。然后在**本机**：
```bash
gear_sonic_deploy/scripts/a3_real_preflight.sh --operator --relay-host 10.42.10.12     # 现在应该 PASS “accepts connections on 10.42.10.12:5570”
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_relay_tool.py listen --host 10.42.10.12 --seconds 10
# 需要看全精度时（每个槽位的 q 均值 / 标准差 / 最小 / 最大、dq、tau、两个 IMU）：
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_relay_tool.py snapshot --host 10.42.10.12 --seconds 10
```
期望（`--listen-only` 下源码保证）：`mode=DISARMED`、`fwd=0`（`fwd=1` 只在没加 `--listen-only` 时出现）；帧序号递增；`valid=1`；`age` 是几毫秒；`q[…]` 不是全 0 也不是 NaN。`imu2=1` 表示骨盆和躯干两个 IMU 都有数据，若是 0，说明只有一个在到——我没有依据说这台机器应该是几，看到 0 时记下来问厂商，不要忽略。`valid=0` 或 `age` 很大说明 iceoryx 那一侧没接上，先不要往下走。听够了在 MDU 上 Ctrl-C 结束中继。

*第 1c 步：急停试验（`mc` 还在跑，不发任何指令，预期要重启整机一次）。*
先满足：**机器人完全挂在龙门架上、脚离地、绳子绷紧承重**；周围没有人和物（四肢会垂下来）；一个专人拿着遥控器、另一个人在旁边；遥控器屏幕显示已连上机器人、电量足。**脚着地时不要按。**
```bash
# 本机终端 P1：重启前的基线（只读；2026-10-16 修了脚本里的两个 bug，之前生成的 mdu_baseline_before_estop.txt 作废，要重新生成）
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
ssh agi@10.42.10.12 'bash -s' < gear_sonic_deploy/scripts/a3_mdu_baseline.sh > mdu_baseline_before_estop.txt; wc -l mdu_baseline_before_estop.txt

# MDU 终端 M1：只听的中继（mc 不停）
cd ~/a3_relay_rockchip && export A3_ROBOT_ENV=/agibot/software/v0/entry/env/env.sh
./run_a3_relay.sh --bind 10.42.10.12 --allow-peer 10.42.10.99 --listen-only

# 本机终端 P2：先等中继起来 5 秒以上（否则前 0.7 s 的帧是无效的），再采急停前的全精度读数，然后录 90 秒（录的过程中按急停）
# （listen 的输出现在是逐行刷新的；之前 `| tee` 时输出被管道缓冲、看起来像卡住——已修）
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_relay_tool.py snapshot --host 10.42.10.12 --seconds 10 | tee snapshot_before_estop.txt
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_relay_tool.py listen --host 10.42.10.12 --seconds 90 | tee listen_during_estop.txt
```
`listen` 开始约 10 秒后，专人在遥控器上**同时按住 L1+L2+R1+R2 超过 0.5 秒**。观察并记下：遥控器 / AimMaster 的界面文字；四肢怎么动；`listen` 里 `valid`、`age`、`q` 的变化（帧是否还在、关节读数怎么变）；中继是否报错。然后用厂商的**正常**方式重启整机（**不要强制关机**）。开机后：`ssh` 登录，确认 `~/a3_relay_rockchip` 还在，跑 `a3_mdu_baseline.sh > mdu_baseline_after_estop.txt` 并 `diff -u mdu_baseline_before_estop.txt mdu_baseline_after_estop.txt`；记下重启花了多久、`mc` 是否自动恢复运行（`./a3_real_preflight.sh --relay --bind-ip 10.42.10.12 --listen-only` 里 `mc` 应为 WARN）。把这些贴回来，再进第 2 步。

*第 1c 步结果（2026-10-16，plan 18.22）：遥控器急停触发后 body-drive 的状态流在不到 0.2 s 内全部停止并持续到重启；重启后配置和我们的文件原样、`mc` 自动恢复（比 ssh 晚几十秒）。机器人在“初始”状态（`mc` 的待机，不站立）。*

*第 2a 步：只停 `mc`，不放行任何指令，看机器人和状态流怎么反应，再用 `start_app` 恢复（同时验证“恢复”这条路）。前提和第 1c 步一样：机器人完全挂着、脚离地、绳子承重；专人拿着遥控器；第二个人；周围清空；期间不要按 AimMaster / 遥控器上的“站起”或任何技能。*
```bash
# MDU 终端 M1：只听的中继
cd ~/a3_relay_rockchip && export A3_ROBOT_ENV=/agibot/software/v0/entry/env/env.sh
./run_a3_relay.sh --bind 10.42.10.12 --allow-peer 10.42.10.99 --listen-only
# 本机终端 P2：录 150 秒（逐行刷新；必须先进仓库目录，.venv_teleop 是相对路径）
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_relay_tool.py listen --host 10.42.10.12 --seconds 150 | tee listen_stop_mc.txt
# 第二个 MDU 终端 M2：先确认 mc 在跑；listen 稳定 15 s 以后再停；等 30 s；然后恢复
pgrep -a -x mc | cut -c1-90          # 进程名（comm）是 mc，命令行才是 ./motion_control；pgrep -x motion_control 匹配不到
curl -sS -i -H 'content-type:application/json' -X POST 'http://127.0.0.1:50080/json/stop_app' -d '{"app_name": "mc"}'
sleep 3; pgrep -a -x mc || echo "mc stopped"
#   ……观察 30 秒（listen 的 valid / age；机器人；AimMaster 上的状态文字）……
curl -sS -i -H 'content-type:application/json' -X POST 'http://127.0.0.1:50080/json/start_app' -d '{"app_name": "mc"}'
sleep 5; pgrep -a -x mc
```
*停止条件：状态流停了（`valid=0`、`age` 一路涨）、机器人有任何动作、AimMaster 出现故障提示——不要再继续，告诉我；需要时整机重启。*

*第 2a 步结果（2026-10-16 21:56–21:58，plan 18.22）：`stop_app` / `start_app` 都返回 `200 {"message":"stop APP success","status":"success"}` / `start APP success`；`mc` 停后 3 s 进程消失、恢复后 5 s 以新进程号回来；同一段 `listen`（108 s、21,573 帧）`valid=1` 全程、状态年龄 ≤ 2.96 ms、关节读数不变、只丢 1 帧（86.7 s 处）——**停 / 起 `mc` 不影响 `hal_ethercat` 的状态发布**。进程名是 `mc`（命令行才是 `./motion_control`）。*

*第 2b 步：停 `mc`，只放行中继（仍然不发任何指令）。进入前的硬性条件（缺一不做；厂商指南 6.1 也要求“safety tether、emergency stop、现场监督”，且说 `P` 只是软件兜底、不能代替物理急停）：*
- *机器人**脚离地、龙门架绳子绷紧并承担全部重量**（不是“脚着地站着、绳子松着”：`mc` 一停，`hal_ethercat` 没有指令时怎么处理智元没回答，可能保持、阻尼或掉使能，站着的机器人可能直接倒）；*
- ***急停 = 智元的“软急停”**（智元文档 4.6 / 4.6.1，2026-10-16 用户截图；除电源键外没有别的物理切断方式）。**触发**：方式一，AimMaster 点悬浮球 → 按住【紧急停机】按钮向右滑到底（弹窗写着“急停可能损坏机器人，右滑以确认执行”）；方式二，**遥控器已连上机器人时，同时按住 L1+L2+R1+R2 超过 0.5 秒**。**效果**：界面显示“机器人急停中 / 重启后再操作”，**四肢下电失能、机器人会摔倒**，有一定风险损坏机器；**恢复 = 重启机器人整机**（用户确认）。注意它是“软”急停：走厂商软件栈，不是独立于 MDU 的硬件断电（电源键才是，且是全部下电；强制关机有风险）。要求：遥控器和 AimMaster **在专人（不是操作员）手里**、电量足、已连上；**机器人必须挂在龙门架上、绳子承重、脚离地**（急停 = 瘫软）；放行指令前先做第 1c 步的急停试验；*
- ***电源键只是最后手段**：它是“全部下电”（电机和 MDU 一起）；正常关机没有风险但不要太频繁，**强制关机有一定风险**（MDU 文件系统）。不要把它当急停用；*
- *旁边有第二个人；*
- *放行后第一次 PD 站立（悬空）稳定下来，**再按一次遥控器急停**，确认它在“我们的指令正在发送”的状态下同样有效。*

```bash
# 在 MDU 上；MDU 基线里 127.0.0.1:50080 确实在 MDU 本机监听，与智元售后一致；readme 里的 10.42.10.10 是别的板，不用
curl -i -H 'content-type:application/json' -X POST 'http://127.0.0.1:50080/json/stop_app' -d '{"app_name": "mc"}'
./a3_real_preflight.sh --relay --bind-ip 10.42.10.12                  # 这次不带 --listen-only：“mc 没在跑”必须是 PASS
./run_a3_relay.sh --bind 10.42.10.12 --allow-peer 10.42.10.99
```
```bash
# 本机终端：看 60 秒（预期 mode=DISARMED、valid=1，这次 fwd=1：中继已允许转发，但本机还没有发过任何指令）
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_relay_tool.py listen --host 10.42.10.12 --seconds 60 | tee listen_2b.txt
```
停掉 `mc` 之后机器人没有智元的站立 / 平衡保护，只靠吊架。**`hal_ethercat` 在指令中断后的行为智元还没回答**：不要把退出中继当停机手段。

*第 2c 步：本机启动部署，IDLE（仍然不发任何指令）。先停掉旧的仿真会话（它占着 5560）。只开部署终端；Pico 终端留到第 2d 步。**部署终端里不要按任何键**（键盘是活的：`S` 会让机器人开始 PD 站立）。预期：部署打印 `startup IDLE, no commands sent`；中继状态行 `cmd_rx=0/s published=0`；`listen` 里 `fwd=1 mode=DISARMED`。看够 60 秒后在部署终端 Ctrl-C。*
```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
./deploy.sh --robot a3_ultra --input-type zmq_manager real --encoder a3-fast --teleop-input zmq --telecom zmq --relay-host 10.42.10.12
```
（会打印 REAL ROBOT 横幅并要求确认 `[y/N]`。）

*第 2d 步前的预演（离线、只读）：PD 站立会把所有关节从当前姿态**线性**拉到默认站姿（3 s），部署里没有自碰撞检查。先用刚采的 `snapshot` 看每个关节要走多远、手臂会不会撞身体：*
```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_relay_tool.py snapshot --host 10.42.10.12 --seconds 5 | tee snapshot_before_2d.txt
.venv_sim/bin/python -I gear_sonic_deploy/scripts/a3_pd_stand_preview.py snapshot_before_2d.txt --collision
```
*行程大于 0.5 rad 的关节、间隙为负或只有几毫米的手臂对身体的连杆对，都要先处理（用手把姿态摆近默认站姿，或确认周围空间）。*

*第 2d 步（**要我放行**；前提：电池够用、2b / 2c 正常、预演看过、下面 5 项物理条件你逐项答“是”）：第一次指令——悬空 PD 站立。**不需要 Pico 管理脚本**（遥操那一步才要；现在开它只会多一条能起站的路径）。部署已在 2c 里以 IDLE 跑着，**不要重启它**。*

*物理条件：① 脚离地、吊绳已受力；② 遥控器（L1+L2+R1+R2）和 AimMaster 的急停各有人拿着；③ 另有一人在旁；④ 除急停手外所有人在手臂 / 腿的摆动范围（约 1 m）之外；⑤ 机器人周围一圈是空的。*

*2d-1（部署终端，按一次 `s`，不要回车）：预期打印 `[mode] idle -> pd_stand`，3 s 后 `Init Done`；中继状态行变 `ACTIVE`，`acc` 增长、`cmd_rx≈50/s`、`rej=0`；另开一个本机终端跑 `listen`，看 q 走向默认值、tau 变化。保持约 20 s，看有没有异响 / 抖动 / 某个关节不动。*

*中止：`p`（被动：kp = kd = 0，肢体松弛）、或 Ctrl-C 部署（中继 60 ms 后 HOLD，再 150 ms 后 DAMP）；有任何不对劲直接急停。*

*（2d-1 / 2d-2 已在 2026-10-16 做过一次：成功，见 plan 18.x “第 2d-1 步结果”。）下面是还没做的：*

*（2d-2b / 2d-3 已在 2026-10-16 23:10 做过：通过，见 plan 18.x “第 2d-2b / 2d-3 结果”。）2d-2b：`p` 之后再按 `s`（同一个部署进程）。`NotifyEnable()` 会换新 session，而 `p` 期间中继一直是 ACTIVE（被动命令是合法命令），旧 owner 还在，所以新 session 的头几帧会被拒（`other_session`），中继随后 HOLD → DAMP（约 0.2 s；此时机器人本来就松弛，阻尼无害），再接受新 session（首帧要在测得位姿 0.35 rad 以内）。后端给 1.5 s 宽限，超时部署自己回 passive。预期：部署打印 `robot enabled: new session …`，`relay mode HOLD/DAMP`，然后 `ACTIVE`，`Init Done`。这条路径没有测过，吊着、松弛状态下测。*

*2d-3：PD 站立 `Init Done` 后，在部署终端 Ctrl-C。`A3ZmqBackend::Stop()` 不发任何东西，所以这就是纯静默的断线：中继 60 ms 后 HOLD（保持最后一条 PD 指令 150 ms），再 300 ms 把 kp 渐变到 0、kd 到阻尼值（DAMP，锁存），机器人在吊绳上从“站姿”变成“阻尼松弛”。之后中继一直在发阻尼指令（所以这一步**不能**回答厂商的 hal_ethercat 超时问题）。要继续，先在 MDU 上 Ctrl-C 中继、重启中继，再重新起部署（新 session）。*

*2d-5（可选，要用户明确说“做”；2d-4 之前）：悬空 PD 站立时，在 MDU 上 `kill -9` 中继，回答 `hal_ethercat` 在指令发布者消失时是保持最后一条（刚度）指令、掉使能还是别的行为。中继 Ctrl-C 总是先阻尼 2 s 再退出，所以只有崩溃才能测到。流程：新部署进程接管仍锁存 DAMP 的中继（新 session，位姿在 0.35 rad 内）→ `s` → `Init Done` 后保持约 10 s → MDU 第二个终端按 **PID** 杀中继 → 看机器人约 15 s：保持 / 松弛 / 漂移。部署会因状态丢失自己回 passive（这时它发零增益，但没有人转发了）。**不要用 `pkill -x a3_zmq_relay`**：2026-10-16 23:2x 第一次做 2d-5 时它什么都没杀（中继的内核进程名是 `aimrt_main`（AimRT 把主线程改了名；MDU 上 `ps -eo pid,ppid,stat,comm,args` 确认：`116695 … aimrt_main /home/agi/a3_relay_rockchip/a3_zmq_relay --aimrt-cfg …`），所以按名字匹配永远匹配不到，必须按命令行匹配），中继活着，那次“保持僵硬”其实是在指令还在流动时看到的，作废。正确做法：**一次粘贴、先确认再杀**（2026-10-16 两次因为 `$RP` 为空没杀成，所以把“取 PID → 校验 → 等人确认 → 杀 → 验证”合成一块，不再依赖前一步的变量）：`RP=$(pgrep -f '^/home/agi/a3_relay_rockchip/a3_zmq_relay '); echo "pid: [$RP]"; case "$RP" in ''|*[!0-9]*) echo "ABORT: need exactly one numeric pid";; *) read -p "Type KILL and Enter to kill relay $RP: " a; if [ "$a" = KILL ]; then kill -9 "$RP"; sleep 1; if kill -0 "$RP" 2>/dev/null; then echo "STILL ALIVE"; else echo "relay killed (pid $RP)"; fi; else echo "not killed"; fi;; esac`（在本机用假进程测过：没有进程 / 两个进程 → ABORT；答错 → not killed；答 KILL → relay killed）。**不要用 `ps -p "$RP" || echo killed`**（`$RP` 为空时 `ps` 报用法错误、退出码非 0，会打印假的 “killed”）。如果机器人一直僵硬地保持：MDU 重启中继（不带 `--listen-only`）、本机重启部署、`s` 之后按 `p`（新中继 DISARMED，首帧在测得位姿 0.35 rad 内被接受）；或急停。观察：松弛 / 保持用眼睛看；之后用 `--listen-only` 的中继读 q 和 tau（刚度保持时重力承载的髋 / 膝 tau 会明显不为 0）。*

*2d-4（最后做，因为急停之后要整机重启）：命令流动时按第二次急停。*

*收尾 / 恢复：* 先在本机停部署，再在 MDU 上 Ctrl-C 中继（它会先阻尼 2 s），最后恢复运控：`curl -i -H 'content-type:application/json' -X POST 'http://127.0.0.1:50080/json/start_app' -d '{"app_name": "mc"}'`（同样先确认地址）。

`--bind` 必须写具体的机器人网地址（拒绝 `0.0.0.0`）；`--allow-peer` 只收这个地址发来的指令帧（ZMQ 没有鉴权，这是第二道门；第一道是 `--bind`，第三道是上面的校验）。其它选项见 `a3_zmq_relay --help`（`--stale-ms`、`--hold-ms`、`--ramp-ms`、各种限幅、`--rt-priority`）。中继退出时会打印本次最大的 q_des 偏差和帧间步长，用它们收紧 `--max-q-err` / `--max-q-step`。

**在本机**：

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
./deploy.sh --robot a3_ultra --input-type zmq_manager real --encoder a3-fast --teleop-input zmq \
    --telecom zmq --relay-host 10.42.10.12
```

`--telecom zmq`（旧写法 `--robot-link zmq`）要求 `--teleop-input zmq`（部署进程里没有 AimRT，`/ta` 订阅不存在）和 `--relay-host`；`sync_mode` 必须是 `latest_cache`、`phase_align_to_sync` 必须是 `false`（默认配置就是这样）。启动后是 IDLE，A+B+X+Y（或 `S`）才站立；横幅会提醒“中继必须已经在跑”。本机终端里会打印 `[a3_zmq] relay mode …` 的变化。

**没有机器人时的联调**：`scripts/a3_relay_sim.sh`（中继接 MuJoCo 仿真，ROS 域 87）+ `deploy.sh … sim … --telecom zmq --relay-host 127.0.0.1`。`a3_relay_tool.py fake-pc` 是一个不带部署的假 PC：按实测姿态保持几秒，然后静默或发垃圾，用来看中继的保护。**它会真的通过中继给仿真 / 机器人发指令**，只在仿真或机器人吊着时用。

**限制与没有验证的**：
- **手（O10）不转发**（`hand_bridge_enabled=false`，和默认运行配置一致）；`--probe` / `--dry-run` 不发指令。
- 部署进程不是实时进程：本机到机器人的这一段必须走有线网，Wi-Fi 不行。10 ms 的新鲜度只用在中继本地；本机侧用 `max_frame_age_ms`（50 ms）。
- 中继退出之后 `hal_ethercat` 在没有指令时的行为**厂商还没回答**（plan 18.12 的问题 3）：不要把退出中继当停机手段，带电时保持中继在跑。
- 阻尼值（`a3_024` 的策略 kd 表）和各个限幅是**初值**，没有在真机上标定；第一次一定在吊着、急停在手的状态下试，并故意断线一次看 `HOLD → DAMP`。
- aarch64 包已构建（0.2），但中继没有在 MDU 上跑过，包也没有在 MDU 或 aarch64 模拟器里运行过；iceoryx 那一跳是现有代码（`A3AimrtBackend`），中继本身只在 x86 + MuJoCo 上验证过。
- 中继和带 AimRT 的完整部署（`run_a3.sh` 的默认）**不能在同一台机器上同时运行**：两者的 AimRT 配置都监听 `127.0.0.1:50901`，后启动的会失败（预检 `--relay` 会查 50901 和 `a3_deploy_onnx_ref`）。另外，包里的 stock 配置还会让中继加载 `ros2_plugin`（在厂商的 ROS 域里多一个名为 `a3_deploy_onnx_ref` 的节点）和 `net_plugin`；这和厂商验证过的部署一致，所以第一次不改它。以后想精简，可以做一份只含 iceoryx 插件的中继专用配置（没做，也没测）。
- **厂商的前提**（2026-10-16 的两次回复，plan 18.17 / 18.18）：开放 `body_drive` 的前提是关闭运控（`mc`）；关 / 开用上面的 `stop_app` / `start_app`。他们**没有**说“可以运行我们的指令发布程序”，只说前提是关运控；`hal_ethercat` 超时行为和 `50080` 在哪台机器上都没答复。所以第 1 步（`--listen-only`，`mc` 不停）可以先做；第 2 步起要你确认过，再去做。

---

### 4.7 从“悬空验证过”到 Pico 遥操：分阶段路线（2026-10-17）

悬空阶段只验证了**通信和失效保护**（PD 站立、`p`、接管、断线、中继崩溃、急停），**策略（a3_fast）从没有在真机上跑过**。遥操 = 策略在跑，策略需要脚着地：悬空时开遥操策略会乱挥（单帧步长 1.6–2.7 rad，q_des 离实测 4–4.8 rad，plan 18.13），**不要在悬空时按 A+X**。

| 阶段 | 做什么 | 过关标准 | 不过关就 |
|---|---|---|---|
| A 只验证 Pico 数据流（**中继用 `--listen-only`，机器人不可能收到任何指令**，`mc` 不用停） | 真 Pico 头显 + XRoboToolkit PC Service + `pico_manager_thread_server.py` + 部署（IDLE）+ `a3_ref_monitor.py`；A+B+X+Y 起 → 部署 1.5 s 后因 listen-only 自己回 passive；A+X 进 POSE，缓慢动手臂 | monitor 四项 PASS（50 Hz、有限、不越关节范围、无跳变），表里的关节符号 / 幅度和动作对得上 | 不进阶段 B |
| B 落地 PD 站立 | 吊绳**逐步放下**，脚分几档承重（25 / 50 / 75 / 100 %，每档停 10 s），吊绳始终拉着不撤；只用 PD 站立；每档用 `a3_relay_tool.py snapshot` 看 tau 和中继 `q_err_max` | 站得住、关节不抖、tau 合理（髋 / 膝 / 踝不饱和）、`q_err_max` 不上升 | `p` / A+B+X+Y 是立刻瘫软：**吊绳必须还拉着**；站不稳就停，查零位 / 增益 |
| C 策略站立 | 操作者摆成默认站姿，A+X（进遥操），再 A+X（策略站立）；先只观察 30 s | 站稳、重心不漂、关节不乱、`rej` 不涨 | A+B+X+Y 失能；吊绳接住 |
| D 小幅遥操 | 慢速抬臂、小幅度动作，逐步加大 | 跟随稳定、没有拒收 | 回策略站立或失能 |

**每个阶段都要有：** 吊绳不撤；遥控器和 AimMaster 急停各有专人；第二个人；周围清空。**已接受的风险（用户 2026-10-17 决定跳过这些项）**：① 断线后约 0.5 s 内从站立变阻尼（吊绳接住）；② 中继崩溃 = 机器人卡在最后一条指令上（只有急停 / 重启中继加 `p` / 断电能放开）；③ `hal_ethercat` 超时、腰软限位极限没有进一步验证。IMU yaw 参考已经查过：骨盆 IMU 与躯干 IMU 的 yaw 差 + 腰 yaw ≈ −179°（4 份快照一致在 1° 内），即同一个 yaw 参考加一个固定的 180° 安装差，没有各自漂移。

## 5. 第一次遥操流程

机器人**吊着**（龙门架 / 吊带受力），安全员在旁边，物理急停在手。流程和 G1 一致：

1. 终端 B 起来后、没有人按手柄时，机器人侧什么都不会动。先在部署终端里确认 `startup IDLE, no commands sent`。
2. 机器人悬空，手柄同时按 **A + B + X + Y**（或部署终端键盘 **`S`**，不要回车）→ 机器人**硬起来**：`[mode] idle -> pd_stand`，从**当前姿态**用 3 秒（150 个 50 Hz 周期）线性过渡到**默认站立姿态**，用 Litchi 真机的 PD 站立增益。完成后打印 `Init Done`。
3. **缓慢放下吊带**，让机器人自己站在地上。**站不稳 = 停**（A+B+X+Y 或 `P`），这说明硬件或标定有问题（比如关节零位不对），不要继续做任何动作。
4. 站稳后，**标定姿态**：站直、双脚并拢、看向前方；上臂贴身，前臂向前弯约 90°（肘成 L），掌心向内，保持 1–2 秒。
5. 把自己的身体摆成和机器人当前姿态接近，再按 **A + X** → `[pico-cmd] A+X -> teleop`，机器人**开始跟随**。先只做缓慢的抬臂、小幅度动作。`[a3_ta_bridge] forwarded=… rejected=0`（走 ZMQ 输入时没有这一行）。
6. 再按 **A + X** → **策略站立**（仍在 `teleop` 模式里，策略 a3_fast 继续运行，参考换成默认站姿，相当于 G1 的 POSE → PLANNER；不是开环的 PD 站立，`a3_pico_command_logic.hpp`），再按 A+X 重新跟随。（本文以前写成“回到 PD 站立、`teleop -> pd_stand`”，与代码不符，2026-10-17 更正。）
7. 结束：按 **A + B + X + Y** → **失能**（passive，kp = kd = 0，和 Litchi 一样用零增益）。**这一步机器人会瘫软**，所以吊带要重新受力 / 有人扶稳。Pico 管理脚本在这次“停止”之后会自己退出，下次要重新运行。

**失联时**（和 G1 一致：安全检查失败 → 停止，不自动恢复）：机器人状态流中断 / 过期 → `[link] robot state lost or stale … -> passive`，机器人失能；姿态流静默超过 1 秒（`teleop.stale_fallback_ms`）→ 自动回到默认站立姿态；经 ZMQ 中继时中继的失联保护锁存 → `[link] the relay is damping … -> passive`。恢复都要**重新按 A+B+X+Y**（Pico 脚本这时还以为自己在运行：第一次按键是它的“停止”，对已失能的机器人无效，再按一次才是“启动”）。

### 模式切换安全

进 POSE 之前一定先让身体与机器人当前姿态对齐；错位时切换，真机会猛然跟姿。重新进入 POSE 时（Pico 退出再进）部署端会按 G1 同一套规则重新开始会话，初始朝向会重新对齐，所以要面向与机器人一致的方向再按 A + X。

---

## 6. 急停与失效行为

按“从最可靠到最不可靠”：

| 方式 | 作用 | 说明 |
|---|---|---|
| **急停（智元“软急停”：遥控器 / AimMaster 悬浮球）** | 四肢下电失能、机器人会摔倒；**恢复要重启** | 要在专人手里，机器人必须挂在龙门架上；它是软件栈里的急停，不是独立硬件断电；电源键（全部下电，强制关机有风险）是最后手段，不是急停 |
| 部署终端键盘 **`P`** | passive：kp = kd = 0 | 要求终端是 TTY（ssh -t / 本地终端），键盘焦点在那个终端。**机器人会瘫软** |
| 手柄 **A + B + X + Y**（机器人已硬起来时） | 同上（passive，kp = kd = 0） | 走网络：Pico 服务器发**一条**停止消息就退出；这条消息丢了或网络断了就停不下来 |
| 部署终端 **Ctrl-C**（或 `kill -INT <PID>`，PID 用 `pgrep -f '(^\|/)a3_deploy_onnx_ref( \|$)'` 查；`pkill -x a3_deploy_onnx_ref` 匹配不到，进程名在内核里被截成 15 个字符） | 部署进程退出，不再发指令 | 不等于 `P`；退出之后机器人的行为取决于厂商底层，不要把它当急停 |

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

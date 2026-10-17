# MuJoCo + PICO：示范动作的录制与回放（用来验证重定向 / 部署逻辑）

**一句话：** 把真人（头显 + 脚踝追踪器）的原始 Pico 流录下来，之后**不需要真人**就能把同一份动作反复放进 MuJoCo，或者离线喂给重定向代码打分。改了重定向 / GMR 表 / 部署逻辑以后，用同一份录制前后对比，就知道改对了没有。

| 想验证什么 | 用哪一步 | 要仿真吗 |
|---|---|---|
| 重定向的腿 / 脚参考对不对（踝是否贴极限、脚位置残差、髋 yaw 范围） | 第 3 节 | 不要 |
| 重定向和离线训练参考的差距（动捕片段） | 第 3 节 `a3_eval_leg_retarget.py` | 不要 |
| 策略把参考执行成什么样（抬脚高度、步方向、打滑、躯干晃动） | 第 4、5 节 | 要 |
| 改动有没有让以上变差 | 第 6 节 | 两者 |

## 1. 约定（先读，事故都出在这里）

- **每次会话一个编号**（下面用 `leg2`），所有输出文件都带它，**永远不复用旧编号**：`pico_leg2.pkl`、`dump_sim_leg2.bin`、`sim_leg2.csv`。（`leg1` 被复用过一次，第一次运行的执行数据被静默覆盖了。）
- 先在仓库根目录，所有命令**整块复制粘贴**，不要手敲（`cd` 打成 `d`、`--no-commands` 前少一个空格都出过事）。
- 端口 5556（Pico）和 5560（参考）同一时间只能一个会话占用。开始前：`ps -eo pid,etimes,args | grep -E "run_sim_loop|a3_deploy_onnx_ref|pico_manager|a3_pose"`，旧的先 Ctrl-C。
- 录制器只保存 `pose` / `command` / `planner`（管理脚本的 `manager_status` 流每秒几万条，不录；一次 140 s 的会话应当约 80 MB，**超过 1 GB 就是不对**）。

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
ls pico_leg2.pkl dump_sim_leg2.bin sim_leg2.csv 2>&1 | head -3        # 都应当是“没有那个文件”
```

## 2. 录制（需要真人）

**2.1 这次运行专用的 dump 配置**（只改 `dump_ticks` / `dump_path`，包里的配置不动）：

```bash
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_make_dump_cfg.py --dump "$PWD/dump_sim_leg2.bin"
```

**2.2 四个终端，按顺序（录制器必须在 Pico 之前）：**

```bash
# 终端 1 — MuJoCo（窗口里按 9 放到地面）
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
source .venv_teleop/bin/activate
A3_SIM_LOG_CSV="$PWD/sim_leg2.csv" python gear_sonic/scripts/run_sim_loop.py --robot a3_ultra

# 终端 2 — 策略 + 重定向链（等终端 1 就绪；默认 track，脚旋转权重 8）
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/gear_sonic_deploy"
source scripts/setup_env.sh
A3_RUNTIME_CFG="$PWD/dist/a3_deploy_x86_64/config/a3_runtime_config.dump.yaml" ./deploy.sh --robot a3_ultra --input-type zmq_manager sim --encoder a3-fast

# 终端 3 — 录制器（只订阅，不影响别的）
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_pose_recorder.py --out pico_leg2.pkl --seconds 900

# 终端 4 — Pico 数据流
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --robot a3_ultra --manager --num_frames_to_send 10
```
终端 3 每 2 s 一行 `pose windows … (~50/s)`；约 50/s 才说明头显和追踪器在发。

**2.3 示范动作。** 用**手柄 A+B+X+Y 进 PD 站立、A+X 进跟随**（这样 `command` 消息带着时间戳进录制；键盘 `S`/`T` 不会被录下来）。每个动作开始前看手机计时，别抢：

| 步骤 | 动作 | 时长 |
|---|---|---|
| 0 | 窗口按 9 放到地面，A+B+X+Y，等 `Init Done`，站稳 5 s；A+X 进跟随 | — |
| 1 | 站姿：双脚平行与髋同宽、脚尖朝前，肘 60–75°；**一动不动** | 15 s |
| 2 | 重心左右转移（脚不离地） | 5 个来回，每个 4 s |
| 3 | 抬左脚约 15 cm，**停住**，放下 | 3 次，每次停 4 s，歇 3 s |
| 4 | 抬右脚，同上 | 3 次 |
| 5 | 左 / 右脚各抬 1 s 就放（快抬） | 各 3 次 |
| 6 | 原地踏步，每步 1 s | 20 步 |
| 7 | 向正前方迈 40 cm 再收回（地上贴一条直线 + 40 cm 标记，沿线迈）：左 3 次、右 3 次，每次停 2 s | 约 40 s |
| 8 | 向前走 4 步再退 4 步，做 2 遍 | 约 30 s |
| 9 | 原地走 20 步，站定 10 s；A+B+X+Y 失能 | — |

**边做边记** 做得和表不一样的地方（例如“第 7 步右脚只迈了一半”）。机器人摔倒没关系，窗口会自己复位，继续做；记下摔在哪一步。

**2.4 结束：** 先终端 3 Ctrl-C（保存），再停 4、2、1。检查：

```bash
ls -l --time-style=full-iso pico_leg2.pkl dump_sim_leg2.bin sim_leg2.csv      # 三个时间都应是刚才；pico_leg2.pkl 约 0.6 MB/s
```

## 3. 离线打分（不需要仿真，约 1 分钟 / 次，要 `gmr` 环境）

```bash
# 录下的真人数据：原表（权重 0）与脚旋转权重 8，同一段动作
PYTHONPATH=. ~/miniconda3/envs/gmr/bin/python gear_sonic_deploy/scripts/a3_eval_pose_recording.py pico_leg2.pkl --foot-rot-weight 0 | tee eval_leg2_stock.txt
PYTHONPATH=. ~/miniconda3/envs/gmr/bin/python gear_sonic_deploy/scripts/a3_eval_pose_recording.py pico_leg2.pkl --foot-rot-weight 8 | tee eval_leg2_w8.txt
# 动捕片段 vs 训练库里同一段动作的离线 A3 参考（不需要录制）
PYTHONPATH=. ~/miniconda3/envs/gmr/bin/python gear_sonic_deploy/scripts/a3_eval_leg_retarget.py --n 24 --foot-rot-weight 8
```
`a3_eval_pose_recording.py` 看这几行：
- **STILL：** 静止时踝 roll 均值，以及贴 ±0.349 极限的帧占比（应为 0%）；
- **ALL frames：** 全部帧里贴极限的占比，髋 yaw 范围（正常约 ±0.4 rad）；
- **foot while moving：** 人脚 → 机器人参考脚的残差（cm）、R²（越接近 1 越好）、延迟。

2026-10-17 第一次真人录制（`pico_leg1_slim.pkl`，139 s）的基准：原表 静止帧 40% 贴极限，权重 8 为 0%；脚位置残差 3.9 / 3.7 cm，R² 0.91 / 0.92，延迟 0 ms。

## 4. 回放进仿真（不需要真人）

**4.1 终端 1、2 同第 2.2 节**，只是编号换成这次回放的（例如 `leg2r`：先 `a3_make_dump_cfg.py --dump "$PWD/dump_sim_leg2r.bin"`，终端 1 的 `A3_SIM_LOG_CSV` 也改成 `sim_leg2r.csv`）。**不要开终端 3（录制器）和终端 4（Pico）**——重放是它们的替身，端口一样。

```bash
# 终端 3 — 重放（先等待，不发姿态）；trigger 文件名每次换新，已存在的文件会让它立刻开始
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_pose_replay_raw.py pico_leg2.pkl --no-commands --wait-for "$PWD/leg2r_go"
```
应当打印 `waiting for …/leg2r_go`。

**4.2 步骤（每一步等上一步的现象出现再做）：**
1. 终端 2 点一下获得键盘焦点，按 **`s`**（不要回车）→ `[mode] idle -> pd_stand`，等 `Init Done`。
2. 终端 1 的窗口按 **`9`**，机器人落地，等 5 s。
3. 终端 2 按 **`t`**（不要回车）→ **必须**看到 `[mode] pd_stand -> teleop`（还没有姿态流时机器人站着不动，正常）。没看到就停，不要往下。
4. 另一个终端：`cd` 到仓库根，`touch leg2r_go`。重放开始，**约等于录制时长**（终端 3 结束时打印 `a3_pose_replay_raw: done`）。之后什么都不用做。

录制的前约 18 s 是人站着不动，所以进跟随（第 3 步）必须在 `touch` 之前。重放结束后流停止，部署会回到站立参考（正常）。

**4.3 结束：** 终端 3 打印 `done` 后，依次 Ctrl-C 终端 2、1，然后 `ls -l --time-style=full-iso dump_sim_leg2r.bin sim_leg2r.csv`。

> **命令管道（未验证）：** 重放脚本还支持 `--command-fifo F`（`echo y > F` = A+B+X+Y，`echo x > F` = A+X），无 TTY 时用。第一次尝试里 `y` 生效（PD 站立）但 `x` 没有让部署进跟随，原因没查清，所以上面用键盘。

## 5. 分析

```bash
.venv_teleop/bin/python gear_sonic_deploy/scripts/a3_analyze_teleop_run.py dump_sim_leg2r.bin | tee analysis_sim_leg2r.txt
```
输出：每段（follow / stand，按**参考**切分）的基座晃动、腿指令抖动、最差的腿指令误差（按 Kp 折成力矩）、切换瞬态、**脚在任务空间的执行**（参考脚 vs 实际脚：残差 cm、抬脚高度、延迟）。再要看仿真里的具体量（躯干 / 骨盆左右倾角、脚底滑动、骨盆高度）直接读 `sim_leg2r.csv`（列：`t, px,py,pz, qw,qx,qy,qz, q:<29+2 个关节>`，50 Hz 的**仿真**时间）。

**对齐注意：** 仿真 `rtf < 1` 时 `sim_*.csv` 的行数少于 dump 的 tick 数（dump 按墙钟 50 Hz）。**不要按行号对齐**，用关节值匹配（取 dump 的一小段腿关节，在 csv 里找最接近的位置），首尾分别对一次看有没有漂移。

## 5b. 无人值守的整段重放 + 打分（2026-10-18）

不需要终端 1–4，也不需要你在场：

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
# 先确认没有别的仿真 / 部署在跑：ss -ltn | grep -E ':(5556|5560)\b' 应当没有输出（脚本发现端口被占会直接退出）
[A3_SIM_GAINS_JSON=增益.json] gear_sonic_deploy/scripts/a3_sim_replay_case.sh rb0 pico_leg1_slim.pkl /你的输出目录   # 约 3.6 min
PYTHONPATH=. ~/miniconda3/envs/gmr/bin/python gear_sonic_deploy/scripts/a3_score_sim_replay.py \
    --csv /你的输出目录/case_rb0/sim.csv --dump /你的输出目录/case_rb0/dump.bin --label rb0
```

- 打分脚本先打印对齐质量（csv 与 dump 的平均关节差，> 0.03 rad 说明对齐不可信）；窗口是 dump 里“参考不等于策略站立参考”的那一段（整段回放约 136 s）。
- **单次运行的噪声：** 同一录制、同一设置重复两次（`rb0`、`rb1`）：脚位置残差 6.7 / 6.8 与 6.5 / 6.7 cm，躯干 roll 标准差 3.55° 与 3.56°，倾斜 p95 12.62° 与 12.71°，基座角速度峰值 2.08 与 1.95 rad/s；**打滑 p90 噪声很大**（18.6 与 12.9 cm），打滑只能看大差别。
- 脚本只杀自己启动的进程（按 PID，部署二进制从它自己的 deploy.sh 的子孙里找）。

## 6. 用它验证一次重定向 / 部署改动

1. **先存基准：** 当前代码跑第 3 节（两行 `eval_*.txt`）和第 4、5 节（`analysis_*.txt`），文件留着。
2. **改代码**（`gear_sonic/utils/teleop/a3/retarget.py`、GMR 表选项、`a3_teleop_retarget_node.py`、部署）。
3. **单元测试：** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=. ~/miniconda3/envs/gmr/bin/python -m pytest -q tests/test_a3_teleop_retarget.py tests/test_a3_pico_session_tools.py`
4. **同一份录制再跑第 3 节**，把 STILL / ALL / foot while moving 与基准并排；动捕片段的 `a3_eval_leg_retarget.py` 也并排（平均腿 RMSE、踝 RMSE、相关）。**任何一项变差都先停。**
5. **再重放进仿真**（第 4、5 节），并排看脚的执行残差、抬脚高度、站立期滑动、躯干倾角。
6. 离线和仿真都不变差，才拿真人再做一次第 2 节确认；真机另说。

## 7. 已知的坑

- **复用会话编号 → 旧数据被覆盖**（`mv` / 程序打开文件都不会问）。编号递增，文件名里带它。
- **旧的部署终端还开着 → 5560 被占**，新会话起不来或接上旧的。开始前 `ps` 查。
- **`--wait-for` 的文件已存在 → 立刻开始重放。** 每次换新名字。
- **dump 只记策略运行的 tick**（跟随和策略站立）；PD 站立阶段不在里面，所以进跟随之前 dump 是 0 字节，正常。
- **录制里没有 `command` 消息** 说明当时用的是键盘 `S`/`T`；要让 A+B+X+Y / A+X 也被录下来，示范时用手柄。
- **重放提前结束（已查清，2026-10-18）：** 录制里**到达时间**有停顿（`pico_leg1_slim.pkl` 每 30 s 一次约 4.8 s 的空白：25.6、55.6、85.6、115.6 s，帧编号却是连续的，帧没丢，是录制器当时卡住了），按到达时间重放会让部署看到一条“死流”，掉回站立参考，不再跟随——所以上次只动了约 20 s。现在 `a3_pose_replay_raw.py` 默认 `--timing frame`（按帧编号 / 50 Hz 发送，间隔恒为 20 ms）；要复现原来的停顿用 `--timing arrival`。重放没跟到最后时，先看终端 3 的最后几行。
- **部署终端的日志是块缓冲的**（重定向到文件时）：`[mode] pd_stand -> teleop` 这类行会晚很久才出现，不能用“日志里有没有这一行”判断刚才的 `x` 成没成；看 dump 有没有数据（跟随 / 策略站立才写）。
- **无人值守脚本里别在重放进程退出后往命令 fifo 写：** 没有读者的 fifo 写入会永远阻塞（我的第一版脚本就这样挂住了，仿真和部署一直跑着）。
- **录制文件超过 1 GB：** 录制器是旧版（没过滤话题），更新后重录；旧文件可以用 `pickle` 只保留以 `pose` / `command` / `planner` 开头的消息。
- **`pgrep -x` / `pkill -x` 匹配不到（进程名被截断或改名），结束进程一律先 `ps` 看清楚 PID。**

## 8. 工具一览（都在 `gear_sonic_deploy/scripts/`，测试在 `tests/`）

| 工具 | 作用 |
|---|---|
| `a3_pose_recorder.py` | 只读订阅 5556，保存 `pose` / `command` / `planner` 原始字节 + 到达时间 |
| `a3_pose_replay_raw.py` | 重放（`--timing frame\|arrival` `--speed` `--start` `--end` `--loop` `--no-commands` `--wait-for` `--command-fifo`） |
| `a3_sim_replay_case.sh` | 无人值守：无头 MuJoCo + 部署 + 整段录制重放（PD 站立 → 跟随 → 重放），约 3.6 min；留下 sim csv + dump；环境变量 `A3_SIM_GAINS_JSON` / `A3_SIM_FRICTION_SCALE` 原样传给仿真 |
| `a3_score_sim_replay.py` | 对上面一次运行打分：摔倒、腿执行误差、脚的任务空间误差和抬脚高度、躯干摆动、支撑脚打滑；csv 与 dump 按关节值对齐 |
| `a3_eval_pose_recording.py` | 对录制离线打分（静止踝、贴极限、髋 yaw、人脚 → 机器人脚） |
| `a3_eval_leg_retarget.py` | 实时重定向 vs 训练库离线参考（动捕片段，按时间对齐 50 / 30 fps） |
| `a3_analyze_teleop_run.py` | 对部署 dump 分段分析，含脚的任务空间执行 |
| `a3_make_dump_cfg.py` | 每次运行一个独立的 dump 文件名 |
| `a3_ref_monitor.py` | 只读监视 `a3_ref` 流（速率、范围、跳变） |

相关：`mujoco_pico_sim_teleop.md`（MuJoCo + Pico 基本流程，5c 是本文的前身）、`pico_real_teleop.md`（真机）。

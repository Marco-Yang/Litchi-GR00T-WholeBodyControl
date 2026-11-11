# G1 真机部署 · 最终完整终端指令

前置：已按 [`g1_real_deploy_host_preflight.md`](g1_real_deploy_host_preflight.md) 全部勾选通过。

本机路径（当前工作站，勿再写 `/path/to/...`）：

```bash
export REPO="/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl"
export DEPLOY="$REPO/gear_sonic_deploy"
# VLA 最终导出（含 config.json / safetensors；同目录下另有 checkpoint-7376）
export VLA_CKPT="/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Gr00T/checkpoints/walk_towards_green_table_skill1_all_rtc_2cam_origin"
# PolicyServer 用的 Isaac-GR00T（经 $REPO/.deps/Isaac-GR00T 软链）
export LITCHI_ROOT="$REPO/.deps/Isaac-GR00T"
export CAMERA_HOST="192.168.123.164"    # G1 Orin 相机
export POLICY_HOST="127.0.0.1"          # PolicyServer 本机
export PROMPT="走到绿色桌子前"
```

**策略包只用 `policy/sonic_v1_1`，禁止 `policy/release`。**  
`--cp policy/sonic_v1_1/model` 是**文件前缀**（拼出 `model_encoder.onnx` / `model_decoder.onnx`），不是 `model/` 目录。

---

## A. 公共准备（每个新终端先做）

```bash
export REPO="/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl"
export DEPLOY="$REPO/gear_sonic_deploy"
source ~/.bashrc
cd "$DEPLOY"
source scripts/setup_env.sh

# 确认无仿真抢接口
pgrep -af 'run_sim_loop|run_sim_record|run_mujoco_vla' && echo 'STOP SIM FIRST' || echo 'OK no sim'
```

网卡（按现场确认）：

```bash
ip -4 -br addr
# 控制网应有 192.168.123.x（本机侧常见 enp11s0 → 192.168.123.100）
# PICO 用无线网卡 IP（不是 123 段；当前常见 wlp8s0）
# 没有 192.168.123.x 时不要开 deploy real
```

---

## B. 路线一：PICO 真机遥操（2 个终端）

### B0. 头显侧（先完成）

```bash
pgrep -af RoboticsServiceProcess || bash /opt/apps/roboticsservice/runService.sh
ip -4 -br addr show wlp8s0   # 头显 PC Service 填此 Wi‑Fi IP
```

头显 XRoboToolkit：填 Wi‑Fi IP → **WORKING** → Head/Controller + Full body（2 Tracker）→ Send。

### B1. 终端 1 — C++ SONIC deploy

```bash
export REPO="/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl"
export DEPLOY="$REPO/gear_sonic_deploy"
cd "$DEPLOY"
source scripts/setup_env.sh

./deploy.sh \
  --robot g1 \
  --cp policy/sonic_v1_1/model \
  --obs-config policy/sonic_v1_1/observation_config.yaml \
  --input-type zmq_manager \
  --motor-kp-scale 4,10=1.5 \
  --motor-kd-scale 4,10=1.5 \
  real
# 提示 Proceed → 输入 y
# 等到日志：Init Done
```

若 `real` 找不到 `192.168.123.x`，把最后的 `real` 换成机器人 IP，例如：

```bash
./deploy.sh --robot g1 \
  --cp policy/sonic_v1_1/model \
  --obs-config policy/sonic_v1_1/observation_config.yaml \
  --input-type zmq_manager \
  --motor-kp-scale 4,10=1.5 \
  --motor-kd-scale 4,10=1.5 \
  192.168.123.161
```

### B2. 终端 2 — PICO manager

```bash
export REPO="/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl"
cd "$REPO"
source .venv_teleop/bin/activate

python gear_sonic/scripts/pico_manager_thread_server.py \
  --robot g1 \
  --manager \
  --vis_vr3pt \
  --vis_smpl
# 确认可视化后可去掉 --vis_* 减负
```

### B3. 操作摘要

1. 标定姿（站直、肘约 90°）→ 手柄 **A+B+X+Y** → PLANNER  
2. 对齐身体 → **A+X** → POSE  
3. 软件急停：部署终端按 **`O`**（勿回车）  
4. 物理急停优先  

详情：[`pico_real_teleop.md`](pico_real_teleop.md)。

---

## C. 路线二：VLA 真机（origin RTC 权重）

架构：PolicyServer（GPU）+ 相机 ZMQ + C++ deploy + `run_vla_inference`。  
**RTC 必须与训练一致：`overlap=28 frozen=5 ramp=3.0`，不要用脚本默认 16/8/5。**

一次导出环境变量（后续终端可复用或各终端再 export）：

```bash
export REPO="/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl"
export DEPLOY="$REPO/gear_sonic_deploy"
export VLA_CKPT="/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Gr00T/checkpoints/walk_towards_green_table_skill1_all_rtc_2cam_origin"
export LITCHI_ROOT="$REPO/.deps/Isaac-GR00T"
export CAMERA_HOST="192.168.123.164"
export POLICY_HOST="127.0.0.1"
export PROMPT="走到绿色桌子前"
```

权重自检：

```bash
test -f "$VLA_CKPT/config.json" && test -f "$VLA_CKPT/model.safetensors.index.json" \
  && echo "VLA_CKPT OK: $VLA_CKPT" || echo "VLA_CKPT MISSING"
test -f "$LITCHI_ROOT/gr00t/eval/run_gr00t_server.py" \
  && echo "LITCHI_ROOT OK: $LITCHI_ROOT" || echo "LITCHI_ROOT MISSING (.deps/Isaac-GR00T)"
```

### C1. 终端 — 相机（在 Orin `192.168.123.164`）

在 **机器人 Orin** 上跑（不是工作站）。Orin 上仓库路径按现场；若已同步到与工作站同结构：

```bash
ssh unitree@192.168.123.164
# Orin 上：
cd /home/unitree/GR00T-WholeBodyControl   # ← 若 Orin 路径不同，改成实际 REPO
bash gear_sonic_deploy/scripts/g1_dual_realsense_rgb_server.sh
# origin 2cam 权重必须双相机（ego_view + head）；不要用单相机脚本
```

工作站可选验图：

```bash
export REPO="/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl"
export CAMERA_HOST="192.168.123.164"
cd "$REPO"
source .venv_inference/bin/activate
python gear_sonic/scripts/run_camera_viewer.py --camera-host "$CAMERA_HOST"
```

### C2. 终端 — Isaac-GR00T PolicyServer（本机 GPU）

```bash
export REPO="/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl"
export VLA_CKPT="/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Gr00T/checkpoints/walk_towards_green_table_skill1_all_rtc_2cam_origin"
export LITCHI_ROOT="$REPO/.deps/Isaac-GR00T"
# 本地已缓存的 Cosmos backbone（避免 HF_HUB_OFFLINE 下再打 huggingface.co）
export HF_HOME="${HOME}/.cache/huggingface"
export GROOT_BACKBONE_PATH="${HF_HOME}/hub/models--nvidia--Cosmos-Reason2-2B/snapshots/9ce19a195e423419c349abfc86fd07178b230561"
test -d "$GROOT_BACKBONE_PATH" || { echo "MISSING Cosmos local snapshot: $GROOT_BACKBONE_PATH"; exit 1; }

cd "$LITCHI_ROOT"
source "$REPO/.venv_inference/bin/activate"
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 GROOT_HF_LOCAL_FIRST=1
export NO_ALBUMENTATIONS_UPDATE=1

python gr00t/eval/run_gr00t_server.py \
  --model-path "$VLA_CKPT" \
  --embodiment-tag UNITREE_G1_SONIC \
  --device cuda:0 \
  --port 5550
```

### C3. 终端 — C++ deploy（与遥操相同，input=zmq）

```bash
export REPO="/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl"
export DEPLOY="$REPO/gear_sonic_deploy"
cd "$DEPLOY"
source scripts/setup_env.sh

# 必须已有 192.168.123.x；仿真 validates 用的 lo 不能用于真机
ip -4 -br addr | grep 192.168.123 || { echo 'MISSING G1 control iface'; exit 1; }

./deploy.sh \
  --robot g1 \
  --cp policy/sonic_v1_1/model \
  --obs-config policy/sonic_v1_1/observation_config.yaml \
  --input-type zmq_manager \
  --motor-kp-scale 4,10=1.5 \
  --motor-kd-scale 4,10=1.5 \
  real
# y → Init Done
```

### C4. 终端 — VLA inference client

```bash
export REPO="/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl"
export CAMERA_HOST="192.168.123.164"
export POLICY_HOST="127.0.0.1"
export PROMPT="走到绿色桌子前"

cd "$REPO"
source .venv_inference/bin/activate

python gear_sonic/scripts/run_vla_inference.py \
  --host "$POLICY_HOST" \
  --port 5550 \
  --embodiment-tag unitree_g1_sonic \
  --prompt "$PROMPT" \
  --action-publish-rate 50 \
  --action-horizon 40 \
  --rtc \
  --rtc-overlap-steps 28 \
  --rtc-frozen-steps 5 \
  --rtc-ramp-rate 3.0 \
  --camera-host "$CAMERA_HOST" \
  --camera-port 5555 \
  --state-zmq-host 127.0.0.1 \
  --state-zmq-port 5557 \
  --action-zmq-host 127.0.0.1 \
  --action-zmq-port 5556
```

> `state`/`action` ZMQ 默认与本机 C++ deploy 同机；若拆机，改成 deploy 机 IP。  
> C4 默认 **暂停**（`Pausing...`）；必须另开 **C5 键盘发布端**，按 `k` → `i` → `p` 才会动。

### C5. 终端 — 键盘发布（VLA 真机必开）

C4 通过 ZMQ `5580` 收键；**不要用** `python - <<'PY'`（heredoc 占 stdin，`input()` 会 EOF）。用下面的 `python -c`：

```bash
export REPO="/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl"
cd "$REPO"
source .venv_inference/bin/activate

python -c "
import zmq, time
ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.bind('tcp://localhost:5580')
time.sleep(0.5)
print('Keyboard ready. Keys: p=pause/resume, k=start/stop C++, i=arm-safe ready, t <prompt>=改提示')
while True:
    key = input()
    if key.startswith('t '):
        pub.send_string('prompt:' + key[2:])
        print('Sent prompt:', key[2:])
    else:
        pub.send_string(key)
        print('Sent:', key)
"
```

出现 `Keyboard ready` 后，**每行输入一个键再回车**，固定顺序：

```text
k
i
p
```

| 键 | 作用 |
|----|------|
| `k` | 启动/停止 C++ 控制环；启动时进 **PLANNER**（站立） |
| `i` | 臂安全就绪：保持/回到 PLANNER 站立，暂停策略，清 RTC **前缀缓存**（不是关 RTC）。**不发** latent 初始姿、不曲臂 |
| `p` | 暂停/继续策略；继续时仍站 PLANNER，**等第一帧 VLA 动作就绪再切 POSE** 下发 |
| `t <文案>` | 运行时改 prompt，并清 RTC 前缀（例：`t 走到绿色桌子前`） |
| `[` / `]` | 切换初始姿左右手开合（当前 `i` 不再发初始姿时一般不用） |

C4 预期日志：

1. `k` → `Started C++ control loop in PLANNER mode`  
2. `i` → `Ready in PLANNER stand...` / `Cleared cached action chunk and RTC prefix...`（**不应**再出现曲臂初始姿）  
3. `p` → `Resumed policy loop` → 随后 `Switched to POSE for first policy action` + `New action chunk (RTC, ...)`

说明：「清 RTC」= 丢掉上一截动作 overlap 前缀，下一段重新开；**`--rtc` 仍开启**。

### C6. 可选：tmux 一键（仍须覆盖 RTC）

```bash
export REPO="/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl"
export CAMERA_HOST="192.168.123.164"
export POLICY_HOST="127.0.0.1"
export PROMPT="走到绿色桌子前"

cd "$REPO"
source .venv_inference/bin/activate

python gear_sonic/scripts/launch_inference.py \
  --deploy-checkpoint policy/sonic_v1_1/model \
  --deploy-obs-config policy/sonic_v1_1/observation_config.yaml \
  --deploy-motor-kp-scale 4,10=1.5 \
  --deploy-motor-kd-scale 4,10=1.5 \
  --policy-host "$POLICY_HOST" \
  --policy-port 5550 \
  --camera-host "$CAMERA_HOST" \
  --prompt "$PROMPT" \
  --rtc \
  --rtc-overlap-steps 28 \
  --rtc-frozen-steps 5 \
  --rtc-ramp-rate 3.0
```

（若 launcher 未暴露某 RTC 旗标，改用 C4 手写 `run_vla_inference.py` + C5 键盘。）

### C7. 启动顺序与注意

1. 插好 G1 控制网线，确认本机 `192.168.123.x`，`ping 192.168.123.164` / `161`  
2. **C1** 相机 → **C2** PolicyServer → **C3** deploy（Init Done）→ **C4** inference → **C5** 键盘 → 输入 `k` `i` `p`  
3. 仿真（`validates` / `run_sim_*`）与真机 **不同时** 开  
4. 本文件只覆盖身体 VLA；若还要控 Revo2 真手，另开 Orin `revo2_modbus_rx`（见 [`pico_real_data_collection.md`](pico_real_data_collection.md) §9）

---

## D. 停止与急停

```bash
# G1 软件急停：焦点在 deploy 终端，按 O（不要回车）

# 结束 deploy 进程（无 O 的阻尼路径；倒了仍按物理急停）
pkill -INT -x g1_deploy_onnx_ref || pkill -TERM -x g1_deploy_onnx_ref

# 不要 kill -9
```

---

## E. 一页速查

| 目的 | 最小终端 |
|------|----------|
| 仅遥操 | B1 deploy + B2 pico |
| VLA | C1 相机 + C2 server + C3 deploy + C4 inference + **C5 键盘**（`k`→`i`→`p`） |
| 策略包 | `policy/sonic_v1_1` only（`--cp policy/sonic_v1_1/model` 前缀） |
| VLA 权重 | `$VLA_CKPT` → `…/Deploy/Gr00T/checkpoints/walk_towards_green_table_skill1_all_rtc_2cam_origin` |
| PolicyServer | `$LITCHI_ROOT` → `$REPO/.deps/Isaac-GR00T` + `.venv_inference` |
| origin RTC | `--rtc-overlap-steps 28 --rtc-frozen-steps 5 --rtc-ramp-rate 3.0` |
| 键盘口 | `tcp://localhost:5580`；`k` 启停，`i` 臂安全就绪，`p` 跑/停策略 |

安全：物理急停 > 软件 `O` > `pkill -INT`。  
仿真与真机 **不同时** 开；真机 DDS 用 `192.168.123.x`，**不要用 `lo`**。

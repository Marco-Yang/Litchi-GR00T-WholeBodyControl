# G1 真机部署 · 最终完整终端指令

前置：已按 [`g1_real_deploy_host_preflight.md`](g1_real_deploy_host_preflight.md) 全部勾选通过。

下文把 `$REPO` 换成部署机上的  
`…/Litchi-GR00T-WholeBodyControl`。  
**策略包只用 `policy/sonic_v1_1`，禁止 `policy/release`。**

---

## A. 公共准备（每个新终端先做）

```bash
export REPO="/path/to/Litchi-GR00T-WholeBodyControl"   # ← 改成现场路径
export DEPLOY="$REPO/gear_sonic_deploy"
source ~/.bashrc
cd "$DEPLOY"
source scripts/setup_env.sh

# 确认无仿真抢接口
pgrep -af 'run_sim_loop|run_sim_record' && echo 'STOP SIM FIRST' || echo 'OK no sim'
```

网卡（按现场改）：

```bash
ip -4 -br addr
# 控制网应有 192.168.123.x；PICO 用无线网卡 IP（不是 123 段）
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
export REPO="/path/to/Litchi-GR00T-WholeBodyControl"
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
export REPO="/path/to/Litchi-GR00T-WholeBodyControl"
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

下面路径按现场改：

```bash
export VLA_CKPT="/path/to/walk_towards_green_table_skill1_all_rtc_2cam_origin/walk_towards_green_table_skill1_all_rtc_2cam_origin"
export CAMERA_HOST="192.168.123.164"    # Orin / 相机机
export POLICY_HOST="127.0.0.1"          # PolicyServer 所在机；本机则 127.0.0.1
export PROMPT="走到绿色桌子前"
```

### C1. 终端 — 相机（在 Orin / 相机主机）

```bash
# 至少 ego_view + head（与 origin 双相机训练一致）
cd /path/to/Litchi-GR00T-WholeBodyControl/gear_sonic_deploy
bash scripts/g1_dual_realsense_rgb_server.sh
# 或：bash scripts/g1_camera_rgb_server.sh
```

部署机可选验图：

```bash
cd "$REPO"
source .venv_inference/bin/activate   # 或 teleop
python gear_sonic/scripts/run_camera_viewer.py --camera-host "$CAMERA_HOST"
```

### C2. 终端 — Isaac-GR00T PolicyServer（GPU 机）

在 **Litchi-Isaac-GR00T** 仓（路径按现场）：

```bash
cd /path/to/Litchi-Isaac-GR00T
source .venv/bin/activate   # 或 uv / 现场约定环境
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 GROOT_HF_LOCAL_FIRST=1

python gr00t/eval/run_gr00t_server.py \
  --model-path "$VLA_CKPT" \
  --embodiment-tag UNITREE_G1_SONIC \
  --device cuda:0 \
  --port 5550
```

### C3. 终端 — C++ deploy（与遥操相同，input=zmq）

```bash
export REPO="/path/to/Litchi-GR00T-WholeBodyControl"
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
# y → Init Done
```

### C4. 终端 — VLA inference client

```bash
export REPO="/path/to/Litchi-GR00T-WholeBodyControl"
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

### C5. 可选：tmux 一键（仍须覆盖 RTC）

```bash
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

（若 launcher 未暴露某 RTC 旗标，改用 C4 手写 `run_vla_inference.py`。）

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
| VLA | C1 相机 + C2 server + C3 deploy + C4 inference |
| 策略包 | `policy/sonic_v1_1` only |
| origin RTC | `--rtc-overlap-steps 28 --rtc-frozen-steps 5 --rtc-ramp-rate 3.0` |

安全：物理急停 > 软件 `O` > `pkill -INT`。  
仿真与真机 **不同时** 开。

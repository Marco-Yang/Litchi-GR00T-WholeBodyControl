# 双缓冲多技能无 RTC 部署：完整逻辑 + 可直接粘贴指令

## 一、架构与端口

| 组件 | 端口 | 作用 |
|------|------|------|
| PolicyServer A | `5550` | active / standby 之一 |
| PolicyServer B | `5551` | 另一个缓冲槽 |
| Supervisor 控制 | `5590` | C4 发 `switch` / 查 `status` |
| 相机 | `5555` | Orin RGB |
| Action ZMQ | `5556` | C4 → C++ |
| State ZMQ | `5557` | C++ → C4 |
| 键盘 | `5580` | C5 → C4 |
| Revo2 UDP | `19500` | C4 → Orin 真手 |

技能表（`gear_sonic/config/multi_skill_nortc_skills.json`）：

| 键 | skill | ckpt | prompt |
|----|-------|------|--------|
| `1` | 走到绿桌 | `skill1_.../checkpoint-3852` | 走到绿色桌子前 |
| `2` | 抓玩偶放篮 | `skill2_.../checkpoint-10952` | 侧平举…放进编织篮里 |
| `3` | 拎篮转向黑箱 | `skill3_.../checkpoint-5336` | 拿起装黄色玩偶的篮子… |

循环预热顺序：`1 → 2 → 3 → 1`。

---

## 二、双缓冲切换逻辑

```
启动:
  5550 ← skill1 (active)
  5551 ← skill2 (standby, 已热好)
  显存 ≈ 2×6.8GB ≈ 14GB

按「2」(温切换 warm=True):
  1) C4 暂停 + hard restart 清旧动作
  2) supervisor 把 active/standby 角色对调 → 立即返回 policy_port=5551
  3) C4 重连 5551，改 prompt，再 hard restart + PolicyServer.reset
  4) 后台: 杀旧 skill1 → 在空端口预热 skill3

按「3」(若 standby 已是 3):
  同温切换 → 后台预热 skill1

跳着切(如 1→3 而 standby=2):
  冷加载: 阻塞把 3 装到 standby → promote → 后台预热 next(3)=1
```

C4 侧每次切技能都会：暂停 → 清 chunk/RTC 前缀 → 回 PLANNER → 等 supervisor → 重连新 `policy_port` → `reset` → 打印 `Skill READY` → 你再 `i` → `p`。

---

## 三、启动前：清掉旧单路 server

```bash
pkill -f run_skill_server_supervisor.py || true
pkill -f run_gr00t_server.py || true
pkill -f run_vla_inference.py || true
```

顺序：**C1 → C2（等 both ready）→ C3（Init Done，无持续 `0x40000000`）→ C3.5 → C4 → C5**。

---

## 四、终端指令

### C1 · Orin 相机

```bash
ssh unitree@192.168.123.164
cd /home/unitree/GR00T-WholeBodyControl
bash gear_sonic_deploy/scripts/g1_dual_realsense_rgb_server.sh
```

### C2 · 双缓冲 supervisor（替代单 ckpt PolicyServer）

```bash
export REPO="/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl"
export HF_HOME="${HOME}/.cache/huggingface"
export GROOT_BACKBONE_PATH="${HF_HOME}/hub/models--nvidia--Cosmos-Reason2-2B/snapshots/9ce19a195e423419c349abfc86fd07178b230561"
bash "$REPO/gear_sonic/scripts/launch_multi_skill_nortc.sh"
```

等到类似：

```text
[supervisor] READY skill=1 ... port=5550
[supervisor] READY skill=2 ... port=5551
[supervisor] both ready — active=1:5550 standby=2:5551
[supervisor] control REP on tcp://*:5590
```

首次约 2× 热启动时间；日志：`/tmp/gr00t_policy_server_skill*_p*.log`。

### C3 · C++ deploy

```bash
export REPO="/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl"
export DEPLOY="$REPO/gear_sonic_deploy"
source ~/.bashrc
cd "$DEPLOY"
source scripts/setup_env.sh
pgrep -af 'run_sim_loop|run_sim_record|run_mujoco_vla' && echo 'STOP SIM FIRST' || echo 'OK no sim'
ip -4 -br addr | grep 192.168.123 || { echo 'MISSING G1 control iface'; exit 1; }

./deploy.sh \
  --robot g1 \
  --cp policy/sonic_v1_1/model \
  --obs-config policy/sonic_v1_1/observation_config.yaml \
  --input-type zmq_manager \
  --motor-kp-scale 4,10=1.5 \
  --motor-kd-scale 4,10=1.5 \
  real
# y → Init Done；确认无持续 0x40000000
```

### C3.5 · Revo2 真手 RX（Orin）

```bash
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Manus
bash scripts/start_orin_revo2_rx.sh
# 须见: receiver ready — publish UDP to 192.168.123.164:19500
```

### C4 · VLA client（多技能 + 无 RTC）

```bash
export REPO="/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl"
export CAMERA_HOST="192.168.123.164"
export ACTION_HORIZON=40
export INFERENCE_RATE="$(python3 -c 'print(50/12)')"

cd "$REPO"
source .venv_inference/bin/activate

python gear_sonic/scripts/run_vla_inference.py \
  --deploy-branch origin \
  --host 127.0.0.1 \
  --port 5550 \
  --embodiment-tag unitree_g1_sonic \
  --skills-config "$REPO/gear_sonic/config/multi_skill_nortc_skills.json" \
  --action-publish-rate 50 \
  --action-horizon "$ACTION_HORIZON" \
  --no-rtc \
  --rate "$INFERENCE_RATE" \
  --camera-host "$CAMERA_HOST" \
  --camera-port 5555 \
  --state-zmq-host 127.0.0.1 \
  --state-zmq-port 5557 \
  --action-zmq-host 127.0.0.1 \
  --action-zmq-port 5556 \
  --revo2-send \
  --revo2-orin-host 192.168.123.164 \
  --revo2-udp-port 19500
```

应见：`Multi-skill enabled`、`dual-buffer ports active=5550/standby=5551`、默认 skill1 prompt。

### C5 · 键盘

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
print('Keys: k i p | 1=walk 2=pick_toy 3=pick_basket | t <prompt>')
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

---

## 五、实机操作流程

**跑 skill1**

```text
k
i
p
```

**切到 skill2（温切换，应很快）**

```text
2
```

等 C4：`Skill READY ... warm=True ... prefetch→3`，再：

```text
i
p
```

**切到 skill3**

```text
3
```

等 `Skill READY ... warm=True ... prefetch→1`，再 `i` → `p`。

**键含义**

| 键 | 作用 |
|----|------|
| `k` | 启/停 C++ 控制环（PLANNER 站） |
| `i` | 臂安全就绪：回 PLANNER、暂停、清前缀 |
| `p` | 跑/停策略（POSE 从首个 action 开始） |
| `1`/`2`/`3` | 切技能（走双缓冲 promote） |
| `t <文案>` | 仅改 prompt，不换 ckpt |

---

## 六、成功判据

- C2：`both ready`；切技能后有 `WARM promote`，后台 `background warm-start next skill=…`
- C4：`PolicyClient reconnect …→… (warm=True)`，然后 `Skill READY`
- `nvidia-smi`：稳态约两路模型占显存（约 14GB 量级）
- 若看到 `warm=False`：走了冷加载（跳序或 standby 尚未预热完），会再等一次完整加载

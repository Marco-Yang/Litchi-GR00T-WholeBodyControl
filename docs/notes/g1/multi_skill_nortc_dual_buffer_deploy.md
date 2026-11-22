# 共享 VLM 多技能无 RTC 部署：完整逻辑 + 可直接粘贴指令

> **当前方案：** 单进程 PolicyServer = **1× 共享 Cosmos VLM（backbone）+ 3× 常驻 action_head**。  
> 三个 skill 的 `backbone.*` 权重完全一致；切换时换 `action_head` **并**换该 skill 的 `statistics.json`（state/action 归一化）。  
> 文件名仍含 `dual_buffer`，实现已不是双整模双端口，而是共享 VLM。

---

## 一、架构与端口

| 组件 | 端口 | 作用 |
|------|------|------|
| PolicyServer（共享 VLM） | `5550` | `get_action` / `reset`（单端口） |
| Skill 控制 | `5590` | C4 发 `switch` / 查 `status` |
| 相机 | `5555` | Orin RGB（ego + head） |
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

---

## 二、共享 VLM 切换逻辑

```
启动（单进程）:
  backbone ← skill1（与 skill2/3 的 backbone 逐 tensor 相同）
  action_heads["1"|"2"|"3"] 全部常驻 GPU
  skill_statistics["1"|"2"|"3"] ← 各 ckpt 的 statistics.json
  显存 ≈ 1×BB + 3×AH ≈ 12–15GB

按「2」（只需按一次）:
  1) C4 暂停 + hard restart 清旧动作；若在 POSE 则回 PLANNER
  2) 控制口 5590: switch skill=2
       → model.action_head = heads["2"]
       → processor.set_statistics(skill2 statistics.json, override=True)
  3) C4 更新 prompt + PolicyServer.reset
  4) **自动等价 i + p**：保持/回到 PLANNER，然后 resume VLA
     （不必再手动按 i、p）
  5) 单端口、单 head 指针；不会双头同时输出
```

**归一化注意：** 各 skill 训练数据分布不同 → `statistics.json` 不同。  
必须「当前 head ↔ 当前 stats」成对切换；只换 head、沿用 skill1 的 stats 会导致动作明显变差。

---

## 三、启动前清理

```bash
pkill -f run_shared_vlm_skill_server.py || true
pkill -f run_skill_server_supervisor.py || true
pkill -f run_gr00t_server.py || true
pkill -f run_vla_inference.py || true
```

顺序：**C1 → C2（等 ready）→ C3（Init Done，无持续 `0x40000000`）→ C3.5 → C4 → C5**。

---

## 四、终端指令

### C1 · Orin 相机

```bash
ssh unitree@192.168.123.164
cd /home/unitree/GR00T-WholeBodyControl
bash gear_sonic_deploy/scripts/g1_dual_realsense_rgb_server.sh
```

### C2 · 共享 VLM multi-skill server

```bash
export REPO="/home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl"
export HF_HOME="${HOME}/.cache/huggingface"
export GROOT_BACKBONE_PATH="${HF_HOME}/hub/models--nvidia--Cosmos-Reason2-2B/snapshots/9ce19a195e423419c349abfc86fd07178b230561"
bash "$REPO/gear_sonic/scripts/launch_multi_skill_nortc.sh"
```

等到类似：

```text
[shared-vlm] ready skills=['1', '2', '3'] active=1 (1×backbone + 3×action_head; per-skill statistics.json loaded)
[shared-vlm] control REP on tcp://*:5590
✓ Server ready — listening on ...:5550
```

切换成功时 C2 日志应含：`SWITCH active→2 ... (action_head + statistics.json)`。

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

**默认：不录 agent 视频**（只做多技能部署）。

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

应见：`Multi-skill enabled`、`shared-VLM policy port=5550`、默认 skill1 prompt。

### C4（可选）· 同时录制 agent 验证视频 / 时间戳

用于后续 **agent 自动切技能** 训练/验证。默认关闭；需要时在上面的 C4 命令末尾 **追加** 三行：

```bash
  --record-agent-video \
  --agent-video-dir /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Gr00T/agents/video \
  --agent-video-fps 10
```

| 参数 | 默认 | 含义 |
|------|------|------|
| `--record-agent-video` | 关 | 打开后才录 |
| `--agent-video-dir` | `…/Gr00T/agents/video` | 输出根目录 |
| `--agent-video-fps` | `10` | 采样帧率 |

**录制生命周期（仅当开启上述选项时）：**

| 时机 | 行为 |
|------|------|
| 首次按 `p` 恢复策略 | 开始录：新建 `session_YYYYMMDD_HHMMSS/` |
| 按 `1` / `2` / `3` 切技能 | 写入 `skill_switch` 事件（`t_sec` + `frame_index`） |
| 按 `s` | **结束并保存** 本段视频（VLA 可继续跑） |
| C4 Ctrl-C 退出 | 若仍在录，也会 stop 并保存 |

**输出文件：**

```text
…/Gr00T/agents/video/session_YYYYMMDD_HHMMSS/
  cameras.mp4          # ego_view + head 横拼
  skill_events.json    # start / skill_switch / stop + 元数据
```

开启后 C4 启动时应见：`Agent video recording ENABLED → …`；  
按 `s` 后应见：`Agent video STOPPED by 's' → saved …`。

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
print('Keys: k i p | 1/2/3=skill(auto i+p) | s=stop agent-video | t <prompt>')
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

## 五、实机操作与按键

### 5.1 开局跑 skill1

```text
k
i
p
```

若开了 `--record-agent-video`，第一次 `p` 同时开始录像。

### 5.2 切技能（自动 i+p）

```text
2          # 切到 skill2 并自动恢复，不必再按 i、p
3          # 切到 skill3，同上
1          # 回到 skill1，同上
```

C4 期望：`Skill READY + auto i/p ... VLA resumed`。

### 5.3 结束 agent 录像（可选功能开启时）

```text
s          # 保存 cameras.mp4 + skill_events.json；策略可继续
```

之后若再 `p`（且当前没有在录），会开 **新的** `session_*`。

### 5.4 按键一览

| 键 | 作用 |
|----|------|
| `k` | 启/停 C++ 控制环（PLANNER 站） |
| `i` | 臂安全就绪：回 PLANNER、暂停、清前缀（仍可用；切技能时已自动做） |
| `p` | 跑/停策略；**首次 resume 时**若开启录制则开始录像 |
| `1` / `2` / `3` | 切技能（换 action_head + statistics）并 **自动 i+p** |
| `s` | **（可选录制开启时）** 结束并保存 agent 视频；VLA 不退出 |
| `t <文案>` | 仅改 prompt，不换 head / stats |

---

## 六、成功判据

- C2：`ready skills=['1','2','3']` + `per-skill statistics.json loaded` + `:5550` / `:5590`
- C4：切技能后 `Skill READY + auto i/p`（近瞬时；无整模重载）
- 切技能后动作质量应接近「单独跑该 skill 整模」；若很差，查 C2 是否打印 `action_head + statistics.json`
- `nvidia-smi`：约 12–15GB（1×backbone + 3×action_head）
- 权重前提：三份 ckpt 的 `backbone.*` 必须完全一致
- **可选录制：** 有 `session_*/cameras.mp4` 与含 `skill_switch` 的 `skill_events.json`；按 `s` 能正常 stop

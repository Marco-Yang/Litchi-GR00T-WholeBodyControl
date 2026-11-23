# VLM agent 自动切技能（长程任务无人工按键）

> 在 [multi_skill_nortc_dual_buffer_deploy.md](multi_skill_nortc_dual_buffer_deploy.md) 的共享 VLM 多技能部署之上，
> 用一个微调过的 **Qwen3-VL-4B-Instruct (LoRA)** 替代人工按 `2` / `3`。  
> 代码：`RL controller/agents/skill_agent/`；示范：`RL controller/agents/video/session_*`。  
> **训练逻辑门外汉详解**：[skill_agent_training_explained.md](skill_agent_training_explained.md)。
>
> **推荐判断器已换成「技能边界裁判」（§零）**：冻结的 VLA 视觉塔 + 508 万参数小 Transformer，
> 显存约 1 GB、每次判断约 10 ms、10 Hz。下文 §一～§四 的 Qwen3-VL-4B LoRA 作为备选保留。

---

## 零、推荐：技能边界裁判（boundary judge）

### 思路

| 要素 | 做法 |
|------|------|
| 视觉 | 直接用 VLA 共享 backbone 的视觉塔（Cosmos-Reason2-2B，三个 skill 的 checkpoint 里逐位相同，`tune_visual=False`），从 VLA checkpoint 加载，冻结；每帧 ego/head 各池化成 3×4=12 个 token |
| 本体 | 与 VLA 同一套 43 维状态（`qpos` / `observation.state`）+ 64 维 SONIC motion token + 本技能开始以来偏航 + 重力投影，10 Hz、最近 2 s |
| 各 skill 归一化差异 | 同一份本体分别用「当前 skill」和「下一 skill」的 VLA `statistics.json`（q01/q99，与 VLA 一样按分位数归一化）各归一化一遍 |
| 技能描述 | 用各 skill 的 q01/q99「统计指纹」当上下文，不用技能编号 → 新增 skill 只需它的 checkpoint 统计 |
| 结构 | 本体时序作 query，交叉注意力到视觉 token（4 层 Transformer decoder，508 万参数） |
| 输出 | P(当前 skill 完成) + 预测还差几秒完成（τ̂），τ̂ 可用于提前触发补偿切换延迟（`--lead-s`） |
| 训练数据 | ① 各 skill 的 VLA 遥操数据（`Datasets/data`，1597 条，每条录制结束=该 skill 完成）② 长程示范（`agents/video`，人按键时刻 − 0.5 s 视为完成；最后一个 skill 为停止录制 − 0.3 s） |

### 离线流水线

```bash
export AGENT="/home/user/YZY/HumanWBM/Demo/RL controller/agents/skill_agent"
cd "$AGENT" && source .venv/bin/activate
export HF_HUB_OFFLINE=1

# 技能注册表：skill_registry.json（prompt / VLA checkpoint / 遥操数据目录 / 任务顺序）
# 0) 遥操数据统计：episode 数、首尾静止时长、交接姿态 → ../boundary_dataset/vla_analysis.json
python analyze_vla_episodes.py

# 1) 建缓存（约 16 min，vis.npy 约 11 GB）→ ../boundary_dataset/
python build_boundary_dataset.py

# 2) 留出 2 条示范 + 每个 skill 1 个遥操批次做验证（约 2 min）
python train_boundary.py --out ../checkpoints/boundary_val
# 3) 回放评估（与 eval_offline.py 同一套触发 / 门禁；输出可叠加到 web_viewer）
python eval_boundary.py --ckpt ../checkpoints/boundary_val/best
python eval_boundary.py --ckpt ../checkpoints/boundary_val/best --blank-images --no-gates --out ../checkpoints/boundary_val/eval_blank_images
python eval_boundary.py --ckpt ../checkpoints/boundary_val/best --hide-proprio --no-gates --out ../checkpoints/boundary_val/eval_hide_proprio

# 4) 全部数据训最终模型（真机用）
python train_boundary.py --val-demo-sessions none --val-vla-sessions none --out ../checkpoints/boundary_all

# 网页回放叠加 P(done) 与 agent 触发点
.venv_viz/bin/python web_viewer.py --eval-dir ../checkpoints/boundary_val/eval
```

新录的长程示范放进 `agents/video/`、新的遥操数据加进 `skill_registry.json` 后，重跑 1→2→4 即可；
新增 skill：在 `skill_registry.json` 里加一项（prompt、VLA checkpoint、遥操数据目录）并改 `task_order`。

### 实测（2026-11-22，留出 `231746` / `232854` 共 6 次切换；`checkpoints/boundary_val_d`，即当前默认参数）

| 判断器 | 漏切 | 相对人按键 | 相对估计完成时刻 | 每次判断 | 显存 | 训练 |
|--------|------|------------|------------------|----------|------|------|
| 边界裁判（遥操 + 6 条示范） | 0 | 平均早 0.47 s | 平均绝对误差 0.17 s | 约 10 ms（编码 9 ms + 判断 1 ms） | 约 1 GB | 2 min |
| Qwen3-VL-4B LoRA（门禁开） | 1 | 5 次平均绝对误差 0.28 s | 平均晚约 0.6 s | 79 ms | 8.6 GB | 29 min |

- 「估计完成时刻」= 人按键 − 0.5 s（最后一个 skill：停止录制 − 0.3 s）。边界裁判在完成那一刻切，省掉人的反应时间；
  4B 因为 2 Hz × 连续 3 次确认，本身就晚约 1.5 s。核对触发瞬间的本体：skill2 身体已静止，skill3 已转到最终角度（−80° / −80°，−85° / −88°）。
- 只给本体（灰图）：6 次中 5 次成功；只给画面：5 次成功但整体晚约 1 s。两路都能单独撑住大部分情况。
- **只用遥操数据、一条示范都不用**：标签排序对（AUC 0.92–0.995），但在 VLA 自主执行时偏保守，skill3 会漏切；
  少量长程示范的作用就是这层校准。

### 真机

```bash
cd "$AGENT" && source .venv/bin/activate
# 第一次：dry-run
python agent_node.py --boundary-ckpt ../checkpoints/boundary_all/best --camera-host 192.168.123.164 --dry-run
# 自动切换
python agent_node.py --boundary-ckpt ../checkpoints/boundary_all/best --camera-host 192.168.123.164
```

应见 `judge ready: boundary vision=<VLA checkpoint> ... VRAM=0.8 GiB`；运行时每 2 s 打印
`skill2 13.0s p_done=0.00 tau=5.81s ... 12ms`。默认 10 Hz、`--max-vram-gb 3`，其余按键 / 门禁 / `--on-finish` 与 §三 相同。

`--lead-s`：τ̂ ≤ 该值也算命中。**C4 现在切技能会 hard restart（暂停、清动作、回 PLANNER）**，提前切会截掉上一个 skill 的收尾，
所以默认不开；等 C4 改成在动作块边界直接换 head 再设为实测切换延迟。

回放测试台（无机器人）：

```bash
python replay_harness.py --session session_20261122_231746            # 终端 A
python agent_node.py --boundary-ckpt ../checkpoints/boundary_val/best --camera-host 127.0.0.1   # 终端 B，输入 k、p
```

---

## 一、架构

```
Orin 相机 :5555 ──┬──────────────► C4 run_vla_inference.py ──► C++ :5556
                  │                       ▲  (键盘 SUB :5580，不改代码)
                  └──► agent_node.py ─────┘  PUB :5580  "k" "p" "skill:2" "skill:3" ...
C++ g1_debug :5557 ─► agent_node.py（模型本体输入 + 本体校验：右臂回位 / 偏航角）
C2 :5590 status   ◄── agent_node.py（每 2 s 对齐当前 skill）
```

- `agent_node.py` **替代 C5 键盘终端**：照常输入 `k / i / p / 1 / 2 / 3 / s / t <prompt>`，原样转发给 C4。
- VLA 运行中（`p` 之后）agent 以 2 Hz 判断「当前 skill 是否完成」，满足条件时自己发 `skill:<下一个>`；
  C4 收到后走与按键完全相同的路径（换 action_head + statistics、hard restart、自动 i+p）。
- skill3 判定完成后默认只打印 `TASK COMPLETE`；`--on-finish stand` 时自动发 `i`（回 PLANNER 站立并暂停）。

### 判定逻辑

每次决策对 `2 s 前 / 1 s 前 / 当前` 三个时刻，各输入：

- 图像：`ego_view` + `head`，缩放到 384×288；
- 本体（与该时刻图像帧对齐，154 维）：`body_q`(29) `body_dq`(29) `left/right_hand_q`(14)
  `token_state`(64) `last_left/right_hand_action`(14) `base_ang_vel`(3) + 本 skill 开始以来的偏航角变化(1)。
  `qpos` 是 `body_q` 与手部关节按机器人模型顺序的重排，信息相同，所以直接用原始字段。

本体向量经训练集均值/方差归一化，由一个小 MLP（`proprio_encoder.pt`）编码成每时刻 2 个 soft token，
替换 prompt 中该时刻图像后面的占位符 `<|fim_pad|>`，与 LoRA 一起训练。

两种模态 dropout（`train_lora.py`）：

- `--image-dropout 0.3`：skill2/3 样本里 30% 把所有图像换成灰图，逼模型从本体判断完成
  （右臂回位、手张开、偏航转约 90°、motion token 变化）。**不加这一项时模型会完全忽略本体**：
  图像一个 epoch 就把训练损失压到 0，实测灰图 + 真实本体时 6 次切换全部 MISS。
  skill1「走到桌前」在本体上不可见（没有里程计），不做图像 dropout。
- `--proprio-dropout 0.05`：少量样本隐藏本体，`g1_debug` 临时缺失时仍可只看图像判断。

训练日志每个 epoch 会多打印一行 `proprio-only (gray images, skills 2/3)`，用来确认本体通路确实学到了东西。

prompt 里写明当前 skill 和完成标准，模型输出 `{"done": true|false, ...}`。
运行时只做一次前向，读 ` true` / ` false` 两个 token 的概率得到 `P(done)`，不做文本生成。

对齐方式与录制一致：录制时 `robot_state.npz` 每行与视频帧共用 `frame_index`；
运行时 agent 以 10 Hz 缓存相机帧和 `g1_debug`，对每个选中的相机帧取接收时间最近的状态样本。

切换必须同时满足：

| 条件 | 默认 |
|------|------|
| `P(done) ≥ threshold`，连续 `confirm` 次 | 0.6，3 次（2 Hz 下约 1.5 s） |
| 当前 skill 已运行 ≥ `min-dwell` | skill1 2 s / skill2 8 s / skill3 6 s |
| 本体校验 skill2 | 右臂偏离静息位 ≤ 1.075 rad 且右手指令 ≤ 0.2（已松手、已回位） |
| 本体校验 skill3 | skill3 开始后偏航角累计 ≤ −40°（已顺时针右转） |

阈值来自示范统计（`agents/dataset/meta.json → proprio_reference`）：
skill2 切换时右臂偏离 0.51–0.77 rad（抓取过程中位数 1.27–1.74），skill3 结束时偏航 −80° ~ −121°。

### 显存

| 进程 | 显存 |
|------|------|
| 桌面 | ≈ 3.3 GB |
| C2 共享 VLM（1×backbone + 3×action_head） | ≈ 12–15 GB |
| agent（Qwen3-VL-4B bf16 + LoRA 合并） | ≈ 9–11 GB，`--max-vram-gb 12` 封顶 |

**先起 C2，再起 agent**；agent 进程被 `set_per_process_memory_fraction` 限额，超限只会 agent 自己 OOM，不会挤爆 VLA。
上真机前用 `nvidia-smi` 确认总占用 < 30 GB。

---

## 二、离线：数据 → 训练 → 评估（一次性）

```bash
export AGENT="/home/user/YZY/HumanWBM/Demo/RL controller/agents/skill_agent"
cd "$AGENT" && source .venv/bin/activate
export HF_HUB_OFFLINE=1

# 1) 可视化示范（时间轴 / 切换对齐的本体曲线 / 关键帧 / 带标签回放视频）→ agents/viz/
python visualize_demos.py

# 2) 构建数据集 → agents/dataset/（帧缓存 + 对齐的本体数组 + samples.jsonl + meta.json）
#    --proprio-tokens 0 可构建纯视觉版本做对照
python build_dataset.py

# 3) 留出 2 条 session 训练并验证（约 30 min，峰值 ≈ 20 GB；训练时不要开 C2）
python train_lora.py --out ../checkpoints/skill_judge_4b_val

# 4) 按真机节点同样的触发逻辑回放留出 session，对比人工按键时刻 → ../checkpoints/skill_judge_4b_val/eval/
python eval_offline.py --adapter ../checkpoints/skill_judge_4b_val/adapter
#    鲁棒性：模拟 g1_debug 断开（本体输入置为缺失）
python eval_offline.py --adapter ../checkpoints/skill_judge_4b_val/adapter --hide-proprio --no-gates \
  --out ../checkpoints/skill_judge_4b_val/eval_no_proprio
#    模型是否真的在用本体：图像换成灰图，只给本体
python eval_offline.py --adapter ../checkpoints/skill_judge_4b_val/adapter --blank-images --no-gates \
  --out ../checkpoints/skill_judge_4b_val/eval_blank_images

# 5) 用全部示范训练最终模型（真机用这个）
python train_lora.py --val-sessions none --out ../checkpoints/skill_judge_4b_all
python eval_offline.py --adapter ../checkpoints/skill_judge_4b_all/adapter --sessions all
```

### 示范数据网页回放 + 切换时间戳修正（推荐）

```bash
cd "$AGENT"
.venv_viz/bin/python web_viewer.py            # 浏览器打开 http://127.0.0.1:8765
# 叠加 agent 的 P(done) 曲线（先跑 eval_offline.py）
.venv_viz/bin/python web_viewer.py --eval-dir ../checkpoints/skill_judge_4b_proprio_val/eval
```

页面内容（全部按同一时间轴对齐）：

- ego / head 原始 640×480 画面；three.js 渲染的 G1 + Revo2（MuJoCo 运动学，鼠标左键旋转 / 右键平移 / 滚轮缩放）；
- 当前技能、训练标签、关键本体值、完成标准；29 个 `body_q` 关节 + 双手 Revo2 指令表；
- 时间轴：录制切换、可拖动的切换标记、训练标签预览、腿速/偏航、手臂偏离、手部指令、motion token 热力图、agent P(done)。

操作：`空格` 播放/暂停，`←/→` 逐帧，`Shift+←/→` ±1 s，`1/2/3` 跳到技能开头；点击 / 拖动时间轴定位；
工具栏可直接输入秒数或帧号。

**修正切换时间戳**：拖动时间轴上的「1→2 / 2→3 / 结束」标记（自动吸附到帧，画面同步跳到该帧），
或在下方表格输入时间 / 点「设为当前时刻」，训练标签预览实时更新；「保存标注」写入
`agents/annotations/<session>.json`（原始录制不改），「恢复原始时间」删除该文件。
`build_dataset.py` / `eval_offline.py` / `visualize_demos.py` 会自动使用修正后的时间
（`build_dataset.py --annotations-dir ''` 可忽略修正）。

### 导出 mp4 / 本地窗口（备选）

```bash
.venv_viz/bin/python demo_viewer.py --export ../viz/replay   # 每个 session 导出 H.264 mp4
.venv_viz/bin/python demo_viewer.py --mujoco-viewer          # 原生 MuJoCo 窗口 + OpenCV 窗口
```

说明：录制里没有基座位置，机器人原地回放（姿态、关节、支撑脚高度是真实的）。
模型为 `validates/assets/g1_29dof_with_revo2_vizclean.xml`，网格目录覆盖为 `data/meshes`（`--mjcf` / `--meshdir` 可改）；
Revo2 手指按 `gear_sonic/utils/mujoco_sim/revo2_hand_config.py` 的指令顺序和弧度系数驱动，远端指节 1:1 跟随近端。
`.venv_viz` 只含 mujoco / opencv(GUI) / matplotlib，与训练环境分开。

训练标签（`common.LabelConfig`）：人工按键前 1.0 s 为「完成」，再往前 1.0 s 为歧义区（不训练），其余为「未完成」；
按键后 0.5 s 内的画面复用为上一 skill 的「完成」样本；skill3 取停止前 1.5 s。

### 无机器人联调（回放测试台）

```bash
# 终端 A：用录制 session 模拟 Orin 相机 / C++ g1_debug / C4 键盘口 / C2 status
python replay_harness.py --session session_20261122_231746
# 终端 B
python agent_node.py --adapter ../checkpoints/skill_judge_4b_all/adapter --camera-host 127.0.0.1
# 在 B 输入 k、p；A 会打印收到的 skill:2 / skill:3 及其相对时间
```

---

## 三、真机部署

C1（相机）/ C2（共享 VLM server）/ C3（C++ deploy）/ C3.5（Revo2）/ C4（VLA client）**与原文档完全一致**；
**C5 键盘换成下面的 agent 终端**（两者都绑定 :5580，不能同时开）。

### C5′ · agent（第一次：dry-run，只判断不切换）

```bash
export AGENT="/home/user/YZY/HumanWBM/Demo/RL controller/agents/skill_agent"
cd "$AGENT" && source .venv/bin/activate
nvidia-smi --query-gpu=memory.used,memory.total --format=csv   # 确认 C2 已占好显存

python agent_node.py \
  --adapter ../checkpoints/skill_judge_4b_all/adapter \
  --camera-host 192.168.123.164 \
  --dry-run
```

dry-run 下照常手动按 `2`、`3`，观察 agent 打印的 `[DRY-RUN] would switch ...` 是否落在你认为该切的时刻附近。

### C5′ · agent（自动切换）

```bash
python agent_node.py \
  --adapter ../checkpoints/skill_judge_4b_all/adapter \
  --camera-host 192.168.123.164
```

应见：`judge ready: base=Qwen/Qwen3-VL-4B-Instruct VRAM=… GiB`。操作：

```text
k          # 启动 C++ 控制环（PLANNER 站）
i
p          # 开始 skill1；之后 agent 自动 skill:2 → skill:3 → TASK COMPLETE
```

### 按键

| 键 | 作用 |
|----|------|
| `k` `i` `p` `s` `1` `2` `3` `t <文案>` | 与原 C5 相同，原样转发给 C4 |
| `a` | 开/关自动切换（关掉后等同纯键盘） |
| `?` | 打印 agent 状态（当前 skill、是否运行、相机延迟、连续命中数） |
| `q` | 退出 agent |

**人工随时可接管**：直接按 `1/2/3` 即可；agent 会跟随新 skill，并每 2 s 通过 :5590 `status` 与 C2 对齐。

### 常用参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `--dry-run` | 关 | 只判断、记录，不发切换 |
| `--threshold` / `--confirm` | 0.6 / 3 | 提前切 → 调高；切得太晚 → 调低 |
| `--min-dwell` | `1:2,2:8,3:6` | 每个 skill 最短运行秒数 |
| `--no-gates` | 关 | 关闭本体校验（只看视觉） |
| `--on-finish` | `none` | `stand`：skill3 完成后发 `i` |
| `--rate` | 2 | 决策频率（Hz） |
| `--max-vram-gb` | 12 | agent 显存上限 |
| `--verbose` | 关 | 每次决策都打印 |

### 日志

`agents/runs/agent_YYYYmmdd_HHMMSS/`：

- `decisions.jsonl`：每次决策的 skill、运行时长、`P(done)`、本体输入状态（ok/missing）、本体校验结果、连续命中数、推理耗时；
- `fire_*_skillN.jpg`：触发切换那一刻输入模型的 6 张图。

---

## 四、迭代

1. C4 加 `--record-agent-video` 继续录制（agent 自动切换的 session 也会写 `skill_switch` 事件）；
   agent 切错时人工按键纠正，这些 session 就是新的训练数据。
2. 新 session 放到 `agents/video/` 后重跑 `visualize_demos.py → build_dataset.py → train_lora.py`。
3. 想换 Cosmos-Reason2-2B：先在 https://huggingface.co/nvidia/Cosmos-Reason2-2B 接受许可，`hf download nvidia/Cosmos-Reason2-2B`，
   训练时加 `--base-model nvidia/Cosmos-Reason2-2B`（同为 Qwen3-VL 架构，代码不用改）。

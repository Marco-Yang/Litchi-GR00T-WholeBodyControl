# 数采 `action.motion_token` 落盘缺陷说明（供修复 agent）

> **给实现 agent：** 请只改本仓库  
> `code/HumanoidWBM/demo/RL_controller/deploy/Litchi-GR00T-WholeBodyControl`  
> （及同树下文档/脚本）。**不要**改评测侧 `demo/RL_controller/Litchi-GR00T-WholeBodyControl` 或 VLA 训练仓，除非用户另行授权。  
> 修完后请按文末 **「修复报告模板」** 整理 diff / 验证结果，交给审核方判断对错。  
> **本文只描述问题与验收标准，不要求你照某一方案硬改；优先满足验收。**

关联 runbook：[`pico_real_data_collection.md`](./pico_real_data_collection.md)（尤其 §0 字段表、§4 启动顺序、§8 事后 refill）。

---

## 1. 现象（必须先对齐的问题）

对 **从未覆写的原盘** 数据集做左右对比：

| 路 | 含义 | 观感 |
|---|---|---|
| 左 | `observation.state` 运动学回放（真机测到的关节） | 站立、胳膊自然下垂 |
| 右 | 原盘 `action.motion_token` → **对齐后的 ZMQ v4**（`cmd_z=0, heading-aligned`）→ SONIC decoder | **从第 0 帧起**抬手、曲臂、呈「走路预备/行走」上肢，并往前走 |

证据目录（抽样已生成）：

```text
/home/zhenyu/workspace/data/demo/outputs/walk_towards_the_green_table_skill1_rtc_2cam/eval_full_stack_wbc/compare_origin_qpos_vs_aligned_v4_20261108_220200/
  2026-11-05-13-35-11/ep001/ep001_L_originQpos_R_originToken_alignedV4_h264.mp4
  2026-11-05-13-35-11/ep005/...
  2026-11-05-13-35-11/ep010/...
  2026-11-05-14-17-12/ep000/...
```

原盘数据根：

```text
/home/zhenyu/workspace/data/demo/dataset/origin/walk_towards_the_green_table/
```

该批 meta 记录策略为 **`policy/sonic_v1_1`**（encoder/decoder/obs-config 均 v1.1）。  
`teleop.stream_mode == 1`（Pico POSE / SMPL 遥操），不是 planner。

**已排除：** 右侧「未对齐旧 v4」（`cmd_z≈0.79`）回放 bug。对比视频日志含  
`v4 token path: installed streamed motion (cmd_z=0, heading-aligned)`，`FIX_OK streamed`。  
曲臂在 **对齐 v4** 下依旧出现 → **问题在落盘 token 内容 / 数采编码链路，不在这次回放脚本。**

数值旁证（`2026-11-05-13-35-11` ep001）：

- 原盘 token vs **当时盘上已 mode2 后处理过的 token**：`L2(mean)≈2.78`，`L2(t=0)≈3.68`（原盘与「可训监督」差极大）
- 另：即便同是 mode2，**离线 ONNX 重编码** vs **机上/在线 encode** 仍常见 `mean L2≈0.25–0.3`（heading/实现差）；不要和上面的 2.78 混为一谈
- 原盘 `token[0]` 范数已 ≈1.67，非「空/静止」token
- 同帧 qpos：肘约 61°/68°、肩 pitch≈0.3（G1 上观感仍是垂臂）；token 解码却是抬手曲臂行走姿

**验收对齐对象（重要）：** 成功标准是落盘 token ≈ **干净 mode2（与 §8 / SMPL+腕+约定 heading 一致）编码结果**，及其解码观感；**不是**令 decode(token) 逐关节等于垂臂 `observation.state`（SONIC latent 本身就不保证这一点）。

---

## 2. 数采设计意图 vs 实际语义（核心错配）

### 2.1 文档宣称

[`pico_real_data_collection.md`](./pico_real_data_collection.md) §0：

- `action.motion_token` ← deploy 的 `token_state`（64D SONIC latent）
- 校验只要求「大量非零」

下游 GR00T `UNITREE_G1_SONIC` **监督的是预测 `motion_token`**，不是回归肘关节。  
因此落盘 token **必须**是「可用、可复现、与 `teleop.smpl_*` 逐步语义一致」的监督；不能只是「当时控机线上碰巧有的 latent」。

### 2.2 实际链路

```text
Pico (pico_manager)
  → ZMQ :5556  protocol v3  (smpl_pose/joints, body_quat_w, joint_pos[29], hands, …)
  → g1_deploy_onnx_ref
       protocol v3 → SetEncodeMode(2)   # SMPL / mode2
       GatherEncoderObservations → encoder ONNX → token_state[64]
  → ZMQ :5557  (同包: body_q + token_state + …)
  → run_data_exporter.py
       observation.state      ← body_q          # 真机实测
       action.motion_token ← token_state      # 参考运动 latent
       teleop.smpl_*       ← Pico SMPL（另路/同帧拼装）
```

**两列同帧抄写本身没有「抄错列」；错在 action 的语义不满足 VLA 监督需求（live 参考 latent ≠ 可复现的 mode2 监督），再叠加腕/heading 构造与事后标准不一致。**

文档 §8 已承认：pipeline **默认事后用 mode2 重填 token，「不再信任录制时的旧 token」**。  
这说明 **live 落盘 token 本就不可信**——修复目标应是：**录制时写出的 token 达到（或非常接近）§8 refill / 干净 mode2 编码的质量**，而不是永远依赖事后覆写。

> **注意（本拷贝树）：** runbook §8 写了 `python -m datasets.data_check refill-tokens`，但当前  
> `deploy/Litchi-GR00T-WholeBodyControl` **顶层未见 `datasets/` 包**。修复时请确认该模块是否未同步进本拷贝；若缺失，用等价的 `sonic_v1_1/model_encoder.onnx` offline encode 做黄金标准即可，并在报告中写明命令。

协议确认：`pico_manager` 调用 `pack_pose_message(...)` 默认 **version=3**（`zmq_planner_sender.py`），与 C++ `SetEncodeMode(2)` 匹配。

---

## 3. 根因拆解（按优先级）

### 根因 A — mode2 不吃肩/肘等 G1 臂关节角，只吃 SMPL 点 + 腕角（架构）

文件：`gear_sonic_deploy/policy/sonic_v1_1/observation_config.yaml`

```yaml
- name: "smpl"
  mode_id: 2
  required_observations:
    - encoder_mode_4
    - smpl_joints_10frame_step1          # 含上肢 SMPL 3D 点，并非「完全无臂信息」
    - smpl_anchor_orientation_heading_10frame_step1
    - motion_joint_positions_wrists_10frame_step1   # G1 关节角仅腕 6D
```

协议侧：v2/v3 收包后 `SetEncodeMode(2)`  
（`.../zmq_endpoint_interface.hpp` **L1710**：`merge_result.motion->SetEncodeMode(2)`）。

**准确表述：** 上肢不是「输入全空」，而是 **没有 G1 肩/肘关节角监督**；解码成弯肘行走姿，来自 SMPL→latent→decoder 的训练先验 + 腕通道信息不足，**不会**被约束成与真机垂臂 `observation.state` 一致。  
勿写成「encoder 完全看不到胳膊」——`smpl_joints` 仍含肘腕等关键点。

### 根因 B — Pico `joint_pos[29]` 几乎全 0，只写 6 个腕关节（实现 / 与 refill 腕解析一致性）

文件：`gear_sonic/scripts/pico_manager_thread_server.py`（**L1430–1487**）

```python
joint_pos = np.zeros(29)
# G1_L_ELBOW_IDX / G1_R_ELBOW_IDX 被注释掉，从不赋值
# 仅写入 23/25/27、24/26/28 左右腕
# 肘 swing 被合并进腕欧拉角
```

对 **mode2** 而言，C++ 本来也只 gather 腕索引，非腕的 0 **不会直接进 encoder**。  
B 的真实危害是：

1. 腕角由「肘并进腕」的现场启发式算出，**未必等于** §8 / Isaac screening 的腕解析；  
2. 维护者易误以为发了全身 `joint_pos`；  
3. 若将来改 mode 或观测，全 0 躯干/臂会埋雷。

### 根因 C — `token_state` ≠ `encode(落盘 teleop.smpl_*[t])` 的干净 1:1（可复现性）

Live 编码还叠：

- 机上 **闭环 heading** / `delta_heading` / IMU init（§8 写的是 **开环 heading** refill）
- Encoder **future-10**（`smpl_joints_10frame_step1`）：线上「未来帧」是实时流；事后对已落盘序列滑窗，边界/时序不同
- Exporter：**SMPL 特征**有延迟门控（`sonic_latency_ms <= 100` 才 `use_smpl`，见 `run_data_exporter.py` **L917**）；**`action.motion_token` 仍无条件抄 `proprio["token_state"]`（L898–901）**  
  → 可能出现：该行 `teleop.smpl_*` 已是 zeros/未更新，但 token 仍是 deploy 当时的非零 latent

→ 同一 parquet 行上 `teleop.smpl_*[t]` 与 `action.motion_token[t]` **不必**可复现对齐。  
VLA / 后处理却常默认它们应对齐——这是监督质量问题的主因之一。

### 根因 D — 命名陷阱（易改错）

| 名字 | 典型值 | 含义 |
|---|---|---|
| `teleop.stream_mode` | 1 = POSE | Pico/UI 流模式 |
| encoder `mode_id` | 2 = smpl | C++ `SetEncodeMode` |

不要把 stream_mode=1 当成「没用 SMPL encoder」。

### 明确不是根因

| 项 | 说明 |
|---|---|
| Exporter 把 token / state 列记反 | 同包字段，抄写正确 |
| 「对齐 v4 回放」导致曲臂 | 原盘 + aligned v4（`cmd_z=0`）仍曲臂；问题在 token |
| `motion_token` 全 0 | 本批非零；是「内容/可复现性错」不是「没记」 |
| 「decode 必须长得像垂臂 qpos」 | 即便修好 mode2 监督，decode 仍可能与 `observation.state` 臂姿有表征差；勿当唯一 KPI |

---

## 4. 修复目标（验收要什么）

### 4.1 产品目标

新采数据满足：

1. **逐步可复现（主 KPI）：** 对同一 episode，用落盘 `teleop.smpl_*`（+ 与写入时相同的腕/heading 规则）经 `sonic_v1_1` mode2 encoder 重算 `token_hat[t]`，与盘上 `action.motion_token[t]` 对齐。  
   - 目标量级：接近 **同一套规则下 offline 自洽**（理想 `max_abs` 在 1/16 量化 1 bin 内，或 `mean L2≪0.25`）。  
   - 原盘相对后处理 token 的 `L2≈2.8` 必须消失。  
   - 若仍保留 live 闭环控机，须说明落盘用的是哪条 token，并保证 **落盘这条** 可复现。
2. **解码观感（次 KPI）：** 落盘 token → ZMQ v4（**必须** heading-aligned / `cmd_z=0`）回放，应与 **同规则干净 mode2 编码再解码** 视觉一致；相对原盘，不应再出现「与可复现 mode2 回放明显不同的系统性抬手曲臂」。  
   - **不要**要求 decode≈垂臂 qpos；三路对比时左 qpos 仅作参考。
3. **事后 refill 可变可选：** §8 / 等价 refill 可留作校验；**新数据不应再「必须 refill 才能训」**。
4. **真机安全：** 不得破坏实时遥操（KP/KD、Revo2/Dex3、急停）。若「控机 token」与「落盘监督 token」分轨，须证明控机链路未回退。

### 4.2 建议修复方向（任选或组合，报告里写清采用哪条）

**方向 1（推荐，与文档 §8 对齐）— 录制时写出「可复现 mode2」token**

- 保证 encoder 输入与 §8 / Isaac mode2 screening **同一套构造规则**：SMPL joints + 腕解析 + **开环（或明确约定的）heading**。
- 若 live 控机仍需闭环 heading，可考虑：**控机用 live token，落盘另写 `action.motion_token_supervised` 或直接覆盖为 open-loop encode 结果**（需改 exporter / deploy 协议时写清兼容性）。
- 修正 `pico_manager`：`joint_pos` 若继续给 mode2 wrists，必须与后处理腕解析一致；不要再「29 维几乎全 0」却假装有全身关节参考。

**方向 2 — 补全上肢进入编码**

- 若产品上要求 token 解码贴近真机臂姿：需要 **改变 encoder 输入定义**（可能不止 wrists），或改用 teleop mode1（VR 3pt）等——**这可能涉及策略/ONNX 训练分布，风险高**，须在报告中论证，不可静默乱改 `observation_config.yaml` 维度。

**方向 3 — 仅加强校验（不够当唯一修复）**

- 数采结束自动：抽样 decode 对比、与 offline encode L2、拒绝「与 smpl 不对齐」的 session。  
- 可作门禁，但 **不能**只加校验不改写入逻辑。

**不要做的事**

- 不要把 `observation.state` 直接塞进 `action.motion_token`（维度/语义都不是 64D SONIC latent）。
- 不要在未对齐 heading/`cmd_z` 的旧 v4 路径上「调通」观感来冒充修好了采集。
- 不要只改评测仓覆写脚本来假装数采已修（那是下游补丁，不是本仓库根治）。

---

## 5. 关键代码锚点（修复时必读）

| 角色 | 路径 |
|---|---|
| 数采 runbook | `docs/notes/g1/pico_real_data_collection.md` |
| Pico 组包 / `joint_pos` | `gear_sonic/scripts/pico_manager_thread_server.py` ~1429–1553 |
| ZMQ pack v3 | `gear_sonic/utils/teleop/zmq/zmq_planner_sender.py` |
| 协议→encode mode2 | `gear_sonic_deploy/.../zmq_endpoint_interface.hpp` **L1710** `SetEncodeMode(2)` |
| Encoder obs / modes | `gear_sonic_deploy/policy/sonic_v1_1/observation_config.yaml` |
| wrists gather | `g1_deploy_onnx_ref.cpp` ~L1759 `motion_joint_positions_wrists_10frame_step1` |
| smpl gather | 同文件 ~L1769 `smpl_joints_10frame_step1` |
| token 编码写入 | `g1_deploy_onnx_ref.cpp`（`GatherEncoderObservations` / `token_state_data_` / publish） |
| 5557 输出 | `gear_sonic_deploy/.../zmq_output_handler.hpp` |
| 落盘 token | `run_data_exporter.py` **L898–901** |
| 落盘 SMPL 门控 | 同文件 **L911–921**（≤100ms 才写入 teleop.smpl；与 token 写入解耦） |
| 官方事后 refill | runbook §8；**本拷贝可能无 `datasets.data_check`，需自备等价 offline encode** |

策略固定 **`sonic_v1_1`**，不要默默换成 `policy/release` 除非报告证明等价。

---

## 6. 建议自测（写入报告）

最少：

1. **单元/离线：** 固定一段已落盘 `teleop.smpl_*`，比较  
   - 新逻辑写出的 token  
   - §8 同款 offline mode2 encode  
   → 报告 L2 / max_abs / 是否逐步对齐。
2. **在线短采：** 采 1–2 条 walking episode（可短），用 aligned-v4 回放 token，与同 episode qpos / SMPL 骨架三路对比视频（路径写进报告）。
3. **回归：** Dex3/Revo2 手指令、录制按键（Grip+A/B）、相机口 5555/5558 不受影响。
4. **文档：** 更新 `pico_real_data_collection.md`：写清 token 语义、是否仍需 §8 refill、验收命令。

---

## 7. 修复报告模板（请原样填好发回审核方）

```markdown
# motion_token 落盘修复报告

## 摘要
- 采用的方向（1/2/3/组合）：
- 一句话说明如何让录制 token 可复现 / 不再系统性曲臂：

## 改动文件列表
| 路径 | 变更类型 | 说明 |
|---|---|---|

## 关键逻辑（before → after）
- Pico joint_pos / 腕：
- heading（闭环 vs 开环）与落盘关系：
- exporter 写入字段：
- 是否双轨（控机 token vs 监督 token）：

## 验证结果
- offline encode vs 落盘：指标与命令
- 短采回放视频路径
- 未通过项 / 残留风险

## 文档更新
- pico_real_data_collection.md 段落链接：

## Diff 统计
- `git diff --stat` 输出：
- 完整 patch 或 commit hash：
```

---

## 8. 给审核方的判据（你改完后对方会用这些判）

1. 是否仍把「非零 token」当成唯一成功条件（不够）。  
2. 落盘 token 是否与 **落盘 SMPL** 在 mode2 规则下可复现对齐。  
3. 新采样本 decode 是否与 **干净 mode2** 回放一致；原盘相对 mode2 的那种系统性偏离是否消失。  
   （不以「等于垂臂 qpos」为否决条件。）  
4. 是否引入未说明的 ONNX/obs 维数破坏或真机不安全改动。  
5. 报告是否可独立复现验证命令；若本树无 `datasets.data_check`，是否提供了替代黄金标准脚本。

---

*文档版本：2026-11-08（校对修订）。问题定性基于 origin 对比视频 + deploy 树源码；评测侧 mode2 覆写仅为下游补救。*

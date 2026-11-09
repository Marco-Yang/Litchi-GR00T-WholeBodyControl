# Walking 数据 mode-2 Isaac 动力学核验（含可视化）

把某个清洗后的 session（如 `datasets/walking/<outputs 同名目录>/`）里的 **SMPL + 手部 qpos** 组装成 SONIC **encode mode 2（smpl）**，在 **Isaac Sim GUI** 里一次拉起多台 G1（每台对应 1 条 episode），同时用 GPU 做关节/SMPL 跳变检测，方便你肉眼观察平衡与抖动。

## 目录

| 文件 | 作用 |
|---|---|
| `convert_to_mode2.py` | LeRobot → `robot_motions/` + `smpl_motions/` |
| `analyze_joint_stability.py` | GPU 检测 state/SMPL/手/重力跳变 |
| `isaac_batch_validate.py` | Isaac 多 G1 批量回放（**默认开 GUI**） |
| `run_pipeline.py` | 一键：转换 → 分析 → Isaac（**默认不重写** `action.motion_token`；旧盘才加 `--token-refill`） |

## 环境

必须用带 Isaac Lab 的环境（本机推荐 `sikan`），并在仓库根目录执行：

```bash
conda activate sikan
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl

# 确认能 import
python -c "import isaaclab, torch; print(torch.cuda.is_available())"
```

Checkpoint 默认：`sonic_v1_1/last.pt`（SMPL encoder / mode 2）。

首次开 Isaac 若卡在 EULA，先：

```bash
export OMNI_KIT_ACCEPT_EULA=YES
export CUDA_VISIBLE_DEVICES=0
```

## 推荐：GUI 可视化 + 自动检测（一键）

先用 **少量 G1** 方便观察（例如 4～8 台）：

```bash
conda activate sikan
cd /home/user/YZY/HumanoidWBM/RL_controller/Deploy/Litchi-GR00T-WholeBodyControl
export OMNI_KIT_ACCEPT_EULA=YES
export CUDA_VISIBLE_DEVICES=0

python -m datasets.data_check pipeline \
  --dataset datasets/walking/2026-11-01-15-43-46 \
  --work-dir datasets/data_check/runs/walking_gui \
  --num-envs 8 \
  --env-spacing 3.0
```

指定 **从第几条到第几条**（含两端）：

```bash
# episode 0..15（共 16 条）；num_envs 建议 ≤ 区间长度
python -m datasets.data_check pipeline \
  --dataset datasets/walking/2026-11-01-15-43-46 \
  --work-dir datasets/data_check/runs/walking_0_15 \
  --start 0 --end 15 \
  --num-envs 8 \
  --env-spacing 3.0

# 等价写法
python -m datasets.data_check pipeline \
  --dataset datasets/walking/2026-11-01-15-43-46 \
  --work-dir datasets/data_check/runs/walking_0_15 \
  --episodes 0-15 \
  --num-envs 8
```

Shell 快捷：

```bash
# args: num_envs  start  end  [work_dir]
bash datasets/data_check/run_gui.sh 8 0 15
bash datasets/data_check/run_gui.sh 4 20 27 datasets/data_check/runs/batch2
```

- 会弹出 **Isaac Sim 窗口**，地面上并行多台 G1。
- 关节跳变报告：`.../joint_stability.json`（另有 `.csv`）
- Isaac 指标：`.../isaac_eval/metrics/`

只看离散几条：

```bash
python -m datasets.data_check pipeline \
  --dataset datasets/walking/2026-11-01-15-43-46 \
  --work-dir datasets/data_check/runs/suspect_gui \
  --episodes 0,3,7,12 \
  --num-envs 4 \
  --env-spacing 4.0
```

## 分步命令

### 1) 转成 mode-2 运动包

```bash
python -m datasets.data_check convert \
  --dataset datasets/walking/2026-11-01-15-43-46 \
  --output datasets/data_check/runs/walking_gui/mode2 \
  --start 0 --end 15
```

### 2) GPU 关节/SMPL 稳定性分析（无需开 Isaac）

```bash
python -m datasets.data_check analyze \
  --dataset datasets/walking/2026-11-01-15-43-46 \
  --output datasets/data_check/runs/walking_gui/joint_stability.json \
  --start 0 --end 15
```

关注字段：`unstable_episode_indices`、`flags`（`ankle_jump` / `balance_joint_jump` / `smpl_jump` / `gravity_tilt_jump`）。

### 3) Isaac GUI 批量回放（多 G1）

> `isaac` 子命令读的是已转换的 `mode2` 目录；先用带 `--start/--end` 的 `convert`/`pipeline` 生成对应子集。

```bash
python -m datasets.data_check isaac \
  --mode2-dir datasets/data_check/runs/walking_0_15/mode2 \
  --checkpoint sonic_v1_1/last.pt \
  --output datasets/data_check/runs/walking_0_15/isaac_eval \
  --num-envs 8 \
  --env-spacing 3.0
```

无界面压测（可加大 `num_envs`）：

```bash
python -m datasets.data_check isaac \
  --mode2-dir datasets/data_check/runs/walking_gui/mode2 \
  --output datasets/data_check/runs/walking_gui/isaac_eval_headless \
  --num-envs 64 \
  --headless
```

先打印命令不启动：

```bash
python -m datasets.data_check isaac \
  --mode2-dir datasets/data_check/runs/walking_gui/mode2 \
  --output /tmp/dry --num-envs 8 --gui --dry-run
```

## 观察建议

1. 先跑 `analyze`，记下 `unstable_episode_indices`。
2. 用 `--episodes` + `--gui --num-envs 4` 只开可疑条，`--env-spacing 4` 拉开间距。
3. GUI 里重点看：脚是否瞬间抽动、骨盆是否突然倾斜、手臂是否闪现。
4. 对照 CSV 里该 episode 的 `first_event_frames`（帧号 ≈ 时间×50）。

## 说明

- **mode 2** = Isaac 侧 `++use_encoder=smpl`（与部署 ZMQ protocol v2/v3 的 encode_mode=2 对应）。
- 手部：`teleop.*_hand_joints` 写入 robot dof 的 14-D hand 段 + `hand_action_*`。
- GUI 默认开启；只有显式 `--headless` 才关窗口。
- `num_envs` 不会超过当前 mode2 目录里的 clip 数。

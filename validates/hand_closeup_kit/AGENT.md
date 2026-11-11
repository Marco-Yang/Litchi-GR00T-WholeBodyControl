# Deploy-host Agent：Revo2 局部手部视频生成 Kit

你是部署主机上的 agent。本目录是**自包含工具包**：用这里的脚本 + `assets/` 里的 scene XML，生成左右手开合特写 H.264 视频。  
**不要**再去改全栈 `validates/lib/run_mujoco_vla.sh` 或真机 deploy；本任务只验证 / 渲染手部 MJCF。

## 0. 本目录内容（全部要用到）

```
validates/hand_closeup_kit/
  AGENT.md                      ← 你正在读的操作说明
  run_all.sh                    ← 一键：检验 → 渲染 → H.264
  check_xml.py                  ← 仅加载 XML / 检查 Revo2 关节
  render_mjcf_hand_closeup.py   ← 左右分屏手部开合渲染
  assets/
    scene_43dof_revo2.xml                 ← 默认 scene（与早期对比片同源）
    g1_29dof_with_revo2.xml               ← 被 scene include；meshdir 已指回仓库 meshes
    scene_43dof_revo2_vizclean.xml        ← 可选，少叠影
    g1_29dof_with_revo2_vizclean.xml
  out/                          ← run_all.sh 自动创建，写产物
```

依赖的 **mesh STL 不复制进 kit**（体积大）。`assets/*.xml` 的 `meshdir` 指向：

`../../../gear_sonic/data/robot_model/model_data/g1/meshes`

因此 **kit 必须留在** `Litchi-GR00T-WholeBodyControl/validates/hand_closeup_kit/`，且仓库里上述 `meshes/` 目录存在。

## 1. 目标产物

成功后应存在：

- `out/revo2_hand_closeup_h264.mp4`（**交付文件**，H.264 / yuv420p）
- `out/revo2_hand_closeup_mid.png`（中间帧抽查：左右都应能看到手指）
- `out/run_all.log`

## 2. 强制执行顺序（按序做，失败即停）

### Step A — 定位仓库与 kit

```bash
# 部署机上 WBC 根目录（按现场改）
export WBC_ROOT="/path/to/Litchi-GR00T-WholeBodyControl"
cd "$WBC_ROOT/validates/hand_closeup_kit"
pwd
ls assets/scene_43dof_revo2.xml assets/g1_29dof_with_revo2.xml
ls "$WBC_ROOT/gear_sonic/data/robot_model/model_data/g1/meshes" | head
```

若 `hand_closeup_kit` 不存在：先 `git pull` 含本 kit 的 `base` 分支。

### Step B — Python / ffmpeg

```bash
# 优先仿真 venv
export SIM_PY="${WBC_ROOT}/.venv_sim/bin/python"
"$SIM_PY" -c "import mujoco, imageio, cv2, numpy; print('ok', mujoco.__version__)"

# ffmpeg：优先项目二进制，否则 PATH
export FFMPEG_BIN="${FFMPEG_BIN:-}"
# 常见路径示例（按现场改）：
#   $WBC_ROOT/../VLA/.tools/bin/ffmpeg
#   /home/user/.../demo/VLA/.tools/bin/ffmpeg
command -v ffmpeg
```

缺包时在对应 venv 安装：`mujoco`、`imageio`、`imageio-ffmpeg`、`opencv-python-headless`、`numpy`。

### Step C — 无头 GL

部署机通常无显示器：

```bash
export MUJOCO_GL=egl
# 若 egl 失败再试：
# export MUJOCO_GL=osmesa
```

### Step D — 一键跑

```bash
cd "$WBC_ROOT/validates/hand_closeup_kit"
chmod +x run_all.sh
./run_all.sh
```

更干净画面（可选）：

```bash
SCENE_VARIANT=vizclean ./run_all.sh
```

### Step E — 验收

```bash
ls -lh out/revo2_hand_closeup_h264.mp4 out/revo2_hand_closeup_mid.png
# 人工或用工具看 mid.png：左、右两格都必须出现多指手，不能是肩膀/前臂截面
grep -E 'CHECK_OK|DONE deliverable|ERROR' out/run_all.log
```

**通过标准**

1. log 有 `CHECK_OK` 与 `DONE deliverable=..._h264.mp4`
2. `hand_joints` 约为 22；无 `rubber_hand` body
3. mid.png / 视频左右两侧都能看到 **Revo2 手指**开合
4. 最终文件名带 `_h264.mp4`，编码器为 libx264（不要只留 raw/mp4v）

## 3. 分步命令（调试用）

```bash
cd "$WBC_ROOT/validates/hand_closeup_kit"
export MUJOCO_GL=egl
export SIM_PY="${WBC_ROOT}/.venv_sim/bin/python"

"$SIM_PY" ./check_xml.py
"$SIM_PY" ./render_mjcf_hand_closeup.py \
  --scene-xml ./assets/scene_43dof_revo2.xml \
  --out-mp4 ./out/revo2_hand_closeup_raw.mp4 \
  --mid-png ./out/revo2_hand_closeup_mid.png

"$FFMPEG_BIN" -y -i ./out/revo2_hand_closeup_raw.mp4 \
  -c:v libx264 -pix_fmt yuv420p -movflags +faststart \
  ./out/revo2_hand_closeup_h264.mp4
rm -f ./out/revo2_hand_closeup_raw.mp4
```

## 4. 逻辑说明（agent 排错用）

1. **加载** `scene_43dof_revo2.xml` → include `g1_29dof_with_revo2.xml` → 从 `meshdir` 读 STL。  
2. **检验** 关节名含 thumb/index/middle/ring/pinky；禁止出现 `*_rubber_hand` body（那是宇树橡胶手）。  
3. **摆臂** 肩/肘把双手放到胸前。  
4. **动画** 手指在 joint range 上按 `(1-cos(2πt))/2` 开合。  
5. **相机**  
   - 左：`azimuth≈160`  
   - 右：`azimuth≈-20`（**禁止**用 ~30，否则顺着前臂看向肩膀，右手出画）  
6. **拼左右图** → imageio 写 raw → **ffmpeg libx264** 出交付件。

## 5. 常见失败 → 处理

| 现象 | 原因 | 处理 |
|------|------|------|
| `from_xml_path` / mesh 找不到 | kit 被挪出仓库，或 `meshes/` 未拉全 | 保持 kit 路径；`git lfs pull`；确认 `gear_sonic/.../g1/meshes` 有 STL |
| `OpenGL platform library has not been loaded` / GLFW DISPLAY | 无头未设 GL | `export MUJOCO_GL=egl` |
| `Image height > framebuffer height` | 渲染分辨率 > 480 | 用脚本默认 480；不要擅自加大 |
| 右侧看不到手 | 右相机 azimuth 错误 | 用本 kit 脚本（已修成 -20）；不要用旧 az=30 |
| 手部闪烁 / 双影 | 旧 XML 双 visual geom | `SCENE_VARIANT=vizclean ./run_all.sh` |
| `No module named mujoco` | Python 环境错 | 改用 `$WBC_ROOT/.venv_sim/bin/python` |
| 只有 raw、播放器打不开 | 未 H.264 重编码 | 必须跑 ffmpeg 步骤；交付 `_h264.mp4` |
| 渲出来是橡胶/Dex3 外形 | 指错 XML | 确认 `--scene-xml` 指向本 kit `assets/scene_43dof_revo2*.xml` |

## 6. 禁止事项

- 不要把 kit 复制到仓库外又忘记改 `meshdir`。  
- 不要用系统乱装的 ffmpeg 若它没有 `libx264`；优先项目 `.tools/bin/ffmpeg`。  
- 不要用 OpenCV `VideoWriter` 的 mp4v 作为最终交付。  
- 不要为了“出手”去改真机 `policy/sonic_v1_1` 或 C++ deploy。

## 7. 完成后向用户汇报

用两三句话报告：

1. `CHECK_OK` 是否通过、`hand_joints` 数量  
2. 交付路径：`.../hand_closeup_kit/out/revo2_hand_closeup_h264.mp4`  
3. 若用了 vizclean 请注明；若右侧曾出画，确认 mid.png 已可见双手  

更完整的原理说明见仓库：  
`docs/notes/g1/mujoco_xml_render_hand_closeup.md`

# MuJoCo XML → 局部灵巧手渲染视频

本文说明如何对 **G1 + 强脑 Revo2** 的 MuJoCo scene XML 做加载检验，并生成左右手局部开合特写（H.264）。  
对应交付物示例：`/home/zhenyu/workspace/outputs/videos/revo2_hand_closeup_h264.mp4`。

可执行脚本：

[`validates/lib/render_mjcf_hand_closeup.py`](../../../validates/lib/render_mjcf_hand_closeup.py)

**部署主机给 Agent 用的自包含工具包**（推荐）：

[`validates/hand_closeup_kit/`](../../../validates/hand_closeup_kit/) — 读其中 [`AGENT.md`](../../../validates/hand_closeup_kit/AGENT.md)，一键 `./run_all.sh`。

---

## 1. 相关 XML（旧非 vizclean / 与 `outputs/videos` 对比片同源）

| 角色 | 路径 |
|------|------|
| **Scene（实际 `ROBOT_SCENE`）** | `gear_sonic/data/robot_model/model_data/g1/scene_43dof_revo2.xml` |
| **机体（被 scene include）** | 同目录 `g1_29dof_with_revo2.xml` |
| mesh | 同目录 `meshes/`（Revo2 STL：`left_thumb_*` / `right_index_*` 等） |

`scene_43dof_revo2.xml` 核心只有 include + 地面：

```xml
<mujoco model="g1_41dof_revo2 scene">
  <!-- body 29 + Revo2 6+6 -->
  <include file="g1_29dof_with_revo2.xml"/>
  ...
</mujoco>
```

`g1_29dof_with_revo2.xml` 注释写明：**sonic G1 body + BrainCo Revo2 hands**。  
关节名为 `left/right_{thumb,index,middle,ring,pinky}_*_joint`（约 **22** 个 finger joint），**不是**宇树橡胶手 / Dex3。

同目录还有部署副本：`gear_sonic_deploy/g1/scene_43dof_revo2.xml`。  
较新的 validates 默认改用无双 geom 叠影的 vizclean：

- `validates/assets/scene_43dof_revo2_vizclean.xml`
- `validates/assets/g1_29dof_with_revo2_vizclean.xml`

旧 XML 可加载、关节正确，但视觉层常有 **双 geom 叠影/闪烁**；检验结构用旧文件即可，要干净画面优先 vizclean。

---

## 2. 逻辑概览

```
scene XML
  → mujoco.MjModel.from_xml_path
  → 检验：nq / finger joints / 无 rubber_hand body / meshdir
  → 摆臂（肩肘腕），使双手到胸前
  → 每帧：手指 open↔close（按 joint range 插值）
  → 左右各一台 FREE camera，lookat = 掌指中心
  → EGL offscreen Renderer → 左右拼图帧序列
  → imageio 写中间 mp4
  → 项目 ffmpeg 重编码 libx264 yuv420p → *_h264.mp4
```

**相机要点（踩过的坑）**

| 手 | `lookat` | `distance` | `azimuth` | `elevation` |
|----|----------|------------|-----------|-------------|
| 左 | 左掌指 body 均值 | `0.18` | `160` | `-20` |
| 右 | 右掌指 body 均值 | `0.18` | **`-20`** | `-20` |

右侧若用约 `azimuth=30`，视线会**顺着前臂朝肩膀**，手部容易出画（此前右侧“看不到手”的原因）。右侧必须用外侧/偏正面方位（`-20` 左右）。

**手指动画**

对所有名字含 `thumb|index|middle|ring|pinky` 的关节：

\[
q = lo + a\cdot(hi-lo),\quad a=\tfrac{1-\cos(2\pi t)}{2}
\]

`t∈[0,1]` 走完一轮即为开→合→开。

**无头显示**

本机无 `DISPLAY` 时必须：

```bash
export MUJOCO_GL=egl   # 优先；不行再试 osmesa
```

分辨率勿超过模型 offscreen framebuffer（该 scene 常见上限 **480**）：`width/height ≤ 480`。

---

## 3. 加载检验（不渲视频）

```bash
WBC=/home/zhenyu/workspace/code/HumanoidWBM/demo/RL_controller/Litchi-GR00T-WholeBodyControl
SIM_PY=$WBC/.venv_sim/bin/python
SCENE=$WBC/gear_sonic/data/robot_model/model_data/g1/scene_43dof_revo2.xml

MUJOCO_GL=egl $SIM_PY - <<PY
import mujoco
from pathlib import Path
scene = Path("$SCENE")
m = mujoco.MjModel.from_xml_path(str(scene))
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)
hands = []
for i in range(m.njnt):
    n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) or ""
    if any(k in n for k in ("thumb", "index", "middle", "ring", "pinky")):
        hands.append(n)
rubber = [
    mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i)
    for i in range(m.nbody)
    if "rubber_hand" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or "")
]
print("nq", m.nq, "hand_joints", len(hands), "rubber_bodies", rubber)
assert len(hands) == 22 and not rubber
print("XML_OK Revo2")
PY
```

期望：`hand_joints 22`，`rubber_bodies []`，打印 `XML_OK Revo2`。

---

## 4. 一键生成特写视频

```bash
WBC=/home/zhenyu/workspace/code/HumanoidWBM/demo/RL_controller/Litchi-GR00T-WholeBodyControl
SIM_PY=$WBC/.venv_sim/bin/python
FFMPEG=/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/.tools/bin/ffmpeg
SCENE=$WBC/gear_sonic/data/robot_model/model_data/g1/scene_43dof_revo2.xml
OUT=/home/zhenyu/workspace/outputs/videos
mkdir -p "$OUT"

# 1) 渲染中间片（OpenCV/imageio 编码器，仅作中间件）
MUJOCO_GL=egl $SIM_PY $WBC/validates/lib/render_mjcf_hand_closeup.py \
  --scene-xml "$SCENE" \
  --out-mp4 "$OUT/revo2_hand_closeup_raw.mp4" \
  --mid-png "$OUT/revo2_hand_closeup_mid.png" \
  --fps 30 --seconds 5

# 2) 强制 H.264 交付（工作区约定）
"$FFMPEG" -y -i "$OUT/revo2_hand_closeup_raw.mp4" \
  -c:v libx264 -pix_fmt yuv420p -movflags +faststart \
  "$OUT/revo2_hand_closeup_h264.mp4"

rm -f "$OUT/revo2_hand_closeup_raw.mp4"
ls -lh "$OUT/revo2_hand_closeup_h264.mp4" "$OUT/revo2_hand_closeup_mid.png"
```

vizclean 版把 `--scene-xml` 换成：

`$WBC/validates/assets/scene_43dof_revo2_vizclean.xml`

---

## 5. 脚本内部对应关系

文件：`validates/lib/render_mjcf_hand_closeup.py`

| 步骤 | 函数 / 代码位置 | 作用 |
|------|-----------------|------|
| 加载 XML | `MjModel.from_xml_path` | 解析 scene + include 的机体/mesh |
| 摆臂 | `_set_q(...)` 肩/肘/腕 | 双手移到胸前便于特写 |
| 收集手指 | `_hand_joints` | 按关节名过滤 Revo2 DOF |
| 掌心 lookat | `_hand_center` | 多 body 均值，比单 tip 稳 |
| 动画 | `amt = (1-cos(2πt))/2` | 平滑开合 |
| 相机 | `cam_l.azimuth=160`, `cam_r.azimuth=-20` | 左右外侧正面，避免右臂遮挡 |
| 拼图 | `np.concatenate([L, sep, R], axis=1)` | 左右分屏 |
| 写盘 | `imageio.mimsave` | 中间 mp4；交付前必须 ffmpeg→H.264 |

CLI：

```text
--scene-xml   必填，scene XML
--out-mp4     必填，中间/原始 mp4
--mid-png     可选，中间帧 PNG 抽查
--fps --seconds --width --height
```

---

## 6. 与全栈 validates 视频的关系

`outputs/videos/ep000_L_originVLA_OL_R_originVLA_CL_h264.mp4` 全机仿真用的也是同一套旧 scene（sim.log 里 `ROBOT_SCENE=.../scene_43dof_revo2.xml`），经 `run_sim_record.py --revo2-hand` 整机录制，不是本特写脚本。

本特写脚本只服务：**快速确认 XML 是否 Revo2、mesh/关节是否可渲、双手局部外观**。  
整机 OL/CL/RTC 对比仍走 `validates/lib/run_mujoco_vla.sh`。

---

## 7. 验收清单

- [ ] `from_xml_path` 成功，`hand_joints==22`，无 `*_rubber_hand` body  
- [ ] `MUJOCO_GL=egl` 下 Renderer 能出图  
- [ ] 左右分屏**都能**看到手指开合（右侧 az≠顺着胳膊）  
- [ ] 最终文件为 `*_h264.mp4`（`libx264` + `yuv420p`），优先用项目 `demo/VLA/.tools/bin/ffmpeg`  
- [ ] 若画面闪烁严重，改用 `validates/assets/*_vizclean.xml` 再渲一版对比  

文档版本：2026-11-11（对照 `scene_43dof_revo2.xml` + `revo2_hand_closeup_h264.mp4`）。

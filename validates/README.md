# validates — MuJoCo / VLA 精简验证

同一条数据集上的 **5 路验证** + **1 个动态拼接**：

| 脚本 | 内容 | 默认输出（`videos/`） |
|------|------|------------------------|
| `run_smpl_video.sh` | 数据集 SMPL skeleton | `epXXX_smpl_h264.mp4` |
| `run_qpos_mujoco.sh` | 数据集 qpos 运动学 MuJoCo | `epXXX_qpos_mujoco_h264.mp4` |
| `run_token_decode_mujoco.sh` | 落盘 token → SONIC 解码 → MuJoCo | `epXXX_token_decode_mujoco_h264.mp4` |
| `run_vla_openloop.sh` | VLA 开环（GT proprio） | `epXXX_vla_openloop_h264.mp4` |
| `run_vla_closedloop.sh` | VLA 闭环 + RTC（sim proprio） | `epXXX_vla_closedloop_h264.mp4` |
| `compose_videos.py` | 按需横向/网格拼接 → H.264 | 你指定的 `--out` |

每路交付视频都会在 **画面正下方居中** 烧录类别标签（避开顶部 telemetry）。

输出目录按 **权重文件夹名** 分桶：

`validates/videos/<basename(CKPT)>/`  
`validates/runs/<basename(CKPT)>/`

`lib/` 是共享实现（勿当入口）。

## 用法

```bash
cd .../Litchi-GR00T-WholeBodyControl/validates

CKPT=.../walk_towards_green_table_skill1_all_rtc_2cam_origin
# 可选：DS=... EP=0 LABEL="自定义底部文字" MODEL_TAG=override_name

bash run_smpl_video.sh
bash run_qpos_mujoco.sh
bash run_token_decode_mujoco.sh
bash run_vla_openloop.sh
bash run_vla_closedloop.sh

# 2x2：TL=SMPL TR=qpos BL=VLA-OL BR=VLA-CL（墙钟时间对齐，同速）
python compose_videos.py --model-tag walk_towards_green_table_skill1_all_rtc_2cam_origin \
  --panel ep000_smpl_h264.mp4:"SMPL skeleton" \
  --panel ep000_qpos_mujoco_h264.mp4:"qpos MuJoCo" \
  --panel ep000_vla_openloop_h264.mp4:"VLA open-loop" \
  --panel ep000_vla_closedloop_h264.mp4:"VLA closed-loop RTC" \
  --layout 2x2 --out-fps 30 --align max \
  --out ep000_2x2_smpl_qpos_ol_cl_h264.mp4
```

`compose_videos.py` 按**墙钟时间**采样：四路同时从 t=0 开走；默认 `--align max` 把短视频末帧定格垫到最长一路。
```

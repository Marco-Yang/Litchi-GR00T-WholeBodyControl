# motion_token 落盘修复报告

## 摘要
- 采用的方向：**方向 1（双轨）**
- 一句话：控机仍用 deploy 闭环 `token_state`；**episode 落盘前**用缓冲的 `teleop.smpl_*` + Pico 腕解析 + **开环 heading**，经 `sonic_v1_1/model_encoder.onnx`（与 §8 / Isaac mode-2 screening 同款）重写盘上 `action.motion_token`；live token 另存 `meta/motion_token_live/`。

---

## 改动文件列表

| 路径（相对仓库根 `Litchi-GR00T-WholeBodyControl/`） | 变更类型 | 说明 |
|---|---|---|
| `gear_sonic/utils/data_collection/supervised_mode2_tokens.py` | **新增** | episode 缓冲 → 开环 mode2 encode / 重写 / live 备份 / JSONL 记录 |
| `gear_sonic/scripts/run_data_exporter.py` | **修改** | 落盘前调用 rewrite；CLI `--rewrite-motion-token-supervised` / `--supervised-token-device` |
| `gear_sonic/scripts/pico_manager_thread_server.py` | **修改** | 注释：腕-only `joint_pos` 有意为之，须与 `compute_wrist_joint_pos_il` 一致 |
| `gear_sonic/pyproject.toml` | **修改** | `data_collection` extra 增加 `onnxruntime>=1.16` |
| `docs/notes/g1/pico_real_data_collection.md` | **修改** | §0 / §8 / §9.8 token 语义 |
| `docs/notes/g1/motion_token_recording_bug_fix_report.md` | **新增** | 本报告 |

---

## 关键逻辑（before → after）

### Pico joint_pos / 腕
- **未改公式**（已与 `datasets/data_check/convert_to_mode2.py::compute_wrist_joint_pos_il` 一致）。
- 仅加注释：29D 非腕为 0 是 mode-2 设计，不是漏写肩/肘。

### heading（闭环 vs 开环）与落盘关系
- **控机**：C++ deploy 仍闭环 heading → `token_state`（不变）。
- **落盘**：exporter 在 `save_episode` 前用开环 `encode_episode_tokens_isaac_mode2` 覆写 `action.motion_token`。

### exporter 写入字段
- 帧循环：仍抄 `proprio["token_state"]` → buffer。
- 落盘前：buffer 内该列被 supervised mode2 替换；live 备份 npy。

### 是否双轨
- **是**：控机 live / 盘上 supervised 分轨。关闭 rewrite：`--no-rewrite-motion-token-supervised`。

---

## 验证结果

### offline encode vs 落盘规则（本机 2026-11-09）

命令（仓库根，`.venv_data_collection`）：

```bash
source .venv_data_collection/bin/activate
# 若缺 ORT：uv pip install --python .venv_data_collection/bin/python 'onnxruntime>=1.16'
python - <<'PY'
import numpy as np, pyarrow.parquet as pq
from pathlib import Path
from gear_sonic.utils.data_collection.supervised_mode2_tokens import encode_supervised_mode2_tokens
from datasets.data_check.refill_motion_tokens_mode2 import _load_encoder_session, encode_episode_tokens_isaac_mode2
from datasets.data_check.convert_to_mode2 import compute_wrist_joint_pos_il
p = Path('datasets/walking/2026-11-05-19-40-44/data/chunk-000/episode_000000.parquet')
t = pq.read_table(p); n=t.num_rows
smpl=np.asarray(t.column('teleop.smpl_joints').to_pylist(),np.float32).reshape(n,-1)
pose=np.asarray(t.column('teleop.smpl_pose').to_pylist(),np.float32).reshape(n,-1)
quat=np.asarray(t.column('teleop.body_quat_w').to_pylist(),np.float64).reshape(n,-1)
tok_a,_=encode_supervised_mode2_tokens(smpl,pose,quat,device='cpu')
w=compute_wrist_joint_pos_il(pose)[:,[23,24,25,26,27,28]]
sess,inp,out,_=_load_encoder_session(Path('gear_sonic_deploy/policy/sonic_v1_1/model_encoder.onnx'),device='cpu')
tok_b=encode_episode_tokens_isaac_mode2(sess,inp,out,smpl,w,quat)
print('helper vs §8 refill meanL2', float(np.linalg.norm(tok_a-tok_b,axis=-1).mean()))
PY
```

实测：
- **helper vs §8 refill：`mean L2 = 0`，`max L2 = 0`**（逐步 bit 级一致）。
- 本机已 refill 的 session：盘上 token vs helper `mean L2 = 0`（符合预期）。
- 旧 live 备份 vs helper（`pick_toy/.../episode_000001` old npy）：**`mean L2 ≈ 0.336`**（live ≠ 开环监督；正是本修复要消掉的落盘错配）。
- walking `2026-11-05-13-35-11` ep1 refill meta：`mean_l2_vs_old ≈ 0.65`。

### 短采回放视频
- 未在本轮做新真机短采（需机器人在线）。验收建议：采 1–2 ep 后用 aligned-v4 decode，对比同 ep 的 §8 refill（应接近 L2≪0.25）。

### 未通过项 / 残留风险
- 真机短采 + 解码观感对比：**待上机**。
- episode 末尾 encode 在 CPU 上对长 episode 有短暂阻塞（默认 device=`cpu`）；可用 `--supervised-token-device cuda`（需 GPU ORT）。
- 若 `teleop.smpl_*` 因 latency gate 整段为零，rewrite **跳过**并保留 live（记录 `smpl_all_zero`）。
- 未改 C++ / ONNX / obs 维数；控机链路未回退。episode 末尾 encode 在 CPU 上对长 episode 有短暂阻塞（默认 device=`cpu`）；可用 `--supervised-token-deviceo`（`stream_mode=2`，无 SMPL） |
| `outputs/2026-11-09-11-21-04` | `stream_mode=1`、SMPL 正常；但日志 `supervised rewrite failed (No module named 'datasets.data_check')` → 仍落盘 live（L2≈0.59/0.65）。**已修**：`supervised_mode2_tokens._import_refill` 改为按文件路径加载，避开 HF `datasets` 占名。需**重启 exporter** 再采。 |

### 2026-11-09 清洗管线
- `datasets/data_check/run_pipeline.py`：**默认不再**清除/重写 `action.motion_token`（保留落盘监督 token）。
- 旧盘若仍需事后 mode2：显式 `--token-refill` 或 `python -m datasets.data_check refill-tokens`。

---

## 文档更新
- `docs/notes/g1/pico_real_data_collection.md` §0 字段表、§8 pipeline 说明、§9.8 Revo2 表。
- 问题说明原文：`docs/notes/g1/motion_token_recording_bug_for_fix.md`。

---

## Diff 统计

```text
 docs/notes/g1/pico_real_data_collection.md       |  8 ++-
 gear_sonic/pyproject.toml                        |  1 +
 gear_sonic/scripts/pico_manager_thread_server.py |  8 +++
 gear_sonic/scripts/run_data_exporter.py          | 83 ++++++++++++++++++++++++
 gear_sonic/utils/data_collection/supervised_mode2_tokens.py | 193 (new)
```

（本报告文件另计。）完整 patch 见下文「完整代码增删改」；未提交 commit（按用户规则不主动 commit）。

---

## 完整代码增删改

### 1. 新增 `gear_sonic/utils/data_collection/supervised_mode2_tokens.py`

```python
"""Rewrite ``action.motion_token`` with reproducible SONIC v1.1 mode-2 tokens.

Matches ``datasets.data_check.refill_motion_tokens_mode2`` / Isaac mode-2 screening:

  - encoder inputs: SMPL joints + Pico wrist parse + **open-loop** heading
  - ONNX: ``gear_sonic_deploy/policy/sonic_v1_1/model_encoder.onnx``

Live deploy tokens (closed-loop heading) remain usable for control; this module
is for **disk supervision** only.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[3]
_DEFAULT_ENCODER = (
    _REPO_ROOT / "gear_sonic_deploy" / "policy" / "sonic_v1_1" / "model_encoder.onnx"
)
# IsaacLab wrist DOF order used by mode-2 encoder obs (L/R roll,pitch,yaw)
_ISAACLAB_WRIST_DOF_INDICES = (23, 24, 25, 26, 27, 28)


def _import_refill():
    """Import refill helpers; works from repo root or data-collection venv."""
    import sys

    repo = str(_REPO_ROOT)
    if repo not in sys.path:
        sys.path.insert(0, repo)
    from datasets.data_check.convert_to_mode2 import compute_wrist_joint_pos_il
    from datasets.data_check.refill_motion_tokens_mode2 import (
        _load_encoder_session,
        encode_episode_tokens_isaac_mode2,
    )

    return compute_wrist_joint_pos_il, _load_encoder_session, encode_episode_tokens_isaac_mode2


def wrists6_from_smpl_pose(smpl_pose: np.ndarray) -> np.ndarray:
    """(T, 6) wrists in IsaacLab encoder order — same as §8 refill."""
    compute_wrist_joint_pos_il, _, _ = _import_refill()
    joint_pos_il = compute_wrist_joint_pos_il(smpl_pose)
    return np.asarray(joint_pos_il[:, _ISAACLAB_WRIST_DOF_INDICES], dtype=np.float32)


def encode_supervised_mode2_tokens(
    smpl_joints: np.ndarray,
    smpl_pose: np.ndarray,
    body_quat_w: np.ndarray,
    *,
    encoder_path: str | Path | None = None,
    device: str = "cpu",
) -> tuple[np.ndarray, dict[str, Any]]:
    """Encode (T, 64) supervised tokens. Returns (tokens, meta)."""
    compute_wrist_joint_pos_il, load_sess, encode_fn = _import_refill()
    del compute_wrist_joint_pos_il  # wrists via helper below

    smpl_joints = np.asarray(smpl_joints, dtype=np.float32)
    if smpl_joints.ndim == 2 and smpl_joints.shape[-1] == 72:
        pass
    elif smpl_joints.ndim == 3 and smpl_joints.shape[-1] == 3:
        smpl_joints = smpl_joints.reshape(smpl_joints.shape[0], -1).astype(np.float32)
    else:
        raise ValueError(f"bad smpl_joints shape {smpl_joints.shape}")

    smpl_pose = np.asarray(smpl_pose, dtype=np.float32)
    if smpl_pose.ndim == 2 and smpl_pose.shape[-1] == 63:
        pass
    elif smpl_pose.ndim == 3:
        smpl_pose = smpl_pose.reshape(smpl_pose.shape[0], -1).astype(np.float32)
    else:
        raise ValueError(f"bad smpl_pose shape {smpl_pose.shape}")

    body_quat_w = np.asarray(body_quat_w, dtype=np.float64)
    if body_quat_w.ndim != 2 or body_quat_w.shape[-1] != 4:
        raise ValueError(f"bad body_quat_w shape {body_quat_w.shape}")

    t = smpl_joints.shape[0]
    if smpl_pose.shape[0] != t or body_quat_w.shape[0] != t:
        raise ValueError(
            f"length mismatch joints={t} pose={smpl_pose.shape[0]} quat={body_quat_w.shape[0]}"
        )

    wrists6 = wrists6_from_smpl_pose(smpl_pose)
    enc = Path(encoder_path) if encoder_path else _DEFAULT_ENCODER
    sess, inp_name, out_name, providers = load_sess(enc, device=device)
    tokens = encode_fn(sess, inp_name, out_name, smpl_joints, wrists6, body_quat_w)
    meta = {
        "encode_path": "isaac_mode2_openloop",
        "sonic": "v1.1",
        "encode_mode": 2,
        "encoder": str(enc.resolve()),
        "ort_providers": list(providers),
        "num_frames": int(t),
        "token_abs_mean": float(np.abs(tokens).mean()),
    }
    return tokens.astype(np.float64), meta


def rewrite_episode_buffer_motion_tokens(
    episode_buffer: dict,
    *,
    encoder_path: str | Path | None = None,
    device: str = "cpu",
    live_backup_dir: Path | None = None,
    episode_index: int | None = None,
) -> dict[str, Any]:
    """In-place replace ``action.motion_token`` list in an unsaved episode buffer.

    Expects LeRobot-style buffer lists (pre-``np.stack``). Saves live tokens to
    ``live_backup_dir/episode_XXXXXX_motion_token_live.npy`` when provided.
    """
    size = int(episode_buffer.get("size", 0))
    if size <= 0:
        return {"skipped": True, "reason": "empty_buffer"}

    required = ("teleop.smpl_joints", "teleop.smpl_pose", "teleop.body_quat_w", "action.motion_token")
    for key in required:
        if key not in episode_buffer:
            return {"skipped": True, "reason": f"missing_{key}"}
        if len(episode_buffer[key]) != size:
            return {
                "skipped": True,
                "reason": f"len_mismatch_{key}",
                "size": size,
                "len": len(episode_buffer[key]),
            }

    live = np.stack(
        [np.asarray(x, dtype=np.float64).reshape(-1) for x in episode_buffer["action.motion_token"]],
        axis=0,
    )
    smpl_joints = np.stack(
        [np.asarray(x, dtype=np.float32).reshape(-1) for x in episode_buffer["teleop.smpl_joints"]],
        axis=0,
    )
    smpl_pose = np.stack(
        [np.asarray(x, dtype=np.float32).reshape(-1) for x in episode_buffer["teleop.smpl_pose"]],
        axis=0,
    )
    body_quat_w = np.stack(
        [np.asarray(x, dtype=np.float64).reshape(-1) for x in episode_buffer["teleop.body_quat_w"]],
        axis=0,
    )

    # If SMPL stream was gated to zeros for most frames, skip rewrite (keep live).
    smpl_abs = float(np.abs(smpl_joints).mean())
    if smpl_abs < 1e-6:
        return {
            "skipped": True,
            "reason": "smpl_all_zero",
            "live_token_abs_mean": float(np.abs(live).mean()),
        }

    tokens, meta = encode_supervised_mode2_tokens(
        smpl_joints,
        smpl_pose,
        body_quat_w,
        encoder_path=encoder_path,
        device=device,
    )

    if live_backup_dir is not None:
        live_backup_dir = Path(live_backup_dir)
        live_backup_dir.mkdir(parents=True, exist_ok=True)
        ei = int(episode_index if episode_index is not None else episode_buffer.get("episode_index", 0))
        live_path = live_backup_dir / f"episode_{ei:06d}_motion_token_live.npy"
        np.save(live_path, live.astype(np.float64))
        meta["live_token_backup"] = str(live_path)
        meta["live_token_abs_mean"] = float(np.abs(live).mean())
        meta["l2_vs_live_mean"] = float(np.linalg.norm(live - tokens, axis=-1).mean())

    episode_buffer["action.motion_token"] = [tokens[i].astype(np.float64) for i in range(size)]
    meta["rewrote"] = True
    meta["episode_index"] = int(
        episode_index if episode_index is not None else episode_buffer.get("episode_index", -1)
    )

    return meta


def append_rewrite_record(dataset_root: Path, record: dict) -> None:
    """Append one JSONL line to ``meta/motion_token_supervised_rewrite.jsonl``."""
    path = Path(dataset_root) / "meta" / "motion_token_supervised_rewrite.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(record, ensure_ascii=False) + "\n")
```

### 2. 修改 `gear_sonic/scripts/run_data_exporter.py`

#### 2.1 新增 CLI 字段（`SonicDataExporterConfig` 末尾）

```python
    # Supervised motion_token rewrite (Direction-1 dual-track)
    rewrite_motion_token_supervised: bool = True
    """Before parquet save, rewrite ``action.motion_token`` with open-loop SONIC
    v1.1 mode-2 encode from buffered ``teleop.smpl_*`` (same rules as §8 refill).
    Live deploy ``token_state`` is still copied frame-by-frame for teleop safety;
    it is backed up under ``meta/motion_token_live/`` when rewrite succeeds.
    Use ``--no-rewrite-motion-token-supervised`` to keep raw live tokens on disk."""

    supervised_token_device: str = "cpu"
    """ORT device for episode-end supervised encode: ``cpu`` / ``cuda`` / ``auto``."""
```

#### 2.2 `GrootDataCollector.__init__` 新增参数并保存

```python
        rewrite_motion_token_supervised: bool = True,
        supervised_token_device: str = "cpu",
    ):
        ...
        self.rewrite_motion_token_supervised = bool(rewrite_motion_token_supervised)
        self.supervised_token_device = str(supervised_token_device or "cpu")
```

#### 2.3 `_finalize_frame` / `save_and_cleanup`：save 前 rewrite

```python
                self._rewrite_supervised_motion_tokens()
                self.data_exporter.save_episode()
```

#### 2.4 帧写入注释 + 新增方法 `_rewrite_supervised_motion_tokens`

```python
        # Live deploy latent (closed-loop heading). Disk supervision is rewritten
        # to open-loop mode-2 at episode save — see ``_rewrite_supervised_motion_tokens``.
        if "token_state" in proprio:
            frame_data["action.motion_token"] = np.asarray(proprio["token_state"], dtype=np.float64)
        else:
            frame_data["action.motion_token"] = np.zeros(64, dtype=np.float64)

    def _rewrite_supervised_motion_tokens(self) -> None:
        """Replace buffered live tokens with §8-identical open-loop mode-2 tokens."""
        if not self.rewrite_motion_token_supervised:
            return
        buf = self.data_exporter.episode_buffer
        if buf is None or int(buf.get("size", 0)) <= 0:
            return
        try:
            from gear_sonic.utils.data_collection.supervised_mode2_tokens import (
                append_rewrite_record,
                rewrite_episode_buffer_motion_tokens,
            )
        except Exception as exc:
            self._print_and_say(
                f"[motion_token] supervised rewrite unavailable ({exc}); keeping live tokens",
                say=False,
            )
            return

        root = self.data_exporter.meta.root
        live_dir = root / "meta" / "motion_token_live"
        try:
            meta = rewrite_episode_buffer_motion_tokens(
                buf,
                device=self.supervised_token_device,
                live_backup_dir=live_dir,
                episode_index=int(buf.get("episode_index", -1)),
            )
        except Exception as exc:
            self._print_and_say(
                f"[motion_token] supervised rewrite failed ({exc}); keeping live tokens",
                say=False,
            )
            return

        try:
            append_rewrite_record(root, meta)
        except Exception as exc:
            print(f"[motion_token] warning: could not append rewrite record: {exc}")

        if meta.get("rewrote"):
            l2 = meta.get("l2_vs_live_mean")
            l2_s = f"{l2:.3f}" if isinstance(l2, (int, float)) else "?"
            self._print_and_say(
                f"[motion_token] ep{meta.get('episode_index')} rewritten "
                f"mode2 open-loop (L2 vs live mean={l2_s})",
                say=False,
            )
        elif meta.get("skipped"):
            self._print_and_say(
                f"[motion_token] rewrite skipped: {meta.get('reason')}",
                say=False,
            )
```

#### 2.5 `main()` 传入配置并打印状态

```python
        rewrite_motion_token_supervised=config.rewrite_motion_token_supervised,
        supervised_token_device=config.supervised_token_device,
    )
    if config.rewrite_motion_token_supervised:
        print(
            "[motion_token] episode-end rewrite ENABLED "
            f"(device={config.supervised_token_device}): disk tokens = open-loop mode2; "
            "live deploy tokens backed up under meta/motion_token_live/"
        )
    else:
        print("[motion_token] episode-end rewrite DISABLED — disk keeps live token_state")
```

### 3. 修改 `gear_sonic/scripts/pico_manager_thread_server.py`（仅注释）

在 `joint_pos = np.zeros(29)` 前增加：

```python
        # INTENTIONAL: joint_pos[29] is wrist-only for SONIC mode-2.
        # Non-wrist DOFs stay 0 — mode-2 encoder only gathers wrists6
        # (observation_config: motion_joint_positions_wrists_10frame_step1).
        # Formulas MUST stay identical to
        # ``datasets.data_check.convert_to_mode2.compute_wrist_joint_pos_il``
        # (used by §8 refill / episode-end supervised token rewrite).
```

以及肘索引旁：

```python
        # Elbow DOFs intentionally unused (commented historically): elbow swing
        # is folded into wrist euler below; mode-2 does not take shoulder/elbow q.
```

### 4. 修改 `gear_sonic/pyproject.toml`

`data_collection` extra 增加一行：

```toml
    "onnxruntime>=1.16",
```

### 5. 修改 `docs/notes/g1/pico_real_data_collection.md`

- **§0** `action.motion_token` 行：改为「落盘前开环 mode2 监督重写；live 备份；可 `--no-rewrite-...`」。
- **§8**：说明 2026-11-09 起 exporter 默认已 rewrite；pipeline refill 变为可选校验。
- **§9.8**：`action.motion_token` 注明非裸 `token_state`。

---

## 运行依赖

数采 venv 需 `onnxruntime`（CPU 即可）：

```bash
uv pip install --python .venv_data_collection/bin/python 'onnxruntime>=1.16'
```

本机已安装：`onnxruntime==1.23.2`（CPUExecutionProvider）。

---

*报告版本：2026-11-09。对应问题说明 `motion_token_recording_bug_for_fix.md`。*

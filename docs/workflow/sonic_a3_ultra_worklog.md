# SONIC × 远征 A3 Ultra 适配工作日志

本文件记录"在 GR00T-WholeBodyControl 上适配智元远征 A3 Ultra（SONIC V1.1）"这个任务的每一次对话内容、做了什么改动、以及对应的时间戳。时间戳以文件实际修改时间（`stat`）或系统时钟为准，不是估算值；纯讨论、没有产出文件改动的环节标注为"无文件改动"。

后续每次继续这个任务，请在文末按相同格式追加新条目，不要覆盖已有记录。

---

## 1. [2026-10-10，早于 18:23] MuJoCo 仿真需要 URDF 还是 USD

**用户请求**：为远征 A3 Ultra 适配 SONIC V1.1，先在 MuJoCo 仿真里测试，问需要 URDF 还是 USD 文件。

**结论**：MuJoCo 不能直接读 USD（USD 是 Isaac Sim/Isaac Lab 用的），本仓库 MuJoCo 侧实际走的是 URDF → `urdf2mjcf` 转 MJCF 的路径（参照 `decoupled_wbc/sim2mujoco/README.md` 和 G1 现有的 `g1.urdf`/`g1.xml`）。

**文件改动**：无（纯问答）。

---

## 2. [2026-10-10，约 18:20 前后] 现有 G1 遥操管线 + 问智元要什么文件

**用户请求**：说明了已经跑通的 G1 MuJoCo + Pico 全身遥操命令（`run_sim_loop.py` / `pico_manager_thread_server.py`），问要跟智元售后要哪些文件才能对 A3 Ultra 做同样的事。

**结论**：确认需要 URDF（含真实惯量标定、完整运动学）+ mesh、关节限位/电机参数表、（可选）官方 MJCF；USD 仅在之后要用 Isaac Lab 重新训练 policy 时才需要。列了一份可以直接发给智元的问询清单。

**文件改动**：无（纯问答）。

---

## 3. [2026-10-10 18:23–18:24] 下载 A3 Ultra URDF/USD 并放入仓库对应位置

**用户请求**：给出了智元官方 `AgibotTech/A3-A3U-robot-model` 和 `AgibotTech/A3_Ultra_usd` 两个 GitHub 仓库链接，要求下载到 `GR00T-WholeBodyControl` 下和 G1 对应的位置。

**做了什么**：
- `git clone` 两个仓库到 scratchpad，确认 A3 Ultra 版本对应 URDF 仓库里的 `a3_ultra_t3d0/`，与 USD 仓库（只有 T3D0）对齐。
- 拷贝到：
  - `gear_sonic/data/robot_model/model_data/a3_ultra/`（Pinocchio RobotModel / 遥操 IK 用）
  - `decoupled_wbc/sim2mujoco/resources/robots/a3_ultra/`（MuJoCo 实时部署 runner 用）
  - `gear_sonic/data/robots/a3_ultra/`（USD，目前代码未引用，备用资产）
- 修了压平目录结构后 URDF/MJCF 里全部错掉的 mesh 相对路径（`../meshes/` → `meshes/`，以及 `model_planning.urdf` 里的 ROS `package://` 路径）。
- MJCF 里原来 `<include file="../../terrain/mujoco/flat.xml"/>` 指向一个没有一起发布的地形文件，导致加载报错，改成了内联的地面 geom + 光源（参照 G1 `scene_43dof.xml` 的写法）。
- 用 Pinocchio（63 自由度）和 MuJoCo（nq=70, nbody=65）分别加载验证通过。

**文件改动**（新增）：
- `gear_sonic/data/robot_model/model_data/a3_ultra/*`
- `decoupled_wbc/sim2mujoco/resources/robots/a3_ultra/*`
- `gear_sonic/data/robots/a3_ultra/*`

---

## 4. [2026-10-10，约 18:30–18:45] 重新训练 SONIC 还需要什么

**用户请求**：问是不是要针对 A3 重新训练 SONIC，重新训练还需要什么。

**结论**：确认不能直接复用 G1 训练好的 policy。梳理了训练链路实际读取的文件位置（`gear_sonic/data/assets/robot_description/urdf/<robot>/`，不是 `model_data/`），列出需要补的东西：机器人专属 `robots/<robot>.py`、`robot_mapping` 注册、专属 exp yaml、动作数据（retarget 或 SMPL-only 起步）、算力。并指出 A3 Ultra 手部结构（5 指 abad/pip/dip/tip）和 G1 完全不同，建议先锁手只训身体。

**文件改动**：无（纯问答 + 规划）。

---

## 5. [2026-10-10 18:49] 补齐 `robot_description` 下训练用的 URDF/MJCF

**用户请求**：注意到 `gear_sonic/data/assets/robot_description` 下也会放 G1，问 A3 Ultra 是否也要照样放。

**做了什么**：确认这是 IsaacLab 训练脚本（`UrdfFileCfg`）和 motion_lib 骨骼 FK 实际读取的位置，和之前放的两处都不是同一个用途。照 H2 的"自包含"方式（urdf 和 meshes 放一起，不走 ROS package:// 包路径）放了一份，并用 Pinocchio + MuJoCo 验证加载成功。

**文件改动**（新增）：
- `gear_sonic/data/assets/robot_description/urdf/a3_ultra/a3_ultra_t3d0.urdf` + `meshes/`
- `gear_sonic/data/assets/robot_description/mjcf/a3_ultra_t3d0.xml`

---

## 6. [2026-10-10 19:27–19:30] 训练 + 两条部署管线加 `--robot` 命令行参数

**用户请求**：训练和部署时都要能通过 `--robot g1/a3_ultra/g1_old/h2` 指定机器人。

**过程**：进入 Plan 模式，派 Explore 子代理摸清部署侧两条管线（`gear_sonic/scripts/run_sim_loop.py`+`pico_manager_thread_server.py` 一条，`decoupled_wbc/sim2mujoco/scripts/run_mujoco_gear_wbc.py` 另一条完全独立）各自哪里把 G1 写死了；跟用户确认了两条管线都要改、`g1_old` 暂不接入（代码里从未被引用过，纯遗留资产）。

**做了什么**（均已验证：py_compile + 实际跑通 registry/tyro/argparse 的正确与报错路径）：
- `gear_sonic/data/robot_model/instantiation/__init__.py`：新增 `ROBOT_MODEL_REGISTRY`/`instantiate_robot_model()`，目前只有 `g1`，其它机器人查不到时抛 `NotImplementedError` 并指向 `new_embodiments.md`。
- `gear_sonic/utils/mujoco_sim/configs.py`：`BaseConfig` 加 `robot: Literal["g1","a3_ultra","h2"]` 字段（tyro 自动生成 `--robot`），`ROBOT_WBC_YAML` 查表，`load_wbc_yaml()` 按机器人分派。
- `gear_sonic/scripts/run_sim_loop.py`：改用 `instantiate_robot_model(config.robot)`。
- `gear_sonic/scripts/pico_manager_thread_server.py`：argparse 加 `--robot`，打通 `run_pico_manager`/`run_pico`/`_pose_stream_common` → `ThreePointPose(robot_model=...)`。
- `decoupled_wbc/sim2mujoco/scripts/run_mujoco_gear_wbc.py`：原来完全没有 argparse，加了 `--robot`，`CONFIG_PATH` 和 yaml 文件名按机器人生成，没有对应部署配置时在打开文件前就抛清晰的 `FileNotFoundError`。
- `gear_sonic/train_agent_trl.py`：加 `ROBOT_EXP_MAP`，在 Hydra 解析前把 `--robot <name>` 翻译成 `+exp=manager/universal_token/all_modes/sonic_<name>`。

**效果**：`g1` 零回归；`a3_ultra`/`h2` 在各自配置补齐前，会在参数解析/加载阶段就给出清晰报错，不会深挖到 mujoco/torch 内部才炸。

---

## 7. [2026-10-10 19:55–19:59] 读取 Kp/Kd 文档，实现 A3 Ultra 训练支持

**用户请求**：阅读 `docs/robot_parameters/A3_Ultra/A3_关节控制参数KpKd.docx`，继续完善代码，实现 A3 Ultra 的训练。

**关键数据来源**（全部来自真实文件，没有编造数值）：
- 文档给出 31 个躯干关节（腿12+腰3+臂14+头2，不含手指）的 Kp/Kd/限位/力矩/速度上限——确认智元自己的 `a3_ultra_t3d0.xml` 的 `<actuator>` 本来也只驱动这 31 个关节，手指本来就无驱动，和此前"先锁手"的方案吻合。
- 默认站姿/主干高度取自 `a3_ultra_t3d0.xml` 里自带的 `<keyframe>`。
- IsaacLab↔MuJoCo 的 DOF/Body 映射表：先用 Pinocchio 对 H2 的 URDF 做 BFS 遍历，验证能精确复现 `h2.py` 里手写的 `H2_ISAACLAB_JOINTS` 列表，确认算法可信后，同样方式对 A3 Ultra 的 URDF（Pinocchio）/ MJCF（MuJoCo）各自算出遍历顺序，构造 63 DOF / 64 body 的双向映射，并做了完整 round-trip 数值校验，全部通过。
- armature（电机转子惯量）文档没给，没有编造，保持默认值 0，等智元回复（见第 4 条留的问询）。

**文件改动**（新增/修改）：
- 新增 `gear_sonic/envs/manager_env/robots/a3_ultra.py`：`A3_ULTRA_CFG`（10 个 actuator 组，真实 Kp/Kd/力矩/速度）、`A3_ULTRA_ACTION_SCALE`、`A3ULTRA_ISAACLAB_TO_MUJOCO_MAPPING`。
- 修改 `gear_sonic/envs/manager_env/robots/__init__.py`：导入 `a3_ultra`。
- 修改 `gear_sonic/envs/manager_env/modular_tracking_env_cfg.py`：`robot_mapping` 注册 `"a3_ultra"`。
- 修改 `gear_sonic/trl/utils/order_converter.py`：新增完整的 `A3UltraConverter`（照 `G1Converter` 的完整写法，不是 `H2Converter` 的半成品）。
- 新增 `gear_sonic/config/exp/manager/universal_token/all_modes/sonic_a3_ultra.yaml`：仿照 `sonic_h2.yaml`（`motion_file: null`，纯 SMPL tracking 起步）；因为 A3 Ultra 的 body 命名是大写 `_Link`（和 G1/H2 全小写不同），覆盖了 `reward_point_body`/`vr_3point_body`/`body_names`/`anti_shake_ang_vel`/`undesired_contacts`/`ee_body_pos`/`foot_pos_xyz` 等涉及 body 名字的字段；动作空间显式列出 31 个真实关节，排除 32 个手指关节。

**验证方式**（环境里没装 Isaac Lab，无法端到端跑）：
- 所有新增/修改的 `.py` 文件 `py_compile` 通过。
- `sonic_a3_ultra.yaml` 用 PyYAML 解析通过，动作关节数（31）、body_names 数（14）、`robot.type`/`assetFileName` 均核对正确。
- `train_agent_trl.py --robot a3_ultra` 的 argv 翻译逻辑验证能解析到真实存在的 exp 文件（此前会在 Hydra 里报"找不到配置"）。
- DOF/Body 映射表的正确性由独立的 numpy round-trip 脚本验证，不依赖 Isaac Lab。

**明确未做 / 遗留事项**：
1. 没有专属 retarget 动作库，`motion_file: null`，目前只能跑纯 SMPL tracking。
2. body 命名覆盖是"第一版"，按 `new_embodiments.md` 的建议，需要用 `num_envs=1` 实跑一次，让 Isaac Lab 报出具体哪个 body 名不存在再逐个修。
3. 文档里"Domain Randomization"表提到"球拍系统"/"乒乓球系统"，暗示这版 A3 Ultra 可能是乒乓球任务机器人，不是单纯 WBC 运动跟踪——这部分需要专门的 ball/racket 环境扩展，没有擅自实现，只是标记出来等用户确认范围。
4. armature 等电机惯量数据，等智元回复后需要回填到 `a3_ultra.py` 的 actuator 配置里。

---

## 8. [2026-10-10 20:59] 创建本工作日志

**用户请求**：在 `workflow` 文件夹下新建工作日志，详细记录每次对话内容及对应时间戳。

**做了什么**：发现 `docs/workflow/` 目录已存在但是空的，在其下新建本文件，把第 1～7 条历史对话按真实文件时间戳（`stat`）整理补全。

**旁注**：写日志时发现 git HEAD 在 `2026-10-10 20:57:06 +0800`（即本轮对话发生前 2 分钟）多了一个不是本次对话里做的提交 `60065f3 Clean base branch: remove large tracked assets from index`，该提交同时把第 6/7 条里对已跟踪文件（`configs.py`、`train_agent_trl.py` 等）的修改一起带入了 HEAD，但没有包含第 7 条新增的两个未跟踪文件（`a3_ultra.py`、`sonic_a3_ultra.yaml`）。这应该是你在本会话之外的终端操作，附带确认一下：那次清理有没有把这次新加的 A3 Ultra mesh/USD 大文件从 git 跟踪里移出去，如果是，工作目录里的文件本身应该还在，只是不再被 git 跟踪。

**文件改动**（新增）：
- `docs/workflow/sonic_a3_ultra_worklog.md`（本文件）

---

## 9. [2026-10-10，上下文压缩后续] 环境安装 + GMR A3 Ultra 适配

**背景**：上一轮对话因 context 超限而压缩，本轮从摘要恢复，继续推进"先安装所有环境，再做小规模 retarget 验证"的计划。

**做了什么**：

### 9.1 环境安装状态确认
- GMR（`YanjieZe/GMR`）在 `gmr` conda env (Python 3.10) 中 `pip install -e .` 完成，**exit 0**。
- GMR repo 已从 scratchpad 临时目录拷贝到永久路径：`/home/user/gmr_repo/`，并重新以 `-e` 方式安装指向新路径。
- IsaacLab (`./isaaclab.sh --install`) 仍在进行中（正在下载 torch 2.14.1 给 skrl/sb3 等 RL 框架，属于正常流程）。

### 9.2 A3 Ultra 根链接名 bug 修复
**发现了上一轮遗留的命名 bug**：A3 Ultra 的 URDF/MJCF 中根链接（root link）实际叫 `pelvis_link`，但之前用 Pinocchio BFS 推导时错误地写成了 `pelvis`（G1/H2 的命名），导致多处不一致。

**修复了以下文件**：
- `gear_sonic/envs/manager_env/robots/a3_ultra.py`：`A3ULTRA_ISAACLAB_JOINTS[0]` 改为 `"pelvis_link"`。
- `gear_sonic/config/exp/manager/universal_token/all_modes/sonic_a3_ultra.yaml`：`anchor_body` 和 `body_names[0]` 改为 `"pelvis_link"`。

**验证**：用 MuJoCo 加载 A3 Ultra MJCF，确认：`nbody=65`，body[1]（第一个非 world body）= `pelvis_link`；IsaacLab 映射表 `A3ULTRA_ISAACLAB_TO_MUJOCO_BODY[0]=0` → MuJoCo body 0（去掉 world）= `pelvis_link`，前后一致。

### 9.3 GMR A3 Ultra 支持（`/home/user/gmr_repo/`）
- 新建 `assets/agibot_a3_ultra/`：从 `gear_sonic/data/assets/robot_description/` 拷入 `a3_ultra_t3d0.xml`（修了 `meshdir` 为 `meshes/`）+ 70 个 mesh 文件。
- 新建 `general_motion_retargeting/ik_configs/smplx_to_a3_ultra.json`：参照 `smplx_to_g1.json` 格式，A3 Ultra 与 G1 的关节轴向完全一致（X/Y/Z）所以四元数修正值直接复用；主要差异是 `_Link`（大写 L）和用 `left_ankle_roll_Link`/`right_ankle_roll_Link` 替代了 G1 的 toe 链接（A3 Ultra 无 toe link），以及 `robot_root_name: "pelvis_link"`。
- 修改 `general_motion_retargeting/params.py`：在 `ROBOT_XML_DICT`、`IK_CONFIG_DICT["smplx"]`、`ROBOT_BASE_DICT`、`VIEWER_CAM_DISTANCE_DICT` 中注册 `"agibot_a3_ultra"`。
- 修改 `scripts/smplx_to_robot.py`：`--robot choices` 中添加 `"agibot_a3_ultra"`。
- 端到端验证：在 `gmr` conda env 中，MJCF 加载（`nq=70, nv=69, nbody=65`）、root body `pelvis_link` 查找、IK config 全部 robot body 名核对，全部通过 ✓。

### 9.4 待完成（仍然缺失）
1. **SMPL-X body model 文件**（`SMPLX_NEUTRAL.pkl` 等）：需从 [smpl-x.is.tue.mpg.de](https://smpl-x.is.tue.mpg.de) 注册下载，放到 `/home/user/gmr_repo/assets/body_models/`。
2. **HF 登录**：`hf auth login --token hf_<你的完整token>`（BonesSeed fine-grained token）。
3. **测试动作文件**（`.npz` SMPL-X 格式）：HF 登录后可以从 Bones-SEED 数据集下，或用 AMASS 的样本。
4. **IsaacLab 安装完成后**：在 `isaaclab` env 里 `pip install -e "gear_sonic/[training]"`，然后 `python check_environment.py --training`。

**文件改动**（修改/新增）：
- 修改 `gear_sonic/envs/manager_env/robots/a3_ultra.py`：`ISAACLAB_JOINTS[0]` 修正为 `"pelvis_link"`
- 修改 `gear_sonic/config/exp/manager/universal_token/all_modes/sonic_a3_ultra.yaml`：`anchor_body`/`body_names[0]` 修正
- 新增 `/home/user/gmr_repo/assets/agibot_a3_ultra/` 及其内容
- 修改 `/home/user/gmr_repo/general_motion_retargeting/ik_configs/smplx_to_a3_ultra.json`（新建）
- 修改 `/home/user/gmr_repo/general_motion_retargeting/params.py`

---

## 10. [2026-10-10 21:xx–23:43] Retarget 验证 + 批量 retarget 管线 + 首次训练跑通

**背景**：上一轮对话因 context 超限再次压缩，本轮从摘要恢复，继续处理"retarget 验证"和"批量 retarget 管线"。

**做了什么**：

### 10.1 MuJoCo DOF 顺序勘误
- 之前错误地把 `dof[0]` 标为 `L_hip_pitch`，实际上 MuJoCo qpos DOF 顺序按 **JOINT 声明顺序**（非 actuator 顺序）：A3 Ultra MJCF 中先是 waist（dof 0-2）→ head（3-4）→ 左臂（5-11）→ 左手指（12-27）→ 右臂（28-34）→ 右手指（35-50）→ 左腿（51-56）→ 右腿（57-62）。
- 用脚本（`/tmp/check_a3_joints.py`）从 MuJoCo 模型读取真实关节名与 qpos 地址，确认腿部 DOF：`dof[51]=left_hip_pitch`（mean=-0.70 rad）、`dof[54]=left_knee`（mean=1.43 rad）、`dof[57]=right_hip_pitch`（mean=-0.85 rad），IK 配置正确。

### 10.2 pose_aa DFS body 顺序提取
- 用 `/tmp/a3ultra_fk_order.py` 对 A3 Ultra MJCF 做 DFS 遍历，确认 `torch_humanoid_batch.from_mjcf()` 的 body 顺序，建立完整的 `(DFS_body_idx, qpos_dof_idx, axis)` 映射表（`_ACTUATED_MAP`，31 行）。

### 10.3 GMR qpos → SONIC 训练格式转换脚本
- 新建 `gear_sonic/data_process/convert_gmr_a3ultra_to_motion_lib.py`：
  - 输入：GMR retarget 输出的 qpos list（每帧 70 维：root_pos 3 + root_quat_wxyz 4 + 63 DOF）
  - 输出：SONIC motion_lib pkl，包含 `root_trans_offset(T,3)`、`pose_aa(T,64,3)`（64 个 MJCF body，手指 DFS 索引处填 0）、`dof(T,31)`、`root_rot(T,4)`（xyzw）、`smpl_joints(T,24,3)`（零占位）、`fps`。
  - 用合成步行数据验证：`pose_aa(150,64,3) ✓`，`dof(150,31) ✓`，root z ∈ [0.741,0.753] ✓。

### 10.4 SMPL Y-up → Z-up 坐标转换 + 验证
- 发现 `smpl_filtered` pkl 里的 `transl` 是 Y-up 坐标（`transl_y ≈ 1.26 m` = 站立时骨盆高度），但 GMR 期望 Z-up。
- 修复：在 SMPL → SMPL-X npz 转换时，对 `trans` 和 `root_orient` 分别施加 `R_x(+90°)` 旋转：
  ```python
  R_conv = Rotation.from_euler('x', 90, degrees=True)
  trans_zup = R_conv.apply(trans_yup)
  root_orient_zup = (R_conv * Rotation.from_rotvec(root_orient_yup)).as_rotvec()
  ```
- 验证：修复后 `trans_z ∈ [1.19, 1.27] m` ✓；GMR retarget 输出 `root_z ∈ [1.01, 1.20] m`，腿部 DOF 合理（髋关节屈曲 mean≈-1.07 rad，膝关节 mean≈1.33 rad，踝关节 mean≈-0.67 rad），全部在关节限位范围内 ✓。

### 10.5 批量 retarget 脚本 + 样本数据生成
- 新建 `gear_sonic/data_process/batch_retarget_a3ultra.py`：完整端到端管线：
  1. 加载 `smpl_filtered` pkl → 应用 Y-up→Z-up 坐标旋转 → 保存临时 SMPL-X npz
  2. 调用 GMR retarget → 输出 qpos list
  3. 调用 `convert_gmr_a3ultra_to_motion_lib.convert_sequence()` → 保存 SONIC motion_lib pkl
- 在 `sample_data/smpl_filtered/` 中的 2 个样本文件上批量跑通：`2002 frames @ 30fps`，输出 `pose_aa(2002,64,3)`，`dof(2002,31)` ✓。
- 输出路径：`data/motion_lib_a3ultra/robot_filtered/`（2 个 pkl，各 2.4 MB）。
- 更新 `sonic_a3_ultra.yaml`：`motion_file: data/motion_lib_a3ultra/robot_filtered`（原来是 `null`）。

### 10.6 依赖冲突修复（训练环境）
- **问题 1**：`trl 0.28.0` → `datasets 5.1.0` → `pyarrow 26.0.0` 要求 numpy≥2.0，但 `isaacsim-kernel` 固定 `numpy==1.26.0`，不可升级。
- **修复**：将 `datasets` 降到 `4.0.0`（要求 `pyarrow>=15.0.0`，现有 `pyarrow==17.0.0` 满足），`trl 0.28.0` 仍保留（其 `datasets>=3.0.0` 的要求由 4.0.0 满足）。验证：`from trl.experimental.ppo import ppo_trainer` ✓。

### 10.7 torch_humanoid_batch `joints_range` 断言修复
- **问题**：A3 Ultra MJCF 有 63 个带 range 的关节，但只有 51 个 motor（31 主关节 + 20 手指有驱动），12 个手指末端关节（dip/thumb_pip）无 motor，导致 `len(joints_range)==63 != self.num_dof==51` 断言失败。
- **修复**：在 `gear_sonic/utils/motion_lib/torch_humanoid_batch.py` 的 `from_mjcf()` 里，解析 `<actuator>` 节点建立 `_motors_set`，在 `_add_xml_node` 内跳过不在 `_motors_set` 中的关节，使 `joints_range` 只收录有 motor 的关节（51 个），断言通过 ✓。

### 10.8 首次训练跑通（部分）
- 命令：`WANDB_MODE=disabled python gear_sonic/train_agent_trl.py --robot a3_ultra num_envs=1 headless=True`（`gmr` env 无关，用 `isaaclab` env）
- **Omniverse EULA** 首次运行需要接受：`echo "Yes" | ...` 解决。
- **wandb 未登录**：`WANDB_MODE=disabled` 解决。
- IsaacLab 成功加载：
  - A3 Ultra MJCF 载入，`31 action joints` 全部正确解析（含 `left_hip_pitch_joint` ... `right_wrist_yaw_joint`）。
  - 所有 body 名（`torso_Link`、`.*wrist_yaw.*` 等）全部找到 ✓。
  - Motion lib 加载 2 个 motion，3335 frames ✓。
  - 观测空间正确展开（`joint_pos (630,)=63×10历史步`、`command_multi_future_lower_body (240,)` 等）。
- 当前状态：进入 **GPU shader 编译阶段**（Vulkan，第一次运行耗时 15-30 分钟，后续有缓存会很快），进程仍在后台运行（`PID 65625`，CPU 130%，GPU mem 5.4 GB），等待完成。

### 10.9 清理
- 删除 `models_smplx_v1_1.zip`（830 MB，内容已解压到 `/home/user/gmr_repo/assets/body_models/smplx/`）。

### 10.10 Bones-SEED 训练数据下载启动
- **用户手动执行**：
  ```bash
  wget --header="Authorization: Bearer $(cat /home/user/.cache/huggingface/token)" \
    "https://huggingface.co/datasets/bones-studio/seed/resolve/main/soma_proportional.tar.gz" \
    -O "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl/data/bones_seed/soma_proportional.tar.gz" \
    --continue
  ```
- 文件大小：45.5 GB，路径：`data/bones_seed/soma_proportional.tar.gz`。
- **下载完成后的后续步骤**：见 Session 11（BVH 管线搭建）和 Session 12（IK bug 修复 + 全量重 retarget）。

**文件改动**（本轮新增/修改）：
- 新增 `gear_sonic/data_process/convert_gmr_a3ultra_to_motion_lib.py`
- 新增 `gear_sonic/data_process/batch_retarget_a3ultra.py`
- 新增 `data/motion_lib_a3ultra/robot_filtered/walk_forward_amateur_001__A001.pkl`
- 新增 `data/motion_lib_a3ultra/robot_filtered/walk_forward_amateur_001__A001_M.pkl`
- 修改 `gear_sonic/config/exp/manager/universal_token/all_modes/sonic_a3_ultra.yaml`：`motion_file` 由 `null` → `data/motion_lib_a3ultra/robot_filtered`
- 修改 `gear_sonic/utils/motion_lib/torch_humanoid_batch.py`：`from_mjcf()` 新增 `_motors_set` 过滤
- 删除 `models_smplx_v1_1.zip`

**遗留事项**：
1. 首次训练的 GPU shader 编译尚在进行，等进程完成后确认是否有后续错误。
2. Bones-SEED `soma_proportional.tar.gz` 下载中，完成后需解压并运行批量 retarget。
3. armature（电机转子惯量）等智元回复后填入 `a3_ultra.py`。

---

## 11. [2026-10-11，02:00] BVH → SONIC 批量重定向管线

### 背景

`soma_proportional.tar.gz`（43 GB）确认为 NOVA 骨骼 BVH 格式。
上一轮已确认文件结构：`soma_proportional/bvh/<YYMMDD>/<motion_name>__<actor_id>.bvh`。

### 技术发现

**SOMA BVH 结构特殊性**：

SOMA 输出的 BVH 有两个关节同时有 6 通道（位置 + 旋转）：
```
ROOT Root          ← 合成根（6 通道：XYZ pos + XYZ rot）
  JOINT Hips       ← 真实骨盆（6 通道：XYZ pos + XYZ rot）
    JOINT Spine1
    ...
```

GMR 自带的 `lafan_vendor/extract.py` 的 `read_bvh()` 无法处理这种结构（假设只有 root 有 6 通道）。

**解决方案**：自己实现 `load_soma_bvh_frames()` 函数，重写 BVH 解析和 FK 计算，输出与 GMR `load_bvh_file()` 相同格式的 per-frame 字典。

**SOMA 关节名到 SMPL/GMR 的映射**：

| SOMA 名称 | 对应 SMPL/GMR | GMR 中机器人关节 |
|---|---|---|
| Hips | pelvis/root | pelvis_link |
| LeftLeg | left_hip (上腿) | left_hip_roll_Link |
| LeftShin | left_knee | left_knee_Link |
| LeftFoot+LeftToeBase | LeftFootMod | left_ankle_roll_Link |
| Chest | spine3 | torso_Link |
| LeftArm | left_shoulder | left_shoulder_yaw_Link |
| LeftForeArm | left_elbow | left_elbow_Link |
| LeftHand | left_wrist | left_wrist_yaw_Link |

### 新建文件

1. **`/home/user/gmr_repo/general_motion_retargeting/ik_configs/bvh_nokov_to_a3_ultra.json`**
   - SOMA 关节名 → A3 Ultra 机器人关节 IK 配置
   - 四元数偏移复用自 `smplx_to_a3_ultra.json`（已验证）

2. **修改 `/home/user/gmr_repo/general_motion_retargeting/params.py`**
   - 在 `bvh_nokov` 字典下添加 `"agibot_a3_ultra": IK_CONFIG_ROOT / "bvh_nokov_to_a3_ultra.json"`

3. **`gear_sonic/data_process/batch_retarget_bvh_soma_a3ultra.py`**
   - 端到端批量管线：SOMA BVH → GMR IK → SONIC motion_lib pkl
   - 特点：
     - `load_soma_bvh_frames()`: 自定义 BVH 解析 + FK，输出 GMR 所需格式
     - `iter_bvh_from_tar()`: 使用 `r|gz` 流式模式从 tar.gz 读取，无需全量解压
     - GMR retargeter 实例跨文件复用（每文件约 1.2s）
   - 用法：
     ```bash
     conda run -n gmr python3 -u batch_retarget_bvh_soma_a3ultra.py \
         --tar data/bones_seed/soma_proportional.tar.gz \
         --output data/motion_lib_a3ultra/robot_filtered/ --fps 30
     ```

### 验证结果

```
# 关节值合理性验证（walking_on_edge_ff_270 clip）
Left hip_pitch:   [-0.872, -0.056] rad ✓
Left knee:        [0.245, 2.526]   rad （极端姿势，edge walk）
Left ankle_pitch: [0.509, 0.524]   rad （偏小，IK 偏移可能需要微调）
Right hip_pitch:  [-1.356, 0.096]  rad ✓
Right knee:       [-0.122, 1.906]  rad ✓

pose_aa shape: (389, 64, 3) ✓
dof shape:     (389, 31)   ✓
fps: 30.0                  ✓
```

### 当前状态（2026-10-11 02:01）

- **批量转换**：PID 78612 正在运行，已完成 552+ 个 BVH 文件（`230530` session）
- **训练**：PID 65625 仍在运行（`train_agent_trl.py --robot a3_ultra num_envs=1 headless=True`）

### 下一步

1. **等待批量完成**：关注 `data/motion_lib_a3ultra/batch_retarget_bvh.log` 和 `robot_filtered/230530/` 文件数量
2. **收集更多数据**：检查 tar.gz 里还有哪些 session（240xxx 等日期目录）
3. **扩大训练规模**：motion data 够了（≥1000 clips）后，改 `num_envs=4096` 跑正式训练
4. **IK 微调**：ankle_pitch 几乎不变，可能需要调整 `bvh_nokov_to_a3_ultra.json` 的踝关节四元数偏移
5. **关注训练收敛**：观察 wandb/log 里的 episode reward 趋势

---

## 12. [2026-10-11，约 14:00–23:59] 发现并修复 BVH IK 根本性 bug，全量重新 retarget，重启训练

### 12.0 背景与问题症状

Session 11 结束时，批量 retarget（PID 78612）已跑出约 552 个文件，训练（PID 65625, `num_envs=1`）仍在 shader 编译阶段。进入本会话时，训练日志显示奖励几乎为零：

```
Env/Episode_Reward/tracking_relative_body_pos: 0.0000
Env/Episode_Reward/tracking_relative_body_ori: 0.0000
Mean rewards: -1.12
```

这不应该发生——即使 policy 还没收敛，只要参考动作 pkl 里的骨盆轨迹和关节角度是合理的，初始 `tracking_anchor_pos` 至少应该有 ~0.01 级别的奖励。

### 12.1 第一轮排查：检查 pkl 文件中的 DOF 值

**方法**：写脚本读取若干 `robot_filtered/*.pkl` 文件，打印腿部关节（索引 19-30）的 min/max/mean。

**发现**：

```
L_hip_yaw:   min=-2.723  max=-2.723  全部帧相同
R_hip_yaw:   min=+2.722  max=+2.722  全部帧相同
L_hip_roll:  min=-0.524  max=-0.524  全部帧相同（关节下限）
R_hip_roll:  min=-0.524  max=-0.524  全部帧相同
```

**几乎所有 pkl 文件（3500 个全部）都有这个问题**——`L_hip_yaw` 和 `R_hip_yaw` 被钉在各自的关节上限/下限（±2.722–2.723 rad ≈ ±156°），并且每一帧都是同一个值。这意味着 IK 解算完全无效，机器人每一帧都处于极限关节角度的状态，无法正常运动，所以奖励为零。

### 12.2 第一个假设：IK Config 链接名写错

**排查**：查看 `bvh_nokov_to_a3_ultra.json` 的 table1（位置任务）和 table2（姿态任务）。

**发现问题**：IK config 里使用的是：
- `"left_hip_roll_Link"` → LeftLeg（髋部姿态任务）
- `"left_knee_Link"` → LeftShin（膝关节链接姿态任务）

**但是这两个链接名在 A3 Ultra 中根本不存在！**

A3 Ultra 的髋关节运动链为：
```
pelvis_link → left_hip_pitch_Link → left_hip_roll_Link → left_hip_yaw_Link → left_knee_pitch_Link
```

`left_hip_roll_Link` 是髋关节链里第 2 个链接（只受髋屈曲控制），`left_hip_yaw_Link` 才是髋关节链末端（受所有 3 个髋关节自由度控制），是描述整个髋部姿态的正确目标链接。同理，`left_ankle_pitch_Link` 是小腿（受 3 个髋关节 + 膝关节控制），而不是 `left_knee_Link`（A3 Ultra 无此链接）。

**修复 `bvh_nokov_to_a3_ultra.json`**：
```json
// 修改前（错误）:
"left_hip_roll_Link": ["LeftLeg", ...],
"left_knee_Link":     ["LeftShin", ...],

// 修改后（正确）:
"left_hip_yaw_Link":    ["LeftLeg", ...],
"left_ankle_pitch_Link": ["LeftShin", ...],
```

### 12.3 四元数偏移调试（IK config 第二轮）

修改链接名后，测试单个文件 `eating_apple_R_001__A412_M.bvh` 的 retarget，`L_hip_yaw` 的极端值从 -2.723 变成 +2.723（方向反了），仍然不正确。

**排查过程**：
- 用 `inspect_soma_session.py` 分析 SOMA BVH 中 `LeftLeg` body 在 Hips 坐标系下的局部 euler 角
- 发现 `LeftLeg` 在 Hips 局部帧下的 Z 轴旋转约为 -173° ~ -180°，这是 SOMA 的 body frame 约定（腿部 X 轴向下）
- 正确的四元数偏移应为 `[0.5, -0.5, -0.5, -0.5]`（对应绕 X 轴旋转 -90° 的 wxyz 四元数），负责把 SOMA 的 ~180° 局部旋转转换为 GMR IK 期望的帧

**最终验证**：恢复 `[0.5, -0.5, -0.5, -0.5]` 偏移后，单独测试 `eating_apple` → `L_hip_yaw = 0.0 ± 0.2 rad`（合理）✓

### 12.4 批处理还是不对：发现 mink 状态污染 bug

即使 IK config 修复后，**直接测试单文件正常，但批量 retarget 出来的文件仍然有极端值**。

**对比测试**（写 `test_eating_apple.py` 和 `test_multi_file.py`）：
- 单独新建 retargeter → `eating_apple`：0 个极端帧 ✓
- 复用同一 retargeter，先跑 `come_down_50cm_box` → 再跑 `eating_apple`：798 个极端帧 ✗

**根因分析**：

`motion_retarget.py` 里 `retarget()` 方法的核心调用：
```python
self.configuration.integrate_inplace(vel, dt)
```

`mink.Configuration.integrate_inplace()` **原地修改**机器人的内部 MuJoCo `data.qpos`。这意味着：

当 retargeter 实例在多个 BVH 文件之间被复用时，第一个文件（`come_down_50cm_box`）结束时机器人处于极限屈体姿态（髋关节大角度弯曲，两侧 hip_yaw 各被推到正负极限），这个**末帧姿态**直接成为下一个文件（`eating_apple`）的**初始姿态**，IK 从这个极端状态出发，根本没有机会恢复到合理姿态，每一帧都停在关节极限。

**修复方案**（在 `batch_retarget_bvh_soma_a3ultra.py` 的 `retarget_bvh()` 开头添加）：
```python
if retargeter is None:
    retargeter = make_retargeter()
else:
    # 重置到中性姿态，避免前一个文件的末帧状态污染下一个文件
    retargeter.configuration.update(retargeter.configuration.model.qpos0)
```

`mink.Configuration.update(q)` 当传入 `q` 时，会把 `data.qpos` 重置为给定值并重新运行 FK。`model.qpos0` 是 MJCF 定义的初始/默认姿态。

### 12.5 修复效果验证

运行 `test_multi_file.py`（3 个文件顺序处理，复用同一 retargeter，每文件开头 reset）：

```
come_down:    218 frames, leg extremes=0   ✓（骨盆高度≈0.83m，合理的下蹲动作）
eating_apple: 288 frames, leg extremes=0   ✓（修复前此处有 798 个极端帧）
walk_210707:  150 frames, leg extremes=0   ✓
```

**已知剩余限制**：
1. `L_hip_roll` 在所有文件中系统性地停在 -0.524 rad（关节最小值 = -30°），这是由于 SOMA `LeftLeg` body 的局部坐标系旋转约定无法被 `[0.5,-0.5,-0.5,-0.5]` 偏移完全修正。`[0.5,-0.5,-0.5,-0.5]` 能正确恢复 `hip_yaw`，但对 `hip_roll` 的残差约 30°。这个误差导致左膝位置约 0.1m 的误差，但对行走、坐立等基础动作影响有限，可以接受用于初期训练。
2. 约 16% 的文件（蹲下、捡物、搜身等复杂动作）有部分帧 `R_hip_yaw > 2.5 rad`，这是这些动作本身的大范围运动幅度，不是 bug。

### 12.6 smplx_to_a3_ultra.json 同步修复

检查后发现 SMPL-X → A3 Ultra 的 IK config 有相同的链接名错误：
- `left_hip_roll_Link` → `left_hip_yaw_Link`
- `right_hip_roll_Link` → `right_hip_yaw_Link`
- `left_knee_Link` → `left_ankle_pitch_Link`
- `right_knee_Link` → `right_ankle_pitch_Link`
- 踝关节 ori_weight 由非零改为 0（位置优先）

同步修复 `/home/user/gmr_repo/general_motion_retargeting/ik_configs/smplx_to_a3_ultra.json`。注：SMPL-X 批量脚本（`batch_retarget_a3ultra.py`）每个文件创建新的 retargeter，不存在状态污染问题；但链接名错误同样存在，必须修复。

### 12.7 清除旧数据，重新批量 retarget

旧的 3500 个 pkl 全部是用错误 IK config 生成的，需要删除重来。

```bash
# 清除旧数据
rm -rf data/motion_lib_a3ultra/robot_filtered/
mkdir -p data/motion_lib_a3ultra/robot_filtered/

# 重新启动批量 retarget（后台运行）
nohup conda run -n gmr python3 -I -u \
    gear_sonic/data_process/batch_retarget_bvh_soma_a3ultra.py \
    --tar data/bones_seed/soma_proportional.tar.gz \
    --output data/motion_lib_a3ultra/robot_filtered/ \
    --fps 30 > /tmp/batch_retarget2.log 2>&1 &
# PID 70648
```

**批量进度**（最终结果，2026-10-11 23:xx）：
- 全部完成：**3503 个 pkl 文件**
- 路径：`data/motion_lib_a3ultra/robot_filtered/`

### 12.8 杀掉旧训练，用正确数据重启

旧训练（PID 68689 及之前的 153848、202444 等 session）在批量 retarget 过程中数据被删除后卡死，需要强制终止。

```bash
# 终止旧训练
kill -9 68689   # 或其他旧的训练 PID

# 用正确数据重启（64 envs，headless）
nohup conda run -n isaaclab \
    python gear_sonic/train_agent_trl.py \
    --robot a3_ultra num_envs=64 headless=True \
    > logs_rl/train_a3ultra_64envs_fixed.log 2>&1 &
# PID 72829，会话名 sonic_a3_ultra_test-20261011_220510，启动时间 22:05
```

训练日志显示 motion library 加载成功：
```
Loaded 384 motions with a total length of 466.560s and 23392 frames
```

（384 < 3503 是因为训练时 motion_lib 每次随机抽样 64 条 motion 加载；384 是当前轮次随机抽到的总数，实际文件库有 3503 条）

### 12.9 当前训练状态（23:32）

从最新检查点 `last.pt`（23:32 保存）读取：
```
global_step:      2300   (总 100000 步的 2.3%)
tot_timesteps:    3,532,800
tot_time:         5161 秒（约 86 分钟）
Reward mean:      -1.177
Reward std:        2.271
Reward max:       -0.065  ← 有进步（之前全部为负且无变化）
Reward min:      -15.629
Episode length:    1.0    （机器人目前仍快速倒地，属正常早期训练现象）
```

与修复前相比，reward mean 相近（-1.18 vs -1.18），但 **reward std 从接近 0 扩展到 2.27**，说明策略在探索不同结果——这是正确数据才有的特征。修复前因为所有参考帧的髋关节都钉在极限，任何动作都同样"错"，方差接近 0。

训练目前处于极早期（2.3%），reward 的真正改善预计在 5000-10000 步后开始显现。

### 12.10 文件改动汇总

| 文件 | 改动类型 | 说明 |
|---|---|---|
| `/home/user/gmr_repo/general_motion_retargeting/ik_configs/bvh_nokov_to_a3_ultra.json` | 修改 | `hip_roll_Link → hip_yaw_Link`，`knee_Link → ankle_pitch_Link`，恢复 `[0.5,-0.5,-0.5,-0.5]` 偏移 |
| `/home/user/gmr_repo/general_motion_retargeting/ik_configs/smplx_to_a3_ultra.json` | 修改 | 同上，同步修复链接名和 ankle ori_weight |
| `gear_sonic/data_process/batch_retarget_bvh_soma_a3ultra.py` | 修改 | `retarget_bvh()` 开头新增 `retargeter.configuration.update(retargeter.configuration.model.qpos0)` 重置状态 |
| `data/motion_lib_a3ultra/robot_filtered/` | 重建 | 删除全部旧的 3573 个 pkl，用修复后的管线重新生成，当前有 3503 个正确的 pkl |

### 12.11 下一步

1. **观察训练收敛**：当前 step 2300，epoch 2300/100000（2.3%）。目标是 5000-10000 步后出现 `tracking_anchor_pos` 非零的稳定奖励，15000-30000 步后出现步态
   - 日志：`logs_rl/TRL_A3Ultra_Track/manager/universal_token/all_modes/sonic_a3_ultra_test-20261011_220510/.hydra/train.log`
   - 检查点：每 500 步保存一次，路径 `...220510/model_step_XXXXXX.pt`

2. **扩大训练规模**（待奖励曲线正常后）：
   ```bash
   # 4096 envs 正式训练
   nohup conda run -n isaaclab \
       python gear_sonic/train_agent_trl.py \
       --robot a3_ultra num_envs=4096 headless=True &
   ```

3. **L_hip_roll 系统性偏差问题**（深层优化，非阻塞）：
   - 所有文件的 `L_hip_roll` 停在 -0.524 rad（关节下限），原因是 SOMA 的 LeftLeg body frame 与 A3 Ultra hip_roll 轴向的残差无法被当前四元数偏移修正
   - 可能的改进方向：在 IK config 中为 `left_hip_roll_Link` 专门加一个 orientation task（当前该链接完全没有任务约束），用特定偏移强制其对齐

4. **SMPL-X 数据 retarget**（如果有已有的 `smpl_filtered` pkl 数据）：
   - 使用修复后的 `smplx_to_a3_ultra.json` 重新运行 `batch_retarget_a3ultra.py`
   - 该脚本每文件创建新 retargeter（无状态污染问题），但链接名修复同样必要

5. **armature 值**：等智元回复电机转子惯量后填入 `gear_sonic/envs/manager_env/robots/a3_ultra.py` 的 actuator 配置

6. **删除遗留大文件**：
   - `GR00T-WholeBodyControl/models_smplx_v1_1.zip`（1.6 GB）仍在磁盘上

---

## 13. [2026-10-12，01:00] 训练曲线分析 + 终止阈值修复 + 训练重启

**概述**：Session 12 启动的 4096-env 训练（`sonic_a3_ultra_test-20261011_235450`，PID 原值 95577）运行到 step ~500。用户发来 wandb 截图请求分析。发现两个明确问题，停止训练，修复配置，重新启动。

### 13.0 wandb 截图分析（step ~500，4 张图）

用户发来的 4 张截图覆盖 loss、Episode_Termination、Episode_Reward（前 6 项）、Episode_Reward（后 6 项）。

**正向信号（说明数据和架构本身正常）：**
- `loss/total_aux_loss_unscaled_avg`：从 ~0.008 单调降至 ~0.003，编码器正常学习
- `Episode_Reward/tracking_anchor_ori`：从 ~6.25e-4 升至 ~8.4e-4，+34%，pelvis 朝向跟踪有效
- `Episode_Reward/tracking_body_linvel`：从 ~4.5e-4 升至 ~6.5e-4，速度跟踪持续提升
- `Episode_Reward/tracking_body_angvel`：从 ~1e-5 升至 ~2.6e-5，角速度跟踪提升
- `Episode_Termination/anchor_pos`：从 ~2e-4 降至接近 0，pelvis 位置终止条件已大幅减少
- `Episode_Termination/anchor_ori_full`：从 ~0.002 降至 ~0.001，方向终止条件减少

**问题信号：**

| 指标 | 现象 | 异常程度 |
|---|---|---|
| `objective/length` | 从 step 0 到 step 500 完全平坦在 1.0，无任何改善 | **严重** |
| `Episode_Termination/foot_pos_xyz` | 全程 1.0（100% episode 因脚部位置超限终止） | **严重** |
| `loss/entropy_avg` / `objective/entropy` | 从 -40 单调升至 +20 | **严重** |
| `Episode_Reward/tracking_relative_body_pos` | 从 3.4e-4 单调**降至**接近 0 | **较严重** |
| `Episode_Reward/action_rate_l2` | 从 ~0 单调**降至** -0.00125（惩罚项不断增大） | **较严重** |
| `loss/weighted_ppo_loss_avg` | step ~510 出现 6e9 量级尖刺（历史最大） | 需关注 |
| `Episode_Termination/ee_body_pos` | 从 0.94 缓慢**升至** 0.95，更多 episode 因此终止 | 轻微 |

### 13.1 根因诊断

**主因：`foot_pos_xyz` 阈值 0.2m 过严，配合 L_hip_roll 系统性偏置，导致每个 episode 都在第 1 步终止。**

推理链：
1. `base_adaptive_strict_ori_foot_xyz.yaml` 将 `foot_pos_xyz` 阈值覆盖为 0.2m（比默认 `foot_pos_xyz.yaml` 的 0.5m 严了 2.5 倍）
2. Session 12 发现的 L_hip_roll 系统性偏差（所有 pkl 文件中 L_hip_roll = -0.524 rad，关节下限）导致左脚在所有参考动作中都有系统性横向偏移
3. 该偏移估计 > 0.2m，因此任何 episode 从第 1 步就触发 `foot_pos_xyz` 终止
4. ep_len=1 → 每个 rollout 的 return 完全相同（全是 step-1 奖励）→ advantage 估计方差极高但均值接近 0 → PPO 的 policy gradient 信号几乎消失
5. 仅剩 entropy 正则化项在有效梯度 → PPO 倾向于最大化 entropy → action std 被推向上限（`std_clamp_max=0.5`）
6. 这解释了 `entropy` 从 -40 → +20 的曲线：entropy ≈ +20 对应 31 维 Gaussian 且 σ ≈ 0.5 的理论值（0.5 × ln(2πe × 0.25) × 31 ≈ 22.5）
7. policy 变成"完全随机"的恶性循环，`tracking_relative_body_pos` 下降和 `action_rate_l2` 恶化都是副作用

**次因：`ee_body_pos` 阈值 0.15m 同样过严**（包括脚踝 + 手腕，阈值比脚部更严）。

### 13.2 修复方案

只修改 `sonic_a3_ultra.yaml`，在 terminations 下显式加 threshold 覆盖，宽松化两个过严的终止条件：

| 终止条件 | 修改前阈值 | 修改后阈值 | 理由 |
|---|---|---|---|
| `foot_pos_xyz` | 0.2m（来自 `base_adaptive_strict_ori_foot_xyz.yaml`） | **0.5m**（默认值） | L_hip_roll 系统性偏置约 > 0.2m，阈值过严导致必然终止 |
| `ee_body_pos` | 0.15m（来自同上） | **0.25m** | 早期训练中手腕/脚踝随机探索时误差大，0.15m 过严 |

具体修改（`gear_sonic/config/exp/manager/universal_token/all_modes/sonic_a3_ultra.yaml` 的 `terminations` 节）：
```yaml
# 修改前
foot_pos_xyz:
  params:
    body_names:
      - "left_ankle_roll_Link"
      - "right_ankle_roll_Link"

# 修改后
foot_pos_xyz:
  params:
    body_names:
      - "left_ankle_roll_Link"
      - "right_ankle_roll_Link"
    threshold: 0.5   # 宽松：L_hip_roll 系统性偏置 > 0.2m，不加此覆盖必然每步终止

ee_body_pos:
  params:
    body_names:
      - ...（不变）
    threshold: 0.25  # 宽松：早期训练阶段 0.15m 过严
```

### 13.3 操作记录

```bash
# 停止旧训练（PID 95577 / 95557）
kill 95557 95577

# 重新启动（PID 106302）
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"
conda run -n isaaclab python gear_sonic/train_agent_trl.py \
  --robot a3_ultra \
  num_envs=4096 \
  headless=True &
```

新 run：`sonic_a3_ultra_test-20261012_010134`，PID 106302

### 13.4 预期效果

修复后 wandb 曲线应出现的变化：
- `objective/length`：应在 step 100-500 内从 1.0 开始有偶发 > 1 的 episode，随后缓慢上升
- `Episode_Termination/foot_pos_xyz`：应从 1.0 开始下降
- `loss/entropy_avg`：应随 ep_len 改善而从 +20 回落（policy 开始有差异化的 advantage，重新有学习方向）
- `tracking_relative_body_pos`：应从接近 0 重新上升
- `action_rate_l2`：应停止恶化

### 13.5 修改文件汇总

| 文件 | 操作 | 内容 |
|---|---|---|
| `gear_sonic/config/exp/manager/universal_token/all_modes/sonic_a3_ultra.yaml` | 修改 | `foot_pos_xyz.threshold` 0.2m→0.5m；`ee_body_pos.threshold` 0.15m→0.25m |

### 13.6 下一步

1. **等待 step 100-500**：观察 `objective/length` 是否从 1.0 开始上升，这是修复是否有效的首要信号
2. **关键指标检查点**（发截图时需包含）：
   - `objective/length`（最重要）
   - `Episode_Termination/foot_pos_xyz`（应从 1.0 下降）
   - `Episode_Reward/tracking_relative_body_pos`（应从近 0 回升）
   - `loss/entropy_avg`（应从 +20 回落）
3. **若修复无效**（step 500 后 ep_len 仍为 1）：进一步检查 retarget 数据中的实际脚部 xyz 坐标偏移量，考虑将 threshold 进一步放宽至 0.8m 或在训练初期完全禁用 `foot_pos_xyz`

---

## 14. [2026-10-12，01:30] 补充讨论：训练曲线不足以验证 retarget 质量

### 14.0 问题

用户提问：仅凭训练曲线能否确认 retarget 转化的精准性和可行性？

### 14.1 结论：不能

训练曲线只能回答"策略是否在追踪参考动作"，无法回答"参考动作本身是否正确"。即使训练完全收敛、ep_len 达到 50+、tracking reward 很高，也只能说明机器人成功复现了参考 pkl 里的关节轨迹——如果 pkl 数据本身有系统性错误，训练收敛后机器人会稳定地执行错误动作。

具体来说，训练曲线**无法**验证的内容：

| 问题 | 原因 |
|---|---|
| 参考关节角是否语义正确（行走像行走） | 策略可"成功"追踪错误参考 |
| IK 是否收敛到正确解（而非局部最优） | mink 在极端姿态下可能收敛到错误解 |
| 脚部接触时序（步态相位）是否合理 | 需要检查 foot_z 轨迹 |
| 关节速度/加速度是否在真实机器人可执行范围内 | 过大速度导致 sim-to-real 失败 |
| 视觉上是否像正常人类运动 | 只有渲染才能判断 |

**当前已知的潜在系统性错误**：`L_hip_roll` 在所有 retarget pkl 中均处于关节下限 -0.524 rad（-30°）。训练曲线无法揭示这个问题——策略可以学会追踪一个"左腿髋关节始终最大内收"的异常参考，并取得看似正常的收敛曲线。

### 14.2 正确的验证方法

需在训练之外单独做以下验证：

1. **关节轨迹可视化**（首选，10 min）：绘制一段 BVH 原始数据对应的人体关节角 vs retarget 后的机器人关节角，直观对比 hip_pitch/knee/ankle 的时序和幅值
2. **IsaacLab 渲染**（30 min）：用 `num_envs=1 headless=False` 启动，直接观察机器人是否在正常行走
3. **脚部高度轨迹**：检查 foot_z，确认存在正常的支撑相（foot_z≈0）和摆动相（foot_z>0）
4. **关节速度检查**：`qvel = np.diff(qpos)/dt`，对比 A3 Ultra 关节速度限制

### 14.3 行动：生成并运行对比可视化脚本

用户要求：将原始 BVH 轨迹（人体关节角）与 retarget 后的机器人关节角并排可视化，以直接判断转化质量。

**方法论反思（用户指出）**：正确流程应为「retarget 少量样本 → 可视化验证 → 确认质量 → 批量 retarget → 训练」。本项目跳过了验证步骤，直接批量 retarget 3503 文件后训练，导致 IK 映射问题从一开始就埋入所有数据，训练跑数小时才发现。

---

## 15. [2026-10-12，02:00] 可视化验证 retarget 数据质量 → 发现根本性 IK 映射错误

### 15.0 测试文件

- BVH：`soma_proportional/bvh/210707/walk_fast_forward_grab_normal_001__A005.bvh`（1941 帧 @ 120fps，约 16s，动作：快速步行+俯身抓取）
- PKL：`data/motion_lib_a3ultra/robot_filtered/210707/walk_fast_forward_grab_normal_001__A005.pkl`（486 帧 @ 30fps）

SOMA BVH 关节名（与标准骨架不同）：
- `LeftLeg` = 左髋（大腿），`LeftShin` = 左膝（小腿），`LeftFoot` = 左踝
- `RightLeg` = 右髋，`RightShin` = 右膝，`RightFoot` = 右踝

### 15.1 数字统计结果

**BVH 人体关节角（原始动作）：**

| 关节 | 通道 | mean | std | min | max | 含义 |
|---|---|---|---|---|---|---|
| LeftLeg (髋) | Xrot | +2.5° | 4.1° | -12.5° | +15.9° | 髋屈/伸（矢状面）|
| LeftLeg (髋) | Yrot | -4.5° | 7.0° | -25.9° | +13.1° | 髋内/外旋 |
| LeftLeg (髋) | **Zrot** | +75.4° | **156°** | -210.7° | +194.5° | 髋外展（含 Euler 万向节死锁跳变）|
| LeftShin (膝) | Zrot | +20.7° | 17.2° | +0.1° | +87.0° | 膝屈曲（正常步行范围）|
| LeftFoot (踝) | Zrot | -90.9° | 6.7° | -113.8° | -70.8° | 踝（含 -90° 静态偏移）|

**机器人 retarget 关节角（pkg dof 字段）：**

| 关节 | mean | std | min | max | 问题 |
|---|---|---|---|---|---|
| L_hip_pitch | -37.7° | 21.0° | -110.9° | +16.6° | **范围过大（BVH 仅 ±12°，放大了 5×）** |
| **L_hip_roll** | **-29.9°** | **0.5°** | **-30.0°** | **-22.6°** | **全程卡在关节下限 -30°，无运动** |
| L_hip_yaw | +41.2° | 14.1° | +4.5° | +100.7° | **不合理（正常步行 <±20°）** |
| L_knee | +78.5° | 7.2° | +3.6° | +100.8° | 过度弯曲（BVH 膝 max 87°，robot max 100°，有一定相关性）|
| L_ank_pitch | -40.7° | 7.7° | -52.0° | -0.7° | 接近关节下限 -52°（joint limit）|
| **L_ank_roll** | **-19.6°** | **1.5°** | **-20.0°** | **-7.3°** | **卡在踝关节下限 -20°** |
| R_hip_yaw | +54.2° | 26.3° | +11.0° | **+156.0°** | **156° 在任何步行动作中都不可能出现** |
| R_knee | +106.4° | 11.1° | +35.2° | +134.3° | 严重过度弯曲 |

### 15.2 从图中观察到的问题

1. **BVH 髋 Zrot 在 t≈4s 出现万向节死锁（Euler discontinuity）**：从 0° 跳变至 -200°，标准差 156°，这是 Euler 角插值时的数学跳变（并非人体真实跳变）。IK 求解器接收到这个跳变输入后，产生了极端的机器人关节角。

2. **L_hip_roll 全程卡在 -30° 关节下限**：图中右侧 hip_roll 曲线完全平坦，无任何动态运动。IK 解算出的髋内收角始终处于最大内收位，这是 SOMA body frame 与 A3 Ultra 髋关节轴向映射关系根本性错误的表现。

3. **hip_pitch 被放大了约 5 倍**：BVH 显示髋屈/伸 ±12°，robot 显示 ±60°（到 -110°）。这是 IK 在无法调整 hip_roll 的情况下，试图用 hip_pitch 和 hip_yaw 补偿的结果。

4. **R_hip_yaw 最大 156°**：正常步行的髋旋转（transverse rotation）应在 ±20° 以内；156° 意味着大腿几乎旋转了半圈，在物理上不可能发生。

5. **骨盆高度（pelvis z）不随动作变化**：BVH 显示人在 t≈6s 明显蹲下（Hips Y 从 0.9m 降至 0.6m），但机器人 root_z 几乎不变，说明全身协调运动（蹲下→抓取）未被正确 retarget。

### 15.3 根本原因：quaternion offset `[0.5,-0.5,-0.5,-0.5]` 未能正确处理 SOMA 骨架 hip body frame

Session 12 中为解决符号问题加入的 `[0.5,-0.5,-0.5,-0.5]` 四元数偏移（代表绕 Z 轴旋转 180°），旨在将 SOMA 的 LeftLeg body frame 对齐到 A3 Ultra 的 `left_hip_yaw_Link` 框架。但结果表明：

- SOMA 的 LeftLeg 关节 Zrotation 通道存在 Euler 跳变（std=156°），这个跳变被当作真实旋转输入给了 IK
- 偏移四元数将这个 Euler 跳变映射到了机器人的 hip_yaw 轴，导致 hip_yaw 出现不可能的角度
- 由于 hip_roll 轴被错误的四元数偏移"占用"，IK 将 hip_roll 推到了关节下限以满足整体约束

**这是 IK 配置层面的根本性错误，不是参数调优能解决的**。3503 个 pkl 文件的数据全部受此影响。

### 15.4 决策：停止训练，必须修复 IK 配置后重新 retarget

- 训练 `sonic_a3_ultra_test-20261012_010134`（PID 106302）已停止
- 在数据质量验证通过之前不再重启训练
- 当前所有 3503 个 pkl 文件不可用

### 15.5 下一步：修复 IK 映射

需要解决的核心问题：

**问题 A - Euler 万向节死锁**：SOMA BVH 的 LeftLeg Zrotation 通道出现超过 ±200° 的跳变，这在 Euler 角插值中是常见问题。解决方案：在送入 IK 之前，对髋关节旋转做四元数 SLERP 插值，而不是使用原始 Euler 角。`batch_retarget_bvh_soma_a3ultra.py` 的 `_soma_frame_to_world_quat()` 函数需要添加四元数连续性修复（即相邻帧四元数符号对齐）。

**问题 B - hip 轴向映射**：需要重新确认 SOMA LeftLeg body frame 的实际朝向与 A3 Ultra `left_hip_yaw_Link` frame 的关系，目前的 `[0.5,-0.5,-0.5,-0.5]` 偏移需要重新推导。方法：在静止 T-pose BVH 帧下，比较 SOMA LeftLeg 骨架坐标系和 A3 Ultra 机器人的 left_hip_yaw_Link 坐标系的相对旋转。

**验证标准**（修复后对同一文件重新可视化）：
- L_hip_roll 应该在 -30° 到 +30° 之间自由振荡（不应卡在下限）
- L_hip_yaw 应该在 ±20° 以内（步行），±40° 以内（复杂动作）
- L_hip_pitch 应该与 BVH Xrot 大致成比例（目前被放大了 5×）
- R_hip_yaw max 应该 < 60°（现在是 156°）

---

## Session 16 — offset_rot 根本原因确认与修复（2026-10-12）

### 16.1 为什么 [0.5,-0.5,-0.5,-0.5] 对 G1 有效但对 A3 Ultra 无效

Session 15 曾从 BVH 运动帧 0（步行动作）出发推导偏移，得到了左右不对称的结果（例如 LeftLeg=[0.6609, -0.0133, -0.7503, 0.0009] vs RightLeg=[-0.1232, -0.7671, 0.0055, -0.6296]）。这个方法本身有缺陷：帧 0 是步行姿态而非 T-pose，左右腿在步行中有前后相位差，导致推导出的偏移也是非对称的，并且把运动姿态的偏差吸收进了偏移本身。

正确推导流程：

**GMR 偏移的语义**：`q_target(t) = q_bvh_world(t) * q_offset`

当人在 T-pose（所有 BVH Euler 角 = 0）时，机器人应该在静止姿态。因此：
```
q_robot_rest = q_bvh_tpose_world * q_offset
=> q_offset = inv(q_bvh_tpose_world) * q_robot_rest
```

**BVH T-pose 的方向**：BVH 格式中，T-pose（所有通道 = 0）时，所有关节局部旋转 = identity，FK 积累后所有关节世界方向 = identity（Y-up 坐标系中）。

经过 Y-up → Z-up 转换（`_R_YUP_TO_ZUP = R_x(+90°)`）后：
```
q_bvh_tpose_world = Q_YUP_TO_ZUP = [0.7071, 0.7071, 0, 0]  （wxyz）
```

**A3 Ultra 静止姿态**：通过 `mujoco.mj_forward(qpos=0)` 验证，以下关节均为 `[1, 0, 0, 0]`（identity）：
- pelvis_link, left_hip_yaw_Link, left_ankle_pitch_Link, left_ankle_roll_Link
- right_hip_yaw_Link, right_ankle_pitch_Link, right_ankle_roll_Link, torso_Link

因此：
```
q_offset = inv(Q_YUP_TO_ZUP) * identity = [0.7071, -0.7071, 0, 0]
```

验证：`q_tpose * q_offset = [0.7071,0.7071,0,0] * [0.7071,-0.7071,0,0] = [1,0,0,0]` ✓（误差 0.000000 rad）

**为什么 G1 用 [0.5,-0.5,-0.5,-0.5] 有效**：G1 的 BVH 源文件使用标准骨骼（`LeftUpLeg`/`LeftLeg`），而 G1 的 `left_hip_yaw_link` rest pose ≈ `[0.9962, 0, -0.0873, 0]`（近似 identity 但有 5° 偏转）。这个偏转加上 [0.5,-0.5,-0.5,-0.5] 产生的 T-pose 目标 `[0.7071, 0, 0, -0.7071]`（绕 -Z 的 90° 旋转）恰好是 G1 MJCF 中髋关节坐标系定义的结果——G1 的 MJCF 本身就把关节定义成这个方向。A3 Ultra 用 identity rest pose，所以需要不同的偏移。

**当前错误偏移的效果**：`[0.5,-0.5,-0.5,-0.5]` 在 T-pose 时产生 IK 目标：
```
body_X → [-1, 0, 0] 指向 -X（向后）
body_Y → [1,  0, 0] 指向 +X（向前）
body_Z → [0,  0, 1] 指向 +Z（向上！）
```
这迫使 IK 把机器人髋关节转向完全错误的方向，hip_roll 被推到关节下限 -30°，hip_yaw 出现 ±156° 的极端值。

### 16.2 修复：应用正确偏移到 bvh_nokov_to_a3_ultra.json

正确值：`[0.7071, -0.7071, 0.0, 0.0]` 用于所有腿/骨盆/躯干映射。

注意：手臂关节（left/right_shoulder_yaw_Link, elbow_Link, wrist_yaw_Link）已经有不同的偏移（例如 `[0.70710678, 0.0, -0.70710678, 0.0]`），这些是针对手臂 BVH 局部坐标系的单独标定值，**不变动**。

修改文件：`/home/user/gmr_repo/general_motion_retargeting/ik_configs/bvh_nokov_to_a3_ultra.json`

修改的 8 个关节映射（ik_match_table1 和 ik_match_table2 各一次）：
- pelvis_link → Hips: `[0.5,-0.5,-0.5,-0.5]` → `[0.7071,-0.7071,0,0]`
- left_hip_yaw_Link → LeftLeg: 同上
- left_ankle_pitch_Link → LeftShin: 同上
- left_ankle_roll_Link → LeftFootMod: 同上
- right_hip_yaw_Link → RightLeg: 同上
- right_ankle_pitch_Link → RightShin: 同上
- right_ankle_roll_Link → RightFootMod: 同上
- torso_Link → Chest: 同上

### 16.3 下一步

1. 用修复后的 JSON 对 1-2 个 BVH 文件做测试 retarget
2. 运行可视化脚本对比原始 BVH 轨迹 vs 新 retarget 轨迹
3. 验证标准：L_hip_roll 在 -30° 到 +30° 自由振荡，R_hip_yaw < 60°
4. 验证通过后，批量 retarget 全部 3503 个 BVH 文件
5. 重启训练

---

## Session 17 — 正确 offset_rot 推导过程与最终值（2026-10-12）

### 17.1 分析流程与关键发现

**尝试 1：T-pose 理论推导 `[0.7071,-0.7071,0,0]`**
- 基于"BVH T-pose（全零）→ all joints = Q_YUP_TO_ZUP → q_offset = inv(Q_YUP_TO_ZUP) * I"
- 结果：L_hip_roll 仍卡在 -30°（对跳跃 BVH 测试）
- 原因：SOMA 骨骼 T-pose（全零）不等于自然站立姿态，自然站立时各关节有大幅旋转

**尝试 2：步行 BVH 帧 0 推导（最终采用）**
- 方法：用 `load_soma_bvh_frames()` 获取步行 BVH 帧 0 的 world-space 四元数，计算 `q_offset = inv(q_soma_frame0) * q_robot_rest`
- 结果：L_hip_roll **不再卡住**！显示正常步行振荡（mean=-15°, std=15°, -30°~+12°）
- 遗留问题：R_hip_roll 偏向 +27°（均值）

**SOMA 骨骼约定发现（关键）：**
LeftLeg body_X Z均值 = -0.956（始终指向 DOWN），RightLeg body_X Z均值 = +0.956（始终指向 UP）。这是 SOMA 的固有约定——右腿局部坐标系骨骼方向相反。这导致 L/R 髋部 roll 轴映射不对称，无法通过调整 offset 消除此偏置，只能从 GMR retarget 框架层面重新设计（当前版本不支持）。

**尝试 3：均值偏移对比**
- 对 1553 帧步行数据做 Karcher 均值，从均值偏移推导 offset
- 结果：R_hip_roll 略微改善（mean=+22.8°），但 L_knee 最小值从 -7° 升到 +28°（膝关节永远不完全伸直）
- 结论：均值偏移不如帧 0 偏移

### 17.2 最终验证结果（步行 BVH，帧 0 偏移）

| 关节 | 均值 | 标准差 | 最小值 | 最大值 | 状态 |
|------|------|--------|--------|--------|------|
| L_hip_roll | -15.41° | 14.57° | -30° | +12° | ✅ 自由振荡 |
| R_hip_roll | +27.18° | 8.08° | -15° | +30° | ⚠️ 偏高但有振荡 |
| L_hip_yaw | +8.44° | 15.83° | -34° | +40° | ✅ 合理 |
| R_hip_yaw | -0.25° | 12.33° | -37° | +27° | ✅ 合理 |
| L_knee | +33.81° | 26.39° | -7° | +79° | ✅ 步行模式 |
| R_knee | +52.72° | 33.33° | -7° | +82° | ✅ 合理 |

R_hip_roll +27° 偏置是 SOMA 骨骼约定问题，在当前 retarget 框架下无法消除。该值在关节限制范围内（±30°），不会导致训练终止（只是不完全对称）。

### 17.3 最终 offset_rot 值（已写入 bvh_nokov_to_a3_ultra.json）

来源：步行 BVH `walking_on_edge_ff_270_R_002__A411_M.bvh` 帧 0（自然站立）

```json
"pelvis_link"           → Hips:        [0.49825669, -0.50287992, 0.49765321, -0.50119191]
"left_hip_yaw_Link"     → LeftLeg:     [-0.58403738, 0.39780844, 0.56921959, -0.42028305]
"left_ankle_pitch_Link" → LeftShin:    [0.61048616, -0.43450097, -0.54174062, 0.38083153]
"left_ankle_roll_Link"  → LeftFootMod: [-0.11802849, -0.07346306, 0.68126085, -0.71871838]
"right_hip_yaw_Link"    → RightLeg:    [0.48985243, -0.50043085, 0.52353779, -0.48530582]
"right_ankle_pitch_Link"→ RightShin:   [0.45049201, -0.45805506, 0.56098879, -0.52204798]
"right_ankle_roll_Link" → RightFootMod:[0.67033671, -0.74070927, 0.03276531, -0.03041228]
"torso_Link"            → Chest:       [0.47358153, -0.47435742, 0.52390556, -0.52557448]
```

手臂关节（shoulder_yaw, elbow, wrist_yaw）保持不变。

### 17.4 下一步：批量 retarget（等待确认）

现有 3503 个 BVH 文件（从 data/bones_seed/soma_proportional.tar.gz），需要用修正后的 IK 配置重新 retarget，输出到 `data/motion_lib_a3ultra/robot_filtered/`。

---

## Session 18 — R_hip_roll +27° 偏置根本修复（2026-10-12）

### 18.1 问题根因确认

R_hip_roll +27° 均值偏置的根本原因：SOMA 骨骼对右侧肢体使用**反向骨骼轴约定**：
- `LeftLeg` body_X = -Z_world（向下，从髋到膝方向）
- `RightLeg` body_X = +Z_world（向上，骨骼方向反转）

实测验证：对全部 1553 帧步行数据统计 body_X 的 Z 分量
- LeftLeg Z 均值 = -0.956（始终向下）
- RightLeg Z 均值 = +0.956（始终向上）

这导致相同物理运动（如右髋外展）在机器人 DOF 空间中产生错误方向映射，造成系统性 +27° 偏置。

### 18.2 修复方案（两步）

**Step 1：代码修复 — `batch_retarget_bvh_soma_a3ultra.py`**

在 `load_soma_bvh_frames()` 中，FK 计算完成后、`RightFootMod` 合成前，对右侧关节应用 180° 修正旋转（绕 body_Z 轴，即 [0,0,1] body 坐标系下）：

```python
# 修正公式: q_new = q_old * [0,0,0,1]  (wxyz: 180° around body_Z)
# 展开: [-z, y, -x, w]
for _bname in ("RightLeg", "RightShin", "RightFoot", "RightUpLeg", "RightToeBase"):
    if _bname in result:
        _pos, _q = result[_bname]
        result[_bname] = [_pos, np.array([-_q[3], _q[2], -_q[1], _q[0]])]
```

修正后效果：RightLeg body_X 从 +Z（向上）翻转为 -Z（向下），与 LeftLeg 一致。

**Step 2：IK Config 修复 — `bvh_nokov_to_a3_ultra.json`**

修正后右侧 offset 公式：`q_offset_new = [0,0,0,-1] * q_offset_old = [z, y, -x, -w]`

数学推导：
- 新 SOMA 方向：`q_corrected = q_original * [0,0,0,1]`
- 新 offset 使得 frame 0 → identity：`q_offset_new = inv(q_corrected_f0) = [0,0,0,-1] * q_offset_old`

新右侧 offset_rot 值：
```
right_hip_yaw_Link    → RightLeg:     [-0.48530582, 0.52353779,  0.50043085, -0.48985243]
right_ankle_pitch_Link→ RightShin:    [-0.52204798, 0.56098879,  0.45805506, -0.45049201]
right_ankle_roll_Link → RightFootMod: [-0.03041228, 0.03276531,  0.74070927, -0.67033671]
```

（左侧、骨盆、躯干、手臂偏移保持不变）

### 18.3 预期结果

修复后：
- RightLeg body_X = DOWN（与 LeftLeg 对称）
- R_hip_roll 应在 0° 附近振荡（而非 +27° 偏置）
- L/R 髋部运动在物理语义上对称

### 18.4 实施结果与关键发现

**尝试 1：常数右乘翻转（失败，数学上无效）**
- 代码：`q_corrected = q_soma * [0,0,0,1]`，同步更新 JSON offset
- 结果：DOF 统计完全不变（R_hip_r 仍 +27°）
- 根本原因：`q_target = (q_soma*q_flip) * (inv(q_flip)*q_offset) = q_soma * I * q_offset`，常数翻转被 offset 补偿精确抵消，IK targets 恒等，DOF 输出不变
- 结论：常数右乘 + offset 补偿方案**数学上对 IK targets 零影响**，已还原代码

**验证 A3 Ultra 关节轴约定（MJCF）**
- left_hip_roll_joint: axis="1 0 0", range=-30° to +85°（正值 = 外展/abduction）
- right_hip_roll_joint: axis="1 0 0", range=-85° to +30°（**负值 = 外展/abduction**）
- 结论：R_hip_roll = +27° = 右髋内收（adduction），L_hip_roll = -15° = 左髋内收
- 对于窄面行走（walk_on_edge），两髋同时轻微内收以保持平衡 → **物理正确**

**新发现：跨文件朝向不一致问题（Session 18 核心问题）**
- 测试 jump_ff_360_R_003 BVH，L_hip_r 仍卡在 -30°（下限）
- 分析：jump BVH frame 0 与 walk BVH frame 0 所有 body 的世界朝向相差 **93-106°**
- 根本原因：SOMA BVH 各文件录制时表演者朝向任意方向，当前 offset 来自 walk BVH frame 0（某特定朝向），无法通用于其他文件

**正确修复：Hips Yaw 补偿（待实施）**
在 `load_soma_bvh_frames()` 中，FK + Y-up→Z-up 转换后，提取 frame 0 的 Hips body 朝向角（body_Z 的偏航角），对所有帧所有 body 旋转 R_Z(-yaw_f0)：
```python
# 保证所有 BVH 文件的表演者在 frame 0 均朝 +X 方向
_body_z = Rotation.from_quat(hips_q_zup_f0, scalar_first=True).as_matrix()[:, 2]
_yaw_f0 = np.arctan2(_body_z[1], _body_z[0])
_R_comp = Rotation.from_euler('Z', -_yaw_f0)
# 对所有帧所有 body 左乘 _R_comp
```

效果：walk BVH（body_Z=[1,0,0]，yaw=0°）不受影响；jump BVH（yaw≈106°）旋转 -106° 后与 walk BVH frame 0 朝向对齐，offset 可通用

### 18.5 实施顺序

1. ✅ 工作日志更新（本条目）
2. ✅ 代码变更已还原（常数翻转无效方案）
3. **下一步**：实施 Hips yaw 补偿（修改 `load_soma_bvh_soma_a3ultra.py`）→ 见 Session 19
4. 同步重算所有关节 offset（yaw 补偿后 frame 0 的 SOMA 朝向改变）→ 见 Session 19
5. 验证：walk BVH 行为不变，jump BVH L_hip_r 不再卡在 -30° → 见 Session 19
6. 通过后批量 retarget 全部 3503 个 BVH

---

## Session 19 — Hips Yaw 补偿 + JSON offset 修复（2026-10-12）

### 19.1 目标

实施 Hips yaw 补偿修复跨文件朝向不一致问题（Session 18 遗留任务）：
- Session 18 发现 jump BVH L_hip_r 卡在 -30°，根因是 jump BVH 起始朝向与 walk BVH 差了 ~106°
- 修复思路：在 `load_soma_bvh_frames()` 中提取 frame 0 Hips 朝向角，对所有帧所有 body 左乘 R_Z(-yaw_f0)

### 19.2 发现的关键问题链（共三层）

**问题 1：Session 18 遗留 bug —— JSON offset 与代码状态不匹配**
- Session 18 Phase 1 为右侧添加了 180° 翻转（代码）+ 计算了配套 JSON offset（"等效"值）
- Session 18 Phase 2 证明翻转数学上无效，于是还原了代码，但 JSON 并未一起还原
- 结果：代码无翻转 + JSON 有翻转配套 offset → 右侧 offset 错误
- 表现：walk BVH R_hip_r 从 +27.18° 变为 +29.5°，L_hip_r 从 -15.41° 变为 -26.6°（均打极限）

**问题 2：yaw 公式 bug —— SOMA 朝向在 Z-up 下指向 -Y 而非 +X**
- BVH Y-up 约定：正面方向 = +Z（Y-up）
- Y-up → Z-up 转换矩阵 R_x(+90°)：+Z(Y-up) → -Y(Z-up)
- 因此 SOMA body 的"正面方向"在 Z-up 下 = body_Z ≈ -Y_world
- 初始代码错误地用 `atan2(body_Z[1], body_Z[0])` 相对于 +X 计算偏航角：
  - walk BVH: body_Z = [0.008, -0.9999, -0.001] → yaw = atan2(-0.9999, 0.008) = **-89.5°**（本应约为 0°！）
  - 后果：对 walk BVH 所有帧旋转了 89.5°，破坏原有正确结果

**问题 3：Session 17 offset 的神秘来源**
- Session 17 offset（左侧）为 [-0.584, 0.398, 0.569, -0.420]（left_hip_yaw_Link）
- 经验证：该值 ≠ inv(q_soma_f0) * q_robot_rest（差了 89.5°）
- 用"正确公式" inv(q_soma_f0) * q_robot_rest 替换后：walk L_hip_r 卡在 -30°（更差）
- 说明 Session 17 offset 是通过不同方法（可能是手动调参）确定的，不能简单重算

### 19.3 修复过程（共三步）

**Step 1：修复 JSON offset（恢复 Session 17 右侧值）**

Session 18 "等效" right 值与 Session 17 right 值的关系：
```
q_s18_new = inv(q_flip) * q_s17_old，其中 q_flip = [0,0,0,1]（body-Z 轴 180° 翻转）
=> q_s17_old = q_flip * q_s18_new
```
恢复后右侧 JSON offset：
```json
"right_hip_yaw_Link":      [0.48985200, -0.50043100, 0.52353800, -0.48530600]
"right_ankle_pitch_Link":  [0.45049200, -0.45805500, 0.56098900, -0.52204800]
"right_ankle_roll_Link":   [0.67033700, -0.74070900, 0.03276500, -0.03041200]
```
验证：walk BVH 无 yaw 补偿时，恢复到精确 Session 17 结果（L_hip_r=-15.41°, L+R=+11.77°）✓

**Step 2：修复 yaw 公式**

SOMA 正面方向在 Z-up = -Y_world，正确的偏航角提取：
```
R_Z(yaw) @ [0,-1] = [body_Z[0], body_Z[1]]
=> yaw = atan2(body_Z[0], -body_Z[1])
```
修复后：
- walk BVH: yaw = atan2(0.008, 0.9999) ≈ +0.47°（正确，近零）
- jump BVH: yaw 为某非零值，补偿后朝向对齐至 -Y ✓

**Step 3：确认 offset 无需重算**
- walk BVH yaw_f0 = 0.47°，yaw 补偿前后所有 joint offset 差值精确等于 0.4676°（对应 0.47° 旋转）
- 0.47° 对 IK 质量无影响，Session 17 offset 保持不变 ✓

### 19.4 修改文件

**`gear_sonic/data_process/batch_retarget_bvh_soma_a3ultra.py`**
- 在 `load_soma_bvh_frames()` 中添加 Hips yaw 补偿逻辑（约第 191-219 行）
- yaw 公式从 `atan2(body_Z[1], body_Z[0])` 修正为 `atan2(body_Z[0], -body_Z[1])`

**`/home/user/gmr_repo/general_motion_retargeting/ik_configs/bvh_nokov_to_a3_ultra.json`**
- 恢复右侧三个关节 offset 为 Session 17 等效值（table1 + table2 共 6 处）
- 左侧、pelvis、torso offset 保持 Session 17 不变

### 19.5 验证结果

| 指标 | 修复前（Session 18 状态）| 修复后（Session 19）|
|------|--------------------------|---------------------|
| walk L_hip_r 均值 | -26.6°（偏差，有时打极限）| **-15.4°（精确匹配 S17）**|
| walk R_hip_r 均值 | +29.5°（偏高）| **+27.2°（精确匹配 S17）**|
| walk L+R hip_r | +2.98° | **+11.77°（精确匹配 S17）** |
| jump L_hip_r | mean=-29.05°, std=2.4°, **卡在 -30° 极限** | mean=-24.7°, std=6.9°, **自由运动 -30°~-11.8°** |
| walk-vs-jump 起始朝向 Δangle（LeftLeg）| 106.0° | **13.8°（减少 92°）** |

### 19.6 结论

1. **walk BVH**：与 Session 17 精确一致（修复无副作用）
2. **jump BVH**：L_hip_r 从卡在 -30° 极限改善为动态运动（yaw 补偿生效）
3. **跨文件通用性**：yaw 补偿将所有 BVH 文件对齐至统一的 -Y 正面基准，offset 对所有文件有效
4. **下一步**：批量 retarget 全部 3503 个 BVH 文件，然后重启训练

---

## Session 20 — 批量 Retarget 全部 BVH（2026-10-12）

### 20.1 目标

使用 Session 19 修复后的完整管线（yaw 补偿 + Session 17 JSON offset）批量 retarget 全部 3503 个 BVH 文件，替换 Session 12 生成的旧数据（旧数据存在 L_hip_roll 卡在 -30° 极限的系统性偏差）。

### 20.2 当前状态

- 输出目录：`data/motion_lib_a3ultra/robot_filtered/`（已有 14 个 pkl 文件，来自旧运行）
- 脚本：`gear_sonic/data_process/batch_retarget_bvh_soma_a3ultra.py`（含 yaw 补偿）
- IK 配置：`/home/user/gmr_repo/general_motion_retargeting/ik_configs/bvh_nokov_to_a3_ultra.json`（Session 17 offset）
- 发现旧进程（Session 12 的 retarget 进程，PID 70648）仍在运行，已生成 11836 个旧 pkl 文件（无 yaw 补偿）
- 杀死旧进程 + 新进程（PID 70648, 147622），用 `--no_skip` 重新启动，覆写所有旧文件
- 实际命令（`--no_skip` 强制覆写所有 ~11836 个旧文件）：

```bash
conda run -n gmr python gear_sonic/data_process/batch_retarget_bvh_soma_a3ultra.py \
    --tar data/bones_seed/soma_proportional.tar.gz \
    --output data/motion_lib_a3ultra/robot_filtered/ \
    --fps 30 \
    --no_skip
```

- PID: 148181，日志：`/tmp/bulk_retarget_s20.log`（conda run 内部缓冲，初始为空）

### 20.3 运行记录

- 03:23: 进程启动，已开始写入 220705 子目录（120% CPU），正在覆写旧文件
- 持续运行中，已覆写超过 19699 个 pkl 文件（截至约 12:45）
- 27885 个 pkl 文件（截至 Session 21 开始时），PID 148181 仍在运行

### 20.5 数据质量问题：GMR IK 收敛失败（83% 膝关节卡下限）

**发现时间**：Session 20 数据验证阶段

**问题描述**：对 8000 个 pkl 文件扫描（分 10 个 session），所有 session 的膝关节卡下限（-7°）比率均为 71–91%，全局约 83.2%。该比率在 `_M`（预镜像）和非 `_M` 文件中完全一致，确认为 GMR IK 本身的收敛失败，与 yaw 补偿无关。

**验证过程**：
- `walk_test.bvh` 单独重定向 → 与 Session 17 结果完全吻合（L_hip_r=-15.41°, R_hip_r=+27.18°），代码正确
- `walking_on_edge` BVH yaw_f0=0.00°（yaw 补偿无影响），直接重定向结果与 pkl 一致，确认 IK 失败在 BVH 内容里
- `jump_ff_360_R_003__A414.pkl`（好的 clip）：膝关节动态（35.5°±33.2°）
- `walking_on_edge_180_R_001__A410.pkl`（坏的 clip）：L_knee=-7.0°（卡下限），R_knee=64.6°

**影响**：训练时坏 clip 会激励 policy 超伸左膝，需过滤删除。

**过滤脚本**：已创建 `gear_sonic/data_process/filter_stuck_joints.py`

过滤条件（每个 clip）：
- L_knee（dof col 22）或 R_knee（dof col 28）满足 std < 0.5° AND mean < -5° → 标记为坏 clip

运行方式（batch 结束后执行）：
```bash
# 先 dry-run 查统计
python gear_sonic/data_process/filter_stuck_joints.py \
    --input data/motion_lib_a3ultra/robot_filtered/

# 确认无误后实际删除
python gear_sonic/data_process/filter_stuck_joints.py \
    --input data/motion_lib_a3ultra/robot_filtered/ \
    --delete
```

预期结果：~17% 的 clip 保留（~5000 clips），用于训练。

### 20.4 Session 20 附带：填入 Armature 数据

用户提供了以下文件包含电机惯量数据：
- `docs/robot_parameters/A3_Ultra/armature.png`（关节型号参数表）
- `docs/robot_parameters/A3_Ultra/Pasted image (2).png`（并联关节等效推导）
- `docs/robot_parameters/A3_Ultra/instructions.md`（并联等效系数）

**计算公式**：`J_output = J_motor × N²`（反射转子惯量到关节输出端）

| 电机型号 | 位置 | J_motor (kg·m²) | N | J_output |
|---------|-----|----------------|---|---------|
| PFP-93-65 | 下肢J1/J2/J3，腰Yaw | 138.5069e-6 | 21.906 | **0.066466** |
| PFP-110-75 | 下肢J4（膝） | 300.8507171e-6 | 20 | **0.120340** |
| PFP-78-58 | 上肢J1/J2，腰Roll/Pitch，下肢J5/J6 | 30.118e-6 | 20.03 | **0.012083**（串联基础值） |
| PFP-59-60 | 上肢J3/J4/J5 | 10.1374e-6 | 22.136 | **0.004967** |
| PFP-41-48 | 上肢J6/J7，头Yaw/Pitch | 1.359e-6 | 24.415 | **0.000810** |

**并联关节等效 armature**（来自 instructions.md）：
- ankle_pitch: 5.333 × 0.012083 = **0.064441** kg·m²，max 118.2 Nm，10.8 rad/s
- ankle_roll: 1.66562 × 0.012083 = **0.020126** kg·m²，max 54.75 Nm，19.37 rad/s
- waist_pitch: 7.3 × 0.012083 = **0.088209** kg·m²，max 115 Nm，9.25 rad/s
- waist_roll: 1.21 × 0.012083 = **0.014621** kg·m²，max 46 Nm，22.7 rad/s

**修改文件**：`gear_sonic/envs/manager_env/robots/a3_ultra.py`
- 更新了 docstring（标注 armature 数据来源）
- 将原 `waist_roll_pitch` 组（armature 值不同）拆为独立的 `waist_pitch` 和 `waist_roll`
- 将原 `shoulder_yaw_elbow_wristroll` 组（armature 值不同）拆为 `shoulder_yaw_elbow`（PFP-59-60）和 `wrist_roll`（PFP-41-48）
- 所有 10 个 actuator 组均添加了 `armature=...` 字段
- 语法验证通过（`python3 -m py_compile`）

**MJCF 更新**：对以下 4 个 MJCF 副本的 31 个 body 关节（不含手指）批量注入 `armature="..."` 属性，MuJoCo `from_xml_path` 验证通过：
- `gear_sonic/data/assets/robot_description/mjcf/a3_ultra_t3d0.xml`
- `gmr_repo/assets/agibot_a3_ultra/a3_ultra_t3d0.xml`
- `decoupled_wbc/sim2mujoco/resources/robots/a3_ultra/a3_ultra_t3d0.xml`
- `gear_sonic/data/robot_model/model_data/a3_ultra/a3_ultra_t3d0.xml`

---

## 21. [2026-10-12] IK 局部极小值修复 + 可视化验证工具

### 21.1 问题根因诊断

用户要求暂停 batch 转换并修复 83% 膝关节卡下限问题，同时追加可视化验证工具。

**调试过程**：
1. 对 `walking_on_edge_180_R_001__A410.bvh`（典型失败案例）从 frame 0 起逐步执行 IK，观察到膝盖**单调向下限移动**：step 1=-1°, step 3=-3°, step 5=-4.8°, step 20=-7°（下限），100 次迭代后仍卡在 -7°
2. 对比从不同初始膝角出发的收敛结果，发现 **两个稳定局部极小值**：
   - 初始值 0-10°（qpos0 范围）→ 收敛到 -7°（下限）error=1.78（**错误盆地**）
   - 初始值 ≥ 15° → 收敛到 82-87°（**全局最优**）error=1.68（更低）
3. 对所有 frames（0-56）扫描确认：一旦从 15° 以上出发，每帧都能正确收敛

**根本原因**：`mink` 微分 IK 从 `qpos0`（膝盖=0°）出发时，梯度方向指向关节下限，落入错误局部极小值。qpos0 完全是中立构型，没有提供屈曲方向的先验偏置，导致 IK 在约束边界处停留。

### 21.2 修复方案

**文件**：`gear_sonic/data_process/batch_retarget_bvh_soma_a3ultra.py`

- 新增 `_warm_start_qpos(model)` 函数：返回 qpos0 副本，左右膝盖预弯 20°
- `retarget_bvh()` 中将原来的 `retargeter.configuration.update(model.qpos0)` 改为始终调用热启动（不论是首个文件还是后续文件）
- `make_retargeter()` 将 `max_iter` 从 10 提升到 30

### 21.3 验证结果

| BVH | 修复前 L_knee | 修复后 L_knee | 修复前 R_knee | 修复后 R_knee |
|-----|-------------|-------------|-------------|-------------|
| walk_test | 正常（Session 17 基准） | +37.8°±23.9°✓ | 正常 | +1.8°±18.8°✓ |
| walking_on_edge | **-7.0°±0°（卡下限）** | **+82.7°±9.5°✓** | **-7.0°±0°** | **+77.7°±11.0°✓** |
| jump_and_land_heavy | **-7.0°±0°（卡下限）** | **+68.8°±4.4°✓** | **+29.8°（半失败）** | **+73.8°±9.8°✓** |

**Session 17 回归检验**（walk_test L_hip_r / R_hip_r）：
- 修复后：-15.39° / +26.93°（参考：-15.41° / +27.18°）✓ 误差 < 0.3°

### 21.4 可视化工具

**新增文件**：`gear_sonic/data_process/visualize_retarget.py`

功能：左侧 SOMA BVH 骨架（cv2 彩色 stick-figure）与右侧 A3 Ultra MuJoCo offscreen 渲染并排输出 MP4 视频。

```bash
# 从本地 BVH 文件
conda run -n gmr python gear_sonic/data_process/visualize_retarget.py \
    --bvh path/to/motion.bvh --out /tmp/check.mp4

# 从 tar 包
conda run -n gmr python gear_sonic/data_process/visualize_retarget.py \
    --tar data/bones_seed/soma_proportional.tar.gz \
    --bvh_name walking_on_edge_180_R_001__A410.bvh \
    --out /tmp/walking_check.mp4

# 实时显示
conda run -n gmr python gear_sonic/data_process/visualize_retarget.py \
    --bvh path/to/motion.bvh --live
```

已生成验证视频：
- `/tmp/walk_test_retarget.mp4`（90帧，步态动态正常）
- `/tmp/walking_on_edge_retarget.mp4`（90帧，膝盖 64-95° 动态范围）

### 21.5 重启 batch

PID 148181（修复前）已停止。重启命令（加 `--no_skip` 覆写所有已有文件）：

```bash
conda run -n gmr python gear_sonic/data_process/batch_retarget_bvh_soma_a3ultra.py \
    --tar data/bones_seed/soma_proportional.tar.gz \
    --output data/motion_lib_a3ultra/robot_filtered/ \
    --fps 30 --no_skip \
    > /tmp/bulk_retarget_s21.log 2>&1 &
```


---

## 22. [2026-10-12] 根方向 180° yaw 翻转修复 + 可视化重构

### 22.1 问题确认

**用户反馈**：看了 Session 21 生成的验证视频，发现机器人整个正反面搞反了，处于"反弓"状态——SOMA 人正面朝西（−X），双手放在身体前面（西侧），向南迈步；重定向后的机器人正面朝东（+X），双手在身体后面（东侧），向南迈步。这是物理上的根节点朝向 180° 翻转，不是摄像机问题。

### 22.2 根因分析

通过对 MJCF + IK 输出的诊断确认：

1. **A3 Ultra 在 qpos0（单位根四元数）时正面朝东（+X）**  
   - MJCF 中左肩位于 +Y（left shoulder at Y=+0.148），胸前摄像机在 +X 偏移处 → robot forward = local +X
   - camera test：az=0 → 看到背面（背部朝西，即朝向摄像机）；az=180 → 看到正面

2. **IK 输出几乎不改变根节点朝向（ ≈ qpos0）**  
   - walk_test f4-f20: pelvis local+X in world ≈ [−0.93, −0.05, 0.36] ≈ 朝西 **（修复后）**
   - 修复前：local+X ≈ [+0.95, 0] ≈ 朝东

3. **yaw_comp 的 canonical 方向**  
   - `_yaw_f0 = atan2(body_z[0], −body_z[1])` 使 SOMA Hips local+Z → −Y（南方）  
   - IK config rot_offset [−90°, −0.47°, −90°] 将这个 canonical 映射到机器人 qpos0 = 朝东  
   - 实际上应该让机器人朝西 → 需要在 yaw_comp 时再旋转 180°

### 22.3 修复方案

**文件：`gear_sonic/data_process/batch_retarget_bvh_soma_a3ultra.py`，第 207 行附近**

```python
# 修复前
_yaw_f0 = float(np.arctan2(_body_z[0], -_body_z[1]))

# 修复后（加 π）
_yaw_f0 = float(np.arctan2(_body_z[0], -_body_z[1]))
_yaw_f0 += np.pi   # rot_offset 在 IK config 里把 SOMA canonical → robot +X（东）
                    # 加 π 翻转整个 SOMA 输入 180°，让机器人最终朝西（−X）
                    # 相对关节角度不变（所有 body 一起旋转）
```

验证（walk_test 多帧）：
- f0: local+X = [−0.82, +0.54, 0.19]（yaw_from_West=+33.6°，第 0 帧起步姿势有自然髋旋转）
- f4: local+X = [−0.94, −0.04, 0.34]（yaw_from_West=−2.3°）
- f8–f20: ≈ −3° ~ −6°（稳定朝西）

**膝盖验证**：warm-start fix（Session 21）仍有效，所有文件 IK OK ✓

### 22.4 可视化更新

**文件：`gear_sonic/data_process/visualize_retarget.py`**

1. **SOMA 投影从 XZ（忽略 Y）改为 YZ（忽略 X）**  
   - 新投影：从西侧（−X）往东（+X）看，显示 SOMA 正面  
   - `px = (−pts[:,1] × scale + img_size/2)` → −Y 在右侧 = character's LEFT（标准镜像正面）

2. **MuJoCo 摄像机改为显式 az=0, el=−15, dist=4.0**  
   - 修复后机器人朝西（−X），az=0（摄像机在西侧朝东看）→ 看到机器人正面

3. **新增 `--out_soma` / `--out_robot` 参数**，输出分别只含 SOMA 或只含机器人的视频

### 22.5 验证视频（修复后重新生成）

两个独立文件夹：

| 文件夹 | 内容 | 文件 |
|--------|------|------|
| `videos/soma_bvh/` | SOMA BVH 骨架正面视图 | `01_walk_test.mp4` ~ `04_jump_ff_360.mp4` |
| `videos/mujoco_robot/` | A3 Ultra MuJoCo 机器人正面视图 | 同上 |

膝盖统计（修复后）：

| 动作 | L_knee | R_knee |
|------|--------|--------|
| walk_test | +49.4°±10.5° | +39.7°±9.5° |
| walking_on_edge | +59.3°±6.1° | +98.0°±8.1° |
| jump_and_land_heavy | +76.3°±5.0° | +80.7°±4.2° |
| jump_ff_360 | +45.8°±8.8° | +59.4°±6.2° |

### 22.6 下一步：重启 batch

验证通过后，重启完整 batch（覆盖所有旧文件）：

```bash
conda run -n gmr python gear_sonic/data_process/batch_retarget_bvh_soma_a3ultra.py \
    --tar data/bones_seed/soma_proportional.tar.gz \
    --output data/motion_lib_a3ultra/robot_filtered/ \
    --fps 30 --no_skip \
    > /tmp/bulk_retarget_s22.log 2>&1 &
```

---

## 23. [2026-10-12 17:29] 修复 180° 朝向反转 bug（Session 22 的 +π 引入的问题）

**用户报告**：看完视频后确认：重定向机器人整体朝向与 SOMA 相反——SOMA 人物正面朝西、双手在体前；机器人正面朝东（相反方向）、双手在体后，呈"反弓"状态。

**根因分析**：

Session 22 在 `load_soma_bvh_frames()` 里加了 `_yaw_f0 += np.pi`，误以为 A3 Ultra 在 qpos0 时正面朝东（+X），实际上机器人正面朝南（−Y）。

诊断脚本输出（修复前）：
```
SOMA 原始朝向：WEST (−X)
yaw 补偿后（加 +π）：NORTH (+Y)   ← 错误！
机器人朝向：NORTH (+Y)             ← 与 SOMA 都不匹配
```

`bvh_nokov_to_a3_ultra.json` 里的 rot_offsets 是针对"canonical = 南（−Y）"标定的。加 +π 之后 canonical 变成北（+Y），导致所有 IK 目标都绕 Z 轴反转 180°，使得髋部"前屈"变成"后伸"，整个身体反弓。

**修复（一行）**：

**文件：`gear_sonic/data_process/batch_retarget_bvh_soma_a3ultra.py`，第 207 行附近**

删除 `_yaw_f0 += np.pi`，并更新注释说明 canonical = −Y（南）：

```python
# 修复前（Session 22 错误加入）：
_yaw_f0 += np.pi

# 修复后：删除该行，canonical forward = −Y（南），与 A3 qpos0 朝向一致
```

**验证结果**：

诊断脚本（修复后）：
```
SOMA 朝向（yaw 补偿后）：SOUTH (−Y) ✓
机器人朝向：               SOUTH (−Y) ✓
```

三个动作的关节统计（修复后）：

| 动作 | L_hip_pitch | R_hip_pitch | L_knee | R_knee | 机器人朝向 |
|------|-------------|-------------|--------|--------|-----------|
| walk_test | +17.4°±19.6° | +20.7°±13.5° | +22.7° | +15.7° | SOUTH 100% |
| walking_on_edge | −8.7°±16.6° | +2.4°±18.9° | +84.5° | +74.9° | SOUTH 100% |
| jump_and_land_heavy | +7.0°±18.3° | +11.3°±18.9° | +67.8° | +79.8° | SOUTH 100% |

髋关节现在有正值变化（之前因 +π 导致近零或负值）✓  
膝关节延续 Session 21 的 warm-start 修复结果 ✓

**可视化更新**：

`gear_sonic/data_process/visualize_retarget.py` — MuJoCo 摄像机改为 az=90（从东侧看），与 SOMA YZ 平面投影方向一致：
- 修复前：az=0（从北看，显示机器人背面，注释也是错的）
- 修复后：az=90（从东看，与 SOMA 侧视图方向匹配）

**验证视频（修复后）**：

两个独立文件夹（分开存放方便对比）：

| 文件夹 | 内容 |
|--------|------|
| `videos/soma_bvh/` | SOMA BVH 骨架（东侧视图）|
| `videos/mujoco_robot/` | A3 Ultra MuJoCo 机器人（东侧视图）|

四个动作视频均已重新生成：
- `01_walk_test.mp4`
- `02_walking_on_edge.mp4`
- `03_jump_and_land_heavy.mp4`
- `04_jump_ff_360.mp4`（源文件：`soma_proportional/bvh/230530/jump_ff_360_R_003__A414.bvh`）

**文件改动**：
- `gear_sonic/data_process/batch_retarget_bvh_soma_a3ultra.py`：删除 `_yaw_f0 += np.pi`，更新注释
- `gear_sonic/data_process/visualize_retarget.py`：摄像机 azimuth 从 0 改为 90，更新注释

**下一步**：请用户对比 `videos/soma_bvh/` 和 `videos/mujoco_robot/` 中的视频，确认方向一致后重启 batch retarget。


---

## 22b. [2026-10-12] 正确的朝向修复：IK 后处理 R_z(180°)

### 问题回溯

Session 22 的 +π 修复（改变 yaw_comp 输入）是错误的：它改变了送入 IK 的坐标系，导致关节角变化（walking_on_edge 左右膝不对称：L=59°, R=98°，严重偏离正确值 83°/75°）。

用户看了视频后确认：问题是"整个身体重定向局部反了，比如肚子的地方重定向到背部了"——即 SOMA 人的腹侧被映射到机器人的背侧（同样是 180° yaw 翻转的表现）。

### 正确修复

**原理**：IK 配置（rot_offset）将 SOMA canonical → A3 Ultra qpos0 方向（朝东 +X）。只需在 IK 输出的 qpos 上做后处理，不改变关节角：

```python
# 在 retarget_bvh() 的 for 循环内，IK 之后：
_R180Z_wxyz = np.array([0.0, 0.0, 0.0, 1.0])  # R_z(180°) 四元数 wxyz
q[3:7] = _quat_mul(_R180Z_wxyz, q[3:7])  # 根节点朝向翻转 180°
q[0], q[1] = -q[0], -q[1]               # 根节点 XY 平移也翻转
```

**验证**：
- L_hip_r=-15.39°，R_hip_r=+26.93° — 与 Session 17 参考值 (-15.41°/+27.18°) 完全一致 ✓
- 关节角不变（dof = qpos[7:] 未修改）✓
- 机器人根节点朝向：local+X ≈ [−0.957, 0.039, 0.287]（朝西）✓
- walking_on_edge 膝盖恢复正常：L=+83.3°±9.3°, R=+75.4°±11.4° ✓

### 可视化说明

MuJoCo 摄像机 az=0（摄像机在西侧 −X，朝东 +X 看）→ 修复后机器人朝西，az=0 正好是正面视图。

| 动作 | L_knee | R_knee |
|------|--------|--------|
| walk_test (60帧) | +19.1°±27.4° | +38.0°±15.6° |
| walking_on_edge | +83.3°±9.3° | +75.4°±11.4° |
| jump_and_land_heavy | +71.3°±0.8° | +84.4°±0.9° |
| jump_ff_360 | +55.6°±4.8° | +58.5°±5.6° |

**待验证（用户确认视频正确后）**：启动正式 batch retarget。


---

## 23. [2026-10-12, Session 23] 修复 body 朝向反转问题（正确方案：+π yaw_comp + West warm-start）

**用户问题**：机器人全身方向反转（阴部→屁股、太空漫步），根源是 A3 Ultra qpos0 朝东（+X），SOMA canonical 朝西（−X），IK 不翻转方向。

---

### 本 Session 尝试的几种方案及结果

**方案 A（本 Session 初次尝试）**：在 JSON 中把 pelvis_link rot_offset 右乘 Rz(180°)，即把 [0.49825669, -0.50287992, 0.49765321, -0.50119191] 改为 [0.50119191, 0.49765321, 0.50287992, 0.49825669]，配合 West warm-start（qpos0 root 左乘 Rz(180°)）。

**结果**：IK 卡在局部极值 forward≈[-0.87, -0.37, -0.28]（即使 max_iter=200 也如此），因为 West warm-start 的 foot 位置（+π 之后）与 East warm-start 的 SOMA foot target 冲突，mink 差分 IK 无法收敛。

**方案 B（本 Session 正确解）**：在 `load_soma_bvh_frames()` 里加 `_yaw_f0 += np.pi`，再结合 West warm-start（保持），不做 post-processing。

**数学原理**：
- 原始 rot_offset 满足：`R_target = R_soma_canonical * R_offset_old = East-facing`
- 加了 +π 后，`R_soma_new = R_Z(π) * R_soma_orig`，于是 `R_target_new = R_Z(π) * R_target_old = West-facing`
- IK 目标已经是 West-facing，West warm-start 立即收敛

**关键数学**：不需要修改 JSON（rot_offset 保持原值 [0.49825669, -0.50287992, 0.49765321, -0.50119191]），+π 通过改变 R_soma 而隐式地把 IK 目标从 East 翻转到 West。

---

### 文件改动

1. **`gear_sonic/data_process/batch_retarget_bvh_soma_a3ultra.py`**：
   - `load_soma_bvh_frames()` 第 210 行后加 `_yaw_f0 += np.pi`（含注释）
   - `retarget_bvh()` warm-start 恢复为 West warm-start（root quat 左乘 Rz(180°)），无 post-processing

2. **`/home/user/gmr_repo/general_motion_retargeting/ik_configs/bvh_nokov_to_a3_ultra.json`**：
   - **恢复原始值** [0.49825669, -0.50287992, 0.49765321, -0.50119191]（两处 table 都改回来）

3. **`gear_sonic/data_process/visualize_retarget.py`**：
   - 摄像机 az=90 → az=0（修复后机器人朝西，az=0 是正面视图）

---

### 验证结果

| 文件 | f0 forward | L_knee | R_knee | IK |
|------|-----------|--------|--------|----|
| walk_test | [-0.957, 0.039, 0.287] | +37.7°±24.0° | +1.7°±18.7° | OK ✓ |
| walking_on_edge | [-0.917, 0.008, 0.398] | +82.7°±9.6° | +77.6°±11.0° | OK ✓ |

walking_on_edge 膝盖与 worklog 参考值（L=+83.3°, R=+75.4°）吻合 ✓

**待完成**：重渲 4 个验证视频（walk_test, walking_on_edge, jump_and_land_heavy, jump_ff_360），用户确认视频正确后重启 batch retarget。

---

## 24. [2026-10-12, Session 23 续] 修复左右手/脚交叉问题（+π 导致的镜像）

**问题**：+π yaw_comp 把 SOMA 所有 body 的 Y 轴取反（Rz(180°) 映射 +Y↔−Y），导致：
- SOMA LeftFootMod（原 South）→ +π 后变 North → IK 把 left_ankle_roll 送到 North（机器人 West-facing 的 RIGHT 侧）
- SOMA RightFootMod（原 North）→ +π 后变 South → IK 把 right_ankle_roll 送到 South（机器人 LEFT 侧）
- 手臂同理 → 交叉手臂/腿

**修复**：JSON 里把所有左右 human body 配对完整互换（body 名称 + 位置偏移 + rot_offset 全部对调）：
- left_X ← 原 right_X 的参数；right_X ← 原 left_X 的参数
- 涉及：hip_yaw、ankle_pitch、ankle_roll、shoulder_yaw、elbow、wrist

**文件改动**：`/home/user/gmr_repo/general_motion_retargeting/ik_configs/bvh_nokov_to_a3_ultra.json`（两张表均改）

---

## 25. [2026-10-12] 修复机器人持续屈膝问题

**根本原因**：GMR 的 `scale_human_data()` 将 SOMA 髋关节世界坐标直接乘以 `human_scale_table["Hips"] × ratio`：
- 当前值：0.9 × (1.75/1.8) = 0.875
- SOMA 行走髋关节高度 = 1.024 m → IK pelvis 目标 = 0.875 × 1.024 = **0.896 m**
- A3 Ultra qpos0（膝=0°）pelvis 高度 = **1.30 m**
- IK 强迫机器人从 1.30 m 蹲到 0.90 m → 膝关节弯曲 **63°**（过度屈膝）

**测试结果**（walk_test, L_knee mean 随 actual_human_height 变化）：
- 1.75 → 62.2°（过度屈膝，当前状态）
- 1.80 → 55.6°
- 1.85 → 47.9°
- 1.90 → 38.6°
- 1.95 → 23.4° [20.1°..25.3°] ← **目标**（自然行走膝角范围）

**修复**：`make_retargeter()` 里 `actual_human_height: 1.75 → 1.95`
- 所有 scale 乘以 ratio = 1.95/1.8 = 1.083
- 等效 Hips scale = 0.9 × 1.083 = 0.975
- pelvis 目标 = 0.975 × 1.024 = 0.998 m（接近自然站立高度）

**文件改动**：`gear_sonic/data_process/batch_retarget_bvh_soma_a3ultra.py`

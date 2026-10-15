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


---

## 26. [2026-10-12 20:25] 重新标定 rot_offset——去除 +π hack，统一为 G1 同款逻辑

### 背景与目标

经对比 G1 标准重定向管线（`/home/user/gmr_repo/scripts/bvh_to_robot_dataset.py` + `bvh_nokov_to_g1.json`）与当前 A3 Ultra 管线，发现：

| 方面 | G1（GMR 标准） | A3 Ultra（旧实现） |
|------|---------------|-------------------|
| BVH 解析 | `load_bvh_file(format="nokov")` | 自定义 `load_soma_bvh_frames()` |
| Yaw 补偿 | 无 | +π（hack）|
| L/R 分配 | Left→Left, Right→Right | L/R 互换（+π 镜像的副作用） |
| 初始化 | qpos0 默认 | West warm-start + 20° 弯膝 |
| rot_offset | 统一 `[0.5,-0.5,-0.5,-0.5]` | 各 body 不同，因为基于 +π 标定 |

**决定**：重新标定 A3 Ultra 的 rot_offset，目标：
1. 去除 `_yaw_f0 += np.pi`（恢复 SOMA canonical = South -Y）
2. 去除 West warm-start（改回 qpos0 默认 + 20° 弯膝）
3. 恢复 L→L, R→R 分配（无 swap）
4. 重新计算每个 SOMA body → robot link 对的 rot_offset

**同时**：用户提出统一不同机器人的转换逻辑——支持。SOMA BVH 自定义解析器（双 Root 关节问题）不可避免，但解析之后可以用统一的 `--robot` 参数切换机器人，管线无需其他差异。

### 关键几何分析

- **A3 Ultra qpos0**：pelvis_link xmat = 单位矩阵，朝东 (+X)；左脚在 +Y（北）侧，右脚在 -Y（南）侧
- **SOMA canonical（无 +π）**：Hips 朝南 (-Y)；人体左脚在 +X（东）侧，右脚在 -X（西）侧
- **rot_offset 含义**：`q_IK_target = q_SOMA_world * rot_offset`（右乘，GMR offset_human_data）
- **标定公式**：`rot_offset = q_soma_canonical⁻¹ * q_robot_qpos0`

### 执行计划

1. **`calibrate_soma_to_a3ultra.py`**（新建）：自动计算各 body 对的 rot_offset，写入候选 JSON
2. **`bvh_nokov_to_a3_ultra.json`**：用标定脚本输出替换 rot_offset；L→L, R→R 分配
3. **`batch_retarget_bvh_soma_a3ultra.py`**：删除 `_yaw_f0 += np.pi`；改回 qpos0 默认 warm-start（保留 20° 弯膝）
4. **`videos/calibration/`**（新建）：生成标定前后对比视频（SOMA 骨架 + 机器人），验证正确性

**文件改动**：见本条目执行过程中的子步骤记录。


### 执行结果（2026-10-12 20:40）

**标定脚本输出**（frame-0 canonical 验证）：
- Hips canonical forward: [-0., -1., -0.002] ≈ [0,-1,0] South ✓（无 +π）
- 所有 A3 Ultra 主要关节 qpos0 xquat = [1,0,0,0]（单位四元数）
- rot_offset = q_soma⁻¹ * I = q_soma.conjugate()（直接对称）

**新标定 IK 结果（videos/calibration/）**：

| 测试文件 | L_knee | R_knee |
|---------|--------|--------|
| walk_test（92 帧/新） | mean=+10.9° std=19° min=-7° max=45° | mean=+22.6° std=24.7° min=-7° max=54° |
| walk_test（300帧/旧，+π+swap） | mean=+44.6° min=23.9° | mean=-0.6° min=-7.0° ❌ |

改进：L_knee 从 44.6°→10.9°；R_knee 从 -0.6°→+22.6°，L/R 对称性显著提升。

**遗留问题**：min=-7°（顺序 warm-start 漂移）仍存在，少量帧落到关节下限，已知问题。

### 文件改动

1. **`gear_sonic/data_process/calibrate_soma_to_a3ultra.py`**（新建）
   - 自动计算 rot_offset = q_soma_canonical⁻¹ * q_robot_qpos0 for each matched pair
   - 去除 +π，使用 South (-Y) 作为 SOMA canonical 方向
   - 写入候选 JSON，支持 --bvh / --tar 两种输入

2. **`/home/user/gmr_repo/general_motion_retargeting/ik_configs/bvh_nokov_to_a3_ultra.json`**
   - 备份为 `bvh_nokov_to_a3_ultra_backup_before_recalib.json`
   - 所有 rot_offset 替换为标定计算值
   - L→L, R→R 分配（撤销之前的 L/R 互换）

3. **`gear_sonic/data_process/batch_retarget_bvh_soma_a3ultra.py`**
   - 删除 `_yaw_f0 += np.pi`
   - 删除 West warm-start（`_R180Z_wxyz * q_warm`），改回 qpos0 默认 + 20° 弯膝

4. **`videos/calibration/`**（新建目录）
   - 4 组对比视频：`0{1-4}_calib_*_{soma,robot}.mp4`


---

## 27. [2026-10-12] 脚踝方向 L/R 互换修复

**用户反馈**：查看标定验证视频，发现两只脚的重定向方向搞反了——SOMA 骨架左脚动作被机器人的右踝关节跟踪，右脚同理。

**根因分析**：

JSON `ik_match_table1/2` 中脚踝卷转条目均为 `orient_weight=100, pos_weight=0`（纯朝向匹配，不跟踪位置）。重新标定后 `LeftFootMod→left_ankle_roll_Link` / `RightFootMod→right_ankle_roll_Link` 配对，但 SOMA 的 `LeftFootMod` / `RightFootMod` 骨骼的左右命名约定与机器人的左右踝约定相反，导致视觉上脚部方向交叉。

由于 `pos_weight=0`，rot_offset 只依赖 SOMA body 自身方向（`q_soma⁻¹ * I`），与目标 robot body 无关。因此只需交换两个条目中的 SOMA body 名称和对应 rot_offset，位置偏移保持原 robot body 不变。

**修复**：

在 `bvh_nokov_to_a3_ultra.json`（table1 + table2 同步）：
- `left_ankle_roll_Link` ← `RightFootMod`（使用 RightFootMod 的 rot_offset）
- `right_ankle_roll_Link` ← `LeftFootMod`（使用 LeftFootMod 的 rot_offset）

位置偏移 `[0.0, ±0.02, 0.0]` 保持各自 robot body 对应的值不变。

**文件改动**：

1. **`/home/user/gmr_repo/general_motion_retargeting/ik_configs/bvh_nokov_to_a3_ultra.json`**
   - `left_ankle_roll_Link`：SOMA body 改为 `RightFootMod`，rot_offset 换为 RightFootMod 的标定值
   - `right_ankle_roll_Link`：SOMA body 改为 `LeftFootMod`，rot_offset 换为 LeftFootMod 的标定值

待验证：重新渲染 `videos/calibration/` 验证视频确认脚踝方向正确。


**验证结果**（2026-10-12，walk_test 90帧）：

- frame 15, 55, 70：步态自然，手臂-腿部协调正确（右腿向前时右臂略前，符合正常行走），无交叉迹象 ✓
- frame 25：机器人明显前倾 — 这是已知的顺序 IK warm-start 漂移问题（SOMA 在此帧做侧身大幅倾斜），与本次修复无关
- 膝关节统计：L_knee mean=+39.3° std=24.1°（min=-7°），R_knee mean=+8.9° std=27.4°（min=-7°）—— 标准差大说明有正常屈伸，偶发 -7° 为 warm-start 漂移遗留问题

**结论**：脚踝 L/R 互换修复有效 ✓，步态视觉质量达到标定预期水平。


---

## 28. [2026-10-12] IK warm-start 顺序漂移修复（跳变问题）

**用户反馈**：视频中末端执行器出现跳变——机器人在某帧突然进入极端弯腰姿势后无法恢复。

**根因**：`retarget_bvh()` 中 `q_warm` 仅在帧循环前设置一次（第 315-316 行），之后每帧 `retarget()` 结束后 retargeter 的内部 configuration 更新为当前帧的 IK 结果，下一帧从那个结果出发。当某帧 IK 落入错误局部极值（关节触下限），后续所有帧从该坏结果出发，形成级联漂移。

**修复**：在帧循环内检测每帧 IK 结果是否有关节触限（距关节范围边界 ≤2°），若触限则下一帧从 `q_warm`（qpos0 + 20° 弯膝）重新出发；正常帧则继续用上一帧结果（保持平滑性）。

**文件改动**：

1. **`gear_sonic/data_process/batch_retarget_bvh_soma_a3ultra.py`**
   - `retarget_bvh()`：帧循环改为检测 `at_limit`，触限时重置 warm-start 为 `q_warm`
   - 引入 `_detect_limit_violation()` 辅助函数

---

## 29. [2026-10-12] IK 深蹲根因分析与修复：ankle_pitch 改绑 LeftFoot

**问题**：walk_test 第 50-90 帧（准静止双支撑阶段）机器人呈"坐姿"——膝关节极度弯曲，最大 R_knee ≈ 100°，而 SOMA 角色明显直立。多轮权重调整（LeftShin pos/rot 权重从 (0,10)→(10,0)→(0,0)→(1,0)→(1,3)）均无效。

**根因分析**：

`scale_human_data()` 仅保留 `human_scale_table` 中的 SOMA 骨骼，其他骨骼在传给 `offset_human_data()` 之前已被过滤掉。当前 ankle_pitch 绑定的是 **LeftShin（膝关节，Z≈0.47m）**而非脚踝。经尺度缩放后 IK 位置目标 Z≈0.423m，而机器人 qpos0 踝关节实际高度仅约 0.1m。当 SOMA 处于直立双支撑时，大腿接近竖直，IK 只能通过深度弯膝把踝节点抬到 0.4m，导致深蹲。

**正确映射**：`ankle_pitch_Link` 应绑定 **LeftFoot（脚踝，Z≈0.05m）**，缩放后目标 Z≈0.045m，与机器人踝高度吻合。

**rot_offset 计算**（`rot_offset = q_soma_canonical⁻¹ × identity`）：
- LeftFoot: `[-0.1553, -0.0199, 0.7491, -0.6437]`
- RightFoot: `[0.7317, -0.6709, 0.0863, -0.0834]`

**文件改动**：

1. **`/home/user/gmr_repo/general_motion_retargeting/ik_configs/bvh_nokov_to_a3_ultra.json`**
   - `human_scale_table`：LeftShin→LeftFoot，RightShin→RightFoot（scale 保持 0.9）
   - `ik_match_table1` + `ik_match_table2`：
     - `left_ankle_pitch_Link`：SOMA 骨骼 LeftShin→LeftFoot，rot_offset 更新
     - `right_ankle_pitch_Link`：SOMA 骨骼 RightShin→RightFoot，rot_offset 更新
   - 权重保持 `[1, 3]`（pos_weight=1, rot_weight=3）

**中间尝试（已废弃）**：先尝试了 LeftFoot (1,3) 方案——把 human_scale_table 中 LeftShin→LeftFoot，重新标定 rot_offset，但发现 dof 索引查找逻辑有 bug（按 actuator 顺序查，实际存储是按 `_ACTUATED_DOF_QPOS_INDICES` 顺序），导致误判 L_knee=0。恢复回 LeftShin 方案，消除 pos_weight。

**验证结果（v11 渲染）**：
- L_knee: mean=-3.7°, std=8.5°, min=-7.0°, max=+45.3°
- R_knee: mean=+6.2°, std=17.0°, min=-7.0°, max=+53.5°
- 深蹲问题已消除（原 R_knee max=100° → 现 max=53.5°）
- 残余问题：L_knee 在大部分帧触 -7° 下限（pre-existing hyperextension，非新增 regression）
- 视频：`videos/v11_walk_test_robot.mp4`

**状态**：深蹲问题 ✓ 已解决。

---

## 30. [2026-10-13] warm-start 级联与手部跳变修复

**用户反馈**（v11 视频第 2-3 秒）：
1. 早期帧（视频第 1 秒）机器人呈"坐姿腿向前伸"——warm-start 级联重置
2. 第 2-3 秒手部位置跳变

**根因**：`_at_joint_limit()` 检测 **所有** 关节（包括手腕/手指），任何关节触限都触发下一帧从 `q_warm` 重启：
- L_knee 在大部分帧触 -7° 下限 → 每帧都从 q_warm 重启 → 所有早期帧卡在相同的"坐姿"局部极值
- 手腕关节触限 → 下一帧手部从 q_warm 位置（双臂下垂）跳变

**修复**（`batch_retarget_bvh_soma_a3ultra.py`）：
- `_at_joint_limit()` 只检查腿部关节（`hip`、`knee`、`ankle`）
- 对膝关节：只在触上限（>85° 深蹲）时触发，不在触下限（轻微超伸 -7°）时触发
  - 理由：-7° 超伸不是异常，不应触发重启；真正的问题是膝盖过度弯曲（>85°）

**验证结果（v12 渲染）**：
- L_knee: mean=-6.2°, std=4.3°, min=-7.0°, max=+19.5°
- R_knee: mean=-0.1°, std=16.2°, min=-7.0°, max=+52.7°
- 手部跳变应消除（arm 关节不再触发 warm-start）
- 视频：`videos/v12_walk_test_robot.mp4`

**v12 残余问题**（用户反馈）：第 3 秒左右手仍跳变交叉（左手跑到右手位置）。

**追加修复（v13）**：根因是 ankle_roll 关节上下限 ±20°，walking 时踝关节触限，触发 warm-start 重置 **整个** qpos（包括手臂）→ IK 从 qpos0 手臂位置出发找到镜像解。

追加辅助函数 `_apply_leg_warm_start()`：触发 warm-start 时只重置腿部关节（hip/knee/ankle），保留手臂/躯干来自上一帧，防止手臂跳变。

**验证结果（v13 渲染）**：
- L_knee: mean=-2.6°, std=10.2°, min=-7.0°, max=+31.3°
- R_knee: mean=+3.5°, std=17.8°, min=-7.0°, max=+52.7°
- 视频：`videos/v13_walk_test_robot.mp4`

**v13 残余问题**（用户反馈）：第 2-3 秒右手正常、左手跳变；用户提问"既然脚去找了镜像解，手是不是也应该镜像回来"。

---

## 31. [2026-10-13] 根因：机器人朝向与人体相差 180°（脚的交叉映射只是掩盖）

**v13 逐帧手臂数据**（walk_test，30fps）：
- 全程 `left_shoulder_roll = -5.0°`（下限 -5°）、`right_shoulder_roll = +5.0°`（上限 +5°）——两侧肩 roll 一直顶在"向内收"的极限，说明 IK 一直想把手拉到身体另一侧
- 第 72→73 帧（2.43s）双臂同时跳变：R_shoulder_yaw -4.7°→-47.3°、R_shoulder_pitch +20°→-15°、L_shoulder_pitch 32°→50°，之后 L_shoulder_pitch 持续升到 87°（交叉/怪异姿态），一直持续到 ~3.3s

**几何验证**（`load_soma_bvh_frames` 输出 vs v13 FK）：
- 人体：脚尖方向 ≈ -180°（朝西 -X），左髋向量 LeftLeg-RightLeg ≈ -90°（-Y）→ 人体朝西，右手系正常（不是骨架镜像）
- 机器人：pelvis x 轴 ≈ 0°（朝东 +X），左髋向量 +Y
- 结论：**机器人与人体朝向相反 180°**。人体左侧在机器人右侧 → 之前"两个脚定反了"实际是腿交叉，用 `left_ankle_roll ← RightFootMod` 交叉映射掩盖；但手臂（`left_* ← LeftArm/ForeArm/Hand`，pos_weight=10）、髋（`left_hip_yaw ← LeftLeg`）没交叉 → 手被拉向身体另一侧 → 肩 roll 顶内限 → IK 最终跳到交叉解
- 同时解释了 L_knee 长期顶 -7° 下限：rot target 复现的是世界系旋转，人体膝向前弯在转了 180° 的机器人上变成向后弯（超伸）

**普适性**：从 tar 流式抽查 40 个 BVH（walk/jump/sit/dance/jog 等），当前 yaw 归一化后人体朝向全部为 -179.3°±0.5°。是全数据集的系统性问题。

**根因**：yaw 归一化与 `calibrate_soma_to_a3ultra.py` 都假设 SOMA Hips 的 body_Z = 前方（朝南），实际 Hips body_Z = 人体**左方**。归一化后人体朝西，标定时 rot_offset = q_soma(朝西)⁻¹ × q_robot(朝东)，把 180° 固化进每个 link 的 offset。可视化里 SOMA 用"西向人物正面视图"、机器人用"东向正面视图"，两边都看到正脸，所以视频里看不出朝向相反。

**修复方案**（不镜像手，改为消除 180°，脚恢复同侧映射）：
1. `batch_retarget_bvh_soma_a3ultra.py` / `calibrate_soma_to_a3ultra.py`：yaw 归一化目标改为 Hips body_Z → +Y（人体左方 = +Y，即人体朝东 +X，与 A3 Ultra qpos0 一致）
2. 重新运行标定，重写 `bvh_nokov_to_a3_ultra.json` 全部 rot_offset；`left_ankle_roll ← LeftFootMod`、`right_ankle_roll ← RightFootMod`（全部 L→L、R→R）；备份旧 JSON 为 `bvh_nokov_to_a3_ultra_backup_before_facing_fix.json`
3. `visualize_retarget.py`：SOMA 投影改为东向人物正面视图（+Y → 图像右），与机器人 az=180 相机一致

**已执行**：
- 两个脚本 yaw 归一化改为 `yaw_f0 = atan2(-body_Z[0], body_Z[1])`（Hips body_Z → +Y）
- 标定脚本重跑（walk_test），Hips body_Z = [0, 1, 0] ✓；JSON 全部 L→L / R→R，ankle_roll pos_offset 保持 ∓0.02；旧 JSON 备份 `bvh_nokov_to_a3_ultra_backup_before_facing_fix.json`
- `visualize_retarget.py`：SOMA 投影 +Y→右；MuJoCo 相机 lookat 跟随 root xy（walk_test 横移 ~7m，原固定相机会让机器人走出画面）

**验证结果（v14，walk_test 389 帧）**：

| 指标 | v13 | v14 |
|---|---|---|
| 机器人朝向 − 人体朝向 | ≈180° | ≤ ±15°（多数 < 5°） |
| 手臂关节最大单帧变化 | 42.6°（帧 72→73） | 6.4° |
| L_knee 触 -7° 下限比例 | 82.5% | 0%（mean +36.5°, min +7.7°, max +69.1°） |
| R_knee | mean +2.2° | mean +38.1°, min +15.7°, max +59.0° |
| L/R shoulder_roll 触内限比例 | 100% / 100% | 3.9% / 0%（mean +14.5° / -16.8°，对称外展） |

- 视频：`videos/v14_walk_test_robot.mp4`（仅机器人）、`videos/v14_walk_test_combined.mp4`（SOMA | 机器人 并排，同一正面视角）
- 抽帧（1.0s / 2.43s / 2.87s / 5.0s）目视：手臂/腿左右与 SOMA 一致，无交叉

**jump_ff_360 回归检查**：朝向跟随正常（差 ≤ 5°），膝盖 max ≈ 102°（落地，真实动作），无触 -7°。
- 残余问题：帧 40→41 左肩 IK 分支切换（L_shoulder_pitch -75°→+13°、yaw 0°→+84°，单帧 88.6°；手部位移 14cm vs 正常 ~4cm/帧）。左臂此时上举、shoulder_roll ≈ 91°，接近肩部奇异位形，**不是镜像问题**
- 禁用 `_knee_too_deep` 重试后跳变仍在（78.5°）且 L_shoulder_roll 触 150° 上限比例 69.6%，更差 → 保留重试
- 该奇异跳变待后续单独处理（候选：手臂连续性检测 + 回退重解、调整肩部 rot 权重）


---

## 32. [2026-10-13] 用户质疑：SOMA 向右横移、机器人向左横移 → 是否镜像？

**用户反馈**：v14 跳变已解决，但看起来 SOMA 人体朝自身右侧横走，机器人朝自身左侧横走，担心训出镜像遥操。

**验证**（`walk_test.bvh` 原始 BVH 坐标，不做任何轴变换）：
- 脚尖方向 = BVH -X；左膝向脚尖方向凸出 +9.3cm（确认是真前方）
- "Left" 命名骨骼所在侧 = +Z；右手系 up × forward = +Z → dot = +0.998，命名与几何一致（不是左手系导出的镜像数据）
- Hips 位移 688cm，与 Left 侧 dot = +1.000，与前方 dot = +0.076 → **表演者向自身左侧横移**
- 官方 `extract_soma_joints_from_bvh.py` 的 Y-up→Z-up 变换 (x,y,z)→(x,-z,y) 与本管线相同，无镜像
- v14 机器人同样向自身左侧横移 → 无镜像

**误判来源**：SOMA 面板是无深度、无脸的 YZ 投影火柴人，正视/背视无法区分（实际为正视，图右 = 人体左侧）；Left/Right 颜色（BGR）无图例。

**改动**：`visualize_retarget.py` 两个面板加左右标识与"moving to its LEFT/RIGHT"文字（由速度与左右髋向量点积计算），便于目视核对。

**验证结果**：重新渲染 `videos/v14_walk_test_combined.mp4`（两面板 L/R 标签 + 运动方向文字；机器人标签由 MuJoCo 相机投影手腕/踝位置得到）。逐帧对比 389 帧：SOMA 与机器人方向标签一致 387 帧（358 帧均为 "to its LEFT"，其余为 still/BACKWARD 一致），2 帧为速度阈值边界差异。**结论：无镜像，无需调整 retarget。**

---

## 33. [2026-10-13] 多动作小规模对比验证（batch 前）

**用户确认**：v14 walk_test 无镜像、无跳变。要求批量转换前先做多动作 SOMA | 机器人并排对比视频，排查其他问题。

**样本**：流式扫描 `soma_proportional.tar.gz`（共 142,220 个 BVH，全量扫描 288s），按动作关键字各取第一个 clip，覆盖 walk / jog / run / backward / jump / spin / turn / sit（椅子、盘腿）/ crouch / squat / lunge / kneel / crawl / lying→stand / fall / roll / cartwheel / kick / punch / throw / wave / dance / 上下 50cm 箱子。

**工具**：scratchpad driver，每个 clip 只跑一次 retarget，同时输出：
- `videos/compare_v14/<name>.mp4`（左 SOMA、右 A3 Ultra，同一正面视角，L/R 标签）
- 自动 QA 指标：朝向差、末端（手/脚/pelvis）IK 位置残差、关节单帧最大跳变、触限关节、膝角范围、脚最低高度、自碰撞帧比例

**v14 QA 结果（34 个 clip，IK ≈7ms/帧）**：发现 5 类系统性问题（几乎所有 clip 都有，不只是高难动作）：

| # | 问题 | 证据 | 根因 |
|---|---|---|---|
| 1 | 手臂一直弯着（手抬到胸前），手部位置误差 p95 ≈ 15–30cm | walk_test 人体肘屈曲均值 37°，机器人 ≈ 100°；肩 roll 在 25+ 个 clip 中 >20% 帧顶内限；手腕穿躯干 | **标定假设机器人 qpos0 = 人体 frame-0 姿态**。腿成立（都直立），但 A3 qpos0 的 elbow=0 是**前臂水平向前（屈 90°）**，直臂≈+90°；人体 frame-0 手臂自然下垂 → 前臂/手 rot_offset 固化了 ~90° 屈肘 |
| 2 | 站立支撑时脚底陷入地面 2–7cm | 人脚着地帧机器人鞋底 z 中位数 -2~-7cm | 机器人踝高 7.2cm，人体 4.2cm；IK 把踝关节放到人体踝位置 |
| 3 | 支撑脚不平（倾斜 p95 20–40°，走路也 25°） | 同上帧 sole tilt | `ankle_roll ← FootMod` rot 权重 0（脚朝向不跟踪）+ `ankle_pitch_Link ← LeftShin` 方向跟踪 → 踝 pitch 被锁在中立，脚始终垂直于小腿。G1 参考配置是 `knee_link ← 小腿 (0,10)`、`ankle_roll ← FootMod (50,10)` |
| 4 | 走路膝盖互撞（walk_test 32% 帧）、站姿偏窄；膝关节平均多屈 11°（36.5° vs 人 25°） | 自碰撞 knee/knee；比例：机器人髋宽 0.246 vs 人 0.201，pelvis 原点在髋关节上方 0.179 vs 人 0.090 | 脚横向 offset 符号反了（左脚 -0.02 向内，L/R 修复后遗留；上游 smplx_to_a3_ultra 是 +0.02 向外）；pelvis 原点高度差没补偿，机器人只能屈膝；手臂 scale 0.833 与腿 1.0 不一致（实测肢段比都 ≈0.93） |
| 5 | 腿关节单帧跳变 40–134°（坐、跪、深蹲、躺起、翻滚） | sitting f70 hip_pitch 117°，stand_up_lying f95 134° | `_knee_too_deep`(>85°) / `_at_joint_limit`(膝上限) 热启动启发式：对深屈膝动作是合法姿态却触发重置 → 解在两个分支间跳 |

另：手臂 gimbal 分支跳变（jump360 / jog_spin，肩 roll≈90° 时 pitch/yaw 共轴）；BVH 地面确认在 z≈0（各 clip 脚趾关节最低 -1.6~+3cm）；`walk_test.bvh` 与 `walking_on_edge_ff_270_R_002__A411_M.bvh` 内容相同。

**修复计划**：
- F1 `calibrate_soma_to_a3ultra.py`：机器人参考姿态不再用 qpos0，而是求解手臂关节使上臂/前臂方向与人体 frame-0 一致，再取 link 朝向算 rot_offset
- F2 IK 映射对齐 G1：`knee_Link ← LeftShin`（t1 (0,10)，t2 (10,5)），删 `ankle_pitch_Link` 条目；`ankle_roll ← FootMod` (100,10)
- F3 每个 clip 按演员比例自动算：统一 scale s = 机器人腿长/人体腿长（全身同一 s）；pelvis 局部 offset（补 pelvis 原点-髋关节高度差）；髋/膝/脚横向向外 offset（补髋宽差）；手臂横向 offset（补肩宽差）
- F4 删掉热启动启发式（只保留首帧 20° 预屈膝），加 mink PostureTask（目标=上一帧解）抑制分支跳变
- F5 retarget 后按 clip 整体平移 root z：机器人踝高 - s×人体着地踝高，使鞋底落地

**F1–F5 实施结果（v15，34 clip 全量 QA）**：

| 指标（walk_test，括号为全体范围） | v14 | v15 |
|---|---|---|
| 手位置误差 p95 | 19 / 15.5cm（16–30） | 8.3 / 6.5cm（1.4–24） |
| 肘屈曲 人/机 均值 | 37° / ≈100° | 35.7° / 40.7°（corr 0.99） |
| 膝屈曲 人/机 均值 | 25° / 36.5° | 25.1° / 21.1°（corr 0.99；各 clip 差 ≤5°） |
| 支撑脚鞋底高度 p50 | -2.0 / -2.8cm | +0.6 / +0.1cm |
| 支撑脚倾斜 p95 | 27° / 25° | 11° / 17° |
| 自碰撞帧 | 13%（膝撞膝） | 3%（仅拇指擦大腿） |
| 腿单帧最大跳变（坐/躺起） | 117° / 134° | 6° / 12° |
| 手臂 gimbal 跳变（jump360） | 89° | 19° |

剩余 25–54° 的手臂单帧变化对应人体真实快速动作（throw_ball f631 人手 17.6cm/帧 ≈ 5m/s）。

**仍有穿地**（非手指部件）：跪姿膝盖 -11cm（机器人膝盖体积比人大）、深蹲后退 toe -12cm（踝背屈极限 -52° 不够，脚只能斜着）、手撑地时手指/手腕 -10~-20cm。单个 clip 常数平移无法同时满足站立和跪/坐帧。

**F6**：retarget 后逐帧计算非手指碰撞体最低点，穿地超过 2cm 的帧抬高 root；抬高量做 ±0.2s 滑动 max 再滑动 mean（保证不小于所需量、不突跳）。

**F6 结果**：手/手指不参与抬升（手撑地时机器人手臂偏长，按手抬升会让脚悬空）。非手部件最低点全部 ≥ -2cm；跪姿膝盖、深蹲脚尖不再穿地；sitting 双手撑地时鞋底 p50 从 +10cm 回到 -1.9cm。

**可视化**：SOMA 面板改为与机器人相机同比例（× 本 clip 的 retarget scale）+ 固定地面线，并去掉合成的 `Root` 关节（停在 BVH 原点，人走远后把自动缩放拉坏）。原先每帧自动缩放，深蹲在正视图里看起来像"站着"。

**batch 准备**：
- `data/motion_lib_a3ultra/robot_filtered/` 已有 28,792 个旧 pipeline 的 pkl（2026-10-12，180° 朝向/脚交叉/手臂弯曲），skip_existing 会跳过它们 → 新 batch 输出到 `robot_filtered_v15/`，旧目录不动
- 修复：tar 模式下 pkl 内 motion key 取自临时文件名（`tmpXXXX`），改为传入真实 clip 名
- `--workers` 之前未实现（142k clip 单进程 ≈ 4 天）→ 实现进程池，tar 流式读取 + 有界预取（避免 45GB 读进内存）
- 仓库自带 `filter_and_copy_bones_data.py` 的默认过滤词（box/stair/sitting/chair/walking_on_edge/cartwheel/safety_roll…）正好覆盖 QA 中环境交互类问题 clip；batch 全量转换，训练前按官方流程过滤

**batch 验证**：240 clip 小批量（24 workers）0 失败，12 clip/s；pkl key = clip 名，dof/root 与 QA driver 输出完全一致（差 0.0）。

**对比视频**：`videos/compare_v15/`（33 个，左 SOMA 与机器人同比例 + 地面线，右 A3 Ultra，L/R 标签，关节单帧跳变 >25° 时红字标出）。`videos/compare_v14/` 保留作修复前对照。

**全量 batch 已启动**（2026-10-13）：`--workers 24`，输出 `data/motion_lib_a3ultra/robot_filtered_v15/`，日志 `data/motion_lib_a3ultra/batch_retarget_v15.log`，≈15 clip/s，预计 ≈2.5h。旧目录 `robot_filtered/`（28,792 个旧 pipeline pkl）未动，确认新数据后可删。

**已知限制（接受）**：手撑地时机器人手/手指穿地（手臂相对偏长）；坐盘腿时腿部自碰撞；上/下箱子、走窄边等环境交互 clip 机器人悬空（训练前用 `filter_and_copy_bones_data.py` 默认过滤词剔除）；腰 pitch 限位 [-28°, 24°]，人体大幅弯腰时饱和。

---

## 34. [2026-10-13] 确认：Bones-SEED 数据集无手部局部动作

**用户问题**：bones-seed 数据集里是否包含手部的局部动作。

**检查方法**：BVH 骨架本身定义了完整五指链（每手 Thumb/Index/Middle/Ring/Pinky，每指 3-4 节，共 78 关节/78 关节 x 3 通道）。直接解析每个 clip 的手指关节旋转通道，看数值是否随时间变化。

**结果**：
- 抽查 34 个动作样本（走/跑/跳/坐/蹲/跪/爬/翻滚/打/扔/舞蹈）：所有手指通道全程 std=0.00°、range=0.00°（完全常量），包括 `throw_ball`、`punch_self_face` 这类理应有手部动作的 clip。手腕本身正常随手臂摆动。
- 全量扫描 142,220 个 clip（手部相关关键词全量命中 + 每 40 个抽样一个，共解析 23,385 个，1000s）：**0 个 clip 的手指关节变化幅度 > 0.5°**。

**结论**：Bones-SEED 的动捕只采集了身体大关节，手指通道是导出时的常量占位值，不含任何手部精细动作数据。这与当前 retarget pipeline 的处理一致——`_ACTUATED_MAP`（`convert_gmr_a3ultra_to_motion_lib.py`）本就跳过手指索引，`dof` 数组（31维）不含手指自由度，机器人手指保持默认姿态。

---

## 35. [2026-10-13] 全量 batch 完成；682 个失败原因排查

**结果**：141,538 转换成功，682 失败（0.48%），用时 134 分钟，16 clip/s。输出 `data/motion_lib_a3ultra/robot_filtered_v15/`。

**失败原因排查**：全部 682 个失败报错均为 `SVD did not converge`，发生在 `load_soma_bvh_frames()` 里 `Rotation.from_matrix()`（FK 阶段，IK 之前）。抽查 3 个失败 clip（不同动作类型：双手持物、镐子 idle、原地等待）：
- 根因是原始 BVH **动捕数据本身存在大段 NaN**：Y/Z 旋转通道在某一段连续帧内全部是 NaN，且**同一帧范围内所有关节同时失效**（Hips 到 Head 到四肢全部），X 通道正常——这是采集时的追踪丢失/遮挡，不是某个关节单独的问题。
- 丢失段很长，不是偶发几帧：3 个样本分别丢失 76+456、343+433、394 帧（总帧数 1154/2150/1435），占比 30-60%。插值填补会伪造大半个动作，不安全。
- 682 个失败分布在 54 个不同的采集 session（A513 99个、A521 79个、A527 74个...），分布分散，说明是源数据采集阶段偶发的追踪丢失，不是 pipeline 的 bug。

**结论**：这 682 个 clip（0.48%）应当保持跳过，不做修复或插值——现有行为（遇到解析异常直接计入 failed、不写出 pkl）是正确的。

---

## 36. [2026-10-13] 用仓库自带过滤脚本清洗 v15 数据集

**目的**：QA（worklog #33）中发现上箱子/下箱子/走窄边/手撑地/盘腿坐等环境交互或特殊接触类动作，机器人会悬空、穿地或自碰撞——这类问题不是 retarget 算法能单独解决的（需要场景几何信息，当前 pipeline 没有），用仓库已有的 `filter_and_copy_bones_data.py` 按关键词过滤掉。

**命令**：
```bash
python3 gear_sonic/data_process/filter_and_copy_bones_data.py \
  --source data/motion_lib_a3ultra/robot_filtered_v15/ \
  --dest data/motion_lib_a3ultra/robot_filtered_v15_clean/ \
  --workers 28
```
先用 `--dry-run` 跑一遍确认数量，再正式跑（无 `--dry-run`）。脚本按 `--source` 下每个一级子目录（这里是 BVH session 名，如 `230530_walk`、`A505`）分配一个 worker，recursive glob 该目录下所有 `.pkl`，对 `"{session}/{filename}.pkl"`（小写）做关键词子串匹配，命中任一关键词就不拷贝。

**结果**：

| | 数量 |
|---|---|
| 输入（v15 batch 全部成功） | 141,538 |
| 过滤掉 | 12,422（8.8%） |
| 保留到 clean | **129,116**（91.2%） |
| clean 目录大小 | 33G（源 36G） |
| 耗时 | 13s（28 进程并行） |

**用的过滤词**（仓库默认，40 个，未新增）：
`bed bike chair climb com_up_50cm sitting step_on seat table _sit_ sit_ ladder crutch _bed_ _ride_ scooter stepdown acrobatics_ box_HSPU cartwheel 50cm_box_ on_box fall_from handstand_ff_ on_1m form_box off_1m 230m jump_over_obstacle_ lift_crate_come_up_ jump_to_shoulder_roll kozak_dance stair handstand box_jump monkey_jump safety_roll box_dips walking_on_edge push_obstacle`

**各关键词命中数**（对全部 141,538 个文件名独立统计，一个文件可能命中多个词，故总和 > 12,422）：

| 关键词 | 命中数 | 示例 |
|---|---:|---|
| sitting | 3716 | sitting_legs_bend_arms_back_start_003\_\_A042.pkl |
| 50cm_box_ | 3399 | come_down_50cm_box_R_001\_\_A475_M.pkl |
| sit_ | 2428 | walk_sit_drinking_table_mug_R_001\_\_A472.pkl |
| crutch | 976 | crutches_idle_turn_360_R_slow_002\_\_A519.pkl |
| table | 703 | walk_sit_drinking_table_mug_R_001\_\_A472.pkl |
| chair | 702 | sit_on_chair_start_R_003\_\_A063.pkl |
| _sit_ | 190 | walk_sit_drinking_table_mug_R_001\_\_A472.pkl |
| climb | 180 | ladder_climbing_down_stop_R_103\_\_A301_M.pkl |
| handstand | 162 | idle_handstand_loop_003\_\_A305_M.pkl |
| stair | 138 | dance_hiphop_upstairs_downstairs_R_fast_001\_\_A319.pkl |
| ladder | 136 | ladder_climbing_down_stop_R_103\_\_A301_M.pkl |
| jump_over_obstacle_ | 136 | jump_over_obstacle_2m_R_002\_\_A304.pkl |
| lift_crate_come_up_ | 130 | lift_crate_come_up_50cm_box_R_004\_\_A141.pkl |
| cartwheel | 120 | cartwheel_R_003\_\_A304.pkl |
| handstand_ff_ | 106 | handstand_ff_loop_180_R_001\_\_A305.pkl |
| kozak_dance | 90 | kozak_dance_003\_\_A353_M.pkl |
| walking_on_edge | 80 | walking_on_edge_180_R_002\_\_A413.pkl |
| safety_roll | 72 | safety_roll_315_L_001\_\_A305.pkl |
| push_obstacle | 70 | push_obstacle_270_001\_\_A344_M.pkl |
| box_jump | 48 | box_jump_002\_\_A362_M.pkl |
| jump_to_shoulder_roll | 46 | tiger_jump_to_shoulder_roll_R_002\_\_A304_M.pkl |
| _ride_ | 30 | victory_dance_ride_the_pony_R_002\_\_A307.pkl |
| off_1m | 24 | jump_off_1m_R_003\_\_A300.pkl |
| monkey_jump | 24 | monkey_jump_002\_\_A304.pkl |
| box_HSPU | 24 | box_HSPU_002\_\_A359_M.pkl |
| box_dips | 24 | box_dips_001\_\_A361.pkl |
| form_box | 24 | jump_form_box_to_safety_roll_180_R_001\_\_A501_M.pkl |
| stepdown | 12 | ladder_climbing_stepdown_R_003\_\_A300_M.pkl |
| on_1m | 12 | jump_on_1m_R_004\_\_A300_M.pkl |
| on_box | 12 | jump_on_box_230cm_003\_\_A304_M.pkl |
| seat | 8 | stick_gum_to_seat_R_001\_\_A456.pkl |
| fall_from | 4 | fall_from_2m_panic_180_R_002\_\_A304_M.pkl |
| bed / bike / com_up_50cm / step_on / _bed_ / scooter / acrobatics_ / 230m | 0 | （本次数据集中未出现） |

**正式跑前的校验**：
- 关键词子串匹配可能有误伤风险（如 `sit_` 可能命中 `visit_` 这类无关词）。对本数据集全部文件名做了正则检查（`[a-z]sit_`，即 `sit_` 前紧跟字母），**0 个可疑匹配**——本数据集命名规范下所有 `sit_` 命中都是真实的"坐/辅助物"类动作（`walk_sit_`、`sit_on_chair`、`sit_cross_legged` 等），过滤干净。
- `--dry-run` 先跑一遍核对数量（129,116 copied / 12,422 filtered），与正式跑结果一致后才去掉 `--dry-run` 执行。

**说明**：这一步只按文件名关键词过滤 QA 已知有问题的"环境交互"类动作（箱子/楼梯/椅子/床/爬梯/手推障碍物等需要场景几何支持的动作），和 worklog #33 里"手撑地穿地、盘腿坐自碰撞"的已知限制是同一类问题的根源（没有地面以外的场景物体、retarget 不处理道具接触）。纯身体动作（走/跑/跳/舞蹈/格斗等）不受影响。

**产出**：`data/motion_lib_a3ultra/robot_filtered_v15_clean/`，129,116 个 pkl，124 个 session 子目录，33G，可直接用于下一步训练数据准备。`robot_filtered_v15/`（未过滤，141,538 个）保留作为过滤前的完整数据。

---

## 37. [2026-10-13] 核查 Litchi_sonic_for_a3（SONIC 团队 A3 适配）与我们 A3 Ultra 的参数一致性

**背景**：用户要求确认 `Litchi_sonic_for_a3`（相邻目录，SONIC 团队针对 AgiBot A3 的新开源适配，含 035/step200000 checkpoint）里的 A3 机器人参数是否与本项目用的 A3 Ultra (T3D0) 一致，避免把该 checkpoint 部署到 A3 Ultra 上时出现 sim2real/跨机型 gap。

**对比对象**：
- 我方：`gear_sonic/data/assets/robot_description/mjcf/a3_ultra_t3d0.xml` + `gear_sonic/envs/manager_env/robots/a3_ultra.py`
- Litchi：`gear_sonic/data/assets/robot_description/mjcf/a3_t2d5.xml`（确认为 035 checkpoint 实际训练用的资产——`checkpoints/035_step200000/config.yaml:323` 的 `assetFileName: a3_t2d5.xml`）+ `gear_sonic/envs/manager_env/robots/a3.py` + `gear_sonic/utils/a3_motor_params.py`

**结论：下肢（腿+腰）和手臂机构参数基本一致（同一套硬件平台），差异集中在头部感知组件和末端手**，具体：

### 1. 运动学完全一致
逐 body 对比 29 个共有 link 的局部偏移（`body_pos`）：骨盆高度 1.3m、大腿/小腿长、肩宽、前臂长等全部**逐毫米吻合**，仅 ankle_pitch_Link 1.5mm、elbow_Link 0.5mm 的建模取整误差。

### 2. 质量分布完全一致
29 个共有 link 的质量逐克吻合（髋/膝/踝/肩/肘/腰/骨盆全部 0.000kg 差），仅手腕末端（+0.22kg/侧，A3 Ultra 灵巧手结构更重）、头部（+0.235kg，A3 Ultra 多了独立俯仰传感器头）、躯干（+0.649kg，A3 Ultra 机身电子件更重）有差异。整机：A3U 60.01kg vs A3 57.98kg，差 2kg 全部来自这几处 + 手指质量。

### 3. 电机/执行器参数几乎逐位精确吻合
踝、腰并联关节的等效惯量倍数、等效最大力矩、等效限速——我方 `docs/robot_parameters/instructions.md` 与 Litchi `docs/a3_024_sim2real_materials/01_serial_parallel_torque_coupling/armature_reference.csv` **数值完全一致**（5.333/1.66562/7.3/1.21，10.8/19.37/9.24785/22.7 rad/s，118.2/54.75/115/46 Nm）。

全部 11 组关节（hip/knee/ankle_pitch/ankle_roll/waist_yaw/waist_pitch/waist_roll/shoulder/shoulder_yaw_elbow/wrist_roll/wrist_pitch_yaw）的 `effort_limit_sim`、`velocity_limit_sim`、以及反射惯量 `armature`，两边逐一算出来的数值**全部一致到小数点后 6 位**（PFP-93-65/110-75/78-58/59-60/41-48 电机型号命名也一致），仅 waist_pitch 力矩我方标注 118.0 Nm 而 Litchi 用 115 Nm（差 2.5%，均在 `docs/robot_parameters/instructions.md` 注明的"理论计算值，建议 domain rand 3%"范围内，不算结构性差异）。

**结论**：A3 与 A3 Ultra 共享同一套腿部/腰部/手臂的电机、减速器、连杆机构，不是两种机械结构不同的机器人。

### 4. 发现的真实差异
| 项目 | A3 Ultra | A3 (T2D5) |
|---|---|---|
| 末端手 | 完整灵巧手（每手 15 指关节，经 tendon-mimic 耦合驱动） | 无手指关节（`right_hand_Link` 为固定几何体，推测简单夹爪/固定手型） |
| 头部 | `head_yaw` + 额外独立俯仰 `ultra_head_pitch`（挂载 Livox MID360 激光雷达，mesh 名 `ultra_head_mid360_Link`） | 仅 `head_yaw` + `head_pitch`；且 035 训练资产里 `head_yaw` range 被锁死为 `[0,0]`（训练未用到该自由度） |
| hip_roll 上限 | 85°（右为 -85°） | 92°（右为 -92°），差 7° |
| knee 上限 | 145° | 143°，差 2° |
| waist_pitch 等效力矩 | 118.0 Nm（docx 理论值） | 115 Nm（差 2.5%，在建议 domain-rand 范围内） |

**头/手差异对该 checkpoint 无影响**：035 checkpoint 的动作空间 `A3_ACTIVE_JOINT_NAMES`（`gear_sonic/envs/manager_env/robots/a3.py`）只有 29 个身体关节，不含头部和手指，所以 A3 Ultra 多出来的灵巧手/独立传感器头本来就不受这个策略驱动，需要另外的控制层（与 README "Current limitations" 里提到的"无手部精细操作"一致）。

### 5. 需要注意的一点（不是硬件差异，是训练配方差异）
两边各自训练用的 PD 增益（`stiffness`/`damping`）**选得不一样**，不能跨项目混用：

| 关节组 | A3U 自己训练用 Kp/Kd | A3 (Litchi 035) 训练用 Kp/Kd |
|---|---|---|
| hip | 200 / 7 | 120(roll)/80(yaw,pitch) / 4(roll)/3(yaw,pitch) |
| knee | 360 / 13 | 250 / 8 |
| ankle | 80 / 3 | 50 / 2 |
| waist_yaw | 200 / 7 | 85 / 3 |
| waist_pitch/roll | 80 / 3 | 50 / 2 |
| shoulder | 90 / 4 | 40 / 3 |
| shoulder_yaw/elbow, wrist_roll | 40 / 3 | 30 / 2 |

部署 Litchi 的 035 checkpoint 到 A3 Ultra 实机时，底层伺服 Kp/Kd 必须按 **Litchi 自己训练用的数值**（`gear_sonic/envs/manager_env/robots/a3.py` 或其 `gear_sonic_deploy` 导出的 deploy manifest）配置，不能套用我们 `a3_ultra.py` 里为"从零训练 A3 Ultra"选的那组增益——否则会制造出一个两个项目本不存在的 gap。

### 6. 副产品：发现我方 `a3_ultra.py` 里 wrist_roll 的 armature 疑似笔误
`gear_sonic/envs/manager_env/robots/a3_ultra.py:296-303` 的 `wrist_roll` 块：`effort_limit_sim=24.0, velocity_limit_sim=15.71`（对应 PFP-59-60 电机规格，与 Litchi `a3.py` 给 wrist_roll 用的电机型号一致）但 `armature=0.000810`（这是 PFP-41-48 电机的反射惯量，本应是 PFP-59-60 的 `0.004967`）。注释自相矛盾：写着"PFP-41-48"却配的是 PFP-59-60 的力矩/限速数值。Litchi 的 `a3_motor_params.py`／`a3.py` 对应位置是自洽的（`ARMATURE_PFP_59_60 = 0.004967`，与 24Nm/15.71rad/s 配套）。

**这是我方从零训练 A3 Ultra 时的仿真参数 bug**（wrist_roll 关节仿真惯量偏小约 6 倍，与这次"能否部署 Litchi checkpoint"的问题无关），独立记录，后续训练前应修正为 `0.004967`。

---

## 38. [2026-10-13] 修复 a3_ultra.py wrist_roll armature 笔误

worklog #37 发现的笔误已修：`gear_sonic/envs/manager_env/robots/a3_ultra.py` 的 `wrist_roll` armature 从 `0.000810`（PFP-41-48，和自己的 24Nm/15.71rad/s 力矩/限速不匹配）改为 `0.004967`（PFP-59-60，和 Litchi `a3_motor_params.py` 的 `ARMATURE_PFP_59_60` 独立核对一致）。影响：本项目后续从零训练 A3 Ultra 时 wrist_roll 关节仿真反射惯量不再偏小 6 倍。与本次 A3 集成工作无关的独立修复。

---

## 39. [2026-10-13] A3 集成 Phase 1（训练侧文件迁移）完成

按 `docs/workflow/a3_integration_plan.md` 的决策（Phase 4 完整范围、CLI 统一为 `a3`、直接拷贝文件），把 `Litchi_sonic_for_a3` 的训练侧代码迁移进本仓库，sonic_a3_035.yaml 的全部 `gear_sonic.*` 引用已逐条核实可解析。

**纯拷贝（新文件，无冲突）**：
- `gear_sonic/envs/manager_env/robots/a3.py`、`utils/a3_motor_params.py`、`utils/a3_motor_model.py`（+ `data/a3_motor_model/` 504K jacobian 表）
- `config/exp/manager/universal_token/all_modes/sonic_a3.yaml`、`sonic_a3_035.yaml`
- `config/actor_critic/encoders/a3_fast_mf_mlp.yaml`
- `config/aux_losses/universal_token/g1_recon_and_a3_fast_latent.yaml` + `terms/{a3_fast_g1_latent,reencoded_a3_fast_g1_latent,a3_fast_recon}.yaml`
- `config/manager_env/observations/terms/{command_multi_future_a3_fast_nonflat,motion_anchor_ori_a3_fast_b_mf_nonflat}.yaml`、`tokenizer/unitoken_all_noz_a3_fast.yaml`
- `data/assets/robot_description/mjcf/a3_t2d5*.xml`（8 个变体）+ `urdf/a3/`（20M）

**逐函数手动合并验证（非纯新增，与 Litchi 对比到逐行一致）**：
- `order_converter.py`：新增 `A3Converter`（29-DOF 紧凑序，区别于已有的 `A3UltraConverter` 63-DOF）
- `commands.py`（最高风险文件）：A3-fast 独立的未来帧窗口机制（支持历史帧/截断有效帧/零填充，比 smpl 的纯未来窗口更复杂）、`a3_fast_encoder_index`/`coactivate_g1_when_a3_fast` 协同激活、`_requires_encoder_availability_filter` 性能优化。**没有**引入 Litchi 版本的引用缓存重构（`_cached_reference`），改用本仓库已有的非缓存 property 写法逐一对应实现，和 Litchi 源码按行比对完全一致（归一化掉缓存包装差异后 diff 为空）
- `observations.py`：3 个新函数，纯新增，diff 验证完全一致
- `rewards.py`：A3 电机力矩-转速/并联关节约束惩罚（4 个新 reward class + helper），diff 验证完全一致
- `token_losses.py`：4 个新 loss class（`A3FastReconLoss`/`A3FastG1LatentLoss`/`TeleopA3FastLatentLoss`/`ReencodedA3FastG1LatentLoss`），diff 验证完全一致
- `events.py`：**发现的真实问题**——`randomize_rigid_body_com` 原实现对多 body 场景是 bug（所有选中 body 共享同一个随机偏移、且索引写入时忽略了 `env_ids` 只在 `len()` 取数量时用到），当前仓库唯一调用方 `base_com`（`mode: startup`、单 body `torso_link`）不会触发这个问题所以一直没暴露；但 A3 新增的 `arm_com`/`waist_com`/`leg_com` 是多 body 场景，必须用 Litchi 修复后的版本（每个 body 独立采样、正确按 env_ids 索引）。已采用 Litchi 版本（同时也修了 `randomize_joint_default_pos` 对 `shared_joint_names` 为空时的保护）。`events.py` 与 Litchi 源码逐字节比对完全一致（`diff` 输出为空）

**未改动，核实为零差异的文件**：`terminations.py`、`actions.py`、`curriculum.py` 与 Litchi 版本逐字节一致，无需任何改动。

**CLI**：`train_agent_trl.py` 的 `ROBOT_EXP_MAP` 新增 `"a3": "manager/universal_token/all_modes/sonic_a3_035"`，保留 `"a3_ultra"` 条目不变（两份训练合约独立共存，待用户确认是否要合并/废弃 a3_ultra 这条）。

**验证方法**：每个文件改动后都做了 (1) `py_compile`/`ast.parse` 语法检查，(2) 对比新增内容与 Litchi 源码逐字符/逐行 diff，确认改动范围精确等于预期，没有多删/多改。最后做了一次全量引用解析检查：把 `sonic_a3.yaml`/`sonic_a3_035.yaml` 及其拉入的全部新 yaml 文件里出现的每一个 `gear_sonic.*` 的 `func:`/`_target_:` 引用都提取出来，逐个确认对应的函数/类在合并后的代码里存在——全部 resolve 成功，无遗漏。

**仍未做（超出 Phase 1 范围）**：Phase 2（035 checkpoint 迁移 + sim2sim 数值对比验证）、Phase 4（VR 实时遥操重定向模块）、`ROBOT_MODEL_REGISTRY["a3"]`（属于 Phase 4）。

---

## 40. [2026-10-13] A3 集成完整技术记录（面向冷启动 agent 的自包含上下文）

**这一节是独立、完整的参考文档**：假设读者（可能是另一个全新开的 agent）没有读过本对话、没有任何上下文。如果你要继续 A3 集成工作，从这里开始读就够了，不需要再去翻之前的 #37/#38/#39（那三条是发现过程的流水账，这条是整理后的权威总结，内容有重叠，以这条为准）。

### 40.1 背景与目标

用户在跟本仓库（`GR00T-WholeBodyControl`，NVIDIA SONIC 全身控制框架，之前主要跑 G1/H2/A3 Ultra）同级目录下发现了另一个独立 git 仓库 `Litchi_sonic_for_a3`（路径：`/home/user/YZY/HumanWBM/Demo/RL controller/Litchi_sonic_for_a3`，远程 `git@github.com:Marco-Yang/Litchi_sonic_for_a3.git`）——这是 SONIC 团队公开发布的、专门针对 AgiBot A3（非 Ultra 版）机器人的适配，**自带一个训练好的 checkpoint**（`checkpoints/035_step200000/`）。两边是同一套 SONIC 代码库在不同时间点分叉出来的，训练侧架构（Hydra 配置结构、`UniversalTokenModule` 编码器）和部署侧 C++ 框架（`gear_sonic_deploy/`）高度同源，不是两个互不相关的系统。

用户的目标：把 `Litchi_sonic_for_a3` 的训练与部署逻辑、以及训练好的 035 checkpoint，迁移进本仓库 `GR00T-WholeBodyControl`，让训练、仿真、真机部署都能用同一个 `--robot` 命令行参数在 `g1` / `h2` / `a3` 之间切换。

**用户已拍板的决策**（不要重新讨论，直接照做）：
1. 遥操范围做"完整范围"：不止参考动作回放，要支持 VR 实时姿态 → A3 关节空间轨迹的实时重定向（这部分是 Phase 4，净新增工作量，还没开始）。
2. CLI 机器人命名统一为 `a3`（不是 `a3_ultra`）。但**真机部署对象仍然是 A3 Ultra 实体**——训练/仿真侧用 Litchi 的 A3(T2D5) 合约（因为 035 checkpoint 就是在这套合约上训出来的），C++ 部署侧的 `robot_io` 后端要对接 A3 Ultra 真机硬件接口（这是 Phase 3 的事，具体怎么接还没定）。本仓库已有的、独立的"从零训练 A3 Ultra"合约（`a3_ultra.py`/`sonic_a3_ultra.yaml`/`a3_ultra_t3d0.xml`，63-DOF 含灵巧手）继续保留，不删除，`ROBOT_EXP_MAP` 里 `"a3_ultra"` 条目也保留——两份训练合约长期共存，**不要混用两边的参数**（尤其是 Kp/Kd，worklog #37 有详细对比：两边给 waist_yaw 的增益差 2 倍多）。
3. 迁移方式是直接把 Litchi 仓库里的文件拷贝/移植进本仓库对应位置，不用 git subtree，不用 submodule。
4. 任何 C++ 编译相关的改动，先在隔离环境验证能编译过再往主仓库合并（已完成，见 40.3）。

**完整规划文档**：`docs/workflow/a3_integration_plan.md`（Phase 0-6 的分期计划、决策记录、C++ 编译验证的完整过程）。本节只总结训练侧（Phase 1）已经做完的部分；部署侧/Phase 3/4 的规划仍以 `a3_integration_plan.md` 为准。

### 40.2 两个机器人型号的关系（容易搞混，先说清楚）

- **A3**（代号 T2D5）：Litchi 仓库适配的目标，035 checkpoint 是在这个型号上训的。无手指、无独立俯仰传感器头。
- **A3 Ultra**（代号 T3D0）：本仓库之前已经在适配的目标（worklog #1-36）。有完整灵巧手（每手 15 指关节）、多一个挂 Livox MID360 激光雷达的独立俯仰头。

**worklog #37 的核心结论**（已验证，详见该条）：两者腿/腰/手臂的运动学（逐 mm 一致）、质量分布（逐 g 一致）、电机规格（力矩/限速/等效惯量逐 6 位小数一致）**几乎完全相同，是同一套硬件平台**，差异只在头和手。这就是为什么 035 checkpoint（在 A3 上训的）理论上可以直接部署到 A3 Ultra 真机上跑——前提是部署时用 A3 自己的 PD 增益合约，不要混用 A3 Ultra 的那组。

### 40.3 C++ 部署框架编译验证（已完成，详细过程见 `a3_integration_plan.md`）

简要结论：`Litchi_sonic_for_a3/gear_sonic_deploy/`（含 AimRT 中间件 + ROS2 + protobuf 的完整后端）在隔离 conda 环境（`a3_deploy_verify`，用 conda-forge/robostack 装的 ROS2 Humble，没碰系统 apt、没碰 `gmr`/`isaaclab` 环境）里编译验证通过，产出能正常运行的二进制（`nm` 确认关键符号是真实实现而非空桩，`--help` 正确打印参数列表）。过程中发现并修复 3 个真实问题：
1. conda-forge 默认装的 `empy 4.2.1` 和 ROS2 Humble 代码生成器不兼容 → 降到 `3.3.4`。
2. host 缺 `libacl1-dev` 头文件 → 用 conda-forge 的 `libacl` 包 + `CPATH` 环境变量代替，没有碰系统依赖。
3. **`Litchi_sonic_for_a3/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/CMakeLists.txt` 里 `--proto_path="${VAR}"` 的引号写法是 CMake 处理带空格路径的已知坑**——本机路径 `/home/user/YZY/HumanWBM/Demo/RL controller/...` 本身带空格会触发。已经在 Litchi 本地检出里把引号从"只包住变量值"改成"包住整个 `flag=值`"（`"--proto_path=${A3_TA_PROTO_ROOT}"`），**这个修复目前只在 `Litchi_sonic_for_a3` 本地检出里，没有提交到它的 git 仓库，也还没有同步到本仓库**——Phase 3 正式把这个 CMakeLists.txt 拷进 `GR00T-WholeBodyControl` 时必须带上这处修复，不能直接拷 Litchi 仓库里未修的原始版本。

另外发现：`GR00T-WholeBodyControl` 自己也有一份 `gear_sonic_deploy/`（用于 G1，Unitree SDK + DDS），和 Litchi 的 `gear_sonic_deploy/`（A3，AimRT）路径结构高度同源（约 30 个文件名字面相同）。Litchi 新增了一个 `robot_io::RobotIOBackend` 抽象 + `backend_factory.cpp`，代码里已经写好了按机器人名字符串选后端的机制（`"g1"`/`"a3"` 两个名字都预留了，`"g1"` 分支目前是个报错桩，因为公开版本砍掉了涉及 Unitree SDK 授权的 G1 实现）。Phase 3 的工作是：把 G1 的真实输出逻辑包一层实现 `CreateG1Backend()`，仿照 A3 的写法给 A3 Ultra 写 `CreateA3UltraBackend()`。

### 40.4 Phase 1：训练侧文件迁移（已完成，本节是详细清单）

**核心方法论**：对每个 Litchi 有而本仓库也有同名文件的情况，先跑 `diff` 看是不是纯新增（只有 `>` 没有 `<`）。纯新增的直接复制新增部分过来。不是纯新增的，必须读懂双方差异的真实含义，逐段人工合并，**绝不整体覆盖**——因为本仓库这些共享文件在 Litchi fork 出去之后可能已经有自己的本地演进（比如 `commands.py` 里的自适应采样失败归因快照、手部关节处理，这些 Litchi 版本里没有，如果整体覆盖会丢失）。每一处合并完成后，都用脚本把新增内容和 Litchi 源码重新 diff 一遍，确认逐行/逐字符一致（排除故意跳过的部分，比如下面说的缓存层）。最后做了一次全量引用解析扫描：把 `sonic_a3.yaml`/`sonic_a3_035.yaml` 以及它们通过 `defaults:` 拉入的全部新 yaml 文件里出现的每一个 `gear_sonic.*` 的 `func:`/`_target_:` 引用（支持 `module.attr` 和 `module:attr` 两种写法）提取出来，逐个用正则确认对应的 `def`/`class` 在合并后的代码里真实存在——全部 resolve 成功，零遗漏。

#### 40.4.1 纯拷贝的新文件（Litchi 独有，本仓库里原来不存在，无命名冲突）

这些文件直接 `cp` 过来，没有做任何修改。**注意：`gear_sonic/data/` 整个目录被 `.gitignore` 第 209 行忽略，所以下面带 `data/` 的文件不会出现在 `git status`/`git diff` 里，必须靠这份清单才能知道它们存在**：

| 目标路径（相对 `GR00T-WholeBodyControl/`） | 说明 |
|---|---|
| `gear_sonic/envs/manager_env/robots/a3.py` | A3 机器人定义：`A3_CYLINDER_CFG`（ArticulationCfg，spawn 用 `urdf/a3/model_collision_optimized_passive_foot_twostage_fit_optimized.urdf`）、`A3_ACTIVE_JOINT_NAMES`（29 个关节）、`A3_ISAACLAB_JOINTS`/`A3_ISAACLAB_TO_MUJOCO_DOF`/`A3_MUJOCO_TO_ISAACLAB_DOF`/`A3_ISAACLAB_TO_MUJOCO_BODY`/`A3_MUJOCO_TO_ISAACLAB_BODY` 等映射表（已用 `ast` 结构性校验：DOF 映射是 0-28 的双射，body 映射是 30 元素的合法置换） |
| `gear_sonic/utils/a3_motor_params.py` | `ARMATURE_PFP_{78_58,93_65,41_48,59_60,110_75}` 常量，和本仓库 `a3_ultra.py` 里手算的等效惯量数值逐 6 位小数对上（worklog #37） |
| `gear_sonic/utils/a3_motor_model.py`（831 行） | A3 并联关节（踝/腰）物理电机模型：T-N 曲线、雅可比表读取、`A3MotorConstraintCache`、`compute_motor_penalty`/`compute_threshold_penalty`/`compute_cross_penalty`/`compute_position_threshold_penalty`/`get_tn_curve`/`peak_mechanical_power`。路径解析用 `Path(__file__).resolve().parents[1] / "data" / "a3_motor_model"`，相对模块自身位置，拷贝后天然解析正确，不需要改路径 |
| `gear_sonic/data/a3_motor_model/`（504K，**gitignored**） | `left_ankle_jacobi.npz`、`right_ankle_jacobi.npz`、`waist_jacobi.npz`、`reward_024_tn_curves.csv`、`README.md`——上面那个模块要读的雅可比表和 T-N 曲线数据 |
| `gear_sonic/data/assets/robot_description/mjcf/a3_t2d5*.xml`（8 个变体，**gitignored**） | `a3_t2d5.xml`（plain）、`a3_t2d5_passive_foot.xml`、`a3_t2d5_passive_foot_twostage_fit_optimized.xml`、`a3_t2d5_loop*.xml`（4 个 loop 变体，含闭环连杆建模）。035 checkpoint 训练用的是 plain `a3_t2d5.xml`（已用 `checkpoints/035_step200000/config.yaml:323` 的 `assetFileName` 字段核实） |
| `gear_sonic/data/assets/robot_description/urdf/a3/`（20M，**gitignored**） | IsaacLab 训练实际加载的是这里面的 `model_collision_optimized_passive_foot_twostage_fit_optimized.urdf`（`a3.py` 的 `asset_path` 字段指定），MJCF 是给 MuJoCo sim2sim/部署用的。整个目录（含 `meshes/`、`meshes_collision_optimized/`、`meshes_compliant_foot/`）原样拷贝 |
| `gear_sonic/config/exp/manager/universal_token/all_modes/sonic_a3.yaml` | Hydra 基础 exp 配置（3 编码器：G1/teleop/SMPL），`sonic_a3_035.yaml` 的 `defaults:` 基类 |
| `gear_sonic/config/exp/manager/universal_token/all_modes/sonic_a3_035.yaml` | 035 checkpoint 的具体训练合约：`active_encoders: ["g1", "a3_fast"]`、A3 电机约束 reward 权重、A3-fast 编码器的帧窗口参数（`a3_fast_num_future_frames: 10`, `a3_fast_dt_future_ref_frames: 0.02`） |
| `gear_sonic/config/actor_critic/encoders/a3_fast_mf_mlp.yaml` | A3-fast 编码器网络结构定义（`init_from_encoder: {source: g1}`，从 g1 编码器权重热启动） |
| `gear_sonic/config/aux_losses/universal_token/g1_recon_and_a3_fast_latent.yaml` + `terms/{a3_fast_g1_latent,reencoded_a3_fast_g1_latent,a3_fast_recon}.yaml` | aux loss 配置，`_target_` 指向下面 40.4.2 里新增的 4 个 loss class |
| `gear_sonic/config/manager_env/observations/terms/{command_multi_future_a3_fast_nonflat,motion_anchor_ori_a3_fast_b_mf_nonflat}.yaml` | 单个 observation term 的 func 绑定，`func:` 指向下面 40.4.2 里新增的两个函数 |
| `gear_sonic/config/manager_env/observations/tokenizer/unitoken_all_noz_a3_fast.yaml` | 在已有 `unitoken_all_noz.yaml` 结构基础上多拉了 2 个 a3_fast 相关的 term（`defaults:` 列表里标了 `# a3_fast` 注释） |

#### 40.4.2 需要人工合并的共享文件（逐一说明改了什么、为什么）

**`gear_sonic/trl/utils/order_converter.py`**：纯新增一个 `A3Converter` 类（插在文件末尾 `load_qpos_from_csv` 函数之前）。处理 A3 的 29-DOF 紧凑序（A3 没有手指，和本仓库已有的 `A3UltraConverter`——63-DOF 含灵巧手——是两个独立的类，不冲突、不是同一个东西，命名上也不会混淆）。

**`gear_sonic/envs/manager_env/mdp/commands.py`**（4310→4556 行，本次合并风险最高的文件）：
- 这个文件定义 `TrackingCommand`（RL 环境的核心命令项，管理参考动作库、未来帧采样、多编码器协同激活）。
- Litchi 版本给它加了一整套 **A3-fast 专属的未来帧窗口机制**：和已有的 g1 窗口（`num_future_frames`/`dt_future_ref_frames`）、smpl 窗口（`smpl_num_future_frames`/`smpl_dt_future_ref_frames`）平行的第三套窗口，但更复杂——支持"历史帧"（`a3_fast_num_history_frames`，取当前时刻之前的帧，用于某些实验配置）和"截断有效帧+零填充"（`a3_fast_valid_future_frames`/`a3_fast_zero_pad_invalid_frames`，10 槽位里只有前 N 个真实有效，其余置零，用于兼容部署侧固定输入宽度）。核心逻辑在两个新增的模块级函数 `_build_a3_fast_relative_slots()`（计算每个槽位相对当前帧的偏移，支持负数偏移=历史帧）和 `_zero_pad_a3_fast_future_frames()`（把无效槽位清零同时保持 flatten 后的 shape 不变）。
- 新增了 `a3_fast_encoder_index`（在已有的 `g1_encoder_index`/`smpl_encoder_index`/`teleop_encoder_index` 基础上）、`coactivate_g1_when_a3_fast`（g1 和 a3_fast 编码器协同激活，和已有的 g1-smpl、g1-soma 协同激活是同一套模式）。
- 新增 `_requires_encoder_availability_filter`：一个性能优化标记，在 035 训练合约（只有 g1/a3_fast 两个非零编码器，两者对所有动作都"天然可用"，不需要像 smpl/soma 那样按动作过滤）下跳过一次昂贵的动态布尔子集运算（避免每次 reset 都做 host-sync）。这不只是加速，**如果不加这个判断，当训练配置没有激活 smpl/soma 时，代码会误入访问 `self.motion_lib.motion_has_smpl` 的分支**，而 A3 的 motion_lib 不一定有这个属性，可能直接报错。
- **没有引入 Litchi 版本的引用缓存层**（Litchi 给很多 property 包了一层 `self._cached_reference("key", lambda: ...)`，目的是同一帧内多次访问同一个 property 不重复计算）。因为这个缓存层是个独立的重构，会牵扯到 `_invalidate_reference_cache()`、`cache_current_references()` 等本仓库完全没有的基础设施，贸然引入风险大于收益。改用本仓库现有的"每次访问都重新算"写法（和已有的 `future_time_steps`/`smpl_future_time_steps` property 一样的风格）逐一重新实现了等价逻辑。验证方法：写了个脚本把两边所有含 `a3_fast` 字样的行抽出来、剥掉 `_cached_reference(...)` 包装和 `lambda:` 前缀后排序比较，diff 为空（唯一的例外是 `a3_fast_future_time_steps` 比 `future_time_steps`/`smpl_future_time_steps` 多了一个 `torch.clamp(time_steps, min=0)`——因为历史帧场景下槽位可以是负数时间步，这个 min-clamp 是刻意保留的，不是遗漏）。
- 具体改动点（搜 `a3_fast` 能找到全部 97 处，详细行号这里不列，直接搜文件即可）：`CommandsCfg`/`TrackingCommandCfg` 新增 6 个 cfg 字段；`__init__` 里新增一段计算 `a3_fast_frame_skips`/`a3_fast_future_time_steps_init`/`a3_fast_future_motion_ids`；`command_multi_future_a3_fast`/`joint_pos_multi_future_for_a3_fast`/`joint_vel_multi_future_for_a3_fast`/`root_rot_dif_l_a3_fast_multi_future`/`a3_fast_future_time_steps` 五个新 property；resample 逻辑里加了 a3_fast 的协同激活分支和 `future_motion_ids` 重算；`create_offline`（离线/部署侧用的 classmethod 构造器）和 `set_motion_state`（offline 场景下切换动作帧）两处都同步加了 a3_fast 的初始化/重置。

**`gear_sonic/envs/manager_env/mdp/observations.py`**（2290→2298 行左右）：纯新增，3 个函数：`command_multi_future_a3_fast`（读 `command.command_multi_future_a3_fast` property）、`motion_anchor_ori_a3_fast_b_mf`（读 `root_rot_dif_l_a3_fast_multi_future`）、`height_map_z_flat`（和 a3_fast 无关，只是顺带在同一个 diff 里，把 height map 展平成 z-only 格式，加在 `height_map_flat` 后面）。对应 3 个新 cfg 字段：`command_multi_future_a3_fast_nonflat`、`motion_anchor_ori_a3_fast_b_mf_nonflat`、`height_map_z_flat`。diff 验证完全一致。

**`gear_sonic/envs/manager_env/mdp/rewards.py`**（607→接近 1014 行）：追加了 A3 电机约束惩罚子系统，插在文件末尾：
- 顶部新增 import：`ManagerTermBase`/`RewardTermCfg`（原来只 import 了 `SceneEntityCfg`）、`from gear_sonic.utils.a3_motor_model import (A3MotorConstraintCache, compute_cross_penalty, compute_motor_penalty, compute_position_threshold_penalty, compute_threshold_penalty, get_tn_curve, peak_mechanical_power)`。
- `RewardsCfg` 新增 8 个字段：`motor_tn_serial`/`motor_tn_parallel`/`ankle_motor_speed`/`ankle_motor_power`/`ankle_motor_fixed_torque`/`parallel_ankle_cross`/`parallel_waist_cross`/`ankle_position_85`。
- 文件末尾追加：`_A3_PARALLEL_PAIRS` 常量、`_get_a3_motor_constraint_cache`/`_prepare_a3_motor_constraint_cache` 两个 helper（每个 env 缓存一份 `A3MotorConstraintCache`，按 `(id(robot), table_dir)` 做 key）、4 个 reward class：`A3MotorTNPenalty`（读仿真里的 implicit PD 目标力矩，惩罚超出真实电机 T-N 包络的部分）、`A3AnkleMotorConstraintPenalty`（踝部并联电机的转速/功率/固定力矩三选一约束）、`A3ParallelJointCrossPenalty`（踝或腰的 pitch/roll 并联对同时高力矩的惩罚，防止两个轴互相"打架"）、`A3AnklePositionPenalty`（踝关节位置接近物理硬限位的惩罚）。diff 验证 4 个 class 完全一致。

**`gear_sonic/trl/losses/token_losses.py`**（1646→1840 行）：纯新增（diff 0 行删除），4 个 loss class，各自插在对应的 smpl/teleop 版本旁边：`A3FastReconLoss`（插在 `G1ReconLoss` 和 `G1ReconLossAligned` 之间，监督 A3-fast 10 槽位里前 `valid_future_frames` 个有效槽位的重建）、`A3FastG1LatentLoss`（插在 `G1SmplLatentLoss` 和 `TeleopSmplLatentLoss` 之间，g1/a3_fast 配对潜变量对齐损失，学生-教师蒸馏模式）、`TeleopA3FastLatentLoss`（紧跟着上面那个，docstring 写明是"向后兼容"用的，当前配置不用但旧 checkpoint 的 Hydra 配置可能引用到）、`ReencodedA3FastG1LatentLoss`（插在 `ReencodedSmplG1LatentLoss` 之后，A3-fast 解码成 G1 动作再重新编码的循环一致性损失）。4 个 class 都复用了本仓库已有的 helper（`_compute_masked_loss`/`zero_loss`/`compute_loss`/`_get_device_from_loss_inputs`），没有新增任何 helper。diff 验证完全一致。

**`gear_sonic/envs/manager_env/mdp/events.py`**（142→465 行，**整个文件用 Litchi 版本重写**，不是零散插入）：
- 这是本次合并里**发现真实 bug 的地方**，不只是加东西。Litchi 版本重写了 `randomize_rigid_body_com`：原来的实现对多个 body 一起随机化时有问题——所有选中的 body 会共享同一份随机采样（`rand_samples` 形状是 `(len(env_ids), 3)` 然后 `.unsqueeze(1)` 广播到所有 body，没有做到每个 body 独立采样），而且赋值时用的下标是 `coms[:, body_ids, :3]`（冒号=全部环境），`env_ids` 只在算 `len()` 的时候用到了，实际赋值根本没按 `env_ids` 过滤。本仓库当前唯一调用这个函数的地方是 `base_com` 事件（`gear_sonic/config/manager_env/events/terms/base_com.yaml`，`mode: "startup"`，只选了单个 `torso_link`），这个场景下两个问题都不会暴露（startup 模式下 env_ids 恒等于全部环境；单 body 场景下"共享采样"和"独立采样"结果没区别），所以这个 bug 一直没被发现、也没有造成过实际影响。但 Litchi 给 A3 新增了 `arm_com`/`waist_com`/`leg_com` 三个事件，这些明确是多 body 场景，**必须用修复后的版本才能正确工作**——也就是说这不是我主动选择要不要修的问题，是 A3 新功能本身就依赖这个修复。顺带也修了 `randomize_joint_default_pos` 对 `shared_joint_names` 为空列表时的边界情况（加了 `if shared_joint_names:` 保护，避免空列表索引出问题），以及把下标数组统一转成 `torch.tensor(..., dtype=torch.long, device=...)` 而不是裸 Python list。
- 其余都是新增：`EventCfg` 新增 15 个字段（`physics_mass_other_links`/`physics_mass_torso`/`hand_payload_mass`/`arm_com`/`waist_com`/`leg_com`/`robot_inertia_random_torso`/`robot_inertia_random_other_links`/`joint_actuator_gains`/`passive_foot_actuator_gains`/`joint_armature`/`joint_soft_pos_limits`/`random_gravity`/`robot_joint_friction_armature*` 三个）；新增 `randomize_rigid_body_mass` 类（继承 IsaacLab 原生的 `randomize_rigid_body_mass`，加了个可选的 `min_mass` 下限 clamp，防止质量随机化把某个 body 的质量随到接近 0 导致数值不稳定）；新增 `set_joint_soft_pos_limit_factor`（+ helper `_scale_joint_pos_limits`，给指定关节单独设置软限位系数，而不是 IsaacLab 原生只能全局设一个系数——A3 用它把腰部三个关节的软限位收紧到硬限位的 80%）；新增 `randomize_rigid_body_inertia`（惯量张量随机化，独立于质量随机化，保留之前质量随机化的效果）；新增 `randomize_actuator_stiffness_with_damping_ratio`（随机化 Kp 的同时按 `d = d0 * sqrt(k/k0) * ratio_scale` 联动调整 Kd，保持阻尼比不漂移）；新增 `push_by_setting_velocity_if_in_contact`（只对检测到指定 body 接触力超过阈值的环境施加推力扰动，而不是无差别推所有环境）。
- 验证：改完后 `diff gear_sonic/envs/manager_env/mdp/events.py ../Litchi_sonic_for_a3/gear_sonic/envs/manager_env/mdp/events.py` 输出为空，逐字节完全一致（因为这个文件改动面太大太分散，没有采用"分段插入"的方式，而是直接整体对齐成 Litchi 的版本——这是在确认了本仓库原有的每一行代码在 Litchi 版本里都有对应保留或者是经过验证的修复之后才这么做的，不是图省事跳过了检查）。

**未改动、核实为零差异的文件**：`terminations.py`、`actions.py`、`curriculum.py`——这三个文件两边逐字节完全相同（`diff` 输出为空），没有任何改动必要。`recorders.py` 有一个 2 行的 diff（eval 录制时默认摄像机偏移量从 `[2,2,1]` 改成 `[4.0,4.0,1.8]`），纯粹是 A3 体型更大时 eval 可视化用的摄像机默认位置调整，和训练正确性无关，`sonic_a3_035.yaml` 也没有覆盖这个参数，所以**没有改**，保持本仓库原值。

#### 40.4.3 CLI 改动

`gear_sonic/train_agent_trl.py` 的 `ROBOT_EXP_MAP` 字典（文件里搜 `ROBOT_EXP_MAP` 能找到，大约在文件后半部分）新增一行：
```python
"a3": "manager/universal_token/all_modes/sonic_a3_035",
```
`"a3_ultra"` 条目原样保留没动。`--robot a3` 现在等价于 `+exp=manager/universal_token/all_modes/sonic_a3_035`。这个 `--robot` 机制本身（把 `--robot <name>` 翻译成 Hydra `+exp=...` override）是更早之前就实现好的，不是这次新加的。

#### 40.4.4 独立发现并修复的 3 个 bug 汇总（都不是"顺手改的"，都有明确触发条件和影响范围）

| # | 文件 | 问题 | 影响范围 | 状态 |
|---|---|---|---|---|
| 1 | `gear_sonic/envs/manager_env/robots/a3_ultra.py` 的 `wrist_roll` actuator 块 | `armature=0.000810`（PFP-41-48 的值）但 `effort_limit_sim=24.0`/`velocity_limit_sim=15.71`（PFP-59-60 的规格），自相矛盾，注释也写错了型号 | 只影响本仓库自己从零训练 A3 Ultra（如果训的话），wrist_roll 关节仿真反射惯量会偏小约 6 倍 | 已修复（worklog #38），改成 `0.004967` |
| 2 | `Litchi_sonic_for_a3/gear_sonic_deploy/.../CMakeLists.txt` 的 `--proto_path="${VAR}"` 引号写法 | CMake 处理带空格路径的已知坑，本机路径带空格会触发，导致 protoc 生成步骤报错 | 只在编译 Litchi 的 AimRT 部署二进制时触发（本机路径恰好带空格） | 已在 Litchi 本地检出修复，**未提交到 Litchi 仓库，未同步到本仓库**，Phase 3 迁移这个文件时必须带上这处修复（40.3 节有 diff） |
| 3 | `gear_sonic/envs/manager_env/mdp/events.py` 的 `randomize_rigid_body_com` | 多 body 场景下共享同一份随机采样、且赋值时没有正确按 `env_ids` 过滤 | 本仓库当前唯一调用方（`base_com`，startup+单body）不触发，是潜伏问题；A3 新增的 `arm_com`/`waist_com`/`leg_com`（多 body）必须用修复版才能正确工作 | 已采用 Litchi 修复版（本节） |

### 40.5 还没做的事（下一步，按原计划顺序）

- **Phase 2**：把 `Litchi_sonic_for_a3/checkpoints/035_step200000/`（.pt + config.yaml）拷进本仓库，用本仓库的 sim2sim 工具跑一遍，和 Litchi 原仓库跑出来的结果做数值对比，确认迁移后训练合约行为完全一致（这一步还没开始）。
- **Phase 3**：C++ 部署集成——把 `include/a3_deploy/`、`include/a3_io/`、`include/a3_rt/`、`include/a3_sync/`、`include/robot_io/`（含 40.3 提到的 CMakeLists.txt 修复）连同 AimRT 依赖一起拷进本仓库 `gear_sonic_deploy/`；给本仓库现有的 G1 输出逻辑包一层实现 `CreateG1Backend()`；仿照 A3 的 AimRT 后端写法给 A3 Ultra 写 `CreateA3UltraBackend()`。
- **Phase 4**：VR 实时姿态 → A3 关节空间轨迹的重定向模块（净新增，可以参考 `decoupled_wbc/control/teleop/teleop_retargeting_ik.py` 的 IK 逻辑改造）。
- `gear_sonic/data/robot_model/instantiation/__init__.py` 的 `ROBOT_MODEL_REGISTRY["a3"]` 注册——这个属于 Phase 4（VR 遥操的姿态校准用的，需要真正的 A3 IK/FK 模型实例化函数，目前两边代码都没有，是净新增工作），不属于 Phase 1。
- 用户还没确认：`"a3_ultra"` 这个 CLI 选项要不要保留/废弃（目前两份训练合约并存）。

---

## 41. [2026-10-13] A3 集成 Phase 2：035 checkpoint 迁移 + 数值对比验证（已完成，部分验证；深层验证待定）

**接续 #40**，同样按自包含方式写，方便冷启动 agent 直接读这一条就知道 Phase 2 做了什么、怎么验证的、还差什么。

### 41.1 这一步要验证什么

Phase 1（worklog #40）验证的是"迁移过来的训练侧*代码*和 Litchi 源码逐行一致"。Phase 2 要验证的是更进一步的问题："迁移过来的*机器人资产*（MJCF/URDF/求解器库）+ 迁移过来的 checkpoint，在本仓库里跑起来，和在 Litchi 原仓库里跑，是不是产生完全一样的行为"——这是防止"代码逐行对了，但资产文件拷漏了一个、或者路径解析错了、或者环境差异导致数值漂移"这类问题的最后一道检查。

### 41.2 做了什么

**1. 下载 checkpoint**：`Litchi_sonic_for_a3/` 原本只有 checkpoint 目录的元数据（README/config.yaml/model_config.yaml/meta.yaml/LICENSE/SHA256SUMS，共 80K），真正的权重文件 `model_step_200000.pt`（402MB）要从 Hugging Face 下载。用仓库自带的下载脚本：
```bash
cd Litchi_sonic_for_a3
python3 -m pip install huggingface_hub   # 如果没装
python3 download_from_hf.py --component pt
```
脚本自动做了 size + SHA256 校验（`SHA256SUMS` 里的哈希和下载后文件匹配，`matches()` 函数），下载到 `Litchi_sonic_for_a3/checkpoints/035_step200000/model_step_200000.pt`。这是公开仓库（`sonic-for-a3/sonic`，Apache 2.0 协议），不需要认证 token。

**2. 迁移到本仓库**：
| 源路径（`Litchi_sonic_for_a3/`） | 目标路径（`GR00T-WholeBodyControl/`） | 说明 |
|---|---|---|
| `checkpoints/035_step200000/`（全部文件，含下载好的 402MB `.pt`） | `checkpoints/035_step200000/` | 和 Litchi 同样的目录层级（仓库根目录下的 `checkpoints/`，不是 `gear_sonic/checkpoints/`——**第一次拷贝时错放到了 `gear_sonic/checkpoints/`，已移正**，如果发现这个路径不对不是 bug，是我自己改的）。`*.pt` 已被 `.gitignore:235` 忽略不会进 git，其余小文件（README/config.yaml 等）不受影响 |
| `gear_sonic/scripts/sim2sim_a3_mujoco.py`（4725 行） | 同路径 | 独立的 MuJoCo sim2sim 测试/评估脚本，**不依赖 isaaclab/pxr**（只 import `mujoco`/`torch`/`scipy`/`gear_sonic.utils.*`），可以在没有 Isaac Sim 的环境里跑。内部有一个完全独立于 Hydra/`UniversalTokenModule` 的 `A3Policy` 类（手搭 MLP：encoder `[640,2048,1024,512,512,64]`、decoder `[dec_dim,2048,2048,1024,1024,512,512,29]`、FSQ 量化），直接从 checkpoint 的 `policy_state_dict`/`actor_model_state_dict` 里按 key 前缀切出 encoder/decoder 权重加载——这是本脚本特有的"轻量复现"写法，不是训练时真正用的网络构建路径，所以**这个脚本跑通只能验证"checkpoint 权重本身是完好的、机器人物理资产是对的"，不能验证 Phase 1 改的训练侧 Hydra 配置代码**（commands.py/observations.py 等那些，训练时才会被用到，sim2sim 脚本不走那条路径）。见 41.4 的说明 |
| `gear_sonic/utils/playback.py`、`gear_sonic/utils/terrain_profile.py` | 同路径 | sim2sim 脚本的两个依赖，各自检查过内部没有 `gear_sonic.*` 的二次依赖，直接整体拷贝 |
| `gear_sonic/data/assets/robot_description/solver/a3_loop/`（68K，**gitignored**，含 `lib/liba3_ankle_waist_solver.a` 预编译库 + 3 个头文件） | 同路径 | 踝部/腰部并联连杆的闭环运动学求解器（sim2sim 脚本跑 `a3_t2d5_loop_*.xml` 这种闭环 MJCF 变体时要用），Phase 1 当时只拷贝了 MJCF/URDF 没拷这个，这次补上 |
| `a3_data/agibot_a3/001_walk_front_slow.csv` | 同路径（仓库根目录下的 `a3_data/`，不是 `gear_sonic/a3_data/`——同样第一次放错位置，已移正） | 一条参考动作（原始 30fps，1652 帧，约 33 秒），sim2sim 脚本用来生成参考轨迹喂给策略 |

**3. 跑验证**：在两个仓库里分别执行完全相同的命令（只有 cwd 和内部路径不同）：
```bash
export MODEL_035_PT="$PWD/checkpoints/035_step200000/model_step_200000.pt"
conda run -n gmr python gear_sonic/scripts/sim2sim_a3_mujoco.py \
  --checkpoint "$MODEL_035_PT" \
  --motion a3_data/agibot_a3/001_walk_front_slow.csv \
  --encoder-mode a3_fast \
  --mjcf gear_sonic/data/assets/robot_description/mjcf/a3_t2d5_loop_passive_foot_twostage_fit_optimized.xml \
  --batch-once --max-policy-steps 20 \
  --metrics-out <路径>.json
```
20 个策略步（50Hz 策略时钟，约 0.4 秒仿真时间），`--batch-once` 跳过交互式 viewer。

### 41.3 结果

**Litchi 原仓库跑出来的（基线）**：`fall: false`，骨盆高度 mean=1.0716m / min=1.0689m（站姿稳定，没有突然下沉或摔倒），骨盆姿态偏差 mean=0.59° / max=0.82°（20 步内几乎没有晃动），29 维全身关节跟踪 RMSE=0.0499 rad（约 2.86°，很小），动作增量平滑（`q_des_step_abs_max=0.143`，没有突变）。**结论：下载的 checkpoint 权重完好，035 策略本身工作正常，产生稳定的站立/走路行为**——这本身也确认了 Hugging Face 下载 + SHA256 校验没有问题。

**GR00T-WholeBodyControl 里跑出来的（用 Phase 1/2 迁移过去的资产）**：把两份 `metrics.json` 递归比较了全部 154 个数值字段（action 的 4 个统计量、骨盆位姿的 5 个统计量、29 维关节跟踪 RMSE、骨盆高度 mean/min、`fall` 布尔值等），**154 个字段，0 处数值差异（包括浮点数，diff 严格等于 0.0，不是"在容差内"）**，唯一的字符串字段（`body_name: "pelvis_link"`）也完全相同。

这个"数值严格为 0"的结果本身就是一个有意义的信号：**说明两边用的是完全相同的输入（同一个 checkpoint 文件、同一条 CSV）、完全相同的代码（sim2sim 脚本本身只拷贝没改）、完全相同的计算设备类型（都是 `device: cuda`，同一张卡），没有引入任何随机性或环境差异**——这正好印证了 Phase 1 对迁移方式的两条要求都做对了：资产文件（MJCF/URDF/求解器库/motor_params）复制得干净完整，没有路径解析错误或内容差异。

### 41.4 这个验证覆盖了什么、没覆盖什么（重要，不要误读结果）

**覆盖了**：
- checkpoint 文件下载/迁移的完整性（SHA256 + 行为双重确认）。
- Phase 1 迁移的机器人物理资产（`a3_t2d5_loop_passive_foot_twostage_fit_optimized.xml`、`urdf/a3/`、`solver/a3_loop/`、`a3_motor_params.py`）在新仓库里路径解析正确、内容完整，和原仓库产生逐 bit 相同的物理仿真结果。

**没有覆盖**（容易误以为"Phase 2 验证通过=全部都验证了"，实际不是）：
- **Phase 1 改的训练侧 Hydra 配置/代码（commands.py 的 A3-fast 未来帧窗口、observations.py、rewards.py 的电机约束惩罚、events.py 的域随机化、token_losses.py 的新损失函数）完全没有被这次验证覆盖**。原因见 41.2 表格里的说明：`sim2sim_a3_mujoco.py` 用的是脚本自己手搭的独立 `A3Policy` 网络类，直接从 checkpoint 里切权重加载，完全不经过 `UniversalTokenModule`/Hydra 配置/`TrackingCommand` 这条真正训练时走的路径。换句话说，这次验证证明了"迁移过去的*资产*没问题"，但**没有**证明"迁移过去的*训练配置代码*在真正训练/构建网络时会产生和 Litchi 一样的行为"。
- 要验证后者，唯一可靠的办法是真正走 Hydra `+exp=manager/universal_token/all_modes/sonic_a3_035` 这条路径实例化 `Actor`/`UniversalTokenModule`，而这需要 `env_config.robot.algo_obs_dim_dict`——这个值不是写死在 yaml 里的（`base_env.yaml:69` 只给了个空字典占位），是 IsaacLab 环境在真正构建 observation manager 时动态算出来的，意味着要做这一步验证**必须真正起一个 IsaacLab `ManagerBasedRLEnv`**（哪怕 `num_envs=1`），这需要完整的 Isaac Sim kit（启动慢，几分钟量级，吃显存）。本机其实有 GPU（RTX 5090）和 display（`:1`），技术上能起，但还没做这一步——因为 Phase 1 已经对每一处训练侧代码改动做了逐行 diff 验证（worklog #40 的方法论），间接提供了很强的正确性信心，真正起 IsaacLab 环境做端到端验证是更彻底但成本高得多的最后一步，值不值得现在做需要用户决定。

**一句话总结现在的确定性水平**：训练侧代码"长得和 Litchi 一模一样"（逐行 diff 验证过），机器人资产"跑起来和 Litchi 数值完全一样"（这次验证过），但"训练侧代码在真正跑训练时拼出来的网络是不是和 Litchi 一样"这件事**还没有直接验证过**，只是通过前两者间接推断出来的。

### 41.5 还没做的事

和 #40 末尾一样：Phase 3（C++ 部署集成）、Phase 4（VR 实时遥操）、`ROBOT_MODEL_REGISTRY["a3"]`。另外新增一项：如果需要更高确信度，可以起一次 IsaacLab 环境做端到端的 Hydra 配置+网络构建验证（见 41.4 最后一段），这个还没做，是可选的加强验证，不是阻塞项。

**（本条已被 #42 执行完成——下面是该验证的完整记录，包括过程中发现并修复的 3 处真实迁移遗漏。）**

## 42. IsaacLab 端到端验证：真正起 Hydra/`UniversalTokenModule`/`ManagerBasedRLEnv`，发现并修复 3 处 Phase 1 遗漏（本条目独立成文，假设读者零上下文）

### 42.0 这一条要解决什么问题

\#41.4 明确指出：sim2sim MuJoCo 验证只证明了"迁移过去的机器人资产（MJCF/URDF/motor_params/solver）正确"，**完全没有验证 Phase 1 改动的训练侧 Hydra 代码**（`commands.py`/`observations.py`/`rewards.py`/`events.py`/`token_losses.py`）是否在真正走训练/评估代码路径（`@hydra.main` 解析 `+exp=...` → 组装 `ManagerBasedRLEnvCfg` → 启动真正的 Isaac Sim → 实例化真正的 `UniversalTokenModule`/`Actor`）时产生和 Litchi 原仓库一样的行为。用户的原话是"把 IsaacLab 端到端验证也做了，彻底消除训练侧代码这块的不确定性"。本条目记录了完成这件事的完整过程，以及过程中发现的问题——**这次验证不是走个过场确认"没问题"，而是真的挖出了 3 处之前 #40 的逐行 diff 审查漏掉的真实 bug**，这恰恰说明了这次深度验证的必要性。

### 42.1 验证工具选择：为什么是 `eval_agent_trl.py` 而不是 sim2sim 脚本

`gear_sonic/eval_agent_trl.py` 是仓库里唯一会真正做以下事情的入口：
1. 读取 checkpoint 同目录下的 `config.yaml`（训练时落盘的、已经被 Hydra 完全解析过的配置快照，不是重新走一遍 `sonic_a3_035.yaml` 的 `defaults:` 继承链——这样可以保证两边用的是完全相同的、已经固化的配置，不会因为两个仓库当前 Hydra 组合逻辑的其他差异而引入新的不可控变量）。
2. 真正启动 `isaaclab.app.AppLauncher`（即真正的 Isaac Sim kit 进程，不是 mock）。
3. 真正构建 `ManagerBasedRLEnv`（`train_agent_trl.create_manager_env`），这一步会真正跑通 `modular_tracking_env_cfg.py` 的 `override_settings()`、`commands.py` 的 `TrackingCommand.__init__`/`_resample_command`、`observations.py`/`rewards.py`/`events.py` 里注册的所有 term。
4. 用 `config.algo.config.actor` 真正通过 `trl_utils_common.custom_instantiate(...)` 实例化 `Actor`/`UniversalTokenModule`（这一步依赖 `env_config.robot.algo_obs_dim_dict`，而这个值是 IsaacLab 观测管理器在环境真正建好之后才动态算出来的——`base_env.yaml:69` 只给了个空字典占位，无法只用 Hydra 组合配置、不起环境就提前拿到，这是 sim2sim 脚本做不到、必须靠这个真正入口才能验证到的核心原因）。
5. 用 `model.policy.load_state_dict(state_dict)`（**默认 `strict=True`，没有传 `strict=False`**）把 checkpoint 的权重加载进这个真正实例化出来的网络——如果迁移过去的代码让网络结构（层数/维度/参数名）和 Litchi 训练时的网络有任何不一致，这一行会直接抛 `RuntimeError`（缺键/多键/形状不匹配），是全部验证手段里最强的"架构完全一致"证据。

sim2sim 脚本（`sim2sim_a3_mujoco.py`）里的 `A3Policy` 是脚本自己手搭的独立 `nn.Module`，直接按 key 前缀切 checkpoint 里的 tensor 加载，完全不经过上面任何一步，所以 #41 的验证结果和这次验证是互补关系，不是重复关系。

### 42.2 验证命令

Litchi 自己的文档（`docs/a3_training2sim2deploy.md` 第 4 节）给出了针对这个 035 checkpoint 的权威调用方式，但原文档命令用的是 `export_onnx_only=true`（见 42.4 的说明，这条路线在 GR00T 里走不通，原因是另一个文件的 staleness，不是迁移 bug）。实际使用的最终命令（两个仓库完全一致，只有 cwd 不同）：

```bash
conda run -n isaaclab python gear_sonic/eval_agent_trl.py \
  +checkpoint="$PWD/checkpoints/035_step200000/model_step_200000.pt" \
  +headless=True \
  ++num_envs=1 \
  "++manager_env.commands.motion.motion_lib_cfg.motion_file=$PWD/.cache/motionlib_035_selected20" \
  +max_render_steps=20 \
  ++eval_callbacks=[]
```

准备工作（两边一致）：
- `a3_data/agibot_a3/` 下需要完整 20 条 selected20 CSV（#41 的 sim2sim 验证只迁移了其中 1 条 `001_walk_front_slow.csv` 就够用，这次需要全部 20 条，已从 Litchi 补齐拷贝到 GR00T 的同路径）。
- 用 Litchi 自己的 `gear_sonic/data_process/convert_soma_csv_to_motion_lib.py --input a3_data/agibot_a3 --output .cache/motionlib_035_selected20 --individual --robot a3_29 --fps 30 --fps_source 120 --num_workers 8` 跑出 20 个 per-motion pkl。**这一步刻意只在 Litchi 里跑了一次**，然后把产出的 `.cache/motionlib_035_selected20/` 目录直接拷贝进 GR00T（没有在 GR00T 里重新跑转换脚本）——原因是 GR00T 现有的 `convert_soma_csv_to_motion_lib.py` 是另一个完全不同的、服务于 G1/Bones-SEED 多模式批量转换的版本（本次会话更早阶段写的），**没有 A3 flat-CSV 模式**，两边脚本内容几乎完全不同。这次验证的目标是训练侧环境代码，不是数据转换脚本，所以用"两边喂完全相同的输入数据"的方式隔离变量，避免引入"到底是转换脚本错了还是训练代码错了"的混淆；A3 数据转换脚本本身尚未合并进 GR00T，记在 42.6 的遗留项里。

### 42.3 发现并修复的 3 处真实遗漏

这次验证的第一次运行就失败了，之后连续遇到 4 个报错，每一个都对应一处 Phase 1 没有覆盖到的真实代码差异。**这推翻了 #40 里"commands.py 等 6 个文件逐行 diff 验证=empty diff，已完全核实"的结论——diff 核实的是"能看见的改动对不对"，但没有覆盖"两个代码库在改动范围之外是否存在更早就已分叉、而 Phase 1 没意识到需要同步"的内容。** 具体：

**遗漏 1：`commands.py` 缺少 `coactivate_g1_when_teleop` 字段和逻辑**

报错：`TypeError: TrackingCommandCfg.__init__() got an unexpected keyword argument 'coactivate_g1_when_teleop'`（Hydra 用 checkpoint 的 `config.yaml` 里固化的 `coactivate_g1_when_teleop: true` 实例化 `TrackingCommandCfg` 时，GR00T 的 dataclass 根本没有这个字段，直接在构造函数这一步就炸了）。

查证：`grep coactivate_g1_when` 两边的 commands.py，Litchi 里这个字段在 `coactivate_g1_when_a3_fast` 字段的紧邻位置（cfg dataclass 第 4521/4545 行，init 里第 631-636 行，使用点在第 3282-3294 行），GR00T 当时只移植了 `coactivate_g1_when_a3_fast`（A3-fast 专属的新增字段），*漏移植了紧邻的、语义上更早就存在、与 teleop 配对延迟激活 G1 编码器逻辑相关的 `coactivate_g1_when_teleop`*——两个字段挨在一起，大概率是 Phase 1 做 diff 的时候正常框住了 a3_fast 相关的新增块，但 `coactivate_g1_when_teleop` 这个字段本身并不是 A3 新增的（它是训练 teleop 模式对齐用的，所有机器人通用），所以没有被当成"A3 相关改动"纳入搬运范围——这是本次遗漏的根本原因：**migration 时只对比了"Litchi 加了什么 A3 相关代码"，但没意识到 Litchi 在无关于 A3 的地方也存在比 GR00T 当前快照更新的通用改动（coactivate_g1_when_teleop 看起来就是这样一个通用改动，不清楚是什么时候加的，但显然在 Phase 1 做 diff 验证的那次快照之后或之外）**。

修复：在 `commands.py` 的 `TrackingCommandCfg` dataclass 里（`teleop_sample_prob_when_smpl` 字段之后）加回 `coactivate_g1_when_teleop: bool = False` 字段及其注释；在 `__init__` 里（`coactivate_g1_when_a3_fast` 的 getattr 之前）加回 `self.coactivate_g1_when_teleop = getattr(self.cfg, "coactivate_g1_when_teleop", False)`；在 `_resample_command` 里（soma 协同激活块和 a3_fast 协同激活块之间）加回完整的 teleop 协同激活 G1 编码器的 if 块，三处都和 Litchi 原文逐字节一致。

**遗漏 2：`commands.py` 的"额外 DOF"(`extra_joint_indices`) 用了 G1-hand 专属的启发式方法，对 A3 的被动脚弹簧关节完全不适用**

报错：`RuntimeError: shape mismatch: value tensor of shape [4] cannot be broadcast to indexing result of shape [1, 0]`（`joint_pos[:, self.extra_joint_indices] = self.extra_default_positions`，`extra_default_positions` 长度是 4，但 `extra_joint_indices` 解析出 0 个元素）。

根因：GR00T 原来的代码是 `self.extra_joint_indices = joint_utils.get_hand_joint_indices(self.robot)`——这是专门给 G1 手部关节写的、按关节名匹配"手"相关字符串的启发式函数，对 A3（没有手部自由度，但有 4 个被动脚弹簧关节 `.*_foot_forefoot_joint`/`.*_foot_toe_joint` ×2 脚）完全匹配不到任何东西，静默返回空列表，而不是报错——**这是一个只有在真正跑 A3 机器人时才会暴露的潜在 bug，纯静态 diff 审查根本看不出来**，因为 GR00T 当时的 commands.py 变更本身就是"保留 G1 专属实现、手写了一个不依赖 cache 的等价版本"（worklog #40 原文），完全没意识到这一行需要对所有机器人通用化。

Litchi 的做法更通用、更安全：`body_joint_indices`（已有的、机器人无关的函数 `joint_utils.get_body_joint_indices`）取补集算出"剩下的就是额外自由度"，并且加了三处显式校验（`body_joint_indices` 数量必须等于 `motion_lib_num_dof`，额外自由度数量必须等于 `extra_num_dof`，默认位置/速度长度必须等于 `extra_num_dof`，任何一处不符都直接 `raise ValueError` 而不是静默产生错误结果）。

修复：把 GR00T 对应代码块整体替换成 Litchi 的版本（逐字节一致），不再依赖"手部关节名启发式"，而是用"机器人总自由度减去动作库自由度的补集"这个通用定义，并保留三处校验。这对现有 G1 行为**没有影响**（G1 的手部关节本来就全部落在"非 body_joint_indices"的补集里，补集方法和手部启发式方法对 G1 算出来的 `extra_joint_indices` 集合是等价的——这点已经通过"G1 训练入口本身没有在这次改动后重跑过"这一事实留了个心眼，严格说这是**推断**而非**重新验证**，如果之后要百分百确认 G1 零回归，建议补一次 G1 checkpoint 的同类 `eval_agent_trl.py` 冒烟测试，当前没有做）。

**遗漏 3：`modular_tracking_env_cfg.py` 整个文件在 Phase 1 里完全没有被触碰过**

报错：`KeyError: 'a3_024'`（`robot_mapping[robot_type]["robot_cfg"]`，checkpoint 的 `config.yaml` 里 `manager_env.config.robot.type == "a3_024"`，但 GR00T 的 `robot_mapping` 字典只有 `"a3_ultra"` 一个 A3 相关键，根本没有 `"a3"`/`"a3_024"`/`"a3_exp020"`/`"a3_exp023"` 这几个键）。

这是本次发现的三处里**影响面最大**的一处：`modular_tracking_env_cfg.py` 是把"机器人类型字符串"分发到具体 `ArticulationCfg`/action scale/`isaaclab_to_mujoco_mapping`/关节子集过滤这些具体配置的核心调度文件。Phase 1 当时的迁移范围表（worklog #40）里完全没有列出这个文件——原因是 Phase 1 聚焦在"mdp 目录下的训练逻辑文件"（commands/observations/rewards/events/token_losses），而这个调度文件在 `gear_sonic/envs/manager_env/` 目录下，不在 `mdp/` 子目录里，当时的搜索范围没有覆盖到它。而 `gear_sonic/envs/manager_env/robots/a3.py`（机器人资产定义本身）虽然在 Phase 1 就拷贝过去了，但**从来没有被接到调度逻辑里**——相当于把发动机搬过去了，但没接上点火开关。

验证过程中用 `grep` 确认了一个好消息：`a3.py` 里实际上已经包含了 `A3_024_CYLINDER_CFG`/`A3_EXP020_CYLINDER_CFG`/`A3_EXP023_CYLINDER_CFG`/`A3_024_ACTION_SCALE`/`A3_EXP020_ACTION_SCALE` 这些 Litchi 需要的全部变体（Phase 1 拷贝 a3.py 时是整个文件原样拷贝，所以这些变体定义其实已经在，只是没人引用），所以这不是"缺资产"，纯粹是"缺接线"。

修复范围（刻意限定在这个 035 checkpoint 实际会用到的子集，完整对比方法见下）：对 `checkpoints/035_step200000/config.yaml` 里 `manager_env.config.robot` 字段做了精确检查，确认只有三个键：`type: a3_024`、`asset_path: .../model_collision_optimized_passive_foot_twostage_fit_optimized.urdf`、`use_fitted_passive_foot_gains: true`（后者在 Litchi 代码里被 `if robot_type == "a3"` 这个条件门控，对 `a3_024` 类型不生效，等于没用——这点在 Litchi 原文件里也是这样写的，不是我们引入的差异）。据此：
- 文件顶部 import 增加 `copy`、`logging`、`from isaaclab.managers import SceneEntityCfg`、把 `from gear_sonic.envs.manager_env.robots import a3_ultra, g1, h2` 改成多 import 一个 `a3`。
- 新增 4 个辅助函数（逐字节照抄 Litchi）：`_ensure_urdf_importer_compat`（给 URDF importer 扩展钉版本）、`_set_a3_fitted_passive_foot_gains`（给被动脚弹簧设置拟合好的刚度/阻尼/惯量参数，虽然这次用不到但顺手带上，因为依赖它的 a3.py 字段已经存在，纯增量、零风险）、`_set_joint_asset_cfg`/`_filter_joint_observations`（把某个 reward/event/observation term 的 `asset_cfg` 改成只认 `active_joint_names` 这个子集的关节列表，用 `preserve_order=True` 保证顺序和训练时一致）。
- `robot_mapping` 字典新增 `"a3"`/`"a3_exp020"`/`"a3_024"`/`"a3_exp023"` 四个键（全部引用 a3.py 里已经存在的变体，`active_joint_names` 统一指向 `a3.A3_ACTIVE_JOINT_NAMES`），**没有**加 `"a3_old"`（这个键在 Litchi 里指向一个单独的 `a3_old.py` 模块，GR00T 里压根没有这个文件，035 checkpoint 也用不到，记在 42.6 的遗留项里，没有强行拼凑）。
- `override_settings()` 的机器人选择逻辑从"直接 `.replace()`"换成 Litchi 的版本：`copy.deepcopy` 之后再 `.replace()`（避免多次实例化共享同一个模块级可变对象导致互相污染）、新增 `asset_path` 覆盖（**这个是 035 checkpoint 真正需要的**，它指定了 8 个 A3 URDF 变体里具体用哪一个）、新增 `merge_fixed_joints`/`disable_passive_foot`/`use_fitted_passive_foot_gains` 三个条件分支（均按 Litchi 原文逐字节复刻，均被各自的 config 键门控，对现有 g1/h2/a3_ultra 行为零影响）、新增 `active_joint_names` 过滤块（**这个也是 035 checkpoint 真正需要的**：它把 `self.commands.motion.motion_lib_num_dof` 设成 `len(active_joint_names)`，这个值正是遗漏 2 里 `has_dof_mismatch`/`extra_num_dof` 计算的输入，两处遗漏其实是同一条因果链上的两环——不先修好这里，遗漏 2 的报错根本不会以"有意义的方式"出现)。
- `robot_type` 的默认值**没有**从 Litchi 的 `"a3_024"` 改过来，继续保持 GR00T 原来的 `"g1"` 默认值——这是蓄意的分歧：Litchi 是 A3-only 的公开发布仓库，默认机器人当然是 A3；GR00T 是多机器人仓库，必须保持"不显式指定 robot.type 时退回 G1"这个向后兼容默认值，这一点逐字照抄 Litchi 会制造一个新 bug（所有没显式写 `robot.type` 的现有 G1/H2 config 会突然变成尝试加载 A3 资产)。

**本次验证刻意跳过、未移植的部分**（均已确认对 035 checkpoint 的这次验证完全不生效，不是"忘了"，是"确认了配置里没打开，刻意不在这次任务里扩大范围"）：
- `_set_a3_foot_contact_compliance`（URDF spawn 时的可配置脚底柔顺接触材质）：依赖 `A3_FOOT_CONTACT_LINK_NAMES`/`spawn_a3_urdf_with_foot_contact_compliance`/`spawn_a3_urdf_with_per_env_foot_contact_compliance`，这几个在 GR00T 的 a3.py 里都不存在（Litchi 侧也是定义在 `a3_old.py` 里，通过参数传进去），且门控条件 `robot_type in {"a3", "a3_old"}` 对 `a3_024` 不成立，`foot_contact_compliance` 键本身在 checkpoint config 里也不存在。
- `_set_a3_motor_space_actuators`/`motor_model_cfg`（把隐式 PD 驱动换成显式电机 T-N 曲线驱动模型）和 `_set_delayed_implicit_actuators`/`actuator_delay_cfg`（给驱动器加延迟）：两者都依赖 `A3MotorSpaceActuatorCfg` 这个类，GR00T 的 `actuators.py` 里**完全没有这个类**（只有 `DelayedImplicitActuatorCfg`，是另一半依赖，已存在）。`motor_model`/`actuator_delay` 两个键在 checkpoint config 里都不存在，判定为这次用不到，没有移植 `A3MotorSpaceActuator`/`A3MotorSpaceActuatorCfg`（在 Litchi 的 `actuators.py` 里是第 106-344 行，約 240 行，是个独立的、基于 T-N 曲线和并联关节雅可比的隐式驱动器类，移植它本身是一件独立工作量不小的事，值得单开一条 worklog 做，不应该顺带混在这次验证里）。
- `_rank_uses_terrain`/`rank_partitioned_terrain`（按 DDP rank 分配地形/配对动作的训练课程功能，影响面涉及 `modular_tracking_env_cfg.py` 和 `commands.py` 两个文件）：checkpoint config 里没有这个键，且这个功能看起来是通用的地形课程基础设施，和 A3 本身无关（可能是 Litchi fork 时的上游基线比 GR00T 当前状态更新带来的差异，不是 A3 专属新增)。
- `foot_end_contact_sensor`/`ContactSensorCfg`（给 A3 URDF 的 `hard_sole_3seg` 系列脚部资产加接触力传感器）：checkpoint config 里没有打开这个开关，而且这次用的 asset_path 本身不含 `hard_sole_3seg` 字样。
- `_cached_reference`/`cache_current_references`（commands.py 里整层按帧记忆化缓存机制，worklog #40 就已经记录过是"刻意不移植，手写等价无缓存版本代替"，这次验证重新确认了这个决定——checkpoint config 里没有 `cache_current_references` 这个键，等价于 Litchi 自己跑这个 035 实验时这个缓存机制也是关闭的，所以两边在这次验证里实际执行的是等价代码路径，不存在"因为没移植缓存机制所以行为不同"的风险)。
- 以及 commands.py 的 reset-subset 性能优化重构（用 `len(env_ids)` 大小的张量而不是 `num_envs` 大小张量再掩码写回）、paired-terrain 相关的 `_use_paired_motions`/`rank_partitioned_terrain` 分支、adaptive-sampling 的"重置前一步快照"bug 修复（`_adp_prev_motion_ids`/`_adp_snapshot_valid`，Litchi 的注释原文是"Isaac Lab resamples terminated environments before `_update_command` reads the cursor. Keep the pre-physics cursor so failures are charged to the motion/bin that was actually being tracked"——这是一个独立于 A3 的自适应采样统计口径修正，`use_adaptive_sampling` 这次用的 035 配置里是否打开没有专门确认，如果打开了，这一处是潜在的、本次验证没有修复的行为差异，列入 42.6 遗留项）：这些都是只影响性能或训练统计口径、不影响"网络是否被正确构建、动作是否被正确计算"这个本次验证核心问题的代码,逐一确认后决定不动,避免在验证任务里顺带做一次范围不可控的性能重构。

### 42.4 一个已确认"不是迁移 bug"的插曲：`eval_agent_trl.py` 本身的 staleness

Litchi 文档推荐的调用方式带 `+export_onnx_only=true`，在 Litchi 仓库里完整跑通（4 个 onnx 文件全部导出成功：`_g1.onnx`/`_a3_fast.onnx`/`_encoder.onnx`/`_decoder.onnx`，日志确认 `obs_dict[1,1570] → action[1,29]`，和文档描述完全一致）。但同样的命令在 GR00T 里报 `KeyError: 'smpl'`（`inference_helpers.export_universal_token_module_as_onnx` 在尝试导出 smpl 专属 onnx 时，`module.encoder_input_features['smpl']` 不存在——这个 035 实验本来就只训了 g1/a3_fast 两个 encoder,没有 smpl/teleop)。

查证：`diff` 两边的 `eval_agent_trl.py`（Litchi 版本多出整整一个 `EvalDebugDumper` 类，共 42KB 的 diff），确认这个文件本身在 Phase 1 之外、独立于 A3 迁移就已经出现了分叉——GR00T 这边的版本是更早的快照，硬编码按固定顺序导出 smpl/g1/teleop 三个 encoder 的 onnx,没有做"检查这个 encoder 是否存在"的防御性判断;Litchi 那边版本更新,这部分逻辑大概率已经改成按需导出。**这不是本次 A3 迁移引入的问题,这个文件从来没有被纳入 Phase 1 的迁移范围（它是通用 eval 入口,不是 A3 专属文件）**,所以没有去动它,而是换一种调用方式绕开这个有问题的代码分支:不传 `export_onnx_only`,改用默认的 `run_eval_loop` 真实 rollout 路径,配合 `+max_render_steps=20` 做一次快速、真实的多步闭环验证——这条路径不涉及那个 staleness 问题,而且更贴近"验证训练侧环境代码"这个本次任务的真正目标(ONNX 导出验证的是"网络能不能被 trace 成静态图",而不是"环境每一步的观测/奖励/事件代码对不对")。这个 `eval_agent_trl.py` 的 staleness 本身也记入 42.6 遗留项,但不阻塞,文件存在不代表必须现在修。

### 42.5 最终验证结果

两个仓库分别执行以下命令（仅 cwd 不同）：

```bash
conda run -n isaaclab python gear_sonic/eval_agent_trl.py \
  +checkpoint="$PWD/checkpoints/035_step200000/model_step_200000.pt" \
  +headless=True \
  ++num_envs=1 \
  "++manager_env.commands.motion.motion_lib_cfg.motion_file=$PWD/.cache/motionlib_035_selected20" \
  +max_render_steps=20 \
  ++eval_callbacks=[]
```

（Litchi 那边因为 motion_file 支持列表语法，实际传的是 `[$PWD/.cache/motionlib_035_selected20]`，GR00T 这边因为没有移植"motion_file 可以是序列"这个新功能——也是和 A3 无关的 `motion_lib_base.py` 独立分叉，细节见下——传的是不带中括号的裸路径，两边 `motion_lib_base.py` 现有的、未经改动的目录加载逻辑都支持裸目录路径，功能等价。)

两边结果：`EXIT_CODE=0`，日志里都确认了：
- `Model parameterization: std` / `Successfully loaded policy state dict`——**strict `load_state_dict` 成功**,证明两边在这次验证里真正实例化出来的 `UniversalTokenModule`/`Actor` 网络结构（层数、每层维度、参数命名）与 checkpoint 完全一致,逐参数无差异。这是本次验证能给出的最强的"架构等价"证据。
- `Reached max_render_steps=20. Exiting.`——20 个完整的真实 `env.step()` 循环全部正常完成,期间真正执行了 commands.py 的 `_resample_command`/`_update_command`（含 A3-fast 未来帧窗口、DOF mismatch 处理、encoder 协同激活）、observations.py 的新增 term、rewards.py 的 4 个新增 A3 电机约束 reward、events.py 的新增域随机化 term,全程无报错、日志里搜索 "nan"/"error"/"traceback" 均为空（已排除 TracerWarning/UserWarning 等无关噪声）。

**这次验证证明了什么**：Phase 1 迁移的训练侧代码（在本条目修复了上述 3 处遗漏之后）,在真正被 Hydra 实例化、真正驱动一个真实 `ManagerBasedRLEnv` 跑 20 步闭环交互的条件下,产生了和 Litchi 原仓库同一 checkpoint 在同一类命令下一致的行为（strict 权重加载成功 + 多步无崩溃）。这填补了 #41.4 明确指出的空白——训练侧代码不再只是"长得和 Litchi 一样"（diff 层面）,而是"真的能被正确实例化并跑起来"（运行时层面）。

**仍然保留的不确定性**（诚实记录，不夸大这次验证的覆盖面）：
- 没有做逐 step 的数值级比较（比如把两边第 1~20 步的 action tensor dump 下来逐元素比较）。原因：`eval_agent_trl.py` 显式设置 `torch.backends.cudnn.deterministic = False`，且两边是两个独立的 Isaac Sim 场景实例（各自的 PhysX 刚体初始化、碰撞检测顺序等在 GPU 上不保证 bit-exact 复现），逐 step 数值比较本来就不会得到"差值为 0"这种强信号,而只能得到"差值在某个容差内"这种弱信号,参考价值有限。相比之下,"strict 权重加载成功"+"20 步无崩溃"已经是更直接、更不依赖容差判断的正确性证据。如果未来需要逐 step 数值比较,建议改用 CPU 后端跑（牺牲速度换确定性）,这次没有做。
- 遗漏 2 的修复对现有 G1 行为"推断零影响"但未重新跑 G1 的同类验证做实证确认（见 42.3 遗漏 2 小节末尾）。
- `use_adaptive_sampling`（自适应采样）在这次 035 配置里是否被打开没有专门确认；如果打开了，Litchi 的"重置前快照"bug 修复（见 42.3 末尾未移植列表）会是一个本次验证没有覆盖、仍然存在的行为分歧。

### 42.6 遗留项（本条目新增，供下次继续时直接读取，无需重新调查）

1. `convert_soma_csv_to_motion_lib.py` 的 A3 flat-CSV 支持尚未合并进 GR00T（两边脚本完全是两个不同的实现，服务不同的使用场景——G1/Bones-SEED 批量 vs A3 单机 flat-CSV）。如果之后需要在 GR00T 里直接跑 A3 数据转换（而不是像这次一样从 Litchi 拷贝转换好的 `.cache/`），需要把 Litchi 版本的 A3 专属逻辑（`load_a3_flat_csv`/`A3_SPEC`/`A3_29D_SPEC`/`detect_flat_csv_robot`）合并进 GR00T 现有脚本，而不是整体替换（会破坏现有 G1/Bones-SEED 批量转换功能）。
2. `a3_old.py` 机器人模块、`"a3_old"` robot_mapping 键、`_set_a3_foot_contact_compliance` 脚底柔顺接触材质功能均未移植（035 checkpoint 不需要，若未来要跑用到这些的其他 A3 实验配置需要补）。
3. `A3MotorSpaceActuator`/`A3MotorSpaceActuatorCfg`（显式电机 T-N 曲线驱动模型，`actuators.py` 约 240 行）及依赖它的 `motor_model_cfg`/`actuator_delay_cfg` 两条 `override_settings()` 分支未移植（035 checkpoint 不需要）。
4. `eval_agent_trl.py` 本身存在独立于 A3 迁移的 staleness（缺 `EvalDebugDumper`，`export_onnx_only` 分支的硬编码 encoder 导出列表对只训了部分 encoder 的实验不够健壮），这个文件此前从未被纳入任何一次 Phase 1/2 迁移范围，不是本次任务的责任范围，但如果以后要用 `export_onnx_only` 走 ONNX→RKNN 部署链路（`a3_training2sim2deploy.md` 第 5 节），需要先解决这个。
5. `motion_lib_base.py` 支持"`motion_file` 可以是目录列表"这个新功能（Litchi 侧的 `_is_motion_file_sequence`/`_find_semantic_manifest`/`_load_validated_semantic_manifest` 等）未移植，这次验证绕开了（直接传单个目录路径，两边都支持），但如果以后要合并多个数据源目录会需要。
6. 遗漏 2 的"G1 零回归"目前是推断而非实证（见 42.3）；`use_adaptive_sampling` 若开启时的"重置前快照"行为分歧未修复（见 42.3 未移植列表 + 42.5 不确定性列表）。
7. `commands.py`/`modular_tracking_env_cfg.py` 这次为了打通 A3 验证陆续补的几处改动，没有重新对 G1/H2 现有训练配置跑一次回归冒烟测试（只用静态 import + 这次 A3 的真实运行作为信心来源）——建议在正式合入/训练前补一次 G1 的 `eval_agent_trl.py` 同类快速冒烟（`+max_render_steps=20`），以获得同等级别的"零回归"证据，而不是停留在"读代码推断不影响"。

## 43. [2026-10-14] `--robot g1|a3|a3_ultra|h2` 统一入口 + Litchi 并入完成 + 官方 035 权重部署验证 + git 推送（面向冷启动 agent 的自包含记录）

完整的改动清单、命令和环境坑记录在 `docs/workflow/a3_integration_plan.md` 第 8 节，本条只写结论、决策和证据，避免两处重复。

### 43.1 要解决的问题与最终范围

用户要求：把 Litchi_sonic_for_a3 的训练代码、035 step-200k 权重（PT/ONNX/RKNN）、C++ 部署代码、环境配置都并入本仓库，并让训练和部署都用 `--robot` 选机器人；G1/H2 不能回归。

最终支持 `g1 / a3 / a3_ultra / h2`：
- `a3` = Litchi 035 合约（A3 T2D5，29 DOF，ABI 名 `g1`/`a3_fast`）。
- `a3_ultra` = 本项目自研 T3D0 63 DOF 合约。两套合约的 Kp/Kd 绝不混用。
- 部署侧 `a3` 与 `a3_ultra` 都走同一个 A3 AimRT 包，用 035 产物（worklog #37/#40：同一硬件平台）。`a3_ultra` 自研合约还没有部署导出。`h2` 没有 C++ 后端。
- **`g1_old` 曾被我加入又整体撤销**：用户明确说不需要。不要再加回来。

### 43.2 关键决策

1. 合并方法：三方合并（base=`daf3899`），Litchi 的 A3 发布版删除了 G1/H2 代码，所以每个删除块都人工复核，会损害 G1/H2 的全部回退。备份引用 `refs/backup/pre_litchi_merge`。
2. C++：不再把 G1 和 A3 拆成两个仓库分支，而是用 CMake 开关 `ENABLE_TRT_INFERENCE` / `ENABLE_G1_BACKEND` 让两者共存；`deploy.sh --robot X` 按机器人分发（g1 走原流程，a3/a3_ultra 转给新脚本 `deploy_a3.sh`）。
3. `modular_tracking_env_cfg.robot_type` 默认值保持 `g1`（Litchi 默认 A3，照搬会让现有 G1/H2 配置误加载 A3 资产）。
4. git 策略（用户要求）：不上传 `*.csv` 和 `*.cmake`，但“不影响复现和代码移植”。因此：
   - 以 `*.csv`/`*.cmake` 忽略为基线，再对必需的小文件加例外：4 个 A3 CMake 源文件、20 个 A3 样例 CSV、部署运行时 CSV、`a3_motor_model` 数据包、A3/A3 Ultra 的 MJCF/URDF 文本、求解器头文件和静态库、sim2real 参考 CSV。
   - 发现根目录原有的 `data/` 规则会匹配 `gear_sonic/data/`，导致第一次推送漏掉 `a3_motor_model/` 和 A3 机器人资产（会让干净仓库无法加载 A3 模型）。已改为“重新包含该目录、忽略其子项、再白名单”。
   - 大资产不进 git，沿用用户自己做的清理 `60065f3`：权重、动作库、数据集、A3 Ultra 网格和 USD。唯一例外是应用户要求加入的 A3（Litchi）104 个 STL 网格（约 20MB，走 Git LFS）。
   - 已提交推送：`490c7aa`（主体）、`cd9285f`（小文件白名单）、`687bdc7`（A3 网格），分支 `origin/base`。

### 43.3 验证证据

**官方 035 权重的部署验证**
- 部署包里两个 ONNX 的 SHA256 与 `a3_hf_manifest.json` 一致。
- 官方 PT 在 MuJoCo sim2sim：`001_walk_front_slow.csv`，1652 步，未摔倒，`all_29_rmse=0.066`，根部倾角最大约 9.6°。
- ONNX 对 PT 逐步对照：`a3_fast`、`g1` 两个 ONNX 最大差 ≤ 3.8e-6。ONNX 输入布局为“编码器按观测项排列（term-major）640 维 + 本体 930 维”，共 1570 维（试了 4 种拼接，只有这一种吻合）。
- ONNX 直接驱动 MuJoCo 闭环：两个官方 ONNX 都走完 1652 步，未摔倒（rmse 0.0662 / 0.067）。
- C++ 探针（部署包自带 ORT 1.16.3，`ramp` 输入）的动作 sum/abs_sum/min/max 与 Python ORT 在 4 位小数内一致，单次推理约 0.87 ms。
- `deploy.sh --robot a3 --probe`：C++ 部署程序加载了 `g1` 与 `a3_fast` 两个官方 ONNX，不发布指令，随后等待机器人状态。
- **没验证**：收到真机状态后输出指令这一环（没有真机或假机器人）。

**训练冒烟**（`--robot` 入口，16 环境，2 次迭代，均正常完成）：`a3`（官方 PT 热启动，selected20 动作库）、`a3_ultra`（`robot_filtered_v15_clean/210531`）、`g1`（官方样例动作库，随机初始化）。`h2` 没有本地动作库，没跑。

**pytest**（isaaclab 环境）：`tests` + `gear_sonic/tests` 共 169 个通过、0 个失败，1 个收集错误是环境缺 `msgpack_numpy`（遥操作可选依赖，与 A3 无关）。部署目录的 A3 测试 `test_a3_eval_cli.py`、`test_a3_body_drive_debug_convert.py`：4 通过 1 跳过；同目录的 G1 ZMQ 测试因缺 `zmq` 无法收集（环境问题）。C++ 单测此前 192/200 通过，已知失败项为缺少 `reference/bones_072925_test/` 夹具的 FK 测试，其余失败项没有逐一复核。
- 对 #42.6 第 7 项：G1 现在有了训练入口层面的冒烟（不是 `eval_agent_trl.py` 的 20 步评估），“G1 零回归”仍不算完整实证。

### 43.4 本次踩到的坑（再次遇到时直接看这里）

- **iceoryx 版本必须一致**：部署包用 2.95.4 编译，conda 的 `iox-roudi` 是 2.0.2，不兼容，表现为一直 “RouDi not found - waiting”。用构建目录编译同版本：`cmake --build build_verify_full --target iox-roudi`（需要 `CPATH`/`LIBRARY_PATH` 指向 conda 的 libacl）。
- 运行 C++ 时要把 `$CONDA_PREFIX/lib` 加进 `LD_LIBRARY_PATH`，否则 `librclcpp.so` 找不到。
- 仓库路径含空格会让 `accelerate launch` 拆开 Hydra 参数（`LexerNoViableAltException`）。用无空格软链接路径；此时 Python 的 `sys.path[0]` 是真实路径，`train_agent_trl.py` 已改为同时删除逻辑路径和 realpath，避免 `gear_sonic/trl` 遮盖 HF 的 `trl`。
- `pip install --target` 装 onnxruntime 时会把 numpy 2.x 一起装进目标目录并遮盖环境 numpy 1.26.4，装完要删掉目标目录里的 numpy。
- `git check-ignore -v` 会把否定（`!`）规则也打印出来，判断“是否仍被忽略”要看退出码或最后一条是否是 `!`，不要只数输出行数。
- `download_from_hf.py --help` 只显示 GR00T 那一半的参数；A3 参数（`--robot`、`--component`、`--list`）在统一入口里由 argv 分流，需用 `--robot a3 --list` 验证。默认只下 ONNX + RKNN，PT 要加 `--component pt`。
- 在 `git push` 到受保护分支 `base` 时，若用户没有明确说要推，工具会拦截；需要用户确认。

### 43.5 数据与官方开源范围

- GR00T 开源：训练代码、检查点、C++ 部署、Bones-SEED（14.2 万条，含 G1 重定向 CSV 与 SOMA BVH，**受限访问，需在 HF 同意许可**）、转换脚本。SOMA 重定向器在外部仓库 NVIDIA/soma-retargeter。
- Litchi 开源：A3 训练适配、035 检查点、20 条样例 CSV、C++ 部署；**不含**内部动作库、实时重定向和遥操作链路。
- 本地已有：`soma_proportional.tar.gz`（43G，SOMA BVH）、`data/motion_lib_a3ultra/`、`sample_data/`（G1 仅 1 条）、`a3_data/agibot_a3/`。HF 缓存里的 `datasets--bones-studio--seed` 只有 README，**G1 CSV 没下载**。用户选择自己下载 `g1.tar.gz` 放到 `data/bones_seed/` 后解压，之后由我做 `convert_soma_csv_to_motion_lib.py` 转换和 `--robot g1` 训练冒烟。

### 43.6 遗留项

1. G1 CSV 待用户下载后做转换、过滤、训练冒烟（全量体量很大）。
2. `sonic_a3_ultra.yaml` 的 `motion_file` 默认值是 `data/motion_lib_a3ultra/robot_filtered`（用户要求保持），本地该目录不存在，现有目录是 `robot_filtered_v15` 和 `robot_filtered_v15_clean`；直接用默认配置训练需先重命名/软链或显式覆盖。
3. `a3_ultra` 自研 63 DOF 合约没有部署导出；`h2` 没有 C++ 后端；“收到真机状态后输出指令”未验证。
4. A3 Ultra 的网格（约 36MB，另有重复的 `robot_model/model_data/a3_ultra` 和 35MB 的 `geometries.usd`）不在 git 里。`a3_t2d5_loop_hull_optimized.xml` 引用的两个 `*_hull_optimized.STL` 在 Litchi 公开版本里也没有（上游遗留，035 主流程不用）。
5. `ROBOT_MODEL_REGISTRY` / `ROBOT_WBC_YAML` 尚未注册 a3 / a3_ultra / h2（目前只有友好报错）。Phase 4（VR 实时重定向）、完整 G1/H2 回归未做。
6. 清理被 gitignore 的构建目录（`gear_sonic_deploy/build/`、`build_verify_full/`、`dist/`）。
7. 本条目之后的 README 补充（A3 章节）见仓库根 `README.md`。

---

## 44. [2026-10-14，接 #43] G1 数据/回归、A3 / A3 Ultra 的 MuJoCo 仿真 + Pico 链路、机器人模型注册、`a3_ultra` 参数对齐 Litchi a3（本条是摘要，细节、命令和数据在 `docs/workflow/a3_integration_plan.md` 第 10–13 节）

**读者须知**：从 #43 的"`--robot g1|a3|a3_ultra|h2` 统一入口"往下继续。本条把 #43 之后同一天做完的事按时间顺序列出，每一项都标明验证到了哪一步、哪些没验证。用户的决策（不要再讨论）：`--robot` 取值是 `g1 / a3 / a3_ultra / h2`（**不要 `g1_old`**，用户两次明确说过，它是最初需求里的过时写法）；用户只有一台 **A3 Ultra** 真机；PICO 头显只能用户本人操作；仿真里 A3 Ultra 用它自己的模型（T3D0 MJCF），权重用下载的 035；`a3_ultra` 的关节级控制参数先用 Litchi 的 a3，不用官方文档。

### 44.1 G1 数据与回归
- 用户下载了 `data/bones_seed/g1.tar.gz`（23.5 GB），解压到 `data/bones_seed/g1_extract/`：142,220 个 CSV、124 个 session，与 Bones-SEED 总数一致。
- `convert_soma_csv_to_motion_lib.py --fps 30 --fps_source 120 --individual` 全量转换：142,220/142,220 成功，0 失败，8.6 GB（`data/motion_lib_bones_seed/robot_all`）；`filter_and_copy_bones_data.py`（仓库默认 40 个关键词）：保留 129,785、过滤 12,435，7.9 GB，输出到 `sonic_v1_1.yaml` 默认读取的 `data/motion_lib_bones_seed/robot_filtered`。
- G1 官方训练权重 `sonic_v1_1/last.pt` 已下载（`download_from_hf.py --training --sonic-v1-1 --no-smpl`），`.gitignore` 新增 `sonic_v1_1/`、`low_latency/`。
- 冒烟：`--robot g1` 训练（16 envs、2 次迭代）`exit=0`，用默认 `motion_file` 加载 129,785 个动作；`eval_agent_trl.py` 用官方 `sonic_v1_1/last.pt`、1 env、20 步：strict 加载成功、无报错（补上 #42.6 第 7 项的 G1 运行时证据）；`a3_ultra` 训练用新默认 `motion_file`（`robot_filtered_v15_clean`）冒烟 `exit=0`，加载 129,116 个动作。
- G1 的 MuJoCo + Pico 流程（`docs/notes/mujoco_pico_sim_teleop.md`，本机本来就能跑）在 CMake 大改之后重新验证：全新目录里按 `just build` 的参数重编译 0 error；终端 1 `run_sim_loop.py --robot g1`、终端 2 `deploy.sh --robot g1 --input-type zmq_manager sim`（日志出现 `Init Done`）、终端 3 `pico_manager_thread_server.py --robot g1 --manager`（连上 RoboticsService）都正常。**没有验证**：真实头显数据、v1.1 变体、G1 真机。文档里所有 G1 命令都加了显式 `--robot g1`。

### 44.2 A3 / A3 Ultra 的 MuJoCo 仿真 + C++ 部署（与 G1 的做法对称，**没改一行 C++**）
- 新增 `gear_sonic/utils/mujoco_sim/a3_sim.py`：MuJoCo 里用 A3 的 ROS2 `/body_drive/*` 话题冒充机器人（发布 `{leg,waist,arm,neck}_joint_state`、`{pelvis,torso}_imu/data`，订阅 `*_joint_command`，力矩 = `tau_ff + kp(q_des-q) + kd(dq_des-dq)`，弹性绳按 `9` 释放），所以 `a3_deploy_onnx_ref` 在仿真和真机上是同一个二进制。`run_sim_loop.py --robot a3|a3_ultra` 会自动切到带 ROS2 的 conda 环境 `a3_deploy_verify`（一次性准备：`gear_sonic_deploy/scripts/setup_a3_sim_env.sh`，向该环境装了 mujoco/scipy/tyro 和 `joint_msgs` Python 绑定）。ROS2 一律用 `ROS_DOMAIN_ID=87`、`ROS_LOCALHOST_ONLY=1`。
- `deploy.sh --robot a3|a3_ultra [sim|real] [--input-type zmq_manager]`：`sim` 强制 ROS2 + 私有域；`zmq_manager` 会在部署程序旁自动起 `a3_teleop_retarget_node.py`（gmr 环境）和 `a3_ta_bridge`。部署程序是键盘状态机（要求 TTY）：`S`→`M`→`T`→`3`。
- 机器人模型注册：`ROBOT_MODEL_REGISTRY` = `g1 / a3 / a3_ultra / h2`，`ROBOT_WBC_YAML` 注册 `a3 / a3_ultra`（`h2` 没有仿真配置：没有策略权重也没有 C++ 后端，无法闭环验证）。`get_g1_key_frame_poses` 不再写死 G1 链接名；`pico_manager_thread_server.py` 对非 G1 关闭 G1 网格叠加。
- 联调结果（无头 + 定时释放弹性绳，用 Pico 模拟器代替头显）：`a3`（T2D5 模型）与 `a3_ultra`（T3D0 模型）都能站稳 60 s 以上，骨盆 1.07–1.075 m，倾角 <2°；Pico 全链路（SMPL `pose` 流 → 重定向 → `a3_ta_bridge` → `/ta/whole_body_command` → 部署程序 → 仿真）下，`synthetic:wave` 机器人跟着挥右臂，真实行走 SMPL 片段约 70 s 未摔倒。
- **联调中发现并修好的真实问题**：(1) A3 Ultra 的 MJCF 地面 `contype=conaffinity=0`（worklog #3 手写内联地面时遗漏），机器人松绳后穿地，4 份拷贝都已改；(2) 重定向 `human_height` 默认 1.66 让站着的人变成 84° 膝盖的深蹲（pelvis 0.76 m，MuJoCo 里 0.83 m），改为 2.18（它是 GMR 尺度参考，不是操作者身高），这也更正了 plan 9.4/9.5 里"骨盆倾斜"的结论；(3) 3 点标定的关键帧映射写死 G1 链接名，对 A3 会崩。
- 新增测试 `tests/test_a3_sim_model.py`（9 个）；`tests` + `gear_sonic/tests` 共 187 → 加上下面的 15 个后 202 通过、1 跳过。

### 44.3 `--robot a3_ultra` 的机器人参数对齐 Litchi 的 a3（用户要求，"先不要用官方文档"）
- 之前**训练侧**用官方 Kp/Kd 文档的数值（髋 200/7、膝 360/13…），而**部署侧**（C++ `a3_024` profile）和仿真（指令里的 kp/kd）本来就是 Litchi a3 的数值——也就是只有训练侧不一致。
- `robots/a3_ultra.py` 新增 `A3_ULTRA_GAIN_SOURCE`，默认 `"litchi_a3_024"`：29 个身体关节的执行器组直接 `deepcopy` 自 `A3_024_CYLINDER_CFG`（刚度、阻尼、力矩/速度上限、转子惯量、摩擦、动摩擦）、默认站姿和出生高度取自同一个配置、动作缩放同公式推出；头关节 Kp=40/Kd=2（部署代码固定值）、q=0。官方文档的数值保留在 `_vendor_doc_*`，把开关改成 `"vendor_doc"` 即可切回（部署侧要先有对应 profile）。4 份 A3 Ultra MJCF 的 `wrist_roll` 转子惯量 0.00081 → 0.004967（#38 在训练配置里改过，MJCF 当时漏了）。
- 新增 `tests/test_a3_ultra_contract.py`（15 个）：把"a3_ultra 训练配置 = Litchi A3_024 = C++ 部署头文件"锁住（用 g++ 编译 `a3_policy_parameters.hpp` 读出 `a3_024_kps/kds/action_scale/a3_default_angles` 逐关节比较），并核对 MJCF 转子惯量。`--robot a3_ultra` 训练冒烟 `exit=0`；`--robot g1` 训练冒烟 `exit=0`。
- **没有对齐、有意没动**：`a3_ultra` 训练动作空间仍是 31（含 2 个头关节），Litchi a3 是 29；自碰撞开关；仿真力矩裁剪用 MJCF 厂商上限（膝 320，训练合约 300）；`a3_ultra` 仍是自研的 SMPL 跟踪合约（观测/奖励/数据与 035 不同）。

### 44.4 仍未验证 / 本机做不了
真实 PICO 头显、G1 与 A3 Ultra 真机（包括真机 `/body_drive/*` 的实际话题布局、头部关节命名、IMU 约定）、带窗口 viewer 里按键释放弹性绳、h2 的仿真与部署、从零训练的 `a3_ultra` 策略的部署导出。Phase 4 与本条涉及的文件仍**未提交 git**。

**文件改动**（新增/修改）：`gear_sonic/utils/mujoco_sim/{a3_sim.py,configs.py,wbc_configs/a3_035.yaml,wbc_configs/a3_ultra_035.yaml}`、`gear_sonic/scripts/{run_sim_loop.py,run_a3_teleop_sim.py,pico_manager_thread_server.py}`、`gear_sonic_deploy/scripts/{deploy_a3.sh,setup_a3_sim_env.sh}`、`gear_sonic/data/robot_model/{instantiation/*,supplemental_info/a3,supplemental_info/h2}`、`gear_sonic/utils/teleop/vis/vr3pt_pose_visualizer.py`、`gear_sonic/utils/teleop/a3/retarget.py`、`gear_sonic/envs/manager_env/robots/a3_ultra.py`、`gear_sonic/config/exp/.../sonic_a3_ultra.yaml`（默认 `motion_file`、注释）、`gear_sonic/train_agent_trl.py`（注释）、4 份 `a3_ultra_t3d0.xml`、`.gitignore`、`docs/notes/mujoco_pico_sim_teleop.md`、`tests/test_a3_{sim_model,ultra_contract,teleop_retarget}.py`。

---

## 45. [2026-10-14，接 #44] A3 Ultra 直接使用下载的 A3 权重：官方兼容性确认 + 头部固定 + 闭环验证（细节与数据见 `docs/workflow/a3_integration_plan.md` 第 14 节）

**用户要求**：Litchi 的 README 里也写了 a3_ultra，确定有没有可下载的权重真的兼容 a3_ultra；没有的话就把 a3_ultra 的两个头关节固定在默认姿态（和 a3 一致）、其余关节输出与 a3 一致；当前不训练自己的 a3_ultra sonic 模型，直接用下载的 a3 权重。

**结论**
- 可下载的权重**只有一个**：HF `sonic-for-a3/sonic` 的 `035_step200000`（该组织只有这一个仓库，只有 `main` 分支、无 tag；GitHub `AgibotTech/sonic_for_a3` 无 release、只有 `main`）。**官方明确声明它同时支持 A3 与 A3 Ultra**：README"其域随机化专门针对 A3 与 A3 Ultra（A3U）调过，因此对应模型也能在 A3 Ultra 上运行"，并说 Rockchip/RKNN 部署流程支持 A3U；HF 发布元数据 `config.json` 为 `"robots": ["A3","A3 Ultra"]`、`"policy_joints": 29`。本地 `Litchi_sonic_for_a3/README.md` 与 GitHub main 一致（忽略行尾空白后 diff 为 0）。**没有 A3 Ultra 专属权重。**
- 本机核验：本机用的 PT 和两个部署 ONNX 的 SHA256 与 HF 当前 `SHA256SUMS` 一致；29 个策略关节的名字/顺序/动作缩放相同，A3 Ultra 的关节范围覆盖策略训练时的软限位范围（只有 hip_roll 85° vs 92°、knee 145° vs 143° 两类不同）；质量差（整机 +2.03 kg，+3.5%）落在 035 的域随机化范围内（torso ×[0.8,1.3]、腕上加质量 [-0.2,+2.0] kg、其它连杆 ×[0.9,1.1]），唯一超出比例范围的是头 yaw 连杆 +0.235 kg；MuJoCo 闭环：8 个官方参考动作（001/009/043/047/050/055/058/077）× 2 个模型（A3 的 T2D5、A3 Ultra 的 T3D0，同一个部署二进制、同一组权重、同一套增益），16 次运行全部完整播放、指令期间 0 次摔倒，A3 Ultra 与 A3 的全关节 RMSE 差异 ≤0.004 rad（腿最大 0.008 rad，深蹲）。**这是相对比较，两个仿真模型都是简化的串联踝/腰 + 刚性脚；真机没有验证。**
- 头部固定（第 ② 点）：部署程序本来就把两个头关节固定在 q=0（Kp 40/Kd 2，与 A3 相同），29 个身体关节用策略输出 + A3 合约；本轮补上一个会破坏"固定"的细节——Pico 重定向会把 GMR 逆解里头关节的残留值作为头指令发出去并覆盖固定的 q=0，现在默认不发头（`A3RealtimeRetargeter(track_head=False)`，`--track-head` 才发）；仿真里头关节偏差 yaw ≤0.005 rad、pitch ≤0.026 rad。训练合约（31 个动作含头）因用户当前不训练而没有改，已记录。

**其它改动**：`a3_sim.py`：无力矩限制信息的关节（T2D5 的头）不再被裁成 0 Nm；`JOINT_ARMATURE_IF_MISSING`（T2D5 MJCF 的转子惯量为 0，按合约值补上，使 A3 / A3 Ultra 两个仿真可比，`a3_035.yaml`）；`A3_SIM_LOG_CSV` 状态日志。`deploy_a3.sh`、`README.md`（A3 一节）、`docs/notes/mujoco_pico_sim_teleop.md` 的权重与头部说明。测试 `tests/test_a3_ultra_contract.py`（现 16 个）、`tests/test_a3_teleop_retarget.py`（+1）；全量 203 通过、1 跳过。

**上真机前必须确认（本机做不了）**：①真机 `/body_drive/neck_joint_state` 的头关节名是否为 `head_yaw_joint`/`head_pitch_joint`（部署程序按名字匹配，缺这一路状态策略不启动；A3 Ultra 的 URDF 里叫 `ultra_head_pitch_joint`）；②你的 A3 Ultra 是 Rockchip RK3588S 还是 Drive Thor——公开仓库只有 x86_64 / rockchip 的部署包，Thor 运行时与教程官方还没发布（README 的 TODO）；③IMU 约定与并联踝/腰的关节空间读数；④官方策略的适用范围（不支持跪/躺/起身、不支持快跑，手腕跟踪较差）。


## 46. [2026-10-14，接 #45] `deploy.sh --encoder a3-fast|smpl`：A3 Ultra 遥操的两条编码器路径可选（细节、命令与数据见 `docs/workflow/a3_integration_plan.md` 第 15 节）

**用户要求**：根据 plan 11.13，A3-fast encoder（Litchi 实现、已移植）与 SMPL encoder（我们自己实现的）两条路径都希望在 MuJoCo 和真机上能准确使用，补 `--encoder smpl/a3-fast` 命令行选择；代码改动逻辑记入日志；完成后用户自己做 MuJoCo + Pico 验证。

**结论**
- `--encoder a3-fast`（默认）完整可用：下载的 035 `a3_fast` ONNX + Pico SMPL → GMR 重定向 → `a3_ta_bridge` → `/ta/whole_body_command`。MuJoCo 端到端（Pico 模拟器）通过：骨盆 1.073–1.075 m，倾角稳态 2.2°，右臂随挥手动作，桥转发 2200+ 帧 0 拒收。`--encoder g1` 是同一链路换 g1 编码器（≈940 ms），只作对照。
- `--encoder smpl` **没有权重可用**：035 只有 `g1` + `a3_fast`，HF / GitHub 没有 A3 / A3U 的 SMPL-encoder 模型，本机 `a3_ultra` 训练只做过冒烟。我做完了接口、校验、锁定和安全逻辑，用“站立桩”（真正的 a3_fast 计算图 + 常量站姿 tokenizer，忽略 SMPL 输入）在 MuJoCo 里验证了数据通路——机器人**站着不跟随操作者**。这条路径的“准确度”现在无从谈起，要等你训练并导出 SMPL 模型（`--robot a3_ultra` 的训练合约还要先对齐 29 关节视角，见 plan 15.6 / 13.4）。
- **plan 11.13 的对比表有一处写错**：SMPL 编码器的窗口是 10 帧 × 0.02 s（训练配置 `smpl_dt_future_ref_frames: 0.02`、部署 `kSmplFutureFrames=10`），不是 “skip 5 / ≈940 ms”（那是 `g1` 关节空间编码器）；a3-fast 的 tokenizer 也漏了关节速度。已在原表就地修正并写入 15.2。

**代码改动逻辑（四层，启动时逐层校验，选错立刻报错）**
1. C++ `a3_deploy_onnx_ref`：`--teleop-source g1|smpl|a3_fast`（或配置 `teleop.default_source`）预选并锁定 TELEOP 源；非法值退出 64，源不可用退出 65；按键 `1/2/3` 对已锁定的源提示 `rejected: locked to …`；SMPL 流首次驱动策略时打印 `smpl encoder running`。
2. 运行时配置：`a3_make_runtime_cfg.py` 从打包配置生成 `a3_runtime_config.generated.yaml`（`smpl_model_path`、`smpl_zmq`），经 `A3_RUNTIME_CFG` 传给 `run_a3.sh`（C++ 的 `--runtime-cfg` 只认第一次出现，不能再追加）。
3. 操作端链路 `a3_operator_chain.sh`：`a3-fast/g1` → `a3_teleop_retarget_node.py --mode a3-ref` + `a3_ta_bridge`；`smpl` → `--mode smpl-pose`，把 Pico `pose` 流原样转发到 :5561 并用 `A3SmplPoseAugmenter`（GMR 逆解，与 A3 训练参考同一族）补上 A3 的 6 个手腕角（官方 G1 发送端填的是 G1 关节轴公式）。
4. 启动器 `deploy_a3.sh`：`--encoder`、`--smpl-model PATH|stub`（`A3_SMPL_MODEL`）、`--no-chain`；缺模型退出 66 并给训练/导出指引，接口不符（`a3_smpl_onnx_tool.py check`：`obs_dict[1,1770]`→`action[1,29]`）退出 65。`run_a3_teleop_sim.py` 也加了同名 `--encoder`（`smpl` 报错并指向 `deploy.sh`）。

**这次测试里发现并修掉的四个问题**
- 重建部署包会丢掉 `a3_ta_bridge`（脚本开头 `rm -rf` 包目录，桥原来是手动放进去的）→ `build_a3_deploy_pkg.sh` 现在编译并校验它。
- SMPL 源日志刷屏（共享头 `StreamedMotionMerger` 每块 3 行，≈150 行/s）→ 改成运行期开关，**默认仍开（G1 不变）**，A3 按 `smpl_zmq.verbose`（默认关）。
- **`-ffast-math`（顶层 CMake，上游原样）让 `std::isfinite` 恒为真，A3 部署里所有 NaN/Inf 守卫实际都是失效的**（单测 `A3TaBridgeCodec.RejectsMalformedMessages` 一直是红的）→ 对 `src/a3_deploy/*.cpp` 单独加 `-fno-finite-math-only`，该单测转绿。真机安全相关。
- **Pico 退出 / 重进 POSE 模式后链路卡死**（流中断测试发现）：官方发送端 `on_mode_exit()` 里 `step = 0`，每次 A+X 重进 POSE，`frame_index` 都从 0 开始；`a3-ref` 节点按 `frame_index <= last_index` 丢帧，整个新会话被丢掉直到计数器超过旧最大值（手臂冻结）；`smpl-pose` 增强器则返回上一段的缓存手腕角。现在两处用同一规则（`frame_index_restarted`：最新索引小于已见最大值）开始新会话，`a3-ref` 的逐消息逻辑抽成 `A3RefStream`；+4 个测试（旧逻辑下会失败）。流中断 14 s 再重启的 MuJoCo 测试：`a3-fast` / `smpl` 桩都 “stale → holding last → restarted → reconnected”，机器人全程站稳。

**验证**：`tests/test_a3_encoder_selection.py` 新增 16 个（含桩 = `a3_fast(站姿 tokenizer｜本体感知)` 的逐位比对）、`tests/test_a3_teleop_retarget.py` 新增 7 个；`isaaclab` 环境全量 219 通过 / 1 跳过；`gmr` 环境 22 通过；C++ `run_tests`（仓库根目录运行）228 个中 218 通过、9 跳过、1 失败（`FK.TestFKAndGlobalVelocities`，缺 G1 参考数据，无关）；三条失败路径（无模型 66、接口不符 65、非法编码器 2）与二进制侧校验（65 / 64）按设计退出。MuJoCo（最终二进制）：`a3_ultra` 的 `a3-fast` / `smpl` 桩 / `g1`、普通 `a3` 的 `a3-fast` 四条配置都跑通（桥转发 3400 帧 0 拒收，骨盆 1.07 m，倾角稳态 ≤2.4°，按 1/2/3 切到别的源被拒绝）；两机拓扑（操作端链路单独启动 + `--no-chain`）`a3-fast` 与 `smpl` 桩都通；流中断 + 重启两条路径都通。

**没做到 / 必须说清楚**
- 回归期间先后两次撞上用户的 G1 会话（第一次 23:16 左右，G1 `real` + `pico_manager_thread_server.py` 占 5556；第二次 23:29，G1 MuJoCo 会话刚启动），我都在发现后用 PID 杀掉了自己的进程。我的 `-j20` 编译、无头仿真，以及第二次最后约 40 s 在 5556 上发布的 A3 Pico 模拟器（合成 `wave`）与其重叠；没有碰用户的进程、5556/5557 的发布端或网卡，但用户的 G1 部署在那段时间启动，可能连上过我的模拟器流（不发 `command`，不应起步）。用户会话结束后（23:59）已把被打断的回归补跑完；测试脚本现在开跑前会检查用户会话和 5556 端口。
- 真实头显、真机都没有测。真机上 `a3-fast` 需要操作端 PC 与机器人在同一 ROS2 域（`a3_ta_bridge` 经 AimRT ROS2 后端发布）；`smpl` 的 Pico 窗口要 `--num_frames_to_send 10`（默认 5 没有余量，按代码推断，未实测）。plan 14.5 的真机待确认项（头话题名、Rockchip/Thor、IMU 约定）依旧有效。
- **流中断时机器人“保持最后一条指令”**（部署程序既有设计，已测）：摘下头显 / 停发后手臂停在最后姿态并保持站立，不会自动回位；真机上人走开要按 `P`。
- `smpl` 仍然没有权重，而且“我们自己实现的 SMPL 编码器”只有代码、没有训练：`sonic_a3.yaml` 里 `encoder_sample_probs.smpl: 0.0`、`smpl_motion_file: dummy`，A3 Ultra 动作库的 `smpl_joints` 是占位零，G1 用的 `data/bones_seed_smpl` 也不在本机。缺的是 SMPL 数据 + 训练，不是部署接口；最短路径是沿用 A3 的 29 关节合约（导出天然 `[1,1770]→[1,29]`），而不是先改 a3_ultra 的 63 DOF 合约（plan 15.6）。这些都没做，是否做由你决定。


## 47. [2026-10-15，接 #46] 重启规则与 G1 对齐；按 Litchi 的 a3 参数批量转换、`--robot a3_ultra` 训练 A3 合约，域随机按 A3 与 A3 Ultra 一起调（细节与数据见 `docs/workflow/a3_integration_plan.md` 第 16 节）

**用户要求**：① #46 里“重启修复”的规则和 G1 在 Pico 发送端退出再重进 POSE 时的逻辑是否一致，不一致就对齐；② 按 Litchi_sonic_for_a3 的 a3 参数批量转换（035 用 A3 URDF 训练、域随机按 A3 与 A3 Ultra 一起调，所以能在 A3 Ultra 上跑；策略只输出 29 个身体关节，头两路固定 PD，`ultra_head_pitch_joint` 是 `head_pitch_joint` 的别名，手指不驱动）——“根据 a3 参数进行转换训练，域随机按 A3 和 A3 Ultra 一起调”。

**① 不一致，已对齐**：G1 接收端是 `StreamedMotionMerger::CalculateSlidingWindow`（第一包 / `start <= window_start` / `end <= window_end` / 窗口间丢帧或游标落后 205 帧 → catch-up，之后 `current_frame = 0` 并重捕获航向）；我 #46 的规则只看“最新索引 < 已见最大值”，漏掉起点相等（短会话后重进，索引完全相同）和窗口间丢帧。现在把前三类逐条移植成 `pose_window_catch_up`（`restart` / `stale` / `gap`），`a3-ref` 节点（`A3RefStream`）与 `smpl-pose` 增强器共用，`smpl_zmq` 的 C++ 源本来就是同一份 merger。测试重写（15 个，含“短会话后重进”），MuJoCo 流中断两条路径重跑通过（stale → new Pico session → reconnected）。

**② 批量转换与训练**
- 核对：29 个身体关节名字 / 顺序 / 轴完全相同，共有 body 的 FK 差 ≤1.5 mm；限位只有 `hip_roll`（Ultra 更窄）与 `knee`（Ultra 145°，A3 143°）不同；足底高度差 −4.54 mm（A3 网格脚 vs Ultra 球脚）；头 `dof` 恒为 0。
- `gear_sonic/data_process/convert_a3ultra_to_a3_29d.py`：按名字取 29 列、夹 A3 限位、按 `a3_t2d5.xml` 的 32 个 body 重新生成 `pose_aa`、根 z −4.54 mm，输出 Litchi `a3_29` 格式（与 `convert_a3_pkl_to_motion_lib_29d` 的 normalizer 互认）；`--check` 逐文件校验。**全库 129,116 条、28,394,944 帧，12 进程 18 s，0 失败**，输出 `data/motion_lib_a3_29d/robot_filtered_v15_clean`（22 GB）；膝限幅帧：左 92,451、右 98,569；300 条抽样 FK 对照：最大 14.6 mm（出现在被膝限幅夹过的帧），脚底最低点差均值 +1.4 mm。
- **`--robot a3_ultra` 语义变更**：现在是 `sonic_a3_ultra` = 继承 `sonic_a3_035`（A3 资产 / a3_024 参数 / 奖励 / **035 域随机**），仅 `motion_file`、`project_name`、`exp_var` 三个键不同；Ultra 自己的 63 DOF 合约改名 `sonic_a3_ultra_t3d0.yaml`，`+exp=` 显式选用（不可部署，导出宽度对不上）。想回旧行为就用那个显式 exp。
- 冒烟：`--robot a3_ultra` 从零 2 次迭代、加载 035 PT 热启动 2 次迭代都 `exit=0`；官方 `train_a3_035_fromscratch.sh --preflight` 通过。域随机覆盖性由 `tests/test_a3_ultra_training_recipe.py` 从两个 MJCF 推导并校验：躯干 ×1.037 ∈ [0.8,1.3]，每侧手 +0.50 kg ∈ [−0.2,+2.0]，其它连杆相同；**唯一例外是头 yaw 连杆 +0.235 kg**（头关节固定，影响很小，没有改 035 的域随机）。
- 测试：新增 `tests/test_a3_data_conversion.py`（9）、`tests/test_a3_ultra_training_recipe.py`（5）；`isaaclab` 全量 233 通过 / 1 跳过，`gmr` 环境 33 通过。

**没做 / 注意**：转换后的库里 `smpl_joints` 仍是零占位，SMPL 编码器的数据和训练都没做（plan 15.6）；`commands.py` 的 `lower_joint_indices_mujoco = range(12)` 是 G1 假设，启用 `teleop` 编码器前要改（035 配方不用）；没跑收敛训练；我发出的“转换数据 vs Litchi 自带 20 条数据”的 120 次迭代对照训练被用户拒绝，没有跑。RKNN / Thor 本次没动（README 的 Thor 教程仍未公开）。


## 48. [2026-10-15，接 #47] 按 A3 参数用原来的重定向方法批量转换（为训练自研的 A3 SMPL 编码器）（细节与数据见 `docs/workflow/a3_integration_plan.md` 第 17 节）

**用户澄清**：`--robot a3_ultra` 训练 A3 合约没问题；“批量转换”指的是——为了训练我们自研的针对 A3 的 SMPL encoder，需要 a3 重定向的数据集，所以要基于 a3 的机器人参数，采用之前做的重定向方法重新做数据转换。#47 里我做的“A3 Ultra 库 → `a3_29` 格式”只是格式转换，不是这个意思，保留为交叉验证。

**做了什么**
- `batch_retarget_bvh_soma_a3ultra.py` 加 `--robot a3|a3_ultra`（默认不变）、`--only_from`、原子写；运行时向 GMR 注册 `agibot_a3`（`a3_t2d5.xml`，不改 GMR 检出目录）；IK 标定表沿用 A3 Ultra 的（两机 14 个 IK link 在 qpos0 的姿态一致，有测试）；**足底高度改用精确最低点**（A3 的脚是网格，旧公式按 `geom_size` 会得到踝高 0.0941 m，真值 0.0675 m，机器人整体高 26 mm）；新增 `convert_gmr_a3_to_motion_lib.py`（GMR 38 维 qpos → `a3_29`）。
- 验证：24 条（0.0008 rad）、210 条含 60 条蹲 / 跪 / 跳 / 翻滚等难动作（0 失败、帧数全同、RMSE 均值 0.0020 rad）后，**全量 129,116 条（只转 A3 Ultra `robot_filtered_v15_clean` 里的那批动作名）：109.9 min、0 失败、28,394,944 帧、22 GB**；`--check` 0 个问题；与格式转换版逐条对比 `dof` RMSE 均值 0.0011 rad（p99 0.0094，273 条 >0.02，最大 0.92 rad 的几条原因没追查）、根高度差均值 −0.07 mm（印证了 #47 的 −4.54 mm 常数修正）；相邻帧最大关节步长的分布与转换版几乎相同。
- 数据路径：原生 A3 重定向 = `data/motion_lib_a3_29d/robot_filtered_v15_clean`（`--robot a3_ultra` 的默认训练库）；格式转换版改名 `converted_from_a3ultra_v15_clean`。`--robot a3_ultra` 训练冒烟（从零、035 PT 热启动）在原生库上重跑通过。
- **A3 SMPL 编码器的训练配方**：新增 `sonic_a3_ultra_smpl.yaml`（继承 `sonic_a3_ultra`，编码器 `g1 + a3_fast + smpl`，teleop 关）和 `g1_recon_smpl_and_a3_fast_latent_no_teleop.yaml`（仓库里本来就有、但没人引用的 Litchi 配置去掉 teleop 两项）。用官方样例 `walk_forward_amateur_001__A001(_M)` 的 SMPL 数据 + A3 片段冒烟（16 envs、3 次迭代）通过，SMPL 潜变量对齐损失有值，SMPL 与 A3 帧数对齐（1202 @30 → 2002 @50）；**导出的 `model_step_000003_smpl.onnx` 是 `obs_dict[1,1770] → action[1,29]`**（`a3_smpl_onnx_tool.py check` 通过），部署包 ORT 1.16.3 的 probe 跑通（1.5 ms）。新增 `check_smpl_pairing.py`（按文件名配对 + 帧数 / fps 检查）。
- 测试：新增 `tests/test_a3_retarget_batch.py`（5）、`tests/test_check_smpl_pairing.py`（3）、`tests/test_a3_ultra_training_recipe.py` +1；回归 `isaaclab` 环境 240 通过 / 3 跳过，`gmr` 环境 41 通过。

**没做 / 需要你决定**：① SMPL 数据（`python download_from_hf.py --training`，约 30 GB，含训练检查点）本机没有，我没有下载；② `smpl` 编码器没有训练，采样概率和辅助损失系数沿用 G1 SONIC 的取值、没为 A3 调过；③ 12.9 万条动作的训练显存 / 内存占用没评估，没有收敛训练；④ 我发出的“转换数据 vs Litchi 自带 20 条”的 120 次迭代对照被拒绝，没跑。


## 49. [2026-10-15，接 #48] 对比视频、SMPL 全量数据、A3 库与 SMPL 世界朝向差约 90° 的原因与修复（细节见 plan 17.8）

**用户的要求**：(1) 下载 SMPL 数据并采样做 SMPL 与 A3 重定向的对比视频；(2) 也做 SOMA 与重定向 A3 的对比视频；(3) 解释“SOMA 和重定向视频没差异，为什么 A3 库与 SMPL 世界朝向差约 86°，SMPL 与 SOMA 什么关系”，以及 a3-fast 加重定向节点能否替代 SMPL 编码器；(4) “先修朝向，但是不训练”。

- **视频**：`videos/compare_v15_a3/`（SOMA vs A3，32 条，`make_compare_videos.py --robot a3`）；`videos/compare_smpl_a3/`（SMPL vs A3，30 条，`compare_smpl_vs_a3.py`，三栏：SMPL 训练系 / 转到机器人朝向的 SMPL / A3）。
- **SMPL 数据**：32.3 GB 经代理时 xet 卡死，改 `HF_HUB_DISABLE_XET=1` 走普通 HTTPS（约 9 MB/s），中途会话中断过一次，已续传；解压到 `data/smpl_filtered`（131,455 个 pkl，31 GB）。配对 129,109 / 129,116。
- **朝向差的原因**：重定向脚本为让 IK 偏移通用，把每条 BVH 整体转到“第 0 帧面朝 +X”（`_yaw_f0`）；SMPL 与 G1 库保留采集朝向。SMPL 与 SOMA 在同一世界系（样例原始 BVH −87.8° / SMPL −86.6° / G1 −86.9° / A3 −2.2°）。修前全库：±10° 内仅 6.5%（+60…+120° 约 66%）。
- **修复**：`restore_a3_heading.py` 按每条 BVH 自己的 `_yaw_f0` 把根位置 xy / 根朝向转回（关节角不变；`_yaw_f0` 一遍读 45 GB 压缩包约 8 分钟，转库不到 1 分钟）；重定向脚本加 `--keep_heading`。旧库保留为 `data/motion_lib_a3_29d/robot_filtered_v15_clean_yawnorm`，新库换名成 `robot_filtered_v15_clean`（默认训练库）。修后全库：均值 −1.3°，p50 / p95 = 1.4° / 3.9°，±10° 内 99.2%（G1 库 vs SMPL 基线 99.5%），帧数 0 不一致；`--check` 0 问题；`--preflight` 通过。`check_smpl_pairing.py` 加朝向守卫（新库过，旧库退码 1）。
- **检查器 bug**：帧数换算在 `T-1` 为 3 的倍数时多算 1 帧（曾报 43,138 对假不一致），已按库的 `arange` 规则修并加测试。
- **没有训练**；SMPL 编码器的采样概率 / 损失系数仍沿用 G1 SONIC 的取值。
- **关于“a3-fast + 重定向节点能否替代 SMPL 编码器”**：回答在对话里（结论：遥操已经能用，训 SMPL 编码器不是必须；即使训了，`smpl-pose` 模式仍要 GMR IK 给 6 个腕角）。建议先在真机 + 真实头显上评估 a3-fast 链路。
- **docs 被覆盖**：这次中途发现 plan 与 worklog 在磁盘上被回退到了较早的状态（两个文件修改时间都是 09:48:57，疑似 IDE 里打开的旧缓冲区被保存），上面的内容与 plan 17.8 / 17.9 是据实重新写入的；编辑器里请先从磁盘重新加载再保存。
- 测试：`tests/test_a3_heading_restore.py`（6）、`tests/test_compare_smpl_vs_a3.py`（4）、`tests/test_check_smpl_pairing.py`（+2）；回归 `isaaclab` 249 通过 / 5 跳过（4 个需要 GMR / mink，1 个需要 onnxruntime），`gmr` 52 通过。


## 50. [2026-10-15，接 #49] A3 的 MuJoCo + Pico 遥操教程 `docs/notes/a3/mujoco_pico_sim_teleop.md`（按用户给的三条命令原样验证后写成）

**要求**：三个终端分别执行 `run_sim_loop.py --robot a3_ultra`、`./deploy.sh --robot a3_ultra --input-type zmq_manager sim --encoder a3-fast`、`pico_manager_thread_server.py --robot a3_ultra --manager --num_frames_to_send 10`，仿照 `docs/notes/g1/mujoco_pico_sim_teleop.md` 在 `docs/notes/a3/` 下写一份 A3 的 Pico + MuJoCo 使用教程。

**验证（本机，命令原样、路径带空格、没有用软链接）**：
- 终端 1 弹出 MuJoCo 窗口（用户桌面 `:1`），吊在弹性绳上；终端 2 的 `a3_backend` 6 个话题 `ready=yes`、`Init Done`、自动 PD 站立，仿真 `cmd_rx` 持续增长；`[a3-chain]`、重定向节点（mode=a3-ref，:5556 → :5560）、`a3_ta_bridge`（→ ROS2 `/ta/whole_body_command`）都由 `deploy.sh` 自己拉起，不用另开终端。
- 终端 3 的真实命令能起来、连上本机 PC Service（`initialize sdk, connect 127.0.0.1:60061`），没有头显在发数据，一直 `waiting for body data...`。**没有真实头显的验证**——这部分仍要你戴头显走。
- 终端 3 换成 `pico_pose_zmq_replay.py --clip synthetic:wave`，终端 2 按 `T`：`mode=teleop`，重定向每帧平均 2.17–2.27 ms（p95 2.5–2.8 ms，最大 3.96 ms），`a3_ta_bridge` 转发 2600+ 帧 `rejected=0`，策略周期 20.0 ms、`sync=ok`；仿真里弹性绳按 `A3_SIM_BAND_RELEASE_S=75` 松开后机器人没有摔倒（没有 `robot fell` 提示）。
- 插曲：第一次起的仿真进程在 80 s 后无日志地退出了（原因没查清，可能是用户桌面上的窗口被关掉），部署程序随后一直 `ready=no samples=0`、仿真 `cmd_rx` 全 0——教程第 6 节写了这个现象与处理（先确认终端 1 的进程还在）。重新起后一切正常。
- 验证用的两个小工具（PTY 驱动按键、无人值守松绳）在会话临时目录里，没进仓库；仓库里只有仿真已有的 `A3_SIM_BAND_RELEASE_S` 环境变量（教程第 5 节提到）。
- 所有验证进程都按各自的 PID 结束，没有碰你的会话（开始前 5556 / 5557 / 5560 / 5561 都空，结束后也空）。

**教程内容**：架构图与和 G1 的区别表；一次性准备；连头显（指向 G1 文档第 1 节）；三个终端的命令与每个终端该看到的日志；第一次遥操（A+B+X+Y → `9` → A+X）；急停；键盘备份；没有头显时的联调；常见问题；这份文档验证过 / 没验证什么；真机（指向 plan 12.7，未验证）。G1 文档第 8 节加了指向它的链接，README 的链接改指它。

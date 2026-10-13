# A3 集成规划：把 Litchi_sonic_for_a3 并入 GR00T-WholeBodyControl

**状态**：范围已拍板（见下）；C++ 部署框架编译验证已完成（见下方专节）；**Phase 1（训练侧文件迁移）已完成并逐项验证**，完整权威记录见 worklog #40（自包含，面向冷启动 agent）；**Phase 2（checkpoint 迁移+数值对比）资产部分已完成验证**，完整记录见 worklog #41；**IsaacLab 端到端验证（真正起 Hydra/`UniversalTokenModule`/`ManagerBasedRLEnv`，strict 权重加载 + 20 步真实 rollout，两仓库均 exit 0）已完成**，过程中发现并修复了 3 处 Phase 1 未覆盖的真实遗漏（`commands.py` 缺 `coactivate_g1_when_teleop`、`commands.py` 的额外 DOF 处理对 A3 被动脚关节不通用、`modular_tracking_env_cfg.py` 整个文件此前从未被移植导致 `robot_mapping`/`asset_path`/`active_joint_names` 调度逻辑缺失），完整记录见 worklog #42（自包含，含遗留项清单）。Phase 3（C++ 部署集成，backend 已知可行）、Phase 4（VR 实时遥操）尚未开始。
**目标**：把 `Litchi_sonic_for_a3`（SONIC 团队针对 AgiBot A3 的开源适配，含 035/step200000 训练好的 checkpoint）的训练与部署逻辑、以及训练好的模型，迁移进本项目 `GR00T-WholeBodyControl`，使仿真和真机部署都能用同一个 `--robot` 命令行参数在 `g1` / `h2` / `a3` 之间手动切换遥操对象。

## 已拍板的决策（2026-10-13）

1. **Phase 4 做"完整范围"**：不止参考动作回放，要新写"VR 实时姿态 → A3 关节空间轨迹"的实时重定向模块，接入已验证能在真机上跑的 AimRT `/ta/whole_body_command` 路径。这是本次集成里工作量最大的一块净新增代码。
2. **CLI 机器人命名统一为 `a3`**：不再区分 `a3`/`a3_ultra` 两个 CLI 选项——因为两者运动学/质量/电机参数基本一致（worklog #37），用一个名字简化命令行。**但真机部署对象仍然是 A3 Ultra 实体**：训练/仿真侧用 Litchi 的 A3 (T2D5) 资产和训练合约（因为 035 checkpoint 就是在这套合约上训出来的），C++ 部署侧的 `robot_io` 后端要对接 A3 Ultra 真机的实际硬件接口（具体是复用 Litchi 的 `CreateA3AimrtBackend()`，还是需要为 A3 Ultra 单独写一个 backend，取决于 A3 Ultra 真机的 AimRT topic 布局是否和 base A3 一致——这是 Phase 3 要核实的具体问题，不是本次决策的范围）。本项目已有的 `a3_ultra.py`/`sonic_a3_ultra.yaml`/`a3_ultra_t3d0.xml`（从零训练 A3 Ultra 用）作为独立的"自研训练合约"保留，不在这次合并掉，但不再通过 CLI 暴露成单独的 `a3_ultra` 选项——具体怎么共存（两份训练合约都留着、`--robot a3` 默认指向哪一份）在 Phase 1 实施时再定。
3. **直接拷贝文件**：不用 git subtree/submodule，Litchi 仓库里需要的文件直接拷进本仓库对应位置。
4. **先在隔离环境验证编译**：进行中，见下方"C++ 部署框架编译验证"一节。

## C++ 部署框架编译验证（进行中）

在 `Litchi_sonic_for_a3/gear_sonic_deploy`（独立于本仓库，不影响 GR00T-WholeBodyControl 现状）里直接验证编译，不用 Docker（机器上没装 Docker，装 Docker 本身是更大的系统改动，没有必要——下面证明直接在隔离 conda 环境里编译完全可行）。

**Stage A（默认选项，不含 AimRT）**：`cmake -S . -B build -DBUILD_SRCS=ON` 后 `cmake --build . --target a3_deploy_onnx_ref`，**一次性编译通过**。这部分覆盖了 ONNX 推理、ZMQ、locomotion planner 等通用层，以及全部 A3 专属但不依赖 AimRT 的代码（obs builder、policy driver、motion library、csv 回放、eval/telemetry、yaw 对齐、安全停机等）——证明这部分代码本身没问题，复用的也正是本项目 G1 部署已经在用的依赖。

**Stage B（完整选项，含 AimRT + ROS2，Phase 4"完整范围"实际需要的路径）**：这一步遇到并修复了三个真实问题，记录下来避免 Phase 3 正式迁移时重复踩坑：

1. **`empy` 版本不兼容**：conda-forge 默认装的 `empy 4.2.1` 把 ROS2 Humble 的 `rosidl_adapter` 代码生成器跑挂了（`AttributeError: module 'em' has no attribute 'BUFFERED_OPT'`）。ROS2 Humble 的消息生成工具链是针对旧版 `em`/`empy 3.3.4` API 写的，`empy` 4.x 是不兼容的重写版本。**修复**：隔离环境里把 `empy` 降级到 `3.3.4`（conda-forge 上有）。
2. **缺 `libacl` 头文件**：Iceoryx（AimRT 的共享内存 IPC 插件依赖）需要 `sys/acl.h`，本机只有 `libacl1`（运行时库）没有 `libacl1-dev`（头文件），且没有免密 sudo 装不了系统包。**修复**：改用 conda-forge 的 `libacl` 包（它把头文件和库都打包在一起，不像 Debian 拆成 `-dev` 子包），装进隔离环境后设置 `CPATH=$CONDA_PREFIX/include` 让系统编译器能找到它——全程没有动系统级依赖。
3. **路径带空格导致 GNU Make 生成的构建规则出错（重要，会影响正式迁移）**：本仓库所在路径是 `/home/user/YZY/HumanWBM/Demo/RL controller/...`——"RL controller"中间有空格。protoc 的自定义构建命令（生成 `aimdk/protocol/*.pb.cc`）在 CMakeLists.txt 里参数是正确加了引号的（`--proto_path="${A3_TA_PROTO_ROOT}"`），但 **CMake 的 "Unix Makefiles" 生成器在处理带空格路径的自定义命令时有已知限制**，实际跑起来 protoc 收到的路径参数被插入了错误的反斜杠转义（`RL\ controller`），导致找不到目录，protoc 报 "File does not reside within any path specified using --proto_path"。**修复**：改用 Ninja 生成器（`cmake -G Ninja ...`）而不是默认的 Unix Makefiles，Ninja 对带空格路径的自定义命令处理更规范，问题消失。
   - **根因排查更正**：一开始怀疑是 Unix Makefiles 生成器的限制，换成 Ninja 生成器后**同样的 `\ ` 转义错误原样出现**，说明和生成器无关，是 **CMake 本身在处理 `add_custom_command` 里 `--proto_path="${VAR}"` 这种"引号只包住变量值、不包住前面的 flag 前缀"的写法时的已知坑**：`${A3_TA_PROTO_ROOT}` 展开后包含空格，CMake 生成最终命令行时把这个写法和它自己的 Makefile 依赖转义逻辑搞混了，在引号内部错误插入了反斜杠。
   - **实际修复**（已验证，已直接改在 `Litchi_sonic_for_a3` 本地检出的 `gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/CMakeLists.txt` 里，**这个改动还未提交到 Litchi 的 git 仓库**，目前只是本地验证用的未提交改动）：
     ```diff
     -              --proto_path="${A3_TA_PROTO_ROOT}"
     -              --cpp_out="${A3_TA_PROTO_GEN_DIR}"
     +              "--proto_path=${A3_TA_PROTO_ROOT}"
     +              "--cpp_out=${A3_TA_PROTO_GEN_DIR}"
     ```
     即把引号从"只包住路径值"改成"包住整个 `flag=值`"。改完后 Unix Makefiles 和 Ninja 都能正确处理带空格的路径。
   - **这一条直接影响 Phase 3**：`GR00T-WholeBodyControl` 本身也在 `/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl`，同样带空格，正式迁移这个文件时要把这处引号修复一起带过去（不能直接拷贝 Litchi 仓库里未修的原始版本）。

**最终结果（2026-10-13，已完整验证通过）**：

| 阶段 | 结果 |
|---|---|
| Stage A（默认选项，无 AimRT） | ✅ 一次性编译通过 |
| Stage B（`-DENABLE_A3_AIMRT_BACKEND=ON -DENABLE_A3_ROS_MSGS=ON`，Phase 4 实际需要的完整路径） | ✅ 修复 3 个问题后完整编译通过，生成 9.7MB 可执行文件 |

修复的 3 个问题：①`empy` 4.2.1→3.3.4（ROS2 Humble 代码生成器版本不兼容）；②conda-forge `libacl` 包 + `CPATH`（host 缺 `libacl1-dev` 头文件）；③CMakeLists.txt 的 `proto_path`/`cpp_out` 引号写法（CMake 处理带空格路径的已知坑，本仓库路径本身带空格会触发）。

验证了产出的二进制不是空壳：`nm` 确认 `CreateA3AimrtBackend` 是真实符号（强符号 `T`），不是默认构建里的 `__attribute__((weak))` 空桩；设置好 `LD_LIBRARY_PATH` 后实际跑起来，`--help` 正确打印完整用法说明（`--runtime-cfg`/`--probe-source g1|smpl|both`/`--auto-start` 等参数，并指向真实的 `a3_runtime_config.yaml`）。

验证用的隔离环境：新建的 conda 环境 `a3_deploy_verify`（`conda create -n a3_deploy_verify -c conda-forge python=3.10 --override-channels`），用 conda-forge + robostack-staging 渠道装 `ros-humble-desktop`，没有碰系统 apt、没有碰 `gmr`/`isaaclab` 环境、没有要求免密 sudo。全程只在 `Litchi_sonic_for_a3/gear_sonic_deploy/` 自己的目录下产出 `build_verify_default/`、`build_verify_ninja/`、`dist/`，完全没有碰 `GR00T-WholeBodyControl`。

**结论**：Phase 4"完整范围"所需的 C++ 部署路径（AimRT 实时通道 + ROS2 消息 + protobuf）编译可行性已经验证，没有发现任何不可解决的依赖或架构问题。Phase 3 正式迁移时，除了常规的文件拷贝，需要额外做：①把这里验证出的 CMakeLists.txt 引号修复一并带过去；②在 `GR00T-WholeBodyControl` 的开发环境里也装 ROS2 Humble（走同样的 conda-forge/robostack 方式，不依赖系统 apt/Docker）；③注意 `empy` 版本（如果环境里装的是 4.x 要降到 3.3.4）。

**范围边界**：两边都用 `gear_sonic` 作为顶层 Python 包名，是两个独立 git 仓库（`Litchi_sonic_for_a3` 远程指向 `Marco-Yang/Litchi_sonic_for_a3.git`）。

---

## 1. 两边结构的总体对应关系

两个项目不是两个独立系统，而是**同一套 NVIDIA SONIC 架构的两个分支**：训练侧 Hydra 配置结构、`UniversalTokenModule` 编码器架构逐字相同；部署侧 C++ 框架 `gear_sonic_deploy` 里约 30 个文件（ONNX 推理封装、ZMQ 输入/输出接口、locomotion planner、state logger、数学工具）文件名完全相同，是字面意义上的 fork，不是独立重写。这个结论贯穿下面每一节，决定了集成策略是"补齐缺的机器人分支"而不是"搭桥对接两套系统"。

| 层 | GR00T-WholeBodyControl（本项目） | Litchi_sonic_for_a3 |
|---|---|---|
| 训练框架 | `gear_sonic/trl`、Hydra、`UniversalTokenModule` | 完全相同（同一套代码，同一个 class） |
| 训练 exp 配置 | `gear_sonic/config/exp/manager/universal_token/all_modes/sonic_{v1_1,release,h2,a3_ultra,bones_seed}.yaml` | `sonic_a3.yaml`、`sonic_a3_035.yaml`，同一目录结构，`override /actor_critic: universal_token/all_mlp_v1` 一字不差 |
| 机器人配置 py | `gear_sonic/envs/manager_env/robots/{g1,h2,a3_ultra}.py` | `gear_sonic/envs/manager_env/robots/a3.py` |
| 机器人资产 | `gear_sonic/data/assets/robot_description/mjcf/a3_ultra_t3d0.xml` | `a3_t2d5*.xml`（4 个变体：plain / passive_foot / loop / loop_passive_foot，含 hull-optimized 版本） |
| 训练好的模型 | 无 A3/A3U checkpoint | `checkpoints/035_step200000/`（.pt + 导出的 config.yaml） |
| Sim2sim | `decoupled_wbc/sim2mujoco/`（G1 专用） | `gear_sonic/scripts/sim2sim_a3_mujoco.py` |
| 真机 C++ 部署 | `gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/`（Unitree G1，`unitree_sdk2` + DDS） | `gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/`（**同一个目录路径**，已改造为 A3，用 AimRT 中间件） |
| VR 遥操 Python 侧 | `gear_sonic/scripts/pico_manager_thread_server.py`（实时 SMPL/VR3点重定向） | 无实时 VR 重定向代码；有 `pico_pose_zmq_minimal.py`（原始 VR 姿态转发，当前未接到 035 checkpoint） |
| 中间件 | ZMQ（Python）+ Unitree DDS（G1 真机） | ZMQ（部分复用同一套接口）+ AimRT（A3 真机，新增） |

---

## 2. 关键发现一：训练侧配置架构完全兼容，主要风险是 Python 包名冲突

- `gear_sonic/config/exp/manager/universal_token/all_modes/` 下两边的 exp yaml 用的是**同一个** `actor_critic: universal_token/all_mlp_v1` 覆盖项，同一个 `UniversalTokenModule`（`gear_sonic/trl/modules/universal_token_modules.py`）。把 Litchi 的 `sonic_a3.yaml`/`sonic_a3_035.yaml` 拷过来、配套的 `robots/a3.py`、`a3_motor_params.py`、`a3_t2d5*.xml` 资产、`config/actor_critic/encoders/{g1_mf_mlp,a3_fast_mf_mlp}.yaml` 一起拷过来，预期能直接跑（需要验证，见第 6 节）。
- **两边 `gear_sonic/pyproject.toml` 都声明 `name = "gear_sonic"`**，`pip install -e` 到同一个环境会互相覆盖。本项目的 `gear_sonic` 已经是开发环境里装好的那份，所以实际操作是"把 Litchi 需要的源文件当作新增文件拷进本项目的 `gear_sonic/` 目录树"，不会真的同时 `pip install` 两个同名包——但拷贝过程中要留意文件名/模块路径是否会覆盖本项目已有同名文件（下面列出已知冲突点）。
- **已知文件级命名冲突**（两边都存在、内容不同，需要合并而不是覆盖）：
  - `gear_sonic/data/robot_model/instantiation/__init__.py`：本项目已有 `ROBOT_MODEL_REGISTRY = {"g1": ...}`（这是之前 `--robot` 参数规划里提到的注册表，目前只注册了 g1）。
  - `gear_sonic/envs/manager_env/mdp/observations.py`：已实际 diff（35 行差异）。**纯新增**，零删除——Litchi 只加了 `command_multi_future_a3_fast_nonflat`/`motion_anchor_ori_a3_fast_b_mf_nonflat`/`height_map_z_flat` 三个字段和对应的三个函数实现。合并风险低，可以直接把这三段函数追加进本项目现有文件。
  - `gear_sonic/envs/manager_env/mdp/commands.py`：已实际 diff（759 行差异，远比 observations.py 复杂）。**不是纯新增**——Litchi 版本缺失本项目现有的多处逻辑：自适应采样的 cursor 快照（`capture_adaptive_cursor_snapshot`/`_adp_prev_motion_ids`/`_adp_snapshot_valid`，看起来是失败归因用的正确性修复）、`extra_joint_indices`/手部关节相关处理（`joint_utils.get_hand_joint_indices`）、`use_paired_motions`、`use_height_map`、`G1Converter().get_isaaclab_to_mujoco_mapping()`。同时 Litchi 新增了一层引用缓存（`_cached_reference_*`/`cache_current_references`/`_invalidate_reference_cache`/`_update_relative_reference`）和 A3-fast 专属方法（`_build_a3_fast_*`、`command_multi_future_a3_fast`、`joint_pos/vel_multi_future_for_a3_fast`、`root_rot_dif_l_a3_fast_multi_future`），并把部分 `self.cfg.xxx` 直接访问重构成了缓存属性（`self._use_paired_motions`、`self._start_from_first_frame` 等）。**这个文件必须逐函数手动合并，不能整体覆盖、也不能直接 diff3 自动合并**——覆盖会丢失本项目现有的自适应采样/手部关节/paired-motions/height-map 能力。
  - `gear_sonic/config/actor_critic/encoders/`：Litchi 多出 `a3_fast_mf_mlp.yaml`；`teleop_mlp.yaml`/`smpl_mlp.yaml`/`g1_mf_mlp.yaml` 两边应该一致或接近一致，拷贝前先 diff。
- **环境版本**：Litchi 的 `docs/a3_training2sim2deploy.md` 提到"要求 PyTorch ≤2.4.0，会把更新的 Isaac Lab 训练环境降级"；本项目 README 标注 `IsaacLab-2.3.2`。两边 `check_environment.py` 都要求 Python 3.10/3.11 + Isaac Sim 4.5/Isaac Lab 2.1（但本项目实际跑在 2.3.2）。**需要先确认用本项目现有环境加载 Litchi 的 035 checkpoint 是否会因为 PyTorch/IsaacLab 版本差异导致加载失败或数值不一致**，这是开始集成前第一个要做的验证，不是假设。

---

## 3. 关键发现二：C++ 部署框架是同一份代码的两个分支，有人已经设计好了多机器人切换接口

对比两个 `gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/` 目录（两边路径完全相同）：

- **完全同名的共享文件**（通用 ONNX 推理/ZMQ/规划器层，约 30 个）：`cnpy.{h,cpp}`、`control_policy.hpp`、`error_monitor.hpp`、`file_sink.{hpp,cpp}`、`localmotion_kplanner*.hpp`、`math_utils.hpp`、`motion_data_reader.hpp`、`observation_config.hpp`、`ort_session.hpp`、`output_interface/{output_interface,ros2_output_handler,zmq_output_handler}.hpp`、`policy_parameters.hpp`、`robot_parameters.hpp`、`state_logger.{hpp,cpp}`、`utils.hpp`、`xml.h`、`input_interface/{input_command,input_interface,keyboard_handler,ros2_input_handler,streamed_motion_merger,zmq_endpoint_interface,zmq_manager,zmq_packed_message_subscriber}.hpp`、`tests/{test_ros2,zmq_pose_subscriber_test,zmq_python_sender_test}.cpp`。
- **本项目独有**（G1/Unitree 专属）：`dex3_hands.hpp`（Unitree 灵巧手）、`fk.hpp/fk.cpp`、`input_interface/{gamepad,gamepad_manager,interface_manager}.hpp`、`motor_gain_scaling.{hpp,cpp}`、主入口 `src/g1_deploy_onnx_ref.cpp`、`unit_tests/test_fk.cpp`。
- **Litchi 独有**（A3 专属，全部新增在子目录里，没有碰本项目原有文件）：`include/a3_deploy/*`（obs builder、action decoder、CSV/motion-library 回放、teleop 引用缓冲、yaw 对齐、安全停机等 ~20 个头文件）、`include/a3_io/*`（AimRT 发布/订阅封装）、`include/a3_rt/*`、`include/a3_sync/*`、`include/robot_io/*`（新增的后端抽象层）、主入口改成了 `src/a3_deploy/main.cpp`。

**最关键的一处**：Litchi 新增了 `include/robot_io/robot_io_backend.hpp`，定义了一个与具体机器人无关的 `RobotIOBackend` 抽象（`RobotState`/`RobotCommand` 结构体，统一的 q/dq/tau/IMU 字段），配套的 `src/robot_io/backend_factory.cpp` 是这样写的：

```cpp
std::unique_ptr<RobotIOBackend> CreateBackend(const std::string& name) {
  if (name == "g1") {
    std::cerr << "[robot_io] legacy backend 'g1' is not part of this A3 release.\n";
    return nullptr;
  }
  if (name == "a3") {
    auto backend = CreateA3AimrtBackend();
    ...
  }
  ...
}
```

也就是说：**这个按字符串名字选择机器人后端的机制已经被设计出来了**，"g1" 和 "a3" 两个名字都已经占好位置，只是公开这份代码时把 G1 的实现（会牵涉 Unitree SDK 的授权/依赖）砍掉只留了一个报错桩。这不是"两套互不相关的部署系统需要我们去对接"，而是"一个已经设计好多机器人接口的框架，缺一块 G1 的实现、也没有 A3 Ultra 的实现"。

这给部署侧集成指出了明确路径（见第 6 节 Phase 3），不是从零设计。

---

## 4. 关键发现三：035 checkpoint 是动作模仿/回放策略，不是即插即用的 VR 遥操策略

这点直接关系"遥操"这个词在集成后到底意味着什么，必须先讲清楚，否则后面会按错误预期去验收。

- README 里的 "G1 encoder" 和 "A3-fast encoder" 名字具有误导性——**不是指 Unitree G1 硬件**，`docs/a3_training2sim2deploy.md` 明确写了这是历史遗留的 checkpoint/ONNX ABI 命名。两者都是同一种"未来参考帧"编码：`g1` 编码器输入 10 帧、每帧间隔 0.1s（未来 1s 窗口）的参考关节轨迹；`a3_fast` 是同样的机制，间隔缩短到 0.02s（未来 0.18s 窗口，低延迟部署用）。两者的参考轨迹来源**都是预先构建好的 motion_lib 数据库**（训练时来自 `a3_data/agibot_a3/*.csv` 转换出的 motion_lib pkl 缓存），035 checkpoint 没有训练 `teleop` 编码器（即不接受本项目 G1 遥操用的那种"单帧当前 VR 3 点目标 + 历史帧"格式）、也没有 `smpl` 编码器。
- 本项目 G1 的实时 VR 遥操路径（`pico_manager_thread_server.py` → ZMQ → C++ 端 `teleop` 编码器）喂给策略的是**当前帧的 VR 3 点目标位置/朝向 + 一小段历史帧做平滑**，不是"未来参考帧窗口"。这和 035 checkpoint 实际训练用的 `g1`/`a3_fast` 编码器输入格式**不是一回事**，不能直接把现有 VR 流接到这个 checkpoint 上。
- 但 Litchi 的 C++ 部署层已经有一条**已经在真机上跑通、低延迟的实时输入通道**：`a3_teleop_reference.hpp/cpp` 订阅 AimRT 频道 `/ta/whole_body_command`，接收**已经被上游重定向好的 A3 关节空间目标轨迹**（骨盆四元数 + 腰/臂/腿关节角度+角速度），直接填进 `g1`/`a3_fast` 编码器期望的同一个 640 维张量。README 首页的遥操演示视频用的就是这条路径。换句话说：**真机遥操 A3 是可行的、已验证的，但需要一个"上游实时重定向模块"把 VR/动捕数据转成 A3 关节空间轨迹再发布到这个 AimRT 频道**——这个模块 Litchi 没有开源（README 原话："an upstream real-time retargeting module is still required"）。

**结论**：把 A3 接进"用命令行参数选遥操机器人"这件事，如果目标是**用 VR 控制器实时操控**，需要新写一个"VR 3 点 → A3 29 维关节空间轨迹"的实时重定向模块（本项目给 G1 用的 IK/SMPL 重定向逻辑在 `decoupled_wbc/control/teleop/teleop_retargeting_ik.py` 里有，可以作为起点改造，但不是直接复用，因为输出目标格式不同：G1 那边是直接算关节角作为策略输入的一部分，这里还需要先过一遍 motion_lib 式的"未来帧窗口"包装）；如果目标只是"按命令行参数让 A3 跑一段预录动作/参考轨迹"（CSV/motion_lib 回放模式，`sim2sim_a3_mujoco.py --motion xxx.csv` 这种），那是现成的，集成成本低得多。

**这是需要你决定的问题，不是技术上含糊**：第一期集成要不要包含"VR 实时遥操 A3"，还是先做"命令行选 A3、跑参考动作回放/sim2sim"，VR 遥操留到下一步。下面的分期计划把这两种情况都列出来了。

---

## 5. A3 vs A3 Ultra 参数一致性（已在 worklog #37 验证，这里只总结结论和集成含义）

腿/腰/手臂的运动学（逐 mm）、质量分布（逐 g）、电机规格（力矩/限速/等效惯量逐 6 位小数）几乎完全一致，是同一套硬件平台；差异只在头部传感器和末端手（A3 Ultra 多灵巧手和独立俯仰头），且这些差异都不在 035 checkpoint 的 29 维动作空间里。**集成含义**：

- 035 checkpoint 理论上可以直接在 A3 Ultra 硬件上跑（Litchi README 也这么声称），但部署时底层伺服 Kp/Kd **必须用 Litchi 训练时用的那组数值**（`gear_sonic/envs/manager_env/robots/a3.py` 里的 `stiffness`/`damping`），不能套用本项目 `a3_ultra.py` 里为"从零训练 A3 Ultra"选的那组更硬的增益（waist_yaw 200/7 vs 85/3，详见上一轮对话）。
- 如果 `--robot` 选项要同时暴露 `a3`（跑 Litchi 的 035 checkpoint）和 `a3_ultra`（本项目未来自己训的模型），两者在 MJCF 资产、Kp/Kd、训练 exp 配置上要保持**各自独立的一套参数**，不要互相覆盖；C++ 部署端 `robot_io` 后端也建议分别注册 `"a3"` 和 `"a3_ultra"` 两个名字（即使硬件几乎一样），避免把两份训练合约（training contract）混用产生本不存在的 gap。
- 顺带：上一轮发现本项目 `a3_ultra.py` 的 `wrist_roll` armature 疑似笔误（0.000810 应为 0.004967），建议在合入前一并修掉。

---

## 6. 分期集成计划

### Phase 0 — 环境核对（集成前必须先做，不是可选）
- 用本项目现有 conda/pip 环境尝试加载 Litchi 的 `checkpoints/035_step200000`（不改任何代码，先确认 PyTorch/IsaacLab 版本差异是否导致加载失败或输出数值漂移）。
- 确认 `gear_sonic/pyproject.toml` 包名冲突的处理方式：直接把 Litchi 的源文件当新增文件合并进本项目现有 `gear_sonic/` 包（推荐，简单），还是保留 Litchi 作为独立可编辑包用命名空间隔离（更复杂，暂不建议）。

### Phase 1 — 训练侧文件迁移
- 拷贝：`robots/a3.py`、`gear_sonic/utils/a3_motor_params.py`、`config/exp/manager/universal_token/all_modes/sonic_a3{,_035}.yaml`、`config/actor_critic/encoders/a3_fast_mf_mlp.yaml`、`data/assets/robot_description/{mjcf,urdf}/a3_t2d5*`。
- 对比并合并（不是整体覆盖）：`commands.py`、`observations.py`、`data/robot_model/instantiation/__init__.py`（把 `"a3"` 加进 `ROBOT_MODEL_REGISTRY`）。
- 把 `--robot` 加进 `train_agent_trl.py` 的 `ROBOT_EXP_MAP`（复用已批准的 `/home/user/.claude/plans/foamy-singing-quilt.md` 里的机制，增加 `"a3": "manager/universal_token/all_modes/sonic_a3_035"`）。

### Phase 2 — 模型/checkpoint 迁移
- 把 `checkpoints/035_step200000/`（.pt + config.yaml）拷进本项目对应位置（建议新建 `checkpoints/` 或放进现有模型存放约定的目录，需要先看本项目现有 checkpoint 怎么组织的）。
- 用本项目的 sim2sim 工具跑一遍，和 Litchi 原仓库跑出来的结果做数值对比，确认迁移后行为一致（这是防止"看起来能跑但其实数值已经漂了"的关键一步）。

### Phase 3 — C++ 部署集成
- 把 `include/a3_deploy/`、`include/a3_io/`、`include/a3_rt/`、`include/a3_sync/`、`include/robot_io/`、对应 `src/` 下的实现，连同 AimRT 依赖（`cmake/GetAimRT.cmake` + patches）一起拷进本项目 `gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/`（沿用 Litchi 的目录名，不要去重命名引入不必要的 diff）。
- 把本项目现有的 G1 输出逻辑（目前直接写在 `src/g1_deploy_onnx_ref.cpp` / `dex3_hands.hpp` / `motor_gain_scaling.cpp` 里，没有走抽象接口）包一层，实现 `CreateG1Backend()`，让 `backend_factory.cpp` 的 `"g1"` 分支不再是报错桩。
- 仿照 A3 的 AimRT 后端，写一个 `CreateA3UltraBackend()`（用于本项目未来自己训练的 A3 Ultra 模型），走同样的 `robot_io::RobotIOBackend` 接口。
- 在可执行文件的启动参数（或 CMake 编译开关，参考 Litchi 现有的 `-DENABLE_A3_BACKEND=ON`）加一个运行时/编译时的机器人选择，三个后端（g1 / a3 / a3_ultra）编译进同一个二进制，用参数选。

### Phase 4 — 遥操路径（需要先决定范围，见第 4 节）
- **最小范围**：命令行选 `--robot a3` 后跑参考动作回放/sim2sim（`sim2sim_a3_mujoco.py` 风格），不涉及实时 VR。
- **完整范围**：另外实现"VR 3 点 → A3 29 维关节空间轨迹"的实时重定向模块，发布到 AimRT `/ta/whole_body_command`，复用已有的 `a3_teleop_reference` 接收端。可以参考 `decoupled_wbc/control/teleop/teleop_retargeting_ik.py` 的 IK 逻辑改造，但这是一块净新增工作量，不是现成迁移。

### Phase 5 — 统一 `--robot` CLI
- 在 Phase 1-4 都验证通过后，把 `g1`/`h2`/`a3_ultra`/`a3` 四个选项同时接入三条路径（训练 `train_agent_trl.py`、仿真部署 `run_sim_loop.py`/`pico_manager_thread_server.py`、C++ 真机部署可执行文件），这部分直接沿用已批准的 `/home/user/.claude/plans/foamy-singing-quilt.md` 机制，只是把机器人列表从 `[g1, a3_ultra, h2]` 扩成 `[g1, a3_ultra, a3, h2]`。

### Phase 6 — 回归验证
- 确认 `--robot g1`（默认）行为和集成前完全一致（三条路径都要跑一遍）。
- 确认 `--robot a3` 能跑通 Phase 4 选定范围内的功能，且数值结果和 Litchi 原仓库一致。
- 确认 `--robot h2`、`--robot a3_ultra` 不受影响。

---

## 7. 需要你决定的问题

1. **遥操范围**：Phase 4 做"最小范围"（参考动作回放）还是"完整范围"（新写 VR 实时重定向）？这直接决定这次集成的工作量差一个数量级。
2. **CLI 机器人命名**：`a3`（跑 Litchi 的 035 checkpoint/A3 base 硬件参数）和 `a3_ultra`（本项目自己训练、专属 A3 Ultra 参数）是否要作为两个独立选项长期共存？还是 `a3` 只是临时过渡，之后统一到 `a3_ultra`？
3. **文件迁移方式**：直接把 Litchi 的文件拷贝进本仓库（本文档默认假设），还是用 git subtree/patch 的方式保留和 Litchi 上游同步的能力（如果 Litchi 之后还会发布 036/040 等新 checkpoint，想持续同步的话这个选择更重要）？
4. **C++ AimRT 依赖**：引入 AimRT 中间件（新的第三方依赖、新的 patch 流程）会不会和本项目现有部署环境/CI 冲突？需不需要先在隔离环境验证编译。

在你对这几点拍板之前，我不会开始实际搬文件/改代码。

---

## 8. 实施记录：Litchi 并入 GR00T-WholeBodyControl 与 `--robot` 统一入口（2026-10-14）

> 本节记录“第 7 节问题拍板之后”实际做完的全部改动、验证与遗留项，面向零上下文读者。所有改动目前都在工作区，**未提交 git**（`git status` 显示 44 个已修改文件、约 +7986/−1083 行，另有大量新增文件）。

### 8.1 最终拍板的需求（覆盖第 7 节的待定项）

1. 把 Litchi 的训练代码、035 step-200k 权重（PT/ONNX/RKNN）、C++ 部署代码、环境配置都并入本仓库。
2. 训练和部署都用 `--robot` 选机器人。**最终支持 `g1 / a3 / a3_ultra / h2`**，不再是第 2 节决策里的“只保留 `a3`”：
   - `a3` = Litchi 035 合约（A3 T2D5，29 DOF，checkpoint ABI 名为 `g1`/`a3_fast`）。
   - `a3_ultra` = 本项目自研 T3D0 63 DOF 合约（31 个驱动），Kp/Kd 来自智元文档。
   - 两套合约**互相独立，不混用 Kp/Kd**（第 5 节的约束）。
3. G1/H2 零回归。
4. 过程中我一度加过 `g1_old`（旧 23 DOF G1），用户明确说不需要，**已全部撤销**（见 8.7）。

### 8.2 合并方法与安全措施

- Litchi 是 GR00T 提交 `daf3899` 的 fork。凡两边都改过的文件，一律做三方合并：base=`git show daf3899:<file>`，ours=GR00T，theirs=Litchi，用 `git merge-file -p`。
- Litchi 的 A3 发布版**删掉了 G1/H2 相关代码**，`merge-file` 会把这些删除当作 Litchi 的改动自动应用。所以每个删除块都人工复核，凡是会损坏 G1/H2 的都回退，只保留 A3 的新增。
- 合并前做了备份引用 `refs/backup/pre_litchi_merge`，需要时可整体回退。
- 其余是 Litchi 独有的新文件，直接拷入同路径，不覆盖 GR00T 已有文件。

### 8.3 训练侧改动

| 文件 | 改动 |
|---|---|
| `gear_sonic/train_agent_trl.py` | ① 新增 `ROBOT_EXP_MAP`：`g1→sonic_v1_1`、`h2→sonic_h2`、`a3→sonic_a3_035`、`a3_ultra→sonic_a3_ultra`。入口把 `--robot X` 从 argv 中取出，翻译成 `+exp=manager/universal_token/all_modes/<exp>`；未知值直接报错并列出可选项。② 兼容 transformers 5.x：新版 `TrainingArguments` 已无 `logging_dir`，会触发 `HfArgumentParser` 报 “Some keys are not used”；改为仅在 `PPOConfig` 无该字段时才 pop，并单独保存 TensorBoard 目录（`tensorboard_log_dir`）。③ 新增 `import dataclasses`。④ **本轮新增**：`sys.path` 过滤改用 `realpath`，软链接路径下也能删掉 `gear_sonic/` 目录，避免 `gear_sonic/trl` 遮盖 HuggingFace 的 `trl` 包。 |
| `gear_sonic/trl/callbacks/tensorboard_callback.py` | `args.logging_dir` 改为 `getattr(args, "logging_dir", None)`。 |
| `gear_sonic/envs/manager_env/modular_tracking_env_cfg.py` | `robot_mapping` 增加 `a3 / a3_exp020 / a3_024 / a3_exp023 / a3_old` 及 `a3_ultra`；`override_settings()` 增加 `asset_path`、`active_joint_names` 等分支。`robot_type` 默认仍为 `g1`（Litchi 默认是 A3，不能照搬，否则现有 G1/H2 配置会误加载 A3 资产）。 |
| `gear_sonic/envs/manager_env/mdp/{commands,observations,rewards,events,actuators,recorders}.py` | 并入 A3-fast 未来帧窗口、`coactivate_g1_when_teleop`、通用化的 extra-DOF 处理（用 body 关节补集代替 G1 手部名称启发式）、A3 电机约束奖励、A3 域随机化等。G1 专属的自适应采样、paired-motions、height-map 逻辑保留。 |
| `gear_sonic/trl/utils/order_converter.py` | `CONVERTER_BY_ROBOT` 覆盖 g1/h2/a3/a3_ultra；关节与刚体顺序表由 `generate_a3_order_mapping.py` 生成，复现 G1 的表时逐位一致。 |
| `gear_sonic/trl/modules/*`、`trl/losses/token_losses.py`、`trl/trainer/*`、`trl/utils/common.py` | 合并 Litchi 对应改动（A3-fast 编码器、辅助损失、输入投影等）。 |
| `gear_sonic/utils/motion_lib/motion_lib_base.py`、`gear_sonic/utils/inference_helpers.py` | 合并 Litchi 的 A3 相关逻辑，保留 GR00T 原有行为。 |
| 新增配置 | `config/exp/.../sonic_a3.yaml`、`sonic_a3_035.yaml`、`actor_critic/encoders/a3_fast_mf_mlp.yaml`、`aux_losses/*a3_fast*`、`manager_env/{events,observations,rewards}` 下的 A3 条目、`base_eval_motor_*.yaml` 等。 |
| 新增机器人/资产/工具 | `robots/a3.py`、`robots/a3_old.py`、`utils/a3_motor_params.py`、`utils/a3_motor_model.py`、`scripts/sim2sim_a3_mujoco.py`、`scripts/generate_a3_order_mapping.py`、`scripts/verify_a3_order_mapping.py`、`data_process/{convert_a3_csv_to_motion_lib,convert_a3_pkl_to_motion_lib_29d,export_a3_pkl_to_flat_csv,augment_motion_speed}.py`、A3 的 URDF/MJCF 资产。 |
| `gear_sonic/config/exp/.../sonic_a3_ultra.yaml` | 本轮曾把 `motion_file` 改为 `robot_filtered_v15_clean`，**按用户要求已改回** `data/motion_lib_a3ultra/robot_filtered`（最终值与原来相同）。 |

### 8.4 仿真与其它入口

- `gear_sonic/utils/mujoco_sim/configs.py`：`BaseConfig.robot` 改为 `Literal["g1","a3","a3_ultra","h2"]`。
- `gear_sonic/scripts/run_sim_loop.py`：新增 `A3_SIM_ROBOTS = ("a3","a3_ultra")`。选 A3 类机器人时不走 G1 的 sim 循环，而是给出明确指引：用 `python gear_sonic/scripts/sim2sim_a3_mujoco.py --checkpoint checkpoints/035_step200000/model_step_200000.pt` 或 `gear_sonic_deploy/deploy.sh --robot a3`。
- `gear_sonic/scripts/pico_manager_thread_server.py`、`decoupled_wbc/sim2mujoco/scripts/run_mujoco_gear_wbc.py`：`--robot` 的 choices 扩为 `["g1","a3","a3_ultra","h2"]`。
- `download_from_hf.py`：统一入口。`--robot a3|a3_ultra` 走 A3 流程（HF 仓库 `sonic-for-a3/sonic`，清单 `a3_hf_manifest.json`，带 SHA256 校验）；`g1` 或不指定走原 GR00T 流程。

### 8.5 C++ 部署侧改动

- **构建开关**：`gear_sonic_deploy/CMakeLists.txt` 新增 `ENABLE_TRT_INFERENCE`（默认 ON）与 `ENABLE_G1_BACKEND`（默认随 TRT）。`src/CMakeLists.txt` 里 `TRTInference` 受前者控制、`audio_thread` 受后者控制；`src/g1/CMakeLists.txt` 里 `freq_test` 受 G1 开关控制。A3 构建用 `-DENABLE_TRT_INFERENCE=OFF -DENABLE_A3_AIMRT_BACKEND=ON`，不再依赖 TensorRT 和 unitree_sdk2。
- `g1_deploy_onnx_ref/CMakeLists.txt`（约 1084 行）：以 Litchi 版本为基础合入 GR00T 内容。G1 可执行文件放在 `if(ENABLE_G1_BACKEND)` 下；输出目录在 G1 开启时为 `target/release`，否则为 `dist/a3_deploy_<arch>`；恢复 Thor/DLA 检测（`IS_THOR`）；G1 的 `EXE_LIBS` 加回 `audio_thread unitree_sdk2`。
- `cmake/ROS2.cmake`：采用 Litchi 版本，并增加 conda 回退（找不到系统 ROS2 时，用 `$CONDA_PREFIX/include/rclcpp` 与 `ROS2_INSTALL_BASE=$CONDA_PREFIX`）。
- `cmake/FindTensorRT.cmake`：保留 GR00T 注释，采用 Litchi 的额外 `PATH_SUFFIXES`。
- `scripts/build_a3_deploy_pkg.sh`：传入 `-DPython3_EXECUTABLE`，优先取 `GS_PYTHON3_EXECUTABLE`，其次是 `$CONDA_PREFIX/bin/python3`（修复系统 Python 3.12 与 conda ROS 的 3.10 混用）。
- **新增启动器** `scripts/deploy_a3.sh`：选项 `--robot a3|a3_ultra`、`--build-only`、`--no-build`、`--`。流程为：缺 ONNX 则下载 035 → 缺部署包则构建 `dist/a3_deploy_<x86_64|rockchip>/` → `exec run_a3.sh`。`a3_ultra` 会提示“沿用 035 合约；T3D0 63 DOF 合约尚无部署导出”。
- `deploy.sh`：新增 `--robot`（也读环境变量 `GS_ROBOT`，默认 `g1`），支持 `--robot X` 与 `--robot=X`。`g1` 走原流程；`a3`/`a3_ultra` 转给 `deploy_a3.sh`；`h2` 报“尚无 C++ 部署后端”（返回码 2）；其它值报 unknown（返回码 1）。
- 其余新增：`include/{a3_deploy,a3_io,a3_rt,a3_sync,robot_io}`、对应 `src/`、AimRT 的 `cmake/GetAimRT.cmake` 与补丁、`proto/`、ROS 消息 `thirdparty/joint_msgs`、RKNN 转换与 Rockchip 交叉编译脚本、A3 运行配置 `config/a3_*.yaml`、约 25 个 C++ 单测。

### 8.6 `--robot` 行为总表

| `--robot` | 训练 `train_agent_trl.py` | 部署 `deploy.sh` | 说明 |
|---|---|---|---|
| `g1`（默认） | `sonic_v1_1`（`g1_model_12_dex`） | 原 G1 流程（TensorRT + unitree_sdk2） | 行为不变 |
| `a3` | `sonic_a3_035`（`a3_024`） | `deploy_a3.sh --robot a3`，035 的 ONNX/RKNN | A3 T2D5，29 DOF |
| `a3_ultra` | `sonic_a3_ultra`（T3D0，63 DOF） | `deploy_a3.sh --robot a3_ultra`，**用 035 合约** | 同一硬件平台（worklog #37/#40）；自研合约尚无部署导出 |
| `h2` | `sonic_h2` | 报错：尚无 C++ 后端 | 同 GR00T 原状 |
| `g1_old` 等其它 | 报错并列出可选项 | 报 unknown | 已撤销 |

### 8.7 用户反馈后的撤销：`g1_old`

用户明确说不需要 `g1_old`，因此删除：`robots/g1_old.py`、`sonic_g1_old.yaml`、`data_process/convert_g1_motion_lib_to_g1_old.py`、`mjcf/g1_23dof_wrist_locked.xml`；回退它在 `robots/__init__.py`、`modular_tracking_env_cfg.py`、`order_converter.py` 里的条目，以及 `torch_humanoid_batch.py` 里为它做的 actuator 注释改动。仓库里原本就有的 `gear_sonic/data/robots/g1_old/` 资产没有动。

### 8.8 验证结果

**官方 035 权重的部署验证**

| 项目 | 结果 |
|---|---|
| ONNX 完整性 | 部署包中 `a3_fast`、`g1` 两个 ONNX 的 SHA256 与 `a3_hf_manifest.json` 一致 |
| 官方 PT 的 MuJoCo sim2sim | `001_walk_front_slow.csv`，1652 步，未摔倒，`all_29_rmse=0.066`，根部倾角最大约 9.6° |
| ONNX 对 PT 逐步对照 | `a3_fast` 与 `g1` 两个 ONNX 最大差 ≤ 3.8e-6；ONNX 输入顺序为“编码器（按观测项排列，term-major）640 维 + 本体观测 930 维”，共 1570 维 |
| ONNX 驱动 MuJoCo 闭环 | 两个官方 ONNX 都完整走完 1652 步，未摔倒，`rmse` 分别为 0.0662 与 0.067 |
| C++ 探针对 Python | 部署包里 `a3_policy_runtime_probe`（ORT 1.16.3，`ramp` 输入）的动作 sum/abs_sum/min/max 与 Python ORT 在 4 位小数内一致；单次推理约 0.87 ms |
| `deploy.sh --robot a3` | `--dry-run` 通过后端初始化；`--probe` 已加载 `g1` 与 `a3_fast` 两个官方 ONNX（输入 1570 维），不发布指令，之后等待机器人状态 |

**未验证**：“收到真机状态后输出指令”这一环没有真机或假机器人，没做；h2 的部署后端不存在。

**训练冒烟**（`--robot` 入口，16 个环境，2 次迭代，均正常完成）

| `--robot` | 动作库 | 初始化 |
|---|---|---|
| `a3` | `.cache/motionlib_035_selected20` | 官方 035 PT 热启动 |
| `a3_ultra` | `data/motion_lib_a3ultra/robot_filtered_v15_clean/210531`（208M，206 个文件） | 随机初始化 |
| `g1` | `sample_data/robot_filtered` + `sample_data/smpl_filtered`（官方样例） | 随机初始化 |
| `h2` | 本地无动作库 | 未跑 |

其它：Hydra compose 对四个机器人均通过（项目名分别为 TRL_G1_Track / TRL_H2_Track / TRL_A3_035 / TRL_A3Ultra_Track）；C++ 单测 192/200 通过，已知失败项是缺少 `reference/bones_072925_test/` 夹具的 FK 测试（原有问题），其余失败项本轮没有逐一复核；`pytest tests gear_sonic/tests` 尚未重跑。

### 8.9 环境与踩坑记录

- 环境：`isaaclab`（py3.11，transformers 5.19.0，trl 0.28.0）用于训练与 sim2sim；`a3_deploy_verify`（py3.10，robostack ROS Humble）用于 C++ 构建与运行；GPU 相关命令需要在沙箱外运行。
- C++ 构建：`empy` 必须是 3.3.4；`libacl` 用 conda 包，并设置 `CPATH=$CONDA_PREFIX/include`、`LIBRARY_PATH=$CONDA_PREFIX/lib`（否则 `ld: cannot find -lacl`、`sys/acl.h` 找不到）；运行时要把 `$CONDA_PREFIX/lib` 加进 `LD_LIBRARY_PATH`，否则 `librclcpp.so` 找不到。
- **iceoryx 版本必须匹配**：部署包用 `_deps/iceoryx-src`（2.95.4）编译，conda 自带的 `iox-roudi` 是 2.0.2，不兼容，客户端会一直 “RouDi not found - waiting”。需要用同版本编译 RouDi：`cmake --build build_verify_full --target iox-roudi`，产物 `build_verify_full/iox-roudi`。
- **仓库路径含空格**（`RL controller`）：`accelerate launch` 会把带空格的 Hydra 参数拆开，导致 `LexerNoViableAltException`。训练请用无空格软链接路径（本轮用 `/tmp/gwbc`），并用上面第 8.3 节的 `realpath` 修复保证 `trl` 不被遮盖。
- Isaac Sim：在干净环境里运行（`env -i` 只带 HOME/USER/LANG/PATH/EULA 变量，`python -u`）；`app.close()` 会卡住，脚本里用 `os._exit(0)`；在激活了 `a3_deploy_verify` 的 shell 里启动会污染 numpy。
- ONNX 对照所需的 onnxruntime 装在临时目录 `/tmp/ort_site`（用 `--target`），装完要删掉它顺带装进去的 numpy 2.x，否则会遮盖环境里的 numpy 1.26.4；没有改动任何 conda 环境。
- 验证脚本（不在仓库内）：`/tmp/a3_onnx_closed_loop.py`（包装 `A3Policy.act`，对照并可驱动 ONNX）、`/tmp/run_smoke.sh`（训练冒烟）。

### 8.10 数据与官方开源范围（回答“能不能自己训练”）

- GR00T 官方开源：训练代码、SONIC 检查点、C++ 部署栈、Bones-SEED 动作库（14.2 万条，含 G1 重定向 CSV 与 SOMA BVH）、转换脚本。SOMA 重定向器本身在外部仓库 NVIDIA/soma-retargeter（或 GMR），本仓库不含。
- Litchi 官方开源：A3 训练适配、035 检查点（PT/ONNX/RKNN）、20 条样例 CSV、C++ 部署。它没有开源内部动作库、实时重定向和遥操作链路（README 的 TODO）。
- 本地已有数据：`data/bones_seed/soma_proportional.tar.gz`（43G）、`data/motion_lib_a3ultra/`（`robot_filtered_v15`、`robot_filtered_v15_clean`）、`sample_data/`、`a3_data/agibot_a3/`（20 条 A3 CSV）。

### 8.11 遗留项

1. `docs/workflow/sonic_a3_ultra_worklog.md` 需追加编号条目（只追加，不覆盖），本节内容可作为其素材。
2. README 的 A3 章节和环境说明（empy 3.3.4、libacl、`GS_PYTHON3_EXECUTABLE`、RouDi 版本、空格路径）。
3. 重跑 `pytest tests gear_sonic/tests`，重点看 `tests/test_a3_035_contract.py`、`test_download_from_hf.py`、`test_convert_a3_onnx_to_rknn.py`。
4. `sonic_a3_ultra.yaml` 的 `motion_file` 默认值 `data/motion_lib_a3ultra/robot_filtered` 在本地不存在（现有目录是 `robot_filtered_v15` 和 `robot_filtered_v15_clean`），默认配置直接训练会找不到数据；需重命名/软链，或训练时显式覆盖。
5. `a3_ultra` 自研 63 DOF 合约尚无部署导出；h2 没有 C++ 后端。
6. 可选：在 `ROBOT_MODEL_REGISTRY` / `ROBOT_WBC_YAML` 注册 a3 / a3_ultra / h2；Phase 4 VR 实时重定向；Phase 6 的 G1/H2 完整回归；清理被 gitignore 的构建目录（`gear_sonic_deploy/build/`、`build_verify_full/`、`dist/`）。

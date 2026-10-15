# A3 集成规划：把 Litchi_sonic_for_a3 并入 GR00T-WholeBodyControl

**状态**：范围已拍板（见下）；C++ 部署框架编译验证已完成（见下方专节）；**Phase 1（训练侧文件迁移）已完成并逐项验证**，完整权威记录见 worklog #40（自包含，面向冷启动 agent）；**Phase 2（checkpoint 迁移+数值对比）资产部分已完成验证**，完整记录见 worklog #41；**IsaacLab 端到端验证（真正起 Hydra/`UniversalTokenModule`/`ManagerBasedRLEnv`，strict 权重加载 + 20 步真实 rollout，两仓库均 exit 0）已完成**，过程中发现并修复了 3 处 Phase 1 未覆盖的真实遗漏（`commands.py` 缺 `coactivate_g1_when_teleop`、`commands.py` 的额外 DOF 处理对 A3 被动脚关节不通用、`modular_tracking_env_cfg.py` 整个文件此前从未被移植导致 `robot_mapping`/`asset_path`/`active_joint_names` 调度逻辑缺失），完整记录见 worklog #42（自包含，含遗留项清单）。**【2026-10-14 更新】Phase 3（C++ 部署集成）已在第 8 节完成，Phase 4（Pico 实时重定向，模拟器验证）已在第 9 节完成，改动已推送（`490c7aa`/`cd9285f`/`687bdc7`/`1a7b47c`）；`--robot` 最终取值为 `g1 / a3 / a3_ultra / h2`，推翻了下方"已拍板的决策"第 2 条的"只保留 a3"，`g1_old` 已撤销不再加回。当前目标对照与剩余缺口见第 10 节。**下文第 1–7 节保留为历史规划记录，其中"尚未开始/待决定"的表述已过期。**后续进展按节**：第 12 节 `run_sim_loop.py --robot a3|a3_ultra` 仿真 + 部署 + Pico；第 13 节 `a3_ultra` 参数对齐 Litchi a3；第 14 节直接用下载的 035 权重；**第 15 节 `deploy.sh --encoder a3-fast|smpl`（`a3-fast` 完整可用；`smpl` 接口、校验、安全逻辑就绪，但没有 A3 / A3U 的 SMPL-encoder 权重，目前只能用站立桩验证数据通路）**。
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
| `a3_ultra` | `sonic_a3_ultra`（T3D0，63 DOF；**2026-10-15 起 `sonic_a3_ultra` 是 A3 合约配方、T3D0 改名 `sonic_a3_ultra_t3d0`，见第 16 节**） | `deploy_a3.sh --robot a3_ultra`，**用 035 合约** | 同一硬件平台（worklog #37/#40）；自研合约尚无部署导出 |
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

其它：Hydra compose 对四个机器人均通过（项目名分别为 TRL_G1_Track / TRL_H2_Track / TRL_A3_035 / TRL_A3Ultra_Track）；C++ 单测 192/200 通过，已知失败项是缺少 `reference/bones_072925_test/` 夹具的 FK 测试（原有问题），其余失败项本轮没有逐一复核。`pytest tests gear_sonic/tests`（isaaclab 环境）已重跑：169 通过、0 失败，1 个收集错误是缺 `msgpack_numpy`（遥操作可选依赖，与 A3 无关）；部署目录的 A3 Python 测试 4 通过 1 跳过，同目录 G1 的 ZMQ 测试因缺 `zmq` 无法收集（环境问题）。

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

1. ~~worklog 追加条目~~ 已完成：`sonic_a3_ultra_worklog.md` #43。
2. ~~README 的 A3 章节和环境说明~~ 已完成：根 `README.md` 的 “AgiBot A3 / A3 Ultra (`--robot`)”。
3. ~~重跑 pytest~~ 已完成，结果见 8.8。
4. `sonic_a3_ultra.yaml` 的 `motion_file` 默认值 `data/motion_lib_a3ultra/robot_filtered` 在本地不存在（现有目录是 `robot_filtered_v15` 和 `robot_filtered_v15_clean`），默认配置直接训练会找不到数据；需重命名/软链，或训练时显式覆盖。
5. `a3_ultra` 自研 63 DOF 合约尚无部署导出；h2 没有 C++ 后端。
6. 可选：在 `ROBOT_MODEL_REGISTRY` / `ROBOT_WBC_YAML` 注册 a3 / a3_ultra / h2；Phase 4 VR 实时重定向；Phase 6 的 G1/H2 完整回归；清理被 gitignore 的构建目录（`gear_sonic_deploy/build/`、`build_verify_full/`、`dist/`）。

## 9. Pico 遥操 A3：实时重定向链路（Phase 4）

### 9.1 为什么需要重定向（Litchi 里有什么、没有什么）

- 035 检查点只有 `g1` 与 `a3_fast` 两个编码器，没有 SMPL / 遥操编码器；它吃的是 A3 的关节参考（29 关节 + 骨盆朝向，640 维 tokenizer 前缀），不是人体姿态。Litchi README 的 “Current limitations” 明确写了：仍需要上游实时重定向模块；TODO 里“Provide a complete open-source teleoperation pipeline”未完成。
- **直接复用的 Litchi 逻辑**（没有重写）：C++ 的 `A3TeleopReferenceBuffer`、yaw 对齐、陈旧保持、站立回退、AimRT `/ta/whole_body_command` 订阅与 `ConvertTaWholeBodyCommand`。Litchi 里的 `a3_zmq_smpl_source` 需要另一个 SMPL ONNX，035 里没有，所以不可用。
- **Litchi 里不存在、本轮新增的部分**：Pico(SMPL) → A3 关节角的实时重定向、传输桥、Pico 模拟器、MuJoCo 遥操闭环。
- G1 不需要这一段：官方 SONIC 的 G1 权重自带 SMPL 编码器。

### 9.2 链路

```
Pico / pico_pose_zmq_replay.py --ZMQ "pose":5556--> a3_teleop_retarget_node.py (GMR, agibot_a3_ultra)
   --ZMQ "a3_ref":5560--+--> sim2sim_a3_mujoco.py --teleop-zmq        (MuJoCo 闭环，035 PT)
                        +--> a3_ta_bridge (C++) --ROS2 RosMsgWrapper(pb)--> /ta/whole_body_command
                                                                       --> a3_deploy_onnx_ref (真机/部署)
```

### 9.3 新增 / 修改的文件

| 文件 | 作用 |
|---|---|
| `gear_sonic/utils/teleop/a3/frames.py` | 关节顺序（mj29，与 TA 位置顺序一致）、`A3TeleopFrame`、`a3_ref` 线格式（545 字节） |
| `.../a3/reference_buffer.py` | C++ `A3TeleopReferenceBuffer` 的 numpy 移植（与 `sim2sim.build_tokenizer_terms` 数值交叉验证） |
| `.../a3/retarget.py` | Pico SMPL 帧 → GMR 人体数据 → 逐帧 IK（热启动）→ 29 关节 + 骨盆四元数 |
| `.../a3/sim_teleop.py` | MuJoCo 实时遥操闭环（站立回退、陈旧保持、yaw 对齐、跌倒/倾角/跟踪误差统计、JSON 报告） |
| `gear_sonic/scripts/a3_teleop_retarget_node.py` | ZMQ `pose` → 重定向 → ZMQ `a3_ref` |
| `gear_sonic/scripts/pico_pose_zmq_replay.py` | Pico 模拟器（回放 SMPL pkl，或 `synthetic:stand` / `synthetic:wave`） |
| `gear_sonic/scripts/sim2sim_a3_mujoco.py` | 新增 `--teleop-zmq/--teleop-seconds/--teleop-delay-ms/--teleop-stale-ms/--teleop-report/--teleop-viewer` |
| `gear_sonic_deploy/.../include/a3_deploy/a3_ta_bridge_codec.hpp`、`src/a3_deploy/a3_ta_bridge_codec.cpp` | `a3_ref` 解析、`TaWholeBodyCommandChannel` 构造、AimRT ROS2 话题名编码 |
| `gear_sonic_deploy/.../tools/a3_ta_bridge/a3_ta_bridge.cpp` + CMake 目标 `a3_ta_bridge` | ZMQ → ROS2 发布（`RosMsgWrapper`，`serialization_type="pb"`） |
| 测试 | `tests/test_a3_teleop_reference.py`（9）、`tests/test_a3_teleop_retarget.py`（4）、`unit_tests/test_a3_ta_bridge_codec.cpp`（5） |

### 9.4 一个重要的准确性修复：骨盆朝向权重

GMR 自带的 `agibot_a3_ultra` 配置里骨盆朝向权重只有 10 / 5。结果是一个普通站立的人会被重定向成**骨盆倾斜约 36–41°**（腰和髋再反向补偿，左右腿不对称）；项目自带的离线脚本 `batch_retarget_a3ultra.py` 出的数据同样如此（流式与离线逐帧差 RMS 0.003 rad，骨盆四元数差 0.06°）。035 策略跟踪骨盆锚点朝向，而 Litchi 的动作数据根部倾角约 1°。因此重定向器默认把骨盆朝向权重提到 50（`--pelvis-rot-weight`，≤0 恢复 GMR 原值）：站立时骨盆倾角 <1.5°，腰关节≈0，行走时平均倾角约 1°。未改动 GMR 仓库本身。

### 9.5 验证结果（MuJoCo，035 PT，25 s，同机实时节奏）

| 场景 | 编码器 / 延迟 | 是否摔倒 | 最大倾角 | 平均跟踪 RMSE | p95 |
|---|---|---|---|---|---|
| 走路 clip（骨盆权重 10/5，修复前） | `a3_fast` / 220 ms | 否 | 15.4° | 0.274 rad | 0.296 |
| 走路 clip | `a3_fast` / 220 ms | 否 | 8.3° | 0.094 rad | 0.129 |
| 走路 clip | `g1` / 940 ms | 否 | 8.5° | 0.090 rad | 0.130 |
| 右臂挥动（`synthetic:wave`） | `a3_fast` / 220 ms | 否 | 4.0° | 0.071 rad | 0.078 |

重定向每帧约 2.3 ms（p95 约 3 ms）。跟踪 RMSE 是“机器人状态 vs 延迟后的参考采样”，包含延迟本身带来的差。

**C++ 桥**：
- `a3_ta_bridge_codec` 的 5 个 gtest 通过；其中以部署端真实的 `ConvertTaWholeBodyCommand` 为准绳，帧经 `Pack → Parse → BuildTaWholeBodyCommand → protobuf 序列化 → Convert` 后 q / dq / 骨盆四元数 / 头部逐位一致（带头与不带头两种）。
- 跨语言、跨进程：Python `pack_frame` → ZMQ → `a3_ta_bridge` → ROS2（`RosMsgWrapper`）→ 用 `ConvertTaWholeBodyCommand` 解码的订阅者，q / dq / 骨盆四元数 / 头部差值均为 **0.0**；桥 50 Hz 转发 497 帧，0 拒收，可正常退出。
- ROS2 话题名 `/ta/whole_body_command/pb_3Aaimdk_2Eprotocol_2ETaWholeBodyCommandChannel` 与 AimRT 的 `Ros2NameEncode` 一致（单测覆盖）。

### 9.6 用法

```bash
# 环境：gmr（重定向与 MuJoCo 闭环）。仓库路径含空格，请用无空格软链接路径。
# 1) Pico（或模拟器）-> pose 流
python gear_sonic/scripts/pico_pose_zmq_replay.py --clip sample_data/smpl_filtered/walk_forward_amateur_001__A001.pkl --stand-seconds 6 --loop
#    或：--clip synthetic:stand / synthetic:wave
# 2) 重定向节点
python gear_sonic/scripts/a3_teleop_retarget_node.py            # pose:5556 -> a3_ref:5560
# 3a) MuJoCo 闭环
python gear_sonic/scripts/sim2sim_a3_mujoco.py --checkpoint checkpoints/035_step200000/model_step_200000.pt \
    --encoder-mode a3_fast --mjcf gear_sonic/data/assets/robot_description/mjcf/a3_t2d5_loop_passive_foot_twostage_fit_optimized.xml \
    --teleop-zmq tcp://localhost:5560 --teleop-seconds 25 --teleop-report /tmp/report.json
# 3b) 真机 / 部署：先启动 a3_deploy_onnx_ref（ROS2 配置 a3_aimrt_config.ros2.yaml），再启动桥
export LD_LIBRARY_PATH=gear_sonic_deploy/dist/a3_deploy_x86_64:$LD_LIBRARY_PATH
ROS_DOMAIN_ID=<与部署一致> gear_sonic_deploy/dist/a3_deploy_x86_64/a3_ta_bridge --zmq tcp://127.0.0.1:5560
```

### 9.7 踩坑：默认 ROS 域 0 会让本机 conda ROS2 把内存吃光（导致 Cursor 闪退）

- 现象：本机 conda `a3_deploy_verify`（ROS Humble / Fast-DDS 2.6.4）里，任何 ROS2 程序（包括官方 `demo_nodes_cpp talker`）在默认域 0 上几百毫秒内内存涨到几十 GB。2026-10-14 04:05 内核 OOM 杀掉了我写的验证订阅者（RSS 54 GB），整机卡死，Cursor 随之闪退。
- 根因（`strace -k`）：`rmw_fastrtps` 的 `node_listener` 线程在反序列化 `rmw_dds_common::ParticipantEntitiesInfo` 时，读到异常大的向量长度（一次分配约 1.6 GB 再翻倍）。本机没有其它 ROS 进程，该数据来自局域网（Wi-Fi 192.168.110.0/24）或 Tailscale 上别的 ROS2 参与者。不是本仓库代码的问题；换到 `ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1` 后 `talker` 与桥都正常。
- 约定：在本机做 ROS2 调试一律使用独立域 + `ROS_LOCALHOST_ONLY=1`，并用 `systemd-run --user --scope -p MemoryMax=2G -p MemorySwapMax=0 …` 限制内存；真机部署时再显式设置与机器人一致的 `ROS_DOMAIN_ID`。桥启动时会打印当前 `ROS_DOMAIN_ID`。
- 附带：跑验证订阅者时反复调用 `rclcpp::spin_some(node)` 会在 Humble 里反复创建 executor，应复用同一个 executor 并用 `spin_once(timeout)`（桥本身不 spin，不受影响）。

### 9.8 环境改动与未验证项

- 向 `gmr` conda 环境额外安装了 `pyzmq`、`msgpack`、`pytest`（用户环境改动，仅此三项）。
- `docs/a3_024_sim2real_materials` 已迁到 `a3_data/a3_024_sim2real_materials`，相关引用（`.gitignore`、`.gitattributes`、几份 docs 与 README）已同步。
- `g1_old` 仍处于已撤销状态，没有恢复。
- **未验证**：真实 Pico 头显的数据（仅用同格式的模拟器验证）；真机与 `a3_deploy_onnx_ref` 实际消费桥发出的话题（只用同一个转换函数的订阅者验证了内容，没有起完整的部署进程）；`g1` 的 Pico 遥操需要 G1 的 TRT/GPU 路径与硬件，本机无法完整验证；`a3_ta_bridge` 未纳入 `run_tests`（该目标在本机的 conda 头文件下因 gtest 混用而无法链接，编解码单测是单独编译运行的）。

## 10. 目标对照与剩余缺口（2026-10-14 核对）

> 本节对照最新目标重新核对一遍：① 训练能用 `--robot` 选机器人；② 用官方权重或自训权重，通过 Pico 在 MuJoCo 和真机上遥操 g1 和 a3。`--robot` 取值沿用已拍板的 `g1 / a3 / a3_ultra / h2`，**不含 `g1_old`**（用户已明确撤销，见 8.7）。以下状态都是核对当前代码和文档得到的，未重新跑的项会标明。

### 10.1 目标 ① 训练 `--robot` 选机器人

| `--robot` | 入口 | 状态 | 证据 |
|---|---|---|---|
| `g1` | `sonic_v1_1` | 冒烟通过（16 envs、2 次迭代） | worklog #43.3 |
| `h2` | `sonic_h2` | Hydra compose 通过；**没有本地动作库，未跑训练** | 8.8 |
| `a3` | `sonic_a3_035` | 冒烟通过（官方 PT 热启动）；IsaacLab strict 加载 + 20 步 rollout 通过 | worklog #42、#43 |
| `a3_ultra` | `sonic_a3_ultra` | 冒烟通过；**未做收敛训练**；`motion_file` 默认路径在本地不存在 | 8.11 第 4 条 |

### 10.2 目标 ② Pico 遥操（MuJoCo / 真机）

| 机器人 | MuJoCo | 真机 | 权重 | 状态 |
|---|---|---|---|---|
| g1 | `run_sim_loop.py` + `deploy.sh --input-type zmq_manager sim` + `pico_manager_thread_server.py --manager`（官方原流程，见 `docs/notes/mujoco_pico_sim_teleop.md`） | `deploy.sh --input-type zmq_manager real`（TRT + unitree_sdk2） | 官方 SONIC，自带 SMPL 编码器 | 本机此前就能跑通。改动后的回归验证见 11.10（能启动并到 `Init Done`；真实头显数据未测） |
| a3 | `a3_teleop_retarget_node.py` → `sim2sim_a3_mujoco.py --teleop-zmq` | `a3_ta_bridge` → `/ta/whole_body_command` → `a3_deploy_onnx_ref` | 官方 035（只有 `g1`/`a3_fast` 编码器，没有 SMPL/teleop 编码器），需上游实时重定向 | MuJoCo 闭环用 Pico 模拟器验证通过；桥的内容用同一转换函数的订阅者验证；**真实头显与真机未验证** |
| a3_ultra | 同 a3 | 同 a3（走 035 合约） | 035；自研 63 DOF 合约无部署导出 | 同 a3 |

### 10.3 本次核对结果

- 重跑 `tests/test_a3_teleop_reference.py`、`tests/test_a3_teleop_retarget.py`（`gmr` 环境）：**13 通过，0 失败**。
- 核对代码：`run_sim_loop.py` 遇到 `a3`/`a3_ultra` 只打印指引并退出，没有和 g1 对称的一条命令入口；`ROBOT_MODEL_REGISTRY` 仍只有 `g1`。a3 的重定向走 GMR（`agibot_a3_ultra`），不经过 `RobotModel`，因此不注册也不影响 a3 遥操。
- 本地 `checkpoints/` 只有 `035_step200000`，没有 G1 官方训练权重。

### 10.4 剩余缺口（按能否在本机完成区分）

**本机可以做：**
1. **G1 零回归实证**：下载 G1 权重（`download_from_hf.py --training --sonic-v1-1 --no-smpl`），跑 `eval_agent_trl.py +max_render_steps=20` 冒烟（worklog #42.6 第 7 项）。需要联网下载，体量以脚本提示为准。
2. **`a3_ultra` 默认 `motion_file`** 指向不存在的 `robot_filtered`：建议改为软链或显式覆盖，需用户确认（用户此前要求保持原值）。
3. **a3 的统一遥操入口**：目前是三条独立命令（9.6），可以包成一个与 g1 对称的启动脚本。

**本机做不了，需要硬件或用户提供：**
4. 真实 Pico 头显数据、g1 真机、a3/A3 Ultra 真机的"收到状态后输出指令"。
5. 自研 `a3_ultra` 合约的部署导出（需先有收敛的权重）。
6. h2 的 C++ 后端。

### 10.5 已知文档历史问题（不改写，仅提示）

worklog 里有两个"#23"和一个"22b"，且 #20.5 的"保留约 17%"预期已被 #33–#36 取代，以 #31、#33 的 v15 为准。

## 11. 本轮执行记录（2026-10-14）：G1 数据/权重、`a3_ultra` 默认数据、A3 统一遥操启动脚本

> 接第 10 节的缺口清单。用户对"本机可做的三项"全部选择执行，并补充：**目前手上只有 A3 Ultra 真机，希望 A3（035）的权重能直接用在 A3 Ultra 上**。`--robot` 仍是 `g1 / a3 / a3_ultra / h2`，`g1_old` 不加回。

### 11.1 `a3_ultra` 的 `motion_file` 默认值（已改）

- `gear_sonic/config/exp/manager/universal_token/all_modes/sonic_a3_ultra.yaml`：`motion_file` 由 `data/motion_lib_a3ultra/robot_filtered`（本地不存在）改为 `data/motion_lib_a3ultra/robot_filtered_v15_clean`（129,116 个 pkl，worklog #36）。
- 这是用户本轮明确选择的改动，覆盖了 8.11 第 4 条/worklog #43.6 第 2 条中"用户要求保持原值"的旧约定。**已验证**：`--robot a3_ultra`（16 envs、2 次迭代，**不加任何 `motion_file` 覆盖**）加载了 `Loaded 129116 motion files from data/motion_lib_a3ultra/robot_filtered_v15_clean`，`exit=0`。此前 worklog #43.3 的冒烟用的是 `210531` 子目录显式覆盖。注意：加载 12.9 万个动作只是冒烟能通过，正式训练的显存/内存占用没有评估过。

### 11.2 A3 统一遥操启动脚本（已完成并实测；注：第 12 节之后 `run_sim_loop.py --robot a3|a3_ultra` 已能直接跑 A3 仿真，不再只是提示，本脚本保留为 Python 单进程闭环，不经过 C++ 部署程序）

- 新增 `gear_sonic/scripts/run_a3_teleop_sim.py`，是 A3 版的 `run_sim_loop.py`：一条命令依次起 Pico 模拟器（可选）→ `a3_teleop_retarget_node.py` → `sim2sim_a3_mujoco.py --teleop-zmq`，结束或 Ctrl-C 时清理子进程。仅用标准库，在 `gmr` 环境里运行。
- 参数：`--robot a3|a3_ultra`（都走 035 合约）、`--source replay|pico`、`--clip`、`--encoder-mode a3_fast|g1`、`--seconds`、`--viewer`、`--report`、`--pose-port`、`--ref-port`、`--dry-run`。`--source pico` 不起模拟器，由用户自己启动头显推流（`pico_manager_thread_server.py` / `pico_pose_zmq_minimal.py`，topic `pose`）。
- `gear_sonic/scripts/run_sim_loop.py`：选 `a3`/`a3_ultra` 时的提示信息改为指向这个脚本。
- 用法（路径含空格，请用无空格软链接，如 `/tmp/gwbc`）：
  ```bash
  cd /tmp/gwbc
  conda run -n gmr python gear_sonic/scripts/run_a3_teleop_sim.py --robot a3_ultra \
      --source replay --clip synthetic:wave --seconds 25 --report /tmp/a3_teleop.json
  ```
- **实测**（`--robot a3_ultra --source replay --clip synthetic:wave --seconds 12`，035 PT，`a3_fast`/220 ms）：`fell=false`，`min_pelvis_z=0.826`，最大倾角 3.9°，平均跟踪 RMSE 0.074 rad（p95 0.081），收到 611 帧，重定向平均 2.5 ms/帧（p95 3.1 ms），运行结束后无残留子进程。
- 重跑 `tests/test_a3_teleop_reference.py`、`tests/test_a3_teleop_retarget.py`：13 通过。
- 没有验证的：`--source pico`（真实头显）、`--viewer`、`--encoder-mode g1` 经过该脚本的路径（9.5 里 `g1` 编码器已用手动命令验证过）。

### 11.3 A3 Ultra 真机使用 035（A3）权重：核对结论

- `deploy_a3.sh --robot a3_ultra` 本来就走 035 合约，`a3_runtime_config.yaml` 的 `policy_control.profile: a3_024`（A3 的踝增益和动作缩放）。**Kp/Kd 必须保持 A3 这一套，不能换成 `a3_ultra.py` 里自研合约的那组**（worklog #37）。〔2026-10-14 更新：`a3_ultra.py` 现在默认就用这一套（第 13 节），官方文档的那组只作为 `A3_ULTRA_GAIN_SOURCE="vendor_doc"` 保留、未启用。〕
- 部署代码（`a3_deploy`、`a3_runtime_config.yaml`）中**没有任何 A3 Ultra 专属处理**。035 的动作空间只有 29 个身体关节，A3 Ultra 多出的灵巧手和独立俯仰头不受策略驱动。
- 以下差异在本机**无法验证**，需要上真机前人工确认（worklog #37）：
  1. hip_roll 软件/硬件限位 A3 Ultra 为 ±85°，A3 为 ±92°，35° 以外策略输出需要被截断；
  2. 整机重 2 kg（躯干 +0.649 kg、手腕 +0.22 kg/侧、头 +0.235 kg、手指），035 训练时没有这部分负载；
  3. 配置里有 `/motion/control/hand_joint_*` 话题，A3 Ultra 真机的 AimRT 话题布局是否与 A3 一致没有核实（plan Phase 3 一直留着这个问题）；
  4. "收到真机状态后输出指令"整段没有真机验证过（8.8）。
- 建议上真机的顺序：先 `deploy.sh --robot a3_ultra --dry-run`/`--probe`，再用站立 CSV 低增益悬吊测试，之后才接 Pico。

### 11.4 G1 数据与权重

- **G1 动作数据**：用户已下载 `data/bones_seed/g1.tar.gz`（23.5 GB，目录结构 `g1/csv/<session>/<clip>.csv`）。已解压到新的空目录 `data/bones_seed/g1_extract/`（磁盘剩余约 1.5 TB，够用）。**解压已完成**（`tar` exit 0）：142,220 个 CSV、124 个 session，与 Bones-SEED 总数一致，约 37 GB。
- **G1 官方训练权重**：已下载到仓库根目录 `sonic_v1_1/`（`last.pt` 1.1 GB、`config.yaml`、`model_config.yaml`），命令 `download_from_hf.py --training --sonic-v1-1 --no-smpl`（不含 SMPL 动作数据）。
  - 这个权重**是需要的**：`g1.tar.gz` 只是动作数据，不含权重；G1 零回归冒烟（`eval_agent_trl.py`，worklog #42.6 第 7 项）要加载训练 checkpoint。
  - 过程中误以为下载还在进行（实际上已完成），原因是我写的等待循环里 `pgrep -f` 匹配到了自己的命令行；已停止该循环，无需再停下载。
- **G1 `motion_file` 默认路径**：`sonic_v1_1.yaml` 指向 `data/motion_lib_bones_seed/robot_filtered` 和 `data/bones_seed_smpl`，这两个目录本地都不存在。冒烟时用显式覆盖。

### 11.5 G1 冒烟结果与剩余待办

**已完成的验证（均在 `isaaclab` 环境，GPU，`env -i` 干净环境，无空格软链接路径 `/tmp/gwbc`）：**

| 项目 | 做法 | 结果 |
|---|---|---|
| G1 动作转换 | `convert_soma_csv_to_motion_lib.py --input data/bones_seed/g1_extract/g1/csv/210531 --output data/motion_lib_bones_seed_g1_smoke/robot_filtered --fps 30 --fps_source 120 --individual --num_workers 8` | 210/210 条成功，0 失败，52 MB，约 5 秒 |
| `--robot g1` 训练冒烟 | 16 envs、2 次迭代，motion_file 指向上面转换的 210 条，smpl_motion_file 用 `sample_data/smpl_filtered` | `exit=0`，`Loaded 210 motion files`，训练循环正常结束 |
| G1 零回归（运行时） | `eval_agent_trl.py +checkpoint=sonic_v1_1/last.pt`，1 env，`+max_render_steps=20` | `exit=0`，`Successfully loaded policy state dict`（strict 加载），`Reached max_render_steps=20`，无 Traceback/形状错误 |
| `--robot a3_ultra` 训练冒烟（默认路径） | 16 envs、2 次迭代，不加 `motion_file` 覆盖 | `exit=0`，加载 129,116 个动作 |

- 这条 G1 评估冒烟补上了 worklog #42.6 第 7 项：为打通 A3 而改动的 `commands.py`、`modular_tracking_env_cfg.py`，在 G1 官方 `sonic_v1_1` 权重下仍能被实例化并跑通 20 步。**仍然没有做逐步数值对比**，也没有对 H2 做同类验证（H2 本地无动作库和权重）。
- 日志里大量 `Failed to create change watch ... errno=28/No space left on device` 是 Isaac Sim 的 inotify 监视器报错（磁盘实际剩余约 1.5 TB），不影响运行；日志里 "nan" 的匹配只是路径 `warp/nanovdb`，不是 NaN。
- 评估会在 `sonic_v1_1/` 下写出 `exported/`、`last_eval_debug_plots/` 和 `model_step_100000.pt`。该目录目前在 git 中是**未跟踪且未被忽略**的状态，含 1.1 GB 权重，提交前不要 `git add -A`，或先把 `sonic_v1_1/` 加入 `.gitignore`。
- 冒烟产物在会话临时目录，不在仓库内；`data/motion_lib_bones_seed_g1_smoke/` 在仓库的 `data/` 下（已被忽略）。

### 11.6 G1 全量转换与过滤（已完成，用户确认执行）

- **转换**：`convert_soma_csv_to_motion_lib.py --input data/bones_seed/g1_extract/g1/csv --output data/motion_lib_bones_seed/robot_all --fps 30 --fps_source 120 --individual --num_workers 28`（`isaaclab` 环境，`python -I`）。结果：**142,220 / 142,220 成功，0 失败**，8.6 GB，124 个 session。日志 `data/motion_lib_bones_seed/convert_g1_all.log`。
- **过滤**：`filter_and_copy_bones_data.py --source data/motion_lib_bones_seed/robot_all --dest data/motion_lib_bones_seed/robot_filtered --workers 28`，使用仓库默认的 40 个关键词（与 worklog #36 对 A3 用的完全相同，未新增）。先 `--dry-run` 核对再正式执行：保留 **129,785**，过滤 **12,435**（合计 142,220），**7.9 GB**，124 个 session。A3 那次是 141,538 → 过滤 12,422，量级一致。
- **路径**：`robot_filtered` 正是 `sonic_v1_1.yaml` 默认的 `motion_file`（`data/motion_lib_bones_seed/robot_filtered`），G1 训练不再需要覆盖该项。`robot_all`（过滤前全量）保留。
- G1 的 SMPL 动作库 `data/bones_seed_smpl` 仍不存在，冒烟时用 `sample_data/smpl_filtered` 覆盖（见 11.5 第 2 条）。
- `.gitignore` 新增 `sonic_v1_1/`、`low_latency/`（官方 G1 训练权重目录）；`git check-ignore` 确认 `sonic_v1_1/last.pt` 被忽略。`data/` 本来就整体被忽略，转换产物不会被提交。

### 11.7 下一步：Pico + MuJoCo 部署测试（用户已确认：训练逻辑与结果检测完成后进行）

用户要求在训练逻辑和结果检测全部完成后，用 **g1 和 a3 的权重**加 **Pico** 做 MuJoCo 仿真部署测试。各自使用现有链路：

| 机器人 | 链路 | 权重 | 现状 |
|---|---|---|---|
| a3 | `run_a3_teleop_sim.py --robot a3 --source pico`（Pico 推 `pose`:5556 → 重定向 → `a3_ref`:5560 → `sim2sim_a3_mujoco.py`） | 官方 035（PT） | 已用 Pico 模拟器实测，真实头显未测 |
| a3_ultra | 同上（`--robot a3_ultra`，035 合约） | 035 | 同上 |
| g1 | `run_sim_loop.py` + `deploy.sh --input-type zmq_manager sim` + `pico_manager_thread_server.py --manager`（官方原流程，操作步骤见 `docs/notes/mujoco_pico_sim_teleop.md`） | 官方 SONIC（`policy/release`，或 `--cp policy/sonic_v1_1/model` 的 v1.1） | **本机此前就能跑通**（已有 `.venv_teleop`、TensorRT、`g1_deploy_onnx_ref`）。改动后的回归验证见 11.10 |

**更正**：本节早先写"g1 链路本机未验证、要先确认 C++ 部署能否构建"是错误的——`docs/notes/mujoco_pico_sim_teleop.md` 记录了这条链路在本机已经跑通。用户已确认 g1 仿真走 `run_sim_loop.py` 原流程，并要求**先测 g1**，目标是：我对 GR00T-WholeBodyControl 的改动之后，g1 在 MuJoCo 里用 Pico 的流程必须依然可用。Pico 头显需用户本人操作。

### 11.8 剩余待办

1. （已完成，见 11.9）G1 冒烟用默认 `robot_filtered` 路径。
2. G1 的 SMPL 动作库 `data/bones_seed_smpl`（`sonic_v1_1.yaml` 默认读取）本地不存在，正式 G1 训练前需要准备；目前冒烟用的是样例数据。
3. 11.7 的 Pico + MuJoCo 部署测试（g1、a3），以及 g1 C++ 部署在本机能否构建运行的核实。
4. 真实 Pico 头显、G1 与 A3 Ultra 真机、`a3_ultra` 自研合约的部署导出、h2 的 C++ 后端：本机做不了（见 10.4、11.3）。
5. A3 Ultra 真机使用 035 前需人工确认的 4 点见 11.3。

### 11.9 G1 默认路径冒烟（已通过）

`--robot g1`（16 envs、2 次迭代）**不覆盖 `motion_file`**，仅覆盖本地缺失的 `smpl_motion_file`（指向 `sample_data/smpl_filtered`）：日志 `Loaded 129785 motion files from data/motion_lib_bones_seed/robot_filtered`，`exit=0`，训练循环正常结束。至此四个 `--robot` 中 `g1`、`a3`、`a3_ultra` 的训练冒烟都不再依赖手工覆盖 `motion_file`；`h2` 仍无本地动作库，未跑。

### 11.10 G1 回归验证：改动之后 MuJoCo + Pico 流程是否仍可用（2026-10-14）

**风险点**：相对干净基线 `60065f3`，G1 路径上的这些文件被改过——`gear_sonic_deploy/CMakeLists.txt`（+141）、`src/CMakeLists.txt`、`src/g1/CMakeLists.txt`、`g1_deploy_onnx_ref/CMakeLists.txt`（约 984 行变动）、`deploy.sh`、`run_sim_loop.py`、`configs.py`、`pico_manager_thread_server.py`。而本机现有 G1 可执行文件是 10 月 10 日编译的，**早于这些改动**，所以必须重新构建才算验证。

**做法与结果**（`.venv_teleop` + `TensorRT_ROOT=$HOME/TensorRT` + CUDA 13.2，与 `mujoco_pico_sim_teleop.md` 一致）：

| 步骤 | 结果 |
|---|---|
| 备份原 `target/release/*` 到会话临时目录（防重建失败） | 完成 |
| 全新目录里按 `just build` 的同样参数 `cmake -DCMAKE_BUILD_TYPE=Release` + `--build -j28`（**不带任何 A3 开关**） | 配置与编译均成功，0 个 error，`g1_deploy_onnx_ref` 重新链接成功（5,497,440 B，原为 5,498,080 B） |
| 终端 1：`run_sim_loop.py`（默认 `--robot g1`，`.venv_teleop`，真实 MuJoCo 窗口） | 正常运行，无 Traceback；仅有 DDS 回环网卡禁用多播的提示 |
| 终端 2：`deploy.sh --input-type zmq_manager sim`（走一遍 `just build`，重新编译到 `build/`） | 编译通过，**日志出现 `Init Done`**；ZMQ 输出接口、调试 socket（5557）初始化成功；日志中无 error/fail/segfault，仅有一条 CMake 的 `FetchContent DOWNLOAD_EXTRACT_TIMESTAMP` 开发者警告 |
| 终端 3：`pico_manager_thread_server.py --manager`（无头显） | 正常启动，初始化 XRT SDK 并连上本机 RoboticsService（127.0.0.1:60061），等待头显数据流。该文件相对基线只改了 `--robot` 的 `choices` |
| Python 侧 | 默认 `robot` 为 `g1`；`load_wbc_yaml()` 读到 `ROBOT_TYPE=g1_29dof` 与 `scene_43dof.xml`；`instantiate_robot_model("g1")` 得到 43 DOF 模型 |

**踩到的小坑**：`deploy.sh` 在启动前有 `read -p "Proceed with deployment? [Y/n]"` 确认；stdin 为空（后台/`</dev/null`）时 `set -e` 会直接 exit 1，不是故障。无人值守测试时用 `(echo y; sleep N) | ./deploy.sh ...` 并保持 stdin 打开。

**没有验证的**（需要用户本人带头显）：
- 真实 PICO 数据流经 `pico_manager_thread_server.py` → ZMQ → C++ 策略 → MuJoCo 的整条数据通路，包括 A+B+X+Y 启动、PLANNER/POSE 切换、站立与行走稳定性；
- `--cp policy/sonic_v1_1/model` 的 v1.1 变体（本轮只测了默认 `policy/release`）；
- G1 真机。

**结论**：除"真实头显数据"外，G1 的 MuJoCo 链路三个终端都能起来、C++ 部署到达 `Init Done`，没有发现我的改动造成的回归。这是"能启动"级别的证据，不等于"遥操表现和改动前一致"。

**建议用户的验证步骤**：按 `docs/notes/mujoco_pico_sim_teleop.md` 第 2、3 节走一遍。重新构建后的 `target/release/g1_deploy_onnx_ref` 已是新版本；若遇到异常，原二进制已备份（会话临时目录，不在仓库内，重启后会丢），也可 `git stash` 回退 CMake 改动后重建。

### 11.11 提交相关提醒

`git status` 显示 Phase 4 的 A3 遥操文件仍是**未跟踪**状态，从未提交：`gear_sonic/scripts/{a3_teleop_retarget_node,pico_pose_zmq_replay,run_a3_teleop_sim}.py`、`gear_sonic/utils/teleop/a3/`、`gear_sonic_deploy/.../{a3_ta_bridge_codec.hpp,a3_ta_bridge_codec.cpp,tools/,test_a3_ta_bridge_codec.cpp}`；另有若干已修改但未提交的文件（含 `g1_deploy_onnx_ref/CMakeLists.txt`、`.gitignore`、`sonic_a3_ultra.yaml`、`run_sim_loop.py`）。是否提交由用户决定。

### 11.12 `docs/notes/mujoco_pico_sim_teleop.md` 的命令统一加 `--robot g1`（2026-10-14，用户要求）

- 用户要求：训练和部署都要通过 `--robot` 指定机器人，所以 G1 的 MuJoCo + Pico 操作指南里的命令也必须显式带 `--robot g1`。
- 已修改 `docs/notes/mujoco_pico_sim_teleop.md`：终端 1 `run_sim_loop.py --robot g1`；终端 2 `./deploy.sh --robot g1 --input-type zmq_manager sim`（v1.1 变体和真机示例同样加上）；终端 3 `pico_manager_thread_server.py --robot g1 --manager ...`。并新增"用 `--robot` 指定机器人"一节，说明三个终端必须用同一个机器人，以及各自对 `a3` / `a3_ultra` / `h2` 的行为。顺带把 `Init done` 更正为实际日志里的 `Init Done`。
- **验证**：`deploy.sh --robot g1 --input-type zmq_manager sim` 与 v1.1 写法（`--robot g1 --cp policy/sonic_v1_1/model --obs-config ...`）都能正确解析并显示对应的 decoder/encoder/obs-config 路径，在确认提示处回答 `n` 后正常取消；`pico_manager_thread_server.py --robot g1 --manager` 运行 15 秒无报错（被超时终止）；`run_sim_loop.py --robot g1` 参数解析正常。

### 11.13 A3 真的没有 SMPL encoder 吗？A3-fast encoder 与 SMPL encoder 的区别（2026-10-14）

#### 结论：035 checkpoint 确实没有 SMPL encoder

这不是推断——`a3_runtime_config.yaml` 里有一行**明确的注释**：

```yaml
# Experiment 035 step200000 has G1 + A3-fast encoders, but no SMPL encoder.
```

`smpl_model_path` 这一行完全被注释掉（没有赋值），而 `model_path`（G1）和 `a3_fast_model_path` 都有实际路径。C++ 主程序里，SMPL policy 只有在 `smpl_onnx_path` 非空时才会加载（`load_smpl_monolithic_policy = smpl_zmq_enabled && !smpl_onnx_path.empty()`），配置里为空则永远不加载。

与此对应：
- **框架代码里有完整的 SMPL 支持**（`A3ZmqSmplSource`、`BuildSmplObsDict`、`kA3SmplObsDictTotalFloats=1770`、`a3_encoder_obs_builder.hpp` 里的 `kSmpl=1` 枚举）——这是为将来预留的接口，不是死代码。
- **真正缺少的是经过 SMPL encoder 训练的 ONNX 文件**：035 发布包只含 `model_step_200000_g1.onnx` 和 `model_step_200000_a3_fast.onnx`，没有 `_smpl.onnx`。
- **上游实时重定向模块未开源**（9.1 节已记录）：`a3_zmq_smpl_source` 需要另一个 SMPL ONNX；即使把这个 ZMQ source 单独起来，没有匹配的策略权重也跑不了推理。

#### A3-fast encoder 与 SMPL encoder 的本质区别

两者都是 `universal_token` 框架下的"编码器分支"，把各自的 tokenizer 输入压缩成同一个 64 维潜在 token，再送进公共的解码器（actor）。区别在于**信号来源**和**设计目的**完全不同。

| | A3-fast encoder | SMPL encoder |
|---|---|---|
| **信号来源** | A3 关节参考轨迹（运动数据库提前离线算好，或实时重定向后的轨迹） | 人体 SMPL 姿态（Pico 头显 + XRoboToolkit SDK 实时估计的全身 24 关节位置） |
| **tokenizer 输入维度** | 640 float：10 帧 × (29 关节位置 + 29 关节速度 + 骨盆 6D 朝向)（`reference_buffer.py` / `BuildDefaultStandTokenizerSlice`：290 + 290 + 60） | 840 float：10 帧 × 24 关节位置 (720) + 10 帧根朝向 6D (60) + 10 帧手腕关节 (60) |
| **obs_dict 总维度** | 1570 = 640(tokenizer) + 930(proprio_history) | 1770 = 840(tokenizer) + 930(proprio_history) |
| **帧率/时延** | skip 1，每帧真实推进，延迟约 220 ms（参考轨迹超前 10 帧×step1=200 ms） | **10 帧 × 0.02 s**（训练配置 `smpl_dt_future_ref_frames: 0.02`；部署 `kSmplFutureFrames=10`，每个控制 tick 推进 1 帧），窗口 0.2 s，延迟与 a3-fast 同量级。**勘误（2026-10-14，见 15.2）**：本格原先写的 “skip 5 / 约 940 ms” 是 `g1` 关节空间编码器的数字，不是 SMPL 编码器的 |
| **驱动方式** | **机器人关节空间**（直接描述"机器人下一步想去哪里"），人体姿态要先经重定向转成 A3 关节角才能送入 | **人体笛卡尔空间**（描述"人的身体现在什么姿态"），策略内部隐式学习人→机器人的映射 |
| **适用场景** | 跟踪运动数据库里的 A3 参考动作；高质量的运动数据库 + 实时重定向 → 高精度动作复现 | 实时遥操（人穿戴头显直接驱动机器人），无需预先制作机器人关节参考，遥操延迟由 SMPL 帧率决定 |
| **优势** | 延迟低；参考轨迹质量可由运动数据库和重定向器保证；训练数据量大（可用全量 Bones-SEED 或 A3 数据库） | 操作更直觉：人身体动机器人就动，不需要离线运动库；遥操链路短 |
| **劣势** | 需要高质量的关节参考轨迹；若重定向精度差（如骨盆倾斜 36°，见 9.4），跟踪 RMSE 急剧恶化 | 对 XRoboToolkit SDK 和 SMPL 估计精度依赖强；A3 没有公开发布该分支的 ONNX | 
| **035 是否有该分支** | ✅ 有（`model_step_200000_a3_fast.onnx`） | ❌ 无（`smpl_model_path` 注释掉，无 ONNX 文件） |

#### 为什么 A3 没有训练 SMPL encoder（推断）

G1 官方 SONIC 权重能直接拿 Pico 走 SMPL encoder，是因为 G1 的人体身形和 SMPL 关节空间与 G1 机器人关节的对应关系相对直接，且 G1 数据库里包含同步的 SMPL 标注（`data/bones_seed_smpl`）。A3 的 Litchi 项目专注在**低延迟的运动跟踪**场景，用 A3-fast encoder（skip 1）把响应延迟从 940 ms 降到 220 ms，这已经满足了 MPC/轨迹跟踪需求。A3 的人→机器人映射（SMPL → A3 29关节）比 G1 更复杂（腰部三轴、踝部并联机构），SMPL encoder 需要额外的训练数据和训练时长，Litchi README 把它归为"Current limitations / TODO"，不在 035 发布范围内。

#### 对本项目的影响

A3 的实时遥操链路（9.2 节）因此必须走**外部重定向**：Pico → XRoboToolkit → SMPL 24 关节 → GMR IK → A3 29 关节参考 → A3-fast encoder。这正是 `a3_teleop_retarget_node.py` 存在的原因：它把 SMPL 信号在策略侧"不可见"的情况下转成 A3-fast 能消化的关节参考轨迹，绕开了 A3 没有 SMPL encoder 这个限制。

## 12. `--robot a3|a3_ultra` 与 G1 对称：MuJoCo 仿真 + C++ 部署 + Pico（2026-10-14，用户要求）

> 用户要求：`run_sim_loop.py --robot a3` 要和 g1 效果一致——导入 a3_ultra 机器人模型、Pico 遥操成功、真机部署同理，并注册 a3 / a3_ultra / h2。已拍板：**A3 Ultra 的 MuJoCo 仿真用 A3 Ultra 自己的模型（T3D0 MJCF）以减小 sim2real gap，权重用 035（a3 与 a3_ultra 共用）**；**仿真器与 C++ 部署程序的接法严格参照 G1 的做法**。本节边做边记，未标"已验证"的都视为未完成。

### 12.1 设计：G1 是怎么接的，A3 照着做

| | G1（原有） | A3 / A3 Ultra（本节新增） |
|---|---|---|
| 仿真器 | `base_sim.py` + `unitree_sdk2py_bridge.py`：MuJoCo 里用 Unitree SDK2 DDS 冒充机器人 | `gear_sonic/utils/mujoco_sim/a3_sim.py`：MuJoCo 里用 A3 body-drive ROS2 话题冒充机器人 |
| 机器人侧接口 | DDS `rt/lowstate` / `rt/lowcmd` | ROS2 `/body_drive/{leg,waist,arm,neck}_joint_state`、`/body_drive/{pelvis,torso}_imu/data`、`/body_drive/*_joint_command`（`joint_msgs/JointState|JointCommand`、`sensor_msgs/Imu`） |
| C++ 部署程序 | `g1_deploy_onnx_ref`，**sim 与 real 同一个二进制** | `a3_deploy_onnx_ref`，**sim 与 real 同一个二进制，未改一行 C++** |
| 力矩 | `tau = tau_ff + kp(q_des-q) + kd(dq_des-dq)` | 同一公式；kp/kd/tau_ff 由部署程序在指令里逐关节下发（035 合约） |
| 悬挂 | 弹性绳，按 `9` 释放，`7/8` 调长度 | 同样（`ElasticBand` 的 A3 版，无 unitree 依赖） |
| 入口 | `run_sim_loop.py --robot g1` | `run_sim_loop.py --robot a3\|a3_ultra`（见 12.3） |

要点：
- 配置在 `gear_sonic/utils/mujoco_sim/wbc_configs/{a3_035,a3_ultra_035}.yaml`，经 `ROBOT_WBC_YAML` 注册（`a3`→T2D5 MJCF，`a3_ultra`→T3D0 MJCF）。
- A3 Ultra 的头部俯仰关节在 MJCF 里叫 `ultra_head_pitch_joint`，话题里仍按 A3 的名字 `head_pitch_joint` 发布（`JOINT_ALIAS`）。20 个手指关节是位置执行器，仿真里保持关键帧值。
- 默认物理步长 0.002 s（G1 是 0.005），状态以 250 Hz 发布（部署侧要求每路样本新鲜度 < 10 ms）；`--sim-frequency` 不是默认值时以命令行为准。

### 12.2 ROS2 环境（一次性）

A3 部署程序走 ROS2，仿真节点需要同一个解释器里同时有 rclpy、`joint_msgs` 的 Python 绑定和 mujoco；G1 的 `.venv_teleop` 没有 ROS2。新增 `gear_sonic_deploy/scripts/setup_a3_sim_env.sh`：向既有 conda 环境 `a3_deploy_verify`（Phase 3 为编译部署包建的）里 `pip install mujoco scipy tyro`、构建并复制 `joint_msgs` Python 绑定、给其 `.so` 设置 RUNPATH。`run_sim_loop.py --robot a3|a3_ultra` 在当前解释器缺 rclpy 时会自动 `execve` 到该环境的 python（可用 `A3_SIM_PYTHON` 覆盖），所以仍然可以从 `.venv_teleop` 里直接敲 `run_sim_loop.py`。

> 用户环境改动：`a3_deploy_verify` 里新增了 mujoco 3.15、scipy、tyro（及其依赖）与 `joint_msgs` Python 包；numpy 仍为 1.26.4。

### 12.3 联调结果（无显示器、无头模式，`A3_SIM_BAND_RELEASE_S=22` 22 s 后自动释放弹性绳）

用**未改动的** `a3_deploy_onnx_ref --auto-start`（ROS2 传输，`ROS_DOMAIN_ID=87`、`ROS_LOCALHOST_ONLY=1`，`systemd-run` 限内存 3G）连 MuJoCo：

| 模型 | 结果 |
|---|---|
| `a3`（T2D5） | 同步 `sync=ok`，约 50 Hz 策略；释放后站稳，骨盆高度 1.07 m，倾角 ≈1° |
| `a3_ultra`（T3D0，用户指定的自身模型） | 首轮释放后立即倒地；定位修复后同样站稳 60 s 以上：骨盆高度 1.075 m，倾角 0.4–1.0°，`safe_halts=0` |

**联调中发现并修复的两个真实问题：**
1. **A3 Ultra 的 MJCF 地面不参与碰撞**（`contype=0 conaffinity=0`，是 worklog #3 当时手写内联地面时遗漏的）——机器人松开弹性绳后直接穿过地面一路掉到 z=-147。已把 `floor` 改为 `contype="1" conaffinity="7"`（与 T2D5 一致），**四份拷贝都改了**：`gear_sonic/data/robot_model/model_data/a3_ultra/`、`gear_sonic/data/assets/robot_description/mjcf/`、`decoupled_wbc/sim2mujoco/resources/robots/a3_ultra/`、`/home/user/gmr_repo/assets/agibot_a3_ultra/`。这也说明此前任何依赖该 MJCF 地面的验证都不成立（GMR 重定向只做 IK、不涉及接触，不受影响）。
2. **弹性绳悬挂高度**：T3D0 关键帧的骨盆高度是 1.2 m，比 035 默认站姿的实际高度（≈1.07 m，两个模型的脚底参考一致）高 13 cm，释放时自由下落。yaml 里新增 `BAND_HEIGHT: 1.07` 并让复位时按它起始。另外自动倒地复位后会重新挂回弹性绳（需要再按 `9` 释放）。

**这条链路验证了什么、没验证什么：**
- 验证了：部署程序在仿真里收得到全部 6 路状态、发得出全部 4 路指令、035 策略（`a3_fast`/`g1` 编码器的站立回退）下的站立稳定，且 A3 Ultra 自身 MJCF 可以承载 035 策略。
- **未验证**：步行/动作回放、Pico 遥操数据流（下面 12.4 做）、带显示器的 viewer 与键盘释放、真机。T3D0 与 T2D5 仿真里的 `armature`/摩擦等物理差异对动作跟踪的影响没有量化。

### 12.4 注册 `a3` / `a3_ultra` / `h2`（已完成并验证）

- `ROBOT_MODEL_REGISTRY`（`gear_sonic/data/robot_model/instantiation/__init__.py`）现在是 `g1 / a3 / a3_ultra / h2`：
  - `a3_ultra`：`model_data/a3_ultra/a3_ultra_t3d0.urdf`（63 DOF = 身体 29 + 头 2 + 手指 32；只把 29 个身体关节作为"actuated"，手指/头不由 035 策略驱动）；
  - `a3`：`assets/robot_description/urdf/a3/model_hard_sole_3seg.urdf`（29 DOF）；
  - `h2`：`urdf/h2/h2.urdf`（31 DOF，身体 29 + 头 2）。
  新增 `supplemental_info/a3/a3_supplemental_info.py`、`supplemental_info/h2/h2_supplemental_info.py`、`instantiation/a3.py`。
- `get_g1_key_frame_poses`（Pico 管理器在 A+B+X+Y 启动时的 3 点标定会调用它）原先写死 G1 的链接名，对 A3 会直接抛 `RuntimeError`；现在优先读机器人 `supplemental_info.key_frame_mapping/offsets`（A3 的链接是大写 `_Link`，局部偏移 G1 专属、A3 未量测故为 0），G1/H2 行为不变。**3 点（VR_3PT）子模式对 A3 没有调过**，A3 链路只用 POSE 流。
- `pico_manager_thread_server.py --robot a3|a3_ultra|h2`：能启动并连上 RoboticsService；`--vis_vr3pt` 的 G1 网格叠加对其它机器人自动关闭。
- `ROBOT_WBC_YAML`：`a3`、`a3_ultra` 已注册（12.1）。**`h2` 没有注册仿真配置**：H2 没有策略权重也没有 C++ 部署后端，本机无法闭环验证，而仿真配置如果只是"能加载"并没有意义，所以只注册了机器人模型（IK/FK/Pico 标定用）。`run_sim_loop.py --robot h2` 仍会给出明确报错。
- 验证：四个机器人都能实例化并做关键帧 FK；`tests/test_a3_sim_model.py::test_robot_registry_has_all_robots`、`test_key_frames_follow_the_robot`。

### 12.5 一个此前漏掉的重定向缺陷：默认尺度让人"蹲着"（已修复，并更正 9.4 / 9.5）

联调时发现：Pico/模拟器里一个**站着不动**的人，在 MuJoCo 里骨盆高度只有 0.83–0.86 m（站立应为 1.07 m）。离线排查（站立 SMPL → 重定向器）：

| `human_height`（GMR 尺度参考） | 膝角 | 说明 |
|---|---|---|
| 1.66（旧默认，SMPL 模板真实身高） | 84° | 骨盆目标 0.76 m，深蹲 |
| 1.95 | 52° | |
| 2.18（新默认） | 12° | 与 035 默认站姿（膝 14°）一致 |
| 2.40 | 3° | 近乎伸直 |

根因：A3 比 SMPL 模板人大得多（站立骨盆 1.07 m vs 约 0.9 m），GMR 用 `actual_human_height / 1.8` 统一缩放，旧默认值没有把这个差距补回来，策略忠实地复现了这个蹲姿。它与 worklog #25 对 BVH 管线发现的是同一类问题。`DEFAULT_HUMAN_HEIGHT_M` 改为 2.18（含说明：它是尺度参考，不是操作者身高）。修复后同一测试里骨盆高度 1.073–1.075 m，倾角 1–2°。

**对 9.4 / 9.5 的更正：**
- 9.4 里"GMR 默认骨盆权重让站立的人骨盆倾斜 36–41°"，大半是这个蹲姿造成的假象；尺度修正后，库存权重（`pelvis_rot_weight=None`）下倾角也 <5°。保留 50 的默认值（无害，更保险）。原先"修复前 0.274 rad 的跟踪 RMSE"之类的数字是在蹲姿参考上测的，不再代表当前行为。
- 9.5 表格里的结果（未摔倒、倾角 4–8°）仍然成立，但它们是**蹲姿**下的结果。
- 测试：`test_stock_gmr_weight_tilts_pelvis` 记录的是旧假象，已替换为 `test_default_scale_reference_gives_the_policy_default_stand` 与 `test_literal_template_height_crouches`。

**仍未量化**：手臂缩放（GMR 表里手臂 0.8 vs 腿 0.9）对到达范围的影响；真实操作者身高/体型不同带来的差异（Pico 只给 SMPL 模板，不估计身体比例）。

### 12.6 Pico 全链路联调（无头显，用 Pico 模拟器；已验证）

按 `docs/notes/mujoco_pico_sim_teleop.md` 第 8 节的三终端流程（终端 3 换成 `pico_pose_zmq_replay.py`），用 pty 脚本按 `S → M → T → 3` 操作部署程序（部署程序要求 TTY，管道输入无效，日志会提示 `stdin is not a TTY`）：

| 输入 | 结果（`a3_ultra`，T3D0 模型，035 权重，ROS2 域 87） |
|---|---|
| `synthetic:wave`（站立 + 右臂挥手），70 s | 四个模式切换全部接受（`idle→pd_stand→motion→teleop`，源 `a3_fast`）；右臂肩 roll 在 -0.4 ~ -1.5 rad 间周期摆动；骨盆高度 1.073–1.075 m，倾角 1–2.3°；未摔倒 |
| 真实行走 SMPL 片段 `walk_forward_amateur_001__A001.pkl`，约 70 s | 未摔倒；骨盆高度 1.02–1.08 m，倾角最大 5.6°；腿部按参考膝角 3–48° 交替运动（原地踏步式，遥操帧不含骨盆平移）；片段后段人体抬起双臂，机器人跟着抬起（是片段内容，不是 IK 异常，离线重定向在同一帧给出同样的肩角） |

结论：Pico → `pico_manager` 的 `pose` 流 → 重定向 → `a3_ta_bridge` → ROS2 `/ta/whole_body_command` → 未改动的 `a3_deploy_onnx_ref` → ROS2 `/body_drive/*` → MuJoCo，整条链路（用模拟器替代头显）可用，结构与 G1 的三终端一致。

### 12.7 仍然没有验证的（上真机/用真头显前必须知道）

- **真实 PICO 头显**：pico_manager 只验证了启动和连接 RoboticsService，没有真实头显数据；A+B+X+Y / A+X 的手柄操作未试。
- **A3 Ultra 真机**：仿真与真机虽然用同一个二进制、同一套话题，但真机的实际话题布局未核实——尤其是 `/body_drive/*` 在 A3 Ultra 上是否与仿真一致、头部（`ultra_head_pitch_joint`）和灵巧手的话题、真机的 IMU 坐标系/加速度符号约定。仿真里的 IMU 按"世界系朝向 + 机体系角速度 + 比力"发布，是对真机 `sensor_msgs/Imu` 的推断。
- 035 在 A3 Ultra 上的差异（plan 11.3）：hip_roll 限位 ±85° vs 92°、整机多约 2 kg（仿真里 T3D0 模型已含这 2 kg，所以仿真比 11.3 的担心更接近真机，但没有真机数据对照）。
- 仿真物理差异没量化：A3 Ultra MJCF 的 armature 是 worklog #20.4 注入的、T2D5 MJCF 的 armature 为 0；没有测它们对跟踪误差的影响。
- 只测了 `a3_fast` 编码器源（键 `3`）；`1`（G1 编码器，940 ms 延迟）源经新链路未测。
- `a3_ultra` 自研 63 DOF 合约仍没有部署导出；H2 没有仿真配置也没有 C++ 后端。
- 带窗口的 viewer（按 `9` 释放弹性绳、`7/8` 调长度）没有在有显示的环境里实际按过键，只测了无头 + 定时释放（`A3_SIM_BAND_RELEASE_S`）。

### 12.8 新增 / 修改的文件（本节）

| 文件 | 内容 |
|---|---|
| `gear_sonic/utils/mujoco_sim/a3_sim.py`（新） | A3 MuJoCo 仿真 + ROS2 桥 + 弹性绳 |
| `gear_sonic/utils/mujoco_sim/wbc_configs/{a3_035,a3_ultra_035}.yaml`（新）、`configs.py` | A3 仿真配置；`ROBOT_WBC_YAML` 注册 a3/a3_ultra；保留机器人专属步长 |
| `gear_sonic/scripts/run_sim_loop.py` | `--robot a3\|a3_ultra` 走 A3 仿真（自动切到 ROS2 环境）；G1 路径改为延迟导入，行为不变 |
| `gear_sonic_deploy/scripts/setup_a3_sim_env.sh`（新） | 一次性准备 ROS2 Python 环境 |
| `gear_sonic_deploy/scripts/deploy_a3.sh` | `sim`/`real` 目标、`--input-type zmq_manager`（自动起重定向 + 桥）、忽略 G1 专属 `--output-type` |
| `gear_sonic/data/robot_model/...`（新 4 个文件 + 注册） | A3 / A3 Ultra / H2 机器人模型 |
| `gear_sonic/utils/teleop/vis/vr3pt_pose_visualizer.py` | 关键帧映射按机器人读取 |
| `gear_sonic/scripts/pico_manager_thread_server.py` | 非 G1 时关闭 G1 网格叠加 |
| `gear_sonic/utils/teleop/a3/retarget.py`、`tests/test_a3_teleop_retarget.py` | 尺度参考 1.66→2.18 及测试 |
| 4 份 `a3_ultra_t3d0.xml` | 地面碰撞修复 |
| `tests/test_a3_sim_model.py`（新，9 个测试） | 模型/地面/IMU/注册表回归 |
| `docs/notes/mujoco_pico_sim_teleop.md` | 新增第 8 节（A3 三终端流程） |

**回归**：`tests` + `gear_sonic/tests` 共 187 个测试通过（排除一个因缺 `msgpack_numpy` 无法收集的既有问题）；G1 的 `run_sim_loop.py --robot g1`、`pico_manager_thread_server.py --robot g1` 重构后仍能启动。

## 13. `--robot a3_ultra` 的机器人参数与 Litchi 的 a3 对齐（2026-10-14，用户要求）

> 用户要求：`--robot a3_ultra` 训练、部署用到的机器人参数（如 Kp/Kd）与 `Litchi_sonic_for_a3` 里的 a3 保持一致，**先不要用官方文档**（`docs/robot_parameters/A3_Ultra/A3_关节控制参数KpKd.docx`）。

### 13.1 之前不一致的地方

| 位置 | `--robot a3_ultra` 之前 | Litchi a3（`a3_024`，035 检查点的合约） |
|---|---|---|
| 训练 `a3_ultra.py` | 官方文档的 Kp/Kd：髋 200/7、膝 360/13、踝 80/3、腰 200/7 & 80/3、肩 90/4、臂 40/3、腕 pitch/yaw 15/1 | 髋 80–120/3–4、膝 250/8、踝 60/1.2、腰 85/3 & 50/2、肩 40/3、臂 30/2、腕 pitch/yaw 20/2 |
| 训练：力矩上限 | 腰 220/46/118、膝 320 | 腰 200/42/105、膝 300 |
| 训练：摩擦 / 动摩擦 | 未设置 | 髋/膝/腰 0.15/0.1、踝 0.25/0.2、臂 0.1/0.1 |
| 训练：默认站姿 / 出生高度 | MJCF 关键帧（髋俯仰 -0.2024、膝 0.2365…，z=1.2） | 髋俯仰 -0.1311、膝 0.2468…，z=1.09 |
| 训练：动作缩放 | 由上面的 effort/stiffness 推出 | 同公式，用 `a3_024` 的值 |
| **部署**（`deploy.sh --robot a3_ultra`） | 已经是 Litchi a3（`policy_control.profile: a3_024`，`a3_024_kps/kds/action_scale`） | 同 |
| MuJoCo 仿真 | Kp/Kd 由部署程序在指令里下发（所以已经是 Litchi a3）；MJCF 的 `wrist_roll` 转子惯量 0.00081 | `wrist_roll` 为 0.004967 |

也就是说，**部署侧本来就是一致的，不一致的是训练侧**，以及 MJCF 里一个遗留的转子惯量笔误。

### 13.2 改了什么

1. **`gear_sonic/envs/manager_env/robots/a3_ultra.py`**：新增开关 `A3_ULTRA_GAIN_SOURCE`，默认 `"litchi_a3_024"`：
   - 29 个身体关节的执行器组（`legs / feet / waist / arms`）直接 `deepcopy` 自 `a3.py` 的 `A3_024_CYLINDER_CFG`——刚度、阻尼、力矩/速度上限、转子惯量、摩擦、动摩擦都是同一份对象的拷贝，不再手抄，所以不会与 Litchi 的定义产生漂移；
   - 默认站姿与出生高度取自 `A3_024_CYLINDER_CFG.init_state`（只取 29 个身体关节；头关节为 0，与部署代码"头固定在 q=0"一致）；
   - 被动脚弹簧组不带过来（A3 Ultra 没有这些关节）；
   - 头关节（Litchi 的 A3 URDF 没有头）：Kp=40 / Kd=2，取自部署代码固定的 `kA3HeadKp / kA3HeadKd`（恰好也等于官方文档的值）；力矩 6 Nm、速度 12.78、转子惯量 0.00081 沿用原值（Litchi 没有对应数据）；
   - 动作缩放仍按 `0.25 × effort / stiffness` 推出，29 个关节与 `A3_024_ACTION_SCALE` 逐个相等；
   - **官方文档的数值没有删除**，原样保留在 `_vendor_doc_actuators()` / `_vendor_doc_init_state()`，把 `A3_ULTRA_GAIN_SOURCE` 改成 `"vendor_doc"` 即可切回（"先"不用的含义）。
2. **4 份 `a3_ultra_t3d0.xml`**（仿真、动作库 FK、`decoupled_wbc`、GMR）：`left/right_wrist_roll_joint` 的 `armature` 0.00081 → 0.004967。这是 worklog #38 已在 `a3_ultra.py` 里修过、但 MJCF 里当时漏掉的同一个笔误；改完 29 个身体关节 + 头的 MJCF 转子惯量与训练合约逐个一致。
3. `sonic_a3_ultra.yaml` 头注释、`train_agent_trl.py` 里 `ROBOT_EXP_MAP` 的注释、`deploy_a3.sh --robot a3_ultra` 的提示文字：改成如实描述上面的状态。

### 13.3 验证

- 新增 `tests/test_a3_ultra_contract.py`（15 个测试），把三处"同一份数字"锁住：
  1. `a3_ultra.py` 的 29 个身体关节与 `A3_024_CYLINDER_CFG` 逐项相等：stiffness、damping、effort、velocity、armature、friction、dynamic_friction；动作缩放、默认站姿、出生高度也相等；
  2. 用 `g++` 编译 C++ 部署头文件 `a3_policy_parameters.hpp`，读出 `a3_024_kps / kds / action_scale / a3_default_angles`，与 A3 Ultra 训练配置逐关节比较——**训练 = Litchi a3 = C++ 部署**三者一致；
  3. MJCF 转子惯量 = 训练合约；头关节 40/2/q=0；保留的 vendor_doc 路径仍可构造。
  这些测试用桩替换 isaaclab 配置类、在子进程里读取数字，不需要启动 Isaac Sim。
- `--robot a3_ultra` 训练冒烟（16 envs、2 次迭代，不加任何覆盖）：`exit=0`，加载 129,116 个动作，`Total number of actuated joints 31 != 63`（31 = 29 身体 + 2 头，32 个手指关节本来就不驱动），说明每个执行器组的正则恰好匹配到预期的关节。
- 仿真：A3 Ultra 仿真 + 未改动的 `a3_deploy_onnx_ref --auto-start` 重跑（转子惯量改动之后）：站稳，骨盆 1.075 m，倾角 0.2–0.4°；部署日志 `policy control profile selected: a3_024 ankle_kp=60 ankle_kd=1.2 ankle_pitch_action_scale=0.4925`。
- 回归：`tests` + `gear_sonic/tests` 共 202 通过、1 跳过（`gmr` 环境下的重定向测试 14 个另行通过）；`--robot g1` 训练冒烟（`robots` 包的导入链变了，所以重跑）`exit=0`，加载 129,785 个动作。

### 13.4 仍然不一致（有意没有动，供你决定）

- **（2026-10-15 已被第 16 节取代：`--robot a3_ultra` 现在训练 A3 的 29 关节合约，下面说的 31 个动作是改名后的 `sonic_a3_ultra_t3d0`。）动作空间**：Litchi a3 的策略只输出 29 个身体关节；`--robot a3_ultra` 的训练动作空间仍是 31 个（多 2 个头关节，`sonic_a3_ultra.yaml` 显式列出）。部署代码把头固定在 q=0，所以一个从零训练出来的 a3_ultra 策略如果会动头，部署时这部分输出会被忽略。要对齐的话需要把头关节移出动作空间，并把动作库 `dof` 从 31 列裁到 29 列。
- **其它训练合约**：`a3_ultra` 仍是自研的 SMPL 跟踪合约（观测/编码器/奖励/终止条件/动作数据与 035 不同，也没有 A3 电机约束类奖励）；这次只对齐了"机器人参数"。
- `articulation_props.enabled_self_collisions`：A3 Ultra 为 `True`，Litchi a3 为 `False`；与控制参数无关，没有改。
- **仿真侧的物理参数**：MJCF 里的关节阻尼/库仑摩擦用的是厂商值，仿真里的力矩裁剪用 MJCF 厂商上限（膝 320、腰 220/46/118），而训练合约是 300 / 200/42/105；Kp/Kd 本身来自部署程序所以一致。这些差异只在力矩饱和时才有影响，没有量化。
- 头关节的力矩上限/速度上限/转子惯量没有 Litchi 对应值，沿用原先的数据。
- 一个从零训练的 `a3_ultra` 策略仍没有部署导出；但低层合约（Kp/Kd、动作缩放、默认站姿）与部署已经相同，导出后可直接用 `a3_024` profile。
- 官方文档的数值以后要用时：改 `A3_ULTRA_GAIN_SOURCE = "vendor_doc"`，同时部署侧要有对应的 profile（目前 C++ 只有 `a3` / `a3_024`），否则训练与部署又会不一致。

## 14. A3 Ultra 直接使用下载的 A3 权重：兼容性确认、头部固定、验证数据（2026-10-14，用户要求）

> 用户的三点：① Litchi 的 README 里也写了 a3_ultra，确定是否有可下载的权重真的能兼容 a3_ultra；② 如果实在没有，就把 a3_ultra 的两个头关节固定在默认姿态（和 a3 一致），其余关节输出与 a3 一致；③ 目前不训练自己的 a3_ultra sonic 模型，直接使用下载的 a3 权重。

### 14.1 结论

1. **有，而且只有一个**：Hugging Face `sonic-for-a3/sonic` 的 `035_step200000`（PT / 4 个 ONNX / 2 个 RKNN）。整个 HF（该组织只有这一个仓库、只有 `main` 分支、没有 tag）和 GitHub（`AgibotTech/sonic_for_a3`，没有 release、只有 `main`）上**没有 A3 Ultra 专属权重**。官方**明确声明它同时支持 A3 与 A3 Ultra**，所以第 ② 点的"没有兼容权重时的退路"不是必须的——但这个退路描述的正是这套权重在 A3 Ultra 上的实际用法（见 14.4），已经落实并写成了默认行为。
2. 除了官方声明，本机还做了接口、质量、关节限位、域随机化覆盖和 MuJoCo 闭环的核验（14.3）：A3 Ultra 的 T3D0 模型与 A3 的 T2D5 模型在**同一个部署二进制、同一组权重、同一套增益**下，7 个官方参考动作的跟踪误差几乎相同。**真机没有验证**（14.5）。
3. 不训练自己的 a3_ultra 模型：`--robot a3_ultra` 的仿真与部署现在就是"下载的 035 权重 + A3 Ultra 自己的模型"，一条命令即可（14.6）。

### 14.2 官方来源（原文）

- `README.md`（本地 `Litchi_sonic_for_a3/README.md`，与 GitHub `AgibotTech/sonic_for_a3` 的 `main` **一致**（忽略行尾空白后 diff 为 0 行））：
  - "Although this checkpoint was trained using the A3 URDF, its domain randomization was specifically tuned to support both **A3 and A3 Ultra (A3U)**. The corresponding models can therefore also run on A3 Ultra."
  - "…a Drive Thor target for A3 Ultra through the Thor runtime variant. **The Rockchip / RKNN deployment workflow also supports A3 Ultra (A3U)**; A3 Ultra can use either the Rockchip deployment path or the Thor runtime variant, depending on the target hardware."
  - Future TODO："Add a detailed deployment-on-Thor tutorial for the A3 Ultra Thor runtime."（**Thor 教程没有公开**）
- HF 发布元数据 `config.json`：`"robots": ["A3", "A3 Ultra"]`、`"policy_joints": 29`、`"encoders": ["g1", "a3_fast"]`、`"source_repository": "https://github.com/AgibotTech/sonic_for_a3"`。
- `035_step200000/README.md`：完整策略 `obs_dict: float32[1,1570]` → `action: float32[1,29]`；"`g1` 只是历史接口名，这些文件控制的是 A3"。
- 说明：官方声明的依据是"域随机化针对 A3 / A3U 调过"，这是设计声明，不是我方在真机上的验证。

### 14.3 本机核验

**① 权重来源**：`checkpoints/035_step200000/model_step_200000.pt` 与部署用的 `model_step_200000_g1.onnx`、`model_step_200000_a3_fast.onnx`（`assets/` 与 `dist/` 两处）的 SHA256 与 HF 上**当前**的 `SHA256SUMS` 逐个一致，即仿真里跑的就是官方发布的文件。

**② 接口**：策略只输出 29 个身体关节（腿 12 + 腰 3 + 臂 14），A3 与 A3 Ultra 的关节名、顺序都相同；A3 Ultra 多出的 2 个头关节与 32 个手指关节不在策略输出里。

**③ 关节限位**（A3 训练 URDF vs A3 Ultra URDF，29 个关节）：只有 3 类不同——`hip_roll` 上限 A3 为 ±92°、A3 Ultra 为 ±85°；`knee` 上限 143° vs 145°；其余 26 个完全相同。训练时 `joint_soft_pos_limits` 把限位收到 80%，即 A3 的 hip_roll 训练范围到 79.8°，仍落在 A3 Ultra 的 85° 之内；**29 个关节中没有任何一个的 A3 Ultra 范围小于策略训练时见过的范围**。

**④ 质量与域随机化覆盖**（URDF 质量，035 `config.yaml` 的随机化范围）：

| 项 | A3 | A3 Ultra | 035 的随机化 | 是否覆盖 |
|---|---|---|---|---|
| 整机 | 57.98 kg | 60.01 kg（+2.03 kg，+3.5%） | — | — |
| `torso_Link` | 17.445 | 18.094（×1.037） | 质量缩放 [0.8, 1.3]；质心 x±2.5 y±5 z±5 cm | 是 |
| 脚（踝 roll 连杆） | 0.653 + 被动脚 0.094 = 0.747 | 0.747（刚性脚，质量合并） | 其它连杆 ×[0.9, 1.1] | 完全相同 |
| 手（腕外侧） | `hand_Link` 0.200 | 手指 0.535 + 手部相机 0.169 = 0.704（约 +0.50 kg/侧） | `hand_payload_mass` 腕上加质量 [-0.2, +2.0] kg | 是 |
| 头 | 2.100（yaw 0.565 + pitch 1.535） | 2.472（+0.372 kg） | 其它连杆 ×[0.9, 1.1] | pitch 连杆 ×1.089 在范围内；yaw 连杆 +0.235 kg 超出比例范围，但绝对量很小 |
| 转子惯量 / 摩擦 | 合约值 | 完全相同（worklog #37、本节 13 已核对） | armature ×[0.8, 1.2]、摩擦 ×[1, 3] | 是 |

**⑤ MuJoCo 闭环**（`run_sim_loop.py --robot {a3,a3_ultra}` + 未改动的 `deploy.sh`，按键 `S → M → [ ] 选动作 → R` 播放官方 CSV；MuJoCo 里弹性绳 22 s 释放；跟踪误差 = 机器人 29 个关节相对参考 CSV 的 RMSE，播放起点用最小误差搜索，两个模型同样处理；A3 的 T2D5 MJCF 里 armature 为 0，仿真器按合约值补上，使两者可比）：

| 官方动作（时长） | RMSE 全部 / 腿 / 臂（rad）A3 | RMSE 全部 / 腿 / 臂（rad）**A3 Ultra** | 骨盆最低 / 最大倾角 A3 | 骨盆最低 / 最大倾角 **A3 Ultra** |
|---|---|---|---|---|
| 001 前行慢走（33 s，补测） | 0.089 / 0.116 / 0.068 | 0.087 / 0.111 / 0.070 | 1.030 m / 9.5° | 1.035 m / 8.4° |
| 009 向左快走（23.5 s） | 0.079 / 0.111 / 0.044 | 0.079 / 0.111 / 0.045 | 1.009 m / 6.8° | 1.013 m / 6.9° |
| 043 深蹲连做（12.8 s） | 0.082 / 0.087 / 0.086 | 0.086 / 0.094 / 0.087 | 0.565 m / 10.4° | 0.566 m / 10.5° |
| 047 左单腿平衡（11.8 s） | 0.057 / 0.062 / 0.059 | 0.056 / 0.059 / 0.060 | 1.049 m / 5.8° | 1.053 m / 4.6° |
| 050 原地踏步（17.6 s） | 0.054 / 0.072 / 0.040 | 0.055 / 0.073 / 0.041 | 1.050 m / 6.1° | 1.055 m / 6.5° |
| 055 前弓步交替（29.2 s） | 0.092 / 0.128 / 0.056 | 0.092 / 0.128 / 0.056 | 0.906 m / 15.9° | 0.912 m / 16.3° |
| 058 前出拳交替（13.5 s） | 0.099 / 0.058 / 0.132 | 0.101 / 0.062 / 0.132 | 1.060 m / 6.6° | 1.066 m / 6.1° |
| 077 双臂上举（14.8 s） | 0.077 / 0.061 / 0.096 | 0.079 / 0.064 / 0.097 | 1.069 m / 6.4° | 1.074 m / 7.0° |

16 次运行（2 个模型 × 8 个动作）全部完整播放、没有摔倒——逐个核对过，日志里所有"robot fell"都出现在部署程序退出、指令停止之后，指令期间 0 次；001 的骨盆轨迹沿参考路径"前行约 2.9 m 再返回"。**全关节 RMSE 的差异 ≤0.004 rad**（腿部最大差 0.008 rad，出现在深蹲，其余更小）。

**这组数据的含义和局限**：它说明在同一条软件/合约链路下，策略在 A3 Ultra 的质量、关节限位、手/头结构上没有退化；但**两个仿真模型都是简化的串联踝/腰 + 刚性脚**（策略是在被动脚 + 并联闭环模型上训练的），所以绝对误差（约 0.09 rad）高于 Litchi 自己 sim2sim 在闭环模型上报的 0.066（它的闭环 MJCF 无法加载 A3 Ultra 的 T3D0 模型，这就是本机用自己的仿真器的原因）。它是 A3 Ultra 相对 A3 的比较，不是真机保证。

### 14.4 头部固定在默认姿态（第 ② 点）与"其余输出与 a3 一致"

- **部署**（本来就是这样，核对后确认）：`ExpandToBackend` 把两个头关节的指令固定为 q=0、Kp=40、Kd=2（`kA3HeadTargetPositionRad / kA3HeadKp / kA3HeadKd`），29 个身体关节用策略输出 × `a3_024` 动作缩放 + 默认站姿、`a3_024` 的 Kp/Kd；官方参考 CSV 的头部两列恒为 0。
- **本轮修了一个会破坏"固定"的细节**：Pico 重定向（`a3_teleop_retarget_node.py` → `a3_ref` → `/ta/whole_body_command`）此前会把 GMR 逆解里两个头关节的值也作为 `has_head=1` 的头部指令发出去，部署程序遇到带头的帧会用它覆盖固定的 q=0。逆解里没有头部任务，这些值只是求解器的残留，不应该驱动头。现在 `A3RealtimeRetargeter(track_head=False)` 为默认，帧里不带头（`has_head=0`），部署程序保持 q=0；需要时用 `a3_teleop_retarget_node.py --track-head` 显式打开。新增测试 `test_head_stays_at_its_default_pose_unless_tracking_is_requested`。
- **仿真侧**：A3 的 T2D5 MJCF 头关节没有力矩限制信息（范围 `[0, 0]`），原先被当成 0 Nm 裁剪，头在仿真里会甩动；现在没有力矩限制的关节不裁剪，部署程序下发的 Kp=40 / Kd=2 把它保持在默认姿态。验证：A3 Ultra 播放官方动作时头关节 pitch 最大偏差 ≤0.026 rad（8 个动作）、yaw ≤0.005 rad（只在 001 上统计）（pitch 的小偏差是约 0.6 Nm 的重力负载在 Kp=40 下的稳态误差）；Pico 挥手链路里头关节读数 `(0.000, 0.015)` rad。
- **训练合约没有改（2026-10-15 更新：第 16 节已改成 A3 合约）**：`--robot a3_ultra` 的从零训练动作空间当时仍是 31 个关节（含头），因为用户当前不训练自己的 a3_ultra 模型；如果以后训练，要让动作空间与下载的权重一致，需要把头关节移出动作空间并把动作库 `dof` 从 31 列裁到 29 列（13.4 已记录）。

### 14.5 仍然没有验证 / 上真机前必须确认

1. **真机**：官方声明 + 本机仿真比较，不能替代真机。首次上机按 `docs/a3_training2sim2deploy.md` 6.1–6.2（安全龙门架/吊带、MDU 接管、`--probe` 无指令探测、`taskset -c 4-5`、`S` PD 站立后再 `M`）。
2. **头部话题的关节命名**：部署程序期望 `/body_drive/neck_joint_state` 里的关节名是 `head_yaw_joint`、`head_pitch_joint`，并按名字匹配、**缺这一路状态会让同步一直不就绪、策略不启动**；而 A3 Ultra 的 URDF/MJCF 里俯仰关节叫 `ultra_head_pitch_joint`。本机仿真里我按 A3 的名字发布；真机上要先看一眼：`ros2 topic echo --once /body_drive/neck_joint_state` 与 `/body_drive/neck_joint_command` 的 `joints[].name`。官方说 Rockchip 部署流程支持 A3U，所以多半沿用 A3 的名字，但**没有证据**。如果不一致，需要在 C++（`NeckStateSub` 的名字表和指令名）里加别名。
3. **A3 Ultra 的算力平台**：官方 README 说 A3U 可以用 Rockchip RK3588S 或 Drive Thor。公开仓库只提供 `x86_64` 与 `rockchip` 的部署包构建（`build_a3_deploy_pkg.sh --arch`）；**Thor 运行时和教程没有公开**（README 的 TODO）。需要先确认你的 A3 Ultra 用的是哪个平台。
4. 仿真模型是简化的（串联踝/腰、刚性脚，无被动脚、无并联闭环），真实脚底柔顺与并联机构的差异、IMU 坐标/符号约定、`/body_drive/*` 在 A3 Ultra 上的其它话题细节都没有真机对照。
5. 手指不驱动（`hand_bridge_enabled: false`），灵巧手相关的话题没有验证。
6. 官方策略对动作有明确的适用范围（README "Current limitations"）：不支持跪/躺/起身等额外接触、不支持快跑；手腕跟踪较差；035 只有 `g1` 与 `a3_fast` 编码器，没有 SMPL / 遥操编码器，所以 Pico 遥操依赖我们新写的实时重定向（见 12 节）。

### 14.6 使用方法（不训练，直接用下载的权重）

```bash
cd "/home/user/YZY/HumanWBM/Demo/RL controller/GR00T-WholeBodyControl"      # 路径含空格时建议用无空格软链接
python download_from_hf.py --robot a3_ultra                  # = 035 的 ONNX + RKNN（--component pt 另取 PyTorch 权重）
gear_sonic_deploy/scripts/setup_a3_sim_env.sh                # 一次性：仿真用的 ROS2 Python 环境
python gear_sonic/scripts/run_sim_loop.py --robot a3_ultra   # 终端 1：A3 Ultra 自己的 MuJoCo 模型
gear_sonic_deploy/deploy.sh --robot a3_ultra sim             # 终端 2：部署程序按键 S → M → R（动作回放）或 T → 3（遥操）
# Pico：终端 2 改为 deploy.sh --robot a3_ultra --input-type zmq_manager sim，终端 3 为 pico_manager_thread_server.py --robot a3_ultra --manager
```

### 14.7 本节新增 / 修改的文件

`gear_sonic/utils/teleop/a3/retarget.py`、`gear_sonic/scripts/a3_teleop_retarget_node.py`（`track_head`）；`gear_sonic/utils/mujoco_sim/a3_sim.py`（无力矩限制的关节不裁剪；`JOINT_ARMATURE_IF_MISSING`；`A3_SIM_LOG_CSV` 状态日志；详细状态打印含位置与头角）、`wbc_configs/a3_035.yaml`（合约转子惯量表）；`gear_sonic_deploy/scripts/deploy_a3.sh`（`a3_ultra` 提示文字）；`tests/test_a3_teleop_retarget.py`、`tests/test_a3_ultra_contract.py`（共 +2 个测试）；`README.md`（A3 一节）、`docs/notes/mujoco_pico_sim_teleop.md`（权重与头部说明）。回归：`tests` + `gear_sonic/tests` 共 203 通过、1 跳过；`gmr` 环境重定向/参考测试 15 通过。

## 15. `--encoder a3-fast|smpl`：A3 Ultra 遥操的两条编码器路径怎么选（2026-10-14，用户要求）

> 用户要求：根据 11.13 节，针对要部署的 A3 Ultra，“A3-fast encoder（Litchi 实现并移植到 GR00T-WholeBodyControl）vs SMPL encoder（我们自己实现的）”，希望这两个在 MuJoCo 和真机都可以准确使用，补充命令行参数 `--encoder smpl/a3-fast` 来选择；把代码改动逻辑记到工作日志里，完成后由用户自己做 MuJoCo + Pico 仿真验证。

### 15.1 结论（先看这张表）

| `--encoder` | 策略权重 | 操作端链路 | MuJoCo | 真机 | 现状 |
|---|---|---|---|---|---|
| `a3-fast`（默认） | 下载的 035 `a3_fast` ONNX（低延迟，≈220 ms） | Pico SMPL → GMR 重定向（A3 关节参考）→ ZMQ `a3_ref`:5560 → `a3_ta_bridge` → ROS2 `/ta/whole_body_command` → 部署程序 | **已端到端验证**（15.5，Pico 用模拟器） | 未验证（15.7） | **完整可用** |
| `g1` | 下载的 035 `g1` ONNX（关节空间编码器，≈940 ms） | 同上 | 链路同上（本节没有重跑；第 9 节有闭环数据） | 未验证 | 可用，更慢，只作对照 |
| `smpl` | **不存在**：035 只有 `g1` 与 `a3_fast` 两个编码器，HF / GitHub 上没有任何 A3 / A3 Ultra 的 SMPL-encoder 权重；本机 `a3_ultra` 训练只做过冒烟，没有可用模型 | Pico SMPL 流 + A3 手腕角 → ZMQ `pose`:5561 → 部署程序的 `smpl_zmq` 源 → SMPL-encoder ONNX（`obs_dict[1,1770] → action[1,29]`） | 数据通路、源锁定、陈旧/回退安全逻辑已验证，用的是**站立桩**（15.4-⑥），所以机器人**站着不跟随操作者** | 未验证 | **接口就绪，等你训练并导出 SMPL 模型**；没有模型时 `--encoder smpl` 会明确报错退出 |

所以：**现在真能“准确跟随人”的只有 `a3-fast`**。`smpl` 这一条我把“模型一到就能用”的接口、校验和安全逻辑都做好并在 MuJoCo 里用桩验证了通路，但不能谎称它已经可以准确遥操——没有权重就没有准确度可言。

### 15.2 勘误：11.13 的对比表里，SMPL 编码器的“帧率/时延”写错了

11.13 表里写的 “SMPL：skip 5（G1 遥操配置），延迟约 940 ms” **不对**，那是 `g1` 关节空间编码器的数字。证据：

- 训练配置：`sonic_a3.yaml:218-221`、`sonic_a3_ultra.yaml:110-113`、`sonic_release.yaml`、`sonic_v1_1.yaml` 等都是 `num_future_frames: 10`、`dt_future_ref_frames: 0.1`（`g1` 编码器，1 s 窗口）与 **`smpl_num_future_frames: 10`、`smpl_dt_future_ref_frames: 0.02`（SMPL 编码器，0.2 s 窗口）**；`sonic_a3_035.yaml:38-39` 的 `a3_fast_dt_future_ref_frames: 0.02`。
- 部署：`A3ZmqSmplSource::BuildTokenizerSlice` 取 `kSmplFutureFrames=10` 个**相邻**帧（每个 20 ms 控制 tick 推进 1 帧），不是隔 5 帧取。
- 因此 SMPL 与 a3-fast 的窗口间隔相同（0.02 s），区别在**输入信号**（人体笛卡尔 vs 机器人关节）而不是帧率；940 ms 只属于 `g1` 编码器。11.13 表里 a3-fast 的 tokenizer 也漏了 29 个关节速度（实际 290 位置 + 290 速度 + 60 朝向 = 640）。两处已在 11.13 原表中就地修正。
- 同一证据还说明：以后训练 A3 Ultra 的 SMPL 编码器时，**`sonic_a3_ultra.yaml` 里现有的 `smpl` 编码器间隔（0.02 s）、tokenizer 观测顺序（`smpl_joints_multi_future_local_nonflat` → `smpl_root_ori_b_multi_future` → `joint_pos_multi_future_wrist_for_smpl` = 720 + 60 + 60 = 840）已经与部署源一致**，不需要为此改部署。

### 15.3 怎么用

**MuJoCo（三个终端，和 G1 的流程一一对应；路径含空格时用无空格软链接，下同）**

```bash
# 终端 1：A3 Ultra 自己的 MuJoCo 模型（窗口里按 9 把机器人从弹性绳放到地面）
python gear_sonic/scripts/run_sim_loop.py --robot a3_ultra

# 终端 2：C++ 部署 + 操作端链路。--encoder 二选一（默认 a3-fast）
cd gear_sonic_deploy
./deploy.sh --robot a3_ultra --input-type zmq_manager sim --encoder a3-fast
./deploy.sh --robot a3_ultra --input-type zmq_manager sim --encoder smpl --smpl-model <model_step_N_smpl.onnx>
./deploy.sh --robot a3_ultra --input-type zmq_manager sim --encoder smpl --smpl-model stub   # 只测数据通路，机器人站着不动

# 终端 3：Pico 数据流（头显连接、标定同 G1；smpl 编码器要 10 帧窗口，所以加 --num_frames_to_send 10，默认 5）
python gear_sonic/scripts/pico_manager_thread_server.py --robot a3_ultra --manager --num_frames_to_send 10
```

部署终端里按键 `S`（PD 站立，弹性绳还吊着时按）→ 放到地面 → `M` → `T`。`--encoder` 决定的源在启动时就选好并**锁定**：按 `1/2/3` 切到别的源会打印 `source g1 rejected: locked to a3_fast by --teleop-source (restart deploy.sh with the other --encoder to change)`，不会真的切换（避免遥操中途被误触切到一个没有数据的源）。没有头显时用 Pico 模拟器代替终端 3：`PYTHONPATH=. python gear_sonic/scripts/pico_pose_zmq_replay.py --clip synthetic:wave --stand-seconds 6 --loop`。

**真机（两台机器：操作端 PC 连头显 + 做重定向，机器人上只跑部署）**

```bash
# 操作端 PC（连 Pico 头显的那台）
python gear_sonic/scripts/pico_manager_thread_server.py --robot a3_ultra --manager --num_frames_to_send 10
gear_sonic_deploy/scripts/a3_operator_chain.sh --encoder a3-fast        # 或 --encoder smpl
# a3-fast：a3_ta_bridge 经 ROS2 把 /ta/whole_body_command 发给机器人，所以这台 PC 要装好部署包与 ROS2 运行库，
#          并使用与机器人相同的 ROS_DOMAIN_ID / 网络（没有系统 ROS2 时设 A3_ROS_PREFIX 指向带 ROS2 库的前缀，默认 conda 环境 a3_deploy_verify）；
#          smpl：只需要 GMR 环境和网络，SMPL 流走 ZMQ :5561

# 机器人上（Rockchip / x86_64 部署包）
./deploy.sh --robot a3_ultra --input-type zmq_manager --no-chain real --encoder a3-fast
./deploy.sh --robot a3_ultra --input-type zmq_manager --no-chain --zmq-host <操作端 PC 的 IP> real --encoder smpl --smpl-model /path/model_step_N_smpl.onnx
```

`--no-chain` 表示不在本机起操作端链路（机器人上通常没有 GMR / mujoco 环境）；`smpl` 时 `--zmq-host` 是 SMPL 流的来源（生成的运行时配置里 `smpl_zmq.host`）。其余真机注意事项（龙门架、`S` 后再 `M`、头部话题名、Thor / Rockchip）沿用 14.5，**首次上机必须读**。

### 15.4 代码改动逻辑

整体思路：**把“选哪个编码器”拆成四层，每层只做一件事，且都在启动时校验**，选错或缺东西时立刻报错退出，而不是等到按 `T` 之后才发现没有数据。

① **C++ 部署程序 `a3_deploy_onnx_ref`（`src/a3_deploy/main.cpp`）：`--teleop-source g1|smpl|a3_fast`**（也可以写配置 `teleop.default_source`，命令行优先）。在加载动作库之前预选并校验：值非法 → 退出码 64；`smpl` 但没有 SMPL 策略或 `smpl_zmq` 没启用、`a3_fast` 但没有 a3_fast 策略 → 退出码 65 并说明原因；通过则打印 `✓ TELEOP encoder source preselected and locked: <源>`。按键 `1/2/3` 的处理多了一个“已锁定”分支（不改变原有的可用性检查）。二进制没给这个参数时行为与以前完全一致（按键自由切换）；但 `deploy_a3.sh` 总是会传它——没写 `--encoder` 就是 `a3-fast`，所以经 `deploy.sh` 启动时源总是已锁定（要换源就改 `--encoder` 重启；以前文档里 “T → 3” 的流程仍然成立，因为 `3` 与锁定的源相同）。另加两处日志：SMPL 流第一次真正驱动策略时打印 `[teleop] smpl encoder running: SMPL stream -> <模型名>`。

② **运行时配置生成（`scripts/a3_make_runtime_cfg.py` + `run_a3.sh` 模板里的 `A3_RUNTIME_CFG`）**：`--encoder smpl` 需要往运行时配置里写 `onnx.smpl_model_path`（`.rknn` 写 `smpl_rknn_model_path`）和 `smpl_zmq`（`enabled/host/port 5561/topic`），而 C++ 的 `--runtime-cfg` 只认**第一次出现**的那个，所以不能在后面再追加一个。做法：脚本从打包好的 `a3_runtime_config.yaml` 生成 `dist/a3_deploy_<arch>/config/a3_runtime_config.generated.yaml`（与原配置同目录，相对路径 `models/...` 照常解析；注释不会保留，文件头有说明），再用环境变量 `A3_RUNTIME_CFG` 让 `run_a3.sh` 使用它。`a3-fast` / `g1` 不需要改配置，只加 `--teleop-source`。

③ **操作端链路 `scripts/a3_operator_chain.sh`（新）**：一个脚本、两种模式，`deploy.sh` 在仿真/单机时会自动在部署程序旁边起它，真机时由你在操作端 PC 上跑（`--no-chain`）。
  - `a3-fast` / `g1`：`a3_teleop_retarget_node.py --mode a3-ref`（Pico SMPL → GMR → A3 关节参考，`a3_ref`:5560）+ `a3_ta_bridge`（→ ROS2 `/ta/whole_body_command`）。
  - `smpl`：`a3_teleop_retarget_node.py --mode smpl-pose`：**不做重定向**，只把 Pico 的 `pose` 流原样转发到 :5561，并补上 A3 的手腕角。原因：官方 G1 发送端（`pico_manager_thread_server.py` 的 `PoseStreamer`）往 `joint_pos` 里填的手腕角用的是 G1 的关节轴公式，A3 的腕关节轴不同；SMPL 编码器的第三段输入（`joint_pos_multi_future_wrist_for_smpl`）要的是 A3 自己的腕角。`A3SmplPoseAugmenter`（`gear_sonic/utils/teleop/a3/retarget.py`）用与 A3 训练参考同一套 GMR 逆解（同一族 IK）逐帧算出 29 关节，把 IsaacLab 顺序 23..28 号槽位（6 个腕关节）填进 `joint_pos`，每个 `frame_index` 只解一次（滑动窗口按帧号缓存，窗口首帧会先 5 次迭代稳定再正式求解）。`smpl_joints`、`body_quat_w`、`frame_index` 原样透传。

④ **启动器 `scripts/deploy_a3.sh`：`--encoder a3-fast|smpl|g1`、`--smpl-model PATH|stub`（或环境变量 `A3_SMPL_MODEL`）、`--no-chain`**。流程：规范化编码器名（`a3_fast`/`a3fast` 也认）→ `smpl` 时解析模型（缺失 → 退出码 66 并打印如何训练 / 导出；`stub` → 自动生成桩）→ 用 `a3_smpl_onnx_tool.py check` 校验接口（不符 → 退出码 65）→ 生成运行时配置 → `--teleop-source` 传给部署程序 → 需要时起操作端链路 → 起 `run_a3.sh`。

⑤ **`scripts/a3_smpl_onnx_tool.py`（新）**：`check` 校验导出的 SMPL 模型接口——输入恰好是 `obs_dict` `[1,1770]`、输出恰好是 `action` `[1,29]`、float32（1770 = 840 SMPL tokenizer + 930 本体感知历史）；`stub` 生成站立桩。

⑥ **站立桩（plumbing 测试用的“SMPL 策略”）**：最初想用全零动作的桩，但零动作 + 策略增益（Kp 偏低）本身站不住，离线 PD 实验约 4 s 就倒，在遥操里一接上就会倒，既吓人又会让你误以为 SMPL 通路坏了。现在的桩是 `obs_dict[1,1770] → Slice 取后 930 维本体感知 → 与常量“默认站姿 tokenizer”（与 C++ `BuildDefaultStandTokenizerSlice` 同一份，由 `reference_buffer.py` 里的 Python 端口生成并有单测钉住）拼成 1570 维 → 真正的 a3_fast 计算图 → action`。即 SMPL tokenizer 被忽略，机器人像部署自带的“无数据站立回退”一样站着；SMPL 数据通路（Pico → ZMQ → 窗口 → tokenizer 切片 → ONNX → PD）、源锁定、陈旧保持 / 重连都是真的在跑。桩的输出与 `a3_fast(站姿 tokenizer | 本体感知)` 逐位相同（5 组随机观测，最大差 0）。**它不跟踪操作者，也不能用来评价 SMPL 路径的准确度。**

⑦ **顺带修的四个问题（都是这次测试里暴露的）**：
  - **包构建会丢掉 `a3_ta_bridge`**：`build_a3_deploy_pkg.sh` 开头 `rm -rf dist/a3_deploy_<arch>`，而桥原来是手动编出来放进去的，重建一次包就没了（`a3-fast` 链路在 `deploy.sh` 里就起不来）。现在脚本会编译 `a3_ta_bridge`（目标存在时），缺失时校验阶段给警告。
  - **SMPL 源的日志刷屏**：`StreamedMotionMerger`（G1 共享头文件）每个数据块打 3 行日志（≈150 行/s），把终端里的 `[mode]`/`[teleop]` 提示冲没了。`DEBUG_LOGGING` 由编译期常量改成运行期开关，**默认仍为开（G1 与上游一致，行为不变）**，A3 的 SMPL 源在 `Start()` 里按 `smpl_zmq.verbose`（默认 false）设置。同样的 SMPL 流，一次 85 s 运行的部署日志 17102 行，改后一次 70 s 运行 614 行。
  - **`-ffast-math` 把 A3 部署里所有 `std::isfinite` 守卫都编没了**：顶层 `CMakeLists.txt`（上游 NVIDIA 原样）用 `-O3 -ffast-math`，其中 `-ffinite-math-only` 会把 `isfinite(x)` 折叠成 `true`。A3 部署里的 NaN / Inf 防护（`a3_ref` 帧解码、遥操参考缓冲、SMPL 源的四元数、策略输出检查、`main.cpp` 的 8 处）因此全部失效；单测 `A3TaBridgeCodec.RejectsMalformedMessages` 在重建前就是红的（含 NaN 的帧被接受），此前的 C++ 单测运行里它和另外几个环境相关的失败一起出现，没有被追查。现在对 `src/a3_deploy/*.cpp` 单独加 `-fno-finite-math-only`（其余 fast-math 优化保留，G1 源码仍与上游编译方式一致），该单测转绿。**这是一条真机安全相关的修复：操作端链路任何一帧带 NaN（逆解发散、数据损坏）以前都会一路进到策略输入。**
  - **Pico 退出 / 重进 POSE 模式后，两条链路都会“卡死”**（流中断测试发现）：官方发送端 `PoseStreamer.on_mode_exit()` 里 `step = 0`，操作者每次离开再进入 POSE 模式（A+X），`frame_index` 都从 0 重新开始。`a3-ref` 节点原来按 `frame_index <= last_index` 丢弃旧帧，于是整个新会话的帧都被丢掉，直到计数器超过上一次的最大值（上一段 37 s 就会冻结 37 s，手臂停在最后一个姿态）；`smpl-pose` 的 `A3SmplPoseAugmenter` 对“不在缓存里的旧帧”返回最近的缓存解，即把**上一段的手腕角**当成新会话的输入。现在两处用**与 G1 接收端相同的规则**（`StreamedMotionMerger::CalculateSlidingWindow` 的 catch-up：起点不前进 / 终点不前进 / 窗口间丢帧，逐条移植为 `pose_window_catch_up`，见 16.2；第一版我写的是“最新索引比已见最大值小”，与 G1 不一致，已按用户要求改掉）开始新会话；`a3-ref` 的逐消息逻辑抽成可测试的 `A3RefStream`。G1 的部署程序本来就靠 `StreamedMotionMerger` 处理这种重启，A3 的 `smpl_zmq` 源复用它，所以 C++ 侧不用改。测试覆盖规则、窗口去重 / 丢帧、重启后继续出帧、短会话重启（索引完全相同）、增强器不再返回旧手腕角，并确认这些测试在旧逻辑下会失败（旧逻辑：重启后 0 帧输出；右肩 roll 槽位与上一段完全相同）。

⑧ **`gear_sonic/scripts/run_a3_teleop_sim.py`（纯 Python 的 PT 闭环，不经过 C++ 部署）**：新增同名的 `--encoder a3-fast|g1|smpl`；`a3-fast`/`g1` 等价于原来的 `--encoder-mode`，`smpl` 直接报错并指向 `deploy.sh --encoder smpl`（这个脚本只加载 035 的 PT 与关节空间编码器）。`pico_manager_thread_server.py` 不需要 `--encoder`：它只负责发 SMPL `pose` 流，两条路径共用。

### 15.5 验证（本机，MuJoCo，无头显）

| 项 | 结果 |
|---|---|
| `deploy.sh --robot a3_ultra --input-type zmq_manager sim --encoder a3-fast`（Pico 模拟器 `synthetic:wave`，S→M→T，85 s） | 链路完整起来：重定向节点平均 2.1 ms/帧（p95 2.3–2.4 ms，最大 2.9 ms），`a3_ta_bridge` 转发 2200+ 帧、拒收 0、帧龄约 2.1–2.9 ms；部署端 `first whole_body_command received`、`initial segment_yaw_offset_deg=89.56`；右臂随挥手动作（肩 roll 到 −1.49 rad、肘到 1.61 rad，约 0.25 Hz 往复）；骨盆 1.073–1.075 m，倾角稳态 2.2°、过渡期最大 3.6°，没有摔倒；`1`/`2` 被拒绝并提示已锁定，`3`（与锁定一致）被接受 |
| `--encoder smpl --smpl-model stub`（同上，T 后 60+ s） | 桩生成并通过接口校验；`✓ SMPL ZMQ source enabled: localhost:5561`、`A3 monolithic smpl policy runtime loaded … input_dim=1770`、`TELEOP encoder source preselected and locked: smpl`；`T` 后 `[teleop] smpl initial yaw_offset_deg=89.5` 与 `smpl encoder running`；机器人站着：骨盆 1.073 m、倾角 ≤1.7°（T 后 10 s 起）；`1`/`3` 被拒绝并提示已锁定；日志 614 行（修日志刷屏前 17102 行） |
| 真实走路数据（`sample_data/smpl_filtered/walk_forward_amateur_001__A001.pkl` 循环回放，`a3-fast`，95 s，经 C++ 部署 + A3 Ultra MuJoCo） | 机器人跟着走起来（骨盆水平位移到 ±6 m 量级，片段循环衔接处会走回头路），**全程没有摔倒**：骨盆最低 1.008 m、倾角最大 11.3°；这是模拟器数据，不是真人 Pico |
| 两机拓扑（`--no-chain`） | 操作端链路单独在干净环境（只有 `ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1`）里启动，部署用 `deploy.sh … --no-chain`：`a3-fast` 桥自己找到 ROS2 库（`a3_operator_chain.sh` 新增 `A3_ROS_PREFIX` 回退）、部署收到 `first whole_body_command`；`smpl` 桩 `smpl_zmq.host` 取 `--zmq-host`，`SMPL ZMQ source enabled`、`smpl encoder running`，骨盆 1.072–1.075 m、倾角 ≤1.6° |
| 流中断 / 重连（Pico 模拟器停 14 s 再重启，`frame_index` 回到 0；修复后） | `a3-fast`：`[teleop] stale 263 ms: holding last whole_body_command` → `[a3-retarget] new Pico session (restart: window [0..9] after [1855..1864]) -- same catch-up rule as G1's receiver` → `[teleop] reconnected: segment_yaw_offset_deg=89.1`，中断期间手臂保持最后姿态、站稳（倾角 ≤2.2°），重连后跟随新会话（模拟器先 6 s 站立段，再挥手；肩 roll 约 6 s 后重新回到 −1.48…−0.46）；`smpl` 桩：`smpl stale 250 ms: holding last command` → `smpl-pose: new Pico session (G1 catch-up rule …)` → `smpl reconnected` → `smpl encoder running`，站稳（倾角 ≤3.3°）。**修复前**：重启后 `a3-ref` 节点再也没发过一帧，手臂永远停在 −0.66 rad |
| 失败路径 | `--encoder smpl` 不给模型 → 退出码 66 + 训练/导出指引；给 a3_fast 的导出当 SMPL 模型 → 退出码 65 + “obs_dict shape must be [1, 1770] … got [1, 1570]”；`--encoder foo` → 2；二进制 `--teleop-source smpl`（没有 SMPL 策略）→ 65，`banana` → 64 |
| 单测 | 新增 `tests/test_a3_encoder_selection.py`（16 个：配置生成各编码器的键、`.rknn` 键、无模型报错、生成文件回读；桩接口、桩 = `a3_fast(站姿 tokenizer｜本体感知)` 的数值比对（含 SMPL tokenizer 不影响输出、本体感知影响输出）、错误动作维度被识别；脚本语法、`--encoder` 校验、帮助文本）；`tests/test_a3_teleop_retarget.py` 新增 7 个（`A3SmplPoseAugmenter`：腕槽位、每帧只解一次、线格式往返；frame_index 重启：规则、窗口去重 / 丢帧、重启后继续出帧、不返回旧手腕角）；全量 `tests` + `gear_sonic/tests`（`isaaclab` 环境）：**219 通过、1 跳过**（跳过的是需要 GMR 的重定向测试模块）；`gmr` 环境重定向 / 参考测试 **22 通过**（13 + 9，含重启回归 4 个）；C++ `run_tests`（在仓库根目录运行）228 个：218 通过、9 跳过、1 失败（`FK.TestFKAndGlobalVelocities`，需要 G1 的参考数据 `reference/bones_072925_test/` 与 `g1/g1_29dof.xml`，本机没有，与本次无关；不在仓库根目录运行时 `A3AimrtBackend.Init*` 也会因相对路径失败） |

**没有做到 / 没有验证的**：
- 上表的两条端到端运行最初用的是 `-fno-finite-math-only` 重编**之前**的二进制；重编（含该修复）之后，`--encoder a3-fast` 与 `--encoder smpl --smpl-model stub` 又各完整跑了一遍（70 s，`S → M → T`，按 `1/2/3`），结果相同：`a3-fast` 桥转发 3400 帧、拒收 0，右臂肩 roll −1.49…−0.27 rad、肘 1.25…1.61 rad，骨盆 1.071–1.075 m，倾角稳态 2.2°（过渡期最大 3.6°）；`smpl` 桩骨盆 1.073–1.075 m、倾角 ≤1.6°，日志 614 行，`1/3` 被拒绝。两次运行末尾骨盆倾角突然变大（22.9° / 65.6°）是部署程序退出、指令停止之后的倒地，和 14.3 里记的一样，不是遥操期间。
- **`--encoder g1` 与 `--robot a3 --encoder a3-fast` 已在你的会话结束后补跑通过**（含 `-fno-finite-math-only` 的二进制）：`g1` 桥转发 3400 帧 0 拒收，右肩 roll −1.48…−0.46，骨盆 1.073–1.075 m、倾角稳态 2.4°，`2` 被拒绝并提示锁定到 g1；普通 A3（T2D5 模型）`a3-fast` 桥转发 3400 帧 0 拒收，骨盆 1.067–1.071 m、倾角 1.4°→3.7°（60 s 内缓慢上升，T2D5 简化模型的特点，没有摔倒），`1` 被拒绝。四条配置（a3_ultra 的 a3-fast / smpl 桩 / g1，a3 的 a3-fast）至此都在最终二进制上跑过。
- **与你的会话撞车的经过**（已处理）：第一次回归（23:16 左右）发现你正在本机运行 G1 的 Pico 会话（`deploy.sh --robot g1 --input-type zmq_manager real` + `pico_manager_thread_server.py`，占着 5556 端口），我立刻停掉了自己的进程，那一轮结果作废（模拟器因端口被占没起来，重定向节点还连到了你的真 Pico 流上）。第二次（23:26 起）在你的会话结束后重跑，跑到 `--encoder g1` 时你又启动了 G1 的 MuJoCo 会话（`run_sim_loop.py --robot g1` 约 23:29:02、`deploy.sh --robot g1 … lo` 约 23:29:25），我在约 23:29:40 发现并用 PID 杀掉了自己的全部进程。**重叠与影响**：我的 `-j20` 编译（23:15–23:16）、两轮无头仿真各约 80 s，以及第二次回归最后约 40 s 里我的 Pico 模拟器（A3 的合成 `wave` SMPL 流）在 5556 端口发布——你的 G1 部署程序在那段时间启动，**可能连上过它**（模拟器不发 `command` 启动消息，G1 不应因此起步）。我没有碰过你的进程、你的 5556/5557 发布端或网卡；我的仿真走 ROS2 域 87 + localhost-only，与你的 DDS 无关。之后的补跑（23:59 起）先确认了机器上没有你的会话；我的测试脚本现在开跑前会检查 `deploy.sh --robot g1` / `pico_manager_thread_server` / `g1_deploy_onnx_ref` 与 5556 端口，发现占用就不跑。
- **真实头显**的数据没有测（只有同格式的模拟器）；**真机**没有测。
- `smpl` 的跟踪准确度无从谈起（没有模型）。

### 15.6 以后拿到 / 训练出 SMPL-encoder 模型时怎么接上（以及“我们自己实现的 SMPL 编码器”现在到哪一步）

**现状核对**：SMPL 编码器的**代码**是有的（`sonic_a3.yaml`（035 与现在的 `sonic_a3_ultra.yaml` 都继承它）与 `sonic_a3_ultra_t3d0.yaml` 都定义了 `smpl` 编码器：输入 `smpl_joints_multi_future_local_nonflat → smpl_root_ori_b_multi_future → joint_pos_multi_future_wrist_for_smpl`，10 帧 × 0.02 s，840 维，与部署源一致），但**没有训练过**：`sonic_a3.yaml` 里 `encoder_sample_probs: {g1: 1.0, teleop: 1.0, smpl: 0.0}`、`smpl_motion_file: dummy`（Litchi 自己就没训它）；本机的 A3 Ultra 动作库（`data/motion_lib_a3ultra/robot_filtered_v15_clean`）里 `smpl_joints` 是占位零；G1 流程用的 `data/bones_seed_smpl` 也不在本机。要得到可用的 SMPL 模型，缺的是**数据 + 训练**，不是部署接口：

1. **数据**：SMPL 编码器要成对的“人体 SMPL 运动 ↔ 机器人参考”。需要取 Bones-SEED 的 SMPL 版本，按 clip 与已重定向的 A3 片段对齐，写进动作库（`smpl_joints` / `smpl_pose` 等）；未做。
2. **训练合约**：已按 A3 的 29 关节合约落地（第 16 节）——`--robot a3_ultra` 现在就训练这套合约（A3 URDF / Kp/Kd / 奖励 / 域随机，数据是转换成 `a3_29` 的 A3 Ultra 动作库）。SMPL 路径只差 1（SMPL 数据）与把 `encoder_sample_probs.smpl` 调成 >0、`active_encoders` 加上 `smpl`（必要时从 035 PT 继续训练并给 `smpl` 编码器随机初始化——需要部分加载，未验证）。导出的 ONNX 天然是 `[1,1770] → [1,29]`。（Ultra 自己的 63 DOF 合约在 `sonic_a3_ultra_t3d0.yaml`，导出是 `obs_dict[1, 840 + 10×(3+63+63+31+3)]` / `action[1,31]`，部署加载不了，所以不用它。）
3. 导出：`python gear_sonic/eval_agent_trl.py +checkpoint=<model.pt> +export_onnx_only=true` → `model_step_<N>_smpl.onnx`（单体，含编码器与解码器）。
4. 校验：`python gear_sonic_deploy/scripts/a3_smpl_onnx_tool.py check <onnx>`（需要装了 `onnx` 的 Python，如 `isaaclab` 环境）。`deploy.sh --encoder smpl` 会自动做；接口不符（比如上面 31 动作的情况）会以退出码 65 明确报错。
5. 运行：`deploy.sh --robot a3_ultra --input-type zmq_manager sim --encoder smpl --smpl-model <onnx>`；Pico 用 `--num_frames_to_send 10`。**先在 MuJoCo 里和 a3-fast 比较同一段动作的跟踪效果**，再考虑真机；Rockchip 要先转 RKNN（`convert_a3_onnx_to_rknn.py`）并传 `.rknn`。
6. 手腕角：Pico 流里的手腕角由 `A3SmplPoseAugmenter` 给出（GMR 逆解），应与训练时 `joint_pos_multi_future_wrist_for_smpl` 的取值来源（动作库里 A3 的腕关节角，同样来自 GMR 重定向）一致；如果训练用的动作库不是这套重定向，要同步调整。

### 15.7 仍未验证 / 上真机前必须确认

1. 真机、真实头显（15.5）。`a3_operator_chain.sh --encoder a3-fast` 在操作端 PC 上要能与机器人处于同一 ROS2 域并通过 AimRT 的 ROS2 后端发布 `/ta/whole_body_command`；真机走 `iceoryx` 还是 `ros2` 传输取决于你的站点（`A3_TRANSPORT`），这条订阅是否在你的真机配置里启用没有核对。
2. 14.5 的所有项（头部话题关节名 `head_pitch_joint` vs `ultra_head_pitch_joint`、Rockchip / Thor、IMU 约定、简化的仿真模型）依旧有效。
3. 重定向质量：`a3-fast` 的人→A3 映射靠 GMR 逆解（骨盆朝向权重 50、`human_height` 2.18，见 9.4 与第 12 节）；模拟器里用的是合成动作（直臂站姿 + 右臂挥手），真人数据下的准确度只能由你的 Pico 测试判断。
4. **流中断时的行为是“保持最后一条指令”**（部署程序的既有设计，上面的中断测试确认）：操作者摘下头显 / Pico 服务停发，机器人手臂会一直停在最后的姿态并保持站立，直到数据恢复或你按 `P`；它不会自己回到站姿。真机上遇到“人走开了”要按 `P`（被动，kp/kd/前馈置 0）或用物理急停，不要指望自动回位。
5. `smpl` 的 `smpl_zmq.conflate: true`（只保留最新消息）与 `--num_frames_to_send 10`（每条消息带 10 帧窗口）配合；用默认 5 帧时窗口只有历史 5 + 新 5 = 10 帧，没有余量，抖动时末尾几帧会重复最后一帧（部署源对越界索引取最后一帧）。（按 `StreamedMotionMerger` 与 `BuildTokenizerSlice` 的代码推断，没有实测。）所以文档里一律要求 10。

### 15.8 本节新增 / 修改的文件

- C++：`src/a3_deploy/main.cpp`（`--teleop-source`、`teleop.default_source`、锁定、运行日志）、`src/a3_deploy/a3_zmq_smpl_source.cpp`（日志开关）、`include/input_interface/streamed_motion_merger.hpp`（`DebugLogging()`，默认开）、`src/g1/g1_deploy_onnx_ref/CMakeLists.txt`（A3 源 `-fno-finite-math-only`）。
- 脚本：`gear_sonic_deploy/scripts/deploy_a3.sh`（重写 `--encoder/--smpl-model/--no-chain`）、`a3_operator_chain.sh`（新）、`a3_make_runtime_cfg.py`（新）、`a3_smpl_onnx_tool.py`（新）、`build_a3_deploy_pkg.sh`（`run_a3.sh` 模板的 `A3_RUNTIME_CFG`；编译 `a3_ta_bridge`）。
- Python：`gear_sonic/utils/teleop/a3/retarget.py`（`A3SmplPoseAugmenter`、`A3RefStream`、`frame_index_restarted`）、`gear_sonic/scripts/a3_teleop_retarget_node.py`（`--mode a3-ref|smpl-pose`、`--out-port`）、`gear_sonic/scripts/run_a3_teleop_sim.py`（`--encoder`）。
- 测试：`tests/test_a3_encoder_selection.py`（新，16 个）、`tests/test_a3_teleop_retarget.py`（+7）。
- 文档：本节、11.13 的两处勘误、`docs/notes/g1/mujoco_pico_sim_teleop.md` 第 8 节与 `docs/notes/g1/pico_real_teleop.md` 第 6 节（`docs/notes/` 下的 G1 文档已被移到 `docs/notes/g1/`，前面各节里写的旧路径 `docs/notes/mujoco_pico_sim_teleop.md` 指的是同一份文件）、`README.md` 的 A3 一节、worklog #46。
- 回归都已补跑完（见 15.5）。部署程序要真 TTY，我用一个 `pty` 小驱动按键；你自己跑直接在终端按 `S → M → T → 1/2/3`。

## 16. 与 G1 的 POSE 重进逻辑对齐；按 Litchi 的 a3 参数批量转换、训练，域随机按 A3 与 A3 Ultra 一起调（2026-10-15，用户要求）

> 用户的两点：① 上一轮的重启修复（“最新 frame_index 比已见过的最大值还小就开始新会话”）和 G1 在 Pico 发送端每次退出再重进 POSE 模式时的逻辑是不是一样？不一样就保持一致。② 按 Litchi_sonic_for_a3 里的 a3 参数做批量转换：035 虽然用 A3 的 URDF 训练，域随机是按 A3 和 A3 Ultra 一起调的，所以这套模型能在 A3 Ultra 上跑；RK3588 的 RKNN 同样支持 A3 Ultra，Drive Thor 走 Thor 运行时（教程待办）；代码按这个范围接了 Ultra：策略只输出 29 个身体关节，头两路另加固定 PD，颈部订阅把 `ultra_head_pitch_joint` 当 `head_pitch_joint` 的别名，灵巧手不在这 29 路里。“也就是说根据 a3 参数进行转换训练，但是域随机按 A3 和 A3 Ultra 一起调。”

### 16.1 结论

1. **不一样，已改成一样。** G1 不是靠“最新索引变小”判断，而是接收端 `StreamedMotionMerger::CalculateSlidingWindow` 的三条 catch-up 规则（16.2）；我原来的规则漏掉了“起点相等”（短会话后重进）与“窗口间丢帧”两种情形。现在把这三条逐条移植成 `pose_window_catch_up`，`a3-ref` 节点（`A3RefStream`）与 `smpl-pose` 增强器用同一条规则，`smpl_zmq` 的 C++ 源本来就是同一份 merger 代码。
2. **a3 参数批量转换已完成并验证**（**第 17 节：用户澄清后，训练库改为在 A3 上用原来的重定向方法重新批量转换**；本节的“格式转换”保留为交叉验证）：A3 Ultra 重定向库（129,116 条）→ Litchi `a3_29` 格式，0 失败，`--check` 全量通过；`--robot a3_ultra` 现在训练“A3 合约 + 035 的域随机 + 转换后的数据”，导出直接是部署接口；Ultra 自己的 63 DOF 合约改名 `sonic_a3_ultra_t3d0`、不再是默认。训练冒烟（从零、035 PT 热启动）都通过。

### 16.2 第 1 点：G1 怎么处理“退出再重进 POSE”，A3 链路怎么对应

**G1（代码事实）**
- 发送端 `pico_manager_thread_server.py` 的 `PoseStreamer.on_mode_exit()`：`frame_buffer.clear()`、`step = 0`——每次离开 POSE 模式，再进入时 `frame_index` 从 0 重新数。
- 接收端 `StreamedMotionMerger::CalculateSlidingWindow`（`include/input_interface/streamed_motion_merger.hpp`），对每条消息的 `[start..end]`（`frame_indices[0]` 与 `[n-1]`）和已合并窗口 `[window_start..window_end]`：
  1. 还没有窗口（第一包）→ 新窗口；
  2. `start <= window_start` → **catch-up**（重启，或旧 / 重复窗口）；
  3. `end <= window_end` → **catch-up**（没有比已合并的更新的帧）；
  4. `start > window_end + frame_step`（窗口之间丢帧）或播放游标落后超过 `MAX_GAP_FRAMES + HISTORY_FRAMES = 205` 帧 → **catch-up**；
  5. 否则正常滑动合并。
  catch-up 时丢掉缓存、窗口 := 新消息；`ZMQEndpointInterface` 再把 `current_frame = 0`、`reinitialize_heading = true`（重新捕获航向）、`play = true`。

**我上一版的规则不同**：只看“最新索引 < 已见最大值”。差别是：① `start == 上一个 start` 或 `end == 上一个 end`（包括“上一段只有几帧，重进后索引完全相同”）G1 会重置、我不会；② 窗口之间丢帧 G1 会重置、我只打印 `dropped N frame(s)`。

**现在（一致）**：`gear_sonic/utils/teleop/a3/retarget.py::pose_window_catch_up(prev_window, window_indices)` 逐条对应第 1–4 条的前三类（`"restart"` = 起点不前进，`"stale"` = 终点不前进，`"gap"` = 窗口间丢帧；第一包不算 catch-up）；G1 里“播放游标落后 205 帧”依赖接收端的播放位置，Python 链路里没有对应物，不移植。

| 层 | G1 | A3 `smpl` 路径 | A3 `a3-fast` / `g1` 路径 |
|---|---|---|---|
| 发送端 | `step = 0` | 同一个发送端（官方 `pico_manager_thread_server.py`） | 同左 |
| 是否开始新会话 | C++ merger 的 catch-up 规则 | **Python 增强器**用 `pose_window_catch_up`（清缓存、IK 重新稳定）→ 转发给 **C++ `A3ZmqSmplSource`（复用同一个 merger）**；两边规则相同，同时翻转 | **Python `A3RefStream`** 用 `pose_window_catch_up`（该路径没有 merger：帧按时间戳进 `/ta/whole_body_command`） |
| 新会话后的动作 | `current_frame = 0` | `playback_frame_ = 0`（`A3ZmqSmplSource` 里 `did_catchup_reset`） | 新帧从 IK 首帧稳定开始 |
| 航向 | `reinitialize_heading = true` | 部署程序在 stale → reconnect 时重算 SMPL 航向偏置（日志 `smpl reconnected: yaw_offset_deg=…`） | 部署程序重算 `segment_yaw_offset`（日志 `reconnected: segment_yaw_offset_deg=…`） |

重进 POSE 总会先有一段 >250 ms 的无数据（人要松开再按 A+X），所以 stale → reconnect 一定先触发，航向重算不会被漏掉。

**验证**：`tests/test_a3_teleop_retarget.py` 重写为 G1 规则：规则表（正常滑动 / 相邻窗口 / 起点回退 / 起点相等 / 终点不前进 / 窗口间丢帧 / 帧间隔 2）、窗口去重与丢帧 catch-up、重启后继续出帧、**短会话后索引完全相同的重启**（“最新索引变小”规则会丢掉全部 5 帧）、增强器不返回旧手腕角（含短会话情形）。MuJoCo 流中断端到端（模拟器停 14 s 再重启，`frame_index` 回到 0）两条路径重跑通过：`a3-fast`：`stale 267 ms: holding last whole_body_command` → `[a3-retarget] new Pico session (restart: window [0..9] after [1855..1864]) -- same catch-up rule as G1's receiver` → `reconnected: segment_yaw_offset_deg=88.9`；`smpl` 桩：`smpl stale 267 ms` → `smpl-pose: new Pico session (G1 catch-up rule …)` → `smpl reconnected: yaw_offset_deg=84.9` → `smpl encoder running`。

### 16.3 第 2 点：理解、核对与转换

**我对这句话的理解**：这就是 035 本身的做法——**用 A3 的参数（A3 URDF、a3_024 的 Kp/Kd / 默认站姿 / 动作缩放、29 个策略关节）做数据转换和训练，域随机把 A3 和 A3 Ultra 一起覆盖**，所以训出来的模型能直接在 A3 Ultra 上跑（头两路固定 PD、手指不驱动）。落到我们的流程：我们的 A3 Ultra 重定向库（`data/motion_lib_a3ultra/robot_filtered_v15_clean`，为 63 DOF 的 T3D0 模型写的）要转换成 Litchi 的 `a3_29` 动作库格式，`--robot a3_ultra` 训练也走 A3 合约。（“批量转换”在本项目里一直指批量把动作数据转成机器人动作库；RKNN 部分只是 Litchi README 的原文，没有对应的转换动作。如果你指的是别的东西，请告诉我。）

**转换前先核对“两个模型是不是同一条运动链”**（`a3_t2d5.xml` vs `a3_ultra_t3d0.xml`，MuJoCo 编译后逐项比较）：

| 项 | 结果 |
|---|---|
| 29 个身体关节的名字 / 深度优先顺序 / 轴 | 完全相同（Ultra 的 `ultra_head_pitch_joint` 对应 A3 的 `head_pitch_joint`）；Ultra 库里的 31 列 `dof` 顺序 = 两个模型的 31 个驱动关节顺序，转换时还用 `pose_aa` 逐列校验 |
| 共有 31 个 body 的位置（200 组随机构型） | 最大差 1.5 mm（两个脚踝连杆，常数偏差；右肘 / 右腕连杆 0.5 mm），朝向差 ≈0 |
| 关节限位 | 只有 `hip_roll`（A3 ±92°，Ultra ±85°，Ultra 更窄）和 `knee`（A3 143°，Ultra 145°，Ultra 更宽）不同；头关节 A3 MJCF 没有范围 |
| 数据里超出 A3 限位的帧（1500 条抽样，34.2 万帧） | 只有膝，约 0.4 %（全库：左膝 92,451 帧、右膝 98,569 帧，占 0.33 % / 0.35 %），最大 145° |
| 头两列 `dof` | 全库抽样里恒为 0 |
| 足底高度（踝 → 足底最低接触点，零位） | A3 网格脚 67.5 mm，Ultra 13 个球脚 72.0 mm，差 **−4.54 mm**（A3 的脚离踝更近） |
| 质量 | 整机 57.98 vs 60.01 kg（+2.03 kg，+3.5 %）；见 16.5 |

**转换脚本** `gear_sonic/data_process/convert_a3ultra_to_a3_29d.py`（全部由两个 MJCF 推导，没有硬编码的数）：
1. 按名字把 31 列 `dof` 取成 29 个策略关节（丢掉头 2 列和手指），顺序 = `a3_t2d5.xml` 的 29 个非头关节（Litchi `A3Spec.dof_output_indices`）；
2. `dof` 夹到 A3 URDF 的限位（只会动到膝）；
3. `pose_aa` 按 `a3_t2d5.xml` 的 32 个 body 重新生成（轴 × 角度，头 body 为 0），根朝向不变；
4. `root_trans_offset` 的 z 加 −4.54 mm（脚底仍然贴地），x/y 不变；`root_rot`（xyzw）、`smpl_joints`（零占位）、帧率保持，`fps` 写成整数；
5. 保持目录结构与文件名（动作库按文件名当 key），每个文件仍是 `{名字: 动作}` 的单键字典；`--check` 模式逐文件校验（键、形状、dtype、有限、A3 限位、头 body 为 0、`pose_aa` = `dof` × 轴、帧数与源文件相同）。

**结果（本机）**：129,116 个文件、28,394,944 帧，12 个进程 18 s（7,200 文件/s），输出 `data/motion_lib_a3_29d/converted_from_a3ultra_v15_clean`（22 GB，源 33 GB；第 17 节起改名，原生 A3 重定向占用 `robot_filtered_v15_clean`），**0 失败**；`--check`：0 个问题、没有遗漏的源文件。
**物理验证**：随机 300 条、约 4.5 千帧，用 MuJoCo 把同一帧分别放进 A3 与 A3 Ultra 模型比 FK——未被夹限的帧共有 body 位置差 ≤1.5 mm；差最大的 14.6 mm 出现在左膝被夹限的帧（源数据该帧左膝 145.0°，夹到 143°：2° × 约 0.4 m 小腿 ≈ 14 mm，已核对）；脚底最低接触点 A3 减 Ultra：均值 +1.4 mm、标准差 2.1 mm、最大 21.9 mm（网格脚与球脚在脚面倾斜时的最低点不同，最大值的原因没有逐帧追查）。
**格式验证**：Litchi 自己的 `convert_a3_pkl_to_motion_lib_29d.normalize_entry` 吃我们的输出，`pose_aa / dof / root_trans_offset` 不变、`root_rot` 同一个旋转（测试里做）；官方 `train_a3_035_fromscratch.sh --preflight` 在转换后的库上通过（`preflight motion_files=129116`）。

### 16.4 `--robot a3_ultra` 现在训练什么

| | 之前 | 现在 |
|---|---|---|
| `--robot a3_ultra` | `sonic_a3_ultra`：Ultra 自己的 T3D0 63 DOF 合约（31 个动作），导出的观测 / 动作宽度部署加载不了 | `sonic_a3_ultra`：**继承 `sonic_a3_035`**——A3 机器人资产与 a3_024 参数、奖励、**035 的域随机**、`g1` + `a3_fast` 编码器；只改三个键：`motion_file`（转换后的库）、`project_name`（`TRL_A3Ultra_Track`）、`exp_var` |
| T3D0 合约 | 默认 | 改名 `sonic_a3_ultra_t3d0.yaml`，`+exp=manager/universal_token/all_modes/sonic_a3_ultra_t3d0` 显式选用（`robots/a3_ultra.py` 与 `test_a3_ultra_contract.py` 保留） |
| `--robot a3` | `sonic_a3_035`（需要自己给 `motion_file`） | 不变 |

Hydra 组合实测：`a3_ultra` 与 `a3` 的合成配置只有 `exp_var`、`manager_env.commands.motion.motion_lib_cfg.motion_file`、`project_name` 三个键不同（`tests/test_a3_ultra_training_recipe.py`）。这是一次**语义变更**（以前 `--robot a3_ultra` 是 T3D0 合约）；要回到旧行为用上面的显式 `+exp=`。

**用法**
```bash
python gear_sonic/data_process/convert_a3ultra_to_a3_29d.py            # （第 16 节的快速通道，18 s；训练默认库见第 17 节：原生 A3 重定向）
# 校验：... --check ；官方启动器预检：MOTION_FILE=$PWD/data/motion_lib_a3_29d/robot_filtered_v15_clean bash train_a3_035_fromscratch.sh --preflight
python gear_sonic/train_agent_trl.py --robot a3_ultra ...               # 路径含空格时用无空格软链接；默认读转换后的库
python gear_sonic/train_agent_trl.py --robot a3_ultra checkpoint=checkpoints/035_step200000/model_step_200000.pt +resume=false ...   # 用下载的 035 PT 热启动
```

**训练冒烟**（16 envs、2 次迭代，`headless=true`）：`--robot a3_ultra` 从零：`Loaded 129116 motion files from data/motion_lib_a3_29d/robot_filtered_v15_clean`、`Loaded 16 motions with a total length of 204 s`，`exit=0`；同样加 `checkpoint=…035…pt`：`Loaded checkpoint from step 200000`，`exit=0`。**没有做**：收敛训练，也没有做“转换数据 vs Litchi 自带 20 条数据”的 100+ 次迭代对照（这条命令我发出后被你拒绝了，没有再跑）。

### 16.5 域随机：“按 A3 和 A3 Ultra 一起调”

用的就是 035 的那一套（`sonic_a3.yaml` 的 events，035 与 `a3_ultra` 配置共用，上面的 Hydra 对比里没有差别）：躯干质量 ×[0.8, 1.3]，其它连杆质量 ×[0.9, 1.1]，手腕外侧加质量 [−0.2, +2.0] kg，质心偏移，摩擦 / 转子惯量，被动脚增益，关节软限位 0.8 等。**覆盖性不是口头断言**，`tests/test_a3_ultra_training_recipe.py` 从两个 MJCF 算出差异再对照配置里的范围：

| A3 → A3 Ultra 的差异 | 数值 | 域随机范围 | 是否覆盖 |
|---|---|---|---|
| 躯干 | 17.465 → 18.114 kg（×1.037） | ×[0.8, 1.3] | 是 |
| 手（腕外侧：手腕连杆 +0.22 kg + 手指 0.28 kg） | 每侧 ≈ +0.50 kg | 加 [−0.2, +2.0] kg | 是 |
| 头 pitch 连杆 | ×1.089 | ×[0.9, 1.1] | 是 |
| **头 yaw 连杆** | 0.565 → 0.800 kg（+0.235 kg，×1.415） | ×[0.9, 1.1] | **否**，唯一例外：头关节固定在 q = 0（Kp 40），这点质量只影响头自身的 PD 下垂；测试把它钉在 < 0.25 kg，变大会报警 |
| 其它连杆 | 完全相同 | — | 是 |
| 整机 | +2.03 kg（+3.5 %） | — | — |
| 关节限位 | 只有 `hip_roll`（Ultra 更窄）与 `knee`（Ultra 宽 2°，转换时夹到 A3 的 143°） | — | 测试钉住“Ultra 只在膝比 A3 宽” |

我没有改 035 的域随机；如果你想把头 yaw 连杆也覆盖进去，需要加一条事件（影响很小，没有做）。

### 16.6 仍然没做 / 需要你知道的

- **SMPL 编码器仍然没有数据和训练**（15.6）：转换后的库里 `smpl_joints` 仍是零占位；要训练得先取到 Bones-SEED 的 SMPL 数据并按 clip 对齐，再把 `encoder_sample_probs.smpl` 调成 >0。`commands.py` 里 `lower_joint_indices_mujoco = range(12)` 是 G1 假设（A3 的 MuJoCo 顺序是腰 + 臂 + 腿，腿在最后），只影响 `teleop` 编码器与 `cat_upper_body_poses` 增强，035 / `a3_ultra` 配方不用它们；以后启用 `teleop` 之前要先改。
- 没有跑收敛训练；没有评估 12.9 万条动作的显存 / 内存占用（冒烟只取了 16 条）。
- 转换对根高度只做了常数修正（−4.54 mm，足底几何差）；脚面倾斜时网格脚与球脚的最低点差最多 ~2 cm（只出现在极少数帧），没有逐帧重新接地。
- RKNN / Thor：本次没有动；README 写的 Thor 教程仍未公开，A3 Ultra 若是 Thor 平台，官方部署包没有，见 14.5。

### 16.7 本节新增 / 修改的文件

`gear_sonic/data_process/convert_a3ultra_to_a3_29d.py`（新）；`gear_sonic/config/exp/manager/universal_token/all_modes/sonic_a3_ultra.yaml`（重写为 A3 合约配方）、`sonic_a3_ultra_t3d0.yaml`（原 `sonic_a3_ultra.yaml` 改名并改头注释 / `project_name`）；`gear_sonic/train_agent_trl.py`（`ROBOT_EXP_MAP` 注释）；`gear_sonic/envs/manager_env/robots/a3_ultra.py`（注释里的配置名）；`gear_sonic/utils/teleop/a3/retarget.py`（`pose_window_catch_up`、`A3RefStream`、增强器的 `catch_ups`）、`gear_sonic/scripts/a3_teleop_retarget_node.py`；测试：`tests/test_a3_data_conversion.py`（新，9 个）、`tests/test_a3_ultra_training_recipe.py`（新，5 个）、`tests/test_a3_teleop_retarget.py`（重写重启部分，15 个）、`tests/test_a3_ultra_contract.py`（只改说明）；文档：本节、15.4 / 15.5 / 15.6 的相应改动、README 的 A3 一节、worklog #47。
回归：`tests` + `gear_sonic/tests`（`isaaclab`）**233 通过、1 跳过**；`gmr` 环境重定向 / 参考 / 转换测试 **33 通过**（15 + 9 + 9）。

## 17. 为 A3 的 SMPL 编码器准备数据：按 A3 参数用原来的重定向方法重新批量转换（2026-10-15，用户澄清）

> 用户澄清：`--robot a3_ultra` 训练 A3 合约没问题；“批量转换”指的是——目前为了训练我们自研的针对 A3 的 SMPL encoder，需要 **a3 重定向的数据集**，因此要基于 a3 的机器人参数，**采用之前我们做的重定向方法**（v15：BVH → GMR → 地面处理 → 动作库）重新做一遍数据转换。

### 17.1 与第 16 节的关系

第 16 节我做的是“把已有的 A3 Ultra 重定向库转成 `a3_29` 格式”（`convert_a3ultra_to_a3_29d.py`，18 s），那只是格式转换，不是你要的东西。这一节按你的意思重做：**同一个重定向流水线，目标机器人换成 A3（035 合约的 T2D5，Litchi 的 a3 参数）**，直接产出 `a3_29` 动作库。第 16 节的转换数据保留，用作交叉验证和快速通道（见 17.3）。

### 17.2 改了什么（`gear_sonic/data_process/batch_retarget_bvh_soma_a3ultra.py`，默认行为不变）

- **`--robot a3_ultra|a3`**（默认 `a3_ultra`，产出 `robot_filtered_v15` 同样的格式；`a3` 产出 Litchi `a3_29` 格式：29 列 `dof`、32 个 body 的 `pose_aa`，`fps` 为整数）。
- **GMR 里还没有 A3**：运行时把 `agibot_a3` 注册进 GMR（模型 `a3_t2d5.xml`、IK 表 `gmr_ik_configs/bvh_nokov_to_a3.json`），不改 GMR 检出目录。
- **IK 标定表沿用 A3 Ultra 的**：表里 14 个 link 在 qpos0 的姿态在两台机器上一致（`tests/test_a3_retarget_batch.py` 逐个比较，<1e-4 rad，位置差 <2.5 mm），所以 `rot_offset` 不需要重标；逐条重定向时的比例、髋 / 肩宽偏移仍然按每条动作和各自机器人模型重新算（`_configure_clip`）。表的副本放在 `gear_sonic/data_process/gmr_ik_configs/`（A3 Ultra 那份原来只在 GMR 检出目录里，未跟踪）。
- **足底高度改成精确值**：原来是 `geom_size[0]` 当球半径（Ultra 是 13 个球脚，没问题），A3 的脚是网格，`geom_size` 只是包围盒，同一公式会得到踝高 0.0941 m（真值 0.0675 m），机器人整体抬高约 26 mm。现在用 `_geom_lowest_z`（球：z−r，网格：最低顶点），Ultra 仍是 0.0720 m。
- `_lift_out_of_floor`（蹲 / 跪时把根抬出地面）本来就按碰撞几何逐帧算，A3 用它自己的碰撞网格。
- 新增 `convert_gmr_a3_to_motion_lib.py`（GMR 38 维 qpos → `a3_29`，头两列丢掉、限位夹到 A3、`wxyz`→`xyzw`）、`--only_from DIR`（只转目录里已有的那些动作名，用来让两台机器人覆盖同一批动作）、**写文件改成原子的**（先写 `.tmp` 再改名，中断后“跳过已存在”不会信任半个文件）。

### 17.3 先验证，再批量

- 同一批 BVH 在 A3 与在 A3 Ultra（再转成 `a3_29`）上的结果对比：**24 条**：全部成功，14 条有可比对象，`dof` RMSE 0.0008 rad（最大 0.0019），根高度偏差均值 +0.00 mm；**210 条**（150 条随机 + 60 条蹲 / 跪 / 蹲起 / 前后弓步 / 跳起落地 / 翻滚 / 踢等难动作）：0 失败、帧数全部相同、格式检查 0 问题，`dof` RMSE 均值 0.0020 rad、最大 0.0304 rad（`kneeling_loop`），根高度偏差均值 +0.40 mm。
  - 这说明两件事：① 同一方法在 A3 上复现了 Ultra 上的结果；② 第 16 节用常数 −4.54 mm 修正根高度是对的（原生 A3 接地给出的平均偏差为 0）。
  - 最大的差异只出现在蹲 / 跪类动作：根高度最多差 14–16 mm（A3 的碰撞网格与 Ultra 的球 / 胶囊不同，“抬出地面”的量不同），髋 yaw 等偏差 ≤0.03 rad——这是改用 A3 参数的实际效果，转换数据没有这一项。

### 17.4 全库批量（本机）

命令（后台运行，`nice -n 19`，16 个进程，机器上的其它会话不受影响）：

```bash
OMP_NUM_THREADS=1 python gear_sonic/data_process/batch_retarget_bvh_soma_a3ultra.py --robot a3 \
    --tar data/bones_seed/soma_proportional.tar.gz --output data/motion_lib_a3_29d/robot_filtered_v15_clean \
    --only_from data/motion_lib_a3ultra/robot_filtered_v15_clean --fps 30 --workers 16     # 日志：同目录 robot_filtered_v15_clean.retarget.log
```

`--only_from` 取 A3 Ultra 的 `robot_filtered_v15_clean`（用户批准过的关键词过滤，去掉坐 / 躺 / 与环境交互类动作）里的动作名，所以两台机器人覆盖**同一批 129,116 条动作**。

| 项 | 结果 |
|---|---|
| 规模 / 耗时 | 129,116 条、28,394,944 帧；109.9 min（16 进程，平均约 19.6 条/s；前约 20 分钟与我的训练冒烟抢 CPU，速度只有 13 条/s 左右）；**0 失败**（A3 Ultra 那次 v15 全量 14.2 万条里有 682 条因为源 BVH 里有 NaN 失败，这批是已经过滤掉它们的子集）；输出 22 GB；没有残留 `.tmp` |
| 格式 | `convert_a3ultra_to_a3_29d.py --check`（键、形状、dtype、有限、A3 限位、头 body 为 0、`pose_aa` = `dof` × 轴、帧数与转换版逐条相同）：**0 个问题，没有遗漏的源文件** |
| 与“转换版”（第 16 节）逐条对比（全部 129,116 条） | `dof` RMSE：均值 **0.0011 rad**、中位数 0.0007、p95 0.0015、p99 0.0094；> 0.02 rad 的 273 条（0.21 %）、> 0.05 的 36 条、> 0.1 的 24 条，最大 0.92 rad（`body_check_001__A161_M`；其余几条最差的是 `crowd_cheer_003__A100_M`、`extreme_repentance_R_001__A549`、`triumph_two_handed_001__A118` 等，原因没有逐条追查——IK 对很小的数值差异敏感，两种解都可能是合法解）。根高度：逐条平均差 −0.07 mm（中位数 0.00，p1 −2.5 mm、p99 +4.5 mm），单帧最大差 p99 12.2 mm、最大 175 mm；根 xy 最大差 57.8 mm |
| 质量剖面（相邻帧最大关节步长 @30 fps） | 两者几乎相同：中位数 0.337 rad、p99 1.009 / 1.010 rad、最大 2.412 rad；步长 > 0.5 rad 的动作：两者都有 22,399 条，只有原生有 34 条、只有转换版有 63 条——原生没有变抖 |

### 17.5 数据集与默认路径

| 目录 | 内容 | 用途 |
|---|---|---|
| `data/motion_lib_a3_29d/robot_filtered_v15_clean` | **A3 原生重定向，已转回采集朝向**（17.8，2026-10-15）：与 SMPL 数据同一世界朝向 | `--robot a3_ultra` / `sonic_a3_ultra*.yaml` / `train_a3_035_fromscratch.sh` 的默认训练库 |
| `data/motion_lib_a3_29d/robot_filtered_v15_clean_yawnorm` | 同一份重定向结果，**每条动作第 0 帧面朝 +X**（批量脚本原样输出，上面一行的“转回之前”） | 只作备份；`g1` / `a3_fast` 编码器用它与用上一行等价 |
| `data/motion_lib_a3_29d/robot_filtered_v15_clean.capture_heading.csv` | 每条动作的原始采集朝向 `_yaw_f0`（弧度 / 度，来自各自的 BVH） | 转回朝向用的表 |
| `data/motion_lib_a3_29d/converted_from_a3ultra_v15_clean` | 第 16 节的格式转换版（由 A3 Ultra 库换算）；**朝向没有转回**（面朝 +X） | 交叉验证 / 快速通道（18 s，`convert_a3ultra_to_a3_29d.py` 的默认输出） |
| `data/motion_lib_a3ultra/robot_filtered_v15_clean` | A3 Ultra（T3D0，31 列 `dof`）重定向库 | 不变，`sonic_a3_ultra_t3d0.yaml` 用 |

所有目录都在被忽略的 `data/` 下。`tests/test_a3_ultra_training_recipe.py` 钉住 `a3_ultra` 配方的默认 `motion_file` 就是第一行。

### 17.6 SMPL 编码器的训练路径：已打通到“导出 → 部署接口校验”，没有训练

SMPL 编码器的代码、观测项、损失其实都在仓库里（来自 Litchi / 官方 SONIC），缺的是组合在一起的配方、SMPL 数据和训练。这次补了配方并用样例数据走通：

- **配方 `sonic_a3_ultra_smpl.yaml`**（继承 `sonic_a3_ultra`）：编码器 `["g1","a3_fast","smpl"]`，`encoder_sample_probs = {g1:1, a3_fast:1, smpl:1, teleop:0}`，`reencode_smpl_g1_recon: true`，`smpl_motion_file: data/smpl_filtered`；辅助损失用新增的 `g1_recon_smpl_and_a3_fast_latent_no_teleop.yaml`——仓库里本来就有 Litchi 的 `g1_recon_smpl_and_a3_fast_latent.yaml`（注释写的是“混合 A3 数据训练：保持 G1 重建、A3-fast 对齐到 G1、在有匹配 SMPL 数据的动作上对齐 SMPL / teleop 潜变量”，没有任何配置引用它），我去掉了 teleop 两项（A3 部署不用 teleop 编码器，而这两项在 teleop 编码器不活跃时会失败）。
- **SMPL 分词器与部署一致**：`smpl_joints_multi_future_local_nonflat`(10×72) | `smpl_root_ori_b_multi_future`(10×6) | `joint_pos_multi_future_wrist_for_smpl`(10×6，`joints_idx` 23…28) = 840，窗口 10 帧 × 0.02 s，与 C++ `A3ZmqSmplSource` 和 Pico 链路里的 `A3SmplPoseAugmenter` 一致（测试里钉住）。
- **冒烟（2 条有 SMPL 配对的官方样例 `walk_forward_amateur_001__A001(_M)`，A3 片段 + `sample_data/smpl_filtered`，16 envs，3 次迭代）**：`exit=0`；`smpl` 编码器被实例化并参与采样，`g1_smpl_latent`、`reencoded_smpl_g1_latent` 等辅助损失都有值；两条动作的 SMPL 帧数与 A3 片段对齐（1202 帧 @30 fps → 2002 帧 @50 fps，库里的断言通过）。**这只是随机初始化 3 次迭代，说明链路通，不说明任何质量。**
- **导出到部署接口**：用 `callbacks.model_save.save_frequency=3` 存出检查点，`eval_agent_trl.py +checkpoint=… +export_onnx_only=true` 导出 `model_step_000003_smpl.onnx` / `_g1` / `_a3_fast` / `_encoder` / `_decoder`；`a3_smpl_onnx_tool.py check`：**`obs_dict[1,1770] → action[1,29]`，float32，通过**（`g1` / `a3_fast` 是 `[1,1570]`，与 035 一致）；部署包里 ORT 1.16.3 的 `a3_policy_runtime_probe --model … --input-dim 1770`：OK，1.5 ms，输出有限；opset 13 / IR 7（与 035 一致）。这是第 15.6 节“导出的 ONNX 天然是 `[1,1770] → [1,29]`”的实测证据。
- **SMPL 数据**：`python download_from_hf.py --training`（约 30 GB，含训练检查点；`--no-smpl` 跳过）下载到 `data/smpl_filtered/`。**本机没有，我没有下载**（体量大，请你决定）。拿到后先用新工具核对配对：`python gear_sonic/data_process/check_smpl_pairing.py --robot data/motion_lib_a3_29d/robot_filtered_v15_clean --smpl data/smpl_filtered`——按文件名配对，并检查帧数（机器人 `T` 帧 @30 fps 要对应 `⌊(T−1)·50/30⌋+1` 帧 @50 fps）和 SMPL 数据的 fps；没有 SMPL 数据的动作只是不能采样 `smpl` 编码器。

### 17.7 仍然没做 / 需要你知道的

- **A3 库与 SMPL 数据的世界朝向不一致已修（17.8）**；之后拿 `check_smpl_pairing.py` 当守卫：库没转回朝向时它会退出码 1。
- **SMPL 数据已下载（2026-10-15，17.8），没有训练**（用户决定先修朝向、不训练）：配方、数据、导出和部署接口都验证过，但 `smpl` 编码器没有任何有意义的训练；`encoder_sample_probs.smpl`、辅助损失系数沿用 G1 SONIC 的取值，没有为 A3 调过。
- 12.9 万条动作的训练显存 / 内存占用没评估（冒烟只取了 2 / 16 条）；没有收敛训练。
- `lower_joint_indices_mujoco = range(12)` 的 G1 假设仍在（只影响 `teleop` 编码器和 `cat_upper_body_poses`；本配方都不用）。
- 重定向本身的已知限制不变：站立 / 行走类动作最可靠，地面接触多的动作（坐、跪、爬）需要你目检（Litchi README 对 SOMA retargeter 也有同样的说明）。
- 我发出的“转换数据 vs Litchi 自带 20 条”的 120 次迭代对照训练被你拒绝，没有跑。

### 17.8 A3 库与 SMPL 数据的世界朝向差约 90°：原因、证据、修复（2026-10-15）

**现象**：官方样例 `walk_forward_amateur_001__A001` 的 SMPL 根朝向与 A3 库根朝向的偏航差 +86.3°（镜像动作 `_M` 为 +93.5°）；G1 库与 SMPL 的差只有 +0.3° / −0.2°。

**原因**：`batch_retarget_bvh_soma_a3ultra.py::load_soma_bvh_frames` 为了让 IK 标定偏移对所有 BVH 通用，**把每条动作整体绕 Z 转到“第 0 帧面朝 +X”**（`_yaw_f0` 补偿）。SMPL 数据和 G1 库保留采集时的原始朝向，所以 A3 库 − SMPL 的偏航差 = 这条动作的原始朝向，每条不同。

**SMPL 与 SOMA 的关系**：SMPL 数据来自同一份动捕，与 BVH 在同一个世界系里。官方样例两条动作的第 0 帧朝向：原始 BVH（归一化之前）−87.8° / −94.1°，SMPL 根 −86.6° / −93.3°，G1 库 −86.9° / −93.0°，A3 库 −2.2° / −1.0°。SMPL 根朝向按训练代码的约定计算（`smpl_y_up` → Y 转 Z → `remove_smpl_base_rot`，即 `TrackingCommand.smpl_root_quat_w`）。

**为什么要紧**：训练里 `smpl_root_ori_b = quat_inv(robot_anchor_quat_w) * smpl_root_quat_w`（`commands.py::smpl_root_quat_w_dif_l_multi_future`）是世界系下的差，机器人参考与 SMPL 必须共用一个世界朝向。没转回时，SMPL 编码器会把每条动作的常数偏航当成“人转了一个角度”（这一条是读代码推出来的，没有拿训练验证）。`smpl_joints_*_local_nonflat` 是相对根的局部量，不受影响；`g1` / `a3_fast` 编码器只看机器人参考的相对量，也不受影响。

**修复前的全库实测（真实 SMPL 数据，129,109 对，`compare_smpl_vs_a3.py --stats`）**：偏航差（A3 − SMPL）圆周均值 +89.6°，|偏差| p5 / p50 / p95 = 5.6° / 89.7° / 174.3°；**±10° 内的只有 6.5%**，+60…+120° 约 66%，−120…−60° 约 16%，±30° 内约 8%，±180° 附近约 4%；条内偏航差标准差 p50 / p95 = 2.0° / 5.2°（每条是常数旋转）。逐条结果：`data/motion_lib_a3_29d/smpl_a3_heading_offset_before.csv`。

**修法**：整个场景只是绕 Z 的刚体旋转，IK 关节角不变，所以不用重跑 110 分钟：
- `gear_sonic/data_process/restore_a3_heading.py`：对每条动作把 `root_trans_offset` 的 xy、`root_rot`、`pose_aa[:, 0]` 绕 Z 转回 `+_yaw_f0`；`_yaw_f0` 取自该动作自己的 BVH（对 45 GB 压缩包读一遍，只解析头和第 0 帧，约 8 分钟，表存成 `…capture_heading.csv`），**不是**拟合 SMPL，所以修完之后拿 SMPL 检查是独立验证。写新目录再换名，旧库保留为 `robot_filtered_v15_clean_yawnorm`。
- `batch_retarget_bvh_soma_a3ultra.py --keep_heading`：以后新跑的库直接保留采集朝向（IK 结果事后转回，关节角与不加这个参数时完全相同，测试里逐位比对）；`bvh_heading_f0()`、`rotate_qpos_about_z()`、`load_soma_bvh_frames(return_yaw=True)` 是它用的函数，默认行为不变。

**修复后的全库实测（同样 129,109 对）**：偏航差圆周均值 **−1.3°**，|偏差| p5 / p50 / p95 = 0.2° / 1.4° / 3.9°，**±10° 内 99.2%**（修前 6.5%），±20° 内 99.7%，帧数 0 不一致；对照基线 G1 库 vs SMPL（同一工具、129,778 对）：均值 +0.3°，p5 / p50 / p95 = 0.1° / 0.6° / 2.8°，±10° 内 99.5%，±20° 内 99.8%。A3 比 G1 多出的约 1° 是重定向出来的骨盆朝向与表演者朝向的固有差（修前第 0 帧 −2.2° / −1.0°），不是转回过程的误差。±20° 之外的约 0.3%（几百条）是爬行 / 躺倒这类根的 +X 轴本来就不是水平面朝向的动作。逐条结果：`…/smpl_a3_heading_offset.csv`，G1 基线：`…/g1_smpl_heading_offset_baseline.csv`。
其他核对：`convert_a3ultra_to_a3_29d.py --check` 在新库上 129,116 个文件 0 问题（格式、形状、A3 关节限位、与 A3 Ultra 库逐条帧数一致）；官方 `train_a3_035_fromscratch.sh --preflight` 通过（`preflight motion_files=129116`）；关节角、高度、`pose_aa` 的非根部分与修前完全相同（测试 + 样例逐位比对）。**没有训练。**

**守卫**：`check_smpl_pairing.py` 现在也检查朝向（`--max_heading_deg 20`、`--max_heading_fraction 0.05`）：新库前 4,000 对 0.2% 超限（中位 |偏差| 1.5°），退码 0；旧的面朝 +X 库 85.5% 超限（中位 89.9°），退码 1 并提示用 `restore_a3_heading.py`。

**检查器自己的一个 bug（已修）**：`check_smpl_pairing.py` / `compare_smpl_vs_a3.py` 起初用 `int((T-1)*50/fps)+1` 换算 A3 帧数，对 `T-1` 是 3 的倍数的动作（约 1/3）多算 1 帧，报出 43,138 对“差 1 帧”。库里实际是 `arange(0, (T-1)/fps, 1/50)`（不含终点），现已改成同一个规则，并加了与 `torch.arange` 逐个 T 对照的测试；修后 0 对不一致。

**SMPL 数据**：`python download_from_hf.py --training` 的 SMPL 部分，7 个分卷共 32.3 GB（经代理时默认的 xet 通道卡死，用 `HF_HUB_DISABLE_XET=1` 走普通 HTTPS，约 9 MB/s；没有走官方脚本而是直接调它用的 `hf_hub_download`，避免重复下载 `sonic_release/last.pt`），解压到 `data/smpl_filtered`（131,455 个 pkl，31 GB）；分卷还留在仓库根目录 `bones_seed_smpl/`（31 GB，可删）。A3 库 129,116 条中 129,109 条按文件名配上，7 条没有 SMPL 数据，2,345 条 SMPL 没有 A3 动作（被关键词过滤掉的）。

**视频**：`videos/compare_v15_a3/`（SOMA vs A3，32 条；`walk_test` 的 BVH 本机没有；格式同 `compare_v15`，整条最多 900 帧，单帧关节跳变 >25° 红字标出；抽看 4 条与 A3 Ultra 版同帧对照，姿态一致）、`videos/compare_smpl_a3/`（SMPL vs A3，30 条 = `compare_v15` 里在训练库中的 22 条 + 随机抽样 8 条，另有 `summary.csv`；左：SMPL 训练系，中：按本条偏航差转到机器人朝向的 SMPL，右：A3 重定向；抽看的 5 条姿态一致）。这批视频是在转回朝向**之前**的库上做的（用的是每条自己的偏航差，不受影响）。

### 17.9 本节新增 / 修改的文件

`gear_sonic/data_process/batch_retarget_bvh_soma_a3ultra.py`（`--robot`、`--only_from`、原子写、网格足底）、`convert_a3ultra_to_a3_29d.py`（默认输出改名为 `converted_from_a3ultra_v15_clean`）、`convert_gmr_a3_to_motion_lib.py`（新）、`gmr_ik_configs/bvh_nokov_to_a3.json` 与 `bvh_nokov_to_a3_ultra.json`（新，副本）、`check_smpl_pairing.py`（新）；`gear_sonic/config/exp/.../sonic_a3_ultra_smpl.yaml`（新）、`gear_sonic/config/aux_losses/universal_token/g1_recon_smpl_and_a3_fast_latent_no_teleop.yaml`（新）；测试：`tests/test_a3_retarget_batch.py`（新，5）、`tests/test_check_smpl_pairing.py`（新，3）、`tests/test_a3_ultra_training_recipe.py`（+1，SMPL 配方）；朝向与对比：`gear_sonic/data_process/restore_a3_heading.py`、`compare_smpl_vs_a3.py`、`make_compare_videos.py`（新）；`batch_retarget_bvh_soma_a3ultra.py`（`--keep_heading`、`bvh_heading_f0`、`rotate_qpos_about_z`、`load_soma_bvh_frames(return_yaw=)`、`_parse_soma_bvh(max_frames=, text=)`）、`check_smpl_pairing.py`（帧数规则、朝向守卫）、`visualize_retarget.py`（`MuJoCoRenderer(title=)`）；测试：`tests/test_a3_heading_restore.py`（新，6）、`tests/test_compare_smpl_vs_a3.py`（新，4）、`tests/test_check_smpl_pairing.py`（+2）；文档：本节、README、worklog #48 / #49 / #50，另有 `docs/notes/a3/mujoco_pico_sim_teleop.md`（A3 的 MuJoCo + Pico 遥操教程，三条命令原样验证过，见 worklog #50）。

**回归（本节全部改动之后）**：`tests` + `gear_sonic/tests`（`isaaclab` 环境，`--ignore=gear_sonic/tests/test_input_readers.py`）**249 通过、5 跳过**（跳过的是需要 GMR / mink 的 4 个和需要 onnxruntime 的 1 个，后者因为临时目录里的 `ort_site` 被清掉了，这次没带）；`gmr` 环境重定向 / 参考 / 转换 / 批量重定向 / 配对检查 / SMPL 对比 / 朝向测试 **52 通过**。

# G1 真机部署主机 · 完整准备与检验清单

本文保证：**在部署主机上**把真机部署所需材料一次备齐、逐项验过，再进遥操或 VLA。  
操作命令见同目录 [`g1_real_deploy_terminal_cmds.md`](g1_real_deploy_terminal_cmds.md)。

相关细文档（本清单通过后按需打开）：

- 遥操：[`pico_real_teleop.md`](pico_real_teleop.md)
- 数采：[`pico_real_data_collection.md`](pico_real_data_collection.md)
- VLA：[`../../source/tutorials/vla_inference.md`](../../source/tutorials/vla_inference.md)
- 安装：[`../../source/getting_started/installation_deploy.md`](../../source/getting_started/installation_deploy.md)

---

## 0. 约定

| 符号 | 含义 |
|------|------|
| `$REPO` | 部署机上的 `Litchi-GR00T-WholeBodyControl` 根目录 |
| `$DEPLOY` | `$REPO/gear_sonic_deploy` |
| 控制网 | 电脑 ↔ G1，通常 `192.168.123.x` |
| PICO Wi‑Fi | 电脑 ↔ 头显，**不是** `192.168.123.x` |
| 策略包 | **只许** `policy/sonic_v1_1`（禁止 `policy/release`） |

开真机前：**禁止**同时跑 `run_sim_loop.py` / MuJoCo validates。旁人盯物理急停。

---

## 1. 代码同步

```bash
cd "$REPO"
git fetch origin
git status -sb
git rev-parse --short HEAD
# 应在 base（或约定分支），与 origin 对齐；有超前 commit 先 push 再在部署机 pull
git pull --ff-only
git lfs pull   # 大文件 / mesh / 部分模型
```

检验：

- [ ] `git status` 无意外脏文件挡住更新
- [ ] `gear_sonic_deploy/target/release/g1_deploy_onnx_ref` 存在且可执行
- [ ] 工作树是 **Litchi-GR00T-WholeBodyControl**（不是旧的非 Litchi 树）

---

## 2. 系统依赖与 TensorRT / CUDA

### 2.1 安装（缺什么装什么）

```bash
cd "$DEPLOY"
chmod +x scripts/install_deps.sh scripts/setup_env.sh
./scripts/install_deps.sh
```

按 [`installation_deploy.md`](../../source/getting_started/installation_deploy.md)：

| 平台 | TensorRT |
|------|----------|
| x86_64 桌面部署机 | **10.13**（TAR 包） |
| G1 机载 Orin | **10.7** + **JetPack 6** |

```bash
# 示例：解压后
export TensorRT_ROOT=$HOME/TensorRT   # 或 deb 安装时常用 /usr
echo "export TensorRT_ROOT=$TensorRT_ROOT" >> ~/.bashrc
source ~/.bashrc
```

### 2.2 环境脚本

```bash
cd "$DEPLOY"
source scripts/setup_env.sh
echo "TensorRT_ROOT=$TensorRT_ROOT"
which nvcc || true
```

### 2.3 二进制动态库（关键）

```bash
cd "$DEPLOY"
source scripts/setup_env.sh
ldd target/release/g1_deploy_onnx_ref | grep -E 'not found|onnxruntime|nvinfer|cudart|zmq|ddsc' || true
```

检验：

- [ ] **没有** `not found`
- [ ] 能看到 `libonnxruntime`、`libnvinfer`、`libcudart`、`libzmq`、`libddsc` 一类解析成功

若缺库：回到 `install_deps.sh` / 设好 `TensorRT_ROOT` / 确认 CUDA 在 `LD_LIBRARY_PATH`（`setup_env.sh` 会配）。

### 2.4 可选：本机重编

仅当二进制与本机 ABI/依赖不匹配时：

```bash
cd "$DEPLOY"
source scripts/setup_env.sh
just build    # 或仓库约定的 cmake/just 流程
```

---

## 3. 模型与配置下载 / 核对

### 3.1 SONIC v1.1（C++ 全身控制，真机必用）

优先已有本地包；缺失时从 HF 拉：

```bash
cd "$REPO"
python download_from_hf.py --sonic-v1-1
# 或按团队内网拷贝整目录 policy/sonic_v1_1 + planner
```

必存在文件：

```bash
ls -lh \
  "$DEPLOY/target/release/g1_deploy_onnx_ref" \
  "$DEPLOY/policy/sonic_v1_1/model_encoder.onnx" \
  "$DEPLOY/policy/sonic_v1_1/model_decoder.onnx" \
  "$DEPLOY/policy/sonic_v1_1/observation_config.yaml" \
  "$DEPLOY/planner/target_vel/V2/planner_sonic.onnx"
```

量级参考（允许略有差异）：

| 文件 | 约 |
|------|-----|
| `g1_deploy_onnx_ref` | ~5MB |
| `model_encoder.onnx` | ~48MB |
| `model_decoder.onnx` | ~143MB |
| `observation_config.yaml` | ~2–3KB |
| `planner_sonic.onnx` | 百 MB 级 |

可选 TRT 引擎（加速；**与本机 GPU/TRT 版本绑定**）：

```bash
ls -lh \
  "$DEPLOY/policy/sonic_v1_1/"*.trt \
  "$DEPLOY/planner/target_vel/V2/"*.trt 2>/dev/null || true
```

检验：

- [ ] 上表 onnx + yaml + planner + 二进制齐全
- [ ] **没有**误用 `policy/release`
- [ ] 换机 / Orin：若 `*.trt` 报错，在本机按文档重导引擎，勿硬拷不匹配架构

### 3.2 VLA 权重（仅跑 VLA 时）

origin RTC 训练产物示例（路径按现场改）：

```text
…/walk_towards_the_green_table_skill1_all_rtc_2cam_origin/walk_towards_green_table_skill1_all_rtc_2cam_origin
```

检验：

- [ ] 目录内有 `config.json`、safetensors / checkpoint、`processor/`
- [ ] PolicyServer 能 `from_pretrained` 加载（见指令文档）
- [ ] **RTC 推理参数**与训练一致：`overlap=28, frozen=5, ramp=3.0, H=40`  
  （脚本默认常是 16/8/5.0，**真机必须显式覆盖**）

### 3.3 Isaac-GR00T / Litchi-Isaac-GR00T

- PolicyServer 在 GPU 机：使用 `Litchi-Isaac-GR00T`（或约定 GR00T 仓）+ 对应 `.venv`
- 部署机若只跑 client：`bash install_scripts/install_inference.sh` → `.venv_inference`

---

## 4. Python 环境

### 4.1 遥操 `.venv_teleop`

```bash
cd "$REPO"
# 若无环境：按 README / install_pico*.sh 创建
bash install_scripts/install_pico.sh          # 或 install_pico_minimal.sh
# 检验
"$REPO/.venv_teleop/bin/python" -c \
  "import xrobotoolkit_sdk, zmq; print('teleop OK')"
```

### 4.2 推理 `.venv_inference`（VLA）

```bash
cd "$REPO"
bash install_scripts/install_inference.sh
"$REPO/.venv_inference/bin/python" -c "import zmq; print('inference OK')"
```

### 4.3 数采（可选）

```bash
bash install_scripts/install_data_collection.sh
```

检验：

- [ ] 遥操：`.venv_teleop` 可用
- [ ] VLA：`.venv_inference` 可用（或 GPU 机 PolicyServer + 本机 client）
- [ ] 无混用非 Litchi 旧 venv

---

## 5. 网络与硬件

### 5.1 控制网（G1）

```bash
ip -4 -br addr
# 应有 192.168.123.x；没有则插网线 / 配静态 IP
ping -c 2 192.168.123.161   # 换成现场机器人 IP
```

`deploy.sh … real` 会自动找 `192.168.123.*`；找不到时改传具体 IP。

### 5.2 PICO Wi‑Fi

```bash
ip -4 -br addr show wlp8s0   # 或现场无线网卡名
pgrep -af RoboticsServiceProcess || bash /opt/apps/roboticsservice/runService.sh
```

头显 XRoboToolkit **PC Service** 填 **Wi‑Fi IP**，状态 **WORKING**，勾选 Head/Controller + Full body（2× 脚 Tracker）。

### 5.3 相机（VLA / 数采）

Orin / 相机机上：

```bash
# 示例：双 RealSense ego+head → ZMQ :5555
bash "$DEPLOY/scripts/g1_dual_realsense_rgb_server.sh"
# 或统一入口
bash "$DEPLOY/scripts/g1_camera_rgb_server.sh"
```

部署机检验：

```bash
ping -c 2 192.168.123.164    # 相机主机
# 可选 viewer
source "$REPO/.venv_teleop/bin/activate"   # 或 inference
python gear_sonic/scripts/run_camera_viewer.py --camera-host 192.168.123.164
```

检验：

- [ ] 控制网通
- [ ] PICO WORKING（仅遥操需要）
- [ ] VLA：`:5555` 有图；与训练模态一致（本 skill 至少 **ego_view + head**）

---

## 6. 进程冲突与安全

```bash
pgrep -af 'run_sim_loop|run_sim_record|g1_deploy_onnx_ref' || echo 'no conflicting deploy/sim'
```

检验：

- [ ] 无残留仿真 / 旧 deploy
- [ ] 物理急停可及；软件急停：部署终端 **`O`**（G1）
- [ ] 首次上电建议吊带 / 龙门

---

## 7. 冒烟：只起 C++ deploy（不动真机策略也可先 Init）

```bash
cd "$DEPLOY"
source scripts/setup_env.sh
./deploy.sh \
  --robot g1 \
  --cp policy/sonic_v1_1/model \
  --obs-config policy/sonic_v1_1/observation_config.yaml \
  --input-type zmq_manager \
  --motor-kp-scale 4,10=1.5 \
  --motor-kd-scale 4,10=1.5 \
  real
# 确认 y → 等到日志 Init Done
```

检验：

- [ ] 日志出现 **`Init Done`**
- [ ] 无 DDS / ChannelFactory / 找不到 interface 的致命错误
- [ ] 需要时 `Ctrl+C` 或 `O` 干净退出（勿 `kill -9`）

---

## 8. 总验收表（上机前勾完）

| # | 项 | 通过 |
|---|----|------|
| 1 | `$REPO` 已 pull，Litchi 树 | ☐ |
| 2 | `setup_env` + `ldd` 无 not found | ☐ |
| 3 | `sonic_v1_1` onnx/yaml + planner + 二进制 | ☐ |
| 4 | 未使用 `policy/release` | ☐ |
| 5 | `.venv_teleop`（遥操）/ `.venv_inference`（VLA） | ☐ |
| 6 | `192.168.123.x` 通；PICO Wi‑Fi 正确（如需） | ☐ |
| 7 | 无仿真抢接口 | ☐ |
| 8 | deploy 能到 `Init Done` | ☐ |
| 9 | VLA：权重可加载；RTC=28/5/3.0；双相机通 | ☐ |
| 10 | 急停与吊带就绪 | ☐ |

全部勾选后再打开 [`g1_real_deploy_terminal_cmds.md`](g1_real_deploy_terminal_cmds.md) 执行正式部署指令。

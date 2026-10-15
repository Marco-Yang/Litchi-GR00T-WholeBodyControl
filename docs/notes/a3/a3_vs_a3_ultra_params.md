# Litchi A3 与 A3 Ultra 参数对比

对比的是两份已经落在仓库里的参数，不是两台机器人重新测量的结果。

| | Litchi A3 | A3 Ultra 参数目录 |
|---|---|---|
| 位置 | `Litchi_sonic_for_a3` | `GR00T-WholeBodyControl/docs/robot_parameters/A3_Ultra` |
| 控制与限幅 | `gear_sonic/envs/manager_env/robots/a3.py` | `A3_关节控制参数KpKd.docx` |
| 电机惯量 | `gear_sonic/utils/a3_motor_params.py` | `armature .png`、`instructions.md`、`Pasted image (2).png` |
| 关节机械限位 | 035 使用的 URDF：`model_collision_optimized_passive_foot_twostage_fit_optimized.urdf` | 同一份 docx 里的上下限列 |

035 训练配置 `sonic_a3_035.yaml` 的 `robot.type` 是 `a3_024`。下文里的「A3 基线」是 `A3_CYLINDER_CFG`，「035 / 024」是在基线上改过力矩和踝关节 Kp/Kd 的 `A3_024_CYLINDER_CFG`。左右关节成对，表里只写一侧。

## 结论

29 个身体关节的位置限位、额定速度和电机型号，两边是同一套数。Ultra 表把弧度保留到小数点后三位，Litchi URDF 用的是同一组角度换成的更长小数，例如髋 pitch 下限都是 −144°。

真正不一样的有四件事：

1. Ultra 表多两个头关节 `head_yaw_joint`、`head_pitch_joint`。Litchi A3 的策略不输出头。
2. Litchi 多四个被动脚关节（前脚掌、脚趾）。它们不进策略，Ultra 表里没有。
3. Kp、Kd 几乎每一组都不一样。Ultra 表的腿和腰刚度明显高于 035 训练时用的值；腕 pitch/yaw 相反，Litchi 更高。
4. 035 又把部分峰值力矩从基线再降了一档：膝 320→300 N·m，腰 yaw 220→200，腰 roll 46→42，腰 pitch 115→105。

所以 035 权重量的是 024 这套更软的 PD 和更低的腰、膝力矩上限。把 Ultra 表里的 Kp/Kd 直接换进 035 部署，动作缩放 `0.25 × 力矩 / Kp` 会变，和训练时的动作幅度对不上。

## 关节集合

Litchi 主动关节 29 个：腿 12、腰 3、臂和腕 14。顺序写在 `A3_ACTIVE_JOINT_NAMES`。

Ultra 表是 31 行，前 29 行名字与 Litchi 主动关节相同，后面加：

| 关节 | Kp | Kd | 下限 | 上限 | 力矩 (N·m) | 速度 (rad/s) |
|---|---:|---:|---:|---:|---:|---:|
| head_yaw_joint | 40 | 2 | −1.047 rad（−60°） | 1.047 rad（60°） | 6 | 12.78 |
| head_pitch_joint | 40 | 2 | −0.436 rad（−25°） | 0.262 rad（15°） | 6 | 12.78 |

电机表把这两个头关节标成上肢 J6、J7，型号 PFP-41-48，和腕 pitch/yaw 同一档电机。

Litchi 另外有四个被动脚关节，只存在于 URDF 和 `passive_foot_springs` 执行器里：

| 关节 | 位置限位 | URDF 力矩/速度 | 仿真 Kp | 仿真 Kd |
|---|---|---|---:|---:|
| `*_foot_forefoot_joint` | ±0.0661 rad（±3.79°） | 20 N·m，20 rad/s | 2396.666 | 1.51683722 |
| `*_foot_toe_joint` | ±0.2306 rad（±13.21°） | 20 N·m，20 rad/s | 27.58407 | 0.0686125693 |

仿真里这两个关节的力矩上限被写成 400 / 100 N·m，和 URDF 的 20 N·m 不同。它们的 armature 是 1e-4 和 3e-5，摩擦为 0。

## 位置限位和速度

29 个主动关节的角度上下限一致。下面用 Ultra 表的三位小数；Litchi URDF 是同一角度的精确弧度，差在四舍五入。

| 关节 | 下限 | 上限 | 速度 (rad/s) |
|---|---:|---:|---:|
| hip_pitch | −2.513（−144°） | 2.932（168°） | 12.04 |
| hip_roll（左） | −0.524（−30°） | 1.606（92°） | 12.04 |
| hip_roll（右） | −1.606（−92°） | 0.524（30°） | 12.04 |
| hip_yaw | −2.723（−156°） | 2.723（156°） | 12.04 |
| knee | −0.122（−7°） | 2.496（143°） | 14.66 |
| ankle_pitch | −0.908（−52°） | 0.524（30°） | 10.80 |
| ankle_roll | −0.349（−20°） | 0.349（20°） | 19.37 |
| waist_yaw | −2.618（−150°） | 2.618（150°） | 12.04 |
| waist_roll | −0.349（−20°） | 0.349（20°） | 22.70 |
| waist_pitch | −0.489（−28°） | 0.419（24°） | 9.25 |
| shoulder_pitch | −2.880（−165°） | 2.880（165°） | 13.61 |
| shoulder_roll（左） | −0.087（−5°） | 2.618（150°） | 13.61 |
| shoulder_roll（右） | −2.618（−150°） | 0.087（5°） | 13.61 |
| shoulder_yaw | −2.793（−160°） | 2.793（160°） | 15.71 |
| elbow | −0.960（−55°） | 1.745（100°） | 15.71 |
| wrist_roll | −2.793（−160°） | 2.793（160°） | 15.71 |
| wrist_pitch / yaw | −1.623（−93°） | 1.623（93°） | 12.78 |

Litchi 的 `velocity_limit_sim` 用的是未四舍五入的同一组数，例如髋是 `12.042771838760874`，肩 pitch 是 `13.613568165555769`。这些速度对应电机表里的输出端额定转速，不是峰值转速。`instructions.md` 写明：额定转速覆盖 90% 以上场景，训练速度限幅用额定值。

## 力矩

URDF 的 `<limit effort>` 和 Ultra 表的力矩列一致。Litchi 基线在腰 pitch 上改用了 `instructions.md` 的等效值 115，而不是 URDF/表里的 118。035 再往下降。

| 关节 | Ultra 表 / URDF | Litchi 基线 `effort_limit_sim` | 035 / 024 |
|---|---:|---:|---:|
| hip_pitch / roll / yaw | 220 | 220 | 220 |
| knee | 320 | 320 | **300** |
| ankle_pitch | 118.2 | 118.2 | 118.2 |
| ankle_roll | 54.75 | 54.75 | 54.75 |
| waist_yaw | 220 | 220 | **200** |
| waist_roll | 46 | 46 | **42** |
| waist_pitch | 118 | **115** | **105** |
| shoulder_pitch / roll | 60 | 60 | 60 |
| shoulder_yaw、elbow、wrist_roll | 24 | 24 | 24 |
| wrist_pitch / yaw | 6 | 6 | 6 |

`instructions.md` 对并联关节的等效峰值力矩是：ankle pitch 118.2、ankle roll 54.75、waist pitch 115、waist roll 46。腰 pitch 这一项，同一目录里的 docx 写 118，说明写 115。Litchi 基线跟的是说明里的 115。

电机表上的峰值扭矩是电机输出端峰值，不是每个等效关节的限幅：PFP-110-75 为 320，PFP-93-65 为 220，PFP-78-58 为 60，PFP-59-60 为 36，PFP-41-48 为 6。踝和腰的 roll/pitch 是 PFP-78-58 经过并联折算后的等效力矩，所以不是 60。

## Kp 和 Kd

这是两份参数差得最大的地方。Ultra 表是关节控制参数；Litchi 写在仿真执行器的 `stiffness` / `damping` 里，也是部署时要跟训练对齐的 PD。

| 关节 | Ultra Kp / Kd | Litchi 基线 | 035 / 024 |
|---|---:|---:|---:|
| hip_pitch | 200 / 7 | 80 / 3 | 80 / 3 |
| hip_roll | 200 / 7 | 120 / 4 | 120 / 4 |
| hip_yaw | 200 / 7 | 80 / 3 | 80 / 3 |
| knee | 360 / 13 | 250 / 8 | 250 / 8 |
| ankle_pitch | 80 / 3 | 50 / 2 | **60 / 1.2** |
| ankle_roll | 80 / 3 | 50 / 2 | **60 / 1.2** |
| waist_yaw | 200 / 7 | 85 / 3 | 85 / 3 |
| waist_roll | 80 / 3 | 50 / 2 | 50 / 2 |
| waist_pitch | 80 / 3 | 50 / 2 | 50 / 2 |
| shoulder_pitch / roll | 90 / 4 | 40 / 3 | 40 / 3 |
| shoulder_yaw、elbow、wrist_roll | 40 / 3 | 30 / 2 | 30 / 2 |
| wrist_pitch / yaw | 15 / 1 | 20 / 2 | 20 / 2 |

024 相对基线只改了踝：Kp 50→60，Kd 2→1.2。臂的 Kp/Kd 两边版本相同。腕 pitch/yaw 是唯一 Litchi Kp 高于 Ultra 表的一组（20 对 15）。

Litchi 用这些数生成动作缩放，公式是 `0.25 × effort_limit / stiffness`。035 的缩放因此是：

| 关节 | 035 动作缩放 |
|---|---:|
| hip_pitch、hip_yaw | 0.6875 |
| hip_roll | 0.4583 |
| knee | 0.3000 |
| ankle_pitch | 0.4925 |
| ankle_roll | 0.2281 |
| waist_yaw | 0.5882 |
| waist_roll | 0.2100 |
| waist_pitch | 0.5250 |
| shoulder_pitch / roll | 0.3750 |
| shoulder_yaw、elbow、wrist_roll | 0.2000 |
| wrist_pitch / yaw | 0.0750 |

Ultra 表本身没有动作缩放。若用表里的 Kp 和力矩套同一个公式，髋会从 0.6875 变成 `0.25 × 220 / 200 = 0.275`，膝会从 0.3000 变成 `0.25 × 320 / 360 ≈ 0.222`。这就是不能把 Ultra 表的 Kp 套到 035 上的原因。

Litchi 还有摩擦，Ultra 表没有对应列：腿和腰静态摩擦 0.15、动摩擦 0.1；踝 0.25 / 0.2；臂和腕都是 0.1。`soft_joint_pos_limit_factor` 在 `a3.py` 里是 0.9；035 配置再把三个腰关节的软限位系数设成 0.8。

## 电机型号和关节侧 armature

电机表和 `a3_motor_params.py` 是同一组数。关节侧惯量按 `J_output = J_motor × N²` 折到输出端，也就是仿真里的 armature。一级减速效率 0.95、二级 0.9 只用于说明里的理论折算，代码里的 armature 没有再乘这两个效率。

| 型号 | 装在哪 | J_motor (kg·m²) | 减速比 N | 关节侧 armature |
|---|---|---:|---:|---:|
| PFP-110-75 | 膝（下肢 J4） | 300.851e-6 | 20 | 0.12034040 |
| PFP-93-65 | 髋 pitch/roll/yaw，腰 yaw | 138.5069e-6 | 21.906 | 0.06646570 |
| PFP-78-58 | 肩 pitch/roll；踝、腰 roll/pitch 的电机 | 30.118e-6 | 20.03 | 0.01208337 |
| PFP-59-60 | 肩 yaw、肘、腕 roll | 10.1374e-6 | 22.136 | 0.00496735 |
| PFP-41-48 | 腕 pitch/yaw；Ultra 的头 yaw/pitch | 1.359e-6 | 24.415 | 0.00081009 |

电机表截图里 PFP-110-75 的 J_motor 写作 `300.850171e-6`，代码写作 `300.851e-6`。折到关节侧后相差约 3.3e-7 kg·m²，可以忽略。

额定扭矩 / 峰值扭矩、额定转速 / 峰值转速：

| 型号 | 额定 / 峰值扭矩 (N·m) | 额定 / 峰值转速 (rpm) |
|---|---|---|
| PFP-110-75 | 70 / 320 | 140 / 190 |
| PFP-93-65 | 45 / 220 | 115 / 206 |
| PFP-78-58 | 17 / 60 | 130 / 240 |
| PFP-59-60 | 10 / 36 | 140 / 170 |
| PFP-41-48 | 2 / 6 | 150 / 200 |

## 并联踝和腰

仿真里的踝、腰 roll/pitch 是串联等效关节。`instructions.md` 给出的放大系数和 Litchi 代码一致，基准都是 PFP-78-58 的关节侧 armature 0.01208337：

| 等效关节 | 惯量倍数 | armature | 等效峰值力矩 | 等效速度 |
|---|---:|---:|---:|---:|
| ankle_pitch | 5.333 | 0.064441 | 118.2 N·m | 10.8 rad/s |
| ankle_roll | 1.66562 | 0.020126 | 54.75 N·m | 19.37 rad/s |
| waist_pitch | 7.3 | 0.088209 | 说明写 115；docx 写 118 | 9.24785 rad/s |
| waist_roll | 1.21 | 0.014621 | 46 N·m | 22.7 rad/s |

说明里把腰 roll 那一行误写成了 “ankle_roll”，倍数 1.21 和力矩 46 对得上腰 roll。`Pasted image (2).png` 补充：等效 armature 理论上随姿态变化，而且踝的 pitch/roll 之间有耦合项；Isaac 里每个关节只能放一个标量，所以上面这些倍数是估计值。

## Ultra 文档里多出来的两张表

这两张表在 Kp/Kd 的 docx 后半部分，Litchi 的 `a3.py` 里没有同名参数。它们描述的是带球拍的训练随机化，不是 A3 本体的机械参数。

观测噪声：

| 观测量 | 尺度 | 分布 |
|---|---|---|
| 关节位置 | 0.03 | 高斯 |
| 关节速度 | 1.0 | 高斯 |
| IMU 姿态 / 角速度 | 0.06 / 0.3 | 高斯 |
| 基座速度 | xy 均匀分布 [−0.3, 0.3] | 均匀 |
| 末端 / 球拍姿态 | rpy 均匀分布 [−0.2, 0.2] | 均匀 |
| 动捕 / 视距 | xy [−0.1, 0.1]，z [−0.05, 0.05]，rpy [−0.2, 0.2] | 均匀 |

域随机范围：躯干惯量缩放 (−0.125, 0.125)，身体惯量缩放 (−0.06, 0.06)，`body_friction` (0.8, 1.4)，编码器零偏 (−0.07, 0.07) rad，足底摩擦 (0.3, 1.2)。后面还有乒乓球初始位姿和外部感知的时间同步误差（−290 ns 到 220 ns），和 A3 关节参数无关。

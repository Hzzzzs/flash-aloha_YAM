# YAM夹爪摩擦补偿：调参和手动示教助力

适用：YAM Ultra 2 + linear_4310，已完成[环境安装](yam_quickstart.md)。
当前为**已通过无硬件测试、待实机调参验证**的可选功能；默认数采行为不变。
它补偿手动开闭时的运动阻力，不更改六个手臂关节的重力补偿、零位或录制动作标签。

## 与ARX参考的区别

参考 [ARX摩擦调参工具](https://github.com/JohnYao93/flash-aloha/blob/main/src/hardware/arx5-sdk/python/examples/tune_gripper_friction_compensation.py)
及其平滑库仑/粘性摩擦模型；[force示例](https://github.com/JohnYao93/flash-aloha/blob/main/src/hardware/arx5-sdk/python/examples/test_gripper_force_compensation.py)
实际使用加速度助力，本次没有引入加速度反馈，也没有直接套ARX参数。

YAM位置/速度反馈为归一化开度及开度每秒；先用SDK的JointMapper转换为电机坐标rad/s，
自动保留标定范围的正负方向，再计算Nm补偿。不能把归一化速度直接当rad/s，或再翻转一次方向。

令 `v_eff = sign(v) × max(|v| − deadband, 0)`，助力为：

```text
tau = coulomb_nm × tanh(v_eff / transition_rad_s) + viscous_nm_s_rad × v_eff
```

限幅后，在运动朝向的行程端点附近衰减。静止死区输出0；不使用常量偏置。
因此它可能减轻滑动过程的阻力，但不能保证消除从完全静止开始运动的静摩擦。
如果机械部件卡住、安装过紧或摩擦异常，不能靠继续增加助力来掩盖。

## 1. 单臂试调（操作员执行）

退出其他机械臂控制程序，托稳被测臂，夹爪内不要放手指，留出启动标定空间。
启动会按现有SDK流程标定夹爪；随后六个关节保持当前位置，不自动回零。

```bash
bash scripts/tune_yam_gripper.sh --arm right
# 右臂结束并托稳后，左臂可单独运行：
bash scripts/tune_yam_gripper.sh --arm left
```

输入 `YAM_GRIPPER_READY` 后，通过夹爪外部握把手动开闭，不要拖动处于保持状态的手臂。
即使读到保存的参数，每次工具启动也先关闭助力。

| 按键 | 操作 |
| --- | --- |
| `1` | 助力开/关，便于同一位置比较手感 |
| `+`或`=` / `-` | 库仑项每次增加/减少0.005Nm |
| `]` / `[` | 粘性项每次增加/减少0.001Nm·s/rad |
| `.` / `,` | 平滑过渡速度增加/减少0.05rad/s |
| `s` | 保存当前这只夹爪的参数，保留另一侧配置 |
| `q` | 原地关闭电机；按之前先托稳机械臂 |

首次从0开始，先按一次`+`，再按`1`，做几次开合，与关助力时比较。
先只调库仑项，粘性项保持0。每次增加后放开外部握把观察，
若出现自己持续滑动、抖动或方向异常，按`1`关闭并减小参数，不能把这种状态当作调好。
合理目标是在开、闭两个方向都更省力，同时放手不会持续自己运动。

默认力矩上限0.03Nm，常用按键不会超过该值；配置允许的硬上限为0.06Nm。
这些是软件试调上限，不是对手指接触安全的认证值。SDK的5N限力通过调整位置目标实现，
**不约束本功能直接加入的前馈力矩**，因此不能把5N当成此时的实际夹持力保证。
速度超过8rad/s、反馈过期/异常会撤去助力并向主流程报错；程序正常异常退出沿用阻尼退出。
Ctrl+C后按h会主动回到启动姿态；阻尼等待中再按Ctrl+C才原地禁用，禁用前需托稳。

## 2. 保存与启用

`s`默认写入 `config/yam-gripper-friction.json`（本机文件，Git忽略）。
保存的是参数，**不是“已验收”标记**；调参工具处于OFF时保存的非零参数，显式用于数采后仍会启用。
左右分别调试和保存；只保存一侧时，另一侧继续无助力。
调参设置日志在 `logs/yam_gripper_tuning/`（跟随FLASH_ALOHA_LOG_DIR），
只记录设置与开关状态，不另造轨迹/观测数据。

确认手感和放手表现后，由操作员显式启动带助力数采：

```bash
# 保持当前已允许的旧电脑临时150/100ms配置；新电脑可按主教程用默认80/50ms。
CAMERA_MAX_AGE_MS=150 CAMERA_MAX_SKEW_MS=100 \
  bash scripts/collect_yam.sh --duration 60 \
  --gripper-friction-config config/yam-gripper-friction.json
```

不加该选项即维持原来无夹爪助力的采集。不与`--pick-place`按键位置控制或`--camera-only`组合。
助力仅作用于手动示教；保持、回放、回起点、阻尼与关闭时清零。
参数按左右写入episode元数据的 `manual_gripper_friction`，qpos/qvel与动作仍按原标准采集。
助力不能改变或修复相机帧龄超时。

## 参数含义与运行边界

| 字段 | 单位/默认值 | 作用 |
| --- | --- | --- |
| coulomb_nm | Nm / 0 | 沿运动方向的平滑库仑助力 |
| viscous_nm_s_rad | Nm·s/rad / 0 | 随运动速度增加的助力；过大可能自驱 |
| transition_rad_s | rad/s / 0.2 | 越大，低速时助力建立越平缓 |
| deadband_rad_s | rad/s / 0.02 | 抑制近静止速度噪声 |
| max_torque_nm | Nm / 0.03 | 所有助力项合计的限幅 |
| max_speed_rad_s | rad/s / 8 | 超过则停止助力并报告错误 |
| end_margin | 归一化行程 / 0.05 | 接近朝向端点的5%行程内衰减 |

实现使用现有YAM控制器的约100Hz监视线程，在SDK命令锁内只更新J7前馈。
固定SDK的`command_joint_state`忽略传入的torques字段，所以不能只给该字典多加一个字段。
主流程超过0.5秒不读取/检查状态则助力归零，恢复读取后仅在手动模式恢复。
软件线程监视不是固件级失联保护，不能保证操作系统整体冻结时仍执行。
无需更改SDK源码补丁或重新安装环境。

无硬件验证：`bash scripts/check_yam.sh`。测试包含速度单位/正反标定方向、死区、限幅、
端点衰减、单J7写入、模式切换清零、反馈异常、心跳过期和左右参数保存。

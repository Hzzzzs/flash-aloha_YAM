# YAM夹爪摩擦与加速度补偿：调参和手动示教助力

适用：YAM Ultra 2 + linear_4310，已完成[环境安装](yam_quickstart.md)。
当前为**已通过无硬件测试、待实机调参验证**的可选功能；默认数采行为不变。
它补偿手动开闭时的运动阻力，不更改六个手臂关节的重力补偿、零位或录制动作标签。

## 与ARX参考的区别

参考 [ARX摩擦调参工具](https://github.com/JohnYao93/flash-aloha/blob/main/src/hardware/arx5-sdk/python/examples/tune_gripper_friction_compensation.py)
及其平滑库仑/粘性摩擦模型；[force示例](https://github.com/JohnYao93/flash-aloha/blob/main/src/hardware/arx5-sdk/python/examples/test_gripper_force_compensation.py)
实际使用加速度助力。现在支持独立启用加速度项，默认关闭，系数默认0；没有直接套ARX参数。

YAM位置/速度反馈为归一化开度及开度每秒；先用SDK的JointMapper转换为电机坐标rad/s，
自动保留标定范围的正负方向，再计算Nm补偿。不能把归一化速度直接当rad/s，或再翻转一次方向。

令 `v_eff = sign(v) × max(|v| − deadband, 0)`，助力为：

```text
tau_friction = coulomb_nm × tanh(v_eff / transition_rad_s) + viscous_nm_s_rad × v_eff
tau = clip(tau_friction + tau_acceleration, -max_torque_nm, max_torque_nm)
```

限幅后，在运动朝向的行程端点附近衰减。静止死区输出0；不使用常量偏置。
因此它可能减轻滑动过程的阻力，但不能保证消除从完全静止开始运动的静摩擦。
如果机械部件卡住、安装过紧或摩擦异常，不能靠继续增加助力来掩盖。

加速度由电机坐标速度差除以相邻SDK反馈的实际时间间隔得到，再用默认40ms时间常数做一阶低通。
重复/不足1ms的反馈不做差分；间隔超过100ms或时间倒退时重新初始化。第一帧不输出加速度助力。
滤波后的加速度扣除0.5rad/s²死区，乘以`acceleration_gain_nm_s2_rad`，与摩擦项共用总限幅和端点衰减。
与ARX示例不同，只在速度超出死区且加速度与速度同向时启用该项：减速、静止时不提供加速度助力。
这仍是运动反馈助力，不测量手指力，也不能在完全静止时判断意图。
关闭助力、切换模式、心跳超时及调参时清空差分历史，恢复后从首帧开始。

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
| `a` | 独立开/关加速度项；仍受`1`总开关控制 |
| `u` / `j` | 加速度系数每次增加/减少0.0001Nm·s²/rad |
| `s` | 保存当前这只夹爪的参数，保留另一侧配置 |
| `q` | 原地关闭电机；按之前先托稳机械臂 |

首次从0开始，先按一次`+`，再按`1`，做几次开合，与关助力时比较。
先只调库仑项，粘性项保持0。每次增加后放开外部握把观察，
若出现自己持续滑动、抖动或方向异常，按`1`关闭并减小参数，不能把这种状态当作调好。
合理目标是在开、闭两个方向都更省力，同时放手不会持续自己运动。

### 加速度助力对照试验

更新后的工具需要退出旧进程后重新启动。启动时总助力OFF；若加载配置，加速度子开关沿用保存值。
先用`-`和`[`将Fc、B归零，隔离加速度项；按一次`u`设置Ka=0.0001，
用`a`使屏幕显示`accel=ON`，用`1`使总开关显示`ON`。若已有保存参数，以屏幕实际值为准。
通过外部握把试开合，用`a`切换比较；必要时每次只增加一次`u`。
Ka允许0–0.002，但此范围是软件试调边界，尚未实机验证最佳值。
如果自行运动或抖动，按`1`撤去全部助力，再减小参数。
先验证单独加速度项，再逐步叠加摩擦项。若摩擦项已占满0.03Nm限幅，加入加速度项不会再增加总输出。
屏幕`assist`是两项合计的实际下发力矩，不是测量的手指力。

默认力矩上限0.03Nm，常用按键不会超过该值；配置允许的硬上限为0.06Nm。
这些是软件试调上限，不是对手指接触安全的认证值。SDK的5N限力通过调整位置目标实现，
**不约束本功能直接加入的前馈力矩**，因此不能把5N当成此时的实际夹持力保证。
速度超过8rad/s、反馈过期/异常会撤去助力并向主流程报错；程序正常异常退出沿用阻尼退出。
Ctrl+C后按h会主动回到启动姿态；阻尼等待中再按Ctrl+C才原地禁用，禁用前需托稳。

## 2. 保存与启用

`s`默认写入 `config/yam-gripper-friction.json`（本机文件，Git忽略）。
保存的是参数，**不是“已验收”标记**；调参工具处于OFF时保存的非零参数，显式用于数采后仍会启用。
加速度子开关`acceleration_enabled`会保存；关闭它再保存则数采不使用加速度项。
新保存配置为v2，读取时兼容旧v1（缺失的加速度参数默认关闭/零）。旧版本代码不能读取v2配置。
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
算法标识为`velocity_tanh_filtered_acceleration_v2`，其中包含加速度启用状态及全部系数。
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
| acceleration_enabled | false | 加速度子开关 |
| acceleration_gain_nm_s2_rad | Nm·s²/rad / 0 | 加速度助力系数，范围0–0.002 |
| acceleration_deadband_rad_s2 | rad/s² / 0.5 | 抑制微小加速度噪声 |
| acceleration_filter_s | s / 0.04 | 加速度一阶滤波时间常数 |

实现使用现有YAM控制器的约100Hz监视线程，在SDK命令锁内只更新J7前馈。
固定SDK的`command_joint_state`忽略传入的torques字段，所以不能只给该字典多加一个字段。
主流程超过0.5秒不读取/检查状态则助力归零，恢复读取后仅在手动模式恢复。
软件线程监视不是固件级失联保护，不能保证操作系统整体冻结时仍执行。
无需更改SDK源码补丁或重新安装环境。

无硬件验证：`bash scripts/check_yam.sh`。测试包含速度单位/正反标定方向、死区、限幅、
端点衰减、单J7写入、模式切换清零、反馈异常、心跳过期和左右参数保存，
以及加速度差分的真实时间间隔、重复反馈、噪声、正反标定方向、历史重置及v1/v2配置。

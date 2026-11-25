# YAM Ultra 2 双臂：安装、数采、回放与迁移

## 当前交付范围

使用i2rt SDK控制两只YAM Ultra 2 + linear_4310，手动引导双臂与夹爪，
采集两台腕部D405的640×480 RGB和14维状态/动作，目标30 Hz，保存后可回放。
未启用深度流；没有第三台俯视相机。训练和自主策略部署仍待接入验证。

[验证报告](validation/2026-11-25-yam.md)区分流程成功、时序限制和动态跟踪误差。
现有旧电脑使用USB2连接；新主机及USB3链路尚未现场验证。

## 1. 从新电脑安装（不连接硬件也能完成）

支持并验证的软件环境：Ubuntu 24.04 x86_64、Python 3.11.15。
需要联网下载源码、Python和依赖，预留约5GB磁盘空间。完整依赖固定在
`scripts/requirements-yam-lock.txt`，包含数采、诊断、离线测试与绘图，不含PyTorch训练栈。

```bash
sudo apt-get update
sudo apt-get install -y git patch ffmpeg can-utils v4l-utils usbutils \
  python3-venv build-essential libgl1 libegl1 libglfw3 libglib2.0-0

git clone --branch review/yam-collection-20261125 https://github.com/Hzzzzs/flash-aloha_YAM.git
cd flash-aloha_YAM

# 已有uv 0.11.24可跳过这三行；工具环境与YAM运行环境分开。
python3 -m venv .venv-bootstrap
.venv-bootstrap/bin/python -m pip install uv==0.11.24
export PATH="$PWD/.venv-bootstrap/bin:$PATH"

bash scripts/setup_yam.sh
bash scripts/check_yam.sh
```

安装器只安装软件：创建 `.venv-yam/` 和 `.yam-deps/i2rt/`，不打开CAN、相机或控制器。
已有同名环境会按锁定清单同步，请不要把其他项目环境作为 `YAM_VENV_DIR`。
无硬件检查验证依赖、补丁、FFmpeg编码器、Ultra2+linear_4310模型加载/FK及模拟测试。
当前预期为112项测试通过（另有4个subtests）；这不等于真实机器人验收。
安装与检查已在不含旧环境/SDK的独立目录执行通过，见[复现记录](validation/2026-11-25-reproduction.md)。

SDK固定为官方提交 `ea116fe4079e991958b22f879874402674ee6590`（包版本1.1.2），
安装器应用 `scripts/patches/i2rt-explicit-motor-disable.patch`，复现原实验环境的两处退出修改：
先发送零力矩/零增益命令，停止控制线程后逐轴DISABLE，再关闭CAN。
比对原本机SDK，官方跟踪文件中只有这两个Python文件存在差异；模型和默认重力参数与该提交一致。
基线、修改前后哈希见 `scripts/patches/i2rt-source.json`；MIT许可随补丁保留。
**这不是新改的控制策略，也不写电机零位。**

安装器还从干净的robologger 0.1.7应用编码补丁。共享libx264路径使用
ultrafast、1编码线程、1颜色转换线程、zerolatency、5Mbps；同码率画质尚未单独验收。
不要另用 `pip install -U` 升级其中某个依赖后仍当成同一验证环境。

新安装默认使用仓库内环境；已有实验室的相邻 `../i2rt/.venv/` 仍可回退使用。
明确指定其他位置时：

```bash
export YAM_SDK_DIR=/absolute/path/to/pinned-and-patched/i2rt
export YAM_PYTHON=/absolute/path/to/venv/bin/python
```

`YAM_SDK_DIR`指定的SDK优先；安装阶段的环境目录可通过`YAM_VENV_DIR`设置。
自定义安装目录后，运行时也需设置对应的上述两个变量。
以下运行命令均在仓库根目录、激活所需设备配置的同一终端执行。

## 2. 设备映射、权限与零位（操作员执行）

### 两台D405

```bash
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id/ /dev/v4l/by-path/
lsusb -t
# 对候选节点逐个检查；应支持YUYV或MJPG的640x480、30fps彩色流。
v4l2-ctl -d /dev/videoX --list-formats-ext
```

不要依据video编号或固定index猜左右/彩色节点；逐台接入识别序列号与对应路径，
再接好两台。优先用by-id，若重复/不可用再用by-path。新USB3链路应在`lsusb -t`
显示5000M或更高；480M仍是USB2。实际是否稳定以采集日志为准。

```bash
cp config/yam-local.env.example config/yam-local.env
# 编辑文件，填入确认过的左右彩色节点；该本机配置被Git忽略。
nano config/yam-local.env
source config/yam-local.env
```

`YAM_CAMERA_LEFT`和`YAM_CAMERA_RIGHT`覆盖设备路径，不改变30Hz、640×480或帧引用规则。
未设置时保留旧实验室的by-path默认值，新电脑必须完成上面的映射。
`.npz`相机文件只提供模式设置，里面的旧设备路径不覆盖显式选择；不是手眼/外参标定。
用户需有video组权限：`sudo usermod -aG video "$USER"`，之后退出桌面会话并重新登录。
回放使用OpenCV窗口，需在有显示器的Linux桌面终端执行；SSH无显示环境只能先做离线检查。

原实验环境允许video组访问CPU唤醒延迟设备。若相机日志提示无法限制CPU延迟，可设置：

```bash
echo 'KERNEL=="cpu_dma_latency", GROUP="video", MODE="0660"' | sudo tee /etc/udev/rules.d/99-yam-latency.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=misc --sysname-match=cpu_dma_latency
```

### 两个USB-CAN接口

两臂使用独立CAN接口。先逐个连接USB-CAN适配器，确认它连接的是哪只机械臂；控制程序必须退出。

```bash
ip -br link
# 替换can0为刚识别到的接口；读取USB序列号，不发送电机命令。
udevadm info --attribute-walk --path=/sys/class/net/can0
```

创建 `/etc/udev/rules.d/90-yam-can.rules`，用**实际USB适配器序列号**替换占位值：

```text
SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="LEFT_ADAPTER_SERIAL", NAME="can_left"
SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="RIGHT_ADAPTER_SERIAL", NAME="can_right"
```

重载规则后，在无控制程序且机械臂已托稳/关闭电机电源的情况下重插USB-CAN；随后配置：

```bash
sudo udevadm control --reload-rules
# 重插适配器后确认名称，再执行；这里不发送运动或标零命令。
ip -br link
sudo ip link set can_left down
sudo ip link set can_left type can bitrate 1000000
sudo ip link set can_left up
sudo ip link set can_right down
sudo ip link set can_right type can bitrate 1000000
sudo ip link set can_right up
ip -details -statistics link show can_left
ip -details -statistics link show can_right
```

名称绑定跟随适配器，不跟随机械臂；如果左右电机线互换，还需重新核对。重启后接口可能需重新up。

### 型号和零位

本交付针对YAM Ultra 2 + linear_4310。状态顺序是左J1–J6、左夹爪、右J1–J6、右夹爪；
关节rad，夹爪归一化0闭合/1打开。

换电脑控制同一套机械臂无需重新标零。电机固件里的零位不是Python环境的一部分；
换成另一套机械臂时，要先按厂家物理参考姿态确认自己的零位，不能复制实验室的角度读数作为偏移。
SDK官方ping脚本包含使能/禁用操作，不能当成纯只读。教程和安装器不自动标零。

曾出现的右臂重力补偿差异在修正零位后明显改善，现阶段大幅前伸仍有不足。
先检查参考姿态与读数，再进行[重力补偿测试](yam_gravity_tests.md)；不要先盲增力矩倍率。

## 3. 相机检查（操作员执行）

```bash
bash scripts/collect_yam.sh --camera-only --duration 60
```

不创建机器人；通过同一录制链路写视频、状态样本和精确帧引用。
入口默认帧龄80ms，跨相机时间差默认50ms；设置的分辨率与采样频率不等于实测结果。
当前旧电脑流程测试已明确允许临时使用：

```bash
CAMERA_MAX_AGE_MS=150 CAMERA_MAX_SKEW_MS=100 \
  bash scripts/collect_yam.sh --camera-only --duration 60
```

未关闭缺帧、时间戳、心跳和完整性检查。图像可能落后状态最多150ms，
该配置的成功不能替代80/50ms稳定性验收。

## 4. 完整双臂手动示教与回放（操作员执行）

```bash
CAMERA_MAX_AGE_MS=150 CAMERA_MAX_SKEW_MS=100 \
  bash scripts/collect_yam.sh --duration 60
```

1. 托稳双臂、留出夹爪启动标定空间，输入 `YAM_CAPTURE_READY`。
2. 重力补偿启动后，手动摆到任务起点，按 `1` 开始。
3. 操作双臂并手动开合夹爪；60秒自动停止，也可按 `1` 提前停止。
4. 停止后双臂保持。复位场景、让开运动路径，再按 `r` 准备回放。
5. **按r会主动移动双臂及夹爪到录制首帧**，并非沿刚才轨迹倒退，也没有碰撞规划。
6. 到首帧后按 `1` 正式回放，`h` 取消；回放中按 `1` 停止进入阻尼。
7. 返回选择菜单后按 `1` 保存，`h` 丢弃，`r` 再次准备回放。
8. 保存/丢弃后返回本次程序启动姿态，再撤去力矩；请托稳。

默认不使用按键控制示教夹爪；`--pick-place`是另外的按键夹爪模式，本流程不要加。
手动开闭费力时，可先用[单臂摩擦助力调参工具](yam_gripper_friction.md)验证手感，
再显式传入`--gripper-friction-config`启用；不带该选项仍保持原行为。
回放时夹爪由记录动作主动控制。异常退出沿用重力支撑/阻尼：按h回启动姿态，
Ctrl+C原地禁用，失去力矩前需托稳。不要与其他控制同一CAN的程序并行运行。

## 5. 数据、保存状态与离线分析

路径统一由 `src/config/paths.py` 管理，默认根目录下的 `data/`、`logs/`：

- 原始示教：`data/yam_bimanual_30hz/data/<session>/`。
- 保存发布：`data/yam_bimanual_30hz/episode_XXXXX.zarr/`。
- 采集日志/实际频率：`logs/yam_capture/<session>/`。
- 回放目标和实测状态：`logs/replay/<date>/<run>/steps_*.npz`。

停止录制时原始数据已经在盘上；**按1保存并正常退出后才发布episode**。
发布目录引用原始视频和状态，不能只复制小的episode目录而丢弃原始data目录。
迁移后还需检查原始manifest的绝对输出路径及数据读取，不能假定原路径总有效。
当前入口不单独录制回放过程的视频；示教视频和回放关节日志是两类证据。

看到`Published:`才表示已发布；按菜单丢弃不应恢复成成功示教。
用终端打印的实际episode路径进行离线读取验收：

```bash
.venv-yam/bin/python scripts/inspect_yam_episode.py \
  data/yam_bimanual_30hz/episode_00000.zarr
```

将`episode_00000`换为本次实际编号。输出应包含(N,14)、同一行qpos动作标签、
双路首/中/末640×480图像读取和实测采样率。
满60秒且未提前停止的采集目标为1800条、约30Hz；查看会话日志`rate.json`与原始`manifest.json`，
确认complete/retained/alignable和publication.complete，没有fatal_error，视频没有丢帧。
提前手动停止时条数更少是正常的，不能单凭程序退出判为采集成功。

第一次流程成功后，在同一时序配置下重复3轮完整任务，逐轮记录帧龄、偏斜、丢帧及采样率；
这是现场复现记录，不替代任务精度或长期运行验收。超时后不要将失败原始数据当成通过样本。

复用原生 `data_processing.episode_data.Episode` 读取，不能把发布目录当成内含
所有数组的普通Zarr。最新成功数据已用该读取器检查，但尚未完成训练DataLoader/训练一步测试。

统计一段回放（只读，不连接设备）：

```bash
.venv-yam/bin/python scripts/analyze_yam_replay.py \
  logs/replay/2026-11-25/151327_20261125-151215-11545b/steps_000.npz
```

一个文件是一段连续回放，不能把重新开始的不同段拼成连续轨迹。
关节误差用度，夹爪用归一化开度的百分点；同一tick发送目标后立即读取状态，
包含运动滞后，不是静态零位或末端毫米精度。没有做时间平移来美化误差。

安装器已包含绘图依赖；为分析命令增加
`--plot /absolute/path/to/tracking.png`。这只生成离线图，不启动回放。

## 6. 下一阶段验收

| 项目 | 当前状态/下一步 |
| --- | --- |
| 双臂抓放录制、读取、回放 | 已通过一次完整现场任务，见验证报告 |
| 连续多轮和更长运行 | 待新主机完整负载测试，记录失败次数和失败原因 |
| USB3迁移 | 核实相机实际5000M或更高连接，重新映射左右与CAN |
| 80/50ms时序 | 新机恢复默认后复测；不把一次较低延迟当稳定保证 |
| 动态跟踪精度 | 已生成误差统计，尚未约定任务允许误差；不能凭回放完成判精度合格 |
| 训练与自主操作 | 待YAM任务/模型配置、DataLoader和训练/部署验证 |

迁移保留本机设备配置、原始数据/日志和既有电机零位记录；SDK及依赖由固定版本安装器重建，
不要复制旧Python虚拟环境目录。相机带载状态变动或零位再次修改后需重新验证，旧轨迹不能盲目复用。

# YAM Ultra 2 双臂：安装、数采、回放与迁移

## 当前交付范围

使用i2rt SDK控制两只YAM Ultra 2 + linear_4310，手动引导双臂与夹爪，
采集两台腕部D405的640×480 RGB和14维状态/动作，目标30 Hz，保存后可回放。
未启用深度流；没有第三台俯视相机。训练和自主策略部署仍待接入验证。

[验证报告](validation/2026-11-25-yam.md)区分流程成功、时序限制和动态跟踪误差。
现有旧电脑使用USB2连接；新主机及USB3链路尚未现场验证。

## 1. 环境与依赖

已验证：Ubuntu 24.04、Python 3.11.15、i2rt 1.1.2、NumPy 2.2.6、
OpenCV 4.11.0.86、MuJoCo 4.0.1、python-can 4.5.0。
采集附加依赖固定在 `scripts/requirements-yam-capture.txt`。
YAM轻量运行环境与上游ARX的Python 3.10/训练环境分开，不需要为数采安装ARX SDK。

推荐目录：

```text
workspace/
├── i2rt/                 # 官方SDK，含 .venv（Python 3.11）
└── flash-aloha_YAM/       # 本仓库
```

本次review版本在 `review/yam-collection-20261125` 分支，尚未合入main。
新工作目录可以这样获取源码（已有仓库无需重复克隆）：

```bash
git clone https://github.com/i2rt-robotics/i2rt.git
git clone --branch review/yam-collection-20261125 https://github.com/Hzzzzs/flash-aloha_YAM.git
cd flash-aloha_YAM
```

先准备官方 [i2rt SDK](https://github.com/i2rt-robotics/i2rt) 和系统工具
`ffmpeg`、`patch`、`can-utils`、`v4l-utils`。从本仓库根目录安装采集依赖：

```bash
# SDK 尚未建环境时执行；已有工作环境不要重新创建。
uv venv ../i2rt/.venv --python 3.11
uv pip install --python ../i2rt/.venv/bin/python -e ../i2rt

uv pip install --python ../i2rt/.venv/bin/python -r scripts/requirements-yam-capture.txt
../i2rt/.venv/bin/python scripts/apply_yam_robologger_patch.py
```

本地验证过的SDK版本是1.1.2；迁移应保留当前SDK源码及修改，
不要把任意更新的官方main自动视为同一已验证版本。
上述SDK克隆获取官方当前源码；完整的干净机器安装复现尚未验收。
补丁给robologger 0.1.7增加可配置的编码预设/线程数，不改变默认行为；
本仓库libx264路径使用ultrafast、1编码线程、1颜色转换线程、zerolatency、5Mbps。
已应用补丁时脚本不会重复修改；重新安装robologger后须重新应用。
更快编码可能影响同码率画质，尚未做训练图像质量验收。

启动器默认使用相邻SDK的Python。其他位置可设置：

```bash
export YAM_SDK_DIR=/absolute/path/to/i2rt
export YAM_PYTHON=/absolute/path/to/venv/bin/python
bash scripts/collect_yam.sh --help
```

## 2. 设备映射与零位

| 项目 | 本机已验证配置 |
| --- | --- |
| 左/右臂 | `can_left` / `can_right`，独立CAN接口，1 Mbit/s |
| 型号 | `yam_ultra_2` + `linear_4310` |
| 左相机 | `pci-0000:00:14.0-usb-0:5.3:1.0-video-index4` |
| 右相机 | `pci-0000:00:14.0-usb-0:5.4:1.0-video-index4` |
| 数组顺序 | 左J1–J6、左夹爪、右J1–J6、右夹爪 |
| 单位 | 关节rad；夹爪归一化0闭合/1打开 |

相机路径位于 `scripts/record_yam.py` 的 `DEVICES`；换电脑/USB端口会变化。
当前仍需现场确定左右身份并更新这两个路径，尚未实现自动迁移。
`calibration/d405_640x480_30.npz`只提供模式设置，不是相机外参或手眼标定文件。
CAN持久命名规则需在新机安装并核对序列号，不能假定系统的can0就是左臂。

曾出现的右臂大幅重力补偿差异与零位偏差有关，修正后明显改善。
经验是先检查物理参考姿态与关节读数，不要盲目增加力矩倍率。
左右外观一致不证明零位一致；当前姿态也不应自动标成全零。
现阶段右臂大幅前伸自由补偿仍有不足，已录制轨迹的主动回放不等于自由补偿验收。

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

复用原生 `data_processing.episode_data.Episode` 读取，不能把发布目录当成内含
所有数组的普通Zarr。最新成功数据已用该读取器检查，但尚未完成训练DataLoader/训练一步测试。

统计一段回放（只读，不连接设备）：

```bash
../i2rt/.venv/bin/python scripts/analyze_yam_replay.py \
  logs/replay/2026-11-25/151327_20261125-151215-11545b/steps_000.npz
```

一个文件是一段连续回放，不能把重新开始的不同段拼成连续轨迹。
关节误差用度，夹爪用归一化开度的百分点；同一tick发送目标后立即读取状态，
包含运动滞后，不是静态零位或末端毫米精度。没有做时间平移来美化误差。

可选曲线：安装 `scripts/requirements-yam-analysis.txt` 后，为分析命令增加
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

迁移保留SDK、配置、依赖补丁、数据/日志和电机零位备份；不要迁移旧Python虚拟环境目录，
应在新机重建。相机带载状态变动或零位再次修改后需重新验证，旧轨迹不能盲目复用。

# 2026-11-25：PR软件复现验证

## 交付版本

- Ubuntu 24.04 x86_64宿主机，uv 0.11.24，Python 3.11.15。
- i2rt官方提交 `ea116fe4079e991958b22f879874402674ee6590`，包版本1.1.2。
- 两处SDK退出补丁；修改前后SHA-256见 `scripts/patches/i2rt-source.json`。
- 其余Python依赖固定在 `scripts/requirements-yam-lock.txt`；robologger 0.1.7应用编码选项补丁。

## 实际执行结果

将Git跟踪文件与本次待提交文件复制到独立临时目录，不带原虚拟环境、SDK、数据或日志。
从该目录运行文档中的 `bash scripts/setup_yam.sh` 和 `bash scripts/check_yam.sh`，
只通过UV变量指定已有uv 0.11.24工具；没有设置旧SDK/Python路径。

| 验证项 | 结果 |
| --- | --- |
| 固定提交SDK从GitHub获取 | 成功；仓库内 `.yam-deps/i2rt/` |
| SDK退出补丁应用 | 成功；源码哈希与原实验环境两处修改一致 |
| 新建Python环境及固定依赖安装 | 成功；仓库内 `.venv-yam/`，未复用旧site-packages |
| robologger从干净包安装并应用补丁 | 成功；安装时不使用旧robologger缓存 |
| uv pip check | 85个已安装包依赖兼容 |
| SDK、原生录制和Episode导入 | 通过；未启动控制器 |
| Ultra2 + linear_4310模型加载与FK | 通过；无需显示窗口或实机 |
| FFmpeg libx264、启动器语法/帮助 | 通过 |
| 无硬件测试 | 85 passed, 4 subtests passed |

测试包括YAM后端、RGB采集、回放分析、标准观测、分布式录制、重力保持诊断、
Ctrl+C清理和日志管道退出；均使用模拟状态/设备。

另用新建的独立Python环境，只读验证旧实验室已保存的 `episode_00003.zarr`：
1109条、14维、30.0037428Hz、动作等于同一行qpos；左右图像第0/554/1108帧均可读取。
此原始数据未上传；拿到PR的人可按教程采集自己的episode后运行同一检查脚本。

## 证据边界

这是同一Ubuntu宿主机上的干净目录/虚拟环境验证，不是重装操作系统或更换电脑后的实机验收。
系统apt安装步骤、设备权限、USB/CAN身份、零位、真实录制与回放仍需现场按教程检查。
本轮没有连接/控制机械臂，没有重标零，没有改重力倍率或放宽时序限制。
历史实机证据见 [双臂任务报告](2026-11-25-yam.md)，新主机USB3和严格80/50ms多轮验收仍待完成。

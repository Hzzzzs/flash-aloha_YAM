# YAM重力补偿测试与诊断

先完成[安装、设备映射与零位确认](yam_quickstart.md)。下面均由现场操作员执行，
会使能电机；不要与数采、回放或其他占用同一CAN接口的程序同时运行。
三组Python逻辑沿用实验室原测试代码，仅启动器改为本仓库环境和日志路径。
**没有自动标零，没有把过去试过的增益或固定力矩设成数采默认值。**

## 1. 默认参数与测试范围

固定SDK的 `i2rt/robots/config/yam_ultra_v2.yml` 定义：

| 参数 | J1–J6值 | 含义 |
| --- | --- | --- |
| gravity_comp_factor | 1.0, 1.2, 1.1, 1.1, 1.2, 1.0 | 按当前姿态计算的各轴模型重力力矩倍率 |
| grav_comp_kd | 0.1, 0.1, 0.1, 0.3, 0.05, 0.05 | 自由补偿模式的速度阻尼 |

左右使用同一模型/默认值。改变姿态后，模型重力力矩随姿态重新计算，倍率乘在这个变化值上；
不是始终输出同一个力矩。相机、支架和线缆质量尚未加入默认模型。
电机零位/安装姿态、线缆受力或硬件个体差异仍会影响结果。

这三个诊断入口关闭夹爪自动标定，夹爪增益为零；正常数采入口会标定夹爪，两者不要混淆。
诊断结束/异常退出会撤去力矩，必须托稳。短测的PASS表示指定窗口未触发守卫，不能当全工作空间验收。

## 2. 单臂短测

只开被测臂电源，另一臂关闭。确认底座固定、急停可用、下方有支撑。
从仓库根目录运行，并按终端提示输入确认词：

```bash
bash scripts/08_gravity_comp_default.sh can_right 3
# 完成并检查上一轮后，可做10秒；左臂将can_right换为can_left。
bash scripts/08_gravity_comp_default.sh can_right 10
```

先保持当前姿态约1秒，再进入自由重力补偿。漂移/速度/温度/力矩超限将退出。
日志：`logs/gravity-comp-default/<timestamp>-<channel>/gravity-comp.log`，另有前后CAN统计。

## 3. 三姿态自由补偿（之前左右臂对照所用测试）

```bash
bash scripts/10_gravity_comp_static_eval.sh can_right 5 3
# 右臂退出并托稳/断电后，再按相同流程测左臂。
bash scripts/10_gravity_comp_static_eval.sh can_left 5 3
```

参数依次是接口、每姿态秒数、姿态数量；这里不传倍率，使用SDK默认值。
依次选弯肘、中等前伸、较大前伸等姿态，不要求两臂角度严格一致。

1. 手动摆到被测姿态，**托稳后按Enter**。
2. 三秒倒计时期间继续托住；看到`Recording now`后才逐渐减少支撑。
3. 如有漂移立刻接住，不要为了等满时间任其下落。记录这次是否完全撤去支撑。
4. 到时程序继续保持当前控制模式并记录，不会立即撤去补偿。
5. 看到窗口结束提示，重新托稳、不移动姿态，再按Enter结束该姿态。
6. 最后一组后程序关闭电机，继续托稳。

输出在 `logs/gravity-comp-static-eval/<timestamp>-<channel>/`：

- `samples.csv`：角度、速度、模型前馈、反馈力矩估计和模型计算的末端漂移。
- `metrics.json`：各姿态起点、关节峰值/末值漂移、末端漂移、参数与模式。
- `evaluation.log`、`can-before.txt`、`can-after.txt`：终端与CAN状态。

`initial_acceptance`只按记录中的末端20mm/关节2°阈值判断；是否有手支撑需现场说明。
末端漂移由模型FK推算，反馈力矩由电机报告，均不是独立外部仪器测量。

## 4. 诊断与临时参数实验（可选）

以下是此前排查右臂的工具，不是完成数采的必需步骤，也不会写回SDK配置。

```bash
# 当前位置PD保持，估计自由补偿之外还需多少保持力矩；一姿态、5秒。
DIAGNOSTIC_HOLD=1 bash scripts/10_gravity_comp_static_eval.sh can_right 5 1

# 自由补偿倍率对照：J2=1.20、J3=1.25（J3默认是1.10）。
bash scripts/10_gravity_comp_static_eval.sh can_right 5 3 1.20 1.25

# 全程托住的诊断，不可将结果算作自由补偿通过。
KEEP_SUPPORTED=1 bash scripts/10_gravity_comp_static_eval.sh can_right 5 1
```

PD保持激活时不要再手动拖动；稳定保持不证明纯重力补偿通过。
`J3_BIAS_NM`是单姿态固定力矩诊断，默认0；仅允许0–0.8Nm、一姿态且不与PD保持组合。
它不是整个工作空间的修复，不建议直接带入数采。
`GUIDE_TARGET_Q`可显示手动引导角度目标，默认空；不要求使用、不自动移动到该角度。

## 5. 双臂同时短测

两臂均上电、均有人/支撑可接住，确认工作空间和夹爪无干涉：

```bash
bash scripts/13_dual_arm_gravity_comp.sh 3
# 前一轮检查后可用10秒。
```

输出在 `logs/dual-arm-gravity-comp/<timestamp>-<duration>s/`。
全部日志路径跟随 `FLASH_ALOHA_LOG_DIR`，与数采的 `config.paths` 保持一致。

## 已知经验

零位偏差曾导致左右表现明显不同；修正后改善，但右臂大幅前伸仍存在补偿不足。
不要从“同型号/同模型”直接得出“必然是电机坏了”，也不要将当前任意姿态标成全零。
若在参考姿态、无手支撑条件下出现漂移，保留日志、参数、负载与姿态说明给厂家核对。
本仓库保留测试工具；历史原始诊断日志未上传。

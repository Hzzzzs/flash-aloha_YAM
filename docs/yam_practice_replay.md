# YAM Ultra 2: camera-free practice and replay

For the current camera-backed collection workflow, start with [the YAM guide](yam_quickstart.md).
From this repository root, after installing i2rt, the operator-run entry points are:

```bash
PYTHONPATH=src:../i2rt ../i2rt/.venv/bin/python scripts/practice.py --robot yam --arm both --duration 10
PYTHONPATH=src:../i2rt ../i2rt/.venv/bin/python src/recording/replay_practice.py --robot yam --arms both --fps 100
```

Both initialize real controllers and calibrate grippers. Read the startup/exit instructions below first.

This integration keeps Flash ALOHA's practice/replay state machine and action
layout: each arm is `[q1..q6, gripper]`, in left-then-right order for two arms.
The gripper coordinate is normalized (`0` closed, `1` open).

The expected local layout is:

```text
YAM驱动数采验证/
├── flash-aloha_YAM/
└── i2rt/
```

The commands below use the already working i2rt virtual environment and do not
load Flash's camera, dataset, training, or policy dependencies.

## 1. Ten-second dual-arm practice check

Support both arms before starting. The linear grippers calibrate during SDK
startup. Both arms then become hand-guided under gravity compensation. At the
end, the SDK closes and the arms become limp, so support them again.

```bash
PYTHONPATH=src:../i2rt ../i2rt/.venv/bin/python scripts/practice.py --robot yam --arm both --duration 10
```

For an open-ended session, omit `--duration`. Ctrl+C enters the safe-exit path:
the arms retain gravity support with zero arm position gain and held grippers;
press `h` to return both arms to their startup poses and close.

## 2. Flash native in-memory record/replay

Clear both workspaces and confirm that the repaired right camera mount and cable are secure.

```bash
PYTHONPATH=src:../i2rt ../i2rt/.venv/bin/python src/recording/replay_practice.py --robot yam --arms both --fps 100
```

Controls:

| State | Key |
|---|---|
| Passive | `3` starts recording; `q` quits |
| Recording | `3` stops |
| Review | `2` prepares replay; `3` replaces the take; `1` releases arms |
| At first recorded pose | `1` starts synchronized replay; `h` cancels |
| During replay | `1` stops into safe mode; then `h` returns to the take start |

The replay tool keeps the take in memory and writes only its diagnostic tracking log under `logs/replay`.
The default YAM gripper force limit is 5 N. A 0.5-second command watchdog moves
the arms into gravity-supported safe stop if the replay loop stalls.

## 3. Full Flash environment

The complete training stack can be installed independently while the lightweight
practice path is used:

```bash
uv sync --extra dev
source .venv/bin/activate
source scripts/setup/paths.sh
python -m pytest
```

This follows `.python-version` (Python 3.10) and installs the policy-training,
dataset, visualization, and test dependencies. It is not required for the two
YAM commands above.

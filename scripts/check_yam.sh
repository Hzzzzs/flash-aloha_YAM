#!/usr/bin/env bash
# Offline checks only. Safe to run without any robot or camera connected.
set -Eeuo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/yam_env.sh"
cd "$REPO_DIR"
"$PYTHON_BIN" scripts/check_yam_environment.py
bash -n scripts/collect_yam.sh scripts/setup_yam.sh scripts/yam_env.sh \
  scripts/08_gravity_comp_default.sh scripts/10_gravity_comp_static_eval.sh \
  scripts/13_dual_arm_gravity_comp.sh
bash -n scripts/tune_yam_gripper.sh
bash scripts/collect_yam.sh --help
bash scripts/tune_yam_gripper.sh --help
"$PYTHON_BIN" -m pytest -q \
  scripts/tests/test_yam_backend.py scripts/tests/test_yam_rgb_collection.py \
  scripts/tests/test_yam_replay_analysis.py scripts/tests/test_standard_capture.py \
  scripts/tests/test_distributed_recording.py \
  scripts/tests/test_gravity_diagnostic_hold.py scripts/tests/test_gravity_eval_shutdown.py \
  scripts/tests/test_yam_gripper_friction.py

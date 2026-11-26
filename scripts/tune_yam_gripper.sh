#!/usr/bin/env bash
# Operator-run. --help does not open any hardware.
set -Eeuo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/yam_env.sh"
exec "$PYTHON_BIN" -u "$REPO_DIR/scripts/tune_yam_gripper.py" "$@"

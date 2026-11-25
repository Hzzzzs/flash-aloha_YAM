#!/usr/bin/env bash
# Operator-run entry; --help performs no hardware access.
set -Eeuo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/yam_env.sh"
export FLASH_ALOHA_CAMERA_PARAMS="$REPO_DIR/calibration/d405_640x480_30.npz"
export CAMERA_MAX_AGE_MS="${CAMERA_MAX_AGE_MS:-80}"
exec "$PYTHON_BIN" -u "$REPO_DIR/scripts/record_yam.py" "$@"

#!/usr/bin/env bash
# Operator-run entry; --help performs no hardware access.
set -Eeuo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_DIR="${YAM_SDK_DIR:-$(dirname "$REPO_DIR")/i2rt}"
PYTHON_BIN="${YAM_PYTHON:-$SDK_DIR/.venv/bin/python}"
export PYTHONPATH="$REPO_DIR/src:$SDK_DIR${PYTHONPATH:+:$PYTHONPATH}"
export FLASH_ALOHA_CAMERA_PARAMS="$REPO_DIR/calibration/d405_640x480_30.npz"
export CAMERA_MAX_AGE_MS="${CAMERA_MAX_AGE_MS:-80}"
exec "$PYTHON_BIN" -u "$REPO_DIR/scripts/record_yam.py" "$@"

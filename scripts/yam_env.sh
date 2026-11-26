#!/usr/bin/env bash
# Shared path selection only; source from launchers, never connect to hardware.
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -d "$REPO_DIR/.yam-deps/i2rt" ]]; then
  SDK_DEFAULT="$REPO_DIR/.yam-deps/i2rt"
else
  SDK_DEFAULT="$(dirname "$REPO_DIR")/i2rt"
fi
SDK_DIR="${YAM_SDK_DIR:-$SDK_DEFAULT}"
if [[ -x "$REPO_DIR/.venv-yam/bin/python" ]]; then
  PYTHON_DEFAULT="$REPO_DIR/.venv-yam/bin/python"
else
  PYTHON_DEFAULT="$SDK_DIR/.venv/bin/python"
fi
PYTHON_BIN="${YAM_PYTHON:-$PYTHON_DEFAULT}"
export PYTHONPATH="$REPO_DIR/src:$SDK_DIR${PYTHONPATH:+:$PYTHONPATH}"
export I2RT_ROOT="$SDK_DIR"

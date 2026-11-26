#!/usr/bin/env bash
# Installs software only. Never opens cameras/CAN or launches controllers.
set -Eeuo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UV_BIN="${UV:-uv}"
SDK_DIR="${YAM_SDK_DIR:-$REPO_DIR/.yam-deps/i2rt}"
VENV_DIR="${YAM_VENV_DIR:-$REPO_DIR/.venv-yam}"
SDK_REV="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["commit"])' "$REPO_DIR/scripts/patches/i2rt-source.json")"
command -v "$UV_BIN" >/dev/null
for tool in git patch ffmpeg; do command -v "$tool" >/dev/null; done
if [[ ! -e "$SDK_DIR" ]]; then
  mkdir -p "$SDK_DIR"
  git -C "$SDK_DIR" init -q
  git -C "$SDK_DIR" remote add origin https://github.com/i2rt-robotics/i2rt.git
  git -C "$SDK_DIR" fetch --depth 1 origin "$SDK_REV"
  git -C "$SDK_DIR" checkout --detach FETCH_HEAD
fi
python3 "$REPO_DIR/scripts/apply_yam_i2rt_patch.py" "$SDK_DIR"
if [[ ! -x "$VENV_DIR/bin/python" ]]; then
  "$UV_BIN" venv --python 3.11.15 "$VENV_DIR"
fi
"$VENV_DIR/bin/python" -c 'import sys; assert sys.version_info[:3] == (3,11,15), "Use Python 3.11.15 in a separate YAM environment"'
"$UV_BIN" pip sync --link-mode copy --python "$VENV_DIR/bin/python" "$REPO_DIR/scripts/requirements-yam-lock.txt"
"$UV_BIN" pip install --link-mode copy --python "$VENV_DIR/bin/python" --no-deps --no-build-isolation -e "$SDK_DIR"
# Install a pristine copy before patching: older uv hardlink installs may have
# shared a modified source file with the cache. Do not mutate that cache again.
"$UV_BIN" pip install --no-cache --python "$VENV_DIR/bin/python" --no-deps --reinstall robologger==0.1.7
"$VENV_DIR/bin/python" "$REPO_DIR/scripts/apply_yam_robologger_patch.py"
"$UV_BIN" pip check --python "$VENV_DIR/bin/python"
printf 'YAM software installed. Python: %s\nSDK: %s\n' "$VENV_DIR/bin/python" "$SDK_DIR"
printf 'Next: bash scripts/check_yam.sh (no hardware access). See docs/yam_quickstart.md.\n'

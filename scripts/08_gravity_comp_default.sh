#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT_DIR/scripts/yam_env.sh"
LOG_ROOT="$("$PYTHON_BIN" -c 'from config.paths import LOG_DIR; print(LOG_DIR)')"
CHANNEL="${1:-}"
DURATION_S="${2:-3}"

if [[ "$CHANNEL" != "can_left" && "$CHANNEL" != "can_right" ]]; then
    printf 'Usage: %s <can_left|can_right> [3|10]\n' "$0" >&2
    exit 1
fi

if [[ "$DURATION_S" != "3" && "$DURATION_S" != "10" ]]; then
    printf 'Duration must be 3 seconds for the first stage or 10 seconds after log review.\n' >&2
    exit 1
fi

ARM_NAME=left
OTHER_ARM=right
CONFIRM=LEFT_GRAVITY_READY
if [[ "$CHANNEL" == "can_right" ]]; then
    ARM_NAME=right
    OTHER_ARM=left
    CONFIRM=RIGHT_GRAVITY_READY
fi

printf 'Guarded %s-second default-model gravity compensation for the %s arm\n' "$DURATION_S" "$ARM_NAME"
printf '  - %s arm is powered ON; %s arm is powered OFF\n' "$ARM_NAME" "$OTHER_ARM"
printf '  - the arm is physically supported and the emergency stop is reachable\n'
printf '  - model is yam_ultra_2 + linear_4310 with repository default dynamics\n'
printf '  - D405, bracket and cable mass are not modelled\n'
printf '  - gripper auto-calibration is disabled and gripper gains are zero\n'
printf '  - after 1 second of supported stabilisation, gravity compensation runs for %s seconds\n' "$DURATION_S"
printf '  - keep the original support directly below the arm; this is not a free-space test\n'
printf '  - keep supporting the arm when the script closes and the joints become limp\n\n'
read -r -p "Type $CONFIRM to continue: " answer
if [[ "$answer" != "$CONFIRM" ]]; then
    printf 'Cancelled.\n'
    exit 1
fi

STAMP="$(date +%Y%m%d-%H%M%S)"
OUT_DIR="$LOG_ROOT/gravity-comp-default/$STAMP-$CHANNEL"
mkdir -p "$OUT_DIR"

ip -details -statistics link show "$CHANNEL" >"$OUT_DIR/can-before.txt"

set +e
"$PYTHON_BIN" "$ROOT_DIR/scripts/08_gravity_comp_default.py" \
    --channel "$CHANNEL" --duration-s "$DURATION_S" 2>&1 | tee "$OUT_DIR/gravity-comp.log"
gravity_status=${PIPESTATUS[0]}
set -e

ip -details -statistics link show "$CHANNEL" >"$OUT_DIR/can-after.txt"
printf 'Session log: %s\n' "$OUT_DIR"
exit "$gravity_status"

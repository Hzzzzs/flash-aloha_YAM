#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT_DIR/scripts/yam_env.sh"
LOG_ROOT="$("$PYTHON_BIN" -c 'from config.paths import LOG_DIR; print(LOG_DIR)')"
CHANNEL="${1:-}"
DURATION_S="${2:-15}"
POSES="${3:-3}"
J2_GRAVITY_FACTOR="${4:-}"
J3_GRAVITY_FACTOR="${5:-}"
GUIDE_TARGET_Q="${GUIDE_TARGET_Q:-}"
KEEP_SUPPORTED="${KEEP_SUPPORTED:-0}"
DIAGNOSTIC_HOLD="${DIAGNOSTIC_HOLD:-0}"
J3_BIAS_NM="${J3_BIAS_NM:-0}"

if [[ "${CHANNEL}" != "can_left" && "${CHANNEL}" != "can_right" ]]; then
  printf 'Usage: %s <can_left|can_right> [duration_s=15] [poses=3] [j2_gravity_factor] [j3_gravity_factor]\n' "$0" >&2
  exit 1
fi

EXTRA_ARGS=()
EXTRA_ARGS+=(--j3-bias-nm "${J3_BIAS_NM}")
case "${DIAGNOSTIC_HOLD}" in
  1) EXTRA_ARGS+=(--diagnostic-hold) ;;
  0) ;;
  *) printf 'DIAGNOSTIC_HOLD must be 0 or 1\n' >&2; exit 1 ;;
esac
case "${KEEP_SUPPORTED}" in
  1) EXTRA_ARGS+=(--keep-supported) ;;
  0) ;;
  *) printf 'KEEP_SUPPORTED must be 0 or 1\n' >&2; exit 1 ;;
esac
if [[ -n "${J2_GRAVITY_FACTOR}" ]]; then
  EXTRA_ARGS+=(--j2-gravity-factor "${J2_GRAVITY_FACTOR}")
fi
if [[ -n "${J3_GRAVITY_FACTOR}" ]]; then
  EXTRA_ARGS+=(--j3-gravity-factor "${J3_GRAVITY_FACTOR}")
fi
if [[ -n "${GUIDE_TARGET_Q}" ]]; then
  EXTRA_ARGS+=(--guide-target-q "${GUIDE_TARGET_Q}")
fi

ARM_NAME=left
OTHER_ARM=right
CONFIRM=LEFT_STATIC_EVAL_READY
if [[ "${CHANNEL}" == "can_right" ]]; then
  ARM_NAME=right
  OTHER_ARM=left
  CONFIRM=RIGHT_STATIC_EVAL_READY
fi

printf 'Multi-pose static gravity-compensation evaluation: %s arm\n' "${ARM_NAME}"
printf '  - %s arm ON; %s arm OFF\n' "${ARM_NAME}" "${OTHER_ARM}"
printf '  - no camera/CAN test is running in another terminal\n'
printf '  - emergency stop is reachable; a support remains directly below the arm\n'
printf '  - test %s poses for %s seconds each\n' "${POSES}" "${DURATION_S}"
if [[ "${KEEP_SUPPORTED}" == 1 ]]; then
  printf '  - SUPPORTED DIAGNOSTIC: support throughout; do not release the arm\n'
fi
if [[ "${DIAGNOSTIC_HOLD}" == 1 ]]; then
  printf '  - DIAGNOSTIC HOLD: after hand-guiding and countdown, hold the measured current pose\n'
  printf '  - do not hand-guide while hold is active; gradually reduce support only when prompted\n'
  printf '  - position hold measures extra effort; it does NOT establish a gravity-compensation pass\n'
fi
if [[ "${J3_BIAS_NM}" != 0 ]]; then
  printf '  - LOCAL J3 BIAS TEST: +%s Nm after the supported countdown; stay near that pose\n' "${J3_BIAS_NM}"
  printf '  - this temporary feedforward is cleared on shutdown; it is not a workspace-wide correction\n'
fi
if [[ -n "${J2_GRAVITY_FACTOR}" ]]; then
  printf '  - temporary J2 gravity factor: %s (repository default remains unchanged)\n' "${J2_GRAVITY_FACTOR}"
fi
if [[ -n "${J3_GRAVITY_FACTOR}" ]]; then
  printf '  - temporary J3 gravity factor: %s (repository default remains unchanged)\n' "${J3_GRAVITY_FACTOR}"
fi
if [[ -n "${GUIDE_TARGET_Q}" ]]; then
  printf '  - live hand-guide target q (rad): %s\n' "${GUIDE_TARGET_Q}"
fi
printf '  - suggested poses: elbow-bent, mid-reach, forward/side reach\n'
printf '  - avoid joint limits, singular straight-arm poses, the table, and the other arm\n'
printf '  - D405/bracket/cable mass remains outside the default model\n\n'
read -r -p "Type ${CONFIRM} to continue: " answer
if [[ "${answer}" != "${CONFIRM}" ]]; then
  printf 'Cancelled.\n'
  exit 1
fi

STAMP="$(date +%Y%m%d-%H%M%S)"
OUT_DIR="${LOG_ROOT}/gravity-comp-static-eval/${STAMP}-${CHANNEL}"
mkdir -p "${OUT_DIR}"
ip -details -statistics link show "${CHANNEL}" >"${OUT_DIR}/can-before.txt"

set +e
# The foreground Python process handles Ctrl+C and disables the motors. Keep
# both this wrapper and tee alive until that cleanup and its logging finish.
trap ':' INT
"$PYTHON_BIN" -u "${ROOT_DIR}/scripts/10_gravity_comp_static_eval.py" \
  --channel "${CHANNEL}" \
  --output-dir "${OUT_DIR}" \
  --duration-s "${DURATION_S}" \
  --poses "${POSES}" \
  "${EXTRA_ARGS[@]}" 2>&1 | tee --ignore-interrupts "${OUT_DIR}/evaluation.log"
pipeline_status=("${PIPESTATUS[@]}")
trap - INT
status=${pipeline_status[0]}
if [[ "$status" == 0 ]]; then
  status=${pipeline_status[1]}
fi
set -e

ip -details -statistics link show "${CHANNEL}" >"${OUT_DIR}/can-after.txt"
printf 'Session log: %s\n' "${OUT_DIR}"
exit "${status}"

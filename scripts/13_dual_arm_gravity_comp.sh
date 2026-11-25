#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT_DIR/scripts/yam_env.sh"
LOG_ROOT="$("$PYTHON_BIN" -c 'from config.paths import LOG_DIR; print(LOG_DIR)')"
DURATION_S="${1:-3}"
if [[ "${DURATION_S}" != 3 && "${DURATION_S}" != 10 ]]; then
  echo "Usage: $0 [3|10]" >&2
  exit 1
fi

printf 'Guarded simultaneous dual-arm gravity compensation (%s seconds)\n' "${DURATION_S}"
printf '  - both bases are fixed; emergency stop is reachable\n'
printf '  - both workspaces are clear and the arms cannot collide\n'
printf '  - a support is directly below EACH arm for shutdown/fault handling\n'
printf '  - the repaired right-camera mount and cable are firmly secured\n'
printf '  - no camera, CAN, teleoperation, or other robot process is running\n'
printf '  - default yam_ultra_2 + linear_4310 dynamics are used\n'
printf '  - D405, bracket and cable mass are not modelled\n\n'
read -r -p 'Type DUAL_GRAVITY_READY to continue: ' answer
if [[ "${answer}" != "DUAL_GRAVITY_READY" ]]; then
  echo "Cancelled."
  exit 1
fi

STAMP="$(date +%Y%m%d-%H%M%S)"
OUT_DIR="${LOG_ROOT}/dual-arm-gravity-comp/${STAMP}-${DURATION_S}s"
mkdir -p "${OUT_DIR}"
for channel in can_left can_right; do
  ip -details -statistics link show "${channel}" >"${OUT_DIR}/${channel}-before.txt"
done

set +e
"$PYTHON_BIN" "${ROOT_DIR}/scripts/13_dual_arm_gravity_comp.py" \
  --output-dir "${OUT_DIR}" --duration-s "${DURATION_S}" 2>&1 | tee "${OUT_DIR}/gravity-comp.log"
status=${PIPESTATUS[0]}
set -e

for channel in can_left can_right; do
  ip -details -statistics link show "${channel}" >"${OUT_DIR}/${channel}-after.txt"
done
echo "Session log: ${OUT_DIR}"
exit "${status}"

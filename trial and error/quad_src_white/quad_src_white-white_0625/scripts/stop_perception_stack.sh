#!/usr/bin/env bash
# Stop perception stack tmux sessions and kill leftover ROS nodes.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/lib/perception_cleanup.sh"

TMUX_SESSION="${TMUX_SESSION:-perception_stack}"
OCR_SESSION="${OCR_SESSION:-ocr_sys}"
USB_SETTLE_SEC="${USB_SETTLE_SEC:-3}"
KILL_LIDAR="${KILL_LIDAR:-0}"

if [[ "${FORCE_KILL}" == "1" || "${1:-}" == "--force" ]]; then
    KILL_LIDAR=1
fi

if [[ "${1:-}" == "--force" ]]; then
    shift
fi

echo "[info] Stopping perception stack (tmux=${TMUX_SESSION}, ocr=${OCR_SESSION}) ..."
perception_stop_all "${KILL_LIDAR}" "${USB_SETTLE_SEC}" "${TMUX_SESSION}" "${OCR_SESSION}"
echo "[info] Done."

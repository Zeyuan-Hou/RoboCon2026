#!/usr/bin/env bash
# Shared kill/verify helpers for perception stack scripts.

perception_cleanup_log() {
    echo "[cleanup] $*"
}

# Kill tmux sessions used by perception + OCR stacks.
perception_kill_tmux_sessions() {
    local tmux_session="${1:-perception_stack}"
    local ocr_session="${2:-ocr_sys}"

    if tmux has-session -t "${tmux_session}" 2>/dev/null; then
        perception_cleanup_log "Killing tmux session: ${tmux_session}"
        tmux kill-session -t "${tmux_session}"
    else
        perception_cleanup_log "No tmux session: ${tmux_session}"
    fi

    if tmux has-session -t "${ocr_session}" 2>/dev/null; then
        perception_cleanup_log "Killing tmux session: ${ocr_session}"
        tmux kill-session -t "${ocr_session}"
    else
        perception_cleanup_log "No tmux session: ${ocr_session}"
    fi
}

# Kill ROS/perception processes that survive tmux kill-session (orphans).
# Args: kill_start_script (0|1) — set 0 when called from start_perception_stack.sh
perception_kill_orphan_processes() {
    local kill_start_script="${1:-1}"

    perception_cleanup_log "Killing perception-related orphan processes ..."

    local patterns=(
        "ros2 run realsense realsense_node"
        "realsense/lib/realsense/realsense_node"
        "ros2 run yolov8_rknn yolov8_node"
        "ros2 run yolov8_rknn eightboxes_node"
        "nodes/ppocr_node.py"
        "nodes/ocr_result_subscriber_node.py"
        "nodes/usbcam_node.py"
        "logs/perception/realsense.log"
        "logs/perception/yolov8.log"
        "logs/perception/eightboxes.log"
        "logs/perception/ppocr.log"
        "logs/perception/ocr_result.log"
    )

    local pattern
    for pattern in "${patterns[@]}"; do
        pkill -f "${pattern}" 2>/dev/null || true
    done

    pkill -f "realsense_node" 2>/dev/null || true
    pkill -f "yolov8_node" 2>/dev/null || true
    pkill -f "eightboxes_node" 2>/dev/null || true
    pkill -f "ppocr_node.py" 2>/dev/null || true
    pkill -f "ocr_result_subscriber_node.py" 2>/dev/null || true
    pkill -f "usbcam_node.py" 2>/dev/null || true

    if [[ "${kill_start_script}" == "1" ]]; then
        # Kill stale foreground wait scripts (e.g. Ctrl+C left tmux orphans but script still waiting).
        pkill -f "scripts/start_perception_stack.sh" 2>/dev/null || true
    fi
}

perception_kill_lidar_launch() {
    perception_cleanup_log "Killing point_lio launch (mapping_mid360) ..."
    pkill -f "mapping_mid360.launch.py" 2>/dev/null || true
}

# Return 0 if no known perception processes remain.
perception_verify_stopped() {
    local leftovers
    leftovers="$(pgrep -af \
        'realsense_node|yolov8_node|eightboxes_node|ppocr_node|ocr_result_subscriber|usbcam_node|start_perception_stack\.sh' \
        2>/dev/null | grep -v "pgrep -af" || true)"

    if [[ -n "${leftovers}" ]]; then
        perception_cleanup_log "WARNING: leftover processes still running:"
        echo "${leftovers}" | sed 's/^/  /'
        return 1
    fi

    perception_cleanup_log "No perception orphan processes detected."
    return 0
}

# Full cleanup: tmux + orphans + optional lidar + USB settle + verify.
perception_stop_all() {
    local kill_lidar="${1:-0}"
    local usb_settle_sec="${2:-3}"
    local tmux_session="${3:-perception_stack}"
    local ocr_session="${4:-ocr_sys}"
    local kill_start_script="${5:-1}"
    local verify="${6:-1}"

    perception_kill_tmux_sessions "${tmux_session}" "${ocr_session}"
    sleep 0.5
    perception_kill_orphan_processes "${kill_start_script}"
    if [[ "${kill_lidar}" == "1" ]]; then
        perception_kill_lidar_launch
    fi
    if (( usb_settle_sec > 0 )); then
        perception_cleanup_log "Waiting ${usb_settle_sec}s for RealSense USB release ..."
        sleep "${usb_settle_sec}"
    fi
    if [[ "${verify}" == "1" ]]; then
        perception_verify_stopped || true
    fi
}

#!/usr/bin/env bash
# Start task-race perception stack in tmux (camera + OCR + YOLO + eightboxes).
# Note: avoid set -u; ROS setup.bash references unset AMENT_* variables.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HITCRT_WS="$(cd "${SCRIPT_DIR}/.." && pwd)"
LIB_DIR="${SCRIPT_DIR}/lib"
# shellcheck source=/dev/null
source "${LIB_DIR}/wait_ros_ready.sh"
# shellcheck source=/dev/null
source "${LIB_DIR}/start_ocr_stack.sh"
# shellcheck source=/dev/null
source "${LIB_DIR}/perception_cleanup.sh"

# ---------- configurable paths ----------
REALSENSE_NEW_WS_DEFAULT="/home/cat/realsense_new"
if [[ ! -d "${REALSENSE_NEW_WS_DEFAULT}" && -d "/home/cat/realsense新" ]]; then
    REALSENSE_NEW_WS_DEFAULT="/home/cat/realsense新"
fi
REALSENSE_WS="${REALSENSE_WS:-${REALSENSE_NEW_WS_DEFAULT}}"
ROS2_WS="${ROS2_WS:-/home/cat/ros2_ws}"
PADDLEOCR_WS="${PADDLEOCR_WS:-/home/cat/paddleocrDeploy_rk3588}"

TMUX_SESSION="${TMUX_SESSION:-perception_stack}"
WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-120}"
REALSENSE_FPS="${REALSENSE_FPS:-15}"
REALSENSE_EXPOSURE="${REALSENSE_EXPOSURE:-230}"
USB_SETTLE_SEC="${USB_SETTLE_SEC:-3}"
YOLO_WARMUP_SEC="${YOLO_WARMUP_SEC:-5}"
START_OCR="${START_OCR:-1}"
START_OCR_USBCAM="${START_OCR_USBCAM:-0}"
OCR_SESSION="${OCR_SESSION:-ocr_sys}"

TF_PARENT="${TF_PARENT:-camera_init}"
TF_CHILD="${TF_CHILD:-aft_mapped}"

LOG_DIR="${HITCRT_WS}/logs/perception"
mkdir -p "${LOG_DIR}"

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

# ---------- helpers ----------
die() {
    echo "[error] $*" >&2
    exit 1
}

require_cmd() {
    command -v "$1" >/dev/null 2>&1 || die "Missing command: $1"
}

safe_source() {
    local setup_file="$1"
    [[ -f "${setup_file}" ]] || die "setup file not found: ${setup_file}"
    set +u
    # shellcheck source=/dev/null
    source "${setup_file}"
    set -e
}

setup_ros_env() {
    safe_source "${ROS_SETUP}"
    export ROS_LOG_DIR="${ROS_LOG_DIR:-${HITCRT_WS}/logs/ros}"
    mkdir -p "${ROS_LOG_DIR}"
}

check_workspace() {
    local ws="$1"
    local name="$2"
    [[ -d "${ws}" ]] || die "${name} workspace missing: ${ws}"
    [[ -f "${ws}/install/setup.bash" ]] || die "${name} not built. Run: cd ${ws} && colcon build"
}

cleanup_before_start() {
    echo "[info] Cleaning up previous perception stack (if any) ..."
    # kill_start_script=0, verify=0: do not kill/flag this running start script.
    perception_stop_all 0 "${USB_SETTLE_SEC}" "${TMUX_SESSION}" "${OCR_SESSION}" 0 0
}

# ---------- preflight ----------
require_cmd tmux
require_cmd ros2
require_cmd timeout

setup_ros_env

check_workspace "${REALSENSE_WS}" "realsense"
check_workspace "${ROS2_WS}" "ros2_ws"

if [[ "${START_OCR}" == "1" ]]; then
    [[ -d "${PADDLEOCR_WS}" ]] || die "PaddleOCR workspace missing: ${PADDLEOCR_WS}"
    require_cmd uv
fi

cleanup_before_start

echo "[info] This script does not start lidar/point_lio."
echo "[info] eightboxes will start only after /tf and ${TF_PARENT}->${TF_CHILD} are ready."
echo ""

ROS_ENV="source ${ROS_SETUP}"
RS_ENV="${ROS_ENV} && source ${REALSENSE_WS}/install/setup.bash"
YOLO_ENV="${ROS_ENV} && source ${ROS2_WS}/install/setup.bash"

RS_CMD="${RS_ENV} && ros2 run realsense realsense_node --fps ${REALSENSE_FPS} --exposure ${REALSENSE_EXPOSURE}"
YOLO_CMD="${YOLO_ENV} && ros2 run yolov8_rknn yolov8_node"
BOX_CMD="${YOLO_ENV} && ros2 run yolov8_rknn eightboxes_node"

echo "[info] Creating tmux session: ${TMUX_SESSION}"
tmux new-session -d -s "${TMUX_SESSION}" -n perception -c "${HITCRT_WS}"

# 3 panes: realsense | yolov8 | eightboxes
tmux split-window -h -t "${TMUX_SESSION}:0.0"
tmux split-window -h -t "${TMUX_SESSION}:0.0"
tmux select-layout -t "${TMUX_SESSION}:0" even-horizontal

# Pane 0: realsense
tmux select-pane -t "${TMUX_SESSION}:0.0" -T realsense
tmux send-keys -t "${TMUX_SESSION}:0.0" \
    "${RS_CMD} 2>&1 | tee -a ${LOG_DIR}/realsense.log" C-m

if [[ "${START_OCR}" == "1" ]]; then
    echo "[info] Step OCR: 启动智力题 PaddleOCR（与相机并行，tmux: ${OCR_SESSION}）..."
    start_paddleocr_stack "${PADDLEOCR_WS}" "${LOG_DIR}" || \
        echo "[warn] OCR 启动失败，可手动: cd ${PADDLEOCR_WS} && ./start_ocr_system.sh"
else
    echo "[info] START_OCR=0，跳过 OCR"
fi

echo "[info] Step 1/3: waiting for /color/image_raw ..."
wait_for_topic "/color/image_raw" "${WAIT_TIMEOUT_SEC}"
wait_for_topic_message "/color/image_raw" "${WAIT_TIMEOUT_SEC}"

# Pane 1: yolov8
tmux select-pane -t "${TMUX_SESSION}:0.1" -T yolov8
tmux send-keys -t "${TMUX_SESSION}:0.1" \
    "${YOLO_CMD} 2>&1 | tee -a ${LOG_DIR}/yolov8.log" C-m

echo "[info] Step 2/3: YOLOv8 warmup ${YOLO_WARMUP_SEC}s ..."
sleep "${YOLO_WARMUP_SEC}"
wait_for_topic "/target_3d_position" 30 || echo "[warn] /target_3d_position not visible yet (YOLO may still be loading)"

echo "[info] Step 3/3: waiting for lidar TF ${TF_PARENT} -> ${TF_CHILD} ..."
wait_for_topic "/tf" "${WAIT_TIMEOUT_SEC}" || die "/tf not visible; start lidar/point_lio before eightboxes"
if [[ "${SKIP_TF_WAIT:-0}" == "1" ]]; then
    echo "[warn] SKIP_TF_WAIT=1: starting eightboxes without TF check"
else
    wait_for_tf "${TF_PARENT}" "${TF_CHILD}" "${WAIT_TIMEOUT_SEC}" || {
        die "TF ${TF_PARENT}->${TF_CHILD} not ready; start lidar/point_lio before eightboxes"
    }
fi

# Pane 2: eightboxes
tmux select-pane -t "${TMUX_SESSION}:0.2" -T eightboxes
tmux send-keys -t "${TMUX_SESSION}:0.2" \
    "${BOX_CMD} 2>&1 | tee -a ${LOG_DIR}/eightboxes.log" C-m

wait_for_topic "/eightboxes" 60 || echo "[warn] /eightboxes not published yet (need 8 regions detected)"

tmux select-pane -t "${TMUX_SESSION}:0.0"

echo ""
echo "=========================================="
echo " Perception stack started (no lidar in script)"
echo "=========================================="
echo "  tmux attach -t ${TMUX_SESSION}    # RealSense + YOLO + eightboxes"
if [[ "${START_OCR}" == "1" ]]; then
    echo "  tmux attach -t ${OCR_SESSION}       # OCR: ppocr + ocr_result"
fi
echo "  logs: ${LOG_DIR}/"
echo ""
echo "Verify:"
echo "  ros2 run tf2_ros tf2_echo ${TF_PARENT} ${TF_CHILD}"
echo "  ros2 topic echo /color/image_raw --once"
echo "  ros2 topic echo /target_3d_position --once"
echo "  ros2 topic echo /eightboxes --once"
echo "  ros2 topic echo /stable_arithmetic_result --once"
echo ""
echo "Stop:"
echo "  ${SCRIPT_DIR}/stop_perception_stack.sh        # kills tmux + orphan nodes"
echo "  ${SCRIPT_DIR}/stop_perception_stack.sh --force  # also kills point_lio launch"
echo "=========================================="

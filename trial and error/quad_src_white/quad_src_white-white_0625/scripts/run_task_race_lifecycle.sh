#!/usr/bin/env bash
# 任务赛一键生命周期脚本。
#
# 这个脚本负责把“任务赛主流程”和“任务赛感知栈”的启动/关闭顺序串起来：
#   1) 等 point_lio 已经发布雷达定位 TF；
#   2) 启动任务赛需要的感知栈；
#   3) 前台启动 task_race.launch.py；
#   4) 监听预扫描完成事件，收到后自动关闭感知栈，避免后续占用算力。
#
# 注意：
#   - 本脚本不会启动或关闭 point_lio，point_lio 需要提前单独启动。
#   - 不使用 set -u，因为 ROS 的 setup.bash 内部会引用一些可能未定义的 AMENT_* 变量。
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 下面这些变量都允许在命令行用环境变量覆盖，方便现场临时调试：
#   HITCRT_WS=/path/to/ws TF_WAIT_TIMEOUT_SEC=180 ./scripts/run_task_race_lifecycle.sh
HITCRT_WS="${HITCRT_WS:-/home/cat/hitcrt_quad2026_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
HITCRT_SETUP="${HITCRT_SETUP:-${HITCRT_WS}/install/setup.bash}"

# point_lio 正常后应在 /tf 中出现 TF_PARENT -> TF_CHILD。
# 等到这条 TF 后，才说明雷达定位链路基本可用，可以继续启动感知和任务赛。
TF_PARENT="${TF_PARENT:-camera_init}"
TF_CHILD="${TF_CHILD:-aft_mapped}"
TF_WAIT_TIMEOUT_SEC="${TF_WAIT_TIMEOUT_SEC:-120}"

# 感知栈启动后额外等待几秒，让节点、模型和话题有时间稳定。
POST_PERCEPTION_DELAY_SEC="${POST_PERCEPTION_DELAY_SEC:-5}"

# /quad/logistics_nav_event 的第一个整数是事件类型。
# 当前默认 0 表示 PRE_SCAN_DONE：任务赛预扫描完成，可以关闭感知栈。
TRIGGER_EVENT="${TRIGGER_EVENT:-0}"

# 感知栈实际由这两个外部脚本管理，生命周期脚本只负责在合适的时间调用它们。
START_PERCEPTION_SCRIPT="${START_PERCEPTION_SCRIPT:-/home/cat/start_perception_stack.sh}"
STOP_PERCEPTION_SCRIPT="${STOP_PERCEPTION_SCRIPT:-/home/cat/stop_perception_stack.sh}"

# wait_ros_ready.sh 提供 wait_for_topic / wait_for_tf 等等待函数。
WAIT_ROS_READY_LIB="${WAIT_ROS_READY_LIB:-${HITCRT_WS}/scripts/lib/wait_ros_ready.sh}"
TASK_RACE_LAUNCH="${TASK_RACE_LAUNCH:-${HITCRT_WS}/src/launch/task_race.launch.py}"

# watcher 是后台 Python 进程，用来监听 PRE_SCAN_DONE 事件。
WATCHER_PID=""

# 标记是否已经尝试启动过感知栈。只有启动过，退出清理时才会尝试关闭。
PERCEPTION_START_ATTEMPTED=0

log() {
    echo "[task-race-lifecycle] $*"
}

die() {
    echo "[task-race-lifecycle][error] $*" >&2
    exit 1
}

require_cmd() {
    command -v "$1" >/dev/null 2>&1 || die "Missing command: $1"
}

require_file() {
    [[ -f "$1" ]] || die "Required file not found: $1"
}

safe_source() {
    local setup_file="$1"
    require_file "${setup_file}"
    set +u
    # shellcheck source=/dev/null
    source "${setup_file}"
    set -e
}

stop_perception() {
    if [[ -f "${STOP_PERCEPTION_SCRIPT}" ]]; then
        log "Stopping perception stack (point_lio is left untouched) ..."
        # 把工作区和日志目录传给停止脚本；停止失败也不阻断清理流程。
        HITCRT_WS="${HITCRT_WS}" LOG_DIR="${HITCRT_WS}/logs/perception" bash "${STOP_PERCEPTION_SCRIPT}" || true
    fi
}

cleanup() {
    local rc=$?

    # 脚本退出时先停掉后台 watcher，避免残留订阅进程。
    if [[ -n "${WATCHER_PID}" ]] && kill -0 "${WATCHER_PID}" 2>/dev/null; then
        log "Stopping PRE_SCAN_DONE watcher ..."
        kill "${WATCHER_PID}" 2>/dev/null || true
        wait "${WATCHER_PID}" 2>/dev/null || true
    fi

    # 如果本脚本启动过感知栈，则退出时兜底关闭感知栈。
    # point_lio 不在本脚本管理范围内，所以不会被关闭。
    if [[ "${PERCEPTION_START_ATTEMPTED}" == "1" ]]; then
        stop_perception
    fi
    return "${rc}"
}

start_prescan_done_watcher() {
    log "Starting watcher: /quad/logistics_nav_event event_type=${TRIGGER_EVENT} -> stop perception"
    # 用内嵌 Python 起一个 ROS 2 节点：
    #   - 订阅 /quad/logistics_nav_event；
    #   - 收到指定事件类型后调用 stop_perception_stack.sh；
    #   - done=true 后退出 watcher。
    python3 - "${TRIGGER_EVENT}" "${STOP_PERCEPTION_SCRIPT}" <<'PY' &
import subprocess
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class PreScanDoneWatcher(Node):
    def __init__(self, trigger_event: int, stop_script: str) -> None:
        super().__init__("task_race_perception_lifecycle_watcher")
        self.trigger_event = trigger_event
        self.stop_script = stop_script
        self.done = False
        self.create_subscription(
            Int32MultiArray,
            "/quad/logistics_nav_event",
            self._on_event,
            10,
        )
        self.get_logger().info(
            f"Watching /quad/logistics_nav_event for event_type={trigger_event}"
        )

    def _on_event(self, msg: Int32MultiArray) -> None:
        # 消息格式约定为 [event_type, waypoint_index, seq, action_id]。
        # 这里只关心第一个字段 event_type。
        data = list(msg.data)
        if not data:
            return
        event_type = data[0]
        if event_type != self.trigger_event:
            return

        self.get_logger().info(
            f"Matched logistics_nav_event={data}; stopping perception stack"
        )
        subprocess.run(["bash", self.stop_script], check=False)
        self.done = True


def main() -> int:
    trigger_event = int(sys.argv[1])
    stop_script = sys.argv[2]

    rclpy.init()
    node = PreScanDoneWatcher(trigger_event, stop_script)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
PY
    WATCHER_PID=$!
}

# 无论正常退出、Ctrl+C 还是收到 TERM，都走 cleanup，保证感知栈被清理。
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# 启动前做依赖检查，尽早报出缺失命令或路径。
require_cmd bash
require_cmd python3
require_cmd ros2

require_file "${START_PERCEPTION_SCRIPT}"
require_file "${STOP_PERCEPTION_SCRIPT}"
require_file "${WAIT_ROS_READY_LIB}"
require_file "${TASK_RACE_LAUNCH}"

# 加载 ROS 和当前工作区环境。必须先 source，后续 ros2/rclpy 才能找到包和消息。
safe_source "${ROS_SETUP}"
safe_source "${HITCRT_SETUP}"
# shellcheck source=/dev/null
source "${WAIT_ROS_READY_LIB}"

# 等待 point_lio 发布 /tf 以及指定坐标变换。
# 这一步只检查 point_lio 是否就绪，不负责启动 point_lio。
log "Waiting for point_lio TF ${TF_PARENT} -> ${TF_CHILD}; this script will not start point_lio."
wait_for_topic "/tf" "${TF_WAIT_TIMEOUT_SEC}"
wait_for_tf "${TF_PARENT}" "${TF_CHILD}" "${TF_WAIT_TIMEOUT_SEC}"

# 雷达定位就绪后再启动感知栈。
log "Starting perception stack ..."
PERCEPTION_START_ATTEMPTED=1
HITCRT_WS="${HITCRT_WS}" LOG_DIR="${HITCRT_WS}/logs/perception" bash "${START_PERCEPTION_SCRIPT}"

if (( POST_PERCEPTION_DELAY_SEC > 0 )); then
    log "Waiting ${POST_PERCEPTION_DELAY_SEC}s for perception stack to settle ..."
    sleep "${POST_PERCEPTION_DELAY_SEC}"
fi

# 感知栈稳定后启动后台 watcher，等待任务赛预扫描完成事件。
start_prescan_done_watcher

# task_race 主流程放在前台运行：终端日志直观，Ctrl+C 也能触发 cleanup。
log "Launching task race main process in foreground ..."
log "Use Ctrl+C here to stop task race and clean perception stack."
cd "${HITCRT_WS}"
ros2 launch "${TASK_RACE_LAUNCH}"

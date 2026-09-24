#!/usr/bin/env bash
# Shared wait helpers for perception stack startup scripts.

wait_log() {
    echo "[wait] $*"
}

# wait_for_topic TOPIC [TIMEOUT_SEC]
# Succeeds when the topic appears in `ros2 topic list`.
wait_for_topic() {
    local topic="$1"
    local timeout="${2:-${WAIT_TIMEOUT_SEC:-120}}"
    local elapsed=0
    local interval=2

    if [[ -z "$topic" ]]; then
        echo "wait_for_topic: topic name required" >&2
        return 1
    fi

    wait_log "Waiting for topic ${topic} (timeout ${timeout}s) ..."
    local topic_pattern="${topic}"
    [[ "${topic_pattern}" == /* ]] || topic_pattern="/${topic_pattern}"

    while (( elapsed < timeout )); do
        if ros2 topic list 2>/dev/null | grep -qxF "${topic_pattern}"; then
            wait_log "Topic ready: ${topic_pattern}"
            return 0
        fi
        sleep "${interval}"
        elapsed=$((elapsed + interval))
    done

    echo "Timeout waiting for topic: ${topic}" >&2
    return 1
}

# wait_for_topic_message TOPIC [TIMEOUT_SEC]
# Succeeds when `ros2 topic echo --once` returns within timeout.
wait_for_topic_message() {
    local topic="$1"
    local timeout="${2:-${WAIT_TIMEOUT_SEC:-120}}"
    local elapsed=0
    local interval=2

    if [[ -z "$topic" ]]; then
        echo "wait_for_topic_message: topic name required" >&2
        return 1
    fi

    local topic_pattern="${topic}"
    [[ "${topic_pattern}" == /* ]] || topic_pattern="/${topic_pattern}"

    wait_log "Waiting for first message on ${topic_pattern} (timeout ${timeout}s) ..."
    while (( elapsed < timeout )); do
        if timeout 8 ros2 topic echo "${topic_pattern}" --once >/dev/null 2>&1; then
            wait_log "Message received on: ${topic}"
            return 0
        fi
        sleep "${interval}"
        elapsed=$((elapsed + interval))
    done

    echo "Timeout waiting for message on topic: ${topic}" >&2
    return 1
}

# tf_topic_has_transform PARENT CHILD
# Parse one /tf message; /tf 在发布 ≠ 已包含目标父子帧对。
tf_topic_has_transform() {
    local parent="$1"
    local child="$2"
    local tmp
    tmp="$(mktemp)"

    if ! timeout 6 ros2 topic echo /tf --once >"${tmp}" 2>/dev/null; then
        rm -f "${tmp}"
        return 1
    fi

    if python3 - "${parent}" "${child}" "${tmp}" <<'PY'
import re
import sys

parent, child, path = sys.argv[1], sys.argv[2], sys.argv[3]
text = open(path, encoding="utf-8", errors="ignore").read()
blocks = re.split(r"\n\s*-\s+header:", text)
for block in blocks:
    m_parent = re.search(r"frame_id:\s*['\"]?([^'\"\n]+)", block)
    m_child = re.search(r"child_frame_id:\s*['\"]?([^'\"\n]+)", block)
    if not m_parent or not m_child:
        continue
    if m_parent.group(1).strip() == parent and m_child.group(1).strip() == child:
        sys.exit(0)
sys.exit(1)
PY
    then
        rm -f "${tmp}"
        return 0
    fi

    rm -f "${tmp}"
    return 1
}

# tf_echo_has_transform PARENT CHILD [PROBE_SEC]
# tf2_echo 会持续输出；用 grep -m1 在首条 Translation 出现即判定成功（忽略开头 Invalid frame 警告）。
tf_echo_has_transform() {
    local parent="$1"
    local child="$2"
    local probe_sec="${3:-15}"

    if timeout "${probe_sec}" ros2 run tf2_ros tf2_echo "${parent}" "${child}" 2>&1 \
        | grep -m1 -qE '^- Translation:'; then
        return 0
    fi
    return 1
}

tf_log_sample_frames() {
    local sample
    sample="$(timeout 5 ros2 topic echo /tf --once 2>/dev/null | grep -E '^  frame_id:|^  child_frame_id:' | head -24 || true)"
    if [[ -n "${sample}" ]]; then
        wait_log "  当前 /tf 里部分坐标系（供对照，非目标 ${TF_PARENT:-?}->${TF_CHILD:-?}）:"
        echo "${sample}" | sed 's/^/    /'
    else
        wait_log "  未能从 /tf 读到 frame_id（话题在但可能没有有效数据）"
    fi
}

# wait_for_tf PARENT_FRAME CHILD_FRAME [TIMEOUT_SEC]
wait_for_tf() {
    local parent="$1"
    local child="$2"
    local timeout="${3:-${WAIT_TIMEOUT_SEC:-120}}"
    local elapsed=0
    local interval=3
    local last_progress=-10

    if [[ -z "$parent" || -z "$child" ]]; then
        echo "wait_for_tf: parent and child frame required" >&2
        return 1
    fi

    wait_log "Waiting for TF ${parent} -> ${child} (timeout ${timeout}s) ..."
    wait_log "  说明: ros2 topic list 能看到 /tf 只表示话题在；还要 /tf 消息里含 ${parent} -> ${child} 这一对变换"
    while (( elapsed < timeout )); do
        if tf_topic_has_transform "${parent}" "${child}"; then
            wait_log "TF ready (from /tf): ${parent} -> ${child}"
            return 0
        fi
        if tf_echo_has_transform "${parent}" "${child}" 15; then
            wait_log "TF ready (from tf2_echo): ${parent} -> ${child}"
            return 0
        fi
        if (( elapsed - last_progress >= 10 )); then
            wait_log "  still waiting (${elapsed}s / ${timeout}s) ..."
            tf_log_sample_frames
            last_progress=$elapsed
        fi
        sleep "${interval}"
        elapsed=$((elapsed + interval))
    done

    echo "Timeout waiting for TF: ${parent} -> ${child}" >&2
    echo "  常见原因:" >&2
    echo "    1) 只开了雷达驱动，未开 point_lio 定位（无 camera_init/aft_mapped）" >&2
    echo "    2) point_lio 仍在初始化，/tf 里暂时是 map/odom 等其它帧" >&2
    echo "    3) 坐标系名称与配置不一致（可设 TF_PARENT / TF_CHILD）" >&2
    tf_log_sample_frames >&2
    echo "  手动检查: ros2 run tf2_ros tf2_echo ${parent} ${child}" >&2
    return 1
}

# wait_for_cmd COMMAND_NAME TIMEOUT_SEC -- command...
# Runs command repeatedly until success or timeout.
wait_for_cmd() {
    local name="$1"
    local timeout="$2"
    shift 2
    local elapsed=0
    local interval=2

    wait_log "Waiting for condition '${name}' (timeout ${timeout}s) ..."
    while (( elapsed < timeout )); do
        if "$@"; then
            wait_log "Condition satisfied: ${name}"
            return 0
        fi
        sleep "${interval}"
        elapsed=$((elapsed + interval))
    done

    echo "Timeout waiting for condition: ${name}" >&2
    return 1
}

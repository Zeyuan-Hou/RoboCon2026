#!/usr/bin/env bash
# Start PaddleOCR stack in detached tmux session (ocr_sys).
# ppocr_node 订阅 /color/image_raw（与 RealSense 共用，默认不启 USB 相机）。

start_paddleocr_stack() {
    local ocr_ws="${1:?ocr workspace path required}"
    local log_dir="${2:?log dir required}"
    local ocr_session="${OCR_SESSION:-ocr_sys}"
    local start_usbcam="${START_OCR_USBCAM:-0}"
    local ocr_visualize="${START_OCR_USBCAM_VISUALIZE:-0}"

    command -v uv >/dev/null 2>&1 || {
        echo "[error] OCR 需要 uv，请安装: curl -LsSf https://astral.sh/uv/install.sh | sh" >&2
        return 1
    }

    [[ -d "${ocr_ws}/nodes" ]] || {
        echo "[error] PaddleOCR 目录无效: ${ocr_ws}" >&2
        return 1
    }

    mkdir -p "${log_dir}"

    if tmux has-session -t "${ocr_session}" 2>/dev/null; then
        echo "[info] Killing old OCR tmux session: ${ocr_session}"
        tmux kill-session -t "${ocr_session}"
        sleep 1
    fi

    local ppocr_cmd="cd ${ocr_ws} && uv run python nodes/ppocr_node.py 2>&1 | tee -a ${log_dir}/ppocr.log"
    local result_cmd="cd ${ocr_ws} && uv run python nodes/ocr_result_subscriber_node.py 2>&1 | tee -a ${log_dir}/ocr_result.log"
    local usbcam_args="--fps 15"
    if [[ "${ocr_visualize}" == "1" ]]; then
        usbcam_args+=" --visualize"
    fi
    local usbcam_cmd="cd ${ocr_ws} && uv run python nodes/usbcam_node.py ${usbcam_args} 2>&1 | tee -a ${log_dir}/usbcam.log"

    echo "[info] OCR 图像源: /color/image_raw（RealSense，与 ppocr_node 一致）"
    if [[ "${start_usbcam}" == "1" ]]; then
        echo "[info] 额外启动 USB 相机 -> /image_raw（与 ppocr 话题不同，需自行 remap 或改配置）"
    fi

    tmux new-session -d -s "${ocr_session}" -n ocr -c "${ocr_ws}"
    tmux send-keys -t "${ocr_session}:0" "${ppocr_cmd}" C-m

    if [[ "${start_usbcam}" == "1" ]]; then
        tmux split-window -h -t "${ocr_session}:0"
        tmux select-pane -t "${ocr_session}:0.0" -T ppocr
        tmux send-keys -t "${ocr_session}:0.1" "${usbcam_cmd}" C-m
        tmux split-window -v -t "${ocr_session}:0.0"
        tmux select-pane -t "${ocr_session}:0.2" -T ocr_result
        tmux send-keys -t "${ocr_session}:0.2" "${result_cmd}" C-m
    else
        tmux split-window -h -t "${ocr_session}:0"
        tmux select-pane -t "${ocr_session}:0.0" -T ppocr
        tmux select-pane -t "${ocr_session}:0.1" -T ocr_result
        tmux send-keys -t "${ocr_session}:0.1" "${result_cmd}" C-m
    fi

    echo "[info] OCR tmux session ready: ${ocr_session}"
    echo "       tmux attach -t ${ocr_session}"
    echo "       节点: ppocr_node + ocr_result_subscriber_node -> /stable_arithmetic_result"
    return 0
}

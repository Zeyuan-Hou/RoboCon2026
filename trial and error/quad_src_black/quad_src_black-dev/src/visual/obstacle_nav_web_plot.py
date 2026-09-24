#!/usr/bin/env python3
"""Web-based 2D obstacle-race path visualizer.

Run this on the robot, then open the page through SSH port forwarding:

    ssh -L 8765:localhost:8765 cat@ROBOT_IP
    python3 src/visual/obstacle_nav_web_plot.py

Local browser:

    http://localhost:8765
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from obstacle_nav_xy_plot import DEFAULT_CONFIG, load_expected_path


HTML_PAGE = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Obstacle Nav XY Plot</title>
  <style>
    html, body {
      margin: 0;
      width: 100%;
      height: 100%;
      background: #f8fafc;
      color: #111827;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    #wrap {
      box-sizing: border-box;
      width: 100%;
      height: 100%;
      padding: 14px;
      display: grid;
      grid-template-rows: auto 1fr;
      gap: 10px;
    }
    #bar {
      display: flex;
      gap: 18px;
      align-items: center;
      flex-wrap: wrap;
      font-size: 14px;
    }
    .legend {
      display: inline-flex;
      align-items: center;
      gap: 6px;
    }
    .swatch {
      width: 22px;
      height: 4px;
      border-radius: 2px;
      display: inline-block;
    }
    #status {
      color: #374151;
      margin-left: auto;
      white-space: nowrap;
    }
    canvas {
      width: 100%;
      height: 100%;
      background: white;
      border: 1px solid #d1d5db;
      box-sizing: border-box;
    }
  </style>
</head>
<body>
  <div id="wrap">
    <div id="bar">
      <strong>Obstacle Nav XY</strong>
      <span class="legend"><span class="swatch" style="background:#2563eb"></span>期望路径</span>
      <span class="legend"><span class="swatch" style="background:#16a34a"></span>实际路径</span>
      <span class="legend"><span class="swatch" style="background:#dc2626"></span>当前位置</span>
      <span id="status">连接中...</span>
    </div>
    <canvas id="plot"></canvas>
  </div>
  <script>
    const canvas = document.getElementById("plot");
    const ctx = canvas.getContext("2d");
    const statusEl = document.getElementById("status");
    let latest = null;

    function resizeCanvas() {
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      const w = Math.max(320, Math.floor(rect.width * dpr));
      const h = Math.max(260, Math.floor(rect.height * dpr));
      if (canvas.width !== w || canvas.height !== h) {
        canvas.width = w;
        canvas.height = h;
      }
    }

    function bounds(data) {
      const xs = data.expected_x.concat(data.actual_x);
      const ys = data.expected_y.concat(data.actual_y);
      if (!xs.length || !ys.length) return [-1, 1, -1, 1];
      let xmin = Math.min(...xs), xmax = Math.max(...xs);
      let ymin = Math.min(...ys), ymax = Math.max(...ys);
      const pad = Math.max(0.5, 0.08 * Math.max(xmax - xmin, ymax - ymin, 1.0));
      return [xmin - pad, xmax + pad, ymin - pad, ymax + pad];
    }

    function makeTransform(b) {
      const [xmin, xmax, ymin, ymax] = b;
      const margin = 54;
      const plotW = Math.max(1, canvas.width - 2 * margin);
      const plotH = Math.max(1, canvas.height - 2 * margin);
      const spanX = Math.max(1e-6, xmax - xmin);
      const spanY = Math.max(1e-6, ymax - ymin);
      const scale = Math.min(plotW / spanX, plotH / spanY);
      const usedW = spanX * scale;
      const usedH = spanY * scale;
      const offX = margin + (plotW - usedW) / 2;
      const offY = margin + (plotH - usedH) / 2;
      return (x, y) => [offX + (x - xmin) * scale, canvas.height - offY - (y - ymin) * scale];
    }

    function drawGrid(toPx, b) {
      ctx.strokeStyle = "#e5e7eb";
      ctx.lineWidth = 1;
      ctx.fillStyle = "#6b7280";
      ctx.font = "12px system-ui";
      const [xmin, xmax, ymin, ymax] = b;
      const step = niceStep(Math.max(xmax - xmin, ymax - ymin) / 8);
      for (let x = Math.ceil(xmin / step) * step; x <= xmax; x += step) {
        const [px1, py1] = toPx(x, ymin);
        const [px2, py2] = toPx(x, ymax);
        ctx.beginPath();
        ctx.moveTo(px1, py1);
        ctx.lineTo(px2, py2);
        ctx.stroke();
        ctx.fillText(x.toFixed(1), px1 + 3, canvas.height - 18);
      }
      for (let y = Math.ceil(ymin / step) * step; y <= ymax; y += step) {
        const [px1, py1] = toPx(xmin, y);
        const [px2, py2] = toPx(xmax, y);
        ctx.beginPath();
        ctx.moveTo(px1, py1);
        ctx.lineTo(px2, py2);
        ctx.stroke();
        ctx.fillText(y.toFixed(1), 12, py1 - 3);
      }
    }

    function niceStep(v) {
      const pow = Math.pow(10, Math.floor(Math.log10(Math.max(v, 1e-6))));
      const n = v / pow;
      if (n < 1.5) return pow;
      if (n < 3.5) return 2 * pow;
      if (n < 7.5) return 5 * pow;
      return 10 * pow;
    }

    function drawLine(xs, ys, toPx, color, width) {
      if (xs.length < 2) return;
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.beginPath();
      for (let i = 0; i < xs.length; i++) {
        const [px, py] = toPx(xs[i], ys[i]);
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();
    }

    function draw() {
      resizeCanvas();
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "white";
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      if (!latest) return;

      const b = bounds(latest);
      const toPx = makeTransform(b);
      drawGrid(toPx, b);
      drawLine(latest.expected_x, latest.expected_y, toPx, "#2563eb", 2.5);
      drawLine(latest.actual_x, latest.actual_y, toPx, "#16a34a", 2.5);

      if (latest.has_pose) {
        const [px, py] = toPx(latest.current.x, latest.current.y);
        const [hx, hy] = toPx(
          latest.current.x + Math.cos(latest.current.yaw) * 0.35,
          latest.current.y + Math.sin(latest.current.yaw) * 0.35
        );
        ctx.fillStyle = "#dc2626";
        ctx.beginPath();
        ctx.arc(px, py, 6, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = "#dc2626";
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(px, py);
        ctx.lineTo(hx, hy);
        ctx.stroke();
      } else {
        ctx.fillStyle = "#6b7280";
        ctx.font = "24px system-ui";
        ctx.fillText("waiting for /quad/lidar_pose_xyyaw ...", 70, 110);
      }

      ctx.fillStyle = "#111827";
      ctx.font = "13px system-ui";
      ctx.fillText(latest.source, 16, 24);
      ctx.fillText("pose: " + latest.pose_topic, 16, 42);
      if (latest.has_pose) {
        const c = latest.current;
        ctx.fillText(`x=${c.x.toFixed(3)} y=${c.y.toFixed(3)} yaw=${c.yaw.toFixed(3)} actual=${latest.actual_count}`, 16, 60);
      }
    }

    async function poll() {
      try {
        const res = await fetch("/data", {cache: "no-store"});
        latest = await res.json();
        statusEl.textContent = latest.has_pose ? "已连接" : "等待实时位置";
        draw();
      } catch (err) {
        statusEl.textContent = "连接失败";
      }
    }

    window.addEventListener("resize", draw);
    setInterval(poll, 120);
    poll();
  </script>
</body>
</html>
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Serve a browser-based obstacle navigation xy plot."
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="Path to quad_parku.yaml.")
    parser.add_argument("--replay-path", default="", help="Override replay path from config.")
    parser.add_argument(
        "--pose-topic",
        default="/quad/lidar_pose_xyyaw",
        help="Live pose topic. Default expects geometry_msgs/Point x/y/z=yaw.",
    )
    parser.add_argument(
        "--pose-type",
        choices=("float_array", "point"),
        default="point",
        help="Pose message type.",
    )
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind host.")
    parser.add_argument("--port", type=int, default=8765, help="HTTP port.")
    parser.add_argument("--max-actual-points", type=int, default=20000)
    parser.add_argument("--max-send-points", type=int, default=6000)
    return parser.parse_args()


def downsample(xs: list[float], ys: list[float], max_points: int) -> tuple[list[float], list[float]]:
    if len(xs) <= max_points:
        return list(xs), list(ys)
    step = max(1, math.ceil(len(xs) / max_points))
    return list(xs[::step]), list(ys[::step])


class WebPoseNode(Node):
    def __init__(self, topic: str, pose_type: str, max_points: int) -> None:
        super().__init__("obstacle_nav_web_plot")
        self.lock = threading.Lock()
        self.actual_x: list[float] = []
        self.actual_y: list[float] = []
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.received = False
        self.max_points = max(10, max_points)

        if pose_type == "point":
            self.subscription = self.create_subscription(Point, topic, self._point_callback, 10)
        else:
            self.subscription = self.create_subscription(
                Float32MultiArray, topic, self._array_callback, 10
            )

    def _append_pose(self, x: float, y: float, yaw: float) -> None:
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw)):
            return
        with self.lock:
            self.actual_x.append(x)
            self.actual_y.append(y)
            if len(self.actual_x) > self.max_points:
                overflow = len(self.actual_x) - self.max_points
                del self.actual_x[:overflow]
                del self.actual_y[:overflow]
            self.current_x = x
            self.current_y = y
            self.current_yaw = yaw
            self.received = True

    def _array_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            return
        yaw = float(msg.data[2]) if len(msg.data) >= 3 else self.current_yaw
        self._append_pose(float(msg.data[0]), float(msg.data[1]), yaw)

    def _point_callback(self, msg: Point) -> None:
        self._append_pose(float(msg.x), float(msg.y), float(msg.z))

    def snapshot(self) -> tuple[list[float], list[float], dict[str, float], bool]:
        with self.lock:
            return (
                list(self.actual_x),
                list(self.actual_y),
                {"x": self.current_x, "y": self.current_y, "yaw": self.current_yaw},
                self.received,
            )


class PlotState:
    def __init__(
        self,
        expected_x: list[float],
        expected_y: list[float],
        source: str,
        pose_topic: str,
        pose_node: WebPoseNode,
        max_send_points: int,
    ) -> None:
        self.expected_x, self.expected_y = downsample(expected_x, expected_y, max_send_points)
        self.expected_count = len(expected_x)
        self.source = source
        self.pose_topic = pose_topic
        self.pose_node = pose_node
        self.max_send_points = max_send_points

    def to_json_bytes(self) -> bytes:
        actual_x, actual_y, current, received = self.pose_node.snapshot()
        send_x, send_y = downsample(actual_x, actual_y, self.max_send_points)
        payload = {
            "source": self.source,
            "pose_topic": self.pose_topic,
            "expected_x": self.expected_x,
            "expected_y": self.expected_y,
            "expected_count": self.expected_count,
            "actual_x": send_x,
            "actual_y": send_y,
            "actual_count": len(actual_x),
            "current": current,
            "has_pose": received,
            "stamp": time.time(),
        }
        return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def make_handler(state: PlotState):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args) -> None:
            return

        def do_GET(self) -> None:
            if self.path == "/" or self.path.startswith("/index"):
                body = HTML_PAGE.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path.startswith("/data"):
                body = state.to_json_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            self.send_error(404)

    return Handler


def main() -> int:
    args = parse_args()
    config_path = Path(args.config).expanduser()
    expected_x, expected_y, _expected_yaw, source = load_expected_path(
        config_path, args.replay_path
    )

    rclpy.init()
    pose_node = WebPoseNode(args.pose_topic, args.pose_type, args.max_actual_points)
    state = PlotState(
        expected_x,
        expected_y,
        source,
        args.pose_topic,
        pose_node,
        max(100, args.max_send_points),
    )
    server = ThreadingHTTPServer((args.host, args.port), make_handler(state))

    stop_event = threading.Event()

    def ros_spin() -> None:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(pose_node, timeout_sec=0.05)

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    print(f"[info] loaded expected path: {source}")
    print(f"[info] expected points: {len(expected_x)}")
    print(f"[info] listening pose topic: {args.pose_topic} ({args.pose_type})")
    print(f"[info] web server: http://{args.host}:{args.port}")
    print("[info] SSH tunnel example: ssh -L 8765:localhost:8765 cat@ROBOT_IP")

    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        server.shutdown()
        server.server_close()
        pose_node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

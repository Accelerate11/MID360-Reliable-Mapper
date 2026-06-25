#!/usr/bin/env python3
import argparse
import json
import math
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


HTML = r"""<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <title>MID360 Live Mapping</title>
  <style>
    html, body { margin:0; height:100%; background:#101214; color:#e8edf2; font-family:Arial, sans-serif; overflow:hidden; }
    #bar { position:fixed; left:0; top:0; right:0; height:44px; background:#181c20; display:flex; align-items:center; gap:18px; padding:0 14px; box-sizing:border-box; border-bottom:1px solid #2c333a; }
    #title { font-weight:700; letter-spacing:.2px; }
    #status { color:#9fb3c8; font-size:14px; }
    #canvas { position:fixed; left:0; top:44px; width:100vw; height:calc(100vh - 44px); display:block; background:#0b0d0f; }
    .ok { color:#49e27a; }
    .warn { color:#ffd45e; }
  </style>
</head>
<body>
  <div id="bar">
    <div id="title">MID360 Live Mapping</div>
    <div id="status">connecting...</div>
  </div>
  <canvas id="canvas"></canvas>
<script>
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const statusEl = document.getElementById('status');
let data = {map: [], scan: [], path: [], stats: {}};
let scale = 28;
let ox = 0, oy = 0;
let dragging = false, lastX = 0, lastY = 0;

function resize() {
  canvas.width = Math.max(320, window.innerWidth);
  canvas.height = Math.max(240, window.innerHeight - 44);
}
window.addEventListener('resize', resize);
resize();

canvas.addEventListener('wheel', e => {
  e.preventDefault();
  scale *= e.deltaY < 0 ? 1.12 : 0.89;
  scale = Math.max(4, Math.min(180, scale));
});
canvas.addEventListener('mousedown', e => { dragging = true; lastX = e.clientX; lastY = e.clientY; });
window.addEventListener('mouseup', () => dragging = false);
window.addEventListener('mousemove', e => {
  if (!dragging) return;
  ox += e.clientX - lastX;
  oy += e.clientY - lastY;
  lastX = e.clientX; lastY = e.clientY;
});

function worldToScreen(p) {
  return [canvas.width / 2 + ox + p[0] * scale, canvas.height / 2 + oy - p[1] * scale];
}

function drawGrid() {
  ctx.strokeStyle = '#222930';
  ctx.lineWidth = 1;
  const step = scale;
  const startX = (canvas.width / 2 + ox) % step;
  const startY = (canvas.height / 2 + oy) % step;
  for (let x = startX; x < canvas.width; x += step) { ctx.beginPath(); ctx.moveTo(x,0); ctx.lineTo(x,canvas.height); ctx.stroke(); }
  for (let y = startY; y < canvas.height; y += step) { ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(canvas.width,y); ctx.stroke(); }
  ctx.strokeStyle = '#4c5966';
  ctx.beginPath();
  ctx.moveTo(0, canvas.height / 2 + oy); ctx.lineTo(canvas.width, canvas.height / 2 + oy);
  ctx.moveTo(canvas.width / 2 + ox, 0); ctx.lineTo(canvas.width / 2 + ox, canvas.height);
  ctx.stroke();
}

function drawPoints(points, color, size) {
  ctx.fillStyle = color;
  for (const p of points) {
    const [x, y] = worldToScreen(p);
    if (x < -5 || y < -5 || x > canvas.width + 5 || y > canvas.height + 5) continue;
    ctx.fillRect(x, y, size, size);
  }
}

function drawPath(path) {
  if (path.length < 2) return;
  ctx.strokeStyle = '#50e37f';
  ctx.lineWidth = 2;
  ctx.beginPath();
  let [x0, y0] = worldToScreen(path[0]);
  ctx.moveTo(x0, y0);
  for (let i = 1; i < path.length; i++) {
    const [x, y] = worldToScreen(path[i]);
    ctx.lineTo(x, y);
  }
  ctx.stroke();
}

function render() {
  ctx.clearRect(0,0,canvas.width,canvas.height);
  drawGrid();
  drawPoints(data.map || [], '#2e9cff', 1.5);
  drawPoints(data.scan || [], '#ffdc5e', 2.2);
  drawPath(data.path || []);
  const s = data.stats || {};
  const age = s.last_update_age ?? 999;
  const cls = age < 2 ? 'ok' : 'warn';
  statusEl.innerHTML = `<span class="${cls}">${age < 2 ? 'live' : 'waiting'}</span> map=${s.map_points || 0} scan=${s.scan_points || 0} frames=${s.frames || 0} scale=${scale.toFixed(1)} px/m`;
  requestAnimationFrame(render);
}
render();

async function poll() {
  try {
    const r = await fetch('/data', {cache:'no-store'});
    data = await r.json();
  } catch (e) {
    statusEl.textContent = 'connection lost';
  }
  setTimeout(poll, 250);
}
poll();
</script>
</body>
</html>
"""


def stamp_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class WebMappingNode(Node):
    def __init__(self, args):
        super().__init__("web_mid360_mapping_viewer")
        self.args = args
        self.lock = threading.Lock()
        self.map_points = []
        self.scan_points = []
        self.path = []
        self.frames = 0
        self.last_update = 0.0
        self.last_process = 0.0
        self.create_subscription(PointCloud2, args.topic, self.cloud_cb, 2)

    def cloud_cb(self, msg):
        now = time.time()
        if now - self.last_process < self.args.period:
            return
        self.last_process = now

        points = self.extract_points(msg, self.args.scan_points)
        if not points:
            return

        cx = sum(p[0] for p in points) / len(points)
        cy = sum(p[1] for p in points) / len(points)

        with self.lock:
            self.scan_points = points
            self.map_points.extend(points[::max(1, len(points) // max(1, self.args.map_add_points))])
            if len(self.map_points) > self.args.map_points:
                self.map_points = self.map_points[-self.args.map_points:]
            self.path.append([round(cx, 3), round(cy, 3)])
            if len(self.path) > 2000:
                self.path = self.path[-2000:]
            self.frames += 1
            self.last_update = now

    def extract_points(self, msg, limit):
        if not msg.data or msg.point_step == 0:
            return []
        offsets = {field.name: field.offset for field in msg.fields}
        if not {"x", "y", "z"}.issubset(offsets):
            return []
        total = min(int(msg.width) * int(msg.height), len(msg.data) // msg.point_step)
        if total <= 0:
            return []
        stride = max(1, total // max(1, limit))
        endian = ">" if msg.is_bigendian else "<"
        fmt = endian + "f"
        out = []
        max_range2 = self.args.max_range * self.args.max_range
        for i in range(0, total, stride):
            base = i * msg.point_step
            try:
                x = struct.unpack_from(fmt, msg.data, base + offsets["x"])[0]
                y = struct.unpack_from(fmt, msg.data, base + offsets["y"])[0]
                z = struct.unpack_from(fmt, msg.data, base + offsets["z"])[0]
            except struct.error:
                break
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if x * x + y * y + z * z > max_range2:
                continue
            out.append([round(x, 3), round(y, 3), round(z, 3)])
            if len(out) >= limit:
                break
        return out

    def snapshot(self):
        with self.lock:
            return {
                "map": self.map_points,
                "scan": self.scan_points,
                "path": self.path,
                "stats": {
                    "map_points": len(self.map_points),
                    "scan_points": len(self.scan_points),
                    "frames": self.frames,
                    "last_update_age": round(time.time() - self.last_update, 2) if self.last_update else 999,
                },
            }


class Handler(BaseHTTPRequestHandler):
    node = None

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path.startswith("/data"):
            body = json.dumps(self.node.snapshot(), separators=(",", ":")).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)

    def log_message(self, fmt, *args):
        return


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/cloud_registered")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--period", type=float, default=0.25)
    parser.add_argument("--scan-points", type=int, default=3500)
    parser.add_argument("--map-add-points", type=int, default=700)
    parser.add_argument("--map-points", type=int, default=80000)
    parser.add_argument("--max-range", type=float, default=60.0)
    args = parser.parse_args()

    rclpy.init()
    node = WebMappingNode(args)
    Handler.node = node
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"web viewer: http://localhost:{args.port}")
    try:
        rclpy.spin(node)
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

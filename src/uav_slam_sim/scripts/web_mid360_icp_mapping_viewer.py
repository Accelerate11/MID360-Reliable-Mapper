#!/usr/bin/env python3
import argparse
import json
import math
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


HTML = r"""<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <title>MID360 ICP Mapping</title>
  <style>
    html, body { margin:0; height:100%; background:#101214; color:#e8edf2; font-family:Arial, sans-serif; overflow:hidden; }
    #bar { position:fixed; left:0; top:0; right:0; height:46px; background:#181c20; display:flex; align-items:center; gap:18px; padding:0 14px; box-sizing:border-box; border-bottom:1px solid #2c333a; }
    #title { font-weight:700; letter-spacing:.2px; }
    #status { color:#9fb3c8; font-size:14px; }
    #hint { margin-left:auto; color:#8192a5; font-size:13px; }
    #canvas { position:fixed; left:0; top:46px; width:100vw; height:calc(100vh - 46px); display:block; background:#0b0d0f; }
    .ok { color:#49e27a; }
    .warn { color:#ffd45e; }
    .bad { color:#ff6a6a; }
  </style>
</head>
<body>
  <div id="bar">
    <div id="title">MID360 ICP Mapping</div>
    <div id="status">connecting...</div>
    <div id="hint">3D: drag rotate | wheel zoom | Shift+drag pan | 2/3 switch | R reset</div>
  </div>
  <canvas id="canvas"></canvas>
<script>
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const statusEl = document.getElementById('status');
let data = {map: [], scan: [], path: [], stats: {}};
let mode = '3d';
let scale = 34;
let ox = 0, oy = 0;
let yaw = -0.72;
let pitch = 0.82;
let dragging = false, lastX = 0, lastY = 0;

function resize() {
  canvas.width = Math.max(320, window.innerWidth);
  canvas.height = Math.max(240, window.innerHeight - 46);
}
window.addEventListener('resize', resize);
resize();

canvas.addEventListener('wheel', e => {
  e.preventDefault();
  scale *= e.deltaY < 0 ? 1.12 : 0.89;
  scale = Math.max(5, Math.min(260, scale));
});
canvas.addEventListener('mousedown', e => { dragging = true; lastX = e.clientX; lastY = e.clientY; });
window.addEventListener('mouseup', () => dragging = false);
window.addEventListener('mousemove', e => {
  if (!dragging) return;
  const dx = e.clientX - lastX;
  const dy = e.clientY - lastY;
  if (mode === '3d' && !e.shiftKey) {
    yaw += dx * 0.008;
    pitch += dy * 0.006;
    pitch = Math.max(-1.25, Math.min(1.25, pitch));
  } else {
    ox += dx;
    oy += dy;
  }
  lastX = e.clientX; lastY = e.clientY;
});
window.addEventListener('keydown', e => {
  if (e.key.toLowerCase() === 'r') { ox = 0; oy = 0; scale = 34; yaw = -0.72; pitch = 0.82; }
  if (e.key === '2') mode = 'top';
  if (e.key === '3') mode = '3d';
});

function worldToScreen(p) {
  return [canvas.width / 2 + ox + p[0] * scale, canvas.height / 2 + oy - p[1] * scale];
}

function project3d(p) {
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const x1 = p[0] * cy - p[1] * sy;
  const y1 = p[0] * sy + p[1] * cy;
  const z1 = p[2] || 0;
  const y2 = y1 * cp - z1 * sp;
  const z2 = y1 * sp + z1 * cp;
  const depth = 70 + z2;
  const persp = 70 / Math.max(12, depth);
  return [
    canvas.width / 2 + ox + x1 * scale * persp,
    canvas.height / 2 + oy - y2 * scale * persp,
    persp,
    z1,
    depth
  ];
}

function zColor(z, base) {
  if (base === 'scan') return '#ffdc5e';
  const t = Math.max(0, Math.min(1, (z + 1.4) / 4.5));
  const r = Math.round(42 + 50 * t);
  const g = Math.round(130 + 105 * t);
  const b = Math.round(255 - 130 * t);
  return `rgb(${r},${g},${b})`;
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

function drawGrid3d() {
  ctx.strokeStyle = '#1f2730';
  ctx.lineWidth = 1;
  const extent = 12;
  for (let i = -extent; i <= extent; i++) {
    let a = project3d([i, -extent, 0]);
    let b = project3d([i, extent, 0]);
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
    a = project3d([-extent, i, 0]);
    b = project3d([extent, i, 0]);
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
  }
  const axes = [
    [[0,0,0], [2,0,0], '#ff5a5a'],
    [[0,0,0], [0,2,0], '#59e07b'],
    [[0,0,0], [0,0,2], '#58a6ff']
  ];
  ctx.lineWidth = 2;
  for (const [a0, b0, color] of axes) {
    const a = project3d(a0), b = project3d(b0);
    ctx.strokeStyle = color;
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
  }
}

function drawPoints(points, color, size) {
  ctx.fillStyle = color;
  for (const p of points) {
    const [x, y] = worldToScreen(p);
    if (x < -5 || y < -5 || x > canvas.width + 5 || y > canvas.height + 5) continue;
    ctx.fillRect(x, y, size, size);
  }
}

function drawPoints3d(points, kind, size, maxDraw) {
  if (!points || !points.length) return;
  const step = Math.max(1, Math.floor(points.length / maxDraw));
  const projected = [];
  for (let i = 0; i < points.length; i += step) {
    const pr = project3d(points[i]);
    if (pr[0] < -20 || pr[1] < -20 || pr[0] > canvas.width + 20 || pr[1] > canvas.height + 20) continue;
    projected.push([pr[0], pr[1], pr[2], pr[3], pr[4]]);
  }
  projected.sort((a, b) => b[4] - a[4]);
  for (const p of projected) {
    ctx.fillStyle = zColor(p[3], kind);
    const s = Math.max(1, size * p[2]);
    ctx.fillRect(p[0], p[1], s, s);
  }
}

function drawPath(path) {
  if (path.length < 2) return;
  ctx.strokeStyle = '#50e37f';
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  let [x0, y0] = worldToScreen(path[0]);
  ctx.moveTo(x0, y0);
  for (let i = 1; i < path.length; i++) {
    const [x, y] = worldToScreen(path[i]);
    ctx.lineTo(x, y);
  }
  ctx.stroke();
}

function drawPath3d(path) {
  if (path.length < 2) return;
  ctx.strokeStyle = '#50e37f';
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  let p0 = project3d([path[0][0], path[0][1], 0.05]);
  ctx.moveTo(p0[0], p0[1]);
  for (let i = 1; i < path.length; i++) {
    const p = project3d([path[i][0], path[i][1], 0.05]);
    ctx.lineTo(p[0], p[1]);
  }
  ctx.stroke();
}

function render() {
  ctx.clearRect(0,0,canvas.width,canvas.height);
  if (mode === '3d') {
    drawGrid3d();
    drawPoints3d(data.map || [], 'map', 2.0, 60000);
    drawPoints3d(data.scan || [], 'scan', 3.0, 5000);
    drawPath3d(data.path || []);
  } else {
    drawGrid();
    drawPoints(data.map || [], '#2e9cff', 1.4);
    drawPoints(data.scan || [], '#ffdc5e', 2.2);
    drawPath(data.path || []);
  }
  const s = data.stats || {};
  const age = s.last_update_age ?? 999;
  let cls = 'ok';
  if (s.tracking === 'weak') cls = 'warn';
  if (age > 2) cls = 'bad';
  statusEl.innerHTML = `<span class="${cls}">${s.tracking || 'waiting'}</span> ${mode.toUpperCase()} map=${s.map_points || 0} scan=${s.scan_points || 0} keyframes=${s.keyframes || 0} pose=(${(s.x || 0).toFixed(2)}, ${(s.y || 0).toFixed(2)}, ${(s.yaw_deg || 0).toFixed(1)} deg) err=${(s.error || 0).toFixed(3)} matches=${s.matches || 0}`;
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
  setTimeout(poll, 180);
}
poll();
</script>
</body>
</html>
"""


def yaw_to_matrix(yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def compose_pose(pose, delta):
    x, y, yaw = pose
    dx, dy, dyaw = delta
    rot = yaw_to_matrix(yaw)
    trans = rot @ np.array([dx, dy], dtype=np.float64)
    return np.array([x + trans[0], y + trans[1], yaw + dyaw], dtype=np.float64)


def transform_points(points, pose):
    if points.size == 0:
        return points
    rot = yaw_to_matrix(float(pose[2]))
    xy = points[:, :2] @ rot.T + pose[:2]
    out = points.copy()
    out[:, :2] = xy
    return out


def voxel_downsample(points, voxel, max_points, dims=2):
    if points.size == 0:
        return points
    keys = np.floor(points[:, :dims] / voxel).astype(np.int32)
    _, idx = np.unique(keys, axis=0, return_index=True)
    sampled = points[np.sort(idx)]
    if sampled.shape[0] > max_points:
        step = max(1, sampled.shape[0] // max_points)
        sampled = sampled[::step][:max_points]
    return sampled


def radius_outlier_filter(points, radius, min_neighbors):
    if points.shape[0] < min_neighbors:
        return points
    cell = radius
    keys = np.floor(points[:, :3] / cell).astype(np.int32)
    grid = {}
    for i, key in enumerate(keys):
        grid.setdefault(tuple(key), []).append(i)

    keep = []
    radius2 = radius * radius
    for i, key in enumerate(keys):
        count = 0
        p = points[i, :3]
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for j in grid.get((key[0] + dx, key[1] + dy, key[2] + dz), []):
                        if j == i:
                            continue
                        d = points[j, :3] - p
                        if float(d @ d) <= radius2:
                            count += 1
                            if count >= min_neighbors:
                                keep.append(i)
                                break
                    if count >= min_neighbors:
                        break
                if count >= min_neighbors:
                    break
            if count >= min_neighbors:
                break
    if len(keep) < max(30, min_neighbors):
        return points
    return points[np.asarray(keep, dtype=np.int32)]


def best_fit_transform_2d(src, dst):
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_c = src - src_mean
    dst_c = dst - dst_mean
    h = src_c.T @ dst_c
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = vt.T @ u.T
    t = dst_mean - r @ src_mean
    yaw = math.atan2(r[1, 0], r[0, 0])
    return r, t, yaw


def build_grid(points, cell):
    grid = {}
    for i, p in enumerate(points):
        key = (int(math.floor(p[0] / cell)), int(math.floor(p[1] / cell)))
        grid.setdefault(key, []).append(i)
    return grid


def nearest_pairs(src_transformed, target, grid, cell, max_dist):
    src_idx = []
    dst_idx = []
    max_d2 = max_dist * max_dist
    for i, p in enumerate(src_transformed):
        cx = int(math.floor(p[0] / cell))
        cy = int(math.floor(p[1] / cell))
        best_j = -1
        best_d2 = max_d2
        for gx in range(cx - 1, cx + 2):
            for gy in range(cy - 1, cy + 2):
                for j in grid.get((gx, gy), []):
                    d = target[j] - p
                    d2 = float(d[0] * d[0] + d[1] * d[1])
                    if d2 < best_d2:
                        best_d2 = d2
                        best_j = j
        if best_j >= 0:
            src_idx.append(i)
            dst_idx.append(best_j)
    return np.asarray(src_idx, dtype=np.int32), np.asarray(dst_idx, dtype=np.int32)


def icp_2d(source, target, iterations, cell, max_match_dist):
    if source.shape[0] < 40 or target.shape[0] < 40:
        return np.array([0.0, 0.0, 0.0], dtype=np.float64), 999.0, 0

    target_grid = build_grid(target, cell)
    r_total = np.eye(2, dtype=np.float64)
    t_total = np.zeros(2, dtype=np.float64)
    last_error = 999.0
    matches = 0

    for _ in range(iterations):
        moved = source @ r_total.T + t_total
        src_i, dst_i = nearest_pairs(moved, target, target_grid, cell, max_match_dist)
        matches = int(src_i.shape[0])
        if matches < 35:
            break

        r_inc, t_inc, _ = best_fit_transform_2d(moved[src_i], target[dst_i])
        r_total = r_inc @ r_total
        t_total = r_inc @ t_total + t_inc

        aligned = source[src_i] @ r_total.T + t_total
        residual = aligned - target[dst_i]
        last_error = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
        if last_error < 0.035:
            break

    yaw = math.atan2(r_total[1, 0], r_total[0, 0])
    return np.array([t_total[0], t_total[1], yaw], dtype=np.float64), last_error, matches


class SimpleIcpMapper(Node):
    def __init__(self, args):
        super().__init__("web_mid360_icp_mapper")
        self.args = args
        self.lock = threading.Lock()
        self.pose = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.prev_scan_xy = None
        self.latest_scan = []
        self.map_points = []
        self.map_voxels = {}
        self.path = [[0.0, 0.0]]
        self.frames = 0
        self.keyframes = 0
        self.matches = 0
        self.error = 0.0
        self.tracking = "waiting"
        self.last_update = 0.0
        self.last_process = 0.0
        self.start_wall = time.time()
        self.start_mono = time.monotonic()
        self.last_key_pose = self.pose.copy()

        self.map_pub = self.create_publisher(PointCloud2, "/simple_icp_map", 2)
        self.scan_pub = self.create_publisher(PointCloud2, "/simple_icp_scan", 2)
        self.path_pub = self.create_publisher(Path, "/simple_icp_path", 2)
        self.create_subscription(CustomMsg, args.topic, self.cloud_cb, 10)

    def cloud_cb(self, msg):
        now = time.monotonic()
        if now - self.last_process < self.args.period:
            return
        self.last_process = now

        points = self.extract_points(msg)
        if points.shape[0] < self.args.min_points:
            self.set_tracking("weak", 999.0, 0)
            return

        points = radius_outlier_filter(points, self.args.radius_filter, self.args.radius_min_neighbors)
        scan_icp = voxel_downsample(points, self.args.icp_voxel, self.args.icp_points, dims=2)
        if scan_icp.shape[0] < self.args.min_points:
            self.set_tracking("weak", 999.0, 0)
            return

        accepted = False
        delta = np.zeros(3, dtype=np.float64)
        error = 0.0
        matches = 0

        if self.prev_scan_xy is None:
            accepted = True
            self.prev_scan_xy = scan_icp[:, :2]
            self.tracking = "initialized"
        else:
            delta, error, matches = icp_2d(
                scan_icp[:, :2],
                self.prev_scan_xy,
                self.args.icp_iterations,
                self.args.match_cell,
                self.args.max_match_dist,
            )
            move = float(math.hypot(delta[0], delta[1]))
            turn = abs(float(delta[2]))
            if (
                matches >= self.args.min_matches
                and error <= self.args.max_error
                and move <= self.args.max_step
                and turn <= math.radians(self.args.max_turn_deg)
            ):
                self.prev_scan_xy = scan_icp[:, :2]
                accepted = True
                moving = move >= self.args.min_motion_dist or turn >= math.radians(self.args.min_motion_turn_deg)
                if moving:
                    self.pose = compose_pose(self.pose, delta)
                    self.tracking = "tracking"
                else:
                    self.tracking = "static"
            else:
                self.tracking = "weak"

        scan_global = transform_points(points, self.pose)
        scan_vis = voxel_downsample(scan_global, self.args.display_voxel, self.args.scan_points, dims=3)

        if accepted and self.tracking != "static":
            d_key = float(np.linalg.norm(self.pose[:2] - self.last_key_pose[:2]))
            a_key = abs(float(self.pose[2] - self.last_key_pose[2]))
            first_key = self.keyframes == 0
            if first_key or d_key >= self.args.keyframe_dist or a_key >= math.radians(self.args.keyframe_turn_deg):
                add = voxel_downsample(scan_global, self.args.map_voxel, self.args.map_add_points, dims=3)
                self.append_map(add)
                self.last_key_pose = self.pose.copy()
                self.keyframes += 1

            if len(self.path) == 0 or math.hypot(self.pose[0] - self.path[-1][0], self.pose[1] - self.path[-1][1]) >= 0.02:
                self.path.append([round(float(self.pose[0]), 3), round(float(self.pose[1]), 3)])
                if len(self.path) > 2000:
                    self.path = self.path[-2000:]

        with self.lock:
            self.latest_scan = [[round(float(p[0]), 3), round(float(p[1]), 3), round(float(p[2]), 3)] for p in scan_vis]
            self.frames += 1
            self.matches = matches
            self.error = error
            self.last_update = now

        self.publish_cloud("/simple_icp_scan", self.scan_pub, scan_vis)
        self.publish_cloud("/simple_icp_map", self.map_pub, np.asarray(self.map_points, dtype=np.float32))
        self.publish_path()

    def extract_points(self, msg):
        pts = []
        min_r2 = self.args.min_range * self.args.min_range
        max_r2 = self.args.max_range * self.args.max_range
        for p in msg.points:
            x = float(p.x)
            y = float(p.y)
            z = float(p.z)
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            r2 = x * x + y * y + z * z
            if r2 < min_r2 or r2 > max_r2:
                continue
            if z < self.args.min_z or z > self.args.max_z:
                continue
            if (p.tag & 0x30) not in (0x00, 0x10):
                continue
            pts.append((x, y, z))
        if not pts:
            return np.empty((0, 3), dtype=np.float64)
        return np.asarray(pts, dtype=np.float64)

    def append_map(self, points):
        if points.size == 0:
            return
        for p in points:
            key = tuple(np.floor(p[:3] / self.args.map_voxel).astype(np.int32))
            old = self.map_voxels.get(key)
            if old is None:
                self.map_voxels[key] = [float(p[0]), float(p[1]), float(p[2]), 1]
            else:
                count = min(old[3] + 1, 1000)
                alpha = 1.0 / count
                old[0] = old[0] * (1.0 - alpha) + float(p[0]) * alpha
                old[1] = old[1] * (1.0 - alpha) + float(p[1]) * alpha
                old[2] = old[2] * (1.0 - alpha) + float(p[2]) * alpha
                old[3] = count
        if len(self.map_voxels) > self.args.map_points * 2:
            items = sorted(self.map_voxels.items(), key=lambda kv: kv[1][3], reverse=True)
            self.map_voxels = dict(items[:self.args.map_points])

        stable = [
            [round(v[0], 3), round(v[1], 3), round(v[2], 3)]
            for v in self.map_voxels.values()
            if v[3] >= self.args.map_min_hits
        ]
        if len(stable) > self.args.map_points:
            stable = stable[-self.args.map_points:]
        with self.lock:
            self.map_points = stable

    def set_tracking(self, state, error, matches):
        with self.lock:
            self.tracking = state
            self.error = error
            self.matches = matches
            self.last_update = time.monotonic()

    def snapshot(self):
        with self.lock:
            return {
                "map": self.map_points,
                "scan": self.latest_scan,
                "path": self.path,
                "stats": {
                    "tracking": self.tracking,
                    "map_points": len(self.map_points),
                    "scan_points": len(self.latest_scan),
                    "frames": self.frames,
                    "keyframes": self.keyframes,
                    "matches": self.matches,
                    "error": self.error,
                    "x": float(self.pose[0]),
                    "y": float(self.pose[1]),
                    "yaw_deg": math.degrees(float(self.pose[2])),
                    "last_update_age": round(time.monotonic() - self.last_update, 2) if self.last_update else 999,
                },
            }

    def publish_cloud(self, frame_id, pub, points):
        if points.size == 0:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_init"
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = int(points.shape[0])
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = False
        msg.data = np.asarray(points[:, :3], dtype=np.float32).tobytes()
        pub.publish(msg)

    def publish_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "camera_init"
        for x, y in self.path:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)


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
    parser.add_argument("--topic", default="/livox/lidar_frame")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--period", type=float, default=0.18)
    parser.add_argument("--min-range", type=float, default=0.55)
    parser.add_argument("--max-range", type=float, default=25.0)
    parser.add_argument("--min-z", type=float, default=-2.0)
    parser.add_argument("--max-z", type=float, default=3.0)
    parser.add_argument("--min-points", type=int, default=80)
    parser.add_argument("--icp-voxel", type=float, default=0.18)
    parser.add_argument("--display-voxel", type=float, default=0.08)
    parser.add_argument("--map-voxel", type=float, default=0.12)
    parser.add_argument("--radius-filter", type=float, default=0.18)
    parser.add_argument("--radius-min-neighbors", type=int, default=2)
    parser.add_argument("--icp-points", type=int, default=1200)
    parser.add_argument("--scan-points", type=int, default=4000)
    parser.add_argument("--map-add-points", type=int, default=2500)
    parser.add_argument("--map-points", type=int, default=90000)
    parser.add_argument("--map-min-hits", type=int, default=2)
    parser.add_argument("--icp-iterations", type=int, default=8)
    parser.add_argument("--match-cell", type=float, default=0.45)
    parser.add_argument("--max-match-dist", type=float, default=0.55)
    parser.add_argument("--min-matches", type=int, default=70)
    parser.add_argument("--max-error", type=float, default=0.35)
    parser.add_argument("--max-step", type=float, default=0.75)
    parser.add_argument("--max-turn-deg", type=float, default=18.0)
    parser.add_argument("--min-motion-dist", type=float, default=0.08)
    parser.add_argument("--min-motion-turn-deg", type=float, default=2.5)
    parser.add_argument("--keyframe-dist", type=float, default=0.18)
    parser.add_argument("--keyframe-turn-deg", type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    node = SimpleIcpMapper(args)
    Handler.node = node
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"ICP web viewer: http://localhost:{args.port}")
    try:
        rclpy.spin(node)
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

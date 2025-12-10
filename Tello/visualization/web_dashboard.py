"""
Web-based dashboard for drone navigation visualization.

Serves an HTML dashboard at http://localhost:8080 showing:
- Live video feed
- Navigation map (canvas-based)
- Real-time statistics
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from typing import Dict, List, Optional
import math


class DashboardHandler(BaseHTTPRequestHandler):
    """HTTP handler for dashboard"""

    # Shared state
    navigation_state = {
        'current_pos': {'x': 0, 'y': 0, 'z': 0, 'yaw': 0},
        'target_pos': {'x': 0, 'y': 0, 'theta': 0},
        'trajectory': [],
        'obstacles': [],
        'status': 'READY',
        'stats': {}
    }
    video_stream_port = 8080  # Default video stream port
    state_lock = threading.Lock()

    def do_GET(self):
        """Handle GET requests"""
        if self.path == '/':
            self.serve_dashboard()
        elif self.path == '/video':
            self.serve_video()
        elif self.path == '/state':
            self.serve_state()
        else:
            self.send_error(404)

    def serve_dashboard(self):
        """Serve main dashboard HTML"""
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

        html = """
<!DOCTYPE html>
<html>
<head>
    <title>Tello Navigation Dashboard</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Arial, sans-serif;
            background: #1a1a1a;
            color: #fff;
            padding: 10px;
        }
        .header {
            text-align: center;
            padding: 15px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            border-radius: 10px;
            margin-bottom: 10px;
        }
        .header h1 { font-size: 24px; margin-bottom: 5px; }
        .status { font-size: 14px; color: #ffd700; }
        .container {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
            height: calc(100vh - 120px);
        }
        .panel {
            background: #2d2d2d;
            border-radius: 10px;
            padding: 15px;
            overflow: hidden;
        }
        .panel h2 {
            font-size: 16px;
            margin-bottom: 10px;
            color: #667eea;
            border-bottom: 2px solid #667eea;
            padding-bottom: 5px;
        }
        #video-panel img {
            width: 100%;
            height: calc(100% - 35px);
            object-fit: contain;
            background: #000;
            border-radius: 5px;
        }
        #map-canvas {
            width: 100%;
            height: calc(100% - 35px);
            background: #1a1a1a;
            border-radius: 5px;
        }
        #stats {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
            margin-top: 10px;
        }
        .stat-box {
            background: #3d3d3d;
            padding: 10px;
            border-radius: 5px;
            text-align: center;
        }
        .stat-label { font-size: 11px; color: #aaa; }
        .stat-value { font-size: 18px; font-weight: bold; color: #ffd700; }
        .obstacle-list {
            max-height: 150px;
            overflow-y: auto;
            margin-top: 10px;
        }
        .obstacle-item {
            padding: 5px;
            margin: 5px 0;
            background: #3d3d3d;
            border-radius: 3px;
            font-size: 12px;
        }
        .threat-high { border-left: 4px solid #ff4444; }
        .threat-medium { border-left: 4px solid #ffaa44; }
        .threat-low { border-left: 4px solid #44ff44; }
    </style>
</head>
<body>
    <div class="header">
        <h1>🚁 Tello Autonomous Navigation Dashboard</h1>
        <div class="status" id="status">Status: INITIALIZING</div>
    </div>

    <div class="container">
        <div class="panel" id="video-panel">
            <h2>📹 Live Video Feed</h2>
            <img id="video-feed" src="" alt="Video Stream">
        </div>

        <div class="panel">
            <h2>🗺️ Navigation Map</h2>
            <canvas id="map-canvas"></canvas>
        </div>

        <div class="panel">
            <h2>📊 Statistics</h2>
            <div id="stats">
                <div class="stat-box">
                    <div class="stat-label">Position</div>
                    <div class="stat-value" id="position">0, 0</div>
                </div>
                <div class="stat-box">
                    <div class="stat-label">Altitude</div>
                    <div class="stat-value" id="altitude">0 cm</div>
                </div>
                <div class="stat-box">
                    <div class="stat-label">Heading</div>
                    <div class="stat-value" id="heading">0°</div>
                </div>
                <div class="stat-box">
                    <div class="stat-label">Distance to Target</div>
                    <div class="stat-value" id="distance">-- cm</div>
                </div>
            </div>
        </div>

        <div class="panel">
            <h2>🚧 Detected Obstacles</h2>
            <div id="obstacle-list" class="obstacle-list"></div>
        </div>
    </div>

    <script>
        // Set video source to video stream port
        fetch('/state')
            .then(r => r.json())
            .then(state => {
                const videoPort = state.video_port || 8080;
                document.getElementById('video-feed').src = `http://${window.location.hostname}:${videoPort}/stream`;
            });

        const canvas = document.getElementById('map-canvas');
        const ctx = canvas.getContext('2d');

        // Set canvas size
        function resizeCanvas() {
            const rect = canvas.getBoundingClientRect();
            canvas.width = rect.width;
            canvas.height = rect.height;
        }
        resizeCanvas();
        window.addEventListener('resize', resizeCanvas);

        // Fetch state and update dashboard
        async function updateDashboard() {
            try {
                const response = await fetch('/state');
                const state = await response.json();

                // Update status
                document.getElementById('status').textContent = 'Status: ' + state.status;

                // Update stats
                const pos = state.current_pos;
                document.getElementById('position').textContent =
                    `${pos.x.toFixed(0)}, ${pos.y.toFixed(0)}`;
                document.getElementById('altitude').textContent = `${pos.z.toFixed(0)} cm`;
                document.getElementById('heading').textContent = `${pos.yaw.toFixed(0)}°`;

                // Calculate distance to target
                const dx = state.target_pos.x - pos.x;
                const dy = state.target_pos.y - pos.y;
                const dist = Math.sqrt(dx*dx + dy*dy);
                document.getElementById('distance').textContent = `${dist.toFixed(0)} cm`;

                // Update obstacles
                updateObstacleList(state.obstacles);

                // Draw map
                drawMap(state);

            } catch (e) {
                console.error('Failed to update:', e);
            }
        }

        function updateObstacleList(obstacles) {
            const list = document.getElementById('obstacle-list');
            if (obstacles.length === 0) {
                list.innerHTML = '<div style="text-align:center;color:#666;padding:20px;">No obstacles detected</div>';
                return;
            }

            list.innerHTML = obstacles.map(obs => `
                <div class="obstacle-item threat-${obs.threat_level}">
                    <strong>${obs.class}</strong> - ${obs.distance_m.toFixed(1)}m (${obs.position})
                </div>
            `).join('');
        }

        function drawMap(state) {
            const w = canvas.width;
            const h = canvas.height;

            // Clear
            ctx.fillStyle = '#1a1a1a';
            ctx.fillRect(0, 0, w, h);

            // Calculate scale
            const maxX = Math.max(state.target_pos.x, 300);
            const maxY = Math.max(state.target_pos.y, 300);
            const scale = Math.min(w / (maxX * 1.5), h / (maxY * 1.5));
            const offsetX = w / 2 - (state.target_pos.x / 2) * scale;
            const offsetY = h / 2 - (state.target_pos.y / 2) * scale;

            function toScreen(x, y) {
                return {
                    x: offsetX + x * scale,
                    y: offsetY + (maxY - y) * scale  // Flip Y
                };
            }

            // Grid
            ctx.strokeStyle = '#333';
            ctx.lineWidth = 1;
            for (let i = 0; i <= maxX; i += 50) {
                const p = toScreen(i, 0);
                ctx.beginPath();
                ctx.moveTo(p.x, 0);
                ctx.lineTo(p.x, h);
                ctx.stroke();
            }
            for (let i = 0; i <= maxY; i += 50) {
                const p = toScreen(0, i);
                ctx.beginPath();
                ctx.moveTo(0, p.y);
                ctx.lineTo(w, p.y);
                ctx.stroke();
            }

            // Start position
            const start = toScreen(0, 0);
            ctx.fillStyle = '#44ff44';
            ctx.beginPath();
            ctx.arc(start.x, start.y, 8, 0, Math.PI * 2);
            ctx.fill();

            // Target position
            const target = toScreen(state.target_pos.x, state.target_pos.y);
            ctx.fillStyle = '#ff4444';
            ctx.beginPath();
            ctx.moveTo(target.x, target.y - 12);
            ctx.lineTo(target.x - 10, target.y + 8);
            ctx.lineTo(target.x + 10, target.y + 8);
            ctx.closePath();
            ctx.fill();

            // Trajectory
            if (state.trajectory.length > 1) {
                ctx.strokeStyle = '#44ff44';
                ctx.lineWidth = 2;
                ctx.beginPath();
                const first = toScreen(state.trajectory[0][0], state.trajectory[0][1]);
                ctx.moveTo(first.x, first.y);
                for (let i = 1; i < state.trajectory.length; i++) {
                    const p = toScreen(state.trajectory[i][0], state.trajectory[i][1]);
                    ctx.lineTo(p.x, p.y);
                }
                ctx.stroke();
            }

            // Obstacles
            state.obstacles.forEach(obs => {
                const p = toScreen(obs.x, obs.y);
                const colors = {high: '#ff4444', medium: '#ffaa44', low: '#ffff44'};
                ctx.fillStyle = colors[obs.threat_level] || '#888';
                ctx.globalAlpha = 0.3;
                ctx.beginPath();
                ctx.arc(p.x, p.y, obs.width_m * 50 * scale, 0, Math.PI * 2);
                ctx.fill();
                ctx.globalAlpha = 1;
            });

            // Current position
            const curr = toScreen(state.current_pos.x, state.current_pos.y);
            ctx.fillStyle = '#4477ff';
            ctx.beginPath();
            ctx.arc(curr.x, curr.y, 10, 0, Math.PI * 2);
            ctx.fill();

            // Heading arrow
            const yawRad = (90 - state.current_pos.yaw) * Math.PI / 180;
            const arrowLen = 30;
            ctx.strokeStyle = '#4477ff';
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.moveTo(curr.x, curr.y);
            ctx.lineTo(curr.x + Math.cos(yawRad) * arrowLen,
                      curr.y - Math.sin(yawRad) * arrowLen);
            ctx.stroke();
        }

        // Update every 100ms
        setInterval(updateDashboard, 100);
        updateDashboard();
    </script>
</body>
</html>
        """
        self.wfile.write(html.encode())

    def serve_video(self):
        """Proxy video stream"""
        # Redirect to video stream endpoint
        self.send_response(302)
        self.send_header('Location', '/stream')
        self.end_headers()

    def serve_state(self):
        """Serve navigation state as JSON"""
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()

        with DashboardHandler.state_lock:
            state = DashboardHandler.navigation_state.copy()
            state['video_port'] = DashboardHandler.video_stream_port

        self.wfile.write(json.dumps(state).encode())

    def log_message(self, format, *args):
        """Suppress log messages"""
        pass


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Threaded HTTP server"""
    daemon_threads = True


class WebDashboard:
    """Web-based navigation dashboard"""

    def __init__(self, target_x, target_y, target_theta, port=8081, video_port=8080):
        self.target_x = target_x
        self.target_y = target_y
        self.target_theta = target_theta
        self.port = port
        self.video_port = video_port
        self.server = None
        self.server_thread = None

        # Initialize state
        with DashboardHandler.state_lock:
            DashboardHandler.navigation_state['target_pos'] = {
                'x': target_x,
                'y': target_y,
                'theta': target_theta
            }
            DashboardHandler.video_stream_port = video_port

    def start(self):
        """Start dashboard server"""
        self.server = ThreadedHTTPServer(('0.0.0.0', self.port), DashboardHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        print(f"[DASHBOARD] Web dashboard started at http://localhost:{self.port}")

    def stop(self):
        """Stop dashboard server"""
        if self.server:
            self.server.shutdown()
            self.server.server_close()

    def update_position(self, x, y, z, yaw):
        """Update drone position"""
        with DashboardHandler.state_lock:
            DashboardHandler.navigation_state['current_pos'] = {
                'x': x, 'y': y, 'z': z, 'yaw': yaw
            }
            # Add to trajectory
            traj = DashboardHandler.navigation_state['trajectory']
            traj.append((x, y))
            if len(traj) > 200:
                traj.pop(0)

    def add_obstacle(self, x, y, distance_m, width_m, threat_level, obj_class):
        """Add obstacle"""
        with DashboardHandler.state_lock:
            obs_list = DashboardHandler.navigation_state['obstacles']
            obs_list.append({
                'x': x, 'y': y,
                'distance_m': distance_m,
                'width_m': width_m,
                'threat_level': threat_level,
                'class': obj_class,
                'position': 'center'  # Simplified
            })
            # Keep only recent obstacles
            if len(obs_list) > 20:
                obs_list.pop(0)

    def set_status(self, status):
        """Update status"""
        with DashboardHandler.state_lock:
            DashboardHandler.navigation_state['status'] = status

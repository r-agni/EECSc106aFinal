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
        'waypoints': [],  # List of (x, y, z) waypoint tuples
        'current_waypoint_index': 0,  # Current target waypoint
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
            background: #000;
            color: #fff;
            overflow: hidden;
        }
        .header {
            text-align: center;
            padding: 10px;
            background: #000;
            border-bottom: 2px solid #667eea;
        }
        .header h1 { 
            font-size: 20px; 
            margin-bottom: 5px;
            color: #667eea;
        }
        .status { 
            font-size: 14px; 
            color: #ffd700;
            font-weight: bold;
        }
        .container {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 0;
            height: calc(100vh - 70px);
            background: #000;
        }
        .panel {
            background: #000;
            border: 1px solid #333;
            overflow: hidden;
            display: flex;
            flex-direction: column;
        }
        .panel-header {
            padding: 8px;
            background: #111;
            border-bottom: 1px solid #667eea;
            font-size: 14px;
            font-weight: bold;
            color: #667eea;
        }
        .panel-content {
            flex: 1;
            overflow: hidden;
            position: relative;
        }
        #video-feed {
            width: 100%;
            height: 100%;
            object-fit: contain;
            background: #000;
        }
        #map-canvas {
            width: 100%;
            height: 100%;
            background: #000;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>TELLO AUTONOMOUS NAVIGATION</h1>
        <div class="status" id="status">INITIALIZING</div>
    </div>

    <div class="container">
        <div class="panel">
            <div class="panel-header">LIVE VIDEO FEED</div>
            <div class="panel-content">
                <img id="video-feed" src="" alt="Video Stream">
            </div>
        </div>

        <div class="panel">
            <div class="panel-header">NAVIGATION MAP</div>
            <div class="panel-content">
                <canvas id="map-canvas"></canvas>
            </div>
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
                document.getElementById('status').textContent = state.status;

                // Draw map
                drawMap(state);

            } catch (e) {
                console.error('Failed to update:', e);
            }
        }


        function drawMap(state) {
            const w = canvas.width;
            const h = canvas.height;

            // Clear
            ctx.fillStyle = '#1a1a1a';
            ctx.fillRect(0, 0, w, h);

            // Calculate scale to fit everything with padding
            const maxX = Math.max(Math.abs(state.target_pos.x), Math.abs(state.current_pos.x), 300);
            const maxY = Math.max(Math.abs(state.target_pos.y), Math.abs(state.current_pos.y), 300);
            const padding = 50;
            const scale = Math.min((w - padding * 2) / (maxX * 2), (h - padding * 2) / (maxY * 2));
            const centerX = w / 2;
            const centerY = h / 2;

            function toScreen(x, y) {
                return {
                    x: centerX + x * scale,
                    y: centerY - y * scale  // Flip Y (screen Y increases downward)
                };
            }

            // Draw grid
            ctx.strokeStyle = '#1a1a1a';
            ctx.lineWidth = 1;
            const gridSize = 50;
            
            // Vertical lines
            for (let x = -maxX; x <= maxX; x += gridSize) {
                const p1 = toScreen(x, -maxY);
                const p2 = toScreen(x, maxY);
                ctx.beginPath();
                ctx.moveTo(p1.x, p1.y);
                ctx.lineTo(p2.x, p2.y);
                ctx.stroke();
            }
            
            // Horizontal lines
            for (let y = -maxY; y <= maxY; y += gridSize) {
                const p1 = toScreen(-maxX, y);
                const p2 = toScreen(maxX, y);
                ctx.beginPath();
                ctx.moveTo(p1.x, p1.y);
                ctx.lineTo(p2.x, p2.y);
                ctx.stroke();
            }
            
            // Draw axes
            ctx.strokeStyle = '#333';
            ctx.lineWidth = 2;
            // X axis
            const xAxis1 = toScreen(-maxX, 0);
            const xAxis2 = toScreen(maxX, 0);
            ctx.beginPath();
            ctx.moveTo(xAxis1.x, xAxis1.y);
            ctx.lineTo(xAxis2.x, xAxis2.y);
            ctx.stroke();
            // Y axis
            const yAxis1 = toScreen(0, -maxY);
            const yAxis2 = toScreen(0, maxY);
            ctx.beginPath();
            ctx.moveTo(yAxis1.x, yAxis1.y);
            ctx.lineTo(yAxis2.x, yAxis2.y);
            ctx.stroke();

            // Draw start position (origin)
            const start = toScreen(0, 0);
            ctx.fillStyle = '#00ff00';
            ctx.beginPath();
            ctx.arc(start.x, start.y, 10, 0, Math.PI * 2);
            ctx.fill();
            ctx.fillStyle = '#000';
            ctx.font = 'bold 12px Arial';
            ctx.textAlign = 'center';
            ctx.fillText('START', start.x, start.y + 4);

            // Draw target position
            const target = toScreen(state.target_pos.x, state.target_pos.y);
            ctx.fillStyle = '#ff0000';
            ctx.beginPath();
            ctx.arc(target.x, target.y, 10, 0, Math.PI * 2);
            ctx.fill();
            ctx.fillStyle = '#fff';
            ctx.font = 'bold 12px Arial';
            ctx.textAlign = 'center';
            ctx.fillText('TARGET', target.x, target.y + 4);

            // Draw trajectory path
            if (state.trajectory.length > 1) {
                ctx.strokeStyle = '#00ffff';
                ctx.lineWidth = 3;
                ctx.beginPath();
                const first = toScreen(state.trajectory[0][0], state.trajectory[0][1]);
                ctx.moveTo(first.x, first.y);
                for (let i = 1; i < state.trajectory.length; i++) {
                    const p = toScreen(state.trajectory[i][0], state.trajectory[i][1]);
                    ctx.lineTo(p.x, p.y);
                }
                ctx.stroke();
            }

            // Draw obstacles with age-based fading
            state.obstacles.forEach(obs => {
                const p = toScreen(obs.x || 0, obs.y || 0);
                const colors = {high: '#ff0000', medium: '#ff8800', low: '#ffff00'};
                const baseColor = colors[obs.threat_level] || '#888';

                // Calculate opacity based on age (fade over 5 seconds)
                const now = Date.now() / 1000;
                const age = obs.timestamp ? (now - obs.timestamp) : 0;
                const maxAge = 5.0;
                const opacity = Math.max(0.2, 1.0 - (age / maxAge));

                // Draw obstacle circle
                ctx.globalAlpha = opacity * 0.5;
                ctx.fillStyle = baseColor;
                ctx.beginPath();
                const radius = (obs.width_m || 0.5) * 100 * scale;
                ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
                ctx.fill();

                // Draw border
                ctx.globalAlpha = opacity;
                ctx.strokeStyle = baseColor;
                ctx.lineWidth = 2;
                ctx.stroke();
                ctx.globalAlpha = 1;

                // Label with class and distance
                ctx.fillStyle = '#fff';
                ctx.font = '10px Arial';
                ctx.textAlign = 'center';
                const label = `${obs.class || 'OBS'} (${obs.distance_m?.toFixed(1)}m)`;
                ctx.fillText(label, p.x, p.y - radius - 5);

                // Age indicator for old obstacles
                if (age > 2.0) {
                    ctx.fillStyle = '#888';
                    ctx.font = '8px Arial';
                    ctx.fillText(`${age.toFixed(0)}s ago`, p.x, p.y + radius + 12);
                }
            });

            // Draw waypoints and planned path
            if (state.waypoints && state.waypoints.length > 0) {
                // Draw dashed line connecting waypoints
                ctx.strokeStyle = '#00aaff';
                ctx.setLineDash([10, 5]);
                ctx.lineWidth = 2;
                ctx.beginPath();
                for (let i = 0; i < state.waypoints.length; i++) {
                    const wp = state.waypoints[i];
                    const p = toScreen(wp[0], wp[1]);
                    if (i === 0) ctx.moveTo(p.x, p.y);
                    else ctx.lineTo(p.x, p.y);
                }
                ctx.stroke();
                ctx.setLineDash([]);

                // Draw waypoint markers
                state.waypoints.forEach((wp, idx) => {
                    const p = toScreen(wp[0], wp[1]);
                    const currentIdx = state.current_waypoint_index || 0;

                    // Color code: green=completed, blue=current, gray=upcoming
                    if (idx < currentIdx) {
                        ctx.fillStyle = '#00ff0088';  // Completed
                    } else if (idx === currentIdx) {
                        ctx.fillStyle = '#00aaff';  // Current
                    } else {
                        ctx.fillStyle = '#888888';  // Upcoming
                    }

                    // Draw circle
                    ctx.beginPath();
                    ctx.arc(p.x, p.y, 8, 0, Math.PI * 2);
                    ctx.fill();
                    ctx.strokeStyle = '#fff';
                    ctx.lineWidth = 2;
                    ctx.stroke();

                    // Draw waypoint number
                    ctx.fillStyle = '#fff';
                    ctx.font = 'bold 10px Arial';
                    ctx.textAlign = 'center';
                    ctx.fillText((idx + 1).toString(), p.x, p.y + 3);

                    // Draw altitude label
                    ctx.fillStyle = '#00aaff';
                    ctx.font = '9px Arial';
                    ctx.fillText(`z=${wp[2]}cm`, p.x, p.y - 15);
                });
            }

            // Draw current drone position
            const curr = toScreen(state.current_pos.x, state.current_pos.y);

            // Drone body
            ctx.fillStyle = '#0088ff';
            ctx.beginPath();
            ctx.arc(curr.x, curr.y, 12, 0, Math.PI * 2);
            ctx.fill();
            ctx.strokeStyle = '#fff';
            ctx.lineWidth = 2;
            ctx.stroke();

            // Heading arrow
            const yawRad = state.current_pos.yaw * Math.PI / 180;
            const arrowLen = 35;
            ctx.strokeStyle = '#ffff00';
            ctx.lineWidth = 4;
            ctx.beginPath();
            ctx.moveTo(curr.x, curr.y);
            ctx.lineTo(curr.x + Math.sin(yawRad) * arrowLen,
                      curr.y - Math.cos(yawRad) * arrowLen);
            ctx.stroke();
            
            // Arrow head
            ctx.fillStyle = '#ffff00';
            ctx.beginPath();
            const headX = curr.x + Math.sin(yawRad) * arrowLen;
            const headY = curr.y - Math.cos(yawRad) * arrowLen;
            ctx.arc(headX, headY, 5, 0, Math.PI * 2);
            ctx.fill();
            
            // Position label
            ctx.fillStyle = '#fff';
            ctx.font = 'bold 11px Arial';
            ctx.textAlign = 'center';
            ctx.fillText(`(${state.current_pos.x.toFixed(0)}, ${state.current_pos.y.toFixed(0)})`, 
                        curr.x, curr.y + 25);
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
                'position': 'center',  # Simplified
                'timestamp': time.time()  # Track detection time
            })
            # Keep only recent obstacles
            if len(obs_list) > 20:
                obs_list.pop(0)

    def set_status(self, status):
        """Update status"""
        with DashboardHandler.state_lock:
            DashboardHandler.navigation_state['status'] = status

    def update_waypoints(self, waypoints, current_index=0):
        """
        Update waypoint visualization.

        Args:
            waypoints: List of (x, y, z) tuples representing waypoints
            current_index: Index of current target waypoint
        """
        with DashboardHandler.state_lock:
            DashboardHandler.navigation_state['waypoints'] = waypoints
            DashboardHandler.navigation_state['current_waypoint_index'] = current_index

import subprocess
import time
import cv2
import asyncio
import websockets
import json
import threading
from queue import Queue, Empty
from typing import Optional, Dict, Any, Set
from datetime import datetime
from djitellopy import Tello
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn


class StateManager:
    """Centralized state management with thread-safe access."""

    def __init__(self):
        self._lock = threading.Lock()
        self._state = {
            "battery": 0,
            "height": 0,
            "speed": {"x": 0, "y": 0, "z": 0},
            "orientation": {"pitch": 0, "roll": 0, "yaw": 0},
            "temperature": 0,
            "barometer": 0.0,
            "flight_time": 0,
            "is_flying": False,
            "connection_status": "disconnected",
            "last_command": None,
            "timestamp": time.time()
        }

    def update(self, **kwargs):
        """Update state with thread-safe access."""
        with self._lock:
            self._state.update(kwargs)
            self._state["timestamp"] = time.time()

    def get_state(self) -> Dict[str, Any]:
        """Get current state snapshot."""
        with self._lock:
            return self._state.copy()

    def set_flying(self, is_flying: bool):
        """Update flying status."""
        self.update(is_flying=is_flying)

    def set_connection_status(self, status: str):
        """Update connection status."""
        self.update(connection_status=status)

    def set_last_command(self, command: str):
        """Update last executed command."""
        self.update(last_command=command)


class DroneController:
    """Handles direct drone communication via djitellopy."""

    WIFI_PROFILE_NAME = "TELLO-5F897A"

    def __init__(self, state_manager: StateManager):
        self.state_manager = state_manager
        self.tello: Optional[Tello] = None
        self._command_lock = threading.Lock()

    def check_network_connection(self) -> bool:
        """Verify network connectivity to Tello."""
        print("[*] Checking network connectivity...")
        try:
            result = subprocess.run(
                ["ping", "-n", "1", "-w", "1000", "192.168.10.1"],
                capture_output=True,
                text=True,
                check=False,
            )
            if result.returncode == 0:
                print("[+] Network ping successful - Tello is reachable")
                return True
            else:
                print("[!] Cannot ping 192.168.10.1 - Tello may not be connected")
                return False
        except Exception as e:
            print(f"[!] Network check failed: {e}")
            return False

    def connect_wifi(self) -> bool:
        """Connect to Tello WiFi network."""
        print(f"[*] Connecting to Wi-Fi profile: {self.WIFI_PROFILE_NAME}...")
        print("[*] Make sure your Tello drone is powered ON!")

        try:
            result = subprocess.run(
                ["netsh", "wlan", "connect", f"name={self.WIFI_PROFILE_NAME}", "interface=Wi-Fi"],
                capture_output=True,
                text=True,
                check=False,
            )
            print(result.stdout)
            if result.returncode != 0:
                print("[!] netsh returned an error:")
                print(result.stderr)
                return False

            print("[*] Waiting for WiFi connection to stabilize (10 seconds)...")
            time.sleep(10)

            if self.check_network_connection():
                print("[+] Wi-Fi connected successfully")
                return True
            else:
                print("[!] WiFi command succeeded but cannot reach drone")
                print("[!] Please ensure:")
                print("    1. Tello drone is powered ON")
                print("    2. Tello WiFi is blinking (ready for connection)")
                print("    3. You're connected to the Tello network")
                return False

        except Exception as e:
            print(f"[!] Failed to run netsh: {e}")
            return False

    def connect_drone(self, retry_count: int = 3) -> bool:
        """Initialize Tello SDK connection with retry logic."""
        for attempt in range(1, retry_count + 1):
            try:
                print(f"[*] Initializing Tello connection (attempt {attempt}/{retry_count})...")
                self.tello = Tello()

                print("[*] Sending SDK mode command...")
                self.tello.connect()

                print("[*] Getting battery status...")
                battery = self.tello.get_battery()
                print(f"[+] Connected! Battery: {battery}%")

                # IMU CALIBRATION - Critical for rotation commands
                print("[*] Waiting for IMU calibration...")
                print("[!] IMPORTANT: Keep drone on a FLAT, STABLE surface!")
                print("    (This prevents 'No valid imu' errors during flight)")
                for i in range(8, 0, -1):
                    print(f"    Calibrating... {i}s", end="\r")
                    time.sleep(1)
                print("    Calibration complete!     ")

                self.state_manager.set_connection_status("connected")
                self.state_manager.update(battery=battery)

                if battery < 10:
                    print("[!] WARNING: Battery critically low! (<10%)")
                elif battery < 20:
                    print("[!] WARNING: Battery low (<20%)")

                return True

            except Exception as e:
                print(f"[!] Connection attempt {attempt} failed: {e}")
                if attempt < retry_count:
                    print(f"[*] Retrying in 3 seconds...")
                    time.sleep(3)
                else:
                    print("[!] All connection attempts failed")
                    print("[!] Troubleshooting tips:")
                    print("    1. Ensure Tello is powered ON and WiFi LED is blinking")
                    print("    2. Make sure you're connected to Tello WiFi network")
                    print("    3. Try restarting the Tello drone")
                    print("    4. Check if another program is using the Tello")
                    self.state_manager.set_connection_status("error")

        return False

    def start_video_stream(self):
        """Enable video streaming from drone."""
        if self.tello:
            try:
                self.tello.streamon()
                print("[+] Video stream enabled")
            except Exception as e:
                print(f"[!] Failed to start video stream: {e}")

    def stop_video_stream(self):
        """Disable video streaming."""
        if self.tello:
            try:
                self.tello.streamoff()
                print("[+] Video stream disabled")
            except Exception as e:
                print(f"[!] Failed to stop video stream: {e}")

    def send_command(self, command: str, **params) -> tuple[bool, str]:
        """Generic command dispatcher with error handling."""
        if not self.tello:
            return False, "Drone not connected"

        with self._command_lock:
            try:
                if command == "takeoff":
                    self.tello.takeoff()
                    self.state_manager.set_flying(True)
                    msg = "Takeoff successful"

                elif command == "land":
                    self.tello.land()
                    self.state_manager.set_flying(False)
                    msg = "Landing successful"

                elif command == "move_forward":
                    distance = params.get("distance", 20)
                    self.tello.move_forward(distance)
                    msg = f"Moved forward {distance}cm"

                elif command == "move_back":
                    distance = params.get("distance", 20)
                    self.tello.move_back(distance)
                    msg = f"Moved back {distance}cm"

                elif command == "move_left":
                    distance = params.get("distance", 20)
                    self.tello.move_left(distance)
                    msg = f"Moved left {distance}cm"

                elif command == "move_right":
                    distance = params.get("distance", 20)
                    self.tello.move_right(distance)
                    msg = f"Moved right {distance}cm"

                elif command == "move_up":
                    distance = params.get("distance", 20)
                    self.tello.move_up(distance)
                    msg = f"Moved up {distance}cm"

                elif command == "move_down":
                    distance = params.get("distance", 20)
                    self.tello.move_down(distance)
                    msg = f"Moved down {distance}cm"

                elif command == "rotate_cw":
                    degrees = params.get("degrees", 30)
                    # Add small delay before rotation to ensure IMU is stable
                    time.sleep(0.3)
                    self.tello.rotate_clockwise(degrees)
                    msg = f"Rotated clockwise {degrees}°"

                elif command == "rotate_ccw":
                    degrees = params.get("degrees", 30)
                    # Add small delay before rotation to ensure IMU is stable
                    time.sleep(0.3)
                    self.tello.rotate_counter_clockwise(degrees)
                    msg = f"Rotated counter-clockwise {degrees}°"

                elif command == "emergency":
                    self.tello.emergency()
                    self.state_manager.set_flying(False)
                    msg = "Emergency stop executed"

                else:
                    return False, f"Unknown command: {command}"

                self.state_manager.set_last_command(f"{command} {params}")
                print(f"[CMD] {msg}")
                return True, msg

            except Exception as e:
                error_msg = f"Command failed: {str(e)}"
                print(f"[!] {error_msg}")
                return False, error_msg

    def get_telemetry(self) -> Dict[str, Any]:
        """Collect all telemetry data from drone."""
        if not self.tello:
            return {}

        try:
            telemetry = {
                "battery": self.tello.get_battery(),
                "height": self.tello.get_height(),
                "temperature": self.tello.get_temperature(),
                "barometer": self.tello.get_barometer(),
                "flight_time": self.tello.get_flight_time(),
            }

            try:
                telemetry["speed"] = {
                    "x": self.tello.get_speed_x(),
                    "y": self.tello.get_speed_y(),
                    "z": self.tello.get_speed_z(),
                }
            except:
                telemetry["speed"] = {"x": 0, "y": 0, "z": 0}

            try:
                telemetry["orientation"] = {
                    "pitch": self.tello.get_pitch(),
                    "roll": self.tello.get_roll(),
                    "yaw": self.tello.get_yaw(),
                }
            except:
                telemetry["orientation"] = {"pitch": 0, "roll": 0, "yaw": 0}

            return telemetry

        except Exception as e:
            print(f"[!] Telemetry collection error: {e}")
            return {}

    def disconnect(self):
        """Graceful disconnect from drone."""
        if self.tello:
            try:
                self.tello.end()
                print("[+] Drone disconnected")
            except:
                pass
            self.tello = None
            self.state_manager.set_connection_status("disconnected")


class TelemetryCollector:
    """Continuously polls and broadcasts drone telemetry."""

    def __init__(self, drone_controller: DroneController, state_manager: StateManager, interval: float = 0.2):
        self.drone_controller = drone_controller
        self.state_manager = state_manager
        self.interval = interval
        self.running = False

    async def collect_loop(self):
        """Main telemetry collection loop."""
        self.running = True
        print(f"[*] Telemetry collection started (interval: {self.interval}s)")

        while self.running:
            try:
                telemetry = self.drone_controller.get_telemetry()
                if telemetry:
                    self.state_manager.update(**telemetry)
            except Exception as e:
                print(f"[!] Telemetry collection error: {e}")

            await asyncio.sleep(self.interval)

    def stop(self):
        """Stop telemetry collection."""
        self.running = False
        print("[*] Telemetry collection stopped")


class MJPEGStreamHandler(BaseHTTPRequestHandler):
    """HTTP handler for MJPEG streaming"""

    frame_queue = None  # Will be set by VideoStreamHandler

    def do_GET(self):
        """Handle HTTP GET requests"""
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            html = """
            <html>
            <head><title>Tello Video Stream</title></head>
            <body style="margin:0; padding:0; background:#000; display:flex; justify-content:center; align-items:center; height:100vh;">
                <div style="text-align:center;">
                    <h1 style="color:#fff; font-family:Arial;">Tello Drone Live Stream</h1>
                    <img src="/stream" style="max-width:90vw; max-height:80vh; border:2px solid #fff;"/>
                </div>
            </body>
            </html>
            """
            self.wfile.write(html.encode())

        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()

            try:
                while True:
                    if MJPEGStreamHandler.frame_queue is not None:
                        try:
                            # Get latest frame (non-blocking)
                            frame = MJPEGStreamHandler.frame_queue.get_nowait()

                            # Encode frame as JPEG
                            ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                            if ret:
                                self.wfile.write(b'--jpgboundary\r\n')
                                self.send_header('Content-type', 'image/jpeg')
                                self.send_header('Content-length', str(len(jpeg)))
                                self.end_headers()
                                self.wfile.write(jpeg.tobytes())
                                self.wfile.write(b'\r\n')
                        except Empty:
                            pass
                    time.sleep(0.033)  # ~30 FPS

            except (BrokenPipeError, ConnectionResetError):
                # Client disconnected
                pass
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        """Suppress access log messages"""
        pass


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread"""
    daemon_threads = True


class VideoStreamHandler:
    """Manages video stream from drone with threading."""

    def __init__(self, enable_http_stream: bool = True, http_port: int = 8080):
        self.capture_thread: Optional[threading.Thread] = None
        self.frame_queue = Queue(maxsize=2)
        self.http_frame_queue = Queue(maxsize=2)  # Separate queue for HTTP streaming
        self.running = False
        self.cap: Optional[cv2.VideoCapture] = None
        self.keyboard_queue = Queue()

        # HTTP streaming
        self.enable_http_stream = enable_http_stream
        self.http_port = http_port
        self.http_server: Optional[ThreadedHTTPServer] = None
        self.http_thread: Optional[threading.Thread] = None

    def start_stream(self):
        """Begin video capture in separate thread."""
        self.running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        print("[+] Video stream handler started")

        # Start HTTP streaming server
        if self.enable_http_stream:
            self._start_http_server()

    def _capture_loop(self):
        """Thread worker function for video capture."""
        print("[*] Opening video stream from udp://0.0.0.0:11111...")
        self.cap = cv2.VideoCapture("udp://0.0.0.0:11111", cv2.CAP_FFMPEG)

        if not self.cap.isOpened():
            print("[!] Could not open video stream")
            return

        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 240)
        print("[+] Video stream opened successfully")

        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue

            # Update main frame queue (for obstacle detection, etc.)
            if not self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except Empty:
                    pass
                self.frame_queue.put(frame)

            # Update HTTP frame queue (for web streaming)
            if self.enable_http_stream and not self.http_frame_queue.full():
                try:
                    self.http_frame_queue.get_nowait()
                except Empty:
                    pass
                self.http_frame_queue.put(frame.copy())

            cv2.imshow("Tello Stream", frame)

            key = cv2.waitKey(1) & 0xFF
            if key != 255:
                self.keyboard_queue.put(chr(key))

        self.cap.release()
        cv2.destroyAllWindows()
        print("[*] Video stream closed")

    def get_keyboard_input(self) -> Optional[str]:
        """Non-blocking keyboard input retrieval."""
        try:
            return self.keyboard_queue.get_nowait()
        except Empty:
            return None

    def _start_http_server(self):
        """Start HTTP streaming server in separate thread"""
        try:
            # Set the shared frame queue
            MJPEGStreamHandler.frame_queue = self.http_frame_queue

            # Create and start server
            self.http_server = ThreadedHTTPServer(('0.0.0.0', self.http_port), MJPEGStreamHandler)
            self.http_thread = threading.Thread(target=self.http_server.serve_forever, daemon=True)
            self.http_thread.start()

            print(f"[+] HTTP video stream started at http://0.0.0.0:{self.http_port}")
            print(f"[+] Open http://localhost:{self.http_port} in your browser to view the stream")
        except Exception as e:
            print(f"[!] Failed to start HTTP streaming server: {e}")

    def _stop_http_server(self):
        """Stop HTTP streaming server"""
        if self.http_server:
            try:
                self.http_server.shutdown()
                self.http_server.server_close()
                if self.http_thread:
                    self.http_thread.join(timeout=2.0)
                print("[+] HTTP video stream stopped")
            except Exception as e:
                print(f"[!] Error stopping HTTP server: {e}")

    def stop_stream(self):
        """Clean shutdown of video stream."""
        self.running = False

        # Stop HTTP server first
        if self.enable_http_stream:
            self._stop_http_server()

        # Stop video capture
        if self.capture_thread:
            self.capture_thread.join(timeout=2.0)

        print("[+] Video stream handler stopped")


class WebSocketServer:
    """WebSocket server for remote client connections."""

    def __init__(self, drone_controller: DroneController, state_manager: StateManager, host: str = "0.0.0.0", port: int = 8765):
        self.drone_controller = drone_controller
        self.state_manager = state_manager
        self.host = host
        self.port = port
        self.clients: Set[websockets.WebSocketServerProtocol] = set()
        self.server = None
        self.broadcast_task = None

    async def start(self):
        """Start WebSocket server."""
        async def handler(websocket):
            await self.handle_client(websocket)

        self.server = await websockets.serve(handler, self.host, self.port)
        print(f"[+] WebSocket server started on ws://{self.host}:{self.port}")

        self.broadcast_task = asyncio.create_task(self.broadcast_telemetry_loop())

    async def handle_client(self, websocket):
        """Handle individual client connection."""
        client_addr = websocket.remote_address
        print(f"[+] Client connected: {client_addr}")
        self.clients.add(websocket)

        try:
            async for message in websocket:
                await self.process_command(websocket, message)

        except websockets.exceptions.ConnectionClosed:
            print(f"[-] Client disconnected: {client_addr}")
        except Exception as e:
            print(f"[!] Error handling client {client_addr}: {e}")

        finally:
            self.clients.discard(websocket)

    async def process_command(self, websocket, message: str):
        """Parse and execute commands from clients."""
        try:
            data = json.loads(message)

            if data.get("type") == "command":
                command = data.get("command")
                params = data.get("params", {})
                request_id = data.get("request_id", "unknown")

                print(f"[NET CMD] {command} {params} (request_id: {request_id})")

                success, msg = self.drone_controller.send_command(command, **params)

                response = {
                    "type": "response",
                    "request_id": request_id,
                    "status": "success" if success else "error",
                    "message": msg
                }

                await websocket.send(json.dumps(response))

        except json.JSONDecodeError:
            error_response = {
                "type": "response",
                "status": "error",
                "message": "Invalid JSON"
            }
            await websocket.send(json.dumps(error_response))

        except Exception as e:
            error_response = {
                "type": "response",
                "status": "error",
                "message": str(e)
            }
            await websocket.send(json.dumps(error_response))

    async def broadcast_telemetry_loop(self):
        """Continuously broadcast telemetry to all connected clients."""
        print("[*] Telemetry broadcast started")

        while True:
            if self.clients:
                state = self.state_manager.get_state()
                telemetry_message = {
                    "type": "telemetry",
                    "timestamp": state["timestamp"],
                    "data": state
                }

                message = json.dumps(telemetry_message)

                disconnected_clients = set()
                for client in self.clients:
                    try:
                        await client.send(message)
                    except:
                        disconnected_clients.add(client)

                self.clients -= disconnected_clients

            await asyncio.sleep(0.2)

    async def stop(self):
        """Stop WebSocket server."""
        if self.broadcast_task:
            self.broadcast_task.cancel()

        if self.server:
            self.server.close()
            await self.server.wait_closed()

        print("[+] WebSocket server stopped")


class TelloDroneServer:
    """Main server coordinating all components."""

    def __init__(self):
        self.state_manager = StateManager()
        self.drone_controller = DroneController(self.state_manager)
        self.telemetry_collector = TelemetryCollector(self.drone_controller, self.state_manager)
        self.video_handler = VideoStreamHandler()
        self.websocket_server = WebSocketServer(self.drone_controller, self.state_manager)
        self.running = False

    async def initialize(self) -> bool:
        """Initialize drone connection and start services."""
        print("=" * 60)
        print("TELLO DRONE SERVER")
        print("=" * 60)

        if not self.drone_controller.connect_wifi():
            print("[!] WiFi connection failed")
            return False

        if not self.drone_controller.connect_drone():
            print("[!] Drone connection failed")
            return False

        self.drone_controller.start_video_stream()

        self.video_handler.start_stream()

        await self.websocket_server.start()

        asyncio.create_task(self.telemetry_collector.collect_loop())

        print("\n" + "=" * 60)
        print("SERVER READY")
        print("=" * 60)
        print("Keyboard controls:")
        print("  f - takeoff")
        print("  w/a/s/d - move forward/left/back/right (20cm)")
        print("  z/c - rotate counter-clockwise/clockwise (30°)")
        print("  l - land")
        print("  q - quit server")
        print(f"\nWebSocket server: ws://0.0.0.0:8765")
        print(f"HTTP video stream: http://localhost:8080")
        print("  (Open in browser to view live stream)")
        print("=" * 60 + "\n")

        return True

    async def run(self):
        """Main server loop."""
        if not await self.initialize():
            return

        self.running = True
        landed = True

        try:
            while self.running:
                key = self.video_handler.get_keyboard_input()

                if key:
                    if key == 'f' and landed:
                        self.drone_controller.send_command("takeoff")
                        landed = False

                    elif key == 'w' and not landed:
                        self.drone_controller.send_command("move_forward", distance=20)

                    elif key == 'a' and not landed:
                        self.drone_controller.send_command("move_left", distance=20)

                    elif key == 's' and not landed:
                        self.drone_controller.send_command("move_back", distance=20)

                    elif key == 'd' and not landed:
                        self.drone_controller.send_command("move_right", distance=20)

                    elif key == 'z' and not landed:
                        self.drone_controller.send_command("rotate_ccw", degrees=30)

                    elif key == 'c' and not landed:
                        self.drone_controller.send_command("rotate_cw", degrees=30)

                    elif key == 'l' and not landed:
                        self.drone_controller.send_command("land")
                        landed = True

                    elif key == 'q':
                        if not landed:
                            self.drone_controller.send_command("land")
                        self.running = False
                        break

                await asyncio.sleep(0.01)

        except KeyboardInterrupt:
            print("\n[*] Keyboard interrupt received")

        finally:
            await self.shutdown()

    async def shutdown(self):
        """Graceful shutdown of all components."""
        print("\n[*] Shutting down server...")

        self.telemetry_collector.stop()
        await self.websocket_server.stop()
        self.drone_controller.stop_video_stream()
        self.video_handler.stop_stream()
        self.drone_controller.disconnect()

        print("[+] Server shutdown complete")


async def main():
    """Entry point."""
    server = TelloDroneServer()
    await server.run()


if __name__ == "__main__":
    asyncio.run(main())

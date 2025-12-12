"""
Simplified drone wrapper utilities for video streaming and connection management.
Uses DJITelloPy API directly.
"""

import subprocess
import time
import cv2
import threading
from queue import Queue, Empty
from typing import Optional, Callable
from djitellopy import Tello
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn


class TelloConnection:
    """Helper functions for connecting to Tello drone"""

    WIFI_PROFILE_NAME = "TELLO-5F897A"

    @staticmethod
    def check_network_connection() -> bool:
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

    @staticmethod
    def connect_wifi(profile_name: str = None) -> bool:
        """Connect to Tello WiFi network."""
        if profile_name is None:
            profile_name = TelloConnection.WIFI_PROFILE_NAME

        print(f"[*] Connecting to Wi-Fi profile: {profile_name}...")
        print("[*] Make sure your Tello drone is powered ON!")

        try:
            result = subprocess.run(
                ["netsh", "wlan", "connect", f"name={profile_name}", "interface=Wi-Fi"],
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

            if TelloConnection.check_network_connection():
                print("[+] Wi-Fi connected successfully")
                return True
            else:
                print("[!] WiFi command succeeded but cannot reach drone")
                return False

        except Exception as e:
            print(f"[!] Failed to run netsh: {e}")
            return False


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Threaded HTTP server for handling multiple simultaneous connections"""
    daemon_threads = True
    allow_reuse_address = True


class MJPEGStreamHandler(BaseHTTPRequestHandler):
    """HTTP handler for MJPEG video streaming"""

    frame_queue: Optional[Queue] = None

    def log_message(self, format, *args):
        """Suppress HTTP request logging"""
        pass

    def do_GET(self):
        """Handle GET requests for video stream"""
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()

            try:
                while True:
                    if self.frame_queue:
                        try:
                            frame = self.frame_queue.get(timeout=1)
                            _, jpg = cv2.imencode('.jpg', frame)
                            self.wfile.write(b"--jpgboundary\r\n")
                            self.send_header('Content-type', 'image/jpeg')
                            self.send_header('Content-length', str(len(jpg)))
                            self.end_headers()
                            self.wfile.write(jpg.tobytes())
                            self.wfile.write(b"\r\n")
                        except Empty:
                            continue
                    else:
                        time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_response(404)
            self.end_headers()


class VideoStreamHandler:
    """Manages video stream from Tello drone with HTTP streaming support"""

    def __init__(self, tello: Tello, enable_http_stream: bool = True, http_port: int = 8080):
        """
        Initialize video stream handler.

        Args:
            tello: DJITelloPy Tello instance
            enable_http_stream: Enable HTTP video streaming
            http_port: Port for HTTP video stream
        """
        self.tello = tello
        self.capture_thread: Optional[threading.Thread] = None
        self.frame_queue = Queue(maxsize=2)
        self.http_frame_queue = Queue(maxsize=2)
        self.running = False
        self.cap: Optional[cv2.VideoCapture] = None

        # HTTP streaming
        self.enable_http_stream = enable_http_stream
        self.http_port = http_port
        self.http_server: Optional[ThreadedHTTPServer] = None
        self.http_thread: Optional[threading.Thread] = None
        self.overlay_processor: Optional[Callable] = None

    def set_overlay_processor(self, processor_func: Callable):
        """Set function to process frames with overlays before streaming"""
        self.overlay_processor = processor_func
        print("[+] Video overlay processor enabled")

    def start_stream(self):
        """Begin video capture in separate thread"""
        self.running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        print("[+] Video stream handler started")

        # Start HTTP streaming server
        if self.enable_http_stream:
            self._start_http_server()

    def _capture_loop(self):
        """Thread worker function for video capture"""
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
                # Apply overlays if processor is set
                display_frame = frame.copy()
                if self.overlay_processor:
                    try:
                        display_frame = self.overlay_processor(display_frame)
                    except Exception as e:
                        print(f"[!] Overlay error: {e}")

                try:
                    self.http_frame_queue.get_nowait()
                except Empty:
                    pass
                self.http_frame_queue.put(display_frame)

            cv2.imshow("Tello Stream", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC key
                break

        self.cap.release()
        cv2.destroyAllWindows()
        print("[*] Video stream closed")

    def _start_http_server(self):
        """Start HTTP streaming server in separate thread"""
        try:
            # Set the shared frame queue
            MJPEGStreamHandler.frame_queue = self.http_frame_queue

            # Create and start server
            self.http_server = ThreadedHTTPServer(('0.0.0.0', self.http_port), MJPEGStreamHandler)
            self.http_thread = threading.Thread(target=self.http_server.serve_forever, daemon=True)
            self.http_thread.start()
            print(f"[+] HTTP video stream available at http://localhost:{self.http_port}/stream")
        except Exception as e:
            print(f"[!] Failed to start HTTP server: {e}")

    def stop_stream(self):
        """Stop video capture and HTTP server"""
        self.running = False

        if self.http_server:
            self.http_server.shutdown()
            print("[+] HTTP server stopped")

        if self.capture_thread:
            self.capture_thread.join(timeout=2)

        print("[+] Video stream stopped")

    def get_latest_frame(self):
        """Get the most recent frame from the stream"""
        try:
            return self.frame_queue.get_nowait()
        except Empty:
            return None

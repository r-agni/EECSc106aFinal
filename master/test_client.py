import asyncio
import websockets
import json
import sys
from datetime import datetime


class TelloTestClient:
    """Simple WebSocket client for testing Tello drone server."""

    def __init__(self, uri: str = "ws://localhost:8765"):
        self.uri = uri
        self.websocket = None
        self.request_counter = 0

    async def connect(self):
        """Connect to WebSocket server."""
        try:
            self.websocket = await websockets.connect(self.uri)
            print(f"[+] Connected to {self.uri}")
            return True
        except Exception as e:
            print(f"[!] Connection failed: {e}")
            return False

    async def send_command(self, command: str, **params):
        """Send a command to the drone."""
        if not self.websocket:
            print("[!] Not connected")
            return

        self.request_counter += 1
        message = {
            "type": "command",
            "command": command,
            "params": params,
            "request_id": f"req_{self.request_counter}"
        }

        try:
            await self.websocket.send(json.dumps(message))
            print(f"[>>>] Sent: {command} {params}")

            response = await asyncio.wait_for(self.websocket.recv(), timeout=5.0)
            response_data = json.loads(response)

            if response_data.get("type") == "response":
                status = response_data.get("status")
                msg = response_data.get("message")
                print(f"[<<<] Response: {status} - {msg}")
            else:
                print(f"[<<<] Received: {response_data}")

        except asyncio.TimeoutError:
            print("[!] Response timeout")
        except Exception as e:
            print(f"[!] Error: {e}")

    async def listen_telemetry(self):
        """Listen for telemetry broadcasts."""
        if not self.websocket:
            print("[!] Not connected")
            return

        print("\n[*] Listening for telemetry (Ctrl+C to stop)...\n")
        try:
            async for message in self.websocket:
                data = json.loads(message)

                if data.get("type") == "telemetry":
                    telemetry = data.get("data", {})
                    timestamp = datetime.fromtimestamp(data.get("timestamp", 0)).strftime("%H:%M:%S")

                    print(f"[{timestamp}] Battery: {telemetry.get('battery')}% | "
                          f"Height: {telemetry.get('height')}cm | "
                          f"Temp: {telemetry.get('temperature')}°C | "
                          f"Flying: {telemetry.get('is_flying')} | "
                          f"Status: {telemetry.get('connection_status')}")

        except KeyboardInterrupt:
            print("\n[*] Stopped listening")

    async def interactive_mode(self):
        """Interactive command-line interface."""
        print("\n" + "=" * 60)
        print("TELLO TEST CLIENT - Interactive Mode")
        print("=" * 60)
        print("Commands:")
        print("  takeoff          - Take off")
        print("  land             - Land")
        print("  forward [dist]   - Move forward (default: 20cm)")
        print("  back [dist]      - Move back (default: 20cm)")
        print("  left [dist]      - Move left (default: 20cm)")
        print("  right [dist]     - Move right (default: 20cm)")
        print("  up [dist]        - Move up (default: 20cm)")
        print("  down [dist]      - Move down (default: 20cm)")
        print("  cw [degrees]     - Rotate clockwise (default: 30°)")
        print("  ccw [degrees]    - Rotate counter-clockwise (default: 30°)")
        print("  telemetry        - Listen to telemetry stream")
        print("  emergency        - Emergency stop")
        print("  quit             - Exit")
        print("=" * 60 + "\n")

        while True:
            try:
                user_input = input(">>> ").strip().lower()

                if not user_input:
                    continue

                parts = user_input.split()
                cmd = parts[0]

                if cmd == "quit":
                    break

                elif cmd == "takeoff":
                    await self.send_command("takeoff")

                elif cmd == "land":
                    await self.send_command("land")

                elif cmd == "forward":
                    dist = int(parts[1]) if len(parts) > 1 else 20
                    await self.send_command("move_forward", distance=dist)

                elif cmd == "back":
                    dist = int(parts[1]) if len(parts) > 1 else 20
                    await self.send_command("move_back", distance=dist)

                elif cmd == "left":
                    dist = int(parts[1]) if len(parts) > 1 else 20
                    await self.send_command("move_left", distance=dist)

                elif cmd == "right":
                    dist = int(parts[1]) if len(parts) > 1 else 20
                    await self.send_command("move_right", distance=dist)

                elif cmd == "up":
                    dist = int(parts[1]) if len(parts) > 1 else 20
                    await self.send_command("move_up", distance=dist)

                elif cmd == "down":
                    dist = int(parts[1]) if len(parts) > 1 else 20
                    await self.send_command("move_down", distance=dist)

                elif cmd == "cw":
                    degrees = int(parts[1]) if len(parts) > 1 else 30
                    await self.send_command("rotate_cw", degrees=degrees)

                elif cmd == "ccw":
                    degrees = int(parts[1]) if len(parts) > 1 else 30
                    await self.send_command("rotate_ccw", degrees=degrees)

                elif cmd == "telemetry":
                    await self.listen_telemetry()

                elif cmd == "emergency":
                    await self.send_command("emergency")

                else:
                    print(f"[!] Unknown command: {cmd}")

            except KeyboardInterrupt:
                print("\n[*] Exiting...")
                break
            except Exception as e:
                print(f"[!] Error: {e}")

    async def close(self):
        """Close WebSocket connection."""
        if self.websocket:
            await self.websocket.close()
            print("[+] Disconnected")


async def main():
    """Entry point."""
    uri = "ws://localhost:8765"
    if len(sys.argv) > 1:
        uri = sys.argv[1]

    client = TelloTestClient(uri)

    if await client.connect():
        await client.interactive_mode()
        await client.close()


if __name__ == "__main__":
    asyncio.run(main())

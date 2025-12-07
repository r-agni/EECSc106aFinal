"""
Test script for autonomous navigation with obstacle avoidance.

This script demonstrates the path planning system with predefined test paths.
"""

import sys
import asyncio
import time

# Add parent directory to path to import tello_server
sys.path.append('..')

from tello_server import DroneController, StateManager, VideoStreamHandler
from obstacle_avoidance.path_executor import PathExecutor
from obstacle_avoidance.config import Config


# ==================== TEST PATH DEFINITIONS ====================

# Scenario 1: Simple forward path (straight line)
test_path_forward = [
    (0, 0, 100),      # Takeoff to 100cm height
    (200, 0, 100),    # Move forward 200cm
    (400, 0, 100),    # Continue forward 200cm more
    (400, 0, 0)       # Land at destination
]

# Scenario 2: L-shaped path (turn corner)
test_path_l_shape = [
    (0, 0, 100),       # Takeoff
    (150, 0, 100),     # Forward 150cm
    (150, 150, 100),   # Right 150cm (turn corner)
    (150, 150, 0)      # Land
]

# Scenario 3: Square circuit (return to start)
test_path_square = [
    (0, 0, 100),       # Takeoff
    (100, 0, 100),     # North 100cm
    (100, 100, 100),   # East 100cm
    (0, 100, 100),     # South 100cm
    (0, 0, 100),       # West 100cm (back to start)
    (0, 0, 0)          # Land
]

# Scenario 4: Short test (for quick testing)
test_path_short = [
    (0, 0, 100),       # Takeoff
    (100, 0, 100),     # Forward 100cm
    (100, 0, 0)        # Land
]


def print_banner(text: str):
    """Print formatted banner"""
    width = 60
    print("\n" + "="*width)
    print(text.center(width))
    print("="*width + "\n")


def main():
    """Main test function"""

    print_banner("TELLO AUTONOMOUS NAVIGATION TEST")

    # Initialize configuration
    config = Config()

    # Initialize drone components
    print("[*] Initializing drone components...")
    state_mgr = StateManager()
    drone_ctrl = DroneController(state_mgr)
    video_handler = VideoStreamHandler()

    # Connect to drone
    print("\n[STEP 1] Connecting to drone...")
    if not drone_ctrl.connect_wifi():
        print("[!] WiFi connection failed - exiting")
        return

    if not drone_ctrl.connect_drone():
        print("[!] Drone connection failed - exiting")
        return

    # Start video stream
    print("\n[STEP 2] Starting video stream...")
    drone_ctrl.start_video_stream()
    video_handler.start_stream()
    time.sleep(2)  # Wait for video stream to stabilize

    # Initialize path executor
    print("\n[STEP 3] Initializing path executor...")
    executor = PathExecutor(drone_ctrl, state_mgr, video_handler, config)

    # Select test path
    print("\n[STEP 4] Select test scenario:")
    print("  1. Forward path (400cm straight line)")
    print("  2. L-shaped path (150cm forward, 150cm right)")
    print("  3. Square circuit (100cm x 100cm square)")
    print("  4. Short test (100cm forward only)")

    choice = input("\nEnter choice (1-4): ").strip()

    paths = {
        "1": ("Forward Path", test_path_forward),
        "2": ("L-Shaped Path", test_path_l_shape),
        "3": ("Square Circuit", test_path_square),
        "4": ("Short Test", test_path_short)
    }

    if choice not in paths:
        print("[!] Invalid choice - defaulting to short test")
        choice = "4"

    path_name, selected_path = paths[choice]

    print_banner(f"EXECUTING: {path_name}")

    print("Path waypoints:")
    for i, wp in enumerate(selected_path):
        print(f"  {i+1}. ({wp[0]:.0f}, {wp[1]:.0f}, {wp[2]:.0f}) cm")

    print("\n[!] IMPORTANT SAFETY NOTES:")
    print("  - Ensure clear flight area (at least 2m x 2m)")
    print("  - Remove obstacles from flight path")
    print("  - System will auto-land if battery < 20%")
    print("  - Press Ctrl+C for emergency stop")

    input("\nPress ENTER to start autonomous mission...")

    # Execute autonomous mission
    print_banner("STARTING AUTONOMOUS NAVIGATION")
    print("[*] Mission started - monitoring for obstacles...")
    print("[*] Press Ctrl+C at any time for emergency stop\n")

    success = executor.execute_path(selected_path)

    # Report results
    if success:
        print_banner("MISSION COMPLETED SUCCESSFULLY")
        print("[+] All waypoints reached")
        print("[+] Obstacle avoidance: OPERATIONAL")
        print("[+] Position tracking: ACTIVE")
    else:
        print_banner("MISSION ABORTED")
        print("[!] Mission did not complete successfully")
        print("[!] Check logs above for details")

    # Cleanup
    print("\n[*] Cleaning up...")
    video_handler.stop_stream()
    drone_ctrl.stop_video_stream()
    drone_ctrl.disconnect()

    print("\n[+] Test completed - system shutdown")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n[!] Keyboard interrupt received")
        print("[!] Emergency landing...")
        # Try to land if possible
        try:
            state_mgr = StateManager()
            drone_ctrl = DroneController(state_mgr)
            drone_ctrl.tello.land() if drone_ctrl.tello else None
        except:
            pass
        print("[!] System shutdown")
    except Exception as e:
        print(f"\n[!] Fatal error: {e}")
        import traceback
        traceback.print_exc()
        print("\n[!] Attempting emergency landing...")
        try:
            state_mgr = StateManager()
            drone_ctrl = DroneController(state_mgr)
            drone_ctrl.tello.land() if drone_ctrl.tello else None
        except:
            pass

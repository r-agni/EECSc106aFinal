"""
Example usage of path planning system.

This script demonstrates how to run the complete ArUco tag discovery
and path planning mission.
"""

import sys
sys.path.append('..')

from path_planning import MissionController, PathPlanningConfig
from tello_server import DroneController, StateManager, VideoStreamHandler


def main():
    """Run path planning mission"""
    print("="*60)
    print("  ARUCO TAG DISCOVERY & PATH PLANNING")
    print("="*60)

    # Initialize state manager
    state_manager = StateManager()

    # Initialize drone controller
    print("\n[SETUP] Initializing drone controller...")
    drone_controller = DroneController(state_manager)

    # Connect to drone
    print("[SETUP] Connecting to Tello...")
    if not drone_controller.connect():
        print("[ERROR] Failed to connect to drone!")
        return

    print("[SETUP] Connected successfully!")

    # Initialize video stream
    print("[SETUP] Starting video stream...")
    video_handler = VideoStreamHandler()
    video_handler.start()

    # Load configuration from .env
    print("[SETUP] Loading configuration from .env...")
    config = PathPlanningConfig()
    print(config)

    # Create mission controller
    print("[SETUP] Creating mission controller...")
    mission = MissionController(
        drone_controller,
        state_manager,
        video_handler,
        config
    )

    try:
        # Run complete mission
        print("\n[MISSION] Starting mission...")
        print("Make sure ArUco tags are placed in the search area!")
        input("Press Enter to start mission...")

        success = mission.run_mission()

        if success:
            print("\n✓ Mission completed successfully!")
        else:
            print("\n✗ Mission failed!")

    except KeyboardInterrupt:
        print("\n[MISSION] User interrupted!")
        mission.stop()

    except Exception as e:
        print(f"\n[ERROR] Mission error: {e}")
        import traceback
        traceback.print_exc()
        mission.stop()

    finally:
        # Cleanup
        print("\n[CLEANUP] Stopping video stream...")
        video_handler.stop()

        print("[CLEANUP] Disconnecting from drone...")
        # Drone will auto-disconnect

        print("[CLEANUP] Done!")


if __name__ == "__main__":
    main()

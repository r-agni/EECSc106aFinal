"""
AR Drone API Usage Examples

This file demonstrates how to use all the functions available in the DroneAPI class.
"""

import time
import cv2
from drone_api import DroneAPI


# ==================== EXAMPLE 1: Basic Flight ====================

def example_basic_flight():
    """Simple takeoff, hover, and land sequence."""
    print("\n=== EXAMPLE 1: Basic Flight ===\n")

    with DroneAPI() as drone:
        # Take off
        drone.takeoff()
        time.sleep(5)  # Wait for takeoff to complete

        # Hover in place
        print("Hovering for 5 seconds...")
        for _ in range(50):
            drone.hover()
            time.sleep(0.1)

        # Land
        drone.land()
        time.sleep(3)


# ==================== EXAMPLE 2: Simple Movement ====================

def example_simple_movement():
    """Demonstrate basic movement commands."""
    print("\n=== EXAMPLE 2: Simple Movement ===\n")

    with DroneAPI() as drone:
        drone.takeoff()
        time.sleep(5)

        # Move forward
        print("Moving forward...")
        for _ in range(20):
            drone.move_forward(speed=0.3)
            time.sleep(0.1)

        # Hover
        print("Hovering...")
        for _ in range(10):
            drone.hover()
            time.sleep(0.1)

        # Move backward
        print("Moving backward...")
        for _ in range(20):
            drone.move_backward(speed=0.3)
            time.sleep(0.1)

        # Land
        drone.land()
        time.sleep(3)


# ==================== EXAMPLE 3: Complete Movement Demo ====================

def example_complete_movement():
    """Demonstrate all movement directions."""
    print("\n=== EXAMPLE 3: Complete Movement Demo ===\n")

    with DroneAPI() as drone:
        drone.takeoff()
        time.sleep(5)

        move_duration = 1.5  # seconds per movement

        # Forward
        print("Moving forward...")
        end_time = time.time() + move_duration
        while time.time() < end_time:
            drone.move_forward(0.4)
            time.sleep(0.05)

        # Hover
        for _ in range(10):
            drone.hover()
            time.sleep(0.1)

        # Backward
        print("Moving backward...")
        end_time = time.time() + move_duration
        while time.time() < end_time:
            drone.move_backward(0.4)
            time.sleep(0.05)

        # Hover
        for _ in range(10):
            drone.hover()
            time.sleep(0.1)

        # Left
        print("Moving left...")
        end_time = time.time() + move_duration
        while time.time() < end_time:
            drone.move_left(0.4)
            time.sleep(0.05)

        # Hover
        for _ in range(10):
            drone.hover()
            time.sleep(0.1)

        # Right
        print("Moving right...")
        end_time = time.time() + move_duration
        while time.time() < end_time:
            drone.move_right(0.4)
            time.sleep(0.05)

        # Hover
        for _ in range(10):
            drone.hover()
            time.sleep(0.1)

        # Rotate clockwise
        print("Rotating clockwise...")
        end_time = time.time() + move_duration
        while time.time() < end_time:
            drone.rotate_clockwise(0.5)
            time.sleep(0.05)

        # Hover
        for _ in range(10):
            drone.hover()
            time.sleep(0.1)

        # Rotate counter-clockwise
        print("Rotating counter-clockwise...")
        end_time = time.time() + move_duration
        while time.time() < end_time:
            drone.rotate_counterclockwise(0.5)
            time.sleep(0.05)

        # Land
        drone.land()
        time.sleep(3)


# ==================== EXAMPLE 4: Square Pattern Flight ====================

def example_square_pattern():
    """Fly in a square pattern."""
    print("\n=== EXAMPLE 4: Square Pattern Flight ===\n")

    with DroneAPI() as drone:
        drone.takeoff()
        time.sleep(5)

        side_duration = 2.0  # seconds per side

        for side in range(4):
            print(f"Flying side {side + 1} of square...")

            # Move forward for one side
            end_time = time.time() + side_duration
            while time.time() < end_time:
                drone.move_forward(0.4)
                time.sleep(0.05)

            # Hover
            for _ in range(5):
                drone.hover()
                time.sleep(0.1)

            # Rotate 90 degrees clockwise
            print("Rotating 90 degrees...")
            end_time = time.time() + 1.0
            while time.time() < end_time:
                drone.rotate_clockwise(0.5)
                time.sleep(0.05)

            # Hover
            for _ in range(5):
                drone.hover()
                time.sleep(0.1)

        drone.land()
        time.sleep(3)


# ==================== EXAMPLE 5: Video Streaming ====================

def example_video_streaming():
    """Stream and display video from drone."""
    print("\n=== EXAMPLE 5: Video Streaming ===\n")

    drone = DroneAPI()

    # Wait for video to be ready
    print("Waiting for video stream...")
    if drone.wait_for_video(timeout=10.0):
        print("Video stream ready!")
    else:
        print("Video stream not available")
        return

    drone.takeoff()
    time.sleep(5)

    print("Displaying video for 10 seconds (press ESC to exit early)...")
    start_time = time.time()

    try:
        while time.time() - start_time < 10.0:
            # Maintain hover
            drone.hover()

            # Display video
            key = drone.display_video()

            # Check for ESC key
            if key == 27:
                print("ESC pressed, exiting...")
                break

            time.sleep(0.03)

    finally:
        drone.land()
        time.sleep(3)
        drone.close()


# ==================== EXAMPLE 6: Telemetry Monitoring ====================

def example_telemetry():
    """Monitor and display telemetry data."""
    print("\n=== EXAMPLE 6: Telemetry Monitoring ===\n")

    with DroneAPI() as drone:
        drone.takeoff()
        time.sleep(5)

        # Monitor telemetry for 10 seconds
        for i in range(10):
            drone.print_status()

            # Move around while monitoring
            if i % 2 == 0:
                drone.move_forward(0.3)
            else:
                drone.hover()

            time.sleep(1)

        drone.land()
        time.sleep(3)


# ==================== EXAMPLE 7: Configuration ====================

def example_configuration():
    """Demonstrate configuration commands."""
    print("\n=== EXAMPLE 7: Configuration ===\n")

    with DroneAPI() as drone:
        # Configure before flight
        print("Configuring drone settings...")

        # Set maximum altitude to 2 meters
        drone.set_max_altitude(2000)

        # Set maximum tilt angle to 20 degrees
        drone.set_max_euler_angle(20.0)

        # Set outdoor mode
        drone.enable_outdoor_mode(False)  # Indoor mode

        # Perform flat trim calibration
        drone.flat_trim()

        time.sleep(1)

        # Now fly
        drone.takeoff()
        time.sleep(5)

        drone.print_status()

        drone.land()
        time.sleep(3)


# ==================== EXAMPLE 8: Programmed Maneuver ====================

def example_maneuver_sequence():
    """Execute a pre-programmed maneuver sequence."""
    print("\n=== EXAMPLE 8: Programmed Maneuver ===\n")

    with DroneAPI() as drone:
        drone.takeoff()
        time.sleep(5)

        # Define a sequence of maneuvers
        maneuvers = [
            (drone.move_forward, [0.5]),
            (drone.hover, []),
            (drone.move_up, [0.3]),
            (drone.hover, []),
            (drone.rotate_clockwise, [0.5]),
            (drone.hover, []),
            (drone.move_backward, [0.4]),
            (drone.hover, []),
            (drone.move_down, [0.3]),
            (drone.hover, []),
        ]

        # Execute maneuvers with 1 second per command
        for maneuver in maneuvers:
            func = maneuver[0]
            args = maneuver[1] if len(maneuver) > 1 else []

            print(f"Executing: {func.__name__}")

            # Execute for 1 second
            end_time = time.time() + 1.0
            while time.time() < end_time:
                func(*args)
                time.sleep(0.05)

        drone.land()
        time.sleep(3)


# ==================== EXAMPLE 9: Video Recording ====================

def example_video_recording():
    """Record video frames during flight."""
    print("\n=== EXAMPLE 9: Video Recording ===\n")

    drone = DroneAPI()

    if not drone.wait_for_video(timeout=10.0):
        print("Video not available")
        return

    drone.takeoff()
    time.sleep(5)

    print("Recording frames...")
    frame_count = 0

    try:
        for i in range(100):  # Record 100 frames
            drone.hover()

            # Save frame every 10 frames
            if i % 10 == 0:
                filename = f"frame_{frame_count:04d}.jpg"
                if drone.save_frame(filename):
                    print(f"Saved {filename}")
                    frame_count += 1

            # Display video
            drone.display_video()

            time.sleep(0.1)

    finally:
        print(f"Recorded {frame_count} frames")
        drone.land()
        time.sleep(3)
        drone.close()


# ==================== EXAMPLE 10: Advanced Direct Control ====================

def example_direct_control():
    """Demonstrate low-level direct flight control."""
    print("\n=== EXAMPLE 10: Advanced Direct Control ===\n")

    with DroneAPI() as drone:
        drone.takeoff()
        time.sleep(5)

        # Circle maneuver using direct control
        print("Flying in a circle using direct control...")

        duration = 5.0  # seconds
        end_time = time.time() + duration

        while time.time() < end_time:
            # Combine forward pitch and yaw for circular motion
            drone.fly_direct(
                pitch=0.3,   # Move forward
                yaw=0.4,     # Rotate clockwise
                roll=0.0,
                gaz=0.0
            )
            time.sleep(0.05)

        # Hover
        for _ in range(20):
            drone.hover()
            time.sleep(0.1)

        drone.land()
        time.sleep(3)


# ==================== MAIN MENU ====================

def main():
    """Run example menu."""
    print("\nAR Drone API Examples")
    print("=" * 50)
    print("1. Basic Flight (takeoff, hover, land)")
    print("2. Simple Movement (forward, backward)")
    print("3. Complete Movement Demo (all directions)")
    print("4. Square Pattern Flight")
    print("5. Video Streaming")
    print("6. Telemetry Monitoring")
    print("7. Configuration")
    print("8. Programmed Maneuver Sequence")
    print("9. Video Recording")
    print("10. Advanced Direct Control")
    print("=" * 50)

    choice = input("\nSelect example (1-10) or 'q' to quit: ")

    examples = {
        '1': example_basic_flight,
        '2': example_simple_movement,
        '3': example_complete_movement,
        '4': example_square_pattern,
        '5': example_video_streaming,
        '6': example_telemetry,
        '7': example_configuration,
        '8': example_maneuver_sequence,
        '9': example_video_recording,
        '10': example_direct_control,
    }

    if choice in examples:
        try:
            examples[choice]()
            print("\nExample completed successfully!")
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
        except Exception as e:
            print(f"\nError: {e}")
    elif choice.lower() == 'q':
        print("Exiting...")
    else:
        print("Invalid choice")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Tello Drone WASD Control Script

Controls:
- W/S: Forward/Backward
- A/D: Left/Right
- Arrow Up/Down: Up/Down
- Arrow Left/Right: Rotate Counter-Clockwise/Clockwise
- T: Takeoff
- L: Land
- Q: Quit
- Space: Emergency Stop

Make sure to install djitellopy first:
pip install djitellopy
"""

from djitellopy import Tello
import keyboard
import time
import sys

# Movement parameters
MOVEMENT_DISTANCE = 30  # cm
ROTATION_ANGLE = 30     # degrees

def main():
    print("Initializing Tello drone...")
    
    # Initialize drone
    drone = Tello()
    
    try:
        # Connect to drone
        drone.connect()
        print(f"Connected! Battery: {drone.get_battery()}%")
        
        print("\n=== TELLO WASD CONTROL ===")
        print("Controls:")
        print("  W/S: Forward/Backward")
        print("  A/D: Left/Right")
        print("  ↑/↓: Up/Down")
        print("  ←/→: Rotate Left/Right")
        print("  T: Takeoff")
        print("  L: Land")
        print("  Q: Quit")
        print("  Space: Emergency Stop")
        print("\nPress 'T' to takeoff and begin flying!")
        print("=" * 30 + "\n")
        
        is_flying = False
        
        while True:
            try:
                # Takeoff
                if keyboard.is_pressed('t') and not is_flying:
                    print("Taking off...")
                    drone.takeoff()
                    is_flying = True
                    print("Airborne! Ready for commands.")
                    time.sleep(2)
                
                # Land
                elif keyboard.is_pressed('l') and is_flying:
                    print("Landing...")
                    drone.land()
                    is_flying = False
                    print("Landed safely.")
                    time.sleep(2)
                
                # Emergency stop
                elif keyboard.is_pressed('space'):
                    print("EMERGENCY STOP!")
                    drone.emergency()
                    is_flying = False
                    time.sleep(1)
                
                # Quit
                elif keyboard.is_pressed('q'):
                    print("Quitting...")
                    if is_flying:
                        print("Landing before exit...")
                        drone.land()
                    break
                
                # Movement commands (only when flying)
                elif is_flying:
                    # Forward/Backward
                    if keyboard.is_pressed('w'):
                        print(f"Moving forward {MOVEMENT_DISTANCE}cm")
                        drone.move_forward(MOVEMENT_DISTANCE)
                        time.sleep(1)
                    
                    elif keyboard.is_pressed('s'):
                        print(f"Moving backward {MOVEMENT_DISTANCE}cm")
                        drone.move_back(MOVEMENT_DISTANCE)
                        time.sleep(1)
                    
                    # Left/Right
                    elif keyboard.is_pressed('a'):
                        print(f"Moving left {MOVEMENT_DISTANCE}cm")
                        drone.move_left(MOVEMENT_DISTANCE)
                        time.sleep(1)
                    
                    elif keyboard.is_pressed('d'):
                        print(f"Moving right {MOVEMENT_DISTANCE}cm")
                        drone.move_right(MOVEMENT_DISTANCE)
                        time.sleep(1)
                    
                    # Up/Down
                    elif keyboard.is_pressed('up'):
                        print(f"Moving up {MOVEMENT_DISTANCE}cm")
                        drone.move_up(MOVEMENT_DISTANCE)
                        time.sleep(1)
                    
                    elif keyboard.is_pressed('down'):
                        print(f"Moving down {MOVEMENT_DISTANCE}cm")
                        drone.move_down(MOVEMENT_DISTANCE)
                        time.sleep(1)
                    
                    # Rotation
                    elif keyboard.is_pressed('left'):
                        print(f"Rotating counter-clockwise {ROTATION_ANGLE}°")
                        drone.rotate_counter_clockwise(ROTATION_ANGLE)
                        time.sleep(1)
                    
                    elif keyboard.is_pressed('right'):
                        print(f"Rotating clockwise {ROTATION_ANGLE}°")
                        drone.rotate_clockwise(ROTATION_ANGLE)
                        time.sleep(1)
                
                # Small delay to prevent CPU overuse
                time.sleep(0.1)
                
            except KeyboardInterrupt:
                print("\nInterrupted! Landing...")
                if is_flying:
                    drone.land()
                break
                
    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        print("Closing connection...")
        drone.end()
        print("Done!")

if __name__ == "__main__":
    # Check if keyboard module is available
    try:
        import keyboard
    except ImportError:
        print("Error: 'keyboard' module not found.")
        print("Install it using: pip install keyboard")
        print("\nNote: On Linux, you may need to run with sudo:")
        print("  sudo python3 tello_wasd_control.py")
        sys.exit(1)
    
    main()
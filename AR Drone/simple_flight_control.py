"""
Simple AR Drone Flight Control - NO VIDEO
Pure WASD keyboard control via terminal input

Controls:
  w - Forward
  s - Backward
  a - Left
  d - Right
  i - Up
  k - Down
  j - Rotate Left
  l - Rotate Right
  space - Takeoff/Land
  h - Hover
  q - Quit
"""

import sys
import time
import msvcrt  # Windows keyboard input
from drone_api import DroneAPI


def get_key():
    """Get a single keypress from terminal (Windows)."""
    if msvcrt.kbhit():
        return msvcrt.getch().decode('utf-8').lower()
    return None


def main():
    print("="*60)
    print("  AR DRONE - SIMPLE FLIGHT CONTROL (NO VIDEO)")
    print("="*60)

    # Connect to drone
    print("\n[*] Connecting to drone...")
    drone = DroneAPI()

    print("[*] Getting drone status...")
    battery = drone.get_battery_percentage()
    altitude = drone.get_altitude()

    print(f"[*] Battery: {battery if battery else 'N/A'}%")
    print(f"[*] Altitude: {altitude if altitude else 'N/A'} mm")

    print("\n" + "="*60)
    print("  CONTROLS:")
    print("  W/S - Forward/Backward")
    print("  A/D - Left/Right")
    print("  I/K - Up/Down")
    print("  J/L - Rotate Left/Right")
    print("  SPACE - Takeoff/Land")
    print("  H - Hover")
    print("  Q - Quit")
    print("="*60)
    print("\nPress SPACE to takeoff!\n")

    is_flying = False
    speed = 0.3
    last_cmd_time = time.time()

    try:
        while True:
            key = get_key()

            if key:
                current_time = time.time()

                # Takeoff/Land
                if key == ' ':
                    if not is_flying:
                        print("[CMD] TAKEOFF")
                        drone.takeoff()
                        is_flying = True
                        time.sleep(3)
                        print("[OK] Airborne!")
                    else:
                        print("[CMD] LANDING")
                        drone.land()
                        is_flying = False
                        time.sleep(2)
                        print("[OK] Landed")

                # Movement commands (only when flying)
                elif is_flying:
                    if key == 'w':
                        print(f"[CMD] FORWARD {int(speed*100)}%")
                        drone.move_forward(speed)
                        last_cmd_time = current_time

                    elif key == 's':
                        print(f"[CMD] BACKWARD {int(speed*100)}%")
                        drone.move_backward(speed)
                        last_cmd_time = current_time

                    elif key == 'a':
                        print(f"[CMD] LEFT {int(speed*100)}%")
                        drone.move_left(speed)
                        last_cmd_time = current_time

                    elif key == 'd':
                        print(f"[CMD] RIGHT {int(speed*100)}%")
                        drone.move_right(speed)
                        last_cmd_time = current_time

                    elif key == 'i':
                        print(f"[CMD] UP {int(speed*100)}%")
                        drone.move_up(speed)
                        last_cmd_time = current_time

                    elif key == 'k':
                        print(f"[CMD] DOWN {int(speed*100)}%")
                        drone.move_down(speed)
                        last_cmd_time = current_time

                    elif key == 'j':
                        print(f"[CMD] ROTATE-LEFT {int(speed*100)}%")
                        drone.rotate_counterclockwise(speed)
                        last_cmd_time = current_time

                    elif key == 'l':
                        print(f"[CMD] ROTATE-RIGHT {int(speed*100)}%")
                        drone.rotate_clockwise(speed)
                        last_cmd_time = current_time

                    elif key == 'h':
                        print("[CMD] HOVER")
                        drone.hover()
                        last_cmd_time = current_time

                # Quit
                if key == 'q':
                    print("[*] Quit requested")
                    break

            # Send hover command periodically to maintain connection
            if time.time() - last_cmd_time > 0.5:
                if is_flying:
                    drone.hover()
                last_cmd_time = time.time()

            time.sleep(0.01)  # Small delay to prevent CPU spinning

    except KeyboardInterrupt:
        print("\n[!] Keyboard interrupt")

    finally:
        if is_flying:
            print("\n[*] Landing before exit...")
            drone.land()
            time.sleep(3)

        print("[*] Closing connection...")
        drone.close()
        print("[*] Goodbye!")


if __name__ == "__main__":
    main()

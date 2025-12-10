import time
import sys
import msvcrt  # Windows-only keyboard

import pyardrone
from pyardrone import at


# --- CONTROL-ONLY DRONE: NO VIDEO CLIENT ---
class ARDroneNoVideo(pyardrone.HelperMixin, pyardrone.ARDroneBase):
    """High-level helpers + navdata, but no internal video connection."""
    pass


def main():
    print("[INFO] Connecting to AR.Drone...")
    drone = ARDroneNoVideo()
    print("[INFO] Connected.")

    # --- Takeoff ---
    print("[FLIGHT] Taking off...")
    drone.takeoff()
    takeoff_time = time.time()

    try:
        while True:
            drone.hover()

            # --- Manual land ---
            if msvcrt.kbhit():
                key = msvcrt.getwch().lower()
                if key == 'l':
                    print("[INPUT] 'l' pressed – landing now.")
                    break

            # --- Automatic land after 2 seconds ---
            if time.time() - takeoff_time >= 2.0:
                print("[TIMER] 5 seconds elapsed – auto-landing.")
                break

            time.sleep(0.01)

    finally:
        # --- Safe landing ---
        print("[FLIGHT] Landing...")
        drone.land()
        time.sleep(4.0)

        # Print ending battery if available
        if drone.navdata_ready.is_set():
            demo = getattr(drone.navdata, "demo", None)
            if demo:
                print(f"[NAVDATA] End battery: {demo.vbat_flying_percentage}%")

        print("[INFO] Done.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[KEYBOARD] Ctrl+C – emergency exit.")
        sys.exit(0)

from pyardrone import ARDrone, at
import time

def main():
    print("[INFO] Connecting to AR.Drone...")
    drone = ARDrone()
    print("[INFO] Connected.")

    # Wait a bit so navdata/state is sane
    drone.navdata_ready.wait(timeout=3.0)

    # According to AR.Drone SDK, video:video_channel:
    # 0 = front camera
    # 1 = bottom camera
    # 2/3 = mixed / picture-in-picture modes (varies by firmware)
    print("[INFO] Switching to BOTTOM camera (video:video_channel = 1)")
    drone.send(at.CONFIG('video:video_channel', 1)) #1 for bottom camera , 0 for front facing camera

    # give the drone time to apply the config before we exit
    time.sleep(1.0)
    print("[INFO] Done. You can now open ffplay on tcp://192.168.1.1:5555.")

if __name__ == "__main__":
    main()
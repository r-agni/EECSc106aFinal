import subprocess
import time
import cv2
from djitellopy import Tello

WIFI_PROFILE_NAME = "TELLO-5F897A"   # make sure this matches `netsh wlan show profiles`


def connect_to_tello_wifi():
    print(f"[*] Connecting to Wi-Fi profile: {WIFI_PROFILE_NAME} ...")
    try:
        result = subprocess.run(
            ["netsh", "wlan", "connect", f"name={WIFI_PROFILE_NAME}", "interface=Wi-Fi"],
            capture_output=True,
            text=True,
            check=False,
        )
        print(result.stdout)
        if result.returncode != 0:
            print("[!] netsh returned an error:")
            print(result.stderr)
            return False

        # Give Windows a moment to fully associate to the Tello network
        time.sleep(5)
        print("[+] Wi-Fi connect command issued.")
        return True

    except Exception as e:
        print(f"[!] Failed to run netsh: {e}")
        return False

def main():
    # 1) Connect to Tello Wi-Fi
    if not connect_to_tello_wifi():
        print("[!] Could not connect to Tello Wi-Fi. Exiting.")
        return

    # 2) Initialize Tello and enter SDK mode
    tello = Tello()
    tello.connect()
    print(f"[+] Battery: {tello.get_battery()}%")

    # 3) Tell Tello to start streaming video
    tello.streamon()   # just sends the 'streamon' command

    # 4) Open the UDP video stream with OpenCV (this is what worked for you already)
    cap = cv2.VideoCapture("udp://0.0.0.0:11111", cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("[!] Could not open video stream with OpenCV")
        tello.streamoff()
        tello.end()
        return
    
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 240) 

    print("f to launch, l to land, wasd to move, zc to rotate left and right, q to quit")
    
    landed = True
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                # Video packet hiccup, just skip this frame
                continue

            cv2.imshow("Tello Stream", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('f') and landed:
                tello.takeoff()
                time.sleep(1)
                landed = False
            elif key == ord('w') and not landed:
                tello.move_forward(20)
            elif key == ord('a') and not landed:
                tello.move_left(20)
            elif key == ord('s') and not landed:
                tello.move_back(20)
            elif key == ord('d') and not landed:
                tello.move_right(20)
            elif key == ord('z') and not landed:
                tello.rotate_counter_clockwise(30)
            elif key == ord('c') and not landed:
                tello.rotate_clockwise(30)
            elif key == ord('l') and not landed:
                tello.land()
                landed = True
            elif key == ord('q'):
                if not landed:
                    tello.land()
                break

    finally:
        print("[*] Cleaning up...")
        cap.release()
        cv2.destroyAllWindows()
        try:
            tello.streamoff()
        except Exception:
            pass
        tello.end()


if __name__ == "__main__":
    main()
# Drone Cooperative System for Humanitarian Missions

Two drones cooperating to navigate and find targets in a disaster scene — an AR Drone and a Tello, each running vision and planning onboard.

EECS/ME 106A final project, Team 14 (Agni Rajinikanth, Alan Bao, Nohl Abdelhadi, Imam Majed Alayeh). [Project website](https://r-agni.github.io/EECSc106aFinal/website/).

## Demo

- [URobot 106A](https://www.youtube.com/watch?v=2fCjf9HZGC8)
- [Self-centering](https://www.youtube.com/watch?v=RAn6AjT8vB4) · [Robot arm following](https://www.youtube.com/watch?v=Rk5S7jcEtkc)

## How it works

- **ArUco marker localization** — `aruco_detection/` detects fiducials and solves the camera-to-world transform so a drone knows where it is relative to known markers.
- **YOLOv8 object detection + avoidance** — `object_detection/` spots obstacles and targets in the live feed and feeds an avoidance layer into the path planner.
- **Path planning** — `path_planning/` turns detections and goals into waypoints under the drone's motion constraints.
- **Two platforms, shared pipeline** — the same perception/planning stack drives both the Parrot AR Drone and the DJI Tello (`AR Drone/`, `Tello/`), each through its own control API.

## Run it

```bash
cd "AR Drone" && python main.py     # or: cd Tello && python main.py
```

See each platform's `README.md` for setup and calibration.

# Lab 06 — Kinect navigation

> **In one sentence:** a Kinect v1 depth camera finds the floor plane, tracks the robot as "the robot-sized blob
> above the floor", and drives it to a point you click on a top-down map.

**Status:** ⏸ works, paused (a low camera covers little floor; next step would be a ceiling-mounted camera).

## In the code

| File | What |
|---|---|
| [`kinect.py`](kinect.py) | libfreenect via ctypes, floor by RANSAC, robot blob (size/height filters, distance-aware), live view |
| [`navigate.py`](navigate.py) | click-to-go: heading from the robot's gyro aligned to the map by driving straight; obstacles; limits |

```bash
python3 labs/06-kinect-navigation/kinect.py        # live depth + top-down map
python3 labs/06-kinect-navigation/navigate.py      # click the robot, then click (or drag) on the map
```

## Key results (measured)

- Kinect vs wheel odometry on `move 50`: **50.9 cm vs 49.8 cm**; ±1 mm jitter standing still; 30 frames/s; works in the dark.
- Tracking range ~2.5 m with the camera at 0.82 m, 30° down → goals limited to 2.3 m and 20 cm from the image edge.

Plan and status: [docs/history/plan-platform.md](../../docs/history/plan-platform.md). *Full write-up coming next.*

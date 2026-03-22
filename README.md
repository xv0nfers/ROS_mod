# USV Vision & Guidance Stack (ROS 2 Humble)

This project is organized as an onboard software stack for an autonomous USV, with emphasis on **reliability, determinism, and robust control** rather than over-complicated perception pipelines.

## Design principle

The core challenge is **control action synthesis**, not adding non-deterministic vision complexity.

- Vision is an input source, not the decision core.
- Prefer simple, stable, and universal algorithms where possible.
- OpenCV pipelines are fully valid for embedded operation (including old SBCs without NPU).
- Neural networks/NPU are optional tools, not architectural requirements.
- The main engineering effort should be in filtering, telemetry fusion, and anti-oscillation control logic.

---

## 1) Installation on USV onboard computer

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+
- OpenCV (system or pip):
  - `python3-opencv` (recommended on embedded Linux)
  - or `opencv-python`

### Workspace setup

```bash
mkdir -p ~/usv_ws/src
cd ~/usv_ws/src
git clone <YOUR_REPO_URL> ROS_mod
cd ROS_mod

# Python deps
pip3 install -r requirements.txt

# System OpenCV (recommended)
sudo apt-get update
sudo apt-get install -y python3-opencv

# ROS deps + build
cd ~/usv_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/usv_ws/install/setup.bash
```

Optional persistent setup:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/usv_ws/install/setup.bash" >> ~/.bashrc
```

---

## 2) Running on USV

### Start perception stack

```bash
ros2 launch yolo_bringup yolo.launch.py
```

### (Optional) Start OpenCV-only feature pipeline

If mission profile prioritizes deterministic low-compute processing, run your OpenCV feature extraction / tracking node and publish target observations to your control layer.

### Topics to connect into control stack

- Camera input: `/camera/image_raw`
- Tracking/detections: `/tracking` (or your OpenCV tracker topic)
- Guidance output (if using selector node): `/usv/target_vector`
- Debug view: `/usv/vision_debug`

---

## 3) Training workflow (if using neural models)

> NN training is optional and should not replace control-model quality.

### Dataset guidance for marine operations

Prioritize:
- USV/boat/canoe classes
- Sea clutter, glare, wake, haze, rain, dusk
- Long range + near range balance

### Example training

```bash
yolo detect train \
  model=yolov8n.pt \
  data=usv_dataset.yaml \
  imgsz=640 \
  epochs=100 \
  batch=32 \
  device=0
```

### Export for edge

```bash
yolo export model=best.pt format=onnx imgsz=640
```

Then convert/calibrate to target runtime (INT8, vendor-specific if NPU is used).

---

## 4) USV control integration notes (important)

To avoid drift and oscillation:
- Use bounded control outputs and rate limits.
- Add filtering with explicit latency budgeting.
- Fuse visual observations with telemetry (IMU/GNSS/heading/speed).
- Apply anti-windup and gain scheduling for high-speed regimes.
- Keep fallback degraded mode when perception confidence drops.

---

## 5) OpenCV and ROS references

- OpenCV: https://opencv.org/
- ROS wiki `vision_opencv`: https://wiki.ros.org/vision_opencv
- `vision_opencv` repository: https://github.com/ros-perception/vision_opencv
- ROS2 OpenCV detection example: https://github.com/luciapferreira/ros2-opencv-object-detection

---

## 6) Quick start

```bash
# Terminal 1
source /opt/ros/humble/setup.bash
source ~/usv_ws/install/setup.bash
ros2 launch yolo_bringup yolo.launch.py

# Terminal 2
source /opt/ros/humble/setup.bash
source ~/usv_ws/install/setup.bash
# Run your selector/controller bridge node here
```

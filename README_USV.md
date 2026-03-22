# USV Autonomous Target Interception Stack (ROS 2 Humble)

This repository is the onboard perception-and-guidance bridge for an autonomous Unmanned Surface Vehicle (USV).
It is tailored for **coastal/marine pursuit and interception** missions with a compute-constrained edge device.

## 1) System purpose (USV context)

The stack provides:
- Real-time visual detection/tracking integration based on YOLO + ByteTrack.
- Marine-specific target selection (USV/boat classes) with a lock-on mechanism.
- Guidance vector output for autopilot/control integration.
- Debug vision stream for operator validation and field tuning.

Mission profile assumptions:
- High-speed surface intercept scenarios (up to ~60 knots).
- Targets remain on waterline; sky/top-band false detections should be ignored.
- Edge inference hardware is limited (NPU class device), so latency-sensitive processing is mandatory.

---

## 2) Repository structure

- `yolo_ros/` – ROS 2 Python package with YOLO inference/tracking nodes.
- `yolo_msgs/` – custom message definitions used by the perception stack.
- `yolo_bringup/` – launch files for model/runtime bring-up.
- `yolo_ros/yolo_ros/usv_target_selector_node.py` – USV target lock and guidance bridge node.

---

## 3) Installation on the USV onboard computer

### 3.1 Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+
- Camera driver publishing `sensor_msgs/Image`
- (Optional but recommended) NPU runtime / vendor toolkit for INT8 model execution

### 3.2 Workspace setup

```bash
mkdir -p ~/usv_ws/src
cd ~/usv_ws/src
git clone <YOUR-REPO-URL> ROS_mod
cd ROS_mod
pip3 install -r requirements.txt
cd ~/usv_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/usv_ws/install/setup.bash
```

### 3.3 Runtime environment (recommended on USV)

```bash
# Add ROS setup to shell profile for persistent sessions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/usv_ws/install/setup.bash" >> ~/.bashrc
```

---

## 4) How to run on the USV

### 4.1 Start perception (YOLO + tracking)

Use the launch file matching your deployed model/runtime.

```bash
ros2 launch yolo_bringup yolov8.launch.py
```

If using generic launch with custom parameters:

```bash
ros2 launch yolo_bringup yolo.launch.py \
  model:=<PATH_OR_NAME_TO_MODEL> \
  device:=cpu \
  use_tracking:=True \
  use_debug:=True
```

> For NPU deployments, set model/runtime options according to your vendor export/runtime path.

### 4.2 Start USV target selector bridge

```bash
ros2 run yolo_ros usv_target_selector_node
```

### 4.3 Key I/O topics for integration

Inputs:
- `/camera/image_raw` (`sensor_msgs/Image`)
- `/tracking` (`vision_msgs/Detection2DArray`) – tracked detections with IDs

Outputs:
- `/usv/target_vector` (`geometry_msgs/Twist`)
  - `linear.x`: normalized azimuth error `[-1..1]`
  - `linear.y`: distance-like heuristic
  - `linear.z`: lock flag (`1.0` locked / `0.0` unlocked)
- `/usv/vision_debug` (`sensor_msgs/Image`) – annotated frame

### 4.4 Typical tuning parameters (field trials)

`usv_target_selector_node` parameters:
- `roi_top_ratio` (default `0.30`) – ignores detections in top frame band.
- `priority_class_ids` (default `[0,1]`) – target classes for lock-on.
- `min_stable_hits` (default `3`) – minimum consistent ID observations before lock.
- `max_lock_misses` (default `5`) – tolerated temporary target loss.
- `max_detection_age_sec` (default `0.25`) – reject stale detections.

Example run with overrides:

```bash
ros2 run yolo_ros usv_target_selector_node --ros-args \
  -p roi_top_ratio:=0.30 \
  -p priority_class_ids:="[0,1]" \
  -p min_stable_hits:=4 \
  -p max_lock_misses:=6
```

---

## 5) Training workflow for USV targets

The model training itself is done in Ultralytics (offline workstation/server), then deployed to the USV edge device.

### 5.1 Dataset recommendations

Collect and label marine-domain data with emphasis on:
- USV hulls from chase and crossing angles.
- Small boats / canoes in cluttered backgrounds.
- Foam, glare, wake, haze, low sun, rain, sea-state variation.
- Long-range tiny targets and near-field large targets.

Suggested classes for interception use:
- `0: usv`
- `1: boat`
- Additional classes as needed (`jetski`, `kayak`, etc.).

### 5.2 Baseline training command (example)

```bash
yolo detect train \
  model=yolov8n.pt \
  data=usv_dataset.yaml \
  imgsz=640 \
  epochs=100 \
  batch=32 \
  device=0
```

### 5.3 Export for edge/NPU

```bash
# ONNX export (example)
yolo export model=best.pt format=onnx imgsz=640
```

Then convert/calibrate to your NPU runtime (INT8) with vendor tools.

### 5.4 Deployment checklist

- Verify class ID mapping matches `priority_class_ids` in ROS.
- Validate FPS on target hardware at mission camera rate (target 30 FPS).
- Confirm tracked output includes stable IDs used by lock logic.
- Run sea trials and tune lock/filter parameters conservatively before aggressive interception behavior.

---

## 6) Operational notes (safety + reliability)

- Always perform controlled test runs before open-water missions.
- Keep a manual override channel active in autopilot/control stack.
- Record bags (`ros2 bag`) during trials for post-mission tuning.
- Re-validate after camera/lens, model, or firmware updates.

---

## 7) Quick start (minimal commands)

```bash
# Terminal 1
source /opt/ros/humble/setup.bash
source ~/usv_ws/install/setup.bash
ros2 launch yolo_bringup yolov8.launch.py

# Terminal 2
source /opt/ros/humble/setup.bash
source ~/usv_ws/install/setup.bash
ros2 run yolo_ros usv_target_selector_node
```

This gives you a running USV perception-to-guidance bridge ready to connect to navigation/control logic.

# Atom 25 · Camera · Read Image / Depth / Intrinsics (Orbbec)

**English** | [简体中文](atom25_camera_guide_zh-CN.md)

**In one line**: subscribe to the Orbbec camera's ROS2 topics (RGB `/camera/color/image_raw`, depth, intrinsics), convert them to numpy images with `cv_bridge`, and save. This is a **perception atom — read-only, it drives no motors and the robot does not move**.

| Companion code | Demo video |
|---|---|
| `atom/demos/atom25_camera_orbbec.py` (native ROS2, RGB), `atom/demos/atom25_camera_orbbec_robust.py` (production, RGB+depth+intrinsics) | in `atom/assets/videos/`, same name as the code |

## 1. At a Glance (read this first)

### 1.1 Run it

Prerequisite: the camera is on the **Orin** and needs **no body_control**. First start the camera driver (see `scripts/`):

```bash
# on the Orin, as user nvidia
./scripts/start_camera.sh          # starts the Orbbec driver; verify: ros2 topic list | grep camera
```

Then run the demo (simplest on the Orin locally; the x86 also works if it shares the ROS graph with the same `ROS_DOMAIN_ID`):

```bash
source /opt/ros/humble/setup.bash   # provides rclpy / sensor_msgs / cv_bridge
python3 atom/demos/atom25_camera_orbbec.py
```

Expected: `收到一帧 RGB 图: 1280x720` → saved to `atom/assets/camera_captures/atom25_rgb.jpg`.

⚠ Read-only subscription; the robot does not move, so it's safe to run repeatedly. Dependencies: `cv_bridge` (a ROS package), `opencv-python` (`cv2`); the robust version also needs `numpy`.

### 1.2 Interface

| Item | Value |
|---|---|
| RGB topic | `/camera/color/image_raw` → `sensor_msgs/Image`, encoding `bgr8` |
| Depth topic | `/camera/depth/image_raw` → `sensor_msgs/Image`, encoding `16UC1`, **unit: millimeters (mm)** |
| Intrinsics topic | `/camera/color/camera_info` → `sensor_msgs/CameraInfo`, `k` is a 3×3 row-major flat array |
| Conversion | `cv_bridge`'s `imgmsg_to_cv2` turns an `Image` into an OpenCV numpy array |
| Resolution | e.g. 1280×720 @30fps (whatever the driver is configured for) |

### 1.3 Key Points (the camera has no "limits" — these are the gotchas)

| Point | Note |
|---|---|
| ⚠ **QoS** | camera images are often published **BEST_EFFORT**. **You must subscribe with `qos_profile_sensor_data`** — a default (RELIABLE) subscription against a BEST_EFFORT driver receives **not a single frame** (shows up as "no image in 5s" even though the topic is publishing). The demo uses sensor_data QoS, compatible with any publisher |
| Depth unit | `16UC1`, each pixel is in **millimeters**; `0` means invalid/no return and must be excluded from stats |
| RGB ≠ depth resolution | color and depth resolutions may differ; index a depth pixel using the **depth image's own dimensions**, not the color width/height |
| Passive push | images are pushed by the subscription — the callback only fires while `rclpy.spin_*` is running |
| Read-only safety | this atom drives no motors; running it repeatedly is risk-free |

## 2. The Three Core Operations

### 2.1 Subscribe to the camera topics (★ QoS is the key)

```python
from rclpy.qos import qos_profile_sensor_data
# Use sensor_data QoS (BEST_EFFORT) — compatible with any publisher, avoids a QoS mismatch dropping all frames
self.sub_ = self.create_subscription(Image, RGB_TOPIC, self._on_rgb, qos_profile_sensor_data)
```

This is the classic camera pitfall: a QoS mismatch silently receives nothing. `ros2 topic info -v <topic>` shows both sides' QoS.

### 2.2 Convert to numpy with cv_bridge

A `sensor_msgs/Image` is not an image array — convert it with `cv_bridge`:

```python
from cv_bridge import CvBridge
self.bridge = CvBridge()

def _on_rgb(self, msg):
    self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")   # → (H,W,3) numpy
```

- Use `bgr8` for RGB (OpenCV's channel order).
- Use `16UC1` for depth (16-bit integer, millimeters per pixel).

### 2.3 Receive a frame + use it (save / compute depth)

Images are pushed passively, so the main flow spins until the first frame arrives, then uses it:

```python
t0 = time.time()
while rclpy.ok() and node.rgb is None and time.time() - t0 < 5.0:
    rclpy.spin_once(node, timeout_sec=0.1)   # a received frame fills node.rgb in the callback
cv2.imwrite("atom25_rgb.jpg", node.rgb)      # save it to see the result
```

The robust version also reads the intrinsics `K` (`fx/fy/cx/cy`), computes depth stats (valid pixels / range / median), takes the center-pixel depth, and saves a pseudo-color depth image.

## 3. Code Walkthrough (core)

`atom25_camera_orbbec.py` (plain version) is **4 modules**. The opposite of a motion atom — it **only subscribes, never publishes** (perception).

### 3.1 Module map

| # | Module | Code anchor | Role | Change per camera/topic? |
|---|---|---|---|---|
| 1 | Config constants | `RGB_TOPIC` / `OUT_DIR` / `FRAME_TIMEOUT` | topic name, output dir, timeout | ✅ topic name |
| 2 | Node & subscription | `CameraDemo.__init__` | `CvBridge` + subscribe to RGB with **sensor_data QoS** | ✅ topic name |
| 3 | Image callback | `_on_rgb` | `imgmsg_to_cv2` → numpy into `self.rgb` | ⭕ generic |
| 4 | Main flow | `main` | spin for the first frame → print resolution → save jpg | ⭕ generic |

### 3.2 Module by module

- **Module 1 — config constants**: the RGB topic name, output dir (`atom/assets/camera_captures`), first-frame timeout.
- **Module 2 — `__init__`**: build `CvBridge`, subscribe to RGB with `qos_profile_sensor_data`. The subscription is stored as `self.sub_`.
- **Module 3 — `_on_rgb`**: converts a `sensor_msgs/Image` to numpy via `imgmsg_to_cv2(..., 'bgr8')` and stores it.
- **Module 4 — `main`**: spins until `self.rgb` is set (first frame) → prints resolution → `cv2.imwrite` saves the jpg.

### 3.3 Reuse it

```python
RGB_TOPIC = "/camera/depth/image_raw"    # subscribe to depth instead (also change desired_encoding to "16UC1")
# for continuous processing: replace the "grab one frame and exit" in main with rclpy.spin(node) and process each frame in the callback
# for depth + intrinsics: see the three-topic robust version
```

> Note: the camera is **read-only perception**, unlike motion atoms (send command + read state) — here there's only "read." To use depth for localization/grasping, first read the intrinsics `K` to back-project pixel + depth into 3D points.

## 4. Tweak & Observe

| Change | Effect |
|---|---|
| point `RGB_TOPIC` at the depth topic + encoding `16UC1` | saves a depth image |
| replace the single-frame grab in `main` with `rclpy.spin(node)` | receives frames continuously (process each in the callback) |
| run the robust `_robust.py` | grabs RGB + depth + intrinsics at once and saves all three |

Predict first, then run, and check against your prediction.

## 5. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| ★ **No image (timeout)** | Check in order: ① `ros2 topic list \| grep camera` — no topics = **driver not started** (run `./scripts/start_camera.sh`) ② is the topic name the same as in the demo ③ `ros2 topic info -v /camera/color/image_raw` shows the publisher QoS — if it's BEST_EFFORT and you reverted to a default subscription, you get nothing (this demo already uses sensor_data QoS) |
| `No module named cv_bridge` | `sudo apt install ros-humble-cv-bridge` |
| `No module named cv2` | `pip install opencv-python` |
| Works on Orin but not x86 | the cross-board ROS graph isn't connected: both boards' `echo $ROS_DOMAIN_ID` must match; or just run the demo on the Orin |
| Depth values look wrong | confirm the unit is mm (`16UC1`); if the driver publishes `32FC1` (meters), change `desired_encoding` and the unit |

## 6. Going Further

- **Robust version (RGB+depth+intrinsics)**: `atom/demos/atom25_camera_orbbec_robust.py` — three subscriptions, `wait_for_frames` until all arrive, prints intrinsics `K` and depth stats, takes the center depth using the depth image's own dimensions, and saves RGB.jpg + 16-bit depth.png + pseudo-color depth.jpg. Callbacks have try/except; spin uses a non-negative timeout.
- **Cloud open-set perception**: the next step is "any object name → detect → segment → grasp candidates" (RexOmni + SAM2 + GraspNet), where the GPU-heavy parts can run in the cloud while the camera/control stay on the robot — material for the later scene/evolution stages.

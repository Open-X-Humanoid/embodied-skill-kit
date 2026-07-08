# 原25 · 相机（Camera）· 读取图像 / 深度 / 内参（Orbbec）

[English](atom25_camera_guide.md) | **简体中文**

**一句话**：订阅 Orbbec 相机的 ROS2 话题（RGB `/camera/color/image_raw`、深度、内参），用 `cv_bridge` 转成 numpy 图像并保存。这是**感知类原子——只读订阅，不控制任何电机、机器人不会动**。

| 配套代码 | 演示视频 |
|---|---|
| `atom/demos/atom25_camera_orbbec.py`（ROS2 原生版，取 RGB）、`atom/demos/atom25_camera_orbbec_robust.py`（生产版，取 RGB+深度+内参） | `atom/assets/videos/` 下与配套代码同名，建议先看 30 秒 |

## 1. 速览（点进来先看这块）

### 1.1 跑起来

前提：相机在 **Orin** 上，**不需要 body_control**。先一键起相机驱动（见 `scripts/`）：

```bash
# 在 Orin 上、nvidia 用户
./scripts/start_camera.sh          # 启动 Orbbec 驱动；验证：ros2 topic list | grep camera
```

然后跑 demo（在 Orin 本地最省事；也可在 x86，只要同一 ROS 图、同 `ROS_DOMAIN_ID`）：

```bash
source /opt/ros/humble/setup.bash   # 提供 rclpy / sensor_msgs / cv_bridge
python3 atom/demos/atom25_camera_orbbec.py
```

预期现象：`收到一帧 RGB 图: 1280x720` → 存到 `atom/assets/camera_captures/atom25_rgb.jpg`。

⚠ 只读订阅、机器人不会动，可安全反复运行。依赖：`cv_bridge`（ROS 包）、`opencv-python`（`cv2`）；生产版还要 `numpy`。

### 1.2 接口

| 项 | 值 |
|---|---|
| RGB 话题 | `/camera/color/image_raw` → `sensor_msgs/Image`，编码 `bgr8` |
| 深度话题 | `/camera/depth/image_raw` → `sensor_msgs/Image`，编码 `16UC1`，**单位毫米(mm)** |
| 内参话题 | `/camera/color/camera_info` → `sensor_msgs/CameraInfo`，`k` 是 3×3 行优先展平 |
| 转换 | `cv_bridge` 的 `imgmsg_to_cv2` 把 `Image` 转成 OpenCV 的 numpy 数组 |
| 分辨率 | 示例 1280×720 @30fps（以驱动实际配置为准） |

### 1.3 关键点（相机没有"限位"，这几条才是坑）

| 要点 | 说明 |
|---|---|
| ⚠ **QoS** | 相机图像常以 **BEST_EFFORT** 发布。**必须用 `qos_profile_sensor_data` 订阅**——用默认(RELIABLE)订阅遇到 BEST_EFFORT 驱动会**一帧都收不到**（表现为"5s 没收到图像"，但话题明明在发）。demo 已用 sensor_data QoS，兼容任何发布者 |
| **深度单位** | `16UC1`，每像素是**毫米**；`0` 表示无效/无返回，统计时要剔除 |
| **RGB≠深度分辨率** | 彩色和深度分辨率可能不同，取深度某点要用**深度图自身尺寸**索引，别拿彩色宽高 |
| **被动推送** | 图像是订阅推送的，必须 `rclpy.spin_*` 跑起来回调才触发 |
| 只读安全 | 本原子不驱动任何电机，反复跑无风险 |

## 2. 三个核心操作

### 2.1 订阅相机话题（★ QoS 是关键）

```python
from rclpy.qos import qos_profile_sensor_data
# 用 sensor_data QoS(BEST_EFFORT) 订阅——兼容任何发布者，避免 QoS 不匹配收不到帧
self.sub_ = self.create_subscription(Image, RGB_TOPIC, self._on_rgb, qos_profile_sensor_data)
```

这是相机原子最容易踩的坑：QoS 不匹配就静默收不到，`ros2 topic info -v <话题>` 能看双方 QoS。

### 2.2 cv_bridge 转 numpy

`sensor_msgs/Image` 不是图像数组，要用 `cv_bridge` 转：

```python
from cv_bridge import CvBridge
self.bridge = CvBridge()

def _on_rgb(self, msg):
    self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")   # → (H,W,3) numpy
```

- RGB 用 `bgr8`（OpenCV 习惯的通道序）。
- 深度用 `16UC1`（每像素 mm 的 16 位整数）。

### 2.3 收帧 + 用（保存 / 算深度）

图像被动推送，主流程先 spin 收到第一帧再用：

```python
t0 = time.time()
while rclpy.ok() and node.rgb is None and time.time() - t0 < 5.0:
    rclpy.spin_once(node, timeout_sec=0.1)   # 收到帧回调会填 node.rgb
cv2.imwrite("atom25_rgb.jpg", node.rgb)      # 存下来看效果
```

生产版还会：读内参 `K`（`fx/fy/cx/cy`）、统计深度（有效像素/范围/中位）、取画面中心深度、存伪彩色深度图。

## 3. 代码解读（核心）

`atom25_camera_orbbec.py`（简洁版）全文 = **4 个模块**。和运动原子相反——它是**只订阅、不发布**（感知）。

### 3.1 模块地图

| # | 模块 | 代码锚点 | 职责 | 换相机/话题要改？ |
|---|---|---|---|---|
| 1 | 配置常量 | `RGB_TOPIC` / `OUT_DIR` / `FRAME_TIMEOUT` | 话题名、输出目录、超时 | ✅ 话题名 |
| 2 | 建节点与订阅 | `CameraDemo.__init__` | `CvBridge` + 用 **sensor_data QoS** 订阅 RGB | ✅ 话题名 |
| 3 | 图像回调 | `_on_rgb` | `imgmsg_to_cv2` 转 numpy 存 `self.rgb` | ⭕ 通用不改 |
| 4 | 主流程 | `main` | spin 收第一帧 → 打印分辨率 → 存 jpg | ⭕ 通用不改 |

### 3.2 逐模块看

- **模块 1 配置常量**：RGB 话题名、输出目录（`atom/assets/camera_captures`）、等首帧超时。
- **模块 2 `__init__`**：建 `CvBridge`、用 `qos_profile_sensor_data` 订阅 RGB。订阅存 `self.sub_`。
- **模块 3 `_on_rgb`**：把 `sensor_msgs/Image` 用 `imgmsg_to_cv2(..., 'bgr8')` 转成 numpy 存起来。
- **模块 4 `main`**：spin 循环等到 `self.rgb` 有值（第一帧）→ 打印分辨率 → `cv2.imwrite` 存 jpg。

### 3.3 举一反三

```python
RGB_TOPIC = "/camera/depth/image_raw"    # 换订阅深度（记得 desired_encoding 改 "16UC1"）
# 想连续处理：把 main 里的"收一帧就退出"改成 rclpy.spin(node)，在回调里处理每帧
# 想要深度+内参：见生产版三路订阅
```

> 注意：相机是**只读感知**，和运动原子（发指令 + 读状态）不同——这里只有"读"。想拿深度做定位/抓取，先读内参 `K` 把像素+深度反投影成 3D 点。

## 4. 改一改，看变化

| 改什么 | 会怎样 |
|---|---|
| `RGB_TOPIC` 改成深度话题 + 编码 `16UC1` | 存的是深度图 |
| `main` 的收一帧改 `rclpy.spin(node)` | 连续收帧（可在回调里做处理） |
| 跑生产版 `_robust.py` | 一次拿 RGB + 深度 + 内参，存三件套 |

先预测再跑，看是否和预期一致。

## 5. 排错

| 现象 | 原因 / 处理 |
|---|---|
| ★ **收不到图像（超时）** | 排查顺序：① `ros2 topic list \| grep camera` 有没有话题——没有=**驱动没起**（跑 `./scripts/start_camera.sh`）② 话题名是否和 demo 里一致 ③ `ros2 topic info -v /camera/color/image_raw` 看发布方 QoS——若是 BEST_EFFORT 而你改回了默认订阅，就收不到（本 demo 已用 sensor_data QoS） |
| `No module named cv_bridge` | `sudo apt install ros-humble-cv-bridge` |
| `No module named cv2` | `pip install opencv-python` |
| 在 x86 上收不到、Orin 上能收 | 跨板 ROS 图没通：两板 `echo $ROS_DOMAIN_ID` 要一致；或直接在 Orin 上跑 demo |
| 深度值看着不对 | 确认单位是 mm（`16UC1`）；若驱动发 `32FC1`(米) 要改 `desired_encoding` 与单位 |

## 6. 进阶

- **生产版（RGB+深度+内参三件套）**：`atom/demos/atom25_camera_orbbec_robust.py`——三路订阅、`wait_for_frames` 收齐再处理、打印内参 `K` 与深度统计、按深度图自身尺寸取中心深度、存 RGB.jpg + 16 位深度.png + 伪彩色深度.jpg。回调带 try/except、非负 spin。
- **上云开放集感知**：进一步是"任意物体名 → 检测 → 分割 → 抓取候选"（RexOmni + SAM2 + GraspNet），GPU 重的部分可上云、相机/控制留机器人侧——属于后续场景/进化阶段的内容。

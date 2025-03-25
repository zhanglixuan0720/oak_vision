# oak_vision

A ROS 2 package for publishing RGB, depth, and VIO (Visual-Inertial Odometry) data from an OAK-D-S2 camera using [DepthAI](https://github.com/luxonis/depthai-python) and [SpectacularAI](https://github.com/SpectacularAI/sdk). This package provides a convenient wrapper node for integration into your robotic perception pipeline.

---

## Features

- ✅ RGB color image stream (raw and compressed)
- ✅ Depth image stream (raw and compressed)
- ✅ Visual-Inertial Odometry (VIO) via SpectacularAI SDK
- ✅ Configurable resolution and framerate
- ✅ Camera intrinsics publishing (`CameraInfo` messages)
- ✅ ROS 2 native launch support with parameters

---

## Requirements

- ROS 2 Foxy
- Python 3.8 (recommended via Conda or virtualenv)
- Dependencies:
  - [depthai](https://pypi.org/project/depthai/)
  - [spectacularAI](https://github.com/SpectacularAI/sdk)
  - `cv_bridge`
  - `image_transport`
  - `sensor_msgs`, `nav_msgs`, `std_msgs`

> ⚠️ Ensure you have a valid SpectacularAI license for VIO functionality.  
> SpectacularAI provides prebuilt SDK packages for **x86 platforms** for **non-commercial academic use**, but **ARM64 builds (e.g., for Jetson devices)** are not publicly available at the moment.

## Installation

> ⚠️ **Before you start**  
> If you have not configured your OAK device before, you must first install DepthAI and configure USB permissions by following the [USB deployment guide](https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide/).  
>  
> For Jetson devices, you also need to configure the system as described in [Deploy with NVIDIA's Jetson](https://docs.luxonis.com/hardware/platform/deploy/to-jetson/).

1. Clone this repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/zhanglixuan0720/oak_vision.git oak_vision
```

2. Install Python dependencies:
```bash
pip install depthai
pip install spectacularAI
```

3. Build the workspace:
```bash
cd ~/ros2_ws
colcon build --packages-select oak_vision
```

4. Source your workspace:
```bash
source install/setup.bash
```

---

## Launch

You can launch the OAK camera node with custom resolution and FPS:

```bash
ros2 launch oak_vision oak_rgbd_vio.launch.py width:=640 height:=400 fps:=30
```
> 💡 If you do **not** have a SpectacularAI license, or you **do not wish to run VIO**, you can set the parameter `mask_vio:=true` to disable visual-inertial odometry:
>
> ```bash
> ros2 launch oak_vision oak_rgbd_vio.launch.py mask_vio:=true
> ```
```

### Default Topics

| Topic | Type |
|-------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` |
| `/camera/color/image_compressed` | `sensor_msgs/CompressedImage` |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` |
| `/camera/depth/image_raw` | `sensor_msgs/Image` |
| `/camera/depth/image_compressed` | `sensor_msgs/CompressedImage` |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` |
| `/camera/odom` | `nav_msgs/Odometry` |

---

## Custom Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `width` | `int` | 640 | Image width |
| `height` | `int` | 480 | Image height |
| `fps` | `int` | 30 | Frames per second (max 30) |

---

## Development & Structure

- `oak_vision/oak_camera.py`: Core class that interfaces with DepthAI pipeline.
- `oak_vision/oak_vision_node.py`: ROS 2 node that wraps the camera class and publishes messages.
- `launch/oak_rgbd_vio.launch.py`: Launch file to start the node with configurable parameters.
- `test/`: ROS 2 linters and style checks.



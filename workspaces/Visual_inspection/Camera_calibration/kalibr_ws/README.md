# Camera Calibration Workspace with Kalibr

A ROS 2 workspace for camera calibration using Kalibr, supporting both Insta360 cameras and standard webcams.

## Overview

This workspace contains packages for:
- **Insta360 360° camera** calibration and processing
- **Standard webcam** calibration
- Image undistortion and cropping tools
- ROS 2 integration with Kalibr (ROS 1) via Docker

## Workspace Structure

```
kalibr_ws/
├── src/
│   ├── gscam/                    # GStreamer camera driver for ROS 2
│   ├── insta360_ros/             # Insta360 camera publisher
│   ├── insta360_undistort/       # Image undistortion node
│   ├── insta360_tools/           # Image cropping utilities
│   └── webcam_publisher/         # Webcam publisher node
├── data/                         # Calibration data and configuration
│   ├── target.yaml               # Calibration target configuration
│   └── *.bag                     # Recorded bag files
├── build/                        # Build artifacts
└── install/                      # Installed packages
```

## Prerequisites

### System Requirements
- Ubuntu 22.04 (or compatible)
- ROS 2 Humble
- Docker (for Kalibr)
- Python 3.10+

### Dependencies

Install required packages:

```bash
sudo apt-get update
sudo apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-image-tools \
    ros-humble-rqt-image-view \
    python3-opencv \
    v4l-utils \
    python3-pip

pip3 install rosbags
```

### Docker Setup for Kalibr

Pull the Kalibr ROS 1 Docker image:

```bash
docker pull kalibr-ros1
```

---

## Part 1: Insta360 Camera Calibration

### Overview
The Insta360 camera outputs both front and back camera feeds in a single image. We split them, record separately, and calibrate each half.

### Step 1: Build Insta360 Packages

```bash
cd ~/kalibr_ws
colcon build --packages-select insta360_ros insta360_undistort insta360_tools
source install/setup.bash
```

### Step 2: Launch Camera Stream

```bash
cd ~/kalibr_ws
ros2 run image_tools cam2image --ros-args -p width:=1280 -p height:=720 -p index:=0
```

### Step 3: Split Front/Back Camera Halves

Since the Insta360 publishes both cameras in one image, use the cropper to split them:

```bash
cd ~/kalibr_ws
source install/setup.bash
cd ~/datacd

# For front camera
ros2 run insta360_tools insta360_cropper --ros-args -p camera_half:=front

# For back camera (in separate recording)
ros2 run insta360_tools insta360_cropper --ros-args -p camera_half:=back
```

### Step 4: Record ROS 2 Bag (SLOW SPEED - Critical!)

**Important:** Move the calibration board **slowly** for proper feature detection.

```bash
ros2 bag record -o calib_front_wall1 /image_cropped
```

Record for 30-60 seconds, covering different angles and positions.

### Step 5: Convert ROS 2 Bag to ROS 1 Format

```bash
cd ~/datacd

rosbags-convert --src calib_front_wall1 --dst calib_front_wall1_ros1.bag \
--src-typestore ros2_humble --dst-typestore ros1_noetic
```

Move the converted bag to the data folder:

```bash
mv calib_front_wall1_ros1.bag ~/kalibr_ws/data/
```

### Step 6: Run Kalibr Calibration in Docker

#### Enable X11 Access
```bash
xhost +local:root
```

#### Launch Kalibr Docker Container
```bash
cd ~/kalibr_ws/data

docker run -it --rm \
-e DISPLAY=$DISPLAY \
-e QT_X11_NO_MITSHM=1 \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v "$PWD:/data" \
kalibr-ros1
```

#### Inside Docker: Source ROS Environments
```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
```

#### Run Calibration with Pinhole-Equidistant Model
```bash
rosrun kalibr kalibr_calibrate_cameras \
--bag /data/calib_front_wall1_ros1.bag \
--topics /image_cropped \
--models pinhole-equi \
--target /data/target.yaml \
--show-extraction
```

**Alternative: Omni-Radtan Model**
```bash
rosrun kalibr kalibr_calibrate_cameras \
--bag /data/calib_front_wall1_ros1.bag \
--topics /image_cropped \
--models omni-radtan \
--target /data/target.yaml \
--show-extraction
```

### Expected Results (Insta360 Front Camera)

```yaml
cam0:
  camera_model: pinhole
  distortion_model: radtan
  distortion_coeffs: [-0.580690551443206, 0.03797047875870684, -0.10257307845492751, 0.14278964948532616]
  intrinsics: [541.7427200632496, 593.7826814887237, 515.2440717965892, 308.1585414244489]
  resolution: [1280, 360]
  rostopic: /image_cropped
```

**Reprojection error: <0.1 px (mean)** - Excellent calibration!

---

## Part 2: Webcam Calibration

### Step 1: Create Webcam Publisher Package

```bash
cd ~/kalibr_ws/src
ros2 pkg create --build-type ament_python webcam_publisher --dependencies rclpy sensor_msgs cv_bridge
```

### Step 2: Create Publisher Node

Create the file structure:

```bash
mkdir -p ~/kalibr_ws/src/webcam_publisher/webcam_publisher
touch ~/kalibr_ws/src/webcam_publisher/webcam_publisher/__init__.py
```

Create `~/kalibr_ws/src/webcam_publisher/webcam_publisher/webcam_publisher.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        
        # Parameters
        self.declare_parameter('device', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        
        device = self.get_parameter('device').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        
        # Publisher
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        
        # OpenCV
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        self.bridge = CvBridge()
        
        # Timer
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Webcam publisher started: {width}x{height} @ {fps}fps')
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'webcam_frame'
            self.publisher_.publish(msg)
    
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Configure setup.py

Edit `~/kalibr_ws/src/webcam_publisher/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'webcam_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Webcam publisher for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webcam_publisher = webcam_publisher.webcam_publisher:main',
        ],
    },
)
```

### Step 4: Build the Package

```bash
cd ~/kalibr_ws
colcon build --packages-select webcam_publisher --symlink-install
source install/setup.bash
```

### Step 5: Launch Webcam Publisher

```bash
ros2 run webcam_publisher webcam_publisher --ros-args \
  -p device:=0 \
  -p width:=640 \
  -p height:=480 \
  -p fps:=30
```

### Step 6: View Camera Stream (Verify Colors)

In a new terminal:

```bash
cd ~/kalibr_ws
source install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Select `/image_raw` from the dropdown menu.

### Step 7: Record ROS 2 Bag

Create directory for webcam calibration data:

```bash
mkdir -p ~/data/webcam_calib
cd ~/data/webcam_calib
```

Record the bag (move calibration board **slowly**):

```bash
ros2 bag record -o webcam_calib_01 /image_raw
```

Record for 30-60 seconds, then press Ctrl+C.

### Step 8: Convert to ROS 1 Format

```bash
cd ~/data/webcam_calib

rosbags-convert --src webcam_calib_01 --dst ~/kalibr_ws/data/webcam_calib_01_ros1.bag \
--src-typestore ros2_humble --dst-typestore ros1_noetic
```

### Step 9: Run Kalibr Calibration

#### Enable X11 and Launch Docker

```bash
xhost +local:root

cd ~/kalibr_ws/data

docker run -it --rm \
-e DISPLAY=$DISPLAY \
-e QT_X11_NO_MITSHM=1 \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v "$PWD:/data" \
kalibr-ros1
```

#### Inside Docker: Run Calibration

```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

rosrun kalibr kalibr_calibrate_cameras \
--bag /data/webcam_calib_01_ros1.bag \
--topics /image_raw \
--models pinhole-radtan \
--target /data/target.yaml \
--show-extraction
```

---

## Calibration Target Configuration

The `target.yaml` file contains the calibration target parameters:

```yaml
# filepath: ~/kalibr_ws/data/target.yaml
target_type: checkerboard
targetCols: 8     # inner corners horizontally (9 squares → 8 corners)
targetRows: 6     # inner corners vertically (7 squares → 6 corners)
rowSpacingMeters: 0.024   # square size in meters (24mm)
colSpacingMeters: 0.024   # square size in meters (24mm)
```

### Creating Your Own Target

- **Checkerboard:** Count the **inner corners** (not squares)
  - 9x7 checkerboard = 8x6 inner corners
- **Measure square size** accurately in meters
- Print on flat, rigid surface (foam board recommended)

---

## Camera Models

### Insta360 (Wide-Angle/Fisheye)
- **pinhole-equi** (Pinhole-Equidistant) - Best results for Insta360
- **omni-radtan** (Omnidirectional-Radtan) - Alternative model

### Standard Webcam
- **pinhole-radtan** (Pinhole-Radial-Tangential) - Standard for webcams

---

## Troubleshooting

### Purple/Blue Color Tint Issue
- **Cause:** YUYV format not converted properly
- **Solution:** Use MJPEG format with `cv2.VideoWriter_fourcc('M','J','P','G')`

### "Device or Resource Busy" Error
- Close all camera applications
- Kill any running camera processes: `pkill -f cam2image`

### Kalibr "Could not read configuration" Error
- Ensure you're running Docker from `~/kalibr_ws/data/` directory
- Verify `target.yaml` exists in the mounted directory

### Low Calibration Quality
- Move calibration board **slower**
- Cover more viewing angles
- Get both close-up and far-away shots
- Ensure good lighting

---

## Tips for Good Calibration

1. **Slow movement** - Kalibr needs time to detect features
2. **Multiple angles** - Tilt board in all directions
3. **Vary distance** - Get close-ups and far shots
4. **Good lighting** - Avoid shadows on the checkerboard
5. **Steady camera** - Don't move the camera, move the board
6. **30-60 seconds** of recording is usually sufficient
7. **Check reprojection error** - Should be <0.5 pixels (ideally <0.2)

---

## Output Files

After calibration, Kalibr generates:

- `camchain-*.yaml` - Camera calibration parameters
- `results-*.txt` - Detailed calibration results
- `report-*.pdf` - Calibration report with visualizations

These files are saved in `~/kalibr_ws/data/`

---

## License

[Your License Here]

## Contributors

[Your Name]

---

## References

- [Kalibr Documentation](https://github.com/ethz-asl/kalibr)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Camera Calibration Tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)

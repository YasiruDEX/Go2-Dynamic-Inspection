# Pipeline Launcher

ROS 2 launch package for the Go2 autonomous exploration pipeline. Provides
modular, configurable launch files for real robot operation, simulation, and
development workflows.

## Features

- **Centralized Configuration**: All topic names, timing delays, and package
  references are defined in `config.py` for easy customization.
- **Reusable Utilities**: Common launch patterns are abstracted in
  `launch_utils.py` to reduce code duplication.
- **Multiple Pipeline Variants**: Pre-configured launch files for different
  use cases (real robot, simulation, mapping-only, development).

## Available Launch Files

| Launch File | Description |
|-------------|-------------|
| `pipeline_real.launch.py` | Full pipeline for real robot with Livox LiDAR |
| `pipeline_simulation.launch.py` | Full pipeline for Gazebo simulation |
| `pipeline_mapping.launch.py` | Mapping-only (DLIO + Open3D SLAM) |
| `pipeline_dev.launch.py` | Quick-start with minimal delays for development |

## Launch Sequence (Real Robot)

```
T=0s   ─── Foxglove Bridge (visualization)
T=5s   ─── DLIO (LiDAR-Inertial Odometry)
T=15s  ─── Open3D SLAM (3D mapping)
T=20s  ─── Vehicle Simulator (motion planning interface)
T=25s  ─── Far Planner (global path planning)
```

## Usage

```bash
# Real robot pipeline
ros2 launch pipeline_launcher pipeline_real.launch.py

# Simulation pipeline
ros2 launch pipeline_launcher pipeline_simulation.launch.py

# Mapping only (no planning)
ros2 launch pipeline_launcher pipeline_mapping.launch.py

# Development mode (fast startup)
ros2 launch pipeline_launcher pipeline_dev.launch.py
```

## Configuration

### Timing Configuration

Edit `launch/config.py` to customize launch delays:

```python
@dataclass(frozen=True)
class TimingConfig:
    foxglove_bridge: float = 0.0
    dlio: float = 5.0
    open3d_slam: float = 15.0
    vehicle_simulator: float = 20.0
    far_planner: float = 25.0
```

### Topic Configuration

Predefined topic configurations for different sensor setups:

```python
# Livox sensors (real robot)
TOPICS_LIVOX = TopicConfig(
    pointcloud="/livox/lidar",
    imu="/livox/imu",
)

# Velodyne sensors (simulation)
TOPICS_VELODYNE = TopicConfig(
    pointcloud="/velodyne_points",
    imu="/imu/data",
)
```

## Dependencies

- `direct_lidar_inertial_odometry` - DLIO
- `open3d_slam_ros` - Open3D SLAM
- `vehicle_simulator` - Motion planning interface
- `far_planner` - Global path planner
- `foxglove_bridge` - Visualization bridge

## Build

```bash
cd ~/Documents/Far_planner_test/workspaces/pipeline_launcher
colcon build --packages-select pipeline_launcher
source install/setup.bash
```

## Project Structure

```
pipeline_launcher/
├── CMakeLists.txt
├── package.xml
├── README.md
└── launch/
    ├── __init__.py           # Package init
    ├── config.py             # Centralized configuration
    ├── launch_utils.py       # Reusable launch utilities
    ├── pipeline_real.launch.py
    ├── pipeline_simulation.launch.py
    ├── pipeline_mapping.launch.py
    └── pipeline_dev.launch.py
```

## License

Apache-2.0

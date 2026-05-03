# mola_mqtt_bridge

ROS2 → MQTT bridge for the **Go2 Mission Planner UI**.  
Forwards MOLA LiDAR-Odometry telemetry to the HiveMQ public broker so the
Go backend (and React frontend) can consume it in real time.

---

## Architecture

```
MOLA LiDAR-Odometry (ROS2)
  │
  ├─ /lidar_odometry/pose          (geometry_msgs/PoseStamped)
  │       └─► MQTT: robot/{robot_id}/status/pose  (JSON)
  │
  └─ /lidar_odometry/localmap_points  (sensor_msgs/PointCloud2)
          └─► MQTT: robo_gen_labs/go2_robot_1/telemetry/points  (binary float32)

                         ▼  HiveMQ broker  ▼

Go Backend (backend_go)
  ├─ subscribes to robot/{robot_id}/status/pose
  │       → stores in PoseMsg, broadcasts over /ws/pose WebSocket
  └─ subscribes to robo_gen_labs/.../telemetry/points
          → stores in PointsMsg, streams over /ws/points WebSocket

React Frontend (frontend)
  ├─ ws://…/ws/pose      → live robot position in map view
  └─ ws://…/ws/points    → live voxel cloud (Three.js Float32Array)
```

---

## Prerequisites

| Requirement | Version |
|---|---|
| ROS2 | Humble / Iron / Jazzy |
| Python | ≥ 3.10 |
| paho-mqtt | ≥ 1.6.1 |

Install the Python MQTT library:
```bash
pip install paho-mqtt
```

---

## Build

Place this package inside any ROS2 workspace `src/` directory, then:

```bash
# From your ROS2 workspace root:
colcon build --packages-select mola_mqtt_bridge
source install/setup.bash
```

Or build it alongside other packages (e.g. inside the mola_lo workspace):
```bash
cd /path/to/ros2_ws
colcon build --symlink-install --packages-select mola_mqtt_bridge
source install/setup.bash
```

---

## Run

```bash
ros2 run mola_mqtt_bridge mola_mqtt_publisher
```

### Environment Variables (optional overrides)

| Variable | Default | Description |
|---|---|---|
| `MQTT_BROKER` | `broker.hivemq.com` | HiveMQ broker hostname |
| `MQTT_PORT` | `1883` | MQTT TCP port |
| `ROBOT_ID` | `robot_01` | Robot identifier (must match Go backend) |

Example with overrides:
```bash
ROBOT_ID=robot_02 ros2 run mola_mqtt_bridge mola_mqtt_publisher
```

---

## MQTT Payload Formats

### Pose — `robot/{robot_id}/status/pose`
```json
{
  "x":  1.23,
  "y": -0.45,
  "z":  0.01,
  "qx": 0.0,
  "qy": 0.0,
  "qz": 0.707,
  "qw": 0.707,
  "frame_id": "map",
  "stamp": 1746395956.123456
}
```

### LocalMap — `robo_gen_labs/go2_robot_1/telemetry/points`

Raw binary payload: a flat array of **little-endian float32** values packed as
`[x0, y0, z0, x1, y1, z1, …]`.  
The Go backend stores this verbatim in `PointsMsg` and forwards it over the
`/ws/points` WebSocket. The React frontend decodes it as a `Float32Array` and
renders it with Three.js.

---

## Go Backend Changes

Two additions were made to `backend_go`:

1. **`mqttclient/client.go`**
   - `PoseMsg []byte` — stores the latest MOLA pose JSON
   - `PoseSubs` + `AddPoseSub` / `RemovePoseSub` / `broadcastPose` — fan-out channels for `/ws/pose`
   - `localmapHandler` — MQTT handler for the localmap points topic
   - `statusHandler` updated — routes pose messages to `broadcastPose` instead of the generic status envelope

2. **`api/websockets.go`** — new `WsPose` handler registered at `/ws/pose`

3. **`main.go`** — `r.GET("/ws/pose", api.WsPose)` route added

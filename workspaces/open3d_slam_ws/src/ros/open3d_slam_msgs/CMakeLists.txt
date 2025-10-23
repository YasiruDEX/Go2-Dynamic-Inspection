cmake_minimum_required(VERSION 3.12)
project(open3d_slam_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

# ------------------ #
# Message generation #
# ------------------ #

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Vertices.msg"
  "msg/PolygonMesh.msg"
  "srv/SaveMap.srv"
  "srv/SaveSubmaps.srv"
  DEPENDENCIES sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime sensor_msgs)

ament_package()

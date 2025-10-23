#pragma once

// Open3D
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/t/geometry/PointCloud.h>

// ROS 2
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"
#include "open3d_slam_msgs/msg/polygon_mesh.hpp"

// Eigen
#include <Eigen/Dense>
#include <string>

namespace open3d_conversions {

void open3dToRos(const open3d::geometry::PointCloud& pointcloud,
                 sensor_msgs::msg::PointCloud2& ros_pc2,
                 const std::string& frame_id = "open3d_pointcloud");

bool rosToOpen3d(const sensor_msgs::msg::PointCloud2& cloud,
                 open3d::geometry::PointCloud& o3d_pc,
                 bool skip_colors = false,
                 bool copy_normals = true);

bool rosToOpen3d(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& ros_pc2,
                 open3d::geometry::PointCloud& o3d_pc,
                 bool skip_colors = false,
                 bool copy_normals = true);

void open3dToRos(const open3d::t::geometry::PointCloud& pointcloud,
                 sensor_msgs::msg::PointCloud2& ros_pc2,
                 const std::string& frame_id = "open3d_pointcloud",
                 int t_num_fields = 2, ...);

void rosToOpen3d(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& ros_pc2,
                 open3d::t::geometry::PointCloud& o3d_tpc,
                 bool skip_colors = false);

void open3dToRos(const open3d::geometry::MeshBase& mesh,
                 const std::string& frame_id,
                 open3d_slam_msgs::msg::PolygonMesh& msg);

void rosToOpen3d(const open3d_slam_msgs::msg::PolygonMesh& msg,
                 open3d::geometry::TriangleMesh& mesh);

void rosToOpen3d(const open3d_slam_msgs::msg::PolygonMesh::ConstSharedPtr& msg,
                 open3d::geometry::TriangleMesh& mesh);

// Helper
int addPointField(sensor_msgs::msg::PointCloud2& cloud_msg,
                  const std::string& name, int count, int datatype, int offset);

int sizeOfPointField(int datatype);

} // namespace open3d_conversions

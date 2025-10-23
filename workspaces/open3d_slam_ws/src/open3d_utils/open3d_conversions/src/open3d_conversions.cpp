// Copyright 2020 Autonomous Robots Lab, University of Nevada, Reno

#include "open3d_conversions/open3d_conversions.hpp"
#include <open3d/core/EigenConverter.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <open3d_slam_msgs/msg/polygon_mesh.hpp>
#include <open3d_slam_msgs/msg/vertices.hpp>

#include <omp.h>
#include <algorithm>
#include <cstring>
#include <sstream>
#include <stdexcept>

namespace open3d_conversions
{

void open3dToRos(
    const open3d::geometry::PointCloud &pointcloud,
    sensor_msgs::msg::PointCloud2 &ros_pc2,
    const std::string &frame_id)
{
  sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
  bool has_colors = pointcloud.HasColors();

  if (has_colors) {
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }

  const size_t num_points = pointcloud.points_.size();
  modifier.resize(num_points);

  ros_pc2.header.frame_id = frame_id;
  ros_pc2.height = 1;
  ros_pc2.width = num_points;
  ros_pc2.is_dense = true;

  const size_t point_step = ros_pc2.point_step;

  int x_offset = -1, y_offset = -1, z_offset = -1;
  int r_offset = -1, g_offset = -1, b_offset = -1;

  for (const auto &field : ros_pc2.fields) {
    if (field.name == "x") x_offset = field.offset;
    else if (field.name == "y") y_offset = field.offset;
    else if (field.name == "z") z_offset = field.offset;
    else if (field.name == "r") r_offset = field.offset;
    else if (field.name == "g") g_offset = field.offset;
    else if (field.name == "b") b_offset = field.offset;
  }

  #pragma omp parallel for simd
  for (int i = 0; i < static_cast<int>(num_points); ++i) {
    uint8_t *ptr = ros_pc2.data.data() + i * point_step;

    const Eigen::Vector3d &p = pointcloud.points_[i];
    *reinterpret_cast<float *>(ptr + x_offset) = static_cast<float>(p.x());
    *reinterpret_cast<float *>(ptr + y_offset) = static_cast<float>(p.y());
    *reinterpret_cast<float *>(ptr + z_offset) = static_cast<float>(p.z());

    if (has_colors) {
      const Eigen::Vector3d &c = pointcloud.colors_[i];
      *(ptr + r_offset) = static_cast<uint8_t>(std::min(255.0, std::max(0.0, 255.0 * c.x())));
      *(ptr + g_offset) = static_cast<uint8_t>(std::min(255.0, std::max(0.0, 255.0 * c.y())));
      *(ptr + b_offset) = static_cast<uint8_t>(std::min(255.0, std::max(0.0, 255.0 * c.z())));
    }
  }
}

bool rosToOpen3d(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &ros_pc2,
    open3d::geometry::PointCloud &o3d_pc,
    bool skip_colors,
    bool copy_normals)
{
  return rosToOpen3d(*ros_pc2, o3d_pc, skip_colors, copy_normals);
}

bool rosToOpen3d(
    const sensor_msgs::msg::PointCloud2 &cloud,
    open3d::geometry::PointCloud &o3d_pc,
    bool skip_colors,
    bool copy_normals)
{
  const uint32_t num_points = cloud.height * cloud.width;
  if (cloud.fields.size() < 3 || num_points == 0) {
    std::cout << "ros_pc2->fields.size(): " << cloud.fields.size() << " Exiting." << std::endl;
    return false;
  }

  int x_offset = -1, y_offset = -1, z_offset = -1;
  int normal_x_offset = -1, normal_y_offset = -1, normal_z_offset = -1;
  int r_offset = -1, g_offset = -1, b_offset = -1;
  bool has_rgb = false;
  bool has_normals = false;

  for (const auto &field : cloud.fields) {
    if (field.name == "x") x_offset = field.offset;
    else if (field.name == "y") y_offset = field.offset;
    else if (field.name == "z") z_offset = field.offset;
    else if (field.name == "normal_x") normal_x_offset = field.offset;
    else if (field.name == "normal_y") normal_y_offset = field.offset;
    else if (field.name == "normal_z") normal_z_offset = field.offset;
    else if (field.name == "r") r_offset = field.offset;
    else if (field.name == "g") g_offset = field.offset;
    else if (field.name == "b") b_offset = field.offset;
  }

  has_normals = (normal_x_offset >= 0 && normal_y_offset >= 0 && normal_z_offset >= 0);
  has_rgb = (r_offset >= 0 && g_offset >= 0 && b_offset >= 0);

  const uint8_t *base_ptr = cloud.data.data();
  const size_t point_step = cloud.point_step;

  o3d_pc.points_.resize(num_points);

  // Use Eigen block-mapping only if data is exactly contiguous for float xyz (x@0, y@4, z@8, point_step==12)
  bool can_eigen_map = (x_offset == 0) && (y_offset == 4) && (z_offset == 8) && (point_step == 12);

  if (can_eigen_map) {
    const float *float_data = reinterpret_cast<const float *>(base_ptr);
    Eigen::Map<const Eigen::Matrix<float, 3, Eigen::Dynamic, Eigen::ColMajor>> mat(float_data, 3, num_points);
    #pragma omp parallel for
    for (int i = 0; i < static_cast<int>(num_points); ++i) {
      o3d_pc.points_[i] = mat.col(i).cast<double>();
    }
  } else {
    bool can_batch_load_xyz = (x_offset == 0) && (y_offset == 4) && (z_offset == 8) && (point_step >= 12);
    if (can_batch_load_xyz) {
      #pragma omp parallel for simd
      for (int i = 0; i < static_cast<int>(num_points); ++i) {
        Eigen::Vector3f temp;
        std::memcpy(&temp, base_ptr + i * point_step, 12);
        o3d_pc.points_[i] = temp.cast<double>();
      }
    } else {
      #pragma omp parallel for simd
      for (int i = 0; i < static_cast<int>(num_points); ++i) {
        const uint8_t *ptr = base_ptr + i * point_step;
        float x = *reinterpret_cast<const float *>(ptr + x_offset);
        float y = *reinterpret_cast<const float *>(ptr + y_offset);
        float z = *reinterpret_cast<const float *>(ptr + z_offset);
        o3d_pc.points_[i] = Eigen::Vector3d(x, y, z);
      }
    }
  }

  if (copy_normals && has_normals) {
    o3d_pc.normals_.resize(num_points);
    bool normals_packed = (normal_x_offset >= 0 && normal_y_offset == normal_x_offset + 4 &&
                           normal_z_offset == normal_x_offset + 8 && point_step == 12);
    if (normals_packed) {
      const float *normal_data = reinterpret_cast<const float *>(base_ptr + normal_x_offset);
      Eigen::Map<const Eigen::Matrix<float, 3, Eigen::Dynamic, Eigen::ColMajor>> mat_normals(normal_data, 3, num_points);
      #pragma omp parallel for
      for (int i = 0; i < static_cast<int>(num_points); ++i) {
        o3d_pc.normals_[i] = mat_normals.col(i).cast<double>();
      }
    } else {
      #pragma omp parallel for simd
      for (int i = 0; i < static_cast<int>(num_points); ++i) {
        const uint8_t *ptr = base_ptr + i * point_step;
        float nx = *reinterpret_cast<const float *>(ptr + normal_x_offset);
        float ny = *reinterpret_cast<const float *>(ptr + normal_y_offset);
        float nz = *reinterpret_cast<const float *>(ptr + normal_z_offset);
        o3d_pc.normals_[i] = Eigen::Vector3d(nx, ny, nz);
      }
    }
  }

  if (skip_colors) {
    return true;
  }

  if (has_rgb) {
    o3d_pc.colors_.resize(num_points);
    #pragma omp parallel for simd
    for (int i = 0; i < static_cast<int>(num_points); ++i) {
      const uint8_t *ptr = base_ptr + i * point_step;
      uint8_t r = *(ptr + r_offset);
      uint8_t g = *(ptr + g_offset);
      uint8_t b = *(ptr + b_offset);
      o3d_pc.colors_[i] = Eigen::Vector3d(
          static_cast<double>(r) / 255.0,
          static_cast<double>(g) / 255.0,
          static_cast<double>(b) / 255.0);
    }
  }
  return true;
}

void open3dToRos(
    const open3d::geometry::MeshBase &mesh,
    const std::string &frame_id,
    open3d_slam_msgs::msg::PolygonMesh &msg)
{
  enum XYZ { x, y, z };
  open3d::geometry::PointCloud point_cloud;
  point_cloud.points_ = mesh.vertices_;
  open3dToRos(point_cloud, msg.cloud, frame_id);

  if (mesh.GetGeometryType() != open3d::geometry::Geometry::GeometryType::TriangleMesh) {
    throw std::runtime_error("Only triangle mesh supported for now!");
  }
  const open3d::geometry::TriangleMesh &triangle_mesh = static_cast<const open3d::geometry::TriangleMesh &>(mesh);

  const int n_triangles = triangle_mesh.triangles_.size();
  msg.polygons.reserve(n_triangles);
  for (int i = 0; i < n_triangles; ++i) {
    open3d_slam_msgs::msg::Vertices triangle;
    triangle.vertices.resize(3);
    triangle.vertices[x] = triangle_mesh.triangles_[i].x();
    triangle.vertices[y] = triangle_mesh.triangles_[i].y();
    triangle.vertices[z] = triangle_mesh.triangles_[i].z();
    msg.polygons.push_back(triangle);
  }
}

void rosToOpen3d(
    const open3d_slam_msgs::msg::PolygonMesh::ConstSharedPtr &msg,
    open3d::geometry::TriangleMesh &mesh)
{
  rosToOpen3d(*msg, mesh);
}

void rosToOpen3d(
    const open3d_slam_msgs::msg::PolygonMesh &msg,
    open3d::geometry::TriangleMesh &mesh)
{
  enum XYZ { x, y, z };
  open3d::geometry::PointCloud point_cloud;
  rosToOpen3d(msg.cloud, point_cloud);
  mesh.vertices_ = point_cloud.points_;

  const int n_triangles = msg.polygons.size();
  mesh.triangles_.reserve(n_triangles);
  for (int i = 0; i < n_triangles; ++i) {
    Eigen::Vector3i triangle(
        msg.polygons[i].vertices[x],
        msg.polygons[i].vertices[y],
        msg.polygons[i].vertices[z]);
    mesh.triangles_.push_back(triangle);
  }
}

// Helper functions

int addPointField(
    sensor_msgs::msg::PointCloud2 &cloud_msg,
    const std::string &name,
    int count,
    int datatype,
    int offset)
{
  sensor_msgs::msg::PointField point_field;
  point_field.name = name;
  point_field.count = count;
  point_field.datatype = datatype;
  point_field.offset = offset;
  cloud_msg.fields.push_back(point_field);

  return offset + point_field.count * sizeOfPointField(datatype);
}

int sizeOfPointField(int datatype)
{
  using sensor_msgs::msg::PointField;
  switch (datatype) {
    case PointField::INT8:
    case PointField::UINT8:
      return 1;
    case PointField::INT16:
    case PointField::UINT16:
      return 2;
    case PointField::INT32:
    case PointField::UINT32:
    case PointField::FLOAT32:
      return 4;
    case PointField::FLOAT64:
      return 8;
    default: {
      std::stringstream err;
      err << "PointField of type " << datatype << " does not exist";
      throw std::runtime_error(err.str());
    }
  }
}

} // namespace open3d_conversions

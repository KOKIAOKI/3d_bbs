#pragma once
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

// std::string createDate() {
//   time_t t = time(nullptr);
//   const tm* local_time = localtime(&t);
//   std::stringstream s;
//   s << "20" << local_time->tm_year - 100 << "_";
//   s << local_time->tm_mon + 1 << "_";
//   s << local_time->tm_mday << "_";
//   s << local_time->tm_hour << "_";
//   s << local_time->tm_min << "_";
//   s << local_time->tm_sec;
//   return (s.str());
// }

template <typename T>
void pcl_to_eigen(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ptr, std::vector<T>& points) {
  points.resize(cloud_ptr->size());
  std::transform(cloud_ptr->begin(), cloud_ptr->end(), points.begin(), [](const pcl::PointXYZ& p) { return T(p.x, p.y, p.z); });
}

void eigen_to_pcl(const std::vector<Eigen::Vector3d>& points, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ptr) {
  cloud_ptr->points.resize(points.size());
  cloud_ptr->width = points.size();
  cloud_ptr->height = 1;
  cloud_ptr->is_dense = true;

  std::transform(points.begin(), points.end(), cloud_ptr->points.begin(), [](const Eigen::Vector3d& v) {
    pcl::PointXYZ point;
    Eigen::Vector3f vf = v.cast<float>();
    point.x = vf.x();
    point.y = vf.y();
    point.z = vf.z();
    return point;
  });
}

void gravity_align(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ptr,
  pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned_cloud_ptr,
  const sensor_msgs::msg::Imu& imu_msg) {
  Eigen::Vector3f acc(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
  Eigen::Vector3f th;
  th.x() = std::atan2(acc.y(), acc.z());
  th.y() = std::atan2(-acc.x(), std::sqrt(acc.y() * acc.y() + acc.z() * acc.z()));
  th.z() = 0.0f;

  Eigen::Matrix3f rot;
  rot = Eigen::AngleAxisf(th.x(), Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(th.y(), Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(th.z(), Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
  mat.block<3, 3>(0, 0) = rot;

  pcl::transformPointCloud(*cloud_ptr, *aligned_cloud_ptr, mat);
}
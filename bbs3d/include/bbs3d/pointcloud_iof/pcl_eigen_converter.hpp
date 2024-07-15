#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <vector>

namespace pciof {
template <typename T>
void pcl_to_eigen(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ptr, std::vector<T>& points) {
  // nullptr check
  if (!cloud_ptr) {
    return;
  }

  points.resize(cloud_ptr->size());
  std::transform(cloud_ptr->begin(), cloud_ptr->end(), points.begin(), [](const pcl::PointXYZ& p) { return T(p.x, p.y, p.z); });
}

template <typename T>
std::vector<Eigen::Matrix<T, 3, 1>> pcl_to_eigen(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ptr) {
  if (!cloud_ptr) {
    return {};
  }

  std::vector<Eigen::Matrix<T, 3, 1>> points(cloud_ptr->size());
  std::transform(cloud_ptr->begin(), cloud_ptr->end(), points.begin(), [](const pcl::PointXYZ& p) { return Eigen::Matrix<T, 3, 1>(p.x, p.y, p.z); });
  return points;
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

pcl::PointCloud<pcl::PointXYZ>::Ptr eigen_to_pcl(const std::vector<Eigen::Vector3d>& points) {
  auto cloud_ptr = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
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

  return cloud_ptr;
}

void eigen_to_pcl(const std::vector<Eigen::Vector3f>& points, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ptr) {
  cloud_ptr->points.resize(points.size());
  cloud_ptr->width = points.size();
  cloud_ptr->height = 1;
  cloud_ptr->is_dense = true;

  std::transform(points.begin(), points.end(), cloud_ptr->points.begin(), [](const Eigen::Vector3f& v) {
    pcl::PointXYZ point;
    point.x = v.x();
    point.y = v.y();
    point.z = v.z();
    return point;
  });
}

pcl::PointCloud<pcl::PointXYZ>::Ptr eigen_to_pcl(const std::vector<Eigen::Vector3f>& points) {
  auto cloud_ptr = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud_ptr->points.resize(points.size());
  cloud_ptr->width = points.size();
  cloud_ptr->height = 1;
  cloud_ptr->is_dense = true;

  std::transform(points.begin(), points.end(), cloud_ptr->points.begin(), [](const Eigen::Vector3f& v) {
    pcl::PointXYZ point;
    point.x = v.x();
    point.y = v.y();
    point.z = v.z();
    return point;
  });

  return cloud_ptr;
}
}  // namespace pciof

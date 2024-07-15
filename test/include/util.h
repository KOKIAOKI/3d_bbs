#pragma once
#include <filesystem>
#include <iostream>
#include <vector>

#include <small_gicp/points/point_cloud.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

std::string create_date() {
  time_t t = time(nullptr);
  const tm* local_time = localtime(&t);
  std::stringstream s;
  s << "20" << local_time->tm_year - 100 << "_";
  s << local_time->tm_mon + 1 << "_";
  s << local_time->tm_mday << "_";
  s << local_time->tm_hour << "_";
  s << local_time->tm_min << "_";
  s << local_time->tm_sec;
  return (s.str());
}

bool can_convert_to_int(const std::vector<std::string>& name_vec) {
  for (const auto& name : name_vec) {
    try {
      std::stoi(std::filesystem::path(name).stem().string());
    } catch (const std::invalid_argument& e) {
      return false;
    } catch (const std::out_of_range& e) {
      return false;
    }
  }
  return true;
}

small_gicp::PointCloud::Ptr pcl_to_small_gicp(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud_ptr) {
  // transform pcl_cloud_ptr to PointCloudPtr
  auto cloud_ptr = std::make_shared<small_gicp::PointCloud>();
  std::transform(pcl_cloud_ptr->points.begin(), pcl_cloud_ptr->points.end(), std::back_inserter(cloud_ptr->points), [](const pcl::PointXYZ& point) {
    return Eigen::Vector4d(point.x, point.y, point.z, 1.0);
  });

  return cloud_ptr;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr snall_gicp_to_pcl(const small_gicp::PointCloud::Ptr& cloud_ptr) {
  // transform PointCloudPtr to pcl_cloud_ptr
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  std::transform(cloud_ptr->points.begin(), cloud_ptr->points.end(), std::back_inserter(pcl_cloud_ptr->points), [](const Eigen::Vector4d& point) {
    return pcl::PointXYZ(point(0), point(1), point(2));
  });
  return pcl_cloud_ptr;
}
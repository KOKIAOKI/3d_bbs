#pragma once
#include <filesystem>
#include <iostream>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace pciof {
inline pcl::PointCloud<pcl::PointXYZ>::Ptr load_point_cloud_file(const std::string& path) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  const auto extension = std::filesystem::path(path).extension().string();
  if (extension == ".pcd") {
    if (pcl::io::loadPCDFile(path, *cloud_ptr) == -1) {
      std::cout << "[WARN] Can not open pcd file: " << path << std::endl;
      return cloud_ptr;
    }
  }
  return cloud_ptr;
}

inline pcl::PointCloud<pcl::PointXYZ>::Ptr create_tar_cloud(const std::vector<std::string>& cloud_set) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  for (const auto& file : cloud_set) {
    const auto cloud_temp_ptr = pciof::load_point_cloud_file(file);
    *cloud_ptr += *cloud_temp_ptr;
  }
  return cloud_ptr;
}
}  // namespace pciof
#pragma once
#include <filesystem>
#include <iostream>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace pciof {
std::vector<std::string> find_point_cloud_files(const std::string& path) {
  std::filesystem::path dir(path);
  std::vector<std::string> files;

  if (!std::filesystem::exists(dir)) {
    std::cout << "[ERROR] Cannot open folder" << std::endl;
    return files;
  }

  for (const auto& entry : std::filesystem::directory_iterator(dir)) {
    const std::string extension = entry.path().extension().string();
    if (extension == ".pcd") {
      files.emplace_back(entry.path().string());
    }
  }

  return files;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr load_point_cloud_file(const std::string& path) {
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

pcl::PointCloud<pcl::PointXYZ>::Ptr create_tar_cloud(const std::vector<std::string>& cloud_set) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  for (const auto& file : cloud_set) {
    const auto cloud_temp_ptr = pciof::load_point_cloud_file(file);
    *cloud_ptr += *cloud_temp_ptr;
  }
  return cloud_ptr;
}
}  // namespace pciof
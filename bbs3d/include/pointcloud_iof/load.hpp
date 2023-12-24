#pragma once
#include <pointcloud_iof/io.hpp>
#include <pointcloud_iof/filter.hpp>
#include <boost/filesystem.hpp>

namespace pciof {
template <typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;

template <typename T>
bool load_tar_clouds(const std::string& tar_folder_path, const T& filter_voxel_width, std::vector<Vector3<T>>& points) {
  boost::filesystem::path dir(tar_folder_path);
  if (!boost::filesystem::exists(dir)) {
    std::cout << "[ERROR] Can not open floder" << std::endl;
    return false;
  }

  for (const auto& file : boost::filesystem::directory_iterator(tar_folder_path)) {
    const std::string file_name = file.path().c_str();
    const std::string extension = file.path().extension().string();
    if (extension != ".pcd" && extension != ".PCD") {
      continue;
    }

    const auto loaded_points = read_pcd<T>(file_name);

    if (loaded_points.empty()) {
      std::cout << "[ERROR] Failed to read point cloud from " << file_name << std::endl;
      return false;
    }

    const auto filtered_points = filter<T>(loaded_points, filter_voxel_width);

    points.reserve(points.size() + filtered_points.size());
    points.insert(points.end(), filtered_points.begin(), filtered_points.end());
  }
  return true;
}

std::vector<std::pair<std::string, std::string>> load_pcd_file_paths(const std::string& folder_path) {
  boost::filesystem::path dir(folder_path);
  if (!boost::filesystem::exists(dir)) {
    return {};  // output error
  }

  std::vector<std::pair<std::string, std::string>> pcd_files;
  for (const auto& file : boost::filesystem::directory_iterator(folder_path)) {
    const std::string extension = file.path().extension().string();
    if (extension != ".pcd" && extension != ".PCD") {
      continue;
    }
    pcd_files.emplace_back(file.path().string(), file.path().stem().string());
  }

  std::sort(pcd_files.begin(), pcd_files.end(), [](const std::pair<std::string, std::string>& a, const std::pair<std::string, std::string>& b) {
    return a.second < b.second;
  });

  return pcd_files;
}

template <typename T>
bool load_src_clouds(std::string src_folder_path, T min_range, T max_range, T voxel_width, std::vector<std::vector<Vector3<T>>>& src_points) {
  const auto pcd_files = load_pcd_file_paths(src_folder_path);
  if (pcd_files.empty()) {
    std::cout << "[ERROR] No pcd files in the folder" << std::endl;
    return false;
  }

  src_points.resize(pcd_files.size());

  for (int i = 0; i < pcd_files.size(); i++) {
    const auto& pcd_file = pcd_files[i];
    const auto loaded_points = read_pcd<T>(pcd_file.first);
    if (loaded_points.empty()) {
      std::cout << "[ERROR] Failed to read point cloud from " << pcd_file.first << std::endl;
      return false;
    }

    const auto narrow_scan_range_points = narrow_scan_range<T>(loaded_points, min_range, max_range);
    const auto filtered_points = filter<T>(narrow_scan_range_points, voxel_width);

    src_points[i].reserve(filtered_points.size());
    src_points[i].insert(src_points[i].end(), filtered_points.begin(), filtered_points.end());
  }
}

template <typename T>
bool load_src_clouds_with_filename(
  std::string src_folder_path,
  T min_range,
  T max_range,
  T voxel_width,
  std::pair<std::string, std::vector<Vector3<T>>>& src_points) {
  const auto pcd_files = load_pcd_file_paths(src_folder_path);
  if (pcd_files.empty()) {
    std::cout << "[ERROR] No pcd files in the folder" << std::endl;
    return false;
  }

  src_points.resize(pcd_files.size());

  for (int i = 0; i < pcd_files.size(); i++) {
    const auto& pcd_file = pcd_files[i];
    const auto loaded_points = read_pcd<T>(pcd_file.first);
    if (loaded_points.empty()) {
      std::cout << "[ERROR] Failed to read point cloud from " << pcd_file.first << std::endl;
      return false;
    }

    const auto narrow_scan_range_points = narrow_scan_range<T>(loaded_points, min_range, max_range);
    const auto filtered_points = filter<T>(narrow_scan_range_points, voxel_width);

    src_points[i].first = pcd_file.second;
    src_points[i].second.reserve(filtered_points.size());
    src_points[i].second.insert(src_points[i].second.end(), filtered_points.begin(), filtered_points.end());
  }
}
}  // namespace pciof
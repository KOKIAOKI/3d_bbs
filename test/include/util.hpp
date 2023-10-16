#pragma once
#include <test.hpp>
#include <boost/filesystem/operations.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/common/distances.h>

std::string createDate() {
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

template <typename T>
bool BBS3DTest::load_tar_clouds(std::vector<T>& points) {
  // Load pcd file
  boost::filesystem::path dir(tar_path);
  if (!boost::filesystem::exists(dir)) {
    std::cout << "[ERROR] Can not open floder" << std::endl;
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  for (const auto& file : boost::filesystem::directory_iterator(tar_path)) {
    const std::string filename = file.path().c_str();
    const std::string extension = file.path().extension().string();
    if (extension != ".pcd" && extension != ".PCD") {
      continue;
    }

    // Check load pcd
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(filename, *cloud_temp_ptr) == -1) {
      std::cout << "[WARN] Can not open pcd file: " << filename << std::endl;
      continue;
    }
    *cloud_ptr += *cloud_temp_ptr;
  }

  if (use_gicp) gicp_ptr->setInputTarget(cloud_ptr);

  // Downsample
  pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  if (valid_tar_vgf) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(tar_leaf_size, tar_leaf_size, tar_leaf_size);
    filter.setInputCloud(cloud_ptr);
    filter.filter(*filtered_cloud_ptr);
    *tar_cloud_ptr = *filtered_cloud_ptr;
  } else {
    *tar_cloud_ptr = *cloud_ptr;
  }

  // pcl to eigen
  pcl_to_eigen(tar_cloud_ptr, points);
  return true;
}

template <typename T>
bool BBS3DTest::load_src_clouds(std::map<std::string, std::vector<T>>& points_set) {
  boost::filesystem::path dir(src_path);
  if (!boost::filesystem::exists(dir)) {
    std::cout << "[ERROR] Can not open floder" << std::endl;
    return false;
  }

  std::vector<boost::filesystem::directory_entry> pcd_files;
  for (const auto& file : boost::filesystem::directory_iterator(src_path)) {
    const std::string extension = file.path().extension().string();
    if (extension != ".pcd" && extension != ".PCD") {
      continue;
    }
    pcd_files.emplace_back(file);
  }

  // TODO
  std::sort(pcd_files.begin(), pcd_files.end(), [](const boost::filesystem::directory_entry& a, const boost::filesystem::directory_entry& b) {
    return a.path().stem().string() < b.path().stem().string();
  });

  for (const auto& file : pcd_files) {
    const std::string file_path = file.path().c_str();
    const std::string file_name = file.path().stem().string();

    // check load pcd
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(file_path, *cloud_ptr) == -1) {
      std::cout << "[WARN] Can not open pcd file: " << file_path << std::endl;
      continue;
    }

    // Downsample
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    if (valid_src_vgf) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::VoxelGrid<pcl::PointXYZ> filter;
      filter.setLeafSize(src_leaf_size, src_leaf_size, src_leaf_size);
      filter.setInputCloud(cloud_ptr);
      filter.filter(*filtered_ptr);
      *src_cloud_ptr = *filtered_ptr;
    } else {
      *src_cloud_ptr = *cloud_ptr;
    }

    // Cut scan range
    if (cut_src_points) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

      for (size_t i = 0; i < src_cloud_ptr->points.size(); ++i) {
        pcl::PointXYZ point = src_cloud_ptr->points[i];
        double norm = pcl::euclideanDistance(point, pcl::PointXYZ(0, 0, 0));

        if (norm >= scan_range.first && norm <= scan_range.second) {
          filtered_cloud_ptr->points.push_back(point);
        }
      }
      *src_cloud_ptr = *filtered_cloud_ptr;
    }

    std::vector<T> points;
    // pcl to eigen
    pcl_to_eigen(src_cloud_ptr, points);

    points_set[file_name] = points;
  }
  return true;
}
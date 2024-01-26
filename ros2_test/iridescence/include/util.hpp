#pragma once
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

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

bool load_tar_clouds(const std::string& tar_path, const float tar_leaf_size, pcl::PointCloud<pcl::PointXYZ>::Ptr& tar_cloud_ptr) {
  // Load pcd file
  boost::filesystem::path dir(tar_path);
  if (!boost::filesystem::exists(dir)) {
    std::cout << "[ERROR] Can not open folder" << std::endl;
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

  // Downsample
  if (tar_leaf_size != 0.0f) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(tar_leaf_size, tar_leaf_size, tar_leaf_size);
    filter.setInputCloud(cloud_ptr);
    filter.filter(*filtered_cloud_ptr);
    *tar_cloud_ptr = *filtered_cloud_ptr;
  } else {
    *tar_cloud_ptr = *cloud_ptr;
  }

  return true;
}

Eigen::Matrix4f gravity_align(const sensor_msgs::msg::Imu& imu_msg) {
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

  return mat;
}

void gravity_align(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ptr,
  pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned_cloud_ptr,
  const sensor_msgs::msg::Imu& imu_msg) {
  Eigen::Matrix4f mat = gravity_align(imu_msg);
  pcl::transformPointCloud(*cloud_ptr, *aligned_cloud_ptr, mat);
}

void transform_pointcloud(
  const std::vector<Eigen::Vector3f>& source_points,
  std::vector<Eigen::Vector3f>& output_points,
  const Eigen::Matrix4f& trans_matrix) {
  output_points.clear();
  output_points.reserve(source_points.size());

  for (const auto& point : source_points) {
    Eigen::Vector4f homog_point(point[0], point[1], point[2], 1.0f);
    Eigen::Vector4f transformed_homog = trans_matrix * homog_point;
    Eigen::Vector3f transformed_point = transformed_homog.head<3>() / transformed_homog[3];
    output_points.push_back(transformed_point);
  }
}
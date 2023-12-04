#include <ros2_test.hpp>
#include <load.hpp>
#include <util.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

ROS2Test::ROS2Test(const rclcpp::NodeOptions& node_options) : Node("gpu_ros2_test", node_options) {
  //  ==== Load config file ====
  std::cout << "[ROS2] Loading config file..." << std::endl;
  std::string config = this->declare_parameter<std::string>("config");
  if (!load_config(config)) {
    std::cout << "[ERROR] Loading config file failed" << std::endl;
    return;
  };

  std::cout << "[ROS2] Loading target clouds..." << std::endl;
  std::vector<Eigen::Vector3f> tar_points;
  if (!load_tar_clouds(tar_points)) {
    std::cout << "[ERROR] Couldn't load target clouds" << std::endl;
    return;
  }
  std::cout << "[ROS2] Target clouds loaded" << std::endl;

  std::cout << "[Voxel map] Creating hierarchical voxel map..." << std::endl;
  std::unique_ptr<gpu::BBS3D> gpu_bbs3d_ptr(new gpu::BBS3D);
  gpu_bbs3d_ptr->set_tar_points(tar_points, min_level_res, max_level);
  gpu_bbs3d_ptr->set_angular_search_range(min_rpy.cast<float>(), max_rpy.cast<float>());
  gpu_bbs3d_ptr->set_score_threshold_percentage(static_cast<float>(score_threshold_percentage));
  std::cout << "[ROS2] 3D-BBS initialized" << std::endl;

  // ==== ROS2 sub====
  cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic_name,
    rclcpp::SensorDataQoS(),
    std::bind(&ROS2Test::cloud_callback, this, std::placeholders::_1));

  imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_name,
    rclcpp::SensorDataQoS(),
    std::bind(&ROS2Test::imu_callback, this, std::placeholders::_1));
}

ROS2Test::~ROS2Test() {}

template <typename T>
bool ROS2Test::load_tar_clouds(std::vector<T>& points) {
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

void ROS2Test::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!imu_buffer.size()) return;

  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *src_cloud);

  // filter
  if (valid_src_vgf) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(src_leaf_size, src_leaf_size, src_leaf_size);
    filter.setInputCloud(src_cloud);
    filter.filter(*filtered_cloud_ptr);
    *src_cloud = *filtered_cloud_ptr;
  }

  // Cut scan range
  if (cut_src_points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < src_cloud->points.size(); ++i) {
      pcl::PointXYZ point = src_cloud->points[i];
      double norm = pcl::euclideanDistance(point, pcl::PointXYZ(0, 0, 0));

      if (norm >= scan_range.first && norm <= scan_range.second) {
        cut_cloud_ptr->points.push_back(point);
      }
    }
    *src_cloud = *cut_cloud_ptr;
  }

  // get index of imu buffer that is closest to the current timestamp
  int imu_index = 0;
  double min_diff = 1000;
  for (int i = 0; i < imu_buffer.size(); ++i) {
    double diff = std::abs(imu_buffer[i].header.stamp.sec - msg->header.stamp.sec);
    if (diff < min_diff) {
      imu_index = i;
      min_diff = diff;
    }
  }

  gravity_align(src_cloud, src_cloud, imu_buffer[imu_index]);
  std::vector<Eigen::Vector3f> src_points;
  pcl_to_eigen(src_cloud, src_points);
  gpu_bbs3d_ptr->set_src_points(src_points);
  gpu_bbs3d_ptr->localize();

  if (!gpu_bbs3d_ptr->has_localized()) {
    std::cout << "[Failed] Score is below the threshold." << std::endl;
    return;
  }

  std::cout << "[Localize] Score: " << gpu_bbs3d_ptr->get_best_score() << std::endl;
}

void ROS2Test::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  imu_buffer.emplace_back(*msg);
  if (imu_buffer.size() > 100) {
    imu_buffer.erase(imu_buffer.begin());
  }
}

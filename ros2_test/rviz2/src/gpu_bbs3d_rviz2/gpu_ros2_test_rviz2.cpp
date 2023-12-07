#include <ros2_test_rviz2.hpp>
#include <load_rviz2.hpp>
#include <util.hpp>
#include <chrono>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

ROS2Test::ROS2Test(const rclcpp::NodeOptions& node_options) : Node("gpu_ros2_test_rviz2", node_options), tf2_broadcaster_(*this) {
  //  ==== Load config file ====
  std::cout << "[ROS2] Loading config file..." << std::endl;
  std::string config = this->declare_parameter<std::string>("config");
  if (!load_config(config)) {
    std::cout << "[ERROR] Loading config file failed" << std::endl;
    return;
  };

  // ==== ROS2 sub====
  click_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/click_loc",
    rclcpp::SensorDataQoS(),
    std::bind(&ROS2Test::click_callback, this, std::placeholders::_1));

  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic_name,
    100,
    std::bind(&ROS2Test::cloud_callback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_name, 100, std::bind(&ROS2Test::imu_callback, this, std::placeholders::_1));

  //==== ROS2 pub ====
  tar_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/tar_points", 10);
  src_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/src_points_on_global_pose", 10);
  global_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/global_pose", 10);
  score_pub_ = this->create_publisher<std_msgs::msg::Int32>("/score", 10);
  time_pub_ = this->create_publisher<std_msgs::msg::Float32>("/time", 10);

  std::cout << "[ROS2] Loading target clouds..." << std::endl;
  std::vector<Eigen::Vector3f> tar_points;
  if (!load_tar_clouds(tar_points)) {
    std::cout << "[ERROR] Couldn't load target clouds" << std::endl;
    return;
  }
  std::cout << "[ROS2] Target clouds loaded" << std::endl;

  std::cout << "[Voxel map] Creating hierarchical voxel map..." << std::endl;
  gpu_bbs3d.set_tar_points(tar_points, min_level_res, max_level);
  gpu_bbs3d.set_angular_search_range(min_rpy.cast<float>(), max_rpy.cast<float>());
  gpu_bbs3d.set_score_threshold_percentage(static_cast<float>(score_threshold_percentage));
  std::cout << "*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*" << std::endl;
  std::cout << "   [ROS2] 3D-BBS initialized" << std::endl;
  std::cout << "*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*" << std::endl;
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

  std::this_thread::sleep_for(std::chrono::seconds(1));

  // publish map
  sensor_msgs::msg::PointCloud2::SharedPtr points_msg(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*tar_cloud_ptr, *points_msg);
  points_msg->header.frame_id = "map";
  points_msg->header.stamp = this->now();
  tar_points_pub_->publish(*points_msg);

  // pcl to eigen
  pcl_to_eigen(tar_cloud_ptr, points);

  // broadcast viewer frame
  broadcast_viewer_frame(points);
  return true;
}

void ROS2Test::broadcast_viewer_frame(const std::vector<Eigen::Vector3f>& points) {
  // Calculate the center of the point cloud
  Eigen::Vector3d inv_vec = -points[0].cast<double>();
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto& point : points) {
    centroid += (point.cast<double>() + inv_vec);
  }
  centroid /= points.size();
  centroid += points[0].cast<double>();

  std::cout << "[Viewer]: "
            << "x: " << centroid[0] << ", y: " << centroid[1] << ", z: " << centroid[2] << std::endl;

  // Create a transform message
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = this->now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "viewer";
  transformStamped.transform.translation.x = static_cast<float>(centroid[0]);
  transformStamped.transform.translation.y = static_cast<float>(centroid[1]);
  transformStamped.transform.translation.z = static_cast<float>(centroid[2]);
  transformStamped.transform.rotation.x = 0.0;
  transformStamped.transform.rotation.y = 0.0;
  transformStamped.transform.rotation.z = 0.0;
  transformStamped.transform.rotation.w = 1.0;

  // Broadcast the transform
  tf2_broadcaster_.sendTransform(transformStamped);
}

void ROS2Test::click_callback(const std_msgs::msg::Bool::SharedPtr msg) {
  std::cout << "click callback" << std::endl;
  if (!imu_buffer.size()) return;
  auto start = std::chrono::system_clock::now();

  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*source_cloud_msg_, *src_cloud);

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
      double norm = pcl::euclideanDistance(point, pcl::PointXYZ(0.0f, 0.0f, 0.0f));

      if (norm >= scan_range.first && norm <= scan_range.second) {
        cut_cloud_ptr->points.push_back(point);
      }
    }
    *src_cloud = *cut_cloud_ptr;
  }

  int imu_index = get_nearest_imu_index(imu_buffer, source_cloud_msg_->header.stamp);
  gravity_align(src_cloud, src_cloud, imu_buffer[imu_index]);

  std::vector<Eigen::Vector3f> src_points;
  pcl_to_eigen(src_cloud, src_points);
  gpu_bbs3d.set_src_points(src_points);

  std::cout << "[Localize] start" << std::endl;
  // auto start_loc = std::chrono::system_clock::now();
  gpu_bbs3d.localize();  // gloal localization
  // auto end_loc = std::chrono::system_clock::now();

  if (!gpu_bbs3d.has_localized()) {
    std::cout << "[Failed] Score is below the threshold." << std::endl;
    return;
  }

  auto end = std::chrono::system_clock::now();
  std::cout << "[Localize] time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0f << "ms" << std::endl;
  std::cout << "[Localize] score: " << gpu_bbs3d.get_best_score() << std::endl;

  // publish results
  float time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0f;
  publish_results(source_cloud_msg_->header, src_cloud, gpu_bbs3d.get_global_pose(), gpu_bbs3d.get_best_score(), time);
}

int ROS2Test::get_nearest_imu_index(const std::vector<sensor_msgs::msg::Imu>& imu_buffer, const builtin_interfaces::msg::Time& stamp) {
  int imu_index = 0;
  double min_diff = 1000;
  for (int i = 0; i < imu_buffer.size(); ++i) {
    double diff = std::abs(
      imu_buffer[i].header.stamp.sec + imu_buffer[i].header.stamp.nanosec * 1e-9 - source_cloud_msg_->header.stamp.sec -
      source_cloud_msg_->header.stamp.nanosec * 1e-9);
    if (diff < min_diff) {
      imu_index = i;
      min_diff = diff;
    }
  }
  return imu_index;
}

void ROS2Test::publish_results(
  const std_msgs::msg::Header& header,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& points_cloud_ptr,
  const Eigen::Matrix4f& best_pose,
  const int best_score,
  const float time) {
  // Publish points
  sensor_msgs::msg::PointCloud2::SharedPtr points_msg(new sensor_msgs::msg::PointCloud2);
  pcl::transformPointCloud(*points_cloud_ptr, *points_cloud_ptr, best_pose);
  pcl::toROSMsg(*points_cloud_ptr, *points_msg);
  points_msg->header.frame_id = "map";
  points_msg->header.stamp = header.stamp;
  src_points_pub_->publish(*points_msg);

  // Publish global pose
  geometry_msgs::msg::PoseStamped::SharedPtr global_pose_msg(new geometry_msgs::msg::PoseStamped);
  global_pose_msg->header.frame_id = "map";
  global_pose_msg->header.stamp = header.stamp;
  global_pose_msg->pose.position.x = best_pose(0, 3);
  global_pose_msg->pose.position.y = best_pose(1, 3);
  global_pose_msg->pose.position.z = best_pose(2, 3);
  Eigen::Quaternionf q(best_pose.block<3, 3>(0, 0));
  global_pose_msg->pose.orientation.x = q.x();
  global_pose_msg->pose.orientation.y = q.y();
  global_pose_msg->pose.orientation.z = q.z();
  global_pose_msg->pose.orientation.w = q.w();
  global_pose_pub_->publish(*global_pose_msg);

  // Publish score
  std_msgs::msg::Int32::SharedPtr score_msg(new std_msgs::msg::Int32);
  score_msg->data = best_score;
  score_pub_->publish(*score_msg);

  // Publish time
  std_msgs::msg::Float32::SharedPtr time_msg(new std_msgs::msg::Float32);
  time_msg->data = time;
  time_pub_->publish(*time_msg);
}

void ROS2Test::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!msg) return;
  source_cloud_msg_ = msg;
}

void ROS2Test::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (!msg) return;
  imu_buffer.emplace_back(*msg);
  if (imu_buffer.size() > 30) {
    imu_buffer.erase(imu_buffer.begin());
  }
}

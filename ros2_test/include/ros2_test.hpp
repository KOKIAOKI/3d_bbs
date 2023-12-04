#pragma once

#include <iostream>
#include <Eigen/Core>
#include <boost/filesystem.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <gpu_bbs3d/bbs3d.cuh>

class ROS2Test : public rclcpp::Node {
public:
  ROS2Test(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
  ~ROS2Test();

private:
  bool load_config(const std::string& config);
  template <typename T>
  bool load_tar_clouds(std::vector<T>& points);
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

  std::vector<sensor_msgs::msg::Imu> imu_buffer;

  // path
  std::string tar_path;

  // topic name
  std::string lidar_topic_name, imu_topic_name;

  // 3D-BBS parameters
  double min_level_res;
  int max_level;

  // angular search range
  Eigen::Vector3d min_rpy;
  Eigen::Vector3d max_rpy;

  // score threshold percentage
  double score_threshold_percentage;

  // downsample
  bool valid_tar_vgf, valid_src_vgf;
  float tar_leaf_size, src_leaf_size;
  bool cut_src_points;
  std::pair<double, double> scan_range;

  // #ifdef BUILD_CUDA
  std::unique_ptr<gpu::BBS3D> gpu_bbs3d_ptr;
  // #endif
};
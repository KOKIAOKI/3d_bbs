#pragma once

#include <iostream>
#include <thread>
#include <Eigen/Core>
#include <boost/filesystem.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <gpu_bbs3d/bbs3d.cuh>

// iridescence
#include <glk/cuda_magic_headers.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <guik/hovered_drawings.hpp>

class ROS2Test : public rclcpp::Node {
public:
  ROS2Test(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
  ~ROS2Test();

private:
  bool load_config(const std::string& config);
  void click_callback();
  int get_nearest_imu_index(const std::vector<sensor_msgs::msg::Imu>& imu_buffer, const builtin_interfaces::msg::Time& stamp);
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // msg
  sensor_msgs::msg::PointCloud2::SharedPtr source_cloud_msg_;
  std::vector<sensor_msgs::msg::Imu> imu_buffer;

  // iridescence
  rclcpp::TimerBase::SharedPtr timer_;

  // 3D-BBS
  gpu::BBS3D gpu_bbs3d;

  // Config
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
  float tar_leaf_size, src_leaf_size;
  double min_scan_range, max_scan_range;

  // timeout
  int timeout_msec;
};

#pragma once

#include <iostream>
#include <thread>
#include <Eigen/Core>
#include <boost/filesystem.hpp>

// ros2
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <gpu_bbs3d/bbs3d.cuh>

class ROS2Test : public rclcpp::Node {
public:
  ROS2Test(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
  ~ROS2Test();

private:
  bool load_config(const std::string& config);
  void broadcast_viewer_frame(const std::vector<Eigen::Vector3f>& points);
  void click_callback(const std_msgs::msg::Bool::SharedPtr msg);
  int get_nearest_imu_index(const std::vector<sensor_msgs::msg::Imu>& imu_buffer, const builtin_interfaces::msg::Time& stamp);
  void publish_results(
    const std_msgs::msg::Header& header,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& points_cloud_ptr,
    const Eigen::Matrix4f& best_pose,
    const int best_score,
    const float time);
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // sub
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr click_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // pub
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tar_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr src_points_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr score_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr time_pub_;

  // tf
  tf2_ros::TransformBroadcaster tf2_broadcaster_;

  // msg buffer
  sensor_msgs::msg::PointCloud2::SharedPtr source_cloud_msg_;
  std::vector<sensor_msgs::msg::Imu> imu_buffer;

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

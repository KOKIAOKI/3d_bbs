#include <pointcloud_iof/pcl_eigen_converter.hpp>
#include <pointcloud_iof/pcd_loader.hpp>
#include <pointcloud_iof/gravity_alignment.hpp>
#include <ros2_test.hpp>
#include <load.hpp>
#include <chrono>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

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

ROS2Test::ROS2Test(const rclcpp::NodeOptions& node_options) : Node("gpu_ros2_test_iridescence", node_options) {
  //  ==== Load config file ====
  std::cout << "[ROS2] Loading config file..." << std::endl;
  std::string config = this->declare_parameter<std::string>("config");
  if (!load_config(config)) {
    std::cout << "[ERROR] Loading config file failed" << std::endl;
  };

  // ==== Set target cloud ====
  std::cout << "[ROS2] Loading target clouds..." << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  if (!pciof::load_tar_clouds(tar_path, tar_leaf_size, tar_cloud_ptr)) {
    std::cout << "[ERROR] Couldn't load target clouds" << std::endl;
  }

  // pcl to eigen
  std::vector<Eigen::Vector3f> tar_points;
  pciof::pcl_to_eigen(tar_cloud_ptr, tar_points);

  std::cout << "[Voxel map] Creating hierarchical voxel map..." << std::endl;
  if (gpu_bbs3d.set_voxelmaps_coords(tar_path)) {
    std::cout << "[Voxel map] Loaded voxelmaps coords directly" << std::endl;
  } else {
    gpu_bbs3d.set_tar_points(tar_points, min_level_res, max_level);
    gpu_bbs3d.set_trans_search_range(tar_points);
  }

  gpu_bbs3d.set_angular_search_range(min_rpy.cast<float>(), max_rpy.cast<float>());
  gpu_bbs3d.set_score_threshold_percentage(static_cast<float>(score_threshold_percentage));
  if (timeout_msec > 0) {
    gpu_bbs3d.enable_timeout();
    gpu_bbs3d.set_timeout_duration_in_msec(timeout_msec);
  }

  // ==== ROS 2 sub ====
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic_name,
    rclcpp::QoS(rclcpp::KeepLast(50)).best_effort(),
    std::bind(&ROS2Test::cloud_callback, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_name, 100, std::bind(&ROS2Test::imu_callback, this, std::placeholders::_1));

  // ==== GUI ====
  auto viewer = guik::viewer();

  auto trg_buffer = std::make_shared<glk::PointCloudBuffer>(tar_points);
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  auto shader_setting_trg = guik::Rainbow(transformation).set_point_scale(1.0f);
  viewer->update_drawable("trg", trg_buffer, shader_setting_trg);
  glk::COLORMAP colormap = glk::COLORMAP::OCEAN;
  viewer->set_colormap(colormap);
  viewer->use_topdown_camera_control();
  viewer->lookat(tar_points[0]);
  viewer->register_ui_callback("ui_callback", [&]() {
    if (ImGui::Button("Localize")) {
      click_callback();
    }
  });

  auto timer_callback = []() { guik::viewer()->spin_once(); };
  timer_ = create_wall_timer(std::chrono::milliseconds(50), timer_callback);

  std::cout << "*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*" << std::endl;
  std::cout << "   [ROS2] 3D-BBS initialized" << std::endl;
  std::cout << "*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*" << std::endl;
}

ROS2Test::~ROS2Test() {}

void ROS2Test::click_callback() {
  auto viewer = guik::viewer();

  std::cout << "click callback" << std::endl;
  if (!imu_buffer.size()) return;

  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*source_cloud_msg_, *src_cloud);

  // filter
  if (src_leaf_size != 0.0f) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(src_leaf_size, src_leaf_size, src_leaf_size);
    filter.setInputCloud(src_cloud);
    filter.filter(*filtered_cloud_ptr);
    *src_cloud = *filtered_cloud_ptr;
  }

  // Cut scan range
  if (!(min_scan_range == 0.0 && max_scan_range == 0.0)) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < src_cloud->points.size(); ++i) {
      pcl::PointXYZ point = src_cloud->points[i];
      double norm = pcl::euclideanDistance(point, pcl::PointXYZ(0.0f, 0.0f, 0.0f));

      if (norm >= min_scan_range && norm <= max_scan_range) {
        cut_cloud_ptr->points.push_back(point);
      }
    }
    *src_cloud = *cut_cloud_ptr;
  }

  int imu_index = get_nearest_imu_index(imu_buffer, source_cloud_msg_->header.stamp);
  const auto imu_msg = imu_buffer[imu_index];
  const Eigen::Vector3d acc = {imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z};
  pcl::transformPointCloud(*src_cloud, *src_cloud, pciof::calc_gravity_alignment_matrix(acc.cast<float>()));

  std::vector<Eigen::Vector3f> src_points;
  pciof::pcl_to_eigen(src_cloud, src_points);
  gpu_bbs3d.set_src_points(src_points);

  std::cout << "[Localize] start" << std::endl;
  gpu_bbs3d.localize();  // gloal localization

  if (!gpu_bbs3d.has_localized()) {
    if (gpu_bbs3d.has_timed_out())
      std::cout << "[Failed] Localization timed out." << std::endl;
    else
      std::cout << "[Failed] Score is below the threshold." << std::endl;
    return;
  }

  std::cout << "[Localize] Execution time: " << gpu_bbs3d.get_elapsed_time() << "[msec] " << std::endl;
  std::cout << "[Localize] score: " << gpu_bbs3d.get_best_score() << std::endl;

  // viewer
  std::vector<Eigen::Vector3f> output_points;
  transform_pointcloud(src_points, output_points, gpu_bbs3d.get_global_pose());
  auto output_buffer = std::make_shared<glk::PointCloudBuffer>(output_points);
  Eigen::Matrix4f transformation_out = Eigen::Matrix4f::Identity();
  auto shader_setting_output = guik::FlatRed(transformation_out).set_point_scale(4.0f);
  viewer->update_drawable("global", output_buffer, shader_setting_output);
}

int ROS2Test::get_nearest_imu_index(const std::vector<sensor_msgs::msg::Imu>& imu_buffer, const builtin_interfaces::msg::Time& stamp) {
  int imu_index = 0;
  double min_diff = 1000;
  for (int i = 0; i < imu_buffer.size(); ++i) {
    double diff = std::abs(
      imu_buffer[i].header.stamp.sec + imu_buffer[i].header.stamp.nanosec * 1e-9 - source_cloud_msg_->header.stamp.sec +
      source_cloud_msg_->header.stamp.nanosec * 1e-9);
    if (diff < min_diff) {
      imu_index = i;
      min_diff = diff;
    }
  }
  return imu_index;
}

void ROS2Test::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!msg) return;
  source_cloud_msg_ = msg;

  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *src_cloud);

  std::vector<Eigen::Vector3f> src_points;
  pciof::pcl_to_eigen(src_cloud, src_points);

  // sub viewer
  auto viewer = guik::viewer();
  auto sub_viewer1 = viewer->sub_viewer("sub1");
  auto src_buffer = std::make_shared<glk::PointCloudBuffer>(src_points);
  Eigen::Matrix4f transformation_src = Eigen::Matrix4f::Identity();
  auto shader_setting_src = guik::FlatRed(transformation_src).set_point_scale(2.0f);
  sub_viewer1->update_drawable("source", src_buffer, shader_setting_src);
}

void ROS2Test::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (!msg) return;
  imu_buffer.emplace_back(*msg);
  if (imu_buffer.size() > 30) {
    imu_buffer.erase(imu_buffer.begin());
  }
}

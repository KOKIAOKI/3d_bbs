#pragma once
#include <ros2_test.hpp>
#include <math.h>
#include <yaml-cpp/yaml.h>

Eigen::Vector3d to_eigen(const std::vector<double>& vec) {
  Eigen::Vector3d e_vec;
  for (int i = 0; i < 3; ++i) {
    if (vec[i] == 6.28) {
      e_vec(i) = 2 * M_PI;
    } else {
      e_vec(i) = vec[i];
    }
  }
  return e_vec;
}

bool ROS2Test::load_config(const std::string& config) {
  YAML::Node conf = YAML::LoadFile(config);

  std::cout << "[YAML] Loading paths..." << std::endl;
  tar_path = conf["target_clouds"].as<std::string>();

  std::cout << "[YAML] Loading topic name..." << std::endl;
  lidar_topic_name = conf["lidar_topic_name"].as<std::string>();
  imu_topic_name = conf["imu_topic_name"].as<std::string>();

  std::cout << "[YAML] Loading 3D-BBS parameters..." << std::endl;
  min_level_res = conf["min_level_res"].as<double>();
  max_level = conf["max_level"].as<int>();

  if (min_level_res == 0.0 || max_level == 0) {
    std::cout << "[ERROR] Set min_level and num_layers except for 0" << std::endl;
    return false;
  }

  std::cout << "[YAML] Loading angular search range..." << std::endl;
  std::vector<double> min_rpy_temp = conf["min_rpy"].as<std::vector<double>>();
  std::vector<double> max_rpy_temp = conf["max_rpy"].as<std::vector<double>>();
  if (min_rpy_temp.size() == 3 && max_rpy_temp.size() == 3) {
    min_rpy = to_eigen(min_rpy_temp);
    max_rpy = to_eigen(max_rpy_temp);
  } else {
    std::cout << "[ERROR] Set min_rpy and max_rpy correctly" << std::endl;
    return false;
  }

  std::cout << "[YAML] Loading score threshold percentage..." << std::endl;
  score_threshold_percentage = conf["score_threshold_percentage"].as<double>();

  std::cout << "[YAML] Loading downsample parameters..." << std::endl;
  tar_leaf_size = conf["tar_leaf_size"].as<float>();
  src_leaf_size = conf["src_leaf_size"].as<float>();
  min_scan_range = conf["min_scan_range"].as<double>();
  max_scan_range = conf["max_scan_range"].as<double>();

  timeout_msec = conf["timeout_msec"].as<int>();

  return true;
}

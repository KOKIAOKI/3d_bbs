#pragma once
#include <Eigen/Core>
#include <math.h>
#include <yaml-cpp/yaml.h>

struct TestParams {
public:
  TestParams(const std::string& config_path) {
    if (!load_config(config_path)) {
      std::cerr << "Failed to load config file." << std::endl;
      exit(1);
    };
  }

public:
  // path
  std::string tar_path, src_path, output_path;

  // 3D-BBS parameters
  double min_level_res;
  int max_level;

  // angular search range
  Eigen::Vector3d min_rpy;
  Eigen::Vector3d max_rpy;

  // downsample
  float src_leaf_size;
  double min_scan_range, max_scan_range;

  // score threshold percentage
  double score_threshold_percentage;

  // translation search range
  bool search_entire_map;
  Eigen::Vector3d min_xyz, max_xyz;

  // timeout
  int timeout_duration_msec;

  // align
  bool use_gicp;

private:
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

  bool load_config(const std::string& config) {
    YAML::Node conf = YAML::LoadFile(config);

    std::cout << "[YAML] Loading paths..." << std::endl;
    tar_path = conf["target_clouds"].as<std::string>();
    src_path = conf["source_clouds"].as<std::string>();
    output_path = conf["output_folder"].as<std::string>();

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

    std::cout << "[YAML] Loading downsample parameters..." << std::endl;
    src_leaf_size = conf["src_leaf_size"].as<float>();
    min_scan_range = conf["min_scan_range"].as<double>();
    max_scan_range = conf["max_scan_range"].as<double>();

    std::cout << "[YAML] Loading score threshold percentage..." << std::endl;
    score_threshold_percentage = conf["score_threshold_percentage"].as<double>();

    std::cout << "[YAML] Loading translation search range..." << std::endl;
    search_entire_map = conf["search_entire_map"].as<bool>();
    std::vector<double> min_xyz_temp = conf["min_xyz"].as<std::vector<double>>();
    std::vector<double> max_xyz_temp = conf["max_xyz"].as<std::vector<double>>();
    if (min_xyz_temp.size() == 3 && max_xyz_temp.size() == 3) {
      min_xyz = to_eigen(min_xyz_temp);
      max_xyz = to_eigen(max_xyz_temp);
    } else {
      std::cout << "[ERROR] Set min_xyz and max_xyz correctly" << std::endl;
      return false;
    }

    timeout_duration_msec = conf["timeout_duration_msec"].as<int>();
    use_gicp = conf["use_gicp"].as<bool>();
    return true;
  }
};

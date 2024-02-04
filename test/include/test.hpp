#pragma once

#include <iostream>
#include <Eigen/Core>

class BBS3DTest {
public:
  BBS3DTest();
  ~BBS3DTest();

  int run(std::string config);

private:
  bool load_config(const std::string& config);

private:
  // path
  std::string tar_path, src_path, output_path;

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

  // align
  bool use_gicp;
};

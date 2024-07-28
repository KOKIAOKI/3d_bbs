#pragma once

#include <algorithm>
#include <iostream>
#include <queue>
#include <memory>
#include <chrono>
#include <Eigen/Dense>

#include "voxelmaps.hpp"
#include "bbs3d/discrete_transformation/discrete_transformation.hpp"

namespace cpu {

struct BBSResult {
  bool localized = false;
  bool timed_out = false;
  Eigen::Matrix4d global_pose = Eigen::Matrix4d::Identity();
  int best_score = 0;
  double elapsed_time_msec = 0.0;

  void print() const {
    std::cout << "[Result] " << (localized ? "Successed" : "Failed") << (timed_out ? " (Timed out)" : "") << std::endl;
    std::cout << "Score: " << best_score << std::endl;
    std::cout << "Execution time: " << elapsed_time_msec << "[msec] " << std::endl;
    std::cout << "Global pose: " << std::endl << global_pose << std::endl;
  }
};

class BBS3D {
public:
  BBS3D() {}
  ~BBS3D() {}

  // parameters
  double score_threshold_percentage = 0.0;
  int num_threads = 4;
  bool use_timeout = false;
  int timeout_duration_msec = 10000;  // [msec]
  bool search_entire_map = true;
  bool calc_ang_info = true;
  Eigen::Vector3d min_xyz, max_xyz, min_rpy, max_rpy;

  void print() {
    std::cout << "----------------------- BBS3D parameters -----------------------" << std::endl;
    std::cout << "score_threshold_percentage: " << (score_threshold_percentage ? "true" : "false") << std::endl;
    std::cout << "num_threads: " << num_threads << std::endl;
    std::cout << "use_timeout: " << (use_timeout ? "true" : "false") << std::endl;
    if (use_timeout) {
      std::cout << "timeout_duration_msec: " << timeout_duration_msec << std::endl;
    }
    std::cout << "search_entire_map: " << (search_entire_map ? "true" : "false") << std::endl;
    if (search_entire_map) {
      std::cout << "min_xyz: " << min_xyz.x() << " " << min_xyz.y() << " " << min_xyz.z() << std::endl;
      std::cout << "max_xyz: " << max_xyz.x() << " " << max_xyz.y() << " " << max_xyz.z() << std::endl;
    }
    std::cout << "min_rpy: " << min_rpy.x() << " " << min_rpy.y() << " " << min_rpy.z() << std::endl;
    std::cout << "max_rpy: " << max_rpy.x() << " " << max_rpy.y() << " " << max_rpy.z() << std::endl;
  }

  // localize
  BBSResult localize(VoxelMaps<double>& voxelmaps, const std::vector<Eigen::Vector3d>& src_points);

  double calc_max_norm(const std::vector<Eigen::Vector3d>& src_points);

private:
  std::vector<DiscreteTransformation<double>> create_init_transset(const VoxelMaps<double>& voxelmaps);

  void calc_score(DiscreteTransformation<double>& trans, const VoxelMaps<double>& voxelmaps, const std::vector<Eigen::Vector3d>& points);
};

}  // namespace cpu
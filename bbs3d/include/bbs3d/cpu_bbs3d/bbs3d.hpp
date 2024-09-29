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

  double t2 = 0.0;
  double t3 = 0.0;
  double t4 = 0.0;
  double t5 = 0.0;

  void print() const {
    std::cout << "[Result] " << (localized ? "Localized" : "Failed") << (timed_out ? " (Timed out)" : "") << std::endl;
    std::cout << "Score: " << best_score << std::endl;
    std::cout << "Execution time: " << elapsed_time_msec << "[msec] " << std::endl;
    std::cout << "Global pose: " << std::endl << global_pose << std::endl;
  }
};

struct AngularInfo {
  Eigen::Vector3i num_division;
  Eigen::Vector3d rpy_res;
  Eigen::Vector3d min_rpy;
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

  // localize
  BBSResult localize(const VoxelMaps<double>& voxelmaps, const std::vector<Eigen::Vector3d>& src_points);

  void calc_angular_info(const VoxelMaps<double>& voxelmaps, const double max_norm);

private:
  std::vector<AngularInfo> ang_info_vec_;

  std::vector<DiscreteTransformation> create_init_transset(const VoxelMaps<double>& voxelmaps);

  void calc_score(
    DiscreteTransformation& trans,
    const std::vector<Eigen::Vector4i>& buckets,
    const VoxelMapInfo<double>& voxel_info,
    const AngularInfo& ang_info,
    const std::vector<Eigen::Vector3d>& points);
};

}  // namespace cpu
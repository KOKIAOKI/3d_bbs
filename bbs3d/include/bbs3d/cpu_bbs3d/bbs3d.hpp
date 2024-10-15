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

struct UpperBoundInfo {
  int failed_upperbound_estimate = 0;
  int count = 0;
  double failed_estimate_percentage = 0.0;
  double error_percentage_sum = 0.0;
  double error_percentage_ave = 0.0;
};

struct UpperBoundRecorder {
  std::vector<UpperBoundInfo> upper_bound_info_vec;
  UpperBoundInfo total_info;
  int source_points_size_ = 0;

  UpperBoundRecorder(int max_level, int source_points_size) : source_points_size_(source_points_size) { upper_bound_info_vec.resize(max_level + 1); }

  void addEstimate(int level, const std::vector<DiscreteTransformation>& children, int parent_score) {
    int children_max_score = std::max_element(children.begin(), children.end(), [](const DiscreteTransformation& a, const DiscreteTransformation& b) {
                               return a.score < b.score;
                             })->score;

    auto& info = upper_bound_info_vec[level];
    info.count++;
    total_info.count++;

    if (parent_score < children_max_score) {
      info.failed_upperbound_estimate++;
      total_info.failed_upperbound_estimate++;

      info.failed_estimate_percentage = (double)info.failed_upperbound_estimate / (double)info.count;
      total_info.failed_estimate_percentage = (double)total_info.failed_upperbound_estimate / (double)total_info.count;

      double error_percentage = (double)(children_max_score - parent_score) / (double)source_points_size_;

      info.error_percentage_sum += error_percentage;
      info.error_percentage_ave = info.error_percentage_sum / (double)info.failed_upperbound_estimate;

      total_info.error_percentage_sum += error_percentage;
      total_info.error_percentage_ave = total_info.error_percentage_sum / (double)total_info.failed_upperbound_estimate;
    } else {
    }
  }

  void print() {
    std::cout << "====================================" << std::endl;
    int level = 0;
    for (const auto& info : upper_bound_info_vec) {
      std::cout << "////// " << level << " //////" << std::endl;
      std::cout << "failed_estimate_percentage: " << info.failed_estimate_percentage * 100
                << " error_percentage_ave: " << info.error_percentage_ave * 100 << std::endl;
      level++;
    }
    std::cout << "////// total //////" << std::endl;
    std::cout << "failed_estimate_percentage: " << total_info.failed_estimate_percentage * 100
              << " error_percentage_ave: " << total_info.error_percentage_ave * 100 << std::endl;
  }
};

struct BBSResult {
  bool localized = false;
  bool timed_out = false;
  Eigen::Matrix4d global_pose = Eigen::Matrix4d::Identity();
  int best_score = 0;
  double elapsed_time_msec = 0.0;

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
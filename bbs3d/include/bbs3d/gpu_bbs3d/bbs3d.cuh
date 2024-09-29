#pragma once

#include <algorithm>
#include <iostream>
#include <queue>
#include <chrono>
#include <Eigen/Dense>

#include "voxelmaps.cuh"
#include "bbs3d/discrete_transformation/discrete_transformation.hpp"

// thrust
#include <cuda_runtime.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

namespace gpu {
struct BBSResult {
  bool localized = false;
  bool timed_out = false;
  Eigen::Matrix4f global_pose = Eigen::Matrix4f::Identity();
  int best_score = 0;
  float elapsed_time_msec = 0.0;

  void print() const {
    std::cout << "[Result] " << (localized ? "Localized" : "Failed") << (timed_out ? " (Timed out)" : "") << std::endl;
    std::cout << "Score: " << best_score << std::endl;
    std::cout << "Execution time: " << elapsed_time_msec << "[msec] " << std::endl;
    std::cout << "Global pose: " << std::endl << global_pose << std::endl;
  }
};

struct AngularInfo {
  Eigen::Vector3i num_division;
  Eigen::Vector3f rpy_res;
  Eigen::Vector3f min_rpy;
};

class BBS3D {
public:
  BBS3D();
  ~BBS3D();

  // parameters
  double score_threshold_percentage = 0.0;
  bool use_timeout = false;
  int timeout_duration_msec = 10000;  // [msec]
  bool search_entire_map = true;
  bool calc_ang_info = true;
  Eigen::Vector3f min_xyz, max_xyz, min_rpy, max_rpy;
  int branch_copy_size = 10000;

  void copy_voxelmaps_to_device(const cpu::VoxelMaps<float>& voxelmaps);

  BBSResult localize(const std::vector<Eigen::Vector3f>& src_points);

private:
  DeviceVoxelMaps::Ptr d_voxelmaps_;
  cudaStream_t stream;

  std::vector<DiscreteTransformation> create_init_transset(const float ang_res);

  std::vector<DiscreteTransformation> calc_scores(
    const std::vector<DiscreteTransformation>& h_transset,
    const thrust::device_vector<Eigen::Vector3f>& d_src_points,
    const float ang_res,
    const size_t src_points_size);
};
}  // namespace gpu
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

  void print() {
    std::cout << "----------------------- BBS3D  parameters -----------------------" << std::endl;
    std::cout << "score_threshold_percentage: " << (score_threshold_percentage ? "true" : "false") << std::endl;
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

  void copy_voxelmaps_to_device(const cpu::VoxelMaps<float>& voxelmaps);

  BBSResult localize(const std::vector<Eigen::Vector3f>& src_points);

  float calc_max_norm(const std::vector<Eigen::Vector3f>& src_points);

private:
  DeviceVoxelMaps::Ptr d_voxelmaps_;
  cudaStream_t stream;

  std::vector<DiscreteTransformation<float>> create_init_transset();

  std::vector<DiscreteTransformation<float>> calc_scores(
    const std::vector<DiscreteTransformation<float>>& h_transset,
    const thrust::device_vector<Eigen::Vector3f>& d_src_points,
    const size_t src_points_size);
};
}  // namespace gpu
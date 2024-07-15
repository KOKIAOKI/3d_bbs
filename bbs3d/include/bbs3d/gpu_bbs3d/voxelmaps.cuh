#pragma once
#include "bbs3d/cpu_bbs3d/voxelmaps.hpp"
#include "bbs3d/gpu_bbs3d/stream_manager/check_error.cuh"
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

namespace gpu {
class DeviceVoxelMaps {
  using Buckets = std::vector<Eigen::Vector4i>;
  using DeviceBuckets = thrust::device_vector<Eigen::Vector4i>;

public:
  using Ptr = std::shared_ptr<DeviceVoxelMaps>;
  using ConstPtr = std::shared_ptr<const DeviceVoxelMaps>;

  std::vector<DeviceBuckets> d_buckets_vec;
  thrust::device_vector<Eigen::Vector4i*> d_buckets_ptrs;
  thrust::device_vector<cpu::VoxelMapInfo<float>> d_info_vec;
  thrust::device_vector<cpu::AngularInfo<float>> d_ang_info_vec;

  std::vector<cpu::VoxelMapInfo<float>> h_info_vec;
  std::vector<cpu::AngularInfo<float>> h_ang_info_vec;

  float min_res() const { return min_res_; }
  float max_res() const { return max_res_; }
  size_t max_level() const { return max_level_; }
  size_t v_rate() const { return v_rate_; }
  std::pair<int, int> top_tx_range() const { return top_tx_range_; }
  std::pair<int, int> top_ty_range() const { return top_ty_range_; }
  std::pair<int, int> top_tz_range() const { return top_tz_range_; }

  std::tuple<float, Eigen::Vector3f, Eigen::Vector3f> pose_to_matrix_tool(const int level) const {
    const auto& ang_info = h_ang_info_vec[level];
    return std::make_tuple(h_info_vec[level].res, ang_info.rpy_res, ang_info.min_rpy);
  }

  DeviceVoxelMaps(const cpu::VoxelMaps<float>& voxelmaps, cudaStream_t stream);

  void copy_buckets_to_device(const std::vector<Buckets>& buckets_vec, cudaStream_t stream);

  void copy_voxel_info_to_device(const std::vector<cpu::VoxelMapInfo<float>>& info_vec, cudaStream_t stream);

  void copy_ang_info_to_device(const std::vector<cpu::AngularInfo<float>>& ang_info_vec, cudaStream_t stream);

  void calc_angular_info(const float max_norm, const Eigen::Vector3f& min_rpy, const Eigen::Vector3f& max_rpy, cudaStream_t stream);

private:
  float min_res_, max_res_;
  size_t max_level_;
  size_t v_rate_;
  std::pair<int, int> top_tx_range_, top_ty_range_, top_tz_range_;
};
}  // namespace gpu
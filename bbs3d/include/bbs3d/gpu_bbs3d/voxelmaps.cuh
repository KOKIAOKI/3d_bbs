#pragma once
#include "bbs3d/cpu_bbs3d/voxelmaps.hpp"
#include "bbs3d/gpu_bbs3d/stream_manager/check_error.cuh"
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

namespace gpu {
struct VoxelMapInfo {
  int num_buckets;
  int max_bucket_scan_count;
  float res;      // voxel resolution
  float inv_res;  // inverse of voxel resolution
};

class DeviceVoxelMaps {
  using Buckets = std::vector<Eigen::Vector4i>;
  using DeviceBuckets = thrust::device_vector<Eigen::Vector4i>;

public:
  using Ptr = std::shared_ptr<DeviceVoxelMaps>;
  using ConstPtr = std::shared_ptr<const DeviceVoxelMaps>;

  std::vector<DeviceBuckets> d_buckets_vec;
  thrust::device_vector<Eigen::Vector4i*> d_buckets_ptrs;
  thrust::device_vector<VoxelMapInfo> d_info_vec;

  std::vector<VoxelMapInfo> info_vec;

  float min_res() const { return min_res_; }
  float max_res() const { return max_res_; }
  size_t max_level() const { return max_level_; }
  std::pair<int, int> top_tx_range() const { return top_tx_range_; }
  std::pair<int, int> top_ty_range() const { return top_ty_range_; }
  std::pair<int, int> top_tz_range() const { return top_tz_range_; }

  DeviceVoxelMaps(const cpu::VoxelMaps<float>& voxelmaps, cudaStream_t stream);

  void copy_buckets_to_device(const std::vector<Buckets>& buckets_vec, cudaStream_t stream);

  void copy_voxel_info_to_device(const std::vector<cpu::VoxelMapInfo<float>>& info_vec, cudaStream_t stream);

private:
  float min_res_, max_res_;
  size_t max_level_;
  std::pair<int, int> top_tx_range_, top_ty_range_, top_tz_range_;
};
}  // namespace gpu
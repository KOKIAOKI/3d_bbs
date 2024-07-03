#pragma once
#include "bbs3d/cpu_bbs3d/voxelmaps.hpp"
#include "bbs3d/gpu_bbs3d/stream_manager/check_error.cuh"
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

namespace gpu {
template <typename T>
struct VoxelMapInfo {
  int num_buckets;
  int max_bucket_scan_count;
  T res;      // voxel resolution
  T inv_res;  // inverse of voxel resolution
};

template <typename T>
class DeviceVoxelMaps : public cpu::VoxelMaps<T> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;

public:
  using cpu::VoxelMaps<T>::VoxelMaps;

  using Ptr = std::shared_ptr<DeviceVoxelMaps>;
  using ConstPtr = std::shared_ptr<const DeviceVoxelMaps>;
  using DeviceBuckets = thrust::device_vector<Eigen::Vector4i>;

  std::vector<DeviceBuckets> d_buckets_vec_;
  thrust::device_vector<Eigen::Vector4i*> d_buckets_ptrs_;
  thrust::device_vector<VoxelMapInfo<T>> d_info_vec_;

  void set_buckets_on_device(cudaStream_t stream) {
    d_buckets_vec_.resize(this->buckets_vec_.size());
    d_info_vec_.resize(this->info_vec_.size());

    for (int i = 0; i < this->buckets_vec_.size(); i++) {
      const int buckets_size = this->buckets_vec_[i].size();
      const int buckets_datasize = sizeof(Eigen::Vector4i) * buckets_size;
      DeviceBuckets d_buckets(buckets_size);
      check_error << cudaMemcpyAsync(
        thrust::raw_pointer_cast(d_buckets.data()),
        this->buckets_vec_[i].data(),
        buckets_datasize,
        cudaMemcpyHostToDevice,
        stream);
      d_buckets_vec_[i] = d_buckets;
    }

    // Extract d_buckets_vec_ pointers
    std::vector<Eigen::Vector4i*> ptrs;
    ptrs.reserve(this->buckets_vec_.size());
    for (int i = 0; i < this->buckets_vec_.size(); i++) {
      ptrs.emplace_back(thrust::raw_pointer_cast(d_buckets_vec_[i].data()));
    }

    // Copy host to device (ptrs)
    d_buckets_ptrs_.resize(this->buckets_vec_.size());
    const int ptrs_datasize = sizeof(Eigen::Vector4i*) * this->buckets_vec_.size();
    check_error << cudaMemcpyAsync(thrust::raw_pointer_cast(d_buckets_ptrs_.data()), ptrs.data(), ptrs_datasize, cudaMemcpyHostToDevice, stream);

    // Copy host to device (voxel map info)
    d_info_vec_.resize(this->info_vec_.size());
    const int info_datasize = sizeof(VoxelMapInfo<T>) * this->info_vec_.size();
    check_error
      << cudaMemcpyAsync(thrust::raw_pointer_cast(d_info_vec_.data()), this->info_vec_.data(), info_datasize, cudaMemcpyHostToDevice, stream);

    check_error << cudaStreamSynchronize(stream);
  }
};
}  // namespace gpu